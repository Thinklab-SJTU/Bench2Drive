#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provide BasicScenario, the basic class of all the scenarios.
"""

from __future__ import print_function

import operator
import py_trees

import carla

from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (WaitForBlackboardVariable,
                                                                               InTimeToArrivalToLocation)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaitForever
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import UpdateAllActorControls
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion


class BasicScenario(object):

    """
    Base class for user-defined scenario
    """

    def __init__(self, name, ego_vehicles, config, world,
                 debug_mode=False, terminate_on_failure=False, criteria_enable=False):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self.name = name
        self.ego_vehicles = ego_vehicles
        self.other_actors = []
        self.parking_slots = []
        self.config = config
        self.world = world
        self.debug_mode = debug_mode
        self.terminate_on_failure = terminate_on_failure
        self.criteria_enable = criteria_enable

        self.route_mode = bool(config.route)
        self.behavior_tree = None
        self.criteria_tree = None

        # If no timeout was provided, set it to 60 seconds
        if not hasattr(self, 'timeout'):
            self.timeout = 60 
        if debug_mode:
            py_trees.logging.level = py_trees.logging.Level.DEBUG

        if not self.route_mode:
            # Only init env for route mode, avoid duplicate initialization during runtime
            self._initialize_environment(world)
            
        self._initialize_actors(config)

        if CarlaDataProvider.is_runtime_init_mode():
            world.wait_for_tick()
        elif CarlaDataProvider.is_sync_mode():
            world.tick()
        else:
            world.wait_for_tick()

        # Main scenario tree
        self.scenario_tree = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Add a trigger and end condition to the behavior to ensure it is only activated when it is relevant
        self.behavior_tree = py_trees.composites.Sequence()

        trigger_behavior = self._setup_scenario_trigger(config)
        if trigger_behavior:
            self.behavior_tree.add_child(trigger_behavior)

        scenario_behavior = self._create_behavior()
        self.behavior_tree.add_child(scenario_behavior)
        self.behavior_tree.name = scenario_behavior.name

        end_behavior = self._setup_scenario_end(config)
        if end_behavior:
            self.behavior_tree.add_child(end_behavior)

        # Create the lights behavior
        lights = self._create_lights_behavior()
        if lights:
            self.scenario_tree.add_child(lights)

        # Create the weather behavior
        weather = self._create_weather_behavior()
        if weather:
            self.scenario_tree.add_child(weather)

        # And then add it to the main tree
        self.scenario_tree.add_child(self.behavior_tree)

        # Create the criteria tree (if needed)
        if self.criteria_enable:
            criteria = self._create_test_criteria()

            # All the work is done, thanks!
            if isinstance(criteria, py_trees.composites.Composite):
                self.criteria_tree = criteria

            # Lazy mode, but its okay, we'll create the parallel behavior tree for you.
            elif isinstance(criteria, list):
                for criterion in criteria:
                    criterion.terminate_on_failure = terminate_on_failure

                self.criteria_tree = py_trees.composites.Parallel(name="Test Criteria",
                                                                  policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
                self.criteria_tree.add_children(criteria)
                self.criteria_tree.setup(timeout=1)

            else:
                raise ValueError("WARNING: Scenario {} couldn't be setup, make sure the criteria is either "
                                 "a list or a py_trees.composites.Composite".format(self.name))

            self.scenario_tree.add_child(self.criteria_tree)

        # Create the timeout behavior
        self.timeout_node = self._create_timeout_behavior()
        if self.timeout_node:
            self.scenario_tree.add_child(self.timeout_node)

        # Add other nodes
        self.scenario_tree.add_child(UpdateAllActorControls())

        self.scenario_tree.setup(timeout=1)

    def _initialize_environment(self, world):
        """
        Default initialization of weather and road friction.
        Override this method in child class to provide custom initialization.
        """

        # Set the appropriate weather conditions
        world.set_weather(self.config.weather)

        # Set the appropriate road friction
        if self.config.friction is not None:
            friction_bp = world.get_blueprint_library().find('static.trigger.friction')
            extent = carla.Location(1000000.0, 1000000.0, 1000000.0)
            friction_bp.set_attribute('friction', str(self.config.friction))
            friction_bp.set_attribute('extent_x', str(extent.x))
            friction_bp.set_attribute('extent_y', str(extent.y))
            friction_bp.set_attribute('extent_z', str(extent.z))

            # Spawn Trigger Friction
            transform = carla.Transform()
            transform.location = carla.Location(-10000.0, -10000.0, 0.0)
            world.spawn_actor(friction_bp, transform)

    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        if config.other_actors:
            new_actors = CarlaDataProvider.request_new_actors(config.other_actors)
            if not new_actors:
                raise Exception("Error: Unable to add actors")

            for new_actor in new_actors:
                self.other_actors.append(new_actor)

    def _setup_scenario_trigger(self, config):
        """
        This function creates a trigger maneuver, that has to be finished before the real scenario starts.
        This implementation focuses on the first available ego vehicle.

        The function can be overloaded by a user implementation inside the user-defined scenario class.
        """
        if config.trigger_points and config.trigger_points[0]:
            start_location = config.trigger_points[0].location
        else:
            return None

        # Scenario is not part of a route, wait for the ego to move
        if not self.route_mode or config.route_var_name is None:
            return InTimeToArrivalToLocation(self.ego_vehicles[0], 2.0, start_location)

        # Scenario is part of a route.
        check_name = "WaitForBlackboardVariable: {}".format(config.route_var_name)
        return WaitForBlackboardVariable(config.route_var_name, True, False, name=check_name)

    def _setup_scenario_end(self, config):
        """
        This function adds and additional behavior to the scenario, which is triggered
        after it has ended. The Blackboard variable is set to False to indicate the scenario has ended.
        The function can be overloaded by a user implementation inside the user-defined scenario class.
        """
        if not self.route_mode or config.route_var_name is None:
            return None

        # Scenario is part of a route.
        end_sequence = py_trees.composites.Sequence()
        name = "Reset Blackboard Variable: {} ".format(config.route_var_name)
        end_sequence.add_child(py_trees.blackboard.SetBlackboardVariable(name, config.route_var_name, False))
        end_sequence.add_child(WaitForever())  # scenario can't stop the route

        return end_sequence

    def _create_behavior(self):
        """
        Pure virtual function to setup user-defined scenario behavior
        """
        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def _create_test_criteria(self):
        """
        Pure virtual function to setup user-defined evaluation criteria for the
        scenario
        """
        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def _create_weather_behavior(self):
        """
        Default empty initialization of the weather behavior,
        responsible of controlling the weather during the simulation.
        Override this method in child class to provide custom initialization.
        """
        pass

    def _create_lights_behavior(self):
        """
        Default empty initialization of the lights behavior,
        responsible of controlling the street lights during the simulation.
        Override this method in child class to provide custom initialization.
        """
        pass

    def _create_timeout_behavior(self):
        """
        Default initialization of the timeout behavior.
        Override this method in child class to provide custom initialization.
        """
        return TimeOut(self.timeout, name="TimeOut")  # Timeout node

    def change_control(self, control):  # pylint: disable=no-self-use
        """
        This is a function that changes the control based on the scenario determination
        :param control: a carla vehicle control
        :return: a control to be changed by the scenario.

        Note: This method should be overriden by the user-defined scenario behavior
        """
        return control

    def get_criteria(self):
        """
        Return the list of test criteria, including all the leaf nodes.
        Some criteria might have trigger conditions, which have to be filtered out.
        """
        criteria = []
        if not self.criteria_tree:
            return criteria

        criteria_nodes = self._extract_nodes_from_tree(self.criteria_tree)
        for criterion in criteria_nodes:
            if isinstance(criterion, Criterion):
                criteria.append(criterion)

        return criteria

    def _extract_nodes_from_tree(self, tree):  # pylint: disable=no-self-use
        """
        Returns the list of all nodes from the given tree
        """
        node_list = [tree]
        more_nodes_exist = True
        while more_nodes_exist:
            more_nodes_exist = False
            for node in node_list:
                if node.children:
                    node_list.remove(node)
                    more_nodes_exist = True
                    for child in node.children:
                        node_list.append(child)

        if len(node_list) == 1 and isinstance(node_list[0], py_trees.composites.Parallel):
            return []

        return node_list

    def terminate(self):
        """
        This function sets the status of all leaves in the scenario tree to INVALID
        """
        # Get list of all nodes in the tree
        node_list = self._extract_nodes_from_tree(self.scenario_tree)

        # Set status to INVALID
        for node in node_list:
            node.terminate(py_trees.common.Status.INVALID)

        # Cleanup all instantiated controllers
        actor_dict = {}
        try:
            check_actors = operator.attrgetter("ActorsWithController")
            actor_dict = check_actors(py_trees.blackboard.Blackboard())
        except AttributeError:
            pass
        for actor_id in actor_dict:
            actor_dict[actor_id].reset()
        py_trees.blackboard.Blackboard().set("ActorsWithController", {}, overwrite=True)

    def remove_all_actors(self):
        """
        Remove all actors
        """
        if not hasattr(self, 'other_actors'):
            return
        for i, _ in enumerate(self.other_actors):
            if self.other_actors[i] is not None:
                if CarlaDataProvider.actor_id_exists(self.other_actors[i].id):
                    CarlaDataProvider.remove_actor_by_id(self.other_actors[i].id)
                self.other_actors[i] = None
        self.other_actors = []

    def get_parking_slots(self):
        """
        Returns occupied parking slots.
        """
        return self.parking_slots
