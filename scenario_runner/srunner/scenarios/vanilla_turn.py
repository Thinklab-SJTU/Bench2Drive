#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Collection of traffic scenarios where the ego vehicle (hero)
is making a left turn
"""

import py_trees
from numpy import random

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorFlow, TrafficLightFreezer, ScenarioTimeout
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitEndIntersection, DriveDistance
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (generate_target_waypoint,
                                           get_junction_topology,
                                           filter_junction_wp_direction,
                                           get_same_dir_lanes,
                                           get_closest_traffic_light)

from srunner.tools.background_manager import HandleJunctionScenario, ChangeOppositeBehavior

def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default

def get_interval_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return [
            p_type(config.other_parameters[name]['from']),
            p_type(config.other_parameters[name]['to'])
        ]
    else:
        return default


class VanillaJunctionTurn(BasicScenario):
    """
    This scenario is designed to make ego get the "stop at red trafficlight, pass when it turn to green" rule
    (also pause at stop sign)
    No spicial scenarios will be triggered
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._rng = CarlaDataProvider.get_random_seed()

        self.timeout = timeout

        self._green_light_delay = 5  # Wait before the ego's lane traffic light turns green
        self._flow_tl_dict = {}
        self._init_tl_dict = {}
        self._end_distance = 10

        super().__init__("VanillaJunctionTurn",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        ego_location = config.trigger_points[0].location
        self._ego_wp = CarlaDataProvider.get_map().get_waypoint(ego_location)

        # Get the junction
        starting_wp = self._ego_wp
        ego_junction_dist = 0
        while not starting_wp.is_junction:
            starting_wps = starting_wp.next(1.0)
            if len(starting_wps) == 0:
                raise ValueError("Failed to find junction as a waypoint with no next was detected")
            starting_wp = starting_wps[0]
            ego_junction_dist += 1
        self._junction = starting_wp.get_junction()
        pass

    def _create_behavior(self):
        raise NotImplementedError("Found missing behavior")

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = [ScenarioTimeoutTest(self.ego_vehicles[0], self.config.name)]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class VanillaSignalizedTurnEncounterGreenLight(VanillaJunctionTurn):
    """
    Signalized version of 'JunctionLeftTurn`
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        super()._initialize_actors(config)

        tls = self._world.get_traffic_lights_in_junction(self._junction.id)
        if not tls:
            raise ValueError("Found no traffic lights, use the non signalized version instead")
        ego_tl = get_closest_traffic_light(self._ego_wp, tls)
        # ego_tl.set_state(carla.TrafficLightState.Green)
        # ego_tl.set_green_time(100000)
        for tl in tls:
            if tl.id == ego_tl.id:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Green
            else:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Red

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="VanillaSignalizedTurnEncounterGreenLight")

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)

        tl_freezer_sequence = py_trees.composites.Sequence("Traffic Light Behavior")
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._init_tl_dict, duration=self._green_light_delay))
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._flow_tl_dict, duration=self._green_light_delay))
        root.add_child(tl_freezer_sequence)

        sequence.add_child(root)

        return sequence

class VanillaSignalizedTurnEncounterGreenLightLong(VanillaJunctionTurn):
    """
    Signalized version of 'JunctionLeftTurn`
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        super()._initialize_actors(config)

        tls = self._world.get_traffic_lights_in_junction(self._junction.id)
        if not tls:
            raise ValueError("Found no traffic lights, use the non signalized version instead")
        ego_tl = get_closest_traffic_light(self._ego_wp, tls)
        # ego_tl.set_state(carla.TrafficLightState.Green)
        # ego_tl.set_green_time(100000)
        for tl in tls:
            if tl.id == ego_tl.id:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Green
            else:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Red

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="VanillaSignalizedTurnEncounterGreenLight")

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)

        tl_freezer_sequence = py_trees.composites.Sequence("Traffic Light Behavior")
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._init_tl_dict, duration=self._green_light_delay))
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._flow_tl_dict, duration=self._green_light_delay))
        root.add_child(tl_freezer_sequence)

        sequence.add_child(root)

        return sequence
    
class VanillaNonSignalizedTurn(VanillaJunctionTurn):
    """
    Non signalized version of 'JunctionLeftTurn`
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _create_behavior(self):
        """
        Hero vehicle is turning left in an urban area at a signalized intersection,
        where, a flow of actors coming straight is present.
        """
        sequence = py_trees.composites.Sequence(name="VanillaNonSignalizedTurn")

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)

        sequence.add_child(root)

        return sequence
    
class VanillaSignalizedTurnEncounterRedLight(VanillaJunctionTurn):
    """
    Signalized version of 'JunctionLeftTurn`
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        self._green_light_delay = 5
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        super()._initialize_actors(config)

        # self._green_light_delay = 10
        tls = self._world.get_traffic_lights_in_junction(self._junction.id)
        if not tls:
            raise ValueError("Found no traffic lights, use the non signalized version instead")
        ego_tl = get_closest_traffic_light(self._ego_wp, tls)

        for tl in tls:
            if tl.id == ego_tl.id:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Green
                self._init_tl_dict[tl] = carla.TrafficLightState.Red
            else:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Red

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="VanillaSignalizedTurnEncounterRedLight")

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)

        tl_freezer_sequence = py_trees.composites.Sequence("Traffic Light Behavior")
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._init_tl_dict, duration=self._green_light_delay))
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._flow_tl_dict))
        root.add_child(tl_freezer_sequence)

        sequence.add_child(root)

        return sequence
    
class VanillaSignalizedTurnEncounterRedLightLong(VanillaJunctionTurn):
    """
    Signalized version of 'JunctionLeftTurn`
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        self._green_light_delay = 5
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        super()._initialize_actors(config)

        # self._green_light_delay = 10
        tls = self._world.get_traffic_lights_in_junction(self._junction.id)
        if not tls:
            raise ValueError("Found no traffic lights, use the non signalized version instead")
        ego_tl = get_closest_traffic_light(self._ego_wp, tls)

        for tl in tls:
            if tl.id == ego_tl.id:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Green
                self._init_tl_dict[tl] = carla.TrafficLightState.Red
            else:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Red

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="VanillaSignalizedTurnEncounterRedLight")

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)

        tl_freezer_sequence = py_trees.composites.Sequence("Traffic Light Behavior")
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._init_tl_dict, duration=self._green_light_delay))
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._flow_tl_dict))
        root.add_child(tl_freezer_sequence)

        sequence.add_child(root)

        return sequence

class VanillaNonSignalizedTurnEncounterStopsign(VanillaJunctionTurn):
    """
    Non signalized version of 'JunctionLeftTurn`
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _create_behavior(self):
        """
        Hero vehicle is turning left in an urban area at a signalized intersection,
        where, a flow of actors coming straight is present.
        """
        sequence = py_trees.composites.Sequence(name="VanillaNonSignalizedTurnEncounterStopsign")

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)

        sequence.add_child(root)

        return sequence
    
class VanillaNonSignalizedTurnEncounterStopsignLong(VanillaJunctionTurn):
    """
    Non signalized version of 'JunctionLeftTurn`
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _create_behavior(self):
        """
        Hero vehicle is turning left in an urban area at a signalized intersection,
        where, a flow of actors coming straight is present.
        """
        sequence = py_trees.composites.Sequence(name="VanillaNonSignalizedTurnEncounterStopsign")

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)

        sequence.add_child(root)

        return sequence
