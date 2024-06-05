#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario in which the ego is about to turn right 
when a vehicle coming from the opposite lane invades the ego's lane, forcing the ego to move right to avoid a possible collision.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (InvadingActorFlow,
                                                                      ScenarioTimeout,
                                                                      ActorDestroy,
                                                                      BatchActorTransformSetter)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitUntilInFrontPosition
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.background_manager import RemoveRoadLane, ChangeOppositeBehavior, ReAddRoadLane


def convert_dict_to_location(actor_dict):
    """
    Convert a JSON string to a Carla.Location
    """
    location = carla.Location(
        x=float(actor_dict['x']),
        y=float(actor_dict['y']),
        z=float(actor_dict['z'])
    )
    return location


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default


class InvadingTurn(BasicScenario):
    """
    This class holds everything required for a scenario in which the ego is about to turn right 
    when a vehicle coming from the opposite lane invades the ego's lane, 
    forcing the ego to move right to avoid a possible collision.

    This scenario is expected to take place on a road that has only one lane in each direction.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(
            self._trigger_location)

        self._flow_frequency = 40 # m
        self._source_dist = 30 # Distance between source and end point

        self._check_distance = 50

        self._distance = get_value_parameter(config, 'distance', float, 100)
        self._offset = get_value_parameter(config, 'offset', float, 0.25)  # meters invaded in the opposite direction
        self._scenario_timeout = 240

        self._obstacle_transforms = []

        super().__init__("InvadingTurn",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Spawn adversary actor
        next_wps = self._reference_waypoint.next(self._distance + self._source_dist)
        if not next_wps:
            raise ValueError("Couldn't find the source location for the actor flow")
        self._forward_wp = next_wps[0]
        self._source_wp = self._forward_wp.get_left_lane()
        if not self._source_wp:
            raise ValueError("Couldn't find the source location for the actor flow")

        self._sink_wp = self._reference_waypoint.get_left_lane()
        if not self._sink_wp:
            raise ValueError("Couldn't find the sink location for the actor flow")

        # Lane offset
        self._offset_constant = 0.7  # Ideally, half the vehicle lane width
        self._true_offset = self._offset + self._sink_wp.lane_width / 2 - self._offset_constant
        self._true_offset *= -1 # Cause left direction

        self._create_obstacle()

    def _create_obstacle(self):

        next_wp = self._source_wp.next(10)[0]
        obstacle_distance = 0.5 * self._distance
        dist = 0
        while dist < obstacle_distance:
            next_wp = next_wp.next(5)[0]

            displacement = 0.8 * next_wp.lane_width / 2
            r_vec = next_wp.transform.get_right_vector()
            spawn_transform = next_wp.transform
            spawn_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=0.3)

            cone = CarlaDataProvider.request_new_actor('*constructioncone*', spawn_transform)
            self.other_actors.append(cone)

            self._obstacle_transforms.append([cone, spawn_transform])

            transform = carla.Transform(spawn_transform.location, spawn_transform.rotation)
            transform.location.z -= 200
            cone.set_transform(transform)
            cone.set_simulate_physics(False)

            dist += 5

        self._obstacle_transforms.reverse()  # So that the closest cones are spawned first

    def _create_behavior(self):
        """
        The adversary vehicle will go to the target place while invading another lane.
        """
        sequence = py_trees.composites.Sequence("InvadingTurn")

        if self.route_mode:
            sequence.add_child(RemoveRoadLane(self._reference_waypoint))
            sequence.add_child(ChangeOppositeBehavior(active=False))

        sequence.add_child(BatchActorTransformSetter(self._obstacle_transforms))

        main_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        main_behavior.add_child(InvadingActorFlow(
            self._source_wp, self._sink_wp, self.ego_vehicles[0], self._flow_frequency, offset=self._true_offset))

        main_behavior.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._forward_wp.transform, True, self._check_distance))
        main_behavior.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        sequence.add_child(main_behavior)
        if self.route_mode:
            sequence.add_child(ReAddRoadLane(0))
            sequence.add_child(ChangeOppositeBehavior(active=True))

        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))

        return sequence

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
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
