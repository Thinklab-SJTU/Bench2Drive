#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario with low visibility, the ego performs a turn only to find out that the end is blocked by another vehicle.
"""

from __future__ import print_function

import carla
import py_trees
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      Idle,
                                                                      ScenarioTimeout,
                                                                      ActorTransformSetter,
                                                                      HandBrakeVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.background_manager import HandleJunctionScenario

from srunner.tools.scenario_helper import generate_target_waypoint_in_route


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


class BlockedIntersection(BasicScenario):
    """
    This class holds everything required for a scenario in which,
    the ego performs a turn only to find out that the end is blocked by another vehicle.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(self._trigger_location)

        self._blocker_distance = 5
        self._trigger_distance = 13
        self._stop_time = 10

        self._scenario_timeout = 240

        self._blocker_transform = None

        super().__init__("BlockedIntersection",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        waypoint = generate_target_waypoint_in_route(self._reference_waypoint, config.route)
        waypoint = waypoint.next(self._blocker_distance)[0]

        self._blocker_transform = waypoint.transform

        # Spawn the blocker vehicle
        blocker = CarlaDataProvider.request_new_actor(
            "vehicle.*.*", self._blocker_transform,
            attribute_filter={'base_type': 'car', 'has_lights': True, 'special_type': ''}
        )
        if blocker is None:
            raise Exception("Couldn't spawn the blocker vehicle")
        self.other_actors.append(blocker)

        blocker.set_simulate_physics(False)
        blocker.set_location(self._blocker_transform.location + carla.Location(z=-200))

        lights = blocker.get_light_state()
        lights |= carla.VehicleLightState.Brake
        blocker.set_light_state(carla.VehicleLightState(lights))

    def _create_behavior(self):
        """
        Just wait for a while after the ego closes in on the blocker, then remove it.
        """
        sequence = py_trees.composites.Sequence(name="BlockedIntersection")

        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=True,
                remove_entries=[],
                remove_exits=[],
                stop_entries=True,
                extend_road_exit=0
            ))
        # Ego go behind the blocker
        main_behavior = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        main_behavior.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        behavior = py_trees.composites.Sequence(name="Approach and Wait")
        behavior.add_child(ActorTransformSetter(self.other_actors[0], self._blocker_transform, True))
        behavior.add_child(HandBrakeVehicle(self.other_actors[0], 1))
        behavior.add_child(InTriggerDistanceToVehicle(
            self.other_actors[-1], self.ego_vehicles[0], self._trigger_distance))
        behavior.add_child(Idle(self._stop_time))
        main_behavior.add_child(behavior)

        sequence.add_child(main_behavior)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

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
