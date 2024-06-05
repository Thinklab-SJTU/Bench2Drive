#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenarios in which another (opposite) vehicle 'illegally' takes
priority, e.g. by running a red traffic light.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      OpenVehicleDoor,
                                                                      SwitchWrongDirectionTest,
                                                                      ScenarioTimeout,
                                                                      Idle,
                                                                      OppositeActorFlow)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance,
                                                                               WaitUntilInFrontPosition)
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import LeaveSpaceInFront, ChangeOppositeBehavior, StopBackVehicles, StartBackVehicles


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


class VehicleOpensDoorTwoWays(BasicScenario):
    """
    This class holds everything required for a scenario in which another vehicle parked at the side lane
    opens the door, forcing the ego to lane change, invading the opposite lane
    """
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()

        self.timeout = timeout
        self._min_trigger_dist = 10
        self._reaction_time = 3.0

        self._opposite_wait_duration = 5
        self._end_distance = 50

        self._parked_distance = get_value_parameter(config, 'distance', float, 50)
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        self._max_speed = get_value_parameter(config, 'speed', float, 60)
        self._scenario_timeout = 240

        self._opposite_interval = get_interval_parameter(config, 'frequency', float, [20, 100])

        super().__init__("VehicleOpensDoorTwoWays", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _get_displaced_location(self, actor, wp):
        """
        Calculates the transforming such that the actor is at the sidemost part of the lane
        """
        # Move the actor to the edge of the lane, or the open door might not reach the ego vehicle
        displacement = (wp.lane_width - actor.bounding_box.extent.y) / 4
        displacement_vector = wp.transform.get_right_vector()
        if self._direction == 'right':
            displacement_vector *= -1

        new_location = wp.transform.location + carla.Location(x=displacement*displacement_vector.x,
                                                              y=displacement*displacement_vector.y,
                                                              z=displacement*displacement_vector.z)
        new_location.z += 0.05  # Just in case, avoid collisions with the ground
        return new_location

    def _move_waypoint_forward(self, wp, distance):
        dist = 0
        next_wp = wp
        while dist < distance:
            next_wps = next_wp.next(1)
            if not next_wps or next_wps[0].is_junction:
                break
            next_wp = next_wps[0]
            dist += 1
        return next_wp

    def _initialize_actors(self, config):
        """
        Creates a parked vehicle on the side of the road
        """
        trigger_location = config.trigger_points[0].location
        starting_wp = self._map.get_waypoint(trigger_location)
        front_wps = starting_wp.next(self._parked_distance)
        if len(front_wps) == 0:
            raise ValueError("Couldn't find a spot to place the adversary vehicle")
        elif len(front_wps) > 1:
            print("WARNING: Found a diverging lane. Choosing one at random")
        self._front_wp = front_wps[0]

        if self._direction == 'left':
            self._parked_wp = self._front_wp.get_left_lane()
        else:
            self._parked_wp = self._front_wp.get_right_lane()

        if self._parked_wp is None:
            raise ValueError("Couldn't find a spot to place the adversary vehicle")

        self.parking_slots.append(self._parked_wp.transform.location)

        self._parked_actor = CarlaDataProvider.request_new_actor(
            "*vehicle.*", self._parked_wp.transform, attribute_filter={'has_dynamic_doors': True, 'base_type': 'car'})
        if not self._parked_actor:
            raise ValueError("Couldn't spawn the parked vehicle")
        self.other_actors.append(self._parked_actor)

        # And move it to the side
        side_location = self._get_displaced_location(self._parked_actor, self._parked_wp)
        self._parked_actor.set_location(side_location)
        self._parked_actor.apply_control(carla.VehicleControl(hand_brake=True))

        self._end_wp = self._move_waypoint_forward(self._front_wp, self._end_distance)

    def _create_behavior(self):
        """
        Leave space in front, as the TM doesn't detect open doors, and change the opposite frequency 
        so that the ego can pass
        """
        reference_wp = self._parked_wp.get_left_lane()
        if not reference_wp:
            raise ValueError("Couldnt find a left lane to spawn the opposite traffic")

        root = py_trees.composites.Sequence(name="VehicleOpensDoorTwoWays")
        if self.route_mode:
            total_dist = self._parked_distance + 20
            root.add_child(LeaveSpaceInFront(total_dist))

        end_condition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        end_condition.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._end_wp.transform, False))

        behavior = py_trees.composites.Sequence(name="Main Behavior")

        # Wait until ego is close to the adversary
        collision_location = self._front_wp.transform.location
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerOpenDoor")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        behavior.add_child(trigger_adversary)

        door = carla.VehicleDoor.FR if self._direction == 'left' else carla.VehicleDoor.FL
        behavior.add_child(OpenVehicleDoor(self._parked_actor, door))
        behavior.add_child(StopBackVehicles())
        behavior.add_child(Idle(self._opposite_wait_duration))
        if self.route_mode:
            behavior.add_child(SwitchWrongDirectionTest(False))
            behavior.add_child(ChangeOppositeBehavior(active=False))
        behavior.add_child(OppositeActorFlow(reference_wp, self.ego_vehicles[0], self._opposite_interval))

        end_condition.add_child(behavior)
        root.add_child(end_condition)

        if self.route_mode:
            root.add_child(SwitchWrongDirectionTest(True))
            root.add_child(ChangeOppositeBehavior(active=True))
        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))
            root.add_child(StartBackVehicles())

        return root

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
