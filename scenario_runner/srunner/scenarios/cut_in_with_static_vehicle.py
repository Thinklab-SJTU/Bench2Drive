#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
# Copyright (c) 2019-2022 Intel Corporation

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import print_function

import py_trees
import carla

from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      BatchActorTransformSetter,
                                                                      CutIn,
                                                                      BasicAgentBehavior,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.background_manager import RemoveRoadLane, LeaveSpaceInFront, ReAddRoadLane, ChangeRoadBehavior


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default


class StaticCutIn(BasicScenario):

    """
    Cut in(with static vehicle) scenario synchronizes a vehicle that is parked at a side lane
    to cut in in front of the ego vehicle, forcing it to break
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)

        self._reaction_time = 2.7  # Time the agent has to react to avoid the collision [s]
        self._min_trigger_dist = 15.0  # Min distance to the collision location that triggers the adversary [m]

        self._back_vehicles = 2
        self._front_vehicles = 3
        self._vehicle_gap = 11

        self._speed = 60 # Km/h

        self._adversary_end_distance = 70

        self._extra_space = 30  # Leave extra space as a vehicle is invading the ego's lane (BA parameter)

        self._side_transforms = []
        self._side_wp = None

        self._attributes = {'base_type': 'car', 'has_lights': True}

        self._blocker_distance = get_value_parameter(config, 'distance', float, 100)
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        super().__init__("StaticCutIn",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Spawn the blocker vehicle
        next_wps = self._reference_waypoint.next(self._blocker_distance)
        if not next_wps:
            raise ValueError("Couldn't find a proper position for the cut in vehicle")
        blocker_wp = next_wps[0]

        # Spawn the vehicles behind the cut in one
        for i in range(self._back_vehicles):
            # Move to the side
            side_wp = blocker_wp.get_left_lane() if self._direction == 'left' else blocker_wp.get_right_lane()
            if not side_wp:
                for actor in self.other_actors:
                    actor.destroy()
                raise ValueError("Couldn't find a proper position for the cut in vehicle")

            if i == 1:
                self._side_wp = side_wp

            # Spawn the actor
            blocker_actor = CarlaDataProvider.request_new_actor(
                'vehicle.*', side_wp.transform, 'scenario', attribute_filter=self._attributes)
            if not blocker_actor:
                for actor in self.other_actors:
                    actor.destroy()
                raise ValueError("Couldn't spawn an actor")
            blocker_actor.apply_control(carla.VehicleControl(hand_brake=True))

            blocker_actor.set_simulate_physics(False)
            blocker_actor.set_location(side_wp.transform.location + carla.Location(z=-500))
            self._side_transforms.append([blocker_actor, side_wp.transform])
            self.other_actors.append(blocker_actor)

            # Move to the front
            next_wps = blocker_wp.next(self._vehicle_gap)
            if not next_wps:
                for actor in self.other_actors:
                    actor.destroy()
                raise ValueError("Couldn't find a proper position for the cut in vehicle")
            blocker_wp = next_wps[0]

        self._collision_wp = blocker_wp

        # Get the cut in behavior
        self._plan, dist, step = ([], 0, 5)
        next_wp = self._collision_wp
        while dist < self._adversary_end_distance:
            next_wps = next_wp.next(step)
            if not next_wps:
                for actor in self.other_actors:
                    actor.destroy()
                raise ValueError("Couldn't find a proper position for the cut in vehicle")
            next_wp = next_wps[0]
            self._plan.append([next_wp, RoadOption.STRAIGHT])

            dist += step

        # Spawn the cut in vehicle
        side_wp = blocker_wp.get_left_lane() if self._direction == 'left' else blocker_wp.get_right_lane()
        if not side_wp:
            for actor in self.other_actors:
                actor.destroy()
            raise ValueError("Couldn't find a proper position for the cut in vehicle")

        self._adversary_actor = CarlaDataProvider.request_new_actor(
            'vehicle.*', side_wp.transform, 'scenario', attribute_filter=self._attributes)
        if not self._adversary_actor:
            for actor in self.other_actors:
                actor.destroy()
            raise ValueError("Couldn't spawn an actor")

        self._adversary_actor.set_simulate_physics(False)
        self._adversary_actor.set_location(side_wp.transform.location + carla.Location(z=-500))
        self._side_transforms.append([self._adversary_actor, side_wp.transform])
        self.other_actors.append(self._adversary_actor)

        # This starts the engine, to allow the adversary to instantly move 
        self._adversary_actor.apply_control(carla.VehicleControl(throttle=1.0, brake=1.0)) 

        # Move to the front
        next_wps = blocker_wp.next(self._vehicle_gap)
        if not next_wps:
            for actor in self.other_actors:
                actor.destroy()
            raise ValueError("Couldn't find a proper position for the cut in vehicle")
        blocker_wp = next_wps[0]

        # Spawn the vehicles in front of the cut in one
        for i in range(self._front_vehicles):
            # Move to the side
            side_wp = blocker_wp.get_left_lane() if self._direction == 'left' else blocker_wp.get_right_lane()
            if not side_wp:
                for actor in self.other_actors:
                    actor.destroy()
                raise ValueError("Couldn't find a proper position for the cut in vehicle")

            # Spawn the actor
            blocker_actor = CarlaDataProvider.request_new_actor(
                'vehicle.*', side_wp.transform, 'scenario', attribute_filter=self._attributes)
            if not blocker_actor:
                for actor in self.other_actors:
                    actor.destroy()
                raise ValueError("Couldn't spawn an actor")
            blocker_actor.apply_control(carla.VehicleControl(hand_brake=True))

            blocker_actor.set_simulate_physics(False)
            blocker_actor.set_location(side_wp.transform.location + carla.Location(z=-500))
            self._side_transforms.append([blocker_actor, side_wp.transform])
            self.other_actors.append(blocker_actor)

            # Move to the front
            next_wps = blocker_wp.next(self._vehicle_gap)
            if not next_wps:
                for actor in self.other_actors:
                    actor.destroy()
                raise ValueError("Couldn't find a proper position for the cut in vehicle")
            blocker_wp = next_wps[0]

    def _create_behavior(self):
        """
        After invoking this scenario, a parked vehicle will wait for the ego to
        be close-by, merging into its lane, forcing it to break.
        """
        sequence = py_trees.composites.Sequence(name="StaticCutIn")
        if self.route_mode:
            total_dist = self._blocker_distance
            total_dist += self._vehicle_gap * (self._back_vehicles + self._front_vehicles + 1)
            sequence.add_child(LeaveSpaceInFront(total_dist))

        sequence.add_child(BatchActorTransformSetter(self._side_transforms))

        collision_location = self._collision_wp.transform.location

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))

        sequence.add_child(trigger_adversary)
        if self.route_mode:
            sequence.add_child(ChangeRoadBehavior(extra_space=self._extra_space))

        if self.route_mode:
            sequence.add_child(RemoveRoadLane(self._side_wp))

        cut_in_behavior = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="CutIn")
        cut_in_direction = 'right' if self._direction == 'left' else 'left'

        cut_in_movement = py_trees.composites.Sequence()
        cut_in_movement.add_child(CutIn(
            self._adversary_actor, self.ego_vehicles[0], cut_in_direction, change_time=3, other_lane_time=2))
        cut_in_movement.add_child(BasicAgentBehavior(
            self._adversary_actor, plan=self._plan, target_speed=self._speed))

        cut_in_behavior.add_child(cut_in_movement)
        cut_in_behavior.add_child(Idle(30))  # Timeout in case a collision happened

        sequence.add_child(cut_in_behavior)

        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))

        if self.route_mode:
            sequence.add_child(ChangeRoadBehavior(extra_space=0))
            sequence.add_child(ReAddRoadLane(1 if self._direction == 'right' else -1))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
