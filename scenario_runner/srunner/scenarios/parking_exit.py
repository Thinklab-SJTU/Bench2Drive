#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario in which the ego is parked between two vehicles and has to maneuver to start the route.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      ActorTransformSetter,
                                                                      WaitForever,
                                                                      ChangeAutoPilot,
                                                                      ScenarioTimeout)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import ChangeRoadBehavior


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


class ParkingExit(BasicScenario):
    """
    This class holds everything required for a scenario in which the ego would be teleported to the parking lane.
    Once the scenario is triggered, the OutsideRouteLanesTest will be deactivated since the ego is out of the driving lane.
    Then blocking vehicles will be generated in front of and behind the parking point.
    The ego need to exit from the parking lane and then merge into the driving lane.
    After the ego is {end_distance} meters away from the parking point, the OutsideRouteLanesTest will be activated and the scenario ends.

    Note 1: For route mode, this shall be the first scenario of the route. The trigger point shall be the first point of the route waypoints.

    Note 2: Make sure there are enough space for spawning blocking vehicles.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._tm = CarlaDataProvider.get_client().get_trafficmanager(
            CarlaDataProvider.get_traffic_manager_port())
        self.timeout = timeout

        self._bp_attributes = {'base_type': 'car', 'generation': 2}
        self._side_end_distance = 50

        self._front_vehicle_distance = get_value_parameter(config, 'front_vehicle_distance', float, 20)
        self._behind_vehicle_distance = get_value_parameter(config, 'behind_vehicle_distance', float, 10)
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        self._flow_distance = get_value_parameter(config, 'flow_distance', float, 25)
        self._max_speed = get_value_parameter(config, 'speed', float, 60)
        self._scenario_timeout = 240

        self._end_distance = self._front_vehicle_distance + 15

        # Get parking_waypoint based on trigger_point
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(self._trigger_location)
        if self._direction == "left":
            self._parking_waypoint = self._reference_waypoint.get_left_lane()
        else:
            self._parking_waypoint = self._reference_waypoint.get_right_lane()

        if self._parking_waypoint is None:
            raise Exception(
                "Couldn't find parking point on the {} side".format(self._direction))

        super().__init__("ParkingExit",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # Spawn the actor in front of the ego
        front_points = self._parking_waypoint.next(
            self._front_vehicle_distance)
        if not front_points:
            raise ValueError("Couldn't find viable position for the vehicle in front of the parking point")

        self.parking_slots.append(front_points[0].transform.location)

        actor_front = CarlaDataProvider.request_new_actor(
            'vehicle.*', front_points[0].transform, rolename='scenario no lights', attribute_filter=self._bp_attributes)
        if actor_front is None:
            raise ValueError("Couldn't spawn the vehicle in front of the parking point")
        actor_front.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(actor_front)

        # And move it to the side
        side_location = self._get_displaced_location(actor_front, front_points[0])
        actor_front.set_location(side_location)

        # Spawn the actor behind the ego
        behind_points = self._parking_waypoint.previous(
            self._behind_vehicle_distance)
        if not behind_points:
            raise ValueError("Couldn't find viable position for the vehicle behind the parking point")

        self.parking_slots.append(behind_points[0].transform.location)

        actor_behind = CarlaDataProvider.request_new_actor(
            'vehicle.*', behind_points[0].transform, rolename='scenario no lights', attribute_filter=self._bp_attributes)
        if actor_behind is None:
            actor_front.destroy()
            raise ValueError("Couldn't spawn the vehicle behind the parking point")
        actor_behind.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(actor_behind)

        # And move it to the side
        side_location = self._get_displaced_location(actor_behind, behind_points[0])
        actor_behind.set_location(side_location)

        # Move the ego to its side position
        self._ego_location = self._get_displaced_location(self.ego_vehicles[0], self._parking_waypoint)
        self.parking_slots.append(self._ego_location)
        self.ego_vehicles[0].set_location(self._ego_location)

        # Spawn the actor at the side of the ego
        actor_side = CarlaDataProvider.request_new_actor(
            'vehicle.*', self._reference_waypoint.transform, attribute_filter=self._bp_attributes)
        if actor_side is None:
            raise ValueError("Couldn't spawn the vehicle at the side of the parking point")
        self.other_actors.append(actor_side)
        self._tm.update_vehicle_lights(actor_side, True)

        self._end_side_transform = self.ego_vehicles[0].get_transform()
        self._end_side_transform.location.z -= 500

    def _get_displaced_location(self, actor, wp):
        """
        Calculates the transforming such that the actor is at the sidemost part of the lane
        """
        # Move the actor to the edge of the lane near the sidewalk
        displacement = (wp.lane_width - actor.bounding_box.extent.y) / 4
        displacement_vector = wp.transform.get_right_vector()
        if self._direction == 'left':
            displacement_vector *= -1

        new_location = wp.transform.location + carla.Location(x=displacement*displacement_vector.x,
                                                              y=displacement*displacement_vector.y,
                                                              z=displacement*displacement_vector.z)
        new_location.z += 0.05  # Just in case, avoid collisions with the ground
        return new_location

    def _create_behavior(self):
        """
        Deactivate OutsideRouteLanesTest, then move ego to the parking point,
        generate blocking vehicles in front of and behind the ego.
        After ego drives away, activate OutsideRouteLanesTest, end scenario.
        """

        sequence = py_trees.composites.Sequence(name="ParkingExit")
        sequence.add_child(ChangeRoadBehavior(spawn_dist=self._flow_distance))
        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        side_actor_behavior = py_trees.composites.Sequence()
        side_actor_behavior.add_child(ChangeAutoPilot(self.other_actors[2], True))
        side_actor_behavior.add_child(DriveDistance(self.other_actors[2], self._side_end_distance))
        side_actor_behavior.add_child(ActorTransformSetter(self.other_actors[2], self._end_side_transform, False))
        side_actor_behavior.add_child(WaitForever())
        root.add_child(side_actor_behavior)

        end_condition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        end_condition.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        root.add_child(end_condition)

        sequence.add_child(root)

        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))
        sequence.add_child(ChangeRoadBehavior(spawn_dist=15))

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
