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
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      ActorTransformSetter,
                                                                      SyncArrivalWithAgent,
                                                                      CutIn)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import HandleJunctionScenario

from srunner.tools.scenario_helper import generate_target_waypoint

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

class HighwayCutIn(BasicScenario):
    """
    This class holds everything required for a scenario in which another vehicle runs a red light
    in front of the ego, forcing it to react. This vehicles are 'special' ones such as police cars,
    ambulances or firetrucks.
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

        self._same_lane_time = 0.3
        self._other_lane_time = 3
        self._change_time = 2
        self._speed_perc = 80
        self._cut_in_distance = 10
        self._extra_space = 170

        self._start_location = convert_dict_to_location(config.other_parameters['other_actor_location'])

        super().__init__("HighwayCutIn",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_waypoint = self._map.get_waypoint(self._start_location)
        self._other_transform = self._other_waypoint.transform

        self._cut_in_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', self._other_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True}
        )
        self.other_actors.append(self._cut_in_vehicle)

        # Move below ground
        self._cut_in_vehicle.set_location(self._other_transform.location - carla.Location(z=100))
        self._cut_in_vehicle.set_simulate_physics(False)


    def _create_behavior(self):
        """
        Hero vehicle is entering a junction in an urban area, at a signalized intersection,
        while another actor runs a red lift, forcing the ego to break.
        """
        behavior = py_trees.composites.Sequence("HighwayCutIn")

        if self.route_mode:
            behavior.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=False,
                remove_entries=[self._other_waypoint],
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=self._extra_space
            ))
        behavior.add_child(ActorTransformSetter(self._cut_in_vehicle, self._other_transform))

        # Sync behavior
        target_wp = generate_target_waypoint(self._other_waypoint)
        front_wps = target_wp.next(self._cut_in_distance)
        if not front_wps:
            raise ValueError("Couldn't find a waypoint to perform the cut in")
        target_wp = front_wps[0]

        trigger_wp = self._map.get_waypoint(self.config.trigger_points[0].location)
        reference_wp = generate_target_waypoint(trigger_wp)
        behavior.add_child(SyncArrivalWithAgent(
            self._cut_in_vehicle, self.ego_vehicles[0], target_wp.transform, reference_wp.transform, 5))

        # Cut in
        behavior.add_child(CutIn(
            self._cut_in_vehicle, self.ego_vehicles[0], 'left', self._speed_perc,
            self._same_lane_time, self._other_lane_time, self._change_time, name="Cut_in")
        )
        behavior.add_child(ActorDestroy(self._cut_in_vehicle))
        return behavior

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
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
