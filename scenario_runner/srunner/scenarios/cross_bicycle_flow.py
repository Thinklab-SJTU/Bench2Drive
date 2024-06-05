#!/usr/bin/env python

# Copyright (c) 2018-2022 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenarios in which the ego has to cross a flow of bycicles
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import BicycleFlow, TrafficLightFreezer, ScenarioTimeout
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitEndIntersection
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import HandleJunctionScenario
from agents.navigation.local_planner import RoadOption


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


def get_interval_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return [
            p_type(config.other_parameters[name]['from']),
            p_type(config.other_parameters[name]['to'])
        ]
    else:
        return default

class CrossingBicycleFlow(BasicScenario):
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

        self._start_flow = convert_dict_to_location(config.other_parameters['start_actor_flow'])
        self._end_dist_flow = 40  # m
        self._sink_distance = 2

        self._end_distance = 40

        self._signalized_junction = False

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._green_light_delay = 5
        self._scenario_timeout = 240
        self._flow_speed = get_value_parameter(config, 'flow_speed', float, 10)
        self._source_dist_interval = get_interval_parameter(config, 'source_dist_interval', float, [20, 50])

        super().__init__("CrossingBicycleFlow",
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
        junction = starting_wp.get_junction()

        # Get the plan
        self._source_wp = self._map.get_waypoint(self._start_flow, lane_type=carla.LaneType.Biking)
        if not self._source_wp or self._source_wp.transform.location.distance(self._start_flow) > 10:
            raise ValueError("Couldn't find a biking lane at the specified location")

        self._plan = []
        plan_step = 0
        wp = self._source_wp
        while True:
            next_wps = wp.next(2)
            if not next_wps:
                raise ValueError("Couldn't find a proper plan for the bicycle flow")
            next_wp = next_wps
            wp = next_wp[0]
            self._plan.append([next_wp[0], RoadOption.LANEFOLLOW])

            if plan_step == 0 and wp.is_junction:
                plan_step += 1
            elif plan_step == 1 and not wp.is_junction:
                plan_step += 1
                exit_loc = wp.transform.location
            elif plan_step == 2 and exit_loc.distance(wp.transform.location) > self._end_dist_flow:
                break

        tls = self._world.get_traffic_lights_in_junction(junction.id)
        if not tls:
            self._signalized_junction = False
        else:
            self._signalized_junction = True
            self._get_traffic_lights(tls, ego_junction_dist)

    def _get_traffic_lights(self, tls, ego_dist):
        """Get the traffic light of the junction, mapping their states"""

        ego_landmark = self._ego_wp.get_landmarks_of_type(ego_dist + 2, "1000001")[0]
        ego_tl = self._world.get_traffic_light(ego_landmark)
        self._flow_tl_dict = {}
        self._init_tl_dict = {}
        for tl in tls:
            if tl.id == ego_tl.id:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Green
                self._init_tl_dict[tl] = carla.TrafficLightState.Red
            else:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Red


    def _create_behavior(self):
        """
        Hero vehicle is entering a junction in an urban area, at a signalized intersection,
        while another actor runs a red lift, forcing the ego to break.
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(BicycleFlow(self._plan, self._source_dist_interval, self._sink_distance, self._flow_speed, True))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        root.add_child(WaitEndIntersection(self.ego_vehicles[0]))

        # Freeze the traffic lights to allow the flow to populate the junction
        if self._signalized_junction:
            tl_freezer_sequence = py_trees.composites.Sequence("Traffic Light Behavior")
            tl_freezer_sequence.add_child(TrafficLightFreezer(self._init_tl_dict, duration=self._green_light_delay))
            tl_freezer_sequence.add_child(TrafficLightFreezer(self._flow_tl_dict))
            root.add_child(tl_freezer_sequence)

        # Add the BackgroundActivity behaviors
        if not self.route_mode:
            return root

        sequence = py_trees.composites.Sequence()
        sequence.add_child(HandleJunctionScenario(
            clear_junction=True,
            clear_ego_entry=True,
            remove_entries=[],
            remove_exits=[],
            stop_entries=True,
            extend_road_exit=0
        ))
        sequence.add_child(root)
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
