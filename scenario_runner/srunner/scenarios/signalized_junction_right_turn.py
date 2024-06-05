#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Collection of traffic scenarios where the ego vehicle (hero)
is making a right turn
"""

from __future__ import print_function

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorFlow, TrafficLightFreezer, ScenarioTimeout
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitEndIntersection, DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (generate_target_waypoint,
                                           get_junction_topology,
                                           filter_junction_wp_direction,
                                           get_same_dir_lanes,
                                           get_closest_traffic_light)

from srunner.tools.background_manager import HandleJunctionScenario


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


class JunctionRightTurn(BasicScenario):
    """
    Scenario where the vehicle is turning right at an intersection an has to avoid
    colliding with vehicles coming from its left
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._direction = 'left'

        self._green_light_delay = 5  # Wait before the ego's lane traffic light turns green
        self._flow_tl_dict = {}
        self._init_tl_dict = {}

        self._end_distance = 10  # Distance after the junction before the scenario ends

        self._flow_speed = get_value_parameter(config, 'flow_speed', float, 20)
        self._source_dist_interval = get_interval_parameter(config, 'source_dist_interval', float, [25, 50])
        self._scenario_timeout = 240

        # The faster the flow, the further they are spawned, leaving time to react to them
        self._source_dist = 5 * self._flow_speed
        self._sink_dist = 3 * self._flow_speed

        super().__init__("JunctionRightTurn",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        ego_location = config.trigger_points[0].location
        self._ego_wp = CarlaDataProvider.get_map().get_waypoint(ego_location)

        # Get the junction
        starting_wp = self._ego_wp
        ego_junction_dist = 0
        while not starting_wp.is_junction:
            starting_wps = starting_wp.next(1.0)
            if len(starting_wps) == 0:
                raise ValueError("Failed to find a junction")
            starting_wp = starting_wps[0]
            ego_junction_dist += 1
        self._junction = starting_wp.get_junction()

        # Get the source entry lane wp
        entry_wps, _ = get_junction_topology(self._junction)
        source_entry_wps = filter_junction_wp_direction(starting_wp, entry_wps, self._direction)
        if not source_entry_wps:
            raise ValueError("Trying to find a lane in the {} direction but none was found".format(self._direction))

        # Get the rightmost lane
        source_entry_wp = source_entry_wps[0]
        while True:
            right_wp = source_entry_wp.get_right_lane()
            if not right_wp or right_wp.lane_type != carla.LaneType.Driving:
                break
            source_entry_wp = right_wp

        # Get the source transform
        source_wp = source_entry_wp
        source_junction_dist = 0
        while source_junction_dist < self._source_dist:
            source_wps = source_wp.previous(5)
            if len(source_wps) == 0:
                raise ValueError("Failed to find a source location")
            if source_wps[0].is_junction:
                break
            source_wp = source_wps[0]
            source_junction_dist += 5

        self._source_wp = source_wp
        source_transform = self._source_wp.transform

        # Get the sink location
        sink_exit_wp = generate_target_waypoint(self._map.get_waypoint(source_transform.location), 0)
        sink_wps = sink_exit_wp.next(self._sink_dist)
        if len(sink_wps) == 0:
            raise ValueError("Failed to find a sink location")
        self._sink_wp = sink_wps[0]

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


class SignalizedJunctionRightTurn(JunctionRightTurn):
    """
    Signalized version of JunctionRightTurn
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        super()._initialize_actors(config)

        tls = self._world.get_traffic_lights_in_junction(self._junction.id)
        ego_tl = get_closest_traffic_light(self._ego_wp, tls)
        source_tl = get_closest_traffic_light(self._source_wp, tls)

        for tl in tls:
            if tl.id == ego_tl.id:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Green
                self._init_tl_dict[tl] = carla.TrafficLightState.Red
            elif tl.id == source_tl.id:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Green
                self._init_tl_dict[tl] = carla.TrafficLightState.Green
            else:
                self._flow_tl_dict[tl] = carla.TrafficLightState.Red
                self._init_tl_dict[tl] = carla.TrafficLightState.Red

    def _create_behavior(self):
        """
        Hero vehicle is turning right in an urban area, at a signalized intersection,
        while other actor coming straight from the left. The ego has to avoid colliding with it
        """

        sequence = py_trees.composites.Sequence(name="JunctionRightTurn")
        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=True,
                remove_entries=get_same_dir_lanes(self._source_wp),
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=self._sink_dist + 20
            ))

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)
        root.add_child(ActorFlow(
            self._source_wp, self._sink_wp, self._source_dist_interval, 2, self._flow_speed, initial_actors=True))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        tl_freezer_sequence = py_trees.composites.Sequence("Traffic Light Behavior")
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._init_tl_dict, duration=self._green_light_delay))
        tl_freezer_sequence.add_child(TrafficLightFreezer(self._flow_tl_dict))
        root.add_child(tl_freezer_sequence)

        sequence.add_child(root)
        return sequence


class NonSignalizedJunctionRightTurn(JunctionRightTurn):
    """
    Non signalized version of JunctionRightTurn
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _create_behavior(self):
        """
        Hero vehicle is turning right in an urban area, at a signalized intersection,
        while other actor coming straight from the left. The ego has to avoid colliding with it
        """
        sequence = py_trees.composites.Sequence(name="JunctionRightTurn")
        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=True,
                remove_entries=get_same_dir_lanes(self._source_wp),
                remove_exits=[],
                stop_entries=True,
                extend_road_exit=self._sink_dist + 20
            ))

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition = py_trees.composites.Sequence()
        end_condition.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        root.add_child(end_condition)
        root.add_child(ActorFlow(
            self._source_wp, self._sink_wp, self._source_dist_interval, 2, self._flow_speed, initial_actors=True))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        sequence.add_child(root)
        return sequence
