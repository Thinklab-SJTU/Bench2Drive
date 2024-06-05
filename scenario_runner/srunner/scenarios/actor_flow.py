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

from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorFlow, ScenarioTimeout, WaitForever
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               WaitEndIntersection,
                                                                               WaitUntilInFrontPosition)
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import (SwitchRouteSources,
                                              ChangeOppositeBehavior,
                                              HandleJunctionScenario,
                                              RemoveRoadLane)
from srunner.tools.scenario_helper import get_same_dir_lanes, generate_target_waypoint_in_route

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

class EnterActorFlow(BasicScenario):
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

        ego_location = config.trigger_points[0].location
        self._reference_waypoint = CarlaDataProvider.get_map().get_waypoint(ego_location)

        self._sink_distance = 2

        self._start_actor_flow = convert_dict_to_location(config.other_parameters['start_actor_flow'])
        self._end_actor_flow = convert_dict_to_location(config.other_parameters['end_actor_flow'])

        self._flow_speed = get_value_parameter(config, 'flow_speed', float, 10)
        self._source_dist_interval = get_interval_parameter(config, 'source_dist_interval', float, [20, 50])
        self._scenario_timeout = 240

        super().__init__("EnterActorFlow",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _create_behavior(self):
        """
        Hero vehicle is entering a junction in an urban area, at a signalized intersection,
        while another actor runs a red lift, forcing the ego to break.
        """
        source_wp = self._map.get_waypoint(self._start_actor_flow)
        sink_wp = self._map.get_waypoint(self._end_actor_flow)

        # Get all lanes
        source_wps = get_same_dir_lanes(source_wp)
        sink_wps = get_same_dir_lanes(sink_wp)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        for source_wp, sink_wp in zip(source_wps, sink_wps):
            root.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], sink_wp.transform.location, self._sink_distance))
            root.add_child(ActorFlow(
                source_wp, sink_wp, self._source_dist_interval, self._sink_distance,
                self._flow_speed, initial_actors=True, initial_junction=True))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        sequence = py_trees.composites.Sequence()
        if self.route_mode:
            grp = CarlaDataProvider.get_global_route_planner()
            route = grp.trace_route(source_wp.transform.location, sink_wp.transform.location)
            extra_space = 20
            for i in range(-2, -len(route)-1, -1):
                current_wp = route[i][0]
                extra_space += current_wp.transform.location.distance(route[i+1][0].transform.location)
                if current_wp.is_junction:
                    break

            sequence.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=True,
                remove_entries=source_wps,
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=extra_space
            ))
            sequence.add_child(SwitchRouteSources(False))
        sequence.add_child(root)
        if self.route_mode:
            sequence.add_child(SwitchRouteSources(True))

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


class EnterActorFlowV2(EnterActorFlow):
    """
    Variation of EnterActorFlow for special highway entry exits with dedicated lanes
    """
    def _create_behavior(self):
        """
        Hero vehicle is entering a junction in an urban area, at a signalized intersection,
        while another actor runs a red lift, forcing the ego to break.
        """
        source_wp = self._map.get_waypoint(self._start_actor_flow)
        sink_wp = self._map.get_waypoint(self._end_actor_flow)

        # Get all lanes
        sink_wps = get_same_dir_lanes(sink_wp)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(ActorFlow(
                source_wp, sink_wp, self._source_dist_interval, self._sink_distance,
                self._flow_speed, initial_actors=True, initial_junction=True))
        for sink_wp in sink_wps:
            root.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], sink_wp.transform.location, self._sink_distance))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        exit_wp = generate_target_waypoint_in_route(self._reference_waypoint, self.config.route)
        exit_wp = exit_wp.next(10)[0]  # just in case the junction maneuvers don't match

        if self.route_mode:
            grp = CarlaDataProvider.get_global_route_planner()
            route = grp.trace_route(source_wp.transform.location, sink_wp.transform.location)
            self._extra_space = 20
            for i in range(-2, -len(route)-1, -1):
                current_wp = route[i][0]
                self._extra_space += current_wp.transform.location.distance(route[i+1][0].transform.location)
                if current_wp.is_junction:
                    break

            sequence_2 = py_trees.composites.Sequence()
            sequence_2.add_child(WaitEndIntersection(self.ego_vehicles[0]))
            sequence_2.add_child(HandleJunctionScenario(
                clear_junction=False,
                clear_ego_entry=False,
                remove_entries=[],
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=self._extra_space
            ))
            sequence_2.add_child(WaitForever())
            root.add_child(sequence_2)

        sequence = py_trees.composites.Sequence()
        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=False,
                clear_ego_entry=True,
                remove_entries=[source_wp],
                remove_exits= get_same_dir_lanes(exit_wp),
                stop_entries=False,
                extend_road_exit=0
            ))
            sequence.add_child(SwitchRouteSources(False))

        sequence.add_child(root)
        if self.route_mode:
            sequence.add_child(SwitchRouteSources(True))

        return sequence


class HighwayExit(BasicScenario):
    """
    This scenario is similar to CrossActorFlow
    It will remove the BackgroundActivity from the lane where ActorFlow starts.
    Then vehicles (cars) will start driving from start_actor_flow location to end_actor_flow location
    in a relatively high speed, forcing the ego to accelerate to cut in the actor flow 
    then exit from the highway.
    This scenario works when Background Activity is running in route mode. And there should be no junctions in front of the ego.
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

        self._start_actor_flow = convert_dict_to_location(config.other_parameters['start_actor_flow'])
        self._end_actor_flow = convert_dict_to_location(config.other_parameters['end_actor_flow'])

        self._sink_distance = 2
        self._end_distance = 40

        self._flow_speed = get_value_parameter(config, 'flow_speed', float, 10)
        self._source_dist_interval = get_interval_parameter(config, 'source_dist_interval', float, [20, 50])
        self._scenario_timeout = 240

        super().__init__("HighwayExit",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _create_behavior(self):
        """
        Vehicles run from the start to the end continuously.
        """
        source_wp = self._map.get_waypoint(self._start_actor_flow)
        sink_wp = self._map.get_waypoint(self._end_actor_flow)

        grp = CarlaDataProvider.get_global_route_planner()
        route = grp.trace_route(source_wp.transform.location, sink_wp.transform.location)
        junction_id = None
        for wp, _ in route:
            if wp.is_junction:
                junction_id = wp.get_junction().id
                break

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(ActorFlow(
            source_wp, sink_wp, self._source_dist_interval, self._sink_distance,
            self._flow_speed, initial_actors=True, initial_junction=True))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        root.add_child(WaitEndIntersection(self.ego_vehicles[0], junction_id))

        sequence = py_trees.composites.Sequence()

        if self.route_mode:
            sequence.add_child(RemoveRoadLane(source_wp))
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


class MergerIntoSlowTraffic(BasicScenario):
    """
    This scenario is similar to EnterActorFlow
    It will remove the BackgroundActivity from the lane where ActorFlow starts.
    Then vehicles (cars) will start driving from start_actor_flow location to end_actor_flow location
    in a relatively low speed, ego car must merger into this slow traffic flow.
    This scenario works when Background Activity is running in route mode. And applies to a confluence
    area at a highway intersection.
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

        ego_location = config.trigger_points[0].location
        self._reference_waypoint = CarlaDataProvider.get_map().get_waypoint(ego_location)

        self._start_actor_flow = convert_dict_to_location(config.other_parameters['start_actor_flow'])
        self._end_actor_flow = convert_dict_to_location(config.other_parameters['end_actor_flow'])
        self._trigger_point=config.trigger_points[0].location

        self._sink_distance = 2

        self._flow_speed = get_value_parameter(config, 'flow_speed', float, 10)
        self._source_dist_interval = get_interval_parameter(config, 'source_dist_interval', float, [20, 50])
        self._scenario_timeout = 240

        super().__init__("MergerIntoSlowTraffic",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _create_behavior(self):
        """
        the ego vehicle mergers into a slow traffic flow from the freeway entrance.
        """
        source_wp = self._map.get_waypoint(self._start_actor_flow)
        sink_wp = self._map.get_waypoint(self._end_actor_flow)

        # Get all lanes
        sink_wps = get_same_dir_lanes(sink_wp)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for wp in sink_wps:
            root.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], wp.transform.location, self._sink_distance))
        root.add_child(ActorFlow(
            source_wp, sink_wp, self._source_dist_interval, self._sink_distance,
            self._flow_speed, initial_actors=True, initial_junction=True))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        sequence = py_trees.composites.Sequence()
        if self.route_mode:

            grp = CarlaDataProvider.get_global_route_planner()
            route = grp.trace_route(source_wp.transform.location, sink_wp.transform.location)
            extra_space = 0
            for i in range(-2, -len(route)-1, -1):
                current_wp = route[i][0]
                extra_space += current_wp.transform.location.distance(route[i+1][0].transform.location)
                if current_wp.is_junction:
                    break

            sequence.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=True,
                remove_entries=[source_wp],
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=extra_space + 20
            ))
            sequence.add_child(SwitchRouteSources(False))
        sequence.add_child(root)
        if self.route_mode:
            sequence.add_child(SwitchRouteSources(True))

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


class MergerIntoSlowTrafficV2(MergerIntoSlowTraffic):
    """
    Variation of MergerIntoSlowTraffic 
    """

    def _create_behavior(self):
        """
        the ego vehicle mergers into a slow traffic flow from the freeway entrance.
        """
        source_wp = self._map.get_waypoint(self._start_actor_flow)
        sink_wp = self._map.get_waypoint(self._end_actor_flow)

        sink_wps = get_same_dir_lanes(sink_wp)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(ActorFlow(
            source_wp, sink_wp, self._source_dist_interval, self._sink_distance,
            self._flow_speed, initial_actors=True, initial_junction=True))
        for sink_wp in sink_wps:
            root.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], sink_wp.transform.location, self._sink_distance))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        exit_wp = generate_target_waypoint_in_route(self._reference_waypoint, self.config.route)
        exit_wp = exit_wp.next(10)[0]  # just in case the junction maneuvers don't match

        if self.route_mode:
            grp = CarlaDataProvider.get_global_route_planner()
            route = grp.trace_route(source_wp.transform.location, sink_wp.transform.location)
            self._extra_space = 20
            for i in range(-2, -len(route)-1, -1):
                current_wp = route[i][0]
                self._extra_space += current_wp.transform.location.distance(route[i+1][0].transform.location)
                if current_wp.is_junction:
                    break

        sequence_2 = py_trees.composites.Sequence()
        sequence_2.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        sequence_2.add_child(HandleJunctionScenario(
            clear_junction=False,
            clear_ego_entry=False,
            remove_entries=[],
            remove_exits=[],
            stop_entries=False,
            extend_road_exit=self._extra_space
        ))
        sequence_2.add_child(WaitForever())
        root.add_child(sequence_2)

        sequence = py_trees.composites.Sequence()
        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=False,
                clear_ego_entry=True,
                remove_entries=[source_wp],
                remove_exits=get_same_dir_lanes(exit_wp),
                stop_entries=False,
                extend_road_exit=0
            ))
            sequence.add_child(SwitchRouteSources(False))
        sequence.add_child(root)
        if self.route_mode:
            sequence.add_child(SwitchRouteSources(True))

        return sequence


class InterurbanActorFlow(BasicScenario):
    """
    Scenario specifically made for the interurban intersections,
    where the ego leaves the interurban road by turning left, crossing an actor flow.
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

        self._start_actor_flow = convert_dict_to_location(config.other_parameters['start_actor_flow'])
        self._end_actor_flow = convert_dict_to_location(config.other_parameters['end_actor_flow'])

        self._sink_distance = 2
        self._end_distance = 40

        self._flow_speed = get_value_parameter(config, 'flow_speed', float, 10)
        self._source_dist_interval = get_interval_parameter(config, 'source_dist_interval', float, [20, 50])
        self._scenario_timeout = 240

        self._reference_wp = self._map.get_waypoint(config.trigger_points[0].location)

        route_entry_wp, route_exit_wp = self._get_entry_exit_route_lanes(self._reference_wp, config.route)
        route_exit_wp = route_exit_wp.next(8)[0]  # Just in case the junction maneuvers don't match
        other_entry_wp = route_exit_wp.get_left_lane()
        if not other_entry_wp or other_entry_wp.lane_type != carla.LaneType.Driving:
            raise ValueError("Couldn't find an end position")

        self._source_wp = self._map.get_waypoint(self._start_actor_flow)
        self._sink_wp = self._map.get_waypoint(self._end_actor_flow)

        self._remove_entries = [route_entry_wp, other_entry_wp, self._source_wp]

        super().__init__("InterurbanActorFlow",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _get_entry_exit_route_lanes(self, wp, route):

        entry_wp = None
        exit_wp = None

        # Get the middle entry
        dist = float('inf')
        index = 0
        for route_index, route_pos in enumerate(route):
            route_location = route_pos[0].location
            trigger_location = wp.transform.location

            route_dist = trigger_location.distance(route_location)
            if route_dist <= dist:
                index = route_index
                dist = route_dist

        reached_junction = False
        for i in range(index, len(route)):
            route_transform, road_option = route[i]

            # Enter the junction
            if not reached_junction and (road_option in (RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT)):
                reached_junction = True
                entry_wp = self._map.get_waypoint(route[i-1][0].location)
                entry_wp = entry_wp.previous(2)[0]  # Just in case

            # End condition for the behavior, at the end of the junction
            if reached_junction and (road_option not in (RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT)):
                exit_wp = self._map.get_waypoint(route_transform.location)
                exit_wp = exit_wp.next(2)[0]  # Just in case
                break

        return (entry_wp, exit_wp)


    def _create_behavior(self):
        """
        Create an actor flow at the opposite lane which the ego has to cross
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(ActorFlow(
            self._source_wp, self._sink_wp, self._source_dist_interval, self._sink_distance, self._flow_speed))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        root.add_child(WaitEndIntersection(self.ego_vehicles[0]))

        sequence = py_trees.composites.Sequence()

        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=False,
                clear_ego_entry=True,
                remove_entries=self._remove_entries,
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=0
            ))
            sequence.add_child(ChangeOppositeBehavior(active=False))
        sequence.add_child(root)
        if self.route_mode:
            sequence.add_child(ChangeOppositeBehavior(active=True))

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


class InterurbanAdvancedActorFlow(BasicScenario):
    """
    Scenario specifically made for the interurban intersections,
    where the ego incorportates into the interurban road by turning left,
    first crossing an actor flow, and then merging into another one.
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

        self._sink_distance = 2

        self._reference_wp = self._map.get_waypoint(config.trigger_points[0].location)
        self._exit_wp = generate_target_waypoint_in_route(self._reference_wp, config.route)

        self._start_actor_flow_1 = convert_dict_to_location(config.other_parameters['start_actor_flow'])
        self._end_actor_flow_1 = convert_dict_to_location(config.other_parameters['end_actor_flow'])

        self._flow_speed = get_value_parameter(config, 'flow_speed', float, 10)
        self._source_dist_interval = get_interval_parameter(config, 'source_dist_interval', float, [20, 50])
        self._scenario_timeout = 240

        super().__init__("InterurbanAdvancedActorFlow",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def get_lane_key(self, waypoint):
        return str(waypoint.road_id) + '*' + str(waypoint.lane_id)

    def _get_junction_entry_wp(self, entry_wp):
        while entry_wp.is_junction:
            entry_wps = entry_wp.previous(0.2)
            if len(entry_wps) == 0:
                return None  # Stop when there's no prev
            entry_wp = entry_wps[0]
        return entry_wp

    def _get_junction_exit_wp(self, exit_wp):
        while exit_wp.is_junction:
            exit_wps = exit_wp.next(0.2)
            if len(exit_wps) == 0:
                return None  # Stop when there's no prev
            exit_wp = exit_wps[0]
        return exit_wp

    def _initialize_actors(self, config):
        
        self._source_wp_1 = self._map.get_waypoint(self._start_actor_flow_1)
        self._sink_wp_1 = self._map.get_waypoint(self._end_actor_flow_1)

        self._source_wp_2 = self._sink_wp_1.get_left_lane()
        if not self._source_wp_2 or self._source_wp_2.lane_type != carla.LaneType.Driving:
            raise ValueError("Couldn't find a position for the actor flow")
        self._sink_wp_2 = self._source_wp_1.get_left_lane()
        if not self._sink_wp_2 or self._sink_wp_2.lane_type != carla.LaneType.Driving:
            raise ValueError("Couldn't find a position for the actor flow")

        if self.route_mode:
            grp = CarlaDataProvider.get_global_route_planner()
            route = grp.trace_route(self._source_wp_2.transform.location, self._sink_wp_2.transform.location)
            self._extra_space = 20
            route_exit_wp = None
            for i in range(-2, -len(route)-1, -1):
                current_wp = route[i][0]
                self._extra_space += current_wp.transform.location.distance(route[i+1][0].transform.location)
                if current_wp.is_junction:
                    junction = current_wp.get_junction()
                    break
                route_exit_wp = current_wp

            route_exit_key = self.get_lane_key(route_exit_wp)

            # Get the route entry waypoint
            route_entry_wp = self._reference_wp
            while True:
                next_wps = route_entry_wp.next(1)
                if not next_wps:
                    break
                if next_wps[0].is_junction:
                    break
                route_entry_wp = next_wps[0]
            route_entry_key = self.get_lane_key(route_entry_wp)

        entry_wps = []
        entry_keys = []
        exit_wps = []
        exit_keys = []

        for entry_wp, exit_wp in junction.get_waypoints(carla.LaneType.Driving):

            entry_wp = self._get_junction_entry_wp(entry_wp)
            entry_key = self.get_lane_key(entry_wp)
            if entry_key != route_entry_key and entry_key not in entry_keys:
                entry_wps.append(entry_wp)
                entry_keys.append(entry_key)

            exit_wp = self._get_junction_exit_wp(exit_wp)
            exit_key = self.get_lane_key(exit_wp)
            if exit_key != route_exit_key and exit_key not in exit_keys:
                exit_wps.append(exit_wp)
                exit_keys.append(exit_key)

        self._remove_entries = entry_wps
        self._remove_exits = exit_wps

    def _create_behavior(self):
        """
        the ego vehicle mergers into a slow traffic flow from the freeway entrance.
        """
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._sink_wp_2.transform))
        root.add_child(ActorFlow(
            self._source_wp_1, self._sink_wp_1, self._source_dist_interval, self._sink_distance, self._flow_speed))
        root.add_child(ActorFlow(
            self._source_wp_2, self._sink_wp_2, self._source_dist_interval, self._sink_distance, self._flow_speed))
        root.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        sequence = py_trees.composites.Sequence()
        if self.route_mode:

            sequence.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=True,
                remove_entries=self._remove_entries,
                remove_exits=self._remove_exits,
                stop_entries=False,
                extend_road_exit=self._extra_space
            ))
            sequence.add_child(SwitchRouteSources(False))
            sequence.add_child(ChangeOppositeBehavior(active=False))

        sequence.add_child(root)

        if self.route_mode:
            sequence.add_child(SwitchRouteSources(True))
            sequence.add_child(ChangeOppositeBehavior(active=True))

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
