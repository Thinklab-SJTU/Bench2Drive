#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario spawning elements to make the town dynamic and interesting
"""

from collections import OrderedDict
import py_trees

import carla

from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.tools.scenario_helper import get_same_dir_lanes, get_opposite_dir_lanes

JUNCTION_ENTRY = 'entry'
JUNCTION_MIDDLE = 'middle'
JUNCTION_EXIT = 'exit'
JUNCTION_EXIT_ROAD = 'exit_road'
JUNCTION_EXIT_INACTIVE = 'exit_inactive'

EGO_JUNCTION = 'junction'
EGO_ROAD = 'road'

def get_lane_key(waypoint):
    """Returns a key corresponding to the waypoint lane. Equivalent to a 'Lane'
    object and used to compare waypoint lanes"""
    return '' if waypoint is None else get_road_key(waypoint) + '*' + str(waypoint.lane_id)

def get_road_key(waypoint):
    """Returns a key corresponding to the waypoint road. Equivalent to a 'Road'
    object and used to compare waypoint roads"""
    return '' if waypoint is None else str(waypoint.road_id)

def is_lane_at_road(lane_key, road_key):
    """Returns whether or not a lane is part of a road"""
    return lane_key.startswith(road_key)

def get_lane_key_from_ids(road_id, lane_id):
    """Returns the lane corresping to a given road and lane ids"""
    return str(road_id) + '*' + str(lane_id)


# Debug variables
DEBUG_ROAD = 'road'
DEBUG_OPPOSITE = 'opposite'
DEBUG_JUNCTION = 'junction'
DEBUG_ENTRY = 'entry'
DEBUG_EXIT = 'exit'
DEBUG_CONNECT = 'connect'

DEBUG_SMALL = 'small'
DEBUG_MEDIUM = 'medium'
DEBUG_LARGE = 'large'

DEBUG_COLORS = {
    DEBUG_ROAD: carla.Color(0, 0, 255),      # Blue
    DEBUG_OPPOSITE: carla.Color(255, 0, 0),  # Red
    DEBUG_JUNCTION: carla.Color(0, 0, 0),    # Black
    DEBUG_ENTRY: carla.Color(255, 255, 0),   # Yellow
    DEBUG_EXIT: carla.Color(0, 255, 255),    # Teal
    DEBUG_CONNECT: carla.Color(0, 255, 0),   # Green
}

DEBUG_TYPE = {
    DEBUG_SMALL: [0.8, 0.1],
    DEBUG_MEDIUM: [0.5, 0.15],
    DEBUG_LARGE: [0.2, 0.2],
}  # Size, height

def draw_string(world, location, string='', debug_type=DEBUG_ROAD, persistent=False):
    """Utility function to draw debugging strings"""
    v_shift, _ = DEBUG_TYPE.get(DEBUG_SMALL)
    l_shift = carla.Location(z=v_shift)
    color = DEBUG_COLORS.get(debug_type, DEBUG_ROAD)
    life_time = 0.06 if not persistent else 100000
    world.debug.draw_string(location + l_shift, string, False, color, life_time)

def draw_point(world, location, point_type=DEBUG_SMALL, debug_type=DEBUG_ROAD, persistent=False):
    """Utility function to draw debugging points"""
    v_shift, size = DEBUG_TYPE.get(point_type, DEBUG_SMALL)
    l_shift = carla.Location(z=v_shift)
    color = DEBUG_COLORS.get(debug_type, DEBUG_ROAD)
    life_time = 0.06 if not persistent else 100000
    world.debug.draw_point(location + l_shift, size, color, life_time)

def draw_arrow(world, location1, location2, arrow_type=DEBUG_SMALL, debug_type=DEBUG_ROAD, persistent=False):
    """Utility function to draw debugging points"""
    if location1 == location2:
        draw_point(world, location1, arrow_type, debug_type, persistent)
    v_shift, thickness = DEBUG_TYPE.get(arrow_type, DEBUG_SMALL)
    l_shift = carla.Location(z=v_shift)
    color = DEBUG_COLORS.get(debug_type, DEBUG_ROAD)
    life_time = 0.06 if not persistent else 100000
    world.debug.draw_arrow(location1 + l_shift, location2 + l_shift, thickness, thickness, color, life_time)


class Source(object):

    """
    Source object to store its position and its responsible actors
    """

    def __init__(self, wp, actors, entry_lane_wp='', active=True):  # pylint: disable=invalid-name
        self.wp = wp  # pylint: disable=invalid-name
        self.actors = actors
        self.active = active

        # For junction sources
        self.entry_lane_wp = entry_lane_wp
        self.previous_lane_keys = []  # Source lane and connecting lanes of the previous junction


class Junction(object):

    """
    Junction object. Stores its topology as well as its state, when active
    """

    def __init__(self, junction, junction_id, route_entry_index=None, route_exit_index=None):
        # Topology
        self.junctions = [junction]
        self.id = junction_id  # pylint: disable=invalid-name
        self.route_entry_index = route_entry_index
        self.route_exit_index = route_exit_index
        self.entry_lane_keys = []
        self.exit_lane_keys = []
        self.route_entry_keys = []
        self.route_exit_keys = []
        self.opposite_entry_keys = []
        self.opposite_exit_keys = []
        self.entry_wps = []
        self.exit_wps = []
        self.entry_directions = {'ref': [], 'opposite': [], 'left': [], 'right': []}
        self.exit_directions = {'ref': [], 'opposite': [], 'left': [], 'right': []}

        # State
        self.entry_sources = []
        self.exit_dict = OrderedDict()
        self.actor_dict = OrderedDict()

        # Junction scenario variables 
        self.stop_non_route_entries = False
        self.clear_middle = False
        self.inactive_entry_keys = []
        self.inactive_exit_keys = []

    def contains_wp(self, wp):
        """Checks whether or not a carla.Waypoint is inside the junction"""
        if not wp.is_junction:
            return False
        other_id = wp.get_junction().id
        for junction in self.junctions:
            if other_id == junction.id:
                return True
        return False


class BackgroundBehavior(AtomicBehavior):
    """
    Handles the background activity
    """

    def __init__(self, ego_actor, route, debug=False, name="BackgroundBehavior"):
        """
        Setup class members
        """
        super(BackgroundBehavior, self).__init__(name)
        self.debug = debug
        self._map = CarlaDataProvider.get_map()
        self._world = CarlaDataProvider.get_world()
        self._tm_port = CarlaDataProvider.get_traffic_manager_port()
        self._tm = CarlaDataProvider.get_client().get_trafficmanager(self._tm_port)
        self._tm.global_percentage_speed_difference(0.0)
        self._rng = CarlaDataProvider.get_random_seed()

        self._attribute_filter = {'base_type': 'car', 'special_type': '', 'has_lights': True, }

        # Global variables
        self._ego_actor = ego_actor
        self._ego_state = EGO_ROAD
        self._ego_wp = None
        self._ego_key = ""
        self._route_index = 0
        self._get_route_data(route)
        self._actors_speed_perc = {}  # Dictionary actor - percentage
        self._all_actors = []
        self._lane_width_threshold = 2.25  # Used to stop some behaviors at narrow lanes to avoid problems [m]

        self._spawn_vertical_shift = 0.2
        self._reuse_dist = 10  # When spawning actors, might reuse actors closer to this distance
        self._spawn_free_radius = 20  # Sources closer to the ego will not spawn actors
        self._fake_junction_ids = []
        self._fake_lane_pair_keys = []

        # Initialisation values
        self._vehicle_lane_change = False
        self._vehicle_lights = True
        self._vehicle_leading_distance = 10
        self._vehicle_offset = 0.1

        # Road variables
        self._road_dict = {}  # Dictionary lane key -> actor source
        self._road_checker_index = 0

        self._road_front_vehicles = 2  # Amount of vehicles in front of the ego
        self._road_back_vehicles = 2  # Amount of vehicles behind the ego
        self._radius_increase_ratio = 1.7  # Meters the radius increases per m/s of the ego

        self._base_junction_detection = 30
        self._detection_ratio = 1.5  # Meters the radius increases per m/s of the ego

        self._road_extra_front_actors = 0  # For cases where we want more space but not more vehicles
        self._road_spawn_dist = 15  # Distance between spawned vehicles [m]
        self._road_extra_space = 0  # Extra space for the road vehicles

        self._active_road_sources = True

        self._base_min_radius = 0
        self._base_max_radius = 0
        self._min_radius = 0
        self._max_radius = 0
        self._detection_dist = 0
        self._get_road_radius()

        # Junction variables
        self._junctions = []  # List with all the junctions part of the route, in order of appearance
        self._active_junctions = []  # List of all the active junctions

        self._junction_sources_dist = 40  # Distance from the entry sources to the junction [m]
        self._junction_sources_max_actors = 6  # Maximum vehicles alive at the same time per source
        self._junction_spawn_dist = 15  # Distance between spawned vehicles [m]
        self._junction_minimum_source_dist = 15  # Minimum distance between sources and their junction

        self._junction_source_perc = 80  # Probability [%] of the source being created

        # Opposite lane variables
        self._opposite_actors = []
        self._opposite_sources = []
        self._opposite_route_index = 0

        self._opposite_spawn_dist = 40  # Distance between spawned vehicles [m]
        self._opposite_sources_dist = 80  # Distance from the ego to the opposite sources [m]. Twice the spawn distance

        self._active_opposite_sources = True  # Flag to (de)activate all opposite sources

        # Scenario variables:
        self._scenario_stopped_actors = []  # Actors stopped by a hard break scenario
        self._scenario_stopped_back_actors = []  # Actors stopped by a open doors scenario
        self._scenario_max_speed = 0  # Max speed of the Background Activity. Deactivated with a value of 0
        self._scenario_junction_entry = False  # Flag indicating the ego is entering a junction
        self._scenario_junction_entry_distance = self._road_spawn_dist  # Min distance between vehicles and ego
        self._scenario_removed_lane = False  # Flag indicating a scenario has removed a lane
        self._scenario_remove_lane_offset = 0

    def _get_route_data(self, route):
        """Extract the information from the route"""
        self._route = []  # Transform the route into a list of waypoints
        self._route_options = []  # Extract the RoadOptions from the route
        self._accum_dist = []  # Save the total traveled distance for each waypoint
        prev_trans = None
        for trans, option in route:
            self._route.append(self._map.get_waypoint(trans.location))
            self._route_options.append(option)
            if prev_trans:
                dist = trans.location.distance(prev_trans.location)
                self._accum_dist.append(dist + self._accum_dist[-1])
            else:
                self._accum_dist.append(0)
            prev_trans = trans

        self._route_length = len(route)
        self._route_index = 0
        self._route_buffer = 3

    def _get_road_radius(self):
        """
        Computes the min and max radius of the road behaviorm which will determine the speed of the vehicles.
        Vehicles closer than the min radius maintain full speed, while those further than max radius are
        stopped. Between the two, the velocity decreases linearly"""
        self._base_min_radius = (self._road_front_vehicles + self._road_extra_front_actors) * self._road_spawn_dist
        self._base_max_radius = (self._road_front_vehicles + self._road_extra_front_actors + 1) * self._road_spawn_dist
        self._min_radius = self._base_min_radius + self._road_extra_space
        self._max_radius = self._base_max_radius + self._road_extra_space

    def initialise(self):
        """Creates the background activity actors. Pressuposes that the ego is at a road"""
        self._create_junction_dict()
        ego_wp = self._route[0]
        same_dir_wps = get_same_dir_lanes(ego_wp)

        self._initialise_road_behavior(same_dir_wps)
        self._initialise_opposite_sources()
        self._initialise_road_checker()

    def update(self):
        prev_ego_index = self._route_index

        # Check if the TM destroyed an actor
        if self._route_index > 0: # TODO: This check is due to intialization problem
            self._check_background_actors()

        # Update ego's route position. For robustness, the route point is used for most calculus
        self._update_ego_data()

        # Parameters and scenarios
        self._update_parameters()

        # Update ego state
        if self._ego_state == EGO_JUNCTION:
            self._monitor_ego_junction_exit()
        self._monitor_incoming_junctions()

        # Update_actors
        if self._ego_state == EGO_JUNCTION:
            self._update_junction_actors()
            self._update_junction_sources()
        else:
            self._update_road_actors()

            self._move_road_sources(prev_ego_index)
            self._update_road_sources()

            self._monitor_topology_changes(prev_ego_index)
            self._monitor_road_changes(prev_ego_index)

            self._move_opposite_sources(prev_ego_index)
            self._update_opposite_sources()

        # Update non junction sources
        self._update_opposite_actors()

        # Update the speed of all vehicles
        self._set_actors_speed()

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Destroy all actors"""
        all_actors = list(self._actors_speed_perc)
        for actor in list(all_actors):
            self._destroy_actor(actor)
        super(BackgroundBehavior, self).terminate(new_status)

    def _check_background_actors(self):
        """Checks if the Traffic Manager has removed a backgroudn actor"""
        alive_ids = [actor.id for actor in CarlaDataProvider.get_all_actors().filter('vehicle*')]
        for actor in list(self._all_actors):
            if actor.id not in alive_ids:
                self._remove_actor_info(actor)

    ################################
    ##       Junction cache       ##
    ################################

    def _create_junction_dict(self):
        """Extracts the junctions the ego vehicle will pass through."""
        data = self._get_junctions_data()
        fake_data, filtered_data = self._filter_fake_junctions(data)
        self._get_fake_lane_pairs(fake_data)
        route_data = self._join_complex_junctions(filtered_data)
        self._add_junctions_topology(route_data)
        self._junctions = route_data

    def _get_junctions_data(self):
        """Gets all the junctions the ego passes through"""
        junction_data = []
        junction_num = 0
        start_index = 0

        # Ignore the junction the ego spawns at
        for i in range(0, self._route_length - 1):
            if not self._is_junction(self._route[i]):
                start_index = i
                break

        for i in range(start_index, self._route_length - 1):
            next_wp = self._route[i+1]
            prev_junction = junction_data[-1] if len(junction_data) > 0 else None

            # Searching for the junction exit
            if prev_junction and prev_junction.route_exit_index is None:
                if not self._is_junction(next_wp) or next_wp.get_junction().id != junction_id:
                    prev_junction.route_exit_index = i+1

            # Searching for a junction
            elif self._is_junction(next_wp):
                junction_id = next_wp.get_junction().id
                if prev_junction:
                    start_dist = self._accum_dist[i]
                    prev_end_dist = self._accum_dist[prev_junction.route_exit_index]

                # Same junction as the prev one and closer than 2 meters
                if prev_junction and prev_junction.junctions[-1].id == junction_id:
                    start_dist = self._accum_dist[i]
                    prev_end_dist = self._accum_dist[prev_junction.route_exit_index]
                    distance = start_dist - prev_end_dist
                    if distance < 2:
                        prev_junction.junctions.append(next_wp.get_junction())
                        prev_junction.route_exit_index = None
                        continue

                junction_data.append(Junction(next_wp.get_junction(), junction_num, i))
                junction_num += 1

        return junction_data

    def _filter_fake_junctions(self, data):
        """
        Filters fake junctions. A fake junction is that which has no intersecting maneuvers
        (i.e, no two maneuvers start / end at the same lane).
        However, this fails for highway entry / exits with dedicated lanes, so specifically check those
        """
        fake_data = []
        filtered_data = []

        for junction_data in data:
            if len (junction_data.junctions) > 1:
                filtered_data.append(junction_data)
                continue  # These are always junctions

            junction = junction_data.junctions[0]

            found_intersecting_maneuvers = False
            used_entries = []
            used_exits = []

            # Search for intersecting maneuvers
            for entry_wp, exit_wp in junction.get_waypoints(carla.LaneType.Driving):
                entry_key = get_lane_key(self._get_junction_entry_wp(entry_wp))
                exit_key = get_lane_key(self._get_junction_exit_wp(exit_wp))

                # Check if a maneuver starts / ends at another one.
                # Checking if it was used isn't enough as some maneuvers are repeated in CARLA maps.
                # Instead, check if the index of both entry and exit are different.
                entry_index = -1 if entry_key not in used_entries else used_entries.index(entry_key)
                exit_index = -1 if exit_key not in used_exits else used_exits.index(exit_key)

                if exit_index != entry_index:
                    found_intersecting_maneuvers = True
                    break

                used_entries.append(entry_key)
                used_exits.append(exit_key)

            if found_intersecting_maneuvers:
                filtered_data.append(junction_data)
                continue

            # Search for highway dedicated lane entries.
            found_highway = False
            used_entry_roads = {}
            used_exit_roads = {}
            for entry_wp, exit_wp in junction.get_waypoints(carla.LaneType.Driving):
                entry_road_key = get_road_key(self._get_junction_entry_wp(entry_wp))
                exit_road_key = get_road_key(self._get_junction_exit_wp(exit_wp))

                # Entries / exits with dedicated lanes have no intersecting maneuvers
                # (as the entry / exit is a lane that finishes, not a maneuvers part of a junction),
                # so they are missfiltered as fake junctions.
                # Detect them by an entry road having 3 or more lanes. TODO: Improve this
                if entry_road_key in used_entry_roads:
                    used_entry_roads[entry_road_key] += 1
                else:
                    used_entry_roads[entry_road_key] = 0

                if exit_road_key in used_exit_roads:
                    used_exit_roads[exit_road_key] += 1
                else:
                    used_exit_roads[exit_road_key] = 0

                if used_entry_roads[entry_road_key] >= 3 or used_exit_roads[exit_road_key] >= 3:
                    found_highway = True
                    break

            if found_highway:
                filtered_data.append(junction_data)
                continue

            fake_data.append(junction_data)

            # TODO: Recheck for old CARLA maps

        return fake_data, filtered_data

    def _get_complex_junctions(self):
        """
        Function to hardcode the topology of some complex junctions. This is done for the roundabouts,
        as the current API doesn't offer that info as well as others such as the gas station at Town04.
        If there are micro lanes between connected junctions, add them to the fake_lane_keys, connecting
        them when their topology is calculated
        """
        complex_junctions = []
        fake_lane_keys = []

        if 'Town03' in self._map.name:
            # Roundabout, take it all as one
            complex_junctions.append([
                self._map.get_waypoint_xodr(1100, -5, 16.6).get_junction(),
                self._map.get_waypoint_xodr(1624, -5, 25.3).get_junction(),
                self._map.get_waypoint_xodr(1655, -5, 8.3).get_junction(),
                self._map.get_waypoint_xodr(1772, 3, 16.2).get_junction(),
                self._map.get_waypoint_xodr(1206, -5, 5.9).get_junction()])
            fake_lane_keys.extend([
                ['37*-4', '36*-4'], ['36*-4', '37*-4'],
                ['37*-5', '36*-5'], ['36*-5', '37*-5'],
                ['38*-4', '12*-4'], ['12*-4', '38*-4'],
                ['38*-5', '12*-5'], ['12*-5', '38*-5']])

            # Gas station
            complex_junctions.append([
                self._map.get_waypoint_xodr(1031, -1, 11.3).get_junction(),
                self._map.get_waypoint_xodr(100, -1, 18.8).get_junction(),
                self._map.get_waypoint_xodr(1959, -1, 22.7).get_junction()])
            fake_lane_keys.extend([
                ['32*-2', '33*-2'], ['33*-2', '32*-2'],
                ['32*-1', '33*-1'], ['33*-1', '32*-1'],
                ['32*4', '33*4'], ['33*4', '32*4'],
                ['32*5', '33*5'], ['33*5', '32*5']])

        elif 'Town04' in self._map.name:
            # Gas station
            complex_junctions.append([
                self._map.get_waypoint_xodr(518, -1, 8.1).get_junction(),
                self._map.get_waypoint_xodr(886, 1, 10.11).get_junction(),
                self._map.get_waypoint_xodr(467, 1, 25.8).get_junction()])

        self._fake_lane_pair_keys.extend(fake_lane_keys)
        return complex_junctions

    def _join_complex_junctions(self, filtered_data):
        """
        Joins complex junctions into one. This makes it such that all the junctions,
        as well as their connecting lanes, are treated as the same junction
        """
        route_data = []
        prev_index = -1

        # If entering a complex, add all its junctions to the list
        for junction_data in filtered_data:
            junction = junction_data.junctions[0]
            prev_junction = route_data[-1] if len(route_data) > 0 else None
            complex_junctions = self._get_complex_junctions()

            # Get the complex index
            current_index = -1
            for i, complex_junctions in enumerate(complex_junctions):
                complex_ids = [j.id for j in complex_junctions]
                if junction.id in complex_ids:
                    current_index = i
                    break

            if current_index == -1:
                # Outside a complex, add it
                route_data.append(junction_data)

            elif current_index == prev_index:
                # Same complex as the previous junction
                prev_junction.route_exit_index = junction_data.route_exit_index

            else:
                # New complex, add it
                junction_ids = [j.id for j in junction_data.junctions]
                for complex_junction in complex_junctions:
                    if complex_junction.id not in junction_ids:
                        junction_data.junctions.append(complex_junction)

                route_data.append(junction_data)

            prev_index = current_index

        return route_data

    def _get_fake_lane_pairs(self, fake_data):
        """Gets a list of entry-exit lanes of the fake junctions"""
        for fake_junctions_data in fake_data:
            for junction in fake_junctions_data.junctions:
                for entry_wp, exit_wp in junction.get_waypoints(carla.LaneType.Driving):
                    while self._is_junction(entry_wp):
                        entry_wps = entry_wp.previous(0.5)
                        if len(entry_wps) == 0:
                            break  # Stop when there's no prev
                        entry_wp = entry_wps[0]
                    if self._is_junction(entry_wp):
                        continue  # Triggered by the loops break

                    while self._is_junction(exit_wp):
                        exit_wps = exit_wp.next(0.5)
                        if len(exit_wps) == 0:
                            break  # Stop when there's no prev
                        exit_wp = exit_wps[0]
                    if self._is_junction(exit_wp):
                        continue  # Triggered by the loops break

                    self._fake_junction_ids.append(junction.id)
                    self._fake_lane_pair_keys.append([get_lane_key(entry_wp), get_lane_key(exit_wp)])

    def _get_junction_entry_wp(self, entry_wp):
        """For a junction waypoint, returns a waypoint outside of it that entrys into its lane"""
        # Exit the junction
        while self._is_junction(entry_wp):
            entry_wps = entry_wp.previous(0.2)
            if len(entry_wps) == 0:
                return None  # Stop when there's no prev
            entry_wp = entry_wps[0]
        return entry_wp

    def _get_junction_exit_wp(self, exit_wp):
        """For a junction waypoint, returns a waypoint outside of it from which the lane exits the junction"""
        while self._is_junction(exit_wp):
            exit_wps = exit_wp.next(0.2)
            if len(exit_wps) == 0:
                return None  # Stop when there's no prev
            exit_wp = exit_wps[0]
        return exit_wp

    def _get_closest_junction_waypoint(self, waypoint, junction_wps):
        """
        Matches a given wp to another one inside the list.
        This is first done by checking its key, and if this fails, the closest wp is chosen
        """
        # Check the lane keys
        junction_keys = [get_lane_key(waypoint_) for waypoint_ in junction_wps]
        if get_lane_key(waypoint) in junction_keys:
            return waypoint

        # Get the closest one
        closest_dist = float('inf')
        closest_junction_wp = None
        route_location = waypoint.transform.location
        for junction_wp in junction_wps:
            distance = junction_wp.transform.location.distance(route_location)
            if distance < closest_dist:
                closest_dist = distance
                closest_junction_wp = junction_wp

        return closest_junction_wp

    def _is_route_wp_behind_junction_wp(self, route_wp, junction_wp):
        """Checks if an actor is behind the ego. Uses the route transform"""
        route_location = route_wp.transform.location
        junction_transform = junction_wp.transform
        junction_heading = junction_transform.get_forward_vector()
        wps_vec = route_location - junction_transform.location
        if junction_heading.x * wps_vec.x + junction_heading.y * wps_vec.y < - 0.09:  # 85º
            return True
        return False

    def _add_junctions_topology(self, route_data):
        """Gets the entering and exiting lanes of a multijunction"""
        for junction_data in route_data:
            used_entry_lanes = []
            used_exit_lanes = []
            entry_lane_wps = []
            exit_lane_wps = []

            if self.debug:
                print(' --------------------- ')
            for junction in junction_data.junctions:
                for entry_wp, exit_wp in junction.get_waypoints(carla.LaneType.Driving):

                    entry_wp = self._get_junction_entry_wp(entry_wp)
                    if not entry_wp:
                        continue
                    if get_lane_key(entry_wp) not in used_entry_lanes:
                        used_entry_lanes.append(get_lane_key(entry_wp))
                        entry_lane_wps.append(entry_wp)
                        if self.debug:
                            draw_point(self._world, entry_wp.transform.location, DEBUG_SMALL, DEBUG_ENTRY, True)

                    exit_wp = self._get_junction_exit_wp(exit_wp)
                    if not exit_wp:
                        continue
                    if get_lane_key(exit_wp) not in used_exit_lanes:
                        used_exit_lanes.append(get_lane_key(exit_wp))
                        exit_lane_wps.append(exit_wp)
                        if self.debug:
                            draw_point(self._world, exit_wp.transform.location, DEBUG_SMALL, DEBUG_EXIT, True)

            # Check for connecting lanes. This is pretty much for the roundabouts, but some weird geometries
            # make it possible for single junctions to have the same road entering and exiting. Two cases,
            # Lanes that exit one junction and enter another (or viceversa)
            exit_lane_keys = [get_lane_key(wp) for wp in exit_lane_wps]
            entry_lane_keys = [get_lane_key(wp) for wp in entry_lane_wps]
            for wp in list(entry_lane_wps):
                if get_lane_key(wp) in exit_lane_keys:
                    entry_lane_wps.remove(wp)
                    if self.debug:
                        draw_point(self._world, wp.transform.location, DEBUG_SMALL, DEBUG_CONNECT, True)

            for wp in list(exit_lane_wps):
                if get_lane_key(wp) in entry_lane_keys:
                    exit_lane_wps.remove(wp)
                    if self.debug:
                        draw_point(self._world, wp.transform.location, DEBUG_SMALL, DEBUG_CONNECT, True)

            # Lanes with a fake junction in the middle (maps junction exit to fake junction entry and viceversa)
            for entry_key, exit_key in self._fake_lane_pair_keys:
                entry_wp = None
                for wp in entry_lane_wps:
                    if get_lane_key(wp) == exit_key:  # A junction exit is a fake junction entry
                        entry_wp = wp
                        break
                exit_wp = None
                for wp in exit_lane_wps:
                    if get_lane_key(wp) == entry_key:  # A junction entry is a fake junction exit
                        exit_wp = wp
                        break
                if entry_wp and exit_wp:
                    entry_lane_wps.remove(entry_wp)
                    exit_lane_wps.remove(exit_wp)
                    if self.debug:
                        draw_point(self._world, entry_wp.transform.location, DEBUG_SMALL, DEBUG_CONNECT, True)
                        draw_point(self._world, exit_wp.transform.location, DEBUG_SMALL, DEBUG_CONNECT, True)

            junction_data.entry_wps = entry_lane_wps
            junction_data.exit_wps = exit_lane_wps
            junction_data.entry_lane_keys = entry_lane_keys
            junction_data.exit_lane_keys = exit_lane_keys
            for exit_wp in exit_lane_wps:
                junction_data.exit_dict[get_lane_key(exit_wp)] = {
                    'actors': [],
                    'max_actors': 0,
                    'ref_wp': None,
                    'max_distance': 0,
                }

            # Filter the entries and exits that correspond to the route
            route_entry_wp = self._route[junction_data.route_entry_index]

            # Junction entry
            for wp in get_same_dir_lanes(route_entry_wp):
                junction_wp = self._get_closest_junction_waypoint(wp, entry_lane_wps)
                junction_data.route_entry_keys.append(get_lane_key(junction_wp))
            for wp in get_opposite_dir_lanes(route_entry_wp):
                junction_wp = self._get_closest_junction_waypoint(wp, exit_lane_wps)
                junction_data.opposite_exit_keys.append(get_lane_key(junction_wp))

            # Junction exit
            if junction_data.route_exit_index:  # Can be None if route ends at a junction
                route_exit_wp = self._route[junction_data.route_exit_index]
                for wp in get_same_dir_lanes(route_exit_wp):
                    junction_wp = self._get_closest_junction_waypoint(wp, exit_lane_wps)
                    junction_data.route_exit_keys.append(get_lane_key(junction_wp))
                for wp in get_opposite_dir_lanes(route_exit_wp):
                    junction_wp = self._get_closest_junction_waypoint(wp, entry_lane_wps)
                    junction_data.opposite_entry_keys.append(get_lane_key(junction_wp))

            # Add the entry directions of each lane with respect to the route. Used for scenarios 7 to 9
            route_entry_yaw = route_entry_wp.transform.rotation.yaw
            for wp in entry_lane_wps:
                diff = (wp.transform.rotation.yaw - route_entry_yaw) % 360
                if diff > 330.0:
                    direction = 'ref'
                elif diff > 225.0:
                    direction = 'right'
                elif diff > 135.0:
                    direction = 'opposite'
                elif diff > 30.0:
                    direction = 'left'
                else:
                    direction = 'ref'

                junction_data.entry_directions[direction].append(get_lane_key(wp))

            # Supposing scenario vehicles go straight, these correspond to the exit lanes of the entry directions
            for wp in exit_lane_wps:
                diff = (wp.transform.rotation.yaw - route_entry_yaw) % 360
                if diff > 330.0:
                    direction = 'ref'
                elif diff > 225.0:
                    direction = 'right'
                elif diff > 135.0:
                    direction = 'opposite'
                elif diff > 30.0:
                    direction = 'left'
                else:
                    direction = 'ref'

                junction_data.exit_directions[direction].append(get_lane_key(wp))

            if self.debug:
                exit_lane = self._route[junction_data.route_exit_index] if junction_data.route_exit_index else None
                print('> R Entry Lane: {}'.format(get_lane_key(self._route[junction_data.route_entry_index])))
                print('> R Exit  Lane: {}'.format(get_lane_key(exit_lane)))
                entry_print = '> J Entry Lanes: '
                for entry_wp in entry_lane_wps:
                    key = get_lane_key(entry_wp)
                    entry_print += key + ' ' * (8 - len(key))
                print(entry_print)
                exit_print = '> J Exit  Lanes: '
                for exit_wp in exit_lane_wps:
                    key = get_lane_key(exit_wp)
                    exit_print += key + ' ' * (8 - len(key))
                print(exit_print)
                route_entry = '> R-J Entry Lanes: '
                for entry_key in junction_data.route_entry_keys:
                    route_entry += entry_key + ' ' * (8 - len(entry_key))
                print(route_entry)
                route_exit = '> R-J Route Exit  Lanes: '
                for exit_key in junction_data.route_exit_keys:
                    route_exit += exit_key + ' ' * (8 - len(exit_key))
                print(route_exit)
                route_oppo_entry = '> R-J Oppo Entry Lanes: '
                for oppo_entry_key in junction_data.opposite_entry_keys:
                    route_oppo_entry += oppo_entry_key + ' ' * (8 - len(oppo_entry_key))
                print(route_oppo_entry)
                route_oppo_exit = '> R-J Oppo Exit  Lanes: '
                for oppo_exit_key in junction_data.opposite_exit_keys:
                    route_oppo_exit += oppo_exit_key + ' ' * (8 - len(oppo_exit_key))
                print(route_oppo_exit)

    def _is_junction(self, waypoint):
        if not waypoint.is_junction or waypoint.junction_id in self._fake_junction_ids:
            return False
        return True

    ################################
    ##       Mode functions       ##
    ################################

    def _add_actor_dict_element(self, actor_dict, actor, exit_lane_key='', at_oppo_entry_lane=False):
        """
        Adds a new actor to the actor dictionary.
        'exit_lane_key' is used to know at which exit lane (if any) is the vehicle
        'at_oppo_entry_lane' whether or not the actor is part of the entry at the opposite lane the route exits through.
        This will be the ones that aren't removed
        """
        actor_dict[actor] = {
            'state': JUNCTION_ENTRY if not exit_lane_key else JUNCTION_EXIT,
            'exit_lane_key': exit_lane_key,
            'at_oppo_entry_lane': at_oppo_entry_lane
        }

    def _switch_to_junction_mode(self, junction):
        """
        Prepares the junction mode, removing all road behaviours.
        Actors that are stopped via a scenario will still wait.
        """
        self._ego_state = EGO_JUNCTION
        for lane in self._road_dict:
            for actor in self._road_dict[lane].actors:
                # TODO: Map the actors to the junction entry to have full control of them.
                # This should remove the 'at_oppo_entry_lane'.
                self._add_actor_dict_element(junction.actor_dict, actor)
                if actor not in self._scenario_stopped_actors:
                    self._actors_speed_perc[actor] = 100

        for lane_key in self._road_dict:
            source = self._road_dict[lane_key]
            source_key = get_lane_key(source.wp)
            if source_key in junction.route_entry_keys:
                junction.entry_sources.append(Source(
                    source.wp, source.actors, entry_lane_wp=source.wp, active=source.active)
                )
                if source_key in junction.inactive_entry_keys:
                    for actor in source.actors:
                        self._destroy_actor(actor)
                    source.active = False
                    junction.inactive_entry_keys.remove(source_key)

            # TODO: Else should map the source to the entry and add it

        self._road_dict.clear()
        self._opposite_sources.clear()

    def _end_junction_behavior(self, junction):
        """
        Destroys unneeded actors (those that aren't part of the route's road),
        moving the rest to other data structures and cleaning up the variables.
        If no other junctions are active, starts road mode
        """
        actor_dict = junction.actor_dict
        route_exit_keys = junction.route_exit_keys
        self._active_junctions.pop(0)

        # Prepare the road dictionary
        if not self._active_junctions:
            for wp in junction.exit_wps:
                if get_lane_key(wp) in route_exit_keys:
                    self._road_dict[get_lane_key(wp)] = Source(wp, [], active=self._active_road_sources)

        else:
            for wp in junction.exit_wps:
                if get_lane_key(wp) in route_exit_keys:
                    # TODO: entry_lane_wp isn't really this one (for cases with road changes)
                    self._active_junctions[0].entry_sources.append(
                        Source(wp, [], entry_lane_wp=wp, active=self._active_road_sources)
                    )

        # Handle the actors
        for actor in list(actor_dict):
            location = CarlaDataProvider.get_location(actor)
            if not location or self._is_location_behind_ego(location):
                self._destroy_actor(actor)
                continue

            # Don't destroy those that are on the route's road opposite lane.
            # Instead, let them move freely until they are automatically destroyed.
            self._actors_speed_perc[actor] = 100
            if actor_dict[actor]['at_oppo_entry_lane']:
                self._opposite_actors.append(actor)
                self._tm.ignore_lights_percentage(actor, 100)
                self._tm.ignore_signs_percentage(actor, 100)
                continue

            # Save those that are on the route's road
            exit_key = actor_dict[actor]['exit_lane_key']
            if exit_key in route_exit_keys:
                if not self._active_junctions:
                    self._road_dict[exit_key].actors.append(actor)
                else:
                    entry_sources = self._active_junctions[0].entry_sources
                    for entry_source in entry_sources: # Add it to the back source
                        if exit_key == get_lane_key(entry_source.wp):
                            entry_sources.actors.append(actor)
                            break
                continue

            # Destroy the rest
            self._destroy_actor(actor)

        # If the junction was part of a scenario, forget about it
        self._scenario_junction_entry = False

        if not self._active_junctions:
            self._ego_state = EGO_ROAD
            self._initialise_opposite_sources()
            self._initialise_road_checker()

    def _search_for_next_junction(self):
        """Check if closeby to a junction. The closest one will always be the first"""
        if not self._junctions:
            return None

        ego_accum_dist = self._accum_dist[self._route_index]
        junction_accum_dist = self._accum_dist[self._junctions[0].route_entry_index]
        if junction_accum_dist - ego_accum_dist < self._detection_dist:  # Junctions closeby
            return self._junctions.pop(0)

        return None

    def _initialise_connecting_lanes(self, junction):
        """
        Moves the actors currently at the exit lane of the last junction
        to entry actors of the newly created junction
        """
        if len(self._active_junctions) > 0:
            prev_junction = self._active_junctions[-1]
            route_exit_keys = prev_junction.route_exit_keys
            exit_dict = prev_junction.exit_dict
            for exit_key in route_exit_keys:
                exit_actors = exit_dict[exit_key]['actors']
                for actor in list(exit_actors):
                    self._remove_actor_info(actor)
                    self._add_actor_dict_element(junction.actor_dict, actor)
                    self._actors_speed_perc[actor] = 100

    def _monitor_incoming_junctions(self):
        """
        Monitors when the ego approaches a junction, triggering that junction when it happens.
        This can be triggered even if there is another junction happening are they work independently
        """
        junction = self._search_for_next_junction()
        if not junction:
            return

        if self._ego_state == EGO_ROAD:
            self._switch_to_junction_mode(junction)
        self._initialise_junction_sources(junction)
        self._initialise_junction_exits(junction)

        self._initialise_connecting_lanes(junction)
        self._active_junctions.append(junction)

        # Forget the fact that a lane was removed, so that it isn't readded in the middle of the junction
        self._scenario_removed_lane = False
        self._scenario_remove_lane_offset = 0

    def _monitor_ego_junction_exit(self):
        """
        Monitors when the ego exits the junctions, preparing the road mode when that happens
        """
        current_junction = self._active_junctions[0]
        exit_index = current_junction.route_exit_index
        if exit_index and self._route_index >= exit_index:
            self._end_junction_behavior(current_junction)

    def _add_incoming_actors(self, junction, source):
        """Checks nearby actors that will pass through the source, adding them to it"""
        source_location = source.wp.transform.location
        if not source.previous_lane_keys:
            source.previous_lane_keys = [get_lane_key(prev_wp) for prev_wp in source.wp.previous(self._reuse_dist)]
            source.previous_lane_keys.append(get_lane_key(source.wp))

        for actor in self._all_actors:
            if actor in source.actors:
                continue  # Don't use actors already part of the source

            actor_location = CarlaDataProvider.get_location(actor)
            if actor_location is None:
                continue  # No idea where the actor is, ignore it
            if source_location.distance(actor_location) > self._reuse_dist:
                continue  # Don't use actors far away

            actor_wp = self._map.get_waypoint(actor_location)
            if get_lane_key(actor_wp) not in source.previous_lane_keys:
                continue  # Don't use actors that won't pass through the source

            self._actors_speed_perc[actor] = 100
            self._remove_actor_info(actor)
            source.actors.append(actor)

            at_oppo_entry_lane = get_lane_key(source.entry_lane_wp) in junction.opposite_entry_keys
            self._add_actor_dict_element(junction.actor_dict, actor, at_oppo_entry_lane=at_oppo_entry_lane)

            return actor

    def _move_road_sources(self, prev_ego_index):
        """
        Moves the road sources so that they are always following the ego from behind
        """
        if prev_ego_index != self._route_index:
            min_distance = self._road_back_vehicles * self._road_spawn_dist
            for lane_key in self._road_dict:
                source = self._road_dict[lane_key]

                # If no actors are found, let the last_location be ego's location 
                # to keep moving the source waypoint forward
                if len(source.actors) == 0:
                    last_location = self._ego_wp.transform.location
                else:
                    last_location = CarlaDataProvider.get_location(source.actors[-1])

                if last_location is None:
                    continue

                # Stop the sources in front of the ego (created by new lanes)
                if not self._is_location_behind_ego(source.wp.transform.location):
                    continue

                # Stop the source from being too close to the ego or last lane vehicle
                source_location = source.wp.transform.location
                ego_location = self._ego_wp.transform.location

                actor_dist = max(0, last_location.distance(source_location) - self._road_spawn_dist)
                ego_dist = max(0, ego_location.distance(source_location) - min_distance)
                move_dist = min(actor_dist, ego_dist)

                # Move the source forward if needed
                if move_dist > 0:
                    new_source_wps = source.wp.next(move_dist)
                    if not new_source_wps:
                        continue
                    source.wp = new_source_wps[0]

    def _update_road_sources(self):
        """
        Manages the sources that spawn actors behind the ego.
        These are always behind the ego and will continuously spawn actors.
        These sources also track the amount of vehicles in front of the ego,
        removing actors if the amount is too high.
        """
        for lane_key in self._road_dict:
            source = self._road_dict[lane_key]
            if self.debug:
                draw_point(self._world, source.wp.transform.location, DEBUG_SMALL, DEBUG_ROAD, False)
                draw_string(self._world, source.wp.transform.location, str(len(source.actors)), DEBUG_ROAD, False)

            # Ensure not too many actors are in front of the ego
            front_veh = 0
            for actor in source.actors:
                location = CarlaDataProvider.get_location(actor)
                if location and not self._is_location_behind_ego(location):
                    front_veh += 1
            if front_veh > self._road_front_vehicles:
                self._destroy_actor(source.actors[0])  # This is always the front most vehicle

            if not source.active:
                continue
            if not self._is_location_behind_ego(source.wp.transform.location):
                continue  # Stop the sources in front of the ego (created by new lanes)
            if len(source.actors) >= self._road_back_vehicles + self._road_front_vehicles:
                continue

            if len(source.actors) == 0:
                location = self._ego_wp.transform.location
            else:
                location = CarlaDataProvider.get_location(source.actors[-1])
                if not location:
                    continue

            distance = location.distance(source.wp.transform.location)

            # Spawn a new actor if the last one is far enough
            if distance > self._road_spawn_dist:
                actor = self._spawn_source_actor(source, self._road_spawn_dist)
                if actor is None:
                    continue

                # Set their initial speed, so that they don't lag behind the ego.
                # Set the speed to the ego's one, but never surpassing by the lane's last vehicle's one
                forward_vec = source.wp.transform.get_forward_vector()
                speed = self._ego_actor.get_velocity().length()
                if len(source.actors):
                    speed = min(speed, source.actors[-1].get_velocity().length())
                actor.set_target_velocity(speed * forward_vec)

                source.actors.append(actor)

    ################################
    ## Behavior related functions ##
    ################################

    def _initialise_road_behavior(self, road_wps):
        """
        Initialises the road behavior, consisting on several vehicle in front of the ego,
        and several on the back and are only spawned outside junctions.
        If there aren't enough actors behind, road sources will be created that will do so later on
        """
        # Vehicles in front
        for wp in road_wps:
            spawn_wps = []

            # Front spawn points
            next_wp = wp
            for _ in range(self._road_front_vehicles):
                next_wps = next_wp.next(self._road_spawn_dist)
                if len(next_wps) != 1 or self._is_junction(next_wps[0]):
                    break  # Stop when there's no next or found a junction
                next_wp = next_wps[0]
                spawn_wps.insert(0, next_wp)

            # Back spawn points
            source_dist = 0
            prev_wp = wp
            for _ in range(self._road_back_vehicles):
                prev_wps = prev_wp.previous(self._road_spawn_dist)
                if len(prev_wps) != 1 or self._is_junction(prev_wps[0]):
                    break  # Stop when there's no next or found a junction
                prev_wp = prev_wps[0]
                spawn_wps.append(prev_wp)
                source_dist += self._road_spawn_dist

            # Spawn actors
            actors = self._spawn_actors(spawn_wps)

            self._road_dict[get_lane_key(wp)] = Source(
                prev_wp, actors, active=self._active_road_sources
            )

    def _initialise_opposite_sources(self):
        """
        All opposite lanes have actor sources that will continually create vehicles,
        creating the sensation of permanent traffic. The actor spawning will be done later on
        (_update_opposite_sources). These sources are at a (somewhat) fixed distance
        from the ego, but they never entering junctions. 
        """
        self._opposite_route_index = None
        if not self._junctions:
            next_junction_index = self._route_length - 1
        else:
            next_junction_index = self._junctions[0].route_entry_index

        ego_accum_dist = self._accum_dist[self._route_index]
        for i in range(self._route_index, next_junction_index):
            if self._accum_dist[i] - ego_accum_dist > self._opposite_sources_dist:
                self._opposite_route_index = i
                break
        if not self._opposite_route_index:
            # Junction is closer than the opposite source distance
            self._opposite_route_index = next_junction_index

        oppo_wp = self._route[self._opposite_route_index]

        for wp in get_opposite_dir_lanes(oppo_wp):
            self._opposite_sources.append(Source(wp, [], active=self._active_opposite_sources))

    def _initialise_road_checker(self):
        """
        Gets the waypoints in front of the ego to continuously check if the road changes
        """
        self._road_checker_index = None

        if not self._junctions:
            upper_limit = self._route_length - 1
        else:
            upper_limit = self._junctions[0].route_entry_index

        ego_accum_dist = self._accum_dist[self._route_index]
        for i in range(self._route_index, upper_limit):
            if self._accum_dist[i] - ego_accum_dist > self._max_radius:
                self._road_checker_index = i
                break
        if not self._road_checker_index:
            self._road_checker_index = upper_limit

    def _initialise_junction_sources(self, junction):
        """
        Initializes the actor sources to ensure the junction is always populated. They are
        placed at certain distance from the junction, but are stopped if another junction is found,
        to ensure the spawned actors always move towards the activated one.
        """
        for wp in junction.entry_wps:
            entry_lane_key = get_lane_key(wp)
            if entry_lane_key in junction.route_entry_keys:
                continue  # Ignore the road from which the route enters

            moved_dist = 0
            prev_wp = wp
            while moved_dist < self._junction_sources_dist:
                prev_wps = prev_wp.previous(5)
                if len(prev_wps) == 0 or self._is_junction(prev_wps[0]):
                    break
                prev_wp = prev_wps[0]
                moved_dist += 5

            # Don't add junction sources too close to the junction
            if moved_dist < self._junction_minimum_source_dist:
                continue

            source = Source(prev_wp, [], entry_lane_wp=wp)
            entry_lane_key = get_lane_key(wp)
            if entry_lane_key in junction.inactive_entry_keys:
                source.active = False
                junction.inactive_entry_keys.remove(entry_lane_key)

            junction.entry_sources.append(source)

        # Real junctions aren't always full of traffic in all lanes, so deactivate some of them.
        # Doing this after the source have been created in case another behavior activates them
        for source in junction.entry_sources:
            if 100 * self._rng.random() > self._junction_source_perc:
                source.active = False

    def _initialise_junction_exits(self, junction):
        """
        Computes and stores the max capacity of the exit. Prepares the behavior of the next road
        by creating actors at the route exit, and the sources that'll create actors behind the ego
        """
        exit_wps = junction.exit_wps
        route_exit_keys = junction.route_exit_keys

        for wp in exit_wps:
            max_actors = 0
            max_distance = junction.exit_dict[get_lane_key(wp)]['max_distance']
            exiting_wps = []

            next_wp = wp

            # Move the initial distance (added by the `_extent_road_exit_space`)
            if max_distance > 0:
                next_wps = next_wp.next(max_distance)
                if not next_wps:
                    continue
                next_wp = next_wps[0]

            for i in range(max(self._road_front_vehicles, 1)):

                # Get the moving distance (first jump is higher to allow space for another vehicle)
                if i == 0:
                    move_dist = 2 * self._junction_spawn_dist
                else:
                    move_dist = self._junction_spawn_dist

                # And move such distance
                next_wps = next_wp.next(move_dist)
                if len(next_wps) == 0:
                    break  # Stop when there's no next
                next_wp = next_wps[0]
                if max_actors > 0 and self._is_junction(next_wp):
                    break  # Stop when a junction is found

                max_actors += 1
                max_distance += move_dist
                exiting_wps.insert(0, next_wp)

            junction.exit_dict[get_lane_key(wp)] = {
                'actors': [], 'max_actors': max_actors, 'ref_wp': wp, 'max_distance': max_distance
            }

            exit_lane_key = get_lane_key(wp)
            if exit_lane_key in junction.inactive_exit_keys:
                continue  # The exit is inactive, don't spawn anything

            if exit_lane_key in route_exit_keys:
                actors = self._spawn_actors(exiting_wps)
                for actor in actors:
                    self._add_actor_dict_element(junction.actor_dict, actor, exit_lane_key=exit_lane_key)
                junction.exit_dict[exit_lane_key]['actors'] = actors

    def _update_junction_sources(self):
        """Checks the actor sources to see if new actors have to be created"""
        for junction in self._active_junctions:
            actor_dict = junction.actor_dict
            for source in junction.entry_sources:
                if self.debug:
                    draw_point(self._world, source.wp.transform.location, DEBUG_SMALL, DEBUG_JUNCTION, False)
                    draw_string(self._world, source.wp.transform.location, str(len(source.actors)), DEBUG_JUNCTION, False)

                entry_lane_key = get_lane_key(source.entry_lane_wp)
                at_oppo_entry_lane = entry_lane_key in junction.opposite_entry_keys

                if not source.active:
                    continue

                self._add_incoming_actors(junction, source)

                # Cap the amount of alive actors
                if len(source.actors) >= self._junction_sources_max_actors:
                    continue

                # Calculate distance to the last created actor
                if len(source.actors) == 0:
                    actor_location = CarlaDataProvider.get_location(self._ego_actor)
                else:
                    actor_location = CarlaDataProvider.get_location(source.actors[-1])

                if not actor_location:
                    continue
                distance = actor_location.distance(source.wp.transform.location)

                # Spawn a new actor if the last one is far enough
                if distance > self._junction_spawn_dist:
                    actor = self._spawn_source_actor(source, self._junction_spawn_dist)
                    if not actor:
                        continue
                    if junction.stop_non_route_entries and get_lane_key(source.entry_lane_wp) not in junction.route_entry_keys:
                        self._actors_speed_perc[actor] = 0
                    self._add_actor_dict_element(actor_dict, actor, at_oppo_entry_lane=at_oppo_entry_lane)
                    source.actors.append(actor)

    def _monitor_topology_changes(self, prev_index):
        """
        Continually check the road in front to see if it has changed its topology.
        If the number of lanes reduces, merge the ending lane with a side one
        If the number of lanes increases, add a new road source.
        """
        def get_road_wp(wp):
            """Goes backwards in the lane to match the wp with the road key dictionary"""
            road_wp = wp
            if get_lane_key(road_wp) in self._road_dict:
                return road_wp

            road_wp = wp
            distance = self._max_radius

            while distance > 0:
                prev_wps = road_wp.previous(1)
                if not prev_wps:
                    return None
                road_wp = prev_wps[0]
                if get_lane_key(road_wp) in self._road_dict:
                    return road_wp
                distance -= 1

            return None

        def get_source_wp(wp):
            """Moves the wp forward until the lane is wide enough to fit a vehicle"""
            source_wp = wp
            while source_wp.lane_width < self._lane_width_threshold + 0.2:
                source_wps = source_wp.next(1)
                if not source_wps:
                    return None
                source_wp = source_wps[0]
            return source_wp

        if self.debug:
            checker_wp = self._route[self._road_checker_index]
            draw_point(self._world, checker_wp.transform.location, DEBUG_SMALL, DEBUG_ROAD, False)

        if prev_index == self._route_index:
            return

        # Get the new route tracking wp
        checker_index = None
        last_index = self._junctions[0].route_entry_index if self._junctions else self._route_length - 1
        current_accum_dist = self._accum_dist[self._route_index]
        for i in range(self._road_checker_index, last_index):
            accum_dist = self._accum_dist[i]
            if accum_dist - current_accum_dist >= self._max_radius:
                checker_index = i
                break
        if not checker_index:
            checker_index = last_index

        if checker_index == self._road_checker_index:
            return

        new_wps = get_same_dir_lanes(self._route[checker_index])
        old_wps = get_same_dir_lanes(self._route[self._road_checker_index])

        new_accum_dist = self._accum_dist[checker_index]
        prev_accum_dist = self._accum_dist[self._road_checker_index]
        route_move_dist = max(new_accum_dist - prev_accum_dist, 0.1)

        if len(new_wps) > len(old_wps):
            # Don't add anything in front if the junction entry has to be empty
            if not self._scenario_junction_entry:

                for new_wp in list(new_wps):

                    prev_wps = new_wp.previous(2 * route_move_dist)  # x2, just in case
                    if prev_wps:
                        continue

                    # Found the new lane, add the actors and source.
                    # Don't spawn actors while the source is in front of the ego
                    source_wp = get_source_wp(new_wp)
                    if not source_wp:
                        continue

                    next_wp = source_wp
                    spawn_wps = []
                    spawn_dist = self._road_front_vehicles + CarlaDataProvider.get_velocity(self._ego_actor)
                    for _ in range(self._road_front_vehicles):
                        next_wps = next_wp.next(spawn_dist)
                        if len(next_wps) != 1 or self._is_junction(next_wps[0]):
                            break  # Stop when there's no next or found a junction
                        next_wp = next_wps[0]
                        spawn_wps.insert(0, next_wp)

                    actors = self._spawn_actors(spawn_wps)

                    if get_lane_key(source_wp) not in self._road_dict:
                        # Lanes created away from the center won't affect the ids of other lanes, so just add the new id
                        self._road_dict[get_lane_key(source_wp)] = Source(source_wp, actors, active=self._active_road_sources)
                    else:
                        # If the lane is inwards, all lanes have their id shifted by 1 outwards
                        # TODO: Doesn't work for more than one lane.
                        added_id = 1 if source_wp.lane_id > 0 else -1
                        new_lane_key = get_lane_key_from_ids(source_wp.road_id, source_wp.lane_id + added_id)
                        self._road_dict[new_lane_key] = self._road_dict[get_lane_key(source_wp)]
                        self._road_dict[get_lane_key(source_wp)] = Source(source_wp, actors, active=self._active_road_sources)

        elif len(new_wps) < len(old_wps):
            for old_wp in list(old_wps):
                next_wps = old_wp.next(2 * route_move_dist)  # x2, just in case
                if next_wps:
                    continue

                # Found the lane that ends, merge it with the one on its side
                road_wp = get_road_wp(old_wp)
                if not road_wp:
                    continue

                # Get a side lane
                right_wp = old_wp.get_right_lane()
                left_wp = old_wp.get_left_lane()
                side_road_wp = None
                if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                    side_road_wp = get_road_wp(right_wp)
                    side_path = [right_wp.transform.location]
                elif left_wp and left_wp.lane_type == carla.LaneType.Driving:
                    side_road_wp = get_road_wp(left_wp)
                    side_path = [left_wp.transform.location]

                if not side_road_wp:
                    # No side lane found part of the road dictionary, remove them
                    for actor in list(self._road_dict[get_lane_key(road_wp)].actors):
                        self._destroy_actor(actor)
                    self._road_dict.pop(get_lane_key(road_wp), None)
                    continue

                # Get the actors
                lane_actors = self._road_dict[get_lane_key(road_wp)].actors
                side_lane_actors = self._road_dict[get_lane_key(side_road_wp)].actors

                # Get their distance to the ego
                actors_with_dist = []
                ego_location = self._ego_wp.transform.location
                for actor in lane_actors + side_lane_actors:
                    actor_location = CarlaDataProvider.get_location(actor)
                    if not actor_location:
                        self._destroy_actor(actor)
                        continue
                    dist = ego_location.distance(actor_location)
                    if not self._is_location_behind_ego(actor_location):
                        dist *= -1
                    actors_with_dist.append([actor, dist])

                # Sort them by distance
                actors_sorted_with_dist = sorted(actors_with_dist, key=lambda a: a[1])
                zero_index = len([a for a in actors_sorted_with_dist if a[1] < 0])
                min_index = max(zero_index - self._road_front_vehicles, 0)
                max_index = min(zero_index + self._road_front_vehicles - 1, len(actors_sorted_with_dist) - 1)

                # Remove the unneeded ones, and make the ending lane actors perform a lane change
                source_actors = []
                for i, (actor, _) in enumerate(actors_sorted_with_dist):
                    if i >= min_index and i <= max_index:
                        source_actors.append(actor)
                        self._tm.set_path(actor, side_path)
                    else:
                        self._destroy_actor(actor)

                # And update the road dict
                self._road_dict[get_lane_key(side_road_wp)].actors = source_actors
                self._road_dict.pop(get_lane_key(road_wp), None)

        self._road_checker_index = checker_index

    def _move_opposite_sources(self, prev_index):
        """
        Moves the sources of the opposite direction back. Additionally, tracks a point a certain distance
        in front of the ego to see if the road topology has to be recalculated
        """
        if self.debug:
            for source in self._opposite_sources:
                draw_point(self._world, source.wp.transform.location, DEBUG_SMALL, DEBUG_OPPOSITE, False)
                draw_string(self._world, source.wp.transform.location, str(len(source.actors)), DEBUG_OPPOSITE, False)
            route_wp = self._route[self._opposite_route_index]
            draw_point(self._world, route_wp.transform.location, DEBUG_SMALL, DEBUG_OPPOSITE, False)

        if prev_index == self._route_index:
            return

        # Get the new route tracking wp
        oppo_route_index = None
        last_index = self._junctions[0].route_entry_index if self._junctions else self._route_length - 1
        current_accum_dist = self._accum_dist[self._route_index]
        for i in range(self._opposite_route_index, last_index):
            accum_dist = self._accum_dist[i]
            if accum_dist - current_accum_dist >= self._opposite_sources_dist:
                oppo_route_index = i
                break
        if not oppo_route_index:
            oppo_route_index = last_index

        # Get the distance moved by the route reference index
        new_accum_dist = self._accum_dist[oppo_route_index]
        prev_accum_dist = self._accum_dist[self._opposite_route_index]
        route_move_dist = new_accum_dist - prev_accum_dist
        if route_move_dist <= 0:
            return  # Sometimes a route wp is behind the pervious one, filter these out

        new_opposite_wps = get_opposite_dir_lanes(self._route[oppo_route_index])

        if len(new_opposite_wps) != len(self._opposite_sources):
            # The topology has changed. Remap the new lanes to the sources
            new_opposite_sources = []
            for wp in new_opposite_wps:
                location = wp.transform.location
                new_source = None
                for source in self._opposite_sources:
                    if location.distance(source.wp.transform.location) < 1.1 * route_move_dist:
                        new_source = source
                        break

                if new_source:
                    new_source.wp = wp
                    new_opposite_sources.append(new_source)
                    self._opposite_sources.remove(new_source)
                else:
                    new_opposite_sources.append(Source(wp, []))

            self._opposite_sources = new_opposite_sources
        else:
            # The topology hasn't changed, move the distance backwards
            for source in self._opposite_sources:
                wp = source.wp
                if not self._is_junction(wp):
                    prev_wps = wp.previous(route_move_dist)
                    if len(prev_wps) == 0:
                        continue
                    prev_wp = prev_wps[0]
                    source.wp = prev_wp

        self._opposite_route_index = oppo_route_index

    def _update_opposite_sources(self):
        """Checks the opposite actor sources to see if new actors have to be created"""
        ego_speed = CarlaDataProvider.get_velocity(self._ego_actor)
        for source in self._opposite_sources:
            if not source.active:
                continue

            # Ending / starting lanes create issues as the lane width gradually decreases until reaching 0,
            # where the lane starts / ends. Avoid spawning anything inside those parts with small lane width
            if source.wp.lane_width < self._lane_width_threshold:
                continue

            # Calculate distance to the last created actor
            if len(source.actors) == 0:
                distance = self._opposite_sources_dist + 1
            else:
                actor_location = CarlaDataProvider.get_location(source.actors[-1])
                if not actor_location:
                    continue
                distance = source.wp.transform.location.distance(actor_location)

            # Spawn a new actor if the last one is far enough
            if distance > self._opposite_spawn_dist:
                actor = self._spawn_source_actor(source)
                if actor is None:
                    continue
                self._tm.ignore_lights_percentage(actor, 100)
                self._tm.ignore_signs_percentage(actor, 100)
                self._opposite_actors.append(actor)
                source.actors.append(actor)

    def _update_parameters(self):
        """
        Changes those parameters that have dynamic behaviors and / or that can be changed by external source.
        This is done using py_trees' Blackboard variables and all behaviors should be at `background_manager.py`.
        The blackboard variable is reset to None to avoid changing them back again next time.
        """
        # Road behavior
        road_behavior_data = py_trees.blackboard.Blackboard().get('BA_ChangeRoadBehavior')
        if road_behavior_data is not None:
            num_front_vehicles, num_back_vehicles, spawn_dist, extra_space = road_behavior_data
            if num_front_vehicles is not None:
                self._road_front_vehicles = num_front_vehicles
            if num_back_vehicles is not None:
                self._road_back_vehicles = num_back_vehicles
            if spawn_dist is not None:
                self._road_spawn_dist = spawn_dist
            if extra_space is not None:
                self._road_extra_space = extra_space
            self._get_road_radius()
            py_trees.blackboard.Blackboard().set('BA_ChangeRoadBehavior', None, True)

        # Opposite behavior
        opposite_behavior_data = py_trees.blackboard.Blackboard().get('BA_ChangeOppositeBehavior')
        if opposite_behavior_data is not None:
            source_dist, spawn_dist, active = opposite_behavior_data
            if source_dist is not None:
                if source_dist < self._junction_sources_dist:
                    print('WARNING: Opposite sources distance is lower than the junction ones. Ignoring it')
                else:
                    self._opposite_sources_dist = source_dist
            if spawn_dist is not None:
                self._opposite_spawn_dist = spawn_dist
                self._opposite_sources_dist = 2 * spawn_dist
            if active is not None:
                self._active_opposite_sources = active
                for source in self._opposite_sources:
                    source.active = active
            py_trees.blackboard.Blackboard().set('BA_ChangeOppositeBehavior', None, True)

        # Junction behavior
        junction_behavior_data = py_trees.blackboard.Blackboard().get('BA_ChangeJunctionBehavior')
        if junction_behavior_data is not None:
            source_dist, spawn_dist, max_actors, source_perc = junction_behavior_data
            if source_dist is not None:
                if source_dist > self._opposite_sources_dist:
                    print('WARNING: Junction sources distance is higher than the opposite ones. Ignoring it')
                else:
                    self._junction_sources_dist = source_dist
            if spawn_dist:
                self._junction_spawn_dist = spawn_dist
            if max_actors is not None:
                self._junction_sources_max_actors = max_actors
            if source_perc is not None:
                self._junction_source_perc = source_perc
            py_trees.blackboard.Blackboard().set('BA_ChangeJunctionBehavior', None, True)

        # Max speed
        max_speed = py_trees.blackboard.Blackboard().get('BA_SetMaxSpeed')
        if max_speed is not None:
            self._scenario_max_speed = max_speed
            py_trees.blackboard.Blackboard().set('BA_SetMaxSpeed', None, True)

        # Stop front vehicles
        stop_data = py_trees.blackboard.Blackboard().get('BA_StopFrontVehicles')
        if stop_data is not None:
            self._stop_road_front_vehicles()
            py_trees.blackboard.Blackboard().set('BA_StopFrontVehicles', None, True)

        # Start front vehicles
        start_data = py_trees.blackboard.Blackboard().get('BA_StartFrontVehicles')
        if start_data is not None:
            self._start_road_front_vehicles()
            py_trees.blackboard.Blackboard().set("BA_StartFrontVehicles", None, True)

        # Stop back vehicles
        stop_back_data = py_trees.blackboard.Blackboard().get('BA_StopBackVehicles')
        if stop_back_data is not None:
            self._stop_road_back_vehicles()
            py_trees.blackboard.Blackboard().set('BA_StopBackVehicles', None, True)

        # Start back vehicles
        start_back_data = py_trees.blackboard.Blackboard().get('BA_StartBackVehicles')
        if start_back_data is not None:
            self._start_road_back_vehicles()
            py_trees.blackboard.Blackboard().set("BA_StartBackVehicles", None, True)

        # Leave space in front
        leave_space_data = py_trees.blackboard.Blackboard().get('BA_LeaveSpaceInFront')
        if leave_space_data is not None:
            self._leave_space_in_front(leave_space_data)
            py_trees.blackboard.Blackboard().set('BA_LeaveSpaceInFront', None, True)

        # Leave crosssing space
        leave_crossing_space_data = py_trees.blackboard.Blackboard().get('BA_LeaveCrossingSpace')
        if leave_crossing_space_data is not None:
            self._leave_crossing_space(leave_crossing_space_data)
            py_trees.blackboard.Blackboard().set('BA_LeaveCrossingSpace', None, True)

        # Remove road lane
        remove_road_lane_data = py_trees.blackboard.Blackboard().get('BA_RemoveRoadLane')
        if remove_road_lane_data is not None:
            self._remove_road_lane(remove_road_lane_data)
            py_trees.blackboard.Blackboard().set('BA_RemoveRoadLane', None, True)

        # Readd road lane
        readd_road_lane_data = py_trees.blackboard.Blackboard().get('BA_ReAddRoadLane')
        if readd_road_lane_data is not None:
            self._readd_road_lane(readd_road_lane_data)
            py_trees.blackboard.Blackboard().set('BA_ReAddRoadLane', None, True)

        # Adapt the BA to the junction scenario
        junction_scenario_data = py_trees.blackboard.Blackboard().get('BA_HandleJunctionScenario')
        if junction_scenario_data is not None:
            self._handle_junction_scenario(junction_scenario_data)
            py_trees.blackboard.Blackboard().set("BA_HandleJunctionScenario", None, True)

        # Switch route sources
        switch_sources_data = py_trees.blackboard.Blackboard().get('BA_SwitchRouteSources')
        if switch_sources_data is not None:
            self._switch_route_sources(switch_sources_data)
            py_trees.blackboard.Blackboard().set("BA_SwitchRouteSources", None, True)

        self._compute_parameters()

    def _compute_parameters(self):
        """Computes the parameters that are dependent on the speed of the ego. """
        ego_speed = CarlaDataProvider.get_velocity(self._ego_actor)
        self._min_radius = self._base_min_radius + self._radius_increase_ratio * ego_speed + self._road_extra_space
        self._max_radius = self._base_max_radius + self._radius_increase_ratio * ego_speed + self._road_extra_space
        self._detection_dist = self._base_junction_detection + self._detection_ratio * ego_speed

    def _stop_road_front_vehicles(self):
        """
        Stops all road vehicles in front of the ego. Use `_start_road_front_vehicles` to make them move again.
        """
        for lane in self._road_dict:
            for actor in self._road_dict[lane].actors:
                location = CarlaDataProvider.get_location(actor)
                if location and not self._is_location_behind_ego(location):
                    self._scenario_stopped_actors.append(actor)
                    self._actors_speed_perc[actor] = 0
                    self._tm.update_vehicle_lights(actor, False)
                    lights = actor.get_light_state()
                    lights |= carla.VehicleLightState.Brake
                    actor.set_light_state(carla.VehicleLightState(lights))

    def _start_road_front_vehicles(self):
        """
        Restarts all road vehicles stopped by `_stop_road_front_vehicles`.
        """
        for actor in self._scenario_stopped_actors:
            self._actors_speed_perc[actor] = 100
            self._tm.update_vehicle_lights(actor, True)
            lights = actor.get_light_state()
            lights &= ~carla.VehicleLightState.Brake
            actor.set_light_state(carla.VehicleLightState(lights))
        self._scenario_stopped_actors = []

    def _stop_road_back_vehicles(self):
        """
        Stops all road vehicles behind the ego. Use `_start_road_back_vehicles` to make them move again.
        """
        for lane in self._road_dict:
            for actor in self._road_dict[lane].actors:
                location = CarlaDataProvider.get_location(actor)
                if location and self._is_location_behind_ego(location):
                    self._actors_speed_perc[actor] = 0
                    self._scenario_stopped_back_actors.append(actor)

    def _start_road_back_vehicles(self):
        """
        Restarts all road vehicles stopped by `_stop_road_back_vehicles`.
        """
        for actor in self._scenario_stopped_back_actors:
            self._actors_speed_perc[actor] = 100
        self._scenario_stopped_back_actors = []

    def _move_actors_forward(self, actors, space):
        """Teleports the actors forward a set distance"""
        for actor in list(actors):
            location = CarlaDataProvider.get_location(actor)
            if not location:
                continue

            actor_wp = self._map.get_waypoint(location)
            new_actor_wps = actor_wp.next(space)
            if len(new_actor_wps) > 0:
                new_transform = new_actor_wps[0].transform
                new_transform.location.z += 0.2
                actor.set_transform(new_transform)
            else:
                self._destroy_actor(actor)

    def _switch_route_sources(self, enabled):
        """
        Disables all sources that are part of the ego's route
        """
        self._active_road_sources = enabled
        for lane in self._road_dict:
            self._road_dict[lane].active = enabled

        for junction in self._active_junctions:
            for source in junction.entry_sources:
                if get_lane_key(source.entry_lane_wp) in junction.route_entry_keys:
                    source.active = enabled

    def _leave_space_in_front(self, space):
        """Teleports all the vehicles in front of the ego forward"""
        if self._ego_key not in self._road_dict:
            return

        if self._active_junctions:
            return

        front_actors = []
        min_distance = float('inf')
        for actor in self._road_dict[self._ego_key].actors:
            location = CarlaDataProvider.get_location(actor)
            if not location or self._is_location_behind_ego(location):
                continue

            front_actors.append(actor)
            distance = location.distance(self._ego_wp.transform.location)
            if distance < min_distance:
                min_distance = distance

        step = space - min_distance
        if step > 0:  # Only move them if needed and only the minimum required distance
            self._move_actors_forward(front_actors, step)

    def _leave_crossing_space(self, collision_wp):
        """Removes all vehicle in the middle of crossing trajectory and stops the nearby ones"""
        destruction_dist = 20
        stop_dist = 30

        opposite_wp = collision_wp.get_left_lane()
        if not opposite_wp:
            return  # Nothing else to do
        opposite_loc = opposite_wp.transform.location

        for actor in list(self._opposite_actors):
            location = actor.get_location()
            if not location:
                continue

            collision_dist = location.distance(opposite_loc)
            if collision_dist < destruction_dist:
                self._destroy_actor(actor)
            elif collision_dist < stop_dist:
                actor.set_target_velocity(carla.Vector3D())

    def _remove_road_lane(self, lane_wp):
        """Removes a road lane"""
        self._scenario_removed_lane = True
        self._scenario_remove_lane_offset = 0
        lane_key = get_lane_key(lane_wp)
        if lane_key not in list(self._road_dict):
            print(f"WARNING: Couldn't find the lane to be removed, '{lane_key}' isn't part of the road behavior")
            return

        self._road_dict[lane_key].active = False
        for actor in list(self._road_dict[lane_key].actors):
            self._destroy_actor(actor)
        self._road_dict.pop(lane_key, None)

    def _readd_road_lane(self, lane_offset):
        """Adds a ego road lane. This is expected to be used after having previously removed such lane"""
        # Check that the ego hasn't moved close to a junction, where we don't want to reinitialize the lane
        if not self._scenario_removed_lane:
            return

        lane_offset += self._scenario_remove_lane_offset

        if lane_offset == 0:
            add_lane_wp = self._ego_wp
            add_lane_key = self._ego_key
        else:
            side_wp = self._ego_wp
            for _ in range(abs(lane_offset)):
                side_wp = side_wp.get_right_lane() if lane_offset > 0 else side_wp.get_left_lane()
                if not side_wp:
                    print(f"WARNING: Couldn't find a lane with the desired offset")
                    return

            add_lane_wp = side_wp
            add_lane_key = get_lane_key(side_wp)

        if add_lane_key in list(self._road_dict):
            print(f"WARNING: Couldn't add a lane {add_lane_key} as it is already part of the road")
            return

        ego_speed = CarlaDataProvider.get_velocity(self._ego_actor)
        spawn_dist = self._road_spawn_dist + 2 * ego_speed

        spawn_wps = []

        next_wp = add_lane_wp
        for _ in range(self._road_front_vehicles):
            next_wps = next_wp.next(spawn_dist)
            if len(next_wps) != 1 or self._is_junction(next_wps[0]):
                break  # Stop when there's no next or found a junction
            next_wp = next_wps[0]
            spawn_wps.insert(0, next_wp)

        source_dist = 0
        prev_wp = add_lane_wp
        for _ in range(self._road_back_vehicles):
            prev_wps = prev_wp.previous(spawn_dist)
            if len(prev_wps) != 1 or self._is_junction(prev_wps[0]):
                break  # Stop when there's no next or found a junction
            prev_wp = prev_wps[0]
            spawn_wps.append(prev_wp)
            source_dist += spawn_dist

        actors = []
        for spawn_wp in spawn_wps:
            actor = self._spawn_actor(spawn_wp)
            if not actor:
                continue
            actor.set_target_velocity(spawn_wp.transform.get_forward_vector() * ego_speed)
            actors.append(actor)

        self._road_dict[add_lane_key] = Source(prev_wp, actors, active=self._active_road_sources)

        self._scenario_removed_lane = False
        self._scenario_remove_lane_offset = 0

    def _handle_junction_scenario(self, junction_data):
        """
        Adapts the BA to the junction scenario by clearing the junction,
        its entries, exits, and by extending the road exit to add more space
        """
        clear_junction, clear_ego_entry, remove_entries, remove_exits, stop_entries, extend_road_exit = junction_data

        if clear_junction:
            self._clear_junction_middle()

        if clear_ego_entry:
            self._clear_ego_entry()

        if remove_entries:
            self._remove_junction_entries(remove_entries)

        if remove_exits:
            self._remove_junction_exits(remove_exits)

        if stop_entries:
            self._stop_non_ego_route_entries()

        if extend_road_exit:
            self._extent_road_exit_space(extend_road_exit)

        self._scenario_removed_lane = False
        self._scenario_remove_lane_offset = 0

    def _clear_junction_middle(self):
        """Clears the junction, and all subsequent actors that enter it"""
        if self._active_junctions:
            junction = self._active_junctions[0]
            actor_dict = junction.actor_dict
            for actor in list(actor_dict):
                if actor_dict[actor]['state'] == JUNCTION_MIDDLE:
                    self._destroy_actor(actor)
            junction.clear_middle = True

        elif self._junctions:
            self._junctions[0].clear_middle = True

    def _clear_ego_entry(self):
        """
        Remove all actors in front of the vehicle.
        """
        def handle_actors(actor_list):
            for actor in list(actor_list):
                location = CarlaDataProvider.get_location(actor)
                if location and not self._is_location_behind_ego(location):
                    self._destroy_actor(actor)

        # This will make the actors behind never overtake the ego
        self._scenario_junction_entry = True

        if self._active_junctions:
            for source in self._active_junctions[0].entry_sources:
                handle_actors(source.actors)
        else:
            for lane_key in list(self._road_dict):
                source = self._road_dict[lane_key]
                handle_actors(source.actors)
                source.active = False

    def _remove_junction_entries(self, wps):
        """Removes a list of entries of the closest junction (or marks them so that they aren't spawned)"""
        if len(self._active_junctions) > 0:
            junction = self._active_junctions[0]
        elif len(self._junctions) > 0:
            junction = self._junctions[0]
        else:
            return

        for wp in wps:
            mapped_key = None
            next_wp = wp
            while not self._is_junction(next_wp):
                lane_key = get_lane_key(next_wp)
                if lane_key in junction.entry_lane_keys:
                    mapped_key = lane_key
                    break
                next_wps = next_wp.next(1)
                if not next_wps:
                    break
                next_wp = next_wps[0]

            if not mapped_key:
                print("WARNING: Couldn't find the asked entry to be removed")
                continue

            if len(self._active_junctions) > 0:
                for source in junction.entry_sources:
                    if get_lane_key(source.wp) == mapped_key:
                        for actor in list(source.actors):
                            self._destroy_actor(actor)
                        source.active = False
            else:
                junction.inactive_entry_keys.append(mapped_key)

    def _remove_junction_exits(self, wps):
        """Removes a list of exit of the closest junction (or marks them so that they aren't spawned)"""
        if len(self._active_junctions) > 0:
            junction = self._active_junctions[0]
        elif len(self._junctions) > 0:
            junction = self._junctions[0]
        else:
            return

        for wp in wps:

            mapped_key = None
            prev_wp = wp
            while not self._is_junction(prev_wp):
                lane_key = get_lane_key(prev_wp)
                if lane_key in junction.exit_dict:
                    mapped_key = lane_key
                    break
                prev_wps = prev_wp.next(1)
                if not prev_wps:
                    break
                prev_wp = prev_wps[0]

            if not mapped_key:
                print("WARNING: Couldn't find the asked exit to be removed")
                continue

            if len(self._active_junctions) > 0:
                for actor in list(junction.exit_dict[mapped_key]['actors']):
                    self._destroy_actor(actor)
            else:
                junction.inactive_exit_keys.append(mapped_key)

    def _stop_non_ego_route_entries(self):
        """Clears the junction, and all subsequent actors that enter it"""
        if self._active_junctions:

            # Remove all actors in the middle
            junction = self._active_junctions[0]
            actor_dict = junction.actor_dict

            entry_sources = junction.entry_sources
            route_entry_keys = junction.route_entry_keys
            for source in entry_sources:
                if get_lane_key(source.entry_lane_wp) not in route_entry_keys:
                    for actor in source.actors:
                        if actor_dict[actor]['state'] == JUNCTION_ENTRY:
                            self._actors_speed_perc[actor] = 0

        elif self._junctions:
            self._junctions[0].stop_non_route_entries = True

    def _extent_road_exit_space(self, space):
        """Increases the space left by the exit vehicles at a specific road"""
        if len(self._active_junctions) > 0:
            junction = self._active_junctions[0]
        elif len(self._junctions) > 0:
            junction = self._junctions[0]
        else:
            return

        route_lane_keys = junction.route_exit_keys

        for exit_lane_key in route_lane_keys:
            junction.exit_dict[exit_lane_key]['max_distance'] += space
            actors = junction.exit_dict[exit_lane_key]['actors']
            self._move_actors_forward(actors, space)
            for actor in actors:
                if junction.actor_dict[actor]['state'] == JUNCTION_EXIT_ROAD:
                    self._actors_speed_perc[actor] = 100
                    junction.actor_dict[actor]['state'] = JUNCTION_EXIT

    #############################
    ##     Actor functions     ##
    #############################
    def _initialise_actor(self, actor):
        """
        Save the actor into the needed structures, disable its lane changes and set the leading distance.
        """
        self._tm.auto_lane_change(actor, self._vehicle_lane_change)
        self._tm.update_vehicle_lights(actor, self._vehicle_lights)
        self._tm.distance_to_leading_vehicle(actor, self._vehicle_leading_distance)
        self._tm.vehicle_lane_offset(actor, self._vehicle_offset)
        self._all_actors.append(actor)

    def _spawn_actor(self, spawn_wp, ego_dist=0):
        """Spawns an actor"""
        ego_location = CarlaDataProvider.get_location(self._ego_actor)
        if ego_location.distance(spawn_wp.transform.location) < ego_dist:
            return None

        spawn_transform = carla.Transform(
            spawn_wp.transform.location + carla.Location(z=self._spawn_vertical_shift),
            spawn_wp.transform.rotation
        )

        actor = CarlaDataProvider.request_new_actor(
            'vehicle.*', spawn_transform, 'background', True,
            attribute_filter={'base_type': 'car', 'has_lights': True}, tick=False
        )

        if not actor:
            return actor
        self._initialise_actor(actor)
        return actor

    def _spawn_actors(self, spawn_wps, ego_dist=0):
        """Spawns several actors in batch"""
        spawn_transforms = []
        ego_location = CarlaDataProvider.get_location(self._ego_actor)
        for wp in spawn_wps:
            if ego_location.distance(wp.transform.location) < ego_dist:
                continue
            spawn_transforms.append(
                carla.Transform(wp.transform.location + carla.Location(z=self._spawn_vertical_shift),
                                wp.transform.rotation)
            )

        actors = CarlaDataProvider.request_new_batch_actors(
            'vehicle.*', len(spawn_transforms), spawn_transforms, True, False, 'background',
            attribute_filter=self._attribute_filter, tick=False)

        if not actors:
            return actors

        for actor in actors:
            self._initialise_actor(actor)

        return actors

    def _spawn_source_actor(self, source, ego_dist=20):
        """Given a source, spawns an actor at that source"""
        ego_location = CarlaDataProvider.get_location(self._ego_actor)
        source_transform = source.wp.transform
        if ego_location.distance(source_transform.location) < ego_dist:
            return None

        new_transform = carla.Transform(
            source_transform.location + carla.Location(z=self._spawn_vertical_shift),
            source_transform.rotation
        )
        actor = CarlaDataProvider.request_new_actor(
            'vehicle.*', new_transform, rolename='background',
            autopilot=True, random_location=False, attribute_filter=self._attribute_filter, tick=False)

        if not actor:
            return actor

        self._initialise_actor(actor)
        return actor

    def _is_location_behind_ego(self, location):
        """Checks if an actor is behind the ego. Uses the route transform"""
        ego_transform = self._route[self._route_index].transform
        ego_heading = ego_transform.get_forward_vector()
        ego_actor_vec = location - ego_transform.location
        if ego_heading.dot(ego_actor_vec) < - 0.17:  # 100º
            return True
        return False

    def _update_road_actors(self):
        """
        Dynamically controls the actor speed in front of the ego.
        Not applied to those behind it so that they can catch up it
        """
        # Updates their speed
        scenario_actors = self._scenario_stopped_actors + self._scenario_stopped_back_actors
        for lane_key in self._road_dict:
            for i, actor in enumerate(self._road_dict[lane_key].actors):
                location = CarlaDataProvider.get_location(actor)
                if not location:
                    continue
                if self.debug:
                    string = 'R_'
                    string += 'B' if self._is_location_behind_ego(location) else 'F'
                    string += '_(' + str(i) + ')'
                    string += '_[' + lane_key + ']'
                    draw_string(self._world, location, string, DEBUG_ROAD, False)

                # Actors part of scenarios are their own category, ignore them
                if actor in scenario_actors:
                    continue

                # TODO: Lane changes are weird with the TM, so just stop them
                actor_wp = self._map.get_waypoint(location)
                if actor_wp.lane_width < self._lane_width_threshold:

                    # Ensure only ending lanes are affected. not sure if it is needed though
                    next_wps = actor_wp.next(0.5)
                    if next_wps and next_wps[0].lane_width < actor_wp.lane_width:
                        actor.set_target_velocity(carla.Vector3D(0, 0, 0))
                        self._actors_speed_perc[actor] = 0
                        lights = actor.get_light_state()
                        lights |= carla.VehicleLightState.RightBlinker
                        lights |= carla.VehicleLightState.LeftBlinker
                        lights |= carla.VehicleLightState.Position
                        actor.set_light_state(carla.VehicleLightState(lights))
                        actor.set_autopilot(False, self._tm_port)
                        continue

                self._set_road_actor_speed(location, actor)

    def _set_road_actor_speed(self, location, actor, multiplier=1):
        """
        Changes the speed of the vehicle depending on its distance to the ego.
        - Front vehicles: Gradually reduces the speed the further they are.
        - Back vehicles: Gradually reduces the speed the further they are to help them catch up to the ego.
        - Junction scenario behavior: Don't let vehicles behind the ego surpass it.
        """
        distance = location.distance(self._ego_wp.transform.location)
        if not self._is_location_behind_ego(location):
            percentage = (self._max_radius - distance) / (self._max_radius - self._min_radius) * 100
            percentage *= multiplier
            percentage = max(min(percentage, 100), 0)
        elif not self._scenario_junction_entry:
            percentage = distance / (self._max_radius - self._min_radius) * 100 + 100
            percentage = max(min(percentage, 200), 0)
        else:
            ego_speed = CarlaDataProvider.get_velocity(self._ego_actor)
            base_percentage = ego_speed / self._ego_target_speed * 100
            true_distance = distance - self._scenario_junction_entry_distance
            percentage = true_distance / (self._max_radius - self._min_radius) * 100 + base_percentage
            percentage = max(min(percentage, 100), 0)

        self._actors_speed_perc[actor] = percentage

    def _monitor_road_changes(self, prev_route_index):
        """
        Checks if the ego changes road, remapping the route keys.
        """
        def get_lane_waypoints(reference_wp, index=0):
            if not reference_wp.is_junction:
                wps = get_same_dir_lanes(reference_wp)
            else: # Handle fake junction by using its entry / exit wps
                wps = []
                for junction_wps in reference_wp.get_junction().get_waypoints(carla.LaneType.Driving):
                    if get_road_key(junction_wps[index]) == get_road_key(reference_wp):
                        wps.append(junction_wps[index])
            return wps

        def get_wp_pairs(old_wps_, new_wps_, dist):
            wp_pairs = []

            unmapped_wps = new_wps_
            for old_wp_ in old_wps_:
                mapped = False
                location = old_wp_.transform.location
                for new_wp_ in unmapped_wps:
                    if location.distance(new_wp_.transform.location) < dist:
                        unmapped_wps.remove(new_wp_)
                        wp_pairs.append([old_wp_, new_wp_])
                        mapped = True
                        break

                if not mapped:
                    wp_pairs.append([old_wp_, None])

            for unmapped_wp in unmapped_wps:
                wp_pairs.append([None, unmapped_wp])

            return wp_pairs

        def is_remapping_needed(current_wp, prev_wp):
            """The road dict mapping is needed if """
            # If the ego just exitted a junction, remap isn't needed
            if self._is_junction(prev_wp):
                return False

            # If the road changes, remap
            if get_road_key(prev_wp) != get_road_key(current_wp):
                return True

            # Some roads have starting / ending lanes in the middle. Remap if that is detected
            prev_wps = get_same_dir_lanes(prev_wp)
            current_wps = get_same_dir_lanes(current_wp)
            if len(prev_wps) != len(current_wps):
                return True

            return False

        def is_road_dict_unchanging(wp_pairs):
            """Sometimes 'monitor_topology_changes' has already done the necessary changes"""
            road_dict_keys = list(self._road_dict)
            if len(wp_pairs) != len(road_dict_keys):
                return False

            for _, new_wp in wp_pairs:
                if get_lane_key(new_wp) not in road_dict_keys:
                    return False
            return True

        if prev_route_index == self._route_index:
            return
        route_wp = self._route[self._route_index]
        prev_route_wp = self._route[prev_route_index]

        if not is_remapping_needed(route_wp, prev_route_wp):
            return

        new_road_dict = {}
        old_wps = get_lane_waypoints(prev_route_wp, 1)
        new_wps = get_lane_waypoints(route_wp, 0)
        check_dist = 1.1 * route_wp.transform.location.distance(prev_route_wp.transform.location)
        wp_pairs = get_wp_pairs(old_wps, new_wps, check_dist)
        # TODO: These pairs are sometimes wrong as some fake intersections have overlapping lanes (highway entries)

        if is_road_dict_unchanging(wp_pairs):
            return

        for old_wp, new_wp in wp_pairs:
            old_key = get_lane_key(old_wp)
            new_key = get_lane_key(new_wp)

            # Lane has ended / started, no need to remap it
            if not new_wp or not old_wp:
                continue

            if self.debug:
                draw_arrow(self._world, old_wp.transform.location, new_wp.transform.location, DEBUG_MEDIUM, DEBUG_ROAD, True)

            # Check that the lane is part of the road dictionary
            if old_key in list(self._road_dict):
                new_road_dict[new_key] = self._road_dict[old_key]
                self._road_dict.pop(old_key)
                continue

        # Add the rest of the unmapped road sources, mainly those created forward at lanes that are starting
        for lane_key in list(self._road_dict):
            new_road_dict[lane_key] = self._road_dict.pop(lane_key)

        self._road_dict = new_road_dict

    def _update_junction_actors(self):
        """
        Handles an actor depending on their previous state. Actors entering the junction have its exit
        monitored through their waypoint. When they exit, they are either moved to a connecting junction,
        or added to the exit dictionary. Actors that exited the junction will stop after a certain distance
        """
        if len(self._active_junctions) == 0:
            return

        max_index = len(self._active_junctions) - 1
        for i, junction in enumerate(self._active_junctions):
            if self.debug:
                route_keys = junction.route_entry_keys + junction.route_exit_keys
                route_oppo_keys = junction.opposite_entry_keys + junction.opposite_exit_keys
                for wp in junction.entry_wps + junction.exit_wps:
                    if get_lane_key(wp) in route_keys:
                        draw_point(self._world, wp.transform.location, DEBUG_MEDIUM, DEBUG_ROAD, False)
                    elif get_lane_key(wp) in route_oppo_keys:
                        draw_point(self._world, wp.transform.location, DEBUG_MEDIUM, DEBUG_OPPOSITE, False)
                    else:
                        draw_point(self._world, wp.transform.location, DEBUG_MEDIUM, DEBUG_JUNCTION, False)

            actor_dict = junction.actor_dict
            exit_dict = junction.exit_dict

            scenario_entry_actor_ids = []
            if self._scenario_junction_entry:
                for source in junction.entry_sources:
                    if get_lane_key(source.wp) in junction.route_entry_keys:
                        scenario_entry_actor_ids.extend([x.id for x in source.actors])

            for actor in list(actor_dict):
                if actor not in actor_dict:
                    continue  # Actor was removed during the loop
                location = CarlaDataProvider.get_location(actor)
                if not location:
                    continue

                state, exit_lane_key, _ = actor_dict[actor].values()
                if self.debug:
                    string = 'J' + str(i+1) + '_' + state[:2]
                    draw_string(self._world, location, string, DEBUG_JUNCTION, False)

                # Special scenario actors. Treat them as road actors
                if actor.id in scenario_entry_actor_ids:
                    self._set_road_actor_speed(location, actor)

                # Monitor its entry
                elif state == JUNCTION_ENTRY:
                    actor_wp = self._map.get_waypoint(location)
                    if self._is_junction(actor_wp) and junction.contains_wp(actor_wp):
                        if junction.clear_middle:
                            self._destroy_actor(actor)  # Don't clutter the junction if a junction scenario is active
                            continue
                        actor_dict[actor]['state'] = JUNCTION_MIDDLE

                # Monitor its exit and destroy an actor if needed
                elif state == JUNCTION_MIDDLE:
                    actor_wp = self._map.get_waypoint(location)
                    actor_lane_key = get_lane_key(actor_wp)
                    if not self._is_junction(actor_wp) and actor_lane_key in exit_dict:
                        if i < max_index and actor_lane_key in junction.route_exit_keys:
                            # Exited through a connecting lane in the route direction.
                            self._remove_actor_info(actor)
                            other_junction = self._active_junctions[i+1]
                            self._add_actor_dict_element(other_junction.actor_dict, actor)

                        elif i > 0 and actor_lane_key in junction.opposite_exit_keys:
                            # Exited through a connecting lane in the opposite direction.
                            # THIS SHOULD NEVER HAPPEN, an entry source should have already added it.
                            other_junction = self._active_junctions[i-1]
                            if actor not in other_junction.actor_dict:
                                self._remove_actor_info(actor)
                                self._add_actor_dict_element(other_junction.actor_dict, actor, at_oppo_entry_lane=True)

                        else:
                            # Check the lane capacity
                            exit_dict[actor_lane_key]['ref_wp'] = actor_wp
                            actor_dict[actor]['state'] = JUNCTION_EXIT
                            actor_dict[actor]['exit_lane_key'] = actor_lane_key

                            actors = exit_dict[actor_lane_key]['actors']
                            if len(actors) > 0 and len(actors) >= exit_dict[actor_lane_key]['max_actors']:
                                self._destroy_actor(actors[0])  # This is always the front most vehicle
                            actors.append(actor)

                # Change them to "road mode" when far enough from the junction
                elif state == JUNCTION_EXIT:
                    distance = location.distance(exit_dict[exit_lane_key]['ref_wp'].transform.location)
                    if distance > exit_dict[exit_lane_key]['max_distance']:
                        if exit_lane_key in junction.route_exit_keys:
                            actor_dict[actor]['state'] = JUNCTION_EXIT_ROAD
                        else:
                            self._actors_speed_perc[actor] = 0
                            actor_dict[actor]['state'] = JUNCTION_EXIT_INACTIVE

                # Set them ready to move so that the ego can smoothly cross the junction
                elif state == JUNCTION_EXIT_ROAD:
                    self._set_road_actor_speed(location, actor, multiplier=1.5)
                    pass

                # Wait
                elif state == JUNCTION_EXIT_INACTIVE:
                    pass

    def _update_opposite_actors(self):
        """
        Updates the opposite actors. This involves tracking their position,
        removing them if too far behind the ego.
        """
        opposite_dist = max(self._opposite_sources_dist, self._opposite_spawn_dist)
        for actor in list(self._opposite_actors):
            location = CarlaDataProvider.get_location(actor)
            if not location:
                continue
            if self.debug:
                draw_string(self._world, location, 'O', DEBUG_OPPOSITE, False)

            distance = location.distance(self._ego_wp.transform.location)
            if distance > opposite_dist and self._is_location_behind_ego(location):
                self._destroy_actor(actor)
                continue

            # Ending / starting lanes create issues as the lane width gradually decreases until reaching 0,
            # where the lane starts / ends. Set their speed to 0, and they'll eventually dissapear.
            actor_wp = self._map.get_waypoint(location)
            if actor_wp.lane_width < self._lane_width_threshold:
                self._actors_speed_perc[actor] = 0

    def _set_actors_speed(self):
        """
        Sets the speed of all the BA actors, using the ego's target speed as reference.
        This avoids issues with the speed limits, as newly created actors don't have that information
        """
        for actor, percentage in self._actors_speed_perc.items():
            speed = self._ego_target_speed * percentage / 100
            if self._scenario_max_speed:
                speed = min(speed, self._scenario_max_speed)

            # TODO: Fix very high speed traffic
            speed = min(speed, 90)
            self._tm.set_desired_speed(actor, speed)

    def _remove_actor_info(self, actor):
        """Removes all the references of the actor"""

        for lane in self._road_dict:
            if actor in self._road_dict[lane].actors:
                self._road_dict[lane].actors.remove(actor)
                break

        if actor in self._opposite_actors:
            self._opposite_actors.remove(actor)
        if actor in self._scenario_stopped_actors:
            self._scenario_stopped_actors.remove(actor)
        if actor in self._scenario_stopped_back_actors:
            self._scenario_stopped_back_actors.remove(actor)

        for opposite_source in self._opposite_sources:
            if actor in opposite_source.actors:
                opposite_source.actors.remove(actor)
                break

        for junction in self._active_junctions:
            junction.actor_dict.pop(actor, None)

            for entry_source in junction.entry_sources:
                if actor in entry_source.actors:
                    entry_source.actors.remove(actor)
                    break

            for exit_keys in junction.exit_dict:
                exit_actors = junction.exit_dict[exit_keys]['actors']
                if actor in exit_actors:
                    exit_actors.remove(actor)
                    break

        self._actors_speed_perc.pop(actor, None)
        if actor in self._all_actors:
            self._all_actors.remove(actor)

    def _destroy_actor(self, actor):
        """Destroy the actor and all its references"""
        self._remove_actor_info(actor)
        try:
            actor.set_autopilot(False, self._tm_port)
            actor.destroy()
        except RuntimeError:
            pass

    def _update_ego_data(self):
        """
        Checks the ego location to see if it has moved closer to the next route waypoint,
        updating its information. This never checks for backwards movements to avoid unnedded confusion.
        It also saves its max speed, used as a baseline for all BA vehicles.
        """
        location = CarlaDataProvider.get_location(self._ego_actor)

        prev_index = self._route_index
        for index in range(self._route_index, min(self._route_index + self._route_buffer, self._route_length)):
            route_wp = self._route[index]

            route_wp_dir = route_wp.transform.get_forward_vector()    # Waypoint's forward vector
            veh_wp_dir = location - route_wp.transform.location       # vector waypoint - vehicle
            dot_ve_wp = veh_wp_dir.x * route_wp_dir.x + veh_wp_dir.y * route_wp_dir.y + veh_wp_dir.z * route_wp_dir.z

            if dot_ve_wp > 0:
                self._route_index = index

        # Monitor route changes for those scenario that remove and readd a specific lane
        if self._scenario_removed_lane:
            for i in range(prev_index, self._route_index):
                option_1 = self._route_options[i]
                option_2 = self._route_options[i+1]
                if option_1 == RoadOption.CHANGELANELEFT or option_2 == RoadOption.CHANGELANELEFT:
                    loc_1 = self._route[i].transform.location
                    loc_2 = self._route[i+1].transform.location
                    if abs(loc_1.distance(loc_2)) > 2.5:  # Lane offset plus a bit forward
                        self._scenario_remove_lane_offset += 1
                elif option_1 == RoadOption.CHANGELANERIGHT or option_2 == RoadOption.CHANGELANERIGHT:
                    loc_1 = self._route[i].transform.location
                    loc_2 = self._route[i+1].transform.location
                    if abs(loc_1.distance(loc_2)) > 2.5:  # Lane offset plus a bit forward
                        self._scenario_remove_lane_offset -= 1

        self._ego_wp = self._route[self._route_index]
        self._ego_key = get_lane_key(self._ego_wp)
        self._ego_target_speed = self._ego_actor.get_speed_limit()

        if self.debug:
            string = 'EGO_' + self._ego_state[0].upper()
            debug_name = DEBUG_ROAD if self._ego_state == EGO_ROAD else DEBUG_JUNCTION
            draw_string(self._world, location, string, debug_name, False)
