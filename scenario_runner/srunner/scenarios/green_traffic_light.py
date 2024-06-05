#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Sets the ego incoming traffic light to green. Support scenario at routes
to let the ego gather speed
"""

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import TrafficLightFreezer
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitEndIntersection
from srunner.scenarios.basic_scenario import BasicScenario


class PriorityAtJunction(BasicScenario):
    """
    Sets the ego incoming traffic light to green. Support scenario at routes
    to let the ego gather speed
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._tl_dict = {}

        self.timeout = timeout
        super().__init__("PriorityAtJunction",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Get the junction and traffic lights
        """
        ego_location = config.trigger_points[0].location
        self._ego_wp = CarlaDataProvider.get_map().get_waypoint(ego_location)

        # Get the junction
        starting_wp = self._ego_wp
        ego_junction_dist = 0
        while not starting_wp.is_junction:
            starting_wps = starting_wp.next(1.0)
            if len(starting_wps) == 0:
                raise ValueError("Failed to find junction")
            starting_wp = starting_wps[0]
            ego_junction_dist += 1
        self._junction = starting_wp.get_junction()

        self._get_traffic_lights(self._junction, ego_junction_dist)

    def _get_traffic_lights(self, junction, junction_dist):
        """Get the traffic light of the junction, mapping their states"""
        tls = self._world.get_traffic_lights_in_junction(junction.id)
        if not tls:
            raise ValueError("No traffic lights found, nothing to do here")

        ego_landmark = self._ego_wp.get_landmarks_of_type(junction_dist + 1, "1000001")[0]
        ego_tl = self._world.get_traffic_light(ego_landmark)
        for tl in tls:
            self._tl_dict[tl] = carla.TrafficLightState.Green if tl.id == ego_tl.id else carla.TrafficLightState.Red

    def _create_behavior(self):
        """
        Freeze the traffic lights until the ego has exited the junction
        """
        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(WaitEndIntersection(self.ego_vehicles[0], self._junction.id))
        root.add_child(TrafficLightFreezer(self._tl_dict))
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        return []

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
