#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Control Loss Vehicle scenario:

The scenario realizes that the vehicle looses control due to
bad road conditions, etc. and checks to see if the vehicle
regains control and corrects it's course.
"""

from numpy import random
import py_trees
import operator

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AddNoiseToRouteEgo
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class ControlLoss(BasicScenario):

    """
    Implementation of "Control Loss Vehicle" (Traffic Scenario 01)

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self.timeout = timeout
        self._randomize = randomize
        self._rng = CarlaDataProvider.get_random_seed()

        self._map = CarlaDataProvider.get_map()
        self._end_distance = 110

        # Friction loss tends to have a much stronger steering compoenent then a throttle one
        self._throttle_mean = 0.03
        self._throttle_std = 0.01
        self._steer_mean = 0.055
        self._steer_std = 0.015

        self._trigger_dist = 2

        super().__init__("ControlLoss", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        if self._randomize:
            self._distance = list(self._rng.randint(low=10, high=80, size=3))
            self._distance = sorted(self._distance)
            self._offset = list(2 * random.rand(3) - 1)
        else:
            self._distance = [14, 48, 74]
            self._offset = [-0.6, 0.8, 0.2]

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        # Get the debris locations
        first_wp, _ = get_waypoint_in_distance(self._reference_waypoint, self._distance[0])
        first_ground_loc = self.world.ground_projection(first_wp.transform.location + carla.Location(z=1), 2)
        first_loc = first_ground_loc.location if first_ground_loc else first_wp.transform.location
        self.first_transform = carla.Transform(first_loc, first_wp.transform.rotation)

        second_wp, _ = get_waypoint_in_distance(self._reference_waypoint, self._distance[1])
        second_ground_loc = self.world.ground_projection(second_wp.transform.location + carla.Location(z=1), 2)
        second_loc = second_ground_loc.location if second_ground_loc else second_wp.transform.location
        self.second_transform = carla.Transform(second_loc, second_wp.transform.rotation)

        third_wp, _ = get_waypoint_in_distance(self._reference_waypoint, self._distance[2])
        third_ground_loc = self.world.ground_projection(third_wp.transform.location + carla.Location(z=1), 2)
        third_loc = third_ground_loc.location if third_ground_loc else third_wp.transform.location
        self.third_transform = carla.Transform(third_loc, third_wp.transform.rotation)

        # Spawn the debris
        first_debris = CarlaDataProvider.request_new_actor(
            'static.prop.dirtdebris01', self.first_transform, rolename='prop')
        second_debris = CarlaDataProvider.request_new_actor(
            'static.prop.dirtdebris01', self.second_transform, rolename='prop')
        third_debris = CarlaDataProvider.request_new_actor(
            'static.prop.dirtdebris01', self.third_transform, rolename='prop')

        # Remove their physics
        first_debris.set_simulate_physics(False)
        second_debris.set_simulate_physics(False)
        third_debris.set_simulate_physics(False)

        self.other_actors.append(first_debris)
        self.other_actors.append(second_debris)
        self.other_actors.append(third_debris)

    def _get_noise_parameters(self):
        """Randomizes the mean to be either positive or negative"""
        return [
            self._rng.choice([self._throttle_mean, -self._throttle_mean]),
            self._throttle_std,
            self._rng.choice([self._steer_mean, -self._steer_mean]),
            self._steer_std
        ]

    def _create_behavior(self):
        """
        The scenario defined after is a "control loss vehicle" scenario.
        """
        root = py_trees.composites.Parallel("ControlLoss", py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sequence = py_trees.composites.Sequence()

        # First debris behavior
        sequence.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.first_transform.location, self._trigger_dist))

        noise_1 = self._get_noise_parameters()
        noise_behavior_1 = py_trees.composites.Parallel("Add Noise 1", py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        noise_behavior_1.add_child(AddNoiseToRouteEgo(self.ego_vehicles[0], *noise_1))
        noise_behavior_1.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.first_transform.location, self._trigger_dist, operator.gt))
        sequence.add_child(noise_behavior_1)

        # Second debris behavior
        sequence.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.second_transform.location, self._trigger_dist))

        noise_2 = self._get_noise_parameters()
        noise_behavior_2 = py_trees.composites.Parallel("Add Noise 2", py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        noise_behavior_2.add_child(AddNoiseToRouteEgo(self.ego_vehicles[0], *noise_2))
        noise_behavior_2.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.second_transform.location, self._trigger_dist, operator.gt))
        sequence.add_child(noise_behavior_2)

        # Third debris behavior
        sequence.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.third_transform.location, self._trigger_dist))

        noise_3 = self._get_noise_parameters()
        noise_behavior_3 = py_trees.composites.Parallel("Add Noise 3", py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        noise_behavior_3.add_child(AddNoiseToRouteEgo(self.ego_vehicles[0], *noise_3))
        noise_behavior_3.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.third_transform.location, self._trigger_dist, operator.gt))
        sequence.add_child(noise_behavior_3)

        end_distance = self._end_distance - self._distance[-1]
        sequence.add_child(DriveDistance(self.ego_vehicles[0], end_distance))

        root.add_child(sequence)
        root.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        return root

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
