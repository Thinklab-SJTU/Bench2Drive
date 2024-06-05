#!/usr/bin/env python

# Copyright (c) 2018-2022 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Hard break scenario:

The scenario spawn a vehicle in front of the ego that drives for a while before
suddenly hard breaking, forcing the ego to avoid the collision
"""

import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import StopFrontVehicles, StartFrontVehicles


class HardBreakRoute(BasicScenario):

    """
    This class uses the is the Background Activity at routes to create a hard break scenario.

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self.timeout = timeout
        self._stop_duration = 10
        self.end_distance = 15

        super().__init__("HardBreak",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        pass

    def _create_behavior(self):
        """
        Uses the Background Activity to force a hard break on the vehicles in front of the actor,
        then waits for a bit to check if the actor has collided. After a set duration,
        the front vehicles will resume their movement
        """
        sequence = py_trees.composites.Sequence("HardBreak")
        sequence.add_child(StopFrontVehicles())
        sequence.add_child(Idle(self._stop_duration))
        sequence.add_child(StartFrontVehicles())
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self.end_distance))

        return sequence

    def _create_test_criteria(self):
        """
        Empty, the route already has a collision criteria
        """
        return []

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
