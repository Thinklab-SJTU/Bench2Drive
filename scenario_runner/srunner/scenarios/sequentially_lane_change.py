import py_trees
from numpy import random

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorFlow, TrafficLightFreezer, ScenarioTimeout
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitEndIntersection, DriveDistance
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (generate_target_waypoint,
                                           get_junction_topology,
                                           filter_junction_wp_direction,
                                           get_same_dir_lanes,
                                           get_closest_traffic_light)

from srunner.tools.background_manager import HandleJunctionScenario, ChangeOppositeBehavior

class SequentialLaneChange(BasicScenario):

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80, activate_scenario=True):
        """
        Setup all relevant parameters and create scenario
        """
        pass
        self._scenario_timeout = 240
        super().__init__("SequentialLaneChange",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        pass

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="SequentialLaneChange")
        end_condition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], 200))
        end_condition.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        sequence.add_child(end_condition)
        return sequence
        pass

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = [ScenarioTimeoutTest(self.ego_vehicles[0], self.config.name)]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria
        pass

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        pass