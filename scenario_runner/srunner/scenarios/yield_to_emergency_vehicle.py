#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario in which the ego has to yield its lane to emergency vehicle.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      Idle,
                                                                      AdaptiveConstantVelocityAgentBehavior)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, YieldToEmergencyVehicleTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               WaitUntilInFront,
                                                                               DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.background_manager import RemoveRoadLane, ReAddRoadLane


class YieldToEmergencyVehicle(BasicScenario):
    """
    This class holds everything required for a scenario in which the ego has to yield its lane to emergency vehicle.
    The background activity will be removed from the lane the emergency vehicle will pass through, 
    and will be recreated once the scenario is over.

    Should be on the highway which is long enough and has no junctions.
    There should be at least two lanes on the highway.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self._ev_idle_time = 10  # seconds

        # km/h. How much the EV is expected to be faster than the EGO
        self._speed_increment = 25

        self._trigger_distance = 50

        if 'distance' in config.other_parameters:
            self._distance = float(config.other_parameters['distance']['value'])
        else:
            self._distance = 140  # m

        # Change some of the parameters to adapt its behavior.
        # 1) ConstantVelocityAgent = infinite acceleration -> reduce the detection radius to pressure the ego
        # 2) Always use the bb check to ensure the EV doesn't run over the ego when it is lane changing
        # 3) Add more wps to improve BB detection
        self._opt_dict = {
            'base_vehicle_threshold': 10, 'detection_speed_ratio': 0.15, 'use_bbs_detection': True,
            'base_min_distance': 1, 'distance_ratio': 0.2
            }

        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(self._trigger_location)

        self._end_distance = 50

        super().__init__("YieldToEmergencyVehicle",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Spawn emergency vehicle
        ev_points = self._reference_waypoint.previous(self._distance)
        if not ev_points:
            raise ValueError("Couldn't find viable position for the emergency vehicle")

        self._ev_start_transform = ev_points[0].transform

        actor = CarlaDataProvider.request_new_actor(
            "vehicle.*.*", self._ev_start_transform, attribute_filter={'special_type': 'emergency'})
        if actor is None:
            raise Exception("Couldn't spawn the emergency vehicle")

        # Move the actor underground and remove its physics so that it doesn't fall
        actor.set_simulate_physics(False)
        new_location = actor.get_location()
        new_location.z -= 500
        actor.set_location(new_location)

        # Turn on special lights
        actor.set_light_state(carla.VehicleLightState(
            carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2))

        self.other_actors.append(actor)

    def _create_behavior(self):
        """
        Spawn the EV behind and wait for it to be close-by. After it has approached,
        give the ego a certain amount of time to yield to it.
        
        Sequence:
        - RemoveRoadLane
        - ActorTransformSetter
        - Parallel:
            - AdaptiveConstantVelocityAgentBehavior
            - Sequence: (End condition 1)
                - InTriggerDistanceToVehicle:
                - Idle
            - Sequence: (End condition 2)
                - WaitUntilInFront
                - DriveDistance
        - ReAddRoadLane
        """
        sequence = py_trees.composites.Sequence(name="YieldToEmergencyVehicle")

        if self.route_mode:
            sequence.add_child(RemoveRoadLane(self._reference_waypoint))

        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._ev_start_transform))

        main_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        end_condition_1 = py_trees.composites.Sequence()
        end_condition_1.add_child(InTriggerDistanceToVehicle(
            self.ego_vehicles[0], self.other_actors[0], self._trigger_distance))
        end_condition_1.add_child(Idle(self._ev_idle_time))

        end_condition_2 = py_trees.composites.Sequence()
        end_condition_2.add_child(WaitUntilInFront(self.other_actors[0], self.ego_vehicles[0]))
        end_condition_2.add_child(DriveDistance(self.other_actors[0], self._end_distance))

        main_behavior.add_child(end_condition_1)
        main_behavior.add_child(end_condition_2)

        main_behavior.add_child(AdaptiveConstantVelocityAgentBehavior(
            self.other_actors[0], self.ego_vehicles[0], speed_increment=self._speed_increment, opt_dict=self._opt_dict))

        sequence.add_child(main_behavior)

        sequence.add_child(ActorDestroy(self.other_actors[0]))

        if self.route_mode:
            sequence.add_child(ReAddRoadLane(0))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criterias = []
        criterias.append(YieldToEmergencyVehicleTest(self.ego_vehicles[0], self.other_actors[0]))
        if not self.route_mode:
            criterias.append(CollisionTest(self.ego_vehicles[0]))

        return criterias

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
