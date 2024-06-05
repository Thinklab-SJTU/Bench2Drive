#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Object crash without prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encountering a cyclist ahead.
"""

from __future__ import print_function

import math
import py_trees
import carla
from math import floor

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      KeepVelocity,
                                                                      Idle,
                                                                      ActorTransformSetter,
                                                                      MovePedestrianWithEgo)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp

from srunner.tools.background_manager import LeaveSpaceInFront, LeaveCrossingSpace


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default


class StationaryObjectCrossing(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryObjectCrossing, self).__init__("Stationaryobjectcrossing",
                                                       ego_vehicles,
                                                       config,
                                                       world,
                                                       debug_mode,
                                                       criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        _distance = 40
        lane_width = self._reference_waypoint.lane_width
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.4, "k": 0.2}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('static.prop.container', self.transform)
        static.set_simulate_physics(True)
        self.other_actors.append(static)

    def _create_behavior(self):
        """
        Only behavior here is to wait
        """
        lane_width = self.ego_vehicles[0].get_world().get_map().get_waypoint(
            self.ego_vehicles[0].get_location()).lane_width
        lane_width = lane_width + (1.25 * lane_width)

        # leaf nodes
        actor_stand = Idle(15)
        actor_removed = ActorDestroy(self.other_actors[0])
        end_condition = DriveDistance(self.ego_vehicles[0], self._ego_vehicle_distance_driven)

        # non leaf nodes
        root = py_trees.composites.Parallel(
            name="StaticObstacle",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(actor_stand)
        scenario_sequence.add_child(actor_removed)
        scenario_sequence.add_child(end_condition)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class DynamicObjectCrossing(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._num_lane_changes = 0

        self._blocker_shift = 0.9
        self._retry_dist = 0.4

        self._adversary_transform = None
        self._blocker_transform = None
        self._collision_wp = None

        self._adversary_speed = 2.0  # Speed of the adversary [m/s]
        self._crossing_angle = get_value_parameter(config, 'crossing_angle', float, 0)
        self._reaction_time = 2.1  # Time the agent has to react to avoid the collision [s]
        self._reaction_time += 0.1 * floor(self._crossing_angle / 5)
        self._min_trigger_dist = 6.0  # Min distance to the collision location that triggers the adversary [m]
        self._ego_end_distance = 40
        self.timeout = timeout

        self._number_of_attempts = 6

        self._distance = get_value_parameter(config, 'distance', float, 12)
        self._blocker_model = get_value_parameter(config, 'blocker_model', str, 'static.prop.vendingmachine')
        if abs(self._crossing_angle) > 90:
            raise ValueError("'crossing_angle' must be between -90 and 90º for the pedestrian to cross the road")
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        super(DynamicObjectCrossing, self).__init__("DynamicObjectCrossing",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _get_sidewalk_transform(self, waypoint, offset):
        """
        Processes the waypoint transform to find a suitable spawning one at the sidewalk.
        It first rotates the transform so that it is pointing towards the road and then moves a
        bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
        """
        if self._direction == "left":
            offset['yaw'] *= -1
            offset['k'] *= -1

        new_rotation = waypoint.transform.rotation
        new_rotation.yaw += offset['yaw']

        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            right_vector = waypoint.transform.get_right_vector()
            offset_dist = offset["k"]
            offset_location = carla.Location(offset_dist * right_vector.x, offset_dist * right_vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += offset['z']

        return carla.Transform(new_location, new_rotation)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Get the waypoint in front of the ego.
        move_dist = self._distance
        waypoint = self._reference_waypoint

        while self._number_of_attempts > 0:
            parking_location = None
            self._collision_dist = 0

            # Move to the front
            location, _ = get_location_in_distance_from_wp(waypoint, move_dist, False)
            waypoint = self._wmap.get_waypoint(location)
            self._collision_wp = waypoint

            # Move to the right
            sidewalk_waypoint = waypoint
            while sidewalk_waypoint.lane_type != carla.LaneType.Sidewalk:
                if self._direction == "right":
                    side_wp = sidewalk_waypoint.get_right_lane()
                else:
                    side_wp = sidewalk_waypoint.get_left_lane()
                if side_wp is None:
                    break  # No more side lanes
                sidewalk_waypoint = side_wp
                if side_wp.lane_type == carla.LaneType.Parking:
                    parking_location = side_wp.transform.location

            # Get the blocker transform and spawn it
            offset = {"yaw": 0 if 'vehicle' in self._blocker_model else 90, "z": 0.0, "k": 1.5}
            self._blocker_transform = self._get_sidewalk_transform(sidewalk_waypoint, offset)
            blocker = CarlaDataProvider.request_new_actor(
                self._blocker_model, self._blocker_transform, rolename="scenario no lights")
            if not blocker:
                self._number_of_attempts -= 1
                move_dist = self._retry_dist
                print("Failed to spawn the blocker")
                continue

            # Get the adversary transform and spawn it
            walker_dist = blocker.bounding_box.extent.x + 0.5
            wps = sidewalk_waypoint.next(walker_dist)
            if not wps:
                raise ValueError("Couldn't find a location to spawn the adversary")
            walker_wp = wps[0]

            offset = {"yaw": 270 - self._crossing_angle, "z": 1.2, "k": 1.2}
            self._adversary_transform = self._get_sidewalk_transform(walker_wp, offset)
            adversary = CarlaDataProvider.request_new_actor('walker.*', self._adversary_transform)
            if adversary is None:
                blocker.destroy()
                self._number_of_attempts -= 1
                move_dist = self._retry_dist
                print("Failed to spawn an adversary")
                continue

            self._collision_dist += waypoint.transform.location.distance(self._adversary_transform.location)

            # Both actors were succesfully spawned, end
            break

        if self._number_of_attempts == 0:
            raise Exception("Couldn't find viable position for the adversary and blocker actors")

        blocker.set_simulate_physics(False)
        adversary.set_location(self._adversary_transform.location + carla.Location(z=-200))
        adversary = self._replace_walker(adversary)

        if parking_location:
            self.parking_slots.append(parking_location)

        self.other_actors.append(adversary)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence(name="CrossingActor")
        if self.route_mode:
            total_dist = self._distance + 10
            sequence.add_child(LeaveSpaceInFront(total_dist))

        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._adversary_transform, True))
        collision_location = self._collision_wp.transform.location

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the adversary
        move_distance = 2 * self._collision_dist  # Cross the whole road (supposing symetry in both directions)
        move_duration = move_distance / self._adversary_speed
        if self.route_mode:
            sequence.add_child(LeaveCrossingSpace(self._collision_wp))
        sequence.add_child(KeepVelocity(
            self.other_actors[0], self._adversary_speed,
            duration=move_duration, distance=move_distance, name="AdversaryCrossing"))

        # Remove everything
        sequence.add_child(ActorDestroy(self.other_actors[0], name="DestroyAdversary"))
        sequence.add_child(ActorDestroy(self.other_actors[1], name="DestroyBlocker"))
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        return sequence

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

    # TODO: Pedestrian have an issue with large maps were setting them to dormant breaks them,
    # so all functions below are meant to patch it until the fix is done
    def _replace_walker(self, adversary):
        """As the adversary is probably, replace it with another one"""
        type_id = adversary.type_id
        adversary.destroy()
        spawn_transform = self.ego_vehicles[0].get_transform()
        spawn_transform.location.z -= 50
        adversary = CarlaDataProvider.request_new_actor(type_id, spawn_transform)
        if not adversary:
            raise ValueError("Couldn't spawn the walker substitute")
        adversary.set_simulate_physics(False)
        adversary.set_location(spawn_transform.location + carla.Location(z=-50))
        return adversary

    def _setup_scenario_trigger(self, config):
        """Normal scenario trigger but in parallel, a behavior that ensures the pedestrian stays active"""
        trigger_tree = super()._setup_scenario_trigger(config)

        if not self.route_mode:
            return trigger_tree

        parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="ScenarioTrigger")

        parallel.add_child(MovePedestrianWithEgo(self.ego_vehicles[0], self.other_actors[0], 100))

        parallel.add_child(trigger_tree)
        return parallel


class ParkingCrossingPedestrian(BasicScenario):

    """
    Variation of DynamicObjectCrossing but now the blocker is now a vehicle
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._num_lane_changes = 0

        self._adversary_speed = 2.0  # Speed of the adversary [m/s]
        self._min_trigger_dist = 6.0  # Min distance to the collision location that triggers the adversary [m]
        self._ego_end_distance = 40
        self.timeout = timeout

        self._bp_attributes = {'base_type': 'car', 'generation': 2}

        self._distance = get_value_parameter(config, 'distance', float, 12)
        self._crossing_angle = get_value_parameter(config, 'crossing_angle', float, 0)
        if abs(self._crossing_angle) > 90:
            raise ValueError("'crossing_angle' must be between -90 and 90º for the pedestrian to cross the road")
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        # Time the agent has to react to avoid the collision [s]
        self._reaction_time = 2.15
        self._reaction_time += 0.1 * floor(self._crossing_angle / 5)

        super().__init__("ParkingCrossingPedestrian",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _get_blocker_transform(self, waypoint):
        """Processes the driving wp to get a waypoint at the side that looks at the road"""
        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            vector = waypoint.transform.get_right_vector()
            if self._direction == 'left':
                vector *= -1

            offset_location = carla.Location(waypoint.lane_width * vector.x, waypoint.lane_width * vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += 0.5

        return carla.Transform(new_location, waypoint.transform.rotation)

    def _get_walker_transform(self, waypoint):
        """Processes the driving wp to get a waypoint at the side that looks at the road"""

        new_rotation = waypoint.transform.rotation
        new_rotation.yaw += 270 - self._crossing_angle if self._direction == 'right' else 90 + self._crossing_angle

        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            vector = waypoint.transform.get_right_vector()
            if self._direction == 'left':
                vector *= -1

            offset_location = carla.Location(waypoint.lane_width * vector.x, waypoint.lane_width * vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += 1.2

        return carla.Transform(new_location, new_rotation)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Get the adversary transform and spawn it
        wps = self._reference_waypoint.next(self._distance)
        if not wps:
            raise ValueError("Couldn't find a location to spawn the adversary")
        blocker_wp = wps[0]

        # Get the adversary transform and spawn it
        self._blocker_transform = self._get_blocker_transform(blocker_wp)
        self.parking_slots.append(self._blocker_transform.location)
        blocker = CarlaDataProvider.request_new_actor(
            'vehicle.*', self._blocker_transform, attribute_filter=self._bp_attributes)
        if blocker is None:
            raise ValueError("Couldn't spawn the adversary")
        self.other_actors.append(blocker)
        blocker.apply_control(carla.VehicleControl(hand_brake=True))

        walker_dist = blocker.bounding_box.extent.x + 0.5
        wps = blocker_wp.next(walker_dist)
        if not wps:
            raise ValueError("Couldn't find a location to spawn the adversary")
        walker_wp = wps[0]

        # Get the adversary transform and spawn it
        self._walker_transform = self._get_walker_transform(walker_wp)
        self.parking_slots.append(self._walker_transform.location)

        walker = CarlaDataProvider.request_new_actor('walker.*', self._walker_transform)
        if walker is None:
            raise ValueError("Couldn't spawn the adversary")

        walker.set_location(self._walker_transform.location + carla.Location(z=-200))
        walker = self._replace_walker(walker)
 
        self.other_actors.append(walker)

        self._collision_wp = walker_wp

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence(name="ParkingCrossingPedestrian")
        if self.route_mode:
            total_dist = self._distance + 15
            sequence.add_child(LeaveSpaceInFront(total_dist))

        sequence.add_child(ActorTransformSetter(self.other_actors[1], self._walker_transform, True))
        collision_location = self._collision_wp.transform.location

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the adversary
        distance = 8.0  # Scenario is meant to be used at a one lane - one direction road
        duration = distance / self._adversary_speed

        sequence.add_child(KeepVelocity(
            self.other_actors[1], self._adversary_speed,
            duration=duration, distance=distance, name="AdversaryCrossing"))

        # Remove everything
        sequence.add_child(ActorDestroy(self.other_actors[1], name="DestroyAdversary"))
        sequence.add_child(ActorDestroy(self.other_actors[0], name="DestroyBlocker"))
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        return sequence

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

    # TODO: Pedestrian have an issue with large maps were setting them to dormant breaks them,
    # so all functions below are meant to patch it until the fix is done
    def _replace_walker(self, walker):
        """As the adversary is probably, replace it with another one"""
        type_id = walker.type_id
        walker.destroy()
        spawn_transform = self.ego_vehicles[0].get_transform()
        spawn_transform.location.z -= 50
        walker = CarlaDataProvider.request_new_actor(type_id, spawn_transform)
        if not walker:
            raise ValueError("Couldn't spawn the walker substitute")
        walker.set_simulate_physics(False)
        walker.set_location(spawn_transform.location + carla.Location(z=-50))
        return walker

    def _setup_scenario_trigger(self, config):
        """Normal scenario trigger but in parallel, a behavior that ensures the pedestrian stays active"""
        trigger_tree = super()._setup_scenario_trigger(config)

        if not self.route_mode:
            return trigger_tree

        parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="ScenarioTrigger")

        parallel.add_child(MovePedestrianWithEgo(self.ego_vehicles[0], self.other_actors[1], 100))

        parallel.add_child(trigger_tree)
        return parallel
