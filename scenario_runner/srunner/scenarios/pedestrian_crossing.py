#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Pedestrians crossing through the middle of the lane.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      KeepVelocity,
                                                                      WaitForever,
                                                                      Idle,
                                                                      ActorTransformSetter,
                                                                      MovePedestrianWithEgo)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import HandleJunctionScenario


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


class PedestrianCrossing(BasicScenario):

    """
    This class holds everything required for a group of natual pedestrians crossing the road.
    The ego vehicle is passing through a road,
    And encounters a group of pedestrians crossing the road.

    This is a single ego vehicle scenario.

    Notice that the initial pedestrian will walk from the start of the junction ahead to end_walker_flow_1.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._rng = CarlaDataProvider.get_random_seed()

        self._adversary_speed = 1.3  # Speed of the adversary [m/s]
        self._reaction_time = 3.5  # Time the agent has to react to avoid the collision [s]
        self._min_trigger_dist = 12.0  # Min distance to the collision location that triggers the adversary [m]
        self._ego_end_distance = 40
        self.timeout = timeout

        self._walker_data = [
            {'x': 0.4, 'y': 1.5, 'z': 1.2, 'yaw': 270},
            {'x': 1, 'y': 2.5, 'z': 1.2, 'yaw': 270},
            {'x': 1.6, 'y': 0.5, 'z': 1.2, 'yaw': 270}
        ]

        for walker_data in self._walker_data:
            walker_data['idle_time'] = self._rng.uniform(0, 1.5)
            walker_data['speed'] = self._rng.uniform(1.3, 2.0)

        super().__init__("PedestrianCrossing",
                          ego_vehicles,
                          config,
                          world,
                          debug_mode,
                          criteria_enable=criteria_enable)

    def _get_walker_transform(self, wp, displacement):
        disp_x = displacement['x']
        disp_y = displacement['y']
        disp_z = displacement['z']
        disp_yaw = displacement['yaw']

        # Displace it to the crosswalk. Move forwards towards the crosswalk
        start_vec = wp.transform.get_forward_vector()
        start_right_vec = wp.transform.get_right_vector()

        spawn_loc = wp.transform.location + carla.Location(
            disp_x * start_vec.x + disp_y * start_right_vec.x,
            disp_x * start_vec.y + disp_y * start_right_vec.y,
            disp_x * start_vec.z + disp_y * start_right_vec.z + disp_z
        )

        spawn_rotation = wp.transform.rotation
        spawn_rotation.yaw += disp_yaw
        return carla.Transform(spawn_loc, spawn_rotation)

    def _initialize_actors(self, config):

        # Get the start point of the initial pedestrian
        collision_wp = self._reference_waypoint
        while True:
            next_wps = collision_wp.next(1)
            if not next_wps:
                raise ValueError("Couldn't find a waypoint to spawn the pedestrians")
            if next_wps[0].is_junction:
                break
            collision_wp = next_wps[0]

        self._collision_wp = collision_wp

        # Get the crosswalk start point
        start_wp = collision_wp
        while start_wp.lane_type != carla.LaneType.Sidewalk:
            wp = start_wp.get_right_lane()
            if wp is None:
                raise ValueError("Couldn't find a waypoint to start the flow")
            start_wp = wp

        # Spawn the walkers
        for i, walker_data in enumerate(self._walker_data):
            spawn_transform = self._get_walker_transform(start_wp, walker_data)
            walker = CarlaDataProvider.request_new_actor('walker.*', spawn_transform)
            if walker is None:
                for walker in self.other_actors:
                    walker.destroy()
                raise ValueError("Failed to spawn an adversary")

            walker.set_location(spawn_transform.location + carla.Location(z=-200))
            walker = self._replace_walker(walker)

            self.other_actors.append(walker)

            collision_dist = spawn_transform.location.distance(self._collision_wp.transform.location)

            # Distance and duration to cross the whole road + a bit more (supposing symetry in both directions)
            move_dist = 2.3 * collision_dist
            walker_data['transform'] = spawn_transform
            walker_data['distance'] = move_dist
            walker_data['duration'] = move_dist / walker_data['speed']

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence(name="PedestrianCrossing")
        if self.route_mode:
            sequence.add_child(HandleJunctionScenario(
                clear_junction=False,
                clear_ego_entry=True,
                remove_entries=[],
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=0
            ))

        for walker_actor, walker_data in zip(self.other_actors, self._walker_data):
            sequence.add_child(ActorTransformSetter(walker_actor, walker_data['transform'], True))

        collision_location = self._collision_wp.transform.location

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the walkers
        main_behavior = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="WalkerMovement")

        for walker_actor, walker_data in zip(self.other_actors, self._walker_data):
            walker_sequence = py_trees.composites.Sequence(name="WalkerCrossing")
            walker_sequence.add_child(Idle(walker_data['idle_time']))
            walker_sequence.add_child(KeepVelocity(
                walker_actor, walker_data['speed'], False, walker_data['duration'], walker_data['distance']))
            walker_sequence.add_child(ActorDestroy(walker_actor, name="DestroyAdversary"))
            walker_sequence.add_child(WaitForever())

            main_behavior.add_child(walker_sequence)

        main_behavior.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))
        sequence.add_child(main_behavior)

        # Remove everything

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

        for i, walker in enumerate(reversed(self.other_actors)):
            parallel.add_child(MovePedestrianWithEgo(self.ego_vehicles[0], walker, 100))

        parallel.add_child(trigger_tree)
        return parallel
