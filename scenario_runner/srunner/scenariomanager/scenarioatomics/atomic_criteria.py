#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides all atomic evaluation criteria required to analyze if a
scenario was completed successfully or failed.

Criteria should run continuously to monitor the state of a single actor, multiple
actors or environmental parameters. Hence, a termination is not required.

The atomic criteria are implemented with py_trees.
"""

import math
import numpy as np
import py_trees
import shapely.geometry

import carla
from agents.tools.misc import get_speed

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType


class Criterion(py_trees.behaviour.Behaviour):

    """
    Base class for all criteria used to evaluate a scenario for success/failure

    Important parameters (PUBLIC):
    - name: Name of the criterion
    - actor: Actor of the criterion
    - optional: Indicates if a criterion is optional (not used for overall analysis)
    - terminate on failure: Whether or not the criteria stops on failure

    - test_status: Used to access the result of the criterion
    - success_value: Result in case of success (e.g. max_speed, zero collisions, ...)
    - acceptable_value: Result that does not mean a failure,  but is not good enough for a success
    - actual_value: Actual result after running the scenario
    - units: units of the 'actual_value'. This is a string and is used by the result writter
    """

    def __init__(self,
                 name,
                 actor,
                 optional=False,
                 terminate_on_failure=False):
        super(Criterion, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        self.name = name
        self.actor = actor
        self.optional = optional
        self._terminate_on_failure = terminate_on_failure
        self.test_status = "INIT"   # Either "INIT", "RUNNING", "SUCCESS", "ACCEPTABLE" or "FAILURE"

        # Attributes to compare the current state (actual_value), with the expected ones
        self.success_value = 0
        self.acceptable_value = None
        self.actual_value = 0
        self.units = "times"

        self.events = []  # List of events (i.e collision, sidewalk invasion...)

    def initialise(self):
        """
        Initialise the criterion. Can be extended by the user-derived class
        """
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def terminate(self, new_status):
        """
        Terminate the criterion. Can be extended by the user-derived class
        """
        if self.test_status in ('RUNNING', 'INIT'):
            self.test_status = "SUCCESS"

        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class MaxVelocityTest(Criterion):

    """
    This class contains an atomic test for maximum velocity.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - max_velocity_allowed: maximum allowed velocity in m/s
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, max_velocity, optional=False, name="CheckMaximumVelocity"):
        """
        Setup actor and maximum allowed velovity
        """
        super(MaxVelocityTest, self).__init__(name, actor, optional)
        self.success_value = max_velocity

    def update(self):
        """
        Check velocity
        """
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None:
            return new_status

        velocity = CarlaDataProvider.get_velocity(self.actor)

        self.actual_value = max(velocity, self.actual_value)

        if velocity > self.success_value:
            self.test_status = "FAILURE"
        else:
            self.test_status = "SUCCESS"

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class DrivenDistanceTest(Criterion):
    """
    This class contains an atomic test to check the driven distance

    Important parameters:
    - actor: CARLA actor to be used for this test
    - distance_success: If the actor's driven distance is more than this value (in meters),
                        the test result is SUCCESS
    - distance_acceptable: If the actor's driven distance is more than this value (in meters),
                           the test result is ACCEPTABLE
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, distance, acceptable_distance=None, optional=False, name="CheckDrivenDistance"):
        """
        Setup actor
        """
        super(DrivenDistanceTest, self).__init__(name, actor, optional)
        self.success_value = distance
        self.acceptable_value = acceptable_distance
        self._last_location = None

    def initialise(self):
        self._last_location = CarlaDataProvider.get_location(self.actor)
        super(DrivenDistanceTest, self).initialise()

    def update(self):
        """
        Check distance
        """
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None:
            return new_status

        location = CarlaDataProvider.get_location(self.actor)

        if location is None:
            return new_status

        if self._last_location is None:
            self._last_location = location
            return new_status

        self.actual_value += location.distance(self._last_location)
        self._last_location = location

        if self.actual_value > self.success_value:
            self.test_status = "SUCCESS"
        elif (self.acceptable_value is not None and
              self.actual_value > self.acceptable_value):
            self.test_status = "ACCEPTABLE"
        else:
            self.test_status = "RUNNING"

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        Set final status
        """
        if self.test_status != "SUCCESS":
            self.test_status = "FAILURE"
        self.actual_value = round(self.actual_value, 2)
        super(DrivenDistanceTest, self).terminate(new_status)


class AverageVelocityTest(Criterion):

    """
    This class contains an atomic test for average velocity.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - avg_velocity_success: If the actor's average velocity is more than this value (in m/s),
                            the test result is SUCCESS
    - avg_velocity_acceptable: If the actor's average velocity is more than this value (in m/s),
                               the test result is ACCEPTABLE
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, velocity, acceptable_velocity=None, optional=False,
                 name="CheckAverageVelocity"):
        """
        Setup actor and average velovity expected
        """
        super(AverageVelocityTest, self).__init__(name, actor, optional)
        self.success_value = velocity
        self.acceptable_value = acceptable_velocity
        self._last_location = None
        self._distance = 0.0

    def initialise(self):
        self._last_location = CarlaDataProvider.get_location(self.actor)
        super(AverageVelocityTest, self).initialise()

    def update(self):
        """
        Check velocity
        """
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None:
            return new_status

        location = CarlaDataProvider.get_location(self.actor)

        if location is None:
            return new_status

        if self._last_location is None:
            self._last_location = location
            return new_status

        self._distance += location.distance(self._last_location)
        self._last_location = location

        elapsed_time = GameTime.get_time()
        if elapsed_time > 0.0:
            self.actual_value = self._distance / elapsed_time

        if self.actual_value > self.success_value:
            self.test_status = "SUCCESS"
        elif (self.acceptable_value is not None and
              self.actual_value > self.acceptable_value):
            self.test_status = "ACCEPTABLE"
        else:
            self.test_status = "RUNNING"

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        Set final status
        """
        if self.test_status == "RUNNING":
            self.test_status = "FAILURE"
        super(AverageVelocityTest, self).terminate(new_status)


class CollisionTest(Criterion):

    """
    This class contains an atomic test for collisions.

    Args:
    - actor (carla.Actor): CARLA actor to be used for this test
    - other_actor (carla.Actor): only collisions with this actor will be registered
    - other_actor_type (str): only collisions with actors including this type_id will count.
        Additionally, the "miscellaneous" tag can also be used to include all static objects in the scene
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    COLLISION_RADIUS = 5  # Two collisions that happen within this distance count as one
    MAX_ID_TIME = 5  # Two collisions with the same id that happen within this time count as one
    EPSILON = 0.1  # Collisions at lower this speed won't be counted as the actor's fault

    def __init__(self, actor, other_actor=None, other_actor_type=None,
                 optional=False, terminate_on_failure=False, name="CollisionTest"):
        """
        Construction with sensor setup
        """
        super(CollisionTest, self).__init__(name, actor, optional, terminate_on_failure)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._other_actor = other_actor
        self._other_actor_type = other_actor_type

        # Attributes to store the last collisions's data
        self._collision_sensor = None
        self._collision_id = None
        self._collision_time = None
        self._collision_location = None

    def initialise(self):
        """
        Creates the sensor and callback"""
        world = CarlaDataProvider.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self._collision_sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self.actor)
        self._collision_sensor.listen(lambda event: self._count_collisions(event))
        super(CollisionTest, self).initialise()

    def update(self):
        """
        Check collision count
        """
        new_status = py_trees.common.Status.RUNNING

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        actor_location = CarlaDataProvider.get_location(self.actor)

        # Check if the last collision can be ignored
        if self._collision_location:
            distance_vector = actor_location - self._collision_location
            if distance_vector.length() > self.COLLISION_RADIUS:
                self._collision_location = None
        if self._collision_id:
            elapsed_time = GameTime.get_time() - self._collision_time
            if elapsed_time > self.MAX_ID_TIME:
                self._collision_id = None

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        Cleanup sensor
        """
        if self._collision_sensor is not None:
            self._collision_sensor.stop()
            self._collision_sensor.destroy()
        self._collision_sensor = None
        super(CollisionTest, self).terminate(new_status)

    def _count_collisions(self, event):     # pylint: disable=too-many-return-statements
        """Update collision count"""
        actor_location = CarlaDataProvider.get_location(self.actor)

        # Check if the care about the other actor
        if self._other_actor and self._other_actor.id != event.other_actor.id:
            return

        if self._other_actor_type:
            if self._other_actor_type == "miscellaneous":  # Special OpenScenario case
                if "traffic" not in event.other_actor.type_id and "static" not in event.other_actor.type_id:
                    return
            elif self._other_actor_type not in event.other_actor.type_id:
                    return

        # To avoid multiple counts of the same collision, filter some of them.
        if self._collision_id == event.other_actor.id:
            return
        if self._collision_location:
            distance_vector = actor_location - self._collision_location
            if distance_vector.length() <= self.COLLISION_RADIUS:
                return

        # If the actor speed is 0, the collision isn't its fault
        if CarlaDataProvider.get_velocity(self.actor) < self.EPSILON:
            return

        # The collision is valid, save the data
        self.test_status = "FAILURE"
        self.actual_value += 1

        self._collision_time = GameTime.get_time()
        self._collision_location = actor_location
        if event.other_actor.id != 0: # Number 0: static objects -> ignore it
            self._collision_id = event.other_actor.id

        if ('static' in event.other_actor.type_id or 'traffic' in event.other_actor.type_id) \
                and 'sidewalk' not in event.other_actor.type_id:
            actor_type = TrafficEventType.COLLISION_STATIC
        elif 'vehicle' in event.other_actor.type_id:
            actor_type = TrafficEventType.COLLISION_VEHICLE
        elif 'walker' in event.other_actor.type_id:
            actor_type = TrafficEventType.COLLISION_PEDESTRIAN
        else:
            return

        collision_event = TrafficEvent(event_type=actor_type, frame=GameTime.get_frame())
        collision_event.set_dict({'other_actor': event.other_actor, 'location': actor_location})
        collision_event.set_message(
            "Agent collided against object with type={} and id={} at (x={}, y={}, z={})".format(
                event.other_actor.type_id,
                event.other_actor.id,
                round(actor_location.x, 3),
                round(actor_location.y, 3),
                round(actor_location.z, 3)))
        self.events.append(collision_event)


class ActorBlockedTest(Criterion):

    """
    This test will fail if the actor has had its linear velocity lower than a specific value for
    a specific amount of time
    Important parameters:
    - actor: CARLA actor to be used for this test
    - min_speed: speed required [m/s]
    - max_time: Maximum time (in seconds) the actor can remain under the speed threshold
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """

    def __init__(self, actor, min_speed, max_time, name="ActorBlockedTest", optional=False, terminate_on_failure=False):
        """
        Class constructor
        """
        super().__init__(name, actor, optional, terminate_on_failure)
        self._min_speed = min_speed
        self._max_time = max_time
        self._time_last_valid_state = None
        self._active = True
        self.units = None  # We care about whether or not it fails, no units attached

    def update(self):
        """
        Check if the actor speed is above the min_speed
        """
        new_status = py_trees.common.Status.RUNNING

        # Deactivate/Activate checking by blackboard message
        active = py_trees.blackboard.Blackboard().get('AC_SwitchActorBlockedTest')
        if active is not None:
            self._active = active
            self._time_last_valid_state = GameTime.get_time()
            py_trees.blackboard.Blackboard().set("AC_SwitchActorBlockedTest", None, overwrite=True)

        if self._active:
            linear_speed = CarlaDataProvider.get_velocity(self.actor)
            if linear_speed is not None:
                if linear_speed < self._min_speed and self._time_last_valid_state:
                    if (GameTime.get_time() - self._time_last_valid_state) > self._max_time:
                        # The actor has been "blocked" for too long, save the data
                        self.test_status = "FAILURE"

                        vehicle_location = CarlaDataProvider.get_location(self.actor)
                        event = TrafficEvent(event_type=TrafficEventType.VEHICLE_BLOCKED, frame=GameTime.get_frame())
                        event.set_message('Agent got blocked at (x={}, y={}, z={})'.format(
                            round(vehicle_location.x, 3),
                            round(vehicle_location.y, 3),
                            round(vehicle_location.z, 3))
                        )
                        event.set_dict({'location': vehicle_location})
                        self.events.append(event)
                else:
                    self._time_last_valid_state = GameTime.get_time()

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE
        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


class KeepLaneTest(Criterion):
    """
    This class contains an atomic test for keeping lane.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, optional=False, name="CheckKeepLane"):
        """
        Construction with sensor setup
        """
        super(KeepLaneTest, self).__init__(name, actor, optional)

        world = self.actor.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self._lane_sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self.actor)
        self._lane_sensor.listen(lambda event: self._count_lane_invasion(event))

    def update(self):
        """
        Check lane invasion count
        """
        new_status = py_trees.common.Status.RUNNING

        if self.actual_value > 0:
            self.test_status = "FAILURE"
        else:
            self.test_status = "SUCCESS"

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        Cleanup sensor
        """
        if self._lane_sensor is not None:
            self._lane_sensor.destroy()
            self._lane_sensor = None
        super(KeepLaneTest, self).terminate(new_status)

    def _count_lane_invasion(self, event):
        """
        Callback to update lane invasion count
        """
        self.actual_value += 1


class ReachedRegionTest(Criterion):

    """
    This class contains the reached region test
    The test is a success if the actor reaches a specified region

    Important parameters:
    - actor: CARLA actor to be used for this test
    - min_x, max_x, min_y, max_y: Bounding box of the checked region
    """

    def __init__(self, actor, min_x, max_x, min_y, max_y, name="ReachedRegionTest"):
        """
        Setup trigger region (rectangle provided by
        [min_x,min_y] and [max_x,max_y]
        """
        super(ReachedRegionTest, self).__init__(name, actor)
        self._min_x = min_x
        self._max_x = max_x
        self._min_y = min_y
        self._max_y = max_y

    def update(self):
        """
        Check if the actor location is within trigger region
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self.actor)
        if location is None:
            return new_status

        in_region = False
        if self.test_status != "SUCCESS":
            in_region = (location.x > self._min_x and location.x < self._max_x) and (
                location.y > self._min_y and location.y < self._max_y)
            if in_region:
                self.test_status = "SUCCESS"
            else:
                self.test_status = "RUNNING"

        if self.test_status == "SUCCESS":
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


class OffRoadTest(Criterion):

    """
    Atomic containing a test to detect when an actor deviates from the driving lanes. This atomic can
    fail when actor has spent a specific time outside driving lanes (defined by OpenDRIVE). Simplified
    version of OnSidewalkTest, and doesn't relly on waypoints with *Sidewalk* lane types

    Args:
        actor (carla.Actor): CARLA actor to be used for this test
        duration (float): Time spent at sidewalks before the atomic fails.
            If terminate_on_failure isn't active, this is ignored.
        optional (bool): If True, the result is not considered for an overall pass/fail result
            when using the output argument
        terminate_on_failure (bool): If True, the atomic will fail when the duration condition has been met.
    """

    def __init__(self, actor, duration=0, optional=False, terminate_on_failure=False, name="OffRoadTest"):
        """
        Setup of the variables
        """
        super(OffRoadTest, self).__init__(name, actor, optional, terminate_on_failure)

        self._map = CarlaDataProvider.get_map()
        self._offroad = False

        self._duration = duration
        self._prev_time = None
        self._time_offroad = 0

    def update(self):
        """
        First, transforms the actor's current position to its corresponding waypoint. This is
        filtered to only use waypoints of type Driving or Parking. Depending on these results,
        the actor will be considered to be outside (or inside) driving lanes.

        returns:
            py_trees.common.Status.FAILURE: when the actor has spent a given duration outside driving lanes
            py_trees.common.Status.RUNNING: the rest of the time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = CarlaDataProvider.get_location(self.actor)

        # Get the waypoint at the current location to see if the actor is offroad
        drive_waypoint = self._map.get_waypoint(
            current_location,
            project_to_road=False
        )
        park_waypoint = self._map.get_waypoint(
            current_location,
            project_to_road=False,
            lane_type=carla.LaneType.Parking
        )
        if drive_waypoint or park_waypoint:
            self._offroad = False
        else:
            self._offroad = True

        # Counts the time offroad
        if self._offroad:
            if self._prev_time is None:
                self._prev_time = GameTime.get_time()
            else:
                curr_time = GameTime.get_time()
                self._time_offroad += curr_time - self._prev_time
                self._prev_time = curr_time
        else:
            self._prev_time = None

        if self._time_offroad > self._duration:
            self.test_status = "FAILURE"

        if self._terminate_on_failure and self.test_status == "FAILURE":
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class EndofRoadTest(Criterion):

    """
    Atomic containing a test to detect when an actor has changed to a different road

    Args:
        actor (carla.Actor): CARLA actor to be used for this test
        duration (float): Time spent after ending the road before the atomic fails.
            If terminate_on_failure isn't active, this is ignored.
        optional (bool): If True, the result is not considered for an overall pass/fail result
            when using the output argument
        terminate_on_failure (bool): If True, the atomic will fail when the duration condition has been met.
    """

    def __init__(self, actor, duration=0, optional=False, terminate_on_failure=False, name="EndofRoadTest"):
        """
        Setup of the variables
        """
        super(EndofRoadTest, self).__init__(name, actor, optional, terminate_on_failure)

        self._map = CarlaDataProvider.get_map()
        self._end_of_road = False

        self._duration = duration
        self._start_time = None
        self._time_end_road = 0
        self._road_id = None

    def update(self):
        """
        First, transforms the actor's current position to its corresponding waypoint. Then the road id
        is compared with the initial one and if that's the case, a time is started

        returns:
            py_trees.common.Status.FAILURE: when the actor has spent a given duration outside driving lanes
            py_trees.common.Status.RUNNING: the rest of the time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = CarlaDataProvider.get_location(self.actor)
        current_waypoint = self._map.get_waypoint(current_location)

        # Get the current road id
        if self._road_id is None:
            self._road_id = current_waypoint.road_id

        else:
            # Wait until the actor has left the road
            if self._road_id != current_waypoint.road_id or self._start_time:

                # Start counting
                if self._start_time is None:
                    self._start_time = GameTime.get_time()
                    return new_status

                curr_time = GameTime.get_time()
                self._time_end_road = curr_time - self._start_time

                if self._time_end_road > self._duration:
                    self.test_status = "FAILURE"
                    self.actual_value += 1
                    return py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


class OnSidewalkTest(Criterion):

    """
    Atomic containing a test to detect sidewalk invasions of a specific actor. This atomic can
    fail when actor has spent a specific time outside driving lanes (defined by OpenDRIVE).

    Args:
        actor (carla.Actor): CARLA actor to be used for this test
        duration (float): Time spent at sidewalks before the atomic fails.
            If terminate_on_failure isn't active, this is ignored.
        optional (bool): If True, the result is not considered for an overall pass/fail result
            when using the output argument
        terminate_on_failure (bool): If True, the atomic will fail when the duration condition has been met.
    """

    def __init__(self, actor, duration=0, optional=False, terminate_on_failure=False, name="OnSidewalkTest"):
        """
        Construction with sensor setup
        """
        super(OnSidewalkTest, self).__init__(name, actor, optional, terminate_on_failure)

        self._map = CarlaDataProvider.get_map()
        self._onsidewalk_active = False
        self._outside_lane_active = False

        self._actor_location = self.actor.get_location()
        self._wrong_sidewalk_distance = 0
        self._wrong_outside_lane_distance = 0
        self._sidewalk_start_location = None
        self._outside_lane_start_location = None
        self._duration = duration
        self._prev_time = None
        self._time_outside_lanes = 0

    def update(self):
        """
        First, transforms the actor's current position as well as its four corners to their
        corresponding waypoints. Depending on their lane type, the actor will be considered to be
        outside (or inside) driving lanes.

        returns:
            py_trees.common.Status.FAILURE: when the actor has spent a given duration outside
                driving lanes and terminate_on_failure is active
            py_trees.common.Status.RUNNING: the rest of the time
        """
        new_status = py_trees.common.Status.RUNNING

        if self._terminate_on_failure and self.test_status == "FAILURE":
            new_status = py_trees.common.Status.FAILURE

        # Some of the vehicle parameters
        current_tra = CarlaDataProvider.get_transform(self.actor)
        current_loc = current_tra.location
        current_wp = self._map.get_waypoint(current_loc, lane_type=carla.LaneType.Any)

        # Case 1) Car center is at a sidewalk
        if current_wp.lane_type == carla.LaneType.Sidewalk:
            if not self._onsidewalk_active:
                self._onsidewalk_active = True
                self._sidewalk_start_location = current_loc

        # Case 2) Not inside allowed zones (Driving and Parking)
        elif current_wp.lane_type not in (carla.LaneType.Driving, carla.LaneType.Parking):

            # Get the vertices of the vehicle
            heading_vec = current_tra.get_forward_vector()
            heading_vec.z = 0
            heading_vec = heading_vec / math.sqrt(math.pow(heading_vec.x, 2) + math.pow(heading_vec.y, 2))
            perpendicular_vec = carla.Vector3D(-heading_vec.y, heading_vec.x, 0)

            extent = self.actor.bounding_box.extent
            x_boundary_vector = heading_vec * extent.x
            y_boundary_vector = perpendicular_vec * extent.y

            bbox = [
                current_loc + carla.Location(x_boundary_vector - y_boundary_vector),
                current_loc + carla.Location(x_boundary_vector + y_boundary_vector),
                current_loc + carla.Location(-1 * x_boundary_vector - y_boundary_vector),
                current_loc + carla.Location(-1 * x_boundary_vector + y_boundary_vector)]

            bbox_wp = [
                self._map.get_waypoint(bbox[0], lane_type=carla.LaneType.Any),
                self._map.get_waypoint(bbox[1], lane_type=carla.LaneType.Any),
                self._map.get_waypoint(bbox[2], lane_type=carla.LaneType.Any),
                self._map.get_waypoint(bbox[3], lane_type=carla.LaneType.Any)]

            lane_type_list = [bbox_wp[0].lane_type, bbox_wp[1].lane_type, bbox_wp[2].lane_type, bbox_wp[3].lane_type]

            # Case 2.1) Not quite outside yet
            if bbox_wp[0].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking) \
                or bbox_wp[1].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking) \
                or bbox_wp[2].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking) \
                    or bbox_wp[3].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking):

                self._onsidewalk_active = False
                self._outside_lane_active = False

            # Case 2.2) At the mini Shoulders between Driving and Sidewalk
            elif carla.LaneType.Sidewalk in lane_type_list:
                if not self._onsidewalk_active:
                    self._onsidewalk_active = True
                    self._sidewalk_start_location = current_loc

            else:
                distance_vehicle_wp = current_loc.distance(current_wp.transform.location)

                # Case 2.3) Outside lane
                if distance_vehicle_wp >= current_wp.lane_width / 2:

                    if not self._outside_lane_active:
                        self._outside_lane_active = True
                        self._outside_lane_start_location = current_loc

                # Case 2.4) Very very edge case (but still inside driving lanes)
                else:
                    self._onsidewalk_active = False
                    self._outside_lane_active = False

        # Case 3) Driving and Parking conditions
        else:
            # Check for false positives at junctions
            if current_wp.is_junction:
                distance_vehicle_wp = math.sqrt(
                    math.pow(current_wp.transform.location.x - current_loc.x, 2) +
                    math.pow(current_wp.transform.location.y - current_loc.y, 2))

                if distance_vehicle_wp <= current_wp.lane_width / 2:
                    self._onsidewalk_active = False
                    self._outside_lane_active = False
                # Else, do nothing, the waypoint is too far to consider it a correct position
            else:

                self._onsidewalk_active = False
                self._outside_lane_active = False

        # Counts the time offroad
        if self._onsidewalk_active or self._outside_lane_active:
            if self._prev_time is None:
                self._prev_time = GameTime.get_time()
            else:
                curr_time = GameTime.get_time()
                self._time_outside_lanes += curr_time - self._prev_time
                self._prev_time = curr_time
        else:
            self._prev_time = None

        if self._time_outside_lanes > self._duration:
            self.test_status = "FAILURE"

        # Update the distances
        distance_vector = CarlaDataProvider.get_location(self.actor) - self._actor_location
        distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))

        if distance >= 0.02:  # Used to avoid micro-changes adding to considerable sums
            self._actor_location = CarlaDataProvider.get_location(self.actor)

            if self._onsidewalk_active:
                self._wrong_sidewalk_distance += distance
            elif self._outside_lane_active:
                # Only add if car is outside the lane but ISN'T in a junction
                self._wrong_outside_lane_distance += distance

        # Register the sidewalk event
        if not self._onsidewalk_active and self._wrong_sidewalk_distance > 0:

            self.actual_value += 1

            onsidewalk_event = TrafficEvent(event_type=TrafficEventType.ON_SIDEWALK_INFRACTION, frame=GameTime.get_frame())
            self._set_event_message(
                onsidewalk_event, self._sidewalk_start_location, self._wrong_sidewalk_distance)
            self._set_event_dict(
                onsidewalk_event, self._sidewalk_start_location, self._wrong_sidewalk_distance)

            self._onsidewalk_active = False
            self._wrong_sidewalk_distance = 0
            self.events.append(onsidewalk_event)

        # Register the outside of a lane event
        if not self._outside_lane_active and self._wrong_outside_lane_distance > 0:

            self.actual_value += 1

            outsidelane_event = TrafficEvent(event_type=TrafficEventType.OUTSIDE_LANE_INFRACTION, frame=GameTime.get_frame())
            self._set_event_message(
                outsidelane_event, self._outside_lane_start_location, self._wrong_outside_lane_distance)
            self._set_event_dict(
                outsidelane_event, self._outside_lane_start_location, self._wrong_outside_lane_distance)

            self._outside_lane_active = False
            self._wrong_outside_lane_distance = 0
            self.events.append(outsidelane_event)

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        If there is currently an event running, it is registered
        """
        # If currently at a sidewalk, register the event
        if self._onsidewalk_active:

            self.actual_value += 1

            onsidewalk_event = TrafficEvent(event_type=TrafficEventType.ON_SIDEWALK_INFRACTION, frame=GameTime.get_frame())
            self._set_event_message(
                onsidewalk_event, self._sidewalk_start_location, self._wrong_sidewalk_distance)
            self._set_event_dict(
                onsidewalk_event, self._sidewalk_start_location, self._wrong_sidewalk_distance)

            self._onsidewalk_active = False
            self._wrong_sidewalk_distance = 0
            self.events.append(onsidewalk_event)

        # If currently outside of our lane, register the event
        if self._outside_lane_active:

            self.actual_value += 1

            outsidelane_event = TrafficEvent(event_type=TrafficEventType.OUTSIDE_LANE_INFRACTION, frame=GameTime.get_frame())
            self._set_event_message(
                outsidelane_event, self._outside_lane_start_location, self._wrong_outside_lane_distance)
            self._set_event_dict(
                outsidelane_event, self._outside_lane_start_location, self._wrong_outside_lane_distance)

            self._outside_lane_active = False
            self._wrong_outside_lane_distance = 0
            self.events.append(outsidelane_event)

        super(OnSidewalkTest, self).terminate(new_status)

    def _set_event_message(self, event, location, distance):
        """
        Sets the message of the event
        """
        if event.get_type() == TrafficEventType.ON_SIDEWALK_INFRACTION:
            message_start = 'Agent invaded the sidewalk'
        else:
            message_start = 'Agent went outside the lane'

        event.set_message(
            '{} for about {} meters, starting at (x={}, y={}, z={})'.format(
                message_start,
                round(distance, 3),
                round(location.x, 3),
                round(location.y, 3),
                round(location.z, 3)))

    def _set_event_dict(self, event, location, distance):
        """
        Sets the dictionary of the event
        """
        event.set_dict({'location': location, 'distance': distance})


class OutsideRouteLanesTest(Criterion):

    """
    Atomic to detect if the vehicle is either on a sidewalk or at a wrong lane. The distance spent outside
    is computed and it is returned as a percentage of the route distance traveled.

    Args:
        actor (carla.ACtor): CARLA actor to be used for this test
        route (list [carla.Location, connection]): series of locations representing the route waypoints
        optional (bool): If True, the result is not considered for an overall pass/fail result
    """

    ALLOWED_OUT_DISTANCE = 0.5  # At least 0.5, due to the mini-shoulder between lanes and sidewalks
    MAX_VEHICLE_ANGLE = 120.0  # Maximum angle between the yaw and waypoint lane
    MAX_WAYPOINT_ANGLE = 150.0  # Maximum change between the yaw-lane angle between frames
    WINDOWS_SIZE = 3  # Amount of additional waypoints checked (in case the first on fails)

    def __init__(self, actor, route, optional=False, name="OutsideRouteLanesTest"):
        """
        Constructor
        """
        super(OutsideRouteLanesTest, self).__init__(name, actor, optional)
        self.units = "%"

        self._route = route
        self._current_index = 0
        self._route_length = len(self._route)
        self._route_transforms, _ = zip(*self._route)

        self._map = CarlaDataProvider.get_map()
        self._last_ego_waypoint = self._map.get_waypoint(self.actor.get_location())

        self._outside_lane_active = False
        self._wrong_lane_active = False
        self._last_road_id = None
        self._last_lane_id = None
        self._total_distance = 0
        self._wrong_distance = 0
        self._wrong_direction_active = True

        self._traffic_event = None

    def update(self):
        """
        Transforms the actor location and its four corners to waypoints. Depending on its types,
        the actor will be considered to be at driving lanes, sidewalk or offroad.

        returns:
            py_trees.common.Status.FAILURE: when the actor has left driving and terminate_on_failure is active
            py_trees.common.Status.RUNNING: the rest of the time
        """
        new_status = py_trees.common.Status.RUNNING

        # Some of the vehicle parameters
        location = CarlaDataProvider.get_location(self.actor)
        if location is None:
            return new_status

        # Deactivate / activate checking by blackboard message
        active = py_trees.blackboard.Blackboard().get('AC_SwitchWrongDirectionTest')
        if active is not None:
            self._wrong_direction_active = active
            py_trees.blackboard.Blackboard().set("AC_SwitchWrongDirectionTest", None, overwrite=True)

        self._is_outside_driving_lanes(location)
        self._is_at_wrong_lane(location)

        if self._outside_lane_active or (self._wrong_direction_active and self._wrong_lane_active):
            self.test_status = "FAILURE"

        # Get the traveled distance
        for index in range(self._current_index + 1,
                           min(self._current_index + self.WINDOWS_SIZE + 1, self._route_length)):
            # Get the dot product to know if it has passed this location
            route_transform = self._route_transforms[index]
            route_location = route_transform.location

            wp_dir = route_transform.get_forward_vector()  # Waypoint's forward vector
            wp_veh = location - route_location  # vector waypoint - vehicle

            if wp_veh.dot(wp_dir) > 0:
                # Get the distance traveled and add it to the total distance
                prev_route_location = self._route_transforms[self._current_index].location
                new_dist = prev_route_location.distance(route_location)
                self._total_distance += new_dist

                # And to the wrong one if outside route lanes
                if self._outside_lane_active or (self._wrong_direction_active and self._wrong_lane_active):
                    self._wrong_distance += new_dist

                if self._wrong_distance:
                    self._set_traffic_event()

                self._current_index = index

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

    def _set_traffic_event(self):
        """
        Creates the traffic event / updates it
        """
        if self._traffic_event is None:
            self._traffic_event = TrafficEvent(event_type=TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION, frame=GameTime.get_frame())
            self.events.append(self._traffic_event)

        percentage = self._wrong_distance / self._total_distance * 100
        self.actual_value = round(percentage, 2)

        self._traffic_event.set_message(
            "Agent went outside its route lanes for about {} meters "
            "({}% of the completed route)".format(
                round(self._wrong_distance, 3),
                round(percentage, 2)))

        self._traffic_event.set_dict({
            'distance': self._wrong_distance,
            'percentage': percentage
        })

        self._traffic_event.set_frame(GameTime.get_frame())

    def _is_outside_driving_lanes(self, location):
        """
        Detects if the ego_vehicle is outside driving lanes
        """
        driving_wp = self._map.get_waypoint(location, lane_type=carla.LaneType.Driving)
        parking_wp = self._map.get_waypoint(location, lane_type=carla.LaneType.Parking)

        driving_distance = location.distance(driving_wp.transform.location)
        if parking_wp is not None:  # Some towns have no parking
            parking_distance = location.distance(parking_wp.transform.location)
        else:
            parking_distance = float('inf')

        if driving_distance >= parking_distance:
            distance = parking_distance
            lane_width = parking_wp.lane_width
        else:
            distance = driving_distance
            lane_width = driving_wp.lane_width

        self._outside_lane_active = bool(distance > (lane_width / 2 + self.ALLOWED_OUT_DISTANCE))

    def _is_at_wrong_lane(self, location):
        """
        Detects if the ego_vehicle has invaded a wrong lane
        """
        waypoint = self._map.get_waypoint(location, lane_type=carla.LaneType.Driving)
        lane_id = waypoint.lane_id
        road_id = waypoint.road_id

        # Lanes and roads are too chaotic at junctions
        if waypoint.is_junction:
            self._wrong_lane_active = False
        elif self._last_road_id != road_id or self._last_lane_id != lane_id:

            if self._last_ego_waypoint.is_junction:
                # Just exited a junction, check the wp direction vs the ego's one
                wp_yaw = waypoint.transform.rotation.yaw % 360
                actor_yaw = self.actor.get_transform().rotation.yaw % 360
                angle = (wp_yaw - actor_yaw) % 360

                if angle < self.MAX_VEHICLE_ANGLE or angle > (360 - self.MAX_VEHICLE_ANGLE):
                    self._wrong_lane_active = False
                else:
                    self._wrong_lane_active = True

            else:
                # Route direction can be considered continuous, check for a big gap.
                last_wp_yaw = self._last_ego_waypoint.transform.rotation.yaw % 360
                wp_yaw = waypoint.transform.rotation.yaw % 360
                angle = (last_wp_yaw - wp_yaw) % 360

                if angle > self.MAX_WAYPOINT_ANGLE and angle < (360 - self.MAX_WAYPOINT_ANGLE):

                    # Is the ego vehicle going back to the lane, or going out? Take the opposite
                    self._wrong_lane_active = not bool(self._wrong_lane_active)

        # Remember the last state
        self._last_lane_id = lane_id
        self._last_road_id = road_id
        self._last_ego_waypoint = waypoint


class WrongLaneTest(Criterion):

    """
    This class contains an atomic test to detect invasions to wrong direction lanes.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """
    MAX_ALLOWED_ANGLE = 120.0
    MAX_WAYPOINT_ANGLE = 150.0

    def __init__(self, actor, optional=False, name="WrongLaneTest"):
        """
        Construction with sensor setup
        """
        super(WrongLaneTest, self).__init__(name, actor, optional)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        self._map = CarlaDataProvider.get_map()
        self._last_lane_id = None
        self._last_road_id = None

        self._in_lane = True
        self._wrong_distance = 0
        self._actor_location = self.actor.get_location()
        self._previous_lane_waypoint = self._map.get_waypoint(self.actor.get_location())
        self._wrong_lane_start_location = None

    def update(self):
        """
        Check lane invasion count
        """

        new_status = py_trees.common.Status.RUNNING

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        lane_waypoint = self._map.get_waypoint(self.actor.get_location())
        current_lane_id = lane_waypoint.lane_id
        current_road_id = lane_waypoint.road_id

        if (self._last_road_id != current_road_id or self._last_lane_id != current_lane_id) \
                and not lane_waypoint.is_junction:
            next_waypoint = lane_waypoint.next(2.0)[0]

            if not next_waypoint:
                return new_status

            # The waypoint route direction can be considered continuous.
            # Therefore just check for a big gap in waypoint directions.
            previous_lane_direction = self._previous_lane_waypoint.transform.get_forward_vector()
            current_lane_direction = lane_waypoint.transform.get_forward_vector()

            p_lane_vector = np.array([previous_lane_direction.x, previous_lane_direction.y])
            c_lane_vector = np.array([current_lane_direction.x, current_lane_direction.y])

            waypoint_angle = math.degrees(
                math.acos(np.clip(np.dot(p_lane_vector, c_lane_vector) /
                                  (np.linalg.norm(p_lane_vector) * np.linalg.norm(c_lane_vector)), -1.0, 1.0)))

            if waypoint_angle > self.MAX_WAYPOINT_ANGLE and self._in_lane:

                self.test_status = "FAILURE"
                self._in_lane = False
                self.actual_value += 1
                self._wrong_lane_start_location = self._actor_location

            else:
                # Reset variables
                self._in_lane = True

            # Continuity is broken after a junction so check vehicle-lane angle instead
            if self._previous_lane_waypoint.is_junction:

                vector_wp = np.array([next_waypoint.transform.location.x - lane_waypoint.transform.location.x,
                                      next_waypoint.transform.location.y - lane_waypoint.transform.location.y])

                vector_actor = np.array([math.cos(math.radians(self.actor.get_transform().rotation.yaw)),
                                         math.sin(math.radians(self.actor.get_transform().rotation.yaw))])

                vehicle_lane_angle = math.degrees(
                    math.acos(np.clip(np.dot(vector_actor, vector_wp) / (np.linalg.norm(vector_wp)), -1.0, 1.0)))

                if vehicle_lane_angle > self.MAX_ALLOWED_ANGLE:

                    self.test_status = "FAILURE"
                    self._in_lane = False
                    self.actual_value += 1
                    self._wrong_lane_start_location = self.actor.get_location()

        # Keep adding "meters" to the counter
        distance_vector = self.actor.get_location() - self._actor_location
        distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))

        if distance >= 0.02:  # Used to avoid micro-changes adding add to considerable sums
            self._actor_location = CarlaDataProvider.get_location(self.actor)

            if not self._in_lane and not lane_waypoint.is_junction:
                self._wrong_distance += distance

        # Register the event
        if self._in_lane and self._wrong_distance > 0:

            wrong_way_event = TrafficEvent(event_type=TrafficEventType.WRONG_WAY_INFRACTION, frame=GameTime.get_frame())
            self._set_event_message(wrong_way_event, self._wrong_lane_start_location,
                                    self._wrong_distance, current_road_id, current_lane_id)
            self._set_event_dict(wrong_way_event, self._wrong_lane_start_location,
                                 self._wrong_distance, current_road_id, current_lane_id)

            self.events.append(wrong_way_event)
            self._wrong_distance = 0

        # Remember the last state
        self._last_lane_id = current_lane_id
        self._last_road_id = current_road_id
        self._previous_lane_waypoint = lane_waypoint

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        If there is currently an event running, it is registered
        """
        if not self._in_lane:

            lane_waypoint = self._map.get_waypoint(self.actor.get_location())
            current_lane_id = lane_waypoint.lane_id
            current_road_id = lane_waypoint.road_id

            wrong_way_event = TrafficEvent(event_type=TrafficEventType.WRONG_WAY_INFRACTION, frame=GameTime.get_frame())
            self._set_event_message(wrong_way_event, self._wrong_lane_start_location,
                                    self._wrong_distance, current_road_id, current_lane_id)
            self._set_event_dict(wrong_way_event, self._wrong_lane_start_location,
                                 self._wrong_distance, current_road_id, current_lane_id)

            self._wrong_distance = 0
            self._in_lane = True
            self.events.append(wrong_way_event)

        super(WrongLaneTest, self).terminate(new_status)

    def _set_event_message(self, event, location, distance, road_id, lane_id):
        """
        Sets the message of the event
        """

        event.set_message(
            "Agent invaded a lane in opposite direction for {} meters, starting at (x={}, y={}, z={}). "
            "road_id={}, lane_id={}".format(
                round(distance, 3),
                round(location.x, 3),
                round(location.y, 3),
                round(location.z, 3),
                road_id,
                lane_id))

    def _set_event_dict(self, event, location, distance, road_id, lane_id):
        """
        Sets the dictionary of the event
        """
        event.set_dict({
            'location': location,
            'distance': distance,
            'road_id': road_id,
            'lane_id': lane_id})


class InRadiusRegionTest(Criterion):

    """
    The test is a success if the actor is within a given radius of a specified region

    Important parameters:
    - actor: CARLA actor to be used for this test
    - x, y, radius: Position (x,y) and radius (in meters) used to get the checked region
    """

    def __init__(self, actor, x, y, radius, name="InRadiusRegionTest"):
        """
        """
        super(InRadiusRegionTest, self).__init__(name, actor)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._x = x     # pylint: disable=invalid-name
        self._y = y     # pylint: disable=invalid-name
        self._radius = radius

    def update(self):
        """
        Check if the actor location is within trigger region
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self.actor)
        if location is None:
            return new_status

        if self.test_status != "SUCCESS":
            in_radius = math.sqrt(((location.x - self._x)**2) + ((location.y - self._y)**2)) < self._radius
            if in_radius:
                route_completion_event = TrafficEvent(event_type=TrafficEventType.ROUTE_COMPLETED, frame=GameTime.get_frame())
                route_completion_event.set_message("Destination was successfully reached")
                self.events.append(route_completion_event)
                self.test_status = "SUCCESS"
            else:
                self.test_status = "RUNNING"

        if self.test_status == "SUCCESS":
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class InRouteTest(Criterion):

    """
    The test is a success if the actor is never outside route. The actor can go outside of the route
    but only for a certain amount of distance

    Important parameters:
    - actor: CARLA actor to be used for this test
    - route: Route to be checked
    - offroad_max: Maximum distance (in meters) the actor can deviate from the route
    - offroad_min: Maximum safe distance (in meters). Might eventually cause failure
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """
    MAX_ROUTE_PERCENTAGE = 30  # %
    WINDOWS_SIZE = 5  # Amount of additional waypoints checked

    def __init__(self, actor, route, offroad_min=None, offroad_max=30, name="InRouteTest", terminate_on_failure=False):
        """
        """
        super(InRouteTest, self).__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self.units = None  # We care about whether or not it fails, no units attached

        self._route = route
        self._offroad_max = offroad_max
        # Unless specified, halve of the max value
        if offroad_min is None:
            self._offroad_min = self._offroad_max / 2
        else:
            self._offroad_min = self._offroad_min

        self._world = CarlaDataProvider.get_world()
        self._route_transforms, _ = zip(*self._route)
        self._route_length = len(self._route)
        self._current_index = 0
        self._out_route_distance = 0
        self._in_safe_route = True

        self._accum_meters = []
        prev_loc = self._route_transforms[0].location
        for i, tran in enumerate(self._route_transforms):
            loc = tran.location
            d = loc.distance(prev_loc)
            accum = 0 if i == 0 else self._accum_meters[i - 1]

            self._accum_meters.append(d + accum)
            prev_loc = loc

        # Blackboard variable
        blackv = py_trees.blackboard.Blackboard()
        _ = blackv.set("InRoute", True)

    def update(self):
        """
        Check if the actor location is within trigger region
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self.actor)
        if location is None:
            return new_status

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        elif self.test_status in ('RUNNING', 'INIT'):

            off_route = True

            shortest_distance = float('inf')
            closest_index = -1

            # Get the closest distance
            for index in range(self._current_index,
                               min(self._current_index + self.WINDOWS_SIZE + 1, self._route_length)):
                ref_location = self._route_transforms[index].location
                distance = math.sqrt(((location.x - ref_location.x) ** 2) + ((location.y - ref_location.y) ** 2))
                if distance <= shortest_distance:
                    closest_index = index
                    shortest_distance = distance

            if closest_index == -1 or shortest_distance == float('inf'):
                return new_status

            # Check if the actor is out of route
            if shortest_distance < self._offroad_max:
                off_route = False
                self._in_safe_route = bool(shortest_distance < self._offroad_min)

            # If actor advanced a step, record the distance
            if self._current_index != closest_index:

                new_dist = self._accum_meters[closest_index] - self._accum_meters[self._current_index]

                # If too far from the route, add it and check if its value
                if not self._in_safe_route:
                    self._out_route_distance += new_dist
                    out_route_percentage = 100 * self._out_route_distance / self._accum_meters[-1]
                    if out_route_percentage > self.MAX_ROUTE_PERCENTAGE:
                        off_route = True

                self._current_index = closest_index

            if off_route:
                # Blackboard variable
                blackv = py_trees.blackboard.Blackboard()
                _ = blackv.set("InRoute", False)

                route_deviation_event = TrafficEvent(event_type=TrafficEventType.ROUTE_DEVIATION, frame=GameTime.get_frame())
                route_deviation_event.set_message(
                    "Agent deviated from the route at (x={}, y={}, z={})".format(
                        round(location.x, 3),
                        round(location.y, 3),
                        round(location.z, 3)))
                route_deviation_event.set_dict({'location': location})

                self.events.append(route_deviation_event)

                self.test_status = "FAILURE"
                self.actual_value += 1
                new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class RouteCompletionTest(Criterion):

    """
    Check at which stage of the route is the actor at each tick

    Important parameters:
    - actor: CARLA actor to be used for this test
    - route: Route to be checked
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """
    WINDOWS_SIZE = 2

    # Thresholds to return that a route has been completed
    DISTANCE_THRESHOLD = 10.0  # meters
    PERCENTAGE_THRESHOLD = 99  # %

    def __init__(self, actor, route, name="RouteCompletionTest", terminate_on_failure=False):
        """
        """
        super(RouteCompletionTest, self).__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self.units = "%"
        self.success_value = 100
        self._route = route
        self._map = CarlaDataProvider.get_map()

        self._index = 0
        self._route_length = len(self._route)
        self._route_transforms, _ = zip(*self._route)
        self._route_accum_perc = self._get_acummulated_percentages()

        self.target_location = self._route_transforms[-1].location

        self._traffic_event = TrafficEvent(event_type=TrafficEventType.ROUTE_COMPLETION, frame=0)
        self._traffic_event.set_dict({'route_completed': self.actual_value})
        self._traffic_event.set_message("Agent has completed {} of the route".format(self.actual_value))
        self.events.append(self._traffic_event)

    def _get_acummulated_percentages(self):
        """Gets the accumulated percentage of each of the route transforms"""
        accum_meters = []
        prev_loc = self._route_transforms[0].location
        for i, tran in enumerate(self._route_transforms):
            d = tran.location.distance(prev_loc)
            new_d = 0 if i == 0 else accum_meters[i - 1]

            accum_meters.append(d + new_d)
            prev_loc = tran.location

        max_dist = accum_meters[-1]
        return [x / max_dist * 100 for x in accum_meters]

    def update(self):
        """
        Check if the actor location is within trigger region
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self.actor)
        if location is None:
            return new_status

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        elif self.test_status in ('RUNNING', 'INIT'):

            for index in range(self._index, min(self._index + self.WINDOWS_SIZE + 1, self._route_length)):
                # Get the dot product to know if it has passed this location
                route_transform = self._route_transforms[index]
                route_location = route_transform.location
                wp_dir = route_transform.get_forward_vector()          # Waypoint's forward vector
                wp_veh = location - route_location                     # vector route - vehicle

                if wp_veh.dot(wp_dir) > 0:
                    self._index = index
                    self.actual_value = self._route_accum_perc[self._index]

            self.actual_value = round(self.actual_value, 2)
            self._traffic_event.set_dict({'route_completed': self.actual_value})
            self._traffic_event.set_message("Agent has completed {} of the route".format(self.actual_value))

            if self.actual_value > self.PERCENTAGE_THRESHOLD \
                    and location.distance(self.target_location) < self.DISTANCE_THRESHOLD:
                self.test_status = "SUCCESS"
                self.actual_value = 100

        elif self.test_status == "SUCCESS":
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        Set test status to failure if not successful and terminate
        """
        self.actual_value = round(self.actual_value, 2)

        self._traffic_event.set_dict({'route_completed': self.actual_value})
        self._traffic_event.set_message("Agent has completed {} of the route".format(self.actual_value))

        if self.test_status == "INIT":
            self.test_status = "FAILURE"
        super(RouteCompletionTest, self).terminate(new_status)


class RunningRedLightTest(Criterion):

    """
    Check if an actor is running a red light

    Important parameters:
    - actor: CARLA actor to be used for this test
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """
    DISTANCE_LIGHT = 15  # m

    def __init__(self, actor, name="RunningRedLightTest", terminate_on_failure=False):
        """
        Init
        """
        super(RunningRedLightTest, self).__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self._world = CarlaDataProvider.get_world()
        self._map = CarlaDataProvider.get_map()
        self._list_traffic_lights = []
        self._last_red_light_id = None
        self.debug = False

        all_actors = CarlaDataProvider.get_all_actors()
        for _actor in all_actors:
            if 'traffic_light' in _actor.type_id:
                center, waypoints = self.get_traffic_light_waypoints(_actor)
                self._list_traffic_lights.append((_actor, center, waypoints))

    # pylint: disable=no-self-use
    def is_vehicle_crossing_line(self, seg1, seg2):
        """
        check if vehicle crosses a line segment
        """
        line1 = shapely.geometry.LineString([(seg1[0].x, seg1[0].y), (seg1[1].x, seg1[1].y)])
        line2 = shapely.geometry.LineString([(seg2[0].x, seg2[0].y), (seg2[1].x, seg2[1].y)])
        inter = line1.intersection(line2)

        return not inter.is_empty

    def update(self):
        """
        Check if the actor is running a red light
        """
        new_status = py_trees.common.Status.RUNNING

        transform = CarlaDataProvider.get_transform(self.actor)
        location = transform.location
        if location is None:
            return new_status

        veh_extent = self.actor.bounding_box.extent.x

        tail_close_pt = self.rotate_point(carla.Vector3D(-0.8 * veh_extent, 0, 0), transform.rotation.yaw)
        tail_close_pt = location + carla.Location(tail_close_pt)

        tail_far_pt = self.rotate_point(carla.Vector3D(-veh_extent - 1, 0, 0), transform.rotation.yaw)
        tail_far_pt = location + carla.Location(tail_far_pt)

        for traffic_light, center, waypoints in self._list_traffic_lights:

            if self.debug:
                z = 2.1
                if traffic_light.state == carla.TrafficLightState.Red:
                    color = carla.Color(155, 0, 0)
                elif traffic_light.state == carla.TrafficLightState.Green:
                    color = carla.Color(0, 155, 0)
                else:
                    color = carla.Color(155, 155, 0)
                self._world.debug.draw_point(center + carla.Location(z=z), size=0.2, color=color, life_time=0.01)
                for wp in waypoints:
                    text = "{}.{}".format(wp.road_id, wp.lane_id)
                    self._world.debug.draw_string(
                        wp.transform.location + carla.Location(x=1, z=z), text, color=color, life_time=0.01)
                    self._world.debug.draw_point(
                        wp.transform.location + carla.Location(z=z), size=0.1, color=color, life_time=0.01)

            center_loc = carla.Location(center)

            if self._last_red_light_id and self._last_red_light_id == traffic_light.id:
                continue
            if center_loc.distance(location) > self.DISTANCE_LIGHT:
                continue
            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            for wp in waypoints:

                tail_wp = self._map.get_waypoint(tail_far_pt)

                # Calculate the dot product (Might be unscaled, as only its sign is important)
                ve_dir = CarlaDataProvider.get_transform(self.actor).get_forward_vector()
                wp_dir = wp.transform.get_forward_vector()

                # Check the lane until all the "tail" has passed
                if tail_wp.road_id == wp.road_id and tail_wp.lane_id == wp.lane_id and ve_dir.dot(wp_dir) > 0:
                    # This light is red and is affecting our lane
                    yaw_wp = wp.transform.rotation.yaw
                    lane_width = wp.lane_width
                    location_wp = wp.transform.location

                    lft_lane_wp = self.rotate_point(carla.Vector3D(0.6 * lane_width, 0, 0), yaw_wp + 90)
                    lft_lane_wp = location_wp + carla.Location(lft_lane_wp)
                    rgt_lane_wp = self.rotate_point(carla.Vector3D(0.6 * lane_width, 0, 0), yaw_wp - 90)
                    rgt_lane_wp = location_wp + carla.Location(rgt_lane_wp)

                    # Is the vehicle traversing the stop line?
                    if self.is_vehicle_crossing_line((tail_close_pt, tail_far_pt), (lft_lane_wp, rgt_lane_wp)):

                        self.test_status = "FAILURE"
                        self.actual_value += 1
                        location = traffic_light.get_transform().location
                        red_light_event = TrafficEvent(event_type=TrafficEventType.TRAFFIC_LIGHT_INFRACTION, frame=GameTime.get_frame())
                        red_light_event.set_message(
                            "Agent ran a red light {} at (x={}, y={}, z={})".format(
                                traffic_light.id,
                                round(location.x, 3),
                                round(location.y, 3),
                                round(location.z, 3)))
                        red_light_event.set_dict({'id': traffic_light.id, 'location': location})

                        self.events.append(red_light_event)
                        self._last_red_light_id = traffic_light.id
                        break

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def rotate_point(self, point, angle):
        """
        rotate a given point by a given angle
        """
        x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x + math.cos(math.radians(angle)) * point.y
        return carla.Vector3D(x_, y_, point.z)

    def get_traffic_light_waypoints(self, traffic_light):
        """
        get area of a given traffic light
        """
        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)

        # Discretize the trigger box into points
        area_ext = traffic_light.trigger_volume.extent
        x_values = np.arange(-0.9 * area_ext.x, 0.9 * area_ext.x, 1.0)  # 0.9 to avoid crossing to adjacent lanes

        area = []
        for x in x_values:
            point = self.rotate_point(carla.Vector3D(x, 0, area_ext.z), base_rot)
            point_location = area_loc + carla.Location(x=point.x, y=point.y)
            area.append(point_location)

        # Get the waypoints of these points, removing duplicates
        ini_wps = []
        for pt in area:
            wpx = self._map.get_waypoint(pt)
            # As x_values are arranged in order, only the last one has to be checked
            if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
                ini_wps.append(wpx)

        # Advance them until the intersection
        wps = []
        for wpx in ini_wps:
            while not wpx.is_intersection:
                next_wp = wpx.next(0.5)[0]
                if next_wp and not next_wp.is_intersection:
                    wpx = next_wp
                else:
                    break
            wps.append(wpx)

        return area_loc, wps


class RunningStopTest(Criterion):

    """
    Check if an actor is running a stop sign

    Important parameters:
    - actor: CARLA actor to be used for this test
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """
    PROXIMITY_THRESHOLD = 4.0  # Stops closer than this distance will be detected [m]
    SPEED_THRESHOLD = 0.1 # Minimum speed to consider the actor has stopped [m/s]
    WAYPOINT_STEP = 0.5  # m

    def __init__(self, actor, name="RunningStopTest", terminate_on_failure=False):
        """
        """
        super(RunningStopTest, self).__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self._world = CarlaDataProvider.get_world()
        self._map = CarlaDataProvider.get_map()
        self._list_stop_signs = []
        self._target_stop_sign = None
        self._stop_completed = False

        self._last_failed_stop = None

        for _actor in CarlaDataProvider.get_all_actors():
            if 'traffic.stop' in _actor.type_id:
                self._list_stop_signs.append(_actor)

    def point_inside_boundingbox(self, point, bb_center, bb_extent, multiplier=1.2):
        """Checks whether or not a point is inside a bounding box."""

        # pylint: disable=invalid-name
        A = carla.Vector2D(bb_center.x - multiplier * bb_extent.x, bb_center.y - multiplier * bb_extent.y)
        B = carla.Vector2D(bb_center.x + multiplier * bb_extent.x, bb_center.y - multiplier * bb_extent.y)
        D = carla.Vector2D(bb_center.x - multiplier * bb_extent.x, bb_center.y + multiplier * bb_extent.y)
        M = carla.Vector2D(point.x, point.y)

        AB = B - A
        AD = D - A
        AM = M - A
        am_ab = AM.x * AB.x + AM.y * AB.y
        ab_ab = AB.x * AB.x + AB.y * AB.y
        am_ad = AM.x * AD.x + AM.y * AD.y
        ad_ad = AD.x * AD.x + AD.y * AD.y

        return am_ab > 0 and am_ab < ab_ab and am_ad > 0 and am_ad < ad_ad  # pylint: disable=chained-comparison

    def is_actor_affected_by_stop(self, wp_list, stop):
        """
        Check if the given actor is affected by the stop.
        Without using waypoints, a stop might not be detected if the actor is moving at the lane edge.
        """
        # Quick distance test
        stop_location = stop.get_transform().transform(stop.trigger_volume.location)
        actor_location = wp_list[0].transform.location
        if stop_location.distance(actor_location) > self.PROXIMITY_THRESHOLD:
            return False

        # Check if the any of the actor wps is inside the stop's bounding box.
        # Using more than one waypoint removes issues with small trigger volumes and backwards movement
        stop_extent = stop.trigger_volume.extent
        for actor_wp in wp_list:
            if self.point_inside_boundingbox(actor_wp.transform.location, stop_location, stop_extent):
                return True

        return False

    def _scan_for_stop_sign(self, actor_transform, wp_list):
        """
        Check the stop signs to see if any of them affect the actor.
        Ignore all checks when going backwards or through an opposite direction"""

        actor_direction = actor_transform.get_forward_vector()

        # Ignore all when going backwards
        actor_velocity = self.actor.get_velocity()
        if actor_velocity.dot(actor_direction) < -0.17:  # 100º, just in case
            return None

        # Ignore all when going in the opposite direction
        lane_direction = wp_list[0].transform.get_forward_vector()
        if actor_direction.dot(lane_direction) < -0.17:  # 100º, just in case
            return None

        for stop in self._list_stop_signs:
            if self.is_actor_affected_by_stop(wp_list, stop):
                return stop

    def _get_waypoints(self, actor):
        """Returns a list of waypoints starting from the ego location and a set amount forward"""
        wp_list = []
        steps = int(self.PROXIMITY_THRESHOLD / self.WAYPOINT_STEP)

        # Add the actor location
        wp = self._map.get_waypoint(actor.get_location())
        wp_list.append(wp)

        # And its forward waypoints
        next_wp = wp
        for _ in range(steps):
            next_wps = next_wp.next(self.WAYPOINT_STEP)
            if not next_wps:
                break
            next_wp = next_wps[0]
            wp_list.append(next_wp)

        return wp_list

    def update(self):
        """
        Check if the actor is running a red light
        """
        new_status = py_trees.common.Status.RUNNING

        actor_transform = self.actor.get_transform()
        check_wps = self._get_waypoints(self.actor)

        if not self._target_stop_sign:
            self._target_stop_sign = self._scan_for_stop_sign(actor_transform, check_wps)
            return new_status

        if not self._stop_completed:
            current_speed = CarlaDataProvider.get_velocity(self.actor)
            if current_speed < self.SPEED_THRESHOLD:
                self._stop_completed = True

        if not self.is_actor_affected_by_stop(check_wps, self._target_stop_sign):
            if not self._stop_completed and self._last_failed_stop != self._target_stop_sign.id:
                # did we stop?
                self.actual_value += 1
                self.test_status = "FAILURE"
                stop_location = self._target_stop_sign.get_transform().location
                running_stop_event = TrafficEvent(event_type=TrafficEventType.STOP_INFRACTION, frame=GameTime.get_frame())
                running_stop_event.set_message(
                    "Agent ran a stop with id={} at (x={}, y={}, z={})".format(
                        self._target_stop_sign.id,
                        round(stop_location.x, 3),
                        round(stop_location.y, 3),
                        round(stop_location.z, 3)))
                running_stop_event.set_dict({'id': self._target_stop_sign.id, 'location': stop_location})

                self.events.append(running_stop_event)

                self._last_failed_stop = self._target_stop_sign.id

            # Reset state
            self._target_stop_sign = None
            self._stop_completed = False

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class MinimumSpeedRouteTest(Criterion):

    """
    Check at which stage of the route is the actor at each tick

    Important parameters:
    - actor: CARLA actor to be used for this test
    - route: Route to be checked
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """
    WINDOWS_SIZE = 2
    RATIO = 1

    def __init__(self, actor, route, checkpoints=1, name="MinimumSpeedRouteTest", terminate_on_failure=False):
        """
        """
        super().__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self.units = "%"
        self.success_value = 100
        self.actual_value = 100

        self._route = route
        self._accum_dist = []
        prev_trans = None
        for trans, _ in route:
            if prev_trans:
                dist = trans.location.distance(prev_trans.location)
                self._accum_dist.append(dist + self._accum_dist[-1])
            else:
                self._accum_dist.append(0)
            prev_trans = trans
        self._route_length = len(self._route)

        self._checkpoints = checkpoints
        self._checkpoint_dist = self._accum_dist[-1] / self._checkpoints

        self._mean_speed = 0
        self._actor_speed = 0
        self._speed_points = 0

        self._current_dist = 0
        self._checkpoint_values = []

        self._index = 0


    def update(self):
        """
        Check if the actor location is within trigger region
        """
        new_status = py_trees.common.Status.RUNNING

        if self._terminate_on_failure and (self.test_status == "FAILURE"):
            new_status = py_trees.common.Status.FAILURE

        # Check the actor progress through the route 
        location = CarlaDataProvider.get_location(self.actor)
        if location is None:
            return new_status

        for index in range(self._index, min(self._index + self.WINDOWS_SIZE + 1, self._route_length)):
            # Get the dot product to know if it has passed this location
            route_transform = self._route[index][0]
            route_location = route_transform.location
            wp_dir = route_transform.get_forward_vector()
            wp_veh = location - route_location

            if wp_veh.dot(wp_dir) > 0:
                self._index = index

        if self._accum_dist[self._index] - self._current_dist > self._checkpoint_dist:
            self._set_traffic_event()
            self._current_dist = self._accum_dist[self._index]
            self._mean_speed = 0
            self._actor_speed = 0
            self._speed_points = 0

        # Get the actor speed
        velocity = CarlaDataProvider.get_velocity(self.actor)
        if velocity is None:
            return new_status

        # Get the speed of the surrounding Background Activity
        all_vehicles = CarlaDataProvider.get_all_actors().filter('vehicle*')
        background_vehicles = [v for v in all_vehicles if v.attributes['role_name'] == 'background']

        if background_vehicles:
            frame_mean_speed = 0
            for vehicle in background_vehicles:
                frame_mean_speed += CarlaDataProvider.get_velocity(vehicle)
            frame_mean_speed /= len(background_vehicles)

            # Record the data
            self._mean_speed += frame_mean_speed
            self._actor_speed += velocity
            self._speed_points += 1

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def _set_traffic_event(self):

        if self._speed_points > 0 and self._mean_speed:
            self._mean_speed /= self._speed_points
            self._actor_speed /= self._speed_points
            checkpoint_value = round(self._actor_speed / (self.RATIO * self._mean_speed) * 100, 2)
        else:
            checkpoint_value = 100

        if checkpoint_value >= self.success_value:
            self.test_status = "SUCCESS"
        else:
            self.test_status = "FAILURE"

            self._traffic_event = TrafficEvent(TrafficEventType.MIN_SPEED_INFRACTION, GameTime.get_frame())
            self._traffic_event.set_dict({'percentage': checkpoint_value})
            self._traffic_event.set_message(f"Average speed is {checkpoint_value}% of the surrounding traffic's one")
            self.events.append(self._traffic_event)

        self._checkpoint_values.append(checkpoint_value)

    def terminate(self, new_status):
        """
        Set the actual value as a percentage of the two mean speeds,
        the test status to failure if not successful and terminate
        """
        # Routes end at around 99%, so make sure the last checkpoint is recorded
        if self._accum_dist[self._index] / self._accum_dist[-1] > 0.95:
            self._set_traffic_event()

        if len(self._checkpoint_values):
            self.actual_value = round(sum(self._checkpoint_values) / len(self._checkpoint_values), 2)
        super().terminate(new_status)


class YieldToEmergencyVehicleTest(Criterion):

    """
    Atomic Criterion to detect if the actor yields its lane to the emergency vehicle behind it.
    Detection is done by checking if the ev is in front of the actor

    Args:
        actor (carla.Actor): CARLA actor to be used for this test
        ev (carla.Actor): The emergency vehicle
        optional (bool): If True, the result is not considered for an overall pass/fail result
    """

    WAITING_TIME_THRESHOLD = 15 # Maximum time for actor to block ev

    def __init__(self, actor, ev, optional=False, name="YieldToEmergencyVehicleTest"):
        """
        Constructor
        """
        self._ev = ev
        self._terminated = False
        super().__init__(name, actor, optional)

    def update(self):
        """
        Monitor that the EV ends up in front of the actor

        returns:
            py_trees.common.Status.RUNNING
        """
        new_status = py_trees.common.Status.RUNNING

        actor_location = CarlaDataProvider.get_location(self.actor)
        if not actor_location:
            return new_status
        ev_location = CarlaDataProvider.get_location(self._ev)
        if not ev_location:
            return new_status

        ev_direction = CarlaDataProvider.get_transform(self._ev).get_forward_vector()
        actor_ev_vector = actor_location - ev_location

        if ev_direction.dot(actor_ev_vector) > 0:
            self.test_status = "FAILURE"
        else:
            self.test_status = "SUCCESS"

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

    def terminate(self, new_status):
        """Set the traffic event, if needed"""
        # Terminates are called multiple times. Do this only once
        if not self._terminated:
            if self.test_status == "FAILURE":
                traffic_event = TrafficEvent(TrafficEventType.YIELD_TO_EMERGENCY_VEHICLE, GameTime.get_frame())
                traffic_event.set_message("Agent failed to yield to an emergency vehicle")
                self.events.append(traffic_event)

            self._terminated = True

        super().terminate(new_status)


class ScenarioTimeoutTest(Criterion):

    """
    Atomic Criterion to detect if the actor has been incapable of finishing an scenario

    Args:
        actor (carla.Actor): CARLA actor to be used for this test
        optional (bool): If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, scenario_name, optional=False, name="ScenarioTimeoutTest"):
        """
        Constructor
        """
        super().__init__(name, actor, optional)
        self.success_value = 0
        self.actual_value = 0
        self._scenario_name = scenario_name

    def update(self):
        """wait"""
        new_status = py_trees.common.Status.RUNNING
        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

    def terminate(self, new_status):
        """check the blackboard for the data and update the criteria if one found"""

        blackboard_name = f"ScenarioTimeout_{self._scenario_name}"

        timeout = py_trees.blackboard.Blackboard().get(blackboard_name)
        if timeout:
            self.actual_value = 1
            self.test_status = "FAILURE"

            traffic_event = TrafficEvent(event_type=TrafficEventType.SCENARIO_TIMEOUT, frame=GameTime.get_frame())
            traffic_event.set_message("Agent timed out a scenario")
            self.events.append(traffic_event)
        py_trees.blackboard.Blackboard().set(blackboard_name, None, True)

        super().terminate(new_status)
