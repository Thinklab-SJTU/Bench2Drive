#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Several atomic behaviors to help with the communication with the background activity,
removing its interference with other scenarios
"""

import py_trees
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class ChangeRoadBehavior(AtomicBehavior):
    """
    Updates the blackboard to change the parameters of the road behavior.
    None values imply that these values won't be changed.

    Args:
        num_front_vehicles (int): Amount of vehicles in front of the ego. Can't be negative
        num_back_vehicles (int): Amount of vehicles behind it. Can't be negative
        switch_source (bool): (De)activatea the road sources.
    """

    def __init__(self, num_front_vehicles=None, num_back_vehicles=None, spawn_dist=None, extra_space=None, name="ChangeRoadBehavior"):
        self._num_front = num_front_vehicles
        self._num_back = num_back_vehicles
        self._spawn_dist = spawn_dist
        self._extra_space = extra_space
        super().__init__(name)

    def update(self):
        py_trees.blackboard.Blackboard().set(
            "BA_ChangeRoadBehavior", [self._num_front, self._num_back, self._spawn_dist, self._extra_space], overwrite=True
        )
        return py_trees.common.Status.SUCCESS


class ChangeOppositeBehavior(AtomicBehavior):
    """
    Updates the blackboard to change the parameters of the opposite road behavior.
    None values imply that these values won't be changed

    Args:
        source_dist (float): Distance between the opposite sources and the ego vehicle. Must be positive
        max_actors (int): Max amount of concurrent alive actors spawned by the same source. Can't be negative
    """

    def __init__(self, source_dist=None, spawn_dist=None, active=None, name="ChangeOppositeBehavior"):
        self._source_dist = source_dist
        self._spawn_dist = spawn_dist
        self._active = active
        super().__init__(name)

    def update(self):
        py_trees.blackboard.Blackboard().set(
            "BA_ChangeOppositeBehavior", [self._source_dist, self._spawn_dist, self._active], overwrite=True
        )
        return py_trees.common.Status.SUCCESS


class ChangeJunctionBehavior(AtomicBehavior):
    """
    Updates the blackboard to change the parameters of the junction behavior.
    None values imply that these values won't be changed

    Args:
        source_dist (float): Distance between the junctiob sources and the junction entry. Must be positive
        max_actors (int): Max amount of concurrent alive actors spawned by the same source. Can't be negative
    """

    def __init__(self, source_dist=None, spawn_dist=None, max_actors=None, source_perc=None, name="ChangeJunctionBehavior"):
        self._source_dist = source_dist
        self._spawn_dist = spawn_dist
        self._max_actors = max_actors
        self._perc = source_perc
        super().__init__(name)

    def update(self):
        py_trees.blackboard.Blackboard().set(
            "BA_ChangeJunctionBehavior", [self._source_dist, self._spawn_dist, self._max_actors, self._perc], overwrite=True
        )
        return py_trees.common.Status.SUCCESS


class SetMaxSpeed(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity that its behavior is restriced to a maximum speed
    """

    def __init__(self, max_speed, name="SetMaxSpeed"):
        self._max_speed = max_speed
        super().__init__(name)

    def update(self):
        py_trees.blackboard.Blackboard().set("BA_SetMaxSpeed", self._max_speed, overwrite=True)
        return py_trees.common.Status.SUCCESS


class StopFrontVehicles(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity that a HardBreak scenario has to be triggered.
    'stop_duration' is the amount of time, in seconds, the vehicles will be stopped
    """

    def __init__(self, name="StopFrontVehicles"):
        super().__init__(name)

    def update(self):
        py_trees.blackboard.Blackboard().set("BA_StopFrontVehicles", True, overwrite=True)
        return py_trees.common.Status.SUCCESS


class StartFrontVehicles(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity that a HardBreak scenario has to be triggered.
    'stop_duration' is the amount of time, in seconds, the vehicles will be stopped
    """

    def __init__(self, name="StartFrontVehicles"):
        super().__init__(name)

    def update(self):
        py_trees.blackboard.Blackboard().set("BA_StartFrontVehicles", True, overwrite=True)
        return py_trees.common.Status.SUCCESS


class StopBackVehicles(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity to stop the vehicles behind the ego as to
    not interfere with the scenarios. This only works at roads, not junctions.
    """
    def __init__(self, name="StopBackVehicles"):
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_StopBackVehicles", True, overwrite=True)
        return py_trees.common.Status.SUCCESS


class StartBackVehicles(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity to restart the vehicles behind the ego.
    """
    def __init__(self, name="StartBackVehicles"):
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_StartBackVehicles", True, overwrite=True)
        return py_trees.common.Status.SUCCESS


class LeaveSpaceInFront(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity that the ego needs more space in front.
    This only works at roads, not junctions.
    """
    def __init__(self, space, name="LeaveSpaceInFront"):
        self._space = space
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_LeaveSpaceInFront", [self._space], overwrite=True)
        return py_trees.common.Status.SUCCESS


class SwitchRouteSources(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity to (de)activate all route sources
    """
    def __init__(self, enabled=True, name="SwitchRouteSources"):
        self._enabled = enabled
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_SwitchRouteSources", self._enabled, overwrite=True)
        return py_trees.common.Status.SUCCESS


class RemoveRoadLane(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity to remove its actors from the given lane 
    and stop generating new ones on this lane, or recover from stopping.

    Args:
        lane_wp (carla.Waypoint): A carla.Waypoint
        active (bool)
    """
    def __init__(self, lane_wp, name="RemoveRoadLane"):
        self._lane_wp = lane_wp
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_RemoveRoadLane", self._lane_wp, overwrite=True)
        return py_trees.common.Status.SUCCESS


class ReAddRoadLane(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity to readd the ego road lane.

    Args:
        offset: 0 to readd the ego lane, 1 for the right side lane, -1 for the left...
        active (bool)
    """
    def __init__(self, offset, name="BA_ReAddRoadLane"):
        self._offset = offset
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_ReAddRoadLane", self._offset, overwrite=True)
        return py_trees.common.Status.SUCCESS


class LeaveSpaceInFront(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity that the ego needs more space in front.
    This only works at roads, not junctions.
    """
    def __init__(self, space, name="LeaveSpaceInFront"):
        self._space = space
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_LeaveSpaceInFront", self._space, overwrite=True)
        return py_trees.common.Status.SUCCESS


class LeaveCrossingSpace(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity that the ego needs more space in front.
    This only works at roads, not junctions.
    """
    def __init__(self, collision_wp, name="LeaveCrossingSpace"):
        self._collision_wp = collision_wp
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set("BA_LeaveCrossingSpace", self._collision_wp, overwrite=True)
        return py_trees.common.Status.SUCCESS

class HandleJunctionScenario(AtomicBehavior):
    """
    Updates the blackboard to tell the background activity to adapt to a junction scenario

    Args:
        clear_junction (bool): Remove all actors inside the junction, and all that enter it afterwards
        clear_ego_entry (bool): Remove all actors part of the ego road to ensure a smooth entry of the ego to the junction.
        remove_entries (list): list of waypoint representing a junction entry that needs to be removed
        remove_exits (list): list of waypoint representing a junction exit that needs to be removed
        stop_entries (bool): Stops all the junction entries
        extend_road_exit (float): Moves the road junction actors forward to leave more space for the scenario.
            It also deactivates the road sources.
        active (bool)
    """
    def __init__(self, clear_junction=True, clear_ego_entry=True, remove_entries=[],
                 remove_exits=[], stop_entries=True, extend_road_exit=0,
                 name="HandleJunctionScenario"):
        self._clear_junction = clear_junction
        self._clear_ego_entry = clear_ego_entry
        self._remove_entries = remove_entries
        self._remove_exits = remove_exits
        self._stop_entries = stop_entries
        self._extend_road_exit = extend_road_exit
        super().__init__(name)

    def update(self):
        """Updates the blackboard and succeds"""
        py_trees.blackboard.Blackboard().set(
            "BA_HandleJunctionScenario",
            [self._clear_junction, self._clear_ego_entry, self._remove_entries,
             self._remove_exits, self._stop_entries, self._extend_road_exit],
            overwrite=True)
        return py_trees.common.Status.SUCCESS
