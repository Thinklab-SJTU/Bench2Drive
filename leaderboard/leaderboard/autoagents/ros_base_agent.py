#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a base ROS autonomous agent interface to control the ego vehicle via a ROS1/ROS2 stack
"""

from __future__ import print_function

import logging
import logging.handlers
import os
import queue
import threading

import carla
import pexpect
import transforms3d

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

EPSILON = 0.001


class ROSLogger(object):

    def __init__(self, name):
        self.name = name
    
        self.logger = logging.getLogger(self.name)
        self.logger.setLevel(logging.INFO)

        logger_path = os.path.join("log", "ros")
        if not os.path.isdir(logger_path):
            os.makedirs(logger_path)

        self.handler = logging.handlers.RotatingFileHandler(os.path.join(logger_path, self.name + ".log"), maxBytes=25*1024*1024, backupCount=3)
        self.handler.setLevel(logging.INFO)
        self.handler.setFormatter(logging.Formatter("%(message)s"))

        self.logger.addHandler(self.handler)

    def write(self, data):
        self.logger.info(data.strip())

    def flush(self):
        pass

    def fileno(self):
        return self.handler.stream.fileno()
    
    def destroy(self):
        self.logger.removeHandler(self.handler)


class ROSLauncher(object):

    def __init__(self, app_name, ros_version, debug=False):
        self.app_name = app_name
        self.ros_version = ros_version
        # force always debugging
        self.debug = True

        self._process = None

        self._log_thread = threading.Thread(target=self.log)
        if self.debug:
            self._logger = ROSLogger(self.app_name)

    def run(self, package, launch_file, parameters={}, wait=False):
        cmdline = [
            "roslaunch" if self.ros_version == 1 else "ros2 launch",
            package,
            launch_file,
            " ".join(["{}:={}".format(k, v) for k, v in parameters.items()]),
            "--wait" if wait and self.ros_version == 1 else ""
        ]
        cmdline = " ".join(cmdline)
        self._process = pexpect.spawn(cmdline, encoding="utf-8", logfile=self._logger if self.debug else None)
        self._log_thread.start()

    def log(self):
        while True:
            try:
                self._process.expect([pexpect.TIMEOUT], timeout=0.1)
            except pexpect.exceptions.EOF as e:
                break

    def is_alive(self):
        if self._process is None:
            return False
        return self._process.isalive()

    def terminate(self):
        assert self._process is not None
        self._process.terminate()

        self._log_thread.join()
        if self.debug:
            self._logger.destroy()


class BridgeHelper(object):

    @classmethod
    def carla2ros_pose(cls, x, y, z, roll, pitch, yaw, to_quat=False):
        out_position = {"x": x, "y": -y, "z": z}
        out_orientation = {"roll": roll, "pitch": -pitch, "yaw": -yaw}
        if to_quat:
            out_orientation = cls.rpy2quat(
                out_orientation["roll"], out_orientation["pitch"], out_orientation["yaw"]
            )
        return {
            "position": out_position,
            "orientation": out_orientation
        }

    @staticmethod
    def rpy2quat(roll, pitch, yaw):
        quat = transforms3d.euler.euler2quat(roll, pitch, yaw)
        return {"x": quat[1], "y": quat[2], "z": quat[3], "w": quat[0]}


class ROSBaseAgent(AutonomousAgent):

    """
    Base class for ROS-based stacks.

    The sensor data is published using carla-ros-bridge. You can find details about
    the utilized datatypes there.
    """

    def __init__(self, ros_version, carla_host, carla_port, debug=False):
        super(ROSBaseAgent, self).__init__(carla_host, carla_port, debug)

        self._bridge_process = ROSLauncher("bridge", ros_version=ros_version, debug=debug)
        self._bridge_process.run(
            package="carla_ros_bridge",
            launch_file="carla_ros_bridge.launch" if ros_version == 1 else "carla_ros_bridge.launch.py",
            parameters={
                "host": carla_host,
                "port": carla_port,
                "timeout": 60,
                "synchronous_mode": True,
                "passive": True,
                "register_all_sensors": False,
                "ego_vehicle_role_name": "\"''\""
            },
            wait=True
        )

        self._agent_process = ROSLauncher("agent", ros_version=ros_version, debug=debug)
        self._agent_process.run(
            **self.get_ros_entrypoint(),
            wait=True
        )

        self._control_queue = queue.Queue(1)
        self._last_control_timestamp = None

    def get_ros_entrypoint(self):
        raise NotImplementedError

    def spawn_object(self, type_, id_, transform, attributes, attach_to=0):
        raise NotImplementedError

    def destroy_object(self, uid):
        raise NotImplementedError

    def _vehicle_control_cmd_callback(self, control_msg):
        if isinstance(control_msg, dict):
            control_timestamp = control_msg["header"]["stamp"]["secs"] + control_msg["header"]["stamp"]["nsecs"] * 1e-9
            control = carla.VehicleControl(
                steer=control_msg["steer"],
                throttle=control_msg["throttle"],
                brake=control_msg["brake"],
                hand_brake=control_msg["hand_brake"],
                reverse=control_msg["reverse"],
                manual_gear_shift=control_msg["manual_gear_shift"],
                gear=control_msg["gear"]
            )
        else:
            control_timestamp = control_msg.header.stamp.sec + control_msg.header.stamp.nanosec * 1e-9
            control = carla.VehicleControl(
                steer=control_msg.steer,
                throttle=control_msg.throttle,
                brake=control_msg.brake,
                hand_brake=control_msg.hand_brake,
                reverse=control_msg.reverse,
                manual_gear_shift=control_msg.manual_gear_shift,
                gear=control_msg.gear
            )

        # Checks that the received control timestamp is not repeated.
        if self._last_control_timestamp is not None and abs(self._last_control_timestamp - control_timestamp) < EPSILON:
            print(
                "\033[93mWARNING: A new vehicle command with a repeated timestamp has been received {} .\033[0m".format(control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        # Checks that the received control timestamp is the expected one.
        # We need to retrieve the simulation time directly from the CARLA snapshot instead of using the GameTime object to avoid
        # a race condition between the execution of this callback and the update of the GameTime internal variables.
        carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        if abs(control_timestamp - carla_timestamp) > EPSILON:
            print(
                "\033[93mWARNING: Expecting a vehicle command with timestamp {} but the timestamp received was {} .\033[0m".format(carla_timestamp, control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        self._last_control_timestamp = control_timestamp
        try:
            self._control_queue.put_nowait((control_timestamp, control))
        except queue.Full:
            print(
                "\033[93mWARNING: A new vehicle command has been received while the previous one has not been yet processed.\033[0m",
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")


    def run_step(self, _, timestamp):
        assert self._bridge_process.is_alive()
        assert self._agent_process.is_alive()

        #control_timestamp, control = self._control_queue.get(True)
        try:
            control_timestamp, control = self._control_queue.get(True, 1.0)
        except queue.Empty:
            control_timestamp, control = 0, carla.VehicleControl()

        carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        if abs(control_timestamp - carla_timestamp) > EPSILON:
            print(
                "\033[93mWARNING: Expecting a vehicle command with timestamp {} but the timestamp received was {} .\033[0m".format(carla_timestamp, control_timestamp),
                 sep=" ")

        return control

    def destroy(self):
        self._agent_process.terminate()
        self._bridge_process.terminate()

        assert not self._agent_process.is_alive()
        assert not self._bridge_process.is_alive()
