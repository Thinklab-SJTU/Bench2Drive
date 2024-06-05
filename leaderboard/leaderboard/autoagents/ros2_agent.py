#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS2 stack
"""

from __future__ import print_function

import threading

import carla

from leaderboard.autoagents.ros_base_agent import BridgeHelper, ROSBaseAgent

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaRoute, CarlaGnssRoute
from carla_msgs.srv import SpawnObject, DestroyObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool


def wait_for_message(node, topic, topic_type, timeout=None):

    s = None
    try:
        future = Future()
        s = node.create_subscription(
            topic_type,
            topic,
            lambda msg: future.set_result(msg.data),
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        rclpy.spin_until_future_complete(node, future, timeout)

    finally:
        if s is not None:
            node.destroy_subscription(s)

    return future.result()


class ROS2Agent(ROSBaseAgent):

    """
    Autonomous agent base class. All user agents have to be derived from this class
    """

    ROS_VERSION = 2

    def __init__(self, carla_host, carla_port, debug=False):
        super(ROS2Agent, self).__init__(self.ROS_VERSION, carla_host, carla_port, debug)

        rclpy.init(args=None)
        self.ros_node = rclpy.create_node("leaderboard_node")

        self._spawn_object_service = self.ros_node.create_client(SpawnObject, "/carla/spawn_object")
        self._destroy_object_service = self.ros_node.create_client(DestroyObject, "/carla/destroy_object")

        self._spawn_object_service.wait_for_service()
        self._destroy_object_service.wait_for_service()

        self._path_publisher = self.ros_node.create_publisher(CarlaRoute, "/carla/hero/global_plan", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._path_gnss_publisher = self.ros_node.create_publisher(CarlaGnssRoute, "/carla/hero/global_plan_gnss", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        self.ctrl_subscriber = self.ros_node.create_subscription(CarlaEgoVehicleControl, "/carla/hero/vehicle_control_cmd", self._vehicle_control_cmd_callback, qos_profile=QoSProfile(depth=1))

        wait_for_message(self.ros_node, "/carla/hero/status", Bool)

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,))
        self.spin_thread.start()

    @staticmethod
    def get_ros_version():
        return ROS2Agent.ROS_VERSION

    def spawn_object(self, type_, id_, transform, attributes, attach_to=0):
        spawn_point = BridgeHelper.carla2ros_pose(
            transform.location.x, transform.location.y, transform.location.z,
            transform.rotation.roll, transform.rotation.pitch, transform.rotation.yaw,
            to_quat=True
        )

        request = SpawnObject.Request()
        request.type = type_
        request.id = id_
        request.attach_to = attach_to
        request.transform = Pose(position=Point(**spawn_point["position"]), orientation=Quaternion(**spawn_point["orientation"]))
        request.random_pose = False
        request.attributes.extend([
            KeyValue(key=str(k), value=str(v)) for k,v in attributes.items()
        ])

        # Call the service synchronously.
        response = self._spawn_object_service.call(request)
        if response.id == -1:
            raise RuntimeError("{} could not be spawned. {}".format(type_, response["error_string"]))
        return response.id

    def destroy_object(self, uid):
        request = DestroyObject.Request()
        request.id = uid

        # Call the servive syncrhonoulsy
        response = self._destroy_object_service.call(request)
        if not response.success:
            raise RuntimeError("{} could not be destroyed".format(uid))
        return response.success

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        super(ROS2Agent, self).set_global_plan(global_plan_gps, global_plan_world_coord)

        path = CarlaRoute()
        for wp in self._global_plan_world_coord:
            path.road_options.append(int(wp[1]))
            pose = BridgeHelper.carla2ros_pose(
                wp[0].location.x, wp[0].location.y, wp[0].location.z,
                wp[0].rotation.roll, wp[0].rotation.pitch, wp[0].rotation.yaw,
                to_quat=True
            )
            path.poses.append(
                Pose(position=Point(**pose["position"]), orientation=Quaternion(**pose["orientation"])))
        self._path_publisher.publish(path)

        path_gnss = CarlaGnssRoute()
        for wp in self._global_plan:
            path_gnss.road_options.append(wp[1])
            path.poses.append(
                NavSatFix(latitude=wp[0]["lat"], longitude=wp[0]["lon"], altitude=wp[0]["z"]))
        self._path_gnss_publisher.publish(path_gnss)

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.spin_thread.join()

        super(ROS2Agent, self).destroy()
