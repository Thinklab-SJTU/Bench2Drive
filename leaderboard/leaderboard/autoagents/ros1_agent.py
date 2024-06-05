#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS stack
"""

from __future__ import print_function

import time

from leaderboard.autoagents.ros_base_agent import BridgeHelper, ROSBaseAgent, ROSLauncher

import roslibpy


class ROS1Server(object):
    client = roslibpy.Ros(host='localhost', port=9090)

    def __init__(self, debug=True):
        self._server_process = ROSLauncher("server", ros_version=1, debug=debug)

    def start(self):
        self._server_process.run(
            package="rosbridge_server",
            launch_file="rosbridge_websocket.launch",
            wait=False
        )
        ROS1Server.client.run(30)

    def shutdown(self):
        self._server_process.terminate()
        assert not self._server_process.is_alive()
        ROS1Server.client.close()


def wait_for_message(client, topic, topic_type, timeout=None):

    class _WFM(object):
        def __init__(self):
            self.msg = None
        def cb(self, msg):
            if self.msg is None:
                self.msg = msg

    wfm = _WFM()
    s = None
    try:
        s = roslibpy.Topic(client, topic, topic_type, reconnect_on_close=False)
        s.subscribe(wfm.cb)
        if timeout is not None:
            timeout_t = time.time() + timeout
        while client.is_connected and wfm.msg is None:
            time.sleep(0.1)
            if timeout is not None and time.time() >= timeout_t:
                raise TimeoutError("timeout exceeded while waiting for message on topic {}".format(topic))

    finally:
        if s is not None:
            s.unsubscribe()

    return wfm.msg


def wait_for_service(client, service, timeout=None):

    if timeout is not None:
        timeout_t = time.time() + timeout

    services = client.get_services()
    while service not in services:
        time.sleep(0.1)
        if timeout is not None and time.time() >= timeout_t:
            raise TimeoutError("timeout exceeded while waiting for service {} to be ready".format(service))
        services = client.get_services()


class ROS1Agent(ROSBaseAgent):

    ROS_VERSION = 1

    def __init__(self, carla_host, carla_port, debug=False):
        super(ROS1Agent, self).__init__(self.ROS_VERSION, carla_host, carla_port, debug)

        client = ROS1Server.client

        self._spawn_object_service = roslibpy.Service(client, "/carla/spawn_object", "carla_msgs/SpawnObject", reconnect_on_close=False)
        self._destroy_object_service = roslibpy.Service(client, "/carla/destroy_object", "carla_msgs/DestroyObject", reconnect_on_close=False)

        wait_for_service(client, "/carla/spawn_object")
        wait_for_service(client, "/carla/destroy_object")

        self._control_subscriber = roslibpy.Topic(client, "/carla/hero/vehicle_control_cmd", "carla_msgs/CarlaEgoVehicleControl", queue_length=1, reconnect_on_close=False)
        self._control_subscriber.subscribe(self._vehicle_control_cmd_callback)

        self._path_publisher = roslibpy.Topic(client, "/carla/hero/global_plan", "carla_msgs/CarlaRoute", latch=True, reconnect_on_close=False)
        self._path_gnss_publisher = roslibpy.Topic(client, "/carla/hero/global_plan_gnss", "carla_msgs/CarlaGnssRoute", latch=True, reconnect_on_close=False)

        wait_for_message(client, "/carla/hero/status", "std_msgs/Bool")

    @staticmethod
    def get_ros_version():
        return ROS1Agent.ROS_VERSION

    def spawn_object(self, type_, id_, transform, attributes, attach_to=0):
        spawn_point = BridgeHelper.carla2ros_pose(
            transform.location.x, transform.location.y, transform.location.z,
            transform.rotation.roll, transform.rotation.pitch, transform.rotation.yaw,
            to_quat=True
        )
        attributes = [{"key": str(key), "value": str(value)} for key, value in attributes.items()]

        request = roslibpy.ServiceRequest({
            "type": type_,
            "id": id_,
            "attach_to": attach_to,
            "transform": spawn_point,
            "random_pose": False,
            "attributes": attributes,
        })

        response = self._spawn_object_service.call(request)
        if response["id"] == -1:
            raise RuntimeError("{} could not be spawned. {}".format(type_, response["error_string"]))
        return response["id"]

    def destroy_object(self, uid):
        request = roslibpy.ServiceRequest({"id": uid})
        response = self._destroy_object_service.call(request)
        if not response["success"]:
            raise RuntimeError("{} could not be destroyed".format(uid))
        return response["success"]

    def run_step(self, input_data, timestamp):
        return super(ROS1Agent, self).run_step(input_data, timestamp)

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        super(ROS1Agent, self).set_global_plan(global_plan_gps, global_plan_world_coord)

        poses = []
        for wp in self._global_plan_world_coord:
            poses.append(BridgeHelper.carla2ros_pose(
                wp[0].location.x, wp[0].location.y, wp[0].location.z,
                wp[0].rotation.roll, wp[0].rotation.pitch, wp[0].rotation.yaw,
                to_quat=True
            ))

        self._path_publisher.publish(roslibpy.Message(
            {
                "header": {
                    "frame_id": "/map"
                },
                "road_options": [ int(wp[1]) for wp in self._global_plan_world_coord ],
                "poses": [ pose for pose in poses ]
            }))

        self._path_gnss_publisher.publish(roslibpy.Message(
            {
                "header": {
                    "frame_id": "/map"
                },
                "road_options": [ int(wp[1]) for wp in self._global_plan ],
                "coordinates": [ {"latitude": wp[0]["lat"], "longitude": wp[0]["lon"], "altitude": wp[0]["z"]} for wp in self._global_plan ]
            }))

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        self._control_subscriber.unsubscribe()
        self._path_publisher.unadvertise()
        self._spawn_object_service.unadvertise()
        self._destroy_object_service.unadvertise()

        super(ROS1Agent, self).destroy()
