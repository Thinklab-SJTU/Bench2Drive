# ROS-based Challenge Agent

Interfacing CARLA from ROS is normally done via [CARLA ROS Bridge](https://github.com/carla-simulator/ros-bridge).
In Challenge Mode this bridging functionality is provided by a RosAgent. It uses the same topics and message-types for the sensors but does not publish tf-transformations.
 
# Requirements

* `roscore` is expected to be running in the docker container. Please adapt your entrypoint.

## Setup

To enable your stack within challenge mode, the following steps need to be taken:

1. Define Sensor Setup
2. Define Startup

### Define Sensor Setup

Derive from RosAgent and implement the sensors() method.

    from srunner.autoagents.ros_agent import RosAgent

    class MyRosAgent(RosAgent):

        def sensors(self):
            return [ <sensor-definition> ]

As an example for the sensor definition, see [HumanAgent.py](../srunner/autoagents/human_agent.py).


### Define Startup

The startup of the stack is done within the shell script `$TEAM_CODE_ROOT/start.sh`.
Therefore the environment variable `TEAM_CODE_ROOT` must be set.

RosAgent takes care of executing and monitoring. The script shall remain running as long as the stack is active.

Example for start.sh

    #!/bin/bash -e
    roslaunch $TEAM_CODE_ROOT/challenge.launch


## Testing

In general, the challenge execution is headless. For diagnosis you're still able to use ros-tools like rviz or rqt. Additionally you
can use [carla_manual_control](https://github.com/carla-simulator/ros-bridge/tree/master/carla_manual_control) from the carla_ros_bridge for visualization (and also controlling the vehicle).

