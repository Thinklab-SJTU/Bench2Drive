#!/bin/bash

DOC_STRING="Build agent leadearboard image."

USAGE_STRING=$(cat <<- END
Usage: $0 [-h|--help] [-r|--ros-distro ROS_DISTRO]

The following ROS distributions are supported:
    * melodic
    * noetic
    * foxy
END
)

usage() { echo "$DOC_STRING"; echo "$USAGE_STRING"; exit 1; }

ROS_DISTRO=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -r |--ros-distro )
      ROS_DISTRO=$2
      if [ "$ROS_DISTRO" != "melodic" ] && [ "$ROS_DISTRO" != "noetic" ] && [ "$ROS_DISTRO" != "foxy" ]; then
        usage
      fi
      shift 2 ;;
    -h | --help )
      usage
      ;;
    * )
      shift ;;
  esac
done

if [ -z "$CARLA_ROOT" ]; then
  echo "Error $CARLA_ROOT is empty. Set \$CARLA_ROOT as an environment variable first."
  exit 1
fi

if [ -z "$SCENARIO_RUNNER_ROOT" ]; then
  echo "Error $SCENARIO_RUNNER_ROOT is empty. Set \$SCENARIO_RUNNER_ROOT as an environment variable first."
  exit 1
fi

if [ ! -z "$ROS_DISTRO" ] && [ -z $CARLA_ROS_BRIDGE_ROOT ]; then
  echo "Error $CARLA_ROS_BRIDGE_ROOT is empty. Set \$CARLA_ROS_BRIDGE_ROOT as an environment variable first."
  exit 1
fi

if [ -z "$LEADERBOARD_ROOT" ]; then
  echo "Error $LEADERBOARD_ROOT is empty. Set \$LEADERBOARD_ROOT as an environment variable first."
  exit 1
fi

if [ -z "$TEAM_CODE_ROOT" ]; then
  echo "Error $TEAM_CODE_ROOT is empty. Set \$TEAM_CODE_ROOT as an environment variable first."
  exit 1
fi

rm -fr .lbtmp
mkdir -p .lbtmp

echo "Copying CARLA Python API"
cp -fr ${CARLA_ROOT}/PythonAPI  .lbtmp
mv .lbtmp/PythonAPI/carla/dist/carla*-py2*.egg .lbtmp/PythonAPI/carla/dist/carla-leaderboard-py2.7.egg
mv .lbtmp/PythonAPI/carla/dist/carla*-py3*.egg .lbtmp/PythonAPI/carla/dist/carla-leaderboard-py3x.egg

echo "Copying Scenario Runner"
cp -fr ${SCENARIO_RUNNER_ROOT}/ .lbtmp
rm -fr .lbtmp/scenario_runner/.git

echo "Copying Leaderboard"
cp -fr ${LEADERBOARD_ROOT}/ .lbtmp
rm -fr .lbtmp/leaderboard/.git

if [ ! -z "$ROS_DISTRO" ]; then
  echo "Copying ROS Bridge"
  cp -fr ${CARLA_ROS_BRIDGE_ROOT}/ .lbtmp/carla_ros_bridge
fi

echo "Copying team code folder"
cp -fr ${TEAM_CODE_ROOT}/ .lbtmp/team_code

cp ${LEADERBOARD_ROOT}/scripts/agent_entrypoint.sh .lbtmp/agent_entrypoint.sh

echo "Building docker"
# build docker image
if [ -z "$ROS_DISTRO" ]; then
  docker build \
    --force-rm  \
    -t leaderboard-user \
    -f ${LEADERBOARD_ROOT}/scripts/Dockerfile.master .lbtmp
else
  docker build \
    --force-rm  \
    -t leaderboard-user:ros-$ROS_DISTRO \
    -f ${LEADERBOARD_ROOT}/scripts/Dockerfile.ros .lbtmp \
    --build-arg ROS_DISTRO=$ROS_DISTRO
fi

rm -fr .lbtmp
