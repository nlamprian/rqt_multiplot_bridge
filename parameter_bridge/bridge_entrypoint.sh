#!/bin/bash

set -e

unset ROS_DISTRO
source "/opt/ros/$ROS1_DISTRO/setup.bash" --
rosparam load /bridge.yaml

exec /ros_entrypoint.sh "$@"
