#!/bin/bash
set -e

# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "/home/uavteam8/catkin_ws/devel/setup.bash"

exec "$@"
