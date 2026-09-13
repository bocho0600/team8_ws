#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source "/home/uavteam8/catkin_ws/devel/setup.bash"

exec "$@"
