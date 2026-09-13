# Shared setup + helpers for both the UAV and the GCS. Sourced by ros.sh
# before the role-specific file (uav.sh or gcs.sh).

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

CATKIN_WS="$HOME/catkin_ws"

# --- Actuator + ArUco landing helpers ---------------------------------------
function servo_open1() { rostopic pub /actuator_control/actuator_a std_msgs/Bool '{data: false}'; }
function servo_open2() { rostopic pub /actuator_control/actuator_a std_msgs/Bool '{data: true}'; }

function _aruco_frame_arg() {
    local arg="$1"
    if [[ "$arg" =~ ^[0-9]+$ ]]; then
        echo "target_aruco_${arg}"
    else
        echo "$arg"
    fi
}

function aruco_land() {
    if [ -z "$1" ]; then
        echo "usage: aruco_land <marker_id|frame_name>"
        echo "   eg: aruco_land 7"
        return 1
    fi
    local frame
    frame="$(_aruco_frame_arg "$1")"
    echo "Requesting LAND on '${frame}'..."
    rosservice call /aruco/land "frame: '${frame}'"
}

function aruco_roi() {
    if [ -z "$1" ]; then
        echo "usage: aruco_roi <marker_id|frame_name>"
        echo "   eg: aruco_roi 7"
        return 1
    fi
    local frame
    frame="$(_aruco_frame_arg "$1")"
    echo "Requesting ROI over '${frame}'..."
    rosservice call /aruco/roi "frame: '${frame}'"
}

# Land at an arbitrary map coordinate (the manual override topic).
function aruco_land_point() {
    if [ $# -ne 3 ]; then
        echo "usage: aruco_land_point <x> <y> <z>"
        echo "   eg: aruco_land_point 1.5 2.0 0.0"
        return 1
    fi
    echo "Requesting LAND at map [$1, $2, $3]..."
    rostopic pub -1 /request_landing_point geometry_msgs/Point \
        "{x: $1, y: $2, z: $3}"
}

# Show what the mission node could actually land on right now: static TF
# frames plus anything it detected during the scan.
function aruco_frames() {
    echo "--- target frames in TF ---"
    rosrun tf tf_monitor 2>/dev/null | grep -o 'target_[a-z0-9_]*' | sort -u
    echo "--- detections seen (from node log) ---"
    echo "  check the demo_ml terminal for 'Target frames found during scan'"
}

export -f servo_open1 servo_open2 _aruco_frame_arg aruco_land aruco_roi aruco_land_point aruco_frames
