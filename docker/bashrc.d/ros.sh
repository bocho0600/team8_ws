# Sourced from ~/.bashrc in every interactive container shell (see Dockerfile).
# Mirrors the tmux-based launch workflow used directly on the UAV Pi, adapted
# for this Dockerized workspace.

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

CATKIN_WS="$HOME/catkin_ws"

# --- Distributed ROS -------------------------------------------------------
# Auto-detects this machine's own IP and points ROS_MASTER_URI at the UAV
# (UAV_IP, from .env / docker-compose) by default, since the UAV runs
# roscore. Call with an explicit master IP to override for one shell:
#   disros 192.168.1.50
disros() {
  export ROS_IP="$(hostname -I | cut -d' ' -f1)"
  echo "Identifying as: $ROS_IP"

  local master="${1:-$UAV_IP}"
  if [ -n "$master" ]; then
    export ROS_MASTER_URI="http://$master:11311"
    echo "Connecting to: $ROS_MASTER_URI"
  fi
}
disros

# --- Full UAV flight stack in tmux ------------------------------------------
# roscore, MAVROS + spar flight control, vision pipeline, path planner,
# vicon/optitrack bridge, the ArUco mission node, position monitors, and bag
# recording — each in its own pane. Run this on the UAV.
function run_uav_stack() {
    echo "Starting UAV flight stack in tmux (session: uav_stack)"
    tmux kill-session -t uav_stack 2>/dev/null

    local gcs_arg=""
    if [ -n "$GCS_IP" ]; then
        gcs_arg="gcs_url:=udp://@${GCS_IP}:14550"
    fi

    tmux new-session -d -s uav_stack
    tmux set-option -t uav_stack mouse on

    p0=$(tmux display-message -p -t uav_stack '#{pane_id}')
    tmux send-keys -t "$p0" "roscore" C-m

    # --- Row 1: roscore | system monitor | free terminal ---
    p1=$(tmux split-window -d -h -p 66 -P -F '#{pane_id}' -t "$p0")
    tmux send-keys -t "$p1" "htop" C-m

    p2=$(tmux split-window -d -h -p 50 -P -F '#{pane_id}' -t "$p1")
    tmux send-keys -t "$p2" "cd $CATKIN_WS" C-m

    # --- Row 2: flight control | vision + servo | vicon/optitrack ---
    p3=$(tmux split-window -d -v -p 80 -P -F '#{pane_id}' -t "$p0")
    tmux send-keys -t "$p3" "sleep 3; roslaunch $CATKIN_WS/launch/control.launch ${gcs_arg}" C-m

    p4=$(tmux split-window -d -v -p 80 -P -F '#{pane_id}' -t "$p1")
    tmux send-keys -t "$p4" "sleep 5; roslaunch $CATKIN_WS/launch/combined_nodes.launch" C-m

    p5=$(tmux split-window -d -v -p 80 -P -F '#{pane_id}' -t "$p2")
    tmux send-keys -t "$p5" "sleep 5; roslaunch qutas_lab_450 environment.launch vicon_server_dvp:=${VICON_SERVER_DVP}" C-m

    # --- Row 3: breadcrumb | ArUco mission (demo_ml) | local position ---
    p6=$(tmux split-window -d -v -p 60 -P -F '#{pane_id}' -t "$p3")
    tmux send-keys -t "$p6" "sleep 8; roslaunch $CATKIN_WS/launch/breadcrumb.launch" C-m

    p7=$(tmux split-window -d -v -p 60 -P -F '#{pane_id}' -t "$p4")
    tmux send-keys -t "$p7" "sleep 10; rosrun spar_node demo_ml" C-m

    p8=$(tmux split-window -d -v -p 60 -P -F '#{pane_id}' -t "$p5")
    tmux send-keys -t "$p8" "sleep 10; rostopic echo /mavros/local_position/pose" C-m

    # --- Row 4: vision pose | rosbag record | kill session ---
    p9=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p6")
    tmux send-keys -t "$p9" "sleep 10; rostopic echo /mavros/vision_pose/pose" C-m

    p10=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p7")
    tmux send-keys -t "$p10" "sleep 10; mkdir -p $CATKIN_WS/bags && rosbag record -a -o $CATKIN_WS/bags/flight" C-m

    # Deliberately no C-m here: pre-loads the kill command without running
    # it, so the whole stack doesn't tear itself down the instant it starts.
    p11=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p8")
    tmux send-keys -t "$p11" "tmux kill-session -t uav_stack"

    tmux select-pane -t "$p0"
    tmux attach-session -t uav_stack
}

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

export -f run_uav_stack servo_open1 servo_open2 _aruco_frame_arg aruco_land aruco_roi aruco_land_point aruco_frames
