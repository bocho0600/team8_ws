# UAV-role setup, sourced by ros.sh when ROLE=uav.
# The UAV runs its own roscore, so it's the ROS master by default.

# Auto-detects this machine's own IP. Leaves ROS_MASTER_URI on its default
# (localhost, i.e. this machine) unless given an explicit master to join.
disros() {
  export ROS_IP="$(hostname -I | cut -d' ' -f1)"
  echo "Identifying as: $ROS_IP"

  if [ -n "$1" ]; then
    export ROS_MASTER_URI="http://$1:11311"
    echo "Connecting to: $ROS_MASTER_URI"
  fi
}

# --- Full UAV flight stack in tmux ------------------------------------------
# roscore, MAVROS + spar flight control, vision pipeline, path planner,
# vicon/optitrack bridge, the ArUco mission node, position monitors, and bag
# recording — each in its own pane.
function run_uav_stack() {
    echo "Starting UAV flight stack in tmux (session: uav_stack)"
    tmux kill-session -t uav_stack 2>/dev/null

    local gcs_arg=""
    if [ -n "$GCS_IP" ]; then
        gcs_arg="gcs_url:=udp://@${GCS_IP}:14550"
    fi

    # Explicit size: a detached session otherwise inherits whatever pty
    # happened to be around at creation time, which can be too small to fit
    # every split ("no space for new pane"). tmux reflows this to whatever
    # terminal actually attaches, so a generous size here is always safe.
    tmux new-session -d -s uav_stack -x 220 -y 60
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

export -f disros run_uav_stack
