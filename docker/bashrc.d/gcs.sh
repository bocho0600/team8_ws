# GCS-role setup, sourced by ros.sh when ROLE=gcs.
# The UAV runs roscore, so the GCS is a client by default.

# Auto-detects this machine's own IP and points ROS_MASTER_URI at the UAV
# (UAV_IP, from .env) by default. Call with an explicit master IP to
# override for one shell, e.g. to point at yourself for local SITL testing:
#   disros $(hostname -I)
disros() {
  export ROS_IP="$(hostname -I | cut -d' ' -f1)"
  echo "Identifying as: $ROS_IP"

  local master="${1:-$UAV_IP}"
  if [ -n "$master" ]; then
    export ROS_MASTER_URI="http://$master:11311"
    echo "Connecting to: $ROS_MASTER_URI"
  fi
}

# --- GCS emulator/monitoring stack in tmux ----------------------------------
# spar_uavasr.launch (spar node + software-in-the-loop uavasr emulator, so
# mission logic can be exercised without a real drone), rviz, position
# monitors, bag recording, and the rqt GUIs.
function gcs_tmux() {
    echo "Starting GCS tmux session (session: gcs_stack)"
    tmux kill-session -t gcs_stack 2>/dev/null

    tmux new-session -d -s gcs_stack -x 220 -y 60
    tmux set-option -t gcs_stack mouse on

    p0=$(tmux display-message -p -t gcs_stack '#{pane_id}')
    tmux send-keys -t "$p0" "roslaunch spar_node spar_uavasr.launch" C-m

    # Columns first, then rows within each column — splitting rows before
    # columns only divides the single row you split, not the whole session.
    p4=$(tmux split-window -d -h -p 70 -P -F '#{pane_id}' -t "$p0")

    # --- Left column: emulator launch | rosbag | local pos | vision pos ---
    p1=$(tmux split-window -d -v -p 75 -P -F '#{pane_id}' -t "$p0")
    tmux send-keys -t "$p1" "sleep 3; mkdir -p $CATKIN_WS/bags && rosbag record -a -o $CATKIN_WS/bags/gcs" C-m

    p2=$(tmux split-window -d -v -p 66 -P -F '#{pane_id}' -t "$p1")
    tmux send-keys -t "$p2" "sleep 3; rostopic echo /mavros/local_position/pose" C-m

    p3=$(tmux split-window -d -v -t "$p2")
    tmux send-keys -t "$p3" "sleep 3; rostopic echo /mavros/vision_pose/pose" C-m

    # --- Right column: rviz | rqt_generic_hud | rqt_mavros_gui | kill switch ---
    tmux send-keys -t "$p4" "sleep 3; rviz -d $CATKIN_WS/src/spar/spar_node/rviz/emulator_configuration.rviz" C-m

    p5=$(tmux split-window -d -v -p 66 -P -F '#{pane_id}' -t "$p4")
    tmux send-keys -t "$p5" "sleep 5; rosrun rqt_generic_hud rqt_generic_hud" C-m

    p6=$(tmux split-window -d -v -p 50 -P -F '#{pane_id}' -t "$p5")
    tmux send-keys -t "$p6" "sleep 5; rosrun rqt_mavros_gui rqt_mavros_gui" C-m

    # Deliberately no C-m here: pre-loads the kill command without running
    # it, so the whole stack doesn't tear itself down the instant it starts.
    p7=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p6")
    tmux send-keys -t "$p7" "tmux kill-session -t gcs_stack"

    tmux select-pane -t "$p0"
    tmux attach-session -t gcs_stack
}

# Same as gcs_tmux, but runs the emulator against a local ROS master instead
# of the real UAV — for testing mission logic standalone (SITL).
function gcs_tmux_sim() {
    echo "Switching to a local ROS master for standalone SITL testing..."
    disros "$(hostname -I | cut -d' ' -f1)"
    gcs_tmux
}

export -f disros gcs_tmux gcs_tmux_sim
