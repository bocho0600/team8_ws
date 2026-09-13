# GCS-role tmux launchers, sourced by ros.sh when ROLE=gcs.
# (Shared setup, disros and the aruco/servo helpers live in ros.sh.)

# --- GCS emulator/monitoring stack in tmux ----------------------------------
# spar_uavasr.launch (spar node + software-in-the-loop uavasr emulator, so
# mission logic can be exercised without a real drone), rviz, position
# monitors, bag recording, and the rqt GUIs.
#
# Panes are pre-loaded but not started; press Enter in a pane to run it.
# Pass --run to start everything automatically (staggered by sleeps).
function gcs_tmux() {
    local TMUX_AUTORUN
    if ! TMUX_AUTORUN="$(_tmux_autorun "$1")"; then
        echo "usage: gcs_tmux [--run]" >&2
        return 1
    fi

    echo "Starting GCS tmux session (session: gcs_stack)"
    tmux kill-session -t gcs_stack 2>/dev/null

    tmux new-session -d -s gcs_stack -x 220 -y 60
    tmux set-option -t gcs_stack mouse on

    p0=$(tmux display-message -p -t gcs_stack '#{pane_id}')
    _pane_cmd "$p0" 0 "roslaunch spar_node spar_uavasr.launch"

    # Columns first, then rows within each column — splitting rows before
    # columns only divides the single row you split, not the whole session.
    p4=$(tmux split-window -d -h -p 70 -P -F '#{pane_id}' -t "$p0")

    # --- Left column: emulator launch | rosbag | local pos | vision pos ---
    p1=$(tmux split-window -d -v -p 75 -P -F '#{pane_id}' -t "$p0")
    _pane_cmd "$p1" 3 "mkdir -p $CATKIN_WS/bags && rosbag record -a -o $CATKIN_WS/bags/gcs"

    p2=$(tmux split-window -d -v -p 66 -P -F '#{pane_id}' -t "$p1")
    _pane_cmd "$p2" 3 "rostopic echo /mavros/local_position/pose"

    p3=$(tmux split-window -d -v -t "$p2")
    _pane_cmd "$p3" 3 "rostopic echo /mavros/vision_pose/pose"

    # --- Right column: rviz | rqt_generic_hud | rqt_mavros_gui | kill switch ---
    _pane_cmd "$p4" 3 "rviz -d $CATKIN_WS/src/spar/spar_node/rviz/emulator_configuration.rviz"

    p5=$(tmux split-window -d -v -p 66 -P -F '#{pane_id}' -t "$p4")
    _pane_cmd "$p5" 5 "rosrun rqt_generic_hud rqt_generic_hud"

    p6=$(tmux split-window -d -v -p 50 -P -F '#{pane_id}' -t "$p5")
    _pane_cmd "$p6" 5 "rosrun rqt_mavros_gui rqt_mavros_gui"

    # Deliberately never auto-run: pre-loads the kill command without running
    # it, so the whole stack doesn't tear itself down the instant it starts.
    p7=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p6")
    tmux send-keys -t "$p7" "tmux kill-session -t gcs_stack"

    if [ "$TMUX_AUTORUN" != "1" ]; then
        echo "Panes are pre-loaded — press Enter in each one to start it (emulator first)."
    fi

    tmux select-pane -t "$p0"
    tmux attach-session -t gcs_stack
}

# Same as gcs_tmux, but runs the emulator against a local ROS master instead
# of the real UAV — for testing mission logic standalone (SITL).
function gcs_tmux_sim() {
    echo "Switching to a local ROS master for standalone SITL testing..."
    disros "$(hostname -I | cut -d' ' -f1)"
    gcs_tmux "$@"
}

export -f gcs_tmux gcs_tmux_sim
