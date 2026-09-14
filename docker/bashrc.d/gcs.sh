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
    _pane_cmd "$p4" 3 "rviz -d $CATKIN_WS/src/image_processing/rviz/gcs_rviz.rviz"

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

# --- GCS standalone single-computer stack in tmux (even 4x3 grid) ----------
# Everything gcs_tmux runs (spar_uavasr.launch emulator, rviz, position
# monitors, bag recording, rqt GUIs), plus breadcrumb.launch, the
# qutas_lab_450 vicon/optitrack bridge, and the ArUco mission node
# (demo_ml) — 11 assigned panes plus one spare, all equally sized.
#
# Panes are pre-loaded but not started; press Enter in a pane to run it.
# Each pane is cleared (Ctrl+L) before its command is pre-filled.
# Pass --run to start everything automatically (staggered by sleeps).
function gcs_tmux_standalone() {
    local TMUX_AUTORUN
    if ! TMUX_AUTORUN="$(_tmux_autorun "$1")"; then
        echo "usage: gcs_tmux_standalone [--run]" >&2
        return 1
    fi

    echo "Starting standalone GCS tmux session (session: gcs_standalone_stack)"
    tmux kill-session -t gcs_standalone_stack 2>/dev/null

    tmux new-session -d -s gcs_standalone_stack -x 220 -y 60
    tmux set-option -t gcs_standalone_stack mouse on

    local panes=()
    panes+=("$(tmux display-message -p -t gcs_standalone_stack '#{pane_id}')")

    # Build 12 equal panes: split off pane 0 each time, then re-tile
    # immediately so no pane ever shrinks below what the next split needs.
    # tmux's "tiled" layout picks the grid shape itself — for a 220x60
    # session with 12 panes that comes out to 4 columns x 3 rows.
    for i in $(seq 1 11); do
        local new_pane
        new_pane=$(tmux split-window -d -P -F '#{pane_id}' -t "${panes[0]}")
        panes+=("$new_pane")
        tmux select-layout -t gcs_standalone_stack tiled
    done

    # Assign commands to the 12 panes, in reading order.
    tmux send-keys -t "${panes[0]}" C-l
    _pane_cmd "${panes[0]}" 0 "roslaunch spar_node spar_uavasr.launch"

    tmux send-keys -t "${panes[1]}" C-l
    _pane_cmd "${panes[1]}" 3 "mkdir -p $CATKIN_WS/bags && rosbag record -a -o $CATKIN_WS/bags/gcs"

    tmux send-keys -t "${panes[2]}" C-l
    _pane_cmd "${panes[2]}" 3 "roslaunch $CATKIN_WS/launch/breadcrumb.launch"

    tmux send-keys -t "${panes[3]}" C-l
    _pane_cmd "${panes[3]}" 3 "roslaunch qutas_lab_450 environment.launch vicon_server_dvp:=${VICON_SERVER_DVP}"

    tmux send-keys -t "${panes[4]}" C-l
    _pane_cmd "${panes[4]}" 8 "rosrun spar_node demo_ml"

    tmux send-keys -t "${panes[5]}" C-l
    _pane_cmd "${panes[5]}" 8 "rostopic echo /mavros/local_position/pose"

    tmux send-keys -t "${panes[6]}" C-l
    _pane_cmd "${panes[6]}" 8 "rostopic echo /mavros/vision_pose/pose"

    tmux send-keys -t "${panes[7]}" C-l
    _pane_cmd "${panes[7]}" 3 "rviz -d $CATKIN_WS/src/image_processing/rviz/gcs_rviz.rviz"

    tmux send-keys -t "${panes[8]}" C-l
    _pane_cmd "${panes[8]}" 5 "rosrun rqt_generic_hud rqt_generic_hud"

    tmux send-keys -t "${panes[9]}" C-l
    _pane_cmd "${panes[9]}" 5 "rosrun rqt_mavros_gui rqt_mavros_gui"

    # Kill switch — pre-loaded, never auto-run.
    tmux send-keys -t "${panes[10]}" C-l
    tmux send-keys -t "${panes[10]}" "tmux kill-session -t gcs_standalone_stack"

    # Spare pane — cleared and left empty for whatever you need.
    tmux send-keys -t "${panes[11]}" C-l

    if [ "$TMUX_AUTORUN" != "1" ]; then
        echo "Panes are pre-loaded — press Enter in each one to start it (emulator first)."
    fi

    tmux select-pane -t "${panes[0]}"
    tmux attach-session -t gcs_standalone_stack
}

# Same as gcs_tmux_standalone, but points ROS_MASTER_URI at this machine
# first — single-computer SITL testing, no second GCS/UAV machine involved.
function gcs_tmux_sim_standalone() {
    echo "Switching to a local ROS master for standalone single-computer testing..."
    disros "$(hostname -I | cut -d' ' -f1)"
    gcs_tmux_standalone "$@"
}

export -f gcs_tmux_standalone gcs_tmux_sim_standalone gcs_tmux gcs_tmux_sim
