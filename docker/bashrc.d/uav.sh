# UAV-role tmux launcher, sourced by ros.sh when ROLE=uav.
# (Shared setup, disros and the aruco/servo helpers live in ros.sh.)

# --- Full UAV flight stack in tmux ------------------------------------------
# roscore, MAVROS + spar flight control, vision pipeline, path planner,
# vicon/optitrack bridge, the ArUco mission node, position monitors, and bag
# recording — each in its own pane.
#
# Panes are pre-loaded but not started; press Enter in a pane to run it.
# Pass --run to start everything automatically (staggered by sleeps).
function run_uav_stack() {
    local TMUX_AUTORUN
    if ! TMUX_AUTORUN="$(_tmux_autorun "$1")"; then
        echo "usage: run_uav_stack [--run]" >&2
        return 1
    fi

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
    _pane_cmd "$p0" 0 "roscore"

    # --- Row 1: roscore | system monitor | free terminal ---
    p1=$(tmux split-window -d -h -p 66 -P -F '#{pane_id}' -t "$p0")
    _pane_cmd "$p1" 0 "htop"

    p2=$(tmux split-window -d -h -p 50 -P -F '#{pane_id}' -t "$p1")
    tmux send-keys -t "$p2" "cd $CATKIN_WS" C-m

    # --- Row 2: flight control | vision + servo | vicon/optitrack ---
    p3=$(tmux split-window -d -v -p 80 -P -F '#{pane_id}' -t "$p0")
    _pane_cmd "$p3" 3 "roslaunch $CATKIN_WS/launch/control.launch ${gcs_arg}"

    p4=$(tmux split-window -d -v -p 80 -P -F '#{pane_id}' -t "$p1")
    _pane_cmd "$p4" 5 "roslaunch $CATKIN_WS/launch/combined_nodes.launch"

    p5=$(tmux split-window -d -v -p 80 -P -F '#{pane_id}' -t "$p2")
    _pane_cmd "$p5" 5 "roslaunch qutas_lab_450 environment.launch vicon_server_dvp:=${VICON_SERVER_DVP}"

    # --- Row 3: breadcrumb | ArUco mission (demo_ml) | local position ---
    p6=$(tmux split-window -d -v -p 60 -P -F '#{pane_id}' -t "$p3")
    _pane_cmd "$p6" 8 "roslaunch $CATKIN_WS/launch/breadcrumb.launch"

    p7=$(tmux split-window -d -v -p 60 -P -F '#{pane_id}' -t "$p4")
    _pane_cmd "$p7" 10 "rosrun spar_node demo_ml"

    p8=$(tmux split-window -d -v -p 60 -P -F '#{pane_id}' -t "$p5")
    _pane_cmd "$p8" 10 "rostopic echo /mavros/local_position/pose"

    # --- Row 4: vision pose | rosbag record | kill session ---
    p9=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p6")
    _pane_cmd "$p9" 10 "rostopic echo /mavros/vision_pose/pose"

    p10=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p7")
    _pane_cmd "$p10" 10 "mkdir -p $CATKIN_WS/bags && rosbag record -a -o $CATKIN_WS/bags/flight"

    # Deliberately never auto-run: pre-loads the kill command without running
    # it, so the whole stack doesn't tear itself down the instant it starts.
    p11=$(tmux split-window -d -v -P -F '#{pane_id}' -t "$p8")
    tmux send-keys -t "$p11" "tmux kill-session -t uav_stack"

    if [ "$TMUX_AUTORUN" != "1" ]; then
        echo "Panes are pre-loaded — press Enter in each one to start it (roscore first)."
    fi

    tmux select-pane -t "$p0"
    tmux attach-session -t uav_stack
}

export -f run_uav_stack
