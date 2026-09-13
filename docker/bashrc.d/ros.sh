# Sourced from ~/.bashrc in every interactive container shell (see Dockerfile).
# Shared ROS setup and helpers, then the role-specific tmux launcher
# (uav.sh or gcs.sh) selected by ROLE in .env.

# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source ~/catkin_ws/devel/setup.bash

# Exported so the tmux launchers in uav.sh/gcs.sh (and the panes they spawn)
# see it too.
export CATKIN_WS="$HOME/catkin_ws"

# --- Role ------------------------------------------------------------------
# The two roles differ only in their default ROS master: the UAV runs its own
# roscore (so it stays self-mastered, no default), the GCS is a client of the
# UAV. Everything else below is shared.
case "$ROLE" in
  uav) export _ROS_DEFAULT_MASTER="" ;;
  gcs) export _ROS_DEFAULT_MASTER="$UAV_IP" ;;
  *)
    echo "ros.sh: ROLE is not 'uav' or 'gcs' (got: '${ROLE:-<unset>}') — run ./setup-env.sh. Defaulting to gcs." >&2
    ROLE=gcs
    export _ROS_DEFAULT_MASTER="$UAV_IP"
    ;;
esac

# --- Distributed ROS -------------------------------------------------------
# Auto-detects this machine's own IP and applies the role's default master.
# Pass an explicit master IP to override for one shell:
#   disros 192.168.1.50
# shellcheck disable=SC2120  # the master argument is intentionally optional
disros() {
  ROS_IP="$(hostname -I | cut -d' ' -f1)"
  export ROS_IP
  echo "Identifying as: $ROS_IP"

  local master="${1:-$_ROS_DEFAULT_MASTER}"
  if [ -n "$master" ]; then
    export ROS_MASTER_URI="http://$master:11311"
    echo "Connecting to: $ROS_MASTER_URI"
  fi
}

# --- Actuator + ArUco landing helpers ---------------------------------------
# Payload A / Payload B. These block until the rack has finished moving and
# print success/message, unlike the old fire-and-forget rostopic pub.
function servo_open1() { rosservice call /actuator_control/drop_a; }
function servo_open2() { rosservice call /actuator_control/drop_b; }

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

# --- tmux launcher helpers ---------------------------------------------------
# By default the tmux panes are pre-loaded with their command but don't run it,
# so you can bring the stack up piece by piece (press Enter in a pane to start
# it). Pass --run to a launcher (or set TMUX_AUTORUN=1) for the old
# fire-everything-at-once behaviour, where the staggered sleeps matter.
_pane_cmd() {
    local pane="$1" delay="$2" cmd="$3"
    if [ "${TMUX_AUTORUN:-0}" = "1" ]; then
        [ "${delay:-0}" -gt 0 ] && cmd="sleep ${delay}; ${cmd}"
        tmux send-keys -t "$pane" "$cmd" C-m
    else
        tmux send-keys -t "$pane" "$cmd"
    fi
}

# Shared launcher argument parsing: echoes the TMUX_AUTORUN value to use.
_tmux_autorun() {
    case "${1:-}" in
        --run|-r) echo 1 ;;
        "")       echo "${TMUX_AUTORUN:-0}" ;;
        *)        return 1 ;;
    esac
}

export -f disros servo_open1 servo_open2 _aruco_frame_arg aruco_land aruco_roi aruco_land_point aruco_frames
export -f _pane_cmd _tmux_autorun

# --- Role-specific tmux launcher --------------------------------------------
# shellcheck source=/dev/null
source "$(dirname "${BASH_SOURCE[0]}")/${ROLE}.sh"

# shellcheck disable=SC2119  # no argument means "use the role's default master"
disros
