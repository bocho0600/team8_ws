#!/bin/bash
# Smoke-test a built catkin_ws image: role dispatch, workspace/package
# resolution and launch-file validity. Runs in CI and locally:
#
#   ./ci/smoke-test.sh [image-tag]
#
# shellcheck disable=SC2016  # snippets are single-quoted on purpose: they must
#                              expand inside the container, not on the host.
set -uo pipefail

IMAGE="${1:-uavteam8/catkin_ws:latest}"
FAILED=0

pass() { printf '  ok    %s\n' "$1"; }
fail() { printf '  FAIL  %s\n' "$1"; FAILED=1; }

# Assert that a command run inside the image emits an expected substring.
# stderr is merged in so role warnings (which go to stderr) are assertable;
# the base image's sudo banner lands there too but is harmless for substring
# matching.
# Usage: expect <description> <expected> <role> <snippet>
expect() {
    local desc="$1" want="$2" role="$3" snippet="$4" got
    got="$(docker run --rm -e "ROLE=$role" -e UAV_IP=192.168.99.1 -e GCS_IP=192.168.99.2 \
        "$IMAGE" bash -ic "$snippet" 2>&1)"
    if [[ "$got" == *"$want"* ]]; then
        pass "$desc"
    else
        fail "$desc (wanted '$want', got '${got//$'\n'/ }')"
    fi
}

echo "Smoke-testing $IMAGE"

echo "- workspace"
expect "all catkin packages resolve" "actuator_control breadcrumb image_processing qutas_lab_450 spar_node uavasr_emulator" uav \
    'rospack list 2>/dev/null | grep -E "^(actuator_control|breadcrumb|image_processing|qutas_lab_450|spar_node|uavasr_emulator) " | cut -d" " -f1 | sort | tr "\n" " " | sed "s/ $//"'
expect "workspace overlays /opt/ros" "/home/uavteam8/catkin_ws/src:/opt/ros/noetic/share" uav \
    'echo $ROS_PACKAGE_PATH'
expect "runs as uavteam8" "uavteam8" uav 'whoami'

echo "- role dispatch"
# The UAV runs its own roscore, so it must NOT be pointed at UAV_IP.
expect "uav stays self-mastered" "http://localhost:11311" uav 'echo $ROS_MASTER_URI'
expect "gcs targets the UAV" "http://192.168.99.1:11311" gcs 'echo $ROS_MASTER_URI'
expect "invalid role warns" "ROLE is not" bogus 'true 2>&1'
expect "invalid role falls back to gcs" "http://192.168.99.1:11311" bogus 'echo $ROS_MASTER_URI'

echo "- role-scoped functions"
expect "uav defines run_uav_stack" "yes" uav 'type run_uav_stack >/dev/null 2>&1 && echo yes'
expect "uav has no gcs launcher" "yes" uav 'type gcs_tmux >/dev/null 2>&1 || echo yes'
expect "gcs defines gcs_tmux" "yes" gcs 'type gcs_tmux >/dev/null 2>&1 && echo yes'
expect "gcs defines gcs_tmux_sim" "yes" gcs 'type gcs_tmux_sim >/dev/null 2>&1 && echo yes'
expect "gcs has no uav launcher" "yes" gcs 'type run_uav_stack >/dev/null 2>&1 || echo yes'
expect "shared helpers on both roles" "yes" gcs \
    'type disros aruco_land aruco_roi aruco_land_point aruco_frames servo_open1 servo_open2 >/dev/null 2>&1 && echo yes'

echo "- launch files"
expect "control.launch resolves its includes" "mavros/launch/node.launch" uav \
    'roslaunch --files $CATKIN_WS/launch/control.launch 2>/dev/null'
expect "combined_nodes.launch parses" "combined_nodes.launch" uav \
    'roslaunch --files $CATKIN_WS/launch/combined_nodes.launch 2>/dev/null'
expect "breadcrumb.launch parses" "breadcrumb.launch" uav \
    'roslaunch --files $CATKIN_WS/launch/breadcrumb.launch 2>/dev/null'
expect "spar_uavasr.launch parses (gcs emulator)" "spar_uavasr.launch" gcs \
    'roslaunch --files $(rospack find spar_node)/launch/spar_uavasr.launch 2>/dev/null'
expect "environment.launch accepts vicon_server_dvp" "environment.launch" uav \
    'roslaunch --files $(rospack find qutas_lab_450)/launch/environment.launch vicon_server_dvp:=10.0.0.1 2>/dev/null'

# These used to come from the amd64-only osrf/*-desktop-full base. They are
# apt-installed now so the image can also build for the Pi's arm64 — guard
# that the swap didn't quietly drop any of them.
echo "- GCS GUI tooling"
expect "rviz installed" "/opt/ros/noetic/bin/rviz" gcs 'command -v rviz'
expect "rqt plugins resolve" "ok" gcs \
    'for p in rqt_gui rqt_gui_py rqt_generic_hud rqt_mavros_gui rqt_eyedropper rqt_quaternion_view; do rospack find $p >/dev/null 2>&1 || exit 1; done; echo ok'
expect "matplotlib present (rqt_quaternion_view)" "ok" gcs \
    'python3 -c "import matplotlib" && echo ok'
# `rospack find` passing says nothing about whether the plugin actually starts.
# Two things used to break only at launch time: a `#!/usr/bin/env python`
# shebang (Noetic ships python3 only, so rosrun died with "python: No such file
# or directory"), and rqt_py_common missing from the image.
expect "rqt scripts have a resolvable interpreter" "ok" gcs \
    'for p in rqt_generic_hud rqt_mavros_gui rqt_eyedropper rqt_quaternion_view; do
         command -v "$(head -1 "$(rospack find $p)/scripts/$p" | sed "s|^#!/usr/bin/env ||")" >/dev/null || exit 1
     done; echo ok'
expect "rqt plugin modules import" "ok" gcs \
    'for m in rqt_generic_hud.generic_hud_options rqt_mavros_gui.mavros_gui_options \
              rqt_quaternion_view.quaternion_view_options rqt_eyedropper.eyedrop; do
         python3 -c "import $m" 2>/dev/null || exit 1
     done; echo ok'

# These are imported at module scope, so a missing one doesn't degrade the node
# -- it kills it, and combined_nodes.launch comes up short a node. `rosdep
# install -r` will NOT catch that: it continues past unresolvable keys (it can't
# resolve tf_conversions at all), so these are installed explicitly.
echo "- node runtime imports"
expect "hardware/TF python modules present" "ok" uav \
    'for m in depthai pigpio tf_conversions; do python3 -c "import $m" 2>/dev/null || exit 1; done; echo ok'
# aruco_subscriber.py was written against the OpenCV 4.7+/5.0 ArucoDetector API;
# Noetic ships 4.2, which only has the free-function API. It must work on both.
expect "aruco detector builds on this OpenCV" "ok" uav \
    'python3 - <<'"'"'PY'"'"'
import importlib.util, numpy as np, os
p = os.environ["CATKIN_WS"] + "/src/image_processing/scripts/aruco_subscriber.py"
s = importlib.util.spec_from_file_location("aruco_subscriber", p)
m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
d = m.ArucoDetector.__new__(m.ArucoDetector)
d.detector = (__import__("cv2").aruco.ArucoDetector(m.ArucoDetector.aruco_dict,
              m.ArucoDetector.aruco_params) if m._HAS_ARUCO_DETECTOR else None)
d.detect_markers(np.zeros((240, 320, 3), dtype=np.uint8))
print("ok")
PY'

# The base image ships without netbase, so /etc/protocols is absent and
# getprotobyname("tcp") fails. vrpn_client_ros resolves protocols by name and
# dies on it -- "VRPN connection is not 'doing okay'" on loop, which reads like
# a network/IP fault and sends you debugging the wrong thing entirely.
echo "- network name databases"
expect "getprotobyname works (vrpn needs it)" "6" uav \
    'python3 -c "import socket; print(socket.getprotobyname(\"tcp\"))"'

echo "- tmux launchers are usable"
expect "tmux present" "tmux" uav 'command -v tmux'
# Regression guard: the kill-switch pane must be sent WITHOUT a C-m, or the
# session tears itself down the moment it starts.
expect "run_uav_stack kill pane is staged, not armed" "yes" uav \
    'declare -f run_uav_stack | grep send-keys | grep kill-session | grep -qv C-m && echo yes'
expect "gcs_tmux kill pane is staged, not armed" "yes" gcs \
    'declare -f gcs_tmux | grep send-keys | grep kill-session | grep -qv C-m && echo yes'
# Panes default to staged (pre-typed, press Enter to run); --run arms them and
# restores the staggered sleeps. tmux is stubbed so this is assertable headless.
expect "panes are staged by default" "yes" uav \
    'tmux() { echo "$*"; }; [ "$(_pane_cmd %0 5 "roslaunch foo")" = "send-keys -t %0 roslaunch foo" ] && echo yes'
expect "autorun arms panes with staggered sleeps" "yes" uav \
    'tmux() { echo "$*"; }; TMUX_AUTORUN=1; [ "$(_pane_cmd %0 5 "roslaunch foo")" = "send-keys -t %0 sleep 5; roslaunch foo C-m" ] && echo yes'
expect "--run flag parses, junk flags rejected" "1 0 bad" uav \
    'echo "$(_tmux_autorun --run) $(_tmux_autorun) $(_tmux_autorun --nope || echo bad)"'

if [ "$FAILED" -ne 0 ]; then
    echo "smoke test FAILED"
    exit 1
fi
echo "smoke test passed"
