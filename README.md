# catkin_ws

ROS Noetic catkin workspace for the UAV (Team 8): MAVROS/PX4 flight control (`spar`), path planning (`breadcrumb`), vision (`image_processing`, `uavasr_emulator`), and rqt tooling.

## Quick start (Docker)

The workspace ships with a ready-to-use Docker environment — no need to install ROS Noetic, mavros, or any package dependencies on the host.

**1. Configure your machine:**

```
cp .env.example .env
```

Fill in the four values in `.env` (Docker Compose reads it automatically — see [Multi-machine setup](#multi-machine-setup-gcs--uav) below for how they're used). `ROLE` differs per machine; the rest are the same on both:

| Variable | What it is |
|---|---|
| `ROLE` | `uav` or `gcs` — picks which tmux launcher/`disros` default this machine gets |
| `UAV_IP` | IP of the Raspberry Pi on the drone (runs `roscore`) |
| `GCS_IP` | IP of the ground control station laptop |
| `VICON_SERVER_DVP` | Motion-capture (Vicon) server address |

**2. Build the image:**

```
docker compose build
```

**3. Start the container:**

```
docker compose run --rm catkin_ws
```

This drops you into a shell inside the container, running as the `uavteam8` user with the workspace already sourced (`ROS_PACKAGE_PATH` and `devel/setup.bash` set up) and `ROS_IP`/`ROS_MASTER_URI` already pointed at the UAV.

**4. Launch the flight stack:**

- **On the UAV**, bring up the entire stack (roscore, MAVROS/spar, vision, path planner, vicon/optitrack, ArUco mission node, position monitors, bag recording) in one tmux session:

  ```
  run_uav_stack
  ```

  See [Multi-machine setup](#multi-machine-setup-gcs--uav) for what each pane does.

- **Individually**, any launch file works the normal way, e.g.:

  ```
  roslaunch /home/uavteam8/catkin_ws/launch/control.launch
  ```

  Override the FCU connection at launch time if needed:

  ```
  roslaunch /home/uavteam8/catkin_ws/launch/control.launch fcu_url:=/dev/ttyUSB0:57600
  ```

## Multi-machine setup (GCS + UAV)

This project runs across (at least) two machines — the Raspberry Pi on the drone (UAV) and a ground control station laptop (GCS) — plus a Vicon/OptiTrack motion-capture rig. `.env` (copied from `.env.example`, gitignored — one copy per machine, with a different `ROLE`) tells the container about all of this.

`ros.sh` (loaded into every interactive shell) sources shared setup from `docker/bashrc.d/common.sh`, then `uav.sh` or `gcs.sh` based on `ROLE` — each defines its own `disros` default, since the two roles want opposite behavior:

- **`ROLE=uav`** (`docker/bashrc.d/uav.sh`) — the UAV runs `roscore`, so it's its own ROS master by default; `disros` (no args) just re-detects `ROS_IP` and leaves `ROS_MASTER_URI` alone. Defines `run_uav_stack`.
- **`ROLE=gcs`** (`docker/bashrc.d/gcs.sh`) — the GCS is a client by default; `disros` (no args) points `ROS_MASTER_URI` at `UAV_IP`. Pass an explicit IP to override for one shell, e.g. `disros 192.168.1.50`. Defines `gcs_tmux` and `gcs_tmux_sim`.

Both roles share:
- **`GCS_IP`** — passed to MAVROS as `gcs_url` (`udp://@$GCS_IP:14550`) by `run_uav_stack`, so QGroundControl/telemetry on the GCS can connect.
- **`VICON_SERVER_DVP`** — passed to `qutas_lab_450`'s `environment.launch` as `vicon_server_dvp`.

**`run_uav_stack`** (run on the UAV) opens a `uav_stack` tmux session with:

| Pane | Command |
|---|---|
| roscore | `roscore` |
| system monitor | `htop` |
| free terminal | — |
| flight control | `control.launch` (MAVROS + `spar`, with `gcs_url` set from `GCS_IP`) |
| vision + servo | `combined_nodes.launch` |
| vicon/optitrack + grid | `qutas_lab_450 environment.launch` (with `vicon_server_dvp` set from `VICON_SERVER_DVP`) |
| path planner | `breadcrumb.launch` |
| ArUco mission node | `rosrun spar_node demo_ml` |
| local position monitor | `rostopic echo /mavros/local_position/pose` |
| vision pose monitor | `rostopic echo /mavros/vision_pose/pose` |
| bag recording | `rosbag record -a` (written to `~/catkin_ws/bags/`) |
| kill switch | `tmux kill-session -t uav_stack` staged, not run — press Enter in that pane to tear the whole stack down |

Each launch is staggered with a `sleep` so `roscore` and MAVROS are up before dependents start.

**`gcs_tmux`** (run on the GCS) opens a `gcs_stack` tmux session for testing mission logic against the software-in-the-loop emulator, without needing the real UAV:

| Pane | Command |
|---|---|
| flight emulator | `roslaunch spar_node spar_uavasr.launch` (spar node + `uavasr_emulator`) |
| bag recording | `rosbag record -a` (written to `~/catkin_ws/bags/`) |
| local position monitor | `rostopic echo /mavros/local_position/pose` |
| vision pose monitor | `rostopic echo /mavros/vision_pose/pose` |
| rviz | `rviz -d ~/catkin_ws/src/spar/spar_node/rviz/emulator_configuration.rviz`¹ |
| HUD | `rosrun rqt_generic_hud rqt_generic_hud` |
| MAVROS GUI | `rosrun rqt_mavros_gui rqt_mavros_gui` |
| kill switch | `tmux kill-session -t gcs_stack` staged, not run |

¹ This rviz config isn't checked into the repo yet — add `src/spar/spar_node/rviz/emulator_configuration.rviz` before relying on this pane.

`gcs_tmux` uses whatever `ROS_MASTER_URI` is already set (the UAV, by default). For standalone testing against the emulator with no real UAV involved, run `gcs_tmux_sim` instead — it points ROS at yourself first (`disros $(hostname -I)`), then starts the same tmux stack.

**Helper functions**, available in any pane/shell once the ArUco mission node (`demo_ml`) is running:

- `servo_open1` / `servo_open2` — publish to `/actuator_control/actuator_a` to trigger the payload servo
- `aruco_land <marker_id|frame>` / `aruco_roi <marker_id|frame>` — call the `/aruco/land` / `/aruco/roi` services
- `aruco_land_point <x> <y> <z>` — publish a manual landing point override
- `aruco_frames` — list detected `target_*` TF frames

**Other launch files** in [`launch/`](launch/):

| File | What it starts |
|---|---|
| `control.launch` | MAVROS + `spar` node (flight control) |
| `combined_nodes.launch` | Vision pipeline (YOLO detector, ArUco, tf2 broadcasters) + servo actuator node |
| `breadcrumb.launch` | `breadcrumb` path-planning node |

Run any of them the same way: `roslaunch /home/uavteam8/catkin_ws/launch/<file>.launch`.

### Notes

- `network_mode: host` is used so mavros/GCS/vrpn can reach other machines on the LAN, the same as running directly on the Pi.
- `./src` and `./launch` are bind-mounted, so source edits on the host are picked up without rebuilding — rebuild (`docker compose build`) whenever a `package.xml` or `CMakeLists.txt` dependency changes.
- X11 is wired up (`DISPLAY` + `/tmp/.X11-unix`) for `rqt_*`/rviz GUIs — run `xhost +local:` on the host first so the container is allowed to connect.
- Serial devices (flight controller, etc.) are commented out in [`docker-compose.yml`](docker-compose.yml) — uncomment and adjust device paths for your host.
- `RPi.GPIO` (used by `actuator_control`) is skipped during the image's `rosdep install` since it only installs against real Raspberry Pi hardware — that package still builds, it just can't drive real GPIO pins from inside the container. `fake-rpi` is installed for anyone who wants to `import RPi.GPIO` for off-Pi simulation.

## Building without Docker

If you're already on a Raspberry Pi (or another machine) with ROS Noetic installed:

```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
roslaunch launch/control.launch
```
