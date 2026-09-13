# catkin_ws

ROS Noetic catkin workspace for the UAV (Team 8): MAVROS/PX4 flight control (`spar`), path planning (`breadcrumb`), vision (`image_processing`, `uavasr_emulator`), and rqt tooling.

## Quick start (Docker)

The workspace ships with a ready-to-use Docker environment — no need to install ROS Noetic, mavros, or any package dependencies on the host. You'll set this up on **two machines**: the Raspberry Pi on the drone (**UAV**) and a ground control station laptop (**GCS**). Both use the exact same repo and image — only `.env` differs between them.

Prerequisites on both machines:
- This repo cloned
- Both machines reachable from each other on the same network (LAN/WiFi)
- Docker installed **from Docker's apt repository, not the snap** — see below

### Installing Docker

Use the official apt packages. Two things to avoid:

- The Canonical **snap** build is strictly confined and blocks what this project needs — passing through the flight controller's serial device (`/dev/tty*`) and `privileged` mode. It also creates no `docker` group, so `usermod -aG docker` fails with `group 'docker' does not exist`.
- The `get.docker.com` convenience script **fails on Ubuntu 20.04 (focal)**, which is what the Pi runs for ROS Noetic. It tries to install `docker-model-plugin`, which isn't published for focal — and since `apt-get install` is atomic, that one missing package aborts the whole install, leaving nothing behind.

Set up the repo and install the packages explicitly instead:

```bash
sudo snap remove docker   # only if the snap is installed
sudo apt-get update && sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Then add yourself to the `docker` group (created by `docker-ce`'s post-install):

```bash
hash -r && sudo usermod -aG docker "$USER" && newgrp docker
docker run --rm hello-world
```

`newgrp docker` applies the group to the current shell; otherwise log out and back in. `hash -r` clears any path bash cached for a previously removed docker binary — without it you may see `/snap/bin/docker: No such file or directory` even after a successful install.

### On the UAV (Raspberry Pi)

1. **Configure.** Run the setup script with this machine's role:

   ```
   ./setup-env.sh uav
   ```

   It prompts for the three network values, pre-filling whatever was saved last time (the Vicon IP defaults to `10.68.42.85` the first time) — press Enter to keep a pre-filled value, or type over it:

   ```
   UAV IP address (Raspberry Pi): 192.168.1.42
   GCS IP address (ground station): 192.168.1.10
   Vicon server IP: 10.68.42.85
   ```

   This writes `.env` (gitignored — it's what `docker compose` reads) with `ROLE=uav` plus these three values. Re-run the script any time an IP changes; it pre-fills from the existing `.env`. Find the Pi's own IP with `hostname -I` if you don't have it handy.

2. **Connect the flight controller.** In [`docker-compose.yml`](docker-compose.yml), uncomment the `devices:` block and set the serial path your FCU actually enumerates as (check with `ls -l /dev/tty*` before and after plugging it in):

   ```yaml
   devices:
     - /dev/ttyAMA1:/dev/ttyAMA1   # Pi GPIO UART — control.launch's default
     # - /dev/ttyUSB0:/dev/ttyUSB0 # USB-attached FCU instead
   ```

   [`control.launch`](catkin_ws/launch/control.launch) defaults to `fcu_url:=/dev/ttyAMA1:921600` — if your FCU is on a different port/baud, either edit that default or pass `fcu_url:=...` when launching (step 4).

3. **Get the image.** Building on the Pi works but is slow (roughly 30–60 minutes, and ~4.3GB of SD card), so prefer the prebuilt arm64 image that CI publishes:

   ```
   docker pull ghcr.io/bocho0600/team8_ws:latest
   docker tag ghcr.io/bocho0600/team8_ws:latest uavteam8/catkin_ws:latest
   ```

   The tag makes `docker compose` use it as-is instead of rebuilding. To build locally anyway — after changing a `package.xml`, say:

   ```
   docker compose build
   ```

4. **Start the container:**

   ```
   docker compose run --rm catkin_ws
   ```

   This drops you into a shell as the `uavteam8` user, with the workspace already sourced and `ROLE=uav` applied — `disros` has already run once (see [Multi-machine setup](#multi-machine-setup-gcs--uav)), so this container is its own ROS master (it runs `roscore`, not the GCS).

5. **Launch.** Bring up the entire flight stack in one tmux session:

   ```
   run_uav_stack
   ```

   This opens 12 panes: `roscore`, a system monitor, a free terminal, `control.launch` (MAVROS + `spar`), `combined_nodes.launch` (vision + servo), `qutas_lab_450 environment.launch` (vicon/optitrack), `breadcrumb.launch`, the ArUco mission node, two position monitors, `rosbag record`, and a staged kill switch. See the [full pane table](#run_uav_stack-uav) for what each one does and how to sanity-check it's working.

   Every pane comes up **pre-typed but not running** — press Enter in a pane to start it, so you can bring the stack up one piece at a time (`roscore` first) and watch each node before adding the next. To fire everything at once instead, with the old staggered `sleep`s:

   ```
   run_uav_stack --run
   ```

   To stop everything, switch to the kill-switch pane (bottom-right) and press Enter — it's pre-typed but not run automatically, so it can't tear the stack down by accident.

   Prefer to launch things one at a time instead? Any launch file works normally, e.g.:

   ```
   roslaunch /home/uavteam8/catkin_ws/launch/control.launch fcu_url:=/dev/ttyUSB0:57600
   ```

### On the GCS (ground control laptop)

1. **Configure.** Same repo, same script, different role:

   ```
   ./setup-env.sh gcs
   ```

   Enter the same `UAV_IP` and `VICON_SERVER_DVP` you used on the UAV, and this laptop's own IP for `GCS_IP`. This writes its own `.env` with `ROLE=gcs` — the two machines' `.env` files aren't shared, they just need to agree on the same `UAV_IP` and `VICON_SERVER_DVP` values.

2. **Allow GUI passthrough** (for `rviz`/`rqt_*`, which run inside the container but display on your desktop):

   ```
   xhost +local:
   ```

3. **Build and start the container:**

   ```
   docker compose build
   docker compose run --rm catkin_ws
   ```

   Building on a laptop is quick enough that this is the default here; the prebuilt image from [Using the prebuilt image](#using-the-prebuilt-image) works on the GCS too if you'd rather skip it.

   With `ROLE=gcs`, `disros` has already pointed `ROS_MASTER_URI` at the UAV (`UAV_IP`) for you. Sanity-check the connection before launching anything:

   ```
   rostopic list
   ```

   If the UAV's `run_uav_stack` (or at least `roscore`) is already running, you should see its topics (e.g. `/mavros/state`) here. If this hangs or comes back empty, see [Troubleshooting connectivity](#troubleshooting-connectivity) below before continuing.

4. **Launch.** Two options depending on what you're doing:

   - **Monitoring/flying with the real UAV** — opens rviz, both rqt GUIs, position monitors, and bag recording against the UAV's `ROS_MASTER_URI`:

     ```
     gcs_tmux
     ```

   - **Testing mission logic standalone**, with no real UAV — runs the same tmux layout, but against the local software-in-the-loop emulator instead:

     ```
     gcs_tmux_sim
     ```

   See the [full pane table](#gcs_tmux--gcs_tmux_sim-gcs) for what each pane does. As with the UAV, panes are pre-typed but not started — press Enter in each to start it, or pass `--run` (`gcs_tmux --run`, `gcs_tmux_sim --run`) to start everything automatically. The kill switch is never auto-run either way.

### Troubleshooting connectivity

- `rostopic list` hangs or times out on the GCS → the UAV's `roscore` probably isn't running yet, or `UAV_IP` is wrong in one of the two `.env` files (they must match). Re-check with `hostname -I` on the UAV.
- Nodes register but no messages arrive → usually a `ROS_IP` mismatch or a firewall blocking the ROS TCP port range. Both containers use `network_mode: host`, so this is a host/network firewall issue, not a container one.
- Re-apply the role's default anytime with `disros` (no args); pass an explicit master IP to override for one shell, e.g. `disros 192.168.1.50`.

## Multi-machine setup (GCS + UAV)

This project runs across (at least) two machines — the UAV and the GCS — plus a Vicon/OptiTrack motion-capture rig. `.env` (generated by `./setup-env.sh`, gitignored — one copy per machine, with a different `ROLE`) tells the container about all of this.

`docker/bashrc.d/ros.sh` (loaded into every interactive shell) holds the shared ROS setup, `disros`, and the aruco/servo helpers, then sources `uav.sh` or `gcs.sh` for the role's tmux launcher. The only behavioral difference between roles is `disros`'s default master:

- **`ROLE=uav`** (`docker/bashrc.d/uav.sh`) — the UAV runs `roscore`, so it's its own ROS master by default; `disros` (no args) just re-detects `ROS_IP` and leaves `ROS_MASTER_URI` alone. Defines `run_uav_stack`.
- **`ROLE=gcs`** (`docker/bashrc.d/gcs.sh`) — the GCS is a client by default; `disros` (no args) points `ROS_MASTER_URI` at `UAV_IP`. Pass an explicit IP to override for one shell, e.g. `disros 192.168.1.50`. Defines `gcs_tmux` and `gcs_tmux_sim`.

Both roles share:
- **`GCS_IP`** — passed to MAVROS as `gcs_url` (`udp://@$GCS_IP:14550`) by `run_uav_stack`, so QGroundControl/telemetry on the GCS can connect.
- **`VICON_SERVER_DVP`** — passed to `qutas_lab_450`'s `environment.launch` as `vicon_server_dvp`.

#### `run_uav_stack` (UAV)

Opens a `uav_stack` tmux session with:

| Pane | Command | How to tell it's working |
|---|---|---|
| roscore | `roscore` | Starts immediately; other panes will fail to register nodes if this dies |
| system monitor | `htop` | — |
| free terminal | — | — |
| flight control | `control.launch` (MAVROS + `spar`, with `gcs_url` set from `GCS_IP`) | Look for `FCU: DeviceError` if the serial device/path is wrong; once connected, `rostopic echo /mavros/state` shows `connected: True` |
| vision + servo | `combined_nodes.launch` | Check for camera/detector errors in this pane's output |
| vicon/optitrack + grid | `qutas_lab_450 environment.launch` (with `vicon_server_dvp` set from `VICON_SERVER_DVP`) | "Connection established" means the VRPN client reached the mocap server |
| path planner | `breadcrumb.launch` | Runs standalone, no external dependency |
| ArUco mission node | `rosrun spar_node demo_ml` | Prints "Waiting for SPAR..." then "Connected to SPAR." once `control.launch` is up |
| local position monitor | `rostopic echo /mavros/local_position/pose` | Should start printing poses once MAVROS is connected |
| vision pose monitor | `rostopic echo /mavros/vision_pose/pose` | Should start printing once the vicon/optitrack bridge is up |
| bag recording | `rosbag record -a` (written to `~/catkin_ws/bags/`) | — |
| kill switch | `tmux kill-session -t uav_stack` staged, not run | Press Enter in this pane to tear the whole stack down |

By default every pane is pre-typed but idle — the table's order is the order to press Enter in. With `--run` (or `TMUX_AUTORUN=1`) the launches fire automatically, staggered with a `sleep` so `roscore` and MAVROS are up before dependents start.

#### `gcs_tmux` / `gcs_tmux_sim` (GCS)

Opens a `gcs_stack` tmux session:

| Pane | Command | How to tell it's working |
|---|---|---|
| flight emulator | `roslaunch spar_node spar_uavasr.launch` (spar node + `uavasr_emulator`) | Only meaningful under `gcs_tmux_sim` (needs a local master); under `gcs_tmux` it launches against whatever `ROS_MASTER_URI` already points at |
| bag recording | `rosbag record -a` (written to `~/catkin_ws/bags/`) | — |
| local position monitor | `rostopic echo /mavros/local_position/pose` | Should print poses once connected to a master that has MAVROS running |
| vision pose monitor | `rostopic echo /mavros/vision_pose/pose` | Same as above |
| rviz | `rviz -d ~/catkin_ws/src/spar/spar_node/rviz/emulator_configuration.rviz`¹ | Needs `xhost +local:` run first on the host |
| HUD | `rosrun rqt_generic_hud rqt_generic_hud` | Needs `xhost +local:` too |
| MAVROS GUI | `rosrun rqt_mavros_gui rqt_mavros_gui` | Needs `xhost +local:` too |
| kill switch | `tmux kill-session -t gcs_stack` staged, not run | Press Enter in this pane to tear the whole stack down |

¹ This rviz config isn't checked into the repo yet — add `src/spar/spar_node/rviz/emulator_configuration.rviz` before relying on this pane.

`gcs_tmux` uses whatever `ROS_MASTER_URI` is already set (the UAV, by default). `gcs_tmux_sim` instead points ROS at yourself first (`disros $(hostname -I)`) before starting the same tmux stack, for standalone testing against the emulator with no real UAV involved.

**Helper functions**, available in any pane/shell once the ArUco mission node (`demo_ml`) is running:

- `servo_open1` / `servo_open2` — publish to `/actuator_control/actuator_a` to trigger the payload servo
- `aruco_land <marker_id|frame>` / `aruco_roi <marker_id|frame>` — call the `/aruco/land` / `/aruco/roi` services
- `aruco_land_point <x> <y> <z>` — publish a manual landing point override
- `aruco_frames` — list detected `target_*` TF frames

**Other launch files** in [`launch/`](catkin_ws/launch/):

| File | What it starts |
|---|---|
| `control.launch` | MAVROS + `spar` node (flight control) |
| `combined_nodes.launch` | Vision pipeline (YOLO detector, ArUco, tf2 broadcasters) + servo actuator node |
| `breadcrumb.launch` | `breadcrumb` path-planning node |

Run any of them the same way: `roslaunch /home/uavteam8/catkin_ws/launch/<file>.launch`.

## CI / CD

Two GitHub Actions workflows:

- **[`ci.yml`](.github/workflows/ci.yml)** — runs on every push and PR. Shellchecks the scripts, exercises `setup-env.sh` (including the regression where the role argument used to be clobbered by an existing `.env`), builds the image, and runs [`ci/smoke-test.sh`](ci/smoke-test.sh) against it.
- **[`publish.yml`](.github/workflows/publish.yml)** — on pushes to `main` and `v*` tags, builds `linux/amd64` and `linux/arm64` on **native runners** (no QEMU) and pushes a multi-arch manifest to `ghcr.io/bocho0600/team8_ws`.

`ci/smoke-test.sh` also runs locally against any built image:

```bash
./ci/smoke-test.sh uavteam8/catkin_ws:latest
```

It asserts the things that are easy to break silently: role dispatch (UAV stays self-mastered, GCS targets `UAV_IP`, bad role warns and falls back), that each role gets only its own launcher, that every launch file still resolves its includes, that the GUI tooling is present, and that the tmux panes (the kill switch always, every other pane unless `--run` is passed) stay *staged* rather than armed.

### Using the prebuilt image

Once `publish.yml` has run, either machine can skip the local build:

```bash
docker pull ghcr.io/bocho0600/team8_ws:latest
```

Docker picks the right architecture automatically. To use it instead of building, point `image:` in [`docker-compose.yml`](docker-compose.yml) at that tag and drop the `build:` block. Building locally stays the default so source edits don't need a round trip through CI.

### Notes

- The image is based on `ros:noetic-perception`, which publishes amd64 **and** arm64 — the `osrf/ros:noetic-desktop-*` tags are amd64-only and cannot be built on the Pi. rviz and the rqt plugins are apt-installed on top instead of coming from the base.
- `network_mode: host` is used so mavros/GCS/vrpn can reach other machines on the LAN, the same as running directly on the Pi.
- `./src` and `./launch` are bind-mounted, so host edits are picked up without rebuilding the image. What each kind of change actually costs:
  - **Launch/YAML files** — nothing; they're read at `roslaunch` time. Restart the node.
  - **Python nodes** — nothing; `catkin_install_python` puts a relay stub in `devel/lib/<pkg>/` that `exec`s the mounted source file at runtime, so it always runs your current copy. Restart the node.
  - **C++ nodes** — `catkin_make` inside the container (not an image rebuild). Use a persistent container (`docker compose up -d` + `docker compose exec catkin_ws bash`) so `devel/` survives; `docker compose run --rm` discards the compile on exit.
  - **`package.xml` / `CMakeLists.txt` dependencies** — `docker compose build`, since new apt/rosdep packages have to be installed into the image.
- `./docker/bashrc.d` is bind-mounted read-only over the copy baked into the image, so edits to the shell helpers (`disros`, `run_uav_stack`/`gcs_tmux`, the aruco/servo helpers) apply in the *next* shell — no rebuild, but existing shells and their tmux panes keep the old definitions until you exit and re-enter.
- Serial devices (flight controller, etc.) are commented out in [`docker-compose.yml`](docker-compose.yml) by default — see step 2 of the [UAV setup](#on-the-uav-raspberry-pi) above.
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
