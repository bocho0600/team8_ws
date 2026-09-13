# catkin_ws

ROS Noetic catkin workspace for the UAV (Team 8): MAVROS/PX4 flight control (`spar`), path planning (`breadcrumb`), vision (`image_processing`, `uavasr_emulator`), and rqt tooling.

## Quick start (Docker)

The workspace ships with a ready-to-use Docker environment — no need to install ROS Noetic, mavros, or any package dependencies on the host.

**1. Build the image:**

```
docker compose build
```

**2. Start the container:**

```
docker compose run --rm catkin_ws
```

This drops you into a shell inside the container, running as the `uavteam8` user with the workspace already sourced (`ROS_PACKAGE_PATH` and `devel/setup.bash` set up).

**3. Launch the flight stack:**

```
roslaunch /home/uavteam8/catkin_ws/launch/control.launch
```

This brings up MAVROS (connecting to the flight controller over `fcu_url`, default `/dev/ttyAMA1:921600`) and the `spar` node. Override the FCU connection at launch time if needed, e.g.:

```
roslaunch /home/uavteam8/catkin_ws/launch/control.launch fcu_url:=/dev/ttyUSB0:57600
```

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
