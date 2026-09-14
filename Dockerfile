# ros:noetic-perception is multi-arch (amd64 + arm64 + armv7) and ships
# vision_opencv/cv_bridge, which image_processing needs. The osrf/*-desktop-*
# tags are amd64-only, so they can't be built on the Raspberry Pi — the GUI
# tools they used to provide are apt-installed below instead.
FROM ros:noetic-perception

ARG USERNAME=uavteam8
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# --- System deps ---------------------------------------------------------
# rosdep (below) resolves everything declared in the package.xml files; the
# heavy, rarely-changing packages are listed here so they land in a layer
# that survives source edits.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        # /etc/protocols and /etc/services. The ros:noetic-perception base
        # omits netbase, so getprotobyname("tcp") fails -- which breaks any
        # library that resolves protocols by name rather than by number.
        # vrpn_client_ros does, and dies with "vrpn_poll_for_accept:
        # getprotobyname() failed" then warns "VRPN connection is not
        # 'doing okay'" forever, no matter what server IP it is given.
        netbase \
        python3-pip \
        python3-rosdep \
        python3-catkin-tools \
        python3-osrf-pycommon \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-vrpn-client-ros \
        ros-noetic-topic-tools \
        ros-noetic-tf-conversions \
        ros-noetic-rviz \
        ros-noetic-rqt-gui \
        ros-noetic-rqt-gui-py \
        ros-noetic-rqt-py-common \
        ros-noetic-image-transport-plugins \
        python3-matplotlib \
        python3-pigpio \
        tmux \
        htop \
    && rm -rf /var/lib/apt/lists/*

# GeographicLib datasets required by mavros for local/global position conversions
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# --- Non-root user matching the paths hardcoded in launch/control.launch --
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && usermod -aG dialout,video,sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

ENV CATKIN_WS=/home/$USERNAME/catkin_ws
WORKDIR $CATKIN_WS

# --- Install package dependencies from source before copying full tree ----
# (keeps the rosdep layer cached across source-only edits)
COPY --chown=$USER_UID:$USER_GID catkin_ws/src ./src
RUN . /opt/ros/noetic/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys="RPi.GPIO" \
    && rm -rf /var/lib/apt/lists/*

# RPi.GPIO itself only installs on real Raspberry Pi hardware, so it's skipped
# above. actuator_control still builds (pure Python, no compiled extension);
# it just can't be run for real outside a Pi. fake-rpi is available for
# anyone who wants to import RPi.GPIO for off-Pi simulation.
RUN pip3 install fake-rpi

# depthai drives the OAK-D camera (dai_publisher_yolov11_runner.py). Only
# useful on the Pi with the camera attached, but it has to import everywhere
# or combined_nodes.launch loses the node outright. --only-binary matters:
# without it pip falls back to the source tarball and compiles the whole C++
# SDK, which turns a 14MB download into a very long build on the Pi. Wheels
# exist for both x86_64 and aarch64.
RUN pip3 install --only-binary=:all: depthai

COPY --chown=$USER_UID:$USER_GID catkin_ws/ .
RUN chown -R $USER_UID:$USER_GID $CATKIN_WS

COPY --chown=$USER_UID:$USER_GID docker/entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x /home/$USERNAME/entrypoint.sh

# ROS environment + role-specific (uav.sh/gcs.sh, picked by ROLE) tmux
# launcher + shared aruco/servo helpers, loaded into every interactive shell
# (tmux panes spawned from one inherit it too).
COPY --chown=$USER_UID:$USER_GID docker/bashrc.d/ /home/$USERNAME/.bashrc.d/
RUN echo '[ -f ~/.bashrc.d/ros.sh ] && source ~/.bashrc.d/ros.sh' >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN . /opt/ros/noetic/setup.sh && catkin_make

ENTRYPOINT ["/home/uavteam8/entrypoint.sh"]
CMD ["bash"]
