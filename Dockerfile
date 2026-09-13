FROM osrf/ros:noetic-desktop-full

ARG USERNAME=uavteam8
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# --- System deps ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-pip \
        python3-rosdep \
        python3-catkin-tools \
        python3-osrf-pycommon \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-vrpn-client-ros \
        ros-noetic-topic-tools \
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
