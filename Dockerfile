ARG ROS_DISTRO=foxy
FROM aica-technology/ros2-ws:${ROS_DISTRO} as core-dependencies

ENV DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && sudo rm -rf /var/lib/apt/lists/*

# install pybullet
RUN sudo pip3 install pybullet


FROM core-dependencies AS workspace

WORKDIR ${HOME}/ros2_ws/
RUN cd src && git clone -b ros2/foxy --single-branch https://github.com/domire8/franka_panda_description.git
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --symlink-install"


# ros user with everything pre-built
FROM workspace AS runtime

COPY --chown=${USER} ./pybullet_ros2/ ./src/pybullet_ros2/
RUN cd ${HOME}/ros2_ws && /bin/bash -c "source ${HOME}/ros2_ws/install/setup.bash; colcon build --symlink-install"

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# start as the user on default login unless the CMD is overridden.
CMD su --login ${USER}


# dev user to be used with shared volume
FROM workspace AS develop

#ENV USER ros2
#ENV HOME /home/${USER}
#
#ARG UID=1000
#ARG GID=1000
#RUN groupmod --gid ${GID} ${USER}
#RUN usermod --uid ${UID} ${USER}
#USER ${USER}

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# start as the user on default login unless the CMD is overridden.
CMD su --login ${USER}
