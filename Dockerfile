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
RUN su ${USER} -c /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --symlink-install"

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*


# ros user with everything pre-built
FROM workspace AS runtime

COPY --chown=${USER} ./pybullet_ros2/ ./src/pybullet_ros2/
RUN su ${USER} -c /bin/bash -c "source ${HOME}/ros2_ws/install/setup.bash; colcon build --symlink-install"
