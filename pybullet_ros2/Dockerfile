ARG ROS_VERSION=foxy
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

ARG ROS_VERSION=foxy
RUN sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_VERSION}-xacro \
    ros-${ROS_VERSION}-joint-state-publisher-gui \
    && sudo rm -rf /var/lib/apt/lists/*

# install pybullet
RUN sudo pip3 install pybullet pyyaml

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib
COPY --chown=${USER} ../pybullet_simulation ./pybullet_simulation
RUN pip3 install --editable ./pybullet_simulation

WORKDIR ${HOME}/ros2_ws/
RUN cd src && git clone -b develop --single-branch https://github.com/aica-technology/franka_panda_description.git
COPY --chown=${USER} ./pybullet_ros2/ ./src/pybullet_ros2/
RUN su ${USER} -c /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash; colcon build --symlink-install"

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
