FROM ghcr.io/aica-technology/ros-control-libraries:noetic

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install pybullet
RUN sudo pip3 install pybullet pyyaml

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib
COPY --chown=${USER} ../pybullet_simulation ./pybullet_simulation
RUN pip3 install --editable ./pybullet_simulation

WORKDIR ${HOME}/ros_ws
RUN cd src && git clone -b noetic --single-branch https://github.com/aica-technology/franka_panda_description.git
RUN chown -R ${USER} ./src/franka_panda_description
COPY --chown=${USER} ./pybullet_ros/pybullet_ros ./src/pybullet_ros
RUN su ${USER} -c /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
