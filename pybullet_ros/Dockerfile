FROM ghcr.io/aica-technology/ros-control-libraries:noetic

# install pybullet
RUN sudo pip3 install pybullet pyyaml

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib
COPY --chown=${USER} ../pybullet_simulation ./pybullet_simulation
RUN pip3 install --editable ./pybullet_simulation

WORKDIR ${HOME}/ros_ws/src
RUN git clone -b melodic-devel --single-branch https://github.com/RobotnikAutomation/robotnik_sensors.git
RUN git clone -b melodic-devel --single-branch https://github.com/RobotnikAutomation/summit_xl_common.git && \
  mv summit_xl_common/summit_xl_description ./summit_xl_description && rm -rf summit_xl_common
COPY --chown=${USER} ./rbkairos_common/rbkairos_description ./rbkairos_description

COPY --chown=${USER} ./robot_descriptions ./
COPY --chown=${USER} ./pybullet_ros/pybullet_ros ./pybullet_ros

WORKDIR ${HOME}/ros_ws
RUN su ${USER} -c /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
