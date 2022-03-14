FROM ghcr.io/aica-technology/ros-control-libraries:noetic

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
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
