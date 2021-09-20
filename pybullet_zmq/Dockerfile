ARG ROS_VERSION=foxy
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

ARG ROS_VERSION=foxy
RUN sudo pip3 install pybullet pyyaml zmq

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib
COPY --chown=${USER} ../pybullet_simulation ./pybullet_simulation
RUN pip3 install --editable ./pybullet_simulation

WORKDIR ${HOME}
COPY --chown=${USER} ./pybullet_zmq/pybullet_zmq/ ./pybullet_zmq/
COPY --chown=${USER} ./robot_descriptions ./pybullet_zmq/robot_descriptions

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
