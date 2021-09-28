ARG ROS_VERSION=foxy
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

RUN sudo pip3 install pybullet pyyaml zmq

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib
COPY --chown=${USER} ../pybullet_simulation ./pybullet_simulation
RUN pip3 install --editable ./pybullet_simulation

COPY --chown={USER} ../network-interfaces/python ./network_interfaces
RUN sudo pip3 install ./network_interfaces

WORKDIR ${HOME}
COPY --chown=${USER} ./robot_descriptions ./robot_descriptions
COPY --chown=${USER} ./pybullet_zmq/ ./pybullet_zmq/
RUN pip3 install --editable ./pybullet_zmq

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
