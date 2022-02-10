ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

RUN sudo pip3 install pybullet

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib
RUN git clone -b develop --depth 1 https://github.com/aica-technology/network-interfaces.git && cd network-interfaces && \
  sudo bash install.sh --auto --no-cpp
RUN rm -r network-interfaces
COPY --chown=${USER} ../pybullet_simulation ./pybullet_simulation
RUN pip3 install --editable ./pybullet_simulation

WORKDIR ${HOME}
COPY --chown=${USER} ./robot_descriptions ./robot_descriptions
COPY --chown=${USER} ./pybullet_zmq/ ./pybullet_zmq/
RUN pip3 install --editable ./pybullet_zmq

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
