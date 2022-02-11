ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION} as pybullet

RUN sudo pip3 install pybullet


FROM pybullet as urdf-creator

RUN sudo apt-get update && sudo apt-get install -y python3-tk && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
USER root
RUN --mount=type=ssh git clone -b develop \
    --depth 1 git@github.com:aica-technology/urdf-creator.git ./urdf-creator
RUN chown -R ${USER}:${USER} ./urdf-creator
USER ${USER}

WORKDIR /usr/local/src
RUN sudo mv /tmp/urdf-creator/source/urdf_creator ./urdf-creator
RUN sudo pip3 install ./urdf-creator
RUN rm -rf /tmp/urdf-creator


FROM urdf-creator as network-interfaces

WORKDIR /tmp
RUN git clone -b develop --depth 1 https://github.com/aica-technology/network-interfaces.git && cd network-interfaces && \
  sudo bash install.sh --auto --no-cpp
RUN rm -rf /tmp/network-interfaces


FROM network-interfaces as robot-descriptions

WORKDIR ${ROS2_WORKSPACE}
COPY --chown=${USER} ./robot_descriptions/ ./src/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"


FROM robot-descriptions as install-source

WORKDIR ${HOME}
COPY --chown=${USER} ./pybullet_simulation ./pybullet_simulation
RUN sudo pip3 install --editable ./pybullet_simulation

WORKDIR ${HOME}
COPY --chown=${USER} ./pybullet_zmq/ ./pybullet_zmq/
RUN sudo pip3 install --editable ./pybullet_zmq

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
