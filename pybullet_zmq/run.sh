#!/usr/bin/env bash

aica-docker interactive aica-technology/zmq-simulator --net host --no-hostname  -v "$(pwd)"/pybullet_zmq:/home/ros2/pybullet_zmq/pybullet_zmq 

# -v "$(pwd)"/path/to/host_folder:/path/to/docker_folder
