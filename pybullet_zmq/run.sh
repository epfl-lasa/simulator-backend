#!/usr/bin/env bash

docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   --net host \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $HOME/.Xauthority:/home/ros/.Xauthority \
	   aica-technology/zmq-simulator

# -v "$(pwd)"/path/to/host_folder:/path/to/docker_folder
