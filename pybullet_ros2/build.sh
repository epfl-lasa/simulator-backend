#!/usr/bin/env bash
ROS_VERSION=galactic

IMAGE_NAME=aica-technology/ros2-simulator
IMAGE_TAG="${ROS_VERSION}"

# BUILD
BUILD_FLAGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    -r) BUILD_FLAGS+=(--no-cache); shift 1;;
    *) echo 'Error in command line parsing' >&2
       exit 1
  esac
done
shift "$(( OPTIND - 1 ))"

BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

docker pull ghcr.io/aica-technology/ros2-control-libraries:"${ROS_VERSION}"
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" --file ./Dockerfile .. || exit
