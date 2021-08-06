#!/usr/bin/env bash

IMAGE_NAME=aica-technology/simulator-backend

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

if [[ "$OSTYPE" != "darwin"* ]]; then
  USER_ID="$(id -u "${USER}")"
  GROUP_ID="$(id -g "${USER}")"
  BUILD_FLAGS+=(--build-arg UID="${USER_ID}")
  BUILD_FLAGS+=(--build-arg GID="${GROUP_ID}")
fi

BUILD_FLAGS+=(-t "${IMAGE_NAME}:latest")

docker pull ghcr.io/aica-technology/ros2-ws:foxy
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit