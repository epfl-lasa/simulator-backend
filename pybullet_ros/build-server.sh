#!/usr/bin/env bash
ROS_VERSION=noetic

IMAGE_NAME=aica-technology/ros-simulator
IMAGE_TAG="${ROS_VERSION}"

SERVE_REMOTE=false
REMOTE_SSH_PORT=7772

HELP_MESSAGE="Usage: build-server.sh [-r] [-v] [-s]
Options:
  -r, --rebuild                   Rebuild the image using the docker
                                  --no-cache option.

  -v, --verbose                   Use the verbose option during the building
                                  process.

  -s, --serve                     Start the remove development server.

  -h, --help                      Show this help message.
"

BUILD_FLAGS=()
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -r|--rebuild) BUILD_FLAGS+=(--no-cache) ; shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain) ; shift ;;
    -s|--serve) SERVE_REMOTE=true ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

docker pull ghcr.io/aica-technology/ros-control-libraries:"${ROS_VERSION}"
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" --file ./Dockerfile .. || exit

if [ "${SERVE_REMOTE}" = true ]; then
  aica-docker server "${IMAGE_NAME}:${IMAGE_TAG}" -u ros -p "${REMOTE_SSH_PORT}"
fi




