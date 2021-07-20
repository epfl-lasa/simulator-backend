#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

IMAGE_NAME=simulator-backend
MULTISTAGE_TARGET=develop
CONTAINER_NAME="${IMAGE_NAME/\//-}"
CONTAINER_NAME="${CONTAINER_NAME/:/-}-${MULTISTAGE_TARGET}"

bash ${PWD}/build.sh --target "${MULTISTAGE_TARGET}"

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/../pybullet_ros2" \
    --opt o="bind" \
    "${IMAGE_NAME}_ros2_pkg_vol"

RUN_FLAGS=(-u ros2)
if [[ "$OSTYPE" == "darwin"* ]]; then
  RUN_FLAGS+=(-e DISPLAY=host.docker.internal:0)
else
  xhost +
  RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
  RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
  RUN_FLAGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  RUN_FLAGS+=(-v "${IMAGE_NAME}"_ros2_pkg_vol:/home/ros2/ros2_ws/src/pybullet_ros2)
fi

xhost +
docker run -it --rm --privileged \
  ${GPU_FLAG} \
  "${RUN_FLAGS[@]}" \
  --hostname "${CONTAINER_NAME}" \
  "${IMAGE_NAME}:${MULTISTAGE_TARGET}" /bin/bash
