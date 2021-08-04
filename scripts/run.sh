#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

IMAGE_NAME=aica-technology/simulator-backend
MULTISTAGE_TARGET=runtime
CONTAINER_NAME="${IMAGE_NAME/\//-}"
CONTAINER_NAME="${CONTAINER_NAME/:/-}-${MULTISTAGE_TARGET}"

bash ${PWD}/build.sh --target "${MULTISTAGE_TARGET}"

# RUN
[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

RUN_FLAGS=(-u ros2)
if [[ "$OSTYPE" == "darwin"* ]]; then
  RUN_FLAGS+=(-e DISPLAY=host.docker.internal:0)
else
  xhost +
  RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
  RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
  RUN_FLAGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

docker run -it --rm --privileged \
  ${GPU_FLAG} \
  "${RUN_FLAGS[@]}" \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_NAME}" \
  "${IMAGE_NAME}:${MULTISTAGE_TARGET}" /bin/bash
