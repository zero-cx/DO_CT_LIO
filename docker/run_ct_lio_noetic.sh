#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-ct-lio-noetic:local}"
CONTAINER_NAME="${CONTAINER_NAME:-ct-lio-noetic}"
HOST_WS="${HOST_WS:-/home/ubuntu20/CT_LIO_ws}"
CONTAINER_USER="${USER:-ubuntu20}"

if docker info >/dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

EXISTING_CONTAINER="$("${DOCKER[@]}" ps -aq --filter "name=^/${CONTAINER_NAME}$")"
if [[ -n "${EXISTING_CONTAINER}" ]]; then
  if [[ "$("${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")" == "true" ]]; then
    exec "${DOCKER[@]}" exec -it "${CONTAINER_NAME}" bash
  fi

  echo "Container '${CONTAINER_NAME}' already exists but is not running."
  echo "Remove it first with: ${DOCKER[*]} rm ${CONTAINER_NAME}"
  exit 1
fi

"${DOCKER[@]}" run --rm -it \
  --name "${CONTAINER_NAME}" \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOST_WS}:/home/${CONTAINER_USER}/CT_LIO_ws:rw" \
  -w "/home/${CONTAINER_USER}/CT_LIO_ws" \
  "${IMAGE_NAME}" \
  bash
