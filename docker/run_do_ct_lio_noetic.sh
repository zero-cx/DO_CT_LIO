#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-do-ct-lio-noetic:local}"
CONTAINER_NAME="${CONTAINER_NAME:-do-ct-lio-noetic}"
HOST_WS="${HOST_WS:-/home/ubuntu20/DO_CT_LIO_ws}"
CONTAINER_USER="${USER:-ubuntu20}"
CONTAINER_WS="/home/${CONTAINER_USER}/DO_CT_LIO_ws"

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
  -v "${HOST_WS}:${CONTAINER_WS}:rw" \
  -w "${CONTAINER_WS}" \
  "${IMAGE_NAME}" \
  bash
