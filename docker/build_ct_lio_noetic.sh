#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="${IMAGE_NAME:-ct-lio-noetic:local}"
USER_NAME="${USER:-ubuntu20}"
USER_UID="$(id -u)"
USER_GID="$(id -g)"

if docker info >/dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi

"${DOCKER[@]}" build \
  --network=host \
  --build-arg USERNAME="${USER_NAME}" \
  --build-arg USER_UID="${USER_UID}" \
  --build-arg USER_GID="${USER_GID}" \
  -t "${IMAGE_NAME}" \
  -f "${SCRIPT_DIR}/Dockerfile.noetic" \
  "${SCRIPT_DIR}/.."

echo "Built image: ${IMAGE_NAME}"
