#!/bin/bash

IMAGE_TAG="isaac-sim-docker:latest"
PRIVACY_EMAIL="user@example.com"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CACHE_DIR="${REPO_ROOT}/_isaac_cache"

xhost +local:docker &>/dev/null || true

# Create cache directory with container user ownership (uid 1234)
mkdir -p "${CACHE_DIR}"
docker run --rm --entrypoint chown -v "${CACHE_DIR}":/cache "${IMAGE_TAG}" -R 1234:1234 /cache

docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm \
 --network=host -e "PRIVACY_CONSENT=Y" -e "PRIVACY_USERID=${PRIVACY_EMAIL}" \
 -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v "${CACHE_DIR}":/isaac-sim/.local/share/ov \
 "${IMAGE_TAG}" "$@"
