#!/bin/bash
# Clean Docker build artifacts and runtime cache

IMAGE_TAG="isaac-sim-docker:latest"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "Cleaning Docker artifacts..."

# _container_temp: build context (host ownership)
rm -rf "${REPO_ROOT}/_container_temp"

# _isaac_cache: runtime cache (uid 1234 ownership, use Docker to clean)
if [[ -d "${REPO_ROOT}/_isaac_cache" ]]; then
    docker run --rm --entrypoint rm -v "${REPO_ROOT}/_isaac_cache":/cache "${IMAGE_TAG}" -rf /cache
    rmdir "${REPO_ROOT}/_isaac_cache" 2>/dev/null || rm -rf "${REPO_ROOT}/_isaac_cache"
fi

echo "Done."
