#! /bin/sh
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Default tag
TAG="isaac-sim-docker:latest"
CONTAINER_PLATFORM=linux/amd64
PUSH_TAG=""

# Parse command line arguments
while [ $# -gt 0 ]; do
    case $1 in
        --tag)
            TAG="$2"
            shift 2
            ;;
        --x86_64)
            CONTAINER_PLATFORM=linux/amd64
            shift
            ;;
        --aarch64)
            CONTAINER_PLATFORM=linux/arm64
            shift
            ;;
        --push)
            PUSH_TAG="--push"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--tag TAG]"
            echo "  --tag TAG    Docker image tag (default: isaac-sim-docker:latest)"
            echo "  --x86_64     Build for x86_64 platform (default)"
            echo "  --aarch64    Build for arm64 platform"
            echo "  --push       Push docker image tag"
            echo "  -h, --help   Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

HOST_ARCH=$(docker info --format '{{.Architecture}}' 2>/dev/null)
if [ -z "$HOST_ARCH" ]; then
    echo "ERROR: Could not detect host architecture. Is Docker running?"
    exit 1
fi
case "$HOST_ARCH" in
    aarch64) HOST_ARCH=arm64 ;;
    x86_64)  HOST_ARCH=amd64 ;;
esac
NATIVE_PLATFORM="linux/$HOST_ARCH"
if docker buildx inspect 2>/dev/null | grep -q "$CONTAINER_PLATFORM"; then
    echo "This builder supports $CONTAINER_PLATFORM builds"
elif [ "$CONTAINER_PLATFORM" = "$NATIVE_PLATFORM" ]; then
    echo "Building natively for $CONTAINER_PLATFORM (host architecture)"
else
    echo "ERROR: This host's buildx builder does NOT support $CONTAINER_PLATFORM"
    echo ""
    echo "To add support, create a builder with:"
    echo "  docker buildx create --name multiarch --platform $CONTAINER_PLATFORM --use"
    echo "  docker buildx inspect --bootstrap"
    exit 1
fi

docker buildx build \
  -t "$TAG" \
  --platform=$CONTAINER_PLATFORM \
  -f tools/docker/Dockerfile \
  $PUSH_TAG _container_temp
