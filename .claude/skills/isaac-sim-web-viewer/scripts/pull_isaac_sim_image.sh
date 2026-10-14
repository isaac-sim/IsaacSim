#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
:
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
:
# You may obtain a copy of the License at
:
# http://www.apache.org/licenses/LICENSE-2.0
:
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
:
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
:
# limitations under the License.

# pull_isaac_sim_image.sh — Pull the prebuilt NGC Isaac Sim image before compose up.

# Pull nvcr.io/nvidia/isaac-sim:<tag> and verify it exists locally.
# Must run after NGC login. Do this before docker compose up.
#
# Exit 0: image is present locally (already was, or pull succeeded)
# Exit 3: nvcr.io login missing (ASK_USER_NGC_CREDENTIALS)
# Exit 4: tag does not exist (ASK_USER_IMAGE_TAG)
# Exit 1: pull failed for another reason
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

IMAGE="${ISAAC_SIM_IMAGE:-}"
CHECK_ONLY=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--image IMAGE] [--check-only]

  --image IMAGE   Prebuilt NGC image. If omitted, resolve the latest stable tag
                  from ${NGC_ISAAC_SIM_TAGS_URL}
  --check-only    Succeed if the image is already local; do not pull

This step runs after NGC login and before docker compose up.
IMAGE must be nvcr.io/nvidia/isaac-sim:<tag>. Other registries are refused.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) IMAGE="${2:?}"; shift 2 ;;
    --check-only) CHECK_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${IMAGE}" ]]; then
  IMAGE="$(resolve_latest_ngc_isaac_sim_image)"
  echo "Using latest NGC catalog tag: ${IMAGE}" >&2
fi

assert_permitted_isaac_sim_image "${IMAGE}"

if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "OK: image already present: ${IMAGE}"
  exit 0
fi

if [[ "${CHECK_ONLY}" -eq 1 ]]; then
  echo "FAIL: image not present locally: ${IMAGE}" >&2
  exit 1
fi

if ! nvcr_logged_in; then
  status=0
  "${SCRIPT_DIR}/ngc_login.sh" || status=$?
  if [[ "${status}" -ne 0 ]]; then
    exit "${status}"
  fi
fi

echo "Pulling ${IMAGE} (required before compose up)" >&2
if ! docker pull "${IMAGE}"; then
  printf '%s\n' \
    "ASK_USER_IMAGE_TAG" \
    "docker pull failed for ${IMAGE}." \
    "The tag may not exist on NGC." \
    "Public NGC tags: ${NGC_ISAAC_SIM_TAGS_URL}" \
    "Ask the operator which published tag to pull, or re-run without --image to pick Latest." \
    "Do not start compose until the image is local." >&2
  exit 4
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "FAIL: pull reported success but ${IMAGE} is not local" >&2
  exit 1
fi

echo "OK: image present: ${IMAGE}"
exit 0
