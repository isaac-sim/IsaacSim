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

# poll_compose_logs.sh — Poll docker compose logs for the web-viewer URL and isaac-sim errors.

# Poll compose status and logs (tools/docker/README.md "Checking status and endpoints"):
#   docker compose -p isim logs
#   docker compose -p isim logs web-viewer
#   docker compose -p isim logs isaac-sim
#
# Exit 0: no fatal isaac-sim errors (warnings are OK)
# Exit 1: fatal error patterns in isaac-sim logs
#
# Prints ISAAC_SIM_STREAMING_APP_LOADED=1 when isaac-sim logs contain
# "Isaac Sim Full Streaming App is loaded". Do not give the operator the
# HTTP URL until that line is present.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

PROJECT="${COMPOSE_PROJECT_NAME:-isim}"
COMPOSE_FILE_ARG=""
REPO_ROOT_ARG=""
TAIL="${LOG_TAIL:-120}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--compose-file FILE] [--repo-root DIR] [--tail N]

Poll docker compose -p ${PROJECT} ps and logs for the web-viewer URL and isaac-sim errors.
Default compose file is this skill's docker/docker-compose.yml (no Isaac Sim checkout).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --compose-file) COMPOSE_FILE_ARG="${2:?}"; shift 2 ;;
    --repo-root) REPO_ROOT_ARG="${2:?}"; shift 2 ;;
    --tail) TAIL="${2:?}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

COMPOSE_FILE="$(resolve_compose_file "${COMPOSE_FILE_ARG}" "${REPO_ROOT_ARG}")"

compose() {
  if [[ -n "${COMPOSE_FILE}" && -f "${COMPOSE_FILE}" ]]; then
    docker compose -p "${PROJECT}" -f "${COMPOSE_FILE}" "$@"
  else
    docker compose -p "${PROJECT}" "$@"
  fi
}

echo "===== compose ps ====="
compose ps || true

echo "===== web-viewer logs (URL) ====="
VIEWER_LOG="$(compose logs --tail "${TAIL}" web-viewer 2>/dev/null || true)"
printf '%s\n' "${VIEWER_LOG}"
URL="$(printf '%s\n' "${VIEWER_LOG}" | grep -Eo 'https?://[0-9A-Za-z._:-]+' | tail -1 || true)"
if [[ -n "${URL}" ]]; then
  echo "WEB_VIEWER_URL=${URL}"
fi

echo "===== isaac-sim logs (tail ${TAIL}) ====="
ISAAC_LOG="$(compose logs --tail "${TAIL}" isaac-sim 2>/dev/null || true)"
printf '%s\n' "${ISAAC_LOG}" | tail -n 40

if isaac_sim_streaming_app_loaded "${PROJECT}" "${COMPOSE_FILE}"; then
  echo "ISAAC_SIM_STREAMING_APP_LOADED=1"
  echo "OK: ${ISAAC_SIM_STREAMING_APP_LOADED}"
elif printf '%s\n' "${ISAAC_LOG}" | grep -Eq 'AppReady|app ready'; then
  echo "isaac-sim reached AppReady; still waiting for: ${ISAAC_SIM_STREAMING_APP_LOADED}"
fi

FATAL_LINES="$(printf '%s\n' "${ISAAC_LOG}" | grep -Ei '\[Error\]|Traceback \(most recent call last\)|Fatal Python error|Segmentation fault' | grep -v 'NVST_R_BUSY' || true)"
if [[ -n "${FATAL_LINES}" ]]; then
  echo "===== isaac-sim fatal lines =====" >&2
  printf '%s\n' "${FATAL_LINES}" >&2
  echo "FAIL: fatal patterns in isaac-sim logs" >&2
  exit 1
fi

echo "OK: no fatal isaac-sim log errors in last ${TAIL} lines"
exit 0
