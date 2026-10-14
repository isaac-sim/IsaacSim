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

# launch_prebuilt_web_viewer.sh — Launch Isaac Sim + web viewer from a prebuilt NGC image and print the HTTP URL.

# Launch Isaac Sim + web viewer from a prebuilt NGC image (no local Isaac image build)
# and print the Chromium HTTP URL on stdout.
#
# Usage:
#   launch_prebuilt_web_viewer.sh [--image IMAGE] [--host HOST]
#       [--port PORT] [--signal-port PORT] [--stream-port PORT] [--dry-run] [--url-only] [--down]
#       [--compose-file FILE] [--repo-root DIR]
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

IMAGE="${ISAAC_SIM_IMAGE:-}"
HOST="${ISAACSIM_HOST:-127.0.0.1}"
PORT="${WEB_VIEWER_PORT:-8210}"
SIGNAL_PORT="${ISAACSIM_SIGNAL_PORT:-49100}"
STREAM_PORT="${ISAACSIM_STREAM_PORT:-47998}"
PROJECT="${COMPOSE_PROJECT_NAME:-isim}"
WAIT_SECONDS="${WAIT_SECONDS:-600}"
COMPOSE_FILE_ARG=""
REPO_ROOT_ARG=""
URL_ONLY=0
BRING_DOWN=0
DRY_RUN=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

  --image IMAGE         Prebuilt NGC image (default: latest stable tag from the NGC catalog)
  --host HOST           ISAACSIM_HOST: 127.0.0.1 (this machine), auto (LAN IP), or an explicit IP
  --port PORT           WEB_VIEWER_PORT (default: ${PORT})
  --signal-port PORT    ISAACSIM_SIGNAL_PORT (default: ${SIGNAL_PORT})
  --stream-port PORT    ISAACSIM_STREAM_PORT, WebRTC media over UDP (default: ${STREAM_PORT})
  --dry-run             Validate compose and print the URL; do not start containers
  --url-only            Do not launch; wait if needed and print the HTTP URL
  --down                Stop the compose project
  --compose-file FILE   Compose file (default: this skill's docker/docker-compose.yml)
  --repo-root DIR       Optional override: use DIR/tools/docker/docker-compose.yml
  -h, --help            Show this help

No Isaac Sim source checkout is required. Compose and the web-viewer Dockerfile
are shipped with this skill.

--image must be nvcr.io/nvidia/isaac-sim:<tag>. Docker Hub and other registries
are refused. This script does not apt/pip/npm anything on the host.

Without --dry-run, --url-only, or --down, this starts containers. There is no confirmation prompt; agents must get operator approval and EULA acknowledgement first.

After compose up, wait until isaac-sim logs contain:
  Isaac Sim Full Streaming App is loaded
Then print the HTTP URL. Do not give the URL before that line.

The browser URL must match --host. 127.0.0.1 only works on this machine.
From another computer use --host auto (or that computer's route to this host).

Environment: ISAAC_SIM_IMAGE, ISAACSIM_HOST, WEB_VIEWER_PORT,
             ISAACSIM_SIGNAL_PORT, ISAACSIM_STREAM_PORT, COMPOSE_PROJECT_NAME,
             WAIT_SECONDS, ISAAC_SIM_DATA, GPU_DEVICE
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --compose-file) COMPOSE_FILE_ARG="${2:?}"; shift 2 ;;
    --repo-root) REPO_ROOT_ARG="${2:?}"; shift 2 ;;
    --image) IMAGE="${2:?}"; shift 2 ;;
    --host) HOST="${2:?}"; shift 2 ;;
    --port) PORT="${2:?}"; shift 2 ;;
    --signal-port) SIGNAL_PORT="${2:?}"; shift 2 ;;
    --stream-port) STREAM_PORT="${2:?}"; shift 2 ;;
    --url-only) URL_ONLY=1; shift ;;
    --down) BRING_DOWN=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

die_if_unsupported_host

COMPOSE_FILE="$(resolve_compose_file "${COMPOSE_FILE_ARG}" "${REPO_ROOT_ARG}")"
if [[ ! -f "${COMPOSE_FILE}" ]]; then
  echo "Compose file not found: ${COMPOSE_FILE}" >&2
  echo "Expected the skill copy at $(skill_compose_file)" >&2
  exit 1
fi
WEB_VIEWER_DOCKERFILE="$(dirname "${COMPOSE_FILE}")/web-viewer/Dockerfile"
if [[ ! -f "${WEB_VIEWER_DOCKERFILE}" ]]; then
  echo "web-viewer Dockerfile not found: ${WEB_VIEWER_DOCKERFILE}" >&2
  exit 1
fi

compose() {
  docker compose -p "${PROJECT}" -f "${COMPOSE_FILE}" "$@"
}

if [[ "${BRING_DOWN}" -eq 1 ]]; then
  # Teardown must not depend on host or port values. A malformed
  # ISAACSIM_HOST / WEB_VIEWER_PORT from a previous launch would otherwise
  # exit here and leave the containers running.
  compose down
  exit 0
fi

HOST_IN="${HOST}"
HOST="$(normalize_isaacsim_host "${HOST}")"
require_isaacsim_bind_host "${HOST}"
if [[ "${HOST_IN}" == "auto" || "${HOST_IN}" == "lan" || "${HOST_IN}" == "remote" ]]; then
  echo "Detected system IP for ISAACSIM_HOST: ${HOST}" >&2
else
  echo "ISAACSIM_HOST=${HOST}" >&2
fi

require_host_port WEB_VIEWER_PORT "${PORT}"
require_host_port ISAACSIM_SIGNAL_PORT "${SIGNAL_PORT}"
require_host_port ISAACSIM_STREAM_PORT "${STREAM_PORT}"

url_from_logs() {
  compose logs web-viewer 2>/dev/null \
    | grep -Eo 'https?://[0-9A-Za-z._:-]+' \
    | grep -E ":${PORT}" \
    | tail -1 || true
}

print_url() {
  print_host_url_hint "${HOST}" "${PORT}"
  printf 'http://%s:%s\n' "${HOST}" "${PORT}"
}

intimate_operator_url() {
  cat >&2 <<EOF
${ISAAC_SIM_STREAMING_APP_LOADED}
Open this URL in Chromium:
EOF
  print_url
}

wait_for_streaming_app() {
  local elapsed=0
  local poll="${SCRIPT_DIR}/poll_compose_logs.sh"
  local fatals=""
  local raw_ps=""
  echo "Waiting for isaac-sim log line: ${ISAAC_SIM_STREAMING_APP_LOADED}" >&2
  while (( elapsed < WAIT_SECONDS )); do
    if fatals="$(isaac_sim_logs_have_fatal "${PROJECT}" "${COMPOSE_FILE}")"; then
      echo "FAIL: fatal patterns in isaac-sim logs before streaming app loaded:" >&2
      printf '%s\n' "${fatals}" >&2
      return 1
    fi
    if isaac_sim_streaming_app_loaded "${PROJECT}" "${COMPOSE_FILE}"; then
      raw_ps="$(compose ps --format '{{.Service}} {{.Health}}' 2>/dev/null || true)"
      if curl -fsS --max-time 5 "http://${HOST}:${PORT}/" >/dev/null 2>&1 \
        || web_viewer_ps_health_is "${raw_ps}" web-viewer healthy; then
        echo "OK: ${ISAAC_SIM_STREAMING_APP_LOADED}" >&2
        return 0
      fi
      echo "Streaming app loaded; waiting for web-viewer HTTP on http://${HOST}:${PORT}/ ..." >&2
    elif (( elapsed == 0 || elapsed % 30 == 0 )); then
      echo "Still waiting (${elapsed}s/${WAIT_SECONDS}s)..." >&2
    fi
    sleep 5
    elapsed=$((elapsed + 5))
  done
  echo "Timed out after ${WAIT_SECONDS}s waiting for: ${ISAAC_SIM_STREAMING_APP_LOADED}" >&2
  compose ps >&2 || true
  "${poll}" --compose-file "${COMPOSE_FILE}" --tail 120 >&2 || true
  return 1
}

if [[ -z "${IMAGE}" ]]; then
  IMAGE="$(resolve_latest_ngc_isaac_sim_image)"
  echo "Using latest NGC catalog tag: ${IMAGE}" >&2
fi

assert_permitted_isaac_sim_image "${IMAGE}"

export ISAAC_SIM_IMAGE="${IMAGE}"
export ISAACSIM_HOST="${HOST}"
export WEB_VIEWER_PORT="${PORT}"
export ISAACSIM_SIGNAL_PORT="${SIGNAL_PORT}"
export ISAACSIM_STREAM_PORT="${STREAM_PORT}"

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "DRY-RUN file=${COMPOSE_FILE}" >&2
  echo "DRY-RUN ISAAC_SIM_IMAGE=${ISAAC_SIM_IMAGE} ISAACSIM_HOST=${ISAACSIM_HOST} WEB_VIEWER_PORT=${WEB_VIEWER_PORT} ISAACSIM_SIGNAL_PORT=${ISAACSIM_SIGNAL_PORT}" >&2
  echo "DRY-RUN would run: docker pull ${ISAAC_SIM_IMAGE}" >&2
  echo "DRY-RUN would then run: docker compose -p ${PROJECT} -f ${COMPOSE_FILE} up --build -d" >&2
  if ! nvcr_logged_in; then
    echo "DRY-RUN note: nvcr.io login is missing; full launch will ask the operator for NGC credentials." >&2
  fi
  compose config >/dev/null
  print_url
  exit 0
fi

if [[ "${URL_ONLY}" -eq 1 ]]; then
  wait_for_streaming_app
  intimate_operator_url
  exit 0
fi

prepare_mounts() {
  local data="${ISAAC_SIM_DATA:-${HOME}/docker/isaac-sim}"
  local hub="${ISAACSIM_HUB_CACHE_PATH:-${data}/hub}"
  local explicit=0
  [[ -n "${ISAAC_SIM_DATA:-}" ]] && explicit=1
  if assert_safe_skill_mount "${data}" \
    && mkdir -p "${data}/"{cache/main,cache/computecache,cache/kit,config,data,logs,pkg} "${hub}" 2>/dev/null \
    && assert_safe_skill_mount "${data}" \
    && assert_safe_skill_mount "${hub}"; then
    widen_for_container_uid "${data}" "${hub}" || exit 1
    export ISAAC_SIM_DATA="${data}"
    export ISAACSIM_HUB_CACHE_PATH="${hub}"
    return 0
  fi
  if [[ "${explicit}" -eq 1 ]]; then
    echo "FAIL: ISAAC_SIM_DATA=${data} must be a real directory under \$HOME/docker/isaac-sim or \$HOME/.cache/isaac-sim-web-viewer (no symlinks)." >&2
    exit 1
  fi
  data="${HOME}/.cache/isaac-sim-web-viewer/data"
  hub="${HOME}/.cache/isaac-sim-web-viewer/hub"
  assert_safe_skill_mount "${HOME}/.cache/isaac-sim-web-viewer" || exit 1
  mkdir -p "${data}/"{cache/main,cache/computecache,cache/kit,config,data,logs,pkg} "${hub}"
  assert_safe_skill_mount "${HOME}/.cache/isaac-sim-web-viewer" || exit 1
  widen_for_container_uid "${HOME}/.cache/isaac-sim-web-viewer" || exit 1
  echo "WARN: default ~/docker/isaac-sim is not usable; using ${data} (uid 1234 ACLs)." >&2
  export ISAAC_SIM_DATA="${data}"
  export ISAACSIM_HUB_CACHE_PATH="${hub}"
}

# Repeat launches leave uid-1234 containers with these mounts. Stop them
# before the ACL walk so they cannot replace a checked path with a symlink.
echo "Stopping compose project '${PROJECT}' before mount ACL update (if any)" >&2
compose stop >/dev/null 2>&1 || true

prepare_mounts

if ! nvcr_logged_in; then
  status=0
  "${SCRIPT_DIR}/ngc_login.sh" || status=$?
  if [[ "${status}" -ne 0 ]]; then
    exit "${status}"
  fi
fi

# Pull the Isaac Sim image first. Compose must not start until this succeeds.
status=0
"${SCRIPT_DIR}/pull_isaac_sim_image.sh" --image "${IMAGE}" || status=$?
if [[ "${status}" -ne 0 ]]; then
  exit "${status}"
fi

echo "Starting compose project '${PROJECT}' with ISAAC_SIM_IMAGE=${ISAAC_SIM_IMAGE}" >&2
compose up --build -d

wait_for_streaming_app
intimate_operator_url
