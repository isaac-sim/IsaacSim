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

# preflight.sh — Check Docker, GPU, compose file, ports, and NGC image before launch.

# Check host, Docker, GPU, compose file, ports, mounts, and optional NGC image.
#
# Usage:
#   preflight.sh [--image IMAGE] [--host HOST] [--port PORT]
#                [--signal-port PORT] [--require-image]
#                [--compose-file FILE] [--repo-root DIR]
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
COMPOSE_FILE_ARG=""
REPO_ROOT_ARG=""
REQUIRE_IMAGE=0
FAILS=0
WARNS=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

  --image IMAGE         Prebuilt NGC image to look for (default: latest NGC catalog tag)
  --host HOST           Planned ISAACSIM_HOST (127.0.0.1, auto, or an IP)
  --port PORT           Planned WEB_VIEWER_PORT
  --signal-port PORT    Planned ISAACSIM_SIGNAL_PORT
  --stream-port PORT    Planned ISAACSIM_STREAM_PORT (WebRTC media, UDP)
  --require-image       Fail if the Isaac Sim image is not present locally
  --compose-file FILE   Compose file (default: this skill's docker/docker-compose.yml)
  --repo-root DIR       Optional override: use DIR/tools/docker/docker-compose.yml
  -h, --help            Show this help
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
    --require-image) REQUIRE_IMAGE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${IMAGE}" ]]; then
  IMAGE="$(resolve_latest_ngc_isaac_sim_image)"
  echo "Using latest NGC catalog tag: ${IMAGE}" >&2
fi

assert_permitted_isaac_sim_image "${IMAGE}"

HOST="$(normalize_isaacsim_host "${HOST}")"
require_isaacsim_bind_host "${HOST}"

fail() { echo "FAIL: $*" >&2; FAILS=$((FAILS + 1)); }
warn() { echo "WARN: $*" >&2; WARNS=$((WARNS + 1)); }
ok() { echo "OK: $*"; }

die_if_unsupported_host
ok "host is Ubuntu/Linux (not Windows/WSL)"

if command -v docker >/dev/null 2>&1; then
  ok "docker $(docker --version | awk '{print $3}' | tr -d ',')"
else
  fail "docker is not on PATH"
fi

if docker compose version >/dev/null 2>&1; then
  ok "docker compose $(docker compose version --short 2>/dev/null || echo present)"
else
  fail "docker compose plugin is missing"
fi

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  ok "GPU $(nvidia-smi -L | head -1)"
else
  fail "nvidia-smi did not list a GPU"
fi

if docker info 2>/dev/null | grep -qi nvidia; then
  ok "Docker NVIDIA runtime is available"
else
  fail "Docker NVIDIA runtime not visible in docker info"
fi

COMPOSE_FILE="$(resolve_compose_file "${COMPOSE_FILE_ARG}" "${REPO_ROOT_ARG}")"
if [[ -f "${COMPOSE_FILE}" ]]; then
  ok "compose file ${COMPOSE_FILE}"
else
  fail "compose file missing: ${COMPOSE_FILE}"
fi
WEB_VIEWER_DOCKERFILE="$(dirname "${COMPOSE_FILE}")/web-viewer/Dockerfile"
if [[ -f "${WEB_VIEWER_DOCKERFILE}" ]]; then
  ok "web-viewer Dockerfile present"
else
  fail "missing ${WEB_VIEWER_DOCKERFILE}"
fi

if [[ -n "${COMPOSE_FILE}" && -f "${COMPOSE_FILE}" ]]; then
  if ISAAC_SIM_IMAGE="${IMAGE}" ISAACSIM_HOST="${HOST}" WEB_VIEWER_PORT="${PORT}" \
    ISAACSIM_SIGNAL_PORT="${SIGNAL_PORT}" ISAACSIM_STREAM_PORT="${STREAM_PORT}" \
    docker compose -p isim -f "${COMPOSE_FILE}" config >/dev/null; then
    ok "compose config validates with ISAAC_SIM_IMAGE=${IMAGE}"
  else
    fail "docker compose config failed"
  fi
fi

# Bind helpers exit 0 (busy), 1 (free), or 2 (invalid). A conversion failure used
# to share exit 1 with a successful bind, so `--stream-port abc` looked free.
check_port() {
  local name="${1:?}" value="${2:-}" proto="${3:?}" flag="${4:?}" busy
  if ! is_valid_host_port "${value}"; then
    fail "${name} ${value} is not a ${proto} port in 1..65535 (fix ${flag})"
    return
  fi
  busy=0
  if [[ "${proto}" == "udp" ]]; then
    udp_port_busy "${value}" || busy=$?
  else
    tcp_port_busy "${value}" || busy=$?
  fi
  if [[ "${busy}" -eq 0 ]]; then
    fail "${name} ${value}/${proto} is already in use (pick another ${flag})"
  elif [[ "${busy}" -eq 2 ]]; then
    fail "${name} ${value} is not a ${proto} port in 1..65535 (fix ${flag})"
  else
    ok "${name} ${value}/${proto} is free"
  fi
}

check_port WEB_VIEWER_PORT "${PORT}" tcp --port
check_port ISAACSIM_SIGNAL_PORT "${SIGNAL_PORT}" tcp --signal-port
check_port ISAACSIM_STREAM_PORT "${STREAM_PORT}" udp --stream-port

DATA_DIR="${ISAAC_SIM_DATA:-${HOME}/docker/isaac-sim}"
HUB_DIR="${ISAACSIM_HUB_CACHE_PATH:-${DATA_DIR}/hub}"
for d in "${DATA_DIR}" "${HUB_DIR}"; do
  if [[ -e "${d}" ]]; then
    uid="$(dir_uid "${d}")"
    if [[ "${uid}" == "1234" ]]; then
      ok "${d} owned by uid 1234"
    else
      warn "${d} owned by uid ${uid:-unknown}; container user is 1234. Run: chown -R 1234:1234 ${d}"
    fi
    # Ownership alone is not enough. A tree owned by uid 1234 with no group or
    # other write bit leaves this user unable to add the missing cache
    # subdirectories, and the launcher silently falls back to its own cache.
    if [[ -w "${d}" ]]; then
      ok "${d} is writable by $(id -un)"
    else
      warn "${d} is not writable by $(id -un); launch will fall back to ${HOME}/.cache/isaac-sim-web-viewer"
    fi
  else
    warn "${d} does not exist yet (launch script will mkdir)"
  fi
done

if nvcr_logged_in; then
  ok "docker has nvcr.io credentials"
else
  fail "ASK_USER_NGC_CREDENTIALS: docker is not logged in to nvcr.io. The operator must docker login nvcr.io locally (do not paste the NGC API key into chat) before pulling ${IMAGE}."
fi

if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  ok "image present: ${IMAGE}"
else
  msg="image not present locally: ${IMAGE}. Full launch needs nvcr.io login and a pull."
  if [[ "${REQUIRE_IMAGE}" -eq 1 ]]; then
    fail "${msg}"
  else
    warn "${msg}"
  fi
fi

echo
echo "Planned web viewer URL: http://${HOST}:${PORT}"
print_host_url_hint "${HOST}" "${PORT}"
if [[ "${FAILS}" -gt 0 ]]; then
  echo "Preflight failed (${FAILS} fail, ${WARNS} warn)." >&2
  exit 1
fi
echo "Preflight passed (${WARNS} warn)."
exit 0
