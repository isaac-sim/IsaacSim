#!/usr/bin/env bash
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

# Shared helpers for isaac-sim-web-viewer scripts.
# Source this file; do not execute it.

_ISAAC_SIM_WEB_VIEWER_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_ISAAC_SIM_WEB_VIEWER_ROOT="$(cd "${_ISAAC_SIM_WEB_VIEWER_SCRIPTS}/.." && pwd)"

# Anything that is not native Linux is out. The compose stack needs the NVIDIA
# container runtime and host networking, neither of which exists on macOS or
# under a Windows shell, and WSL cannot pass the GPU through this way. Catching
# these here matters because the launcher gates on this before pulling ~15 GB.
unsupported_host() {
  local uname_s
  uname_s="$(uname -s 2>/dev/null || true)"
  if [[ "${uname_s}" != "Linux" ]]; then
    return 0
  fi
  if grep -qiE 'microsoft|wsl' /proc/version 2>/dev/null; then
    return 0
  fi
  return 1
}

# Report the distribution ID, or the empty string when it cannot be determined.
host_distro_id() {
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    (. /etc/os-release && printf '%s\n' "${ID:-}")
  fi
}

die_if_unsupported_host() {
  local uname_s distro
  uname_s="$(uname -s 2>/dev/null || true)"
  if unsupported_host; then
    echo "Docker Compose web viewer deployment is supported only on Ubuntu hosts and DGX Spark systems. Detected '${uname_s}'; Windows hosts, including WSL, and macOS are not supported." >&2
    exit 2
  fi
  # Other Linux distributions usually work given the NVIDIA container runtime,
  # so this is a warning rather than a hard stop, but it is the first thing to
  # mention if compose startup fails.
  distro="$(host_distro_id)"
  if [[ -n "${distro}" && "${distro}" != "ubuntu" ]]; then
    echo "WARN: this skill is validated on Ubuntu and DGX Spark; detected '${distro}'. Continuing, but NVIDIA container runtime setup may differ." >&2
  fi
}

skill_compose_file() {
  printf '%s/docker/docker-compose.yml\n' "${_ISAAC_SIM_WEB_VIEWER_ROOT}"
}

skill_web_viewer_dockerfile() {
  printf '%s/docker/web-viewer/Dockerfile\n' "${_ISAAC_SIM_WEB_VIEWER_ROOT}"
}

# Default compose file is shipped with this skill. No Isaac Sim source checkout.
# Optional overrides: compose file path, or a repo that has tools/docker/docker-compose.yml.
resolve_compose_file() {
  local compose_arg="${1:-}"
  local repo_arg="${2:-}"
  if [[ -n "${compose_arg}" ]]; then
    printf '%s\n' "$(cd "$(dirname "${compose_arg}")" && pwd)/$(basename "${compose_arg}")"
    return 0
  fi
  if [[ -n "${repo_arg}" ]]; then
    printf '%s/tools/docker/docker-compose.yml\n' "$(cd "${repo_arg}" && pwd)"
    return 0
  fi
  skill_compose_file
}

# Host ports must be integers in 1..65535. 0 is valid for bind() (ephemeral) but
# not as a published WebRTC/HTTP port, so reject it here rather than in the bind
# helpers — otherwise a conversion failure and a successful bind both look "free".
is_valid_host_port() {
  local p="${1:-}"
  [[ "${p}" =~ ^[1-9][0-9]{0,4}$ ]] || return 1
  (( 10#${p} >= 1 && 10#${p} <= 65535 ))
}

require_host_port() {
  local name="${1:?}" value="${2:-}"
  if ! is_valid_host_port "${value}"; then
    echo "ASK_USER_PORT: ${name}=${value} is not a TCP/UDP port in 1..65535." >&2
    exit 2
  fi
}

require_isaacsim_bind_host() {
  local host="${1:-}"
  if ! python3 "${_ISAAC_SIM_WEB_VIEWER_SCRIPTS}/validate_stream_bind.py" --host "${host}"; then
    exit 2
  fi
}

assert_safe_skill_mount() {
  python3 "${_ISAAC_SIM_WEB_VIEWER_SCRIPTS}/skill_mounts.py" assert-safe "${1:?}"
}

# Exact equality to "healthy". `grep -i healthy` also matches `unhealthy`.
web_viewer_ps_health_is() {
  local raw="${1:-}" service="${2:-web-viewer}" want="${3:-healthy}"
  printf '%s\n' "${raw}" | awk -v s="${service}" -v w="${want}" '$1 == s && $2 == w { found = 1 } END { exit !found }'
}

widen_for_container_uid() {
  local path host_uid
  host_uid="$(id -u)"
  for path in "$@"; do
    assert_safe_skill_mount "${path}" || return 1
    if [[ "${host_uid}" == "1234" ]]; then
      continue
    fi
    if ! command -v setfacl >/dev/null 2>&1; then
      echo "FAIL: setfacl is required when the host uid is not 1234. Install the acl package. Refusing chmod a+rwX." >&2
      return 1
    fi
    python3 "${_ISAAC_SIM_WEB_VIEWER_SCRIPTS}/skill_mounts.py" apply-acl "${path}" "${host_uid}" || return 1
  done
}

# Exit 0 = in use, 1 = free, 2 = not a host port. Callers must not treat 2 as free.
_port_bind_status() {
  local proto="${1:?}" port="${2:?}"
  python3 - "${proto}" "${port}" <<'PY'
import socket, sys

proto, raw = sys.argv[1], sys.argv[2]
try:
    port = int(raw)
except (TypeError, ValueError):
    sys.exit(2)
if not 1 <= port <= 65535:
    sys.exit(2)
kind = socket.SOCK_STREAM if proto == "tcp" else socket.SOCK_DGRAM
s = socket.socket(socket.AF_INET, kind)
try:
    s.bind(("0.0.0.0", port))
except OSError:
    sys.exit(0)
finally:
    s.close()
sys.exit(1)
PY
}

tcp_port_busy() {
  _port_bind_status tcp "${1:?}"
}

# Isaac Sim carries WebRTC media over UDP on ISAACSIM_STREAM_PORT. Because the
# container uses host networking, a host process already holding that UDP port
# yields a viewer that connects but never shows video, so check it separately
# from the TCP ports.
udp_port_busy() {
  _port_bind_status udp "${1:?}"
}

dir_uid() {
  python3 - "${1:?}" <<'PY'
import os, sys
path = sys.argv[1]
try:
    print(os.stat(path).st_uid)
except FileNotFoundError:
    print("")
PY
}

# True if Docker already has nvcr.io credentials (config.json auths or credHelpers).
nvcr_logged_in() {
  python3 - <<'PY'
import json
from pathlib import Path
p = Path.home() / ".docker" / "config.json"
if not p.exists():
    raise SystemExit(1)
try:
    data = json.loads(p.read_text())
except Exception:
    raise SystemExit(1)
keys = []
auths = data.get("auths") or {}
if isinstance(auths, dict):
    keys.extend(auths.keys())
helpers = data.get("credHelpers") or {}
if isinstance(helpers, dict):
    keys.extend(helpers.keys())
store = data.get("credsStore")
# A credsStore can hold nvcr.io without listing it. Treat as unknown/not-present
# unless nvcr.io is named in auths/credHelpers.
for key in keys:
    lowered = str(key).lower()
    if "nvcr.io" in lowered:
        raise SystemExit(0)
raise SystemExit(1)
PY
}

ngc_api_key_from_env() {
  if [[ -n "${NGC_API_KEY:-}" ]]; then
    printf '%s' "${NGC_API_KEY}"
    return 0
  fi
  if [[ -n "${NGC_CLI_API_KEY:-}" ]]; then
    printf '%s' "${NGC_CLI_API_KEY}"
    return 0
  fi
  return 1
}

print_ask_ngc_credentials() {
  cat >&2 <<'EOF'
ASK_USER_NGC_CREDENTIALS
NGC credentials are not present for nvcr.io. The Isaac Sim prebuilt image cannot be pulled until this machine is logged in.

Do not paste an NGC API key into this chat or into tool-call arguments.

Authenticate locally, then re-run the check:
  docker login nvcr.io
  # username: $oauthtoken
  # password: NGC API key, typed only at that docker prompt
  # key page: https://ngc.nvidia.com/setup/api-key

Or export NGC_API_KEY in the local shell (not in chat) and run scripts/ngc_login.sh.
Stop here until scripts/ngc_login.sh --check prints OK.
EOF
}

NGC_ISAAC_SIM_TAGS_URL="${NGC_ISAAC_SIM_TAGS_URL:-https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/isaac-sim/-/tags}"

# Do not give the operator the HTTP URL until isaac-sim logs this line.
ISAAC_SIM_STREAMING_APP_LOADED="${ISAAC_SIM_STREAMING_APP_LOADED:-Isaac Sim Full Streaming App is loaded}"

isaac_sim_compose_logs() {
  local project="${1:-${COMPOSE_PROJECT_NAME:-isim}}"
  local compose_file="${2:-}"
  if [[ -n "${compose_file}" && -f "${compose_file}" ]]; then
    docker compose -p "${project}" -f "${compose_file}" logs isaac-sim 2>/dev/null || true
  else
    docker compose -p "${project}" logs isaac-sim 2>/dev/null || true
  fi
}

isaac_sim_streaming_app_loaded() {
  local project="${1:-${COMPOSE_PROJECT_NAME:-isim}}"
  local compose_file="${2:-}"
  isaac_sim_compose_logs "${project}" "${compose_file}" | grep -Fq "${ISAAC_SIM_STREAMING_APP_LOADED}"
}

isaac_sim_logs_have_fatal() {
  local project="${1:-${COMPOSE_PROJECT_NAME:-isim}}"
  local compose_file="${2:-}"
  local lines
  lines="$(isaac_sim_compose_logs "${project}" "${compose_file}" | grep -Ei '\[Error\]|Traceback \(most recent call last\)|Fatal Python error|Segmentation fault' | grep -v 'NVST_R_BUSY' || true)"
  if [[ -n "${lines}" ]]; then
    printf '%s\n' "${lines}"
    return 0
  fi
  return 1
}

# Print the tag the NGC catalog marks Latest. Exits 4 when the catalog cannot be
# read, so the resolver's hardcoded fallback is never silently launched as if it
# were Latest; the operator has to supply --image instead.
resolve_latest_ngc_isaac_sim_image() {
  local image status=0
  image="$(python3 "${_ISAAC_SIM_WEB_VIEWER_SCRIPTS}/resolve_ngc_isaac_sim_image.py")" || status=$?
  if [[ "${status}" -ne 0 ]]; then
    echo "ASK_USER_IMAGE_TAG: could not resolve Latest from the NGC catalog. Ask the operator for a published tag and pass --image nvcr.io/nvidia/isaac-sim:<tag>." >&2
    exit 4
  fi
  printf '%s\n' "${image}"
}

# Refuse Docker Hub / GHCR / other registries. Only NVIDIA NGC Isaac Sim may be pulled.
assert_permitted_isaac_sim_image() {
  local image="${1:-}"
  local tag
  if [[ -z "${image}" ]]; then
    echo "ASK_USER_IMAGE_TAG: empty image ref. Use nvcr.io/nvidia/isaac-sim:<tag>." >&2
    exit 4
  fi
  case "${image}" in
    nvcr.io/nvidia/isaac-sim:*)
      tag="${image#nvcr.io/nvidia/isaac-sim:}"
      if [[ -z "${tag}" || "${tag}" == *"/"* || "${tag}" == *[[:space:]]* ]]; then
        echo "ASK_USER_IMAGE_TAG: refusing ${image}. Expected nvcr.io/nvidia/isaac-sim:<tag>." >&2
        exit 4
      fi
      return 0
      ;;
    *)
      echo "ASK_USER_IMAGE_TAG: refusing ${image}." >&2
      echo "This skill may only pull nvcr.io/nvidia/isaac-sim:<tag> from NVIDIA NGC." >&2
      echo "Do not pull Docker Hub, GHCR, Quay, or other registries." >&2
      echo "Do not apt/pip/npm extra software on the host." >&2
      exit 4
      ;;
  esac
}

# First non-loopback, non-docker-bridge IPv4. Used when the browser will run on another machine.
detect_lan_ipv4() {
  python3 - <<'PY'
import re
import subprocess

SKIP_IFACE_EXACT = {"lo", "docker0"}
SKIP_IFACE_PREFIXES = (
    "docker",
    "br-",
    "veth",
    "virbr",
    "cni",
    "flannel",
    "cbr",
    "lxc",
    "tun",
    "tap",
    "wg",
    "tailscale",
)


def skip_iface(name: str) -> bool:
    n = name.split("@", 1)[0].lower()
    if n in SKIP_IFACE_EXACT:
        return True
    return n.startswith(SKIP_IFACE_PREFIXES)


def usable(ip: str) -> bool:
    if not ip or ip.count(".") != 3:
        return False
    if ip.startswith("127.") or ip.startswith("169.254."):
        return False
    if ip.startswith("172.17."):
        return False
    return True


def looks_like_docker_bridge_ip(ip: str) -> bool:
    # docker0 is 172.17; compose/user-defined bridges commonly 172.18–172.31.
    parts = ip.split(".")
    if len(parts) != 4 or parts[0] != "172":
        return False
    try:
        second = int(parts[1])
    except ValueError:
        return False
    return 17 <= second <= 31


def iface_addrs():
    try:
        out = subprocess.check_output(["ip", "-4", "-o", "addr", "show"], text=True)
    except Exception:
        return []
    rows = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[2] != "inet":
            continue
        iface = parts[1].split("@", 1)[0]
        ip = parts[3].split("/")[0]
        rows.append((iface, ip))
    return rows


def default_iface():
    try:
        out = subprocess.check_output(["ip", "-4", "route", "show", "default"], text=True)
    except Exception:
        return None
    match = re.search(r"\bdev\s+(\S+)", out)
    return match.group(1) if match else None


def default_route_src():
    """Return (dev, src) for the normal default route, not a more-specific 10/8 VPN path."""
    try:
        out = subprocess.check_output(["ip", "-4", "route", "get", "1.1.1.1"], text=True)
    except Exception:
        return None, None
    line = out.splitlines()[0] if out.strip() else ""
    dev = re.search(r"\bdev\s+(\S+)", line)
    src = re.search(r"\bsrc\s+(\S+)", line)
    return (dev.group(1) if dev else None, src.group(1) if src else None)


dev, src = default_route_src()
if src and usable(src) and not skip_iface(dev or ""):
    print(src)
    raise SystemExit(0)

addrs = iface_addrs()
pref = default_iface()
if pref and not skip_iface(pref):
    for iface, ip in addrs:
        if iface == pref and usable(ip):
            print(ip)
            raise SystemExit(0)

for iface, ip in addrs:
    if skip_iface(iface) or not usable(ip):
        continue
    print(ip)
    raise SystemExit(0)

try:
    for tok in subprocess.check_output(["hostname", "-I"], text=True).split():
        if usable(tok) and not looks_like_docker_bridge_ip(tok):
            print(tok)
            raise SystemExit(0)
except SystemExit:
    raise
except Exception:
    pass

print("127.0.0.1")
PY
}

normalize_isaacsim_host() {
  local host="${1:-}"
  case "${host}" in
    ""|local|localhost)
      printf '%s\n' "127.0.0.1"
      ;;
    auto|lan|remote)
      detect_lan_ipv4
      ;;
    *)
      printf '%s\n' "${host}"
      ;;
  esac
}

print_host_url_hint() {
  local host="${1:?}"
  local port="${2:?}"
  if [[ "${host}" == "127.0.0.1" || "${host}" == "localhost" ]]; then
    cat >&2 <<EOF
Open http://${host}:${port} in Chromium on THIS machine only.
A browser on another computer must not use 127.0.0.1 — relaunch with --host auto (or the reachable IP) and rebuild.
EOF
  else
    cat >&2 <<EOF
Open http://${host}:${port} in Chromium (same URL on this machine or another computer that can reach ${host}).
Do not open http://127.0.0.1:${port} for this session — the stream host is baked as ${host}.
Only one browser tab can hold the stream.
EOF
  fi
}
