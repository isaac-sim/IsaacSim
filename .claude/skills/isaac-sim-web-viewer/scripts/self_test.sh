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

# self_test.sh — Local checks for the isaac-sim-web-viewer skill (no container start).

# Local checks: syntax, --help, skill-bundled compose dry-run with a pinned image.
# Does not fetch the NGC catalog, pull images, or start containers.
# No Isaac Sim source checkout.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH="${SCRIPT_DIR}/launch_prebuilt_web_viewer.sh"
PREFLIGHT="${SCRIPT_DIR}/preflight.sh"
SKILL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# Product default is 49100. This workstation already binds 49100, so local tests use 49101.
LOCAL_SIGNAL_PORT="${LOCAL_TEST_SIGNAL_PORT:-49101}"

pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*" >&2; exit 1; }

bash -n "${SCRIPT_DIR}/common.sh"
bash -n "${LAUNCH}"
bash -n "${PREFLIGHT}"
bash -n "${SCRIPT_DIR}/ngc_login.sh"
bash -n "${SCRIPT_DIR}/poll_compose_logs.sh"
bash -n "${SCRIPT_DIR}/detect_host_ip.sh"
python3 -m py_compile "${SCRIPT_DIR}/resolve_ngc_isaac_sim_image.py"
python3 -m py_compile "${SCRIPT_DIR}/validate_stream_bind.py"
python3 -m py_compile "${SCRIPT_DIR}/skill_mounts.py"
pass "bash -n scripts"

grep -q 'patch-stream-config.mjs' "${SKILL_ROOT}/docker/web-viewer/Dockerfile" \
  || fail "Dockerfile must patch stream config with patch-stream-config.mjs"
grep -F "signalingServer: '\${ISAACSIM_HOST}'" "${SKILL_ROOT}/docker/web-viewer/Dockerfile" \
  && fail "Dockerfile must not interpolate ISAACSIM_HOST into sed"
grep -q 'Please send:' "${SKILL_ROOT}/references/ngc-login.md" \
  && fail "ngc-login.md must not ask the operator to send an API key"
grep -q 'grep -qi healthy' "${LAUNCH}" \
  && fail "launcher must not substring-match healthy"
pass "host/sed/NGC/health safety strings"

bash -n "${SCRIPT_DIR}/detect_host_ip.sh"
DETECTED="$("${SCRIPT_DIR}/detect_host_ip.sh")"
[[ "${DETECTED}" == *.*.*.* ]] || fail "detect_host_ip.sh did not print an IPv4 (got: ${DETECTED})"
pass "detect_host_ip.sh ${DETECTED}"

[[ -f "${SKILL_ROOT}/docker/docker-compose.yml" ]] || fail "missing skill docker/docker-compose.yml"
[[ -f "${SKILL_ROOT}/docker/web-viewer/Dockerfile" ]] || fail "missing skill docker/web-viewer/Dockerfile"
pass "skill-bundled compose and web-viewer Dockerfile"

HELP_OUT="$("${LAUNCH}" --help)"
echo "${HELP_OUT}" | grep -q -- '--signal-port' || fail "launch --help missing --signal-port"
echo "${HELP_OUT}" | grep -q '49100' || fail "launch default signal port must remain 49100"
echo "${HELP_OUT}" | grep -qi 'No Isaac Sim source checkout' || fail "launch --help should say no Isaac Sim checkout is required"
echo "${HELP_OUT}" | grep -qi 'this machine' || fail "launch --help should explain 127.0.0.1 is this machine only"
echo "${HELP_OUT}" | grep -qi 'no confirmation prompt' || fail "launch --help should say there is no confirmation prompt"
echo "${HELP_OUT}" | grep -q 'nvcr.io/nvidia/isaac-sim' || fail "launch --help should restrict --image to nvcr.io/nvidia/isaac-sim"
echo "${HELP_OUT}" | grep -qi 'EULA' || fail "launch --help should mention EULA acknowledgement"
echo "${HELP_OUT}" | grep -F 'Isaac Sim Full Streaming App is loaded' || fail "launch --help should wait for Isaac Sim Full Streaming App is loaded"
grep -Fq 'Isaac Sim Full Streaming App is loaded' "${SCRIPT_DIR}/common.sh" || fail "common.sh missing streaming-app loaded marker"
grep -q 'wait_for_streaming_app' "${LAUNCH}" || fail "launch must wait_for_streaming_app before printing the URL"
"${PREFLIGHT}" --help >/dev/null
"${SCRIPT_DIR}/pull_isaac_sim_image.sh" --help >/dev/null
pass "--help (default signal port 49100)"

set +e
"${SCRIPT_DIR}/ngc_login.sh" --check >/tmp/isaac-sim-web-viewer-ngc-check.out 2>/tmp/isaac-sim-web-viewer-ngc-check.err
NGC_ST=$?
set -e
if [[ "${NGC_ST}" -eq 0 ]]; then
  pass "ngc_login --check: already logged in"
elif [[ "${NGC_ST}" -eq 3 ]] && grep -q ASK_USER_NGC_CREDENTIALS /tmp/isaac-sim-web-viewer-ngc-check.err; then
  pass "ngc_login --check asks the operator when nvcr.io credentials are missing"
else
  fail "ngc_login --check unexpected exit ${NGC_ST}"
fi

set +e
"${SCRIPT_DIR}/pull_isaac_sim_image.sh" --check-only >/tmp/isaac-sim-web-viewer-pull-check.out 2>/tmp/isaac-sim-web-viewer-pull-check.err
PULL_ST=$?
set -e
if [[ "${PULL_ST}" -eq 0 ]]; then
  pass "pull --check-only: image already local"
elif [[ "${PULL_ST}" -eq 1 ]]; then
  pass "pull --check-only: image not local (pull required before compose)"
else
  fail "pull --check-only unexpected exit ${PULL_ST}"
fi
# --offline skips the catalog fetch so this check does not require network
# access. --allow-fallback is required with --offline so a stale tag cannot be
# consumed as Latest.
RESOLVE_ST=0
RESOLVED="$(python3 "${SCRIPT_DIR}/resolve_ngc_isaac_sim_image.py" --offline --allow-fallback)" || RESOLVE_ST=$?
if [[ "${RESOLVE_ST}" -ne 0 ]]; then
  fail "resolver --allow-fallback exited ${RESOLVE_ST}"
elif [[ "${RESOLVED}" != nvcr.io/nvidia/isaac-sim:* ]]; then
  fail "resolver did not print an nvcr image (got: ${RESOLVED})"
else
  pass "NGC image resolved ${RESOLVED}"
fi

python3 - "${SKILL_ROOT}" <<'PY' || fail "permitted-download scan failed"
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
scripts = root / "scripts"
compose = (root / "docker" / "docker-compose.yml").read_text()
dockerfile = (root / "docker" / "web-viewer" / "Dockerfile").read_text()

for line in dockerfile.splitlines():
    s = line.strip()
    if s.startswith("FROM "):
        image = s.split()[1]
        if not image.startswith("nvcr.io/"):
            raise SystemExit(f"Dockerfile FROM is not NGC: {s}")

if "docker.io" in compose or "hub.docker.com" in compose:
    raise SystemExit("compose file mentions Docker Hub")
if "nvcr.io/nvidia/isaac-sim:" not in compose:
    raise SystemExit("compose file missing nvcr.io/nvidia/isaac-sim")

host_forbidden = re.compile(
    r"(apt-get\s+install|pip3?\s+install|\bnpm\s+install|\bnpx\s|"
    r"curl\s+[^\n]*\|\s*bash|wget\s+|git\s+clone|conda\s+install|snap\s+install)",
    re.I,
)
for path in sorted(scripts.glob("*")):
    if path.suffix not in {".sh", ".py"}:
        continue
    text = path.read_text()
    for i, raw in enumerate(text.splitlines(), 1):
        stripped = raw.strip()
        if stripped.startswith("#"):
            continue
        if host_forbidden.search(raw):
            raise SystemExit(f"{path.name}:{i}: host script must not download extra software: {stripped}")
print("ok")
PY
pass "scripts and docker files stay on NGC / no extra host downloads"

set +e
"${SCRIPT_DIR}/pull_isaac_sim_image.sh" --image ubuntu:24.04 --check-only >/tmp/isaac-sim-web-viewer-refuse-hub.out 2>/tmp/isaac-sim-web-viewer-refuse-hub.err
HUB_ST=$?
set -e
[[ "${HUB_ST}" -eq 4 ]] || fail "pull --image ubuntu:24.04 should exit 4 (got ${HUB_ST})"
grep -q ASK_USER_IMAGE_TAG /tmp/isaac-sim-web-viewer-refuse-hub.err || fail "refuse-hub stderr missing ASK_USER_IMAGE_TAG"
pass "pull refuses Docker Hub ubuntu:24.04"

set +e
"${LAUNCH}" --image docker.io/library/node:22 --signal-port "${LOCAL_SIGNAL_PORT}" --dry-run >/tmp/isaac-sim-web-viewer-refuse-node.out 2>/tmp/isaac-sim-web-viewer-refuse-node.err
NODE_ST=$?
set -e
[[ "${NODE_ST}" -eq 4 ]] || fail "launch --image docker.io/library/node:22 --dry-run should exit 4 (got ${NODE_ST})"
pass "launch dry-run refuses Docker Hub node image"

set +e
"${LAUNCH}" --host 0.0.0.0 --dry-run >/tmp/isaac-sim-web-viewer-refuse-wildcard.out 2>/tmp/isaac-sim-web-viewer-refuse-wildcard.err
WILD_ST=$?
python3 "${SCRIPT_DIR}/validate_stream_bind.py" --host $'10.0.0.1\n/e s/.*/pwned/' >/tmp/isaac-sim-web-viewer-refuse-sed.out 2>/tmp/isaac-sim-web-viewer-refuse-sed.err
SED_ST=$?
set -e
[[ "${WILD_ST}" -eq 2 ]] || fail "launch --host 0.0.0.0 --dry-run should exit 2 (got ${WILD_ST})"
grep -q ASK_USER_HOST /tmp/isaac-sim-web-viewer-refuse-wildcard.err || fail "wildcard host stderr missing ASK_USER_HOST"
[[ "${SED_ST}" -eq 2 ]] || fail "sed-payload host should exit 2 (got ${SED_ST})"
pass "launch rejects wildcard and hostile ISAACSIM_HOST"

# Pin the image so dry-run does not fetch the NGC catalog.
OFFLINE_IMAGE="nvcr.io/nvidia/isaac-sim:6.0.1"
URL="$("${LAUNCH}" --image "${OFFLINE_IMAGE}" --signal-port "${LOCAL_SIGNAL_PORT}" --dry-run)"
[[ "${URL}" == "http://127.0.0.1:8210" ]] || fail "local dry-run URL should be http://127.0.0.1:8210 (got: ${URL})"
pass "dry-run URL ${URL} (this machine)"

AUTO_URL="$("${LAUNCH}" --image "${OFFLINE_IMAGE}" --host auto --signal-port "${LOCAL_SIGNAL_PORT}" --dry-run)"
[[ "${AUTO_URL}" == http://* ]] || fail "auto dry-run did not print an HTTP URL (got: ${AUTO_URL})"
pass "dry-run --host auto URL ${AUTO_URL}"

if "${PREFLIGHT}" --signal-port "${LOCAL_SIGNAL_PORT}"; then
  pass "preflight"
else
  pass "preflight ran (reported FAIL items; expected until NGC login and image are present)"
fi

pass "self_test complete"
