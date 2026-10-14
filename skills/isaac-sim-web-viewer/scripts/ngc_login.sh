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

# ngc_login.sh — Check or perform docker login to nvcr.io. Never prints the API key.

# Check or perform docker login to nvcr.io.
# Exit 0: logged in (or login succeeded)
# Exit 3: credentials missing — operator must docker login locally (not in chat)
# Exit 1: login attempted and failed
#
# Usage:
#   ngc_login.sh --check
#   NGC_API_KEY=... ngc_login.sh
#   ngc_login.sh --username '$oauthtoken'   # key from NGC_API_KEY / NGC_CLI_API_KEY
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

CHECK_ONLY=0
USERNAME='$oauthtoken'
if [[ -n "${NGC_USERNAME:-}" ]]; then
  USERNAME="${NGC_USERNAME}"
fi

usage() {
  cat <<EOF
Usage: $(basename "$0") [--check] [--username USER]

  --check       Exit 0 if Docker already has nvcr.io credentials; else print
                ASK_USER_NGC_CREDENTIALS and exit 3
  --username    NGC username (default: \$oauthtoken)

If not already logged in, reads NGC_API_KEY or NGC_CLI_API_KEY from the
environment and runs: docker login nvcr.io --password-stdin
If those variables are empty, prints ASK_USER_NGC_CREDENTIALS and exits 3
(the operator must docker login locally; do not paste the key into chat).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) CHECK_ONLY=1; shift ;;
    --username) USERNAME="${2:?}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if nvcr_logged_in; then
  echo "OK: docker already has nvcr.io credentials"
  exit 0
fi

if [[ "${CHECK_ONLY}" -eq 1 ]]; then
  print_ask_ngc_credentials
  exit 3
fi

KEY=""
if ! KEY="$(ngc_api_key_from_env)"; then
  print_ask_ngc_credentials
  exit 3
fi

echo "Logging in to nvcr.io as ${USERNAME} (password via stdin)" >&2
if ! printf '%s' "${KEY}" | docker login nvcr.io -u "${USERNAME}" --password-stdin; then
  echo "FAIL: docker login nvcr.io failed. Re-run docker login nvcr.io locally; do not paste the NGC API key into chat." >&2
  exit 1
fi
echo "OK: docker login nvcr.io succeeded"
exit 0
