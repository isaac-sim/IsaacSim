#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

SCRIPT_DIR=$(dirname ${BASH_SOURCE})

# Check EULA acceptance first
"${SCRIPT_DIR}/tools/eula_check.sh"
EULA_STATUS=$?

if [ $EULA_STATUS -ne 0 ]; then
    echo "Error: NVIDIA Software License Agreement and Product-Specific Terms for NVIDIA Omniverse must be accepted to proceed."
    exit 1
fi

# Provision libsurvive into _build/target-deps on Linux (no-op if already installed)
if [[ "$(uname -s)" == "Linux" ]]; then
    bash "${SCRIPT_DIR}/source/scripts/build_libsurvive.sh" "${SCRIPT_DIR}"
fi

set -e
source "$SCRIPT_DIR/repo.sh" build $@ || exit $?
