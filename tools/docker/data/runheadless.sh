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

# Optional flags from env (only add each if set); pass before "$@" so docker run args can override
EXTRA_FLAGS=""
[ -n "${OMNI_SERVER}" ] && EXTRA_FLAGS="${EXTRA_FLAGS} --/persistent/isaac/asset_root/default=${OMNI_SERVER}"
[ -n "${ISAACSIM_HOST}" ] && EXTRA_FLAGS="${EXTRA_FLAGS} --/exts/omni.kit.livestream.app/primaryStream/publicIp=${ISAACSIM_HOST}"
[ -n "${ISAACSIM_SIGNAL_PORT}" ] && EXTRA_FLAGS="${EXTRA_FLAGS} --/exts/omni.kit.livestream.app/primaryStream/signalPort=${ISAACSIM_SIGNAL_PORT}"
[ -n "${ISAACSIM_STREAM_PORT}" ] && EXTRA_FLAGS="${EXTRA_FLAGS} --/exts/omni.kit.livestream.app/primaryStream/streamPort=${ISAACSIM_STREAM_PORT}"

/isaac-sim/license.sh && /isaac-sim/privacy.sh && /isaac-sim/isaac-sim.streaming.sh \
    --merge-config="/isaac-sim/config/open_endpoint.toml" \
    $EXTRA_FLAGS \
    "$@"
