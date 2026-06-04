#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})

# Add symlink to Isaac Examples
echo Creating extension_examples symlink...
pushd ${SCRIPT_DIR}
if [ ! -L extension_examples ] && [ ! -e extension_examples ]; then
    ln -s exts/isaacsim.examples.interactive/isaacsim/examples/interactive extension_examples
    echo Symlink extension_examples created.
else
    echo Symlink or folder extension_examples exists.
fi
popd

# Install icon
echo Installing Icon...
ISAAC_SIM_SCRIPT="${SCRIPT_DIR}/isaac-sim.sh"
if [ -f "${ISAAC_SIM_SCRIPT}" ]; then
    # Convert to absolute path
    ABSOLUTE_ISAAC_SIM_SCRIPT=$(realpath "${ISAAC_SIM_SCRIPT}")
    echo "Found isaac-sim.sh at: ${ABSOLUTE_ISAAC_SIM_SCRIPT}"
    ${SCRIPT_DIR}/python.sh ${SCRIPT_DIR}/data/icon/install_icon.py "${ABSOLUTE_ISAAC_SIM_SCRIPT}"
else
    echo "Warning: isaac-sim.sh not found at ${ISAAC_SIM_SCRIPT}"
    ${SCRIPT_DIR}/python.sh ${SCRIPT_DIR}/data/icon/install_icon.py
fi

