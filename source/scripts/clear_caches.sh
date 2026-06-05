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

echo "Clearing Caches Script"
echo "Note: This script will delete folders on you system and is not reversible."

CLEAR_PATH=~/.cache/ov/shaders
echo -e "\nClearing shader cache... ${CLEAR_PATH}"
read -p "Are you sure (Y/[N])? " -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    rm -rf ${CLEAR_PATH}
    echo -e "\nClearing shader cache DONE."
fi

CLEAR_PATH=~/.cache/ov/texturecache
echo -e "\nClearing texturecache... ${CLEAR_PATH}"
read -p "Are you sure (Y/[N])? " -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    rm -rf ${CLEAR_PATH}
    echo -e "\nClearing texturecache DONE."
fi

CLEAR_PATH=~/.cache/ov/Kit/107.3
echo -e "\nClearing Kit cache... ${CLEAR_PATH}"
read -p "Are you sure (Y/[N])? " -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    rm -rf ${CLEAR_PATH}
    echo -e "\nClearing Kit cache DONE."
fi

echo
read -n 1 -s -r -p "Press any key to continue"
