#!/bin/bash
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

if [ -z "$ACCEPT_EULA" ]
then
    echo
    echo 'The NVIDIA Isaac Sim Additional Software and Materials License must be accepted before'
    echo 'Isaac Sim can start. The license terms for this product can be viewed at'
    echo 'https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/'
    echo
    echo 'Please accept the EULA above by setting the ACCEPT_EULA environment variable.'
    echo 'e.g.: -e "ACCEPT_EULA=Y"'
    echo
    exit 1
else
    echo
    echo 'The NVIDIA Isaac Sim Additional Software and Materials License must be accepted before'
    echo 'Isaac Sim can start. The license terms for this product can be viewed at'
    echo 'https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/'
    echo
fi
