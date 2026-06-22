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

echo 'Omniverse Software collects installation and configuration details about your software, hardware, and network'
echo 'configuration (e.g., version of operating system, applications installed, type of hardware, network speed, IP'
echo 'address) based on our legitimate interest in improving your experience. To improve performance, troubleshooting'
echo 'and diagnostic purposes of our software, we also collect session behavior, error and crash logs.'
echo
echo 'Data Collection in container mode is completely anonymous unless specified. You may opt-out of this collection'
echo 'anytime by not setting the PRIVACY_CONSENT environment variable.'
echo
echo 'To opt-in set the PRIVACY_CONSENT environment variable when running the container. Set the '
echo 'PRIVACY_USERID environment variable to tag the telemetry data with a user ID.'
