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

"${ISAAC_SIM_DIR:?ISAAC_SIM_DIR is not set}/isaac-sim.sh"

# Use a system ROS 2 install (needed when bundled libs lack message types or
# RMW implementations you require). Source in the parent shell so
# `LD_LIBRARY_PATH` is set before the process starts.
# source /opt/ros/humble/setup.bash
# "${ISAAC_SIM_DIR:?}/isaac-sim.sh"

# Disable auto-sourcing of `setup_ros_env.sh`. The parent shell must already
# provide `ROS_DISTRO`, `RMW_IMPLEMENTATION`, and `LD_LIBRARY_PATH`.
# "${ISAAC_SIM_DIR:?}/isaac-sim.sh" --no-ros-env
