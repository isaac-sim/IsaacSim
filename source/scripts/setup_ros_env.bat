:: SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
::
:: Licensed under the Apache License, Version 2.0 (the "License");
:: you may not use this file except in compliance with the License.
:: You may obtain a copy of the License at
::
:: http://www.apache.org/licenses/LICENSE-2.0
::
:: Unless required by applicable law or agreed to in writing, software
:: distributed under the License is distributed on an "AS IS" BASIS,
:: WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
:: See the License for the specific language governing permissions and
:: limitations under the License.

@echo off

REM setup_ros_env.bat - Configure ROS2 environment for Isaac Sim on Windows

REM Get script directory and set Isaac Sim root path
set SCRIPT_DIR=%~dp0
REM Remove trailing backslash from SCRIPT_DIR
set SCRIPT_DIR=%SCRIPT_DIR:~0,-1%
set ISAAC_SIM_ROOT=%SCRIPT_DIR%

set DEFAULT_ROS_DISTRO=humble

set BRIDGE_EXT_PATH=%ISAAC_SIM_ROOT%\exts\isaacsim.ros2.bridge

REM Set ROS_DISTRO if not already set
if "%ROS_DISTRO%"=="" (
    set ROS_DISTRO=%DEFAULT_ROS_DISTRO%

    REM Update PATH to include ROS2 bridge libraries
    set "PATH=%PATH%;%BRIDGE_EXT_PATH%\%DEFAULT_ROS_DISTRO%\lib"
    
)

REM Set RMW implementation to Fast DDS if not already set
if "%RMW_IMPLEMENTATION%"=="" (
    set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
)
