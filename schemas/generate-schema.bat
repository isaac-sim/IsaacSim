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
setlocal enabledelayedexpansion

REM Default configuration
set "config=release"
set "platform=windows-x86_64"

REM Parse command line arguments
:parse_args
if "%~1"=="" goto end_parse
if "%~1"=="-c" (
    set "config=%~2"
    shift
    shift
    goto parse_args
)
if "%~1"=="--config" (
    set "config=%~2"
    shift
    shift
    goto parse_args
)
if "%~1"=="-p" (
    set "platform=%~2"
    shift
    shift
    goto parse_args
)
if "%~1"=="--platform" (
    set "platform=%~2"
    shift
    shift
    goto parse_args
)
echo Unknown option: %~1
echo Usage: %0 [-c^|--config release^|debug] [-p^|--platform windows-x86_64^|linux-x86_64^|linux-aarch64]
exit /b 1

:end_parse

REM Validate configuration
if not "%config%"=="release" if not "%config%"=="debug" (
    echo Error: Configuration must be 'release' or 'debug'
    exit /b 1
)

REM Validate platform
if not "%platform%"=="windows-x86_64" if not "%platform%"=="linux-x86_64" if not "%platform%"=="linux-aarch64" (
    echo Error: Platform must be 'windows-x86_64', 'linux-x86_64', or 'linux-aarch64'
    exit /b 1
)

REM Get the repo root directory

REM The batch file is in schemas/ subdirectory
REM Check if %~dp0 contains "\schemas" to determine if we're in the right location
echo %~dp0 | find "\schemas" >nul
if %ERRORLEVEL% EQU 0 (
    pushd "%~dp0"
    cd ..
    set "repo_root=%CD%"
    popd
) else (
    set "repo_root=%CD%"
)

REM Set PATH to include USD bin and lib directories (for DLLs)
set "PATH=%repo_root%\_build\target-deps\usd\%config%\bin;%repo_root%\_build\target-deps\usd\%config%\lib;%PATH%"

REM Find the omni.kit.pip_archive directory in extscache
set "pip_archive_path="
for /d %%i in ("%repo_root%\_build\%platform%\%config%\extscache\omni.kit.pip_archive-*") do (
    set "pip_archive_path=%%i\pip_prebundle"
    goto :found_pip_archive
)
:found_pip_archive

REM Set PYTHONPATH with Windows path separators
set "PYTHONPATH=%repo_root%\_build\target-deps\usd\%config%\lib\python;%repo_root%\_build\target-deps\pip_cloud_prebundle;%repo_root%\_build\%platform%\%config%\kit\exts\omni.kit.pip_archive\pip_prebundle"
if defined pip_archive_path (
    set "PYTHONPATH=%PYTHONPATH%;%pip_archive_path%"
)

REM Execute the usdGenSchema command
"%repo_root%\_build\target-deps\python\python.exe" "%repo_root%\_build\target-deps\usd\%config%\bin\usdGenSchema" "%repo_root%\source\extensions\isaacsim.robot.schema\robot_schema\RobotSchema.usda" "%repo_root%\source\extensions\isaacsim.robot.schema\robot_schema"
"%repo_root%\_build\target-deps\python\python.exe" "%repo_root%\_build\target-deps\usd\%config%\bin\usdGenSchema" "%repo_root%\source\extensions\isaacsim.robot.schema\sensor_schema\SensorSchema.usda" "%repo_root%\source\extensions\isaacsim.robot.schema\sensor_schema"
"%repo_root%\_build\target-deps\python\python.exe" "%repo_root%\_build\target-deps\usd\%config%\bin\usdGenSchema" "%repo_root%\source\extensions\isaacsim.robot.schema\range_sensor_schema\RangeSensorSchema.usda" "%repo_root%\source\extensions\isaacsim.robot.schema\range_sensor_schema"
"%repo_root%\_build\target-deps\python\python.exe" "%repo_root%\_build\target-deps\usd\%config%\bin\usdGenSchema" "%repo_root%\source\extensions\isaacsim.robot_motion.schema\robot_motion_schema\RobotMotionSchema.usda" "%repo_root%\source\extensions\isaacsim.robot_motion.schema\robot_motion_schema"