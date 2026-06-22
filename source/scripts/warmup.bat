:: SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
setlocal

:: Use half of available CPU cores for the warmup not to take all the resources from user's PC during installation
set /a TASKING_THREAD_CNT = %NUMBER_OF_PROCESSORS% / 2
set PORTABLE_ROOT=%~dp0portable_root
call "%~dp0kit\kit.exe"  "%%~dp0apps/isaacsim.exp.base.kit" ^
    --no-window ^
    --portable-root "%PORTABLE_ROOT%" ^
    --portable ^
    --/persistent/renderer/startupMessageDisplayed=true ^
    --ext-folder "%~dp0/exts" ^
    --ext-folder "%~dp0/apps" ^
    --/app/extensions/excluded/2='omni.kit.telemetry' ^
    --/app/settings/persistent=0 ^
    --/app/settings/loadUserConfig=0 ^
    --/structuredLog/enable=0 ^
    --/app/hangDetector/enabled=0 ^
    --/crashreporter/skipOldDumpUpload=1 ^
    --/app/content/emptyStageOnStart=1 ^
    --/rtx/materialDb/syncLoads=1 ^
    --/omni/kit/plugin/syncUsdLoads=1 ^
    --/rtx/hydra/materialSyncLoads=1 ^
    --/app/asyncRendering=0 ^
    --/app/quitAfter=1000 ^
    --/app/fastShutdown=1 ^
    --/app/file/ignoreUnsavedStage=1 ^
    --/app/warmupMode=0 ^
    --/exts/omni.kit.registry.nucleus/registries/0/name=0 ^
    --/log/flushStandardStreamOutput=1 ^
    --/plugins/carb.tasking.plugin/threadCount=%TASKING_THREAD_CNT% ^
    %*
if %ERRORLEVEL% neq 0 (echo "Error warming up shader cache.") else (echo "Shader cache is warmed up.")

call "%~dp0python.bat" "%~dp0standalone_examples\testing\isaacsim.simulation_app\test_viewport_ready.py" ^
    --portable-root "%PORTABLE_ROOT%" ^
    --/log/flushStandardStreamOutput=1 ^
    --no-window ^
    --silent
if %ERRORLEVEL% neq 0 (echo "Error warming up python app shader cache.") else (echo "Python app shader cache is warmed up.")

:: Always succeed in case kit crashed or hanged
exit /b 0
