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

:: Add symlink to Isaac Examples
pushd "%~dp0"
if exist extension_examples cmd /c rmdir extension_examples
call mklink /D extension_examples exts\isaacsim.examples.interactive\isaacsim\examples\interactive
:: Powershell New-Item -ItemType Junction -Path "extension_examples" -Target "exts\isaacsim.examples.interactive\isaacsim\examples\interactive"
if %ERRORLEVEL% neq 0 (echo "Symlink extension_examples not created.") else (echo "Symlink extension_examples created.")
popd
