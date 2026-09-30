:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0

@echo off
setlocal enableextensions

set "PYTHONDONTWRITEBYTECODE=1"
call "%~dp0..\..\tools\packman\python.bat" "%~dp0..\..\tools\pixi_run.py" examples -- %*
exit /b %ERRORLEVEL%
