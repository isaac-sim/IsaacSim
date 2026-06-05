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

echo "Clearing Caches Script"
echo "Note: This script will delete folders on you system and is not reversible."

:SHADERCACHE
SET "CLEAR_PATH=%userprofile%\AppData\Local\ov\cache\shaders"
echo.
echo Clearing shader cache... (%CLEAR_PATH%)
SET /P AREYOUSURE=Are you sure (Y/[N])?
IF /I "%AREYOUSURE%" NEQ "Y" GOTO TEXTURECACHE
del /S /F /Q %CLEAR_PATH%
echo Clearing shader cache DONE.

:TEXTURECACHE
SET "CLEAR_PATH=%userprofile%\AppData\Local\ov\cache\texturecache"
echo.
echo Clearing texturecache... (%CLEAR_PATH%)
SET /P AREYOUSURE=Are you sure (Y/[N])?
IF /I "%AREYOUSURE%" NEQ "Y" GOTO KITCACHE
del /S /F /Q %CLEAR_PATH%
echo Clearing texturecache DONE.

:KITCACHE
SET "CLEAR_PATH=%userprofile%\AppData\Local\ov\cache\Kit\107.3"
echo.
echo Clearing Kit cache... (%CLEAR_PATH%)
SET /P AREYOUSURE=Are you sure (Y/[N])?
IF /I "%AREYOUSURE%" NEQ "Y" GOTO END
del /S /F /Q %CLEAR_PATH%
echo Clearing Kit cache DONE.


:END
endlocal

:: Always succeed in case kit crashed or hanged
pause
exit
