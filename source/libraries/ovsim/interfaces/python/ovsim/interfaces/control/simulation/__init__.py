# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

from collections.abc import Callable

__all__ = [
    "GetParameterFunction",
    "InitializeFunction",
    "InputParameterType",
    "InvalidateFunction",
    "OutputParameterType",
    "PauseFunction",
    "PlayFunction",
    "SetParameterFunction",
    "StepFunction",
    "StopFunction",
]

#: Values accepted by provider parameter setters.
InputParameterType = bool | int | float | str
#: Values returned by provider parameter getters.
OutputParameterType = bool | int | float | str

#: Start or resume simulation.
PlayFunction = Callable[[], None]
#: Pause simulation.
PauseFunction = Callable[[], None]
#: Stop simulation.
StopFunction = Callable[[], None]
#: Initialize simulation resources.
InitializeFunction = Callable[[], None]
#: Invalidate simulation resources.
InvalidateFunction = Callable[[], None]
#: Advance simulation by one step.
StepFunction = Callable[[], None]

#: Set a named provider parameter.
SetParameterFunction = Callable[[str, str, InputParameterType], None]
#: Get a named provider parameter.
GetParameterFunction = Callable[[str, str], OutputParameterType]
