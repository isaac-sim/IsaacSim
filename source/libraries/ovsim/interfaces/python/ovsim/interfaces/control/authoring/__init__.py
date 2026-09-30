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
    "AddReferenceToStageFunction",
    "CloseStageFunction",
    "CreatePrimAttributeFunction",
    "CreateStageFunction",
    "DefinePrimFunction",
    "ExportStageToStringFunction",
    "GetParameterFunction",
    "ImportStageFromStringFunction",
    "InputParameterType",
    "MovePrimFunction",
    "OpenStageFunction",
    "OutputParameterType",
    "RemovePrimAttributeFunction",
    "RemovePrimFunction",
    "SaveStageFunction",
    "SetParameterFunction",
]

#: Values accepted by provider parameter setters.
InputParameterType = bool | int | float | str
#: Values returned by provider parameter getters.
OutputParameterType = bool | int | float | str

#: Create an empty stage.
CreateStageFunction = Callable[[], bool]
#: Open a stage from a USD path.
OpenStageFunction = Callable[[str], bool]
#: Save a stage to a USD path.
SaveStageFunction = Callable[[str], bool]
#: Import a stage from a USD string.
ImportStageFromStringFunction = Callable[[str], bool]
#: Export the stage to a USD string.
ExportStageToStringFunction = Callable[[], str]
#: Close the current stage.
CloseStageFunction = Callable[[], bool]
#: Add a referenced USD asset to the stage.
AddReferenceToStageFunction = Callable[[str, str, str], bool]

#: Define a prim.
DefinePrimFunction = Callable[[str, str], bool]
#: Move a prim.
MovePrimFunction = Callable[[str, str], bool]
#: Remove a prim.
RemovePrimFunction = Callable[[str], bool]

#: Create a prim attribute.
CreatePrimAttributeFunction = Callable[[str, str, str], bool]
#: Remove a prim attribute.
RemovePrimAttributeFunction = Callable[[str, str], bool]

#: Set a named provider parameter.
SetParameterFunction = Callable[[str, str, InputParameterType], None]
#: Get a named provider parameter.
GetParameterFunction = Callable[[str, str], OutputParameterType]
