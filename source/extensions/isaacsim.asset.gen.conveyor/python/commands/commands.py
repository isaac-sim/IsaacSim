# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated conveyor belt command implementations."""

from __future__ import annotations

__all__ = ["CreateConveyorBelt"]

from typing import Any

import carb
import omni
import omni.kit.commands
from pxr import Usd

from ..impl.conveyor import create_conveyor_belt


class CreateConveyorBelt(omni.kit.commands.Command):
    """Deprecated command that creates a conveyor belt action graph.

    .. deprecated:: Use create_conveyor_belt(stage, conveyor_prim) directly instead.

    Args:
        prim_name: Name for the conveyor belt graph prim.
        conveyor_prim: The rigid body prim to apply the conveyor belt to.
    """

    def __init__(self, prim_name: str = "ConveyorBeltGraph", conveyor_prim: Any = None) -> None:
        carb.log_warn(
            "CreateConveyorBelt command is deprecated and will be removed in a future version. "
            "Use create_conveyor_belt(stage, conveyor_prim) directly instead."
        )
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        self._stage = omni.usd.get_context().get_stage()
        self._prim_path = None

    def do(self) -> Usd.Prim:
        """Creates the conveyor belt action graph.

        Returns:
            The created conveyor node prim.
        """
        if self._conveyor_prim is None:
            _selection = omni.usd.get_context().get_selection()
            selected_paths = _selection.get_selected_prim_paths()
            if selected_paths:
                self._conveyor_prim = self._stage.GetPrimAtPath(selected_paths[0])
            else:
                default_prim = self._stage.GetDefaultPrim()
                if default_prim and default_prim.IsValid():
                    self._conveyor_prim = default_prim
                else:
                    self._conveyor_prim = self._stage.GetPrimAtPath("/")

        self._prim = create_conveyor_belt(self._stage, self._conveyor_prim, self._prim_name)
        self._prim_path = str(self._prim.GetParent().GetPath())
        return self._prim

    def undo(self) -> bool:
        """Removes the created conveyor belt action graph.

        Returns:
            True if the prim was successfully removed.
        """
        if self._prim:
            return self._stage.RemovePrim(self._prim_path)


omni.kit.commands.register_all_commands_in_module(__name__)
