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

"""Deprecated surface gripper command implementations."""

import carb
import omni.kit.commands
from pxr import Usd

from .surface_gripper import create_surface_gripper


class CreateSurfaceGripper(omni.kit.commands.Command):
    """Deprecated command that creates a Surface Gripper prim.

    .. deprecated:: Use create_surface_gripper(stage, prim_path) directly instead.

    Args:
        prim_path: Path where the surface gripper will be created.
    """

    def __init__(self, prim_path: str = "") -> None:
        carb.log_warn(
            "CreateSurfaceGripper command is deprecated and will be removed in a future version. "
            "Use create_surface_gripper(stage, prim_path) directly instead."
        )
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        self._stage = omni.usd.get_context().get_stage()
        if prim_path == "":
            selection = omni.usd.get_context().get_selection()
            paths = selection.get_selected_prim_paths()
            if paths:
                prim_path = paths[0]
            else:
                default_prim = self._stage.GetDefaultPrim()
                if default_prim and default_prim.IsValid():
                    prim_path = str(default_prim.GetPath())
                else:
                    prim_path = "/"
        self._prim_path = prim_path

    def do(self) -> Usd.Prim:
        """Creates the Surface Gripper prim at the specified path.

        Returns:
            The created Surface Gripper prim.
        """
        self._prim = create_surface_gripper(self._stage, self._prim_path)
        self._prim_path = str(self._prim.GetPath())
        return self._prim

    def undo(self) -> bool | None:
        """Removes the created Surface Gripper prim from the stage.

        Returns:
            True if the prim was successfully removed, False otherwise.
        """
        if self._prim:
            return self._stage.RemovePrim(self._prim_path)
        return None


omni.kit.commands.register_all_commands_in_module(__name__)
