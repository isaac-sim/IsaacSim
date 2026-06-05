# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated simulation command utilities."""

import carb
import omni.kit.commands
import omni.kit.utils
from isaacsim.core.utils.bindings._isaac_utils import transforms
from isaacsim.core.utils.stage import get_current_stage, get_current_stage_id


class IsaacSimSpawnPrim(omni.kit.commands.Command):
    """Command to spawn a new prim in the stage and set its transform. This uses dynamic_control to properly handle physics objects and articulation.

    Typical usage example:

    .. code-block:: python

        omni.kit.commands.execute(
            "IsaacSimSpawnPrim",
            usd_path="/path/to/file.usd",
            prim_path="/World/Prim",
            translation=(0, 0, 0),
            rotation=(0, 0, 0, 1),
        )

    Args:
        usd_path: Path to the USD file to reference.
        prim_path: Path where the prim will be created in the stage.
        translation: Translation vector for the prim's position.
        rotation: Rotation quaternion for the prim's orientation.
    """

    def __init__(
        self, usd_path: str, prim_path: str, translation: carb.Float3 = (0, 0, 0), rotation: carb.Float4 = (0, 0, 0, 1)
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._stage = get_current_stage()
        self._stage_id = get_current_stage_id()
        self._context = omni.usd.get_context()

    def do(self) -> bool:
        """Spawn a new prim in the stage with the specified USD reference and transform.

        Defines an Xform prim at the given path, adds the USD file as a reference, and applies the specified
        transformation using dynamic control for proper physics handling.

        Returns:
            True when the spawn operation completes successfully.
        """
        self._prim = self._stage.DefinePrim(self._prim_path, "Xform")
        self._prim.GetReferences().AddReference(self._usd_path)
        if self._translation is not None and self._rotation is not None:
            transforms.set_transform(
                self._stage_id,
                str(self._prim.GetPath()),
                tuple(self._translation),
                tuple(self._rotation),
            )

        return True

    def undo(self) -> None:
        """Undo the prim spawn operation.

        Currently not implemented - the spawned prim remains in the stage.
        """


class IsaacSimTeleportPrim(omni.kit.commands.Command):
    """Command to set a transform of a prim. This uses dynamic_control to properly handle physics objects and articulation.

    Typical usage example:

    .. code-block:: python

        omni.kit.commands.execute(
            "IsaacSimTeleportPrim",
            prim_path="/World/Prim",
            translation=(0, 0, 0),
            rotation=(0, 0, 0, 1),
        )

    Args:
        prim_path: Path to the prim to teleport.
        translation: Translation vector as (x, y, z).
        rotation: Rotation quaternion as (x, y, z, w).
    """

    def __init__(
        self, prim_path: str, translation: carb.Float3 = (0, 0, 0), rotation: carb.Float4 = (0, 0, 0, 1)
    ) -> None:
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._stage = get_current_stage()
        self._stage_id = get_current_stage_id()
        self._context = omni.usd.get_context()

    def do(self) -> bool:
        """Execute the teleport operation by setting the prim's transform.

        Returns:
            True when the teleport operation completes successfully.
        """
        if self._translation is not None and self._rotation is not None:
            transforms.set_transform(self._stage_id, str(self._prim_path), self._translation, self._rotation)
        return True

    def undo(self) -> None:
        """Undo the teleport operation.

        Note:
            This method currently has no implementation and does not restore the prim's previous transform.
        """


class IsaacSimScalePrim(omni.kit.commands.Command):
    """Command to set a scale of a prim.

    Typical usage example:

    .. code-block:: python

        omni.kit.commands.execute(
            "IsaacSimScalePrim",
            prim_path="/World/Prim",
            scale=(1.5, 1.5, 1.5),
        )

    Args:
        prim_path: Path to the prim to scale.
        scale: Scale values for x, y, and z axes.
    """

    def __init__(self, prim_path: str, scale: carb.Float3 = (0, 0, 0)) -> None:
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._stage = get_current_stage()
        self._stage_id = get_current_stage_id()
        self._context = omni.usd.get_context()

    def do(self) -> bool:
        """Execute the prim scaling operation by applying the specified scale values.

        Returns:
            True if the scaling operation was successful.
        """
        if self._scale is not None:
            transforms.set_scale(self._stage_id, str(self._prim_path), self._scale)
        return True

    def undo(self) -> None:
        """Revert the prim scaling operation."""


class IsaacSimDestroyPrim(omni.kit.commands.Command):
    """Command to delete a prim. This variant has less overhead than other commands as it doesn't store an undo operation.

    Typical usage example:

    .. code-block:: python

        omni.kit.commands.execute(
            "IsaacSimDestroyPrim",
            prim_path="/World/Prim",
        )

    Args:
        prim_path: Path to the prim to delete.
    """

    def __init__(self, prim_path: str) -> None:
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)

    def do(self) -> bool:
        """Delete the prim from the stage.

        Returns:
            True if the deletion command was executed.
        """
        delete_cmd = omni.usd.commands.DeletePrimsCommand([self._prim_path])
        delete_cmd.do()

    def undo(self) -> None:
        """No-op undo operation as this command doesn't support undo functionality."""


omni.kit.commands.register_all_commands_in_module(__name__)
