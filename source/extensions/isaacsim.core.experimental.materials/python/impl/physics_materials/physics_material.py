# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from abc import ABC, abstractmethod

import isaacsim.core.experimental.utils.stage as stage_utils
import warp as wp
from isaacsim.core.experimental.prims import Prim
from pxr import Usd, UsdShade


class PhysicsMaterial(Prim, ABC):
    """Base class for physics materials.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        resolve_paths: Whether to resolve the given paths (true) or use them as is (false).
    """

    def __init__(self, paths: str | list[str], *, resolve_paths: bool = True) -> None:
        super().__init__(paths, resolve_paths=resolve_paths)
        if not hasattr(self, "_materials"):
            self._materials = []

    """
    Properties.
    """

    @property
    def materials(self) -> list[UsdShade.Material]:
        """USD materials encapsulated by the wrapper.

        Returns:
            List of USD materials.

        Example:

        .. code-block:: python

            >>> prims.materials
            [UsdShade.Material(Usd.Prim(</World/prim_0>)),
             UsdShade.Material(Usd.Prim(</World/prim_1>)),
             UsdShade.Material(Usd.Prim(</World/prim_2>))]
        """
        return self._materials

    """
    Static methods.
    """

    @staticmethod
    @abstractmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating material instances of this type.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating material instances (shape ``(N, 1)``).
        """
        pass

    @staticmethod
    def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[PhysicsMaterial | None]:
        """Fetch instances of physics material from prims (or prim paths) at the given paths.

        Backends: :guilabel:`usd`.

        Args:
            paths: Prim paths (or prims) to get physics material instances from.

        Returns:
            List of physics material instances or ``None`` if the prim is not a supported physics material type.

        Example:

        .. code-block:: python

            >>> import omni.kit.commands
            >>> import isaacsim.core.experimental.utils.stage as stage_utils
            >>> from isaacsim.core.experimental.materials import PhysicsMaterial
            >>>
            >>> # given a USD stage with the prims at paths /World, /World/A (USD Preview Surface)
            >>> omni.kit.commands.execute(
            ...     "AddRigidBodyMaterialCommand", stage=stage_utils.get_current_stage(), path="/World/A"
            ... )  # doctest: +NO_CHECK
            >>>
            >>> # fetch physics material instances
            >>> PhysicsMaterial.fetch_instances(["/World", "/World/A"])
            [None, <isaacsim.core.experimental.materials.impl.physics_materials.rigid_body.RigidBodyMaterial object at 0x...>]
        """
        # defer imports to avoid circular dependencies
        from .rigid_body import RigidBodyMaterial
        from .surface_deformable import SurfaceDeformableMaterial
        from .volume_deformable import VolumeDeformableMaterial

        classes = [RigidBodyMaterial, SurfaceDeformableMaterial, VolumeDeformableMaterial]

        instances = []
        stage = stage_utils.get_current_stage(backend="usd")
        for item in paths if isinstance(paths, (list, tuple)) else [paths]:
            prim = stage.GetPrimAtPath(item) if isinstance(item, str) else item
            instance = None
            for cls in classes:
                if cls.are_of_type(prim).numpy().item():
                    instance = cls(prim.GetPath().pathString)
                    break
            instances.append(instance)
        return instances

    """
    Internal static methods.
    """

    @staticmethod
    def _get_material(stage: Usd.Stage, path: str) -> UsdShade.Material | None:
        """Get the material for a given material path."""
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.IsA(UsdShade.Material):
            return UsdShade.Material(prim)
        return None
