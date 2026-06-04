# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module for creating and managing physics materials with friction and restitution properties."""

from __future__ import annotations

import carb
import isaacsim.core.utils.stage as stage_utils
from pxr import Usd, UsdPhysics, UsdShade


class PhysicsMaterial(object):
    """Physics material for defining friction and restitution properties.

    Args:
        prim_path: USD prim path for the material.
        name: Name identifier.
        static_friction: Static friction coefficient.
        dynamic_friction: Dynamic friction coefficient.
        restitution: Restitution (bounciness) coefficient.

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "physics_material",
        static_friction: float | None = None,
        dynamic_friction: float | None = None,
        restitution: float | None = None,
    ) -> None:
        self._name = name
        self._prim_path = prim_path

        stage = stage_utils.get_current_stage()
        if stage.GetPrimAtPath(prim_path).IsValid():
            carb.log_info(f"Physics Material Prim already defined at path: {prim_path}")
            self._material = UsdShade.Material(stage.GetPrimAtPath(prim_path))
        else:
            self._material = UsdShade.Material.Define(stage, prim_path)
        self._prim = stage.GetPrimAtPath(prim_path)
        if self._prim.HasAPI(UsdPhysics.MaterialAPI):
            self._material_api = UsdPhysics.MaterialAPI(self._prim)
        else:
            self._material_api = UsdPhysics.MaterialAPI.Apply(self._prim)
        if static_friction is not None:
            self.set_static_friction(static_friction)
        if dynamic_friction is not None:
            self.set_dynamic_friction(dynamic_friction)
        if restitution is not None:
            self.set_restitution(restitution)
        return

    @property
    def prim_path(self) -> str:
        """USD prim path for the material.

        Returns:
            The prim path string.

        """
        return self._prim_path

    @property
    def prim(self) -> Usd.Prim:
        """USD prim object for the material.

        Returns:
            The Usd.Prim object.

        """
        return self._prim

    @property
    def name(self) -> str:
        """Material name identifier.

        Returns:
            The material name.

        """
        return self._name

    @property
    def material(self) -> UsdShade.Material:
        """USD material object for the physics material.

        Returns:
            The UsdShade.Material object.

        """
        return self._material

    def set_dynamic_friction(self, friction: float) -> None:
        """Set the dynamic friction coefficient.

        Args:
            friction: The dynamic friction coefficient value.

        """
        attr = self._material_api.GetDynamicFrictionAttr()
        if attr.Get() is None:
            self._material_api.CreateDynamicFrictionAttr().Set(friction)
        else:
            attr.Set(friction)
        return

    def get_dynamic_friction(self) -> float:
        """Get the dynamic friction coefficient.

        Returns:
            The dynamic friction coefficient value.

        """
        attr = self._material_api.GetDynamicFrictionAttr()
        value = attr.Get()
        if value is None:
            carb.log_warn("A dynamic friction attribute is not set yet")
            return None
        return value

    def set_static_friction(self, friction: float) -> None:
        """Set the static friction coefficient.

        Args:
            friction: The static friction coefficient value.

        """
        attr = self._material_api.GetStaticFrictionAttr()
        if attr.Get() is None:
            self._material_api.CreateStaticFrictionAttr().Set(friction)
        else:
            attr.Set(friction)
        return

    def get_static_friction(self) -> float:
        """Get the static friction coefficient.

        Returns:
            The static friction coefficient value.

        """
        attr = self._material_api.GetStaticFrictionAttr()
        value = attr.Get()
        if value is None:
            carb.log_warn("A static friction attribute is not set yet")
            return None
        return value

    def set_restitution(self, restitution: float) -> None:
        """Set the restitution (bounciness) coefficient.

        Args:
            restitution: The restitution coefficient value.

        """
        attr = self._material_api.GetRestitutionAttr()
        if attr.Get() is None:
            self._material_api.CreateRestitutionAttr().Set(restitution)
        else:
            attr.Set(restitution)
        return

    def get_restitution(self) -> float:
        """Get the restitution (bounciness) coefficient.

        Returns:
            The restitution coefficient value.

        """
        attr = self._material_api.GetRestitutionAttr()
        value = attr.Get()
        if value is None:
            carb.log_warn("A restitution attribute is not set yet")
            return None
        return value
