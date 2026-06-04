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

"""High level wrapper for creating/encapsulating USD PreviewSurface material prims for basic rendering."""

from __future__ import annotations

import carb
import isaacsim.core.utils.stage as stage_utils
import numpy as np
from isaacsim.core.api.materials.visual_material import VisualMaterial
from pxr import Gf, Sdf, UsdShade


class PreviewSurface(VisualMaterial):
    """USD PreviewSurface material for basic rendering.

    Args:
        prim_path: USD prim path for the material.
        name: Name identifier.
        shader: Existing shader to use.
        color: Diffuse color RGB.
        roughness: Surface roughness (0-1).
        metallic: Metallic value (0-1).

    Raises:
        ValueError: If the material's shader is not of type USD Preview Surface.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "preview_surface",
        shader: UsdShade.Shader | None = None,
        color: np.ndarray | None = None,
        roughness: float | None = None,
        metallic: float | None = None,
    ) -> None:
        stage = stage_utils.get_current_stage()
        if stage.GetPrimAtPath(prim_path).IsValid():
            carb.log_info(f"Material Prim already defined at path: {prim_path}")
            material = UsdShade.Material(stage.GetPrimAtPath(prim_path))
        else:
            material = UsdShade.Material.Define(stage, prim_path)

        if shader is None:
            if stage.GetPrimAtPath(f"{prim_path}/shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/shader"))
            elif stage.GetPrimAtPath(f"{prim_path}/Shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/Shader"))
            else:
                shader = UsdShade.Shader.Define(stage, f"{prim_path}/shader")
        VisualMaterial.__init__(
            self,
            prim_path=prim_path,
            prim=stage.GetPrimAtPath(prim_path),
            shaders_list=[shader],
            material=material,
            name=name,
        )
        shader_id = shader.GetIdAttr().Get()
        if shader_id and shader_id != "UsdPreviewSurface":
            raise ValueError(
                f"The material's shader at path {prim_path} (with id {shader_id}) is not of type USD Preview Surface"
            )
        shader.CreateIdAttr("UsdPreviewSurface")
        if color is not None:
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*color.tolist()))
        if roughness is not None:
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        if metallic is not None:
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return

    def set_color(self, color: np.ndarray) -> None:
        """Set the diffuse color.

        Args:
            color: RGB color array.

        """
        if self.shaders_list[0].GetInput("diffuseColor").Get() is None:
            self.shaders_list[0].CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*color.tolist()))
        else:
            self.shaders_list[0].GetInput("diffuseColor").Set(Gf.Vec3f(*color.tolist()))
        return

    def get_color(self) -> np.ndarray:
        """Get the diffuse color.

        Returns:
            RGB color array or None if not set.

        """
        if self.shaders_list[0].GetInput("diffuseColor").Get() is None:
            carb.log_warn("A color attribute is not set yet")
            return None
        else:
            return np.array(self.shaders_list[0].GetInput("diffuseColor").Get())

    def set_roughness(self, roughness: float) -> None:
        """Set the surface roughness.

        Args:
            roughness: Roughness value (0-1).

        """
        if self.shaders_list[0].GetInput("roughness").Get() is None:
            self.shaders_list[0].CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        else:
            self.shaders_list[0].GetInput("roughness").Set(roughness)
        return

    def get_roughness(self) -> float:
        """Get the surface roughness.

        Returns:
            Roughness value or None if not set.

        """
        if self.shaders_list[0].GetInput("roughness").Get() is None:
            carb.log_warn("A roughness attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("roughness").Get()

    def set_metallic(self, metallic: float) -> None:
        """Set the metallic value.

        Args:
            metallic: Metallic value (0-1).

        """
        if self.shaders_list[0].GetInput("metallic").Get() is None:
            self.shaders_list[0].CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        else:
            self.shaders_list[0].GetInput("metallic").Set(metallic)
        return

    def get_metallic(self) -> float:
        """Get the metallic value.

        Returns:
            Metallic value or None if not set.

        """
        if self.shaders_list[0].GetInput("metallic").Get() is None:
            carb.log_warn("A metallic attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("metallic").Get()
