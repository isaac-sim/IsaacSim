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

"""High level wrapper for creating/encapsulating ``OmniPBR`` physically-based rendering material prims."""

from __future__ import annotations

import carb
import numpy as np
from isaacsim.core.api.materials.visual_material import VisualMaterial
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, UsdShade


class OmniPBR(VisualMaterial):
    """OmniPBR physically-based rendering material.

    Args:
        prim_path: USD prim path for the material.
        name: Name identifier.
        shader: Existing shader to use.
        texture_path: Path to diffuse texture.
        texture_scale: Texture UV scale (x, y).
        texture_translate: Texture UV translation (x, y).
        color: Diffuse color RGB.

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "omni_pbr",
        shader: UsdShade.Shader | None = None,
        texture_path: str | None = None,
        texture_scale: np.ndarray | None = None,
        texture_translate: np.ndarray | None = None,
        color: np.ndarray | None = None,
    ) -> None:
        stage = get_current_stage()
        if is_prim_path_valid(prim_path=prim_path):
            carb.log_info(f"Material Prim already defined at path: {prim_path}")
            material = UsdShade.Material(get_prim_at_path(prim_path))
        else:
            material = UsdShade.Material.Define(stage, prim_path)

        _new_shader = False
        if shader is None:
            if is_prim_path_valid(prim_path=f"{prim_path}/shader"):
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(get_prim_at_path(f"{prim_path}/shader"))
            elif is_prim_path_valid(f"{prim_path}/Shader"):
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/Shader"))
                shader = UsdShade.Shader(get_prim_at_path(f"{prim_path}/Shader"))
            else:
                shader = UsdShade.Shader.Define(stage, f"{prim_path}/shader")
                _new_shader = True
        VisualMaterial.__init__(
            self,
            prim_path=prim_path,
            prim=get_prim_at_path(prim_path),
            shaders_list=[shader],
            material=material,
            name=name,
        )
        if _new_shader:
            shader_out = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
            shader.CreateIdAttr("OmniPBR")
            shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f)
            shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float)
            shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float)
            shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset)
            shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool)
            shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2)
            shader.CreateInput("texture_translate", Sdf.ValueTypeNames.Float2)
            material.CreateSurfaceOutput("mdl").ConnectToSource(shader_out)
            material.CreateVolumeOutput("mdl").ConnectToSource(shader_out)
            material.CreateDisplacementOutput("mdl").ConnectToSource(shader_out)
            shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
            shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
            shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        if color is not None:
            self.set_color(color)
        if texture_path is not None:
            self.set_texture(texture_path)
        if texture_scale is not None:
            self.set_texture_scale(texture_scale[0], texture_scale[1])
        if texture_translate is not None:
            self.set_texture_translate(texture_translate[0], texture_translate[1])
        if _new_shader:
            self.set_project_uvw(True)
            self.set_reflection_roughness(0.5)
        return

    def set_color(self, color: np.ndarray) -> None:
        """Set the diffuse color.

        Args:
            color: RGB color array.

        """
        if self.shaders_list[0].GetInput("diffuse_color_constant").Get() is None:
            self.shaders_list[0].CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*color.tolist())
            )
        else:
            self.shaders_list[0].GetInput("diffuse_color_constant").Set(Gf.Vec3f(*color.tolist()))
        return

    def get_color(self) -> np.ndarray:
        """Get the diffuse color.

        Returns:
            RGB color array.

        """
        if self.shaders_list[0].GetInput("diffuse_color_constant").Get() is None:
            carb.log_warn("A color attribute is not set yet")
            return None
        else:
            return np.array(self.shaders_list[0].GetInput("diffuse_color_constant").Get())

    def set_texture(self, path: str) -> None:
        """Set the diffuse texture path.

        Args:
            path: Path to the texture file.

        """
        if self.shaders_list[0].GetInput("diffuse_texture").Get() is None:
            self.shaders_list[0].CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(path)
        else:
            self.shaders_list[0].GetInput("diffuse_texture").Set(path)
        return

    def get_texture(self) -> str:
        """Get the diffuse texture path.

        Returns:
            Path to the texture file.

        """
        if self.shaders_list[0].GetInput("diffuse_texture").Get() is None:
            carb.log_warn("A diffuse_texture attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("diffuse_texture").Get()

    def set_texture_scale(self, x: float, y: float) -> None:
        """Set the texture UV scale.

        Args:
            x: Scale in U direction.
            y: Scale in V direction.

        """
        if self.shaders_list[0].GetInput("texture_scale").Get() is None:
            self.shaders_list[0].CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f([x, y]))
        else:
            self.shaders_list[0].GetInput("texture_scale").Set(Gf.Vec2f([x, y]))
        return

    def set_texture_translate(self, x: float, y: float) -> None:
        """Set the texture UV translation.

        Args:
            x: Translation in U direction.
            y: Translation in V direction.

        """
        if self.shaders_list[0].GetInput("texture_translate").Get() is None:
            self.shaders_list[0].CreateInput("texture_translate", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f([x, y]))
        else:
            self.shaders_list[0].GetInput("texture_translate").Set(Gf.Vec2f([x, y]))
        return

    def get_texture_scale(self) -> np.ndarray:
        """Get the texture UV scale.

        Returns:
            Array with (x, y) scale values.

        """
        if self.shaders_list[0].GetInput("texture_scale").Get() is None:
            carb.log_warn("A texture_scale attribute is not set yet")
            return None
        else:
            return np.array(self.shaders_list[0].GetInput("texture_scale").Get())

    def get_texture_translate(self) -> np.ndarray:
        """Get the texture UV translation.

        Returns:
            Array with (x, y) translation values.

        """
        if self.shaders_list[0].GetInput("texture_translate").Get() is None:
            carb.log_warn("A texture_translate attribute is not set yet")
            return None
        else:
            return np.array(self.shaders_list[0].GetInput("texture_translate").Get())

    def set_project_uvw(self, flag: bool) -> None:
        """Enable or disable UVW projection.

        Args:
            flag: True to enable projection, False to disable.

        """
        if self.shaders_list[0].GetInput("project_uvw").Get() is None:
            self.shaders_list[0].CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(flag)
        else:
            self.shaders_list[0].GetInput("project_uvw").Set(flag)
        return

    def get_project_uvw(self) -> bool:
        """Get the UVW projection state.

        Returns:
            True if projection is enabled, False otherwise.

        """
        if self.shaders_list[0].GetInput("project_uvw").Get() is None:
            carb.log_warn("A project_uvw attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("project_uvw").Get()

    def set_reflection_roughness(self, amount: float) -> None:
        """Set the reflection roughness.

        Args:
            amount: Roughness value (0-1).

        """
        if self.shaders_list[0].GetInput("reflection_roughness_constant").Get() is None:
            self.shaders_list[0].CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(amount)
        else:
            self.shaders_list[0].GetInput("reflection_roughness_constant").Set(amount)
        return

    def get_reflection_roughness(self) -> float:
        """Get the reflection roughness.

        Returns:
            Roughness value.

        """
        if self.shaders_list[0].GetInput("reflection_roughness_constant").Get() is None:
            carb.log_warn("A reflection_roughness_constant attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("reflection_roughness_constant").Get()

    def set_metallic_constant(self, amount: float) -> None:
        """Set the metallic constant.

        Args:
            amount: Metallic value (0-1).

        """
        if self.shaders_list[0].GetInput("metallic_constant").Get() is None:
            self.shaders_list[0].CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(amount)
        else:
            self.shaders_list[0].GetInput("metallic_constant").Set(amount)
        return

    def get_metallic_constant(self) -> float:
        """Get the metallic constant.

        Returns:
            Metallic value.

        """
        if self.shaders_list[0].GetInput("metallic_constant").Get() is None:
            carb.log_warn("A metallic_constant attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("metallic_constant").Get()
