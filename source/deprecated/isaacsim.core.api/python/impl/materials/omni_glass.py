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

"""High level wrapper for creating/encapsulating Omniverse Glass (``OmniGlass``) material prims."""

from __future__ import annotations

import carb
import isaacsim.core.utils.stage as stage_utils
import numpy as np
from isaacsim.core.api.materials.visual_material import VisualMaterial
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid, move_prim
from pxr import Gf, Sdf, UsdShade


class OmniGlass(VisualMaterial):
    """High level wrapper for creating/encapsulating Omniverse Glass (``OmniGlass``) material prims.

    Args:
        prim_path: USD prim path for the material.
        name: Name identifier.
        shader: Existing shader to use.
        color: Glass tint color RGB.
        ior: Index of refraction.
        depth: Glass depth/thickness.
        thin_walled: Whether to use thin-walled mode.

    Raises:
        RuntimeError: If omni.kit.material.library extension is not enabled.
        Exception: If the shader is not defined.
        ValueError: If the material's shader is not of type OmniGlass.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "omni_glass",
        shader: UsdShade.Shader | None = None,
        color: np.ndarray | None = None,
        ior: float | None = None,
        depth: float | None = None,
        thin_walled: bool | None = None,
    ) -> None:
        stage = stage_utils.get_current_stage()
        if is_prim_path_valid(prim_path=prim_path):
            material = UsdShade.Material(get_prim_at_path(prim_path))
        else:
            try:
                from omni.kit.material.library import CreateAndBindMdlMaterialFromLibrary
            except Exception as e:
                carb.log_error(e)
                carb.log_error("Enable the omni.kit.material.library extension before using OmniGlass")
                raise RuntimeError(
                    "omni.kit.material.library extension is not enabled. Enable it before using OmniGlass."
                ) from e

            mtl_created_list = []
            CreateAndBindMdlMaterialFromLibrary(
                mdl_name="OmniGlass.mdl", mtl_name="OmniGlass", mtl_created_list=mtl_created_list
            ).do()
            move_prim(path_from=mtl_created_list[0], path_to=prim_path)
            material = UsdShade.Material(get_prim_at_path(prim_path))
        # omni.usd.create_material_input just calls the USD shader CreateInput(...) and adds a min / max rang,
        # display name, etc...  We don't need that here, so we can just call the USD shader api directly
        if shader is None:
            if stage.GetPrimAtPath(f"{prim_path}/shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/shader"))
            elif stage.GetPrimAtPath(f"{prim_path}/Shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/Shader"))
            else:
                raise Exception("omni glass shader is not defined")
        VisualMaterial.__init__(
            self,
            prim_path=prim_path,
            prim=stage.GetPrimAtPath(prim_path),
            shaders_list=[shader],
            material=material,
            name=name,
        )
        shader_id = shader.GetIdAttr().Get()
        if shader_id and shader_id != "OmniGlass":
            raise ValueError(
                f"The material's shader at path {prim_path} (with id {shader_id}) is not of type OmniGlass"
            )
        shader.CreateIdAttr("OmniGlass")
        if color is not None:
            shader.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color.tolist()))
        if ior is not None:
            shader.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(ior)
        if depth is not None:
            shader.CreateInput("depth", Sdf.ValueTypeNames.Float).Set(depth)
        if thin_walled is not None:
            shader.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(thin_walled)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        return

    def set_color(self, color: np.ndarray) -> None:
        """Set the glass tint color.

        Args:
            color: RGB color array.

        """
        if self.shaders_list[0].GetInput("glass_color").Get() is None:
            self.shaders_list[0].CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color.tolist()))
        else:
            self.shaders_list[0].GetInput("glass_color").Set(Gf.Vec3f(*color.tolist()))
        return

    def get_color(self) -> np.ndarray | None:
        """Get the glass tint color.

        Returns:
            RGB color array or None if not set.

        """
        if self.shaders_list[0].GetInput("glass_color").Get() is None:
            carb.log_warn("A color attribute is not set yet")
            return None
        else:
            return np.array(self.shaders_list[0].GetInput("glass_color").Get())

    def set_ior(self, ior: float) -> None:
        """Set the index of refraction for the glass material.

        Args:
            ior: Index of refraction value.

        """
        if self.shaders_list[0].GetInput("glass_ior").Get() is None:
            self.shaders_list[0].CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(ior)
        else:
            self.shaders_list[0].GetInput("glass_ior").Set(ior)
        return

    def get_ior(self) -> float | None:
        """Index of refraction for the glass material.

        Returns:
            Index of refraction value or None if not set.

        """
        if self.shaders_list[0].GetInput("glass_ior").Get() is None:
            carb.log_warn("A glass_ior attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("glass_ior").Get()

    def set_depth(self, depth: float) -> None:
        """Set the glass depth/thickness.

        Args:
            depth: Glass depth/thickness value.

        """
        if self.shaders_list[0].GetInput("depth").Get() is None:
            self.shaders_list[0].CreateInput("depth", Sdf.ValueTypeNames.Float).Set(depth)
        else:
            self.shaders_list[0].GetInput("depth").Set(depth)
        return

    def get_depth(self) -> float | None:
        """Glass depth/thickness.

        Returns:
            Glass depth/thickness value or None if not set.

        """
        if self.shaders_list[0].GetInput("depth").Get() is None:
            carb.log_warn("A depth attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("depth").Get()

    def set_thin_walled(self, thin_walled: float) -> None:
        """Set the thin-walled mode for the glass material.

        Args:
            thin_walled: Thin-walled mode value.

        """
        if self.shaders_list[0].GetInput("thin_walled").Get() is None:
            self.shaders_list[0].CreateInput("thin_walled", Sdf.ValueTypeNames.Float).Set(thin_walled)
        else:
            self.shaders_list[0].GetInput("thin_walled").Set(thin_walled)
        return

    def get_thin_walled(self) -> float | None:
        """Thin-walled mode for the glass material.

        Returns:
            Thin-walled mode value or None if not set.

        """
        if self.shaders_list[0].GetInput("thin_walled").Get() is None:
            carb.log_warn("A thin_walled attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("thin_walled").Get()
