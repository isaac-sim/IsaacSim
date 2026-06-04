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

"""Deprecated render product utility functions."""

from isaacsim.core.utils.prims import set_prim_hide_in_stage_window, set_prim_no_delete
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, Usd, UsdRender


def add_aov(render_product_path: str, aov_name: str) -> None:
    """Adds an AOV/Render Var to an existing render product.

    Args:
        render_product_path: Path to the render product prim.
        aov_name: Name of the render var we want to add to this render product.

    Raises:
        RuntimeError: If the render product path is invalid.
        RuntimeError: If the renderVar could not be created.
        RuntimeError: If the renderVar could not be added to the render product.
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        render_var_prim_path = Sdf.Path(f"/Render/Vars/{aov_name}")
        render_var_prim = stage.GetPrimAtPath(render_var_prim_path)
        if not render_var_prim:
            render_var_prim = stage.DefinePrim(render_var_prim_path)
        if not render_var_prim:
            raise RuntimeError(f'Cannot create renderVar "{render_var_prim_path}"')
        render_var_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(aov_name)
        render_prod_var_rel = render_prod_prim.GetOrderedVarsRel()
        if not render_prod_var_rel:
            render_prod_prim.CreateOrderedVarsRel()
        if not render_prod_var_rel:
            raise RuntimeError(f'cannot set orderedVars relationship for renderProduct "{render_product_path}"')
        render_prod_var_rel.AddTarget(render_var_prim_path)

        set_prim_hide_in_stage_window(render_var_prim, True)
        set_prim_no_delete(render_var_prim, True)


def get_camera_prim_path(render_product_path: str) -> str:
    """Get the current camera for a render product.

    Args:
        render_product_path: Path to the render product prim.

    Returns:
        Path to the camera prim attached to this render product.

    Raises:
        RuntimeError: If the render product path is invalid.
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        return render_prod_prim.GetCameraRel().GetTargets()[0]


def set_camera_prim_path(render_product_path: str, camera_prim_path: str) -> None:
    """Sets the camera prim path for a render product.

    Also applies the OmniRtxCameraExposureAPI_1 schema to the camera prim and sets the exposure:time attribute to 0.02.

    Args:
        render_product_path: Path to the render product prim.
        camera_prim_path: Path to the camera prim.

    Raises:
        RuntimeError: If the render product path is invalid.
        RuntimeError: If the camera prim path is invalid.
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        camera_prim = stage.GetPrimAtPath(camera_prim_path)
        if not camera_prim.IsValid():
            raise RuntimeError(f'Invalid camera prim "{camera_prim_path}"')
        render_prod_prim.GetCameraRel().SetTargets([camera_prim_path])
    camera_prim.AddAppliedSchema("OmniRtxCameraExposureAPI_1")
    camera_prim.CreateAttribute("exposure:time", Sdf.ValueTypeNames.Float).Set(0.02)


def get_resolution(render_product_path: str) -> tuple[int]:
    """Get resolution for a render product.

    Args:
        render_product_path: Path to the render product prim.

    Returns:
        A tuple containing (width, height).

    Raises:
        RuntimeError: If the render product path is invalid.
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        return render_prod_prim.GetResolutionAttr().Get()


def set_resolution(render_product_path: str, resolution: tuple[int]) -> None:
    """Set resolution for a render product.

    Args:
        render_product_path: Path to the render product prim.
        resolution: Width and height for render product.

    Raises:
        RuntimeError: If the render product path is invalid.
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        render_prod_prim.GetResolutionAttr().Set(Gf.Vec2i(resolution[0], resolution[1]))
