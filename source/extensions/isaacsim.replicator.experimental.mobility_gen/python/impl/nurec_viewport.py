# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Render the robot chase view through an authored PPISP graph (SPG teleop).

An SPG/PPISP NuRec export carries a PPISP graph per authored camera on its `/Render/<cam>`
RenderProduct (`HdrColor -> controller -> CRF -> vignetting -> LdrColor`). That graph is image-space:
it processes the rendered image, so it is independent of the camera's intrinsics/focal length, and
the output resolution comes from the RenderProduct. `route_chase_through_ppisp` therefore points the
RenderProduct's camera straight at the robot chase camera (keeping its own FOV), forces identity
exposure on it (so the graph is the sole exposure authority), applies the stage's baked render
settings, and binds the active viewport.
"""

from __future__ import annotations

from typing import Any

# Camera exposure values + the API schemas that make them authoritative, so the PPISP graph is the
# sole exposure authority (the HdrColor it consumes must not be pre-exposed / auto-exposed).
_IDENTITY_EXPOSURE = {
    "exposure": 0.0,
    "exposure:fStop": 1.0,
    "exposure:iso": 0.0,
    "exposure:responsivity": 1.0,
    "exposure:time": 1.0,
}
_EXPOSURE_API_SCHEMAS = ("OmniRtxCameraAutoExposureAPI_1", "OmniRtxCameraExposureAPI_1")


def route_chase_through_ppisp(stage: Any, chase_camera_path: str) -> str | None:
    """Render the chase camera through the export's first authored PPISP RenderProduct.

    Points the RenderProduct's camera at `chase_camera_path` (keeping the chase camera's own
    intrinsics), forces identity exposure on it, applies the stage's baked render settings, and binds
    the active viewport to the RenderProduct.

    Args:
        stage: The open USD stage.
        chase_camera_path: The robot chase camera to render through the PPISP graph.

    Returns:
        The bound RenderProduct prim path, or None if the stage has no authored RenderProduct (or no
        active viewport).
    """
    import carb
    import omni.kit.viewport.utility as vp_util

    carb.log_warn(f"[nurec] route_chase_through_ppisp: chase_camera={chase_camera_path}")

    # 1) Enumerate the authored RenderProducts (each carries a PPISP graph) and pick the first.
    render_products = _render_products(stage)
    if not render_products:
        carb.log_warn("[nurec] step 1/4: no authored RenderProduct under /Render; cannot route chase through PPISP")
        return None
    carb.log_warn(f"[nurec] step 1/4: authored RenderProducts under /Render: {render_products}")
    rp_path, authored_cam_path = next(iter(render_products.items()))
    carb.log_warn(
        f"[nurec] step 2/4: using RP={rp_path} (authored camera {authored_cam_path}) -> chase {chase_camera_path}"
    )

    # 2) Identity exposure on the chase camera (the graph is the exposure authority); apply the
    #    stage's baked render settings (e.g. rtx:post:tonemap:op) that File->Open would have applied.
    _force_identity_exposure(stage, chase_camera_path)
    _log_camera(stage.GetPrimAtPath(chase_camera_path), label="step 2/4: chase camera (PPISP input)")
    _apply_stage_render_settings(stage)

    # 3) Point the RenderProduct's PPISP graph at the chase camera.
    stage.GetPrimAtPath(rp_path).GetRelationship("camera").SetTargets([chase_camera_path])
    carb.log_warn(f"[nurec] step 3/4: repointed {rp_path}.camera -> {chase_camera_path}")

    # 4) Bind the active viewport to the RenderProduct.
    viewport = vp_util.get_active_viewport()
    if viewport is None:
        carb.log_warn("[nurec] step 4/4: no active viewport; PPISP graph not bound")
        return None
    viewport.render_product_path = rp_path
    carb.log_warn(
        f"[nurec] step 4/4: PPISP chase view bound: active viewport -> {rp_path} -> camera {chase_camera_path}"
    )
    return rp_path


def _force_identity_exposure(stage: Any, camera_path: str) -> None:
    """Apply identity exposure (and disable auto-exposure) on `camera_path`.

    Applies the exposure API schemas so the values are honored by the renderer (matching the export's
    `_no_isp` PPISP camera).

    Args:
        stage: The USD stage containing the camera.
        camera_path: Path to the camera prim to modify.
    """
    from pxr import Sdf

    prim = stage.GetPrimAtPath(camera_path)
    if not prim.IsValid():
        return
    for schema in _EXPOSURE_API_SCHEMAS:
        prim.AddAppliedSchema(schema)
    for name, value in _IDENTITY_EXPOSURE.items():
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Float).Set(value)
    prim.CreateAttribute("omni:rtx:autoExposure:enabled", Sdf.ValueTypeNames.Bool).Set(False)


def _apply_stage_render_settings(stage: Any) -> None:
    """Apply the stage's baked `customLayerData['renderSettings']` as carb settings.

    The Kit UI applies these on File->Open; a programmatic open does not, so apply them here (e.g.
    `rtx:post:tonemap:op`). Keys are colon-namespaced (`rtx:post:tonemap:op`) -> carb path
    (`/rtx/post/tonemap/op`).

    Args:
        stage: The USD stage whose baked render settings are applied.
    """
    import carb
    import carb.settings

    layer_data = stage.GetRootLayer().customLayerData or {}
    render_settings = layer_data.get("renderSettings") or {}
    if not render_settings:
        carb.log_warn("[nurec] no customLayerData renderSettings on the stage")
        return
    settings = carb.settings.get_settings()
    for key, value in render_settings.items():
        path = "/" + str(key).replace(":", "/")
        settings.set(path, value)
        carb.log_warn(f"[nurec] applied stage renderSetting: {path} = {value}")


def _log_camera(prim: Any, *, label: str) -> None:
    """Log a camera's path, intrinsics, exposure, and applied schemas (for tracing correctness).

    Args:
        prim: The camera prim to log.
        label: Label prefix identifying the log line.
    """
    import carb
    from pxr import Usd, UsdGeom

    gf = UsdGeom.Camera(prim).GetCamera(Usd.TimeCode.Default())
    exposure = {n: prim.GetAttribute(n).Get() for n in _IDENTITY_EXPOSURE if prim.GetAttribute(n).HasAuthoredValue()}
    auto_exposure = prim.GetAttribute("omni:rtx:autoExposure:enabled").Get()
    carb.log_warn(
        f"[nurec] {label} {prim.GetPath()}: focalLength={gf.focalLength} "
        f"hAperture={gf.horizontalAperture} vAperture={gf.verticalAperture} clip={gf.clippingRange} "
        f"exposure={exposure} autoExposure:enabled={auto_exposure} schemas={list(prim.GetAppliedSchemas())}"
    )


def _render_products(stage: Any) -> dict[str, str]:
    """Return `{render_product_path: camera_target_path}` for authored RenderProducts under `/Render`.

    The camera target is `""` for a RenderProduct with no `camera` relationship.

    Args:
        stage: The USD stage to scan for RenderProducts.

    Returns:
        Mapping of each RenderProduct path to its camera target path.
    """
    out: dict[str, str] = {}
    render_scope = stage.GetPrimAtPath("/Render")
    if not render_scope.IsValid():
        return out
    for child in render_scope.GetChildren():
        if child.GetTypeName() == "RenderProduct":
            targets = child.GetRelationship("camera").GetTargets()
            out[str(child.GetPath())] = str(targets[0]) if targets else ""
    return out
