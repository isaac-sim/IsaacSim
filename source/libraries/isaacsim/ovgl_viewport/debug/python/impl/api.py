# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""High-level Python interface for the OVGL debug viewport."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from types import TracebackType
from typing import TYPE_CHECKING

import numpy as np
from isaacsim.ovgl_viewport.debug.bindings._bindings import Camera as _NativeCamera
from isaacsim.ovgl_viewport.debug.bindings._bindings import CameraPose, Frame
from isaacsim.ovgl_viewport.debug.bindings._bindings import Viewport as _NativeViewport
from isaacsim.ovgl_viewport.debug.bindings._bindings import ViewportConfig as _NativeViewportConfig
from pxr import Gf, Sdf, Usd, UsdGeom, UsdUtils

if TYPE_CHECKING:
    from isaacsim.foundation.objects import Stage, Xform

DEFAULT_CAMERA_PATH = "/ViewportCamera"
DEFAULT_RENDER_PRODUCT_PATH = "/Render/ViewportProduct"
DEFAULT_RENDER_VAR_PATH = "/Render/Vars/LdrColor"


@dataclass(slots=True)
class Camera:
    """Initial state for interactive viewport navigation.

    Args:
        target: Camera look-at target in stage coordinates.
        yaw_radians: Horizontal view angle in radians.
        pitch_radians: Vertical view angle in radians.
        distance: Distance from the camera to `target` in stage units.
    """

    target: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_radians: float = -0.65
    pitch_radians: float = 0.38
    distance: float = 6.0

    def _to_native(self) -> _NativeCamera:
        """Build the native camera configuration.

        Returns:
            Native camera configuration.
        """
        camera = _NativeCamera()
        camera.target = self.target
        camera.yaw_radians = self.yaw_radians
        camera.pitch_radians = self.pitch_radians
        camera.distance = self.distance
        return camera


def author_viewport(
    stage: Stage,
    *,
    camera_path: str = DEFAULT_CAMERA_PATH,
    render_product_path: str = DEFAULT_RENDER_PRODUCT_PATH,
    render_var_path: str = DEFAULT_RENDER_VAR_PATH,
    resolution: tuple[int, int] = (1280, 720),
) -> None:
    """Author the camera and render product required by an OVGL viewport.

    Args:
        stage: OpenUSD Foundation stage to author.
        camera_path: Absolute path for the viewport camera.
        render_product_path: Absolute path for the render product.
        render_var_path: Absolute path for the LDR color render variable.
        resolution: Render width and height in pixels.

    Raises:
        RuntimeError: If `stage` is invalid, is not an OpenUSD stage, or cannot be found in the USD stage cache.
        ValueError: If `resolution` is not positive.
    """
    if not stage.is_valid():
        raise RuntimeError("Cannot author a viewport on an invalid stage.")
    if stage.get_backend() != "openusd":
        raise RuntimeError("Viewport authoring requires an OpenUSD Foundation stage.")
    if resolution[0] <= 0 or resolution[1] <= 0:
        raise ValueError("Viewport resolution must contain positive pixel extents.")

    usd_stage = UsdUtils.StageCache.Get().Find(Usd.StageCache.Id.FromLongInt(stage.get_stage_id()))
    if not usd_stage:
        raise RuntimeError("Foundation did not retain the OpenUSD stage in the USD stage cache.")

    camera = UsdGeom.Camera.Define(usd_stage, camera_path)
    camera.CreateFocalLengthAttr(18.147562)
    camera.CreateClippingRangeAttr(Gf.Vec2f(1.0, 10_000_000.0))
    camera_xform = UsdGeom.Xformable(camera)
    camera_xform.ClearXformOpOrder()
    camera_xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 7.0))
    camera_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0))

    stage.define_prim(render_var_path, "RenderVar")
    render_var = usd_stage.GetPrimAtPath(render_var_path)
    render_var.CreateAttribute(
        "dataType", Sdf.ValueTypeNames.Token, custom=False, variability=Sdf.VariabilityUniform
    ).Set("color4f")
    render_var.CreateAttribute(
        "sourceName", Sdf.ValueTypeNames.String, custom=False, variability=Sdf.VariabilityUniform
    ).Set("LdrColor")

    stage.define_prim(render_product_path, "RenderProduct")
    render_product = usd_stage.GetPrimAtPath(render_product_path)
    render_product.GetRelationship("camera").SetTargets([camera_path])
    render_product.GetRelationship("orderedVars").SetTargets([render_var_path])
    render_product.CreateAttribute(
        "resolution", Sdf.ValueTypeNames.Int2, custom=False, variability=Sdf.VariabilityUniform
    ).Set(Gf.Vec2i(*resolution))


class Viewport:
    """Interactive OVGL viewport for a populated Foundation OVStage.

    Args:
        stage: Populated OVStage that remains alive while the viewport is open.
        render_product_path: Absolute path of the authored render product to display.
        camera_path: Absolute path of the camera controlled by viewport navigation.
        title: User-facing window title.
        width: Initial window width in pixels.
        height: Initial window height in pixels.
        maximum_frames: Frame limit, or zero to continue until closed.
        visible: Whether to create a user-visible window.
        camera: Initial and reset navigation state.
        camera_pose_writer: Optional replacement for the default Foundation camera-pose writer.

    Raises:
        RuntimeError: If `stage` is invalid or is not an OVStage.
    """

    __slots__ = ("_camera_pose_writer", "_camera_xform", "_native", "_stage")

    def __init__(
        self,
        stage: Stage,
        *,
        render_product_path: str = DEFAULT_RENDER_PRODUCT_PATH,
        camera_path: str = DEFAULT_CAMERA_PATH,
        title: str = "Isaac Sim OVGL Viewport",
        width: int = 1280,
        height: int = 720,
        maximum_frames: int = 0,
        visible: bool = True,
        camera: Camera | None = None,
        camera_pose_writer: Callable[[CameraPose], None] | None = None,
    ) -> None:
        self._native: _NativeViewport | None = None
        self._stage: Stage | None = None
        self._camera_xform: Xform | None = None
        self._camera_pose_writer: Callable[[CameraPose], None] | None = None

        if not stage.is_valid():
            raise RuntimeError("Cannot create a viewport for an invalid stage.")
        if stage.get_backend() != "ovstage":
            raise RuntimeError("Viewport rendering requires a Foundation OVStage.")

        if camera_pose_writer is None:
            from isaacsim.foundation.objects import Xform
            from isaacsim.foundation.utils.stage import use_stage

            with use_stage(stage):
                camera_xform = Xform(camera_path, resolve_paths=False, reset_xform_op_properties=False)

            def write_camera_pose(pose: CameraPose) -> None:
                camera_xform.set_world_poses(
                    positions=np.asarray([pose.position], dtype=np.float64),
                    orientations=np.asarray([pose.orientation], dtype=np.float64),
                )

            camera_pose_writer = write_camera_pose
            self._camera_xform = camera_xform

        native_config = _NativeViewportConfig()
        native_config.render_product_path = render_product_path
        native_config.title = title
        native_config.width = width
        native_config.height = height
        native_config.maximum_frames = maximum_frames
        native_config.visible = visible
        native_config.camera = (camera or Camera())._to_native()

        self._stage = stage
        self._camera_pose_writer = camera_pose_writer
        self._native = _NativeViewport(stage.get_stage_ptr(), camera_pose_writer, native_config)

    def __enter__(self) -> Viewport:
        """Return the open viewport.

        Returns:
            This viewport.
        """
        self._require_native()
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the viewport when leaving its context.

        Args:
            exception_type: Type of an exception raised in the context, if any.
            exception: Exception raised in the context, if any.
            traceback: Traceback for an exception raised in the context, if any.
        """
        self.close()

    @property
    def closed(self) -> bool:
        """Return whether the viewport has been closed."""
        return self._native is None

    def close(self) -> None:
        """Release the native viewport and its borrowed stage references."""
        if self._native is not None:
            self._native.close()
        self._native = None
        self._camera_pose_writer = None
        self._camera_xform = None
        self._stage = None

    def poll_events(self) -> bool:
        """Process input and return whether the viewport remains open.

        Returns:
            Whether the viewport remains open.
        """
        return self._require_native().poll_events()

    def render(self) -> Frame:
        """Render and optionally present the latest sealed OVStage ordinal.

        Visible viewports present directly on the GPU and return an empty RGBA payload. Invisible viewports return
        captured RGBA pixels.

        Returns:
            Borrowed frame data that remains valid until the next render or viewport closure.
        """
        return self._require_native().render()

    def _require_native(self) -> _NativeViewport:
        """Return the native viewport or fail after closure.

        Returns:
            Open native viewport.

        Raises:
            RuntimeError: If this viewport has been closed.
        """
        if self._native is None:
            raise RuntimeError("Cannot use a closed OVGL viewport.")
        return self._native


__all__ = [
    "Camera",
    "CameraPose",
    "DEFAULT_CAMERA_PATH",
    "DEFAULT_RENDER_PRODUCT_PATH",
    "DEFAULT_RENDER_VAR_PATH",
    "Frame",
    "Viewport",
    "author_viewport",
]
