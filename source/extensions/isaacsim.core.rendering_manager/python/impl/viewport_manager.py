# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Core class that provides APIs for managing viewports."""

from __future__ import annotations

import asyncio
import re
import time

import carb
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.kit.viewport.utility
import omni.kit.viewport.window
import warp as wp
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender

from .rendering_manager import RenderingManager


class ViewportManager:
    """Core class that provides APIs for managing viewports."""

    @classmethod
    def wait_for_viewport(
        cls, *, viewport: str | "ViewportAPI" | None = None, max_frames: int = 60, sleep_time: float = 0.02
    ) -> tuple[bool, int]:
        """Wait for the viewport to be ready.

        Calling this method ensures that a new frame is rendered by the viewport during an app update (or rendering step).

        Args:
            viewport: The viewport to wait for. If not provided, the active viewport is used.
                See :py:meth:`get_viewport_api` for supported viewport sources.
            max_frames: The maximum number of frames to wait for.
            sleep_time: Time, in seconds, to sleep between frames.
                Setting a positive value reduces the number of frames required for the viewport to be ready.

        Returns:
            A tuple containing a boolean indicating whether the viewport is ready and the number of frames waited for.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> ViewportManager.wait_for_viewport()
            (True, 0)
        """
        viewport = cls.get_viewport_api(viewport)
        if not viewport:
            return False, 0
        is_viewport_ready = lambda: viewport.frame_info.get("viewport_handle") is not None
        for i in range(max_frames):
            if is_viewport_ready():
                return True, i
            RenderingManager.render()
            if sleep_time > 0:
                time.sleep(sleep_time)
        return is_viewport_ready(), max_frames

    @classmethod
    async def wait_for_viewport_async(
        cls, *, viewport: str | "ViewportAPI" | None = None, max_frames: int = 60, sleep_time: float = 0.02
    ) -> tuple[bool, int]:
        """Wait for the viewport to be ready.

        This method is the asynchronous version of :py:meth:`wait_for_viewport`.

        Args:
            viewport: The viewport to wait for. If not provided, the active viewport is used.
                See :py:meth:`get_viewport_api` for supported viewport sources.
            max_frames: The maximum number of frames to wait for.
            sleep_time: Time, in seconds, to sleep between frames.
                Setting a positive value reduces the number of frames required for the viewport to be ready.

        Returns:
            A tuple containing a boolean indicating whether the viewport is ready and the number of frames waited for.
        """
        viewport = cls.get_viewport_api(viewport)
        if not viewport:
            return False, 0
        is_viewport_ready = lambda: viewport.frame_info.get("viewport_handle") is not None
        for i in range(max_frames):
            if is_viewport_ready():
                return True, i
            await RenderingManager.render_async()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
        return is_viewport_ready(), max_frames

    @classmethod
    def set_camera(
        cls,
        camera: str | Usd.Prim | UsdGeom.Camera,
        *,
        render_product_or_viewport: str | Usd.Prim | UsdRender.Product | "ViewportAPI" | None = None,
    ) -> None:
        """Set a render product or viewport's camera.

        .. list-table:: Available Omniverse Kit cameras:
            :header-rows: 1

            * - Camera view
              - USD path (on session layer)
            * - Perspective view
              - ``/OmniverseKit_Persp``
            * - Top view
              - ``/OmniverseKit_Top``
            * - Front view
              - ``/OmniverseKit_Front``
            * - Right view
              - ``/OmniverseKit_Right``

        Args:
            camera: The camera to set.
            render_product_or_viewport: The render product or viewport to set the camera for.
                If not provided, the active viewport is used.
                See :py:meth:`get_viewport_api` or :py:meth:`get_render_product` for supported sources.

        Raises:
            ValueError: Invalid camera.
            ValueError: Invalid render product or viewport.

        Example:

        .. code-block:: python

            >>> import isaacsim.core.experimental.utils.stage as stage_utils
            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # set the active viewport's camera to the top view
            >>> ViewportManager.set_camera("/OmniverseKit_Top")
            >>>
            >>> # set the active viewport's camera to a custom camera
            >>> camera = stage_utils.define_prim("/Camera", "Camera")
            >>> ViewportManager.set_camera(camera)
        """
        prim = prim_utils.get_prim_at_path(camera)
        if not prim.IsValid() or not prim.IsA(UsdGeom.Camera):
            raise ValueError(f"The camera ({camera}) is not a valid USD Camera prim")
        path = prim_utils.get_prim_path(prim)
        viewport = cls.get_viewport_api(render_product_or_viewport)
        if viewport is not None:
            viewport.camera_path = path
            return
        render_product = cls.get_render_product(render_product_or_viewport)
        if render_product is not None:
            stage = stage_utils.get_current_stage(backend="usd")
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                render_product.GetCameraRel().SetTargets([path])
            return
        raise ValueError(
            f"Unable to set camera: unknown render product or viewport '{render_product_or_viewport}' ({type(render_product_or_viewport)})"
        )

    @classmethod
    def get_camera(
        cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | "ViewportAPI" | None = None
    ) -> UsdGeom.Camera:
        """Get a render product or viewport's camera.

        Args:
            render_product_or_viewport: The render product or viewport to get the camera from.
                If not provided, the active viewport is used.

        Returns:
            USD Camera prim.

        Raises:
            ValueError: Invalid render product or viewport.
            ValueError: No camera prim target found for a given render product.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # get the camera of the active viewport
            >>> ViewportManager.get_camera()
            UsdGeom.Camera(Usd.Prim(</OmniverseKit_Persp>))
        """
        viewport = cls.get_viewport_api(render_product_or_viewport)
        if viewport is not None:
            return UsdGeom.Camera(prim_utils.get_prim_at_path(viewport.camera_path))
        render_product = cls.get_render_product(render_product_or_viewport)
        if render_product is not None:
            target = render_product.GetCameraRel().GetTargets()
            if not len(target):
                raise ValueError(f"Unable to get camera: no camera targets found for render product {render_product}")
            return UsdGeom.Camera(prim_utils.get_prim_at_path(target[0]))
        raise ValueError(
            f"Unable to get camera: unknown render product or viewport '{render_product_or_viewport}' ({type(render_product_or_viewport)})"
        )

    @classmethod
    def get_viewport_api(
        cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | "ViewportAPI" | None = None
    ) -> "ViewportAPI" | None:
        """Get a viewport API (also identified as viewport) instance.

        Supported sources are:

        .. list-table::
            :header-rows: 1

            * - Source
              - Return value
            * - Unspecified (default)
              - The active viewport window's viewport API instance
            * - ``ViewportAPI`` instance
              - The given viewport API instance
            * - Viewport window title (name)
              - The given viewport window's viewport API instance

        Args:
            render_product_or_viewport: The render product or viewport to get the viewport API from.
                If not provided, the active viewport window is used.

        Returns:
            Viewport API instance (as a proxy), or None if no viewport API is found.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # get viewport API from the active viewport window
            >>> ViewportManager.get_viewport_api()
            <weakproxy at 0x... to ViewportAPI at 0x...>
            >>>
            >>> # get viewport API from a viewport window
            >>> ViewportManager.get_viewport_api("Viewport")
            <weakproxy at 0x... to ViewportAPI at 0x...>
        """
        # unspecified source
        if render_product_or_viewport is None:
            return omni.kit.viewport.utility.get_active_viewport()
        # viewport API
        elif "ViewportAPI" in render_product_or_viewport.__class__.__name__:
            return render_product_or_viewport
        # USD prim
        elif isinstance(render_product_or_viewport, Usd.Prim):
            return cls.get_viewport_api(render_product_or_viewport.GetPath().pathString)
        # Usd RenderProduct prim
        elif isinstance(render_product_or_viewport, UsdRender.Product):
            # TODO: get the viewport from the render product prim
            return None
        # str
        elif isinstance(render_product_or_viewport, (str, Sdf.Path)):
            render_product_or_viewport = (
                render_product_or_viewport.pathString
                if isinstance(render_product_or_viewport, Sdf.Path)
                else render_product_or_viewport
            )
            prim = prim_utils.get_prim_at_path(render_product_or_viewport)
            # valid USD prim
            if prim.IsValid():
                # render product
                if prim.IsA(UsdRender.Product):
                    render_product = UsdRender.Product(prim)
                    # TODO: get the viewport from the render product prim
                    return None
            # viewport name
            viewport_window = omni.kit.viewport.utility.get_active_viewport_window(render_product_or_viewport)
            if viewport_window is not None:
                return viewport_window.viewport_api
        return None

    @classmethod
    def get_render_product(
        cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | "ViewportAPI" | None = None
    ) -> UsdRender.Product | None:
        """Get an USD RenderProduct prim that describes an artifact produced by a render.

        Supported sources are:

        .. list-table::
            :header-rows: 1

            * - Source
              - Return value
            * - Unspecified (default)
              - The active viewport's render product
            * - ``ViewportAPI`` instance
              - The given viewport's render product
            * - ``UsdRender.Product`` prim instance
              - The given prim instance
            * - USD RenderProduct prim path
              - The ``UsdRender.Product`` prim at the given path
            * - Viewport window title (name)
              - The given viewport window's render product

        Args:
            render_product_or_viewport: The render product or viewport to get the USD RenderProduct prim from.
                If not provided, the active viewport is used.

        Returns:
            USD RenderProduct prim, or None if no render product is found.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # get render product from the active viewport
            >>> ViewportManager.get_render_product()
            UsdRender.Product(Usd.Prim(</Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0>))
            >>>
            >>> # get render product from a viewport API
            >>> viewport_api = ViewportManager.get_viewport_api()
            >>> ViewportManager.get_render_product(viewport_api)
            UsdRender.Product(Usd.Prim(</Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0>))
            >>>
            >>> # get render product from a USD RenderProduct prim path
            >>> path = "/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0"
            >>> ViewportManager.get_render_product(path)
            UsdRender.Product(Usd.Prim(</Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0>))
            >>>
            >>> # get render product from a viewport window
            >>> ViewportManager.get_render_product("Viewport")
            UsdRender.Product(Usd.Prim(</Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0>))
        """
        # unspecified source
        if render_product_or_viewport is None:
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport is not None:
                return cls.get_render_product(viewport.render_product_path)
        # viewport
        elif "ViewportAPI" in render_product_or_viewport.__class__.__name__:
            return cls.get_render_product(render_product_or_viewport.render_product_path)
        # USD prim
        elif isinstance(render_product_or_viewport, Usd.Prim):
            return cls.get_render_product(render_product_or_viewport.GetPath().pathString)
        # Usd RenderProduct prim
        elif isinstance(render_product_or_viewport, UsdRender.Product):
            return render_product_or_viewport
        # str
        elif isinstance(render_product_or_viewport, (str, Sdf.Path)):
            render_product_or_viewport = (
                render_product_or_viewport.pathString
                if isinstance(render_product_or_viewport, Sdf.Path)
                else render_product_or_viewport
            )
            prim = prim_utils.get_prim_at_path(render_product_or_viewport)
            # valid USD prim
            if prim.IsValid():
                # render product
                if prim.IsA(UsdRender.Product):
                    return UsdRender.Product(prim)
            # viewport name
            viewport_window = omni.kit.viewport.utility.get_active_viewport_window(render_product_or_viewport)
            if viewport_window is not None and viewport_window.viewport_api is not None:
                return cls.get_render_product(viewport_window.viewport_api.render_product_path)
        return None

    @classmethod
    def get_resolution(
        cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | "ViewportAPI" | None = None
    ) -> tuple[int, int]:
        """Get a render product or viewport's resolution: width x height.

        Args:
            render_product_or_viewport: The render product or viewport to get the resolution from.
                If not provided, the active viewport is used.
                See :py:meth:`get_viewport_api` or :py:meth:`get_render_product` for supported sources.

        Returns:
            Resolution as a tuple of width and height.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> ViewportManager.get_resolution()
            (1280, 720)
        """
        viewport = cls.get_viewport_api(render_product_or_viewport)
        if viewport is not None:
            return viewport.resolution
        render_product = cls.get_render_product(render_product_or_viewport)
        if render_product is not None:
            return tuple(render_product.GetResolutionAttr().Get())
        raise ValueError(
            f"Unable to get resolution: unknown render product or viewport '{render_product_or_viewport}' ({type(render_product_or_viewport)})"
        )

    @classmethod
    def set_resolution(
        cls,
        resolution: tuple[int, int] | str,
        *,
        render_product_or_viewport: str | Usd.Prim | UsdRender.Product | "ViewportAPI" | None = None,
    ) -> None:
        """Set a render product or viewport's resolution: width x height.

        Args:
            resolution: The resolution as a tuple of width and height.
            render_product_or_viewport: The render product or viewport to set the resolution for.
                If not provided, the active viewport is used.
                See :py:meth:`get_viewport_api` or :py:meth:`get_render_product` for supported sources.

        Raises:
            ValueError: Invalid render product or viewport.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # set the active viewport's resolution to (640, 480)
            >>> ViewportManager.set_resolution((640, 480))
            >>>
            >>> # check the resolution
            >>> ViewportManager.get_resolution()
            (640, 480)
            >>> ViewportManager.get_viewport_api().resolution
            (640, 480)
        """
        viewport = cls.get_viewport_api(render_product_or_viewport)
        if viewport is not None:
            viewport.resolution = resolution
            return
        render_product = cls.get_render_product(render_product_or_viewport)
        if render_product is not None:
            stage = stage_utils.get_current_stage(backend="usd")
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                render_product.GetResolutionAttr().Set(Gf.Vec2i(*resolution))
            return
        raise ValueError(
            f"Unable to set resolution: unknown render product or viewport '{render_product_or_viewport}' ({type(render_product_or_viewport)})"
        )

    @classmethod
    def create_viewport_window(
        cls,
        *,
        camera: str | Usd.Prim | UsdGeom.Camera = "/OmniverseKit_Persp",
        title: str | None = None,
        resolution: tuple[int, int] = (1280, 720),
    ) -> "ViewportWindow":
        """Create a viewport window.

        Args:
            camera: The camera to use for the viewport. If not provided, the default (perspective) camera is used.
            title: The viewport window title (name). If not provided, a default title is generated.
            resolution: The viewport window resolution: (width, height).

        Returns:
            The viewport window (as a proxy).

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>> import isaacsim.core.experimental.utils.stage as stage_utils
            >>>
            >>> # create a viewport window with the default camera and resolution
            >>> window = ViewportManager.create_viewport_window()
            >>> window.title
            'Viewport 1'
            >>> window.viewport_api.camera_path
            Sdf.Path('/OmniverseKit_Persp')
            >>>
            >>> # create a viewport window with a custom camera and resolution
            >>> camera = stage_utils.define_prim("/Camera", "Camera")
            >>> window = ViewportManager.create_viewport_window(
            ...     camera=camera,
            ...     resolution=(640, 480),
            ...     title="Custom Viewport",
            ... )
            >>> window.title
            'Custom Viewport'
            >>> window.viewport_api.camera_path
            Sdf.Path('/Camera')
            >>> window.viewport_api.resolution
            (640, 480)
        """
        if not title:
            title = f"Viewport {len(list(omni.kit.viewport.window.get_viewport_window_instances()))}"
        window = omni.kit.viewport.window.ViewportWindow(
            name=title, usd_context_name="", width=resolution[0], height=resolution[1]
        )
        cls.set_camera(camera, render_product_or_viewport=window.viewport_api)
        cls.set_resolution(resolution, render_product_or_viewport=window.viewport_api)
        return window

    @classmethod
    def get_viewport_windows(cls, *, include: list[str] | None = None, exclude: list[str] | None = None) -> list:
        """Get viewport windows.

        Args:
            include: Viewport window titles (names) to get. If not provided, all viewport windows will be included.
                Regular expressions are supported.
            exclude: Viewport window titles (names) to exclude from getting. Excluded names take precedence over
                included names. Regular expressions are supported.

        Returns:
            List of viewport windows (as proxies).

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # get all viewport windows
            >>> windows = ViewportManager.get_viewport_windows()
            >>> windows
            [<weakproxy at 0x... to ViewportWindow at 0x...>]
            >>> windows[0].title
            'Viewport'
            >>> windows[0].viewport_api.camera_path
            Sdf.Path('/OmniverseKit_Persp')
            >>> windows[0].viewport_api.resolution
            (1280, 720)
        """
        if include is None:
            include = [".*"]
        if exclude is None:
            exclude = []

        windows = []
        for window in omni.kit.viewport.window.get_viewport_window_instances():
            if window:
                title = window.title
                if any(re.fullmatch(pattern, title) for pattern in exclude):
                    continue
                if not any(re.fullmatch(pattern, title) for pattern in include):
                    continue
                windows.append(window)
        return windows

    @classmethod
    def destroy_viewport_windows(
        cls, *, include: list[str] | None = None, exclude: list[str] | None = None
    ) -> list[str]:
        """Destroy viewport windows.

        Args:
            include: Viewport window titles (names) to destroy. If not provided, all viewport windows will be destroyed.
                Regular expressions are supported.
            exclude: Viewport window titles (names) to exclude from destruction. Excluded names take precedence
                over included names. Regular expressions are supported.

        Returns:
            List of destroyed viewport window titles (names).

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # given the following viewport windows: "Viewport", "Viewport 1", and "Custom Viewport",
            >>> # destroy all viewport windows except the default one: "Viewport"
            >>> ViewportManager.destroy_viewport_windows(exclude=["Viewport"])
            ['Viewport 1', 'Custom Viewport']
        """
        if include is None:
            include = [".*"]
        if exclude is None:
            exclude = []

        destroyed_windows = []
        for window in cls.get_viewport_windows(include=include, exclude=exclude):
            destroyed_windows.append(window.title)
            window.destroy()
        return destroyed_windows

    @classmethod
    def set_camera_view(
        cls,
        camera: str | Usd.Prim | UsdGeom.Camera,
        *,
        eye: list | np.ndarray | wp.array | None = None,
        target: list | np.ndarray | wp.array | None = None,
        relative_tracking: bool = False,
    ) -> None:
        """Set the camera view.

        This method sets the camera view by adjusting its position and orientation, while taking into account
        the camera's center of interest (COI) attribute: ``omni:kit:centerOfInterest``, if it exists.

        Depending on the forwarded arguments and the existence of the *COI* attribute,
        the camera view is adjusted as follows:

        .. list-table::
            :header-rows: 1

            * - ``COI``
              - ``eye``
              - ``target``
              - Behavior
            * - no
              - no
              - no
              - The camera's view will not change. A warning will be logged.
            * - no
              - no
              - yes
              - The camera remains in place but rotates to face the *target*.
            * - no
              - yes
              - no
              - The camera teleports to the *eye* while keeping its orientation.
            * - no
              - yes
              - yes
              - The camera teleports to the *eye* and rotates to face the *target*.
            * - yes
              - no
              - no
              - The camera remains in place but rotates to face the *COI*.
            * - yes
              - no
              - yes
              - The ``relative_tracking`` flag determines the behavior:

                * If ``relative_tracking`` is ``False`` (default), the camera remains in place but rotates to face
                  the *target*. The *COI* is updated to the *target* value.
                * If ``relative_tracking`` is ``True``, the camera teleports to face the *target*, while keeping the
                  same orientation and distance relative to the *COI*. The *COI* is updated to the *target* value.
            * - yes
              - yes
              - no
              - The camera teleports to the *eye* and rotates to face the *COI*.
            * - yes
              - yes
              - yes
              - The camera teleports to the *eye* and rotates to face the *target*.
                The *COI* is updated to the *target* value.

        Args:
            camera: The camera path or instance.
            eye: The eye position (position of the camera in the world frame).
            target: The target position (position of the target to look at in the world frame).
            relative_tracking: Whether to track the target relative to the current camera position.

        Raises:
            ValueError: The camera is not a valid USD Camera prim.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import ViewportManager
            >>>
            >>> # set the active viewport's camera view to look at a target at (1.0, 2.0, 3.0)
            >>> camera = ViewportManager.get_camera()
            >>> ViewportManager.set_camera_view(camera, target=[1.0, 2.0, 3.0])  # doctest: +NO_CHECK
        """

        def _adjust_for_collinearity(
            xformable: UsdGeom.Xformable, position: Gf.Vec3d, epsilon: float = 1e-5
        ) -> Gf.Vec3d:
            world_transform = xformable.ComputeLocalToWorldTransform(time_code)
            world_transform.Orthonormalize()
            up_direction = Gf.Vec3d(0, 0, 1) if stage_utils.get_stage_up_axis() == "Z" else Gf.Vec3d(0, 1, 0)
            result = (world_transform.ExtractTranslation() - position).GetCross(up_direction).GetLength()
            return position + Gf.Vec3d(epsilon, 0, 0) if result < epsilon else position

        def _set_eye(xformable: UsdGeom.Xformable, position: Gf.Vec3d) -> None:
            parent_inverse_transform = xformable.ComputeParentToWorldTransform(time_code).GetInverse()
            local_transform = xformable.ComputeLocalToWorldTransform(time_code) * parent_inverse_transform
            new_local_transform = Gf.Matrix4d(local_transform).SetTranslateOnly(
                parent_inverse_transform.Transform(position)
            )
            omni.kit.commands.create(
                "TransformPrimCommand",
                path=prim.GetPath(),
                new_transform_matrix=new_local_transform,
                old_transform_matrix=local_transform,
                time_code=time_code,
            ).do()

        def _set_target(xformable: UsdGeom.Xformable, position: Gf.Vec3d) -> None:
            parent_inverse_transform = xformable.ComputeParentToWorldTransform(time_code).GetInverse()
            local_transform = xformable.ComputeLocalToWorldTransform(time_code) * parent_inverse_transform
            position_in_parent = parent_inverse_transform.Transform(local_transform.Transform(Gf.Vec3d(0, 0, 0)))
            # adjust for collinearity, if needed
            position = _adjust_for_collinearity(xformable, position)
            up_direction = Gf.Vec3d(0, 0, 1) if stage_utils.get_stage_up_axis() == "Z" else Gf.Vec3d(0, 1, 0)
            center_in_parent = parent_inverse_transform.Transform(position)
            new_local_transform = transform_utils.look_at_matrix(position_in_parent, center_in_parent, up_direction)
            omni.kit.commands.create(
                "TransformPrimCommand",
                path=prim.GetPath(),
                new_transform_matrix=new_local_transform,
                old_transform_matrix=local_transform,
                time_code=time_code,
            ).do()

        prim = prim_utils.get_prim_at_path(camera)
        if not prim.IsValid() or not prim.IsA(UsdGeom.Camera):
            raise ValueError(f"The camera ({camera}) is not a valid USD Camera prim")
        xformable = UsdGeom.Xformable(prim)
        time_code = Usd.TimeCode.Default()
        # get centerOfInterest attribute
        coi_attr = None
        if prim.HasAttribute("omni:kit:centerOfInterest"):
            coi_attr = prim.GetAttribute("omni:kit:centerOfInterest")
        # process input arguments
        if eye is not None:
            eye = ops_utils.place(eye, dtype=wp.float32, device="cpu").numpy().flatten().tolist()
            eye = Gf.Vec3d(eye[0], eye[1], eye[2])
        if target is not None:
            target = ops_utils.place(target, dtype=wp.float32, device="cpu").numpy().flatten().tolist()
            target = Gf.Vec3d(target[0], target[1], target[2])
        # determine transform strategy
        update_coi = False
        if coi_attr is None:
            if eye is None and target is None:
                carb.log_warn(
                    "Neither the 'eye' nor the 'target' was provided. "
                    "As the 'omni:kit:centerOfInterest' attribute was not authored, the camera view will not change."
                )
        else:
            if eye is None and target is None:
                target = coi_attr.Get(time_code)
            elif eye is None and target is not None:
                update_coi = True
                if relative_tracking:
                    world_transform = xformable.ComputeLocalToWorldTransform(time_code)
                    world_transform.Orthonormalize()
                    eye = (
                        world_transform.ExtractTranslation()
                        - coi_attr.Get(time_code)
                        + Gf.Vec3d(target[0], target[1], target[2])
                    )
            elif eye is not None and target is None:
                target = coi_attr.Get(time_code)
            else:
                update_coi = True
        # set eye position
        if eye is not None:
            _set_eye(xformable, eye)
        # set target position
        if target is not None:
            _set_target(xformable, target)
        # update center of interest (if exists)
        if update_coi:
            coi_attr.Set(target)
