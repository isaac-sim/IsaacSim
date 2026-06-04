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

"""Viewport manipulator for visualizing robot joint connections."""

import asyncio
from pathlib import Path
from typing import Any

import omni.kit.app
from omni.ui import scene as sc
from pxr import Gf

from .gesture import OverlayMenuClick, PreventOthers
from .utils import (
    CONNECTION_COLOR,
    LINE_BACKGROUND_COLOR,
    LINE_COLOR,
    get_active_viewport,
    get_camera_pose,
    get_camera_position,
    get_stage_safe,
    is_in_front_of_camera,
    is_position_in_viewport,
    joint_has_both_bodies,
    to_float_list,
)


class ConnectionManipulator(sc.Manipulator):
    """Viewport manipulator for drawing joint connection lines and overlays.

    Renders visual connections between parent and child joints in the 3D viewport,
    including directional arrows and clickable overlay indicators for joints
    that share the same screen position.

    Args:
        icon_scale: Scale factor for icons (currently unused).
        **kwargs: Additional arguments passed to the parent manipulator class.
    """

    def __init__(self, icon_scale: float = 1.0, **kwargs: Any) -> None:
        extension_path = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        self._connection_arrow_path = str(Path(extension_path).joinpath("data/icons/LinkArrow.svg"))
        self._stage: Any | None = None
        self._subscription: Any | None = None
        self._connections_panel: sc.Transform | None = None
        self._overlays_panel: sc.Transform | None = None
        super().__init__(**kwargs)
        self._connection_panel_map: dict[Any, sc.Transform] = {}
        self._overlay_panel_map: dict[Any, sc.Transform] = {}
        self._connection_arrow_images: dict[Any, Any] = {}
        self._cached_camera_position: Gf.Vec3d | None = None
        self._cached_camera_forward: Gf.Vec3d | None = None
        self._cached_viewport_api: Any | None = None
        self._redraw_future: asyncio.Task | None = None

    REDRAW_DEBOUNCE_MS = 300

    def _clear_connections_visuals(self) -> None:
        """Clear all connection lines and overlays without rebuilding.

        Stops drawing until the next debounced redraw, so the manipulator
        does not show misaligned arrows while the robot is updating.
        """
        connections = getattr(self, "_connections_panel", None)
        overlays = getattr(self, "_overlays_panel", None)
        if not connections or not overlays:
            return
        self._clear_all_panels()
        connections.clear()
        overlays.clear()

    async def _debounced_redraw(self) -> None:
        """Run after DEBOUNCE_MS of no model updates, then invalidate to redraw."""
        try:
            await asyncio.sleep(self.REDRAW_DEBOUNCE_MS / 1000.0)
        except asyncio.CancelledError:
            return
        self._redraw_future = None
        self.invalidate()

    def on_build(self) -> None:
        """Build the manipulator's scene graph structure.

        Called when the manipulator needs to construct its visual elements.
        Creates transform containers for connection lines and overlay indicators.
        """
        self._connections_panel = sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 0))
        self._overlays_panel = sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 0))
        self._overlay_panel_map = {}
        self.rebuild_connections()

    def _cache_camera_pose(self) -> None:
        """Cache camera pose at the start of a rebuild cycle.

        Stores camera position and forward direction to avoid redundant
        lookups during the same rebuild cycle.
        """
        camera_pose = get_camera_pose()
        if camera_pose:
            self._cached_camera_position, self._cached_camera_forward = camera_pose
        else:
            self._cached_camera_position = None
            self._cached_camera_forward = None

    def _get_cached_camera_pose(self) -> tuple[Gf.Vec3d, Gf.Vec3d] | None:
        """Retrieve the cached camera pose.

        Returns:
            Tuple of (camera_position, camera_forward) or None if not cached.
        """
        if self._cached_camera_position is not None and self._cached_camera_forward is not None:
            return (self._cached_camera_position, self._cached_camera_forward)
        return None

    def rebuild_connections(self, check_visibility: bool = True, connection_item: Any | None = None) -> None:
        """Rebuild connection visualizations.

        Clears and redraws either a single connection or all connections
        based on the provided parameters.

        Args:
            check_visibility: Whether to check viewport visibility before drawing.
            connection_item: Optional specific connection to rebuild.
                If None, rebuilds all connections.

        Example:

        .. code-block:: python

            manipulator.rebuild_connections()
        """
        self._stage = get_stage_safe()
        self._cache_camera_pose()
        self._cached_viewport_api = get_active_viewport()

        if connection_item:
            self._rebuild_single_connection(connection_item, check_visibility)
        else:
            self._rebuild_all_connections(check_visibility)

    def _rebuild_single_connection(self, connection_item: Any, check_visibility: bool) -> None:
        """Rebuild a single connection's visualization.

        Args:
            connection_item: The connection item to rebuild.
            check_visibility: Whether to check viewport visibility.
        """
        connection_panel = self._connection_panel_map.get(connection_item.joint_prim_path, None)
        if connection_panel:
            connection_panel.clear()
            self.build_connection_line(connection_item, check_visibility)

        overlay_panel = self._overlay_panel_map.get(connection_item.joint_prim_path, None)
        if overlay_panel:
            overlay_panel.clear()
        self.build_overlay(connection_item)

    def _rebuild_all_connections(self, check_visibility: bool) -> None:
        """Rebuild all connection visualizations.

        Args:
            check_visibility: Whether to check viewport visibility.

        Returns:
            None.
        """
        self._clear_all_panels()

        if not self._connections_panel or not self._overlays_panel:
            return

        self._connections_panel.clear()
        self._overlays_panel.clear()

        joint_connections = self.model.get_joint_connections()

        with self._connections_panel:
            for joint_connection in joint_connections:
                self.build_connection_line(joint_connection, check_visibility)

        with self._overlays_panel:
            for joint_connection in joint_connections:
                self.build_overlay(joint_connection)

    def _clear_all_panels(self) -> None:
        """Clear all connection and overlay panels."""
        for connection_panel in self._connection_panel_map.values():
            if connection_panel:
                connection_panel.clear()
        self._connection_panel_map = {}

        for overlay_panel in self._overlay_panel_map.values():
            if overlay_panel:
                overlay_panel.clear()
        self._overlay_panel_map = {}

    def build_connection_line(self, connection: Any, check_visibility: bool) -> None:
        """Build the visual elements for a single connection line.

        Args:
            connection: The connection item to visualize.
            check_visibility: Whether to check if endpoints are in the viewport.

        Returns:
            None.

        Example:

        .. code-block:: python

            manipulator.build_connection_line(connection, check_visibility=True)
        """
        if not connection:
            return

        # Only draw if the joint has both rigid body 0 and 1 defined
        if not joint_has_both_bodies(connection.joint_prim):
            return
        if connection.parent_joint_prim and not joint_has_both_bodies(connection.parent_joint_prim):
            return

        start_position = connection.joint_start_position
        end_position = connection.joint_end_position
        if start_position is None or end_position is None:
            return

        if check_visibility:
            viewport_api = self._cached_viewport_api
            start_visible = is_position_in_viewport(start_position, viewport_api)
            end_visible = is_position_in_viewport(end_position, viewport_api)
            if not start_visible or not end_visible:
                return

        transform = sc.Transform()
        self._connection_panel_map[connection.joint_prim_path] = transform
        with transform:
            self._draw_connection_line(start_position, end_position)
        transform.visible = connection.visible

    def build_overlay(self, connection: Any) -> None:
        """Build the overlay indicator for joints with overlapping positions.

        Creates a clickable circle at the joint position that opens a menu
        for selecting from multiple joints at the same screen location.

        Args:
            connection: The connection item to create an overlay for.

        Returns:
            None.

        Example:

        .. code-block:: python

            manipulator.build_overlay(connection)
        """
        if not connection:
            return
        if len(connection.overlay_paths) == 0:
            return

        joint_position = connection.joint_end_position
        if joint_position is None:
            return

        camera_pose = self._get_cached_camera_pose()
        if camera_pose:
            camera_position, camera_forward = camera_pose
            if not is_in_front_of_camera(joint_position, camera_position, camera_forward):
                return

        transform = sc.Transform()
        self._overlay_panel_map[connection.joint_prim_path] = transform
        with transform:
            self._build_overlay_circle(joint_position, connection)

    def _build_overlay_circle(self, joint_position: Gf.Vec3d, connection: Any) -> None:
        """Build the clickable overlay circle at a joint position.

        Args:
            joint_position: The world position for the overlay.
            connection: The connection item for gesture handling.
        """
        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(*joint_position)):
            with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.NDC):
                hit_area = sc.Arc(0.02, tesselation=11, color=(0, 0, 0, 0), wireframe=False, sector=False)
                sc.Arc(
                    0.015,
                    tesselation=11,
                    color=LINE_BACKGROUND_COLOR,
                    wireframe=True,
                    sector=False,
                    thickness=6,
                )
                sc.Arc(
                    0.015,
                    tesselation=11,
                    color=CONNECTION_COLOR,
                    wireframe=True,
                    sector=False,
                    thickness=5,
                )
                click_gesture = OverlayMenuClick(connection)
                click_gesture.manager = PreventOthers()
                hit_area.gestures = [click_gesture]

    def update_connection_position(self, item: Any) -> None:
        """Update the position of an existing connection visualization.

        Args:
            item: The connection item whose position has changed.

        Example:

        .. code-block:: python

            manipulator.update_connection_position(item)
        """
        self.rebuild_connections(check_visibility=True, connection_item=item)

    def on_model_updated(self, item: Any | None) -> None:
        """Handle model update notifications.

        When the robot (or camera) changes, clears connection visuals and
        schedules a single redraw after a short debounce so arrows are not
        drawn with stale positions. Redraw runs only after changes settle.

        If the model's ``_force_redraw_requested`` flag is set (by
        :meth:`~.model.ConnectionModel.force_rebuild`), the debounce is
        skipped and ``invalidate()`` is called immediately so the viewport
        lines update on the very next frame.

        Args:
            item: Changed item, or None for a full rebuild.

        Returns:
            None.
        """
        if getattr(self.model, "_force_redraw_requested", False):
            self.model._force_redraw_requested = False
            if self._redraw_future is not None:
                self._redraw_future.cancel()
                self._redraw_future = None
            self.invalidate()
            return
        if not item:
            if self._connections_panel is None:
                self.invalidate()
            else:
                self.rebuild_connections()
            return
        if item:
            item.needs_position_refresh = True
        if self._redraw_future is not None:
            self._redraw_future.cancel()
            self._redraw_future = None
        self._clear_connections_visuals()
        self._redraw_future = asyncio.ensure_future(self._debounced_redraw())

    def _draw_connection_line(self, start_position: Gf.Vec3d, end_position: Gf.Vec3d) -> None:
        """Draw the complete connection line with arrow.

        Draws line segments from start to midpoint and midpoint to end,
        with a directional arrow at the midpoint.

        Args:
            start_position: The starting world position.
            end_position: The ending world position.

        Returns:
            None.
        """
        direction_vector = end_position - start_position
        if direction_vector.GetLength() == 0:
            return

        inset_start = start_position + direction_vector * 0.1
        first_midpoint = start_position + direction_vector * 0.45
        center_point = start_position + direction_vector * 0.5
        second_midpoint = start_position + direction_vector * 0.55
        inset_end = end_position - direction_vector * 0.1

        self._draw_line_segment(inset_start, inset_end)
        self._draw_connection_arrow(center_point, direction_vector)

    def _draw_line_segment(
        self, start_position: Gf.Vec3d, end_position: Gf.Vec3d, draw_background: bool = True
    ) -> Any | None:
        """Draw a single line segment between two positions.

        Args:
            start_position: The start position.
            end_position: The end position.
            draw_background: If True, draws a thicker background line.

        Returns:
            The created line element, or None if not drawn.
        """
        camera_pose = self._get_cached_camera_pose()
        if camera_pose:
            camera_position, camera_forward = camera_pose
            if not is_in_front_of_camera(start_position, camera_position, camera_forward):
                return None
            if not is_in_front_of_camera(end_position, camera_position, camera_forward):
                return None

        start_floats = to_float_list(start_position)
        end_floats = to_float_list(end_position)

        if draw_background:
            sc.Line(start_floats, end_floats, color=LINE_COLOR, thickness=4)
        return sc.Line(start_floats, end_floats, color=LINE_BACKGROUND_COLOR, thickness=3)

    def _draw_connection_arrow(self, arrow_position: Gf.Vec3d, direction_vector: Gf.Vec3d) -> None:
        """Draw a directional arrow at the connection midpoint.

        Calculates arrow head vectors based on camera position and
        connection direction.

        Args:
            arrow_position: The position for the arrow center.
            direction_vector: The direction vector of the connection.

        Returns:
            None.
        """
        camera_pose = self._get_cached_camera_pose()
        if not camera_pose:
            camera_pose = get_camera_pose()
        camera_position = camera_pose[0] if camera_pose else get_camera_position()
        camera_forward = camera_pose[1] if camera_pose else None
        if not camera_position:
            return

        camera_direction = camera_position - arrow_position
        perpendicular = Gf.Cross(camera_direction, direction_vector)
        if perpendicular.GetLength() == 0:
            return
        perpendicular = perpendicular / perpendicular.GetLength()

        normalized_direction = direction_vector / direction_vector.GetLength()
        arrow_wing_left = perpendicular + normalized_direction
        arrow_wing_right = perpendicular - normalized_direction

        arrow_length = 0.01
        view_distance = camera_direction.GetLength()
        if camera_forward is not None:
            depth = Gf.Dot(arrow_position - camera_position, camera_forward)
            if depth > 0.0:
                view_distance = depth
        if view_distance > 0.0:
            arrow_length = arrow_length * view_distance

        if arrow_wing_left.GetLength() > 0:
            arrow_wing_left = arrow_wing_left / arrow_wing_left.GetLength() * arrow_length
        if arrow_wing_right.GetLength() > 0:
            arrow_wing_right = arrow_wing_right / arrow_wing_right.GetLength() * arrow_length

        self._draw_line_segment(arrow_position, arrow_position - arrow_wing_left)
        self._draw_line_segment(arrow_position, arrow_position + arrow_wing_right)
