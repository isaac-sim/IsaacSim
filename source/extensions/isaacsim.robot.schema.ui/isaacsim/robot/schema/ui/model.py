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

"""Viewport model classes for joint connection visualization."""

import asyncio
import math
from collections import defaultdict
from enum import Enum, auto
from typing import Any

import omni.kit.app
import omni.kit.viewport.utility as viewport_utility
from omni.ui import scene as sc
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics

from .utils import (
    get_active_viewport,
    get_joint_position,
    get_link_position,
    get_prim_safe,
    get_stage_safe,
    world_to_screen_position,
)


class RebuildType(Enum):
    """Enumeration of rebuild types for connection visualization.

    Defines the scope of rebuild required when the scene or camera changes.
    """

    NONE = auto()
    """No rebuild required."""
    CAMERA_ONLY = auto()
    """Only camera-related updates needed, no joint position recalculation."""
    SINGLE_JOINT = auto()
    """Rebuild limited to a single joint connection."""
    FULL = auto()
    """Complete rebuild of all joint connections and visualization."""


class ConnectionItem(sc.AbstractManipulatorItem):
    """Represents a visual connection between two joints in the viewport.

    Stores the joint prim references, robot root path, and position data
    needed to draw connection lines between parent and child joints.

    Args:
        joint_prim: The joint prim this connection represents.
        parent_joint_prim: The parent joint prim, or None for root joints.
        parent_link_prim: The parent link prim for root joints, if available.
        robot_root_path: The path to the robot root prim.
        joint_pos: The initial world position of this joint.
        parent_joint_pos: The initial world position of the parent joint.
    """

    def __init__(
        self,
        joint_prim: Usd.Prim,
        parent_joint_prim: Usd.Prim | None,
        parent_link_prim: Usd.Prim | None,
        robot_root_path: Sdf.Path,
        joint_pos: Gf.Vec3d | None,
        parent_joint_pos: Gf.Vec3d | None,
    ) -> None:
        self._joint_position = joint_pos
        self._parent_joint_position = parent_joint_pos
        self._visible = True
        self._overlay_paths: list[Sdf.Path] = []
        self._overlay_names: list[str] = []
        self._joint_prim_path = joint_prim.GetPath()
        self._parent_joint_path = parent_joint_prim.GetPath() if parent_joint_prim else None
        self._parent_link_path = parent_link_prim.GetPath() if parent_link_prim else None
        self._robot_root_path = robot_root_path
        self._needs_position_refresh = False
        super().__init__()

    @classmethod
    def from_paths(
        cls,
        joint_prim_path: Sdf.Path,
        parent_joint_path: Sdf.Path | None,
        parent_link_path: Sdf.Path | None,
        robot_root_path: Sdf.Path,
        joint_pos: Gf.Vec3d | None,
        parent_joint_pos: Gf.Vec3d | None,
    ) -> "ConnectionItem":
        """Construct a ConnectionItem from paths and positions (e.g. from a background worker).

        Prims are resolved from the default USD context when joint_prim etc. are accessed.

        Args:
            joint_prim_path: Path of the joint prim.
            parent_joint_path: Path of the parent joint, or None.
            parent_link_path: Path of the parent link for root joints, or None.
            robot_root_path: Path to the robot root prim.
            joint_pos: World position of the joint.
            parent_joint_pos: World position of the parent joint.

        Returns:
            A ConnectionItem that resolves prims via get_prim_safe when accessed.
        """
        self = cls.__new__(cls)
        self._joint_position = joint_pos
        self._parent_joint_position = parent_joint_pos
        self._visible = True
        self._overlay_paths = []
        self._overlay_names = []
        self._joint_prim_path = joint_prim_path
        self._parent_joint_path = parent_joint_path
        self._parent_link_path = parent_link_path
        self._robot_root_path = robot_root_path
        self._needs_position_refresh = False
        super(ConnectionItem, self).__init__()
        return self

    @property
    def needs_position_refresh(self) -> bool:
        """Return True if joint positions need recalculation.

        Returns:
            True if positions need refresh, False otherwise.

        Example:

        .. code-block:: python

            if item.needs_position_refresh:
                pass
        """
        return self._needs_position_refresh

    @needs_position_refresh.setter
    def needs_position_refresh(self, value: bool) -> None:
        """Set the position refresh flag.

        Args:
            value: Boolean indicating if refresh is needed.

        Example:

        .. code-block:: python

            item.needs_position_refresh = True
        """
        self._needs_position_refresh = value

    @property
    def joint_prim(self) -> Usd.Prim | None:
        """Return the joint prim this connection represents.

        Returns:
            The joint prim, or None if invalid.

        Example:

        .. code-block:: python

            joint = item.joint_prim
        """
        return get_prim_safe(self._joint_prim_path)

    @property
    def parent_joint_prim(self) -> Usd.Prim | None:
        """Return the parent joint prim.

        Returns:
            The parent joint prim, or None if this is a root joint or invalid.

        Example:

        .. code-block:: python

            parent_joint = item.parent_joint_prim
        """
        return get_prim_safe(self._parent_joint_path)

    @property
    def robot_root_prim(self) -> Usd.Prim | None:
        """Return the robot root prim.

        Returns:
            The robot root prim, or None if invalid.

        Example:

        .. code-block:: python

            robot_root = item.robot_root_prim
        """
        return get_prim_safe(self._robot_root_path)

    @property
    def joint_start_position(self) -> Gf.Vec3d | None:
        """Return the world position of the parent joint (connection start).

        Refreshes positions if the refresh flag is set.

        Returns:
            The parent joint world position.

        Example:

        .. code-block:: python

            start_pos = item.joint_start_position
        """
        if self._needs_position_refresh:
            self._refresh_positions()
        return self._parent_joint_position

    @property
    def joint_end_position(self) -> Gf.Vec3d | None:
        """Return the world position of this joint (connection end).

        Refreshes positions if the refresh flag is set.

        Returns:
            The joint world position.

        Example:

        .. code-block:: python

            end_pos = item.joint_end_position
        """
        if self._needs_position_refresh:
            self._refresh_positions()
        return self._joint_position

    def _refresh_positions(self) -> None:
        """Recalculate joint positions from the USD stage."""
        self._joint_position = get_joint_position(self._robot_root_path, self._joint_prim_path)
        if self._parent_joint_path:
            self._parent_joint_position = get_joint_position(self._robot_root_path, self._parent_joint_path)
        elif self._parent_link_path:
            self._parent_joint_position = get_link_position(self._parent_link_path)
        self._needs_position_refresh = False

    @property
    def visible(self) -> bool:
        """Return True if this connection should be drawn.

        Returns:
            True if the connection should be drawn.

        Example:

        .. code-block:: python

            is_visible = item.visible
        """
        return self._visible

    @property
    def overlay_prims(self) -> list[Usd.Prim]:
        """Return overlapping joint prims at this position.

        Returns:
            List of joint prims that share this screen position.

        Example:

        .. code-block:: python

            overlaps = item.overlay_prims
        """
        result = []
        for path in self._overlay_paths:
            prim = get_prim_safe(path)
            if prim:
                result.append(prim)
        return result

    @overlay_prims.setter
    def overlay_prims(self, value: Any) -> None:
        """Set the list of overlapping joints.

        Args:
            value: List of prims or paths for overlapping joints.

        Example:

        .. code-block:: python

            item.overlay_prims = [joint_prim]
        """
        if isinstance(value, list):
            self._overlay_paths = []
            for item in value:
                if hasattr(item, "GetPath"):
                    self._overlay_paths.append(item.GetPath())
                elif isinstance(item, Sdf.Path):
                    self._overlay_paths.append(item)
        else:
            self._overlay_paths = value

    @property
    def overlay_paths(self) -> list[Sdf.Path]:
        """Return overlapping joint paths.

        Returns:
            List of paths for overlapping joints.

        Example:

        .. code-block:: python

            paths = item.overlay_paths
        """
        return self._overlay_paths

    @overlay_paths.setter
    def overlay_paths(self, value: list[Sdf.Path]) -> None:
        """Set the overlay paths directly.

        Args:
            value: List of joint paths.

        Example:

        .. code-block:: python

            item.overlay_paths = [path]
        """
        self._overlay_paths = value

    @property
    def overlay_names(self) -> list[str]:
        """Return display names of overlapping joints.

        Returns:
            List of joint names for overlay menu display.

        Example:

        .. code-block:: python

            names = item.overlay_names
        """
        return self._overlay_names

    @overlay_names.setter
    def overlay_names(self, value: list[str]) -> None:
        """Set the overlay display names.

        Args:
            value: List of name strings.

        Example:

        .. code-block:: python

            item.overlay_names = ["jointA", "jointB"]
        """
        self._overlay_names = value

    @property
    def parent_joint_path(self) -> Sdf.Path | None:
        """Return the path of the parent joint.

        Returns:
            Parent joint path, or None for root joints.

        Example:

        .. code-block:: python

            parent_path = item.parent_joint_path
        """
        return self._parent_joint_path

    @property
    def joint_prim_path(self) -> Sdf.Path:
        """Return the path of this joint.

        Returns:
            The joint prim path.

        Example:

        .. code-block:: python

            joint_path = item.joint_prim_path
        """
        return self._joint_prim_path

    @property
    def robot_root_path(self) -> Sdf.Path:
        """Return the path of the robot root prim.

        Returns:
            The robot root path.

        Example:

        .. code-block:: python

            root_path = item.robot_root_path
        """
        return self._robot_root_path

    def is_valid(self) -> bool:
        """Return True if this connection item references valid prims.

        Returns:
            True if both the joint and robot root prims are valid.

        Example:

        .. code-block:: python

            if item.is_valid():
                pass
        """
        stage = get_stage_safe()
        if not stage:
            return False
        if not get_prim_safe(self._joint_prim_path, stage):
            return False
        if not get_prim_safe(self._robot_root_path, stage):
            return False
        if self._parent_joint_path:
            if not get_prim_safe(self._parent_joint_path, stage):
                return False
        return True


class _ForceRedrawItem:
    """Marker class for immediate-redraw requests from ConnectionModel.force_rebuild().

    Used by ``ConnectionManipulator.on_model_updated`` to detect when the model
    requests an immediate redraw.  The model sets ``_force_redraw_requested = True``
    and fires
    ``_item_changed(None)`` (the only value accepted by the C++ binding).
    The manipulator reads the flag and calls ``invalidate()`` directly,
    bypassing the debounce.
    """


class ConnectionModel(sc.AbstractManipulatorModel):
    """Model managing the state of joint connections for viewport visualization.

    Tracks all joint connections, listens for USD stage changes, and queues
    batched updates; the manipulator debouncer coalesces rapid changes.
    """

    OVERLAY_REFRESH_INTERVAL_MS = 100.0
    """Minimum time interval in milliseconds between overlay clustering operations."""

    def __init__(self) -> None:
        super().__init__()
        self._joint_connections: list[ConnectionItem] = []
        self._stage: Usd.Stage | None = None
        self._world_unit = 0.0
        self._usd_listener = None
        self._is_self_editing = False
        self._joint_connections_map: dict[Sdf.Path, ConnectionItem] = {}
        self._joint_connections_by_prefix: dict[Sdf.Path, set[ConnectionItem]] = defaultdict(set)
        self._pending_rebuild = False
        self._pending_connections: set[ConnectionItem] = set()
        self._pending_rebuild_type = RebuildType.NONE
        self._last_overlay_refresh_ms: float | None = None
        self._last_camera_rebuild_ms: float | None = None
        self._force_redraw_requested: bool = False
        self._refresh_stage()

    def _revoke_usd_listener(self) -> None:
        """Revoke the USD change listener so no more stage change callbacks fire."""
        if self._usd_listener:
            self._usd_listener.Revoke()
            self._usd_listener = None

    def _refresh_stage(self) -> None:
        """Refresh the stage reference and USD listener.

        Re-establishes the connection to the current stage and sets up
        the USD notice listener for change tracking. Registers a listener
        only when there are joint connections to visualize.
        """
        self._revoke_usd_listener()
        self._stage = get_stage_safe()
        self._world_unit = 0.0
        if self._stage:
            self._world_unit = UsdGeom.GetStageMetersPerUnit(self._stage)
        if self._world_unit == 0.0:
            self._world_unit = 0.1
        if self._stage and self._joint_connections_map:
            self._usd_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_usd_changed, self._stage)

    def set_joint_connections(self, joint_connections: list[ConnectionItem]) -> None:
        """Set the list of joint connections to visualize and trigger rebuild.

        Args:
            joint_connections: List of connection items.

        Returns:
            None.

        Example:

        .. code-block:: python

            model.set_joint_connections(connections)
        """
        self._joint_connections = joint_connections
        self._joint_connections_map = {connection.joint_prim_path: connection for connection in joint_connections}
        if not joint_connections:
            self._joint_connections_by_prefix = defaultdict(set)
            self._refresh_overlay_groups()
            self._revoke_usd_listener()
            self._pending_rebuild = False
            self._pending_connections.clear()
            self._pending_rebuild_type = RebuildType.NONE
            self._item_changed(None)
            return
        self._build_connection_prefix_map()
        self._refresh_overlay_groups()
        self.rebuild_connections()
        self._refresh_stage()

    def _build_connection_prefix_map(self) -> None:
        """Build a prefix map for fast USD change lookups.

        Registers joint paths and the joint's body0/body1 (link) paths so
        that xform changes on links during tracking trigger a redraw.
        """
        self._joint_connections_by_prefix = defaultdict(set)
        stage = get_stage_safe()
        for connection in self._joint_connections:
            paths_to_register = [connection.joint_prim_path]
            joint_prim = get_prim_safe(connection.joint_prim_path, stage)
            if joint_prim:
                joint = UsdPhysics.Joint(joint_prim)
                if joint:
                    for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()):
                        targets = rel.GetTargets()
                        if targets:
                            paths_to_register.append(targets[0])
            for path in paths_to_register:
                while True:
                    self._joint_connections_by_prefix[path].add(connection)
                    if path == Sdf.Path.absoluteRootPath:
                        break
                    path = path.GetParentPath()

    def _on_usd_changed(self, notice: Any, stage: Any) -> None:
        """Handle USD stage change notifications.

        Determines which joints are affected by the change and queues
        appropriate rebuild operations.

        Args:
            notice: USD change notice.
            stage: USD stage that changed.

        Returns:
            None.
        """
        if self._is_self_editing:
            return

        # Skip processing if a rebuild is already pending to avoid cascading
        if self._pending_rebuild:
            return

        if not self._joint_connections_map:
            return

        camera_path = viewport_utility.get_viewport_window_camera_path()
        camera_changed = False
        affected_connections: set[ConnectionItem] = set()

        for path in notice.GetChangedInfoOnlyPaths():
            prim_path = path.GetPrimPath() if path.IsPropertyPath() else path

            if camera_path == prim_path:
                camera_changed = True
                continue

            connections = self._joint_connections_by_prefix.get(prim_path)
            if connections:
                for connection in connections:
                    connection.needs_position_refresh = True
                affected_connections.update(connections)

        # Resync paths (e.g. first-time xform-op or joint-attribute authoring on
        # body/joint prims) also require a position refresh.  Without this, a
        # one-shot pose application whose USD writes land in GetResyncedPaths()
        # (new attribute creation) is silently ignored and the manipulator never
        # redraws.
        for path in notice.GetResyncedPaths():
            prim_path = path.GetPrimPath() if path.IsPropertyPath() else path
            connections = self._joint_connections_by_prefix.get(prim_path)
            if connections:
                for connection in connections:
                    connection.needs_position_refresh = True
                affected_connections.update(connections)

        if affected_connections:
            if len(affected_connections) == 1:
                self._queue_rebuild(next(iter(affected_connections)), RebuildType.FULL)
            else:
                self._queue_rebuild(None, RebuildType.FULL)
        elif camera_changed:
            now_ms = omni.kit.app.get_app().get_time_since_start_ms()
            if self._last_camera_rebuild_ms is None or (now_ms - self._last_camera_rebuild_ms) >= 300.0:
                self._last_camera_rebuild_ms = now_ms
                self._queue_rebuild(None, RebuildType.CAMERA_ONLY)

    def destroy(self) -> None:
        """Revoke the USD listener and clear state. Call before dropping the model."""
        self._revoke_usd_listener()
        self._stage = None
        self._joint_connections = []
        self._joint_connections_map = {}
        self._joint_connections_by_prefix = defaultdict(set)
        self._pending_rebuild = False
        self._pending_connections.clear()
        self._pending_rebuild_type = RebuildType.NONE

    def get_joint_connections(self) -> list[ConnectionItem]:
        """Return the list of all joint connections.

        Returns:
            List of connection items.

        Example:

        .. code-block:: python

            connections = model.get_joint_connections()
        """
        return self._joint_connections

    def rebuild_connections(
        self, connection: ConnectionItem | None = None, rebuild_type: RebuildType = RebuildType.FULL
    ) -> None:
        """Request a rebuild of connection visualizations.

        Args:
            connection: Optional specific connection to rebuild; None rebuilds all.
            rebuild_type: Type of rebuild to perform.

        Example:

        .. code-block:: python

            model.rebuild_connections()
        """
        if connection:
            connection.needs_position_refresh = True
            self._queue_rebuild(connection, RebuildType.SINGLE_JOINT)
        else:
            if rebuild_type != RebuildType.CAMERA_ONLY:
                for connection in self._joint_connections_map.values():
                    connection.needs_position_refresh = True
            self._queue_rebuild(None, rebuild_type)

    def force_rebuild(self) -> None:
        """Mark all connections as needing position refresh and redraw immediately.

        Unlike :meth:`rebuild_connections`, this method bypasses the
        ``_pending_rebuild`` guard and the async deferred-rebuild chain.  It
        sets ``_force_redraw_requested`` and fires ``_item_changed(None)``,
        which the manipulator recognises as a signal to skip its debounce and
        call ``invalidate()`` on the next frame.

        Intended for one-shot pose applications ("Set Robot to Pose") where the
        USD body-transform writes have already been committed synchronously and
        we just need the viewport lines to catch up immediately.
        """
        for connection in self._joint_connections_map.values():
            connection.needs_position_refresh = True
        self._force_redraw_requested = True
        self._item_changed(None)

    def _queue_rebuild(
        self, connection: ConnectionItem | None = None, rebuild_type: RebuildType = RebuildType.FULL
    ) -> None:
        """Queue a deferred rebuild operation.

        Batches multiple rebuild requests into a single deferred update
        to avoid redundant processing.

        Args:
            connection: Optional specific connection for targeted rebuild.
            rebuild_type: Type of rebuild requested.

        Returns:
            None.
        """
        if connection:
            self._pending_connections.add(connection)
            if self._pending_rebuild_type == RebuildType.NONE:
                self._pending_rebuild_type = rebuild_type
            elif rebuild_type == RebuildType.FULL:
                self._pending_rebuild_type = RebuildType.FULL
            elif rebuild_type == RebuildType.SINGLE_JOINT and self._pending_rebuild_type == RebuildType.CAMERA_ONLY:
                self._pending_rebuild_type = RebuildType.SINGLE_JOINT
        else:
            self._pending_connections.clear()
            if rebuild_type == RebuildType.FULL:
                self._pending_rebuild_type = RebuildType.FULL
            elif self._pending_rebuild_type == RebuildType.NONE:
                self._pending_rebuild_type = rebuild_type

        if self._pending_rebuild:
            return
        self._pending_rebuild = True
        asyncio.ensure_future(self._deferred_rebuild())

    async def _deferred_rebuild(self) -> None:
        """Execute the deferred rebuild after the next frame update.

        Waits for the next application update to avoid interrupting
        active gesture processing, then performs the queued rebuild.
        """
        try:
            await omni.kit.app.get_app().next_update_async()
        finally:
            rebuild_type = self._pending_rebuild_type
            self._pending_rebuild_type = RebuildType.NONE

            connection = None
            if self._pending_connections:
                if len(self._pending_connections) == 1:
                    connection = next(iter(self._pending_connections))
                self._pending_connections.clear()

            if self._should_refresh_overlays(rebuild_type):
                self._refresh_overlay_groups()

            # Keep _pending_rebuild True during _item_changed to coalesce
            # any USD changes triggered by the manipulator rebuild
            if connection and rebuild_type == RebuildType.SINGLE_JOINT:
                self._item_changed(connection)
            else:
                self._item_changed(None)
            for i in range(4):
                await omni.kit.app.get_app().next_update_async()
            # Reset after _item_changed completes to prevent immediate re-queue
            self._pending_rebuild = False

    def _should_refresh_overlays(self, rebuild_type: RebuildType) -> bool:
        """Return True if overlay clustering should run for this rebuild.

        Args:
            rebuild_type: Pending rebuild type.

        Returns:
            True if overlays should be refreshed for this rebuild.
        """
        if not self._joint_connections:
            return False
        if rebuild_type == RebuildType.FULL:
            return True
        now_ms = omni.kit.app.get_app().get_time_since_start_ms()
        if self._last_overlay_refresh_ms is None:
            return True
        return (now_ms - self._last_overlay_refresh_ms) >= self.OVERLAY_REFRESH_INTERVAL_MS

    def _refresh_overlay_groups(self, screen_epsilon: float = 0.02, world_epsilon: float = 0.5) -> None:
        """Cluster overlapping joints based on screen proximity.

        Groups joints that appear at the same screen position so they
        can be displayed in an overlay menu for selection.

        Args:
            screen_epsilon: Distance threshold in normalized screen coordinates (0-1).
            world_epsilon: Distance threshold in world units for fallback clustering.
        """
        self._last_overlay_refresh_ms = omni.kit.app.get_app().get_time_since_start_ms()
        viewport_api = get_active_viewport()

        for connection in self._joint_connections:
            if connection:
                connection.overlay_paths = []
                connection.overlay_names = []

        valid_items = self._collect_valid_connection_positions(viewport_api)
        robot_groups = self._group_connections_by_robot(valid_items)
        self._cluster_and_assign_overlays(robot_groups, screen_epsilon, world_epsilon)

    def _collect_valid_connection_positions(self, viewport_api: Any) -> list[tuple[int, Any, Any]]:
        """Collect screen positions for all valid connections.

        Args:
            viewport_api: The viewport API for coordinate conversion.

        Returns:
            List of tuples (index, screen_position, robot_root_path).
        """
        valid_items = []
        for index, connection in enumerate(self._joint_connections):
            if not connection or not connection.is_valid():
                continue
            end_position = connection.joint_end_position
            if end_position is None:
                continue
            screen_position = world_to_screen_position(end_position, viewport_api)
            if screen_position is None:
                continue
            robot_root = getattr(connection, "robot_root_path", None)
            valid_items.append((index, screen_position, robot_root))
        return valid_items

    def _group_connections_by_robot(self, valid_items: list[tuple[int, Any, Any]]) -> dict[Any, list[tuple[int, Any]]]:
        """Group connection items by their robot root path.

        Args:
            valid_items: List of (index, screen_position, robot_root) tuples.

        Returns:
            Dictionary mapping robot_root_path to list of (index, screen_position).
        """
        robot_groups = defaultdict(list)
        for index, screen_position, robot_root in valid_items:
            robot_groups[robot_root].append((index, screen_position))
        return robot_groups

    def _cluster_and_assign_overlays(
        self, robot_groups: dict[Any, list[tuple[int, Any]]], screen_epsilon: float, world_epsilon: float
    ) -> None:
        """Apply union-find clustering and assign overlay groups.

        Args:
            robot_groups: Dictionary of robot-grouped connection items.
            screen_epsilon: Screen-space distance threshold.
            world_epsilon: World-space distance threshold.
        """
        for robot_root, items in robot_groups.items():
            item_count = len(items)
            if item_count <= 1:
                continue

            clusters = self._compute_clusters(items, screen_epsilon, world_epsilon)
            self._assign_overlays_from_clusters(clusters)

    def _compute_clusters(
        self, items: list[tuple[int, Any]], screen_epsilon: float, world_epsilon: float
    ) -> dict[int, list[int]]:
        """Compute clusters using union-find algorithm.

        Args:
            items: List of (index, screen_position) tuples.
            screen_epsilon: Screen-space distance threshold.
            world_epsilon: World-space distance threshold.

        Returns:
            Dictionary mapping cluster root to list of item indices.
        """
        item_count = len(items)
        parent = list(range(item_count))

        def find_root(index: int) -> int:
            """Find the root representative for an item index.

            Args:
                index: The item index to resolve.

            Returns:
                The root index for the item.
            """
            if parent[index] != index:
                parent[index] = find_root(parent[index])
            return parent[index]

        def union_sets(index_a: int, index_b: int) -> None:
            """Union two disjoint sets by index.

            Args:
                index_a: Index of the first item.
                index_b: Index of the second item.
            """
            root_a, root_b = find_root(index_a), find_root(index_b)
            if root_a != root_b:
                parent[root_a] = root_b

        for i in range(item_count):
            for j in range(i + 1, item_count):
                position_i = items[i][1]
                position_j = items[j][1]
                if len(position_i) != len(position_j):
                    continue
                if len(position_i) == 2:
                    distance = math.sqrt((position_i[0] - position_j[0]) ** 2 + (position_i[1] - position_j[1]) ** 2)
                    threshold = screen_epsilon
                else:
                    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(position_i, position_j)))
                    threshold = world_epsilon
                if distance <= threshold:
                    union_sets(i, j)

        clusters = defaultdict(list)
        for i in range(item_count):
            clusters[find_root(i)].append(items[i][0])
        return clusters

    def _assign_overlays_from_clusters(self, clusters: dict[int, list[int]]) -> None:
        """Assign overlay paths and names from computed clusters.

        Args:
            clusters: Dictionary mapping cluster root to list of connection indices.
        """
        for indices in clusters.values():
            if len(indices) <= 1:
                continue
            base_index = min(indices)
            base_connection = self._joint_connections[base_index]
            for index in indices:
                if index == base_index:
                    continue
                other_connection = self._joint_connections[index]
                if not other_connection or not other_connection.is_valid():
                    continue
                base_connection.overlay_paths.append(other_connection.joint_prim_path)
                base_connection.overlay_names.append(other_connection.joint_prim_path.name)
