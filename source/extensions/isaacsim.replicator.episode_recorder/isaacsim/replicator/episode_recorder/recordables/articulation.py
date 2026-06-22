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

"""USD-only articulation pose recordable.

Samples the per-link world pose of every ``UsdGeom.Xformable`` descendant of the
articulation root (plus the root itself) via a single batched
:class:`isaacsim.core.experimental.prims.XformPrim`. No physics tensor backend, no
simulation view — reads and writes go through USD / Fabric so replay is fully
deterministic and is unaffected by timeline state, physics stepping, or PhysX view
invalidation.

Channel layout:

* ``positions``    — shape ``(L, 3)``, float32, world meters.
* ``orientations`` — shape ``(L, 4)``, float32, quaternion ``wxyz`` in world.

``L`` is the discovered link count, frozen at :meth:`on_session_open` time and stored in
the manifest so the replayer binds to the same prim set.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from typing import Any

import carb
import numpy as np

from ..base import ChannelDescriptor, Recordable, ReplayPolicy
from ..registry import register_recordable
from ._utils import is_missing_xform_ops_error, to_numpy_f32

_ZERO_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)


@register_recordable
class ArticulationRecordable(Recordable):
    """USD-only link-pose recordable for an articulated body.

    Args:
        group: HDF5 group path (e.g. ``"state/robot"``).
        prim_path: Absolute USD path of the articulation root.
        link_paths: Optional pre-known list of link prim paths in batch order. When
            provided (e.g. by :meth:`from_manifest`), the recordable binds to exactly
            those paths at replay time — no stage walk. When ``None``, the paths are
            discovered at :meth:`on_session_open` from the articulation subtree.
        include_root: Whether to include the articulation root prim itself as the first
            batch entry (default ``True``).
    """

    TYPE_ID = "articulation"

    def __init__(
        self,
        *,
        group: str,
        prim_path: str,
        link_paths: list[str] | None = None,
        include_root: bool = True,
    ) -> None:
        super().__init__(group=group)
        self.prim_path = str(prim_path)
        self.include_root = bool(include_root)
        self._link_paths: list[str] = [str(p) for p in (link_paths or [])]
        self._wrapper: Any = None
        self._xform_ops_reset: bool = False

    @property
    def num_links(self) -> int:
        """Run the num links operation."""
        return len(self._link_paths)

    @property
    def link_paths(self) -> list[str]:
        """Run the link paths operation."""
        return list(self._link_paths)

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        """Describe the recorded channels.

        Returns:
            Channel descriptors keyed by channel name.
        """
        n = self.num_links
        return {
            "positions": ChannelDescriptor(shape=(n, 3), dtype="f4", units="meters", space="world"),
            "orientations": ChannelDescriptor(shape=(n, 4), dtype="f4", quaternion_order="wxyz", space="world"),
        }

    def on_session_open(self, stage: Any) -> None:
        """Open the recordable session.

        Args:
            stage: USD stage to use.
        """
        prim = stage.GetPrimAtPath(self.prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"ArticulationRecordable: prim {self.prim_path} not found on stage.")
        if not self._link_paths:
            self._link_paths = self._discover_link_paths(stage)
        if not self._link_paths:
            carb.log_warn(
                f"[ArticulationRecordable {self.prim_path}] no Xformable links found under root "
                f"(this is normal for a PhysX fixed-base articulation whose "
                f"ArticulationRootAPI is applied to a joint). The recordable will be a no-op "
                f"for this session."
            )
            self._wrapper = None
            return
        self._build_wrapper()

    def on_session_close(self) -> None:
        """Close the recordable session."""
        self._wrapper = None
        self._xform_ops_reset = False

    def _discover_link_paths(self, stage: Any) -> list[str]:
        """Walk the articulation subtree and return rigid link prim paths.

        Children are visited in deterministic DFS order (``prim.GetChildren()`` order).
        Joint prims are skipped because they carry relationship metadata, and visual /
        collision containers are left to inherit from their rigid link parents.

        Fixed-base PhysX articulations apply ``ArticulationRootAPI`` to a
        ``UsdPhysicsJoint`` rather than to the link-Xform subtree; the joint is not
        ``Xformable`` and has no Xformable descendants of its own. When that case is
        detected, the walk starts from the nearest Xformable ancestor *only if* that
        ancestor has at least one Xformable descendant outside the joint itself (i.e.
        the articulation's link subtree really does live there). This avoids promoting
        an unrelated scope like ``/World`` to a "link" for an orphan joint.

        Args:
            stage: USD stage to use.

        Returns:
            Discovered link prim paths.
        """
        from pxr import UsdGeom, UsdPhysics

        root = stage.GetPrimAtPath(self.prim_path)
        walk_root = root
        if not root.IsA(UsdGeom.Xformable) or root.IsA(UsdPhysics.Joint):
            ancestor = root.GetParent()
            while ancestor.IsValid() and not ancestor.IsA(UsdGeom.Xformable):
                ancestor = ancestor.GetParent()
            if ancestor.IsValid() and self._has_xformable_descendant(ancestor, exclude_path=str(root.GetPath())):
                walk_root = ancestor
                carb.log_info(
                    f"[ArticulationRecordable {self.prim_path}] root is not Xformable "
                    f"(likely a PhysX joint-as-root); discovering links under Xformable "
                    f"ancestor {walk_root.GetPath()}."
                )
            else:
                return []

        xform_paths: list[str] = []
        rigid_link_paths: list[str] = []
        seen: set[str] = set()

        def add_xformable(prim: Any) -> None:
            if not prim.IsA(UsdGeom.Xformable):
                return
            p = str(prim.GetPath())
            if p in seen:
                return
            xform_paths.append(p)
            seen.add(p)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_link_paths.append(p)

        if self.include_root:
            add_xformable(walk_root)

        stack = deque(walk_root.GetChildren())
        while stack:
            child = stack.popleft()
            if not child.IsValid() or not child.IsActive():
                continue
            if child.IsA(UsdPhysics.Joint):
                continue
            add_xformable(child)
            stack.extend(child.GetChildren())

        selected_paths = rigid_link_paths or xform_paths
        if not self.include_root or not walk_root.IsA(UsdGeom.Xformable):
            return selected_paths

        root_path = str(walk_root.GetPath())
        if root_path in selected_paths:
            return selected_paths
        return [root_path, *selected_paths]

    @staticmethod
    def _has_xformable_descendant(prim: Any, *, exclude_path: str | None = None) -> bool:
        """Return whether ``prim`` has a useful Xformable descendant.

        Joint descendants and ``exclude_path`` are ignored. The result decides whether
        walking up to ``prim`` is justified.

        Args:
            prim: Prim to inspect.
            exclude_path: Prim path to exclude from the descendant search.

        Returns:
            True if a matching xformable descendant exists, otherwise False.
        """
        from pxr import UsdGeom, UsdPhysics

        stack = deque(prim.GetChildren())
        while stack:
            child = stack.popleft()
            if not child.IsValid() or not child.IsActive():
                continue
            if child.IsA(UsdPhysics.Joint):
                continue
            if exclude_path is not None and str(child.GetPath()) == exclude_path:
                continue
            if child.IsA(UsdGeom.Xformable):
                return True
            stack.extend(child.GetChildren())
        return False

    def _build_wrapper(self) -> None:
        from isaacsim.core.experimental.prims import XformPrim

        self._wrapper = XformPrim(self._link_paths)

    def _zeros_frame(self) -> dict[str, np.ndarray]:
        n = self.num_links
        return {
            "positions": np.zeros((n, 3), dtype=np.float32),
            "orientations": np.tile(_ZERO_QUAT, (n, 1)).astype(np.float32),
        }

    def pose_paths(self) -> list[str] | None:
        """Enlist every discovered link into the recorder's shared pose batch.

        When the no-op fallback is active (no Xformable links were found) this returns
        ``None`` so the recorder skips this recordable in the batch and uses
        :meth:`sample` (which returns zeros) instead.

        Returns:
            Prim paths for shared pose batching, or None when disabled.
        """
        if self._wrapper is None or not self._link_paths:
            return None
        return list(self._link_paths)

    def consume_pose_batch(self, positions: np.ndarray, orientations: np.ndarray) -> dict[str, np.ndarray]:
        """Consume a batched pose sample.

        Args:
            positions: Batched positions for this recordable.
            orientations: Batched orientations for this recordable.

        Returns:
            Sampled frame data keyed by channel name.
        """
        n = self.num_links
        return {
            "positions": positions.reshape(n, 3),
            "orientations": orientations.reshape(n, 4),
        }

    def sample(self) -> dict[str, np.ndarray]:
        """Sample one frame of data.

        Returns:
            Sampled frame data keyed by channel name.
        """
        if self._wrapper is None:
            return self._zeros_frame()
        try:
            pos_wp, quat_wp = self._wrapper.get_world_poses()
        except Exception as exc:
            carb.log_warn(f"[ArticulationRecordable {self.prim_path}] get_world_poses failed: {exc}")
            return self._zeros_frame()
        positions = to_numpy_f32(pos_wp).reshape(self.num_links, 3)
        orientations = to_numpy_f32(quat_wp).reshape(self.num_links, 4)
        return {"positions": positions, "orientations": orientations}

    def apply(self, frame: Mapping[str, np.ndarray], *, policy: ReplayPolicy) -> None:
        """Apply one recorded frame.

        Args:
            frame: Frame data keyed by channel name.
            policy: Replay policy controlling error handling.
        """
        if self._wrapper is None:
            if policy.strictness == "strict":
                raise RuntimeError(f"ArticulationRecordable {self.prim_path}: not bound.")
            return
        try:
            positions = np.asarray(frame["positions"], dtype=np.float32).reshape(self.num_links, 3)
            orientations = np.asarray(frame["orientations"], dtype=np.float32).reshape(self.num_links, 4)
        except (KeyError, ValueError) as exc:
            if policy.strictness == "strict":
                raise
            carb.log_warn(f"[ArticulationRecordable {self.prim_path}] malformed frame: {exc}")
            return

        def _write() -> None:
            self._wrapper.set_world_poses(positions=positions, orientations=orientations)

        try:
            _write()
            return
        except Exception as exc:
            if self._xform_ops_reset or not is_missing_xform_ops_error(exc):
                if policy.strictness == "strict":
                    raise
                carb.log_warn(f"[ArticulationRecordable {self.prim_path}] apply failed: {exc}")
                return
            try:
                self._wrapper.reset_xform_op_properties()
                self._xform_ops_reset = True
                _write()
            except Exception as retry_exc:
                if policy.strictness == "strict":
                    raise
                carb.log_warn(
                    f"[ArticulationRecordable {self.prim_path}] apply failed after reset_xform_op_properties: {retry_exc}"
                )

    def to_manifest(self) -> dict[str, Any]:
        """Serialize this object to a manifest entry.

        Returns:
            JSON-friendly manifest entry.
        """
        return {
            "type": self.TYPE_ID,
            "group": self.group,
            "prim_path": self.prim_path,
            "include_root": self.include_root,
            "link_paths": list(self._link_paths),
        }

    @classmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> ArticulationRecordable:
        """Create an instance from a manifest entry.

        Args:
            entry: Manifest entry used to reconstruct the recordable.

        Returns:
            Recordable reconstructed from the manifest entry.
        """
        return cls(
            group=entry["group"],
            prim_path=entry["prim_path"],
            link_paths=list(entry.get("link_paths", [])),
            include_root=bool(entry.get("include_root", True)),
        )
