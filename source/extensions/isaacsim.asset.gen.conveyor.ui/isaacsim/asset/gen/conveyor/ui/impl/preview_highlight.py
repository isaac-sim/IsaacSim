"""Viewport selection-group helper used to tint the conveyor preview ghost geometry.

Replaces the previous translucent MDL "Ghost" material binding. The selection-group
approach mirrors the pattern used by `isaacsim.robot.poser.ui.utils.fk_helpers` and
`isaacsim.robot_setup.collision_detector.widget`: register a viewport selection group
once, configure its outline + shade colours, and assign the preview gprims to it. The
group survives reference-target changes on the preview prim because `apply_to_subtree`
re-collects gprims and re-assigns them on demand.
"""

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

__all__ = ["ConveyorPreviewHighlight"]

import carb
import omni.usd
from pxr import Usd, UsdGeom

# Green tint matching the request. Outline alpha is 1.0 for visibility, shade alpha is
# kept low so the underlying geometry remains readable when the preview overlaps with
# the actual stage.
_OUTLINE_RGBA: tuple[float, float, float, float] = (0.1, 1.0, 0.3, 1.0)
_SHADE_RGBA: tuple[float, float, float, float] = (0.1, 1.0, 0.3, 0.25)


class ConveyorPreviewHighlight:
    """Manages a single viewport selection group used to tint the preview prim subtree.

    Lifetime: a `ConveyorPreviewHighlight` instance owns one group id allocated via
    `omni.usd.UsdContext.register_selection_group` on first `apply_to_subtree` call.
    The group id persists across reference-target swaps on the same preview prim and is
    only released when `shutdown` is called (typically from the widget shutdown path).
    """

    def __init__(self) -> None:
        self._group_id: int | None = None
        self._assigned_paths: set[str] = set()

    def apply_to_subtree(self, stage: Usd.Stage, root_prim: Usd.Prim) -> None:
        """Assign every Gprim under `root_prim` to the highlight group.

        Re-applies on each call because Kit reference-target swaps change which
        descendant prims exist; previously assigned paths that no longer exist are
        dropped from the tracked set.

        Args:
            stage: The USD stage owning the preview prim. May be ``None`` (no-op).
            root_prim: Subtree root whose Gprim descendants receive the highlight.
        """
        if stage is None or not root_prim.IsValid():
            self.clear()
            return

        gid = self._ensure_group()
        if gid == 0:
            self.clear()
            return

        ctx = omni.usd.get_context()
        desired: set[str] = set()
        for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if prim.IsA(UsdGeom.Gprim):
                desired.add(str(prim.GetPath()))

        # Detach prims that left the subtree (e.g. previous reference target gprims).
        for path in self._assigned_paths - desired:
            ctx.set_selection_group(0, path)
        # Re-attach every desired prim, not just (desired - assigned). Kit's
        # `set_selected_prim_paths` resets a prim's selection group to 0 on selection
        # changes, so over-asserting on every call is necessary to keep the highlight
        # stable across the selection events we react to.
        for path in desired:
            ctx.set_selection_group(gid, path)

        self._assigned_paths = desired

    def clear(self) -> None:
        """Move all currently highlighted prims back to the default selection group."""
        if not self._assigned_paths:
            return
        ctx = omni.usd.get_context()
        for path in self._assigned_paths:
            ctx.set_selection_group(0, path)
        self._assigned_paths.clear()

    def shutdown(self) -> None:
        """Reset and release the owned group, restoring default colours."""
        self.clear()
        if self._group_id is not None and self._group_id != 0:
            ctx = omni.usd.get_context()
            # Kit does not expose `unregister_selection_group`; the group id is leaked
            # for the process lifetime. Resetting the colours keeps the leaked group
            # invisible if it is ever re-bound.
            ctx.set_selection_group_outline_color(self._group_id, (1.0, 1.0, 1.0, 0.0))
            ctx.set_selection_group_shade_color(self._group_id, (0.0, 0.0, 0.0, 0.0))
        self._group_id = None

    def _ensure_group(self) -> int:
        """Lazily register the selection group and configure its green colours."""
        if self._group_id is not None:
            return self._group_id
        ctx = omni.usd.get_context()
        gid = ctx.register_selection_group()
        if gid == 0:
            carb.log_error(
                "ConveyorPreviewHighlight: register_selection_group returned 0; "
                "preview tint disabled (default group must not be reused)"
            )
            self._group_id = 0
            return 0
        ctx.set_selection_group_outline_color(gid, _OUTLINE_RGBA)
        ctx.set_selection_group_shade_color(gid, _SHADE_RGBA)
        self._group_id = gid
        return gid
