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

"""Runtime manager: discover IsaacVirtualGantry prims and drive one rope each."""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import replace
from typing import Any

from .virtual_gantry import (
    _ATTR_BODY_OFFSET,
    _ATTR_DAMPING,
    _ATTR_EMA_ALPHA,
    _ATTR_ENABLED,
    _ATTR_MIN_ROPE_LENGTH,
    _ATTR_ROPE_LENGTH,
    _ATTR_STIFFNESS,
    _ATTR_VISUALIZE,
    _REL_ATTACH_BODY,
    GANTRY_PRIM_TYPE,
    VirtualGantry,
    _first_target,
    _get_attr,
    anchor_from_prim,
    articulation_path_from_prim,
    config_from_prim,
    write_status_to_prim,
)

_LOGGER = logging.getLogger(__name__)

# The manager owned by the running backend extension, or None when it is not
# started. Kept here (rather than in extension.py) so it stays importable
# without Kit, and exposed so the companion
# ``isaacsim.robot_setup.virtual_gantry.ui`` extension can drive the live
# ropes from its hotkeys without the backend depending on any UI module.
_ACTIVE_MANAGER: "VirtualGantryManager | None" = None


def get_manager() -> "VirtualGantryManager | None":
    """Return the live gantry manager, or None if the extension isn't running.

        Resolve this lazily at call time rather than caching it: the UI
        extension can start before the backend, in which case an early lookup
        would latch None forever.

    Returns:
        The value, or ``None`` when unavailable.
    """
    return _ACTIVE_MANAGER


def _set_active_manager(manager: "VirtualGantryManager | None") -> None:
    """Publish (or clear) the manager. Called by the backend extension only.

    Args:
        manager: The manager to publish, or ``None`` to clear it.
    """
    global _ACTIVE_MANAGER
    _ACTIVE_MANAGER = manager


class VirtualGantryManager:
    """Owns one :class:`VirtualGantry` per ``IsaacVirtualGantry`` prim on the stage.

    The managed set is re-scanned periodically (see ``_SYNC_EVERY``), so gantries
    created from the Create menu mid-run are picked up live (and deleted ones
    dropped) without a Stop/Play, and dropped on :meth:`reset` (timeline Stop /
    stage change) so the next play rebuilds against fresh articulation views.
    """

    _SYNC_EVERY = 15  # re-scan the stage for gantry prims every N physics steps

    def __init__(self) -> None:
        self._gantries: dict[str, tuple[VirtualGantry, Any]] = {}
        self._last_rope: dict[str, float] = {}  # last authored ropeLength per prim
        self._tick = 0

    def reset(self, clear_overlay: bool = True) -> None:
        """Disable and drop all managed gantries; forces a rebuild next step.

                ``clear_overlay`` must be False when resetting from a stage close /
                extension shutdown: clearing the debug_draw overlay while the stage and
                renderer are tearing down segfaults. Pass True only on timeline Stop,
                where the stage is still alive.

        Args:
            clear_overlay: Whether to erase the debug overlay as well.
        """
        for gantry, _prim in self._gantries.values():
            try:
                gantry.disable(clear_drawing=clear_overlay)
            except Exception:  # noqa: BLE001 — teardown is best-effort.
                pass
        if clear_overlay and self._gantries:
            self._clear_overlay()
        self._gantries = {}
        self._last_rope = {}
        self._tick = 0

    def step(self, dt: float) -> None:
        """Sync enabled-state from each prim, advance its rope, write back status.

        Args:
            dt: Physics timestep in seconds.
        """
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        # Re-scan on the first step and periodically thereafter so menu-created
        # gantries activate mid-run (once wired) without a Stop/Play.
        self._tick += 1
        if self._tick == 1 or self._tick % self._SYNC_EVERY == 0:
            self._sync(stage)

        # The rope/anchor overlay is drawn with debug_draw, whose clear is
        # global, so it must be cleared once here (not per gantry) and then
        # each enabled gantry re-draws its own line below.
        if any(g.wants_visualization() for g, _ in self._gantries.values()):
            self._clear_overlay()

        for path, (gantry, prim) in list(self._gantries.items()):
            if not prim.IsValid():
                continue
            enabled = bool(_get_attr(prim, _ATTR_ENABLED, True))
            if enabled and not gantry.is_enabled():
                gantry.enable()
                self._reanchor(gantry, prim)
            elif not enabled and gantry.is_enabled():
                # clear_drawing=False: the debug_draw clear is global, and this
                # runs inside the per-gantry loop, after earlier-iterated
                # gantries have already drawn their rope lines for this frame.
                # The one clear this step needs already happened above.
                gantry.disable(clear_drawing=False)
            # Apply the authored rope length only when it actually changes
            # (a property-panel edit), not every step: forcing it continuously
            # would clobber the transient [ ] hotkey adjustments, which mutate
            # the running rope without writing the prim. A negative value means
            # "auto" (derived on enable).
            rope = _get_attr(prim, _ATTR_ROPE_LENGTH, -1.0)
            rope = float(rope) if rope is not None else -1.0
            if rope != self._last_rope.get(path):
                self._last_rope[path] = rope
                gantry.set_configured_rope_length(rope)
            # Live-sync the tunable gains/params so property-panel edits apply
            # immediately to the running rope.
            gantry.update_tunables(
                kp=_get_attr(prim, _ATTR_STIFFNESS, None),
                kd=_get_attr(prim, _ATTR_DAMPING, None),
                ema_alpha=_get_attr(prim, _ATTR_EMA_ALPHA, None),
                min_rope_length=_get_attr(prim, _ATTR_MIN_ROPE_LENGTH, None),
                body_offset=_get_attr(prim, _ATTR_BODY_OFFSET, None),
                visualize=_get_attr(prim, _ATTR_VISUALIZE, None),
            )
            try:
                cmd = gantry.step(float(dt))
            except Exception as exc:  # noqa: BLE001 — never let one rope kill the bus.
                _LOGGER.warning(f"[virtual-gantry] step failed for {path}: {exc!r}")
                continue
            if not gantry.is_enabled():
                status = "Disabled"
            elif cmd.applied:
                status = "Taut"
            else:
                status = "Slack"
            write_status_to_prim(prim, status)

    def toggle_all(self) -> None:
        """Flip the ``enabled`` attribute on every managed gantry (the ``G`` hotkey).

        Only flips the prim attribute; the per-step enabled-sync in :meth:`step`
        performs the actual enable/disable + re-anchor in physics-step context,
        where the articulation view read is valid. (Doing it directly here, on the
        keyboard/UI thread, makes the re-anchor's link-transform read unreliable.)
        """
        for _path, (_gantry, prim) in self._gantries.items():
            attr = prim.GetAttribute(_ATTR_ENABLED)
            if attr is not None and attr.IsValid():
                attr.Set(not bool(attr.Get()))

    def adjust_rope_length_all(self, delta: float) -> None:
        """Lengthen/shorten every managed gantry's rope live (the ``[`` / ``]`` hotkeys).

                Transient: the adjustment is NOT written back to the prim's ``ropeLength``,
                so a disable -> enable re-derives the rope (just-taut at the current pose,
                or the authored ``ropeLength``) and catches the robot in place rather than
                restoring the last adjusted height. Mirrors the deploy gantry.

        Args:
            delta: Signed length change in meters.
        """
        for _path, (gantry, _prim) in self._gantries.items():
            gantry.adjust_rope_length(float(delta))

    @staticmethod
    def _reanchor(gantry: VirtualGantry, prim: Any) -> None:
        """Move the anchor prim above the attach body's current x/y (keep its z).

                Mirrors the deploy gantry: every enable re-anchors over the body where it
                is now, so a disable -> fall -> enable catches the robot in place instead
                of yanking it back to the prim's original spot.

                ``attach_world_position`` is in world space while the translate op is in
                the prim's parent space, so the target is converted through the parent's
                inverse (as :func:`~.create._set_anchor_world` does). Without that, a
                gantry parented under anything with a non-identity transform re-anchors
                to the wrong world spot on every enable.

        Args:
            gantry: The live gantry being re-anchored.
            prim: The gantry prim to read from.
        """
        pos = gantry.attach_world_position()
        if pos is None:
            return
        from pxr import Gf, Usd, UsdGeom

        xform = UsdGeom.Xformable(prim)
        op = next(
            (o for o in xform.GetOrderedXformOps() if o.GetOpType() == UsdGeom.XformOp.TypeTranslate),
            None,
        )
        if op is None:
            op = xform.AddTranslateOp()
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        # Keep the anchor's current world height; only track the body in x/y.
        cur_world = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        target_world = Gf.Vec3d(float(pos[0]), float(pos[1]), float(cur_world[2]))
        parent = prim.GetParent()
        if parent is not None and parent.IsValid():
            target_world = cache.GetLocalToWorldTransform(parent).GetInverse().Transform(target_world)
        op.Set(Gf.Vec3d(target_world))

    def _sync(self, stage: Any) -> None:
        """Add newly created/wired gantry prims, drop deleted ones.

        Args:
            stage: The USD stage to operate on.
        """
        found = {prim.GetPath().pathString: prim for prim in stage.Traverse() if prim.GetTypeName() == GANTRY_PRIM_TYPE}
        # Drop gantries whose prim was deleted.
        for path in list(self._gantries):
            if path not in found:
                gantry, _prim = self._gantries.pop(path)
                self._last_rope.pop(path, None)
                try:
                    gantry.disable()
                except Exception:  # noqa: BLE001
                    pass
        # Add new prims. An unwired prim (no resolvable articulation) is skipped
        # silently and retried on a later sync, once the user sets its relationships.
        for path, prim in found.items():
            if path in self._gantries:
                continue
            try:
                articulation = self._make_articulation(stage, prim)
                if articulation is None:
                    continue
                # Build disabled; the per-step enabled-sync performs enable +
                # re-anchor uniformly (whether initial, mid-run, or re-enable).
                gantry = VirtualGantry(
                    articulation,
                    replace(config_from_prim(prim), enabled=False),
                    anchor_provider=self._make_anchor_provider(prim),
                )
                self._gantries[path] = (gantry, prim)
                _LOGGER.info(f"[virtual-gantry] managing {path}")
            except Exception as exc:  # noqa: BLE001 — skip a broken prim, keep the rest.
                _LOGGER.warning(f"[virtual-gantry] failed to build {path}: {exc!r}")

    @staticmethod
    def _clear_overlay() -> None:
        """Clear the global debug-draw overlay (rope lines + anchor markers)."""
        try:
            from isaacsim.util.debug_draw import _debug_draw

            draw = _debug_draw.acquire_debug_draw_interface()
            draw.clear_lines()
            draw.clear_points()
        except Exception:  # noqa: BLE001 — visualization is best-effort only.
            pass

    @staticmethod
    def _make_anchor_provider(prim: Any) -> Callable[[], Any]:
        def _provider() -> Any:
            return anchor_from_prim(prim)

        return _provider

    @staticmethod
    def _make_articulation(stage: Any, prim: Any) -> Any:
        from isaacsim.core.experimental.prims import Articulation
        from pxr import UsdPhysics

        art_path = articulation_path_from_prim(prim)
        if not art_path:
            # Fall back to the first ArticulationRoot ancestor of the attach body.
            target = _first_target(prim, _REL_ATTACH_BODY)
            cur = stage.GetPrimAtPath(target) if target is not None else None
            while cur is not None and cur.IsValid() and cur.GetPath().pathString != "/":
                if cur.HasAPI(UsdPhysics.ArticulationRootAPI):
                    art_path = cur.GetPath().pathString
                    break
                cur = cur.GetParent()
        if not art_path:
            return None
        return Articulation(art_path)
