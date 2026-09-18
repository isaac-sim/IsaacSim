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

"""Drop-line visual cues for 2D teleoperation depth perception.

Each side renders one vertical cylinder from a tracked world position down
to a configurable reference *Z*.
Prims live in an anonymous session sublayer under ``/Teleop/VisualCues/``
and are never saved with the stage or recorded by the episode recorder.
"""

from __future__ import annotations

from collections.abc import Callable, Generator
from contextlib import contextmanager
from typing import Literal

import carb
import carb.eventdispatcher
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from ._xform_utils import WorldPosePrimCache, read_world_pose_arrays

SideName = Literal["left", "right"]
PoseProvider = Callable[[SideName], tuple[float, float, float] | None]


class VisualCuesManager:
    """Manages per-side drop-line visual cues in the anonymous teleop session layer."""

    CUES_SCOPE = "/Teleop/VisualCues"
    SIDE_PATHS: dict[SideName, str] = {
        "left": f"{CUES_SCOPE}/Left",
        "right": f"{CUES_SCOPE}/Right",
    }
    CYLINDER_CHILD = "Cylinder"
    MATERIAL_CHILD = "Material"

    DEFAULT_REFERENCE_Z = 0.0
    DEFAULT_SIZE = 0.025
    DEFAULT_OPACITY = 0.5
    MIN_SIZE = 0.001
    MIN_OPACITY = 0.05
    MAX_OPACITY = 1.0

    _CYLINDER_RADIUS_FACTOR = 0.12
    _SIDE_COLORS: dict[SideName, tuple[float, float, float]] = {
        "left": (0.2, 0.75, 0.95),
        "right": (0.95, 0.55, 0.15),
    }

    def __init__(self) -> None:
        self._reference_z: float = self.DEFAULT_REFERENCE_Z
        self._size: float = self.DEFAULT_SIZE
        self._opacity: float = self.DEFAULT_OPACITY
        self._override_paths: dict[SideName, str] = {"left": "", "right": ""}
        self._active_sides: set[SideName] = set()
        self._layer: Sdf.Layer | None = None
        self._override_caches: dict[str, WorldPosePrimCache] = {}
        self._pose_provider: PoseProvider | None = None
        self._update_subscription = None
        self._last_status: dict[SideName, str] = {"left": "", "right": ""}

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    @property
    def reference_z(self) -> float:
        """World *Z* height where cue cylinders end."""
        return self._reference_z

    @property
    def size(self) -> float:
        """Uniform scale used to derive the cue-cylinder radius."""
        return self._size

    @property
    def opacity(self) -> float:
        """Display opacity applied to all cue geometry."""
        return self._opacity

    def set_pose_provider(self, provider: PoseProvider | None) -> None:
        """Register a callback that returns auto-linked controller input positions."""
        self._pose_provider = provider

    def set_reference_z(self, value: float) -> None:
        """Set the world *Z* plane used as the drop-line floor."""
        self._reference_z = float(value)

    def set_size(self, value: float) -> None:
        """Set cue visual scale and refresh active geometry."""
        self._size = max(self.MIN_SIZE, float(value))
        self._refresh_active_appearance()

    def set_opacity(self, value: float) -> None:
        """Set cue opacity and refresh active geometry."""
        self._opacity = max(self.MIN_OPACITY, min(self.MAX_OPACITY, float(value)))
        self._refresh_active_appearance()

    def get_override_prim_path(self, side: SideName) -> str:
        """Return the manual prim override for a side (empty = auto-link)."""
        return self._override_paths[side]

    def set_override_prim_path(self, side: SideName, path: str) -> None:
        """Set or clear the manual prim override for a side."""
        side = side.lower()  # type: ignore[assignment]
        normalized = path.strip()
        self._override_paths[side] = normalized
        if normalized:
            self._override_caches[normalized] = WorldPosePrimCache(normalized)
        self._last_status[side] = ""

    def is_side_active(self, side: SideName) -> bool:
        """True if drop-line geometry exists for the given side."""
        return side in self._active_sides

    def get_side_status(self, side: SideName) -> str:
        """Return the last resolved status message for a side."""
        return self._last_status.get(side, "")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def show_side(self, side: SideName) -> tuple[bool, str]:
        """Create drop-line geometry for one side."""
        side = side.lower()  # type: ignore[assignment]
        if not stage_utils.is_stage_set() and omni.usd.get_context().get_stage() is None:
            return False, "No USD stage available"
        stage = stage_utils.get_current_stage()
        if side in self._active_sides and self._layer_is_attached(stage):
            return True, f"{side.capitalize()} drop line active"
        if self._layer is not None and not self._layer_is_attached(stage):
            self._invalidate_stale_layer()

        layer = self._ensure_layer(stage)
        side_path = self.SIDE_PATHS[side]
        color = self._SIDE_COLORS[side]

        with Usd.EditContext(stage, layer):
            stage_utils.define_prim(side_path, "Xform")
            self._ensure_side_material(stage, side_path, color)
            self._create_cylinder_child(stage, f"{side_path}/{self.CYLINDER_CHILD}", side_path)

        self._active_sides.add(side)
        self._ensure_update_subscription()
        self._apply_appearance(side)
        self._update_side_geometry(side)
        print(f"[Teleop][VisualCues] Show {side} drop line at '{side_path}'.")
        return True, f"{side.capitalize()} drop line active"

    def hide_side(self, side: SideName) -> bool:
        """Remove drop-line geometry for one side."""
        side = side.lower()  # type: ignore[assignment]
        if side not in self._active_sides:
            return True

        side_path = self.SIDE_PATHS[side]
        if self._layer is not None:
            spec = self._layer.GetPrimAtPath(side_path)
            if spec and spec.nameParent:
                del spec.nameParent.nameChildren[spec.name]

        self._active_sides.discard(side)
        self._last_status[side] = ""
        if not self._active_sides:
            self._release_update_subscription()
        print(f"[Teleop][VisualCues] Hide {side} drop line.")
        return True

    def hide_all(self) -> None:
        """Remove all drop-line geometry and drop the anonymous layer."""
        self._clear_teleop_selection()
        self._active_sides.clear()
        self._last_status = {"left": "", "right": ""}
        self._release_update_subscription()
        self._remove_layer()
        print("[Teleop][VisualCues] Removed all drop lines (layer dropped).")

    def clear_cached_state(self) -> None:
        """Tear down cues when the USD stage closes."""
        self._override_caches.clear()
        self.hide_all()

    # ------------------------------------------------------------------
    # Anonymous session sublayer
    # ------------------------------------------------------------------

    def _ensure_layer(self, stage: Usd.Stage) -> Sdf.Layer:
        if self._layer is not None and self._layer_is_attached(stage):
            return self._layer
        if self._layer is not None:
            self._invalidate_stale_layer()

        self._layer = Sdf.Layer.CreateAnonymous("anon_teleop_visual_cues")
        session = stage.GetSessionLayer()
        session.subLayerPaths.append(self._layer.identifier)
        print(f"[Teleop][VisualCues] Created anonymous visual-cues layer: {self._layer.identifier}")
        return self._layer

    def _remove_layer(self) -> None:
        if self._layer is None:
            return

        if stage_utils.is_stage_set() or omni.usd.get_context().get_stage() is not None:
            stage = stage_utils.get_current_stage()
            session = stage.GetSessionLayer()
            ident = self._layer.identifier
            if ident in session.subLayerPaths:
                session.subLayerPaths.remove(ident)

        self._layer = None

    def _layer_is_attached(self, stage: Usd.Stage) -> bool:
        """Return whether the manager's layer belongs to ``stage``'s local layer stack."""
        if self._layer is None:
            return False
        identifier = self._layer.identifier
        return any(layer.identifier == identifier for layer in stage.GetLayerStack(includeSessionLayers=True))

    def _invalidate_stale_layer(self) -> None:
        """Release stage-bound state after the owning stage has been replaced."""
        self._active_sides.clear()
        self._last_status = {"left": "", "right": ""}
        self._override_caches.clear()
        self._release_update_subscription()
        self._layer = None

    @contextmanager
    def _edit_ctx(self) -> Generator[Usd.Stage | None, None, None]:
        stage = (
            stage_utils.get_current_stage()
            if stage_utils.is_stage_set() or omni.usd.get_context().get_stage() is not None
            else None
        )
        if stage is not None and self._layer is not None and self._layer_is_attached(stage):
            with Usd.EditContext(stage, self._layer):
                yield stage
            return
        if self._layer is not None:
            self._invalidate_stale_layer()
        yield None

    # ------------------------------------------------------------------
    # Per-frame update
    # ------------------------------------------------------------------

    def _ensure_update_subscription(self) -> None:
        if self._update_subscription is not None:
            return
        self._update_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_update,
            observer_name="isaacsim.replicator.teleop.VisualCuesManager.update",
        )

    def _release_update_subscription(self) -> None:
        sub = self._update_subscription
        if sub is None:
            return
        sub.reset()
        self._update_subscription = None

    def _on_update(self, _event: object) -> None:
        if not self._active_sides:
            return
        try:
            stage = stage_utils.get_current_stage()
        except ValueError:
            self._invalidate_stale_layer()
            return
        if not self._layer_is_attached(stage):
            self._invalidate_stale_layer()
            return
        for side in list(self._active_sides):
            self._update_side_geometry(side)

    def _resolve_world_position(self, side: SideName) -> tuple[tuple[float, float, float] | None, str]:
        override = self._override_paths[side].strip()
        if override:
            if not stage_utils.is_stage_set() and omni.usd.get_context().get_stage() is None:
                return None, "No stage"
            stage = stage_utils.get_current_stage()
            prim = stage.GetPrimAtPath(override)
            if not prim or not prim.IsValid():
                return None, f"Override prim not found: {override}"
            if not prim.IsA(UsdGeom.Xformable):
                return None, f"Override prim not Xformable: {override}"
            cache = self._override_caches.get(override)
            if cache is None:
                cache = WorldPosePrimCache(override)
                self._override_caches[override] = cache
            pos_arr, _ = read_world_pose_arrays(cache)
            pos = pos_arr.reshape(-1, 3)[0]
            return (float(pos[0]), float(pos[1]), float(pos[2])), f"Override — {override}"

        if self._pose_provider is None:
            return None, "No controller input"
        pos = self._pose_provider(side)
        if pos is None:
            return None, "No controller input"
        return pos, "Auto — controller input"

    def _update_side_geometry(self, side: SideName) -> None:
        position, status = self._resolve_world_position(side)
        self._last_status[side] = status
        if position is None:
            self._set_side_visible(side, False)
            return

        x, y, z_top = position
        z_ref = self._reference_z
        if z_top <= z_ref:
            self._last_status[side] = f"{status} (below Reference Z)"
            self._set_side_visible(side, False)
            return

        cylinder_height = z_top - z_ref
        cylinder_radius = max(self._size * self._CYLINDER_RADIUS_FACTOR, 0.001)

        side_path = self.SIDE_PATHS[side]
        cylinder_path = f"{side_path}/{self.CYLINDER_CHILD}"

        with self._edit_ctx() as stage:
            if stage is None:
                return
            self._set_side_anchor(stage, side_path, x, y, z_top)
            self._set_cylinder_local(
                stage,
                cylinder_path,
                -cylinder_height * 0.5,
                cylinder_radius,
                cylinder_height,
            )
            self._set_side_visible(side, True, stage=stage)

    @staticmethod
    def _set_side_anchor(stage: Usd.Stage, side_path: str, x: float, y: float, z_top: float) -> None:
        """Place the side root Xform at the tracked controller world position."""
        prim = stage.GetPrimAtPath(side_path)
        if not prim or not prim.IsValid():
            return
        xformable = UsdGeom.Xformable(prim)
        xform_ops = xformable.GetOrderedXformOps()
        if not xform_ops:
            xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z_top))
        else:
            xform_ops[0].Set(Gf.Vec3d(x, y, z_top))

    @staticmethod
    def _set_cylinder_local(
        stage: Usd.Stage,
        path: str,
        local_z: float,
        radius: float,
        height: float,
    ) -> None:
        """Update cylinder size and local offset relative to the side anchor."""
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return
        cylinder = UsdGeom.Cylinder(prim)
        cylinder.GetRadiusAttr().Set(radius)
        cylinder.GetHeightAttr().Set(height)
        cylinder.GetAxisAttr().Set(UsdGeom.Tokens.z)

        xformable = UsdGeom.Xformable(prim)
        xform_ops = xformable.GetOrderedXformOps()
        if not xform_ops:
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, local_z))
        else:
            xform_ops[0].Set(Gf.Vec3d(0.0, 0.0, local_z))

    def _set_side_visible(self, side: SideName, visible: bool, stage: Usd.Stage | None = None) -> None:
        if stage is None:
            with self._edit_ctx() as edit_stage:
                if edit_stage is not None:
                    self._set_side_visible(side, visible, stage=edit_stage)
            return
        side_path = self.SIDE_PATHS[side]
        token = UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible
        for path in (side_path, f"{side_path}/{self.CYLINDER_CHILD}"):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                UsdGeom.Imageable(prim).GetVisibilityAttr().Set(token)

    # ------------------------------------------------------------------
    # Appearance
    # ------------------------------------------------------------------

    def _ensure_side_material(
        self,
        stage: Usd.Stage,
        side_path: str,
        color: tuple[float, float, float],
    ) -> UsdShade.Material:
        """Create or return the preview-surface material for one side."""
        material_path = f"{side_path}/{self.MATERIAL_CHILD}"
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(self._opacity)
        shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    def _create_cylinder_child(self, stage: Usd.Stage, path: str, side_path: str) -> None:
        cylinder = UsdGeom.Cylinder.Define(stage, path)
        cylinder.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        cylinder.CreateRadiusAttr().Set(self._size * self._CYLINDER_RADIUS_FACTOR)
        cylinder.CreateHeightAttr().Set(self._size)
        gprim = UsdGeom.Gprim(cylinder.GetPrim())
        gprim.CreateDoubleSidedAttr().Set(False)
        UsdGeom.PrimvarsAPI(cylinder.GetPrim()).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        material = UsdShade.Material(stage.GetPrimAtPath(f"{side_path}/{self.MATERIAL_CHILD}"))
        UsdShade.MaterialBindingAPI.Apply(cylinder.GetPrim()).Bind(material)

    def _apply_appearance(self, side: SideName) -> None:
        side_path = self.SIDE_PATHS[side]
        color = self._SIDE_COLORS[side]
        with self._edit_ctx() as stage:
            if stage is None:
                return
            material_path = f"{side_path}/{self.MATERIAL_CHILD}"
            shader_path = f"{material_path}/PreviewSurface"
            shader_prim = stage.GetPrimAtPath(shader_path)
            if shader_prim and shader_prim.IsValid():
                shader = UsdShade.Shader(shader_prim)
                shader.GetInput("emissiveColor").Set(Gf.Vec3f(*color))
                shader.GetInput("opacity").Set(self._opacity)
            prim = stage.GetPrimAtPath(f"{side_path}/{self.CYLINDER_CHILD}")
            if prim and prim.IsValid():
                gprim = UsdGeom.Gprim(prim)
                gprim.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
                gprim.GetDisplayOpacityAttr().Set([self._opacity])

    def _refresh_active_appearance(self) -> None:
        for side in list(self._active_sides):
            self._apply_appearance(side)

    @staticmethod
    def _clear_teleop_selection() -> None:
        usd_ctx = omni.usd.get_context()
        if not usd_ctx:
            return
        selection = usd_ctx.get_selection()
        if not selection:
            return
        selected = selection.get_selected_prim_paths()
        if any(p.startswith("/Teleop") for p in selected):
            selection.clear_selected_prim_paths()
