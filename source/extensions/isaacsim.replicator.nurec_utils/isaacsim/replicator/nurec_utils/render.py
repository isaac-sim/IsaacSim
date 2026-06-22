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

"""Rendering primitives for NuRec USDs, with or without PPISP.

`RenderTargetFactory` acquires a camera's render product (the NuRec USD's authored one for
an SPG/PPISP stage, or one created on the asset's camera for a plain stage); `CameraRenderer`
renders frames from it at rig keyframes or explicit poses. Reads the `LdrColor` output.

Imports `omni`/`carb`/`pxr` at load time, so a `SimulationApp` must exist before importing it.
"""

from __future__ import annotations

import os
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass
from typing import Any

import carb
import numpy as np
import omni.replicator.core as rep
import omni.timeline
from pxr import Sdf, Usd

from .manifest import write_manifest
from .metrics.scoped_timer import ScopedTimer
from .rendering_setup import setup_for_rendering
from .usd_utils import (
    KeyframeIndex,
    camera_world_position,
    open_stage,
    rig_keyframe_time_codes,
    save_image,
    stage_time_params,
)

RENDER_SCOPE_PATH = "/Render"

# Identity (pass-through) camera exposure for an SPG/PPISP render.
_IDENTITY_EXPOSURE = {
    "exposure": 0.0,
    "exposure:fStop": 1.0,
    "exposure:iso": 0.0,
    "exposure:responsivity": 1.0,
    "exposure:time": 1.0,
}


def _timer(timing_label: str | None, name: str) -> AbstractContextManager[Any]:
    """Return a scoped timer when a timing label was requested, else a no-op context.

    Args:
        timing_label: The run label that enables timing, or None to disable it.
        name: The timer name, suffixed onto `timing_label`.

    Returns:
        A `ScopedTimer` when `timing_label` is set, else a no-op context manager.
    """
    return ScopedTimer(f"{timing_label}.{name}") if timing_label else nullcontext()


def _log_timer(timing_label: str | None, label: str, timer: Any) -> None:
    if timing_label and timer is not None and timer.elapsed_time_ms is not None:
        carb.log_warn(f"[nurec timing] {timing_label} {label}: {timer.elapsed_time_ms:.3f} ms")


def discover_render_products(stage: Usd.Stage) -> dict[str, str]:
    """Return `{camera_logical_name: render_product_prim_path}` for the authored RPs.

    These NuRec USDs author one RenderProduct per camera under `/Render/<cam_name>`.

    Args:
        stage: The open USD stage.

    Returns:
        Mapping of camera logical name to its RenderProduct prim path.
    """
    render_scope = stage.GetPrimAtPath(RENDER_SCOPE_PATH)
    if not render_scope.IsValid():
        return {}
    out: dict[str, str] = {}
    for child in render_scope.GetChildren():
        if child.GetTypeName() == "RenderProduct":
            out[child.GetName()] = str(child.GetPath())
    return out


def discover_cameras(stage: Usd.Stage) -> dict[str, str]:
    """Return `{camera_name: camera_prim_path}` for the `Camera` prims on the stage.

    Used to render a plain (non-SPG) NuRec USD that has no authored RenderProducts.

    Args:
        stage: The open USD stage.

    Returns:
        Mapping of camera prim name to its prim path.
    """
    out: dict[str, str] = {}
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Camera":
            out[prim.GetName()] = str(prim.GetPath())
    return out


def copy_render_product(
    stage: Usd.Stage,
    src_rp_path: str,
    dst_rp_path: str,
    camera_path: str,
    *,
    package_path: str | None = None,
) -> tuple[int, int]:
    """Copy an authored RenderProduct to ``dst_rp_path`` in the session layer, rebased onto a camera.

    The copy renders ``camera_path`` through the same graph; any packaged asset references (e.g. SPG/PPISP
    controllers) are package-qualified (``<package>[member]``) so they resolve from the session layer.

    Args:
        stage: The open USD stage.
        src_rp_path: The authored RenderProduct to clone.
        dst_rp_path: The path for the cloned RenderProduct (created in the session layer).
        camera_path: The camera the clone renders.
        package_path: The package the asset members live in (``<package>[member]``). Defaults to the
            stage's root layer identifier (the opened ``.usdz``).

    Returns:
        The clone's ``(width, height)`` resolution.
    """
    session = stage.GetSessionLayer()
    src, dst = Sdf.Path(src_rp_path), Sdf.Path(dst_rp_path)
    if session.GetPrimAtPath(dst):
        with Usd.EditContext(stage, session):
            stage.RemovePrim(dst)
    src_prim = stage.GetPrimAtPath(src_rp_path)
    if not src_prim.IsValid():
        raise ValueError(f"copy_render_product: no RenderProduct prim at {src_rp_path!r}")
    prim_stack = src_prim.GetPrimStack()
    if not prim_stack:
        raise ValueError(f"copy_render_product: empty prim stack for {src_rp_path!r}")
    src_layer = prim_stack[0].layer
    resolution = src_prim.GetAttribute("resolution").Get()
    if package_path is None:
        package_path = stage.GetRootLayer().identifier
    # Ensure the destination's ancestors (e.g. /Render) exist in the session layer so CopySpec can
    # register the copy in the parent's children.
    Sdf.CreatePrimInLayer(session, dst)
    Sdf.CopySpec(src_layer, src, session, dst)

    def _package_qualified(asset_path: str) -> str:
        return f"{package_path}[{os.path.basename(asset_path)}]"

    with Usd.EditContext(stage, session):
        dst_prim = stage.GetPrimAtPath(dst_rp_path)
        dst_prim.CreateAttribute("resolution", Sdf.ValueTypeNames.Int2).Set(resolution)
        for prim in Usd.PrimRange(dst_prim):
            for attr in prim.GetAttributes():
                conns = attr.GetConnections()
                remapped = [c.ReplacePrefix(src, dst) for c in conns]
                if remapped != conns:
                    attr.SetConnections(remapped)
            for rel in prim.GetRelationships():
                targets = rel.GetTargets()
                remapped = [t.ReplacePrefix(src, dst) for t in targets]
                if remapped != targets:
                    rel.SetTargets(remapped)
            spg = prim.GetAttribute("info:spg:sourceAsset")
            if spg and spg.HasAuthoredValue():
                spg.Set(Sdf.AssetPath(_package_qualified(spg.Get().path)))
            spec = session.GetPrimAtPath(prim.GetPath())
            if spec and spec.referenceList.prependedItems:
                spec.referenceList.prependedItems = [
                    Sdf.Reference(_package_qualified(ref.assetPath)) for ref in spec.referenceList.prependedItems
                ]
        stage.GetPrimAtPath(dst_rp_path).GetRelationship("camera").SetTargets([Sdf.Path(camera_path)])

    res = stage.GetPrimAtPath(dst_rp_path).GetAttribute("resolution").Get()
    return int(res[0]), int(res[1])


@dataclass
class RenderTarget:
    """A wired render product for one camera: what to attach an annotator to, plus its camera/size.

    `tex` holds the HydraTexture for an authored-RP (SPG) target; `render_product` holds the
    created Replicator render product for a plain target. Exactly one is set.
    """

    attach_target: Any
    camera_path: str
    width: int
    height: int
    tex: Any = None
    render_product: Any = None

    def release(self) -> None:
        """Release the render resources (HydraTexture or created render product)."""
        if self.tex is not None:
            self.tex.set_updates_enabled(False)
            self.tex = None
        if self.render_product is not None:
            self.render_product.destroy()
            self.render_product = None


class RenderTargetFactory:
    """Acquires a render target for a camera, binding an authored PPISP RenderProduct or creating one.

    When ``has_spg`` is set it binds a HydraTexture to an authored RenderProduct; otherwise it creates a
    RenderProduct on the camera prim. ``create`` acquires the target for one camera; ``clone`` copies an
    authored RenderProduct so several cameras can capture through PPISP at the same time.

    Args:
        has_spg: Whether to bind authored PPISP RenderProducts rather than create plain ones.
        resolution: (width, height) for a RenderProduct created on a camera prim.
    """

    def __init__(self, has_spg: bool, *, resolution: tuple[int, int] = (1920, 1080)) -> None:
        self._has_spg = has_spg
        self._resolution = resolution

    def create(
        self,
        stage: Usd.Stage,
        name: str,
        *,
        rp_path: str | None = None,
        camera_path: str | None = None,
    ) -> RenderTarget:
        """Build the render target for one camera.

        Args:
            stage: The open USD stage.
            name: Logical camera name (for the texture name and logs).
            rp_path: Authored RenderProduct prim path (SPG stage).
            camera_path: Camera prim path to create a RenderProduct on (plain stage).

        Returns:
            The wired RenderTarget.
        """
        if self._has_spg:
            return self._from_authored_rp(stage, name, rp_path)
        return self._from_camera(name, camera_path)

    def _from_authored_rp(self, stage: Usd.Stage, name: str, rp_path: str) -> RenderTarget:
        """Bind a HydraTexture to the NuRec USD's authored RenderProduct.

        Args:
            stage: The open USD stage.
            name: Logical camera name.
            rp_path: Authored RenderProduct prim path.

        Returns:
            The wired RenderTarget (carrying the HydraTexture).

        Raises:
            RuntimeError: If the HydraTexture factory or texture can't be acquired.
            ValueError: If the RenderProduct has no camera target.
        """
        import omni.hydratexture

        factory = omni.hydratexture.acquire_hydra_texture_factory_interface()
        if factory is None:
            raise RuntimeError("Failed to acquire omni.hydratexture factory interface")
        rp_prim = stage.GetPrimAtPath(rp_path)
        res = rp_prim.GetAttribute("resolution").Get()
        width, height = int(res[0]), int(res[1])
        targets = rp_prim.GetRelationship("camera").GetTargets()
        if not targets:
            raise ValueError(f"authored RP {rp_path} has no camera target")
        camera_path = str(targets[0])
        tex = factory.create_hydra_texture(
            name=f"spg_capture_{name}",
            width=width,
            height=height,
            usd_camera_path=camera_path,
            hydra_engine_name="rtx",
        )
        if tex is None:
            raise RuntimeError(f"create_hydra_texture returned None for {name}")
        tex.set_render_product_path(rp_path)
        carb.log_warn(f"[{name}] HydraTexture {width}x{height} cam={camera_path} -> {rp_path}")
        return RenderTarget(
            attach_target=rp_path,
            camera_path=camera_path,
            width=width,
            height=height,
            tex=tex,
        )

    def _from_camera(self, name: str, camera_path: str) -> RenderTarget:
        """Create a RenderProduct on an existing camera prim.

        Args:
            name: Logical camera name.
            camera_path: Camera prim path.

        Returns:
            The wired RenderTarget (carrying the created render product).
        """
        width, height = self._resolution
        rp = rep.create.render_product(camera_path, (width, height), force_new=False)
        carb.log_warn(f"[{name}] created RenderProduct {width}x{height} on {camera_path}")
        return RenderTarget(
            attach_target=rp,
            camera_path=camera_path,
            width=width,
            height=height,
            render_product=rp,
        )

    def clone(
        self,
        stage: Usd.Stage,
        name: str,
        *,
        src_rp_path: str,
        camera_path: str,
        dst_rp_path: str | None = None,
        package_path: str | None = None,
    ) -> RenderTarget:
        """Copy an authored RenderProduct for ``camera_path`` and bind it.

        Returns a RenderTarget like ``create``; several clones can be live at once so multiple cameras
        capture through PPISP at the same time.

        Args:
            stage: The open USD stage.
            name: Logical camera name (for the texture name and the default clone path).
            src_rp_path: The authored RenderProduct to copy.
            camera_path: The camera the clone renders.
            dst_rp_path: Path for the cloned RenderProduct; defaults to ``/Render/<name>_clone``.
            package_path: Package the SPG members live in; defaults to the stage's root layer.

        Returns:
            The wired RenderTarget for the clone.
        """
        if dst_rp_path is None:
            dst_rp_path = f"{RENDER_SCOPE_PATH}/{name}_clone"
        copy_render_product(stage, src_rp_path, dst_rp_path, camera_path, package_path=package_path)
        return self._from_authored_rp(stage, name, dst_rp_path)


class CameraRenderer:
    """Renders one camera's render product at rig keyframes or at explicit poses.

    Construct from a `RenderTarget` (see `RenderTargetFactory`), then call
    `render_at_keyframe(ts_ns, tol_us)` or `render_at_pose(pose)` per frame, and `close()`.

    Args:
        stage: The open USD stage (time mapping and rig keyframes).
        name: Logical camera name (for logs).
        simulation_app: The running SimulationApp.
        target: The wired render target to render from.
        warmup_steps: RTPT accumulation ticks per frame.
        force_identity_exposure: Force the camera to pass-through exposure (SPG/PPISP stages,
            where PPISP is the photometric authority).
    """

    @classmethod
    def open(
        cls,
        stage: Usd.Stage,
        name: str,
        simulation_app: Any,
        target: RenderTarget,
        **kwargs: Any,
    ) -> "CameraRenderer":
        """Wire a CameraRenderer to `target`, releasing the target if construction fails.

        Args:
            stage: The open USD stage (time mapping and rig keyframes).
            name: Logical camera name (for logs).
            simulation_app: The running SimulationApp.
            target: The wired render target to render from.
            **kwargs: Additional keyword arguments forwarded to the constructor.

        Returns:
            The wired CameraRenderer.
        """
        try:
            return cls(stage, name, simulation_app, target, **kwargs)
        except Exception:
            target.release()
            raise

    def __init__(
        self,
        stage: Usd.Stage,
        name: str,
        simulation_app: Any,
        target: RenderTarget,
        *,
        warmup_steps: int = 800,
        force_identity_exposure: bool = True,
    ) -> None:
        self._sim = simulation_app
        self._warmup_steps = int(warmup_steps)
        self.name = name
        self._stage = stage  # kept for `render_at_pose` (camera transform override)
        self._pose_op = None  # the xformOp authored on first `render_at_pose`
        self._saved_xform = None
        self.camera_path = target.camera_path  # exposed for pose capture (e.g. pose-heatmaps)
        self.width, self.height = target.width, target.height
        self._tex = target.tex
        self._render_product = target.render_product

        self._tcps, offset_us = stage_time_params(stage)
        self._keyframe_index = KeyframeIndex.from_stage(stage, self._tcps, offset_us)

        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.set_auto_update(False)

        if force_identity_exposure:
            fixed = ensure_identity_exposure(stage, self.camera_path)
            if fixed:
                carb.log_warn(f"[{name}] forced camera exposure to identity: {fixed}")

        # Pause hydra-texture updates on a created render product while attaching the annotator.
        if self._render_product is not None:
            self._render_product.hydra_texture.set_updates_enabled(False)
        self._annotator = rep.AnnotatorRegistry.get_annotator("LdrColor")
        self._annotator.attach(target.attach_target)
        if self._render_product is not None:
            self._render_product.hydra_texture.set_updates_enabled(True)
        carb.log_warn(f"[{name}] LdrColor annotator attached")

        # Prime so the SPG kernels compile and the gaussian/NuRec plugin warms up.
        for _ in range(5):
            simulation_app.update()
        try:
            rep.orchestrator.step()
        except Exception as exc:
            carb.log_warn(f"[{name}] priming orchestrator.step() failed: {exc!r}")

    def render_at_keyframe(self, ts_ns: int, tol_us: float = 1.0) -> tuple[np.ndarray, float] | None:
        """Render at the sensor-rig keyframe whose timestamp matches `ts_ns` within `tol_us`.

        Renders the exact training pose for the requested timestamp (the matched keyframe),
        not an interpolated one. Requires the stage to have a sensor rig — returns None (with
        an error) when it has none.

        Args:
            ts_ns: The requested timestamp in nanoseconds.
            tol_us: Match tolerance in microseconds.

        Returns:
            A tuple of (HxWx3 uint8 RGB, matched time code), or None when the stage has no
            sensor rig, no keyframe is within tolerance, or no frame was produced.
        """
        if not self._keyframe_index:
            carb.log_error(f"[{self.name}] render_at_keyframe needs a sensor rig; stage has none — use render_at_pose")
            return None
        time_code = self._keyframe_index.match(ts_ns, tol_us)
        if time_code is None:
            return None
        rgb = self._render(time_code, self._tcps)
        if rgb is None:
            return None
        return rgb, time_code

    def render_at_pose(self, pose: list[float], time_code: float = 0.0) -> np.ndarray | None:
        """Render the camera at an explicit world pose, with scene content at `time_code`.

        `pose` is TUM order [tx, ty, tz, qx, qy, qz, qw] and is used as the camera's world
        transform (the rig parent is ignored). After the first pose render the camera no longer
        follows the rig, so don't mix `render_at_pose` and `render_at_keyframe` on one instance.

        Args:
            pose: World pose [tx, ty, tz, qx, qy, qz, qw].
            time_code: USD time code for the (possibly time-dependent) scene content.

        Returns:
            HxWx3 uint8 RGB, or None if no frame was produced.
        """
        from pxr import Gf

        tx, ty, tz, qx, qy, qz, qw = pose
        matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))
        matrix.SetTranslateOnly(Gf.Vec3d(tx, ty, tz))
        if self._pose_op is None:
            self._saved_xform = _snapshot_camera_xform(self._stage, self.camera_path)
            self._pose_op = _override_camera_pose(self._stage, self.camera_path)
        self._pose_op.Set(matrix)
        return self._render(time_code, self._tcps)

    def _render(self, time_code: float, tcps: float, *, attempts: int = 8) -> np.ndarray | None:
        """Render at `time_code`; return HxWx3 uint8 RGB, or None if no frame after `attempts`.

        Moves the timeline to `time_code`, then steps the render graph and ticks the app until
        the annotator yields a frame.

        Args:
            time_code: The USD time code to render at.
            tcps: Time codes per second (maps the time code to a timeline time).
            attempts: Max step/tick rounds to wait for the annotator to yield a frame.

        Returns:
            HxWx3 uint8 RGB, or None if no frame was produced after `attempts`.
        """
        self._timeline.set_current_time(time_code / tcps if tcps > 0 else 0.0)
        arr = None
        n = 0
        while arr is None and n < attempts:
            try:
                rep.orchestrator.step()
            except Exception as exc:
                carb.log_warn(f"[{self.name}] orchestrator.step exc on attempt {n}: {exc!r}")
            for _ in range(max(self._warmup_steps // 8, 5)):
                self._sim.update()
            arr = _coerce_array(self._annotator.get_data())
            n += 1
        if arr is None:
            return None
        if arr.ndim == 3 and arr.shape[-1] == 4:  # LdrColor is typically RGBA8; drop alpha
            arr = arr[..., :3]
        return arr

    def close(self) -> None:
        """Detach the annotator, restore the camera's rig coupling, and release the render target."""
        if self._pose_op is not None:
            _restore_camera_xform(self._stage, self.camera_path, self._saved_xform)
            self._pose_op = None
            self._saved_xform = None
        self._annotator.detach()
        if self._tex is not None:
            self._tex.set_updates_enabled(False)
            self._tex = None
        if self._render_product is not None:
            self._render_product.destroy()
            self._render_product = None


# --- Internal helpers ---------------------------------------------------------------------


def _snapshot_camera_xform(stage: Usd.Stage, camera_path: str) -> tuple[Any, bool]:
    """Capture a camera's xform-op order and reset-stack flag, to restore after a pose override.

    Args:
        stage: The open USD stage.
        camera_path: The camera prim path.

    Returns:
        The `(xformOpOrder, resetXformStack)` pair to pass back to `_restore_camera_xform`.
    """
    from pxr import UsdGeom

    xform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_path))
    return xform.GetXformOpOrderAttr().Get(), xform.GetResetXformStack()


def _override_camera_pose(stage: Usd.Stage, camera_path: str) -> Any:
    """Replace a camera's xform with a single world transform op (rig parent ignored).

    Args:
        stage: The open USD stage.
        camera_path: The camera prim path.

    Returns:
        The added transform op, whose value the caller sets to the world pose.
    """
    from pxr import UsdGeom

    xform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_path))
    xform.ClearXformOpOrder()
    op = xform.AddTransformOp()
    xform.SetResetXformStack(True)  # ignore the rig parent -> op value is the world pose
    return op


def _restore_camera_xform(stage: Usd.Stage, camera_path: str, snapshot: tuple[Any, bool]) -> None:
    """Restore a camera's xform-op order and reset-stack flag captured by `_snapshot_camera_xform`.

    Undoes an `_override_camera_pose` so the camera follows the rig again, keeping the pose
    override from leaking to the next consumer of the same prim.

    Args:
        stage: The open USD stage.
        camera_path: The camera prim path.
        snapshot: The `(xformOpOrder, resetXformStack)` pair from `_snapshot_camera_xform`.
    """
    from pxr import UsdGeom

    xform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_path))
    saved_order, saved_reset = snapshot
    xform.ClearXformOpOrder()
    xform.SetResetXformStack(bool(saved_reset))
    if saved_order:
        xform.GetXformOpOrderAttr().Set(saved_order)


def _coerce_array(data: Any) -> np.ndarray | None:
    """Pull a numpy array out of a Replicator annotator `get_data()` payload.

    Handles a bare ndarray, a modern ``{"data": ndarray, "info": {...}}`` dict, or
    ``None`` (graph hasn't produced a frame yet).

    Args:
        data: The annotator payload (ndarray, dict, or None).

    Returns:
        A numpy ndarray, or None when the payload isn't a valid array yet (used to gate
        the retry loop).
    """
    if data is None:
        return None
    payload = data["data"] if isinstance(data, dict) else data
    if payload is None:
        return None
    try:
        arr = np.asarray(payload)
    except Exception:
        return None
    if arr.ndim < 2 or arr.size == 0:
        return None
    return arr


def ensure_identity_exposure(stage: Usd.Stage, camera_path: str, *, override: bool = True) -> list[str]:
    """Check (and, by default, set) `camera_path`'s exposure to identity.

    Identity is `exposure=0, fStop=1, iso=0, responsivity=1, time=1` with auto-exposure off.

    Args:
        stage: The open USD stage.
        camera_path: Prim path of the camera to normalize.
        override: When True (default), author the non-identity attributes; when False, only
            report them.

    Returns:
        The attributes that were not identity (each as `name=old->new`); empty when the
        camera was already identity.
    """
    prim = stage.GetPrimAtPath(camera_path)
    if not prim.IsValid():
        return []
    fixed: list[str] = []
    for name, want in _IDENTITY_EXPOSURE.items():
        attr = prim.GetAttribute(name)
        current = attr.Get() if attr and attr.HasAuthoredValue() else None
        if current != want:
            fixed.append(f"{name}={current!r}->{want!r}")
            if override:
                prim.CreateAttribute(name, Sdf.ValueTypeNames.Float).Set(want)
    ae = prim.GetAttribute("omni:rtx:autoExposure:enabled")
    ae_current = ae.Get() if ae and ae.HasAuthoredValue() else None
    if ae_current not in (False, 0):
        fixed.append(f"omni:rtx:autoExposure:enabled={ae_current!r}->False")
        if override:
            prim.CreateAttribute("omni:rtx:autoExposure:enabled", Sdf.ValueTypeNames.Bool).Set(False)
    return fixed


def _resolve_camera_targets(stage: Usd.Stage, has_spg: bool, wanted: set[str]) -> dict[str, dict]:
    """Return per-camera kwargs for `RenderTargetFactory.create`, for the wanted cameras.

    SPG NuRec USDs use their authored RenderProducts (`rp_path=`); plain NuRec USDs use located
    `Camera` prims (`camera_path=`).

    Args:
        stage: The open USD stage.
        has_spg: Whether the stage is an SPG (PPISP) NuRec USD.
        wanted: Camera names to keep.

    Returns:
        Mapping of camera name to the factory `create` kwargs.
    """
    if has_spg:
        return {c: {"rp_path": rp} for c, rp in discover_render_products(stage).items() if c in wanted}
    return {c: {"camera_path": p} for c, p in discover_cameras(stage).items() if c in wanted}


def _open_camera_renderer(
    factory: RenderTargetFactory,
    stage: Usd.Stage,
    cam: str,
    simulation_app: Any,
    create_kwargs: dict,
    *,
    warmup_steps: int,
    force_identity_exposure: bool,
) -> CameraRenderer:
    """Acquire a render target for `cam` and wire a CameraRenderer to it.

    Releases the render target if wiring the CameraRenderer fails, so a failed setup leaks no
    HydraTexture or created render product.

    Args:
        factory: Factory that creates render targets.
        stage: The open USD stage.
        cam: Camera name to render.
        simulation_app: The running SimulationApp.
        create_kwargs: Extra kwargs forwarded to `factory.create`.
        warmup_steps: RTPT accumulation ticks per frame.
        force_identity_exposure: Whether to force pass-through exposure on the camera.

    Returns:
        The wired CameraRenderer.
    """
    target = factory.create(stage, cam, **create_kwargs)
    return CameraRenderer.open(
        stage,
        cam,
        simulation_app,
        target,
        warmup_steps=warmup_steps,
        force_identity_exposure=force_identity_exposure,
    )


def render_keyframes(
    simulation_app: Any,
    stage_path: str,
    output_dir: str,
    per_camera_ts: dict[str, list[int]],
    *,
    warmup_steps: int = 800,
    keyframe_tol_us: float = 1.0,
    config_path: str | None = None,
    resolution: tuple[int, int] = (1920, 1080),
    timing_label: str | None = None,
) -> str | None:
    """Render each camera at the sensor-rig keyframe matching each requested timestamp.

    Requires the stage to carry a sensor rig (otherwise nothing renders — use `render_poses`).
    Each frame is saved as `<output_dir>/<camera>/<ts_ns>.png`. Assumes a running SimulationApp
    (the caller owns boot/teardown).

    Args:
        simulation_app: The already-booted SimulationApp.
        stage_path: Path/URL of the NuRec USD to render.
        output_dir: Directory to write frames + manifest into.
        per_camera_ts: Mapping of camera name to the timestamps (ns) to render.
        warmup_steps: RTPT accumulation ticks per frame.
        keyframe_tol_us: Tolerance (us) for matching a requested timestamp to a rig keyframe.
        config_path: Optional YAML overriding the shipped render config (carb overrides).
        resolution: (width, height) for the RenderProduct created on a plain NuRec USD.
        timing_label: Optional label used for scoped timing output.

    Returns:
        The path of the written manifest, or None when nothing was rendered.
    """
    with _timer(timing_label, "stage_open") as timer:
        stage = open_stage(stage_path)
    _log_timer(timing_label, "stage open", timer)
    if stage is None:
        carb.log_error(f"Failed to open stage: {stage_path}")
        return None
    with _timer(timing_label, "setup_for_rendering") as timer:
        success, _, has_spg, _ = setup_for_rendering(stage, config_path)
    _log_timer(timing_label, "setup_for_rendering", timer)
    if not success:
        return None
    # Must stay after setup_for_rendering and before any render tick.
    with _timer(timing_label, "post_setup_update") as timer:
        simulation_app.update()
    _log_timer(timing_label, "post setup app update", timer)
    with _timer(timing_label, "rig_keyframe_scan") as timer:
        has_rig_keyframes = bool(rig_keyframe_time_codes(stage))
    _log_timer(timing_label, "rig keyframe scan", timer)
    if not has_rig_keyframes:
        carb.log_error(f"{stage_path} has no sensor rig; keyframes mode is unavailable (use poses mode).")
        return None

    with _timer(timing_label, "resolve_render_targets") as timer:
        targets = _resolve_camera_targets(stage, has_spg, set(per_camera_ts))
    _log_timer(timing_label, "resolve render targets", timer)
    if not targets:
        carb.log_error(f"No render targets for requested cameras {sorted(per_camera_ts)}.")
        return None

    factory = RenderTargetFactory(has_spg, resolution=resolution)
    pairs: list[dict] = []
    for cam, create_kwargs in targets.items():
        try:
            with _timer(timing_label, f"{cam}.renderer_setup") as timer:
                cap = _open_camera_renderer(
                    factory,
                    stage,
                    cam,
                    simulation_app,
                    create_kwargs,
                    warmup_steps=warmup_steps,
                    force_identity_exposure=has_spg,
                )
            _log_timer(timing_label, f"{cam} renderer setup", timer)
        except Exception as exc:
            carb.log_error(f"[{cam}] renderer setup failed: {exc!r}; skipping camera.")
            continue
        camera_attempts = 0
        camera_frames = 0
        camera_render_ms = 0.0
        try:
            for ts_ns in per_camera_ts[cam]:
                camera_attempts += 1
                carb.log_info(f"[{cam}] rendering frame {camera_attempts}/{len(per_camera_ts[cam])} ts_ns={ts_ns}")
                with _timer(timing_label, f"{cam}.render_frame") as timer:
                    result = cap.render_at_keyframe(ts_ns, keyframe_tol_us)
                if timing_label and timer is not None and timer.elapsed_time_ms is not None:
                    camera_render_ms += timer.elapsed_time_ms
                if result is None:
                    carb.log_warn(
                        f"[{cam}] ts_ns={ts_ns}: no keyframe within {keyframe_tol_us}us / no frame; skipping."
                    )
                    continue
                camera_frames += 1
                rgb, time_code = result
                if rgb.dtype != np.uint8:
                    rgb = rgb.astype(np.uint8)
                out_path = os.path.join(output_dir, cam, f"{ts_ns}.png")
                save_image(out_path, rgb)
                pairs.append(
                    {
                        "camera": cam,
                        "rendered": out_path,
                        "ts_ns": ts_ns,
                        "position": camera_world_position(stage, cap.camera_path, time_code),
                    }
                )
                carb.log_warn(f"[{cam}] ts_ns={ts_ns} -> {out_path}")
        finally:
            cap.close()
        if timing_label and camera_attempts:
            avg_ms = camera_render_ms / camera_attempts
            carb.log_warn(
                f"[nurec timing] {timing_label} {cam} rendering: "
                f"{camera_render_ms:.3f} ms total, {avg_ms:.3f} ms avg over "
                f"{camera_attempts} requested frame(s), {camera_frames} saved"
            )

    return write_manifest(output_dir, stage_path, list(targets), pairs) if pairs else None


def render_poses(
    simulation_app: Any,
    stage_path: str,
    output_dir: str,
    per_camera_poses: dict[str, list[tuple[int, list[float]]]],
    *,
    warmup_steps: int = 800,
    config_path: str | None = None,
    resolution: tuple[int, int] = (1920, 1080),
) -> str | None:
    """Render each camera at explicit TUM poses and write a manifest.

    The view comes entirely from the supplied pose; the timestamp is symbolic — it only names
    the output frame, saved as `<output_dir>/<camera>/<ts_ns>.png`. Assumes a running
    SimulationApp (the caller owns boot/teardown).

    Args:
        simulation_app: The already-booted SimulationApp.
        stage_path: Path/URL of the NuRec USD to render.
        output_dir: Directory to write frames + manifest into.
        per_camera_poses: Mapping of camera name to (ts_ns, pose) entries, where ts_ns is the
            symbolic output-filename label and pose is [tx, ty, tz, qx, qy, qz, qw].
        warmup_steps: RTPT accumulation ticks per frame.
        config_path: Optional YAML overriding the shipped render config (carb overrides).
        resolution: (width, height) for the RenderProduct created on a plain NuRec USD.

    Returns:
        The path of the written manifest, or None when nothing was rendered.
    """
    stage = open_stage(stage_path)
    if stage is None:
        carb.log_error(f"Failed to open stage: {stage_path}")
        return None
    success, _, has_spg, _ = setup_for_rendering(stage, config_path)
    if not success:
        return None
    # Must stay after setup_for_rendering and before any render tick.
    simulation_app.update()

    targets = _resolve_camera_targets(stage, has_spg, set(per_camera_poses))
    if not targets:
        carb.log_error(f"No render targets for requested cameras {sorted(per_camera_poses)}.")
        return None

    factory = RenderTargetFactory(has_spg, resolution=resolution)
    pairs: list[dict] = []
    for cam, create_kwargs in targets.items():
        try:
            cap = _open_camera_renderer(
                factory,
                stage,
                cam,
                simulation_app,
                create_kwargs,
                warmup_steps=warmup_steps,
                force_identity_exposure=has_spg,
            )
        except Exception as exc:
            carb.log_error(f"[{cam}] renderer setup failed: {exc!r}; skipping camera.")
            continue
        try:
            for ts_ns, pose in per_camera_poses[cam]:
                rgb = cap.render_at_pose(pose)
                if rgb is None:
                    carb.log_warn(f"[{cam}] pose ts_ns={ts_ns}: no frame; skipping.")
                    continue
                if rgb.dtype != np.uint8:
                    rgb = rgb.astype(np.uint8)
                out_path = os.path.join(output_dir, cam, f"{ts_ns}.png")
                save_image(out_path, rgb)
                pairs.append(
                    {
                        "camera": cam,
                        "rendered": out_path,
                        "ts_ns": ts_ns,
                        "position": pose[:3],
                    }
                )
                carb.log_warn(f"[{cam}] pose ts_ns={ts_ns} -> {out_path}")
        finally:
            cap.close()

    return write_manifest(output_dir, stage_path, list(targets), pairs) if pairs else None
