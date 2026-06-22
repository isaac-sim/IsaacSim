# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Structured light camera sensor authoring.

Provides :class:`StructuredLightCamera`, an :class:`RtxCamera` subclass that manages
a cluster of :class:`UsdLux.RectLight` projector prims. Each projector pattern is
activated in turn based on the current simulation time and a user-supplied list of
rational timestamps, enabling structured light imaging workflows.
"""

from __future__ import annotations

from fractions import Fraction
from pathlib import Path
from typing import Any

import carb
import carb.eventdispatcher
import carb.events
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.timeline
import warp as wp
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

from .rtx_camera import RtxCamera

# Fraction precision when converting from float simulation time (nanoseconds).
_TIME_FRACTION_LIMIT = 10**9


def _to_fraction(value: tuple[int, int]) -> Fraction:
    """Convert a ``(numerator, denominator)`` tuple to a :class:`fractions.Fraction`.

    Args:
        value: Rational tuple to convert.

    Returns:
        Fraction with the same rational value.

    Raises:
        TypeError: If either element is not an ``int``.
    """
    if (
        not isinstance(value[0], int)
        or not isinstance(value[1], int)
        or isinstance(value[0], bool)
        or isinstance(value[1], bool)
    ):
        raise TypeError(f"Rational time must be a tuple of two ints; got {value!r}")
    return Fraction(value[0], value[1])


def _fraction_to_tuple(value: Fraction) -> tuple[int, int]:
    """Convert a :class:`fractions.Fraction` to a ``(numerator, denominator)`` tuple.

    Args:
        value: Fraction to convert.

    Returns:
        Rational tuple with the same value.
    """
    return (value.numerator, value.denominator)


class StructuredLightCamera(RtxCamera):
    r"""Structured light camera sensor: an :class:`RtxCamera` with cycling projectors.

    Extends :class:`RtxCamera` by creating a set of :class:`UsdLux.RectLight` prims
    (one per projector pattern) under a shared parent Xform and cycling through them
    based on the current simulation time and a list of user-supplied rational
    timestamps.

    .. note::

        This class is an authoring class — it creates and manages USD prims. For
        data acquisition, wrap an instance in :class:`CameraSensor`:

        .. code-block:: python

            cam = StructuredLightCamera(...)
            sensor = CameraSensor(cam, resolution=(720, 1280), annotators=["rgb"])

    **Timestamps**

    ``projector_timestamps`` is a list of ``(numerator, denominator)`` rational
    tuples, one per pattern. Each tuple represents the simulation time (in seconds)
    at which that pattern becomes active. The first entry **must** represent
    :math:`t = 0` (typically ``(0, 1)``, though any ``(0, k)`` is accepted) and
    the list **must** be strictly increasing. Rational tuples avoid the
    floating-point precision issues that arise when timestamps span nanoseconds to
    milliseconds.

    After the last timestamp, the cycle repeats with period
    ``projector_cycle_period``. If not supplied at construction, the period is
    inferred as ``timestamps[-1] + (timestamps[1] - timestamps[0])`` for
    :math:`N \\geq 2` or ``Fraction(1, 30)`` for :math:`N = 1`. Calling
    :meth:`set_projector_timestamps` preserves an explicitly-supplied period and
    re-infers an implicit one.

    **Warnings**

    - If the observed simulation ``dt`` exceeds the minimum projector timestamp
      interval (patterns would be impossible to resolve), a warning is logged
      once.
    - If a single tick advances the active pattern by more than one index (mod
      ``N``), a warning is logged once per tick where this occurs.

    Args:
        path: Prim path of the USD Camera prim to create or wrap.
        projector_light_patterns: Paths to projector pattern images (e.g. PNG
            files). One :class:`UsdLux.RectLight` prim is created per pattern.
        projector_direction_texture: Path to the projector direction texture
            (typically an EXR file) that defines the per-pixel projection
            direction. Applied to every pattern's RectLight prim.
        projector_prim_path: Prim path of the projector parent Xform. Defaults to
            ``f"{path}/projectors"``.
        projector_position: World-frame position of the projector Xform (shape
            ``(3,)``). Defaults to the camera's world position.
        projector_orientation: World-frame quaternion (``wxyz``) of the projector
            Xform (shape ``(4,)``). Defaults to the camera's world orientation.
        projector_timestamps: Activation times for each pattern as
            ``list[tuple[int, int]]``. First entry must be ``(0, 1)``; the list
            must be strictly increasing. Defaults to
            ``[(i, 30) for i in range(N)]``.
        projector_cycle_period: Explicit cycle period as ``(numerator,
            denominator)``. Defaults to an inferred value (see above).
        projector_intensity: RectLight intensity. Defaults to ``150000.0``.
        projector_width: RectLight width. Defaults to ``1.0``.
        projector_height: RectLight height. Defaults to ``1.0``.
        tick_rate: RTX camera tick rate in Hz. See :class:`RtxCamera`.
        schemas: Additional API schemas to apply. See :class:`RtxCamera`.
        attributes: Additional attributes to set on the Camera prim. See
            :class:`RtxCamera`.
        positions: Camera world-frame positions (shape ``(1, 3)``). See
            :class:`RtxCamera`.
        translations: Camera local-frame translations (shape ``(1, 3)``). See
            :class:`RtxCamera`.
        orientations: Camera world-frame orientations (shape ``(1, 4)``,
            ``wxyz``). See :class:`RtxCamera`.
        scales: Camera scales (shape ``(1, 3)``). See :class:`RtxCamera`.
        reset_xform_op_properties: Whether to reset the Camera's xformOp stack.
            See :class:`RtxCamera`.

    Raises:
        ValueError: If ``projector_light_patterns`` is empty.
        ValueError: If ``projector_direction_texture`` is ``None``.
        ValueError: If ``projector_timestamps`` is invalid (wrong length, first
            entry not zero, not strictly increasing).
        ValueError: If ``projector_cycle_period`` is not strictly greater than
            the last timestamp.

    Example:

    .. code-block:: python

        >>> from pathlib import Path
        >>> from isaacsim.sensors.experimental.rtx import StructuredLightCamera
        >>>
        >>> patterns = [Path(f"patterns/image_{i:02d}.png") for i in range(10)]
        >>> timestamps = [(i, 1000) for i in range(10)]  # 1 ms spacing
        >>> cam = StructuredLightCamera(
        ...     "/World/camera",
        ...     projector_light_patterns=patterns,
        ...     projector_direction_texture=Path("direction_texture.exr"),
        ...     projector_timestamps=timestamps,
        ... )  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        path: str,
        projector_light_patterns: list[str | Path],
        projector_direction_texture: str | Path,
        *,
        projector_prim_path: str | None = None,
        projector_position: np.ndarray | None = None,
        projector_orientation: np.ndarray | None = None,
        projector_timestamps: list[tuple[int, int]] | None = None,
        projector_cycle_period: tuple[int, int] | None = None,
        projector_intensity: float = 150000.0,
        projector_width: float = 1.0,
        projector_height: float = 1.0,
        tick_rate: float | None = None,
        schemas: list[str] | None = None,
        attributes: dict[str, Any] | None = None,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
    ) -> None:
        # Define attributes first so that destroy() / __del__ are safe if __init__ raises.
        self._rect_light_prims: list[Usd.Prim] = []
        # Track prims that *this* instance created on the stage so _cleanup_partial_prims
        # can unwind on construction failure without destroying caller-authored prims.
        self._created_rect_light_paths: list[str] = []
        self._created_projector_xform: bool = False
        self._created_camera_prim: bool = False
        self._app_update_sub = None
        self._active_pattern_index = 0
        self._warned_coarse_dt = False
        self._prev_sim_time: float | None = None
        self._cycle_period_is_explicit: bool = projector_cycle_period is not None
        # Validate pattern inputs early (before super().__init__) so failed instances do
        # not leave a half-constructed Camera prim on the stage.
        if not projector_light_patterns:
            raise ValueError("At least one projector pattern must be provided.")
        if projector_direction_texture is None:
            raise ValueError("projector_direction_texture must be provided.")
        # Store the raw pattern identifiers (filesystem paths or asset URIs) as-is.
        # Wrapping with ``pathlib.Path`` would strip the double slash in URIs like
        # ``omniverse://server/...``; instead, pass the values verbatim to ``Sdf.AssetPath``
        # and let USD's asset resolver handle them.
        self._projector_patterns: list[str | Path] = list(projector_light_patterns)
        self._projector_direction_texture: str | Path = projector_direction_texture

        # Resolve and validate timestamps.
        num_patterns = len(self._projector_patterns)
        raw_timestamps = (
            projector_timestamps if projector_timestamps is not None else [(i, 30) for i in range(num_patterns)]
        )
        timestamps_frac = self._validate_timestamps(raw_timestamps, num_patterns)
        self._timestamps_frac: list[Fraction] = timestamps_frac
        self._cycle_period_frac: Fraction = self._resolve_cycle_period(timestamps_frac, projector_cycle_period)

        self._projector_intensity = projector_intensity
        self._projector_width = projector_width
        self._projector_height = projector_height

        # Resolve whether the Camera prim at ``path`` is being created fresh or wrapped
        # around an existing prim. Only fresh prims are safe to remove on rollback.
        existent_paths, _ = self.resolve_paths(path)
        self._created_camera_prim = len(existent_paths) == 0

        # Initialize the RtxCamera base — this creates or wraps the Camera prim.
        super().__init__(
            path,
            tick_rate=tick_rate,
            schemas=schemas,
            attributes=attributes,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )

        # Default projector path is a child Xform of the Camera prim.
        self._projector_prim_path = projector_prim_path or f"{self.paths[0]}/projectors"
        camera_path = self.paths[0]
        self._projector_is_child_of_camera = self._projector_prim_path.startswith(f"{camera_path}/")

        # Resolve projector pose. When the projector is a child of the camera and no
        # explicit pose is supplied, it already inherits the camera's transform via
        # parent composition, so we write an identity local transform.
        self._projector_position, self._projector_orientation = self._resolve_projector_pose(
            projector_position,
            projector_orientation,
            inherit_from_parent=self._projector_is_child_of_camera,
        )

        # Create projector prims; on failure, clean up the fresh Camera prim to avoid
        # leaving partial scene state on the stage.
        try:
            self._create_projector_lights()
            self._set_active_pattern(0)
        except Exception:
            self._cleanup_partial_prims()
            raise

        # Subscribe to app-update events (Events 2.0) so the pattern selection
        # follows simulation time. ``order=1`` ensures this observer fires
        # *after* :mod:`omni.replicator.core` orchestrator's update observer
        # (which subscribes with ``order=0`` and is the code that advances the
        # timeline). Without this explicit ordering the observer would read a
        # stale ``omni.timeline.get_current_time()`` and select the previous
        # pattern, causing the renderer to capture the wrong RectLight on the
        # tick that the orchestrator schedules a frame. The observer is
        # released by ``destroy()``.
        self._app_update_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_app_update,
            observer_name="isaacsim.sensors.experimental.rtx.StructuredLightCamera.projector_tick",
            order=1,
        )

    def _cleanup_partial_prims(self) -> None:
        """Best-effort removal of prims this instance created on the stage.

        Invoked from the constructor when ``_create_projector_lights`` raises. Only
        removes prims that were freshly created by this instance — caller-authored
        prims at overlapping paths (e.g. a pre-existing projector Xform supplied via
        ``projector_prim_path``) are left untouched. Errors are swallowed.
        """
        try:
            stage = stage_utils.get_current_stage(backend="usd")
        except Exception:
            return
        # Remove only RectLight prims that *this* instance created.
        for path in self._created_rect_light_paths:
            try:
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    stage.RemovePrim(path)
            except Exception:
                pass
        self._created_rect_light_paths.clear()
        self._rect_light_prims.clear()
        # Remove the projector parent Xform only if this instance created it.
        projector_path = getattr(self, "_projector_prim_path", None)
        if projector_path and self._created_projector_xform:
            try:
                proj_prim = stage.GetPrimAtPath(projector_path)
                if proj_prim.IsValid():
                    stage.RemovePrim(projector_path)
            except Exception:
                pass
        # Remove the Camera prim only if this instance created it (not wrapped).
        if self._created_camera_prim:
            try:
                camera_path = self.paths[0]
                cam_prim = stage.GetPrimAtPath(camera_path)
                if cam_prim.IsValid():
                    stage.RemovePrim(camera_path)
            except Exception:
                pass

    # -- validation helpers --

    @staticmethod
    def _validate_timestamps(
        timestamps: list[tuple[int, int]],
        expected_count: int,
    ) -> list[Fraction]:
        """Convert raw tuples to :class:`Fraction` values and validate the schedule.

        Args:
            timestamps: Raw activation timestamp tuples.
            expected_count: Expected number of timestamps.

        Returns:
            Validated activation timestamps.
        """
        if not isinstance(timestamps, (list, tuple)):
            raise ValueError(
                f"'projector_timestamps' must be a list of (numerator, denominator) tuples; got {type(timestamps).__name__}"
            )
        if len(timestamps) != expected_count:
            raise ValueError(
                f"'projector_timestamps' length ({len(timestamps)}) must equal the number of patterns ({expected_count})"
            )
        fracs: list[Fraction] = []
        for i, ts in enumerate(timestamps):
            if not isinstance(ts, (tuple, list)) or len(ts) != 2:
                raise ValueError(f"'projector_timestamps[{i}]' must be a (numerator, denominator) tuple; got {ts!r}")
            try:
                frac = _to_fraction(ts)
            except (TypeError, ValueError, ZeroDivisionError) as err:
                raise ValueError(f"'projector_timestamps[{i}]' is not a valid rational: {ts!r}") from err
            if frac < 0:
                raise ValueError(f"'projector_timestamps[{i}]' must be non-negative; got {frac} ({ts!r})")
            fracs.append(frac)
        if fracs[0] != Fraction(0):
            raise ValueError(
                f"'projector_timestamps[0]' must represent t=0 (typically (0, 1); any (0, k) is accepted); "
                f"got {timestamps[0]!r}"
            )
        for i in range(1, len(fracs)):
            if fracs[i] <= fracs[i - 1]:
                raise ValueError(
                    f"'projector_timestamps' must be strictly increasing; got {timestamps[i - 1]!r} followed by {timestamps[i]!r}"
                )
        return fracs

    @staticmethod
    def _resolve_cycle_period(
        timestamps_frac: list[Fraction],
        cycle_period: tuple[int, int] | None,
    ) -> Fraction:
        r"""Resolve the projector cycle period as a :class:`Fraction`.

        If ``cycle_period`` is not provided, it is inferred as
        ``timestamps[-1] + (timestamps[1] - timestamps[0])`` for :math:`N \\geq 2`
        or ``Fraction(1, 30)`` for :math:`N = 1`.

        Args:
            timestamps_frac: Validated activation timestamps.
            cycle_period: Explicit cycle period, or None to infer it.

        Returns:
            Resolved cycle period.
        """
        if cycle_period is not None:
            try:
                period_frac = _to_fraction(cycle_period)
            except (TypeError, ValueError, ZeroDivisionError) as err:
                raise ValueError(f"'projector_cycle_period' is not a valid rational: {cycle_period!r}") from err
            if period_frac <= timestamps_frac[-1]:
                raise ValueError(
                    f"'projector_cycle_period' ({period_frac}) must be strictly greater than the last timestamp "
                    f"({timestamps_frac[-1]})"
                )
            return period_frac
        if len(timestamps_frac) == 1:
            return Fraction(1, 30)
        first_interval = timestamps_frac[1] - timestamps_frac[0]
        return timestamps_frac[-1] + first_interval

    def _resolve_projector_pose(
        self,
        projector_position: np.ndarray | None,
        projector_orientation: np.ndarray | None,
        *,
        inherit_from_parent: bool,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return the local-frame projector pose.

        The returned pose is written as local xformOps on the projector parent Xform.
        If ``inherit_from_parent`` is True (the projector lives beneath the camera) and
        no explicit pose is supplied, an identity local transform is used so the
        projector inherits the camera's world pose via USD composition. Otherwise, any
        missing component defaults to the camera's world pose.

        Args:
            projector_position: Explicit projector position, or None to use the default.
            projector_orientation: Explicit projector orientation, or None to use the default.
            inherit_from_parent: Whether the projector Xform inherits from the camera.

        Returns:
            Projector position and orientation to author on the Xform.
        """
        if projector_position is not None and projector_orientation is not None:
            return np.asarray(projector_position, dtype=np.float64), np.asarray(projector_orientation, dtype=np.float64)
        identity_position = np.zeros(3, dtype=np.float64)
        identity_orientation = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        if inherit_from_parent:
            default_position = identity_position
            default_orientation = identity_orientation
        else:
            # Read the camera's world pose directly from USD.
            transform = UsdGeom.Xformable(self.prims[0]).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            if not transform.Orthonormalize():
                carb.log_warn(
                    "StructuredLightCamera: camera transform is degenerate; "
                    "projector pose fallback may be inaccurate."
                )
            default_position = np.array(transform.ExtractTranslation(), dtype=np.float64)
            rot_quat = transform.ExtractRotationQuat()
            default_orientation = np.array(
                [rot_quat.GetReal(), *rot_quat.GetImaginary()],
                dtype=np.float64,
            )
        position = (
            np.asarray(projector_position, dtype=np.float64) if projector_position is not None else default_position
        )
        orientation = (
            np.asarray(projector_orientation, dtype=np.float64)
            if projector_orientation is not None
            else default_orientation
        )
        return position, orientation

    # -- prim creation --

    def _create_projector_lights(self) -> None:
        """Create the projector parent Xform and one :class:`UsdLux.RectLight` per pattern.

        Idempotent: if the projector parent Xform or individual RectLight prims already
        exist, they are verified to have the correct type and then re-stamped with the
        caller-supplied attributes.
        """
        stage = stage_utils.get_current_stage(backend="usd")

        # Phase 1: validate target RectLight paths up-front so a type collision raises
        # before any caller-authored prims are mutated.
        for i in range(len(self._projector_patterns)):
            light_prim_path = f"{self._projector_prim_path}/RectLight_{i:02d}"
            existing = stage.GetPrimAtPath(light_prim_path)
            if existing.IsValid() and not existing.IsA(UsdLux.RectLight):
                raise RuntimeError(
                    f"Prim at '{light_prim_path}' already exists but is not a UsdLux.RectLight "
                    f"(actual type: '{existing.GetTypeName()}')."
                )

        # Phase 2: define the projector parent Xform if it does not already exist.
        projector_existed = stage.GetPrimAtPath(self._projector_prim_path).IsValid()
        stage_utils.define_prim(self._projector_prim_path, type_name="Xform")
        self._created_projector_xform = not projector_existed
        projector_xform_prim = prim_utils.get_prim_at_path(self._projector_prim_path)
        # Only write projector xformOps when this instance created the Xform.
        # Caller-authored Xforms keep their pre-existing transforms intact.
        if self._created_projector_xform:
            self._apply_projector_xform_ops(projector_xform_prim)

        # Phase 3: create or re-stamp one RectLight per pattern under the projector Xform.
        for i, pattern_path in enumerate(self._projector_patterns):
            light_prim_path = f"{self._projector_prim_path}/RectLight_{i:02d}"
            existing = stage.GetPrimAtPath(light_prim_path)
            if existing.IsValid():
                light = UsdLux.RectLight(existing)
                prim = existing
                created = False
            else:
                light = UsdLux.RectLight.Define(stage, light_prim_path)
                prim = light.GetPrim()
                created = True
                self._created_rect_light_paths.append(light_prim_path)

            if not prim.HasAPI(UsdLux.ShapingAPI):
                UsdLux.ShapingAPI.Apply(prim)
            light.GetIntensityAttr().Set(self._projector_intensity)
            light.GetWidthAttr().Set(self._projector_width)
            light.GetHeightAttr().Set(self._projector_height)
            # Preserve the caller-supplied path verbatim to keep URL schemes and
            # asset-resolver identifiers intact; USD's resolver handles relative paths.
            prim.CreateAttribute("inputs:texture:file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(pattern_path)))
            prim.CreateAttribute("isProjector", Sdf.ValueTypeNames.Bool).Set(True)
            prim.CreateAttribute("projector:directionTexture:file", Sdf.ValueTypeNames.Asset).Set(
                Sdf.AssetPath(str(self._projector_direction_texture))
            )
            prim.CreateAttribute("visibleInPrimaryRay", Sdf.ValueTypeNames.Bool).Set(False)
            imageable = UsdGeom.Imageable(prim)
            if i == 0:
                imageable.MakeVisible()
            else:
                imageable.MakeInvisible()
            self._rect_light_prims.append(prim)
            if not created:
                carb.log_info(f"StructuredLightCamera: re-stamped existing RectLight at '{light_prim_path}'")

        carb.log_info(
            f"StructuredLightCamera: created {len(self._rect_light_prims)} RectLight prims at "
            f"'{self._projector_prim_path}'"
        )

    def _apply_projector_xform_ops(self, projector_xform_prim: Usd.Prim) -> None:
        """Write translate/orient xformOps onto the projector parent Xform.

        If the Xform already has translate/orient ops, their values are overwritten;
        otherwise the ops are added.

        Args:
            projector_xform_prim: Projector parent Xform prim.
        """
        xformable = UsdGeom.Xformable(projector_xform_prim)
        existing_ops = {op.GetOpType(): op for op in xformable.GetOrderedXformOps()}
        translate_op = existing_ops.get(UsdGeom.XformOp.TypeTranslate) or xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*(float(v) for v in self._projector_position)))
        orient_op = existing_ops.get(UsdGeom.XformOp.TypeOrient) or xformable.AddOrientOp()
        orient_op.Set(
            Gf.Quatf(
                float(self._projector_orientation[0]),
                float(self._projector_orientation[1]),
                float(self._projector_orientation[2]),
                float(self._projector_orientation[3]),
            )
        )

    # -- app-update callback and pattern selection --

    def _on_app_update(self, event: carb.eventdispatcher.Event) -> None:
        """Select the active pattern based on the current simulation time.

        Args:
            event: App update event.
        """
        current_time = omni.timeline.get_timeline_interface().get_current_time()
        if current_time is None or current_time < 0:
            return
        new_index = self._pattern_index_at_time(current_time)

        # Coarse-dt / skip warnings.
        if self._prev_sim_time is not None and current_time > self._prev_sim_time:
            dt = current_time - self._prev_sim_time
            self._check_coarse_dt(dt)
            prev_index = self._active_pattern_index
            if new_index != prev_index:
                n = len(self._rect_light_prims)
                advance = (new_index - prev_index) % n
                if advance > 1:
                    skipped = [(prev_index + k) % n for k in range(1, advance)]
                    carb.log_warn(
                        f"StructuredLightCamera: projector tick skipped patterns {skipped} "
                        f"(sim_time={current_time:.9f}s, dt={dt:.9f}s). Consider reducing the "
                        "physics timestep or widening the projector timestamps."
                    )
        self._prev_sim_time = current_time

        if new_index != self._active_pattern_index:
            self._set_active_pattern(new_index)

    def _pattern_index_at_time(self, current_time: float) -> int:
        """Return the active pattern index for the given simulation time.

        Args:
            current_time: Current simulation time in seconds.

        Returns:
            Active pattern index.
        """
        t_frac = Fraction(current_time).limit_denominator(_TIME_FRACTION_LIMIT)
        phase = t_frac % self._cycle_period_frac
        active_idx = 0
        for i, ts_frac in enumerate(self._timestamps_frac):
            if ts_frac <= phase:
                active_idx = i
            else:
                break
        return active_idx

    def _check_coarse_dt(self, dt: float) -> None:
        """Warn once if the observed simulation dt is larger than the minimum pattern interval.

        Args:
            dt: Observed simulation time step in seconds.
        """
        if self._warned_coarse_dt or len(self._timestamps_frac) < 2:
            return
        min_interval_frac = min(
            self._timestamps_frac[i] - self._timestamps_frac[i - 1] for i in range(1, len(self._timestamps_frac))
        )
        # Also consider the wrap-around interval from the last pattern back to the first.
        wrap_interval = self._cycle_period_frac - self._timestamps_frac[-1]
        min_interval_frac = min(min_interval_frac, wrap_interval)
        min_interval = float(min_interval_frac)
        if dt > min_interval:
            carb.log_warn(
                f"StructuredLightCamera: observed simulation dt ({dt:.9f}s) is larger than the minimum "
                f"projector timestamp interval ({min_interval:.9f}s). Patterns may be skipped."
            )
            self._warned_coarse_dt = True

    def _set_active_pattern(self, pattern_index: int) -> None:
        """Enable the specified pattern and disable all others.

        Invalid prims (e.g., when the stage is closing) are skipped; the active index
        is still updated so the caller's view of state stays consistent.

        Args:
            pattern_index: Pattern index to activate.
        """
        for i, prim in enumerate(self._rect_light_prims):
            if not prim.IsValid():
                continue
            imageable = UsdGeom.Imageable(prim)
            if i == pattern_index:
                imageable.MakeVisible()
            else:
                imageable.MakeInvisible()
        self._active_pattern_index = pattern_index

    # -- public API --

    def destroy(self) -> None:
        """Unregister the app-update observer and release references.

        Safe to call multiple times. After ``destroy()``, the instance will no
        longer update the active pattern on simulation-time changes.
        """
        sub = self._app_update_sub
        if sub is not None:
            # Events 2.0 returns an ``ObservableSubscription`` whose ``reset()``
            # method releases the underlying C++ observer. Setting the attribute
            # to ``None`` afterwards drops the final Python reference.
            reset = getattr(sub, "reset", None)
            if callable(reset):
                try:
                    reset()
                except Exception:
                    pass
            self._app_update_sub = None

    def __del__(self) -> None:
        """Release the app-update subscription on garbage collection."""
        try:
            self.destroy()
        except Exception:
            pass
        # Chain to the base class destructor so XformPrim/Prim can release their own
        # Simulation-Manager callbacks and internal state.
        try:
            super_del = getattr(super(), "__del__", None)
            if callable(super_del):
                super_del()
        except Exception:
            pass

    def post_reset(self) -> None:
        """Reset projector state so the next tick starts fresh.

        Clears the previous-tick simulation-time cache, resets the coarse-dt
        warning flag, and activates pattern 0.
        """
        self._prev_sim_time = None
        self._warned_coarse_dt = False
        self._active_pattern_index = 0
        if self._rect_light_prims:
            self._set_active_pattern(0)

    def get_active_pattern_index(self) -> int:
        """Return the index of the currently active projector pattern.

        Returns:
            Currently active pattern index.
        """
        return self._active_pattern_index

    def set_active_pattern_manual(self, pattern_index: int) -> None:
        """Manually activate the pattern at ``pattern_index``.

        Bypasses time-based cycling until the next app-update tick, at which
        point the pattern will be re-selected based on the current simulation
        time. Useful for offline tests and deterministic capture loops.

        Args:
            pattern_index: Index of the pattern to activate.

        Raises:
            IndexError: If ``pattern_index`` is out of range.
        """
        n = len(self._rect_light_prims)
        if pattern_index < 0 or pattern_index >= n:
            raise IndexError(f"Pattern index {pattern_index} out of range [0, {n - 1}]")
        self._set_active_pattern(pattern_index)

    def get_num_patterns(self) -> int:
        """Return the number of projector patterns.

        Returns:
            Number of projector patterns.
        """
        return len(self._projector_patterns)

    def get_projector_prim_path(self) -> str:
        """Return the prim path of the projector parent Xform.

        Returns:
            Projector parent Xform prim path.
        """
        return self._projector_prim_path

    def get_rect_light_prims(self) -> list[Usd.Prim]:
        """Return the list of :class:`UsdLux.RectLight` prims, one per pattern.

        Returns:
            RectLight prims for the projector patterns.
        """
        return list(self._rect_light_prims)

    def get_projector_direction_texture(self) -> str | Path:
        """Return the projector direction texture identifier as supplied at construction.

        Returns:
            Projector direction texture identifier.
        """
        return self._projector_direction_texture

    def get_projector_timestamps(self) -> list[tuple[int, int]]:
        """Return the projector activation timestamps as rational tuples.

        Returns:
            Projector activation timestamps.
        """
        return [_fraction_to_tuple(f) for f in self._timestamps_frac]

    def set_projector_timestamps(self, timestamps: list[tuple[int, int]]) -> None:
        """Replace the projector activation timestamps.

        Behavior depends on how the cycle period was originally set:

        - If the cycle period was **implicit** (inferred from the original
          timestamps), it is re-inferred from the new timestamps.
        - If the cycle period was **explicit** (supplied at construction or via
          :meth:`set_projector_cycle_period`), it is preserved. The call raises
          ``ValueError`` if the explicit cycle period is no longer strictly
          greater than the new last timestamp.

        After a successful call, the coarse-dt warning is rearmed, the previous
        simulation-time cache is cleared, and pattern 0 is re-activated so the
        next tick starts in a consistent state.

        Args:
            timestamps: New activation times. See class docstring for the schema.

        Raises:
            ValueError: If ``timestamps`` is invalid or if an explicit cycle
                period becomes inconsistent with the new timestamps.
        """
        num = len(self._projector_patterns)
        new_fracs = self._validate_timestamps(timestamps, num)
        if self._cycle_period_is_explicit:
            if self._cycle_period_frac <= new_fracs[-1]:
                raise ValueError(
                    f"Explicit projector cycle period ({self._cycle_period_frac}) is no longer "
                    f"strictly greater than the new last timestamp ({new_fracs[-1]}). Either pass "
                    "compatible timestamps or call set_projector_cycle_period(None) first to re-infer."
                )
        else:
            self._cycle_period_frac = self._resolve_cycle_period(new_fracs, cycle_period=None)
        self._timestamps_frac = new_fracs
        self._warned_coarse_dt = False
        self._prev_sim_time = None
        if self._rect_light_prims:
            self._set_active_pattern(0)

    def get_projector_cycle_period(self) -> tuple[int, int]:
        """Return the projector cycle period as a rational tuple.

        Returns:
            Projector cycle period.
        """
        return _fraction_to_tuple(self._cycle_period_frac)

    def set_projector_cycle_period(self, period: tuple[int, int] | None) -> None:
        """Set the projector cycle period.

        Args:
            period: New cycle period as ``(numerator, denominator)``, or ``None``
                to mark the period as implicit and re-infer from the current
                timestamps.

        Raises:
            ValueError: If ``period`` is not strictly greater than the last timestamp.
        """
        self._cycle_period_frac = self._resolve_cycle_period(self._timestamps_frac, period)
        self._cycle_period_is_explicit = period is not None
        self._warned_coarse_dt = False
        self._prev_sim_time = None
