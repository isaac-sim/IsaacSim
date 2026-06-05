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

"""Authoring class for raycast sensors (USD prim creation/wrapping)."""

from __future__ import annotations

from typing import Any

import numpy as np
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import warp as wp
from pxr import Vt

from ._sensor_base import _PhysicsSensorAuthoring
from .common import _create_sensor_prim


def _validate_ray_arrays(
    ray_origins: list | np.ndarray | None,
    ray_directions: list | np.ndarray | None,
    ray_time_offsets: list | np.ndarray | None,
    *,
    require_match: bool = False,
) -> int | None:
    """Validate ray array lengths and return ``num_rays``.

    Args:
        ray_origins: Per-ray origin translations.
        ray_directions: Per-ray direction vectors.
        ray_time_offsets: Per-ray time offsets.
        require_match: If True, validate ``ray_time_offsets`` length even when
            neither ``ray_origins`` nor ``ray_directions`` is provided
            (treats ``num_rays`` as 1 when both are missing). Used by the
            create-new path. The wrap-existing path leaves ``num_rays``
            unconstrained when only time offsets are supplied.

    Returns:
        ``num_rays`` derived from origins/directions, or ``None`` when neither
        is provided and ``require_match`` is False.

    Raises:
        ValueError: If lengths are inconsistent.
    """
    if ray_origins is not None and ray_directions is not None:
        if len(ray_origins) != len(ray_directions):
            raise ValueError(
                f"ray_origins length ({len(ray_origins)}) != ray_directions length ({len(ray_directions)})"
            )

    if ray_origins is not None:
        num_rays = len(ray_origins)
    elif ray_directions is not None:
        num_rays = len(ray_directions)
    elif require_match:
        num_rays = 1
    else:
        num_rays = None

    if ray_time_offsets is not None and num_rays is not None and len(ray_time_offsets) != num_rays:
        raise ValueError(f"ray_time_offsets length ({len(ray_time_offsets)}) != num_rays ({num_rays})")

    return num_rays


class Raycast(_PhysicsSensorAuthoring):
    """Authoring wrapper for an Isaac raycast sensor USD prim.

    Creates or wraps an ``IsaacRaycastSensor`` prim. Use this class when you
    only need to author the prim (set transforms, configure ray geometry)
    without bringing up the C++ backend. For data acquisition, use
    :class:`RaycastSensor`.

    Args:
        path: USD path where the sensor should be located.
        positions: World-frame positions (shape ``(N, 3)``). Mutually exclusive with ``translations``.
        translations: Local-frame translations (shape ``(N, 3)``).
        orientations: Orientations as ``wxyz`` quaternions (shape ``(N, 4)``).
        min_range: Minimum detection range in stage length units. When wrapping
            an existing prim, applied as an override; ``None`` leaves the prim unchanged.
        max_range: Maximum detection range in stage length units. When wrapping
            an existing prim, applied as an override; ``None`` leaves the prim unchanged.
        ray_origins: Per-ray origin translations as Nx3 array. When wrapping
            an existing prim, applied as an override; ``None`` leaves the prim unchanged.
        ray_directions: Per-ray direction vectors as Nx3 array. When wrapping
            an existing prim, applied as an override; ``None`` leaves the prim unchanged.
        ray_time_offsets: Per-ray time offsets in seconds. When wrapping an
            existing prim, applied as an override; ``None`` leaves the prim unchanged.
        output_frame: Output coordinate frame (``"SENSOR"`` or ``"WORLD"``).
            When wrapping an existing prim, applied as an override; ``None``
            leaves the prim unchanged.
        report_hit_prim_paths: Whether to resolve hit prim USD paths. When
            wrapping an existing prim, applied as an override; ``None`` leaves
            the prim unchanged.

    Example:

    .. code-block:: python

        >>> from isaacsim.sensors.experimental.physics import Raycast
        >>>
        >>> raycast = Raycast.create(
        ...     "/World/Robot/body/raycast",
        ...     ray_origins=[[0, 0, 0]],
        ...     ray_directions=[[1, 0, 0]],
        ...     translations=[[0.0, 0.0, 0.0]],
        ... )  # doctest: +NO_CHECK
    """

    _PRIM_TYPE = "IsaacRaycastSensor"
    _SCHEMA_CLASS = IsaacSensorSchema.IsaacRaycastSensor
    _DEFAULT_MIN_RANGE = 0.4
    _DEFAULT_MAX_RANGE = 100.0
    _DEFAULT_OUTPUT_FRAME = "SENSOR"
    _DEFAULT_REPORT_HIT_PRIM_PATHS = False

    def __init__(
        self,
        path: str,
        *,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
        min_range: float | None = None,
        max_range: float | None = None,
        ray_origins: list | np.ndarray | None = None,
        ray_directions: list | np.ndarray | None = None,
        ray_time_offsets: list | np.ndarray | None = None,
        output_frame: str | None = None,
        report_hit_prim_paths: bool | None = None,
    ) -> None:
        # Validate ray arrays up-front (regardless of wrap-or-create)
        _validate_ray_arrays(ray_origins, ray_directions, ray_time_offsets)

        super().__init__(
            path,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
            min_range=min_range,
            max_range=max_range,
            ray_origins=ray_origins,
            ray_directions=ray_directions,
            ray_time_offsets=ray_time_offsets,
            output_frame=output_frame,
            report_hit_prim_paths=report_hit_prim_paths,
        )

    def _create_prim(
        self,
        *,
        min_range: float | None = None,
        max_range: float | None = None,
        ray_origins: list | np.ndarray | None = None,
        ray_directions: list | np.ndarray | None = None,
        ray_time_offsets: list | np.ndarray | None = None,
        output_frame: str | None = None,
        report_hit_prim_paths: bool | None = None,
        **_: Any,
    ) -> IsaacSensorSchema.IsaacRaycastSensor:
        """Create a new IsaacRaycastSensor prim with default ray attributes applied."""
        if ray_origins is None or ray_directions is None:
            raise ValueError(
                "Raycast.create requires both 'ray_origins' and 'ray_directions' "
                "(the C++ backend disables sensors whose ray-array lengths don't match numRays).",
            )

        if min_range is None:
            min_range = self._DEFAULT_MIN_RANGE
        if max_range is None:
            max_range = self._DEFAULT_MAX_RANGE
        if output_frame is None:
            output_frame = self._DEFAULT_OUTPUT_FRAME
        if report_hit_prim_paths is None:
            report_hit_prim_paths = self._DEFAULT_REPORT_HIT_PRIM_PATHS

        num_rays = _validate_ray_arrays(ray_origins, ray_directions, ray_time_offsets, require_match=True)

        prim, _path = _create_sensor_prim(
            "/" + self._sensor_name,
            self._body_prim_path,
            IsaacSensorSchema.IsaacRaycastSensor,
        )

        prim.CreateMinRangeAttr(min_range)
        prim.CreateMaxRangeAttr(max_range)

        if ray_origins is not None:
            origins = Vt.Vec3fArray([(float(o[0]), float(o[1]), float(o[2])) for o in ray_origins])
            prim.CreateRayOriginsAttr(origins)

        if ray_directions is not None:
            directions = Vt.Vec3fArray([(float(d[0]), float(d[1]), float(d[2])) for d in ray_directions])
            prim.CreateRayDirectionsAttr(directions)

        if ray_time_offsets is not None:
            offsets = Vt.FloatArray([float(t) for t in ray_time_offsets])
            prim.CreateRayTimeOffsetsAttr(offsets)

        prim.CreateNumRaysAttr(num_rays)
        prim.CreateOutputFrameOfReferenceAttr(output_frame)
        prim.CreateReportHitPrimPathsAttr(report_hit_prim_paths)

        return prim

    def _update_attributes(
        self,
        *,
        min_range: float | None = None,
        max_range: float | None = None,
        ray_origins: list | np.ndarray | None = None,
        ray_directions: list | np.ndarray | None = None,
        ray_time_offsets: list | np.ndarray | None = None,
        output_frame: str | None = None,
        report_hit_prim_paths: bool | None = None,
        **_: Any,
    ) -> None:
        """Apply user-provided attribute overrides when wrapping an existing prim."""
        if min_range is not None:
            self._isaac_sensor_prim.CreateMinRangeAttr(min_range)
        if max_range is not None:
            self._isaac_sensor_prim.CreateMaxRangeAttr(max_range)
        if ray_origins is not None or ray_directions is not None or ray_time_offsets is not None:
            if ray_origins is not None:
                origins = Vt.Vec3fArray([(float(o[0]), float(o[1]), float(o[2])) for o in ray_origins])
                self._isaac_sensor_prim.CreateRayOriginsAttr(origins)
                self._isaac_sensor_prim.CreateNumRaysAttr(len(ray_origins))
            if ray_directions is not None:
                directions = Vt.Vec3fArray([(float(d[0]), float(d[1]), float(d[2])) for d in ray_directions])
                self._isaac_sensor_prim.CreateRayDirectionsAttr(directions)
                if ray_origins is None:
                    self._isaac_sensor_prim.CreateNumRaysAttr(len(ray_directions))
            if ray_time_offsets is not None:
                offsets = Vt.FloatArray([float(t) for t in ray_time_offsets])
                self._isaac_sensor_prim.CreateRayTimeOffsetsAttr(offsets)
        if output_frame is not None:
            self._isaac_sensor_prim.CreateOutputFrameOfReferenceAttr(output_frame)
        if report_hit_prim_paths is not None:
            self._isaac_sensor_prim.CreateReportHitPrimPathsAttr(report_hit_prim_paths)
