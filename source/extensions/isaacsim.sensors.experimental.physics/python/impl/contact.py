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

"""Authoring class for contact sensors (USD prim creation/wrapping)."""

from __future__ import annotations

from typing import Any

import numpy as np
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.usd
import warp as wp
from isaacsim.core.experimental.utils import prim as prim_utils
from pxr import Gf, PhysxSchema, UsdPhysics

from ._sensor_base import _PhysicsSensorAuthoring
from .common import _create_sensor_prim


class Contact(_PhysicsSensorAuthoring):
    """Authoring wrapper for an Isaac contact sensor USD prim.

    Creates or wraps an ``IsaacContactSensor`` prim. Use this class when you
    only need to author the prim (set transforms, configure thresholds and
    radius) without bringing up the C++ backend. For data acquisition, use
    :class:`ContactSensor`.

    Args:
        path: USD path where the sensor should be located.
        positions: World-frame positions (shape ``(N, 3)``). Mutually exclusive with ``translations``.
        translations: Local-frame translations (shape ``(N, 3)``).
        orientations: Orientations as ``wxyz`` quaternions (shape ``(N, 4)``).
        min_threshold: Minimum force threshold in Newtons. When wrapping an
            existing prim, applied as an override; ``None`` leaves the prim
            unchanged.
        max_threshold: Maximum force threshold in Newtons. When wrapping an
            existing prim, applied as an override; ``None`` leaves the prim
            unchanged.
        radius: Contact detection radius. Negative means no radius filtering.
            When wrapping an existing prim, applied as an override; ``None``
            leaves the prim unchanged.
        color: Sensor visualization color as RGBA. Applied only when creating
            a new prim.

    Raises:
        ValueError: If the sensor does not have an enabled rigid-body ancestor.

    Example:

    .. code-block:: python

        >>> from isaacsim.sensors.experimental.physics import Contact
        >>>
        >>> contact = Contact.create(
        ...     "/World/Robot/foot/contact_sensor",
        ...     min_threshold=1.0,
        ...     max_threshold=1000.0,
        ...     translations=[[0.0, 0.0, 0.0]],
        ... )  # doctest: +NO_CHECK
    """

    _PRIM_TYPE = "IsaacContactSensor"
    _SCHEMA_CLASS = IsaacSensorSchema.IsaacContactSensor
    _DEFAULT_MIN_THRESHOLD = 0.0
    _DEFAULT_MAX_THRESHOLD = 100000.0
    _DEFAULT_RADIUS = -1.0

    def __init__(
        self,
        path: str,
        *,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
        min_threshold: float | None = None,
        max_threshold: float | None = None,
        radius: float | None = None,
        color: Gf.Vec4f = Gf.Vec4f(1, 1, 1, 1),
    ) -> None:
        super().__init__(
            path,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
            min_threshold=min_threshold,
            max_threshold=max_threshold,
            radius=radius,
            color=color,
        )

    def _find_physics_parent(self) -> str:
        """Walk ancestors and return the nearest C++ runtime-compatible rigid body path.

        Raises:
            ValueError: If no ancestor matches the C++ backend's
            ``findParentRigidBody`` criteria. The sensor would otherwise
            silently never produce valid readings.
        """
        parent_path = self._body_prim_path
        while parent_path and parent_path != "/":
            prim = prim_utils.get_prim_at_path(parent_path)
            if not prim.IsValid():
                parent_path = "/".join(parent_path.split("/")[:-1])
                continue
            rigid_body_enabled_attr = prim.GetAttribute("physics:rigidBodyEnabled")
            rigid_body_enabled_attr_valid = rigid_body_enabled_attr.IsValid()

            if rigid_body_enabled_attr_valid and rigid_body_enabled_attr.Get():
                return parent_path
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not rigid_body_enabled_attr_valid:
                return parent_path
            parent_path = "/".join(parent_path.split("/")[:-1])
        raise ValueError("Contact Sensor needs to be created under an enabled rigid-body prim.")

    def _on_existing_prim(self, prim: Any, **_: Any) -> None:
        """Pin ``_body_prim_path`` to the nearest runtime-compatible rigid-body ancestor.

        Used by the wrap-existing-prim path so subsequent attribute updates
        target the rigid-body ancestor instead of an intermediate Xform.
        """
        self._body_prim_path = self._find_physics_parent()

    def _create_prim(
        self,
        *,
        min_threshold: float | None = None,
        max_threshold: float | None = None,
        radius: float | None = None,
        color: Gf.Vec4f = Gf.Vec4f(1, 1, 1, 1),
        **_: Any,
    ) -> IsaacSensorSchema.IsaacContactSensor:
        """Create a new IsaacContactSensor prim with default attributes applied."""
        # The C++ backend requires a runtime-compatible rigid-body ancestor for
        # contact reporting to function. Validate at create time so we don't
        # silently produce a sensor that never returns valid readings. Don't
        # mutate _body_prim_path: the sensor is created at the user's requested
        # path, even when the rigid body lives further up.
        physics_parent_path = self._find_physics_parent()

        if min_threshold is None:
            min_threshold = self._DEFAULT_MIN_THRESHOLD
        if max_threshold is None:
            max_threshold = self._DEFAULT_MAX_THRESHOLD
        if radius is None:
            radius = self._DEFAULT_RADIUS

        prim, _path = _create_sensor_prim(
            "/" + self._sensor_name,
            self._body_prim_path,
            IsaacSensorSchema.IsaacContactSensor,
        )
        prim.CreateThresholdAttr().Set((min_threshold, max_threshold))
        prim.CreateColorAttr().Set(color)
        prim.CreateRadiusAttr().Set(radius)

        # Apply PhysxContactReportAPI to the parent body so contacts are reported.
        stage = omni.usd.get_context().get_stage()
        parent_prim = stage.GetPrimAtPath(physics_parent_path)
        contact_report = PhysxSchema.PhysxContactReportAPI.Apply(parent_prim)
        contact_report.CreateThresholdAttr(min_threshold)

        return prim

    def _update_attributes(
        self,
        *,
        min_threshold: float | None = None,
        max_threshold: float | None = None,
        radius: float | None = None,
        **_: Any,
    ) -> None:
        """Apply user-provided attribute overrides when wrapping an existing prim."""
        if min_threshold is not None:
            self.set_min_threshold(min_threshold)
        if max_threshold is not None:
            self.set_max_threshold(max_threshold)
        if radius is not None:
            self.set_radius(radius)

    def get_radius(self) -> float | None:
        """Get the contact detection radius.

        Returns:
            Radius in stage units. Negative means no radius filtering.
        """
        return self._prim.GetAttribute("radius").Get()

    def set_radius(self, value: float) -> None:
        """Set the contact detection radius.

        Args:
            value: Radius in stage units. Use negative to disable radius filtering.
        """
        if self.get_radius() is None:
            self._isaac_sensor_prim.CreateRadiusAttr().Set(value)
        else:
            self._prim.GetAttribute("radius").Set(value)

    def get_min_threshold(self) -> float | None:
        """Minimum force threshold in Newtons.

        Returns:
            Minimum threshold in Newtons, or ``None`` if not set.
        """
        threshold = self._prim.GetAttribute("threshold").Get()
        if threshold is not None:
            return threshold[0]
        return None

    def set_min_threshold(self, value: float) -> None:
        """Set the minimum force threshold.

        Contacts with force below this threshold are ignored.

        Args:
            value: Threshold in Newtons.
        """
        if self.get_min_threshold() is None:
            self._isaac_sensor_prim.CreateThresholdAttr().Set((value, self._DEFAULT_MAX_THRESHOLD))
        else:
            self._prim.GetAttribute("threshold").Set((value, self.get_max_threshold()))

    def get_max_threshold(self) -> float | None:
        """Maximum force threshold in Newtons.

        Returns:
            Maximum threshold in Newtons, or ``None`` if not set.
        """
        threshold = self._prim.GetAttribute("threshold").Get()
        if threshold is not None:
            return threshold[1]
        return None

    def set_max_threshold(self, value: float) -> None:
        """Set the maximum force threshold.

        Contact forces are clamped to this maximum value.

        Args:
            value: Threshold in Newtons.
        """
        if self.get_max_threshold() is None:
            self._isaac_sensor_prim.CreateThresholdAttr().Set((self._DEFAULT_MIN_THRESHOLD, value))
        else:
            self._prim.GetAttribute("threshold").Set((self.get_min_threshold(), value))
