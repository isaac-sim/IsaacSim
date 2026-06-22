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
# ruff: noqa: N802
# Methods intentionally mirror the PascalCase generated USD schema API
# (N802)
"""Drop-in compatibility wrapper for omni.isaac.IsaacSensorSchema.

Replicates the generated schema API using generic USD attribute access,
so that existing callsites continue to work without changes.
"""

import warnings
from typing import Any

from pxr import Sdf, Tf, Usd


class IsaacBaseSensor:
    """Compatibility wrapper for pxr::IsaacSensorIsaacBaseSensor.

    Args:
        prim: Prim or prim-like object to wrap.
    """

    _TYPE_NAME = "IsaacBaseSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacBaseSensor"

    def __init__(self, prim: Usd.Prim) -> None:
        if isinstance(prim, Usd.Prim):
            self._prim = prim
        elif hasattr(prim, "GetPrim"):
            self._prim = prim.GetPrim()
        else:
            self._prim = prim

    @classmethod
    def Define(cls, stage: Usd.Stage, path: str) -> "IsaacBaseSensor":
        """Create the underlying prim on `stage` at `path` and return the wrapper.

        Args:
            stage: Stage that receives the new prim.
            path: Path where the prim is defined.

        Returns:
            Wrapper bound to the defined prim.
        """
        prim = stage.DefinePrim(path, cls._TYPE_NAME)
        return cls(prim)

    @classmethod
    def _GetStaticTfType(cls) -> Tf.Type:
        """Return the TfType used by `Usd.Prim.HasAPI` when passed this class.

        Returns:
            Tf type used by USD schema checks.
        """
        return Tf.Type.FindByName(cls._TF_TYPE_NAME)

    def GetPrim(self) -> Usd.Prim:
        """Return the wrapped USD prim.

        Returns:
            Wrapped USD prim.
        """
        return self._prim

    def GetPath(self) -> Sdf.Path:
        """Return the wrapped prim's `Sdf.Path`.

        Returns:
            Wrapped prim path.
        """
        return self._prim.GetPath()

    # --- IsaacBaseSensor attributes ---

    def GetEnabledAttr(self) -> Usd.Attribute:
        """Return the `enabled` bool attribute (may be invalid if unauthored).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("enabled")

    def CreateEnabledAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `enabled` bool attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("enabled", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr

    def GetSensorPeriodAttr(self) -> Usd.Attribute:
        """Return the `sensorPeriod` float attribute.

        Returns:
            Requested USD attribute.

        .. deprecated:: 6.2.0
            Only used by the deprecated ``isaacsim.sensors.physx`` extension.
        """
        warnings.warn(
            "sensorPeriod is deprecated since isaacsim.robot.schema 6.2.0. "
            "It is only used by the deprecated isaacsim.sensors.physx extension.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self._prim.GetAttribute("sensorPeriod")

    def CreateSensorPeriodAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `sensorPeriod` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.

        .. deprecated:: 6.2.0
            Only used by the deprecated ``isaacsim.sensors.physx`` extension.
        """
        warnings.warn(
            "sensorPeriod is deprecated since isaacsim.robot.schema 6.2.0. "
            "It is only used by the deprecated isaacsim.sensors.physx extension.",
            DeprecationWarning,
            stacklevel=2,
        )
        attr = self._prim.CreateAttribute("sensorPeriod", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr


class IsaacContactSensor(IsaacBaseSensor):
    """Compatibility wrapper for pxr::IsaacSensorIsaacContactSensor."""

    _TYPE_NAME = "IsaacContactSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacContactSensor"

    # --- IsaacContactSensor attributes ---

    def GetThresholdAttr(self) -> Usd.Attribute:
        """Return the `threshold` float2 attribute (min, max contact threshold).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("threshold")

    def CreateThresholdAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `threshold` float2 attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("threshold", Sdf.ValueTypeNames.Float2)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRadiusAttr(self) -> Usd.Attribute:
        """Return the `radius` float attribute (contact sphere radius).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("radius")

    def CreateRadiusAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `radius` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("radius", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetColorAttr(self) -> Usd.Attribute:
        """Return the `color` float4 attribute (sensor debug draw color).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("color")

    def CreateColorAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `color` float4 attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("color", Sdf.ValueTypeNames.Float4)
        if value is not None:
            attr.Set(value)
        return attr


class IsaacImuSensor(IsaacBaseSensor):
    """Compatibility wrapper for pxr::IsaacSensorIsaacImuSensor."""

    _TYPE_NAME = "IsaacImuSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacImuSensor"

    # --- IsaacImuSensor attributes ---

    def GetLinearAccelerationFilterWidthAttr(self) -> Usd.Attribute:
        """Return the `linearAccelerationFilterWidth` int attribute.

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("linearAccelerationFilterWidth")

    def CreateLinearAccelerationFilterWidthAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `linearAccelerationFilterWidth` int attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("linearAccelerationFilterWidth", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr

    def GetAngularVelocityFilterWidthAttr(self) -> Usd.Attribute:
        """Return the `angularVelocityFilterWidth` int attribute.

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("angularVelocityFilterWidth")

    def CreateAngularVelocityFilterWidthAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `angularVelocityFilterWidth` int attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("angularVelocityFilterWidth", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr

    def GetOrientationFilterWidthAttr(self) -> Usd.Attribute:
        """Return the `orientationFilterWidth` int attribute.

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("orientationFilterWidth")

    def CreateOrientationFilterWidthAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `orientationFilterWidth` int attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("orientationFilterWidth", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr


class IsaacLightBeamSensor(IsaacBaseSensor):
    """Compatibility wrapper for pxr::IsaacSensorIsaacLightBeamSensor.

    Args:
        prim: Prim or prim-like object to wrap.

    .. deprecated:: 6.2.0
        Use ``IsaacRaycastSensor`` with ``isaacsim.sensors.experimental.physics`` instead.
    """

    _TYPE_NAME = "IsaacLightBeamSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacLightBeamSensor"

    def __init__(self, prim: Usd.Prim) -> None:
        warnings.warn(
            "IsaacLightBeamSensor is deprecated since isaacsim.robot.schema 6.2.0. "
            "Use IsaacRaycastSensor with isaacsim.sensors.experimental.physics instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(prim)

    # ── IsaacLightBeamSensor attributes ─────────────────────────────────

    def GetNumRaysAttr(self) -> Usd.Attribute:
        """Return the `numRays` int attribute (number of rays cast per frame).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("numRays")

    def CreateNumRaysAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `numRays` int attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("numRays", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr

    def GetCurtainLengthAttr(self) -> Usd.Attribute:
        """Return the `curtainLength` float attribute (extent of the light curtain).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("curtainLength")

    def CreateCurtainLengthAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `curtainLength` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("curtainLength", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetForwardAxisAttr(self) -> Usd.Attribute:
        """Return the `forwardAxis` float3 attribute (principal ray direction).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("forwardAxis")

    def CreateForwardAxisAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `forwardAxis` float3 attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("forwardAxis", Sdf.ValueTypeNames.Float3)
        if value is not None:
            attr.Set(value)
        return attr

    def GetCurtainAxisAttr(self) -> Usd.Attribute:
        """Return the `curtainAxis` float3 attribute (curtain spread direction).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("curtainAxis")

    def CreateCurtainAxisAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `curtainAxis` float3 attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("curtainAxis", Sdf.ValueTypeNames.Float3)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMinRangeAttr(self) -> Usd.Attribute:
        """Return the `minRange` float attribute (near clip of the sensor).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("minRange")

    def CreateMinRangeAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `minRange` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMaxRangeAttr(self) -> Usd.Attribute:
        """Return the `maxRange` float attribute (far clip of the sensor).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("maxRange")

    def CreateMaxRangeAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `maxRange` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr


class IsaacRaycastSensor(IsaacBaseSensor):
    """Compatibility wrapper for pxr::IsaacSensorIsaacRaycastSensor."""

    _TYPE_NAME = "IsaacRaycastSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacRaycastSensor"

    # ── IsaacRaycastSensor attributes ───────────────────────────────────

    def GetNumRaysAttr(self) -> Usd.Attribute:
        """Return the `numRays` uint attribute (number of rays cast per frame).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("numRays")

    def CreateNumRaysAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `numRays` uint attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("numRays", Sdf.ValueTypeNames.UInt)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMinRangeAttr(self) -> Usd.Attribute:
        """Return the `minRange` float attribute (near clip of the sensor).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("minRange")

    def CreateMinRangeAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `minRange` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMaxRangeAttr(self) -> Usd.Attribute:
        """Return the `maxRange` float attribute (far clip of the sensor).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("maxRange")

    def CreateMaxRangeAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `maxRange` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRayOriginsAttr(self) -> Usd.Attribute:
        """Return the `rayOrigins` float3 array attribute (per-ray origin points).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("rayOrigins")

    def CreateRayOriginsAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `rayOrigins` float3 array attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("rayOrigins", Sdf.ValueTypeNames.Float3Array)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRayDirectionsAttr(self) -> Usd.Attribute:
        """Return the `rayDirections` float3 array attribute (per-ray direction vectors).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("rayDirections")

    def CreateRayDirectionsAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `rayDirections` float3 array attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("rayDirections", Sdf.ValueTypeNames.Float3Array)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRayTimeOffsetsAttr(self) -> Usd.Attribute:
        """Return the `rayTimeOffsets` float array attribute (per-ray sub-frame time offsets).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("rayTimeOffsets")

    def CreateRayTimeOffsetsAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `rayTimeOffsets` float array attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("rayTimeOffsets", Sdf.ValueTypeNames.FloatArray)
        if value is not None:
            attr.Set(value)
        return attr

    def GetOutputFrameOfReferenceAttr(self) -> Usd.Attribute:
        """Return the `outputFrameOfReference` token attribute (output coordinate frame).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("outputFrameOfReference")

    def CreateOutputFrameOfReferenceAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `outputFrameOfReference` token attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("outputFrameOfReference", Sdf.ValueTypeNames.Token)
        if value is not None:
            attr.Set(value)
        return attr

    def GetReportHitPrimPathsAttr(self) -> Usd.Attribute:
        """Return the `reportHitPrimPaths` bool attribute (emit hit prim paths in output).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("reportHitPrimPaths")

    def CreateReportHitPrimPathsAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `reportHitPrimPaths` bool attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("reportHitPrimPaths", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr


class _APISchemaWrapper:
    """Base for API schema compatibility wrappers.

    Supports ``SchemaClass.Apply(prim)`` and ``prim.HasAPI(SchemaClass)``
    by exposing a ``_GetStaticTfType()`` classmethod that USD's ``HasAPI``
    dispatches to when passed a Python type instead of a ``TfType``.

    Args:
        prim: Prim or prim-like object to wrap.
    """

    _SCHEMA_NAME = ""

    def __init__(self, prim: Usd.Prim) -> None:
        if isinstance(prim, Usd.Prim):
            self._prim = prim
        elif hasattr(prim, "GetPrim"):
            self._prim = prim.GetPrim()
        else:
            self._prim = prim

    @classmethod
    def Apply(cls, prim: Usd.Prim) -> "_APISchemaWrapper":
        """Apply `_SCHEMA_NAME` to `prim` and return a wrapper bound to it.

        Args:
            prim: Prim to apply the schema to.

        Returns:
            Wrapper bound to the prim.
        """
        prim.AddAppliedSchema(cls._SCHEMA_NAME)
        return cls(prim)

    def GetPrim(self) -> Usd.Prim:
        """Return the wrapped USD prim.

        Returns:
            Wrapped USD prim.
        """
        return self._prim

    def GetPath(self) -> Sdf.Path:
        """Return the wrapped prim's `Sdf.Path`.

        Returns:
            Wrapped prim path.
        """
        return self._prim.GetPath()

    @classmethod
    def _GetStaticTfType(cls) -> Tf.Type:
        """Return the TfType used by `Usd.Prim.HasAPI` when passed this class.

        Returns:
            Tf type used by USD schema checks.
        """
        return Tf.Type.FindByName(cls._TF_TYPE_NAME)


class IsaacRtxLidarSensorAPI(_APISchemaWrapper):
    """Compatibility wrapper for pxr::IsaacSensorIsaacRtxLidarSensorAPI."""

    _SCHEMA_NAME = "IsaacRtxLidarSensorAPI"
    _TF_TYPE_NAME = "IsaacSensorIsaacRtxLidarSensorAPI"


class IsaacRtxRadarSensorAPI(_APISchemaWrapper):
    """Compatibility wrapper for pxr::IsaacSensorIsaacRtxRadarSensorAPI."""

    _SCHEMA_NAME = "IsaacRtxRadarSensorAPI"
    _TF_TYPE_NAME = "IsaacSensorIsaacRtxRadarSensorAPI"
