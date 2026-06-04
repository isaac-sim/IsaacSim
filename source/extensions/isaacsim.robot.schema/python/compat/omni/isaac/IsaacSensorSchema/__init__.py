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
# ruff: noqa: N802, D107, ANN001, ANN201, ANN202, ANN204, ANN206
# Methods intentionally mirror the PascalCase generated USD schema API
# (N802), skip an __init__ docstring in favour of the class docstring
# (D107), and defer typing to the upstream pxr signatures (ANN00x/20x).
"""Drop-in compatibility wrapper for omni.isaac.IsaacSensorSchema.

Replicates the generated schema API using generic USD attribute access,
so that existing callsites continue to work without changes.
"""

import warnings

from pxr import Sdf, Tf, Usd


class IsaacBaseSensor:
    """Compatibility wrapper for pxr::IsaacSensorIsaacBaseSensor."""

    _TYPE_NAME = "IsaacBaseSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacBaseSensor"

    def __init__(self, prim):
        if isinstance(prim, Usd.Prim):
            self._prim = prim
        elif hasattr(prim, "GetPrim"):
            self._prim = prim.GetPrim()
        else:
            self._prim = prim

    @classmethod
    def Define(cls, stage: Usd.Stage, path: str):
        """Create the underlying prim on `stage` at `path` and return the wrapper."""
        prim = stage.DefinePrim(path, cls._TYPE_NAME)
        return cls(prim)

    @classmethod
    def _GetStaticTfType(cls):
        """Return the TfType used by `Usd.Prim.HasAPI` when passed this class."""
        return Tf.Type.FindByName(cls._TF_TYPE_NAME)

    def GetPrim(self) -> Usd.Prim:
        """Return the wrapped USD prim."""
        return self._prim

    def GetPath(self):
        """Return the wrapped prim's `Sdf.Path`."""
        return self._prim.GetPath()

    # --- IsaacBaseSensor attributes ---

    def GetEnabledAttr(self):
        """Return the `enabled` bool attribute (may be invalid if unauthored)."""
        return self._prim.GetAttribute("enabled")

    def CreateEnabledAttr(self, value=None):
        """Create the `enabled` bool attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("enabled", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr

    def GetSensorPeriodAttr(self):
        """Return the `sensorPeriod` float attribute.

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

    def CreateSensorPeriodAttr(self, value=None):
        """Create the `sensorPeriod` float attribute and optionally set `value`.

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

    def GetThresholdAttr(self):
        """Return the `threshold` float2 attribute (min, max contact threshold)."""
        return self._prim.GetAttribute("threshold")

    def CreateThresholdAttr(self, value=None):
        """Create the `threshold` float2 attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("threshold", Sdf.ValueTypeNames.Float2)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRadiusAttr(self):
        """Return the `radius` float attribute (contact sphere radius)."""
        return self._prim.GetAttribute("radius")

    def CreateRadiusAttr(self, value=None):
        """Create the `radius` float attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("radius", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetColorAttr(self):
        """Return the `color` float4 attribute (sensor debug draw color)."""
        return self._prim.GetAttribute("color")

    def CreateColorAttr(self, value=None):
        """Create the `color` float4 attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("color", Sdf.ValueTypeNames.Float4)
        if value is not None:
            attr.Set(value)
        return attr


class IsaacImuSensor(IsaacBaseSensor):
    """Compatibility wrapper for pxr::IsaacSensorIsaacImuSensor."""

    _TYPE_NAME = "IsaacImuSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacImuSensor"

    # --- IsaacImuSensor attributes ---

    def GetLinearAccelerationFilterWidthAttr(self):
        """Return the `linearAccelerationFilterWidth` int attribute."""
        return self._prim.GetAttribute("linearAccelerationFilterWidth")

    def CreateLinearAccelerationFilterWidthAttr(self, value=None):
        """Create the `linearAccelerationFilterWidth` int attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("linearAccelerationFilterWidth", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr

    def GetAngularVelocityFilterWidthAttr(self):
        """Return the `angularVelocityFilterWidth` int attribute."""
        return self._prim.GetAttribute("angularVelocityFilterWidth")

    def CreateAngularVelocityFilterWidthAttr(self, value=None):
        """Create the `angularVelocityFilterWidth` int attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("angularVelocityFilterWidth", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr

    def GetOrientationFilterWidthAttr(self):
        """Return the `orientationFilterWidth` int attribute."""
        return self._prim.GetAttribute("orientationFilterWidth")

    def CreateOrientationFilterWidthAttr(self, value=None):
        """Create the `orientationFilterWidth` int attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("orientationFilterWidth", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr


class IsaacLightBeamSensor(IsaacBaseSensor):
    """Compatibility wrapper for pxr::IsaacSensorIsaacLightBeamSensor.

    .. deprecated:: 6.2.0
        Use ``IsaacRaycastSensor`` with ``isaacsim.sensors.experimental.physics`` instead.
    """

    _TYPE_NAME = "IsaacLightBeamSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacLightBeamSensor"

    def __init__(self, prim):
        warnings.warn(
            "IsaacLightBeamSensor is deprecated since isaacsim.robot.schema 6.2.0. "
            "Use IsaacRaycastSensor with isaacsim.sensors.experimental.physics instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(prim)

    # ── IsaacLightBeamSensor attributes ─────────────────────────────────

    def GetNumRaysAttr(self):
        """Return the `numRays` int attribute (number of rays cast per frame)."""
        return self._prim.GetAttribute("numRays")

    def CreateNumRaysAttr(self, value=None):
        """Create the `numRays` int attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("numRays", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr

    def GetCurtainLengthAttr(self):
        """Return the `curtainLength` float attribute (extent of the light curtain)."""
        return self._prim.GetAttribute("curtainLength")

    def CreateCurtainLengthAttr(self, value=None):
        """Create the `curtainLength` float attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("curtainLength", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetForwardAxisAttr(self):
        """Return the `forwardAxis` float3 attribute (principal ray direction)."""
        return self._prim.GetAttribute("forwardAxis")

    def CreateForwardAxisAttr(self, value=None):
        """Create the `forwardAxis` float3 attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("forwardAxis", Sdf.ValueTypeNames.Float3)
        if value is not None:
            attr.Set(value)
        return attr

    def GetCurtainAxisAttr(self):
        """Return the `curtainAxis` float3 attribute (curtain spread direction)."""
        return self._prim.GetAttribute("curtainAxis")

    def CreateCurtainAxisAttr(self, value=None):
        """Create the `curtainAxis` float3 attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("curtainAxis", Sdf.ValueTypeNames.Float3)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMinRangeAttr(self):
        """Return the `minRange` float attribute (near clip of the sensor)."""
        return self._prim.GetAttribute("minRange")

    def CreateMinRangeAttr(self, value=None):
        """Create the `minRange` float attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMaxRangeAttr(self):
        """Return the `maxRange` float attribute (far clip of the sensor)."""
        return self._prim.GetAttribute("maxRange")

    def CreateMaxRangeAttr(self, value=None):
        """Create the `maxRange` float attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr


class IsaacRaycastSensor(IsaacBaseSensor):
    """Compatibility wrapper for pxr::IsaacSensorIsaacRaycastSensor."""

    _TYPE_NAME = "IsaacRaycastSensor"
    _TF_TYPE_NAME = "IsaacSensorIsaacRaycastSensor"

    # ── IsaacRaycastSensor attributes ───────────────────────────────────

    def GetNumRaysAttr(self):
        """Return the `numRays` uint attribute (number of rays cast per frame)."""
        return self._prim.GetAttribute("numRays")

    def CreateNumRaysAttr(self, value=None):
        """Create the `numRays` uint attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("numRays", Sdf.ValueTypeNames.UInt)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMinRangeAttr(self):
        """Return the `minRange` float attribute (near clip of the sensor)."""
        return self._prim.GetAttribute("minRange")

    def CreateMinRangeAttr(self, value=None):
        """Create the `minRange` float attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMaxRangeAttr(self):
        """Return the `maxRange` float attribute (far clip of the sensor)."""
        return self._prim.GetAttribute("maxRange")

    def CreateMaxRangeAttr(self, value=None):
        """Create the `maxRange` float attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRayOriginsAttr(self):
        """Return the `rayOrigins` float3 array attribute (per-ray origin points)."""
        return self._prim.GetAttribute("rayOrigins")

    def CreateRayOriginsAttr(self, value=None):
        """Create the `rayOrigins` float3 array attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("rayOrigins", Sdf.ValueTypeNames.Float3Array)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRayDirectionsAttr(self):
        """Return the `rayDirections` float3 array attribute (per-ray direction vectors)."""
        return self._prim.GetAttribute("rayDirections")

    def CreateRayDirectionsAttr(self, value=None):
        """Create the `rayDirections` float3 array attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("rayDirections", Sdf.ValueTypeNames.Float3Array)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRayTimeOffsetsAttr(self):
        """Return the `rayTimeOffsets` float array attribute (per-ray sub-frame time offsets)."""
        return self._prim.GetAttribute("rayTimeOffsets")

    def CreateRayTimeOffsetsAttr(self, value=None):
        """Create the `rayTimeOffsets` float array attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("rayTimeOffsets", Sdf.ValueTypeNames.FloatArray)
        if value is not None:
            attr.Set(value)
        return attr

    def GetOutputFrameOfReferenceAttr(self):
        """Return the `outputFrameOfReference` token attribute (output coordinate frame)."""
        return self._prim.GetAttribute("outputFrameOfReference")

    def CreateOutputFrameOfReferenceAttr(self, value=None):
        """Create the `outputFrameOfReference` token attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("outputFrameOfReference", Sdf.ValueTypeNames.Token)
        if value is not None:
            attr.Set(value)
        return attr

    def GetReportHitPrimPathsAttr(self):
        """Return the `reportHitPrimPaths` bool attribute (emit hit prim paths in output)."""
        return self._prim.GetAttribute("reportHitPrimPaths")

    def CreateReportHitPrimPathsAttr(self, value=None):
        """Create the `reportHitPrimPaths` bool attribute and optionally set `value`."""
        attr = self._prim.CreateAttribute("reportHitPrimPaths", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr


class _APISchemaWrapper:
    """Base for API schema compatibility wrappers.

    Supports ``SchemaClass.Apply(prim)`` and ``prim.HasAPI(SchemaClass)``
    by exposing a ``_GetStaticTfType()`` classmethod that USD's ``HasAPI``
    dispatches to when passed a Python type instead of a ``TfType``.
    """

    _SCHEMA_NAME = ""

    def __init__(self, prim):
        if isinstance(prim, Usd.Prim):
            self._prim = prim
        elif hasattr(prim, "GetPrim"):
            self._prim = prim.GetPrim()
        else:
            self._prim = prim

    @classmethod
    def Apply(cls, prim: Usd.Prim):
        """Apply `_SCHEMA_NAME` to `prim` and return a wrapper bound to it."""
        prim.AddAppliedSchema(cls._SCHEMA_NAME)
        return cls(prim)

    def GetPrim(self) -> Usd.Prim:
        """Return the wrapped USD prim."""
        return self._prim

    def GetPath(self):
        """Return the wrapped prim's `Sdf.Path`."""
        return self._prim.GetPath()

    @classmethod
    def _GetStaticTfType(cls):
        """Return the TfType used by `Usd.Prim.HasAPI` when passed this class."""
        return Tf.Type.FindByName(cls._TF_TYPE_NAME)


class IsaacRtxLidarSensorAPI(_APISchemaWrapper):
    """Compatibility wrapper for pxr::IsaacSensorIsaacRtxLidarSensorAPI."""

    _SCHEMA_NAME = "IsaacRtxLidarSensorAPI"
    _TF_TYPE_NAME = "IsaacSensorIsaacRtxLidarSensorAPI"


class IsaacRtxRadarSensorAPI(_APISchemaWrapper):
    """Compatibility wrapper for pxr::IsaacSensorIsaacRtxRadarSensorAPI."""

    _SCHEMA_NAME = "IsaacRtxRadarSensorAPI"
    _TF_TYPE_NAME = "IsaacSensorIsaacRtxRadarSensorAPI"
