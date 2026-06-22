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
"""Drop-in compatibility wrapper for omni.isaac.RangeSensorSchema.

.. deprecated:: 6.2.0
    The RangeSensor, Lidar, and Generic schemas are deprecated.
    Use ``IsaacRaycastSensor`` with ``isaacsim.sensors.experimental.physics``
    or ``isaacsim.sensors.experimental.rtx`` instead.
"""

import warnings
from typing import Any

from pxr import Sdf, Tf, Usd

warnings.warn(
    "omni.isaac.RangeSensorSchema is deprecated since isaacsim.robot.schema 6.2.0. "
    "The RangeSensor, Lidar, and Generic schemas are deprecated. "
    "Use IsaacRaycastSensor with isaacsim.sensors.experimental.physics "
    "or isaacsim.sensors.experimental.rtx instead.",
    DeprecationWarning,
    stacklevel=2,
)


class RangeSensor:
    """Compatibility wrapper for pxr::RangeSensorRangeSensor.

    Args:
        prim: Prim or prim-like object to wrap.
    """

    _TYPE_NAME = "RangeSensor"
    _TF_TYPE_NAME = "RangeSensorRangeSensor"

    def __init__(self, prim: Usd.Prim) -> None:
        if isinstance(prim, Usd.Prim):
            self._prim = prim
        elif hasattr(prim, "GetPrim"):
            self._prim = prim.GetPrim()
        else:
            self._prim = prim

    @classmethod
    def Define(cls, stage: Usd.Stage, path: str) -> "RangeSensor":
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

    # --- RangeSensor attributes ---

    def GetEnabledAttr(self) -> Usd.Attribute:
        """Return the `enabled` bool attribute.

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

    def GetDrawPointsAttr(self) -> Usd.Attribute:
        """Return the `drawPoints` bool attribute (debug-draw hit points).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("drawPoints")

    def CreateDrawPointsAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `drawPoints` bool attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("drawPoints", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr

    def GetDrawLinesAttr(self) -> Usd.Attribute:
        """Return the `drawLines` bool attribute (debug-draw rays).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("drawLines")

    def CreateDrawLinesAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `drawLines` bool attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("drawLines", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr

    def GetMinRangeAttr(self) -> Usd.Attribute:
        """Return the `minRange` float attribute (near clip).

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
        """Return the `maxRange` float attribute (far clip).

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


class Lidar(RangeSensor):
    """Compatibility wrapper for pxr::RangeSensorLidar."""

    _TYPE_NAME = "Lidar"
    _TF_TYPE_NAME = "RangeSensorLidar"

    # --- Lidar-specific attributes ---

    def GetYawOffsetAttr(self) -> Usd.Attribute:
        """Return the `yawOffset` float attribute (azimuthal starting angle).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("yawOffset")

    def CreateYawOffsetAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `yawOffset` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("yawOffset", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetRotationRateAttr(self) -> Usd.Attribute:
        """Return the `rotationRate` float attribute (revolutions per second).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("rotationRate")

    def CreateRotationRateAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `rotationRate` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("rotationRate", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetHighLodAttr(self) -> Usd.Attribute:
        """Return the `highLod` bool attribute (high-resolution toggle).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("highLod")

    def CreateHighLodAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `highLod` bool attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("highLod", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr

    def GetHorizontalFovAttr(self) -> Usd.Attribute:
        """Return the `horizontalFov` float attribute (degrees).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("horizontalFov")

    def CreateHorizontalFovAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `horizontalFov` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("horizontalFov", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetVerticalFovAttr(self) -> Usd.Attribute:
        """Return the `verticalFov` float attribute (degrees).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("verticalFov")

    def CreateVerticalFovAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `verticalFov` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("verticalFov", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetHorizontalResolutionAttr(self) -> Usd.Attribute:
        """Return the `horizontalResolution` float attribute (degrees per sample).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("horizontalResolution")

    def CreateHorizontalResolutionAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `horizontalResolution` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("horizontalResolution", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetVerticalResolutionAttr(self) -> Usd.Attribute:
        """Return the `verticalResolution` float attribute (degrees per sample).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("verticalResolution")

    def CreateVerticalResolutionAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `verticalResolution` float attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("verticalResolution", Sdf.ValueTypeNames.Float)
        if value is not None:
            attr.Set(value)
        return attr

    def GetEnableSemanticsAttr(self) -> Usd.Attribute:
        """Return the `enableSemantics` bool attribute (emit semantic labels).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("enableSemantics")

    def CreateEnableSemanticsAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `enableSemantics` bool attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("enableSemantics", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr


class Generic(RangeSensor):
    """Compatibility wrapper for pxr::RangeSensorGeneric."""

    _TYPE_NAME = "Generic"
    _TF_TYPE_NAME = "RangeSensorGeneric"

    # --- Generic-specific attributes ---

    def GetSamplingRateAttr(self) -> Usd.Attribute:
        """Return the `samplingRate` int attribute (samples per second).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("samplingRate")

    def CreateSamplingRateAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `samplingRate` int attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("samplingRate", Sdf.ValueTypeNames.Int)
        if value is not None:
            attr.Set(value)
        return attr

    def GetStreamingAttr(self) -> Usd.Attribute:
        """Return the `streaming` bool attribute (continuous output toggle).

        Returns:
            Requested USD attribute.
        """
        return self._prim.GetAttribute("streaming")

    def CreateStreamingAttr(self, value: Any = None) -> Usd.Attribute:
        """Create the `streaming` bool attribute and optionally set `value`.

        Args:
            value: Value to set on the attribute.

        Returns:
            Created USD attribute.
        """
        attr = self._prim.CreateAttribute("streaming", Sdf.ValueTypeNames.Bool)
        if value is not None:
            attr.Set(value)
        return attr
