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

"""Data types for 2D and 3D pose representation and sensor configuration."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Point2d:
    """A 2D point with x and y coordinates."""

    x: float
    y: float


@dataclass
class Pose2d(Point2d):
    """A 2D pose with position (x, y) and heading angle theta."""

    theta: float


@dataclass
class Pose3d:
    """A 3D pose with position and orientation arrays."""

    position: np.ndarray
    orientation: np.ndarray


# =========================================================
#  Sensor configuration
# =========================================================


@dataclass
class CameraConfig:
    """Configuration for a single camera sensor attached to an existing USD prim.

    The camera prim must already exist in the loaded robot USD (e.g. a ``UsdGeom.Camera``
    baked into the robot asset).  Intrinsics are intentionally not stored here — they are
    read directly from the ``UsdGeom.Camera`` prim at render time.

    Args:
        name: Logical name used as the attribute name on the sensor rig.
        sensor_prim_path: USD prim path relative to the robot root prim.
        width_px: Render width in pixels.
        height_px: Render height in pixels.
        frame_id: Optional ROS-style frame ID string.
    """

    name: str
    sensor_prim_path: str
    width_px: int = 0
    height_px: int = 0
    frame_id: str = ""

    def __post_init__(self) -> None:
        """Validate that required fields are non-empty after construction."""
        missing = []
        if not self.sensor_prim_path:
            missing.append("sensor_prim_path")
        if self.width_px <= 0:
            missing.append("width_px")
        if self.height_px <= 0:
            missing.append("height_px")
        if missing:
            raise ValueError(f"CameraConfig {self.name!r} missing required fields: {missing}")

    @classmethod
    def from_dict(cls, data: dict) -> "CameraConfig":
        """Construct a CameraConfig from a parsed YAML sensor entry dict.

        Args:
            data: Parsed YAML sensor entry dictionary.

        Returns:
            The constructed camera configuration.
        """
        return cls(
            name=data["name"],
            sensor_prim_path=data.get("sensor_prim_path", ""),
            width_px=data.get("width_px", 0),
            height_px=data.get("height_px", 0),
            frame_id=data.get("frame_id", ""),
        )


# Union of all sensor config types.  Extend as new sensor types are added.
SensorConfig = CameraConfig
