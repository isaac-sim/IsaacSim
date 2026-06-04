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

"""Sensor rig — a Module grouping USD-backed cameras mounted on a robot."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import carb

if TYPE_CHECKING:
    from .types import SensorConfig
from isaacsim.core.experimental.utils.stage import get_current_stage

from .common import Module, _join_sdf_paths


def parse_sensor_entries(entries: list[dict[str, Any]]) -> list[SensorConfig]:
    """Parse a list of sensor dicts (from YAML) into typed sensor config objects.

    Supported ``type`` values: ``camera``. ``lidar``, ``imu``, and ``radar`` entries are
    logged and skipped — not yet implemented.

    Args:
        entries: List of sensor entry dicts in the YAML sensor format.

    Returns:
        List of :class:`~.types.CameraConfig` objects for supported sensors.
    """
    from .types import CameraConfig

    configs: list[SensorConfig] = []
    for entry in entries:
        sensor_type = entry.get("type", "")
        name = entry.get("name", "<unnamed>")

        if sensor_type == "camera":
            try:
                configs.append(CameraConfig.from_dict(entry))
            except (ValueError, KeyError) as e:
                carb.log_error(f"parse_sensor_entries: skipping camera {name!r} — {e}")
        elif sensor_type in ("lidar", "imu", "radar"):
            carb.log_info(f"parse_sensor_entries: skipping {sensor_type!r} sensor {name!r} (not yet supported)")
        else:
            carb.log_warn(f"parse_sensor_entries: unknown sensor type {sensor_type!r} for {name!r}, skipping")

    return configs


def _attach_sensors(owner: object, configs: list[SensorConfig], robot_root_path: str) -> None:
    """Attach camera sensors to *owner* for each config in *configs*.

    Each ``CameraConfig.sensor_prim_path`` is resolved against *robot_root_path* to get the
    absolute USD path of an existing ``UsdGeom.Camera`` prim.  A Replicator render product is
    attached to that prim — no new prims are created.

    Args:
        owner: Object that receives each sensor as a named attribute.
        configs: List of :class:`~.types.CameraConfig` objects.
        robot_root_path: Absolute USD path of the robot root prim (e.g. ``"/World/carter"``).
    """
    from .camera import MobilityGenCamera
    from .types import CameraConfig

    stage = get_current_stage(backend="usd")

    for cfg in configs:
        if not isinstance(cfg, CameraConfig):
            carb.log_warn(f"_attach_sensors: unsupported config type {type(cfg).__name__!r}, skipping")
            continue

        cam_path = _join_sdf_paths(robot_root_path, cfg.sensor_prim_path)
        if not stage.GetPrimAtPath(cam_path).IsValid():
            carb.log_error(f"_attach_sensors: prim not found at '{cam_path}' for camera '{cfg.name}', skipping")
            continue

        sensor = MobilityGenCamera(cam_path, (cfg.width_px, cfg.height_px))
        setattr(owner, cfg.name, sensor)
        carb.log_info(f"_attach_sensors: camera '{cfg.name}' at '{cam_path}' ({cfg.width_px}x{cfg.height_px})")


class MobilityGenSensorRig(Module):
    """Groups USD-backed camera sensors on a robot for cascade data-capture operations.

    Each sensor is attached as a named attribute (e.g. ``rig.front_left``) pointing to a
    :class:`~.camera.MobilityGenCamera`.  The standard :class:`~.common.Module` cascade
    methods — :meth:`update_state`, :meth:`enable_rgb_rendering`, :meth:`state_dict_rgb` —
    propagate to all sensors automatically.

    Sensors must already exist as ``UsdGeom.Camera`` prims in the loaded robot USD.
    ``sensor_prim_path`` (relative to the robot root prim) is resolved to an absolute stage
    path; a Replicator render product is attached to that prim.  No new prims are created.

    Typical usage::

        robot = CarterMultiSensorRobot.build("/World/carter")
        robot.sensor_rig.enable_rgb_rendering()
        robot.sensor_rig.finalize_rendering()
        # each step:
        robot.update_state()
        images = robot.state_dict_rgb()
    """

    @classmethod
    def from_sensor_configs(cls, configs: list[SensorConfig], robot_root_path: str) -> "MobilityGenSensorRig":
        """Build a rig from a list of :class:`~.types.SensorConfig`.

        Args:
            configs: List of :class:`~.types.CameraConfig` objects.
            robot_root_path: Absolute USD path of the robot root prim used to resolve
                ``sensor_prim_path`` values.

        Returns:
            New :class:`MobilityGenSensorRig` with sensors attached as named attributes.
        """
        rig = cls()
        _attach_sensors(rig, configs, robot_root_path)
        return rig
