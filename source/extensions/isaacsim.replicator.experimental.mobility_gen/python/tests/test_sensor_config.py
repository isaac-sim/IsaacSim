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

"""Verifies CameraConfig validation and sensor entry parsing without requiring an Isaac Sim runtime. The tests cover required fields, frame IDs, dictionary construction, error messages, valid camera entries, and skipping invalid or unsupported sensor definitions."""

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import omni.kit.test

# ---------------------------------------------------------------------------
# Load types.py (no isaacsim deps — only dataclasses + numpy)
# ---------------------------------------------------------------------------
_TYPES_SRC = (Path(__file__).parent.parent / "impl" / "types.py").read_text()
_types_globs: dict = {}
exec(compile(_TYPES_SRC, "types.py", "exec"), _types_globs)
CameraConfig = _types_globs["CameraConfig"]
SensorConfig = _types_globs["SensorConfig"]

# ---------------------------------------------------------------------------
# Load sensor_rig.py with stubs for carb and isaacsim imports
# ---------------------------------------------------------------------------
_SENSOR_RIG_SRC = (Path(__file__).parent.parent / "impl" / "sensor_rig.py").read_text()

# Strip relative imports; inject stubs via exec globals
_SENSOR_RIG_STRIPPED = "\n".join(
    line
    for line in _SENSOR_RIG_SRC.splitlines()
    if not line.startswith("from .common")
    and not line.startswith("from isaacsim")
    and not line.startswith("if TYPE_CHECKING")
    and "from .types import" not in line
    and "from .camera import" not in line
)

_carb_stub = MagicMock()
_sensor_rig_globs = {
    "carb": _carb_stub,
    "get_current_stage": MagicMock(),
    "Module": object,
    "_join_sdf_paths": lambda *a: "/".join(a),
    "CameraConfig": CameraConfig,
    "SensorConfig": SensorConfig,
}
exec(compile(_SENSOR_RIG_STRIPPED, "sensor_rig.py", "exec"), _sensor_rig_globs)
parse_sensor_entries = _sensor_rig_globs["parse_sensor_entries"]


# ---------------------------------------------------------------------------
# CameraConfig tests
# ---------------------------------------------------------------------------


class TestCameraConfigConstruction(omni.kit.test.AsyncTestCase):
    """CameraConfig must validate required fields on construction."""

    def _valid(self, **kwargs: Any) -> object:
        defaults = {"name": "front", "sensor_prim_path": "/cam", "width_px": 640, "height_px": 480}
        defaults.update(kwargs)
        return CameraConfig(**defaults)

    async def test_valid_construction(self) -> None:
        """Exercise test valid construction."""
        cfg = self._valid()
        self.assertEqual(cfg.name, "front")
        self.assertEqual(cfg.sensor_prim_path, "/cam")
        self.assertEqual(cfg.width_px, 640)
        self.assertEqual(cfg.height_px, 480)
        self.assertEqual(cfg.frame_id, "")

    async def test_frame_id_default_empty(self) -> None:
        """Exercise test frame id default empty."""
        cfg = self._valid()
        self.assertEqual(cfg.frame_id, "")

    async def test_frame_id_set(self) -> None:
        """Exercise test frame id set."""
        cfg = self._valid(frame_id="base_link")
        self.assertEqual(cfg.frame_id, "base_link")

    async def test_missing_sensor_prim_path_raises(self) -> None:
        """Exercise test missing sensor prim path raises."""
        with self.assertRaises(ValueError):
            self._valid(sensor_prim_path="")

    async def test_zero_width_raises(self) -> None:
        """Exercise test zero width raises."""
        with self.assertRaises(ValueError):
            self._valid(width_px=0)

    async def test_negative_width_raises(self) -> None:
        """Exercise test negative width raises."""
        with self.assertRaises(ValueError):
            self._valid(width_px=-1)

    async def test_zero_height_raises(self) -> None:
        """Exercise test zero height raises."""
        with self.assertRaises(ValueError):
            self._valid(height_px=0)

    async def test_error_message_lists_all_missing_fields(self) -> None:
        """Exercise test error message lists all missing fields."""
        with self.assertRaises(ValueError) as ctx:
            CameraConfig(name="cam", sensor_prim_path="", width_px=0, height_px=0)
        msg = str(ctx.exception)
        self.assertIn("sensor_prim_path", msg)
        self.assertIn("width_px", msg)
        self.assertIn("height_px", msg)


class TestCameraConfigFromDict(omni.kit.test.AsyncTestCase):
    """CameraConfig.from_dict must round-trip a YAML sensor entry dict."""

    def _full_dict(self, **overrides: Any) -> dict:
        d = {
            "name": "rear",
            "sensor_prim_path": "/robot/rear_cam",
            "width_px": 1920,
            "height_px": 1080,
            "frame_id": "rear_frame",
        }
        d.update(overrides)
        return d

    async def test_from_dict_all_fields(self) -> None:
        """Exercise test from dict all fields."""
        cfg = CameraConfig.from_dict(self._full_dict())
        self.assertEqual(cfg.name, "rear")
        self.assertEqual(cfg.sensor_prim_path, "/robot/rear_cam")
        self.assertEqual(cfg.width_px, 1920)
        self.assertEqual(cfg.height_px, 1080)
        self.assertEqual(cfg.frame_id, "rear_frame")

    async def test_from_dict_minimal_fields(self) -> None:
        """Exercise test from dict minimal fields."""
        cfg = CameraConfig.from_dict({"name": "front", "sensor_prim_path": "/cam", "width_px": 640, "height_px": 480})
        self.assertEqual(cfg.frame_id, "")

    async def test_from_dict_missing_required_raises(self) -> None:
        """Exercise test from dict missing required raises."""
        with self.assertRaises((ValueError, KeyError)):
            CameraConfig.from_dict({"name": "front", "sensor_prim_path": ""})


# ---------------------------------------------------------------------------
# parse_sensor_entries tests
# ---------------------------------------------------------------------------


class TestParseSensorEntries(omni.kit.test.AsyncTestCase):
    """parse_sensor_entries must convert YAML dicts into typed config objects."""

    def _camera_entry(self, **overrides: Any) -> dict:
        d = {"type": "camera", "name": "front", "sensor_prim_path": "/cam", "width_px": 640, "height_px": 480}
        d.update(overrides)
        return d

    async def test_empty_list_returns_empty(self) -> None:
        """Exercise test empty list returns empty."""
        self.assertEqual(parse_sensor_entries([]), [])

    async def test_single_valid_camera(self) -> None:
        """Exercise test single valid camera."""
        result = parse_sensor_entries([self._camera_entry()])
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], CameraConfig)
        self.assertEqual(result[0].name, "front")

    async def test_multiple_cameras(self) -> None:
        """Exercise test multiple cameras."""
        entries = [
            self._camera_entry(name="front"),
            self._camera_entry(name="rear", sensor_prim_path="/rear_cam"),
        ]
        result = parse_sensor_entries(entries)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0].name, "front")
        self.assertEqual(result[1].name, "rear")

    async def test_invalid_camera_skipped(self) -> None:
        """A camera entry with missing required fields is skipped (logged as error)."""
        bad = {"type": "camera", "name": "bad", "sensor_prim_path": "", "width_px": 0, "height_px": 0}
        result = parse_sensor_entries([bad])
        self.assertEqual(result, [])

    async def test_unsupported_types_skipped(self) -> None:
        """lidar, imu, radar entries are skipped without raising."""
        entries = [
            {"type": "lidar", "name": "top_lidar"},
            {"type": "imu", "name": "imu0"},
            {"type": "radar", "name": "radar0"},
        ]
        result = parse_sensor_entries(entries)
        self.assertEqual(result, [])

    async def test_unknown_type_skipped(self) -> None:
        """Exercise test unknown type skipped."""
        result = parse_sensor_entries([{"type": "sonar", "name": "s0"}])
        self.assertEqual(result, [])

    async def test_valid_and_invalid_mixed(self) -> None:
        """Only valid camera entries appear in the result."""
        entries = [
            self._camera_entry(name="good"),
            {"type": "lidar", "name": "skip_lidar"},
            {"type": "camera", "name": "bad", "sensor_prim_path": "", "width_px": 0, "height_px": 0},
        ]
        result = parse_sensor_entries(entries)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].name, "good")
