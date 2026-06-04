# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for configured SRTX sensor-set resolution and setup."""

from __future__ import annotations

import json
import sys
import types
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

import carb
import omni
import omni.graph.core as og
import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.replicator.core as rep
import omni.usd
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from isaacsim.ros2.nodes.impl import ros2_common
from pxr import UsdGeom

EXTENSION_ROOT = Path(__file__).resolve().parents[1]
CAMERA_HELPER_PATH = EXTENSION_ROOT / "nodes" / "OgnROS2CameraHelper.py"
CAMERA_INFO_HELPER_PATH = EXTENSION_ROOT / "nodes" / "OgnROS2CameraInfoHelper.py"
LIDAR_HELPER_PATH = EXTENSION_ROOT / "nodes" / "OgnROS2RtxLidarHelper.py"

USE_SRTX_SETTING = "/exts/omni.replicator.srtx/enabled"
SRTX_SENSOR_SET_NAME_SETTING = "/exts/omni.replicator.srtx/sensorSetName"
SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING = "/exts/omni.replicator.srtx/sensorSetNameByRenderProductPath"
SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING = "/exts/omni.replicator.srtx/sensorSetRenderProductPathsByName"
MIN_SRTX_CONFIGURED_SENSOR_SET_VERSION = "1.1.1"


def _parse_extension_version(version: str) -> tuple[int, int, int]:
    """Parse the numeric major.minor.patch portion of an extension version."""

    numeric_version = version.split("+", 1)[0].split("-", 1)[0]
    parts = []
    for raw_part in numeric_version.split(".")[:3]:
        try:
            parts.append(int(raw_part))
        except ValueError:
            parts.append(0)
    return tuple((parts + [0, 0, 0])[:3])


def _get_srtx_extension_version() -> str | None:
    """Return the enabled SRTX extension version, enabling it first if available."""

    if not ros2_common.is_srtx_supported_platform():
        return None

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = ext_manager.get_enabled_extension_id("omni.replicator.srtx")
    if ext_id is None:
        try:
            ext_manager.set_extension_enabled_immediate("omni.replicator.srtx", True)
        except Exception:
            return None
        ext_id = ext_manager.get_enabled_extension_id("omni.replicator.srtx")
    if ext_id is None:
        return None
    ext_dict = ext_manager.get_extension_dict(ext_id)
    if ext_dict is None:
        return None
    if hasattr(ext_dict, "get_dict"):
        ext_dict = ext_dict.get_dict()
    package = ext_dict.get("package", {})
    version = package.get("version")
    return version if isinstance(version, str) else None


def _srtx_extension_is_too_old_for_configured_sensor_sets() -> bool:
    """Return True when the available SRTX extension predates configured sensor-set support."""

    version = _get_srtx_extension_version()
    if version is None:
        return True
    return _parse_extension_version(version) < _parse_extension_version(MIN_SRTX_CONFIGURED_SENSOR_SET_VERSION)


class FakeSrtxInstance:
    """Record SRTX interactions for configuration and setup assertions."""

    def __init__(self, declare_result: bool = True, register_handle: int = 77) -> None:
        """Initialize the fake SRTX instance with configurable outcomes."""
        self.declare_result = declare_result
        self.register_handle = register_handle
        self.declare_calls: list[tuple[str, list[str]]] = []
        self.add_sensor_calls: list[tuple[str, str, str]] = []
        self.register_calls: list[tuple[str, str, object]] = []

    def declare_sensor_set(self, sensor_set_name: str, sensor_paths: list[str]) -> bool:
        """Record the declaration request and return the configured outcome."""
        self.declare_calls.append((sensor_set_name, list(sensor_paths)))
        return self.declare_result

    def add_sensor(self, sensor_set_name: str, sensor_name: str, sensor_path: str) -> None:
        """Record local sensor registration for a sensor set."""
        self.add_sensor_calls.append((sensor_set_name, sensor_name, sensor_path))

    def register_frame_callback(self, sensor_set_name: str, rendervar_path: str, capsule: object) -> int:
        """Record callback registration and return the configured handle."""
        self.register_calls.append((sensor_set_name, rendervar_path, capsule))
        return self.register_handle


class RecordingSrtxInstance:
    """Record calls made by real ROS 2 OmniGraph helper nodes to SRTX."""

    def __init__(self) -> None:
        self.declare_calls: list[tuple[str, list[str]]] = []
        self.add_sensor_calls: list[tuple[str, str, str]] = []
        self.register_calls: list[tuple[str, str, object, int]] = []
        self.unregister_calls: list[tuple[str, int]] = []
        self.stop_capture_calls: list[str] = []
        self.start_capture_calls: list[tuple[str, list[str]]] = []
        self._next_handle = 1

    def declare_sensor_set(self, sensor_set_name: str, sensor_paths: list[str]) -> bool:
        self.declare_calls.append((sensor_set_name, list(sensor_paths)))
        return True

    def add_sensor(self, sensor_set_name: str, sensor_name: str, sensor_path: str) -> None:
        self.add_sensor_calls.append((sensor_set_name, sensor_name, sensor_path))

    def register_frame_callback(self, sensor_set_name: str, rendervar_path: str, capsule: object) -> int:
        handle = self._next_handle
        self._next_handle += 1
        self.register_calls.append((sensor_set_name, rendervar_path, capsule, handle))
        return handle

    def unregister_frame_callback(self, sensor_set_name: str, handle: int) -> None:
        self.unregister_calls.append((sensor_set_name, handle))

    def stop_continuous_capture(self, sensor_set_name: str) -> None:
        self.stop_capture_calls.append(sensor_set_name)

    def start_continuous_capture(self, sensor_set_name: str, output_paths: list[str]) -> None:
        self.start_capture_calls.append((sensor_set_name, list(output_paths)))


class RecordingSrtxCore:
    """Minimal SrtxCore replacement used by setup-path integration tests."""

    _instances: dict[str, RecordingSrtxInstance] = {}

    @staticmethod
    def get_instance(usd_scene: str) -> RecordingSrtxInstance | None:
        return RecordingSrtxCore._instances.get(usd_scene)

    @staticmethod
    def create_instance(usd_scene: str) -> RecordingSrtxInstance:
        instance = RecordingSrtxInstance()
        RecordingSrtxCore._instances[usd_scene] = instance
        return instance

    @staticmethod
    def reset_all() -> None:
        RecordingSrtxCore._instances.clear()


@contextmanager
def mock_srtx_core_binding():
    """Patch only the SrtxCore binding while keeping the real SRTX Python package active."""

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("omni.replicator.srtx", True)

    import omni.replicator.srtx as srtx
    import omni.replicator.srtx.impl.sensorset_utils as sensorset_utils

    if not callable(getattr(srtx, "prepare_configured_sensorset", None)):
        raise AssertionError("omni.replicator.srtx.prepare_configured_sensorset is required for this integration test")

    RecordingSrtxCore.reset_all()
    with (
        patch.object(srtx, "SrtxCore", RecordingSrtxCore),
        patch.object(sensorset_utils, "SrtxCore", RecordingSrtxCore),
    ):
        try:
            yield
        finally:
            RecordingSrtxCore.reset_all()


def _load_module(module_name: str, file_path: Path) -> types.ModuleType:
    """Load an extension node helper module from its generated Python file."""
    import importlib.util

    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


@contextmanager
def mock_srtx_binding(srtx_instance: FakeSrtxInstance):
    """Patch only the SRTX Python binding needed by unit-level setup tests."""

    try:
        import omni.replicator.srtx as srtx

        created_module = False
        saved_module = None
        saved_parent_attr = getattr(omni.replicator, "srtx", None)
        had_parent_attr = hasattr(omni.replicator, "srtx")
    except ImportError:
        srtx = types.ModuleType("omni.replicator.srtx")
        created_module = True
        saved_module = sys.modules.get("omni.replicator.srtx")
        saved_parent_attr = getattr(omni.replicator, "srtx", None)
        had_parent_attr = hasattr(omni.replicator, "srtx")
        sys.modules["omni.replicator.srtx"] = srtx
        omni.replicator.srtx = srtx

    class SrtxCore:
        @staticmethod
        def get_instance(_usd_scene: str) -> FakeSrtxInstance:
            return srtx_instance

    with (
        patch.object(srtx, "SrtxCore", SrtxCore, create=True),
        patch.object(srtx, "prepare_configured_sensorset", None, create=True),
    ):
        try:
            yield
        finally:
            if created_module:
                if saved_module is None:
                    sys.modules.pop("omni.replicator.srtx", None)
                else:
                    sys.modules["omni.replicator.srtx"] = saved_module
                if had_parent_attr:
                    omni.replicator.srtx = saved_parent_attr
                else:
                    delattr(omni.replicator, "srtx")


@contextmanager
def mock_laser_scan_capsule_binding(captured_kwargs: dict[str, object]):
    """Patch the ROS 2 laser scan capsule binding with the current pybind signature."""

    module_name = "isaacsim.ros2.nodes.bindings._ros2_nodes"
    ros2_nodes = types.ModuleType(module_name)

    def create_laser_scan_publisher_capsule(
        *,
        topic_name: str,
        frame_id: str,
        node_namespace: str,
        queue_size: int,
        qos_profile: str,
        azimuth_range_start: float,
        azimuth_range_end: float,
        depth_range_min: float,
        depth_range_max: float,
        rotation_rate: float,
        horizontal_resolution: float,
        horizontal_fov: float,
    ) -> str:
        captured_kwargs.update(
            {
                "topic_name": topic_name,
                "frame_id": frame_id,
                "node_namespace": node_namespace,
                "queue_size": queue_size,
                "qos_profile": qos_profile,
                "azimuth_range_start": azimuth_range_start,
                "azimuth_range_end": azimuth_range_end,
                "depth_range_min": depth_range_min,
                "depth_range_max": depth_range_max,
                "rotation_rate": rotation_rate,
                "horizontal_resolution": horizontal_resolution,
                "horizontal_fov": horizontal_fov,
            }
        )
        return "laser-scan-capsule"

    ros2_nodes.create_laser_scan_publisher_capsule = create_laser_scan_publisher_capsule

    with patch.dict(sys.modules, {module_name: ros2_nodes}):
        yield


class TestConfiguredSrtxSensorSets(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        if omni.usd.get_context().get_stage() is None:
            await omni.usd.get_context().new_stage_async()
        self._settings = carb.settings.get_settings()
        self._saved_settings = {
            USE_SRTX_SETTING: self._settings.get(USE_SRTX_SETTING),
            SRTX_SENSOR_SET_NAME_SETTING: self._settings.get(SRTX_SENSOR_SET_NAME_SETTING),
            SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING: self._settings.get(
                SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING
            ),
            SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING: self._settings.get(
                SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING
            ),
        }
        self._clear_srtx_settings()

    async def tearDown(self) -> None:
        self._restore_srtx_settings()

    def _clear_srtx_settings(self) -> None:
        self._settings.set_bool(USE_SRTX_SETTING, False)
        self._settings.set(SRTX_SENSOR_SET_NAME_SETTING, "")
        self._settings.set(SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING, "")
        self._settings.set(SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING, "")

    def _restore_srtx_settings(self) -> None:
        for setting_name, value in self._saved_settings.items():
            if setting_name == USE_SRTX_SETTING:
                self._settings.set_bool(setting_name, bool(value) if value is not None else False)
            else:
                self._settings.set(setting_name, value if value is not None else "")

    async def test_get_srtx_sensor_set_config_returns_configured_mapping(self) -> None:
        """Configured render products should resolve to the shared set and full path list."""
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/A": "ss-configured"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/Render/Product/A", "/Render/Product/B"]}),
        )

        config = ros2_common.get_srtx_sensor_set_config("/Render/Product/A")

        self.assertEqual(config.name, "ss-configured")
        self.assertEqual(config.render_product_paths, ["/Render/Product/A", "/Render/Product/B"])

    async def test_get_srtx_sensor_set_config_falls_back_on_missing_paths(self) -> None:
        """Missing declaration paths should log an error and use the fallback name."""
        error_logs: list[str] = []
        self._settings.set(ros2_common.SRTX_SENSOR_SET_NAME_SETTING, "bridge-fallback")
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/A": "ss-configured"}),
        )

        with patch.object(carb, "log_error", side_effect=error_logs.append):
            config = ros2_common.get_srtx_sensor_set_config("/Render/Product/A")

        self.assertEqual(config.name, "bridge-fallback")
        self.assertIsNone(config.render_product_paths)
        self.assertTrue(any("missing declaration paths" in message for message in error_logs))

    async def test_prepare_srtx_sensor_set_declares_configured_sensor_set(self) -> None:
        """Configured sensor sets should be declared before use."""
        srtx_instance = FakeSrtxInstance()
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/A": "ss-configured"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/Render/Product/A", "/Render/Product/B"]}),
        )

        with mock_srtx_binding(srtx_instance):
            sensor_set_name = ros2_common.prepare_srtx_sensor_set(srtx_instance, "/Render/Product/A")

        self.assertEqual(sensor_set_name, "ss-configured")
        self.assertEqual(
            srtx_instance.declare_calls,
            [("ss-configured", ["/Render/Product/A", "/Render/Product/B"])],
        )

    async def test_prepare_srtx_sensor_set_skips_declaration_for_fallback(self) -> None:
        """Fallback sensor-set names should not invoke explicit declaration."""
        srtx_instance = FakeSrtxInstance()
        self._settings.set(ros2_common.SRTX_SENSOR_SET_NAME_SETTING, "bridge-fallback")

        with mock_srtx_binding(srtx_instance):
            sensor_set_name = ros2_common.prepare_srtx_sensor_set(srtx_instance, "/Render/Product/A")

        self.assertEqual(sensor_set_name, "bridge-fallback")
        self.assertEqual(srtx_instance.declare_calls, [])

    async def test_prepare_srtx_sensor_set_returns_none_when_declaration_rejected(self) -> None:
        """Rejected declarations should surface as setup failure."""
        error_logs: list[str] = []
        srtx_instance = FakeSrtxInstance(declare_result=False)
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/A": "ss-configured"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/Render/Product/A", "/Render/Product/B"]}),
        )

        with mock_srtx_binding(srtx_instance), patch.object(carb, "log_error", side_effect=error_logs.append):
            sensor_set_name = ros2_common.prepare_srtx_sensor_set(srtx_instance, "/Render/Product/A")

        self.assertIsNone(sensor_set_name)
        self.assertTrue(any("declaration was rejected" in message for message in error_logs))

    async def test_camera_setup_declares_and_registers_configured_sensor_set(self) -> None:
        """Camera helper setup should declare and reuse the configured shared sensor set."""
        srtx_instance = FakeSrtxInstance()
        camera_helper = _load_module("test_ogn_ros2_camera_helper", CAMERA_HELPER_PATH)
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/A": "ss-configured"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/Render/Product/A", "/Render/Product/B"]}),
        )
        capture_calls: list[tuple[str, str]] = []
        camera_helper.ensure_render_var_on_product = (
            lambda stage, render_product_path, aov, compression_type, is_image: (
                True,
                f"{render_product_path}/{aov}",
            )
        )
        camera_helper._start_or_extend_continuous_capture = (
            lambda srtx_instance, sensor_set_name, output_path: capture_calls.append((sensor_set_name, output_path))
        )
        state = types.SimpleNamespace(initialized=True)

        with mock_srtx_binding(srtx_instance):
            success = camera_helper.OgnROS2CameraHelper._setup_srtx(
                config={"srtx": True, "aov": "LdrColorSD", "is_image": True},
                init_params={
                    "topicName": "rgb",
                    "frameId": "camera",
                    "nodeNamespace": "",
                    "queueSize": 10,
                    "qosProfile": "sensor-data",
                },
                render_product_path="/Render/Product/A",
                sensor_type="rgb",
                state=state,
                compression_type=None,
            )

        self.assertTrue(success)
        self.assertEqual(
            srtx_instance.declare_calls,
            [("ss-configured", ["/Render/Product/A", "/Render/Product/B"])],
        )
        self.assertEqual(
            srtx_instance.add_sensor_calls,
            [("ss-configured", "A", "/Render/Product/A")],
        )
        self.assertEqual(
            srtx_instance.register_calls[0][0:2],
            ("ss-configured", "/Render/Product/A/LdrColorSD"),
        )
        self.assertEqual(capture_calls, [("ss-configured", "/Render/Product/A/LdrColorSD")])
        self.assertEqual(state._srtx_sensor_set, "ss-configured")

    async def test_camera_info_setup_declares_and_uses_configured_sensor_set(self) -> None:
        """CameraInfo helper should use the render-product-specific configured sensor set."""
        srtx_instance = FakeSrtxInstance()
        camera_info_helper = _load_module("test_ogn_ros2_camera_info_helper", CAMERA_INFO_HELPER_PATH)
        self._settings.set_bool(USE_SRTX_SETTING, True)
        self._settings.set(ros2_common.SRTX_SENSOR_SET_NAME_SETTING, "bridge-fallback")
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/A": "ss-configured"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/Render/Product/A", "/Render/Product/B"]}),
        )
        camera_info = types.SimpleNamespace(
            width=640,
            height=480,
            distortion_model="plumb_bob",
            k=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            d=[],
        )
        state = types.SimpleNamespace(
            initialized=False,
            _srtx_callback_handles=[],
            _srtx_callback_sensor_sets=[],
            _srtx_capsules=[],
            _srtx_sensor_set=None,
        )
        db = types.SimpleNamespace(
            per_instance_state=state,
            inputs=types.SimpleNamespace(
                enabled=True,
                renderProductPath="/Render/Product/A",
                renderProductPathRight="",
                resetSimulationTimeOnStop=True,
                frameSkipCount=0,
                useSystemTime=False,
                frameId="camera",
                topicName="camera_info",
                nodeNamespace="",
                queueSize=10,
                qosProfile="sensor-data",
                context=0,
            ),
        )
        publisher_calls: list[tuple[str, str]] = []

        def record_camera_info_publisher(
            state,
            stage,
            srtx_instance,
            sensor_set_name,
            frameId,
            topicName,
            nodeNamespace,
            queueSize,
            qosProfile,
            camera_info,
            render_product_path,
        ) -> bool:
            publisher_calls.append((sensor_set_name, render_product_path))
            return True

        with (
            mock_srtx_binding(srtx_instance),
            patch.object(camera_info_helper, "read_camera_info", return_value=(camera_info, object())),
            patch.object(camera_info_helper, "collect_namespace", return_value=""),
            patch.object(
                camera_info_helper.OgnROS2CameraInfoHelper,
                "add_srtx_camera_info_publisher",
                side_effect=record_camera_info_publisher,
            ),
        ):
            success = camera_info_helper.OgnROS2CameraInfoHelper.compute(db)

        self.assertTrue(success)
        self.assertTrue(state.initialized)
        self.assertEqual(
            srtx_instance.declare_calls,
            [("ss-configured", ["/Render/Product/A", "/Render/Product/B"])],
        )
        self.assertEqual(publisher_calls, [("ss-configured", "/Render/Product/A")])

    async def test_camera_info_setup_uses_configured_sensor_sets_for_stereo(self) -> None:
        """CameraInfo helper should use left and right configured sensor sets for stereo."""
        srtx_instance = FakeSrtxInstance()
        camera_info_helper = _load_module("test_ogn_ros2_camera_info_helper", CAMERA_INFO_HELPER_PATH)
        self._settings.set_bool(USE_SRTX_SETTING, True)
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/Left": "ss-left", "/Render/Product/Right": "ss-right"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-left": ["/Render/Product/Left"], "ss-right": ["/Render/Product/Right"]}),
        )

        def make_camera_info() -> types.SimpleNamespace:
            return types.SimpleNamespace(
                width=640,
                height=480,
                distortion_model="plumb_bob",
                k=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                p=[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                d=[],
            )

        state = types.SimpleNamespace(
            initialized=False,
            _srtx_callback_handles=[],
            _srtx_callback_sensor_sets=[],
            _srtx_capsules=[],
            _srtx_sensor_set=None,
        )
        db = types.SimpleNamespace(
            per_instance_state=state,
            inputs=types.SimpleNamespace(
                enabled=True,
                renderProductPath="/Render/Product/Left",
                renderProductPathRight="/Render/Product/Right",
                resetSimulationTimeOnStop=True,
                frameSkipCount=0,
                useSystemTime=False,
                frameId="left_camera",
                frameIdRight="right_camera",
                topicName="left/camera_info",
                topicNameRight="right/camera_info",
                nodeNamespace="",
                queueSize=10,
                qosProfile="sensor-data",
                context=0,
            ),
        )
        publisher_calls: list[tuple[str, str]] = []
        np = camera_info_helper.np

        def record_camera_info_publisher(
            state,
            stage,
            srtx_instance,
            sensor_set_name,
            frameId,
            topicName,
            nodeNamespace,
            queueSize,
            qosProfile,
            camera_info,
            render_product_path,
        ) -> bool:
            publisher_calls.append((sensor_set_name, render_product_path))
            return True

        with (
            mock_srtx_binding(srtx_instance),
            patch.object(
                camera_info_helper,
                "read_camera_info",
                side_effect=[(make_camera_info(), object()), (make_camera_info(), object())],
            ),
            patch.object(camera_info_helper, "compute_relative_pose", return_value=(np.zeros(3), np.eye(3))),
            patch.object(camera_info_helper, "collect_namespace", return_value=""),
            patch.object(
                camera_info_helper.cv,
                "stereoRectify",
                return_value=(np.eye(3), np.eye(3), np.zeros((3, 4)), np.zeros((3, 4)), None, None, None),
            ),
            patch.object(
                camera_info_helper.OgnROS2CameraInfoHelper,
                "add_srtx_camera_info_publisher",
                side_effect=record_camera_info_publisher,
            ),
        ):
            success = camera_info_helper.OgnROS2CameraInfoHelper.compute(db)

        self.assertTrue(success)
        self.assertTrue(state.initialized)
        self.assertEqual(
            srtx_instance.declare_calls,
            [("ss-left", ["/Render/Product/Left"]), ("ss-right", ["/Render/Product/Right"])],
        )
        self.assertEqual(
            publisher_calls,
            [("ss-right", "/Render/Product/Right"), ("ss-left", "/Render/Product/Left")],
        )

    async def test_lidar_setup_declares_and_registers_configured_sensor_set(self) -> None:
        """Lidar helper setup should declare and reuse the configured shared sensor set."""
        srtx_instance = FakeSrtxInstance()
        lidar_helper = _load_module("test_ogn_ros2_rtx_lidar_helper", LIDAR_HELPER_PATH)
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/Lidar": "ss-configured"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/Render/Product/Lidar", "/Render/Product/Camera"]}),
        )
        capture_calls: list[tuple[str, str]] = []
        lidar_helper.ensure_render_var_on_product = (
            lambda stage, render_product_path, aov, compression_type=None, is_image=False: (
                True,
                f"{render_product_path}/{aov}",
            )
        )
        lidar_helper._start_or_extend_continuous_capture = (
            lambda srtx_instance, sensor_set_name, output_path: capture_calls.append((sensor_set_name, output_path))
        )
        state = types.SimpleNamespace(initialized=True)

        with mock_srtx_binding(srtx_instance):
            success = lidar_helper.OgnROS2RtxLidarHelper._setup_srtx(
                init_params={
                    "topicName": "point_cloud",
                    "frameId": "lidar",
                    "nodeNamespace": "",
                    "queueSize": 10,
                    "qosProfile": "sensor-data",
                },
                render_product_path="/Render/Product/Lidar",
                state=state,
                sensor_type="point_cloud",
                compression_type=None,
            )

        self.assertTrue(success)
        self.assertEqual(
            srtx_instance.declare_calls,
            [("ss-configured", ["/Render/Product/Lidar", "/Render/Product/Camera"])],
        )
        self.assertEqual(
            srtx_instance.add_sensor_calls,
            [("ss-configured", "Lidar", "/Render/Product/Lidar")],
        )
        self.assertEqual(
            srtx_instance.register_calls[0][0:2],
            ("ss-configured", "/Render/Product/Lidar/GenericModelOutput"),
        )
        self.assertEqual(capture_calls, [("ss-configured", "/Render/Product/Lidar/GenericModelOutput")])
        self.assertEqual(state._srtx_sensor_set, "ss-configured")

    async def test_laser_scan_setup_does_not_forward_removed_max_points_metadata(self) -> None:
        """LaserScan setup should ignore stale max_points metadata before calling pybind."""
        srtx_instance = FakeSrtxInstance()
        lidar_helper = _load_module("test_ogn_ros2_rtx_lidar_helper", LIDAR_HELPER_PATH)
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
            json.dumps({"/Render/Product/Lidar": "ss-configured"}),
        )
        self._settings.set(
            ros2_common.SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
            json.dumps({"ss-configured": ["/Render/Product/Lidar", "/Render/Product/Camera"]}),
        )
        capture_calls: list[tuple[str, str]] = []
        capsule_kwargs: dict[str, object] = {}
        scan_meta = {
            "azimuth_range_start": -180.0,
            "azimuth_range_end": 180.0,
            "depth_range_min": 0.1,
            "depth_range_max": 60.0,
            "rotation_rate": 10.0,
            "horizontal_resolution": 0.1125,
            "horizontal_fov": 360.0,
            "max_points": 3200,
        }
        state = types.SimpleNamespace(initialized=True)
        fake_camera = types.SimpleNamespace(GetPrim=lambda: object())

        with (
            mock_srtx_binding(srtx_instance),
            mock_laser_scan_capsule_binding(capsule_kwargs),
            patch.object(
                lidar_helper,
                "ensure_render_var_on_product",
                lambda stage, render_product_path, aov, compression_type=None, is_image=False: (
                    True,
                    f"{render_product_path}/{aov}",
                ),
            ),
            patch.object(
                lidar_helper,
                "_start_or_extend_continuous_capture",
                lambda srtx_instance, sensor_set_name, output_path: capture_calls.append(
                    (sensor_set_name, output_path)
                ),
            ),
            patch.object(lidar_helper.ViewportManager, "get_camera", return_value=fake_camera),
            patch.object(lidar_helper.OgnROS2RtxLidarHelper, "_read_laser_scan_metadata", return_value=scan_meta),
        ):
            success = lidar_helper.OgnROS2RtxLidarHelper._setup_srtx(
                init_params={
                    "topicName": "scan",
                    "frameId": "lidar",
                    "nodeNamespace": "",
                    "queueSize": 10,
                    "qosProfile": "sensor-data",
                },
                render_product_path="/Render/Product/Lidar",
                state=state,
                sensor_type="laser_scan",
                compression_type=None,
            )

        self.assertTrue(success)
        self.assertEqual(
            capsule_kwargs,
            {
                "topic_name": "scan",
                "frame_id": "lidar",
                "node_namespace": "",
                "queue_size": 10,
                "qos_profile": "sensor-data",
                "azimuth_range_start": -180.0,
                "azimuth_range_end": 180.0,
                "depth_range_min": 0.1,
                "depth_range_max": 60.0,
                "rotation_rate": 10.0,
                "horizontal_resolution": 0.1125,
                "horizontal_fov": 360.0,
            },
        )
        self.assertEqual(
            srtx_instance.register_calls[0],
            ("ss-configured", "/Render/Product/Lidar/GenericModelOutput", "laser-scan-capsule"),
        )
        self.assertEqual(capture_calls, [("ss-configured", "/Render/Product/Lidar/GenericModelOutput")])


class TestConfiguredSrtxSensorSetsRealNodes(ROS2TestCase):
    """Exercise configured SRTX setup through real ROS 2 OmniGraph nodes."""

    async def tearDown(self) -> None:
        settings = carb.settings.get_settings()
        settings.set_bool(USE_SRTX_SETTING, False)
        settings.set(SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING, "")
        settings.set(SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING, "")
        await super().tearDown()

    @unittest.skipIf(
        _srtx_extension_is_too_old_for_configured_sensor_sets(),
        f"Requires omni.replicator.srtx >= {MIN_SRTX_CONFIGURED_SENSOR_SET_VERSION}",
    )
    async def test_real_camera_and_lidar_helpers_declare_configured_sensor_set(self) -> None:
        """Real camera and RTX lidar helper nodes should reuse the configured shared sensor set."""

        with mock_srtx_core_binding():
            stage = omni.usd.get_context().get_stage()
            usd_scene = str(stage.GetRootLayer().identifier)
            srtx_instance = RecordingSrtxCore.create_instance(usd_scene)

            UsdGeom.Camera.Define(stage, "/World/SrtxCamera")
            camera_render_product = rep.create.render_product(
                "/World/SrtxCamera", resolution=(64, 64), render_vars=["LdrColorSD"]
            )
            camera_render_product_path = camera_render_product.path

            _, lidar_prim = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path="/World/SrtxLidar")
            self.assertEqual(lidar_prim.GetTypeName(), "OmniLidar")
            lidar_render_product = rep.create.render_product(
                lidar_prim.GetPath(), resolution=(64, 64), render_vars=["GenericModelOutput", "RtxSensorMetadata"]
            )
            lidar_render_product_path = lidar_render_product.path

            sensor_set_name = "configured-sensors"
            declaration_paths = [camera_render_product_path, lidar_render_product_path]
            settings = carb.settings.get_settings()
            settings.set_bool(USE_SRTX_SETTING, True)
            settings.set(
                SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING,
                json.dumps(
                    {
                        camera_render_product_path: sensor_set_name,
                        lidar_render_product_path: sensor_set_name,
                    }
                ),
            )
            settings.set(
                SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING,
                json.dumps({sensor_set_name: declaration_paths}),
            )

            graph_path = "/SrtxConfiguredSensorSetGraph"
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("Impulse", "omni.graph.action.OnImpulseEvent"),
                        ("CameraPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("LidarPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CameraPublish.inputs:renderProductPath", camera_render_product_path),
                        ("CameraPublish.inputs:topicName", "configured_srtx_camera"),
                        ("CameraPublish.inputs:frameId", "srtx_camera"),
                        ("CameraPublish.inputs:type", "rgb"),
                        ("CameraPublish.inputs:resetSimulationTimeOnStop", True),
                        ("LidarPublish.inputs:renderProductPath", lidar_render_product_path),
                        ("LidarPublish.inputs:topicName", "configured_srtx_lidar"),
                        ("LidarPublish.inputs:frameId", "srtx_lidar"),
                        ("LidarPublish.inputs:type", "point_cloud"),
                        ("LidarPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("Impulse.outputs:execOut", "CameraPublish.inputs:execIn"),
                        ("Impulse.outputs:execOut", "LidarPublish.inputs:execIn"),
                    ],
                },
            )

            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            og.Controller.attribute(graph_path + "/Impulse.state:enableImpulse").set(True)
            await self.simulate_until_condition(lambda: False, max_frames=6)

            self.assertEqual(
                srtx_instance.declare_calls,
                [(sensor_set_name, declaration_paths), (sensor_set_name, declaration_paths)],
            )
            self.assertIn((sensor_set_name, "SrtxCamera", camera_render_product_path), srtx_instance.add_sensor_calls)
            self.assertIn(
                (sensor_set_name, lidar_render_product_path.rsplit("/", 1)[-1], lidar_render_product_path),
                srtx_instance.add_sensor_calls,
            )
            registered_outputs = {call[0:2] for call in srtx_instance.register_calls}
            self.assertEqual(
                registered_outputs,
                {
                    (sensor_set_name, camera_render_product_path + "/LdrColorSD"),
                    (sensor_set_name, lidar_render_product_path + "/GenericModelOutput"),
                },
            )
            self.assertIn(
                (
                    sensor_set_name,
                    sorted(
                        [
                            camera_render_product_path + "/LdrColorSD",
                            lidar_render_product_path + "/GenericModelOutput",
                        ]
                    ),
                ),
                [(name, sorted(paths)) for name, paths in srtx_instance.start_capture_calls],
            )

            self._timeline.stop()
            stage.RemovePrim(graph_path)
            await omni.kit.app.get_app().next_update_async()


if __name__ == "__main__":
    unittest.main()
