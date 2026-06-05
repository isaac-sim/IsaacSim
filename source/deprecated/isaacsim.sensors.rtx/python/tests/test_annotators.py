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

"""Tests for RTX sensor annotators with multi-tick and multi-frame validation."""

import asyncio
import math
from typing import Any

import carb
import isaacsim.sensors.rtx.generic_model_output as gmo_utils
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.sensors.rtx import (
    LidarRtx,
    get_gmo_data,
)
from omni.replicator.core import Writer
from pxr import Gf

from .common import create_sarcophagus

DEBUG_DRAW_PRINT = False

DEFAULT_CONFIG = None  # Default configuration for tests
DEFAULT_VARIANT = None  # Default variant for tests
NEAR_EDGE_THRESHOLD = 0.5  # Threshold for near edge returns in degrees


def _extract_gmo_raw(rp_data: Any) -> Any:
    """Extract raw GMO array from render product data, returning None if unavailable.

    Args:
        rp_data: Render product data dictionary.

    Returns:
        Raw GMO numpy array or None if unavailable.
    """
    gmo_raw = rp_data.get("GenericModelOutput")
    if gmo_raw is None:
        return None
    if isinstance(gmo_raw, dict):
        gmo_raw = gmo_raw.get("data")
    if gmo_raw is None:
        return None
    if isinstance(gmo_raw, np.ndarray) and gmo_raw.size == 0:
        return None
    return gmo_raw


class _SarcophagusTestCase(omni.kit.test.AsyncTestCase):
    """Base test case that creates a sarcophagus scene with known geometry."""

    _OCTANT_DIMENSIONS = [
        (10, 10, 5),
        (10, 10, 7),
        (25, 25, 17),
        (25, 25, 19),
        (15, 15, 9),
        (15, 15, 11),
        (20, 20, 13),
        (20, 20, 15),
    ]

    async def setUp(self) -> None:
        """Set up the test environment with a new stage and sarcophagus scene."""
        await create_new_stage_async()
        await update_stage_async()
        self._octant_dimensions = list(self._OCTANT_DIMENSIONS)
        self.cube_info = create_sarcophagus()
        self._timeline = omni.timeline.get_timeline_interface()
        self._hydra_texture = None
        self._writer = None

    async def tearDown(self) -> None:
        """Tear down the test environment and stop timeline."""
        self._timeline.stop()
        if self._writer is not None:
            self._writer.detach()
            self._writer = None
        if self._hydra_texture is not None:
            self._hydra_texture.destroy()
            self._hydra_texture = None
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()


class TestGenericModelOutput(_SarcophagusTestCase):
    """Test the Generic Model Output annotator."""

    class _GmoTestWriter(Writer):
        """Custom Writer that validates GenericModelOutput data each frame.

        Args:
            test_instance: Test case instance for assertions.
            sensor_type: Sensor type string, e.g., "lidar" or "radar".
            sensor_prim: USD prim for the sensor.
        """

        def __init__(self, test_instance: Any = None, sensor_type: Any = None, sensor_prim: Any = None) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [
                rep.annotators.get("GenericModelOutput"),
                rep.annotators.get("StableIdMap"),
            ]
            self._test = test_instance
            self._sensor_type = sensor_type
            self._prev_timestamp_ns = None
            self._stable_id_map = None
            self._octant_dims = np.array(test_instance._octant_dimensions) if test_instance else None
            self._expected_material_ids = None
            self.bad_magic_count = 0
            self.num_elements_zero_count = 0
            self.valid_frame_count = 0
            self._sensor_prim = sensor_prim
            if self._sensor_type == "lidar":
                tick_rate = self._sensor_prim.GetAttribute("omni:sensor:tickRate").Get()
                pattern_firing_rate_hz = self._sensor_prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get()
                fire_time_ns = np.array(
                    self._sensor_prim.GetAttribute("omni:sensor:Core:emitterState:s001:fireTimeNs").Get()
                )
                max_fire_time_ns = np.max(fire_time_ns)
                max_fire_time_ns_diff = np.max(np.diff(fire_time_ns))
                self._max_timeOffsetNs_expected = max(
                    max_fire_time_ns_diff, 1.0 / pattern_firing_rate_hz * 1e9 - max_fire_time_ns
                )
            elif self._sensor_type == "radar":
                tick_rate = 60.0
                self._max_timeOffsetNs_expected = 1.0
            self._expected_advance_ns = round(1.0 / tick_rate * 1e9)
            np.seterr(divide="ignore")

        def write(self, data: Any) -> None:
            if "renderProducts" not in data:
                return
            for rp_name, rp_data in data["renderProducts"].items():
                gmo_raw = _extract_gmo_raw(rp_data)
                if gmo_raw is None:
                    continue

                gmo = get_gmo_data(gmo_raw)

                if gmo.magicNumber != gmo_utils.getMagicNumberGMO():
                    self.bad_magic_count += 1
                    return

                if gmo.numElements == 0:
                    self.num_elements_zero_count += 1
                    return

                ts = int(gmo.timestampNs)
                if ts == self._prev_timestamp_ns:
                    return

                if self._prev_timestamp_ns is not None:
                    delta = ts - self._prev_timestamp_ns
                    self._test.assertAlmostEqual(
                        delta,
                        self._expected_advance_ns,
                        delta=10,
                    )
                self._prev_timestamp_ns = ts

                if self._stable_id_map is None:
                    sid_raw = rp_data.get("StableIdMap")
                    if isinstance(sid_raw, dict):
                        sid_raw = sid_raw.get("data")
                    self._stable_id_map = LidarRtx.decode_stable_id_mapping(sid_raw.tobytes())

                self._test_point_cloud(gmo)
                self._test_intensity(gmo)
                self._test_timestamp(gmo)
                if self._sensor_type == "lidar":
                    self._test_emitter_id(gmo)
                    self._test_channel_id(gmo)
                    self._test_material_id(gmo)
                    self._test_velocity(gmo)
                    self._test_object_id(gmo)
                    self._test_echo_id(gmo)
                    self._test_tick_state(gmo)
                elif self._sensor_type == "radar":
                    self._test_radial_velocity(gmo)

                self.valid_frame_count += 1

        _OCTANT_TO_CUBE = np.array([0, 0, 3, 3, 1, 1, 2, 2], dtype=int)

        def _test_point_cloud(self, gmo: Any) -> None:
            """Tests sensor returns stored in GMO buffer against expected range.

            Args:
                gmo: GMO data object with sensor return fields.
            """
            unit_vecs = np.concatenate(
                [
                    np.cos(np.radians(gmo.x))[..., None],
                    np.sin(np.radians(gmo.x))[..., None],
                    np.sin(np.radians(gmo.y))[..., None],
                ],
                axis=1,
            )
            unit_vecs = unit_vecs / np.linalg.norm(unit_vecs, axis=1, keepdims=True)
            octant = (unit_vecs[:, 0] < 0) * 4 + (unit_vecs[:, 1] < 0) * 2 + (unit_vecs[:, 2] < 0)
            dims = self._octant_dims[octant]

            ratio = np.divide(dims, np.abs(unit_vecs))
            expected_range = np.min(ratio, axis=1)
            plane_idx = np.argmin(ratio, axis=1)

            if DEBUG_DRAW_PRINT and self.valid_frame_count == 0:
                import matplotlib.pyplot as plt

                fig = plt.figure()
                ax = fig.add_subplot(111, projection="3d")
                ax.scatter(
                    np.multiply(unit_vecs[:, 0], gmo.z),
                    np.multiply(unit_vecs[:, 1], gmo.z),
                    np.multiply(unit_vecs[:, 2], gmo.z),
                    c="b",
                )
                ax.scatter(
                    np.multiply(unit_vecs[:, 0], expected_range),
                    np.multiply(unit_vecs[:, 1], expected_range),
                    np.multiply(unit_vecs[:, 2], expected_range),
                    c="r",
                )
                plt.savefig("test_returns_cartesian.png")
                plt.close()

            percent_diffs = np.divide(np.abs(expected_range - gmo.z), expected_range)

            edge_azimuths = np.arange(-180, 181, 45).reshape(1, -1)
            near_edge = np.any(np.abs(gmo.x[:, None] - edge_azimuths) < NEAR_EDGE_THRESHOLD, axis=1)
            self._not_near_edge = ~near_edge

            num_exceeding_threshold = np.sum(np.logical_and(percent_diffs > 2e-2, self._not_near_edge))
            num_returns = np.size(gmo.x)
            carb.log_warn(f"num_returns: {num_returns}")
            pct_exceeding_threshold = num_exceeding_threshold / num_returns * 100
            valid_threshold = 1.0 if num_returns >= 100 else 10.0
            self._test.assertLessEqual(
                pct_exceeding_threshold,
                valid_threshold,
                f"Expected fewer than 1% of returns to differ from expected range by more than 2%. "
                f"{num_exceeding_threshold} of {num_returns} returns exceeded threshold.",
            )

            cube_idx = self._OCTANT_TO_CUBE[octant] * 4 + plane_idx
            cube_idx[np.bitwise_and(octant % 2 == 1, plane_idx == 2)] += 1
            self._cube_prim_paths = cube_idx

        def _test_intensity(self, gmo: Any) -> None:
            if self._sensor_type == "lidar":
                self._test.assertTrue(np.all(gmo.scalar >= 0), "Intensities are not non-negative.")
            elif self._sensor_type == "radar":
                self._test.assertTrue(np.all(gmo.scalar != 0), "Intensities are not zero.")

        def _test_timestamp(self, gmo: Any) -> None:
            timestamp_diffs = np.diff(gmo.timeOffsetNs)
            self._test.assertTrue(np.all(timestamp_diffs >= 0), "Timestamps are not monotonically increasing.")
            max_timestamp_diff = np.max(timestamp_diffs)
            self._test.assertLessEqual(max_timestamp_diff, self._max_timeOffsetNs_expected)

        def _test_emitter_id(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.emitterId >= 0), "Emitter IDs are not non-negative.")
            self._test.assertTrue(np.all(gmo.emitterId < 1024), "Emitter IDs are expected to be less than 1024.")

        def _test_channel_id(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.channelId >= 0), "Channel IDs are not non-negative.")
            self._test.assertTrue(np.all(gmo.channelId < 1024), "Channel IDs are expected to be less than 1024.")

        def _test_material_id(self, gmo: Any) -> None:
            self._test.assertEqual(
                len(gmo.matId),
                len(self._cube_prim_paths),
                "Expected same number of material ids as number of returns.",
            )
            if self._expected_material_ids is None:
                self._expected_material_ids = np.array(
                    [
                        self._test.cube_info[f"/World/cube_{i}"]["material_id"]
                        for i in range(len(self._test.cube_info.keys()))
                    ],
                    dtype=gmo.matId.dtype,
                )
            expected = self._expected_material_ids[self._cube_prim_paths]
            mask = self._not_near_edge
            checked_count = int(np.sum(mask))
            failure_count = int(np.sum(gmo.matId[mask] != expected[mask])) if checked_count > 0 else 0
            failure_pct = (failure_count / checked_count * 100) if checked_count > 0 else 0
            self._test.assertLess(
                failure_pct,
                1.0,
                f"Expected fewer than 1% of returns to fail material ID check. "
                f"{failure_count} of {checked_count} returns ({failure_pct:.2f}%) failed.",
            )

        def _test_velocity(self, gmo: Any) -> None:
            self._test.assertTrue(np.allclose(gmo.velocities, 0, atol=5e-3), "Velocities are expected to be 0.")

        def _test_object_id(self, gmo: Any) -> None:
            self._test.assertGreater(len(self._stable_id_map), 0, "Expected non-empty stable id map.")
            object_ids = np.array(LidarRtx.get_object_ids(gmo.objId))
            self._test.assertEqual(
                len(object_ids),
                len(self._cube_prim_paths),
                "Expected same number of object ids as number of returns.",
            )
            mask = self._not_near_edge
            masked_oids = object_ids[mask]
            unexpected_object_ids = set(masked_oids) - self._stable_id_map.keys()
            self._test.assertFalse(
                len(unexpected_object_ids) > 0,
                f"Expected no unexpected object ids. Unexpected: {unexpected_object_ids}. "
                f"Actual: {set(masked_oids)}, Allowed: {self._stable_id_map.keys()}",
            )
            expected_paths = np.array([f"/World/cube_{i}" for i in self._cube_prim_paths[mask]])
            resolved_paths = np.array([self._stable_id_map[oid] for oid in masked_oids])
            checked_count = len(expected_paths)
            failure_count = int(np.sum(resolved_paths != expected_paths)) if checked_count > 0 else 0
            failure_pct = (failure_count / checked_count * 100) if checked_count > 0 else 0
            self._test.assertLess(
                failure_pct,
                1.0,
                f"Expected fewer than 1% of returns to fail object ID check. "
                f"{failure_count} of {checked_count} returns ({failure_pct:.2f}%) failed.",
            )

        def _test_echo_id(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.echoId == 0), "Echo IDs are expected to be 0.")

        def _test_tick_state(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.tickStates == 0), "Tick states are expected to be 0.")

        def _test_radial_velocity(self, gmo: Any) -> None:
            self._test.assertLessEqual(
                np.max(np.abs(gmo.rv_ms)), 1e-2, "Radial velocity is expected to be (close to) 0."
            )

    _writer_registered = False

    async def setUp(self) -> None:
        """Set up test environment and register GMO writer."""
        await super().setUp()
        if not TestGenericModelOutput._writer_registered:
            rep.WriterRegistry.register(TestGenericModelOutput._GmoTestWriter)
            TestGenericModelOutput._writer_registered = True

    async def _test_sensor(self, sensor_type: str, **kwargs: Any) -> None:
        COLLECTION_SECONDS = 3.0

        sensor_kwargs = {
            "path": "sensor",
            "parent": None,
            "translation": Gf.Vec3d(0.0, 0.0, 0.0),
            "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        }
        sensor_kwargs.update(kwargs)

        _, self.sensor = omni.kit.commands.execute(f"IsaacSensorCreateRtx{sensor_type.capitalize()}", **sensor_kwargs)
        sensor_type_name = self.sensor.GetTypeName()
        self.assertEqual(
            sensor_type_name,
            f"Omni{sensor_type.capitalize()}",
            f"Expected Omni{sensor_type.capitalize()} prim, got {sensor_type_name}. Was sensor prim created?",
        )

        self._hydra_texture = rep.create.render_product(
            self.sensor.GetPath(),
            [32, 32],
            name="RtxSensorRenderProduct",
        )

        self._writer = rep.WriterRegistry.get("_GmoTestWriter")
        self._writer.initialize(test_instance=self, sensor_type=sensor_type, sensor_prim=self.sensor)
        self._writer.attach([self._hydra_texture.path])

        total_frames = int(COLLECTION_SECONDS * 60)
        self._timeline.set_end_time(COLLECTION_SECONDS + 1.0)
        self._timeline.play()
        for _ in range(total_frames):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        carb.log_warn(f"bad magic number frames: {self._writer.bad_magic_count}")
        carb.log_warn(f"numElements==0 frames: {self._writer.num_elements_zero_count}")
        carb.log_warn(f"valid frames: {self._writer.valid_frame_count}")

        self.assertGreater(self._writer.valid_frame_count, 0, "Expected at least one valid GMO frame.")

    async def test_lidar(self) -> None:
        """Test GMO annotator with a lidar sensor."""
        kwargs = {
            "omni:sensor:Core:outputFrameOfReference": "WORLD",
            "omni:sensor:Core:auxOutputType": "FULL",
        }
        await self._test_sensor("lidar", **kwargs)

    async def test_radar(self) -> None:
        """Test GMO annotator with a radar sensor."""
        kwargs = {
            "omni:sensor:WpmDmat:outputFrameOfReference": "WORLD",
            "omni:sensor:WpmDmat:auxOutputType": "BASIC",
        }
        await self._test_sensor("radar", **kwargs)


class TestIsaacCreateRTXLidarScanBuffer(_SarcophagusTestCase):
    """Test the Isaac Create RTX Lidar Scan Buffer annotator."""

    _SCAN_BUFFER_INIT_PARAMS = {
        "outputAzimuth": True,
        "outputElevation": True,
        "outputDistance": True,
        "outputIntensity": True,
        "outputTimestamp": True,
        "outputEmitterId": True,
        "outputChannelId": True,
        "outputMaterialId": True,
        "outputTickId": True,
        "outputHitNormal": True,
        "outputVelocity": True,
        "outputObjectId": True,
        "outputEchoId": True,
        "outputTickState": True,
    }

    class _ScanBufferTestWriter(Writer):
        """Custom Writer that validates IsaacCreateRTXLidarScanBuffer against GenericModelOutput.

        Args:
            test_instance: Test case instance for assertions.
        """

        def __init__(self, test_instance: Any = None) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [
                rep.annotators.get("GenericModelOutput"),
                rep.annotators.get(
                    "IsaacCreateRTXLidarScanBuffer",
                    init_params=TestIsaacCreateRTXLidarScanBuffer._SCAN_BUFFER_INIT_PARAMS,
                ),
            ]
            self._test = test_instance
            self._gmo_buffer = {}
            self.bad_magic_count = 0
            self.num_elements_zero_count = 0
            self.valid_frame_count = 0

        def write(self, data: Any) -> None:
            if "renderProducts" not in data:
                return
            for rp_name, rp_data in data["renderProducts"].items():
                gmo_raw = _extract_gmo_raw(rp_data)
                if gmo_raw is None:
                    continue

                gmo = get_gmo_data(gmo_raw)

                if gmo.magicNumber != gmo_utils.getMagicNumberGMO():
                    self.bad_magic_count += 1
                    return

                if gmo.numElements == 0:
                    self.num_elements_zero_count += 1
                    return

                key = gmo.timestampNs + gmo.timeOffsetNs[0].astype(np.uint64)
                self._gmo_buffer[key] = {
                    "azimuth": gmo.x.copy(),
                    "elevation": gmo.y.copy(),
                    "distance": gmo.z.copy(),
                    "intensity": gmo.scalar.copy(),
                    "timestamp": gmo.timeOffsetNs.astype(np.uint64) + gmo.timestampNs,
                    "emitterId": gmo.emitterId.copy(),
                    "channelId": gmo.channelId.copy(),
                    "materialId": gmo.matId.copy(),
                    "tickId": gmo.tickId.copy(),
                    "hitNormal": gmo.hitNormals.copy().reshape(-1, 3),
                    "velocity": gmo.velocities.copy().reshape(-1, 3),
                    "objectId": gmo.objId.copy(),
                    "echoId": gmo.echoId.copy(),
                    "tickState": gmo.tickStates.copy(),
                }

                sb = rp_data.get("IsaacCreateRTXLidarScanBuffer")
                if sb is None or not isinstance(sb, dict):
                    continue
                if "data" not in sb or not isinstance(sb["data"], np.ndarray) or sb["data"].size == 0:
                    continue

                self._validate_scan_buffer(sb)
                self.valid_frame_count += 1

        def _validate_scan_buffer(self, sb: Any) -> None:
            """Validate scan buffer structure and compare against buffered GMO data.

            Args:
                sb: Scan buffer data dictionary.
            """
            t = self._test

            point_cloud = sb["data"]
            t.assertGreater(point_cloud.shape[0], 0, "Expected non-empty data.")
            t.assertEqual(point_cloud.shape[1], 3)
            t.assertEqual(point_cloud.dtype, np.float32)
            num_points = point_cloud.shape[0]

            t.assertIn("transform", sb)
            transform = sb["transform"]
            t.assertEqual(transform.shape[0], 16, "Expected non-empty transform.")
            t.assertEqual(transform.dtype, np.float64)
            t.assertTrue(
                np.allclose(transform, np.eye(4, dtype=np.float64).flatten()),
                "Expected identity transform.",
            )

            t.assertIn("radialVelocityMS", sb)
            t.assertEqual(sb["radialVelocityMS"].size, 0, "Expected empty radial velocity data.")

            expected_keys = {
                "azimuth": (num_points, np.float32),
                "elevation": (num_points, np.float32),
                "distance": (num_points, np.float32),
                "intensity": (num_points, np.float32),
                "timestamp": (num_points, np.uint64),
                "emitterId": (num_points, np.uint32),
                "channelId": (num_points, np.uint32),
                "materialId": (num_points, np.uint32),
                "tickId": (num_points, np.uint32),
                "hitNormal": (num_points, np.float32),
                "velocity": (num_points, np.float32),
                "objectId": (num_points * 4, np.uint32),
                "echoId": (num_points, np.uint8),
                "tickState": (num_points, np.uint8),
            }

            for attribute, (expected_size, expected_dtype) in expected_keys.items():
                t.assertIn(attribute, sb, f"Expected {attribute} in scan buffer data.")
                attribute_data = sb[attribute]
                t.assertEqual(
                    attribute_data.shape[0],
                    expected_size,
                    f"Expected {attribute} to have {expected_size} points, got {attribute_data.shape[0]}.",
                )
                t.assertEqual(
                    attribute_data.dtype,
                    expected_dtype,
                    f"Expected {attribute} to have dtype {expected_dtype}, got {attribute_data.dtype}.",
                )

            self._compare_to_gmo(sb, expected_keys)

        def _compare_to_gmo(self, sb: Any, expected_keys: Any) -> None:
            """Compare scan buffer output against buffered GMO data.

            Args:
                sb: Scan buffer data dictionary.
                expected_keys: Dictionary of expected attribute keys with (size, dtype) tuples.
            """
            t = self._test
            ts_key = sb["timestamp"][0]
            t.assertIn(
                ts_key,
                self._gmo_buffer.keys(),
                f"Expected timestamp {ts_key} in GMO buffer (keys: {list(self._gmo_buffer.keys())}).",
            )
            gmo_data = self._gmo_buffer[ts_key]

            for attribute in expected_keys:
                attribute_data = sb[attribute]
                comparison_data = gmo_data[attribute]
                expected_shape = comparison_data.shape
                expected_dtype = comparison_data.dtype
                if attribute == "objectId":
                    expected_shape = (comparison_data.shape[0] // 4,)
                    expected_dtype = np.uint32

                t.assertAlmostEqual(
                    attribute_data.shape[0],
                    expected_shape[0],
                    delta=0.01 * expected_shape[0],
                    msg=f"Expected {attribute} shape {expected_shape[0]}, got {attribute_data.shape[0]}.",
                )
                t.assertEqual(
                    attribute_data.dtype,
                    expected_dtype,
                    f"Expected {attribute} dtype {expected_dtype}, got {attribute_data.dtype}.",
                )

                if attribute == "objectId":
                    t.assertTrue(
                        np.all(attribute_data.view(np.uint8) == comparison_data.view(np.uint8)),
                        "Expected objectId to match GMO buffer.",
                    )
                else:
                    t.assertTrue(
                        np.allclose(attribute_data, comparison_data),
                        f"Expected {attribute} to match GMO buffer.",
                    )

    _writer_registered = False

    async def setUp(self) -> None:
        """Set up test environment and register scan buffer writer."""
        await super().setUp()
        if not TestIsaacCreateRTXLidarScanBuffer._writer_registered:
            rep.WriterRegistry.register(TestIsaacCreateRTXLidarScanBuffer._ScanBufferTestWriter)
            TestIsaacCreateRTXLidarScanBuffer._writer_registered = True

    async def _test_annotator_outputs(self) -> None:
        COLLECTION_SECONDS = 3.0

        kwargs = {
            "path": "lidar",
            "parent": None,
            "translation": Gf.Vec3d(0.0, 0.0, 0.0),
            "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            "omni:sensor:Core:outputFrameOfReference": "WORLD",
            "omni:sensor:Core:auxOutputType": "FULL",
        }

        _, self.sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", **kwargs)
        sensor_type = self.sensor.GetTypeName()
        self.assertEqual(
            sensor_type, "OmniLidar", f"Expected OmniLidar prim, got {sensor_type}. Was sensor prim created?"
        )

        self._hydra_texture = rep.create.render_product(
            self.sensor.GetPath(),
            [32, 32],
            name="RtxSensorRenderProduct",
        )

        self._writer = rep.WriterRegistry.get("_ScanBufferTestWriter")
        self._writer.initialize(test_instance=self)
        self._writer.attach([self._hydra_texture.path])

        total_frames = int(COLLECTION_SECONDS * 60)
        self._timeline.set_end_time(COLLECTION_SECONDS + 1.0)
        self._timeline.play()
        for _ in range(total_frames):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        carb.log_warn(f"bad magic number frames: {self._writer.bad_magic_count}")
        carb.log_warn(f"numElements==0 frames: {self._writer.num_elements_zero_count}")
        carb.log_warn(f"valid frames: {self._writer.valid_frame_count}")

        self.assertGreater(self._writer.valid_frame_count, 0, "Expected at least one valid scan buffer frame.")

    async def test_3d_lidar(self) -> None:
        """Test IsaacCreateRTXLidarScanBuffer annotator with a 3D lidar sensor."""
        await self._test_annotator_outputs()


class TestIsaacComputeRTXLidarFlatScan(_SarcophagusTestCase):
    """Test class for IsaacComputeRTXLidarFlatScan annotator."""

    _EXPECTED_FLAT_SCAN_KEYS = [
        "azimuthRange",
        "depthRange",
        "horizontalFov",
        "horizontalResolution",
        "intensitiesData",
        "linearDepthData",
        "numCols",
        "numRows",
        "rotationRate",
    ]

    class _FlatScanTestWriter(Writer):
        """Custom Writer that validates IsaacComputeRTXLidarFlatScan against GenericModelOutput.

        Args:
            test_instance: Test case instance for assertions.
            sensor_attrs: Dictionary of sensor attributes for validation.
        """

        def __init__(self, test_instance: Any = None, sensor_attrs: Any = None) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [
                rep.annotators.get("GenericModelOutput"),
                rep.annotators.get("IsaacComputeRTXLidarFlatScan"),
            ]
            self._test = test_instance
            self._attrs = sensor_attrs
            self._is_2d_lidar = all(abs(e) < 1e-3 for e in sensor_attrs["elevationDeg"])
            carb.log_warn(f"is_2d_lidar: {self._is_2d_lidar}")
            self.bad_magic_count = 0
            self.num_elements_zero_count = 0
            self.valid_frame_count = 0

        def write(self, data: Any) -> None:
            if "renderProducts" not in data:
                return
            for rp_name, rp_data in data["renderProducts"].items():
                gmo_raw = _extract_gmo_raw(rp_data)
                if gmo_raw is None:
                    continue

                gmo = get_gmo_data(gmo_raw)

                if gmo.magicNumber != gmo_utils.getMagicNumberGMO():
                    self.bad_magic_count += 1
                    return

                if gmo.numElements == 0:
                    self.num_elements_zero_count += 1
                    return

                fs = rp_data.get("IsaacComputeRTXLidarFlatScan")
                if fs is None or not isinstance(fs, dict):
                    continue
                if "numCols" not in fs:
                    continue

                self._validate_flat_scan(fs, gmo)
                self.valid_frame_count += 1

        def _validate_flat_scan(self, fs: Any, gmo: Any) -> None:
            t = self._test

            for key in TestIsaacComputeRTXLidarFlatScan._EXPECTED_FLAT_SCAN_KEYS:
                t.assertIn(key, fs, f"Expected {key} in flat scan data.")

            if not self._is_2d_lidar:
                t.assertTrue(
                    np.allclose(np.zeros([1, 2]), fs["azimuthRange"]),
                    f"azimuthRange: {fs['azimuthRange']}",
                )
                t.assertTrue(
                    np.allclose(np.zeros([1, 2]), fs["depthRange"]),
                    f"depthRange: {fs['depthRange']}",
                )
                t.assertEqual(0.0, fs["horizontalFov"])
                t.assertEqual(0.0, fs["horizontalResolution"])
                t.assertEqual(0, fs["intensitiesData"].size)
                t.assertEqual(0, fs["linearDepthData"].size)
                t.assertEqual(0, fs["numCols"])
                t.assertEqual(1, fs["numRows"])
                t.assertEqual(0.0, fs["rotationRate"])
                return

            if fs["numCols"] == 0:
                return

            attrs = self._attrs
            if attrs["is_solid_state"]:
                expectedMinAzimuth = min(attrs["azimuthDeg"])
                expectedMaxAzimuth = max(attrs["azimuthDeg"])
                expectedHorizontalFov = expectedMaxAzimuth - expectedMinAzimuth
                expectedHorizontalResolution = expectedHorizontalFov / len(attrs["azimuthDeg"])
                if expectedMaxAzimuth > 180.0:
                    expectedMinAzimuth -= 180.0
                    expectedMaxAzimuth -= 180.0
            else:
                expectedMinAzimuth = -180.0
                expectedMaxAzimuth = 180.0
                expectedHorizontalFov = 360.0
                expectedHorizontalResolution = 360.0 * attrs["patternFiringRateHz"] / attrs["scanRateBaseHz"]

            t.assertAlmostEqual(fs["azimuthRange"][0], expectedMinAzimuth)
            t.assertAlmostEqual(fs["azimuthRange"][1], expectedMaxAzimuth - expectedHorizontalResolution)
            t.assertAlmostEqual(fs["horizontalResolution"], expectedHorizontalResolution)
            t.assertAlmostEqual(fs["horizontalFov"], expectedHorizontalFov)
            t.assertAlmostEqual(fs["rotationRate"], attrs["scanRateBaseHz"])
            t.assertAlmostEqual(fs["depthRange"][0], attrs["nearRangeM"])
            t.assertAlmostEqual(fs["depthRange"][1], attrs["farRangeM"])

            expectedNumCols = round(fs["horizontalFov"] / fs["horizontalResolution"])
            t.assertEqual(fs["numCols"], expectedNumCols)
            t.assertEqual(fs["numRows"], 1)
            t.assertEqual(fs["linearDepthData"].size, expectedNumCols)
            t.assertEqual(fs["intensitiesData"].size, expectedNumCols)

            indices = np.clip(
                ((gmo.x - expectedMinAzimuth) / expectedHorizontalResolution).astype(int),
                0,
                expectedNumCols - 1,
            )
            expectedLinearDepthData = np.full(expectedNumCols, -1.0, dtype=np.float32)
            expectedIntensitiesData = np.zeros(expectedNumCols, dtype=np.uint8)
            expectedLinearDepthData[indices] = gmo.z
            expectedIntensitiesData[indices] = (gmo.scalar * 255.0).astype(np.uint8)

            t.assertTrue(
                np.allclose(fs["linearDepthData"], expectedLinearDepthData),
                "linearDepthData does not match expected values from GMO.",
            )
            t.assertTrue(
                np.allclose(fs["intensitiesData"], expectedIntensitiesData),
                "intensitiesData does not match expected values from GMO.",
            )

    _writer_registered = False

    async def setUp(self) -> None:
        """Set up test environment and register flat scan writer."""
        await super().setUp()
        if not TestIsaacComputeRTXLidarFlatScan._writer_registered:
            rep.WriterRegistry.register(TestIsaacComputeRTXLidarFlatScan._FlatScanTestWriter)
            TestIsaacComputeRTXLidarFlatScan._writer_registered = True

    async def _test_annotator_outputs(self, config: str = DEFAULT_CONFIG, variant: str = DEFAULT_VARIANT) -> None:
        COLLECTION_SECONDS = 3.0

        kwargs = {
            "path": "lidar",
            "parent": None,
            "translation": Gf.Vec3d(0.0, 0.0, 0.0),
            "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            "config": config,
            "variant": variant,
            "omni:sensor:Core:outputFrameOfReference": "WORLD",
            "omni:sensor:Core:auxOutputType": "BASIC",
        }

        _, self.sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", **kwargs)
        sensor_type = self.sensor.GetTypeName()
        self.assertEqual(
            sensor_type, "OmniLidar", f"Expected OmniLidar prim, got {sensor_type}. Was sensor prim created?"
        )

        sensor_attrs = {
            "azimuthDeg": self.sensor.GetAttribute("omni:sensor:Core:emitterState:s001:azimuthDeg").Get(),
            "elevationDeg": self.sensor.GetAttribute("omni:sensor:Core:emitterState:s001:elevationDeg").Get(),
            "scanRateBaseHz": self.sensor.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get(),
            "patternFiringRateHz": self.sensor.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get(),
            "nearRangeM": self.sensor.GetAttribute("omni:sensor:Core:nearRangeM").Get(),
            "farRangeM": self.sensor.GetAttribute("omni:sensor:Core:farRangeM").Get(),
            "is_solid_state": self.sensor.GetAttribute("omni:sensor:Core:scanType").Get() == "SOLID_STATE",
        }

        self._hydra_texture = rep.create.render_product(
            self.sensor.GetPath(),
            [32, 32],
            name="RtxSensorRenderProduct",
        )

        self._writer = rep.WriterRegistry.get("_FlatScanTestWriter")
        self._writer.initialize(test_instance=self, sensor_attrs=sensor_attrs)
        self._writer.attach([self._hydra_texture.path])

        total_frames = int(COLLECTION_SECONDS * 60)
        self._timeline.set_end_time(COLLECTION_SECONDS + 1.0)
        self._timeline.play()
        for _ in range(total_frames):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        carb.log_warn(f"bad magic number frames: {self._writer.bad_magic_count}")
        carb.log_warn(f"numElements==0 frames: {self._writer.num_elements_zero_count}")
        carb.log_warn(f"valid frames: {self._writer.valid_frame_count}")

        self.assertGreater(self._writer.valid_frame_count, 0, "Expected at least one valid flat scan frame.")

    async def test_3d_lidar(self) -> None:
        """Test IsaacComputeRTXLidarFlatScan annotator with a 3D lidar sensor."""
        await self._test_annotator_outputs(config="Example_Rotary", variant=None)

    async def test_2d_lidar(self) -> None:
        """Test IsaacComputeRTXLidarFlatScan annotator with a 2D lidar sensor."""
        await self._test_annotator_outputs(config="SICK_picoScan150", variant="Profile_1")


class TestIsaacCreateRTXRadarPointCloud(_SarcophagusTestCase):
    """Test class for IsaacCreateRTXRadarPointCloud annotator."""

    class _RadarTestWriter(Writer):
        """Custom Writer that validates IsaacCreateRTXRadarPointCloud output.

        Args:
            test_instance: Test case instance for assertions.
        """

        def __init__(self, test_instance: Any = None) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [
                rep.annotators.get(
                    "IsaacCreateRTXRadarPointCloud",
                    init_params={"outputIntensity": True, "outputRadialVelocityMS": True},
                ),
            ]
            self._test = test_instance
            self.valid_frame_count = 0

        def write(self, data: Any) -> None:
            if "renderProducts" not in data:
                return
            for rp_name, rp_data in data["renderProducts"].items():
                rd = rp_data.get("IsaacCreateRTXRadarPointCloud")
                if rd is None or not isinstance(rd, dict):
                    continue
                if "data" not in rd or not isinstance(rd["data"], np.ndarray) or rd["data"].size == 0:
                    continue
                self._validate(rd)
                self.valid_frame_count += 1

        def _validate(self, rd: Any) -> None:
            t = self._test
            for key in ["data", "intensity", "radialVelocityMS"]:
                t.assertIn(key, rd, f"Expected {key} in radar point cloud data.")

            t.assertGreater(rd["data"].shape[0], 0, "Expected non-empty data.")
            t.assertEqual(rd["data"].shape[1], 3)
            t.assertTrue(np.all(rd["intensity"] != 0))
            t.assertLessEqual(
                np.max(np.abs(rd["radialVelocityMS"])), 1e-2, "Radial velocity is expected to be (close to) 0."
            )

    _writer_registered = False

    async def setUp(self) -> None:
        """Set up test environment and register radar writer."""
        await super().setUp()
        if not TestIsaacCreateRTXRadarPointCloud._writer_registered:
            rep.WriterRegistry.register(TestIsaacCreateRTXRadarPointCloud._RadarTestWriter)
            TestIsaacCreateRTXRadarPointCloud._writer_registered = True

    async def test_rtx_radar(self) -> None:
        """Test IsaacCreateRTXRadarPointCloud annotator with a radar sensor."""
        COLLECTION_SECONDS = 3.0

        kwargs = {
            "path": "radar",
            "parent": None,
            "translation": Gf.Vec3d(0.0, 0.0, 0.0),
            "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            "omni:sensor:WpmDmat:outputFrameOfReference": "WORLD",
            "omni:sensor:WpmDmat:auxOutputType": "BASIC",
        }

        _, self.sensor = omni.kit.commands.execute("IsaacSensorCreateRtxRadar", **kwargs)
        sensor_type = self.sensor.GetTypeName()
        self.assertEqual(
            sensor_type, "OmniRadar", f"Expected OmniRadar prim, got {sensor_type}. Was sensor prim created?"
        )

        self._hydra_texture = rep.create.render_product(
            self.sensor.GetPath(),
            [32, 32],
            name="RtxSensorRenderProduct",
            render_vars=["GenericModelOutput"],
        )

        self._writer = rep.WriterRegistry.get("_RadarTestWriter")
        self._writer.initialize(test_instance=self)
        self._writer.attach([self._hydra_texture.path])

        total_frames = int(COLLECTION_SECONDS * 60)
        self._timeline.set_end_time(COLLECTION_SECONDS + 1.0)
        self._timeline.play()
        for _ in range(total_frames):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        carb.log_warn(f"valid radar frames: {self._writer.valid_frame_count}")
        self.assertGreater(self._writer.valid_frame_count, 0, "Expected at least one valid radar frame.")


class TestTimestepVsScanRate(omni.kit.test.AsyncTestCase):
    """Verify GMO timestamps advance by scan_period across dt / scan-rate combos.

    Exercises all 27 combinations of:
      - physics_dt:    1/30 s, 1/60 s, 1/120 s  (via SimulationManager)
      - timeline_dt:   1/30 s, 1/60 s, 1/120 s  (via RenderingManager)
      - scan rate:     5 Hz, 10 Hz, 20 Hz        (via scanRateBaseHz + tickRate)

    Asserted invariants:
      1. First valid timestamp within pipeline warmup budget (~0.25 s).
      2. Every new scan has ``numElements > 0`` and ``scanComplete == True``.
      3. Timestamp advances by exactly ``scan_period`` between consecutive scans.
    """

    COLLECTION_SECONDS = 3.0
    TIMESTAMP_TOLERANCE_NS = 10

    class _CadenceTestWriter(Writer):
        """Writer that collects unique GMO scan timestamps."""

        def __init__(self) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [rep.annotators.get("GenericModelOutput")]
            self.scans = []
            self.warmup_count = 0
            self._last_ts = None

        def reset(self) -> None:
            self.scans.clear()
            self.warmup_count = 0
            self._last_ts = None

        def write(self, data: Any) -> None:
            if "renderProducts" not in data:
                self.warmup_count += 1
                return

            for rp_name, rp_data in data["renderProducts"].items():
                gmo_raw = _extract_gmo_raw(rp_data)
                if gmo_raw is None:
                    self.warmup_count += 1
                    return

                gmo = get_gmo_data(gmo_raw)
                if gmo.magicNumber != gmo_utils.getMagicNumberGMO():
                    self.warmup_count += 1
                    return

                if gmo.numElements == 0:
                    return

                ts = int(gmo.timestampNs)
                if ts != self._last_ts:
                    self.scans.append((ts, gmo.numElements, gmo.scanComplete))
                    self._last_ts = ts

    _writer_registered = False

    async def setUp(self) -> None:
        """Set up test environment with a new stage and cadence writer."""
        await create_new_stage_async()
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        create_sarcophagus(enable_nonvisual_material=False)
        self._timeline = omni.timeline.get_timeline_interface()
        self._hydra_texture = None
        self._writer = None
        if not TestTimestepVsScanRate._writer_registered:
            rep.WriterRegistry.register(TestTimestepVsScanRate._CadenceTestWriter)
            TestTimestepVsScanRate._writer_registered = True

    async def tearDown(self) -> None:
        """Tear down the test environment and stop timeline."""
        self._timeline.stop()
        if self._writer is not None:
            self._writer.detach()
            self._writer = None
        if self._hydra_texture is not None:
            self._hydra_texture.destroy()
            self._hydra_texture = None
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        await update_stage_async()

    async def _run_test(self, physics_dt: float, timeline_dt: float, scan_rate_hz: float) -> None:
        """Run a single combination and assert timestamp invariants.

        Args:
            physics_dt: Physics simulation time step in seconds.
            timeline_dt: Timeline rendering time step in seconds.
            scan_rate_hz: Lidar scan rate in Hz.
        """
        label = f"phys={physics_dt:.6f} tl={timeline_dt:.6f} scan={scan_rate_hz}Hz"

        SimulationManager.set_physics_dt(dt=physics_dt)
        SimulationManager.initialize_physics()
        RenderingManager.set_dt(dt=timeline_dt)

        kwargs = {
            "path": "lidar",
            "parent": None,
            "translation": Gf.Vec3d(0.0, 0.0, 0.0),
            "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            "omni:sensor:Core:auxOutputType": "FULL",
            "omni:sensor:Core:scanRateBaseHz": scan_rate_hz,
            "omni:sensor:tickRate": scan_rate_hz,
        }
        _, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", **kwargs)

        self._hydra_texture = rep.create.render_product(
            sensor.GetPath(),
            [32, 32],
            name="RtxSensorRenderProduct",
        )

        self._writer = rep.WriterRegistry.get("_CadenceTestWriter")
        self._writer.initialize()
        self._writer.attach([self._hydra_texture.path])

        scan_period = 1.0 / scan_rate_hz
        scan_period_ns = round(scan_period * 1e9)
        total_frames = int(self.COLLECTION_SECONDS / timeline_dt)

        self._writer.reset()
        self._timeline.set_end_time(self.COLLECTION_SECONDS + 1.0)
        self._timeline.play()
        for _ in range(total_frames):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        scans = self._writer.scans
        self.assertGreater(len(scans), 2, f"[{label}] Need ≥3 scans for timestamp verification")

        # -- 1. Warmup: first scan timestamp within budget --
        warmup_budget_s = 0.25
        max_missed = max(1, math.ceil(warmup_budget_s / scan_period))
        max_first_ts = max_missed * scan_period_ns
        first_ts = scans[0][0]
        carb.log_warn(
            f"[{label}] warmup={self._writer.warmup_count} frames,"
            f" first_ts={first_ts}ns (max={max_first_ts}ns, {max_missed} scan(s))"
        )
        self.assertLessEqual(
            first_ts,
            max_first_ts,
            f"[{label}] First valid ts {first_ts}ns > {max_first_ts}ns" f" (missed >{max_missed} scans)",
        )

        # -- 2. Every new scan: numElements > 0 and scanComplete == True --
        for i, (ts, ne, sc) in enumerate(scans):
            self.assertGreater(ne, 0, f"[{label}] scan {i}: numElements should be > 0")
            self.assertTrue(sc, f"[{label}] scan {i}: scanComplete should be True")

        # -- 3. Timestamp delta = scan_period_ns --
        # Skip first segment (scan #0 → #1): scan #0 is a backlogged
        # pipeline-warmup scan whose timestamp delta is unreliable.
        for i in range(2, len(scans)):
            ts_delta = scans[i][0] - scans[i - 1][0]
            self.assertAlmostEqual(
                ts_delta,
                scan_period_ns,
                delta=self.TIMESTAMP_TOLERANCE_NS,
                msg=f"[{label}] scan {i}: ts delta {ts_delta} != expected {scan_period_ns}",
            )

    # ------------------------------------------------------------------
    # 27 combinations: physics_dt x timeline_dt x scan_rate
    # ------------------------------------------------------------------

    # --- physics_dt = 1/30 ---

    async def test_phys30_tl30_scan5(self) -> None:
        """Test phys=1/30s, timeline=1/30s, scan=5Hz."""
        await self._run_test(1.0 / 30, 1.0 / 30, 5.0)

    async def test_phys30_tl30_scan10(self) -> None:
        """Test phys=1/30s, timeline=1/30s, scan=10Hz."""
        await self._run_test(1.0 / 30, 1.0 / 30, 10.0)

    async def test_phys30_tl30_scan20(self) -> None:
        """Test phys=1/30s, timeline=1/30s, scan=20Hz."""
        await self._run_test(1.0 / 30, 1.0 / 30, 20.0)

    async def test_phys30_tl60_scan5(self) -> None:
        """Test phys=1/30s, timeline=1/60s, scan=5Hz."""
        await self._run_test(1.0 / 30, 1.0 / 60, 5.0)

    async def test_phys30_tl60_scan10(self) -> None:
        """Test phys=1/30s, timeline=1/60s, scan=10Hz."""
        await self._run_test(1.0 / 30, 1.0 / 60, 10.0)

    async def test_phys30_tl60_scan20(self) -> None:
        """Test phys=1/30s, timeline=1/60s, scan=20Hz."""
        await self._run_test(1.0 / 30, 1.0 / 60, 20.0)

    async def test_phys30_tl120_scan5(self) -> None:
        """Test phys=1/30s, timeline=1/120s, scan=5Hz."""
        await self._run_test(1.0 / 30, 1.0 / 120, 5.0)

    async def test_phys30_tl120_scan10(self) -> None:
        """Test phys=1/30s, timeline=1/120s, scan=10Hz."""
        await self._run_test(1.0 / 30, 1.0 / 120, 10.0)

    async def test_phys30_tl120_scan20(self) -> None:
        """Test phys=1/30s, timeline=1/120s, scan=20Hz."""
        await self._run_test(1.0 / 30, 1.0 / 120, 20.0)

    # --- physics_dt = 1/60 ---

    async def test_phys60_tl30_scan5(self) -> None:
        """Test phys=1/60s, timeline=1/30s, scan=5Hz."""
        await self._run_test(1.0 / 60, 1.0 / 30, 5.0)

    async def test_phys60_tl30_scan10(self) -> None:
        """Test phys=1/60s, timeline=1/30s, scan=10Hz."""
        await self._run_test(1.0 / 60, 1.0 / 30, 10.0)

    async def test_phys60_tl30_scan20(self) -> None:
        """Test phys=1/60s, timeline=1/30s, scan=20Hz."""
        await self._run_test(1.0 / 60, 1.0 / 30, 20.0)

    async def test_phys60_tl60_scan5(self) -> None:
        """Test phys=1/60s, timeline=1/60s, scan=5Hz."""
        await self._run_test(1.0 / 60, 1.0 / 60, 5.0)

    async def test_phys60_tl60_scan10(self) -> None:
        """Test phys=1/60s, timeline=1/60s, scan=10Hz."""
        await self._run_test(1.0 / 60, 1.0 / 60, 10.0)

    async def test_phys60_tl60_scan20(self) -> None:
        """Test phys=1/60s, timeline=1/60s, scan=20Hz."""
        await self._run_test(1.0 / 60, 1.0 / 60, 20.0)

    async def test_phys60_tl120_scan5(self) -> None:
        """Test phys=1/60s, timeline=1/120s, scan=5Hz."""
        await self._run_test(1.0 / 60, 1.0 / 120, 5.0)

    async def test_phys60_tl120_scan10(self) -> None:
        """Test phys=1/60s, timeline=1/120s, scan=10Hz."""
        await self._run_test(1.0 / 60, 1.0 / 120, 10.0)

    async def test_phys60_tl120_scan20(self) -> None:
        """Test phys=1/60s, timeline=1/120s, scan=20Hz."""
        await self._run_test(1.0 / 60, 1.0 / 120, 20.0)

    # --- physics_dt = 1/120 ---

    async def test_phys120_tl30_scan5(self) -> None:
        """Test phys=1/120s, timeline=1/30s, scan=5Hz."""
        await self._run_test(1.0 / 120, 1.0 / 30, 5.0)

    async def test_phys120_tl30_scan10(self) -> None:
        """Test phys=1/120s, timeline=1/30s, scan=10Hz."""
        await self._run_test(1.0 / 120, 1.0 / 30, 10.0)

    async def test_phys120_tl30_scan20(self) -> None:
        """Test phys=1/120s, timeline=1/30s, scan=20Hz."""
        await self._run_test(1.0 / 120, 1.0 / 30, 20.0)

    async def test_phys120_tl60_scan5(self) -> None:
        """Test phys=1/120s, timeline=1/60s, scan=5Hz."""
        await self._run_test(1.0 / 120, 1.0 / 60, 5.0)

    async def test_phys120_tl60_scan10(self) -> None:
        """Test phys=1/120s, timeline=1/60s, scan=10Hz."""
        await self._run_test(1.0 / 120, 1.0 / 60, 10.0)

    async def test_phys120_tl60_scan20(self) -> None:
        """Test phys=1/120s, timeline=1/60s, scan=20Hz."""
        await self._run_test(1.0 / 120, 1.0 / 60, 20.0)

    async def test_phys120_tl120_scan5(self) -> None:
        """Test phys=1/120s, timeline=1/120s, scan=5Hz."""
        await self._run_test(1.0 / 120, 1.0 / 120, 5.0)

    async def test_phys120_tl120_scan10(self) -> None:
        """Test phys=1/120s, timeline=1/120s, scan=10Hz."""
        await self._run_test(1.0 / 120, 1.0 / 120, 10.0)

    async def test_phys120_tl120_scan20(self) -> None:
        """Test phys=1/120s, timeline=1/120s, scan=20Hz."""
        await self._run_test(1.0 / 120, 1.0 / 120, 20.0)
