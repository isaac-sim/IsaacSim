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

"""Verify LidarSensor GMO writer integration, stable IDs, object IDs, and auxiliary output channels."""

from typing import Any

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import (
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
    parse_object_ids,
    parse_stable_id_map_data,
)
from omni.replicator.core import Writer
from pxr import Sdf

from .common import create_sarcophagus

NEAR_EDGE_THRESHOLD = 0.5  # degrees — skip returns near octant edges

# Half-dimensions of the 8 sarcophagus octants (x, y, z), derived from
# the cube layout in create_sarcophagus(). Each pair of entries corresponds
# to one of the four quadrants (top / bottom).
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

# Maps an octant index to the base cube index used by create_sarcophagus().
_OCTANT_TO_CUBE = np.array([0, 0, 3, 3, 1, 1, 2, 2], dtype=int)


class TestLidarSensor(omni.kit.test.AsyncTestCase):
    """Test rtx lidar sensor via a Writer attached through LidarSensor."""

    # ------------------------------------------------------------------
    # Writer
    # ------------------------------------------------------------------

    class _GmoTestWriter(Writer):
        """Custom Writer that validates GenericModelOutput data each frame.

        Args:
            test_instance: Test case instance used for assertions.
            sensor_prim: Sensor prim used to derive timing expectations.
        """

        def __init__(self, test_instance: Any = None, sensor_prim: Any = None) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [
                rep.annotators.get("GenericModelOutput"),
                rep.annotators.get("StableIdMap"),
            ]
            self._test = test_instance
            self._prev_timestamp_ns = None
            self._stable_id_map = None
            self._octant_dims = np.array(_OCTANT_DIMENSIONS)
            self._expected_material_ids = None
            self.num_elements_zero_count = 0
            self.valid_frame_count = 0
            self._sensor_prim = sensor_prim

            tick_rate = sensor_prim.GetAttribute("omni:sensor:tickRate").Get()
            pattern_rate_hz = sensor_prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get()
            scan_rate_base_hz = sensor_prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get()
            fire_time_ns = np.array(sensor_prim.GetAttribute("omni:sensor:Core:emitterState:s001:fireTimeNs").Get())
            self._scan_period_ns = 1.0 / scan_rate_base_hz * 1e9
            self._tick_period_ns = 1.0 / pattern_rate_hz * 1e9
            self._expected_advance_ns = round(1.0 / tick_rate * 1e9) if tick_rate > 0 else 0

        # -- frame callback ------------------------------------------

        def write(self, data: Any) -> None:
            if "renderProducts" not in data:
                return
            for _rp_name, rp_data in data["renderProducts"].items():
                gmo_raw = rp_data.get("GenericModelOutput")
                if isinstance(gmo_raw, dict):
                    gmo_raw = gmo_raw.get("data")

                gmo = parse_generic_model_output_data(gmo_raw)

                if gmo.numElements == 0:
                    self.num_elements_zero_count += 1
                    return

                ts = int(gmo.timestampNs)

                if self._prev_timestamp_ns is not None and self._expected_advance_ns > 0:
                    delta = ts - self._prev_timestamp_ns
                    self._test.assertAlmostEqual(delta, self._expected_advance_ns, delta=10)
                self._prev_timestamp_ns = ts

                self.valid_frame_count += 1
                if self.valid_frame_count == 1:
                    return

                if self._stable_id_map is None:
                    sid_raw = rp_data.get("StableIdMap")
                    if isinstance(sid_raw, dict):
                        sid_raw = sid_raw.get("data")
                    if sid_raw is not None:
                        self._stable_id_map = parse_stable_id_map_data(sid_raw)

                self._test_point_cloud(gmo)
                self._test_intensity(gmo)
                self._test_timestamp(gmo)
                self._test_emitter_id(gmo)
                self._test_channel_id(gmo)
                self._test_material_id(gmo)
                self._test_velocity(gmo)
                self._test_object_id(gmo)
                self._test_echo_id(gmo)
                self._test_tick_state(gmo)

        # -- per-field validators -------------------------------------

        def _test_point_cloud(self, gmo: Any) -> None:
            """Validate sensor returns against expected range from sarcophagus geometry.

            Args:
                gmo: Parsed GenericModelOutput data.
            """
            unit_vecs = np.stack(
                [np.cos(np.radians(gmo.x)), np.sin(np.radians(gmo.x)), np.sin(np.radians(gmo.y))],
                axis=1,
            )

            unit_vecs /= np.linalg.norm(unit_vecs, axis=1, keepdims=True)
            octant = (unit_vecs[:, 0] < 0) * 4 + (unit_vecs[:, 1] < 0) * 2 + (unit_vecs[:, 2] < 0)
            dims = self._octant_dims[octant]

            with np.errstate(divide="ignore"):
                ratio = np.divide(dims, np.abs(unit_vecs))
            expected_range = np.min(ratio, axis=1)
            plane_idx = np.argmin(ratio, axis=1)

            percent_diffs = np.divide(np.abs(expected_range - gmo.z), expected_range)

            edge_azimuths = np.arange(-180, 181, 45).reshape(1, -1)
            near_edge = np.any(np.abs(gmo.x[:, None] - edge_azimuths) < NEAR_EDGE_THRESHOLD, axis=1)
            self._not_near_edge = ~near_edge

            num_exceeding = np.sum(np.logical_and(percent_diffs > 2e-2, self._not_near_edge))
            num_returns = np.size(gmo.x)
            pct_exceeding = num_exceeding / num_returns * 100
            valid_threshold = 1.0 if num_returns >= 100 else 10.0
            self._test.assertLessEqual(
                pct_exceeding,
                valid_threshold,
                f"{num_exceeding} of {num_returns} returns exceeded 2% range threshold.",
            )

            cube_idx = _OCTANT_TO_CUBE[octant] * 4 + plane_idx
            cube_idx[np.bitwise_and(octant % 2 == 1, plane_idx == 2)] += 1
            self._cube_prim_paths = cube_idx

        def _test_intensity(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.scalar >= 0), "Intensities must be non-negative.")

        def _test_timestamp(self, gmo: Any) -> None:
            self._test.assertEqual(np.min(gmo.timeOffsetNs), 0, "Time offsets must be non-negative.")
            self._test.assertLessEqual(
                np.max(gmo.timeOffsetNs), self._scan_period_ns, "Time offsets must be less than scan period."
            )
            max_gap = np.max(np.diff(np.sort(gmo.timeOffsetNs)))
            self._test.assertLessEqual(max_gap, self._tick_period_ns, "Time offsets must be less than tick period.")

        def _test_emitter_id(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.emitterId >= 0), "Emitter IDs must be non-negative.")
            self._test.assertTrue(np.all(gmo.emitterId < 1024), "Emitter IDs must be < 1024.")

        def _test_channel_id(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.channelId >= 0), "Channel IDs must be non-negative.")
            self._test.assertTrue(np.all(gmo.channelId < 1024), "Channel IDs must be < 1024.")

        def _test_material_id(self, gmo: Any) -> None:
            self._test.assertEqual(len(gmo.matId), len(self._cube_prim_paths))
            if self._expected_material_ids is None:
                self._expected_material_ids = np.array(
                    [self._test.cube_info[f"/World/cube_{i}"]["material_id"] for i in range(len(self._test.cube_info))],
                    dtype=gmo.matId.dtype,
                )
            expected = self._expected_material_ids[self._cube_prim_paths]
            mask = self._not_near_edge
            checked = int(np.sum(mask))
            failures = int(np.sum(gmo.matId[mask] != expected[mask])) if checked > 0 else 0
            failure_pct = (failures / checked * 100) if checked > 0 else 0
            self._test.assertLess(failure_pct, 1.0, f"{failures}/{checked} material-ID mismatches.")

        def _test_velocity(self, gmo: Any) -> None:
            self._test.assertTrue(np.allclose(gmo.velocities, 0, atol=5e-3), "Velocities should be ~0.")

        def _test_object_id(self, gmo: Any) -> None:
            if self._stable_id_map is None:
                return
            self._test.assertGreater(len(self._stable_id_map), 0)
            obj_ids = parse_object_ids(gmo.objId)
            self._test.assertEqual(len(obj_ids), len(self._cube_prim_paths))
            mask = self._not_near_edge
            masked_oids = [oid for oid, m in zip(obj_ids, mask) if m]
            unexpected = set(masked_oids) - self._stable_id_map.keys()
            self._test.assertFalse(len(unexpected) > 0, f"Unexpected object IDs: {unexpected}")
            expected_paths = np.array([f"/World/cube_{i}" for i in self._cube_prim_paths[mask]])
            resolved_paths = np.array([self._stable_id_map.get(oid, "") for oid in masked_oids])
            checked = len(expected_paths)
            failures = int(np.sum(resolved_paths != expected_paths)) if checked > 0 else 0
            failure_pct = (failures / checked * 100) if checked > 0 else 0
            self._test.assertLess(failure_pct, 1.0, f"{failures}/{checked} object-ID mismatches.")

        def _test_echo_id(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.echoId == 0), "Echo IDs should be 0.")

        def _test_tick_state(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.tickStates == 0), "Tick states should be 0.")

    # ------------------------------------------------------------------
    # Test lifecycle
    # ------------------------------------------------------------------

    _writer_registered = False

    async def setUp(self) -> None:
        """Create sarcophagus geometry and register the lidar GMO validation writer."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        self.cube_info = create_sarcophagus()
        self._timeline = omni.timeline.get_timeline_interface()
        if not TestLidarSensor._writer_registered:
            rep.WriterRegistry.register(TestLidarSensor._GmoTestWriter)
            TestLidarSensor._writer_registered = True

    async def tearDown(self) -> None:
        """Stop lidar playback and flush one app update after each runtime test."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_gmo_writer(self) -> None:
        """Validate GenericModelOutput via a Writer attached to a LidarSensor."""
        COLLECTION_SECONDS = 3.0

        lidar = Lidar(
            "/World/lidar",
            attributes={
                "omni:sensor:Core:outputFrameOfReference": "WORLD",
            },
            aux_output_level="FULL",
        )
        sensor = LidarSensor(lidar, annotators=["generic-model-output"])
        writer = sensor.attach_writer(
            "_GmoTestWriter",
            test_instance=self,
            sensor_prim=lidar.prims[0],
        )

        total_frames = int(COLLECTION_SECONDS * 60)
        self._timeline.set_end_time(COLLECTION_SECONDS + 1.0)
        self._timeline.play()
        for _ in range(total_frames):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        carb.log_warn(f"numElements==0 frames: {writer.num_elements_zero_count}")
        carb.log_warn(f"valid frames: {writer.valid_frame_count}")

        self.assertGreater(writer.valid_frame_count, 0, "Expected at least one valid GMO frame.")

    async def test_aux_output_level_sets_channels_attribute(self) -> None:
        """Verify aux_output_level sets the channels attribute on the sensor prim."""
        for level in ("NONE", "BASIC", "EXTRA", "FULL"):
            await stage_utils.create_new_stage_async()
            await ViewportManager.wait_for_viewport_async()
            create_sarcophagus()

            lidar = Lidar("/World/lidar", aux_output_level=level)
            sensor = LidarSensor(lidar, annotators=["generic-model-output"])
            self.assertEqual(lidar.aux_output_level, level)

            prim = prim_utils.get_prim_at_path(lidar.paths[0])
            attr = prim.GetAttribute("_replicator:rendervar:GenericModelOutput:channels")
            self.assertTrue(attr.IsValid())
            self.assertEqual(attr.GetTypeName(), Sdf.ValueTypeNames.StringArray)
            self.assertEqual(list(attr.Get()), [level])

            del sensor
            await omni.kit.app.get_app().next_update_async()

    async def test_aux_output_level_default_is_none(self) -> None:
        """Verify the default aux_output_level is NONE."""
        lidar = Lidar("/World/lidar")
        self.assertEqual(lidar.aux_output_level, "NONE")

    async def test_aux_output_level_invalid_raises(self) -> None:
        """Verify invalid aux_output_level raises ValueError."""
        with self.assertRaises(ValueError):
            Lidar("/World/lidar", aux_output_level="INVALID")
