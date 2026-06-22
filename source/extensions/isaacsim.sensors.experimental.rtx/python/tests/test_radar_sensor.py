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

"""Verify RadarSensor GMO writer integration, static-scene returns, and auxiliary output channels."""

from typing import Any

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import (
    Radar,
    RadarSensor,
    parse_generic_model_output_data,
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

# Radar uses a fixed 60 Hz tick rate.
_RADAR_TICK_RATE_HZ = 60.0


class TestRadarSensor(omni.kit.test.AsyncTestCase):
    """Test rtx radar sensor via a Writer attached through RadarSensor."""

    # ------------------------------------------------------------------
    # Writer
    # ------------------------------------------------------------------

    class _GmoRadarTestWriter(Writer):
        """Custom Writer that validates GenericModelOutput data each frame for radar.

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
            self.num_elements_zero_count = 0
            self.valid_frame_count = 0
            self._sensor_prim = sensor_prim
            tick_rate = sensor_prim.GetAttribute("omni:sensor:tickRate").Get()
            self._expected_advance_ns = round(1.0 / tick_rate * 1e9) if tick_rate and tick_rate > 0 else 0
            # In kit 110.1.1, RTX Radar autotriggers regardless of tickRate attribute
            self._expected_advance_ns = 1.0 / 60.0 * 1e9

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

                self._test_point_cloud(gmo)
                self._test_intensity(gmo)
                self._test_radial_velocity(gmo)

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

            percent_diffs = np.divide(np.abs(expected_range - gmo.z), expected_range)

            edge_azimuths = np.arange(-180, 181, 45).reshape(1, -1)
            near_edge = np.any(np.abs(gmo.x[:, None] - edge_azimuths) < NEAR_EDGE_THRESHOLD, axis=1)

            num_exceeding = np.sum(np.logical_and(percent_diffs > 2e-2, ~near_edge))
            num_returns = np.size(gmo.x)
            pct_exceeding = num_exceeding / num_returns * 100
            valid_threshold = 1.0 if num_returns >= 100 else 10.0
            self._test.assertLessEqual(
                pct_exceeding,
                valid_threshold,
                f"{num_exceeding} of {num_returns} returns exceeded 2% range threshold.",
            )

        def _test_intensity(self, gmo: Any) -> None:
            self._test.assertTrue(np.all(gmo.scalar != 0), "Radar intensities must be non-zero.")

        def _test_radial_velocity(self, gmo: Any) -> None:
            self._test.assertLessEqual(
                np.max(np.abs(gmo.rv_ms)), 1e-2, "Radial velocity should be ~0 for static scene."
            )

    # ------------------------------------------------------------------
    # Test lifecycle
    # ------------------------------------------------------------------

    _writer_registered = False

    async def setUp(self) -> None:
        """Enable Motion BVH, create sarcophagus geometry, and register the radar GMO writer."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        # Enable Motion BVH required by radar
        carb.settings.get_settings().set("/renderer/raytracingMotion/enabled", True)
        self.cube_info = create_sarcophagus()
        self._timeline = omni.timeline.get_timeline_interface()
        if not TestRadarSensor._writer_registered:
            rep.WriterRegistry.register(TestRadarSensor._GmoRadarTestWriter)
            TestRadarSensor._writer_registered = True

    async def tearDown(self) -> None:
        """Stop radar playback and flush one app update after each runtime test."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_gmo_writer(self) -> None:
        """Validate GenericModelOutput via a Writer attached to a RadarSensor."""
        COLLECTION_SECONDS = 3.0

        radar = Radar(
            "/World/radar",
            attributes={
                "omni:sensor:WpmDmat:outputFrameOfReference": "WORLD",
            },
            aux_output_level="BASIC",
        )
        sensor = RadarSensor(radar, annotators=["generic-model-output"])
        writer = sensor.attach_writer(
            "_GmoRadarTestWriter",
            test_instance=self,
            sensor_prim=radar.prims[0],
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
        for level in ("NONE", "BASIC"):
            await stage_utils.create_new_stage_async()
            await ViewportManager.wait_for_viewport_async()
            carb.settings.get_settings().set("/renderer/raytracingMotion/enabled", True)
            create_sarcophagus()

            radar = Radar("/World/radar", aux_output_level=level)
            sensor = RadarSensor(radar, annotators=["generic-model-output"])
            self.assertEqual(radar.aux_output_level, level)

            prim = prim_utils.get_prim_at_path(radar.paths[0])
            attr = prim.GetAttribute("_replicator:rendervar:GenericModelOutput:channels")
            self.assertTrue(attr.IsValid())
            self.assertEqual(attr.GetTypeName(), Sdf.ValueTypeNames.StringArray)
            self.assertEqual(list(attr.Get()), [level])

            del sensor
            await omni.kit.app.get_app().next_update_async()

    async def test_aux_output_level_default_is_none(self) -> None:
        """Verify the default aux_output_level is NONE."""
        radar = Radar("/World/radar")
        self.assertEqual(radar.aux_output_level, "NONE")

    async def test_aux_output_level_invalid_raises(self) -> None:
        """Verify invalid aux_output_level raises ValueError for radar."""
        with self.assertRaises(ValueError):
            Radar("/World/radar", aux_output_level="FULL")
