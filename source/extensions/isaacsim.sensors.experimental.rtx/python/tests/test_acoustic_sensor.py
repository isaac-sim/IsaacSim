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

"""Verify AcousticSensor GMO writer integration and acoustic auxiliary output channel attributes."""

from typing import Any

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.sensors.experimental.rtx.generic_model_output as gmo_utils
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import (
    Acoustic,
    AcousticSensor,
    parse_generic_model_output_data,
)
from omni.replicator.core import Writer
from pxr import Sdf


class TestAcousticSensor(omni.kit.test.AsyncTestCase):
    """Test rtx acoustic sensor via a Writer attached through AcousticSensor."""

    # ------------------------------------------------------------------
    # Writer
    # ------------------------------------------------------------------

    class _GmoAcousticTestWriter(Writer):
        """Custom Writer that validates GenericModelOutput data each frame for acoustic.

        Args:
            test_instance: Test case instance used for assertions.
        """

        def __init__(self, test_instance: Any = None) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [
                rep.annotators.get("GenericModelOutput"),
            ]
            self._test = test_instance
            self.num_elements_zero_count = 0
            self.valid_frame_count = 0

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

                self.valid_frame_count += 1
                if self.valid_frame_count == 1:
                    return

                self._check_acoustic_gmo(gmo)

        def _check_acoustic_gmo(self, gmo: Any) -> None:
            t = self._test

            # Modality
            t.assertEqual(gmo.modality, gmo_utils.Modality.ACOUSTIC)

            # Coordinate type is not applicable for acoustic
            t.assertEqual(gmo.elementsCoordsType, gmo_utils.CoordsType.NOT_APPLICABLE)

            # numElements is a multiple of numSamplesPerSgw
            num_samples_per_sgw = gmo.numSamplesPerSgw
            t.assertGreater(num_samples_per_sgw, 0)
            num_signal_ways = gmo.numElements // num_samples_per_sgw
            t.assertEqual(gmo.numElements, num_signal_ways * num_samples_per_sgw)

            # Frame timestamps are ordered
            t.assertLess(gmo.frameStart.timestampNs, gmo.frameEnd.timestampNs)
            frame_duration_ns = gmo.frameEnd.timestampNs - gmo.frameStart.timestampNs

            # Scalars (amplitude samples) are finite
            t.assertTrue(np.all(np.isfinite(gmo.scalar[: gmo.numElements])))

            # timeOffsetNs at signal-way boundaries are non-negative and within frame duration
            for sgw in range(num_signal_ways):
                idx = sgw * num_samples_per_sgw
                offset = gmo.timeOffsetNs[idx]
                t.assertGreaterEqual(offset, 0, f"Signal way {sgw}: negative timeOffsetNs")
                t.assertLess(offset, frame_duration_ns, f"Signal way {sgw}: timeOffsetNs >= frame duration")

            # No overflow: all timeOffsetNs within element range are non-negative
            t.assertTrue(np.all(gmo.timeOffsetNs[: gmo.numElements] >= 0))

    # ------------------------------------------------------------------
    # Test lifecycle
    # ------------------------------------------------------------------

    _writer_registered = False

    async def setUp(self) -> None:
        """Create acoustic reflector targets and register the GMO validation writer."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        # Place reflective targets
        for i, (x, y) in enumerate([(3, 0), (5, 2), (4, -2)]):
            Cube(f"/World/target_{i}", sizes=1.0, positions=np.array([float(x), float(y), 0.0]))
        self._timeline = omni.timeline.get_timeline_interface()
        if not TestAcousticSensor._writer_registered:
            rep.WriterRegistry.register(TestAcousticSensor._GmoAcousticTestWriter)
            TestAcousticSensor._writer_registered = True

    async def tearDown(self) -> None:
        """Stop acoustic playback and flush one app update after each runtime test."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _create_acoustic_sensor(self, aux_output_level: Any = "BASIC") -> Any:
        """Create an Acoustic + AcousticSensor with a small sensor array.

        Args:
            aux_output_level: Auxiliary output level for the acoustic sensor.

        Returns:
            Acoustic authoring object and sensor runtime.
        """
        acoustic = Acoustic(
            "/World/acoustic",
            aux_output_level=aux_output_level,
            tick_rate=30.0,
            attributes={
                "omni:sensor:WpmAcoustic:centerFrequency": 51200.0,
                "omni:sensor:WpmAcoustic:sensorMount:m001:position": (0.0, -0.05, 0.0),
                "omni:sensor:WpmAcoustic:sensorMount:m001:rotation": (0.0, 0.0, 0.0),
                "omni:sensor:WpmAcoustic:sensorMount:m002:position": (0.0, 0.0, 0.0),
                "omni:sensor:WpmAcoustic:sensorMount:m002:rotation": (0.0, 0.0, 0.0),
                "omni:sensor:WpmAcoustic:sensorMount:m003:position": (0.0, 0.05, 0.0),
                "omni:sensor:WpmAcoustic:sensorMount:m003:rotation": (0.0, 0.0, 0.0),
                "omni:sensor:WpmAcoustic:rxGroup:g001:receiverIndices": [0, 1],
                "omni:sensor:WpmAcoustic:rxGroup:g002:receiverIndices": [1, 2],
            },
        )
        sensor = AcousticSensor(acoustic, annotators=["generic-model-output"])
        return acoustic, sensor

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_gmo_writer(self) -> None:
        """Validate acoustic GenericModelOutput via a Writer attached to an AcousticSensor."""
        COLLECTION_SECONDS = 3.0

        acoustic, sensor = self._create_acoustic_sensor(aux_output_level="BASIC")
        writer = sensor.attach_writer("_GmoAcousticTestWriter", test_instance=self)

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
            Cube("/World/target", sizes=1.0, positions=np.array([3.0, 0.0, 0.0]))

            acoustic, sensor = self._create_acoustic_sensor(aux_output_level=level)
            self.assertEqual(acoustic.aux_output_level, level)

            prim = prim_utils.get_prim_at_path(acoustic.paths[0])
            attr = prim.GetAttribute("_replicator:rendervar:GenericModelOutput:channels")
            self.assertTrue(attr.IsValid(), "channels attribute must exist on prim")
            self.assertEqual(attr.GetTypeName(), Sdf.ValueTypeNames.StringArray)
            self.assertEqual(list(attr.Get()), [level])

            del sensor
            await omni.kit.app.get_app().next_update_async()

    async def test_aux_output_level_default_is_none(self) -> None:
        """Verify the default aux_output_level is NONE."""
        acoustic = Acoustic("/World/acoustic2", tick_rate=30.0)
        self.assertEqual(acoustic.aux_output_level, "NONE")

    async def test_aux_output_level_invalid_raises(self) -> None:
        """Verify invalid aux_output_level raises ValueError."""
        with self.assertRaises(ValueError):
            Acoustic("/World/acoustic3", tick_rate=30.0, aux_output_level="FULL")
