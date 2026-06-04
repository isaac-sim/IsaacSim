# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test the deferred-radar-construction workaround for the lidar FIF empty-slot race.

The race lives in ``rtx.sensors.lidar.core.plugin`` and manifests as a fatal
crash within the first frames after pressing Play when an RTX Radar and one or
more RTX Lidars come online in the same render frame with Motion BVH enabled.

The documented workaround in the standalone Python flow is to:

1. Author the radar's USD prim before ``timeline.play()`` (lightweight, USD-only).
2. Start playback and let the lidar sensors warm up for several frames.
3. Only then construct the ``RadarSensor`` wrapper, which is what binds the
   radar to the renderer and triggers FIF slot allocation.

This test exercises the workaround end-to-end and asserts that every sensor's
``GenericModelOutput`` writer eventually observes ``numElements > 0``.
"""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import (
    Lidar,
    LidarSensor,
    Radar,
    RadarSensor,
    parse_generic_model_output_data,
)
from omni.replicator.core import Writer

from .common import create_sarcophagus


class TestMultiSensorWarmup(omni.kit.test.AsyncTestCase):
    """Verify the deferred-radar workaround produces valid GMO output for all sensors."""

    # ------------------------------------------------------------------
    # Writer
    # ------------------------------------------------------------------

    class _GmoMultiSensorWarmupCountingWriter(Writer):
        """Minimal Writer that counts GMO frames with ``numElements > 0``.

        One instance is created per ``attach_writer`` call, so each sensor's
        writer maintains independent counters. The class name is intentionally
        specific (rather than ``_GmoCountingWriter``) to avoid colliding with
        any future generic counting writer registered by another test.
        """

        def __init__(self):
            self.annotators = [rep.annotators.get("GenericModelOutput")]
            self.data_structure = "renderProduct"
            self.valid_frame_count = 0
            self.zero_element_frame_count = 0

        def write(self, data):
            if "renderProducts" not in data:
                return
            for _rp_name, rp_data in data["renderProducts"].items():
                gmo_raw = rp_data.get("GenericModelOutput")
                if isinstance(gmo_raw, dict):
                    gmo_raw = gmo_raw.get("data")
                gmo = parse_generic_model_output_data(gmo_raw)
                if gmo.numElements > 0:
                    self.valid_frame_count += 1
                else:
                    self.zero_element_frame_count += 1

    # ------------------------------------------------------------------
    # Test lifecycle
    # ------------------------------------------------------------------

    _writer_registered = False

    async def setUp(self):
        super().setUp()
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        # Radar requires Motion BVH; this is also what enables the FIF race
        # window we are working around.
        carb.settings.get_settings().set("/renderer/raytracingMotion/enabled", True)
        create_sarcophagus()
        self._timeline = omni.timeline.get_timeline_interface()
        if not TestMultiSensorWarmup._writer_registered:
            rep.WriterRegistry.register(TestMultiSensorWarmup._GmoMultiSensorWarmupCountingWriter)
            TestMultiSensorWarmup._writer_registered = True

    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_radar_attached_after_lidar_warmup_yields_valid_output(self):
        """All three sensors produce valid GMO output when the radar is deferred.

        Pattern under test (mirrors the documented user-facing workaround):

        - Two ``LidarSensor`` instances are constructed pre-play (full wrap:
          USD prim + render product + annotators).
        - Only the radar's USD authoring object (``Radar``) is constructed
          pre-play.
        - ``timeline.play()`` is called; the lidars run alone for
          ``WARMUP_FRAMES`` ticks, giving their FIF slots time to stabilize.
        - The ``RadarSensor`` wrapper is then constructed during play. This
          is the operation that creates the radar's render product and
          allocates its FIF slots; it is the analog of ``attach_writer`` in
          the OmniGraph helper flow.
        - The simulation runs for an additional ``COLLECTION_FRAMES`` ticks.

        Each sensor's writer is checked independently for at least one frame
        with ``numElements > 0``.

        Note: ``WARMUP_FRAMES`` is load-bearing on affected hardware - it is
        what closes the FIF race window. On the CI hardware this test typically
        runs against, the race does not fire even with ``WARMUP_FRAMES = 0``, so
        CI cannot directly observe a regression if the warmup is removed. Do
        not lower this value casually; see
        :ref:`isaac_sim_sensors_multitick_known_issue_radar_lidar_fif_race`.
        """
        WARMUP_FRAMES = 5
        COLLECTION_FRAMES = int(3.0 * 60)
        WRITER_NAME = "_GmoMultiSensorWarmupCountingWriter"

        # Lidar 1: full wrap pre-play.
        lidar_1 = Lidar("/World/lidar_1")
        lidar_sensor_1 = LidarSensor(lidar_1, annotators=["generic-model-output"])
        lidar_sensor_1.attach_writer(WRITER_NAME)

        # Lidar 2: full wrap pre-play.
        lidar_2 = Lidar("/World/lidar_2")
        lidar_sensor_2 = LidarSensor(lidar_2, annotators=["generic-model-output"])
        lidar_sensor_2.attach_writer(WRITER_NAME)

        # Radar: USD authoring object only - no render product yet.
        radar = Radar(
            "/World/radar",
            attributes={"omni:sensor:WpmDmat:outputFrameOfReference": "WORLD"},
        )

        # Start play. The lidars come online here; the radar is still
        # absent from the render graph.
        self._timeline.play()
        for _ in range(WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        # Now safe to wrap the radar. This call is what creates the render
        # product and binds annotators, mirroring the operation that opens
        # the FIF race window when done concurrently with lidar attachment.
        radar_sensor = RadarSensor(radar, annotators=["generic-model-output"])
        radar_sensor.attach_writer(WRITER_NAME)

        for _ in range(COLLECTION_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        self._timeline.stop()

        # Each sensor has its own writer instance with independent counters.
        lidar_1_writer = lidar_sensor_1._writers[WRITER_NAME]
        lidar_2_writer = lidar_sensor_2._writers[WRITER_NAME]
        radar_writer = radar_sensor._writers[WRITER_NAME]

        carb.log_warn(
            f"lidar_1: valid={lidar_1_writer.valid_frame_count} " f"zero={lidar_1_writer.zero_element_frame_count}"
        )
        carb.log_warn(
            f"lidar_2: valid={lidar_2_writer.valid_frame_count} " f"zero={lidar_2_writer.zero_element_frame_count}"
        )
        carb.log_warn(
            f"radar:   valid={radar_writer.valid_frame_count} " f"zero={radar_writer.zero_element_frame_count}"
        )

        self.assertGreater(
            lidar_1_writer.valid_frame_count,
            0,
            "lidar_1 produced no GMO frames with numElements > 0",
        )
        self.assertGreater(
            lidar_2_writer.valid_frame_count,
            0,
            "lidar_2 produced no GMO frames with numElements > 0",
        )
        self.assertGreater(
            radar_writer.valid_frame_count,
            0,
            "radar produced no GMO frames with numElements > 0 "
            "(deferred-attach workaround appears to have broken radar binding)",
        )
