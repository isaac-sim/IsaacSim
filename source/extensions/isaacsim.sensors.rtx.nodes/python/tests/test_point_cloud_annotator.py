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

"""Verifies the RTX sensor point cloud annotator converts spherical sensor output to Cartesian coordinates and passes Cartesian data through unchanged."""

from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from isaacsim.sensors.experimental.rtx.tests.common import create_sarcophagus
from omni.replicator.core import Writer, WriterRegistry

COLLECTION_SECONDS = 3.0
ABS_TOL = 1e-4


class TestIsaacExtractRTXSensorPointCloud(omni.kit.test.AsyncTestCase):
    """Test the OgnIsaacExtractRTXSensorPointCloud node with SPHERICAL and CARTESIAN outputs."""

    # ------------------------------------------------------------------
    # Writer
    # ------------------------------------------------------------------

    class _PointCloudTestWriter(Writer):
        """Writer that validates extracted point cloud against raw GMO each frame.

        Args:
            test_instance: Test case used for assertions.
            coords_type: Coordinate type expected in the GMO buffer.
        """

        def __init__(self, test_instance: Any = None, coords_type: Any = None) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [
                rep.annotators.get("GenericModelOutput"),
                rep.annotators.get("IsaacExtractRTXSensorPointCloud"),
            ]
            self._test = test_instance
            self._coords_type = coords_type
            self.num_empty_frames = 0
            self.valid_frame_count = 0

        def write(self, data: Any) -> None:
            if "renderProducts" not in data:
                return
            for _rp_name, rp_data in data["renderProducts"].items():
                gmo_raw = rp_data.get("GenericModelOutput")
                if isinstance(gmo_raw, dict):
                    gmo_raw = gmo_raw.get("data")

                pc_raw = rp_data.get("IsaacExtractRTXSensorPointCloud")
                if isinstance(pc_raw, dict):
                    pc_data = pc_raw.get("data")
                    pc_buffer_size = pc_raw.get("bufferSize", 0)
                else:
                    pc_data = pc_raw
                    pc_buffer_size = 0

                if gmo_raw is None or pc_data is None:
                    self.num_empty_frames += 1
                    return

                gmo = parse_generic_model_output_data(gmo_raw)
                if gmo.numElements == 0:
                    self.num_empty_frames += 1
                    return

                self.valid_frame_count += 1

                n = gmo.numElements
                t = self._test

                # Reshape extracted point cloud to (N, 3)
                if hasattr(pc_data, "numpy"):
                    pc_np = pc_data.numpy()
                elif isinstance(pc_data, np.ndarray):
                    pc_np = pc_data
                else:
                    t.fail(f"Unexpected point-cloud data type: {type(pc_data)}")
                pc_points = pc_np.reshape(-1, 3)[:n]

                t.assertEqual(pc_points.shape[0], n, f"Point count mismatch: {pc_points.shape[0]} != {n}")

                if self._coords_type == "SPHERICAL":
                    self._validate_spherical_to_cartesian(gmo, pc_points, n)
                elif self._coords_type == "CARTESIAN":
                    self._validate_cartesian_passthrough(gmo, pc_points, n)

        def _validate_spherical_to_cartesian(self, gmo: Any, pc_points: Any, n: Any) -> None:
            """Validate the node's spherical-to-Cartesian conversion against reference.

            Args:
                gmo: GenericModelOutput buffer with spherical coordinate arrays.
                pc_points: Cartesian point cloud produced by the annotator.
                n: Number of points to validate.
            """
            az = np.ctypeslib.as_array(gmo.x, shape=(n,)).copy()
            el = np.ctypeslib.as_array(gmo.y, shape=(n,)).copy()
            dist = np.ctypeslib.as_array(gmo.z, shape=(n,)).copy()

            az_rad = np.deg2rad(az)
            el_rad = np.deg2rad(el)
            rxy = dist * np.cos(el_rad)
            expected_x = rxy * np.cos(az_rad)
            expected_y = rxy * np.sin(az_rad)
            expected_z = dist * np.sin(el_rad)

            mask = dist < 1e-6
            expected_x[mask] = 0.0
            expected_y[mask] = 0.0
            expected_z[mask] = 0.0

            np.testing.assert_allclose(pc_points[:, 0], expected_x, atol=ABS_TOL, err_msg="X mismatch")
            np.testing.assert_allclose(pc_points[:, 1], expected_y, atol=ABS_TOL, err_msg="Y mismatch")
            np.testing.assert_allclose(pc_points[:, 2], expected_z, atol=ABS_TOL, err_msg="Z mismatch")

        def _validate_cartesian_passthrough(self, gmo: Any, pc_points: Any, n: Any) -> None:
            """Validate Cartesian data is passed through unchanged (no coordinate conversion).

            Args:
                gmo: GenericModelOutput buffer with Cartesian coordinate arrays.
                pc_points: Cartesian point cloud produced by the annotator.
                n: Number of points expected in the cloud.
            """
            self._test.assertEqual(pc_points.shape[0], n)
            x = np.ctypeslib.as_array(gmo.x, shape=(n,)).copy()
            y = np.ctypeslib.as_array(gmo.y, shape=(n,)).copy()
            z = np.ctypeslib.as_array(gmo.z, shape=(n,)).copy()
            np.testing.assert_allclose(pc_points[:, 0], x, atol=ABS_TOL, err_msg="X mismatch (cartesian passthrough)")
            np.testing.assert_allclose(pc_points[:, 1], y, atol=ABS_TOL, err_msg="Y mismatch (cartesian passthrough)")
            np.testing.assert_allclose(pc_points[:, 2], z, atol=ABS_TOL, err_msg="Z mismatch (cartesian passthrough)")

    # ------------------------------------------------------------------
    # Test lifecycle
    # ------------------------------------------------------------------

    _writer_registered = False

    async def setUp(self) -> None:
        """Register the custom writer that compares GMO and point-cloud annotator output."""
        super().setUp()
        if not TestIsaacExtractRTXSensorPointCloud._writer_registered:
            WriterRegistry.register(TestIsaacExtractRTXSensorPointCloud._PointCloudTestWriter)
            TestIsaacExtractRTXSensorPointCloud._writer_registered = True

    async def tearDown(self) -> None:
        """Run the base async-test cleanup after point-cloud writer assertions."""
        super().tearDown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    async def _run_sensor_test(self, coords_type: str) -> None:
        """Create a lidar, attach the comparison writer, and run simulation.

        Args:
            coords_type: Coordinate type used by the lidar output.
        """
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()

        create_sarcophagus()

        lidar = Lidar.create(
            "/World/lidar",
            config="Example_Rotary",
            attributes={
                "omni:sensor:Core:elementsCoordsType": coords_type,
            },
            aux_output_level="BASIC",
        )
        sensor = LidarSensor(lidar, annotators=["generic-model-output"])
        writer = sensor.attach_writer(
            "_PointCloudTestWriter",
            test_instance=self,
            coords_type=coords_type,
        )

        timeline = omni.timeline.get_timeline_interface()
        total_frames = int(COLLECTION_SECONDS * 60)
        timeline.set_end_time(COLLECTION_SECONDS + 1.0)
        timeline.play()
        for _ in range(total_frames):
            await omni.kit.app.get_app().next_update_async()
        timeline.stop()

        self.assertGreater(writer.valid_frame_count, 0, f"No valid frames (coords_type={coords_type})")

        del sensor
        await omni.kit.app.get_app().next_update_async()

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_spherical_to_cartesian(self) -> None:
        """Node correctly converts spherical GMO data to Cartesian each frame."""
        await self._run_sensor_test("SPHERICAL")

    async def test_cartesian_passthrough(self) -> None:
        """Node handles Cartesian GMO data each frame."""
        await self._run_sensor_test("CARTESIAN")
