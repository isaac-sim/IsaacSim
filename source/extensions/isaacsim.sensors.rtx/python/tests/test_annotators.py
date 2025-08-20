# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import unittest
from pathlib import Path

import carb
import matplotlib.pyplot as plt
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS, get_gmo_data
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

DEBUG_DRAW_PRINT = False


class TestRTXSensorAnnotator(omni.kit.test.AsyncTestCase):
    """Test class for RTX sensor annotators"""

    # This class is not meant to be run as a test, but rather to be used as a base class for other tests
    __test__ = False
    _assets_root_path = get_assets_root_path()
    _accumulate_returns = False

    async def setUp(self):
        """Setup test environment with a cube and lidar"""
        await create_new_stage_async()
        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()
        await update_stage_async()

        # Ordering octants in binary order, such that octant 0 is +++, octant 1 is ++-, etc. for XYZ.
        self._octant_dimensions = [
            (10, 10, 5),
            (10, 10, 7),
            (25, 25, 17),
            (25, 25, 19),
            (15, 15, 9),
            (15, 15, 11),
            (20, 20, 13),
            (20, 20, 15),
        ]

        # Autogenerate sarcophagus
        dims = [(10, 5, 7), (15, 9, 11), (20, 13, 15), (25, 17, 19)]
        i = 0
        for l, h1, h2 in dims:
            h = h1 + h2
            x_sign = -1 if 0 < i < 3 else 1
            y_sign = -1 if i > 1 else 1
            signs = np.array([x_sign, y_sign, 1])
            VisualCuboid(
                prim_path=f"/World/cube_{i*4}",
                name=f"cube_{i*4}",
                position=np.multiply(signs, np.array([l + 0.5, l / 2, h1 - h / 2])),
                scale=np.array([1, l, h]),
            )
            VisualCuboid(
                prim_path=f"/World/cube_{i*4+1}",
                name=f"cube_{i*4+1}",
                position=np.multiply(signs, np.array([l / 2, l + 0.5, h1 - h / 2])),
                scale=np.array([l, 1, h]),
            )
            VisualCuboid(
                prim_path=f"/World/cube_{i*4+2}",
                name=f"cube_{i*4+2}",
                position=np.multiply(signs, np.array([l / 2, l / 2, h1 + 0.5])),
                scale=np.array([l, l, 1]),
            )
            VisualCuboid(
                prim_path=f"/World/cube_{i*4+3}",
                name=f"cube_{i*4+3}",
                position=np.multiply(signs, np.array([l / 2, l / 2, -h2 - 0.5])),
                scale=np.array([l, l, 1]),
            )
            i += 1

        self._timeline = omni.timeline.get_timeline_interface()
        self._annotator_data = None
        self.hydra_texture = None

    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()

    async def _test_annotator_result(self):
        """Tests the annotator result."""
        self.assertIn("gmoBufferPointer", self._annotator_data)
        self.assertIn("gmoDeviceIndex", self._annotator_data)
        self.assertNotEqual(self._annotator_data["gmoBufferPointer"], 0, "Expected nonzero GMO buffer pointer.")

        gmo = get_gmo_data(self._annotator_data["gmoBufferPointer"])
        await self._test_returns(az_vals=gmo.x, el_vals=gmo.y, r_vals=gmo.z)
        return

    async def _test_returns(
        self, az_vals: np.array, el_vals: np.array, r_vals: np.array, cartesian: bool = False
    ) -> None:
        """Tests sensor returns stored in GMO buffer against expected range, for a given set of azimuth and elevation values.

        Args:
            az_vals (np.array): azimuth values (degrees)
            el_vals (np.array): elevation values (degrees)
            r_vals (np.array): range values (meters)
            cartesian (bool): If True, assumes inpus are in Cartesian coordinates - az = x, el = y, r = z, all in meters. Default False.
        """
        # NOTE: if an element of unit_vecs is 0, indicating the return vector is parallel to the plane, the result of np.divide will be inf
        # Suppress the error
        np.seterr(divide="ignore")

        if not cartesian:
            # Get cartesian unit vectors from spherical coordinates
            azr = np.deg2rad(az_vals)
            elr = np.deg2rad(el_vals)
            x = np.multiply(np.cos(azr), np.cos(elr))
            y = np.multiply(np.sin(azr), np.cos(elr))
            z = np.sin(elr)
            unit_vecs = np.concatenate((x[..., None], y[..., None], z[..., None]), axis=1)
        else:
            unit_vecs = np.concatenate((az_vals[..., None], el_vals[..., None], r_vals[..., None]), axis=1)
            r_vals = np.linalg.norm(unit_vecs, axis=1)
            # Normalize unit vectors
            unit_vecs = np.divide(unit_vecs, np.repeat(r_vals[:, None], 3, axis=1))
            # Get spherical coordinates from cartesian unit vectors
            az_vals = np.arctan2(unit_vecs[:, 1], unit_vecs[:, 0])
            el_vals = np.arcsin(unit_vecs[:, 2])
        # Get octant dimensions and indices
        octant = (unit_vecs[:, 0] < 0) * 4 + (unit_vecs[:, 1] < 0) * 2 + (unit_vecs[:, 2] < 0)
        dims = np.array([self._octant_dimensions[o] for o in octant])

        if DEBUG_DRAW_PRINT:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")
            x_plot = np.multiply(unit_vecs[:, 0], r_vals)
            y_plot = np.multiply(unit_vecs[:, 1], r_vals)
            z_plot = np.multiply(unit_vecs[:, 2], r_vals)
            ax.scatter(x_plot, y_plot, z_plot)
            plt.savefig(f"test_returns_{self.fig_name}.png")
            plt.close()

        # Let alpha be the angle between the normal to the plane and the return vector
        # Let the distance from the origin of the return vector along the normal vector to the plane be l =  dims(idx)
        # Let expected range R be the distance along the return vector to the point of intersection with the plane
        # Then, cos(alpha) = l / R
        # Next, observe cos(alpha) = n-hat dot r-hat, where n-hat is the unit normal vector to the plane
        # and r-hat is the unit return vector
        # Therefore, R = l / (n-hat dot r-hat)
        # n-hat dot r-hat is simply the index of the unit return vector corresponding to the plane
        # This simplifies the computation of expected range to elementwise-division of the dimensions by the unit return vectors
        # The minimum of these values is the expected range to the first plane the return vector will intersect
        expected_range = np.min(np.divide(dims, np.abs(unit_vecs)), axis=1)

        # Compute percent differences, and count the number of returns that exceed the threshold of 1%
        percent_diffs = np.divide(np.abs(expected_range - r_vals), expected_range)

        # Find where returns were not within 0.5deg of an octant edge
        not_near_edge = [
            not (abs(a) < 0.5 or abs(a - 90) < 0.5 or abs(a - 180) < 0.5 or abs(a + 180) < 0.5) for a in az_vals
        ]

        # Compute the number of returns that exceed the threshold of 1%, beyond edges of the octants
        num_exceeding_threshold = np.sum(np.logical_and(percent_diffs > 1e-2, np.array(not_near_edge)))
        num_returns = np.size(az_vals)
        carb.log_warn(f"num_returns: {num_returns}")
        pct_exceeding_threshold = num_exceeding_threshold / num_returns * 100
        valid_threshold = 1.0 if num_returns >= 100 else 10.0
        self.assertLessEqual(
            pct_exceeding_threshold,
            valid_threshold,
            f"Expected fewer than 1% of returns to differ from expected range by more than 1%. {num_exceeding_threshold} of {num_returns} returns exceeded threshold.",
        )


class TestIsaacComputeRTXLidarFlatScan(TestRTXSensorAnnotator):
    """Test the Isaac Compute RTX Lidar Flat Scan annotator"""

    __test__ = True
    _annotator = rep.AnnotatorRegistry.get_annotator("IsaacComputeRTXLidarFlatScan")
    _accumulate_returns = True

    async def _test_annotator_result(self):
        """Tests the annotator result."""

        # Render to get annotator result
        await self.my_world.reset_async()
        self._timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
            self._annotator_data = self._annotator.get_data()
            if self._annotator_data and "numCols" in self._annotator_data and self._annotator_data["numCols"] > 0:
                break

        self._timeline.stop()

        # Call the main test method
        # self._annotator_data = self._annotator.get_data()

        # Test that all expected keys are present in the annotator data, then copy those values to the test class attributes
        for expected_key in [
            "azimuthRange",
            "depthRange",
            "horizontalFov",
            "horizontalResolution",
            "intensitiesData",
            "linearDepthData",
            "numCols",
            "numRows",
            "rotationRate",
        ]:
            self.assertIn(expected_key, self._annotator_data)
            setattr(self, expected_key, self._annotator_data[expected_key])

        # Confirm default values for 3D lidar
        if not self._is_2d_lidar:
            self.assertTrue(np.allclose(np.zeros([1, 2]), self.azimuthRange), f"azimuthRange: {self.azimuthRange}")
            self.assertTrue(np.allclose(np.zeros([1, 2]), self.depthRange), f"depthRange: {self.depthRange}")
            self.assertEqual(0.0, self.horizontalFov)
            self.assertEqual(0.0, self.horizontalResolution)
            self.assertEqual(0, self.intensitiesData.size)
            self.assertEqual(0, self.linearDepthData.size)
            self.assertEqual(0, self.numCols)
            self.assertEqual(1, self.numRows)
            self.assertEqual(0.0, self.rotationRate)
            return

        # Construct azimuth and elevation vectors, then test returns
        num_elements = self.numCols
        self.assertGreater(num_elements, 0, "Expecting more than zero elements in output.")
        azimuth_vals = np.linspace(self.azimuthRange[0], self.azimuthRange[1], num_elements)
        elevation_vals = np.zeros_like(azimuth_vals)
        # Remove any returns with range below threshold
        valid_returns = self.linearDepthData > 1e-3
        await self._test_returns(
            az_vals=azimuth_vals[valid_returns],
            el_vals=elevation_vals[valid_returns],
            r_vals=self.linearDepthData[valid_returns],
        )


class TestIsaacExtractRTXSensorPointCloudNoAccumulator(TestRTXSensorAnnotator):
    """Test the Isaac Extract RTX Sensor Point Cloud annotator without accumulator"""

    __test__ = True
    _annotator = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
    _WARMUP_FRAMES = 6
    _ADDITIONAL_FRAMES = 0

    async def test_rtx_radar(self):
        # Create sensor prim
        _, self.sensor = omni.kit.commands.execute(f"IsaacSensorCreateRtxRadar")
        self.assertIsNotNone(self.sensor)

        self.fig_name = f"radar_sensor"

        # Create render product and attach to sensor
        self.hydra_texture = rep.create.render_product(
            self.sensor.GetPath(),
            [32, 32],
            name="RtxSensorRenderProduct",
            render_vars=["GenericModelOutput", "RtxSensorMetadata"],
        )
        # Attach annotator to render product
        self._annotator.attach([self.hydra_texture.path])

        # Render to get annotator result
        await self.my_world.reset_async()
        self._timeline.play()
        await omni.syntheticdata.sensors.next_render_simulation_async(self.hydra_texture.path, self._WARMUP_FRAMES)
        self._timeline.stop()

        # Call the main test method
        self._annotator_data = self._annotator.get_data()
        await self._test_annotator_result()

    async def _test_annotator_result(self):
        """Tests the annotator result."""
        # Render to get annotator result
        await self.my_world.reset_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Render a minimum number of frames to ensure the sensor renderer has warmed up
        await omni.syntheticdata.sensors.next_render_simulation_async(self.hydra_texture.path, self._WARMUP_FRAMES)
        # Render additional frames until we get valid data, or until we've rendered the maximum number of frames
        for _ in range(self._ADDITIONAL_FRAMES):
            await omni.kit.app.get_app().next_update_async()
            self._annotator_data = self._annotator.get_data()
            if (
                self._annotator_data
                and "info" in self._annotator_data
                and self._annotator_data["info"]["accumulatedData"]
            ):
                break
        self._timeline.stop()

        self._annotator_data = self._annotator.get_data()
        # Test that all expected keys are present in the annotator data, then copy those values to the test class attributes
        self.assertIn("data", self._annotator_data)
        self.data = self._annotator_data["data"]

        self.assertIn("info", self._annotator_data)
        for expected_key in [
            "transform",
        ]:
            self.assertIn(expected_key, self._annotator_data["info"])
            setattr(self, expected_key, self._annotator_data["info"][expected_key])

        # Test data (cartesian)
        self.assertGreater(self.data.shape[0], 0, "Expected non-empty data.")
        self.assertEqual(self.data.shape[1], 3)
        await self._test_returns(self.data[:, 0], self.data[:, 1], self.data[:, 2], cartesian=True)

        return


class TestIsaacCreateRTXLidarScanBuffer(TestRTXSensorAnnotator):
    """Test the Isaac Create RTX Lidar Scan Buffer annotator"""

    __test__ = True
    _annotator = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
    _WARMUP_FRAMES = 3
    _ADDITIONAL_FRAMES = 17

    async def _test_annotator_result(self):
        """Tests the annotator result."""
        # Render to get annotator result
        await self.my_world.reset_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Render a minimum number of frames to ensure the sensor renderer has warmed up
        await omni.syntheticdata.sensors.next_render_simulation_async(self.hydra_texture.path, self._WARMUP_FRAMES)
        # Render additional frames until we get valid data, or until we've rendered the maximum number of frames
        for _ in range(self._ADDITIONAL_FRAMES):
            # Wait for a single frame
            await omni.kit.app.get_app().next_update_async()
            self._annotator_data = self._annotator.get_data()
            if self._annotator_data and "data" in self._annotator_data and self._annotator_data["data"].size > 0:
                break
        self._timeline.stop()

        # Test that all expected keys are present in the annotator data, then copy those values to the test class attributes
        for expected_key in [
            "azimuth",
            "beamId",
            "data",
            "distance",
            "elevation",
            "emitterId",
            "index",
            "intensity",
            "materialId",
            "normal",
            "objectId",
            "timestamp",
            "velocity",
        ]:
            self.assertIn(expected_key, self._annotator_data)
            setattr(self, expected_key, self._annotator_data[expected_key])

        self.assertIn("info", self._annotator_data)
        for expected_key in [
            "numChannels",
            "numEchos",
            "numReturnsPerScan",
            "renderProductPath",
            "ticksPerScan",
            "transform",
            "azimuth",
            "beamId",
            "distance",
            "elevation",
            "emitterId",
            "index",
            "intensity",
            "materialId",
            "normal",
            "objectId",
            "timestamp",
            "velocity",
        ]:
            self.assertIn(expected_key, self._annotator_data["info"])
            setattr(self, expected_key, self._annotator_data["info"][expected_key])

        # Test data (cartesian)
        self.assertGreater(self.data.shape[0], 0, "Expected non-empty data.")
        self.assertEqual(self.data.shape[1], 3)
        await self._test_returns(self.data[:, 0], self.data[:, 1], self.data[:, 2], cartesian=True)

        return


# Map annotator classes to their corresponding sensor types
annotators = {
    TestIsaacComputeRTXLidarFlatScan: ["lidar"],
    TestIsaacExtractRTXSensorPointCloudNoAccumulator: ["lidar", "radar"],
    TestIsaacCreateRTXLidarScanBuffer: ["lidar"],
}

ETM_SKIP_LIST = [
    "SICK_microScan3_Profile_1",
    "SICK_microScan3_Profile_3",
    "SICK_picoScan150_Profile_11",
    "SICK_picoScan150_Profile_1",
    "OS1_OS1_REV6_32ch10hz512res",
    "OS2_OS2_REV6_128ch10hz512res",
    "SICK_picoScan150_Profile_2",
    "Simple_Example_Solid_State_None",
]


def _create_test_for_annotator_with_omni_lidar_prim(config: str = None, variant: str = None):
    """Create OmniLidar prim with specified config and variants, then attach an annotator and run for several frames.

    Args:
        sensor_type (Literal[&quot;lidar&quot;, &quot;radar&quot;], optional): _description_. Defaults to "lidar".
        prim_type (Literal[&quot;sensor&quot;, &quot;camera&quot;], optional): _description_. Defaults to "sensor".
        config (str, optional): _description_. Defaults to None.
    """

    async def test_function(self):

        if os.getenv("ETM_ACTIVE") and f"{config}_{variant}" in ETM_SKIP_LIST:
            raise unittest.SkipTest("Skipping test in ETM.")

        self.fig_name = f"lidar_sensor_{config}_{variant}"

        # Create sensor prim
        kwargs = {
            "path": "lidar",
            "parent": None,
            "translation": Gf.Vec3d(0.0, 0.0, 0.0),
            "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            "config": config,
            "variant": variant,
            "omni:sensor:Core:outputFrameOfReference": "WORLD",
            # "omni:sensor:Core:skipDroppingInvalidPoints": True,
        }
        _, self.sensor = omni.kit.commands.execute(f"IsaacSensorCreateRtxLidar", **kwargs)
        sensor_type = self.sensor.GetTypeName()
        self.assertEqual(
            sensor_type, "OmniLidar", f"Expected OmniLidar prim, got {sensor_type}. Was sensor prim created?"
        )

        # Create render product and attach to sensor
        self.hydra_texture = rep.create.render_product(
            self.sensor.GetPath(),
            [32, 32],
            name="RtxSensorRenderProduct",
            render_vars=["GenericModelOutput", "RtxSensorMetadata"],
        )
        # Attach annotator to render product
        self._annotator.attach([self.hydra_texture.path])

        # Test attributes of the sensor prim
        elevationDeg = self.sensor.GetAttribute("omni:sensor:Core:emitterState:s001:elevationDeg").Get()
        self._is_2d_lidar = all([abs(i) < 1e-3 for i in list(elevationDeg)])

        await self._test_annotator_result()

    return test_function


# Iterate over all supported lidar configs and variants, creating a test for each as sensor prims
data_source = "gpu"
for config_path in SUPPORTED_LIDAR_CONFIGS:
    config_name = Path(config_path).stem
    for variant in SUPPORTED_LIDAR_CONFIGS[config_path] or [None]:
        for test_class in annotators:
            test_func = _create_test_for_annotator_with_omni_lidar_prim(config=config_name, variant=variant)
            test_name = f"lidar_sensor_{config_name}_{variant}_{data_source}"
            test_func.__name__ = f"test_{test_name}"
            test_func.__doc__ = f"Test {test_class.__name__} annotator results using OmniLidar prim, with config {config_name} and variant {variant} and data on {data_source.upper()}."
            setattr(test_class, test_func.__name__, test_func)
        # Clear test_class to prevent the automated test loader in unittests.loader.TestLoader.loadTestsFromModule
        # from duplicating tests
        test_class = None
