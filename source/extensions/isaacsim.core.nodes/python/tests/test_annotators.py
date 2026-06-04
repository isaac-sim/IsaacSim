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

import time

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import _simulation_manager


class TestAnnotators(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()
        self._render_product_path = prim_utils.get_prim_path(ViewportManager.get_render_product())

        GroundPlane("/World/ground_plane")
        stage_utils.define_prim("/DistantLight", "DistantLight")

        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[-6, 0, 6.5], target=[-6, 0, -1])
        await app_utils.update_app_async()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        pass

    # ----------------------------------------------------------------------
    async def test_noop(self):
        annotator = rep.AnnotatorRegistry.get_annotator("IsaacNoop")
        annotator.attach([self._render_product_path])
        app_utils.play()
        await app_utils.update_app_async(steps=2)
        annotator.detach()

    # async def test_read_camera_info(self):
    #     annotator = rep.AnnotatorRegistry.get_annotator("IsaacReadCameraInfo")
    #     annotator.attach([self._render_product_path])
    #     app_utils.play()
    #     await app_utils.update_app_async(steps=2)
    #     data = annotator.get_data()
    #     # print(data)
    #     self.assertAlmostEqual(data["focalLength"], 18.14756202697754)
    #     annotator.detach()

    async def test_read_times(self):
        annotator_read_sim_time = rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime")
        annotator_read_sim_time.initialize(resetOnStop=True)
        annotator_read_sim_time.attach([self._render_product_path])
        annotator_read_system_time = rep.AnnotatorRegistry.get_annotator("IsaacReadSystemTime")
        annotator_read_system_time.attach([self._render_product_path])
        fabric_time_annotator = rep.AnnotatorRegistry.get_annotator("ReferenceTime")
        fabric_time_annotator.attach([self._render_product_path])

        # Testing that reset on stop works
        app_utils.play()
        await app_utils.update_app_async()
        app_utils.stop()
        await app_utils.update_app_async()
        app_utils.play()
        await app_utils.update_app_async(steps=10)
        fabric_time_data = fabric_time_annotator.get_data()
        data_read_sim_time = annotator_read_sim_time.get_data()
        data_read_system_time = annotator_read_system_time.get_data()

        sim_manager_iface = _simulation_manager.acquire_simulation_manager_interface()
        reference_time = (fabric_time_data["referenceTimeNumerator"], fabric_time_data["referenceTimeDenominator"])
        expected_sim_time = sim_manager_iface.get_simulation_time_at_time(reference_time)

        # The annotator sim time and get_simulation_time_at_time both correspond
        # to the same rendered frame, which may lag behind get_simulation_time()
        # due to render pipeline latency under multi-tick rendering.
        self.assertGreater(expected_sim_time, 0.0)
        self.assertAlmostEqual(data_read_sim_time["simulationTime"], expected_sim_time)
        self.assertAlmostEqual(data_read_system_time["systemTime"], time.time(), delta=0.5)
        annotator_read_sim_time.detach()
        annotator_read_system_time.detach()
        fabric_time_annotator.detach()

    async def test_convert_rgba_to_rgb(self):
        import omni.syntheticdata._syntheticdata as sd

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        annotator = rep.AnnotatorRegistry.get_annotator(rv + "IsaacConvertRGBAToRGB")
        annotator.attach([self._render_product_path])

        app_utils.play()
        await app_utils.update_app_async(steps=10)
        data = annotator.get_data()
        self.assertTrue(np.all(data["data"] > 150))
        annotator.detach()

    async def test_convert_depth_to_point_cloud(self):
        import omni.syntheticdata._syntheticdata as sd

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        annotator = rep.AnnotatorRegistry.get_annotator(rv + "IsaacConvertDepthToPointCloud")
        annotator.attach([self._render_product_path])

        app_utils.play()
        await app_utils.update_app_async(steps=10)
        data = annotator.get_data()
        self.assertTrue(np.all(np.linalg.norm(data["data"], axis=1) > 0))
        annotator.detach()
