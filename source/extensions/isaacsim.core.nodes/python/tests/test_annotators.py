# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.api import SimulationContext
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.robots import Robot
from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
from isaacsim.core.utils.stage import get_current_stage, open_stage_async
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Sdf, UsdLux


class TestAnnotators(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._viewport_api = get_active_viewport()
        self._render_product_path = self._viewport_api.get_render_product_path()
        self._simulation_context = SimulationContext()
        await self._simulation_context.initialize_simulation_context_async()

        ground_plane = GroundPlane("/World/ground_plane", visible=True)
        self._stage = get_current_stage()
        distantLight = UsdLux.DistantLight.Define(self._stage, Sdf.Path("/DistantLight"))

        set_camera_view(eye=[-6, 0, 6.5], target=[-6, 0, -1], camera_prim_path="/OmniverseKit_Persp")
        await omni.kit.app.get_app().next_update_async()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        # self._action.execute(viewport_api=self._viewport_api, visible=True)
        pass

    # ----------------------------------------------------------------------
    async def test_noop(self):
        annotator = rep.AnnotatorRegistry.get_annotator("IsaacNoop")
        annotator.attach([self._render_product_path])
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        annotator.detach()

    # async def test_read_camera_info(self):
    #     annotator = rep.AnnotatorRegistry.get_annotator("IsaacReadCameraInfo")
    #     annotator.attach([self._render_product_path])
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
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
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.syntheticdata.sensors.next_render_simulation_async(self._render_product_path, 10)
        fabric_time_data = fabric_time_annotator.get_data()
        data_read_sim_time = annotator_read_sim_time.get_data()
        data_read_system_time = annotator_read_system_time.get_data()
        self.assertAlmostEqual(
            data_read_sim_time["simulationTime"], 0.01666666753590107 * 12
        )  # Two extra steps happen to init physics on play
        self.assertAlmostEqual(data_read_system_time["systemTime"], time.time(), delta=0.5)

        # TODO: Its not clear why reading the time directly from the rational time results in time being one frame ahead of the annotatot time
        self._core_nodes_interface = _isaacsim_core_nodes.acquire_interface()
        current_sim_time = self._core_nodes_interface.get_sim_time_at_time(
            (fabric_time_data["referenceTimeNumerator"], fabric_time_data["referenceTimeDenominator"])
        )
        self.assertAlmostEqual(current_sim_time, 0.01666666753590107 * 13)
        annotator_read_sim_time.detach()
        annotator_read_system_time.detach()
        fabric_time_annotator.detach()

    async def test_convert_rgba_to_rgb(self):
        import omni.syntheticdata._syntheticdata as sd

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        annotator = rep.AnnotatorRegistry.get_annotator(rv + "IsaacConvertRGBAToRGB")
        annotator.attach([self._render_product_path])

        self._timeline.play()
        await omni.syntheticdata.sensors.next_render_simulation_async(self._render_product_path, 10)
        data = annotator.get_data()
        self.assertTrue(np.all(data["data"] > 150))
        annotator.detach()

    async def test_convert_depth_to_point_cloud(self):
        import omni.syntheticdata._syntheticdata as sd

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        annotator = rep.AnnotatorRegistry.get_annotator(rv + "IsaacConvertDepthToPointCloud")
        annotator.attach([self._render_product_path])

        self._timeline.play()
        await omni.syntheticdata.sensors.next_render_simulation_async(self._render_product_path, 10)
        data = annotator.get_data()
        self.assertTrue(np.all(np.linalg.norm(data["data"], axis=1) > 0))

        annotator.detach()
