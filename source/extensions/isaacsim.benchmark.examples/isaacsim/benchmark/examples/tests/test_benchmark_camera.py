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

import asyncio

import carb
import numpy as np
import omni.kit.test
from isaacsim.benchmark.services import BaseIsaacBenchmarkAsync
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.sensors.camera import Camera
from omni.kit.viewport.utility import get_active_viewport


class TestBenchmarkCamera(BaseIsaacBenchmarkAsync):
    async def setUp(self):
        await super().setUp()
        self._num_app_updates = carb.settings.get_settings().get_as_int(
            "/exts/isaacsim.benchmark.examples/num_app_updates"
        )
        pass

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        await super().tearDown()
        pass

    async def benchmark_camera(self, n_camera, resolution):
        self.benchmark_name = f"cameras_{n_camera}_resolution_{resolution[0]}_{resolution[1]}"
        self.set_phase("loading")

        scene_path = carb.settings.get_settings().get_as_string("/exts/isaacsim.benchmark.examples/scene_path")
        if len(scene_path) > 0:
            await self.fully_load_stage(self.assets_root_path + scene_path)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        cameras = []

        for i in range(n_camera):
            render_product_path = None
            if i == 0:
                viewport_api = get_active_viewport()
                render_product_path = viewport_api.get_render_product_path()
            cameras.append(
                Camera(
                    prim_path="/Cameras/Camera_" + str(i),
                    position=np.array([-8, 13, 2.0]),
                    resolution=resolution,
                    orientation=euler_angles_to_quat([90, 0, 90 + i * 360 / n_camera], degrees=True),
                    render_product_path=render_product_path,
                )
            )

            await omni.kit.app.get_app().next_update_async()
            cameras[i].initialize()

        # make sure scene is loaded in all viewports
        while is_stage_loading():
            print("asset still loading, waiting to finish")
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        await self.store_measurements()

        # perform benchmark
        self.set_phase("benchmark")

        for _ in range(self._num_app_updates):
            await omni.kit.app.get_app().next_update_async()

        await self.store_measurements()

        timeline.stop()
        cameras = None

    # ----------------------------------------------------------------------
    async def test_benchmark_1_camera_720p(self):
        await self.benchmark_camera(1, [1280, 720])
