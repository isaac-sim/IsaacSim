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

import carb.tokens
import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.asset.gen.omap.bindings import _omap
from isaacsim.asset.gen.omap.utils import compute_coordinates, generate_image, update_location

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import PhysxSchema, Sdf, UsdGeom, UsdPhysics


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestOccupancyMapGenerator(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._om = _omap.acquire_omap_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        pass

    def compute_index(self, p, scale, size, min_b):
        return int(p[1] / scale - min_b[1] / scale) * int(size[0] / scale) + int(p[0] / scale - min_b[0] / scale)

    def add_cube(self, path, size, offset):

        cubeGeom = UsdGeom.Cube.Define(self._stage, path)
        cubePrim = self._stage.GetPrimAtPath(path)

        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)
        UsdPhysics.CollisionAPI.Apply(cubePrim)

        return cubeGeom

    # test to make sure this runs
    async def test_no_sim(self):
        await omni.usd.get_context().new_stage_async()
        context = omni.usd.get_context()
        self._stage = context.get_stage()
        UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))

        self.add_cube("/cube_1", 1.00, (1.00, 0, 0))
        self.add_cube("/cube_2", 1.00, (1.00, 2.00, 0))
        self.add_cube("/cube_3", 1.00, (-1.50, -1.50, 0))
        self._physx = omni.physx.get_physx_interface()
        await omni.kit.app.get_app().next_update_async()
        generator = _omap.Generator(self._physx, context.get_stage_id())
        generator.update_settings(0.05, 4, 5, 6)
        generator.set_transform((0, 0, 0), (-2.00, -2.00, 0), (2.00, 2.00, 0))
        generator.generate2d()
        buffer = generator.get_buffer()
        self.assertEqual(len(buffer), 0)

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_simple_room(self):
        (result, error) = await open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )
        # Make sure the stage loaded
        self.assertTrue(result)
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        update_location(self._om, (0, 0, 0.40 - 0.95), (-5.025, -5.025, 0), (5.025, 5.025, 0))
        cell_size = 0.05
        self._om.set_cell_size(cell_size)
        await omni.kit.app.get_app().next_update_async()
        self._om.generate()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        points = self._om.get_occupied_positions()
        # self.assertAlmostEqual(len(points), 786, delta = 1)
        scale = cell_size
        top_left, top_right, bottom_left, bottom_right, image_coords = compute_coordinates(self._om, scale)
        min_b = self._om.get_min_bound()
        max_b = self._om.get_max_bound()

        self.assertEqual(top_left, (4.975, -5.025000190734863))
        self.assertEqual(top_right, (-5.025000190734863, -5.025000190734863))
        self.assertEqual(bottom_left, (4.975, 4.975))
        self.assertEqual(bottom_right, (-5.025000190734863, 4.975))
        self.assertEqual((float(image_coords[0][0]), float(image_coords[1][0])), (5.025000190734863, 4.975))

        # size = [0, 0, 0]

        # size[0] = max_b[0] - min_b[0]
        # size[1] = max_b[1] - min_b[1]
        dims = self._om.get_dimensions()
        image_buffer = generate_image(self._om, [0, 0, 0, 255], [127, 127, 127, 255], [255, 255, 255, 255])

        # check pixel values
        self.assertEqual(image_buffer[(85 * dims[0] + 75) * 4 + 0], 0)
        self.assertEqual(image_buffer[(64 * dims[0] + 47) * 4 + 0], 255)
        self.assertEqual(image_buffer[(30 * dims[0] + 14) * 4 + 0], 127)
        self.assertEqual(image_buffer[(197 * dims[0] + 107) * 4 + 0], 0)

        # raw data: computed index, point value, point index
        # no reason for picking these specific points
        # 368 (-382.5,-322.5,42.5) 0
        # 98 (47.5,-332.5,42.5) 200
        # 14952 (-442.5,87.5,42.5) 400
        # 28922 (-12.5,477.5,42.5) 600
        # 27937 (402.5,447.5,42.5) 824

        # self.assertEqual(self.compute_index(points[0], scale, size, min_b), 363)
        # self.assertEqual(self.compute_index(points[200], scale, size, min_b), 114)
        # self.assertEqual(self.compute_index(points[400], scale, size, min_b), 20470)
        # self.assertEqual(self.compute_index(points[600], scale, size, min_b), 15307)
        # self.assertEqual(self.compute_index(points[780], scale, size, min_b), 28649)

        # This test currently fails from PIL not loading on TC

        pass

    async def test_synthetic(self):
        await omni.usd.get_context().new_stage_async()
        context = omni.usd.get_context()
        self._stage = context.get_stage()
        self.add_cube("/cube_1", 1.00, (1.00, 0, 0))
        self.add_cube("/cube_2", 1.00, (1.00, 2.00, 0))
        self.add_cube("/cube_3", 1.00, (-1.50, -1.50, 0))
        self._physx = omni.physx.get_physx_interface()

        await omni.kit.app.get_app().next_update_async()
        UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(self._stage, "/World/physicsScene")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreateSolverTypeAttr("TGS")
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        generator = _omap.Generator(self._physx, context.get_stage_id())
        generator.update_settings(0.05, 4, 5, 6)
        generator.set_transform((0, 0, 0), (-2.00, -2.00, 0), (2.00, 2.00, 0))

        for frame in range(1):
            await omni.kit.app.get_app().next_update_async()
            generator.generate2d()

        min_bounds = generator.get_min_bound()
        self.assertEqual(min_bounds[0], -2.00)
        self.assertEqual(min_bounds[1], -2.00)
        max_bounds = generator.get_max_bound()
        self.assertEqual(max_bounds[0], 2.00)
        self.assertEqual(max_bounds[1], 2.00)

        dims = generator.get_dimensions()
        self.assertEqual(dims[0], 80)
        self.assertEqual(dims[1], 80)
        # TODO: add raw occupied position checks
        # print(generator.get_occupied_positions())
        buffer = np.array(generator.get_buffer())
        self.assertEqual(len(buffer), dims[0] * dims[1])
        buffer = np.reshape(buffer, (dims[0], dims[1]))

        self.assertEqual(buffer[0, 79], 4)
        self.assertEqual(buffer[5, 75], 4)
        self.assertEqual(buffer[10, 59], 4)
        self.assertEqual(buffer[50, 20], 4)
        self.assertEqual(buffer[75, 29], 4)
        self.assertEqual(buffer[40, 40], 5)
        self.assertEqual(buffer[75, 20], 4)
