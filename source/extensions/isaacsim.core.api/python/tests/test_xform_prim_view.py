# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import numpy as np
import omni.kit.test
from isaacsim.core.api import World

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdGeom

from .common import CoreTestCase


class TestXFormPrimView(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World()
        await self._my_world.initialize_simulation_context_async()
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
        define_prim(prim_path="/World/Frame_1")
        define_prim(prim_path="/World/Frame_2")
        define_prim(prim_path="/World/Frame_3")
        define_prim(prim_path="/World/Frame_1/Target")
        define_prim(prim_path="/World/Frame_2/Target")
        define_prim(prim_path="/World/Frame_3/Target")
        self._frankas_view = XFormPrim(prim_paths_expr="/World/Franka_[1-2]", name="frankas_view")
        self._targets_view = XFormPrim(prim_paths_expr="/World/Frame_[1-3]/Target", name="targets_view")
        self._frames_view = XFormPrim(prim_paths_expr="/World/Frame_[1-3]", name="frames_view")
        pass

    async def tearDown(self):
        await super().tearDown()

    async def test_list_of_regular_exprs(self):
        view = XFormPrim(prim_paths_expr=["/World/Franka_[1-2]", "/World/Frame_*"], name="random_view")
        self.assertTrue(view.count == 5)

    async def test_world_poses(self):
        current_positions, current_orientations = self._frankas_view.get_world_poses()
        self.assertTrue(np.isclose(current_positions, np.zeros([2, 3], dtype=np.float32)).all())
        expected_orientations = np.zeros([2, 4], dtype=np.float32)
        expected_orientations[:, 0] = 1
        self.assertTrue(np.isclose(current_orientations, expected_orientations).all())

        new_positions = np.array([[10.0, 10.0, 0], [-40, -40, 0]])
        new_orientations = euler_angles_to_quats(np.array([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0]]))
        self._frankas_view.set_world_poses(positions=new_positions, orientations=new_orientations)
        current_positions, current_orientations = self._frankas_view.get_world_poses()
        self.assertTrue(np.isclose(current_positions, new_positions).all())
        self.assertTrue(
            np.logical_or(
                np.isclose(current_orientations, new_orientations, atol=1e-05).all(axis=1),
                np.isclose(current_orientations, -new_orientations, atol=1e-05).all(axis=1),
            ).all()
        )
        return

    @unittest.skipIf(os.getenv("ETM_ACTIVE"), "skipped in ETM")
    async def test_world_poses_fabric(self):
        current_positions, current_orientations = self._frankas_view.get_world_poses(usd=False)
        self.assertTrue(np.isclose(current_positions, np.zeros([2, 3], dtype=np.float32)).all())
        expected_orientations = np.zeros([2, 4], dtype=np.float32)
        expected_orientations[:, 0] = 1
        self.assertTrue(np.isclose(current_orientations, expected_orientations).all())

        new_positions = np.array([[10.0, 10.0, 0], [-40, -40, 0]])
        new_orientations = euler_angles_to_quats(np.array([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0]]))
        self._frankas_view.set_world_poses(positions=new_positions, orientations=new_orientations, usd=False)
        current_positions, current_orientations = self._frankas_view.get_world_poses(usd=False)
        self.assertTrue(np.isclose(current_positions, new_positions).all())
        self.assertTrue(
            np.logical_or(
                np.isclose(current_orientations, new_orientations, atol=1e-05).all(axis=1),
                np.isclose(current_orientations, -new_orientations, atol=1e-05).all(axis=1),
            ).all()
        )
        return

    async def test_local_pose(self):
        self._frames_view.set_local_poses(
            translations=np.array([[0, 0, 0], [0, 10, 5], [0, 3, 5]]),
            orientations=euler_angles_to_quats(
                np.array([[0, 180, 0], [180, 0, 0], [0, 0, 180]]), extrinsic=True, degrees=True
            ),
        )
        self._targets_view.set_local_poses(
            translations=np.array([[0, 20, 10], [0, 30, 20], [0, 50, 10]]),
            orientations=euler_angles_to_quats(
                np.array([[0, -180, 0], [-180, 0, 0], [0, 0, -180]]), extrinsic=True, degrees=True
            ),
        )
        current_positions, current_orientations = self._targets_view.get_world_poses()
        self.assertTrue(np.isclose(current_positions, np.array([[0, 20, -10], [0, -20, -15], [0, -47, 15]])).all())
        self.assertTrue(
            np.isclose(current_orientations, euler_angles_to_quats(np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]))).all()
        )
        return

    async def test_local_on_init(self):
        initial_translations = np.array([[0, 0, 0], [0, 10, 5]])
        view = XFormPrim(
            prim_paths_expr="/World/Franka_[1-2]", name="frankas_view_2", translations=initial_translations
        )
        current_translations, current_orientations = view.get_local_poses()
        self.assertTrue(np.isclose(current_translations, initial_translations).all())
        return

    async def test_visibilities(self):
        prim_paths = "/World/Franka_[1-2]"
        visibilities = np.array([False, True])
        view = XFormPrim(prim_paths_expr=prim_paths, visibilities=visibilities)
        for i in range(len(view.prim_paths)):
            imageable = UsdGeom.Imageable(view.prims[i])
            visibility_attr = imageable.GetVisibilityAttr().Get()
            if visibilities[i]:
                self.assertEqual(visibility_attr, "inherited")
            else:
                self.assertEqual(visibility_attr, "invisible")

    async def test_scale_units_resolve(self):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World()
        await self._my_world.initialize_simulation_context_async()
        os.path.dirname(os.path.abspath(__file__))
        add_reference_to_stage(
            usd_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/cm_asset.usd"), prim_path="/World"
        )
        prim_view = XFormPrim(prim_paths_expr="/World/cereal_box")
        xformable = UsdGeom.Xformable(prim_view.prims[0])
        self.assertTrue("xformOp:scale:unitsResolve" not in xformable.GetXformOpOrderAttr().Get())
        self.assertTrue(np.isclose(prim_view.get_local_scales()[0], np.array([0.01, 0.01, 0.01])).all())
