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

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.stage_templates
import omni.kit.test
import usdrt
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Usd, UsdLux, UsdPhysics


class TestPrim(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_prim_variants(self):
        assets_root_path = await get_assets_root_path_async()
        assert assets_root_path is not None, "Could not find Isaac Sim's root assets path"

        prim = stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            path="/franka",
        )
        # test cases
        # - collection
        ground_truth = {"Mesh": ["Performance", "Quality"], "Gripper": ["AlternateFinger", "Default"]}
        self.assertEqual(prim_utils.get_prim_variant_collection(prim), ground_truth, "Wrong variant collection")
        # - get variants (default)
        ground_truth = [("Gripper", "Default"), ("Mesh", "Performance")]
        self.assertEqual(prim_utils.get_prim_variants(prim), ground_truth, "Wrong default variants")
        # - set variants
        prim_utils.set_prim_variants(prim, variants=[("Gripper", "AlternateFinger"), ("Mesh", "Quality")])
        ground_truth = [("Gripper", "AlternateFinger"), ("Mesh", "Quality")]
        self.assertEqual(prim_utils.get_prim_variants(prim), ground_truth, "Wrong authored variants")

    async def test_prim_and_path(self):
        stage_utils.define_prim("/World/A", "Cube")
        # test cases
        # - USD
        prim = prim_utils.get_prim_at_path("/World/A")
        path = prim_utils.get_prim_path(prim)
        self.assertIsInstance(prim, Usd.Prim)
        self.assertTrue(prim.IsValid())
        self.assertEqual(path, "/World/A")
        # - USDRT/Fabric
        with backend_utils.use_backend("usdrt"):
            prim = prim_utils.get_prim_at_path("/World/A")
        path = prim_utils.get_prim_path(prim)
        self.assertIsInstance(prim, usdrt.Usd.Prim)
        self.assertTrue(prim.IsValid())
        self.assertEqual(path, "/World/A")
        # - Invalid path
        self.assertFalse(prim_utils.get_prim_at_path("/World/B").IsValid())

    async def test_get_all_matching_child_prims(self):
        stage_utils.define_prim("/World")
        stage_utils.define_prim("/World/A0", "Sphere")
        for i in range(3):
            stage_utils.define_prim(f"/World/A0/B{i}", "Cube" if i % 2 else "Sphere")
        for i in range(3):
            stage_utils.define_prim(f"/World/A0/B0/C{i}", "Cube" if i % 2 else "Sphere")
        # test cases
        # - valid case
        predicate = lambda prim, path: prim.GetTypeName() == "Sphere"
        # -- max_depth: None
        # --- USD
        children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate)
        for child in children:
            self.assertIsInstance(child, Usd.Prim)
        children = [prim_utils.get_prim_path(child) for child in children]
        self.assertEqual(children, ["/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2"])
        # --- USDRT/Fabric
        with backend_utils.use_backend("usdrt"):
            children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate)
            for child in children:
                self.assertIsInstance(child, usdrt.Usd.Prim)
        children = [prim_utils.get_prim_path(child) for child in children]
        self.assertEqual(children, ["/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2"])
        # -- max_depth: 0
        children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, max_depth=0)
        children = [prim_utils.get_prim_path(child) for child in children]
        self.assertEqual(children, [])
        # -- max_depth: 1
        children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, max_depth=1)
        children = [prim_utils.get_prim_path(child) for child in children]
        self.assertEqual(children, ["/World/A0/B0", "/World/A0/B2"])
        # -- max_depth: 2
        children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, max_depth=2)
        children = [prim_utils.get_prim_path(child) for child in children]
        self.assertEqual(children, ["/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2"])
        # - self-include
        # -- max_depth: None
        children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, include_self=True)
        children = [prim_utils.get_prim_path(child) for child in children]
        self.assertEqual(children, ["/World/A0", "/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2"])
        # -- max_depth: 0
        children = prim_utils.get_all_matching_child_prims(
            "/World/A0", predicate=predicate, include_self=True, max_depth=0
        )
        children = [prim_utils.get_prim_path(child) for child in children]
        self.assertEqual(children, ["/World/A0"])
        # exceptions
        self.assertRaises(
            ValueError, prim_utils.get_all_matching_child_prims, "/World/A0", predicate=predicate, max_depth=-1
        )

    async def test_get_first_matching_child_prim(self):
        stage_utils.define_prim("/World")
        stage_utils.define_prim("/World/A")
        for i in range(5):
            stage_utils.define_prim(f"/World/A/B{i}", "Cube" if i % 2 else "Sphere")
        # test cases
        # - valid case
        # -- USD
        predicate = lambda prim, path: prim.GetTypeName() == "Sphere"
        child = prim_utils.get_first_matching_child_prim("/", predicate=predicate, include_self=True)
        self.assertEqual(prim_utils.get_prim_path(child), "/World/A/B0")
        self.assertIsInstance(child, Usd.Prim)
        # -- USDRT/Fabric
        with backend_utils.use_backend("usdrt"):
            predicate = lambda prim, path: prim.GetTypeName() == "Cube"
            child = prim_utils.get_first_matching_child_prim("/World", predicate=predicate, include_self=True)
        self.assertEqual(prim_utils.get_prim_path(child), "/World/A/B1")
        self.assertIsInstance(child, usdrt.Usd.Prim)
        # - no match
        self.assertIsNone(prim_utils.get_first_matching_child_prim("/World/A", predicate=lambda *_: False))
        # - self-include
        predicate = lambda prim, path: prim.GetTypeName() == "Xform"
        # -- include self
        child = prim_utils.get_first_matching_child_prim("/World", predicate=predicate, include_self=True)
        self.assertEqual(prim_utils.get_prim_path(child), "/World")
        # -- exclude self
        child = prim_utils.get_first_matching_child_prim("/World", predicate=predicate, include_self=False)
        self.assertEqual(prim_utils.get_prim_path(child), "/World/A")

    async def test_get_first_matching_parent_prim(self):
        stage_utils.define_prim("/World")
        stage_utils.define_prim("/World/Cube", "Cube")
        stage_utils.define_prim("/World/Cube/Sphere", "Sphere")
        # test cases
        # - valid case
        # -- USD
        predicate = lambda prim, path: prim.GetTypeName() == "Xform"
        parent = prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate)
        self.assertEqual(prim_utils.get_prim_path(parent), "/World")
        self.assertIsInstance(parent, Usd.Prim)
        # -- USDRT/Fabric
        with backend_utils.use_backend("usdrt"):
            parent = prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate)
        self.assertEqual(prim_utils.get_prim_path(parent), "/World")
        self.assertIsInstance(parent, usdrt.Usd.Prim)
        # - no match
        self.assertIsNone(prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=lambda *_: False))
        # - root prim (pseudo-root prim)
        predicate = lambda prim, path: path == "/"
        self.assertIsNone(prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate))
        # - self-include
        predicate = lambda prim, path: prim.GetTypeName() == "Sphere"
        # -- include self
        parent = prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate, include_self=True)
        self.assertEqual(prim_utils.get_prim_path(parent), "/World/Cube/Sphere")
        # -- exclude self
        self.assertIsNone(
            prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate, include_self=False)
        )

    async def test_has_api(self):
        prim = stage_utils.define_prim("/World/A", "Cube")
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdLux.LightAPI.Apply(prim)
        # test cases
        # - all
        self.assertTrue(prim_utils.has_api("/World/A", UsdPhysics.RigidBodyAPI, test="all"))
        self.assertTrue(prim_utils.has_api("/World/A", ["PhysicsRigidBodyAPI", UsdLux.LightAPI], test="all"))
        self.assertFalse(
            prim_utils.has_api("/World/A", ["PhysicsMassAPI", "PhysicsRigidBodyAPI", UsdLux.LightAPI], test="all")
        )
        # - any
        self.assertTrue(prim_utils.has_api("/World/A", UsdPhysics.RigidBodyAPI, test="any"))
        self.assertTrue(
            prim_utils.has_api("/World/A", ["PhysicsMassAPI", "PhysicsRigidBodyAPI", UsdLux.LightAPI], test="any")
        )
        self.assertFalse(prim_utils.has_api("/World/A", "PhysicsMassAPI", test="any"))
        # - none
        self.assertTrue(prim_utils.has_api("/World/A", "PhysicsMassAPI", test="none"))
        self.assertFalse(prim_utils.has_api("/World/A", UsdPhysics.RigidBodyAPI, test="none"))
        self.assertFalse(
            prim_utils.has_api("/World/A", ["PhysicsMassAPI", "PhysicsRigidBodyAPI", UsdLux.LightAPI], test="none")
        )
        # exceptions
        self.assertRaises(ValueError, prim_utils.has_api, "/World/A", "UnexistingAPI", test="unknown")
