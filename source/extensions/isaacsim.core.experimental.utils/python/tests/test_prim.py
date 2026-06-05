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

"""Test for prim."""

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.foundation as foundation_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.stage_templates
import omni.kit.test
import usdrt
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


class TestPrim(omni.kit.test.AsyncTestCase):
    """Test prim."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_prim_variants(self):
        """Test prim variants."""
        assets_root_path = await get_assets_root_path_async(skip_check=True)
        prim = stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            path="/franka",
        )
        # test cases
        # - collection
        ground_truth = {
            "Mesh": ["Performance", "Quality"],
            "Gripper": ["AlternateFinger", "Default", "None", "Robotiq_2F_85"],
        }
        self.assertEqual(prim_utils.get_prim_variant_collection(prim), ground_truth, "Wrong variant collection")
        # - get variants (default)
        ground_truth = [("Gripper", "Default"), ("Mesh", "Performance")]
        self.assertEqual(prim_utils.get_prim_variants(prim), ground_truth, "Wrong default variants")
        # - set variants
        prim_utils.set_prim_variants(prim, variants=[("Gripper", "AlternateFinger"), ("Mesh", "Quality")])
        ground_truth = [("Gripper", "AlternateFinger"), ("Mesh", "Quality")]
        self.assertEqual(prim_utils.get_prim_variants(prim), ground_truth, "Wrong authored variants")

    async def test_prim_and_path(self):
        """Test prim and path."""
        usd_prim = stage_utils.define_prim("/World/A", "Cube")
        with backend_utils.use_backend("usdrt"):
            usdrt_prim = stage_utils.define_prim("/World/B", "Cube")
        # test cases
        # - USD
        for item in ["/World/A", usd_prim, UsdGeom.Cube(usd_prim)]:
            prim = prim_utils.get_prim_at_path(item)
            path = prim_utils.get_prim_path(item)
            self.assertIsInstance(prim, Usd.Prim)
            self.assertTrue(prim.IsValid())
            self.assertEqual(path, "/World/A")
        # - USDRT/Fabric
        for item in ["/World/B", usdrt_prim, usdrt.UsdGeom.Cube(usdrt_prim)]:
            with backend_utils.use_backend("usdrt"):
                prim = prim_utils.get_prim_at_path(item)
                path = prim_utils.get_prim_path(item)
            self.assertIsInstance(prim, usdrt.Usd.Prim)
            self.assertTrue(prim.IsValid())
            self.assertEqual(path, "/World/B")
        # - Invalid path
        self.assertFalse(prim_utils.get_prim_at_path("/World/C").IsValid())

    async def test_find_matching_prim_paths(self):
        """Test find matching prim paths."""
        stage_utils.define_prim("/World/A")
        for i in range(2):
            stage_utils.define_prim(f"/World/A{i}")
            stage_utils.define_prim(f"/World/A{i}/B")
            for j in range(3):
                stage_utils.define_prim(f"/World/A{i}/B{j}")
                stage_utils.define_prim(f"/World/A{i}/B{j}/C")
        # test cases
        for backend in ["usd", "usdrt", "fabric"]:
            with backend_utils.use_backend(backend):
                # - valid prim path
                match = ["/World/A0/B0"]
                self.assertEqual(prim_utils.find_matching_prim_paths("/World/A0/B0"), match)
                match = ["/World/A0/B0", "/World/A0/B0/C"]
                self.assertEqual(prim_utils.find_matching_prim_paths("/World/A0/B0", traverse=True), match)
                # - regex
                # --
                match = ["/World/A0/B0", "/World/A0/B1", "/World/A0/B2"]
                self.assertEqual(prim_utils.find_matching_prim_paths("/World/A0/B[0-9]"), match)
                match = [
                    "/World/A0/B0",
                    "/World/A0/B0/C",
                    "/World/A0/B1",
                    "/World/A0/B1/C",
                    "/World/A0/B2",
                    "/World/A0/B2/C",
                ]
                self.assertEqual(prim_utils.find_matching_prim_paths("/World/A0/B[0-9]", traverse=True), match)
                # --
                match = ["/World/A0/B1", "/World/A0/B2", "/World/A1/B1", "/World/A1/B2"]
                self.assertEqual(prim_utils.find_matching_prim_paths("/World/.*/.*[1,2]"), match)
                match = [
                    "/World/A0/B1",
                    "/World/A0/B1/C",
                    "/World/A0/B2",
                    "/World/A0/B2/C",
                    "/World/A1/B1",
                    "/World/A1/B1/C",
                    "/World/A1/B2",
                    "/World/A1/B2/C",
                ]
                self.assertEqual(prim_utils.find_matching_prim_paths("/World/.*/.*[1,2]", traverse=True), match)
                # --
                match = []
                self.assertEqual(prim_utils.find_matching_prim_paths(".*C.*"), [])
                match = [
                    "/World/A0/B0/C",
                    "/World/A0/B1/C",
                    "/World/A0/B2/C",
                    "/World/A1/B0/C",
                    "/World/A1/B1/C",
                    "/World/A1/B2/C",
                ]
                self.assertEqual(prim_utils.find_matching_prim_paths(".*C.*", traverse=True), match)

    async def test_get_all_matching_child_prims(self):
        """Test get all matching child prims."""
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
        """Test get first matching child prim."""
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
        """Test get first matching parent prim."""
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
        """Test has api."""
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

    async def test_attributes(self):
        """Test attributes."""
        stage_utils.define_prim(path := "/World/A", "Xform")
        for i, format_ in enumerate([str, Sdf.ValueTypeNames, usdrt.Sdf.ValueTypeNames]):
            for j, value_type_name in enumerate(foundation_utils.get_value_type_names(format=format_)):
                for backend in ["usd", "usdrt", "fabric"]:
                    with backend_utils.use_backend(backend):
                        name = f"{backend}_attr_{i}_{j}"
                        attribute = prim_utils.create_prim_attribute(path, name=name, type_name=value_type_name)
                        self.assertIsInstance(attribute, Usd.Attribute if backend == "usd" else usdrt.Usd.Attribute)
                        attribute.Get()
                        # exceptions
                        with self.assertRaises(RuntimeError):
                            prim_utils.create_prim_attribute(path, name=name, type_name=value_type_name, exist_ok=False)

    async def test_is_prim_non_root_articulation_link(self):
        """Test is prim non root articulation link."""
        assets_root_path = await get_assets_root_path_async(skip_check=True)
        stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            path="/franka",
        )
        # test cases
        for backend in ["usd", "usdrt", "fabric"]:
            with backend_utils.use_backend(backend):
                self.assertFalse(prim_utils.is_prim_non_root_articulation_link("/franka"))
                self.assertFalse(prim_utils.is_prim_non_root_articulation_link(prim_utils.get_prim_at_path("/franka")))
                # - link prims
                self.assertTrue(prim_utils.is_prim_non_root_articulation_link("/franka/panda_link1"))
                self.assertTrue(prim_utils.is_prim_non_root_articulation_link("/franka/panda_link0"))
                # - non-link prims
                self.assertFalse(prim_utils.is_prim_non_root_articulation_link("/franka/panda_hand/geometry"))
                self.assertFalse(prim_utils.is_prim_non_root_articulation_link("/franka/rootJoint"))

    async def test_get_prim_attribute_value(self):
        """Test get prim attribute value."""
        # create a Cube prim (has scalar 'size' attribute)
        prim = stage_utils.define_prim("/World/Cube", "Cube")
        # create custom vector attribute to test vector-to-list conversion
        prim_utils.create_prim_attribute("/World/Cube", name="testVec3f", type_name=Sdf.ValueTypeNames.Float3)
        prim.GetAttribute("testVec3f").Set((1.0, 2.0, 3.0))
        # test cases for all backends
        for backend in ["usd", "usdrt", "fabric"]:
            with backend_utils.use_backend(backend):
                # - scalar attribute (size)
                size = prim_utils.get_prim_attribute_value("/World/Cube", "size")
                self.assertEqual(size, 2.0)
                # - vector attribute (float3) - should be returned as list
                vec_value = prim_utils.get_prim_attribute_value("/World/Cube", "testVec3f")
                self.assertEqual(list(vec_value), [1.0, 2.0, 3.0])
                # - using prim instance instead of path
                size = prim_utils.get_prim_attribute_value(prim_utils.get_prim_at_path("/World/Cube"), "size")
                self.assertEqual(size, 2.0)
        # - exception: non-existent attribute
        with self.assertRaises(ValueError):
            prim_utils.get_prim_attribute_value("/World/Cube", "nonexistent_attr")

    async def test_get_prim_attribute_names(self):
        """Test get prim attribute names."""
        for backend in ["usd", "usdrt", "fabric"]:
            with backend_utils.use_backend(backend):
                prim_type = "Xform" if backend == "usdrt" else "Cube"
                prim_path = f"/World/AttrPrim_{backend}"
                prim = stage_utils.define_prim(prim_path, prim_type)
                attr_name = f"customAttr_{backend}"
                prim_utils.create_prim_attribute(prim_path, name=attr_name, type_name=usdrt.Sdf.ValueTypeNames.Float)
                prim.GetAttribute(attr_name).Set(1.25)

                attribute_names = prim_utils.get_prim_attribute_names(prim_path)

                if backend not in ["usdrt", "fabric"]:
                    self.assertIn("size", attribute_names)
                    self.assertIn("extent", attribute_names)
                self.assertIn(attr_name, attribute_names)
