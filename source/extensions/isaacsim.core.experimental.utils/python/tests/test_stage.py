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

"""Test for stage."""

import os
import tempfile

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.stage_templates
import omni.kit.test
import omni.usd
import usdrt
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils


class TestStage(omni.kit.test.AsyncTestCase):
    """Test stage."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # ---------------
        # Warning: don't create stage in the setUp method since we test the stage creation
        # ---------------

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        # ------------------
        stage_utils.close_stage()
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_context_manager(self):
        """Test context manager."""
        await stage_utils.create_new_stage_async()
        stage_in_memory = Usd.Stage.CreateInMemory()
        default_stage = omni.usd.get_context().get_stage()
        default_stage_id = UsdUtils.StageCache.Get().GetId(default_stage).ToLongInt()
        self.assertIs(stage_utils.get_current_stage(), default_stage)
        self.assertFalse(stage_utils.is_stage_set())
        self.assertEqual(stage_utils.get_stage_id(default_stage), default_stage_id)
        # - USD stage
        with stage_utils.use_stage(stage_in_memory):
            self.assertIs(stage_utils.get_current_stage(), stage_in_memory)
            self.assertTrue(stage_utils.is_stage_set())
            self.assertIsNot(stage_utils.get_current_stage(), default_stage)
            self.assertNotEqual(stage_utils.get_stage_id(stage_utils.get_current_stage()), default_stage_id)
        self.assertIs(stage_utils.get_current_stage(), default_stage)
        self.assertFalse(stage_utils.is_stage_set())
        self.assertEqual(stage_utils.get_stage_id(stage_utils.get_current_stage()), default_stage_id)
        # - USDRT/Fabric stage
        # -- via function argument
        with stage_utils.use_stage(stage_in_memory):
            self.assertIsInstance(stage_utils.get_current_stage(backend="usdrt"), usdrt.Usd.Stage)
            self.assertIsInstance(stage_utils.get_current_stage(backend="fabric"), usdrt.Usd.Stage)
        self.assertIsInstance(stage_utils.get_current_stage(backend="usdrt"), usdrt.Usd.Stage)
        self.assertIsInstance(stage_utils.get_current_stage(backend="fabric"), usdrt.Usd.Stage)
        # -- via context manager
        with backend_utils.use_backend("usdrt"):
            with stage_utils.use_stage(stage_in_memory):
                self.assertIsInstance(stage_utils.get_current_stage(), usdrt.Usd.Stage)
            self.assertIsInstance(stage_utils.get_current_stage(), usdrt.Usd.Stage)
        with backend_utils.use_backend("fabric"):
            with stage_utils.use_stage(stage_in_memory):
                self.assertIsInstance(stage_utils.get_current_stage(), usdrt.Usd.Stage)
            self.assertIsInstance(stage_utils.get_current_stage(), usdrt.Usd.Stage)

    async def test_create_new_stage(self):
        """Test create new stage."""
        templates = sorted([name for item in omni.kit.stage_templates.get_stage_template_list() for name in item])
        self.assertEqual(templates, ["default stage", "empty", "sunlight"], f"Available templates: {templates}")
        # test cases
        # - sync
        for template in [None, "gridroom"] + templates:
            stage = stage_utils.create_new_stage(template=template)
            self.assertIsInstance(stage, Usd.Stage)
            self.assertIs(stage, stage_utils.get_current_stage())
            self.assertEqual(
                stage.GetPrimAtPath("/World").IsValid(),
                template is not None,
                f"Invalid stage content for the given template: {template}",
            )
        # - async
        for template in [None, "gridroom"] + templates:
            stage = await stage_utils.create_new_stage_async(template=template)
            self.assertIsInstance(stage, Usd.Stage)
            self.assertIs(stage, stage_utils.get_current_stage())
            self.assertEqual(
                stage.GetPrimAtPath("/World").IsValid(),
                template is not None,
                f"Invalid stage content for the given template: {template}",
            )

    async def test_is_stage_loading(self):
        """Test is stage loading."""
        await stage_utils.create_new_stage_async()
        stage_utils.is_stage_loading()

    async def test_open_stage(self):
        """Test open stage."""
        assets_root_path = await get_assets_root_path_async(skip_check=True)
        # test cases
        # - sync
        result, stage = stage_utils.open_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        )
        self.assertTrue(result, "Failed to open stage")
        self.assertTrue(stage.GetPrimAtPath("/panda/panda_hand").IsValid())
        # - async
        await stage_utils.create_new_stage_async()
        result, stage = await stage_utils.open_stage_async(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        )
        self.assertTrue(result, "Failed to open stage")
        self.assertTrue(stage.GetPrimAtPath("/panda/panda_hand").IsValid())

    async def test_save_close_stage(self):
        """Test save close stage."""
        assets_root_path = await get_assets_root_path_async(skip_check=True)
        # create and populate stage
        await stage_utils.create_new_stage_async()
        stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            path="/World/panda",
            variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
        )
        # save and close stage, then open it again
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp_dir:
            tmp_file = os.path.join(tmp_dir, "test_save_close_stage.usd")
            # - save stage
            stage_utils.save_stage(usd_path=tmp_file)
            self.assertTrue(os.path.exists(tmp_file) and os.path.isfile(tmp_file))
            # - close stage
            self.assertTrue(stage_utils.close_stage())
            self.assertRaises(ValueError, stage_utils.get_current_stage)
            # - open stage
            result, stage = stage_utils.open_stage(usd_path=tmp_file)
            self.assertTrue(result)
            self.assertTrue(stage.GetPrimAtPath("/World/panda/panda_hand").IsValid())

    async def test_add_reference_to_stage(self):
        """Test add reference to stage."""
        assets_root_path = await get_assets_root_path_async(skip_check=True)
        # create and populate stage
        await stage_utils.create_new_stage_async()
        prim = stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            path="/World/panda",
            variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
        )
        self.assertIsInstance(prim, Usd.Prim)
        self.assertEqual(prim.GetPath(), "/World/panda")
        self.assertEqual(prim.GetVariantSet("Gripper").GetVariantSelection(), "AlternateFinger")
        self.assertEqual(prim.GetVariantSet("Mesh").GetVariantSelection(), "Performance")

    async def test_define_prim(self):
        """Test define prim."""
        await stage_utils.create_new_stage_async()
        specs = [
            # UsdGeomTokensType
            ("Camera", UsdGeom.Camera),
            ("Capsule", UsdGeom.Capsule),
            ("Cone", UsdGeom.Cone),
            ("Cube", UsdGeom.Cube),
            ("Cylinder", UsdGeom.Cylinder),
            ("Mesh", UsdGeom.Mesh),
            ("Plane", UsdGeom.Plane),
            ("Points", UsdGeom.Points),
            ("Scope", UsdGeom.Scope),
            ("Sphere", UsdGeom.Sphere),
            ("Xform", UsdGeom.Xform),
            # UsdLuxTokensType
            ("CylinderLight", UsdLux.CylinderLight),
            ("DiskLight", UsdLux.DiskLight),
            ("DistantLight", UsdLux.DistantLight),
            ("DomeLight", UsdLux.DomeLight),
            ("RectLight", UsdLux.RectLight),
            ("SphereLight", UsdLux.SphereLight),
            # UsdPhysicsTokensType
            ("PhysicsScene", UsdPhysics.Scene),
        ]
        # USD prim
        for token, prim_type in specs:
            prim = stage_utils.define_prim(f"/{token}", type_name=token)
            self.assertTrue(prim.IsA(prim_type), f"Prim ({prim.GetPath()}) is not a {prim_type}")
        # USDRT prim
        with backend_utils.use_backend("usdrt"):
            for token, _ in specs:
                prim = stage_utils.define_prim(f"/{token}", type_name=token)
                self.assertIsInstance(prim, usdrt.Usd.Prim, f"Prim ({prim.GetPath()}) is not a USDRT prim")
        # exceptions
        # - non-absolute path
        self.assertRaises(ValueError, stage_utils.define_prim, "World")
        # - non-valid path
        self.assertRaises(ValueError, stage_utils.define_prim, "/World/")
        # - prim already exists with a different type
        self.assertRaises(RuntimeError, stage_utils.define_prim, "/Sphere", type_name="Cube")

    async def test_delete_prim(self):
        """Test delete prim."""
        assets_root_path = await get_assets_root_path_async(skip_check=True)
        # create and populate stage
        await stage_utils.create_new_stage_async()
        prim = stage_utils.define_prim("/World/A", "Xform")
        stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            path="/World/panda",
            variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
        )
        # test cases
        self.assertTrue(stage_utils.delete_prim(prim))
        self.assertFalse(stage_utils.delete_prim("/World/panda/panda_hand"))  # ancestral prim (cannot be deleted)
        self.assertTrue(stage_utils.delete_prim("/World/panda"))
        # exceptions
        self.assertRaisesRegex(ValueError, "not a valid prim", stage_utils.delete_prim, "/World/A")

    async def test_move_prim(self):
        """Test move prim."""
        stage = await stage_utils.create_new_stage_async()
        prim_a = stage_utils.define_prim("/World/A", "Xform")
        prim_b = stage_utils.define_prim("/World/B", "Xform")
        # test cases
        # - move A to (inside) B
        result, path = stage_utils.move_prim(prim_a, prim_b)
        self.assertTrue(result)
        self.assertEqual(path, "/World/B/A")
        self.assertEqual(stage.GetPrimAtPath("/World/B/A").IsValid(), True)
        self.assertEqual(stage.GetPrimAtPath("/World/A").IsValid(), False)
        # - move A next to B
        result, path = stage_utils.move_prim("/World/B/A", "/World")
        self.assertTrue(result)
        self.assertEqual(path, "/World/A")
        self.assertEqual(stage.GetPrimAtPath("/World/A").IsValid(), True)
        self.assertEqual(stage.GetPrimAtPath("/World/B/A").IsValid(), False)
        # - move A to root
        result, path = stage_utils.move_prim("/World/A", "/")
        self.assertTrue(result)
        self.assertEqual(path, "/A")
        self.assertEqual(stage.GetPrimAtPath("/A").IsValid(), True)
        self.assertEqual(stage.GetPrimAtPath("/World/A").IsValid(), False)
        # - move A to an unexisting path
        result, path = stage_utils.move_prim("/A", "/World/C")
        self.assertTrue(result)
        self.assertEqual(path, "/World/C")
        self.assertEqual(stage.GetPrimAtPath("/World/C").IsValid(), True)
        self.assertEqual(stage.GetPrimAtPath("/A").IsValid(), False)
        # exceptions
        # - prim is not a valid prim
        self.assertRaisesRegex(ValueError, "not a valid prim", stage_utils.move_prim, "/ABC", "/")
        # - destination path is not a valid path string
        self.assertRaisesRegex(ValueError, "not a valid path string", stage_utils.move_prim, "/World/B", "?")
        # - destination path has unexisting parents
        self.assertRaisesRegex(ValueError, "unexisting parent", stage_utils.move_prim, "/World/B", "/World/X/Y")

    async def test_stage_units(self):
        """Test stage units."""
        await stage_utils.create_new_stage_async()
        # test cases
        # - default units
        self.assertEqual(stage_utils.get_stage_units(), (1.0, 1.0))
        # - random units
        for meters_per_unit, kilograms_per_unit in np.random.rand(10, 2):
            stage_utils.set_stage_units(meters_per_unit=meters_per_unit, kilograms_per_unit=kilograms_per_unit)
            self.assertEqual(stage_utils.get_stage_units(), (meters_per_unit, kilograms_per_unit))

    async def test_stage_up_axis(self):
        """Test stage up axis."""
        await stage_utils.create_new_stage_async()
        # test cases
        # - default up axis
        self.assertEqual(stage_utils.get_stage_up_axis(), "Z")
        # - supported up axis
        for up_axis in ["Y", "Z", "y", "z"]:
            stage_utils.set_stage_up_axis(up_axis)
            self.assertEqual(stage_utils.get_stage_up_axis(), up_axis.upper())

    async def test_stage_time_code(self):
        """Test stage time code."""
        await stage_utils.create_new_stage_async()
        # test cases
        # - default time code
        self.assertEqual(stage_utils.get_stage_time_code(), (0.0, 100.0, 60.0))
        # - random time code
        for start_time_code, end_time_code, time_codes_per_second in np.random.rand(10, 3):
            stage_utils.set_stage_time_code(
                start_time_code=start_time_code,
                end_time_code=end_time_code,
                time_codes_per_second=time_codes_per_second,
            )
            self.assertEqual(stage_utils.get_stage_time_code(), (start_time_code, end_time_code, time_codes_per_second))

    async def test_generate_next_free_path(self):
        """Test generate next free path."""
        await stage_utils.create_new_stage_async()
        stage_utils.get_current_stage(backend="usd").SetDefaultPrim(stage_utils.define_prim("/World"))
        stage_utils.define_prim("/World/Xform")
        for path, expected_path_with_default_prim, expected_path_without_default_prim in [
            (None, "/World/Prim", "/Prim"),
            ("", "/World/Prim", "/Prim"),
            ("/", "/World/Prim", "/Prim"),
            ("/World", "/World/World", "/World_01"),
            ("/World/Xform", "/World/Xform_01", "/World/Xform_01"),
            ("ABC", "/World/ABC", "/ABC"),
            ("/ABC", "/World/ABC", "/ABC"),
        ]:
            for prepend_default_prim in [True, False]:
                self.assertEqual(
                    stage_utils.generate_next_free_path(path, prepend_default_prim=prepend_default_prim),
                    expected_path_with_default_prim if prepend_default_prim else expected_path_without_default_prim,
                    f"path: {path}, prepend_default_prim: {prepend_default_prim}",
                )
