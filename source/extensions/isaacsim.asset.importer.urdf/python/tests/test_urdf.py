# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies URDF importer behavior across basic, textured, saved, scaled, floating-base, and asset-backed robot imports. Covers joint limits, mimic parsing, materials, collision generation, missing assets, debug mode, and generated USD output."""

import asyncio
import gc
import os
import shutil
import tempfile
import unittest

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import pxr
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr, PhysxSchema
from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdShade


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestUrdf(omni.kit.test.AsyncTestCase):
    """Test URDF importer conversion workflows.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...
    """

    # Before running each test
    async def setUp(self) -> None:
        """Prepare shared test fixtures.

        Example:

        .. code-block:: python

            >>> import omni.timeline
            >>> omni.timeline.get_timeline_interface()  # doctest: +SKIP
        """
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._tmpdir = tempfile.mkdtemp(prefix="urdf_test_")
        # Redirect tempfile's default directory to self._tmpdir so any
        # mkdtemp() calls made during the test (e.g. the URDF importer's
        # private scratch dir in non-debug mode) are rooted inside
        # self._tmpdir and cleaned up deterministically in tearDown.
        self._prev_tempdir = tempfile.tempdir
        tempfile.tempdir = self._tmpdir
        self._success = False
        self.importer = URDFImporter()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test output and wait for stage unloading to complete.

        Example:

        .. code-block:: python

            >>> import asyncio
            >>> asyncio.sleep(0)  # doctest: +SKIP
        """
        if self._timeline.is_playing():
            self._timeline.stop()
        # Flush several run-loop frames so the timeline plugin fully processes
        # the stop event before the next test's new_stage_async() destroys the
        # current stage (avoiding a SIGSEGV in UsdStage::~UsdStage).
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        self._stage = None
        gc.collect()
        tempfile.tempdir = self._prev_tempdir
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    async def standard_checks(self, prim_path: str) -> None:
        """Validate standard properties of imported URDF prims.

        Checks that the stage uses meters as units and that all meshes have
        non-zero vertex counts.

        Args:
            prim_path: USD path to the root prim to validate.
        """
        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self._stage), 1.0)

        prim = self._stage.GetPrimAtPath(prim_path)
        # check that all meshes have >0 vertices
        prim_range = [c for c in pxr.Usd.PrimRange(prim, pxr.Usd.TraverseInstanceProxies()) if UsdGeom.Mesh(c)]
        for prim in prim_range:
            mesh = UsdGeom.Mesh(prim)
            self.assertGreater(len(mesh.GetFaceVertexCountsAttr().Get()), 0)

        self._timeline.play()
        for _ in range(30):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        await omni.kit.app.get_app().next_update_async()

    def _import_urdf(
        self,
        urdf_path: str,
        usd_path: str | None = None,
        collision_from_visuals: bool | None = None,
        collision_type: str | None = None,
        allow_self_collision: bool | None = None,
        merge_mesh: bool | None = None,
        debug_mode: bool | None = None,
    ) -> tuple[str, str]:
        """Import a URDF file with the new importer API.

        Args:
            urdf_path: Absolute path to the URDF file.
            usd_path: Output directory for generated USD assets. Defaults to self._tmpdir.
            collision_from_visuals: Whether to generate collisions from visuals.
            collision_type: Collision geometry type.
            allow_self_collision: Whether to enable self-collision.
            merge_mesh: Whether to merge meshes after conversion.
            debug_mode: Whether to keep intermediate outputs.

        Returns:
            Tuple of (output_path, prim_path).
        """
        config = URDFImporterConfig()
        config.urdf_path = os.path.normpath(urdf_path)
        config.usd_path = os.path.normpath(usd_path) if usd_path else os.path.normpath(self._tmpdir)
        if collision_from_visuals is not None:
            config.collision_from_visuals = collision_from_visuals
        if collision_type is not None:
            config.collision_type = collision_type
        if allow_self_collision is not None:
            config.allow_self_collision = allow_self_collision
        if merge_mesh is not None:
            config.merge_mesh = merge_mesh
        if debug_mode is not None:
            config.debug_mode = debug_mode

        self.importer.config = config
        output_path = os.path.normpath(self.importer.import_urdf())
        omni.usd.get_context().open_stage(output_path)
        self._stage = omni.usd.get_context().get_stage()
        prim_path = f"/{os.path.splitext(os.path.basename(urdf_path))[0]}"
        return output_path, prim_path

    # basic urdf test: joints and links are imported correctly
    async def test_urdf_basic(self) -> None:
        """Import basic URDF and validate joints and links are imported correctly."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_basic.urdf"))
        )
        path, prim_path = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        prim = self._stage.GetPrimAtPath("/test_basic")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = self._stage.GetPrimAtPath("/test_basic/Physics/root_to_base")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        wristJoint = self._stage.GetPrimAtPath("/test_basic/Physics/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")

        fingerJoint = self._stage.GetPrimAtPath("/test_basic/Physics/finger_1_joint")
        self.assertNotEqual(fingerJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(fingerJoint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertAlmostEqual(fingerJoint.GetAttribute("physics:upperLimit").Get(), 0.08)

        fingerLink = self._stage.GetPrimAtPath(
            "/test_basic/Geometry/root_link/base_link/link_1/link_2/palm_link/finger_link_1"
        )
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:diagonalInertia").Get()[0], 1.0)
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:mass").Get(), 3)

        fingerLink3 = self._stage.GetPrimAtPath(
            "/test_basic/Geometry/root_link/base_link/link_1/link_2/palm_link/finger_link_3"
        )
        self.assertAlmostEqual(fingerLink3.GetAttribute("physics:diagonalInertia").Get()[0], 0.000999, delta=1e-2)
        print(
            fingerLink3.GetAttribute("physics:principalAxes").Get().GetReal(),
            fingerLink3.GetAttribute("physics:principalAxes").Get().GetImaginary(),
        )
        self.assertAlmostEqual(fingerLink3.GetAttribute("physics:principalAxes").Get().GetReal(), 0.88, delta=1e-2)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self._stage), 1.0)
        self._success = True

    async def test_urdf_massless(self) -> None:
        """Import URDF with massless links and validate physics properties."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_massless.urdf"))
        )
        path, prim_path = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        prim = self._stage.GetPrimAtPath("/test_massless")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        rootLink = self._stage.GetPrimAtPath("/test_massless/Geometry/root_link")
        self.assertEqual(rootLink.GetAttribute("physics:mass").Get(), None)

        no_mass_no_collision_no_inertia = self._stage.GetPrimAtPath(
            "/test_massless/Geometry/root_link/no_mass_no_collision_no_inertia"
        )
        self.assertEqual(no_mass_no_collision_no_inertia.GetAttribute("physics:diagonalInertia").Get(), None)
        self.assertEqual(no_mass_no_collision_no_inertia.GetAttribute("physics:mass").Get(), None)

        mass_no_collision_no_inertia = self._stage.GetPrimAtPath(
            "/test_massless/Geometry/root_link/no_mass_no_collision_no_inertia/mass_no_collision_no_inertia"
        )
        self.assertAlmostEqual(mass_no_collision_no_inertia.GetAttribute("physics:diagonalInertia").Get()[0], 0.0)
        self.assertAlmostEqual(mass_no_collision_no_inertia.GetAttribute("physics:mass").Get(), 10.0)

        mass_collision_no_inertia = self._stage.GetPrimAtPath(
            "/test_massless/Geometry/root_link/no_mass_no_collision_no_inertia/mass_no_collision_no_inertia/mass_collision_no_inertia"
        )
        self.assertAlmostEqual(mass_collision_no_inertia.GetAttribute("physics:diagonalInertia").Get()[0], 0.0)
        self.assertAlmostEqual(mass_collision_no_inertia.GetAttribute("physics:mass").Get(), 10.0)

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self._stage), 1.0)
        self._success = True

    async def test_urdf_save_to_file(self) -> None:
        """Import URDF and save to a specified file path."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_basic.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)
        await omni.kit.app.get_app().next_update_async()
        self._stage = pxr.Usd.Stage.Open(output_path)
        prim = self._stage.GetPrimAtPath("/test_basic")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = self._stage.GetPrimAtPath("/test_basic/Physics/root_to_base")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        wristJoint = self._stage.GetPrimAtPath("/test_basic/Physics/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")

        fingerJoint = self._stage.GetPrimAtPath("/test_basic/Physics/finger_1_joint")
        self.assertNotEqual(fingerJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(fingerJoint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertAlmostEqual(fingerJoint.GetAttribute("physics:upperLimit").Get(), 0.08)

        fingerLink = self._stage.GetPrimAtPath(
            "/test_basic/Geometry/root_link/base_link/link_1/link_2/palm_link/finger_link_1"
        )
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:diagonalInertia").Get()[0], 1.0)
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:mass").Get(), 3)

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self._stage), 1.0)
        self._success = True

    async def test_urdf_save_twice_to_file(self) -> None:
        """Import URDF twice to the same location and verify no conflicts."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_basic.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)
        await omni.kit.app.get_app().next_update_async()
        stats = os.stat(output_path)
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)
        stats_2 = os.stat(output_path)
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    async def test_urdf_textured_obj(self) -> None:
        """Import URDF with OBJ mesh textures and validate texture import."""
        urdf_path = os.path.normpath(
            os.path.join(self._extension_path, "data", "urdf", "tests", "test_textures_urdf", "cube_obj.urdf")
        )
        output_path, _ = self._import_urdf(urdf_path)
        await omni.kit.app.get_app().next_update_async()
        result = omni.client.list(os.path.normpath(os.path.join(os.path.dirname(output_path), "Textures")))
        self.assertEqual(result[0], omni.client.Result.OK)
        self.assertEqual(len(result[1]), 2)  # Metallic texture is unsuported by assimp on OBJ
        self._success = True

    async def test_urdf_textured_in_memory(self) -> None:
        """Import URDF with textures and validate in-memory processing."""
        base_path = os.path.normpath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_textures_urdf"))
        basename = "cube_obj"

        urdf_path = os.path.normpath(os.path.join(base_path, f"{basename}.urdf"))
        output_path, _ = self._import_urdf(urdf_path)
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    @unittest.skipIf(os.getenv("ETM_ACTIVE"), "Skipped in ETM: Unknown reason for ETM failing to load DAE.")
    async def test_urdf_textured_dae(self) -> None:
        """Import URDF with DAE mesh textures and validate texture import."""
        base_path = os.path.normpath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_textures_urdf"))
        basename = "cube_dae"

        urdf_path = os.path.normpath(os.path.join(base_path, f"{basename}.urdf"))
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)

        mats_path = os.path.normpath(os.path.join(os.path.dirname(output_path), "Textures"))

        await omni.kit.app.get_app().next_update_async()
        result = omni.client.list(mats_path)
        self.assertEqual(result[0], omni.client.Result.OK)
        self.assertEqual(len(result[1]), 1)  # only albedo is supported for Collada
        self._success = True

    async def test_urdf_overwrite_file(self) -> None:
        """Import URDF twice to overwrite existing file and validate results."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_basic.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)
        await omni.kit.app.get_app().next_update_async()
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)
        await omni.kit.app.get_app().next_update_async()

        self._stage = pxr.Usd.Stage.Open(output_path)
        prim = self._stage.GetPrimAtPath("/test_basic")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = self._stage.GetPrimAtPath("/test_basic/Physics/root_to_base")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        wristJoint = self._stage.GetPrimAtPath("/test_basic/Physics/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")

        fingerJoint = self._stage.GetPrimAtPath("/test_basic/Physics/finger_1_joint")
        self.assertNotEqual(fingerJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(fingerJoint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertAlmostEqual(fingerJoint.GetAttribute("physics:upperLimit").Get(), 0.08)

        fingerLink = self._stage.GetPrimAtPath(
            "/test_basic/Geometry/root_link/base_link/link_1/link_2/palm_link/finger_link_1"
        )
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:diagonalInertia").Get()[0], 1.0)
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:mass").Get(), 3)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self._stage), 1.0)
        self._success = True

    # advanced urdf test: test for all the categories of inputs that an urdf can hold
    async def test_urdf_advanced(self) -> None:
        """Import advanced URDF with various features and validate all categories."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_advanced.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        # check if object is there
        prim = self._stage.GetPrimAtPath("/test_advanced")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # check color are imported
        mesh = self._stage.GetPrimAtPath("/test_advanced/Geometry/root_link/base_link/box")
        self.assertNotEqual(mesh.GetPath(), Sdf.Path.emptyPath)
        mat, rel = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        self.assertTrue(Gf.IsClose(mat.GetInput("diffuseColor").Get(), Gf.Vec3f(0.0, 0.0, 0.60383), 1e-5))

        elbowPrim = self._stage.GetPrimAtPath("/test_advanced/Physics/elbow_joint")
        self.assertNotEqual(elbowPrim.GetPath(), Sdf.Path.emptyPath)
        self.assertAlmostEqual(elbowPrim.GetAttribute(PhysxAttr.JOINT_FRICTION.name).Get(), 0.1)
        self.assertAlmostEqual(elbowPrim.GetAttribute("drive:angular:physics:damping").Get(), 0.1)

        # check position of a link
        joint_pos = elbowPrim.GetAttribute("physics:localPos0").Get()
        self.assertTrue(Gf.IsClose(joint_pos, Gf.Vec3f(0, 0, 0.4), 1e-2))

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        self._success = True

    async def test_urdf_mtl(self) -> None:
        """Import URDF with MTL material files and validate material binding."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_mtl.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)
        self._stage = omni.usd.get_context().get_stage()

        mesh = self._stage.GetPrimAtPath("/test_mtl/Geometry/cube/test_mtl")
        self.assertTrue(UsdShade.MaterialBindingAPI(mesh) is not None)
        mat, rel = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        self.assertTrue(Gf.IsClose(mat.GetInput("diffuseColor").Get(), Gf.Vec3f(0.60383, 0.0, 0.0), 1e-5))
        self._success = True

    async def test_urdf_material(self) -> None:
        """Import URDF with material and validate material binding."""
        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_material.urdf")

        output_path, _ = self._import_urdf(urdf_path)
        stage = omni.usd.get_context().get_stage()

        mesh = stage.GetPrimAtPath("/test_material/Geometry/base/box")
        self.assertTrue(UsdShade.MaterialBindingAPI(mesh) is not None)
        mat, rel = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        self.assertTrue(Gf.IsClose(mat.GetInput("diffuseColor").Get(), Gf.Vec3f(1.0, 0.0, 0.0), 1e-5))
        self._success = True

    async def test_urdf_mtl_stl(self) -> None:
        """Import URDF with STL meshes and MTL materials and validate material binding."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_mtl_stl.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path, usd_path=self._tmpdir)
        self._stage = omni.usd.get_context().get_stage()

        mesh = self._stage.GetPrimAtPath("/test_mtl_stl/Geometry/cube/cube")
        self.assertTrue(UsdShade.MaterialBindingAPI(mesh) is not None)
        mat, rel = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        self.assertTrue(Gf.IsClose(mat.GetInput("diffuseColor").Get(), Gf.Vec3f(0.60383, 0.0, 0.0), 1e-5))
        self._success = True

    async def test_urdf_carter(self) -> None:
        """Import Carter robot URDF and validate basic structure."""
        urdf_path = os.path.normpath(
            os.path.abspath(
                os.path.join(self._extension_path, "data", "urdf", "robots", "carter", "urdf", "carter.urdf")
            )
        )
        output_path, prim_path = self._import_urdf(urdf_path)
        self.assertTrue(prim_path, "/carter")
        # TODO add checks here
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    async def test_urdf_parse_mimic(self) -> None:
        """Test urdf parse mimic."""
        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_mimic.urdf")
        _, prim_path = self._import_urdf(urdf_path)
        self.assertTrue(prim_path, "/test_mimic")

        stage = omni.usd.get_context().get_stage()

        # Verify source joint exists and has no mimic API
        source_joint = stage.GetPrimAtPath("/test_mimic/Physics/source_joint")
        self.assertNotEqual(source_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertFalse(source_joint.HasAPI("NewtonMimicAPI"))
        self.assertFalse(source_joint.HasAPI(PhysxSchema.MIMIC_JOINT_API))

        # Verify a_mimic_joint (lexicographically BEFORE source_joint) has NewtonMimicAPI configured.
        # The runtime consumes NewtonMimicAPI directly; the importer no longer authors PhysxMimicJointAPI.
        a_mimic_joint = stage.GetPrimAtPath("/test_mimic/Physics/a_mimic_joint")
        self.assertNotEqual(a_mimic_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertTrue(a_mimic_joint.HasAPI("NewtonMimicAPI"))
        self.assertFalse(a_mimic_joint.HasAPI(PhysxSchema.MIMIC_JOINT_API))

        self.assertAlmostEqual(a_mimic_joint.GetAttribute("newton:mimicCoef1").Get(), 1.5)
        self.assertAlmostEqual(a_mimic_joint.GetAttribute("newton:mimicCoef0").Get(), 0.1)
        ref_joint_targets = a_mimic_joint.GetRelationship("newton:mimicJoint").GetTargets()
        self.assertEqual(len(ref_joint_targets), 1)
        self.assertEqual(ref_joint_targets[0], source_joint.GetPath())

        # Verify z_mimic_joint (lexicographically AFTER source_joint) has NewtonMimicAPI configured.
        z_mimic_joint = stage.GetPrimAtPath("/test_mimic/Physics/z_mimic_joint")
        self.assertNotEqual(z_mimic_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertTrue(z_mimic_joint.HasAPI("NewtonMimicAPI"))
        self.assertFalse(z_mimic_joint.HasAPI(PhysxSchema.MIMIC_JOINT_API))

        self.assertAlmostEqual(z_mimic_joint.GetAttribute("newton:mimicCoef1").Get(), -1.0)
        self.assertAlmostEqual(z_mimic_joint.GetAttribute("newton:mimicCoef0").Get(), 0.0)
        ref_joint_targets = z_mimic_joint.GetRelationship("newton:mimicJoint").GetTargets()
        self.assertEqual(len(ref_joint_targets), 1)
        self.assertEqual(ref_joint_targets[0], source_joint.GetPath())
        self._success = True

    async def test_urdf_franka(self) -> None:
        """Import Franka robot URDF and validate mesh geometry."""
        urdf_path = os.path.normpath(
            os.path.abspath(
                os.path.join(
                    self._extension_path,
                    "data",
                    "urdf",
                    "robots",
                    "franka_description",
                    "robots",
                    "panda_arm_hand.urdf",
                )
            )
        )
        output_path, prim_path = self._import_urdf(urdf_path)
        await self.standard_checks(prim_path)
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    async def test_urdf_ur10(self) -> None:
        """Import UR10 robot URDF and validate mesh geometry."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "robots", "ur10", "urdf", "ur10.urdf"))
        )
        output_path, prim_path = self._import_urdf(urdf_path)
        await self.standard_checks(prim_path)
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    async def test_urdf_kaya(self) -> None:
        """Import Kaya robot URDF and validate mesh geometry."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "robots", "kaya", "urdf", "kaya.urdf"))
        )
        output_path, prim_path = self._import_urdf(urdf_path)
        await self.standard_checks(prim_path)
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    async def test_missing(self) -> None:
        """Import URDF with missing mesh files and validate error handling."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_missing.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    # This sample corresponds to the example in the docs, keep this and the version in the docs in sync
    async def test_doc_sample(self) -> None:
        """Test the documented example workflow for importing and configuring URDF."""
        from pxr import Gf, Sdf, UsdLux, UsdPhysics

        # Get path to extension data:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        extension_path = ext_manager.get_extension_path(ext_id)
        # import URDF
        output_path, _ = self._import_urdf(
            os.path.normpath(os.path.join(extension_path, "data", "urdf", "robots", "carter", "urdf", "carter.urdf"))
        )
        # get stage handle
        self._stage = omni.usd.get_context().get_stage()
        prim = self._stage.GetPrimAtPath("/carter")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")

        # enable physics
        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/physicsScene"))
        # set gravity
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        # add ground plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -50), Gf.Vec3f(0.5)
        )

        # add lighting
        distantLight = UsdLux.DistantLight.Define(self._stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        ####
        #### Next Docs section
        ####

        # get handle to the Drive API for both wheels
        left_wheel_drive = UsdPhysics.DriveAPI.Get(self._stage.GetPrimAtPath("/carter/Physics/left_wheel"), "angular")
        right_wheel_drive = UsdPhysics.DriveAPI.Get(self._stage.GetPrimAtPath("/carter/Physics/right_wheel"), "angular")

        # Set the velocity drive target in degrees/second
        left_wheel_drive.GetTargetVelocityAttr().Set(150)
        right_wheel_drive.GetTargetVelocityAttr().Set(150)

        # Set the drive damping, which controls the strength of the velocity drive
        left_wheel_drive.GetDampingAttr().Set(15000)
        right_wheel_drive.GetDampingAttr().Set(15000)

        # Set the drive stiffness, which controls the strength of the position drive
        # In this case because we want to do velocity control this should be set to zero
        left_wheel_drive.GetStiffnessAttr().Set(0)
        right_wheel_drive.GetStiffnessAttr().Set(0)
        self._success = True

    # Make sure that a urdf with more than 63 links imports
    async def test_64(self) -> None:
        """Import URDF with more than 63 links and validate large model support."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_large.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        prim = self._stage.GetPrimAtPath("/test_large")
        self.assertTrue(prim)
        await omni.kit.app.get_app().next_update_async()
        self._success = True

    # basic urdf test: joints and links are imported correctly
    async def test_urdf_floating(self) -> None:
        """Import URDF with floating base and validate link transforms."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_floating.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        prim = self._stage.GetPrimAtPath("/test_floating")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        await omni.kit.app.get_app().next_update_async()

        prim = self._stage.GetPrimAtPath("/test_floating")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = self._stage.GetPrimAtPath("/test_floating/Physics/root_to_base")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        link_1 = self._stage.GetPrimAtPath("/test_floating/Geometry/root_link/base_link/link_1")
        self.assertNotEqual(link_1.GetPath(), Sdf.Path.emptyPath)
        link_1_trans = np.array(omni.usd.get_world_transform_matrix(link_1).ExtractTranslation())

        self.assertAlmostEqual(np.linalg.norm(link_1_trans - np.array([0, 0, 0.45])), 0, delta=0.03)
        floating_link = self._stage.GetPrimAtPath("/test_floating/Geometry/root_link/base_link/link_1/floating_link")
        self.assertNotEqual(floating_link.GetPath(), Sdf.Path.emptyPath)
        floating_link_trans = np.array(omni.usd.get_world_transform_matrix(floating_link).ExtractTranslation())

        self.assertAlmostEqual(np.linalg.norm(floating_link_trans - np.array([0, 0, 1.450])), 0, delta=0.03)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        self._success = True

    async def test_urdf_scale(self) -> None:
        """Import URDF and validate stage units and scaling."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_basic.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        prim = self._stage.GetPrimAtPath("/test_basic")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        await omni.kit.app.get_app().next_update_async()

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self._stage), 1.0)
        self._success = True

    async def test_urdf_drive_none(self) -> None:
        """Import URDF and validate joint drive API presence for appropriate joints."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_basic.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()

        prim = self._stage.GetPrimAtPath("/test_basic")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        await omni.kit.app.get_app().next_update_async()

        self.assertFalse(self._stage.GetPrimAtPath("/test_basic/Physics/root_to_base").HasAPI(UsdPhysics.DriveAPI))
        self.assertTrue(self._stage.GetPrimAtPath("/test_basic/Physics/elbow_joint").HasAPI(UsdPhysics.DriveAPI))

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        self._success = True

    async def test_urdf_usd(self) -> None:
        """Import URDF referencing USD geometry and validate USD prim import."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_usd.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        self.assertNotEqual(self._stage.GetPrimAtPath("/test_usd/cube/visuals/mesh_0/Cylinder"), Sdf.Path.emptyPath)
        self.assertNotEqual(self._stage.GetPrimAtPath("/test_usd/cube/visuals/mesh_1/Torus"), Sdf.Path.emptyPath)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        self._success = True

    # test negative joint limits
    async def test_urdf_limits(self) -> None:
        """Import URDF with negative joint limits and validate limit configuration."""
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_limits.urdf"))
        )
        output_path, _ = self._import_urdf(urdf_path)
        self._stage = omni.usd.get_context().get_stage()
        prim = self._stage.GetPrimAtPath("/test_limits")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        await omni.kit.app.get_app().next_update_async()

        # ensure the import completed.
        prim = self._stage.GetPrimAtPath("/test_limits")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # ensure the joint limits are set on the elbow
        elbowJoint = self._stage.GetPrimAtPath("/test_limits/Physics/elbow_joint")
        self.assertNotEqual(elbowJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(elbowJoint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertTrue(elbowJoint.HasAPI(UsdPhysics.DriveAPI))

        # ensure the joint limits are set on the wrist
        wristJoint = self._stage.GetPrimAtPath("/test_limits/Physics/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertTrue(wristJoint.HasAPI(UsdPhysics.DriveAPI))

        # ensure the joint limits are set on the finger1
        finger1Joint = self._stage.GetPrimAtPath("/test_limits/Physics/finger_1_joint")
        self.assertNotEqual(finger1Joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(finger1Joint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertTrue(finger1Joint.HasAPI(UsdPhysics.DriveAPI))

        # ensure the joint limits are set on the finger2
        finger2Joint = self._stage.GetPrimAtPath("/test_limits/Physics/finger_2_joint")
        self.assertNotEqual(finger2Joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(finger2Joint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertTrue(finger2Joint.HasAPI(UsdPhysics.DriveAPI))

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        self._success = True

    # test collision from visuals
    async def test_collision_from_visuals(self) -> None:
        """Import URDF with collision from visuals enabled and validate collision geometry."""
        # import a urdf file without collision
        urdf_path = os.path.normpath(
            os.path.abspath(
                os.path.join(self._extension_path, "data", "urdf", "tests", "test_collision_from_visuals.urdf")
            )
        )
        USD_GEOMETRY_TYPES = {"Mesh", "Cube", "Sphere", "Capsule", "Cylinder", "Cone"}

        output_path, _ = self._import_urdf(urdf_path, collision_from_visuals=True)
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        # ensure the import completed.
        prim = self._stage.GetPrimAtPath("/test_collision_from_visuals")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        for child_prim in self._stage.Traverse():
            prim_type = child_prim.GetTypeName()
            if prim_type in USD_GEOMETRY_TYPES:
                imageable = UsdGeom.Imageable(child_prim)
                purpose = imageable.GetPurposeAttr().Get() or "default"
                if purpose not in {"default", "render"}:
                    continue

                self.assertTrue(child_prim.HasAPI(UsdPhysics.CollisionAPI))
                collision_api = UsdPhysics.CollisionAPI(child_prim)
                collision_enabled_attr = collision_api.GetCollisionEnabledAttr()
                self.assertTrue(collision_enabled_attr.Get())

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # nothing crashes
        self._timeline.stop()
        self._success = True

    async def test_debug_mode(self) -> None:
        """Import with debug mode enabled and validate results.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf import URDFImporterConfig
            >>> URDFImporterConfig()
            <...>
        """
        urdf_path = os.path.normpath(os.path.join(self._extension_path, "data", "urdf", "tests", "test_basic.urdf"))
        output_path, _ = self._import_urdf(urdf_path, debug_mode=True)

        debug_dir = os.path.normpath(os.path.join(os.path.dirname(output_path), "..", "_debug_test_basic"))
        temp_usd_path = os.path.join(debug_dir, "temp_test_basic", "test_basic.usd")
        usdex_usd_path = os.path.join(debug_dir, "usdex_test_basic", "test_basic.usdc")

        self.assertTrue(os.path.isdir(debug_dir), f"Debug scratch directory not found: {debug_dir}")
        self.assertTrue(os.path.exists(temp_usd_path), f"Temp USD file not found: {temp_usd_path}")
        self.assertTrue(os.path.exists(usdex_usd_path), f"USDEx USD file not found: {usdex_usd_path}")
        self.assertTrue(os.path.exists(output_path), f"Output path not found: {output_path}")
        self._success = True
