# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the deprecated CreateConveyorBelt command."""

import omni.kit.commands
import omni.kit.test
from pxr import Gf, UsdGeom, UsdPhysics


class TestCreateConveyorBeltCommand(omni.kit.test.AsyncTestCase):
    """Test the deprecated CreateConveyorBelt omni.kit.commands command."""

    async def setUp(self) -> None:
        """Set up the test environment."""
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        scene = UsdPhysics.Scene.Define(self._stage, "/physics")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up after each test.

        Drain a few app updates so any PhysX UJITSO mesh cook queued by
        ``MeshCollisionAPI`` resolves against the *current* stage. Without
        this, cooks queued from one test surface during the next test's
        ``setUp`` (which has already replaced the stage), producing
        ``PhysX could not find USD stage`` errors that the kit-test
        ``[error]`` pattern matcher then promotes to a test failure.
        """
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

    async def test_create_conveyor_belt_command(self) -> None:
        """Test creating a conveyor belt via the deprecated command."""
        cubeGeom = UsdGeom.Cube.Define(self._stage, "/cube")
        cube_prim = self._stage.GetPrimAtPath("/cube")
        cubeGeom.CreateSizeAttr(1.0)
        cubeGeom.AddTranslateOp().Set((0, 0, 0))
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        UsdPhysics.CollisionAPI.Apply(cube_prim)

        success, og_prim = omni.kit.commands.execute("CreateConveyorBelt", conveyor_prim=cube_prim)
        self.assertTrue(success)
        self.assertIsNotNone(og_prim)
        self.assertTrue(og_prim.IsValid())

    async def test_create_conveyor_belt_command_without_physics(self) -> None:
        """Test creating a conveyor belt via the deprecated command on a prim without RigidBodyAPI."""
        cubeGeom = UsdGeom.Cube.Define(self._stage, "/cube_no_physics")
        cube_prim = self._stage.GetPrimAtPath("/cube_no_physics")
        cubeGeom.CreateSizeAttr(1.0)
        cubeGeom.AddTranslateOp().Set((0, 0, 0))

        success, og_prim = omni.kit.commands.execute("CreateConveyorBelt", conveyor_prim=cube_prim)
        self.assertTrue(success)
        self.assertIsNotNone(og_prim)
        self.assertTrue(og_prim.IsValid())

        # Cube collisions resolve to PhysX box shapes, so MeshCollisionAPI is
        # neither necessary nor schema-valid (the API's ``appliesTo`` is
        # UsdGeomMesh). Applying it would author a spec PhysX ignores.
        self.assertTrue(cube_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(cube_prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertFalse(cube_prim.HasAPI(UsdPhysics.MeshCollisionAPI))

    async def test_create_conveyor_belt_command_on_mesh_sets_convex_hull(self) -> None:
        """Regression: a UsdGeomMesh without RigidBodyAPI must receive
        ``MeshCollisionAPI`` with the ``convexHull`` approximation so PhysX does
        not reject the triangle-mesh collision on the resulting dynamic body.

        This is the actual scenario that emitted the per-parse PhysX error
        message ``triangle mesh collision (approximation None/MeshSimplification)
        cannot be a part of a dynamic body``.
        """
        mesh = UsdGeom.Mesh.Define(self._stage, "/mesh_no_physics")
        mesh_prim = mesh.GetPrim()
        # A tetrahedron (4 non-coplanar points) is the smallest mesh that PhysX
        # can cook to a non-degenerate convex hull. A single triangle would
        # leave PhysX's hull triangulator empty and emit
        # ``triangulator is false`` / ``stream.getSize() > 0 is false`` errors
        # asynchronously, which the kit-test pattern matcher promotes to a
        # test failure even though the test itself only inspects schemas.
        mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)])
        mesh.CreateFaceVertexCountsAttr([3, 3, 3, 3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2])
        mesh.AddTranslateOp().Set((0, 0, 0))

        success, og_prim = omni.kit.commands.execute("CreateConveyorBelt", conveyor_prim=mesh_prim)
        self.assertTrue(success)
        self.assertIsNotNone(og_prim)
        self.assertTrue(og_prim.IsValid())

        self.assertTrue(mesh_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(mesh_prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI))
        approximation = UsdPhysics.MeshCollisionAPI(mesh_prim).GetApproximationAttr().Get()
        self.assertEqual(approximation, "convexHull")
