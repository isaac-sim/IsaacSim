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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
# from isaacsim.robot.surface_gripper._surface_gripper import Surface_Gripper, Surface_Gripper_Properties

"""Tests for surface gripper functionality and the GripperView interface in Isaac Sim."""

import time

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.robot.surface_gripper import GripperView, create_surface_gripper
from isaacsim.robot.surface_gripper.bindings._surface_gripper import GripperStatus
from omni.physx.scripts.physicsUtils import add_ground_plane
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics
from usd.schema.isaac import robot_schema


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSurfaceGripperView(omni.kit.test.AsyncTestCase):
    """Test suite for surface gripper functionality and the GripperView interface.

    This test class validates the behavior of surface grippers in Isaac Sim, including initialization,
    status management, property configuration, and action application. It tests both single and multiple
    gripper scenarios with various joint configurations.

    The test suite covers:
        - Surface gripper initialization and setup
        - Gripper status retrieval and validation
        - Property setting and getting (max grip distance, force limits, retry intervals)
        - Action application and performance testing
        - Multi-joint gripper configurations

    Each test creates a physics scene with rigid body cubes and surface grippers, then validates
    the expected behavior through the GripperView interface.
    """

    async def createRigidCube(
        self,
        boxActorPath: str,
        mass: float,
        scale: list[float],
        position: list[float],
        rotation: list[float],
        color: list[float],
    ):
        """Creates a rigid cube actor in the USD stage with physics properties.

        Args:
            boxActorPath: USD path where the cube will be created.
            mass: Mass of the cube in kilograms. If 0 or negative, the cube becomes kinematic.
            scale: Scale factors for the cube dimensions [x, y, z].
            position: World position of the cube [x, y, z].
            rotation: Quaternion rotation of the cube [x, y, z, w].
            color: RGB color values [r, g, b] in the range [0, 255].
        """
        p = Gf.Vec3f(position[0], position[1], position[2])
        orientation = Gf.Quatf(rotation[3], rotation[0], rotation[1], rotation[2])
        color = Gf.Vec3f(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)
        size = 1.0
        scale = Gf.Vec3f(scale[0], scale[1], scale[2])

        cubeGeom = UsdGeom.Cube.Define(self.stage, boxActorPath)
        cubePrim = self.stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(p)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])
        # await omni.kit.app.get_app().next_update_async()
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)

        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        if mass > 0:
            massAPI.CreateMassAttr(mass)
        else:
            rigid_api.CreateKinematicEnabledAttr(True)

        # await omni.kit.app.get_app().next_update_async()
        UsdPhysics.CollisionAPI(cubePrim)

    # Helper for setting up the physics stage
    async def setup_physics(self):
        """Sets up the physics environment for testing.

        Creates a physics scene with lighting, sets stage units and up-axis, and adds a ground plane.
        """
        # Set Up Physics scene
        distantLight = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        self._scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/physicsScene"))
        add_ground_plane(self.stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, 0), Gf.Vec3f(1.0))

    # Helper for setting up the surface gripper
    async def setup_gripper(self, count: int, num_joints: int = 1):
        """Sets up multiple surface gripper test environments.

        Args:
            count: Number of gripper environments to create.
            num_joints: Number of attachment joints per gripper.
        """
        for i in range(count):
            env_path = "/env" + str(i)
            env_prim = UsdGeom.Xform.Define(self.stage, env_path)
            env_prim.AddTranslateOp().Set(Gf.Vec3f(i * 0.2, 0.0, 0))
            box0_path = env_path + "/box0"
            box1_path = env_path + "/box1"
            box0_props = [box0_path, 1.0, [0.1, 0.1, 0.1], [0, 0, 0.05], [0, 0, 0, 1], [80, 80, 255]]
            box1_props = [box1_path, 1.0, [0.1, 0.1, 0.1], [0, 0, 0.15], [0, 0, 0, 1], [255, 80, 80]]
            spacing = 0.1 / (num_joints + 1)
            start_y = -0.05 + spacing
            create_surface_gripper(self.stage, env_path)
            gripper_path = env_path + "/SurfaceGripper"
            gripper_prim = self.stage.GetPrimAtPath(gripper_path)
            gripper_prim.GetAttribute(robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name).Set(500000)
            gripper_prim.GetAttribute(robot_schema.Attributes.SHEAR_FORCE_LIMIT.name).Set(500000)
            gripper_prim.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(0.01)
            attachment_points_rel = gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name)

            djoint_paths = []

            await self.createRigidCube(*box0_props)
            await self.createRigidCube(*box1_props)

            for j in range(num_joints):
                d6Joint_path = Sdf.Path(box1_path + "/d6Joint" + str(j))
                djoint_paths.append(d6Joint_path)
                joint_prim = UsdPhysics.Joint.Define(self.stage, d6Joint_path)

                robot_schema.ApplyAttachmentPointAPI(joint_prim.GetPrim())
                joint_prim.GetPrim().GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name).Set(UsdPhysics.Tokens.x)

                for limit in ["rotX", "rotY", "rotZ", "transX", "transY", "transZ"]:

                    lim_api = UsdPhysics.LimitAPI.Apply(joint_prim.GetPrim(), limit)
                    lim_api.CreateHighAttr().Set(-1)
                    lim_api.CreateLowAttr().Set(1)

                # joint_prim.CreateDriveTypeAttr().Set(UsdPhysics.Tokens.d6)
                joint_prim.CreateBody0Rel().SetTargets([box1_path])
                # joint_prim.CreateBody1Rel().SetTargets([box0_path])

                joint_prim.CreateLocalPos0Attr().Set(Gf.Vec3f(0, start_y + j * spacing, -0.499))
                joint_prim.CreateLocalRot0Attr().Set(Gf.Quatf(0.5, -0.5, 0.5, 0.5))
                joint_prim.CreateLocalPos1Attr().Set(Gf.Vec3f(i * 0.2, start_y + j * spacing, 0.099))
                joint_prim.CreateLocalRot1Attr().Set(Gf.Quatf(0.5, -0.5, 0.5, 0.5))

            attachment_points_rel.SetTargets(djoint_paths)

        self.gripper_view = GripperView(
            paths="/env.*/SurfaceGripper",
        )

    # Before running each test
    async def setUp(self):
        """Sets up the test environment before each test.

        Creates a new USD stage and initializes the timeline interface.
        """
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        pass

    # After running each test
    async def tearDown(self):
        """Cleans up after each test."""
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_initialize_surface_gripper(self):
        """Tests basic surface gripper initialization and simulation setup."""

        await self.setup_physics()
        await self.setup_gripper(2)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        pass

    async def test_get_surface_gripper_status(self):
        """Tests getting and setting surface gripper status values.

        Verifies that gripper status can be retrieved correctly after applying actions,
        both with and without simulation steps, and with selective gripper updates.
        """
        gripper_count = 2
        await self.setup_physics()
        await self.setup_gripper(2)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()

        # set status of the grippers and make sure we can retrieve it after a step
        status_exp = [GripperStatus.Open, GripperStatus.Open]
        status_values = [-0.5, -0.5]
        self.gripper_view.apply_gripper_action(status_values)

        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()

        status = self.gripper_view.get_surface_gripper_status()
        for i in range(gripper_count):
            self.assertEqual(GripperStatus(status[i]), status_exp[i])

        # set status of the grippers and make sure we can retrieve them without a step
        status_exp = [GripperStatus.Closed, GripperStatus.Closed]
        status_values = [0.5, 0.5]
        self.gripper_view.apply_gripper_action(status_values)

        status = self.gripper_view.get_surface_gripper_status()
        for i in range(gripper_count):
            self.assertEqual(GripperStatus(status[i]), status_exp[i])

        # set status of only some grippers
        status_new_exp = [GripperStatus.Closed, GripperStatus.Open]
        status_values = [0.0, -0.5]
        changed_gripper_indices = [1]
        self.gripper_view.apply_gripper_action(status_values, changed_gripper_indices)

        status = self.gripper_view.get_surface_gripper_status()
        self.assertEqual(GripperStatus(status[0]), status_new_exp[0])
        self.assertEqual(GripperStatus(status[1]), status_new_exp[1])

        pass

    async def test_surface_gripper_properties(self):
        """Tests getting and setting surface gripper properties.

        Verifies that gripper properties (max grip distance, force limits, retry interval)
        can be set and retrieved correctly, including partial property updates and
        selective gripper updates.
        """
        gripper_count = 2
        await self.setup_physics()
        await self.setup_gripper(gripper_count)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()

        # set all properties of the grippers and make sure we can retrieve them after a step
        max_grip_distance_exp = [1.0, 1.0]
        coaxial_force_limit_exp = [2.0, 2.0]
        shear_force_limit_exp = [0.3, 0.5]
        retry_interval_exp = [0.1, 0.2]
        self.gripper_view.set_surface_gripper_properties(
            max_grip_distance_exp, coaxial_force_limit_exp, shear_force_limit_exp, retry_interval_exp
        )

        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()

        max_grip_distance, coaxial_force_limit, shear_force_limit, retry_interval = (
            self.gripper_view.get_surface_gripper_properties()
        )
        for i in range(gripper_count):
            self.assertAlmostEqual(max_grip_distance[i], max_grip_distance_exp[i])
            self.assertAlmostEqual(coaxial_force_limit[i], coaxial_force_limit_exp[i])
            self.assertAlmostEqual(shear_force_limit[i], shear_force_limit_exp[i])
            self.assertAlmostEqual(retry_interval[i], retry_interval_exp[i])

        # set all properties of the grippers and make sure we can retrieve them without a step
        max_grip_distance_exp = [2.0, 2.0]
        coaxial_force_limit_exp = [1.0, 1.0]
        shear_force_limit_exp = [0.03, 0.1]
        retry_interval_exp = [0.7, 0.8]
        self.gripper_view.set_surface_gripper_properties(
            max_grip_distance_exp, coaxial_force_limit_exp, shear_force_limit_exp, retry_interval_exp
        )

        max_grip_distance, coaxial_force_limit, shear_force_limit, retry_interval = (
            self.gripper_view.get_surface_gripper_properties()
        )
        for i in range(gripper_count):
            self.assertAlmostEqual(max_grip_distance[i], max_grip_distance_exp[i])
            self.assertAlmostEqual(coaxial_force_limit[i], coaxial_force_limit_exp[i])
            self.assertAlmostEqual(shear_force_limit[i], shear_force_limit_exp[i])
            self.assertAlmostEqual(retry_interval[i], retry_interval_exp[i])

        # set only some properties of the grippers
        max_grip_distance_exp = [0.0, 0.8]
        shear_force_limit_exp = [0.2, 0.5]
        self.gripper_view.set_surface_gripper_properties(
            max_grip_distance=max_grip_distance_exp, shear_force_limit=shear_force_limit_exp
        )

        max_grip_distance, coaxial_force_limit, shear_force_limit, retry_interval = (
            self.gripper_view.get_surface_gripper_properties()
        )
        for i in range(gripper_count):
            self.assertAlmostEqual(max_grip_distance[i], max_grip_distance_exp[i])
            self.assertAlmostEqual(coaxial_force_limit[i], coaxial_force_limit_exp[i])
            self.assertAlmostEqual(shear_force_limit[i], shear_force_limit_exp[i])
            self.assertAlmostEqual(retry_interval[i], retry_interval_exp[i])

        # set properties of only some grippers
        max_grip_distance_new_exp = [0.0, 0.52]
        coaxial_force_limit_new_exp = [0.0, 0.23]
        shear_force_limit_new_exp = [0.0, 0.07]
        retry_interval_new_exp = [0.0, 0.3]
        changed_gripper_indices = [1]
        self.gripper_view.set_surface_gripper_properties(
            max_grip_distance_new_exp,
            coaxial_force_limit_new_exp,
            shear_force_limit_new_exp,
            retry_interval_new_exp,
            changed_gripper_indices,
        )

        max_grip_distance, coaxial_force_limit, shear_force_limit, retry_interval = (
            self.gripper_view.get_surface_gripper_properties()
        )
        self.assertAlmostEqual(max_grip_distance[0], max_grip_distance_exp[0])
        self.assertAlmostEqual(coaxial_force_limit[0], coaxial_force_limit_exp[0])
        self.assertAlmostEqual(shear_force_limit[0], shear_force_limit_exp[0])
        self.assertAlmostEqual(retry_interval[0], retry_interval_exp[0])
        self.assertAlmostEqual(max_grip_distance[1], max_grip_distance_new_exp[1])
        self.assertAlmostEqual(coaxial_force_limit[1], coaxial_force_limit_new_exp[1])
        self.assertAlmostEqual(shear_force_limit[1], shear_force_limit_new_exp[1])
        self.assertAlmostEqual(retry_interval[1], retry_interval_new_exp[1])

        pass

    async def test_surface_gripper_apply_action(self, gripper_count: int = 9 * 100, num_joints: int = 1):
        """Tests performance of applying gripper actions to multiple grippers.

        Args:
            gripper_count: Number of grippers to test with.
            num_joints: Number of joints per gripper.
        """
        await self.setup_physics()
        await self.setup_gripper(gripper_count, num_joints)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        start_time = time.time()
        self.gripper_view.apply_gripper_action([0.5] * gripper_count)
        elapsed_time = time.time()
        await omni.kit.app.get_app().next_update_async()
        new_elapsed_time = time.time()
        await omni.kit.app.get_app().next_update_async()
        post_elapsed_time = time.time()
        status = self.gripper_view.get_surface_gripper_status()
        print(f"apply_gripper_action elapsed time: {elapsed_time- start_time} seconds")
        print(f"simulate time: {new_elapsed_time- elapsed_time} seconds")
        print(f"post simulate time: {post_elapsed_time- new_elapsed_time} seconds")
        self.assertTrue((np.array(status) == int(GripperStatus.Closed)).all())
        # self.assertTrue((status == GripperStatus.Closed).all())
        pass

    async def test_surface_gripper_apply_action_multi_joints(self):
        """Tests gripper action application with multiple joints per gripper.

        Runs the gripper action test with 100 grippers having 9 joints each.
        """
        await self.test_surface_gripper_apply_action(100, 9)
