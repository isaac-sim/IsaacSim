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

"""Verifies the GripperView interface for initializing surface grippers, reading status and properties, and applying single- and multi-joint gripper actions."""

import faulthandler
import os
import threading
import time
from collections.abc import Callable, Iterator
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from typing import Any

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import warp as wp
from isaacsim.robot.surface_gripper import GripperView, create_surface_gripper
from isaacsim.robot.surface_gripper.bindings._surface_gripper import GripperStatus
from omni.physx.scripts.physicsUtils import add_ground_plane
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics
from usd.schema.isaac import robot_schema


@contextmanager
def _fail_on_native_stall() -> Iterator[None]:
    """Fail the Kit process if a native call or executor cleanup stops making progress.

    Yields:
        Control to the test with a wall-clock watchdog covering native calls and teardown.
    """
    # A Python future timeout cannot cancel native work. Keep the watchdog armed through
    # executor shutdown so a stalled worker cannot hang the entire extension test process.
    faulthandler.dump_traceback_later(60, exit=True)
    try:
        yield
    finally:
        faulthandler.cancel_dump_traceback_later()


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

    async def create_rigid_cube(
        self,
        boxActorPath: str,
        mass: float,
        scale: list[float],
        position: list[float],
        rotation: list[float],
        color: list[float],
    ) -> None:
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
    async def setup_physics(self) -> Any:
        """Sets up the physics environment for testing.

        Creates a physics scene with lighting, sets stage units and up-axis, and adds a ground plane.

        Returns:
            None.
        """
        # Set Up Physics scene
        distantLight = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        self._scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/physicsScene"))
        add_ground_plane(self.stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, 0), Gf.Vec3f(1.0))

    # Helper for setting up the surface gripper
    async def setup_gripper(self, count: int, num_joints: int = 1) -> Any:
        """Sets up multiple surface gripper test environments.

        Args:
            count: Number of gripper environments to create.
            num_joints: Number of attachment joints per gripper.

        Returns:
            None.
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

            await self.create_rigid_cube(*box0_props)
            await self.create_rigid_cube(*box1_props)

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
    async def setUp(self) -> None:
        """Sets up the test environment before each test.

        Creates a new USD stage and initializes the timeline interface.
        """
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

    # After running each test
    async def tearDown(self) -> None:
        """Cleans up after each test."""
        await omni.kit.app.get_app().next_update_async()

    async def test_initialize_surface_gripper(self) -> None:
        """Tests basic surface gripper initialization and simulation setup."""
        await self.setup_physics()
        await self.setup_gripper(2)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()

    async def test_get_surface_gripper_status(self) -> None:
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

    async def test_surface_gripper_properties(self) -> None:
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

    async def test_surface_gripper_apply_action(self, gripper_count: int = 9 * 100, num_joints: int = 1) -> None:
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

    async def test_surface_gripper_apply_action_multi_joints(self) -> None:
        """Tests gripper action application with multiple joints per gripper.

        Runs the gripper action test with 100 grippers having 9 joints each.
        """
        await self.test_surface_gripper_apply_action(100, 9)

    async def test_property_array_types_and_atomic_validation(self) -> None:
        """Supported arrays round-trip; malformed batches do not partially author USD."""
        for path in ("/Gripper0", "/Gripper1"):
            robot_schema.CreateSurfaceGripper(self.stage, path)
        properties = ("max_grip_distance", "coaxial_force_limit", "shear_force_limit", "retry_interval")
        factories = [list, lambda x: np.array(x, dtype=np.float32), lambda x: np.array(x, dtype=np.float64)]
        for device in wp.get_devices():
            factories.append(lambda x, device=device: wp.array(x, dtype=wp.float32, device=device))
        for factory in factories:
            values = factory([0.125, 0.25])
            view = GripperView("/Gripper.*", **dict.fromkeys(properties, values))
            for result in view.get_surface_gripper_properties():
                np.testing.assert_allclose(result, [0.125, 0.25])
            view.set_surface_gripper_properties(**{name: factory([0.5, 0.75]) for name in properties}, indices=[1])
            for result in view.get_surface_gripper_properties():
                np.testing.assert_allclose(result, [0.125, 0.75])
            before = view.get_surface_gripper_properties()
            with self.assertRaises(ValueError):
                view.set_surface_gripper_properties(max_grip_distance=[1.0, 2.0], retry_interval=[1.0])
            self.assertEqual(before, view.get_surface_gripper_properties())
            with self.assertRaises(ValueError):
                view.set_surface_gripper_properties(max_grip_distance=[1.0, 2.0], indices=[0, 2])
            self.assertEqual(before, view.get_surface_gripper_properties())

    async def test_reject_non_grippers_before_authoring(self) -> None:
        """Wrong or changed schemas fail with the offending path and no partial writes."""
        real = robot_schema.CreateSurfaceGripper(self.stage, "/Gripper0")
        fake = UsdGeom.Xform.Define(self.stage, "/Gripper1").GetPrim()
        before = self.stage.GetRootLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, "/Gripper1"):
            GripperView("/Gripper.*", translations=[[1, 2, 3], [4, 5, 6]])
        self.assertEqual(before, self.stage.GetRootLayer().ExportToString())
        self.stage.RemovePrim(fake.GetPath())
        robot_schema.CreateSurfaceGripper(self.stage, "/Gripper1")
        view = GripperView("/Gripper.*")
        old_value = real.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Get()
        self.stage.GetPrimAtPath("/Gripper1").SetTypeName("Xform")
        with self.assertRaisesRegex(ValueError, "/Gripper1"):
            view.set_surface_gripper_properties(max_grip_distance=[1.0, 2.0])
        self.assertEqual(old_value, real.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Get())
        with self.assertRaisesRegex(ValueError, "/Gripper1"):
            view.apply_gripper_action([1.0, 1.0])
        self.stage.RemovePrim("/Gripper1")
        with self.assertRaisesRegex(ValueError, "/Gripper1"):
            view.get_surface_gripper_properties()

    async def test_removed_attachment_joint(self) -> None:
        """A joint removed after initialization must not leave a usable cached pointer."""
        await self.setup_physics()
        await self.setup_gripper(1)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        self.stage.RemovePrim("/env0/box1/d6Joint0")
        for _ in range(4):
            await omni.kit.app.get_app().next_update_async()
        self.gripper_view.apply_gripper_action([1.0])
        for _ in range(4):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self.gripper_view.get_gripped_objects(), [[]])
        self.gripper_view.apply_gripper_action([-1.0])
        self._timeline.stop()

    async def test_deleted_active_joint_without_force_limits(self) -> None:
        """Deleting an active joint opens the gripper even with both force limits disabled."""
        await self.setup_physics()
        await self.setup_gripper(2)
        self.gripper_view.set_surface_gripper_properties(
            coaxial_force_limit=[0.0, 500000.0], shear_force_limit=[0.0, 500000.0], retry_interval=[0.0, 0.0]
        )
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        self.gripper_view.apply_gripper_action([1.0, 1.0])
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(
            [GripperStatus(value) for value in self.gripper_view.get_surface_gripper_status()],
            [GripperStatus.Closed] * 2,
        )
        self.assertTrue(all(self.gripper_view.get_gripped_objects()))
        for i in range(2):
            self.stage.RemovePrim(f"/env{i}/box1/d6Joint0")
        for _ in range(4):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(
            [GripperStatus(value) for value in self.gripper_view.get_surface_gripper_status()], [GripperStatus.Open] * 2
        )
        self.assertEqual(self.gripper_view.get_gripped_objects(), [[], []])
        self._timeline.stop()

    async def test_unique_and_mixed_batch_paths(self) -> None:
        """Parallel batches retain all success flags and per-path order for mixed duplicate requests."""
        await self.setup_physics()
        count = (os.cpu_count() or 1) + 1
        await self.setup_gripper(count)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        paths = [f"/env{i}/SurfaceGripper" for i in range(count)]
        interface = self.gripper_view.surface_gripper_interface
        self.assertEqual(list(interface.close_gripper_batch(paths)), [True] * count)
        self.assertEqual(
            [GripperStatus(value) for value in interface.get_gripper_status_batch(paths)],
            [GripperStatus.Closed] * count,
        )
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()
        self.assertTrue(all(interface.get_gripped_objects_batch(paths)))
        self.assertEqual(list(interface.open_gripper_batch(paths)), [True] * count)
        self.assertEqual(
            [GripperStatus(value) for value in interface.get_gripper_status_batch(paths)], [GripperStatus.Open] * count
        )
        # Equal strings are separately allocated, so grouping must compare contents, not pointers.
        mixed_paths = paths + [f"/env{i}/SurfaceGripper" for i in range(count)]
        actions = [1.0] * count + [-1.0] * count
        self.assertEqual(list(interface.set_gripper_action_batch(mixed_paths, actions)), [True] * (2 * count))
        expected = [GripperStatus.Open] * count
        self.assertEqual([GripperStatus(value) for value in interface.get_gripper_status_batch(paths)], expected)
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()
        objects = interface.get_gripped_objects_batch(paths)
        self.assertEqual(list(objects), [[] for _ in paths])
        # Neutral commands report false; alternating results exercise independent byte storage.
        actions = [-1.0 if i % 2 else 0.0 for i in range(count)]
        self.assertEqual(list(interface.set_gripper_action_batch(paths, actions)), [bool(i % 2) for i in range(count)])
        self._timeline.stop()

    async def test_duplicate_batch_actions_preserve_order(self) -> None:
        """A batch processes every duplicate occurrence and returns every success flag."""
        await self.setup_physics()
        await self.setup_gripper(1)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        # Exceeds the former worker threshold even on high-core-count hosts.
        count = 2 * (os.cpu_count() or 1)
        paths = ["/env0/SurfaceGripper"] * count
        values = [1.0, -1.0] * (count // 2)
        interface = self.gripper_view.surface_gripper_interface
        results = interface.set_gripper_action_batch(paths, values)
        self.assertEqual(list(results), [True] * count)
        self.assertEqual(interface.get_gripper_status(paths[-1]), GripperStatus.Open)
        self._timeline.stop()

    async def test_async_main_thread_gripper_calls(self) -> None:
        """Main-thread reads and mutations complete while physics runs asynchronously."""
        with _fail_on_native_stall():
            import omni.physx

            await self.setup_physics()
            await self.setup_gripper(1)
            PhysxSchema.PhysxSceneAPI.Apply(self._scene.GetPrim()).CreateUpdateTypeAttr().Set(
                PhysxSchema.Tokens.Asynchronous
            )
            interface = self.gripper_view.surface_gripper_interface
            main_thread = threading.get_ident()
            callback_threads: set[int] = set()

            def query_after_step(dt: float) -> None:
                """Exercise reentrant reads on the physics callback thread.

                Args:
                    dt: Physics timestep.
                """
                interface.get_gripper_status("/env0/SurfaceGripper")
                interface.get_gripped_objects("/env0/SurfaceGripper")
                callback_threads.add(threading.get_ident())

            subscription = None
            self._timeline.play()
            try:
                for _ in range(12):
                    await omni.kit.app.get_app().next_update_async()
                subscription = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
                    query_after_step, False, 1
                )
                for write_to_usd in (False, True):
                    interface.set_write_to_usd(write_to_usd)
                    for _ in range(12):
                        await omni.kit.app.get_app().next_update_async()
                        self.assertTrue(interface.close_gripper("/env0/SurfaceGripper"))
                        self.assertIsInstance(interface.get_gripped_objects("/env0/SurfaceGripper"), list)
                        self.assertTrue(interface.open_gripper("/env0/SurfaceGripper"))
                        self.assertEqual(interface.get_gripper_status("/env0/SurfaceGripper"), GripperStatus.Open)
                        self.assertEqual(interface.get_gripped_objects("/env0/SurfaceGripper"), [])
                        self.assertEqual(
                            list(interface.set_gripper_action_batch(["/env0/SurfaceGripper"] * 2, [1.0, -1.0])),
                            [True, True],
                        )
                self.assertTrue(
                    callback_threads - {main_thread}, "No off-main physics callback completed its API reads"
                )
            finally:
                self._timeline.stop()
                interface.set_write_to_usd(False)
                if subscription is not None:
                    subscription.unsubscribe()

    async def test_duplicate_batch_actions_with_asynchronous_physics(self) -> None:
        """Exercise the original duplicate-path workload on worker-thread physics steps."""
        with _fail_on_native_stall():
            import omni.physx

            await self.setup_physics()
            await self.setup_gripper(1)
            update_type = PhysxSchema.PhysxSceneAPI.Apply(self._scene.GetPrim()).CreateUpdateTypeAttr()
            update_type.Set(PhysxSchema.Tokens.Asynchronous)
            self.assertEqual(update_type.Get(), PhysxSchema.Tokens.Asynchronous)

            main_thread = threading.get_ident()
            step_threads: set[int] = set()

            def record_step(dt: float) -> None:
                """Record which OS thread runs a physics step.

                Args:
                    dt: Physics timestep.
                """
                step_threads.add(threading.get_ident())

            physics = omni.physx.get_physx_interface()
            step_subscription = None
            interface = self.gripper_view.surface_gripper_interface
            # This must exceed the former `parallelForIndex` serial threshold on this host.
            duplicate_count = 2 * ((os.cpu_count() or 1) // 2 + 1)
            paths = ["/env0/SurfaceGripper"] * duplicate_count
            close_then_open = [1.0, -1.0] * (duplicate_count // 2)
            try:
                self._timeline.play()
                for _ in range(12):
                    await omni.kit.app.get_app().next_update_async()
                step_subscription = physics.subscribe_physics_on_step_events(record_step, True, 0)
                for _ in range(8):
                    await omni.kit.app.get_app().next_update_async()
                self.assertTrue(
                    step_threads - {main_thread}, "Asynchronous physics never ran a step off the main thread"
                )

                async def call_while_pumping(fn: Callable[..., Any], *args: Any) -> Any:
                    result = pool.submit(fn, *args)
                    for _ in range(120):
                        await omni.kit.app.get_app().next_update_async()
                        if result.done():
                            return result.result()
                    self.fail("Native gripper operation did not complete after 120 Kit updates")

                with ThreadPoolExecutor(max_workers=1) as pool:
                    try:
                        self.assertTrue(await call_while_pumping(interface.close_gripper, "/env0/SurfaceGripper"))
                        for _ in range(8):
                            await omni.kit.app.get_app().next_update_async()
                        self.assertTrue(await call_while_pumping(interface.get_gripped_objects, "/env0/SurfaceGripper"))
                        for _ in range(40):
                            self.assertEqual(
                                list(
                                    await call_while_pumping(interface.set_gripper_action_batch, paths, close_then_open)
                                ),
                                [True] * duplicate_count,
                            )
                            await omni.kit.app.get_app().next_update_async()
                        self.assertEqual(
                            await call_while_pumping(interface.get_gripper_status, "/env0/SurfaceGripper"),
                            GripperStatus.Open,
                        )
                    finally:
                        self._timeline.stop()
                self.assertTrue(step_threads - {main_thread}, "No worker-thread physics step was observed")
            finally:
                self._timeline.stop()
                if step_subscription is not None:
                    step_subscription.unsubscribe()

    async def test_deleted_active_joint_with_asynchronous_physics(self) -> None:
        """An async physics step must release a deleted active joint without stale-pointer use."""
        with _fail_on_native_stall():
            import omni.physx

            await self.setup_physics()
            await self.setup_gripper(1)
            PhysxSchema.PhysxSceneAPI.Apply(self._scene.GetPrim()).CreateUpdateTypeAttr().Set(
                PhysxSchema.Tokens.Asynchronous
            )
            self.gripper_view.set_surface_gripper_properties(
                coaxial_force_limit=[0.0], shear_force_limit=[0.0], retry_interval=[0.0]
            )
            main_thread = threading.get_ident()
            step_threads: set[int] = set()

            def record_step(dt: float) -> None:
                """Record the physics callback thread.

                Args:
                    dt: Physics timestep.
                """
                step_threads.add(threading.get_ident())

            step_subscription = None
            try:
                self._timeline.play()
                for _ in range(12):
                    await omni.kit.app.get_app().next_update_async()
                step_subscription = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
                    record_step, True, 0
                )
                for _ in range(8):
                    await omni.kit.app.get_app().next_update_async()
                self.assertTrue(
                    step_threads - {main_thread}, "Asynchronous physics never ran a step off the main thread"
                )
                interface = self.gripper_view.surface_gripper_interface

                async def call_while_pumping(pool: ThreadPoolExecutor, fn: Callable[..., Any], *args: Any) -> Any:
                    result = pool.submit(fn, *args)
                    for _ in range(120):
                        await omni.kit.app.get_app().next_update_async()
                        if result.done():
                            return result.result()
                    self.fail("Native gripper operation did not complete after 120 Kit updates")

                with ThreadPoolExecutor(max_workers=1) as pool:
                    try:
                        self.assertTrue(await call_while_pumping(pool, interface.close_gripper, "/env0/SurfaceGripper"))
                        for _ in range(8):
                            await omni.kit.app.get_app().next_update_async()
                        self.assertEqual(
                            await call_while_pumping(pool, interface.get_gripper_status, "/env0/SurfaceGripper"),
                            GripperStatus.Closed,
                        )
                        self.assertTrue(
                            await call_while_pumping(pool, interface.get_gripped_objects, "/env0/SurfaceGripper")
                        )
                        self.stage.RemovePrim("/env0/box1/d6Joint0")
                        for _ in range(8):
                            await omni.kit.app.get_app().next_update_async()
                        self.assertEqual(
                            await call_while_pumping(pool, interface.get_gripper_status, "/env0/SurfaceGripper"),
                            GripperStatus.Open,
                        )
                        self.assertEqual(
                            await call_while_pumping(pool, interface.get_gripped_objects, "/env0/SurfaceGripper"), []
                        )
                    finally:
                        self._timeline.stop()
            finally:
                self._timeline.stop()
                if step_subscription is not None:
                    step_subscription.unsubscribe()

    async def test_concurrent_native_batches_and_lifecycle(self) -> None:
        """Overlap GIL-releasing native batches with physics, property notices, removal and stop."""
        with _fail_on_native_stall():
            await self.setup_physics()
            await self.setup_gripper(2)
            self._timeline.play()
            for _ in range(8):
                await omni.kit.app.get_app().next_update_async()
            interface = self.gripper_view.surface_gripper_interface
            interface.set_write_to_usd(True)
            paths = ["/env0/SurfaceGripper", "/env1/SurfaceGripper"] * 16
            barrier = threading.Barrier(4)
            stop = threading.Event()

            def exercise_batches(worker: int) -> int:
                """Exercise native calls from a competing Python thread.

                Args:
                    worker: Index choosing the worker's operation.

                Returns:
                    Number of completed operations.
                """
                barrier.wait(timeout=10)
                completed = 0
                while not stop.is_set():
                    if worker == 0:
                        result = interface.set_gripper_action_batch(paths, [1.0, -1.0] * 16)
                    elif worker == 1:
                        result = interface.get_gripped_objects_batch(paths)
                    else:
                        result = interface.open_gripper_batch(paths)
                    self.assertEqual(len(result), len(paths))
                    completed += 1
                return completed

            pool = ThreadPoolExecutor(max_workers=3)
            futures = [pool.submit(exercise_batches, worker) for worker in range(3)]
            try:
                barrier.wait(timeout=10)
                for _ in range(12):
                    await omni.kit.app.get_app().next_update_async()
                # Only remove gripper schema prims, not externally owned PhysX joints/actors.
                self.stage.RemovePrim("/env0/SurfaceGripper")
                for _ in range(4):
                    await omni.kit.app.get_app().next_update_async()
                self._timeline.stop()
                for _ in range(4):
                    await omni.kit.app.get_app().next_update_async()
            finally:
                stop.set()
                self._timeline.stop()
                deadline = time.monotonic() + 10
                while not all(future.done() for future in futures) and time.monotonic() < deadline:
                    await omni.kit.app.get_app().next_update_async()
                pool.shutdown(wait=True)
                if all(future.done() for future in futures):
                    interface.set_write_to_usd(False)
            for future in futures:
                self.assertTrue(future.done(), "A native batch deadlocked against a callback")
                self.assertGreater(future.result(), 0)
            self.assertEqual(list(interface.open_gripper_batch(paths)), [False] * len(paths))

    async def test_native_calls_from_physics_callbacks(self) -> None:
        """Keep synchronous gripper commands callable from pre-step and post-step subscribers."""
        import omni.physx

        await self.setup_physics()
        await self.setup_gripper(1)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        interface = self.gripper_view.surface_gripper_interface
        counts = [0, 0]

        def before_step(dt: float) -> None:
            """Open the gripper before a simulation step.

            Args:
                dt: Physics timestep.
            """
            self.assertTrue(interface.open_gripper("/env0/SurfaceGripper"))
            counts[0] += 1

        def after_step(dt: float) -> None:
            """Close and query the gripper after a simulation step.

            Args:
                dt: Physics timestep.
            """
            self.assertTrue(interface.close_gripper("/env0/SurfaceGripper"))
            self.assertEqual(len(interface.get_gripped_objects_batch(["/env0/SurfaceGripper"])), 1)
            counts[1] += 1

        physics = omni.physx.get_physx_interface()
        before = physics.subscribe_physics_on_step_events(before_step, True, 0)
        after = physics.subscribe_physics_on_step_events(after_step, False, 0)
        try:
            for _ in range(4):
                await omni.kit.app.get_app().next_update_async()
        finally:
            before.unsubscribe()
            after.unsubscribe()
        self.assertGreater(counts[0], 0)
        self.assertEqual(counts[0], counts[1])
        self._timeline.stop()
