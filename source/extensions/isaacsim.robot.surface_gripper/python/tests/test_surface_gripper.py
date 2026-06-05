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

"""Test suite for the surface gripper functionality in Isaac Sim."""

import os

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import omni.physics.tensors
import omni.timeline
import omni.usd
from isaacsim.robot.surface_gripper import GripperView, create_surface_gripper
from isaacsim.robot.surface_gripper.bindings._surface_gripper import GripperStatus
from pxr import Gf, PhysxSchema
from usd.schema.isaac import robot_schema


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSurfaceGripper(omni.kit.test.AsyncTestCase):
    """Test suite for the surface gripper functionality in Isaac Sim.

    This test class validates the behavior of surface grippers, which are robotic end-effectors that can
    attach to and manipulate objects through surface contact forces. The tests cover gripper creation,
    configuration, opening/closing operations, object grasping, force limits, and multi-gripper scenarios.

    The test suite uses a gantry scene with multiple boxes positioned at different locations to test various
    gripper interactions. Each test method focuses on specific aspects of gripper functionality:

    - Creation and basic configuration of surface grippers
    - Property setting and retrieval (grip distance, force limits, retry intervals)
    - Open/close operations and status monitoring
    - Single and multi-object grasping capabilities
    - Distance thresholds for successful gripping
    - Retry mechanisms when objects move into range
    - Force-based grip breaking (both shear and coaxial forces)
    - Multi-gripper coordination and independent control

    The surface gripper uses attachment points defined by joints and applies configurable force limits
    to determine when objects should be gripped or released. Force thresholds prevent damage to objects
    while ensuring reliable grasping operations.
    """

    async def load_gantry_scene(self):
        """Loads the gantry scene with surface gripper from USD file.

        Loads the SurfaceGripper_gantry.usda scene file and sets it as the default prim in the stage.
        """
        usd_path = os.path.abspath(
            os.path.join(
                app_utils.get_extension_path("isaacsim.robot.surface_gripper"), "data", "SurfaceGripper_gantry.usda"
            )
        )
        stage_utils.add_reference_to_stage(usd_path, "/World")
        self._stage.SetDefaultPrim(self._stage.GetPrimAtPath("/World"))
        await omni.kit.app.get_app().next_update_async()

    async def setup_gripper_view(self, count: int):
        """Sets up surface gripper view with specified number of grippers.

        Creates surface gripper prims, configures their attachment points, and initializes a GripperView
        with default properties including grip distance, force limits, and retry interval.

        Args:
            count: Number of surface grippers to create and configure.
        """
        # Create and configure the surface gripper(s)
        for i in range(count):
            if i == 0:
                gripper_prim_path = "/World/SurfaceGripper"
                gripper_joints_prim_path = "/World/Surface_Gripper_Joints"
            else:
                gripper_prim_path = "/World/SurfaceGripper_0" + str(i)
                gripper_joints_prim_path = "/World/Surface_Gripper_Joints_0" + str(i)
            robot_schema.CreateSurfaceGripper(self._stage, gripper_prim_path)

            gripper_prim = self._stage.GetPrimAtPath(gripper_prim_path)
            attachment_points_rel = gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name)
            gripper_joints = [p.GetPath() for p in self._stage.GetPrimAtPath(gripper_joints_prim_path).GetChildren()]
            attachment_points_rel.SetTargets(gripper_joints)

        self.gripper_view = GripperView(paths="/World/SurfaceGripper.*")
        self.gripper_view.set_surface_gripper_properties(
            max_grip_distance=[0.02] * count,
            coaxial_force_limit=[0.005] * count,
            shear_force_limit=[5] * count,
            retry_interval=[1.0] * count,
        )

    async def update_joint_target_positions(self, joint_x_target: float, joint_y_target: float, joint_z_target: float):
        """Updates the target positions for gantry joints and simulates movement.

        Sets target positions for x, y, and z joints sequentially, then simulates for 1 second
        to allow the joints to reach their target positions.

        Args:
            joint_x_target: Target position for the x-axis joint.
            joint_y_target: Target position for the y-axis joint.
            joint_z_target: Target position for the z-axis joint.
        """
        joint_x = self._stage.GetPrimAtPath("/World/Joints/x_joint")
        joint_x.GetAttribute("drive:linear:physics:targetPosition").Set(joint_x_target)
        await omni.kit.app.get_app().next_update_async()
        joint_y = self._stage.GetPrimAtPath("/World/Joints/y_joint")
        joint_y.GetAttribute("drive:linear:physics:targetPosition").Set(joint_y_target)
        await omni.kit.app.get_app().next_update_async()
        joint_z = self._stage.GetPrimAtPath("/World/Joints/z_joint")
        joint_z.GetAttribute("drive:linear:physics:targetPosition").Set(joint_z_target)
        await omni.kit.app.get_app().next_update_async()
        # Simulate for 1 second (60 frames at 60fps)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

    async def setUp(self):
        """Sets up the test environment before each test case.

        Creates a new stage with meter units, loads the gantry scene, and initializes
        the timeline interface for test execution.
        """
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        self._stage = stage_utils.get_current_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.kit.app.get_app().next_update_async()
        await self.load_gantry_scene()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Cleans up the test environment after each test case.

        Waits for stage loading to complete and performs necessary cleanup operations
        to ensure a clean state for subsequent tests.
        """
        await omni.kit.app.get_app().next_update_async()
        # self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # Simulate for 1 second (60 frames at 60fps)
            for _ in range(60):
                await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_create_surface_gripper(self):
        """Tests the creation of a surface gripper.

        Verifies that a single surface gripper can be created and initialized properly
        with the timeline running.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        pass

    async def test_configure_surface_gripper(self):
        """Tests surface gripper property configuration.

        Verifies that surface gripper properties (max grip distance, coaxial force limit,
        shear force limit, and retry interval) are set correctly to their expected values.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        max_grip_distance, coaxial_force_limit, shear_force_limit, retry_interval = (
            self.gripper_view.get_surface_gripper_properties()
        )
        # expected values, set during configuration:
        expected_properties = [0.02, 0.005, 5, 1.0]
        self.assertAlmostEqual(max_grip_distance[0], expected_properties[0])
        self.assertAlmostEqual(coaxial_force_limit[0], expected_properties[1])
        self.assertAlmostEqual(shear_force_limit[0], expected_properties[2])
        self.assertAlmostEqual(retry_interval[0], expected_properties[3])

    async def test_close_open_close_surface_gripper(self):
        """Tests the complete close-open-close cycle of surface gripper operation.

        Positions the gripper over an object, tests closing to grip the object,
        opening to release it, and closing again to re-grip the same object.
        Verifies gripper status and gripped objects at each stage.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()

        await self.update_joint_target_positions(0.0, 0.0, 0.140)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closed)

        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 1)
        self.assertEqual(gripped_object_list[0][0], "/World/Boxes/Cube_28")

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Open the gripper
        self.gripper_view.apply_gripper_action([-0.5])
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Open)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(120):
            await omni.kit.app.get_app().next_update_async()
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closed)
        self.assertEqual(len(gripped_object_list[0]), 1)
        self.assertEqual(gripped_object_list[0][0], "/World/Boxes/Cube_28")

    async def test_multi_object_close(self):
        """Tests surface gripper ability to grip multiple objects simultaneously.

        Positions the gripper to interact with multiple objects, verifies it can
        grip two objects at once, release them, and grip them again. Tests the
        complete multi-object gripping cycle.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()

        await self.update_joint_target_positions(0.175, 0.0, 0.140)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        expected_gripped_object_list = ["/World/Boxes/Cube_20", "/World/Boxes/Cube_24"]
        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closed)
        await omni.kit.app.get_app().next_update_async()

        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 2)
        self.assertTrue(sorted(set(gripped_object_list[0])) == sorted(set(expected_gripped_object_list)))
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Open the gripper
        self.gripper_view.apply_gripper_action([-0.5])
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Open)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closed)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 2)
        self.assertTrue(sorted(set(gripped_object_list[0])) == sorted(set(expected_gripped_object_list)))

    async def test_close_threshold(self):
        """Tests surface gripper behavior when objects are beyond grip threshold.

        Positions the gripper at a distance where objects are too far to be gripped,
        verifies that close attempts fail and the gripper remains open with no
        gripped objects throughout the test cycle.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()

        await self.update_joint_target_positions(0.0, 0.0, 0.125)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Open)

        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Open the gripper
        self.gripper_view.apply_gripper_action([-0.5])
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Open)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Open)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)

    async def test_retry_interval(self):
        """Test the retry interval functionality of the surface gripper.

        Verifies that a gripper in closing state will continue attempting to grip objects when moved into range
        during the retry interval period.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()

        await self.update_joint_target_positions(0.0, 0.0, 0.125)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # Begin closing the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(6):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closing)

        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)

        await self.update_joint_target_positions(0.0, 0.0, 0.140)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closed)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 1)
        self.assertEqual(gripped_object_list[0][0], "/World/Boxes/Cube_28")

    async def test_shear_break_forces(self):
        """Test the shear force breaking functionality of the surface gripper.

        Verifies that the gripper releases objects when shear forces exceed the configured limit.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()

        await self.update_joint_target_positions(0.0, 0.0, 0.140)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # Max shear force is 5
        # Box 28 has mass of 0.2
        box_28_prim_path = "/World/Boxes/Cube_28"
        box_28_prim = self._stage.GetPrimAtPath(box_28_prim_path)
        forceApi = PhysxSchema.PhysxForceAPI.Apply(box_28_prim)
        forceApi.GetForceAttr().Set(Gf.Vec3f(0.0, -4, 0.0))
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closed)

        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 1)
        self.assertEqual(gripped_object_list[0][0], box_28_prim_path)

        forceApi.GetForceAttr().Set(Gf.Vec3f(0.0, -500, 0.0))
        await omni.kit.app.get_app().next_update_async()
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Open)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)

    async def test_coaxial_break_force(self):
        """Test the coaxial force breaking functionality of the surface gripper.

        Verifies that the gripper releases objects when coaxial forces exceed the configured limit.
        """
        gripper_count = 1
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()

        await self.update_joint_target_positions(0.0, 0.0, 0.140)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # Close the gripper
        self.gripper_view.apply_gripper_action([0.5])
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        # Max coaxial force is 0.005
        # Box 28 has mass of 0.2
        box_28_prim_path = "/World/Boxes/Cube_28"
        box_28_prim = self._stage.GetPrimAtPath(box_28_prim_path)
        forceApi = PhysxSchema.PhysxForceAPI.Apply(box_28_prim)
        forceApi.GetForceAttr().Set(Gf.Vec3f(0.0, 0.0, -0.003))
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Closed)

        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 1)
        self.assertEqual(gripped_object_list[0][0], box_28_prim_path)

        await self.update_joint_target_positions(0.0, 0.0, 0.0)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        forceApi.GetForceAttr().Set(Gf.Vec3f(0.0, 0.0, -1000))
        await omni.kit.app.get_app().next_update_async()
        for _ in range(120):
            await omni.kit.app.get_app().next_update_async()
        self.assertEqual(GripperStatus(self.gripper_view.get_surface_gripper_status()[0]), GripperStatus.Open)
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)

    async def test_multi_gripper_scene(self):
        """Test multiple surface grippers operating simultaneously in the same scene.

        Verifies that two surface grippers can independently grip and release objects with different statuses.
        """
        gripper_count = 2
        await self.setup_gripper_view(gripper_count)
        self._timeline.play()

        # Move grippers down to touch the boxes
        # First gripper should be "Closing" (Some gripper joints are off the box)
        # Second gripper should be "Closed"
        await self.update_joint_target_positions(0.0, 0.05, 0.14)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        gripper_prim_paths = ["/World/SurfaceGripper", "/World/SurfaceGripper_01"]

        # Close the grippers
        actions = [0.5, 0.5]
        expected_statuses = [GripperStatus.Closing, GripperStatus.Closed]

        self.gripper_view.apply_gripper_action(actions)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        statuses = self.gripper_view.get_surface_gripper_status()
        for i in range(gripper_count):
            self.assertEqual(GripperStatus(statuses[i]), expected_statuses[i])

        # Check gripped objects
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 1)
        self.assertEqual(gripped_object_list[0][0], "/World/Boxes/Cube_28")

        self.assertEqual(len(gripped_object_list[1]), 1)
        self.assertEqual(gripped_object_list[1][0], "/World/Boxes/Cube_30")

        # Open the grippers
        actions = [-0.5, -0.5]
        expected_statuses = [GripperStatus.Open, GripperStatus.Open]

        self.gripper_view.apply_gripper_action(actions)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        statuses = self.gripper_view.get_surface_gripper_status()
        for i in range(gripper_count):
            self.assertEqual(GripperStatus(statuses[i]), expected_statuses[i])

        # Check gripped objects
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 0)

        self.assertEqual(len(gripped_object_list[1]), 0)

        # Close the grippers
        actions = [0.5, 0.5]
        expected_statuses = [GripperStatus.Closing, GripperStatus.Closed]

        self.gripper_view.apply_gripper_action(actions)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        statuses = self.gripper_view.get_surface_gripper_status()
        for i in range(gripper_count):
            self.assertEqual(GripperStatus(statuses[i]), expected_statuses[i])

        # Check gripped objects
        gripped_object_list = self.gripper_view.get_gripped_objects()
        self.assertEqual(len(gripped_object_list[0]), 1)
        self.assertEqual(gripped_object_list[0][0], "/World/Boxes/Cube_28")

        self.assertEqual(len(gripped_object_list[1]), 1)
        self.assertEqual(gripped_object_list[1][0], "/World/Boxes/Cube_30")


class TestCreateSurfaceGripper(omni.kit.test.AsyncTestCase):
    """Test create_surface_gripper direct Python API."""

    async def test_create_surface_gripper_direct(self) -> None:
        """Test creating a surface gripper using create_surface_gripper directly."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        prim_path = "/World"
        gripper_prim = create_surface_gripper(stage, prim_path)
        self.assertIsNotNone(gripper_prim)
        self.assertTrue(gripper_prim.IsValid())
        self.assertEqual(str(gripper_prim.GetPath()), prim_path + "/SurfaceGripper")
