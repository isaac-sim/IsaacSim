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

"""Tests for the robot assembler assembly and disassembly workflows."""

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
from isaacsim.core.experimental.objects import SphereLight
from isaacsim.robot_setup.assembler import RobotAssembler
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf, UsdGeom


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestRobotAssembler(omni.kit.test.AsyncTestCase):
    """Test the robot assembler assembly, cancellation, and finishing workflows."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with robot assembler and stage."""
        self._physics_fps = 60
        self._physics_dt = 1 / self._physics_fps  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.robot_setup.assembler")

        self._robot_assembler = RobotAssembler()

        await stage_utils.create_new_stage_async()

        self.stage = omni.usd.get_context().get_stage()
        self.stage.DefinePrim(Sdf.Path("/World"), "Xform")

        await self._prepare_stage()

        await app_utils.update_app_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test environment and reset the assembler."""
        self._timeline.stop()
        self._robot_assembler.reset()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await app_utils.update_app_async()
        await app_utils.update_app_async()

    async def _create_light(self) -> None:
        sphere_light = SphereLight("/World/SphereLight", radii=2.0, positions=[6.5, 0, 12])
        sphere_light.set_intensities(100000)

    def assertListsSame(self, l1: list, l2: list) -> None:  # noqa: N802
        """Assert that two lists contain the same elements regardless of order.

        Args:
            l1: First list to compare.
            l2: Second list to compare.
        """
        for item in l1:
            self.assertTrue(item in l2, f"{l1}, {l2}")

        self.assertTrue(len(l2) == len(l1), f"{l1}, {l2}")

    async def _prepare_stage(self) -> None:
        # Set settings to ensure deterministic behavior
        # Initialize the robot
        # Play the timeline

        self._timeline.stop()
        await self._create_light()
        assets_root_path = await get_assets_root_path_async()
        stage_utils.add_reference_to_stage(
            assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd", "/World/ur10e"
        )
        stage_utils.add_reference_to_stage(
            assets_root_path + "/Isaac/Robots/WonikRobotics/AllegroHand/allegro_hand_instanceable.usd",
            "/World/allegro_hand",
        )

        self._robot_base = "/World/ur10e"
        self._robot_base_mount = "/ee_link"
        self._robot_attach = "/World/allegro_hand"
        self._robot_attach_mount = "/allegro_mount"

        await app_utils.update_app_async()

    def apply_rotation(self, axis: list, angle: float) -> None:
        """Apply a rotation to the attachment robot prim.

        Args:
            axis: Rotation axis vector.
            angle: Rotation angle in degrees.
        """
        prim = self.stage.GetPrimAtPath(self._robot_assembler._attachment_robot_prim)

        xformable = UsdGeom.Xformable(prim)
        old_matrix = xformable.GetLocalTransformation()
        old_rotation = old_matrix.ExtractRotation()
        rotation = Gf.Rotation(axis, angle)
        new_matrix = Gf.Matrix4d().SetRotateOnly(rotation) * old_matrix

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath(),
            old_transform_matrix=old_matrix,
            new_transform_matrix=new_matrix,
        )

    async def test_robot_assembler_begin_assembly(self) -> None:
        """Test beginning a robot assembly with rotation adjustments."""
        self._robot_assembler.begin_assembly(
            self.stage,
            self._robot_base,
            self._robot_base + self._robot_base_mount,
            self._robot_attach,
            self._robot_attach + self._robot_attach_mount,
            "Gripper",
            "allegro_hand",
        )

        self.apply_rotation([0, 0, 1], -90)
        self.apply_rotation([0, 1, 0], -90)

        await self._assert_pose()

    async def test_robot_assembler_cancel_assembly(self) -> None:
        """Test canceling a robot assembly restores original pose."""
        self._robot_assembler.begin_assembly(
            self.stage,
            self._robot_base,
            self._robot_base + self._robot_base_mount,
            self._robot_attach,
            self._robot_attach + self._robot_attach_mount,
            "Gripper",
            "allegro_hand",
        )
        self._robot_assembler.cancel_assembly()
        await app_utils.update_app_async(steps=5)
        attachment_mount_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_attach + self._robot_attach_mount)
        )
        self.assertAlmostEqual(np.linalg.norm(attachment_mount_pose - np.eye(4)), 0.0, 2)

    async def test_robot_assembler_cancel_twice(self) -> None:
        """Test canceling assembly twice in succession."""
        for i in range(2):
            await self.test_robot_assembler_cancel_assembly()

    async def test_robot_assembler_cancel_on_idle_instance_is_noop(self) -> None:
        """Regression: ``cancel_assembly`` on a fresh instance must be a no-op.

        Calling it on a never-started instance must not raise
        ``AttributeError: '_assembly_identifier'``.

        Reproduces the failure mode where calling ``cancel_assembly`` (or any
        flow that reaches it via ``reset()``) before ``begin_assembly`` has
        ever run crashed because the assembly-sublayer state attributes were
        only initialized inside ``begin_assembly``.
        """
        fresh_assembler = RobotAssembler()
        # Must not raise.
        fresh_assembler.cancel_assembly()
        # Allow the async cleanup scheduled by cancel_assembly to settle.
        await app_utils.update_app_async(steps=2)
        # A second cancel on the now-reset instance must also be safe.
        fresh_assembler.cancel_assembly()
        await app_utils.update_app_async(steps=2)
        # Direct reset on a fresh instance must likewise be safe.
        RobotAssembler().reset()

    async def _assert_pose(self) -> None:
        robot_base_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_base + self._robot_base_mount)
        )
        attachment_mount_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_attach + self._robot_attach_mount)
        )

        dist = np.linalg.norm(robot_base_pose.ExtractTranslation() - attachment_mount_pose.ExtractTranslation())
        self.assertAlmostEqual(float(dist), 0.0, 2)

    async def _assert_assembled(self) -> None:
        self._robot_assembler.assemble()

        attachment_root_joint = self.stage.GetPrimAtPath(self._robot_attach + "/root_joint")
        self.assertFalse(attachment_root_joint.IsActive())

        await self._assert_pose()

        self._timeline.play()
        await app_utils.update_app_async(steps=20)
        await self._assert_pose()

    async def test_robot_assembler_assemble(self) -> None:
        """Test assembling robots and verifying pose during simulation."""
        await self.test_robot_assembler_begin_assembly()
        await self._assert_assembled()
        self._timeline.stop()
        await app_utils.update_app_async()

    async def test_robot_assembler_assemble_twice(self) -> None:
        """Test assembling robots twice with a cancel in between."""
        await self.test_robot_assembler_assemble()
        self._robot_assembler.cancel_assembly()
        await app_utils.update_app_async(steps=5)
        await self.test_robot_assembler_assemble()

    async def test_robot_assembler_finish_assembly(self) -> None:
        """Test finishing assembly and verifying pose persists after timeline stop."""
        await self.test_robot_assembler_assemble()
        self._robot_assembler.finish_assemble()
        await app_utils.update_app_async(steps=10)
        attachment_root_joint = self.stage.GetPrimAtPath(self._robot_attach + "/root_joint")
        self.assertFalse(attachment_root_joint.IsActive())

        await self._assert_pose()

        self._timeline.play()
        await app_utils.update_app_async(steps=20)
        await self._assert_pose()

        self._timeline.stop()
        await app_utils.update_app_async()
        await self._assert_pose()
