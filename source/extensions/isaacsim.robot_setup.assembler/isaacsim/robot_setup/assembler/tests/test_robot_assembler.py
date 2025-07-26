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

import asyncio
import os
from typing import List

import carb
import numpy as np
import omni.kit.test
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
    update_stage_async,
)
from isaacsim.robot_setup.assembler import RobotAssembler
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf, UsdGeom, UsdLux


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestRobotAssembler(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_fps = 60
        self._physics_dt = 1 / self._physics_fps  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", self._physics_fps)
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", self._physics_fps)
        omni.timeline.get_timeline_interface().set_target_framerate(self._physics_fps)

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.robot_setup.assembler")

        self._robot_assembler = RobotAssembler()

        await create_new_stage_async()

        self.stage = omni.usd.get_context().get_stage()
        self.stage.DefinePrim(Sdf.Path("/World"), "Xform")
        omni.usd.get_context().get_stage().SetTimeCodesPerSecond(self._physics_fps)

        await self._prepare_stage()

        await update_stage_async()

        pass

    async def _wait_n_frames(self, n):
        for i in range(n):
            await update_stage_async()

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        self._robot_assembler.reset()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await update_stage_async()
        await update_stage_async()
        pass

    async def _create_light(self):
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(sphereLight.GetPath().pathString).set_world_pose([6.5, 0, 12])

    def assertListsSame(self, l1, l2):
        for item in l1:
            self.assertTrue(item in l2, f"{l1}, {l2}")

        self.assertTrue(len(l2) == len(l1), f"{l1}, {l2}")

    async def _prepare_stage(self):
        # Set settings to ensure deterministic behavior
        # Initialize the robot
        # Play the timeline

        self._timeline.stop()
        await self._create_light()
        assets_root_path = await get_assets_root_path_async()
        add_reference_to_stage(assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd", "/World/ur10e")
        add_reference_to_stage(
            assets_root_path + "/Isaac/Robots/WonikRobotics/AllegroHand/allegro_hand_instanceable.usd",
            "/World/allegro_hand",
        )

        self._robot_base = "/World/ur10e"
        self._robot_base_mount = "/ee_link"
        self._robot_attach = "/World/allegro_hand"
        self._robot_attach_mount = "/allegro_mount"

        await update_stage_async()

    def apply_rotation(self, axis, angle):
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

    async def test_robot_assembler_begin_assembly(self):
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

    async def test_robot_assembler_cancel_assembly(self):
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
        await self._wait_n_frames(5)
        attachment_mount_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_attach + self._robot_attach_mount)
        )
        self.assertAlmostEqual(np.linalg.norm(attachment_mount_pose - np.eye(4)), 0.0, 2)

    async def test_robot_assembler_cancel_twice(self):
        for i in range(2):
            await self.test_robot_assembler_cancel_assembly()

    async def _assert_pose(self):
        robot_base_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_base + self._robot_base_mount)
        )
        attachment_mount_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_attach + self._robot_attach_mount)
        )

        dist = np.linalg.norm(robot_base_pose.ExtractTranslation() - attachment_mount_pose.ExtractTranslation())
        self.assertAlmostEqual(float(dist), 0.0, 2)

    async def _assert_assembled(self):
        self._robot_assembler.assemble()

        attachment_root_joint = self.stage.GetPrimAtPath(self._robot_attach + "/root_joint")
        self.assertFalse(attachment_root_joint.IsActive())

        await self._assert_pose()

        self._timeline.play()
        await self._wait_n_frames(20)
        await self._assert_pose()

    async def test_robot_assembler_assemble(self):
        await self.test_robot_assembler_begin_assembly()
        await self._assert_assembled()
        self._timeline.stop()
        await self._wait_n_frames(1)

    async def test_robot_assembler_assemble_twice(self):
        await self.test_robot_assembler_assemble()
        self._robot_assembler.cancel_assembly()
        await self._wait_n_frames(5)
        await self.test_robot_assembler_assemble()

    async def test_robot_assembler_finish_assembly(self):
        await self.test_robot_assembler_assemble()
        self._robot_assembler.finish_assemble()
        await self._wait_n_frames(10)
        attachment_root_joint = self.stage.GetPrimAtPath(self._robot_attach + "/root_joint")
        self.assertFalse(attachment_root_joint.IsActive())

        await self._assert_pose()

        self._timeline.play()
        await self._wait_n_frames(20)
        await self._assert_pose()

        self._timeline.stop()
        await self._wait_n_frames(1)
        await self._assert_pose()
