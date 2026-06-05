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

"""Tests for USD stage utility functions."""

import asyncio

import carb
import omni.kit.test
import omni.usd
import usdrt
from isaacsim.core.utils.prims import create_prim, define_prim, get_prim_at_path
from isaacsim.core.utils.stage import (
    add_reference_to_stage,
    clear_stage,
    create_new_stage_async,
    create_new_stage_in_memory,
    get_current_stage,
    get_current_stage_id,
    update_stage_async,
    use_stage,
)
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Usd, UsdGeom, UsdUtils


class TestStage(omni.kit.test.AsyncTestCase):
    """Test cases for Stage."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.set_extension_enabled("omni.physx.fabric", True)
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()

    def is_current_stage_in_memory() -> bool:
        """This function checks if the current stage is in memory.

        Compares the stage id of the current stage with the stage id of the context stage.

        Returns:
            If the current stage is in memory.
        """
        # grab current stage id
        stage_id = get_current_stage_id()

        # grab context stage id
        context_stage = omni.usd.get_context().get_stage()
        with use_stage(context_stage):
            context_stage_id = get_current_stage_id()

        # check if stage ids are the same
        return stage_id != context_stage_id

    def attach_stage_to_usd_context() -> None:
        """Attach stage in memory to usd context.

        This function should be called during or after scene is created and before stage is simulated or rendered.

        Note:
            If the stage is not in memory, this function will return without doing anything.
        """
        import omni.physxfabric

        # is stage is not in memory, we don't need to attach it
        if not TestStage.is_current_stage_in_memory():
            return

        stage_id = get_current_stage_id()

        omni.usd.get_context().attach_stage_with_callback(stage_id)

    async def test_clear_stage(self) -> None:
        """Test clear stage."""
        await create_new_stage_async()
        prim = define_prim(prim_path="/Test", prim_type="Xform")
        self.assertTrue(prim.IsValid())
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
        await update_stage_async()

        self.assertTrue(robot.IsValid())
        clear_stage()
        await update_stage_async()
        self.assertFalse(prim.IsValid())
        self.assertFalse(robot.IsValid())

    async def test_add_reference_to_stage_units(self) -> None:
        """Test add reference to stage units."""
        await create_new_stage_async()
        clear_stage()
        # setup different units
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            raise Exception("Asset root path doesn't exist")

        # use just hand to avoid scene graph instancing
        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/Props/panda_hand.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")

        xform_ref = stage.GetPrimAtPath("/World/Franka")
        self.assertTrue(not xform_ref.GetAttribute("xformOp:scale:unitsResolve"))

        clear_stage()

        # enable omni.usd.metrics.assembler.ui
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.set_extension_enabled_immediate("omni.usd.metrics.assembler.ui", True)

        # setup different units
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")

        xform_ref = stage.GetPrimAtPath("/World/Franka")
        self.assertEqual(xform_ref.GetAttribute("xformOp:scale:unitsResolve").Get(), [100.0, 100.0, 100.0])

        ext_manager.set_extension_enabled_immediate("omni.usd.metrics.assembler.ui", False)

    async def test_context_manager(self) -> None:
        """Test context manager."""
        await create_new_stage_async()
        stage_in_memory = Usd.Stage.CreateInMemory()
        default_stage = omni.usd.get_context().get_stage()
        default_stage_id = UsdUtils.StageCache.Get().GetId(default_stage).ToLongInt()
        self.assertIs(get_current_stage(), default_stage)
        self.assertEqual(get_current_stage_id(), default_stage_id)
        # - USD stage
        with use_stage(stage_in_memory):
            self.assertIs(get_current_stage(), stage_in_memory)
            self.assertIsNot(get_current_stage(), default_stage)
            self.assertNotEqual(get_current_stage_id(), default_stage_id)
        self.assertIs(get_current_stage(), default_stage)
        self.assertEqual(get_current_stage_id(), default_stage_id)
        # - get Fabric stage
        with use_stage(stage_in_memory):
            self.assertIsInstance(get_current_stage(fabric=True), usdrt.Usd.Stage)
        self.assertIsInstance(get_current_stage(fabric=True), usdrt.Usd.Stage)

    async def test_stage_in_memory(self) -> None:
        """Test stage in memory."""
        # create stage in memory
        stage_in_memory = create_new_stage_in_memory()

        prim_path = "/World/Test"
        # set-up with context to use stage in memory
        with use_stage(stage_in_memory):
            # create test prim
            create_prim(prim_path)

            # verify stage is in memory
            self.assertTrue(TestStage.is_current_stage_in_memory())

            # verify prim exists in stage in memory
            prim = get_prim_at_path(prim_path)
            self.assertTrue(prim.IsValid())

            # verify prim does not exist in context stage
            context_stage = omni.usd.get_context().get_stage()
            with use_stage(context_stage):
                prim = get_prim_at_path(prim_path)
                self.assertFalse(prim.IsValid())

            # attach stage to context
            TestStage.attach_stage_to_usd_context()

        # verify stage is no longer in memory
        self.assertFalse(TestStage.is_current_stage_in_memory())

        # verify prim now exists in context stage
        prim = get_prim_at_path(prim_path)
        self.assertTrue(prim.IsValid())
