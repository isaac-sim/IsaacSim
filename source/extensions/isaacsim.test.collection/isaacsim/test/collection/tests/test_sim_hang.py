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

"""Tests for various hang and crash bugs in Isaac Sim simulation."""

import asyncio

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Usd, UsdPhysics


def create_fixed_cuboid(stage: Usd.Stage, prim_path: str, position: list, scale: list) -> None:
    """Create a fixed (kinematic) cuboid using experimental Cube, RigidPrim, and GeomPrim.

    Args:
        stage: USD stage to create the prim in.
        prim_path: USD path for the cuboid prim.
        position: Position as [x, y, z] coordinates.
        scale: Scale factor(s) for the cuboid dimensions.
    """
    # Create the cube geometry at the specified position and scale
    Cube(prim_path, positions=[position], scales=[scale])
    # Apply collision via GeomPrim
    GeomPrim(prim_path, apply_collision_apis=True)
    # Apply rigid body physics via RigidPrim
    RigidPrim(prim_path)
    # Set kinematic enabled to make it fixed/static
    prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestHangBugs(omni.kit.test.AsyncTestCase):
    """Tests for various hang and crash bugs in simulation."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with new stage."""
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()
        await app_utils.update_app_async()

        await stage_utils.create_new_stage_async()

        await app_utils.update_app_async()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await app_utils.update_app_async()

    async def test_prim_visibility_bug(self) -> None:
        """Test that making prim invisible and deleting it does not crash."""
        # Bug report:
        #     From the Test Runner run this test case
        #     The test case will pass, and a few seconds after that, Sim will segfault and crash

        # The repro is simple:
        #     Make a prim
        #     Set it to be invisible
        #     Delete it

        from pxr import UsdGeom

        self._timeline = omni.timeline.get_timeline_interface()
        stage = omni.usd.get_context().get_stage()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        prim_path = "/test_crasher"

        cubeGeom = UsdGeom.Cube.Define(stage, prim_path)

        # This is somehow necessary to cause the bug
        imageable = UsdGeom.Imageable(cubeGeom)
        imageable.MakeInvisible()

        await omni.kit.app.get_app().next_update_async()

        from omni.usd.commands import DeletePrimsCommand

        DeletePrimsCommand([cubeGeom.GetPath()]).do()

        # But Sim will segfault a within a few seconds after this returns.  The time varies wildly

    async def test_segfault_bug(self) -> None:
        """Test that recreating cuboids with different scales does not segfault."""
        # Bug Report:
        #     A strange combination of events has to take place.

        #     The Franka USD is added to the stage

        #     A cuboid is created with position and scale
        #     Then it is referenced again with the same position a different scale

        #     The position and scale arguments have to be present for this segfault to happen

        # It has to be the Franka USD that gets loaded here to cause the issue
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        stage_utils.add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()
        stage = omni.usd.get_context().get_stage()

        # Start Simulation and wait
        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        obs_pos = np.array([0.3, 0.20, 0.50])

        await omni.kit.app.get_app().next_update_async()

        obs = create_fixed_cuboid(stage, "/scene/obstacle", obs_pos, 0.1 * np.ones(3))

        await omni.kit.app.get_app().next_update_async()

        # Delete and recreate with different scale
        stage_utils.delete_prim("/scene/obstacle")
        await omni.kit.app.get_app().next_update_async()

        obs = create_fixed_cuboid(stage, "/scene/obstacle", obs_pos, 0.1 * np.array([2.0, 3.0, 1.0]))

        for i in range(100):
            carb.log_info(f"Iteration {i}")
            await app_utils.update_app_async()

    async def test_freeze_sim(self) -> None:
        """Test that repeatedly opening stages does not cause simulation freeze."""
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        for i in range(100):
            result, error = await open_stage_async(usd_path)
            await app_utils.update_app_async()
            self.assertTrue(result)

            carb.log_info(f"Opened Stage {i+1} times")
