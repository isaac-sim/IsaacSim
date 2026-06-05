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

"""Test module for physics simulation behavior and USD integration in Isaac Sim."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils.app import get_extension_path
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


class TestPhysics(omni.kit.test.AsyncTestCase):
    """Tests for physics simulation behavior and USD integration."""

    async def setUp(self):
        """Set up test environment with new stage and physics settings."""
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        carb.settings.get_settings().set("persistent/app/stage/upAxis", "Z")
        # force editor and physics to have the same rate (should be 60)
        self._physics_rate = 60
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up test environment and wait for assets to load."""
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_usd_updates(self):
        """Test that physics updates propagate to USD when enabled."""
        carb.settings.get_settings().set_int("physics/updateToUsd", True)
        # Create cube geometry with collision and rigid body physics
        cube_prim = Cube("/World/Cube", positions=[[0, 0, 25]])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        rigid_prim = RigidPrim("/World/Cube")

        omni.timeline.get_timeline_interface().play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell due to gravity
        positions, _ = cube_prim.get_world_poses()
        position = positions.numpy()[0]
        self.assertAlmostEqual(position[2], 20.013252, 0)
        carb.settings.get_settings().set_int("physics/updateToUsd", False)
        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()
        omni.timeline.get_timeline_interface().play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        positions, _ = cube_prim.get_world_poses()
        position = positions.numpy()[0]
        self.assertAlmostEqual(position[2], 25.0, 0)
        carb.settings.get_settings().set_int("physics/updateToUsd", True)

    async def test_rigid_body(self):
        """Test rigid body physics equations of motion under gravity."""
        dt = 1.0 / self._physics_rate

        # add scene
        self._scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        self._scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Add a cube
        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(self._stage, cubePath)
        cubeGeom.CreateSizeAttr(100)
        cubePrim = self._stage.GetPrimAtPath(cubePath)
        # await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        await omni.kit.app.get_app().next_update_async()

        rigid_prim = RigidPrim(cubePath)

        # test acceleration, velocity, position
        omni.timeline.get_timeline_interface().play()
        # warm up simulation
        await omni.kit.app.get_app().next_update_async()
        # get initial position
        a = -9.81

        # simulate for one second
        time_elapsed = dt
        for frame in range(30):
            positions, _ = rigid_prim.get_world_poses()
            p_0 = positions.numpy()[0]
            v_0 = np.array(rigidBodyAPI.GetVelocityAttr().Get())
            await omni.kit.app.get_app().next_update_async()

            positions, _ = rigid_prim.get_world_poses()
            p_1 = positions.numpy()[0]
            v_1 = np.array(rigidBodyAPI.GetVelocityAttr().Get())
            # print("time elapsed", time_elapsed)
            v_expected = v_0[2] + a * dt
            self.assertAlmostEqual(v_1[2], v_expected, 0)
            # print(v_1[2], v_expected)
            p_expected = p_0[2] + v_expected * dt
            # print(p_1[2], p_expected)
            self.assertAlmostEqual(p_1[2], p_expected, 0)
            time_elapsed += dt
        omni.timeline.get_timeline_interface().stop()

    async def test_reparenting(self):
        """Test that prim reparenting works during simulation."""
        timeline = omni.timeline.get_timeline_interface()
        omni.kit.commands.execute("CreatePrim", prim_type="Xform")
        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute("MovePrim", path_from="/Xform", path_to="/AnotherPath")
        timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()

    async def test_stage_up_axis(self):
        """Test gravity direction changes with stage up axis setting."""
        timeline = omni.timeline.get_timeline_interface()
        # Make a new stage Z up
        carb.settings.get_settings().set("persistent/app/stage/upAxis", "Z")
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        # Add a cube for testing gravity
        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(stage, cubePath)
        cubeGeom.CreateSizeAttr(1.00)
        cubePrim = stage.GetPrimAtPath(cubePath)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        await omni.kit.app.get_app().next_update_async()

        rigid_prim = RigidPrim(cubePath)

        timeline.play()
        for frame in range(58):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell -Z
        positions, _ = rigid_prim.get_world_poses()
        position = positions.numpy()[0]
        self.assertAlmostEqual(position[2], -4.9867, delta=0.01)
        timeline.stop()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        await omni.kit.app.get_app().next_update_async()

        timeline.play()
        for frame in range(58):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell -Y
        positions, _ = rigid_prim.get_world_poses()
        position = positions.numpy()[0]
        self.assertAlmostEqual(position[1], -4.9867, delta=0.01)

    async def test_stage_units(self):
        """Test physics behavior is consistent across different stage units."""
        timeline = omni.timeline.get_timeline_interface()
        # Make a new stage Z up
        carb.settings.get_settings().set("persistent/app/stage/upAxis", "Z")
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        # Add a cube for testing gravity
        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(stage, cubePath)
        cubeGeom.CreateSizeAttr(1.00)
        cubePrim = stage.GetPrimAtPath(cubePath)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        await omni.kit.app.get_app().next_update_async()

        rigid_prim = RigidPrim(cubePath)

        timeline.play()
        for frame in range(58):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell -Z
        positions, _ = rigid_prim.get_world_poses()
        position = positions.numpy()[0]
        self.assertAlmostEqual(position[2], -4.9867, delta=0.01)
        timeline.stop()
        # switch to meters
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        await omni.kit.app.get_app().next_update_async()

        timeline.play()
        for frame in range(58):
            await omni.kit.app.get_app().next_update_async()
        positions, _ = rigid_prim.get_world_poses()
        position = positions.numpy()[0]
        self.assertAlmostEqual(position[2], -4.9867, delta=0.01)

    async def test_articulation_reference(self):
        """Test articulation maintains consistent pose across timeline resets."""
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        stage = omni.usd.get_context().get_stage()
        timeline = omni.timeline.get_timeline_interface()

        stage_utils.add_reference_to_stage(asset_path, "/franka")

        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()

        hand_xform = XformPrim("/franka/panda_hand")

        await omni.kit.app.get_app().next_update_async()
        positions, _ = hand_xform.get_world_poses()
        trans_a = positions.numpy()[0]

        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()

        positions, _ = hand_xform.get_world_poses()
        trans_b = positions.numpy()[0]

        self.assertAlmostEqual(np.linalg.norm(trans_a - trans_b), 0, delta=0.03)

    async def test_articulation_drive(self):
        """Test articulation drive behavior with and without articulation root."""
        timeline = omni.timeline.get_timeline_interface()
        extension_path = get_extension_path("isaacsim.test.collection")
        usd_path = extension_path + "/data/tests/articulation_drives_opposite.usd"
        result, error = await stage_utils.open_stage_async(usd_path)
        # Make sure the stage loaded
        self.assertTrue(result)

        # get handle to each robot's chassis
        robot_joints_xform = XformPrim("/World/mock_robot/body")
        robot_articulation_xform = XformPrim("/World/mock_robot_with_articulation_root/body")

        # simulate for 60 frames
        timeline.play()
        for frame in range(120):
            await omni.kit.app.get_app().next_update_async()

        # compare position in the x direction
        positions_1, _ = robot_joints_xform.get_world_poses()
        positions_2, _ = robot_articulation_xform.get_world_poses()
        xpos_1 = positions_1.numpy()[0][0]
        xpos_2 = positions_2.numpy()[0][0]
        pos_diff = np.linalg.norm(xpos_1 - xpos_2)
        self.assertAlmostEqual(pos_diff, 0, delta=1)
        self.assertGreater(xpos_1, 2)
        self.assertGreater(xpos_2, 2)

    async def test_delete(self) -> None:
        """Test deleting articulations during simulation does not crash."""
        self._timeline = omni.timeline.get_timeline_interface()
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        prim_a = self._stage.DefinePrim("/World/Franka_1", "Xform")
        prim_a.GetReferences().AddReference(
            self._assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        prim_b = self._stage.DefinePrim("/World/Franka_2", "Xform")
        prim_b.GetReferences().AddReference(
            self._assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        with Sdf.ChangeBlock():
            omni.usd.commands.DeletePrimsCommand(["/World/Franka_1"]).do()
            omni.usd.commands.DeletePrimsCommand(["/World/Franka_2"]).do()
        await omni.kit.app.get_app().next_update_async()
