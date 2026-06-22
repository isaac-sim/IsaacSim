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

"""Test suite for O3dyn omnidirectional robot simulation including loading, movement, and reference testing."""

import carb
import carb.tokens
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils.app import get_extension_path
from isaacsim.core.experimental.utils.stage import open_stage_async
from isaacsim.core.experimental.utils.transform import quaternion_to_euler_angles
from isaacsim.storage.native import get_assets_root_path_async


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestO3dyn(omni.kit.test.AsyncTestCase):
    """Tests for the O3dyn omnidirectional robot simulation."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with O3dyn robot asset path."""
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self._extension_path = get_extension_path("isaacsim.test.collection")
        ## setup carter_v1:
        # open local carter_v1:
        # (result, error) = await omni.usd.get_context().open_stage_async(
        #     self._extension_path + "/data/tests/carter_v1.usd"
        # )

        # add in carter (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/Fraunhofer/O3dyn/o3dyn.usd"

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_loading(self) -> None:
        """Test that the O3dyn robot loads and settles at expected position."""
        result, error = await open_stage_async(self.usd_path)

        stage = omni.usd.get_context().get_stage()

        # Make sure the stage loaded
        self.assertTrue(result)

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

        self._timeline.play()
        for i in range(150):
            await omni.kit.app.get_app().next_update_async()
        base_link_path = str(stage.GetDefaultPrim().GetPath().AppendPath("base_link"))
        positions, _ = XformPrim(base_link_path).get_world_poses()
        translate = positions.numpy()[0]
        self.assertAlmostEqual(translate[0], 0.00, delta=0.01)
        self.assertAlmostEqual(translate[1], 0.00, delta=0.01)

        self.assertAlmostEqual(translate[2], -0.01, delta=0.01)
        self._timeline.stop()

    # general, slowly building up speed testcase
    async def test_add_as_reference(self) -> None:
        """Test loading O3dyn as a USD reference with ground plane."""
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)

        GroundPlane("/World/groundPlane", sizes=1000.0, positions=[[0.0, 0.0, -0.12]], colors=[1.0, 1.0, 1.0])
        await app_utils.update_app_async()

        self._timeline.play()
        for i in range(120):
            await omni.kit.app.get_app().next_update_async()

        base_link_path = str(robot_prim.GetPath().AppendPath("base_link"))
        positions, _ = XformPrim(base_link_path).get_world_poses()
        translate = positions.numpy()[0]
        self.assertAlmostEqual(translate[0], 0.00, delta=0.01)
        self.assertAlmostEqual(translate[1], 0.00, delta=0.01)

        self.assertAlmostEqual(translate[2], -0.05, delta=0.01)
        self._timeline.stop()

    async def test_move_forward(self) -> None:
        """Test O3dyn moves forward when all wheels rotate in same direction."""
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/groundPlane", sizes=1000.0, positions=[[0.0, 0.0, -0.12]], colors=[1.0, 1.0, 1.0])
        await app_utils.update_app_async()

        for prim in stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("wheel_drive")).GetChildren():
            prim.GetAttribute("drive:angular:physics:targetVelocity").Set(100)
        self._timeline.play()
        for i in range(300):
            await omni.kit.app.get_app().next_update_async()

        base_link_path = str(robot_prim.GetPath().AppendPath("base_link"))
        positions, _ = XformPrim(base_link_path).get_world_poses()
        translate = positions.numpy()[0]
        self.assertGreater(translate[0], 1.0)
        self.assertAlmostEqual(translate[1], 0.00, delta=0.02)
        self._timeline.stop()

    async def test_move_sideways(self) -> None:
        """Test O3dyn moves sideways using mecanum wheel strafing."""
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/groundPlane", sizes=1000.0, positions=[[0.0, 0.0, -0.12]], colors=[1.0, 1.0, 1.0])
        await app_utils.update_app_async()

        for prim in stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("wheel_drive")).GetChildren():
            if prim.GetName() in ["wheel_fr_joint", "wheel_rl_joint"]:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(100)
            else:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(-100)
        self._timeline.play()
        for i in range(300):
            await omni.kit.app.get_app().next_update_async()

        base_link_path = str(robot_prim.GetPath().AppendPath("base_link"))
        positions, _ = XformPrim(base_link_path).get_world_poses()
        translate = positions.numpy()[0]
        self.assertAlmostEqual(translate[0], 0.00, delta=0.1)
        self.assertGreater(
            translate[1],
            1.00,
        )
        self._timeline.stop()

    async def test_rotate(self) -> None:
        """Test O3dyn rotates in place using differential wheel speeds."""
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/groundPlane", sizes=1000.0, positions=[[0.0, 0.0, -0.12]], colors=[1.0, 1.0, 1.0])
        await app_utils.update_app_async()

        for prim in stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("wheel_drive")).GetChildren():
            if prim.GetName() in ["wheel_fl_joint", "wheel_rl_joint"]:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(150)
            else:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(-150)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # TODO: regenerate goldens
        for i in range(298):
            await omni.kit.app.get_app().next_update_async()

        base_link_path = str(robot_prim.GetPath().AppendPath("base_link"))
        positions, orientations = XformPrim(base_link_path).get_world_poses()
        translate = positions.numpy()[0]
        # Robot origin is not at center of rotation, give it some slack on X/Y
        self.assertLess(
            abs(translate[0]),
            0.3,
        )
        self.assertLess(
            abs(translate[1]),
            0.3,
        )
        # Get rotation angle from quaternion using euler angles (Z rotation = yaw)
        euler_angles = quaternion_to_euler_angles(orientations, degrees=True)
        rotation = abs(euler_angles.numpy()[0][2])  # Z rotation (yaw) in degrees
        self.assertGreater(
            rotation,
            45,
        )
        self._timeline.stop()
