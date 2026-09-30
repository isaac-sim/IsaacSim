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

import math

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation, XformPrim
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

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.usd_path = self._assets_root_path + "/Isaac/Robots_Multiphysics/Fraunhofer/O3dyn/o3dyn.usda"

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def _step(self, frame_count: int) -> None:
        for _ in range(frame_count):
            await omni.kit.app.get_app().next_update_async()

    @staticmethod
    def _set_wheel_velocities(wheel_prims, velocity_by_name: dict[str, float], default: float = 0.0) -> None:
        for prim in wheel_prims:
            prim.GetAttribute("drive:angular:physics:targetVelocity").Set(velocity_by_name.get(prim.GetName(), default))

    @staticmethod
    def _local_xy_displacement(displacement, start_yaw: float) -> tuple[float, float]:
        yaw = math.radians(start_yaw)
        return (
            math.cos(yaw) * displacement[0] + math.sin(yaw) * displacement[1],
            -math.sin(yaw) * displacement[0] + math.cos(yaw) * displacement[1],
        )

    def _stop_robot(self, robot: Articulation, wheel_prims) -> None:
        self._set_wheel_velocities(wheel_prims, {})
        robot.set_dof_velocities([0.0])
        robot.set_velocities(linear_velocities=[0.0, 0.0, 0.0], angular_velocities=[0.0, 0.0, 0.0])

    async def test_loading_reference_and_motion(self) -> None:
        """Test O3dyn loading, referencing, and motion behaviors in sequence."""
        result, error = await open_stage_async(self.usd_path)
        stage = omni.usd.get_context().get_stage()

        with self.subTest("direct stage loading"):
            self.assertTrue(result, error)

        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

        self._timeline.play()
        await self._step(150)
        base_link_path = str(stage.GetDefaultPrim().GetPath().AppendPath("base_link"))
        positions, _ = XformPrim(base_link_path).get_world_poses()
        translate = positions.numpy()[0]
        with self.subTest("direct stage settling"):
            self.assertAlmostEqual(translate[0], 0.00, delta=0.01)
            self.assertAlmostEqual(translate[1], 0.00, delta=0.01)
            self.assertAlmostEqual(translate[2], -0.01, delta=0.01)
        self._timeline.stop()

        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")
        robot_prim.GetReferences().AddReference(self.usd_path)

        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/groundPlane", sizes=1000.0, positions=[[0.0, 0.0, -0.12]], colors=[1.0, 1.0, 1.0])
        robot = Articulation(str(robot_prim.GetPath()))
        await app_utils.update_app_async()

        self._timeline.play()
        await self._step(120)

        base_link_path = str(robot_prim.GetPath().AppendPath("base_link"))
        base_link = XformPrim(base_link_path)
        positions, orientations = base_link.get_world_poses()
        translate = positions.numpy()[0]
        with self.subTest("reference loading"):
            self.assertAlmostEqual(translate[0], 0.00, delta=0.01)
            self.assertAlmostEqual(translate[1], 0.00, delta=0.01)
            self.assertAlmostEqual(translate[2], -0.05, delta=0.01)

        wheel_prims = stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("wheel_drive")).GetChildren()

        forward_start = translate.copy()
        forward_start_yaw = quaternion_to_euler_angles(orientations, degrees=True).numpy()[0][2]
        self._set_wheel_velocities(wheel_prims, {}, default=100.0)
        await self._step(300)
        positions, _ = base_link.get_world_poses()
        forward_displacement = self._local_xy_displacement(positions.numpy()[0] - forward_start, forward_start_yaw)
        with self.subTest("forward motion"):
            self.assertGreater(forward_displacement[0], 1.0)
            self.assertGreater(forward_displacement[0], 10.0 * abs(forward_displacement[1]))

        self._stop_robot(robot, wheel_prims)
        await self._step(1)
        positions, orientations = base_link.get_world_poses()
        sideways_start = positions.numpy()[0]
        sideways_start_yaw = quaternion_to_euler_angles(orientations, degrees=True).numpy()[0][2]
        self._set_wheel_velocities(
            wheel_prims,
            {"wheel_fr_joint": 100.0, "wheel_rl_joint": 100.0},
            default=-100.0,
        )
        await self._step(300)
        positions, _ = base_link.get_world_poses()
        sideways_displacement = self._local_xy_displacement(positions.numpy()[0] - sideways_start, sideways_start_yaw)
        with self.subTest("sideways motion"):
            self.assertGreater(sideways_displacement[1], 1.00)
            self.assertGreater(sideways_displacement[1], 10.0 * abs(sideways_displacement[0]))

        self._stop_robot(robot, wheel_prims)
        await self._step(1)
        positions, orientations = base_link.get_world_poses()
        rotation_start = positions.numpy()[0]
        rotation_start_yaw = quaternion_to_euler_angles(orientations, degrees=True).numpy()[0][2]
        self._set_wheel_velocities(
            wheel_prims,
            {"wheel_fl_joint": 150.0, "wheel_rl_joint": 150.0},
            default=-150.0,
        )
        await self._step(299)
        positions, orientations = base_link.get_world_poses()
        rotation_displacement = positions.numpy()[0] - rotation_start
        rotation_end_yaw = quaternion_to_euler_angles(orientations, degrees=True).numpy()[0][2]
        rotation = abs((rotation_end_yaw - rotation_start_yaw + 180.0) % 360.0 - 180.0)
        with self.subTest("rotation"):
            # Robot origin is not at center of rotation, give it some slack on X/Y.
            self.assertLess(abs(rotation_displacement[0]), 0.3)
            self.assertLess(abs(rotation_displacement[1]), 0.3)
            self.assertGreater(rotation, 45.0)

        self._timeline.stop()
