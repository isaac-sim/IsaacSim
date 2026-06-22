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

"""Tests for Nova Carter goal-driven navigation with path planning functionality."""

import carb
import carb.tokens
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
import usdrt.Sdf
from isaacsim.core.experimental.utils.app import get_extension_path
from isaacsim.core.experimental.utils.stage import open_stage_async
from isaacsim.core.experimental.utils.transform import quaternion_to_euler_angles
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, PhysicsSchemaTools

from .robot_helpers import (
    init_robot_sim,
    setup_robot_og,
)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDriveGoalCarterv2(omni.kit.test.AsyncTestCase):
    """Tests for Nova Carter goal-driven navigation with path planning."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with Nova Carter and navigation graph."""
        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self._extension_path = get_extension_path("isaacsim.test.collection")

        # add in carter (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        result, error = await open_stage_async(self.usd_path)
        PhysicsSchemaTools.addGroundPlane(
            omni.usd.get_context().get_stage(), "/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )

        # Make sure the stage loaded
        self.assertTrue(result)

        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60
        self._physics_dt = 1 / self._physics_rate

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)

        # Set physics dt and rendering dt via managers
        SimulationManager.set_physics_dt(self._physics_dt)
        RenderingManager.set_dt(self._physics_dt)

        await app_utils.update_app_async()

        self._stage = omni.usd.get_context().get_stage()

        await omni.kit.app.get_app().next_update_async()
        # setup omnigraph
        self.graph_path = "/ActionGraph"
        graph, _ = setup_robot_og(
            self.graph_path, "joint_wheel_left", "joint_wheel_right", "/nova_carter/chassis_link", 0.14, 0.4132
        )

        keys = og.Controller.Keys
        og.Controller.edit(
            graph,
            {
                keys.CREATE_NODES: [
                    ("GetPrimLocalToWorldTransform", "omni.graph.nodes.GetPrimLocalToWorldTransform"),
                    ("GetRotationQuaternion", "omni.graph.nodes.GetMatrix4Quaternion"),
                    ("GetTranslation", "omni.graph.nodes.GetMatrix4Translation"),
                    ("QuinticPathPlanner", "isaacsim.robot.wheeled_robots.QuinticPathPlanner"),
                    ("CheckGoal2D", "isaacsim.robot.wheeled_robots.CheckGoal2D"),
                    ("StanleyControlPID", "isaacsim.robot.wheeled_robots.StanleyControlPID"),
                ],
                keys.CONNECT: [
                    (self.graph_path + "/OnPlaybackTick.outputs:tick", "QuinticPathPlanner.inputs:execIn"),
                    ("QuinticPathPlanner.outputs:execOut", "CheckGoal2D.inputs:execIn"),
                    ("CheckGoal2D.outputs:execOut", "StanleyControlPID.inputs:execIn"),
                    (
                        "GetPrimLocalToWorldTransform.outputs:localToWorldTransform",
                        "GetRotationQuaternion.inputs:matrix",
                    ),
                    ("GetPrimLocalToWorldTransform.outputs:localToWorldTransform", "GetTranslation.inputs:matrix"),
                    ("GetRotationQuaternion.outputs:quaternion", "QuinticPathPlanner.inputs:currentOrientation"),
                    ("GetRotationQuaternion.outputs:quaternion", "CheckGoal2D.inputs:currentOrientation"),
                    ("GetRotationQuaternion.outputs:quaternion", "StanleyControlPID.inputs:currentOrientation"),
                    ("GetTranslation.outputs:translation", "QuinticPathPlanner.inputs:currentPosition"),
                    ("GetTranslation.outputs:translation", "CheckGoal2D.inputs:currentPosition"),
                    ("GetTranslation.outputs:translation", "StanleyControlPID.inputs:currentPosition"),
                    ("QuinticPathPlanner.outputs:pathArrays", "StanleyControlPID.inputs:pathArrays"),
                    ("QuinticPathPlanner.outputs:target", "CheckGoal2D.inputs:target"),
                    ("QuinticPathPlanner.outputs:target", "StanleyControlPID.inputs:target"),
                    ("QuinticPathPlanner.outputs:targetChanged", "CheckGoal2D.inputs:targetChanged"),
                    ("QuinticPathPlanner.outputs:targetChanged", "StanleyControlPID.inputs:targetChanged"),
                    ("CheckGoal2D.outputs:reachedGoal", "StanleyControlPID.inputs:reachedGoal"),
                    (
                        "StanleyControlPID.outputs:angularVelocity",
                        self.graph_path + "/DifferentialController.inputs:angularVelocity",
                    ),
                    (
                        "StanleyControlPID.outputs:linearVelocity",
                        self.graph_path + "/DifferentialController.inputs:linearVelocity",
                    ),
                    (self.graph_path + "/computeOdom.outputs:linearVelocity", "StanleyControlPID.inputs:currentSpeed"),
                ],
                keys.SET_VALUES: [
                    ("QuinticPathPlanner.inputs:targetPosition", (-5, -5, 0)),
                    ("QuinticPathPlanner.inputs:targetOrientation", (0, 0, 0, 1)),
                    ("GetPrimLocalToWorldTransform.inputs:usePath", False),
                    (
                        self.graph_path + "/computeOdom.inputs:chassisPrim",
                        [usdrt.Sdf.Path("/nova_carter/chassis_link")],
                    ),
                    ("GetPrimLocalToWorldTransform.inputs:prim", [usdrt.Sdf.Path("/nova_carter/chassis_link")]),
                ],
            },
        )

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_quintic_planner(self) -> None:
        """Test quintic polynomial path planner generates valid paths."""
        # Start Simulation and wait
        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        # get output values - path arrays (velocity, x, y, and yaw arrays concatenated into one), abbreviated target array (pos.x, pos.y, rot), and target changed bool
        pathArrays = og.Controller.get(
            og.Controller.attribute(self.graph_path + "/QuinticPathPlanner.outputs:pathArrays")
        )
        target = og.Controller.attribute(self.graph_path + "/QuinticPathPlanner.outputs:target").get()
        targetChanged = og.Controller.attribute(self.graph_path + "/QuinticPathPlanner.outputs:targetChanged").get()

        self.assertGreater(len(pathArrays), 0)  # check if path arrays were generated
        self.assertGreater(len(target), 0)  # check if target was generated
        print("target changed: ", targetChanged)
        self.assertTrue(targetChanged)  # target should be changed if first frame of simulation

        # use the following line for a test of full simulation - check that number of frames from simulation start to reaching goal is approx equal to length of each path array
        self.array_idx_len = int(len(pathArrays) / 4)

        self._timeline.stop()

        print("quintic passed")

    async def test_check_goal_2d(self) -> None:
        """Test 2D goal checking detects when robot reaches target."""
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/nova_carter")
        await omni.kit.app.get_app().next_update_async()

        # Get position and rotation data provided to node, then set target equal to pos/rot and check for reachedGoal booleans
        pos = og.Controller.attribute(self.graph_path + "/GetTranslation.outputs:translation").get()
        # Get quaternion in XYZW format and convert to WXYZ for experimental API
        quat_xyzw = og.Controller.attribute(self.graph_path + "/GetRotationQuaternion.outputs:quaternion").get()
        quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        euler = quaternion_to_euler_angles(quat_wxyz)
        rot = euler.numpy().flatten()[2]  # Extract yaw angle

        # disconnect target & targetChanged from QuinticPathPlanner to avoid overwriting artificial values
        og.Controller.disconnect(
            self.graph_path + "/QuinticPathPlanner.outputs:target", self.graph_path + "/CheckGoal2D.inputs:target"
        )
        og.Controller.disconnect(
            self.graph_path + "/QuinticPathPlanner.outputs:targetChanged",
            self.graph_path + "/CheckGoal2D.inputs:targetChanged",
        )

        # artificial target/targetChanged values
        og.Controller.attribute(self.graph_path + "/CheckGoal2D.inputs:targetChanged").set(True)
        og.Controller.attribute(self.graph_path + "/CheckGoal2D.inputs:target").set([pos[0], pos[1], rot])

        for i in range(500):
            await omni.kit.app.get_app().next_update_async()  # allow for check goal node to run and update outputs

        reached = og.Controller.get(og.Controller.attribute(self.graph_path + "/CheckGoal2D.outputs:reachedGoal"))
        self.assertTrue(
            reached[0]
        )  # reachedGoal output - corresponds to x and y position (as hypotenuse compared to threshold)
        self.assertTrue(reached[1])  # reachedGoal output - corresponds to z orientation

        self._timeline.stop()

        print("check goal passed")

    async def test_stanley_control_pid(self) -> None:
        """Test Stanley control provides valid steering commands."""
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/nova_carter")

        # allow time for carter to move from origin and start driving
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        # check stanley outputs, if != 0, stanley seems to be providing some steering control
        # (further tests of full simulation are needed to determine if carter ends up at target position)
        angular = og.Controller.attribute(self.graph_path + "/StanleyControlPID.outputs:angularVelocity").get()
        linear = og.Controller.attribute(self.graph_path + "/StanleyControlPID.outputs:linearVelocity").get()

        self.assertNotEqual(angular, 0)
        self.assertGreater(linear, 0)

        self._timeline.stop()

        print("stanley passed")
