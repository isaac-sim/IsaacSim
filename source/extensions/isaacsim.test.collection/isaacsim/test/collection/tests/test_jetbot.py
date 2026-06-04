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

"""Tests for the NVIDIA JetBot robot simulation including loading, movement, acceleration, braking, spinning, and circular motion behaviors."""

import carb
import carb.tokens
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async

from .robot_helpers import init_robot_sim, setup_robot_og


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestJetBot(omni.kit.test.AsyncTestCase):
    """Tests for the NVIDIA JetBot robot simulation."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with JetBot robot."""
        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # add in jetbot (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        result, error = await open_stage_async(self.usd_path)
        # Make sure the stage loaded
        self.assertTrue(result)

        await omni.kit.app.get_app().next_update_async()

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

        # setup omnigraph
        self.graph_path = "/ActionGraph"
        graph, self.odom_node = setup_robot_og(
            self.graph_path, "left_wheel_joint", "right_wheel_joint", "/jetbot", 0.0335, 0.118
        )

    # After running each test
    async def tearDown(self):
        """Clean up test environment and stop timeline."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_loading(self):
        """Test that the JetBot robot loads and can move forward."""
        stage_utils.delete_prim("/ActionGraph")
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # get the jetbot
        self.ar = Articulation("/jetbot")
        # Wait for physics to be ready
        await omni.kit.app.get_app().next_update_async()
        starting_pos, _ = self.ar.get_world_poses()
        self.starting_pos = starting_pos.numpy()[0]
        dof_indices = self.ar.get_dof_indices(["left_wheel_joint", "right_wheel_joint"])
        self.ar.set_dof_velocity_targets(velocities=np.array([[1.0, 1.0]]), dof_indices=dof_indices)

        # move the jetbot
        for i in range(60):
            await omni.kit.app.get_app().next_update_async()

        current_pos, _ = self.ar.get_world_poses()
        self.current_pos = current_pos.numpy()[0]

        delta = np.linalg.norm(self.current_pos - self.starting_pos)
        print("Diff is ", delta)
        self.assertTrue(delta > 0.02)

    # general, slowly building up speed testcase
    # note, jetbot cannot exceed 0.42 m/s
    async def test_accel(self):
        """Test acceleration behavior with gradually increasing velocities."""
        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/jetbot")

        for x in range(1, 5):
            forward_velocity = x * 0.10
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            for i in range(15):
                await omni.kit.app.get_app().next_update_async()
            print(x, forward_velocity, og.DataView.get(odom_velocity)[0])
            if og.DataView.get(odom_ang_vel)[2] > 0.8:
                print("spinning out of control, linear velocity: " + str(forward_velocity))
                self._timeline.stop()
            else:
                self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], forward_velocity, delta=1e-1)
            await omni.kit.app.get_app().next_update_async()

        self._timeline.stop()

    # braking from different init speeds
    async def test_brake(self):
        """Test braking behavior from various initial velocities."""
        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/jetbot")
        for x in range(1, 5):
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            forward_velocity = x * 0.10
            angular_velocity = x * 0.10
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )
            for j in range(30):
                await omni.kit.app.get_app().next_update_async()
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(0.0)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(0.0)
            for j in range(10):
                await omni.kit.app.get_app().next_update_async()
            print(x, forward_velocity, og.DataView.get(odom_velocity)[0])
            print(x, angular_velocity, og.DataView.get(odom_ang_vel)[2])

            self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], 0.0, delta=5e-1)
            self.assertAlmostEqual(og.DataView.get(odom_ang_vel)[2], 0.0, delta=5e-1)

            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()

    async def test_spin(self):
        """Test spinning behavior at different angular velocities."""
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        for x in range(1, 6):
            # Start Simulation and wait
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            await init_robot_sim("/jetbot")

            angular_velocity = 0.6 * x
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )

            # wait until const velocity reached
            for i in range(30):
                await omni.kit.app.get_app().next_update_async()

            curr_ang_vel = float(og.DataView.get(odom_ang_vel)[2])
            self.assertAlmostEqual(curr_ang_vel, angular_velocity, delta=2e-1)

        self._timeline.stop()

    # go in circle
    async def test_circle(self):
        """Test circular motion and verify return to starting position."""
        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)
        odom_position = og.Controller.attribute("outputs:position", self.odom_node)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/jetbot")
        forward_velocity = -0.1
        angular_velocity = -0.5
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(forward_velocity)
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
            angular_velocity
        )
        for j in range(782):
            await omni.kit.app.get_app().next_update_async()
        self.assertAlmostEqual(og.DataView.get(odom_position)[0], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_position)[1], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], forward_velocity, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_ang_vel)[2], angular_velocity, delta=5e-2)

        await omni.kit.app.get_app().next_update_async()
