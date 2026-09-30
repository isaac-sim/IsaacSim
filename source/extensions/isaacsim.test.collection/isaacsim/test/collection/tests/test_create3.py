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

"""Tests for iRobot Create3 robot simulation including loading, movement, acceleration, braking, spinning, and circular motion behaviors."""

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
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path_async

from .robot_helpers import (
    init_robot_sim,
    setup_robot_og,
    simulate_arc,
    wait_for_forward_displacement,
    wait_for_stable_odometry,
)


# Class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestCreate3(omni.kit.test.AsyncTestCase):
    """Tests for the iRobot Create3 robot simulation."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with Create3 robot."""

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # add in create3 (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots_Multiphysics/iRobot/Create3/create_3/create_3.usda"
        result, _ = await stage_utils.open_stage_async(self.usd_path)
        # Make sure the stage loaded
        self.assertTrue(result)

        await app_utils.update_app_async()

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

        # setup omnigraph
        self.graph_path = "/ActionGraph"
        graph, self.odom_node = setup_robot_og(
            self.graph_path,
            "left_wheel_joint",
            "right_wheel_joint",
            "/create3",
            0.03575,
            0.233,
        )

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        app_utils.stop()
        await app_utils.update_app_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_loading(self) -> None:
        """Test that the Create3 robot loads and can move forward."""
        stage_utils.delete_prim("/ActionGraph")
        # Start Simulation and wait
        app_utils.play()
        await app_utils.update_app_async()

        # get the create3
        self.ar = Articulation("/create3")
        # Wait for physics to be ready
        await app_utils.update_app_async()
        starting_pos, starting_orientation = self.ar.get_world_poses()
        self.starting_pos = starting_pos.numpy()[0]

        dof_indices = self.ar.get_dof_indices(["left_wheel_joint", "right_wheel_joint"])
        self.ar.set_dof_velocity_targets(velocities=np.array([[1.0, 1.0]]), dof_indices=dof_indices)

        await wait_for_forward_displacement(
            self.ar,
            self.starting_pos,
            starting_orientation.numpy()[0],
            minimum_distance=0.02,
        )

    # Building up speed tests
    async def test_accel(self) -> None:
        """Test acceleration behavior with gradually increasing velocities."""
        # Start sim and wait
        app_utils.play()
        await app_utils.update_app_async()

        await init_robot_sim("/create3")

        for x in range(1, 5):
            expected_velocity = x * 0.05
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                expected_velocity
            )
            result = await wait_for_stable_odometry(
                self.odom_node,
                expected_linear_velocity=expected_velocity,
                linear_tolerance=2e-2,
                max_updates=60,
            )
            self.assertLess(abs(result.angular_velocity), 0.1)

        app_utils.stop()

    # braking from different init speeds
    async def test_brake(self) -> None:
        """Test braking behavior from various initial velocities."""
        # Start Simulation and wait
        app_utils.play()
        await app_utils.update_app_async()

        await init_robot_sim("/create3")
        for x in range(1, 5):
            expected_velocity = x * 0.05
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                expected_velocity
            )
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                expected_velocity
            )
            moving_result = await wait_for_stable_odometry(
                self.odom_node,
                expected_linear_velocity=expected_velocity,
                expected_angular_velocity=expected_velocity,
                linear_tolerance=4e-2,
                angular_tolerance=4e-2,
                max_updates=60,
            )
            self.assertGreater(moving_result.linear_velocity, 0.02)
            self.assertGreater(moving_result.angular_velocity, 0.02)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(0.0)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(0.0)
            await wait_for_stable_odometry(
                self.odom_node,
                expected_linear_velocity=0.0,
                expected_angular_velocity=0.0,
                max_updates=20,
            )

        app_utils.stop()

    async def test_spin(self) -> None:
        """Test spinning behavior at different angular velocities."""
        app_utils.play()
        await app_utils.update_app_async()
        await init_robot_sim("/create3")

        for x in range(1, 6):
            expected_velocity = 0.25 * x
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                expected_velocity
            )
            result = await wait_for_stable_odometry(
                self.odom_node,
                expected_angular_velocity=expected_velocity,
                angular_tolerance=1.2e-1,
                max_updates=60,
            )
            self.assertGreater(result.angular_velocity, 0.1)

        app_utils.stop()

    # go in circle
    async def test_circle(self) -> None:
        """Test that the robot performs a quarter in-place rotation."""
        app_utils.play()
        await app_utils.update_app_async()

        await init_robot_sim("/create3")
        forward_velocity = -1e-5
        angular_velocity = 1.287
        result = await simulate_arc(
            self.graph_path,
            self.odom_node,
            "/create3",
            forward_velocity=forward_velocity,
            angular_velocity=angular_velocity,
            target_angle=0.5 * np.pi,
            max_updates=250,
        )

        self.assertLess(result.max_path_error, 5e-2)
        self.assertLess(result.max_displacement, 5e-2)
        np.testing.assert_allclose(result.position_delta, result.expected_position_delta, atol=5e-2)
        self.assertAlmostEqual(result.linear_velocity, 0.0, delta=5e-2)
        self.assertGreater(result.angular_velocity, 1.0)
