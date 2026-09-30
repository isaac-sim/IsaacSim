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

"""Tests for the Nova Carter (Carter v2) robot simulation including movement, acceleration, and navigation behaviors."""

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
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path_async

from .robot_helpers import (
    init_robot_sim,
    setup_robot_og,
    simulate_arc,
    wait_for_forward_displacement,
    wait_for_stable_odometry,
)


async def ramp_velocity(forward_velocity: float, angular_velocity: float, ramp_frames: int, graph_path: str) -> None:
    """Gradually ramp velocity commands over multiple frames.

    Args:
        forward_velocity: Target linear velocity to ramp to.
        angular_velocity: Target angular velocity to ramp to.
        ramp_frames: Number of frames over which to ramp the velocity.
        graph_path: USD path to the action graph.
    """
    for i in range(ramp_frames):
        og.Controller.attribute(graph_path + "/DifferentialController.inputs:linearVelocity").set(
            forward_velocity * ((i + 1) / ramp_frames)
        )
        og.Controller.attribute(graph_path + "/DifferentialController.inputs:angularVelocity").set(
            angular_velocity * ((i + 1) / ramp_frames)
        )
        await app_utils.update_app_async()


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestCarterv2(omni.kit.test.AsyncTestCase):
    """Tests for the Nova Carter (Carter v2) robot simulation."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with Nova Carter robot."""

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # add in carter (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        result, _ = await stage_utils.open_stage_async(self.usd_path)

        # Make sure the stage loaded
        self.assertTrue(result)
        await app_utils.update_app_async()
        GroundPlane("/groundPlane", sizes=[3000.0], colors=[[0.5, 0.5, 0.5]], templates=None)

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

        # setup omnigraph
        self.graph_path = "/ActionGraph"
        graph, self.odom_node = setup_robot_og(
            self.graph_path,
            "joint_wheel_left",
            "joint_wheel_right",
            "/nova_carter/chassis_link",
            0.14,
            0.4132,
        )

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        app_utils.stop(commit=False)
        await app_utils.update_app_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_loading(self) -> None:
        """Test that the Nova Carter robot loads and can move forward."""
        stage_utils.delete_prim("/ActionGraph")
        # Start Simulation and wait
        app_utils.play(commit=False)
        await app_utils.update_app_async()

        # get the robot articulation
        self.ar = Articulation("/nova_carter")
        # Wait for physics to be ready
        await app_utils.update_app_async()
        starting_pos, starting_orientation = self.ar.get_world_poses()
        self.starting_pos = starting_pos.numpy()[0]
        dof_indices = self.ar.get_dof_indices(["joint_wheel_left", "joint_wheel_right"])
        self.ar.set_dof_velocity_targets(velocities=np.array([[1.0, 1.0]]), dof_indices=dof_indices)

        await wait_for_forward_displacement(
            self.ar,
            self.starting_pos,
            starting_orientation.numpy()[0],
            minimum_distance=0.02,
        )

    # general, slowly building up speed testcase
    async def test_accel(self) -> None:
        """Test acceleration behavior with gradually increasing velocities."""
        # Start Simulation and wait
        app_utils.play(commit=False)
        await app_utils.update_app_async()

        await init_robot_sim("/nova_carter")

        for x in range(1, 5):
            forward_velocity = x * 0.15
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            result = await wait_for_stable_odometry(
                self.odom_node,
                expected_linear_velocity=forward_velocity,
                max_updates=15,
            )
            self.assertLess(abs(result.angular_velocity), 0.8)

        app_utils.stop(commit=False)

    # braking from different init speeds
    async def test_brake(self) -> None:
        """Test braking behavior from various initial velocities."""
        # Start Simulation and wait
        app_utils.play(commit=False)
        await app_utils.update_app_async()

        await init_robot_sim("/nova_carter")
        for x in range(1, 5):
            app_utils.play(commit=False)
            await app_utils.update_app_async()
            forward_velocity = x * 0.15
            angular_velocity = x * 0.15
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )
            moving_result = await wait_for_stable_odometry(
                self.odom_node,
                expected_linear_velocity=forward_velocity,
                expected_angular_velocity=angular_velocity,
                linear_tolerance=1e-1,
                angular_tolerance=1e-1,
                max_updates=30,
            )
            self.assertGreater(moving_result.linear_velocity, 0.04)
            self.assertGreater(moving_result.angular_velocity, 0.04)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(0.0)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(0.0)
            await wait_for_stable_odometry(
                self.odom_node,
                expected_linear_velocity=0.0,
                expected_angular_velocity=0.0,
                max_updates=30,
            )

            app_utils.stop(commit=False)
            await app_utils.update_app_async()

    async def test_spin(self) -> None:
        """Test spinning behavior at different angular velocities."""
        # Start Simulation and wait
        app_utils.play(commit=False)
        await app_utils.update_app_async()
        await init_robot_sim("/nova_carter")

        velocity_tolerance = 1e-1
        for x in range(1, 3):
            angular_velocity = 0.6 * x
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )

            await wait_for_stable_odometry(
                self.odom_node,
                expected_angular_velocity=angular_velocity,
                angular_tolerance=velocity_tolerance,
                max_updates=200,
            )

    # go in circle
    async def test_circle(self) -> None:
        """Test that the robot follows a quarter-circle trajectory."""
        app_utils.play(commit=False)
        await app_utils.update_app_async()

        await init_robot_sim("/nova_carter")
        forward_velocity = -0.2
        angular_velocity = -1.0
        result = await simulate_arc(
            self.graph_path,
            self.odom_node,
            "/nova_carter",
            forward_velocity=forward_velocity,
            angular_velocity=angular_velocity,
            target_angle=0.5 * np.pi,
            max_updates=250,
        )

        self.assertLess(result.max_path_error, 0.1)
        self.assertGreater(result.max_displacement, 0.2)
        np.testing.assert_allclose(result.position_delta, result.expected_position_delta, atol=0.1)
        self.assertAlmostEqual(result.linear_velocity, forward_velocity, delta=5e-2)
        self.assertAlmostEqual(result.angular_velocity, angular_velocity, delta=2e-1)
