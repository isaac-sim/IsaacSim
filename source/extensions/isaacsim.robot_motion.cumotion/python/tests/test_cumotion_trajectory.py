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

"""Test suite for CumotionTrajectory class."""

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.cumotion as cu_mg
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from omni.kit.app import get_app


class TestCumotionTrajectoryFranka(omni.kit.test.AsyncTestCase):
    """Test suite for CumotionTrajectory with Franka robot."""

    async def setUp(self) -> None:
        """Set up test environment before each test."""
        await stage_utils.create_new_stage_async()
        await get_app().next_update_async()

        # Initialize timeline
        self._timeline = omni.timeline.get_timeline_interface()

        # Create cumotion robot configuration
        self.cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        # Create trajectory generator to create test trajectories
        self.trajectory_generator = cu_mg.TrajectoryGenerator(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
        )

    async def tearDown(self) -> None:
        """Clean up after each test."""
        # Stop timeline if running
        if self._timeline.is_playing():
            self._timeline.stop()

        await get_app().next_update_async()

    # ============================================================================
    # Test initialization
    # ============================================================================

    async def test_cumotion_trajectory_initialization(self) -> None:
        """Test that CumotionTrajectory initializes correctly with valid inputs."""
        # Create a trajectory using the generator
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        generator_trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        # Extract the underlying cumotion trajectory
        cumotion_traj = generator_trajectory._trajectory

        # Create CumotionTrajectory directly
        trajectory = cu_mg.CumotionTrajectory(
            trajectory=cumotion_traj,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_robot=self.cumotion_robot,
        )

        self.assertIsNotNone(trajectory)
        self.assertGreater(trajectory.duration, 0.0)

    async def test_cumotion_trajectory_invalid_joint_space(self) -> None:
        """Test that initialization fails when controlled joints are not a subset of robot_joint_space."""
        # Create a trajectory using the generator
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        generator_trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        # Extract the underlying cumotion trajectory
        cumotion_traj = generator_trajectory._trajectory

        # Try to create with invalid joint space (doesn't contain all controlled joints)
        invalid_joint_space = ["joint1", "joint2"]

        with self.assertRaises(ValueError):
            cu_mg.CumotionTrajectory(
                trajectory=cumotion_traj,
                robot_joint_space=invalid_joint_space,
                cumotion_robot=self.cumotion_robot,
            )

    async def test_cumotion_trajectory_with_device(self) -> None:
        """Test that CumotionTrajectory can be initialized with a device parameter."""
        # Create a trajectory using the generator
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        generator_trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        # Extract the underlying cumotion trajectory
        cumotion_traj = generator_trajectory._trajectory

        # Create with device parameter
        device = wp.get_device("cuda:0") if wp.is_cuda_available() else wp.get_device("cpu")
        trajectory = cu_mg.CumotionTrajectory(
            trajectory=cumotion_traj,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_robot=self.cumotion_robot,
            device=device,
        )

        self.assertIsNotNone(trajectory)
        self.assertGreater(trajectory.duration, 0.0)

    # ============================================================================
    # Test properties and methods
    # ============================================================================

    async def test_duration_property(self) -> None:
        """Test that duration property returns correct value."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        duration = trajectory.duration
        self.assertGreater(duration, 0.0)
        self.assertIsInstance(duration, float)

    async def test_get_active_joints(self) -> None:
        """Test that get_active_joints returns correct joint names."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        active_joints = trajectory.get_active_joints()
        self.assertEqual(active_joints, self.cumotion_robot.controlled_joint_names)
        self.assertIsInstance(active_joints, list)

    # ============================================================================
    # Test get_target_state edge cases
    # ============================================================================

    async def test_get_target_state_time_boundaries(self) -> None:
        """Test get_target_state at exact time boundaries."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        duration = trajectory.duration

        # At time 0.0 (should return state)
        state_at_zero = trajectory.get_target_state(0.0)
        self.assertIsNotNone(state_at_zero)
        self.assertIsNotNone(state_at_zero.joints.positions)
        self.assertIsNotNone(state_at_zero.joints.velocities)

        # At time duration (should return state)
        state_at_end = trajectory.get_target_state(duration)
        self.assertIsNotNone(state_at_end)
        self.assertIsNotNone(state_at_end.joints.positions)
        self.assertIsNotNone(state_at_end.joints.velocities)

        # Just before 0.0 (should return None)
        state_before = trajectory.get_target_state(-1e-10)
        self.assertIsNone(state_before)

        # Just after duration (should return None)
        state_after = trajectory.get_target_state(duration + 1e-10)
        self.assertIsNone(state_after)

    async def test_get_target_state_at_midpoint(self) -> None:
        """Test get_target_state at midpoint returns valid state."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        # At midpoint
        midpoint_time = trajectory.duration / 2.0
        state_mid = trajectory.get_target_state(midpoint_time)

        self.assertIsNotNone(state_mid)
        self.assertIsNotNone(state_mid.joints)
        self.assertIsNotNone(state_mid.joints.positions)
        self.assertIsNotNone(state_mid.joints.velocities)

        # Verify positions and velocities are valid arrays
        positions = state_mid.joints.positions.numpy()
        velocities = state_mid.joints.velocities.numpy()

        self.assertEqual(len(positions), len(self.cumotion_robot.controlled_joint_names))
        self.assertEqual(len(velocities), len(self.cumotion_robot.controlled_joint_names))
        self.assertTrue(np.all(np.isfinite(positions)))
        self.assertTrue(np.all(np.isfinite(velocities)))

    async def test_get_target_state_returns_robot_state_structure(self) -> None:
        """Test that get_target_state returns properly structured RobotState."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = q_start.copy()
        q_end[1] = np.pi / 4

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        state = trajectory.get_target_state(0.0)

        self.assertIsNotNone(state)
        self.assertIsInstance(state, mg.RobotState)
        self.assertIsNotNone(state.joints)
        self.assertIsNone(state.root)
        self.assertIsNone(state.links)
        self.assertIsNone(state.sites)

        # Verify joint state structure
        self.assertEqual(len(state.joints.position_names), len(self.cumotion_robot.controlled_joint_names))
        self.assertEqual(len(state.joints.velocity_names), len(self.cumotion_robot.controlled_joint_names))
        self.assertEqual(state.joints.position_names, self.cumotion_robot.controlled_joint_names)
        self.assertEqual(state.joints.velocity_names, self.cumotion_robot.controlled_joint_names)
