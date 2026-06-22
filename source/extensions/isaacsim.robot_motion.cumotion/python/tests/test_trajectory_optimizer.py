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

"""Verifies cuMotion trajectory optimizer setup and planning for Franka joint-space, task-space, goal-set, constrained-path, and obstacle-aware targets. The tests also cover invalid inputs, configuration access, and returned trajectory properties."""

import cumotion
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.cumotion as cu_mg
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim
from omni.kit.app import get_app


class TestTrajectoryOptimizerFranka(omni.kit.test.AsyncTestCase):
    """Test suite for TrajectoryOptimizer with Franka robot."""

    async def setUp(self) -> None:
        """Set up test environment before each test."""
        await stage_utils.create_new_stage_async()
        await get_app().next_update_async()

        # Initialize timeline
        self._timeline = omni.timeline.get_timeline_interface()

        # Create cumotion robot configuration
        self.cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        # Create world interface
        self.world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
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
    async def test_trajectory_optimizer_initialization(self) -> None:
        """Test that TrajectoryOptimizer initializes correctly."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        self.assertIsNotNone(trajectory_optimizer)
        self.assertIsNotNone(trajectory_optimizer.get_trajectory_optimizer_config())
        self.assertIsNotNone(trajectory_optimizer.get_cumotion_robot())

    async def test_trajectory_optimizer_with_custom_tool_frame(self) -> None:
        """Test initialization with explicitly specified tool frame."""
        tool_frames = self.cumotion_robot.robot_description.tool_frame_names()
        self.assertGreater(len(tool_frames), 0)

        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            tool_frame=tool_frames[0],
        )

        self.assertIsNotNone(trajectory_optimizer)

    async def test_trajectory_optimizer_with_default_tool_frame(self) -> None:
        """Test initialization with default tool frame (None specified)."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            tool_frame=None,
        )

        self.assertIsNotNone(trajectory_optimizer)

    async def test_trajectory_optimizer_invalid_joint_space(self) -> None:
        """Test that invalid joint space raises ValueError."""
        # Create a joint space that doesn't contain all controlled joints
        invalid_joint_space = ["joint1", "joint2"]  # Not a superset of controlled joints

        with self.assertRaises(ValueError):
            cu_mg.TrajectoryOptimizer(
                cumotion_robot=self.cumotion_robot,
                robot_joint_space=invalid_joint_space,
                cumotion_world_interface=self.world_binding.get_world_interface(),
            )

    # TODO: GET A CUSTOM CONFIG FILE TO TEST.
    # async def test_trajectory_optimizer_with_custom_config_file(self):
    #     """Test initialization with custom configuration file."""
    #     # Note: This test assumes a config file exists. If not, it will use defaults.
    #     trajectory_optimizer = cu_mg.TrajectoryOptimizer(
    #         cumotion_robot=self.cumotion_robot,
    #         robot_joint_space=self.cumotion_robot.controlled_joint_names,
    #         cumotion_world_interface=self.world_binding.get_world_interface(),
    #         trajectory_optimizer_config_filename="trajectory_optimizer_config.yaml",
    #     )

    #     self.assertIsNotNone(trajectory_optimizer)

    # async def test_trajectory_optimizer_with_relative_config_path(self):
    #     """Test initialization with relative configuration file path (auto-detected)."""
    #     # Test that relative paths are automatically resolved relative to directory
    #     trajectory_optimizer = cu_mg.TrajectoryOptimizer(
    #         cumotion_robot=self.cumotion_robot,
    #         robot_joint_space=self.cumotion_robot.controlled_joint_names,
    #         cumotion_world_interface=self.world_binding.get_world_interface(),
    #         trajectory_optimizer_config_filename=None,  # Use defaults
    #     )

    #     self.assertIsNotNone(trajectory_optimizer)

    # async def test_trajectory_optimizer_with_absolute_config_path(self):
    #     """Test initialization with absolute configuration file path (auto-detected)."""
    #     import pathlib

    #     # Create a test config file path (absolute)
    #     config_file_relative = self.cumotion_robot.directory / "trajectory_optimizer_config.yaml"
    #     if not config_file_relative.exists():
    #         # If the relative config file doesn't exist, test with None (defaults)
    #         trajectory_optimizer = cu_mg.TrajectoryOptimizer(
    #             cumotion_robot=self.cumotion_robot,
    #             robot_joint_space=self.cumotion_robot.controlled_joint_names,
    #             cumotion_world_interface=self.world_binding.get_world_interface(),
    #             trajectory_optimizer_config_filename=None,
    #         )
    #         self.assertIsNotNone(trajectory_optimizer)
    #         return

    #     # Test with absolute path - should be auto-detected as absolute
    #     trajectory_optimizer = cu_mg.TrajectoryOptimizer(
    #         cumotion_robot=self.cumotion_robot,
    #         robot_joint_space=self.cumotion_robot.controlled_joint_names,
    #         cumotion_world_interface=self.world_binding.get_world_interface(),
    #         trajectory_optimizer_config_filename=str(config_file_relative.absolute()),
    #     )

    #     self.assertIsNotNone(trajectory_optimizer)

    # ============================================================================
    # Test plan_to_goal with CSpaceTarget
    # ============================================================================

    async def test_plan_to_cspace_target(self) -> None:
        """Test planning to a configuration space target."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 4

        q_target = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_target[1] = -np.pi / 2

        # Create CSpaceTarget with target configuration
        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, cspace_target)

        # Note: Planning may fail, so we just check it doesn't crash
        # If successful, trajectory should be a CumotionTrajectory
        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)

    async def test_plan_to_cspace_target_numpy_array(self) -> None:
        """Test planning with numpy array input."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 4

        q_target = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_target[1] = -np.pi / 2

        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, cspace_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    async def test_plan_to_cspace_target_warp_array(self) -> None:
        """Test planning with warp array input."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial_np = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_initial_np[1] = -np.pi / 4
        q_initial_wp = wp.array(q_initial_np, dtype=wp.float32)

        q_target_np = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_target_np[1] = -np.pi / 2

        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target_np,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial_wp, cspace_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    async def test_plan_to_cspace_target_list(self) -> None:
        """Test planning with list input."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 4
        q_initial_list = q_initial.tolist()

        q_target = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_target[1] = -np.pi / 2
        q_target_list = q_target.tolist()

        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target_list,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial_list, cspace_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    # ============================================================================
    # Test plan_to_goal with TaskSpaceTarget
    # ============================================================================

    async def test_plan_to_task_space_target(self) -> None:
        """Test planning to a task space target."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        # Get initial pose
        initial_pose = kinematics.pose(q_initial, tool_frame_name)

        # Create a reachable target position
        target_position = initial_pose.translation + np.array([0.1, 0.0, 0.0])

        # Create TaskSpaceTarget
        task_space_target = cumotion.TrajectoryOptimizer.TaskSpaceTarget(
            translation_constraint=cumotion.TrajectoryOptimizer.TranslationConstraint.target(target_position),
            orientation_constraint=cumotion.TrajectoryOptimizer.OrientationConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, task_space_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)

        # Verify final pose is close to target
        final_state = trajectory.get_target_state(trajectory.duration)
        if final_state is not None:
            final_pose = kinematics.pose(final_state.joints.positions.numpy(), tool_frame_name)
            self.assertTrue(
                np.allclose(target_position, final_pose.translation, atol=0.05),
                f"Target: {target_position}, Final: {final_pose.translation}",
            )

    async def test_plan_to_task_space_target_with_orientation(self) -> None:
        """Test planning to a task space target with orientation constraint."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        initial_pose = kinematics.pose(q_initial, tool_frame_name)
        target_position = initial_pose.translation + np.array([0.05, 0.0, 0.0])
        target_orientation = initial_pose.rotation

        # Create TaskSpaceTarget with orientation
        task_space_target = cumotion.TrajectoryOptimizer.TaskSpaceTarget(
            translation_constraint=cumotion.TrajectoryOptimizer.TranslationConstraint.target(target_position),
            orientation_constraint=cumotion.TrajectoryOptimizer.OrientationConstraint.terminal_target(
                target_orientation
            ),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, task_space_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    async def test_plan_to_task_space_target_with_linear_path_constraint(self) -> None:
        """Test planning with linear translation path constraint."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        initial_pose = kinematics.pose(q_initial, tool_frame_name)
        target_position = initial_pose.translation + np.array([0.08, 0.0, 0.0])

        # Create TaskSpaceTarget with linear path constraint
        task_space_target = cumotion.TrajectoryOptimizer.TaskSpaceTarget(
            translation_constraint=cumotion.TrajectoryOptimizer.TranslationConstraint.linear_path_constraint(
                target_position
            ),
            orientation_constraint=cumotion.TrajectoryOptimizer.OrientationConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, task_space_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    # ============================================================================
    # Test plan_to_goal with TaskSpaceTargetGoalset
    # ============================================================================

    async def test_plan_to_task_space_goalset(self) -> None:
        """Test planning to a task space goalset."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        initial_pose = kinematics.pose(q_initial, tool_frame_name)

        # Create multiple target positions
        target_positions = [
            initial_pose.translation + np.array([0.05, 0.0, 0.0]),
            initial_pose.translation + np.array([0.0, 0.05, 0.0]),
            initial_pose.translation + np.array([0.0, 0.0, 0.05]),
        ]

        # Create TaskSpaceTargetGoalset
        task_space_goalset = cumotion.TrajectoryOptimizer.TaskSpaceTargetGoalset(
            translation_constraints=cumotion.TrajectoryOptimizer.TranslationConstraintGoalset.target(target_positions),
            orientation_constraints=cumotion.TrajectoryOptimizer.OrientationConstraintGoalset.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, task_space_goalset)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    # ============================================================================
    # Test error handling
    # ============================================================================

    async def test_plan_to_goal_invalid_goal_type(self) -> None:
        """Test that invalid goal type raises ValueError."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()

        # Pass an invalid goal type
        invalid_goal = "not a valid goal"

        with self.assertRaises(ValueError):
            trajectory_optimizer.plan_to_goal(q_initial, invalid_goal)

    async def test_plan_to_goal_invalid_initial_position_dimensions(self) -> None:
        """Test that invalid initial position dimensions raise RuntimeError."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        # Wrong number of joints for initial position
        q_initial_wrong = np.array([0.0, 1.0, 2.0])

        # Use a valid target configuration
        q_target = self.cumotion_robot.robot_description.default_cspace_configuration()
        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        with self.assertRaises(ValueError):
            trajectory_optimizer.plan_to_goal(q_initial_wrong, cspace_target)

    async def test_plan_to_goal_planning_failure_returns_none(self) -> None:
        """Test that planning failures return None instead of raising exceptions."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()

        # Create an unreachable target (very far away)
        unreachable_position = np.array([10.0, 10.0, 10.0])

        task_space_target = cumotion.TrajectoryOptimizer.TaskSpaceTarget(
            translation_constraint=cumotion.TrajectoryOptimizer.TranslationConstraint.target(unreachable_position),
            orientation_constraint=cumotion.TrajectoryOptimizer.OrientationConstraint.none(),
        )

        # Should return None, not raise an exception
        trajectory = trajectory_optimizer.plan_to_goal(q_initial, task_space_target)
        self.assertIsNone(trajectory)

    # ============================================================================
    # Test configuration access
    # ============================================================================

    async def test_get_cumotion_robot(self) -> None:
        """Test getting the robot configuration."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        config = trajectory_optimizer.get_cumotion_robot()
        self.assertIsNotNone(config)
        self.assertEqual(config, self.cumotion_robot)

    async def test_get_trajectory_optimizer_config(self) -> None:
        """Test getting the trajectory optimizer configuration."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        config = trajectory_optimizer.get_trajectory_optimizer_config()
        self.assertIsNotNone(config)
        self.assertIsInstance(config, cumotion.TrajectoryOptimizerConfig)

    async def test_modify_trajectory_optimizer_config(self) -> None:
        """Test modifying trajectory optimizer configuration parameters."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        config = trajectory_optimizer.get_trajectory_optimizer_config()

        # Modify a parameter
        config.set_param(
            "enable_self_collision",
            cumotion.TrajectoryOptimizerConfig.ParamValue(False),
        )

        # Verify config is still valid
        self.assertIsNotNone(config)

    # ============================================================================
    # Test planning with obstacles
    # ============================================================================

    async def test_plan_to_goal_with_obstacles(self) -> None:
        """Test planning around obstacles."""
        # Create world binding with obstacles
        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=["/World/Obstacle"],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        # Add a collision cube
        Cube(paths=["/World/Obstacle"], sizes=[1.0], translations=[0.3, 0.0, 0.45], scales=[0.3, 0.3, 0.2])
        GeomPrim(paths=["/World/Obstacle"], apply_collision_apis=True)

        world_binding.initialize()
        await get_app().next_update_async()

        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[0] = -np.pi / 2

        q_target = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_target[0] = np.pi / 2

        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, cspace_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    # ============================================================================
    # Test edge cases
    # ============================================================================

    async def test_plan_to_goal_same_start_and_target(self) -> None:
        """Test planning when start and target are the same."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_target = q_initial.copy()

        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, cspace_target)

        # May return None or a valid trajectory
        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)

    async def test_plan_to_goal_trajectory_properties(self) -> None:
        """Test that generated trajectory has valid properties."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 4

        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]
        initial_pose = kinematics.pose(q_initial, tool_frame_name)

        target_position = initial_pose.translation + np.array([0.05, 0.0, 0.0])

        task_space_target = cumotion.TrajectoryOptimizer.TaskSpaceTarget(
            translation_constraint=cumotion.TrajectoryOptimizer.TranslationConstraint.target(target_position),
            orientation_constraint=cumotion.TrajectoryOptimizer.OrientationConstraint.none(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, task_space_target)

        self.assertIsNotNone(trajectory)
        # Check duration is positive
        self.assertGreater(trajectory.duration, 0.0)

        # Check active joints match configuration
        active_joints = trajectory.get_active_joints()
        self.assertEqual(len(active_joints), len(self.cumotion_robot.controlled_joint_names))
        self.assertEqual(active_joints, self.cumotion_robot.controlled_joint_names)

        # Check we can evaluate at different times
        state_start = trajectory.get_target_state(0.0)
        self.assertIsNotNone(state_start)

        state_mid = trajectory.get_target_state(trajectory.duration / 2.0)
        self.assertIsNotNone(state_mid)

        state_end = trajectory.get_target_state(trajectory.duration)
        self.assertIsNotNone(state_end)

    async def test_plan_to_goal_with_cspace_target_path_constraints(self) -> None:
        """Test planning with CSpaceTarget that has path constraints."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 4

        q_target = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_target[1] = -np.pi / 2

        # Create CSpaceTarget with linear translation path constraint
        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.linear(),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, cspace_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

    async def test_plan_to_goal_with_orientation_axis_constraint(self) -> None:
        """Test planning with orientation axis constraint."""
        trajectory_optimizer = cu_mg.TrajectoryOptimizer(
            cumotion_robot=self.cumotion_robot,
            robot_joint_space=self.cumotion_robot.controlled_joint_names,
            cumotion_world_interface=self.world_binding.get_world_interface(),
        )

        q_initial = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        initial_pose = kinematics.pose(q_initial, tool_frame_name)
        target_position = initial_pose.translation + np.array([0.05, 0.0, 0.0])

        # Create TaskSpaceTarget with axis orientation constraint
        tool_frame_axis = np.array([0.0, 0.0, 1.0])  # Z-axis
        world_target_axis = np.array([0.0, 0.0, 1.0])  # Keep Z-axis aligned

        task_space_target = cumotion.TrajectoryOptimizer.TaskSpaceTarget(
            translation_constraint=cumotion.TrajectoryOptimizer.TranslationConstraint.target(target_position),
            orientation_constraint=cumotion.TrajectoryOptimizer.OrientationConstraint.terminal_axis(
                tool_frame_axis, world_target_axis
            ),
        )

        trajectory = trajectory_optimizer.plan_to_goal(q_initial, task_space_target)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
