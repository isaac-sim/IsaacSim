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

"""Test suite for TrajectoryGenerator class."""

import cumotion
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.cumotion as cu_mg
import numpy as np
import omni.kit.test
import warp as wp
from omni.kit.app import get_app


class TestTrajectoryGeneratorFranka(omni.kit.test.AsyncTestCase):
    """Test suite for TrajectoryGenerator with Franka robot."""

    async def setUp(self) -> None:
        """Set up test environment before each test."""
        await stage_utils.create_new_stage_async()
        await get_app().next_update_async()

        # Initialize timeline
        self._timeline = omni.timeline.get_timeline_interface()

        # Create cumotion robot configuration
        self.cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        # Create trajectory generator
        self.trajectory_generator = cu_mg.TrajectoryGenerator(
            cumotion_robot=self.cumotion_robot,
            # NOTE: would normally use Articulation.dof_names to enforce
            # that the controlled joints will be valid for the Articulation.
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

    async def test_trajectory_generator_initialization(self) -> None:
        """Test that TrajectoryGenerator initializes correctly."""
        self.assertIsNotNone(self.trajectory_generator)
        self.assertIsNotNone(self.trajectory_generator.get_cspace_trajectory_generator())

    async def test_get_cspace_trajectory_generator(self) -> None:
        """Test getting the underlying CSpaceTrajectoryGenerator."""
        cspace_gen = self.trajectory_generator.get_cspace_trajectory_generator()
        self.assertIsNotNone(cspace_gen)
        self.assertIsInstance(cspace_gen, cumotion.CSpaceTrajectoryGenerator)

    # ============================================================================
    # Test generate_trajectory_from_cspace_waypoints with numpy arrays
    # ============================================================================

    async def test_generate_trajectory_from_cspace_waypoints_numpy(self) -> None:
        """Test trajectory generation from c-space waypoints as numpy array."""
        # Create simple waypoints - move joint 1 from -pi/2 to pi/2
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_start[1] = -np.pi / 2

        q_mid = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_mid[1] = 0.0

        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints = np.array([q_start, q_mid, q_end])

        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)

        self.assertTrue(np.allclose(trajectory.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(np.allclose(trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), q_end))

    async def test_generate_trajectory_from_cspace_waypoints_warp_array(self) -> None:
        """Test trajectory generation from c-space waypoints as warp array."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_start[1] = -np.pi / 2

        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints_np = np.array([q_start, q_end])
        waypoints_wp = wp.from_numpy(waypoints_np, dtype=wp.float64)

        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints_wp)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)
        self.assertTrue(np.allclose(trajectory.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(np.allclose(trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), q_end))

    async def test_generate_trajectory_from_cspace_waypoints_list(self) -> None:
        """Test trajectory generation from c-space waypoints as list."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_start[1] = -np.pi / 2

        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints = [q_start.tolist(), q_end.tolist()]

        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertTrue(np.allclose(trajectory.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(np.allclose(trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), q_end))

    # ============================================================================
    # Test generate_trajectory_from_cspace_waypoints with timing
    # ============================================================================

    async def test_generate_trajectory_with_time_stamps_numpy(self) -> None:
        """Test trajectory generation with explicit time stamps as numpy array."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_start[1] = -np.pi / 2

        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints = np.array([q_start, q_end])
        times = np.array([0.0, 20.0])

        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(
            waypoints=waypoints, times=times
        )

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        # Duration should be approximately 2.0 seconds
        self.assertAlmostEqual(trajectory.duration, 20.0, places=1)
        self.assertTrue(np.allclose(trajectory.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(np.allclose(trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), q_end))

    async def test_generate_trajectory_with_time_stamps_warp_array(self) -> None:
        """Test trajectory generation with explicit time stamps as warp array."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints_np = np.array([q_start, q_end])
        times_np = np.array([0.0, 15.0])

        waypoints_wp = wp.from_numpy(waypoints_np, dtype=wp.float64)
        times_wp = wp.from_numpy(times_np, dtype=wp.float64)

        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(
            waypoints=waypoints_wp, times=times_wp
        )

        self.assertIsNotNone(trajectory)
        self.assertAlmostEqual(trajectory.duration, 15.0, places=1)
        self.assertTrue(np.allclose(trajectory.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(np.allclose(trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), q_end))

    async def test_generate_trajectory_with_time_stamps_list(self) -> None:
        """Test trajectory generation with explicit time stamps as list."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints = [q_start.tolist(), q_end.tolist()]
        times = [0.0, 10.0]

        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(
            waypoints=waypoints, times=times
        )

        self.assertIsNotNone(trajectory)
        self.assertAlmostEqual(trajectory.duration, 10.0, places=1)
        self.assertTrue(np.allclose(trajectory.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(np.allclose(trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), q_end))

    async def test_generate_trajectory_with_interpolation_mode(self) -> None:
        """Test trajectory generation with different interpolation modes."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints = np.array([q_start, q_end])
        times = np.array([0.0, 10.0])

        # Test with LINEAR interpolation
        trajectory_linear = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(
            waypoints=waypoints,
            times=times,
            interpolation_mode=cumotion.CSpaceTrajectoryGenerator.InterpolationMode.LINEAR,
        )
        self.assertIsNotNone(trajectory_linear)
        self.assertTrue(np.allclose(trajectory_linear.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(
            np.allclose(trajectory_linear.get_target_state(trajectory_linear.duration).joints.positions.numpy(), q_end)
        )

        # Test with CUBIC_SPLINE interpolation
        trajectory_cubic = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(
            waypoints=waypoints,
            times=times,
            interpolation_mode=cumotion.CSpaceTrajectoryGenerator.InterpolationMode.CUBIC_SPLINE,
        )
        self.assertIsNotNone(trajectory_cubic)
        self.assertTrue(np.allclose(trajectory_cubic.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(
            np.allclose(trajectory_cubic.get_target_state(trajectory_cubic.duration).joints.positions.numpy(), q_end)
        )

    # ============================================================================
    # Test error handling for waypoints
    # ============================================================================

    async def test_generate_trajectory_invalid_waypoint_dimensions(self) -> None:
        """Test that invalid waypoint dimensions raise an error."""
        # 1D array instead of 2D
        waypoints_1d = np.array([0.0, 1.0, 2.0])

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints_1d)

    async def test_generate_trajectory_invalid_joint_count(self) -> None:
        """Test that waypoints with wrong number of joints raise an error."""
        # Franka has 7 joints, provide waypoints with 6
        waypoints_wrong_size = np.array([[0.0, 1.0, 2.0, 3.0, 4.0, 5.0]])

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints_wrong_size)

    async def test_generate_trajectory_invalid_time_dimensions(self) -> None:
        """Test that invalid time array dimensions raise an error."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = np.array([q_start, q_end])
        # 2D time array instead of 1D
        times_2d = np.array([[0.0], [1.0]])

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints=waypoints, times=times_2d)

    async def test_generate_trajectory_mismatched_waypoint_time_count(self) -> None:
        """Test that mismatched waypoint and time counts raise an error."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = np.array([q_start, q_end])
        # 3 times for 2 waypoints
        times = np.array([0.0, 10.0, 20.0])

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints=waypoints, times=times)

    async def test_generate_trajectory_with_negative_times(self) -> None:
        """Test that negative time values raise an error."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = np.array([q_start, q_end])
        times = np.array([-1.0, 1.0])  # Negative start time

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints=waypoints, times=times)

    async def test_generate_trajectory_with_negative_end_time(self) -> None:
        """Test that negative end time raises an error."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = np.array([q_start, q_end])
        times = np.array([0.0, -0.5])  # Negative end time

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints=waypoints, times=times)

    async def test_generate_trajectory_with_non_increasing_times(self) -> None:
        """Test that non-increasing time values raise an error."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_mid = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = np.array([q_start, q_mid, q_end])
        times = np.array([0.0, 1.0, 0.5])  # Time decreases at end

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints=waypoints, times=times)

    async def test_generate_trajectory_with_equal_consecutive_times(self) -> None:
        """Test that equal consecutive time values raise an error."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_mid = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = np.array([q_start, q_mid, q_end])
        times = np.array([0.0, 1.0, 1.0])  # Two equal times

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints=waypoints, times=times)

    async def test_generate_trajectory_with_all_zero_times(self) -> None:
        """Test that all zero time values raise an error."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = np.array([q_start, q_end])
        times = np.array([0.0, 0.0])  # Both times are zero

        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints=waypoints, times=times)

    # ============================================================================
    # Test generate_trajectory_from_path_specification with CSpacePathSpec
    # ============================================================================

    async def test_generate_trajectory_from_cspace_path_spec(self) -> None:
        """Test trajectory generation from CSpacePathSpec."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_start[1] = -np.pi / 2

        # Create CSpacePathSpec
        path_spec = cumotion.create_cspace_path_spec(q_start)

        q_mid = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_mid[1] = 0.0
        path_spec.add_cspace_waypoint(q_mid)

        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2
        path_spec.add_cspace_waypoint(q_end)

        # Convert to linear path
        linear_path = cumotion.create_linear_cspace_path(path_spec)

        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(
            path_specification=linear_path
        )

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)
        self.assertTrue(np.allclose(trajectory.get_target_state(0.0).joints.positions.numpy(), q_start))
        self.assertTrue(np.allclose(trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), q_end))

    # ============================================================================
    # Test generate_trajectory_from_path_specification with TaskSpacePathSpec
    # ============================================================================

    async def test_generate_trajectory_from_task_space_path_spec(self) -> None:
        """Test trajectory generation from TaskSpacePathSpec."""
        # Get current pose at default configuration
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        initial_pose = kinematics.pose(q_default, tool_frame_name)

        # Create TaskSpacePathSpec
        path_spec = cumotion.create_task_space_path_spec(initial_pose)

        # Add a translation
        target_position = initial_pose.translation + np.array([0.1, 0.0, 0.0])
        path_spec.add_translation(target_position)

        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(
            path_specification=path_spec, tool_frame_name=tool_frame_name
        )

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)

        initial_trajectory_pose = kinematics.pose(
            trajectory.get_target_state(0.0).joints.positions.numpy(), tool_frame_name
        )
        final_trajectory_pose = kinematics.pose(
            trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), tool_frame_name
        )

        print("#" * 100)
        print(f"initial_pose.matrix(): {initial_pose.matrix()}")
        print(f"initial_trajectory_pose.matrix(): {initial_trajectory_pose.matrix()}")
        print("#" * 100)

        print("#" * 100)
        print(f"target_position: {target_position}")
        print(f"final_trajectory_pose.translation: {final_trajectory_pose.translation}")
        print("#" * 100)

        self.assertTrue(np.allclose(initial_pose.matrix(), initial_trajectory_pose.matrix(), atol=1e-2))
        self.assertTrue(np.allclose(target_position, final_trajectory_pose.translation, atol=1e-2))

    async def test_generate_trajectory_from_task_space_path_spec_linear_path(self) -> None:
        """Test trajectory generation from TaskSpacePathSpec with linear path."""
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        initial_pose = kinematics.pose(q_default, tool_frame_name)

        # Create TaskSpacePathSpec with linear path
        path_spec = cumotion.create_task_space_path_spec(initial_pose)

        # Move in a line
        target_pose = cumotion.Pose3(
            rotation=initial_pose.rotation, translation=initial_pose.translation + np.array([0.0, 0.1, 0.0])
        )
        path_spec.add_linear_path(target_pose)

        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(
            path_specification=path_spec, tool_frame_name=tool_frame_name
        )

        self.assertIsNotNone(trajectory)
        self.assertGreater(trajectory.duration, 0.0)

        initial_trajectory_pose = kinematics.pose(
            trajectory.get_target_state(0.0).joints.positions.numpy(), tool_frame_name
        )
        final_trajectory_pose = kinematics.pose(
            trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), tool_frame_name
        )

        self.assertTrue(np.allclose(initial_pose.matrix(), initial_trajectory_pose.matrix(), atol=1e-2))
        self.assertTrue(np.allclose(target_pose.matrix(), final_trajectory_pose.matrix(), atol=1e-2))

    async def test_generate_trajectory_from_task_space_with_custom_configs(self) -> None:
        """Test trajectory generation with custom TaskSpacePathConversionConfig and IkConfig."""
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        initial_pose = kinematics.pose(q_default, tool_frame_name)

        # Create TaskSpacePathSpec
        path_spec = cumotion.create_task_space_path_spec(initial_pose)
        target_position = initial_pose.translation + np.array([0.05, 0.0, 0.0])
        path_spec.add_translation(target_position)

        # Create custom configs
        task_space_config = cumotion.TaskSpacePathConversionConfig()
        task_space_config.max_position_deviation = 0.005
        task_space_config.min_position_deviation = 0.001

        ik_config = cumotion.IkConfig()
        ik_config.max_num_descents = 50

        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(
            path_specification=path_spec,
            tool_frame_name=tool_frame_name,
            task_space_conversion_config=task_space_config,
            inverse_kinematics_config=ik_config,
        )

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)

        initial_trajectory_pose = kinematics.pose(
            trajectory.get_target_state(0.0).joints.positions.numpy(), tool_frame_name
        )
        final_trajectory_pose = kinematics.pose(
            trajectory.get_target_state(trajectory.duration).joints.positions.numpy(), tool_frame_name
        )

        self.assertTrue(np.allclose(initial_pose.matrix(), initial_trajectory_pose.matrix(), atol=1e-2))
        self.assertTrue(np.allclose(target_position, final_trajectory_pose.translation, atol=1e-2))

    # ============================================================================
    # Test generate_trajectory_from_path_specification with CompositePathSpec
    # ============================================================================

    async def test_generate_trajectory_from_composite_path_spec(self) -> None:
        """Test trajectory generation from CompositePathSpec."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_start[1] = -np.pi / 4

        # Create CompositePathSpec
        composite_spec = cumotion.create_composite_path_spec(q_start)

        # Add a c-space segment
        cspace_spec = cumotion.create_cspace_path_spec(q_start)
        q_mid = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_mid[1] = 0.0
        cspace_spec.add_cspace_waypoint(q_mid)

        composite_spec.add_cspace_path_spec(cspace_spec, cumotion.CompositePathSpec.TransitionMode.FREE)

        # Add a task-space segment
        kinematics = self.cumotion_robot.kinematics
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]
        pose_mid = kinematics.pose(q_mid, tool_frame_name)

        task_spec = cumotion.create_task_space_path_spec(pose_mid)
        target_position = pose_mid.translation + np.array([0.05, 0.0, 0.0])
        task_spec.add_translation(target_position)

        composite_spec.add_task_space_path_spec(task_spec, cumotion.CompositePathSpec.TransitionMode.LINEAR_TASK_SPACE)

        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(
            path_specification=composite_spec, tool_frame_name=tool_frame_name
        )

        self.assertIsNotNone(trajectory)
        self.assertIsInstance(trajectory, cu_mg.CumotionTrajectory)
        self.assertGreater(trajectory.duration, 0.0)

    async def test_generate_trajectory_from_composite_path_spec_skip_transition(self) -> None:
        """Test CompositePathSpec with SKIP transition mode."""
        q1 = self.cumotion_robot.robot_description.default_cspace_configuration()
        q1[1] = -np.pi / 4

        composite_spec = cumotion.create_composite_path_spec(q1)

        # First segment
        spec1 = cumotion.create_cspace_path_spec(q1)
        q2 = self.cumotion_robot.robot_description.default_cspace_configuration()
        spec1.add_cspace_waypoint(q2)
        composite_spec.add_cspace_path_spec(spec1, cumotion.CompositePathSpec.TransitionMode.FREE)

        # Second segment with SKIP transition
        spec2 = cumotion.create_cspace_path_spec(q2)
        q3 = self.cumotion_robot.robot_description.default_cspace_configuration()
        q3[1] = np.pi / 4
        spec2.add_cspace_waypoint(q3)
        composite_spec.add_cspace_path_spec(spec2, cumotion.CompositePathSpec.TransitionMode.SKIP)

        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(
            path_specification=composite_spec
        )

        self.assertIsNotNone(trajectory)

    # ============================================================================
    # Test trajectory properties
    # ============================================================================

    async def test_generated_trajectory_has_valid_properties(self) -> None:
        """Test that generated trajectory has valid properties."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        self.assertIsNotNone(trajectory)

        # Check duration is positive
        self.assertGreater(trajectory.duration, 0.0)

        # Check active joints match configuration
        active_joints = trajectory.get_active_joints()
        self.assertEqual(len(active_joints), len(self.cumotion_robot.controlled_joint_names))
        self.assertEqual(active_joints, self.cumotion_robot.controlled_joint_names)

    async def test_trajectory_evaluation_at_different_times(self) -> None:
        """Test evaluating trajectory at different time points."""
        q_start = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_start[1] = -np.pi / 2

        q_end = self.cumotion_robot.robot_description.default_cspace_configuration()
        q_end[1] = np.pi / 2

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        # Evaluate at start
        state_start = trajectory.get_target_state(0.0)
        self.assertIsNotNone(state_start)
        self.assertIsNotNone(state_start.joints.positions)
        self.assertIsNotNone(state_start.joints.velocities)

        # Evaluate at middle
        state_mid = trajectory.get_target_state(trajectory.duration / 2.0)
        self.assertIsNotNone(state_mid)

        # Evaluate at end
        state_end = trajectory.get_target_state(trajectory.duration)
        self.assertIsNotNone(state_end)

        # Evaluate beyond end (should return None)
        state_beyond = trajectory.get_target_state(trajectory.duration + 1.0)
        self.assertIsNone(state_beyond)

        # Evaluate before start (should return None)
        state_before = trajectory.get_target_state(-1.0)
        self.assertIsNone(state_before)

    async def test_trajectory_respects_joint_limits(self) -> None:
        """Test that trajectory waypoints respect joint limits."""
        kinematics = self.cumotion_robot.kinematics

        # Get joint limits
        joint_lower_limits = []
        joint_upper_limits = []
        for i in range(kinematics.num_cspace_coords()):
            limits = kinematics.cspace_coord_limits(i)
            joint_lower_limits.append(limits.lower)
            joint_upper_limits.append(limits.upper)

        # Create waypoints within limits
        q_start = np.array([(l + u) / 2.0 for l, u in zip(joint_lower_limits, joint_upper_limits)])
        q_end = q_start.copy()
        q_end[1] = joint_lower_limits[1] + 0.5  # Move joint 1 near lower limit

        waypoints = np.array([q_start, q_end])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        self.assertIsNotNone(trajectory)
        self.assertGreater(trajectory.duration, 0.0)

    async def test_multi_waypoint_trajectory(self) -> None:
        """Test trajectory with multiple waypoints."""
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()

        waypoints = []
        for i in range(5):
            q = q_default.copy()
            q[1] = -np.pi / 4 + i * np.pi / 8
            waypoints.append(q)

        waypoints_array = np.array(waypoints)
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints_array)

        self.assertIsNotNone(trajectory)
        self.assertGreater(trajectory.duration, 0.0)

        # Verify we can evaluate at multiple points
        num_samples = 10
        for i in range(num_samples):
            t = i * trajectory.duration / (num_samples - 1)
            state = trajectory.get_target_state(t)
            self.assertIsNotNone(state)

    # ============================================================================
    # Test tool frame handling
    # ============================================================================

    async def test_trajectory_generation_with_explicit_tool_frame(self) -> None:
        """Test trajectory generation with explicitly specified tool frame."""
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_names = self.cumotion_robot.robot_description.tool_frame_names()

        self.assertTrue(len(tool_frame_names) > 0)
        tool_frame_name = tool_frame_names[0]
        initial_pose = kinematics.pose(q_default, tool_frame_name)

        path_spec = cumotion.create_task_space_path_spec(initial_pose)
        target_position = initial_pose.translation + np.array([0.05, 0.0, 0.0])
        path_spec.add_translation(target_position)

        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(
            path_specification=path_spec, tool_frame_name=tool_frame_name
        )

        self.assertIsNotNone(trajectory)

    async def test_trajectory_generation_with_default_tool_frame(self) -> None:
        """Test trajectory generation with default tool frame (None specified)."""
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        kinematics = self.cumotion_robot.kinematics
        tool_frame_names = self.cumotion_robot.robot_description.tool_frame_names()

        self.assertTrue(len(tool_frame_names) > 0)

        # Use default tool frame by not specifying tool_frame_name
        tool_frame_name = tool_frame_names[0]
        initial_pose = kinematics.pose(q_default, tool_frame_name)

        path_spec = cumotion.create_task_space_path_spec(initial_pose)
        target_position = initial_pose.translation + np.array([0.03, 0.0, 0.0])
        path_spec.add_translation(target_position)

        # Don't specify tool_frame_name, should use default
        trajectory = self.trajectory_generator.generate_trajectory_from_path_specification(path_specification=path_spec)

        self.assertIsNotNone(trajectory)

    # ============================================================================
    # Test edge cases
    # ============================================================================

    async def test_empty_waypoints_trajectory(self) -> None:
        """Test trajectory generation with no waypoints raises RuntimeError."""
        waypoints = np.array([]).reshape(0, 7)  # Empty array with correct number of columns

        # Empty waypoints should raise a RuntimeError
        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

    async def test_single_waypoint_trajectory(self) -> None:
        """Test trajectory generation with a single waypoint raises RuntimeError."""
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        waypoints = np.array([q_default])

        # A single waypoint should raise a RuntimeError
        with self.assertRaises(RuntimeError):
            self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

    async def test_very_close_waypoints(self) -> None:
        """Test trajectory generation with waypoints very close together."""
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()

        q1 = q_default.copy()
        q2 = q_default.copy()
        q2[1] += 0.001  # Very small change

        waypoints = np.array([q1, q2])
        trajectory = self.trajectory_generator.generate_trajectory_from_cspace_waypoints(waypoints)

        # Should still generate a valid trajectory
        self.assertIsNotNone(trajectory)
        self.assertGreaterEqual(trajectory.duration, 0.0)
