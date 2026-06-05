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

"""Tests for the graph-based motion planner functionality with Franka robot configurations."""

import cumotion
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.cumotion as cu_mg

# Import extension python module we are testing with absolute import path, as if we are an external user (i.e. a different extension)
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim
from omni.kit.app import get_app


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of the module will make it auto-discoverable by omni.kit.test
class TestGraphBasedMotionPlannerFranka(omni.kit.test.AsyncTestCase):
    """Test suite for the ``GraphBasedMotionPlanner`` class with Franka robot configurations.

    This test class validates the functionality of the cuMotion graph-based motion planner using a Franka robot.
    It tests various planning scenarios including configuration space planning, task space planning, obstacle
    avoidance, error handling, and input validation. The tests cover both successful planning cases and edge cases
    where planning may fail or encounter errors.

    The test suite includes validation of:
    - Configuration space (joint space) motion planning
    - Task space pose and translation target planning
    - Obstacle avoidance capabilities
    - Error handling for invalid inputs and unreachable targets
    - Custom configuration file usage
    - Different input formats (numpy arrays, warp arrays, Python lists)
    - Joint limit validation
    - Custom tool frame specification
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment before each test."""
        await stage_utils.create_new_stage_async()

        await get_app().next_update_async()

        # Initialize timeline
        self._timeline = omni.timeline.get_timeline_interface()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment after each test."""
        # Stop timeline if running
        if self._timeline.is_playing():
            self._timeline.stop()

        # Clean up physics callbacks if any
        if hasattr(self, "sample") and self.sample:
            self.sample.physics_cleanup()

        await get_app().next_update_async()

    async def test_plan_to_cspace_target(self) -> None:
        """Test configuration space planning between two joint configurations."""
        # create a cumotion robot configuration:
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        # create a WorldBinding to populate the cumotion world interface:
        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        # We should be able to plan from a start position to an end-position with nothing in the way:
        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 2
        q_final = cumotion_robot.robot_description.default_cspace_configuration()
        q_final[1] = np.pi / 2

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial, q_final=q_final)

        self.assertIsNotNone(path)
        print(f"path found: {path.get_waypoints()}")

    async def test_plan_to_pose_target_no_obstacles(self) -> None:
        """Test task space pose planning to a reachable target."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()

        # Define a reachable pose target
        target_position = [0.5, 0.0, 0.5]
        target_orientation = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion (w, x, y, z)

        path = graph_based_path_planner.plan_to_pose_target(
            q_initial=q_initial, position=target_position, orientation=target_orientation
        )

        self.assertIsNotNone(path)
        self.assertGreater(path.get_waypoints_count(), 1)

    async def test_plan_to_translation_target_no_obstacles(self) -> None:
        """Test translation-only planning to a reachable target."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()

        # Define a reachable translation target
        translation_target = [0.4, 0.2, 0.4]

        path = graph_based_path_planner.plan_to_translation_target(
            q_initial=q_initial, translation_target=translation_target
        )

        self.assertIsNotNone(path)
        self.assertGreater(path.get_waypoints_count(), 1)

    async def test_plan_with_obstacle_avoidance(self) -> None:
        """Test planning around an obstacle blocking the path."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=["/World/Obstacle"],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        # Add a collision cube:
        Cube(paths=["/World/Obstacle"], sizes=[1.0], translations=[0.3, 0.0, 0.45], scales=[0.3, 0.3, 0.2])
        GeomPrim(paths=["/World/Obstacle"], apply_collision_apis=True)

        world_binding.initialize()
        await get_app().next_update_async()

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[0] = -np.pi / 2
        q_final = cumotion_robot.robot_description.default_cspace_configuration()
        q_final[0] = np.pi / 2

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial, q_final=q_final)

        self.assertIsNotNone(path)
        self.assertGreater(path.get_waypoints_count(), 1)

    async def test_no_path_exists(self) -> None:
        """Test that None is returned when no path can be found."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        # Modify config to make planning very difficult (very few iterations)
        config = graph_based_path_planner.get_graph_planner_config()
        config.set_param("max_iterations", cumotion.MotionPlannerConfig.ParamValue(1))

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()

        # Try to plan to a very difficult/unreachable pose
        very_far_position = [10.0, 10.0, 10.0]
        target_orientation = [1.0, 0.0, 0.0, 0.0]

        path = graph_based_path_planner.plan_to_pose_target(
            q_initial=q_initial, position=very_far_position, orientation=target_orientation
        )

        self.assertIsNone(path)

    async def test_invalid_joint_dimensions(self) -> None:
        """Test error handling with incorrect joint dimension inputs."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_valid = cumotion_robot.robot_description.default_cspace_configuration()
        q_wrong_size = np.array([0.0, 0.0, 0.0])  # Wrong number of joints

        # Test plan_to_cspace_target with wrong initial size
        with self.assertRaises(RuntimeError):
            graph_based_path_planner.plan_to_cspace_target(q_initial=q_wrong_size, q_final=q_valid)

        # Test plan_to_cspace_target with wrong final size
        with self.assertRaises(RuntimeError):
            graph_based_path_planner.plan_to_cspace_target(q_initial=q_valid, q_final=q_wrong_size)

        # Test plan_to_pose_target with wrong initial size
        with self.assertRaises(RuntimeError):
            graph_based_path_planner.plan_to_pose_target(
                q_initial=q_wrong_size, position=[0.5, 0.0, 0.5], orientation=[1.0, 0.0, 0.0, 0.0]
            )

        # Test plan_to_translation_target with wrong initial size
        with self.assertRaises(RuntimeError):
            graph_based_path_planner.plan_to_translation_target(
                q_initial=q_wrong_size, translation_target=[0.5, 0.0, 0.5]
            )

    async def test_plan_with_custom_config_file(self) -> None:
        """Test planning with a custom configuration file."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        # Use custom config file for graph planner
        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
            graph_planner_config_filename="graph_based_motion_planner_config.yaml",
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 4
        q_final = cumotion_robot.robot_description.default_cspace_configuration()
        q_final[1] = np.pi / 4

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial, q_final=q_final)

        self.assertIsNotNone(path)
        self.assertGreater(path.get_waypoints_count(), 1)

    async def test_plan_with_absolute_config_path(self) -> None:
        """Test planning with absolute configuration file path (auto-detected)."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        # Use absolute path to config file - should be auto-detected as absolute
        config_file_relative = cumotion_robot.directory / "graph_based_motion_planner_config.yaml"
        self.assertTrue(config_file_relative.exists())

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
            graph_planner_config_filename=str(config_file_relative.absolute()),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()
        q_initial[1] = -np.pi / 4
        q_final = cumotion_robot.robot_description.default_cspace_configuration()
        q_final[1] = np.pi / 4

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial, q_final=q_final)

        self.assertIsNotNone(path)
        self.assertGreater(path.get_waypoints_count(), 1)

    async def test_same_start_and_goal(self) -> None:
        """Test planning when start and goal configurations are identical."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()
        q_final = q_initial.copy()

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial, q_final=q_final)

        # Should return a valid path (even if trivial)
        self.assertIsNotNone(path)

    async def test_invalid_orientation_format(self) -> None:
        """Test error handling for invalid orientation formats in pose planning."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()
        target_position = [0.5, 0.0, 0.5]

        # Test with wrong-sized orientation (not 4 elements for quaternion or 3x3 for rotation matrix)
        invalid_orientation = [1.0, 0.0, 0.0]  # 3 elements (not valid)

        with self.assertRaises(ValueError):
            graph_based_path_planner.plan_to_pose_target(
                q_initial=q_initial, position=target_position, orientation=invalid_orientation
            )

    async def test_custom_tool_frame(self) -> None:
        """Test planning with a custom tool frame."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        # Get available tool frames
        tool_frames = cumotion_robot.robot_description.tool_frame_names()
        self.assertGreater(len(tool_frames), 0)

        # Use the first available tool frame explicitly
        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
            tool_frame=tool_frames[0],
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()
        q_final = cumotion_robot.robot_description.default_cspace_configuration()
        q_final[1] = np.pi / 4

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial, q_final=q_final)

        self.assertIsNotNone(path)

    async def test_warp_array_inputs(self) -> None:
        """Test planning with warp array inputs."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        # Create warp arrays for initial and final configurations
        q_initial_np = cumotion_robot.robot_description.default_cspace_configuration()
        q_initial_np[1] = -np.pi / 4
        q_initial_wp = wp.array(q_initial_np, dtype=wp.float32)

        q_final_np = cumotion_robot.robot_description.default_cspace_configuration()
        q_final_np[1] = np.pi / 4
        q_final_wp = wp.array(q_final_np, dtype=wp.float32)

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial_wp, q_final=q_final_wp)

        self.assertIsNotNone(path)
        self.assertGreater(path.get_waypoints_count(), 1)

    async def test_list_inputs(self) -> None:
        """Test planning with list inputs."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        # Use Python lists instead of numpy arrays
        q_initial_list = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.75]
        q_final_list = [0.5, -0.3, 0.2, -1.8, 0.1, 1.2, 0.5]

        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial_list, q_final=q_final_list)

        self.assertIsNotNone(path)
        self.assertGreater(path.get_waypoints_count(), 1)

    async def test_joint_limit_violations(self) -> None:
        """Test planning with targets that may violate joint limits."""
        cumotion_robot = cu_mg.load_cumotion_supported_robot("franka")

        world_binding = mg.WorldBinding(
            world_interface=cu_mg.CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=[],
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )

        graph_based_path_planner = cu_mg.GraphBasedMotionPlanner(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=world_binding.get_world_interface(),
        )

        q_initial = cumotion_robot.robot_description.default_cspace_configuration()

        # Create a target configuration with extreme joint values
        q_final = cumotion_robot.robot_description.default_cspace_configuration()
        q_final[0] = 5.0 * np.pi  # Very large rotation

        # Planning may fail or return None if target violates limits
        path = graph_based_path_planner.plan_to_cspace_target(q_initial=q_initial, q_final=q_final)

        # Either path is None (planner rejected it) or it found a valid path
        # We just check that it doesn't crash
        if path is not None:
            self.assertGreater(path.get_waypoints_count(), 0)
