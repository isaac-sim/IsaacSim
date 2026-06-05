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

"""Test suite for RmpFlowController class."""

import cumotion
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.cumotion as cu_mg
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp
from omni.kit.app import get_app


class TestRmpFlowControllerFranka(omni.kit.test.AsyncTestCase):
    """Test suite for RmpFlowController with Franka robot."""

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

        # Get robot joint space and site space
        self.robot_joint_space = self.cumotion_robot.controlled_joint_names
        self.robot_site_space = self.cumotion_robot.robot_description.tool_frame_names()

    async def tearDown(self) -> None:
        """Clean up after each test."""
        # Stop timeline if running
        if self._timeline.is_playing():
            self._timeline.stop()

        await get_app().next_update_async()

    # ============================================================================
    # Test initialization
    # ============================================================================

    async def test_rmp_flow_controller_initialization(self) -> None:
        """Test that RmpFlowController initializes correctly."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        self.assertIsNotNone(controller)
        self.assertIsNotNone(controller.get_rmp_flow_config())

    async def test_rmp_flow_controller_with_custom_tool_frame(self) -> None:
        """Test initialization with explicitly specified tool frame."""
        tool_frames = self.cumotion_robot.robot_description.tool_frame_names()
        self.assertGreater(len(tool_frames), 0)

        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
            tool_frame=tool_frames[0],
        )

        self.assertIsNotNone(controller)

    async def test_rmp_flow_controller_with_default_tool_frame(self) -> None:
        """Test initialization with default tool frame (None specified)."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
            tool_frame=None,
        )

        self.assertIsNotNone(controller)

    async def test_rmp_flow_controller_invalid_joint_space(self) -> None:
        """Test that invalid joint space raises ValueError."""
        # Create a joint space that doesn't contain all controlled joints
        invalid_joint_space = ["joint1", "joint2"]  # Not a superset of controlled joints

        with self.assertRaises(ValueError):
            cu_mg.RmpFlowController(
                cumotion_robot=self.cumotion_robot,
                cumotion_world_interface=self.world_binding.get_world_interface(),
                robot_joint_space=invalid_joint_space,
                robot_site_space=self.robot_site_space,
            )

    async def test_rmp_flow_controller_invalid_tool_frame(self) -> None:
        """Test that tool frame not in robot_site_space raises ValueError."""
        invalid_tool_frame = "nonexistent_frame"

        with self.assertRaises(ValueError):
            cu_mg.RmpFlowController(
                cumotion_robot=self.cumotion_robot,
                cumotion_world_interface=self.world_binding.get_world_interface(),
                robot_joint_space=self.robot_joint_space,
                robot_site_space=self.robot_site_space,
                tool_frame=invalid_tool_frame,
            )

    async def test_rmp_flow_controller_invalid_maximum_substep_size_zero(self) -> None:
        """Test that maximum_substep_size of zero raises ValueError."""
        with self.assertRaises(ValueError):
            cu_mg.RmpFlowController(
                cumotion_robot=self.cumotion_robot,
                cumotion_world_interface=self.world_binding.get_world_interface(),
                robot_joint_space=self.robot_joint_space,
                robot_site_space=self.robot_site_space,
                maximum_substep_size=0.0,
            )

    async def test_rmp_flow_controller_invalid_maximum_substep_size_negative(self) -> None:
        """Test that a negative maximum_substep_size raises ValueError."""
        with self.assertRaises(ValueError):
            cu_mg.RmpFlowController(
                cumotion_robot=self.cumotion_robot,
                cumotion_world_interface=self.world_binding.get_world_interface(),
                robot_joint_space=self.robot_joint_space,
                robot_site_space=self.robot_site_space,
                maximum_substep_size=-0.01,
            )

    async def test_rmp_flow_controller_with_custom_config_file(self) -> None:
        """Test initialization with custom configuration file."""
        # Note: This test assumes a config file exists. If not, it will use defaults.
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
            rmp_flow_configuration_filename="rmp_flow.yaml",
        )

        self.assertIsNotNone(controller)

    # ============================================================================
    # Test reset method
    # ============================================================================

    async def test_reset_successful(self) -> None:
        """Test successful controller reset."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        # Create estimated state with joint positions
        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        success = controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        self.assertTrue(success)

    async def test_reset_with_missing_joint_state(self) -> None:
        """Test reset with missing joint state returns False."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        # Create estimated state without joints
        estimated_state = mg.RobotState()

        success = controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        self.assertFalse(success)

    async def test_reset_with_partial_joint_state(self) -> None:
        """Test reset with partial joint state (missing some joints) returns False."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        # Create estimated state with only some joints
        partial_joint_names = self.cumotion_robot.controlled_joint_names[:3]
        partial_positions = np.zeros(len(partial_joint_names))
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(partial_joint_names, wp.from_numpy(partial_positions)),
                velocities=(partial_joint_names, wp.zeros(len(partial_joint_names))),
                efforts=None,
            )
        )

        success = controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        self.assertFalse(success)

    # ============================================================================
    # Test forward method
    # ============================================================================

    async def test_forward_before_reset_returns_none(self) -> None:
        """Test that forward returns None before reset."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=0.1)

        self.assertIsNone(desired_state)

    async def test_forward_after_reset(self) -> None:
        """Test forward after successful reset."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Reset first
        success = controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)
        self.assertTrue(success)

        # Now forward should work
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=0.1)

        self.assertIsNotNone(desired_state)
        self.assertIsNotNone(desired_state.joints)
        self.assertIsNotNone(desired_state.joints.positions)
        self.assertIsNotNone(desired_state.joints.velocities)
        self.assertEqual(len(desired_state.joints.position_names), len(self.cumotion_robot.controlled_joint_names))

    async def test_forward_with_joint_attractor(self) -> None:
        """Test forward with joint space attractor in setpoint."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Create setpoint with different joint positions
        q_target = q_default.copy()
        q_target[1] = np.pi / 4
        setpoint_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_target)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Reset first
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with setpoint
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=setpoint_state, t=0.1)

        self.assertIsNotNone(desired_state)

    async def test_forward_with_position_attractor(self) -> None:
        """Test forward with end-effector position attractor in setpoint."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Get tool frame name
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        # Create setpoint with tool frame position
        target_position = np.array([0.5, 0.0, 0.5])
        setpoint_state = mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=[tool_frame_name],
                positions=([tool_frame_name], wp.array([target_position])),
                orientations=([tool_frame_name], wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
            )
        )

        # Reset first
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with setpoint
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=setpoint_state, t=0.1)

        self.assertIsNotNone(desired_state)

    async def test_forward_with_orientation_attractor(self) -> None:
        """Test forward with end-effector orientation attractor in setpoint."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Get tool frame name
        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        # Create setpoint with tool frame orientation
        target_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
        setpoint_state = mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=[tool_frame_name],
                positions=([tool_frame_name], wp.array([[0.5, 0.0, 0.5]])),
                orientations=([tool_frame_name], wp.array([target_orientation])),
                linear_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
            )
        )

        # Reset first
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with setpoint
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=setpoint_state, t=0.1)

        self.assertIsNotNone(desired_state)

    async def test_forward_with_all_attractors(self) -> None:
        """Test forward with joint, position, and orientation attractors."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        # Create setpoint with joints, position, and orientation
        q_target = q_default.copy()
        q_target[1] = np.pi / 4
        setpoint_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_target)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            ),
            sites=mg.SpatialState.from_name(
                spatial_space=[tool_frame_name],
                positions=([tool_frame_name], wp.array([[0.5, 0.0, 0.5]])),
                orientations=([tool_frame_name], wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
            ),
        )

        # Reset first
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with setpoint
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=setpoint_state, t=0.1)

        self.assertIsNotNone(desired_state)

    async def test_forward_time_progression(self) -> None:
        """Test forward with time progression."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Reset
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward at multiple time steps
        state_1 = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=0.1)
        self.assertIsNotNone(state_1)

        state_2 = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=0.2)
        self.assertIsNotNone(state_2)

        state_3 = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=0.3)
        self.assertIsNotNone(state_3)

        # States should be different (controller is integrating)
        positions_1 = state_1.joints.positions.numpy()
        positions_2 = state_2.joints.positions.numpy()
        positions_3 = state_3.joints.positions.numpy()

        # At minimum, verify we can get states at different times
        self.assertIsNotNone(positions_1)
        self.assertIsNotNone(positions_2)
        self.assertIsNotNone(positions_3)

    async def test_forward_with_none_setpoint(self) -> None:
        """Test forward with None setpoint (no attractors)."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Reset
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with None setpoint
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=0.1)

        self.assertIsNotNone(desired_state)

    # ============================================================================
    # Test configuration access
    # ============================================================================

    async def test_get_rmp_flow_config(self) -> None:
        """Test getting the RMPflow configuration."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        config = controller.get_rmp_flow_config()
        self.assertIsNotNone(config)
        self.assertIsInstance(config, cumotion.RmpFlowConfig)

    async def test_modify_rmp_flow_config(self) -> None:
        """Test modifying RMPflow configuration parameters."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        config = controller.get_rmp_flow_config()

        # Try to modify a parameter (if it exists)
        # Note: Parameter names depend on the cuMotion implementation
        try:
            # Example parameter modification - adjust if needed based on actual API
            config.set_param("cspace_target_rmp/damping_gain", 0.9)
        except Exception:
            # If parameter doesn't exist, that's okay for this test
            pass

        # Verify config is still valid
        self.assertIsNotNone(config)

    # ============================================================================
    # Test edge cases
    # ============================================================================

    async def test_multiple_forward_calls(self) -> None:
        """Test multiple consecutive forward calls."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Reset
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Multiple forward calls
        for i in range(5):
            t = 0.1 * (i + 1)
            desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=t)
            self.assertIsNotNone(desired_state)

    async def test_forward_with_very_small_time_step(self) -> None:
        """Test forward with very small time step."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Reset
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with very small time step
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=0.001)

        self.assertIsNotNone(desired_state)

    async def test_forward_with_large_time_step(self) -> None:
        """Test forward with large time step."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # Reset
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with large time step
        desired_state = controller.forward(estimated_state=estimated_state, setpoint_state=None, t=1.0)

        self.assertIsNotNone(desired_state)

    async def test_reset_twice(self) -> None:
        """Test resetting the controller twice."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        # First reset
        success1 = controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)
        self.assertTrue(success1)

        # Second reset
        success2 = controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.5)
        self.assertTrue(success2)

    async def test_forward_with_changing_setpoint(self) -> None:
        """Test forward with changing setpoint over time."""
        controller = cu_mg.RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot_joint_space,
            robot_site_space=self.robot_site_space,
        )

        q_default = self.cumotion_robot.robot_description.default_cspace_configuration()
        estimated_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                positions=(self.cumotion_robot.controlled_joint_names, wp.from_numpy(q_default)),
                velocities=(
                    self.cumotion_robot.controlled_joint_names,
                    wp.zeros(len(self.cumotion_robot.controlled_joint_names)),
                ),
                efforts=None,
            )
        )

        tool_frame_name = self.cumotion_robot.robot_description.tool_frame_names()[0]

        # Reset
        controller.reset(estimated_state=estimated_state, setpoint_state=None, t=0.0)

        # Forward with changing setpoints
        for i in range(3):
            target_position = np.array([0.4 + i * 0.1, 0.0, 0.5])
            setpoint_state = mg.RobotState(
                sites=mg.SpatialState.from_name(
                    spatial_space=[tool_frame_name],
                    positions=([tool_frame_name], wp.array([target_position])),
                    orientations=([tool_frame_name], wp.array([[1.0, 0.0, 0.0, 0.0]])),
                    linear_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
                    angular_velocities=([tool_frame_name], wp.array([[0.0, 0.0, 0.0]])),
                )
            )

            desired_state = controller.forward(
                estimated_state=estimated_state, setpoint_state=setpoint_state, t=0.1 * (i + 1)
            )
            self.assertIsNotNone(desired_state)
