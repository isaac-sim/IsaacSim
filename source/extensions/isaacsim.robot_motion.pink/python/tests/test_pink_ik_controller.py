# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies PinkIKController initialization, reset, forward solve behavior, task and limit configuration, solver handling, tool-frame validation, and RobotState outputs."""

import os
import tempfile
import warnings

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import pink
import pink.limits
import pink.tasks
import warp as wp
from isaacsim.robot_motion.pink import PinkIKController, load_pink_robot
from qpsolvers.warnings import SparseConversionWarning

_TEST_URDF = """\
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
  </link>
  <link name="link1">
    <visual><geometry><box size="0.1 0.1 0.5"/></geometry></visual>
  </link>
  <link name="link2">
    <visual><geometry><box size="0.1 0.1 0.5"/></geometry></visual>
  </link>
  <link name="end_effector">
    <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="end_effector"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
</robot>
"""

_JOINT_NAMES = ["joint1", "joint2", "joint3"]
_TOOL_FRAME = "end_effector"


def _make_estimated_state(joint_names: list[str], positions: np.ndarray) -> mg.RobotState:
    """Build a RobotState with joint positions and zero velocities.

    Args:
        joint_names: Joint names matching the position vector.
        positions: Joint position values.

    Returns:
        Robot state containing joint positions and zero velocities.
    """
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=joint_names,
            positions=(joint_names, wp.from_numpy(positions.astype(np.float32))),
            velocities=(joint_names, wp.zeros(len(joint_names), dtype=wp.float32)),
            efforts=None,
        )
    )


def _make_site_setpoint(tool_frame: str, position: np.ndarray, quaternion: np.ndarray) -> mg.RobotState:
    """Build a RobotState with a site target pose.

    Args:
        tool_frame: Site name to target.
        position: Desired site position.
        quaternion: Desired site orientation in Isaac Sim quaternion order.

    Returns:
        Robot state containing the target site pose.
    """
    return mg.RobotState(
        sites=mg.SpatialState.from_name(
            spatial_space=[tool_frame],
            positions=([tool_frame], wp.array([position.astype(np.float32)])),
            orientations=([tool_frame], wp.array([quaternion.astype(np.float32)])),
        ),
    )


class TestPinkIKController(omni.kit.test.AsyncTestCase):
    """Test suite for PinkIKController with a simple 3-joint test robot."""

    async def setUp(self) -> None:
        """Prepare the Pink IKController test fixture."""
        self._tmpdir = tempfile.mkdtemp()
        self._urdf_path = os.path.join(self._tmpdir, "robot.urdf")
        with open(self._urdf_path, "w") as f:
            f.write(_TEST_URDF)
        self.pink_robot = load_pink_robot(self._urdf_path)

    async def tearDown(self) -> None:
        """Clean up the Pink IKController test fixture."""
        if os.path.exists(self._urdf_path):
            os.remove(self._urdf_path)
        if os.path.exists(self._tmpdir):
            os.rmdir(self._tmpdir)

    # ========================================================================
    # Initialization
    # ========================================================================

    async def test_initialization_default(self) -> None:
        """Default initialization succeeds."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        self.assertIsNotNone(controller)
        self.assertIsNotNone(controller.get_frame_task())
        self.assertIsNotNone(controller.get_posture_task())

    async def test_initialization_custom_costs(self) -> None:
        """Custom position, orientation, and posture costs are accepted."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            position_cost=2.0,
            orientation_cost=0.5,
            posture_cost=1e-2,
            dt=1.0 / 60.0,
        )
        self.assertIsNotNone(controller)

    async def test_initialization_no_posture_task(self) -> None:
        """Setting posture_cost=None disables the PostureTask."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            posture_cost=None,
            dt=1.0 / 60.0,
        )
        self.assertIsNone(controller.get_posture_task())

    async def test_initialization_with_extra_task(self) -> None:
        """Extra tasks are accepted at construction."""
        damping = pink.tasks.DampingTask(cost=1e-4)
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            extra_tasks=[damping],
            dt=1.0 / 60.0,
        )
        self.assertIsNotNone(controller)

    async def test_extra_limits_extend_default_limits(self) -> None:
        """Extra limits are additive and preserve PINK's default safety limits."""
        extra_limit = object()
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            extra_limits=[extra_limit],
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        limits = controller._get_limits()
        self.assertEqual(len(limits), 3)
        self.assertTrue(any(isinstance(limit, pink.limits.ConfigurationLimit) for limit in limits))
        self.assertTrue(any(isinstance(limit, pink.limits.VelocityLimit) for limit in limits))
        self.assertIn(extra_limit, limits)

    async def test_invalid_joint_space(self) -> None:
        """ValueError when robot_joint_space doesn't contain all controlled joints."""
        with self.assertRaises(ValueError):
            PinkIKController(
                pink_robot=self.pink_robot,
                robot_joint_space=["joint1"],
                robot_site_space=[_TOOL_FRAME],
                tool_frame=_TOOL_FRAME,
                dt=1.0 / 60.0,
            )

    async def test_invalid_tool_frame_not_in_site_space(self) -> None:
        """ValueError when tool_frame is not in robot_site_space."""
        with self.assertRaises(ValueError):
            PinkIKController(
                pink_robot=self.pink_robot,
                robot_joint_space=_JOINT_NAMES,
                robot_site_space=[_TOOL_FRAME],
                tool_frame="nonexistent_frame",
                dt=1.0 / 60.0,
            )

    async def test_invalid_tool_frame_not_in_model(self) -> None:
        """ValueError when tool_frame doesn't exist in the Pinocchio model."""
        with self.assertRaises(ValueError):
            PinkIKController(
                pink_robot=self.pink_robot,
                robot_joint_space=_JOINT_NAMES,
                robot_site_space=["ghost_frame"],
                tool_frame="ghost_frame",
                dt=1.0 / 60.0,
            )

    # ========================================================================
    # Reset
    # ========================================================================

    async def test_reset_successful(self) -> None:
        """Reset returns True with valid joint positions."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        success = controller.reset(estimated_state=state, setpoint_state=None, t=0.0)
        self.assertTrue(success)

    async def test_reset_with_missing_joints(self) -> None:
        """Reset returns False when estimated_state has no joints."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = mg.RobotState()
        success = controller.reset(estimated_state=state, setpoint_state=None, t=0.0)
        self.assertFalse(success)

    async def test_reset_with_partial_joints(self) -> None:
        """Reset returns False when not all controlled joints are present."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        partial = _make_estimated_state(["joint1"], np.array([0.0]))
        success = controller.reset(estimated_state=partial, setpoint_state=None, t=0.0)
        self.assertFalse(success)

    async def test_reset_twice(self) -> None:
        """Resetting twice both succeed."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        self.assertTrue(controller.reset(state, None, 0.0))
        self.assertTrue(controller.reset(state, None, 0.5))

    # ========================================================================
    # Forward
    # ========================================================================

    async def test_forward_before_reset_returns_none(self) -> None:
        """Forward returns None before reset is called."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)
        self.assertIsNone(result)

    async def test_forward_after_reset(self) -> None:
        """Forward returns a valid RobotState after reset."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)
        self.assertIsNotNone(result)
        self.assertIsNotNone(result.joints)
        self.assertIsNotNone(result.joints.positions)
        self.assertIsNotNone(result.joints.velocities)
        self.assertEqual(len(result.joints.position_names), 3)

    async def test_forward_with_site_target(self) -> None:
        """Forward with an end-effector target pose returns a valid result."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        target_pos = np.array([0.3, 0.0, 0.8])
        target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        setpoint = _make_site_setpoint(_TOOL_FRAME, target_pos, target_quat)

        result = controller.forward(estimated_state=state, setpoint_state=setpoint, t=0.1)
        self.assertIsNotNone(result)

    async def test_forward_returns_finite_values(self) -> None:
        """All output joint positions and velocities are finite."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)
        self.assertIsNotNone(result)

        positions = result.joints.positions.numpy()
        velocities = result.joints.velocities.numpy()
        self.assertTrue(np.all(np.isfinite(positions)))
        self.assertTrue(np.all(np.isfinite(velocities)))

    async def test_forward_with_none_setpoint(self) -> None:
        """Forward with None setpoint does not crash."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)
        self.assertIsNotNone(result)

    async def test_multiple_forward_calls(self) -> None:
        """Multiple consecutive forward calls all return valid states."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        for i in range(5):
            result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1 * (i + 1))
            self.assertIsNotNone(result)
            self.assertTrue(np.all(np.isfinite(result.joints.positions.numpy())))

    async def test_forward_with_joint_setpoint(self) -> None:
        """Forward with a joint-space setpoint updates the posture target."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            posture_cost=1.0,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        q_target = np.array([0.5, 0.3, -0.2], dtype=np.float32)
        setpoint = _make_estimated_state(_JOINT_NAMES, q_target)

        result = controller.forward(estimated_state=state, setpoint_state=setpoint, t=0.1)
        self.assertIsNotNone(result)

    async def test_forward_with_changing_setpoint(self) -> None:
        """Changing site targets across steps works correctly."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        for i in range(3):
            target_pos = np.array([0.2 + i * 0.1, 0.0, 0.8])
            target_quat = np.array([1.0, 0.0, 0.0, 0.0])
            setpoint = _make_site_setpoint(_TOOL_FRAME, target_pos, target_quat)
            result = controller.forward(estimated_state=state, setpoint_state=setpoint, t=0.1 * (i + 1))
            self.assertIsNotNone(result)

    # ========================================================================
    # Output structure
    # ========================================================================

    async def test_output_robot_state_structure(self) -> None:
        """Output RobotState has joints populated but root/links/sites None."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)
        result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)

        self.assertIsNotNone(result)
        self.assertIsNotNone(result.joints)
        self.assertIsNone(result.root)
        self.assertIsNone(result.links)
        self.assertIsNone(result.sites)

    async def test_output_joint_names_match(self) -> None:
        """Output joint names match the controlled joint names."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)
        result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)

        self.assertEqual(set(result.joints.position_names), set(_JOINT_NAMES))
        self.assertEqual(set(result.joints.velocity_names), set(_JOINT_NAMES))

    # ========================================================================
    # Solver selection
    # ========================================================================

    async def test_osqp_solver(self) -> None:
        """Controller works with the osqp solver backend."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            solver="osqp",
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)
        result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)
        self.assertIsNotNone(result)

    async def test_osqp_solver_does_not_emit_sparse_conversion_warning(self) -> None:
        """OSQP receives sparse matrices directly to keep stderr clean."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            solver="osqp",
            dt=1.0 / 60.0,
        )
        state = _make_estimated_state(_JOINT_NAMES, np.zeros(3))
        controller.reset(state, None, 0.0)

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            result = controller.forward(estimated_state=state, setpoint_state=None, t=0.1)

        self.assertIsNotNone(result)
        self.assertFalse(any(isinstance(item.message, SparseConversionWarning) for item in caught))

    # ========================================================================
    # Task accessor
    # ========================================================================

    async def test_get_frame_task_type(self) -> None:
        """get_frame_task returns a FrameTask instance."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            dt=1.0 / 60.0,
        )
        self.assertIsInstance(controller.get_frame_task(), pink.tasks.FrameTask)

    async def test_get_posture_task_type(self) -> None:
        """get_posture_task returns a PostureTask instance when configured."""
        controller = PinkIKController(
            pink_robot=self.pink_robot,
            robot_joint_space=_JOINT_NAMES,
            robot_site_space=[_TOOL_FRAME],
            tool_frame=_TOOL_FRAME,
            posture_cost=1e-3,
            dt=1.0 / 60.0,
        )
        self.assertIsInstance(controller.get_posture_task(), pink.tasks.PostureTask)
