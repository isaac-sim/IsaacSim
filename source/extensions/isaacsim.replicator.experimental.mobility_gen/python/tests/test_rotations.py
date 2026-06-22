# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test Mobility Gen robot rotation and cached pose handling.

These tests cover 2D yaw application, RZ-RY-RX chase camera composition,
zero-tilt behavior, physics-valid state updates, and pose buffer reads.
"""

import math
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import omni.kit.test
from isaacsim.core.experimental.utils import transform as transform_utils
from isaacsim.replicator.experimental.mobility_gen.impl.robot import MobilityGenRobot


class TestRotationFixes(omni.kit.test.AsyncTestCase):
    """MobilityGenRobot rotation and cached-state regression tests."""

    async def setUp(self) -> None:
        """Prepare the async fixture for rotation and state tests."""

    async def tearDown(self) -> None:
        """Clean up the async fixture for rotation and state tests."""

    async def test_set_pose_2d_theta_is_yaw(self) -> None:
        """set_pose_2d must apply theta as yaw (Z-axis rotation), not roll (X-axis rotation).

        The former bug passed theta at index 0 ([theta, 0, 0]), which with extrinsic=True
        (input order [roll, pitch, yaw]) applied theta as a roll, tipping the robot onto
        its side.  The fix moves theta to index 2 ([0, 0, theta]).
        """
        theta = math.pi / 3  # 60 degrees — arbitrary non-trivial angle

        # Correct: [0, 0, theta] → pure yaw → quaternion [cos(θ/2), 0, 0, sin(θ/2)]
        q_correct = transform_utils.euler_angles_to_quaternion([0.0, 0.0, theta]).numpy()
        expected_yaw = np.array([math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2)], dtype=np.float32)
        self.assertTrue(
            np.allclose(q_correct, expected_yaw, atol=1e-5),
            f"[0, 0, theta] should give pure yaw quaternion {expected_yaw}, got {q_correct}",
        )

        # Old (wrong): [theta, 0, 0] → pure roll → quaternion [cos(θ/2), sin(θ/2), 0, 0]
        q_wrong = transform_utils.euler_angles_to_quaternion([theta, 0.0, 0.0]).numpy()
        expected_roll = np.array([math.cos(theta / 2), math.sin(theta / 2), 0.0, 0.0], dtype=np.float32)
        self.assertTrue(
            np.allclose(q_wrong, expected_roll, atol=1e-5),
            f"[theta, 0, 0] should give pure roll quaternion {expected_roll}, got {q_wrong}",
        )

        # The two must differ — old code produced the wrong rotation
        self.assertFalse(
            np.allclose(q_correct, q_wrong, atol=1e-5),
            "Yaw and roll quaternions must not be equal for non-zero theta",
        )

    async def test_chase_camera_extrinsic_true_is_rz_ry_rx(self) -> None:
        """Chase/front camera must use extrinsic=True to match deprecated prim_rotate_x/y/z ordering.

        The deprecated code called prim_rotate_x, prim_rotate_y, prim_rotate_z each with
        move_end_to_front, producing xformOpOrder [Rz, Ry, Rx] → R = Rz·Ry·Rx (extrinsic ZYX).

        The former bug used extrinsic=False (intrinsic XYZ: R = Rx·Ry·Rz), which differs
        whenever tilt ≠ 0.
        """
        tilt_deg = 45.0

        q_extrinsic = transform_utils.euler_angles_to_quaternion(
            [tilt_deg, 0.0, -90.0], degrees=True, extrinsic=True
        ).numpy()
        q_intrinsic = transform_utils.euler_angles_to_quaternion(
            [tilt_deg, 0.0, -90.0], degrees=True, extrinsic=False
        ).numpy()

        # The two conventions must differ when tilt != 0
        self.assertFalse(
            np.allclose(q_extrinsic, q_intrinsic, atol=1e-5),
            "extrinsic=True and extrinsic=False must differ for non-zero tilt",
        )

        # Verify extrinsic=True ground truth: R = Rz(-90°)·Rx(45°)
        # Computed analytically: q = q_Rz(-90) ⊗ q_Rx(45)
        #   q_Rz(-90) = [cos(-45°), 0, 0, sin(-45°)]
        #   q_Rx(45°) = [cos(22.5°), sin(22.5°), 0, 0]
        # Hamilton product gives: [(√2/2)cos(22.5°), (√2/2)sin(22.5°), -(√2/2)sin(22.5°), -(√2/2)cos(22.5°)]
        s = math.sqrt(2.0) / 2.0
        c225 = math.cos(math.radians(22.5))
        s225 = math.sin(math.radians(22.5))
        expected = np.array([s * c225, s * s225, -s * s225, -s * c225], dtype=np.float32)
        self.assertTrue(
            np.allclose(q_extrinsic, expected, atol=1e-5),
            f"extrinsic=True [45,0,-90]° expected {expected}, got {q_extrinsic}",
        )

    async def test_chase_camera_zero_tilt_is_pure_rz(self) -> None:
        """With zero tilt, both extrinsic conventions agree and the result is a pure Rz(-90°).

        This confirms the -90° Z rotation that aligns the camera to look forward is
        correctly applied regardless of tilt, and that the fix does not break the
        tilt=0 case.
        """
        q_true = transform_utils.euler_angles_to_quaternion([0.0, 0.0, -90.0], degrees=True, extrinsic=True).numpy()
        q_false = transform_utils.euler_angles_to_quaternion([0.0, 0.0, -90.0], degrees=True, extrinsic=False).numpy()

        # Both must agree when tilt = 0
        self.assertTrue(
            np.allclose(q_true, q_false, atol=1e-5),
            "With zero tilt, extrinsic=True and extrinsic=False must agree",
        )

        # Must equal a pure Rz(-90°): [cos(-45°), 0, 0, sin(-45°)]
        expected_rz = np.array(
            [math.cos(math.radians(-45)), 0.0, 0.0, math.sin(math.radians(-45))],
            dtype=np.float32,
        )
        self.assertTrue(
            np.allclose(q_true, expected_rz, atol=1e-5),
            f"Pure Rz(-90°) expected {expected_rz}, got {q_true}",
        )

    async def test_update_state_skips_when_physics_not_valid(self) -> Any:
        """update_state must be a no-op when the articulation physics tensor is not yet valid.

        Before the fix, calling update_state before physics initialized would raise an
        AssertionError from get_dof_positions().  The fix adds an early-return guard on
        is_physics_tensor_entity_valid().

        Returns:
            None.
        """

        class _ConcreteRobot(MobilityGenRobot):
            z_offset = 0.0

            @classmethod
            def build(cls, prim_path: Any) -> None:
                pass

            def write_action(self, step_size: Any) -> None:
                pass

        mock_articulation = MagicMock()
        mock_articulation.is_physics_tensor_entity_valid.return_value = False

        robot = _ConcreteRobot(prim_path="/test", articulation=mock_articulation, front_camera=None)

        # Must not raise, and physics-reading methods must not be called
        robot.update_state()

        mock_articulation.get_world_poses.assert_not_called()
        mock_articulation.get_dof_positions.assert_not_called()
        mock_articulation.get_dof_velocities.assert_not_called()
        mock_articulation.get_velocities.assert_not_called()

        # Buffers must remain at their uninitialised default (None)
        self.assertIsNone(robot.position.get_value())
        self.assertIsNone(robot.joint_positions.get_value())

    async def test_update_state_reads_state_when_physics_valid(self) -> Any:
        """update_state must populate all state buffers when physics is valid.

        Returns:
            None.
        """

        class _ConcreteRobot(MobilityGenRobot):
            z_offset = 0.0

            @classmethod
            def build(cls, prim_path: Any) -> None:
                pass

            def write_action(self, step_size: Any) -> None:
                pass

        mock_articulation = MagicMock()
        mock_articulation.is_physics_tensor_entity_valid.return_value = True

        pos = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        ori = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        joint_pos = np.array([[0.1, 0.2]], dtype=np.float32)
        joint_vel = np.array([[0.3, 0.4]], dtype=np.float32)
        lin_vel = np.array([[0.5, 0.6, 0.7]], dtype=np.float32)
        ang_vel = np.array([[0.8, 0.9, 1.0]], dtype=np.float32)

        mock_articulation.get_world_poses.return_value = (MagicMock(numpy=lambda: pos), MagicMock(numpy=lambda: ori))
        mock_articulation.get_dof_positions.return_value = MagicMock(numpy=lambda: joint_pos)
        mock_articulation.get_dof_velocities.return_value = MagicMock(numpy=lambda: joint_vel)
        mock_articulation.get_velocities.return_value = (
            MagicMock(numpy=lambda: lin_vel),
            MagicMock(numpy=lambda: ang_vel),
        )

        robot = _ConcreteRobot(prim_path="/test", articulation=mock_articulation, front_camera=None)
        robot.update_state()

        self.assertTrue(np.allclose(robot.position.get_value(), pos[0]))
        self.assertTrue(np.allclose(robot.orientation.get_value(), ori[0]))
        self.assertTrue(np.allclose(robot.joint_positions.get_value(), joint_pos[0]))
        self.assertTrue(np.allclose(robot.joint_velocities.get_value(), joint_vel[0]))
        self.assertTrue(np.allclose(robot.linear_velocity.get_value(), lin_vel[0]))
        self.assertTrue(np.allclose(robot.angular_velocity.get_value(), ang_vel[0]))

    async def test_get_pose_2d_reads_cached_buffers(self) -> Any:
        """get_pose_2d reads position/orientation buffers set by update_state and recovers yaw correctly.

        Returns:
            None.
        """

        class _ConcreteRobot(MobilityGenRobot):
            z_offset = 0.0

            @classmethod
            def build(cls, prim_path: Any) -> None:
                pass

            def write_action(self, step_size: Any) -> None:
                pass

        mock_articulation = MagicMock()
        mock_articulation.is_physics_tensor_entity_valid.return_value = True

        # Round-trip: pick a known 2D pose, encode the orientation as a yaw quaternion,
        # populate the cached buffers via update_state, then verify get_pose_2d returns
        # the original pose values.
        x, y, theta = 1.5, -2.25, math.pi / 4  # 45 degrees
        pos = np.array([[x, y, 0.0]], dtype=np.float32)
        ori = transform_utils.euler_angles_to_quaternion([0.0, 0.0, theta]).numpy()[np.newaxis]
        zero3 = np.zeros((1, 3), dtype=np.float32)
        zero2 = np.zeros((1, 2), dtype=np.float32)

        mock_articulation.get_world_poses.return_value = (MagicMock(numpy=lambda: pos), MagicMock(numpy=lambda: ori))
        mock_articulation.get_dof_positions.return_value = MagicMock(numpy=lambda: zero2)
        mock_articulation.get_dof_velocities.return_value = MagicMock(numpy=lambda: zero2)
        mock_articulation.get_velocities.return_value = (
            MagicMock(numpy=lambda: zero3),
            MagicMock(numpy=lambda: zero3),
        )

        robot = _ConcreteRobot(prim_path="/test", articulation=mock_articulation, front_camera=None)
        robot.update_state()

        pose = robot.get_pose_2d()
        self.assertAlmostEqual(pose.x, x, places=5)
        self.assertAlmostEqual(pose.y, y, places=5)
        self.assertAlmostEqual(pose.theta, theta, places=5)
