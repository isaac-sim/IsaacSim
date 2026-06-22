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

"""Verifies conversion between Isaac Sim position-quaternion poses and Pinocchio SE3 transforms. The tests also cover joint-position mapping from named Isaac Sim joints into Pinocchio model order."""

import numpy as np
import omni.kit.test
import pinocchio as pin
import warp as wp
from isaacsim.robot_motion.pink.impl.utils import (
    isaac_sim_position_quaternion_to_se3,
    map_joint_positions_to_pinocchio,
    se3_to_isaac_sim_position_quaternion,
)


class TestSE3Conversions(omni.kit.test.AsyncTestCase):
    """Test suite for isaac_sim_position_quaternion_to_se3 and se3_to_isaac_sim_position_quaternion."""

    async def setUp(self) -> None:
        """Prepare the SE3 Conversions test fixture."""

    async def tearDown(self) -> None:
        """Clean up the SE3 Conversions test fixture."""

    # ========================================================================
    # isaac_sim_position_quaternion_to_se3
    # ========================================================================

    async def test_identity_transform(self) -> None:
        """Identity position and quaternion produce identity SE3."""
        se3 = isaac_sim_position_quaternion_to_se3(
            position=np.array([0.0, 0.0, 0.0]),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self.assertTrue(np.allclose(se3.translation, [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(se3.rotation, np.eye(3)))

    async def test_translation_only(self) -> None:
        """Non-zero translation with identity rotation."""
        se3 = isaac_sim_position_quaternion_to_se3(
            position=np.array([1.0, 2.0, 3.0]),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self.assertTrue(np.allclose(se3.translation, [1.0, 2.0, 3.0]))
        self.assertTrue(np.allclose(se3.rotation, np.eye(3)))

    async def test_rotation_90_about_z(self) -> None:
        """90-degree rotation about Z axis."""
        # Isaac Sim quaternion (w, x, y, z): 90 deg about Z
        c = np.cos(np.pi / 4)
        s = np.sin(np.pi / 4)
        se3 = isaac_sim_position_quaternion_to_se3(
            position=np.zeros(3),
            quaternion=np.array([c, 0.0, 0.0, s]),
        )
        expected_rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        self.assertTrue(np.allclose(se3.rotation, expected_rot, atol=1e-10))

    async def test_rotation_90_about_y(self) -> None:
        """90-degree rotation about Y axis."""
        c = np.cos(np.pi / 4)
        s = np.sin(np.pi / 4)
        se3 = isaac_sim_position_quaternion_to_se3(
            position=np.zeros(3),
            quaternion=np.array([c, 0.0, s, 0.0]),
        )
        expected_rot = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)
        self.assertTrue(np.allclose(se3.rotation, expected_rot, atol=1e-10))

    async def test_accepts_warp_arrays(self) -> None:
        """Function accepts wp.array inputs."""
        se3 = isaac_sim_position_quaternion_to_se3(
            position=wp.array([1.0, 2.0, 3.0], dtype=wp.float32),
            quaternion=wp.array([1.0, 0.0, 0.0, 0.0], dtype=wp.float32),
        )
        self.assertTrue(np.allclose(se3.translation, [1.0, 2.0, 3.0], atol=1e-5))

    async def test_accepts_lists(self) -> None:
        """Function accepts plain Python lists."""
        se3 = isaac_sim_position_quaternion_to_se3(
            position=[1.0, 0.0, 0.0],
            quaternion=[1.0, 0.0, 0.0, 0.0],
        )
        self.assertTrue(np.allclose(se3.translation, [1.0, 0.0, 0.0]))

    async def test_invalid_position_size(self) -> None:
        """ValueError for position with wrong number of elements."""
        with self.assertRaises(ValueError):
            isaac_sim_position_quaternion_to_se3(
                position=np.array([1.0, 2.0]),
                quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
            )

    async def test_invalid_quaternion_size(self) -> None:
        """ValueError for quaternion with wrong number of elements."""
        with self.assertRaises(ValueError):
            isaac_sim_position_quaternion_to_se3(
                position=np.array([0.0, 0.0, 0.0]),
                quaternion=np.array([1.0, 0.0, 0.0]),
            )

    # ========================================================================
    # se3_to_isaac_sim_position_quaternion
    # ========================================================================

    async def test_identity_se3_to_pos_quat(self) -> None:
        """Identity SE3 produces zero translation and identity quaternion."""
        pos, quat = se3_to_isaac_sim_position_quaternion(pin.SE3.Identity())
        self.assertTrue(np.allclose(pos, [0.0, 0.0, 0.0]))
        # Identity quaternion: w=1 or w=-1 (both valid)
        self.assertTrue(np.allclose(np.abs(quat), [1.0, 0.0, 0.0, 0.0], atol=1e-10))

    async def test_translation_se3_to_pos_quat(self) -> None:
        """SE3 with translation only."""
        se3 = pin.SE3(np.eye(3), np.array([5.0, -3.0, 1.0]))
        pos, quat = se3_to_isaac_sim_position_quaternion(se3)
        self.assertTrue(np.allclose(pos, [5.0, -3.0, 1.0]))

    # ========================================================================
    # Round-trip conversions
    # ========================================================================

    async def test_round_trip_identity(self) -> None:
        """Round-trip conversion preserves identity transform."""
        original_pos = np.array([0.0, 0.0, 0.0])
        original_quat = np.array([1.0, 0.0, 0.0, 0.0])

        se3 = isaac_sim_position_quaternion_to_se3(original_pos, original_quat)
        pos_out, quat_out = se3_to_isaac_sim_position_quaternion(se3)

        self.assertTrue(np.allclose(pos_out, original_pos))
        self.assertTrue(np.allclose(quat_out, original_quat, atol=1e-10))

    async def test_round_trip_arbitrary_pose(self) -> None:
        """Round-trip conversion preserves an arbitrary pose."""
        original_pos = np.array([1.5, -0.5, 0.75])
        c = np.cos(np.pi / 6)
        s = np.sin(np.pi / 6)
        original_quat = np.array([c, 0.0, s, 0.0])

        se3 = isaac_sim_position_quaternion_to_se3(original_pos, original_quat)
        pos_out, quat_out = se3_to_isaac_sim_position_quaternion(se3)

        self.assertTrue(np.allclose(pos_out, original_pos, atol=1e-10))
        # Quaternion sign ambiguity: q and -q represent the same rotation
        sign = np.sign(quat_out[0]) * np.sign(original_quat[0])
        self.assertTrue(np.allclose(sign * quat_out, original_quat, atol=1e-10))

    async def test_round_trip_complex_rotation(self) -> None:
        """Round-trip through a composed rotation about multiple axes."""
        original_pos = np.array([-2.0, 3.5, 0.1])
        # Compose three axis-angle rotations into a quaternion
        r1 = pin.Quaternion(pin.AngleAxis(0.3, np.array([1.0, 0.0, 0.0])))
        r2 = pin.Quaternion(pin.AngleAxis(0.7, np.array([0.0, 1.0, 0.0])))
        r3 = pin.Quaternion(pin.AngleAxis(-0.2, np.array([0.0, 0.0, 1.0])))
        r_composed = r1 * r2 * r3
        original_quat = np.array([r_composed.w, r_composed.x, r_composed.y, r_composed.z])

        se3 = isaac_sim_position_quaternion_to_se3(original_pos, original_quat)
        pos_out, quat_out = se3_to_isaac_sim_position_quaternion(se3)

        self.assertTrue(np.allclose(pos_out, original_pos, atol=1e-10))
        sign = np.sign(quat_out[0]) * np.sign(original_quat[0])
        self.assertTrue(np.allclose(sign * quat_out, original_quat, atol=1e-10))


class TestJointMapping(omni.kit.test.AsyncTestCase):
    """Test suite for joint position mapping between Isaac Sim and Pinocchio."""

    async def setUp(self) -> None:
        """Prepare the Joint Mapping test fixture."""
        # Build a 3-revolute-joint model programmatically
        self.model = pin.Model()
        parent_id = 0
        for i in range(3):
            joint_name = f"joint{i + 1}"
            joint_placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.5 * (i > 0)]))
            joint_id = self.model.addJoint(parent_id, pin.JointModelRZ(), joint_placement, joint_name)
            self.model.appendBodyToJoint(joint_id, pin.Inertia.Random(), pin.SE3.Identity())
            parent_id = joint_id
        # Add EE frame
        self.model.addFrame(
            pin.Frame(
                "end_effector", parent_id, 0, pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.5])), pin.FrameType.OP_FRAME
            )
        )

    async def tearDown(self) -> None:
        """Clean up the Joint Mapping test fixture."""

    async def test_map_all_joints(self) -> None:
        """Mapping all joints produces correct q vector."""
        joint_names = ["joint1", "joint2", "joint3"]
        positions = np.array([0.1, 0.2, 0.3])
        q = map_joint_positions_to_pinocchio(joint_names, positions, self.model)
        self.assertTrue(np.allclose(q, [0.1, 0.2, 0.3]))

    async def test_map_partial_joints(self) -> None:
        """Mapping a subset fills unspecified joints from neutral."""
        joint_names = ["joint2"]
        positions = np.array([0.5])
        q = map_joint_positions_to_pinocchio(joint_names, positions, self.model)
        self.assertAlmostEqual(q[0], 0.0)
        self.assertAlmostEqual(q[1], 0.5)
        self.assertAlmostEqual(q[2], 0.0)

    async def test_map_with_base_q(self) -> None:
        """Unspecified joints retain values from q_current."""
        q_current = np.array([1.0, 2.0, 3.0])
        joint_names = ["joint1"]
        positions = np.array([0.5])
        q = map_joint_positions_to_pinocchio(joint_names, positions, self.model, q_current=q_current)
        self.assertAlmostEqual(q[0], 0.5)
        self.assertAlmostEqual(q[1], 2.0)
        self.assertAlmostEqual(q[2], 3.0)

    async def test_unknown_joint_ignored(self) -> None:
        """Joints not in the model are silently skipped."""
        joint_names = ["nonexistent_joint"]
        positions = np.array([1.0])
        q = map_joint_positions_to_pinocchio(joint_names, positions, self.model)
        self.assertTrue(np.allclose(q, pin.neutral(self.model)))

    async def test_warp_array_input(self) -> None:
        """Accepts warp arrays for joint positions."""
        joint_names = ["joint1", "joint2", "joint3"]
        positions = wp.array([0.1, 0.2, 0.3], dtype=wp.float32)
        q = map_joint_positions_to_pinocchio(joint_names, positions, self.model)
        self.assertTrue(np.allclose(q, [0.1, 0.2, 0.3], atol=1e-5))
