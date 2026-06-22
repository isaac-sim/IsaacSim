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

"""Test suite for batch_compute_collider_transforms function."""

import isaacsim.robot_motion.cumotion as cu_mg
import numpy as np
import omni.kit.test
import warp as wp


def _random_unit_quaternions(n: int, rng: np.random.Generator) -> np.ndarray:
    """Sample unit quaternions uniformly on S^3.

    Args:
        n: Number of quaternions to sample.
        rng: Random number generator to sample from.

    Returns:
        ``(n, 4)`` float32 array in ``(w, x, y, z)`` order.
    """
    q = rng.standard_normal((n, 4)).astype(np.float32)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q


class TestBatchColliderTransforms(omni.kit.test.AsyncTestCase):
    """Test suite for batch collider transform computation."""

    async def setUp(self) -> None:
        """Sets up the test environment before each test method."""

    async def tearDown(self) -> None:
        """Cleans up the test environment after each test method."""

    # ============================================================================
    # Test batch_compute_collider_transforms
    # ============================================================================

    async def test_batch_compute_collider_transforms_identity(self) -> None:
        """Test batch collider transform computation with identity transforms."""
        # Identity transforms everywhere - output should equal input
        position_base_to_world = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        quaternion_base_to_world = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        # 2 objects
        positions_world_to_object = wp.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=wp.float32)
        quaternions_world_to_object = wp.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        # 5 colliders: 2 for object 0, 3 for object 1
        positions_object_to_collider = wp.array(
            [
                [0.1, 0.0, 0.0],  # Object 0, collider 0
                [0.2, 0.0, 0.0],  # Object 0, collider 1
                [0.3, 0.0, 0.0],  # Object 1, collider 0
                [0.4, 0.0, 0.0],  # Object 1, collider 1
                [0.5, 0.0, 0.0],  # Object 1, collider 2
            ],
            dtype=wp.float32,
        )
        quaternions_object_to_collider = wp.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            dtype=wp.float32,
        )

        num_colliders_per_object = [2, 3]

        # Compute transforms
        output = cu_mg.impl.utils.batch_compute_collider_transforms(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            num_colliders_per_object=num_colliders_per_object,
        )

        # Expected: T_base_collider = T_base_world * T_world_object * T_object_collider
        # With identity base and rotations, this is just position addition
        expected_positions_base = np.array(
            [
                [1.1, 0.0, 0.0],  # Object 0 at [1,0,0] + collider at [0.1,0,0]
                [1.2, 0.0, 0.0],  # Object 0 at [1,0,0] + collider at [0.2,0,0]
                [2.3, 0.0, 0.0],  # Object 1 at [2,0,0] + collider at [0.3,0,0]
                [2.4, 0.0, 0.0],  # Object 1 at [2,0,0] + collider at [0.4,0,0]
                [2.5, 0.0, 0.0],  # Object 1 at [2,0,0] + collider at [0.5,0,0]
            ]
        )

        # For identity base frame, world and base transforms should be the same
        expected_positions_world = expected_positions_base.copy()

        self.assertTrue(np.allclose(output.positions_base_to_collider.numpy(), expected_positions_base, atol=1e-5))
        self.assertTrue(np.allclose(output.positions_world_to_collider.numpy(), expected_positions_world, atol=1e-5))

        # All quaternions should remain identity
        self.assertTrue(np.allclose(output.quaternions_base_to_collider.numpy()[:, 0], 1.0, atol=1e-5))  # w component
        self.assertTrue(np.allclose(output.quaternions_world_to_collider.numpy()[:, 0], 1.0, atol=1e-5))  # w component

    async def test_batch_compute_collider_transforms_with_base_translation(self) -> None:
        """Test with translated base frame."""
        # Base frame translated by [-1, 0, 0]
        position_base_to_world = wp.array([[-1.0, 0.0, 0.0]], dtype=wp.float32)
        quaternion_base_to_world = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        # 1 object at world origin
        positions_world_to_object = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        quaternions_world_to_object = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        # 1 collider at [1, 0, 0] relative to object
        positions_object_to_collider = wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32)
        quaternions_object_to_collider = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        num_colliders_per_object = [1]

        output = cu_mg.impl.utils.batch_compute_collider_transforms(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            num_colliders_per_object=num_colliders_per_object,
        )

        # Collider is at [1,0,0] in world frame
        expected_position_world = np.array([[1.0, 0.0, 0.0]])
        self.assertTrue(np.allclose(output.positions_world_to_collider.numpy(), expected_position_world, atol=1e-5))

        # In base frame collider is at [0, 0, 0]
        expected_position_base = np.array([[0.0, 0.0, 0.0]])
        self.assertTrue(np.allclose(output.positions_base_to_collider.numpy(), expected_position_base, atol=1e-5))

    async def test_batch_compute_collider_transforms_with_rotation(self) -> None:
        """Test with 90-degree rotation about Z-axis."""
        # Base frame rotated 90 degrees about Z-axis
        position_base_to_world = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        quaternion_base_to_world = wp.array([[np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]], dtype=wp.float32)

        # 1 object at [1, 0, 0] in world
        positions_world_to_object = wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32)
        quaternions_world_to_object = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        # 1 collider at origin relative to object
        positions_object_to_collider = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        quaternions_object_to_collider = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        num_colliders_per_object = [1]

        output = cu_mg.impl.utils.batch_compute_collider_transforms(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            num_colliders_per_object=num_colliders_per_object,
        )

        # World frame: collider at [1, 0, 0]
        expected_position_world = np.array([[1.0, 0.0, 0.0]])
        expected_quaternion_world = np.array([[1.0, 0.0, 0.0, 0.0]])

        # Base frame: point at [1, 0, 0] rotated 90 degrees about Z becomes [0, 1, 0]
        expected_position_base = np.array([[0.0, 1.0, 0.0]])
        expected_quaternion_base = np.array([[np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]])

        self.assertTrue(np.allclose(output.positions_world_to_collider.numpy(), expected_position_world, atol=1e-5))
        self.assertTrue(np.allclose(output.quaternions_world_to_collider.numpy(), expected_quaternion_world, atol=1e-5))

        self.assertTrue(np.allclose(output.positions_base_to_collider.numpy(), expected_position_base, atol=1e-5))
        self.assertTrue(np.allclose(output.quaternions_base_to_collider.numpy(), expected_quaternion_base, atol=1e-5))

    async def test_batch_compute_collider_transforms_complex_scenario(self) -> None:
        """Test with multiple objects, colliders, and complex transforms."""
        # Base frame: translated and rotated
        position_base_to_world = wp.array([[-1.0, 0.0, 0.0]], dtype=wp.float32)
        # 90 degree rotation about Y-axis
        quaternion_base_to_world = wp.array([[np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]], dtype=wp.float32)

        # 2 objects
        positions_world_to_object = wp.array(
            [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=wp.float32  # Object 0  # Object 1
        )
        quaternions_world_to_object = wp.array(
            [
                [1.0, 0.0, 0.0, 0.0],  # Object 0: no rotation
                [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)],  # Object 1: 90 deg about Z
            ],
            dtype=wp.float32,
        )

        # 3 colliders: 1 for object 0, 2 for object 1
        positions_object_to_collider = wp.array(
            [
                [0.0, 0.0, 0.0],  # Object 0, collider 0 (at object origin)
                [1.0, 0.0, 0.0],  # Object 1, collider 0
                [0.0, 1.0, 0.0],  # Object 1, collider 1
            ],
            dtype=wp.float32,
        )
        quaternions_object_to_collider = wp.array(
            [[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=wp.float32
        )

        num_colliders_per_object = [1, 2]

        output = cu_mg.impl.utils.batch_compute_collider_transforms(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            num_colliders_per_object=num_colliders_per_object,
        )

        # Verify we got 3 output transforms
        self.assertEqual(output.positions_base_to_collider.shape[0], 3)
        self.assertEqual(output.quaternions_base_to_collider.shape[0], 3)
        self.assertEqual(output.positions_world_to_collider.shape[0], 3)
        self.assertEqual(output.quaternions_world_to_collider.shape[0], 3)

        # Verify all quaternions are normalized (both base and world)
        for i in range(3):
            quat_base = output.quaternions_base_to_collider.numpy()[i]
            quat_base_norm = np.linalg.norm(quat_base)
            self.assertTrue(np.isclose(quat_base_norm, 1.0, atol=1e-5))

            quat_world = output.quaternions_world_to_collider.numpy()[i]
            quat_world_norm = np.linalg.norm(quat_world)
            self.assertTrue(np.isclose(quat_world_norm, 1.0, atol=1e-5))

    async def test_batch_compute_collider_transforms_single_collider(self) -> None:
        """Test edge case with single object and single collider."""
        position_base_to_world = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        quaternion_base_to_world = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        positions_world_to_object = wp.array([[1.0, 2.0, 3.0]], dtype=wp.float32)
        quaternions_world_to_object = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        positions_object_to_collider = wp.array([[0.5, 0.5, 0.5]], dtype=wp.float32)
        quaternions_object_to_collider = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        num_colliders_per_object = [1]

        output = cu_mg.impl.utils.batch_compute_collider_transforms(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            num_colliders_per_object=num_colliders_per_object,
        )

        # With identity base frame, world and base positions should be the same
        expected_position = np.array([[1.5, 2.5, 3.5]])
        self.assertTrue(np.allclose(output.positions_base_to_collider.numpy(), expected_position, atol=1e-5))
        self.assertTrue(np.allclose(output.positions_world_to_collider.numpy(), expected_position, atol=1e-5))

    # ============================================================================
    # Test compute_collider_transforms_cpu (NumPy mirror of the kernel)
    # ============================================================================

    async def test_compute_collider_transforms_cpu_identity(self) -> None:
        """CPU mirror produces identity-case results matching the kernel test."""
        position_base_to_world = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        quaternion_base_to_world = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)

        positions_world_to_object = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
        quaternions_world_to_object = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=np.float32)

        positions_object_to_collider = np.array(
            [[0.1, 0.0, 0.0], [0.2, 0.0, 0.0], [0.3, 0.0, 0.0], [0.4, 0.0, 0.0], [0.5, 0.0, 0.0]],
            dtype=np.float32,
        )
        quaternions_object_to_collider = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (5, 1))
        collider_to_object_indices = np.array([0, 0, 1, 1, 1], dtype=np.int32)

        pos_base, quat_base, pos_world, quat_world = cu_mg.impl.utils.compute_collider_transforms_cpu(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            collider_to_object_indices=collider_to_object_indices,
        )

        expected_positions = np.array(
            [[1.1, 0.0, 0.0], [1.2, 0.0, 0.0], [2.3, 0.0, 0.0], [2.4, 0.0, 0.0], [2.5, 0.0, 0.0]],
            dtype=np.float32,
        )
        self.assertTrue(np.allclose(pos_base, expected_positions, atol=1e-5))
        self.assertTrue(np.allclose(pos_world, expected_positions, atol=1e-5))
        # All quaternions should remain identity (w == 1, xyz == 0).
        self.assertTrue(np.allclose(quat_base, np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), atol=1e-5))
        self.assertTrue(np.allclose(quat_world, np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), atol=1e-5))

    async def test_compute_collider_transforms_cpu_with_base_translation(self) -> None:
        """CPU mirror applies a translated base frame correctly."""
        position_base_to_world = np.array([[-1.0, 0.0, 0.0]], dtype=np.float32)
        quaternion_base_to_world = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)

        positions_world_to_object = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        quaternions_world_to_object = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        positions_object_to_collider = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        quaternions_object_to_collider = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        collider_to_object_indices = np.array([0], dtype=np.int32)

        pos_base, _, pos_world, _ = cu_mg.impl.utils.compute_collider_transforms_cpu(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            collider_to_object_indices=collider_to_object_indices,
        )

        self.assertTrue(np.allclose(pos_world, np.array([[1.0, 0.0, 0.0]], dtype=np.float32), atol=1e-5))
        self.assertTrue(np.allclose(pos_base, np.array([[0.0, 0.0, 0.0]], dtype=np.float32), atol=1e-5))

    async def test_compute_collider_transforms_cpu_with_rotation(self) -> None:
        """CPU mirror handles a 90-degree base rotation about Z."""
        position_base_to_world = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        quaternion_base_to_world = np.array([[np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]], dtype=np.float32)

        positions_world_to_object = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        quaternions_world_to_object = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        positions_object_to_collider = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        quaternions_object_to_collider = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        collider_to_object_indices = np.array([0], dtype=np.int32)

        pos_base, quat_base, pos_world, quat_world = cu_mg.impl.utils.compute_collider_transforms_cpu(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            collider_to_object_indices=collider_to_object_indices,
        )

        # World frame: collider at object origin.
        self.assertTrue(np.allclose(pos_world, np.array([[1.0, 0.0, 0.0]], dtype=np.float32), atol=1e-5))
        self.assertTrue(np.allclose(quat_world, np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), atol=1e-5))
        # Base frame: rotating [1,0,0] by +90deg about Z yields [0,1,0].
        self.assertTrue(np.allclose(pos_base, np.array([[0.0, 1.0, 0.0]], dtype=np.float32), atol=1e-5))
        self.assertTrue(
            np.allclose(
                quat_base,
                np.array([[np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]], dtype=np.float32),
                atol=1e-5,
            )
        )

    async def test_compute_collider_transforms_cpu_accepts_1d_base_pose(self) -> None:
        """The CPU mirror documents that the base pose may be ``(3,)`` / ``(4,)``."""
        position_base_to_world = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # (3,)
        quaternion_base_to_world = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # (4,)

        positions_world_to_object = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        quaternions_world_to_object = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        positions_object_to_collider = np.array([[0.5, 0.5, 0.5]], dtype=np.float32)
        quaternions_object_to_collider = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        collider_to_object_indices = np.array([0], dtype=np.int32)

        pos_base, _, pos_world, _ = cu_mg.impl.utils.compute_collider_transforms_cpu(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            collider_to_object_indices=collider_to_object_indices,
        )

        expected = np.array([[1.5, 2.5, 3.5]], dtype=np.float32)
        self.assertTrue(np.allclose(pos_base, expected, atol=1e-5))
        self.assertTrue(np.allclose(pos_world, expected, atol=1e-5))

    async def test_compute_collider_transforms_cpu_zero_colliders(self) -> None:
        """``M = 0`` is a legal input (cache build can produce empty buffers); shapes must be preserved."""
        position_base_to_world = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        quaternion_base_to_world = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)

        positions_world_to_object = np.zeros((0, 3), dtype=np.float32)
        quaternions_world_to_object = np.zeros((0, 4), dtype=np.float32)
        positions_object_to_collider = np.zeros((0, 3), dtype=np.float32)
        quaternions_object_to_collider = np.zeros((0, 4), dtype=np.float32)
        collider_to_object_indices = np.zeros((0,), dtype=np.int32)

        pos_base, quat_base, pos_world, quat_world = cu_mg.impl.utils.compute_collider_transforms_cpu(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            collider_to_object_indices=collider_to_object_indices,
        )

        self.assertEqual(pos_base.shape, (0, 3))
        self.assertEqual(quat_base.shape, (0, 4))
        self.assertEqual(pos_world.shape, (0, 3))
        self.assertEqual(quat_world.shape, (0, 4))

    async def test_compute_collider_transforms_cpu_returns_float32(self) -> None:
        """``float64`` inputs should still produce ``float32`` outputs (matches the kernel signature)."""
        position_base_to_world = np.array([[0.5, 0.0, 0.0]], dtype=np.float64)
        quaternion_base_to_world = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64)

        positions_world_to_object = np.array([[1.0, 2.0, 3.0]], dtype=np.float64)
        quaternions_world_to_object = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64)
        positions_object_to_collider = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
        quaternions_object_to_collider = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64)
        collider_to_object_indices = np.array([0], dtype=np.int32)

        pos_base, quat_base, pos_world, quat_world = cu_mg.impl.utils.compute_collider_transforms_cpu(
            position_base_to_world=position_base_to_world,
            quaternion_base_to_world=quaternion_base_to_world,
            positions_world_to_object=positions_world_to_object,
            quaternions_world_to_object=quaternions_world_to_object,
            positions_object_to_collider=positions_object_to_collider,
            quaternions_object_to_collider=quaternions_object_to_collider,
            collider_to_object_indices=collider_to_object_indices,
        )

        self.assertEqual(pos_base.dtype, np.float32)
        self.assertEqual(quat_base.dtype, np.float32)
        self.assertEqual(pos_world.dtype, np.float32)
        self.assertEqual(quat_world.dtype, np.float32)

    # ============================================================================
    # CPU vs Warp kernel parity
    # ============================================================================

    async def test_cpu_and_kernel_agree_on_random_inputs(self) -> None:
        """CPU mirror and Warp kernel produce numerically identical outputs."""
        rng = np.random.default_rng(seed=42)
        n_objects = 7
        colliders_per_object = [1, 2, 3, 1, 4, 1, 2]
        m_colliders = sum(colliders_per_object)

        position_base_to_world_np = rng.standard_normal((1, 3)).astype(np.float32)
        quaternion_base_to_world_np = _random_unit_quaternions(1, rng)
        positions_world_to_object_np = rng.standard_normal((n_objects, 3)).astype(np.float32)
        quaternions_world_to_object_np = _random_unit_quaternions(n_objects, rng)
        positions_object_to_collider_np = rng.standard_normal((m_colliders, 3)).astype(np.float32)
        quaternions_object_to_collider_np = _random_unit_quaternions(m_colliders, rng)
        collider_to_object_indices_np = np.repeat(np.arange(n_objects, dtype=np.int32), colliders_per_object)

        cpu_pos_base, cpu_quat_base, cpu_pos_world, cpu_quat_world = cu_mg.impl.utils.compute_collider_transforms_cpu(
            position_base_to_world=position_base_to_world_np,
            quaternion_base_to_world=quaternion_base_to_world_np,
            positions_world_to_object=positions_world_to_object_np,
            quaternions_world_to_object=quaternions_world_to_object_np,
            positions_object_to_collider=positions_object_to_collider_np,
            quaternions_object_to_collider=quaternions_object_to_collider_np,
            collider_to_object_indices=collider_to_object_indices_np,
        )

        gpu_output = cu_mg.impl.utils.batch_compute_collider_transforms(
            position_base_to_world=wp.from_numpy(position_base_to_world_np, dtype=wp.float32),
            quaternion_base_to_world=wp.from_numpy(quaternion_base_to_world_np, dtype=wp.float32),
            positions_world_to_object=wp.from_numpy(positions_world_to_object_np, dtype=wp.float32),
            quaternions_world_to_object=wp.from_numpy(quaternions_world_to_object_np, dtype=wp.float32),
            positions_object_to_collider=wp.from_numpy(positions_object_to_collider_np, dtype=wp.float32),
            quaternions_object_to_collider=wp.from_numpy(quaternions_object_to_collider_np, dtype=wp.float32),
            num_colliders_per_object=colliders_per_object,
            collider_to_object_indices=wp.from_numpy(collider_to_object_indices_np, dtype=wp.int32),
        )

        atol = 1e-5
        self.assertTrue(np.allclose(cpu_pos_base, gpu_output.positions_base_to_collider.numpy(), atol=atol))
        self.assertTrue(np.allclose(cpu_pos_world, gpu_output.positions_world_to_collider.numpy(), atol=atol))
        # Quaternions can differ by sign and still represent the same rotation.
        gpu_quat_base = gpu_output.quaternions_base_to_collider.numpy()
        gpu_quat_world = gpu_output.quaternions_world_to_collider.numpy()
        for cpu_q, gpu_q in ((cpu_quat_base, gpu_quat_base), (cpu_quat_world, gpu_quat_world)):
            sign_aligned = np.where(np.sum(cpu_q * gpu_q, axis=1, keepdims=True) < 0.0, -gpu_q, gpu_q)
            self.assertTrue(np.allclose(cpu_q, sign_aligned, atol=atol))
