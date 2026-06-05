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

"""Tests for PyTorch transformation utilities."""

import math

import omni.kit.test
import torch
from isaacsim.core.utils.torch.transformations import get_world_from_local_position


class TestTorchTransformations(omni.kit.test.AsyncTestCase):
    """Test suite for torch transformation utilities."""

    async def setUp(self) -> None:
        """Set up test fixtures."""

    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    async def test_get_world_from_local_position_identity(self) -> None:
        """Test with identity rotation - position should just be translated."""
        # Identity quaternion [w, x, y, z] = [1, 0, 0, 0]
        # Global pose: position [1, 2, 3], quaternion [1, 0, 0, 0]
        pose_global = torch.tensor([[1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0]], dtype=torch.float32)
        pos_offset_local = torch.tensor([[0.5, 0.5, 0.5]], dtype=torch.float32)

        result = get_world_from_local_position(pos_offset_local, pose_global)

        # With identity rotation, world pos = local pos + global translation
        expected = torch.tensor([[1.5, 2.5, 3.5]], dtype=torch.float32)
        self.assertTrue(
            torch.allclose(result, expected, atol=1e-5),
            f"Identity rotation failed: got {result}, expected {expected}",
        )

    async def test_get_world_from_local_position_90deg_z_rotation(self) -> None:
        """Test with 90 degree rotation around Z axis.

        This test specifically catches the quaternion format bug where [x, y, z, 0]
        was used instead of [0, x, y, z] for pure quaternion embedding.

        A 90-degree rotation around Z axis transforms:
        - X axis -> Y axis
        - Y axis -> -X axis
        - Z axis -> Z axis

        So a local point at [1, 0, 0] should end up at [0, 1, 0] in rotated frame.
        """
        # 90 degrees around Z axis: quaternion [w, x, y, z] = [cos(45°), 0, 0, sin(45°)]
        angle = math.pi / 2  # 90 degrees
        qw = math.cos(angle / 2)
        qz = math.sin(angle / 2)

        # Global pose: position [0, 0, 0], quaternion [cos(45°), 0, 0, sin(45°)]
        pose_global = torch.tensor([[0.0, 0.0, 0.0, qw, 0.0, 0.0, qz]], dtype=torch.float32)

        # Local point at [1, 0, 0]
        pos_offset_local = torch.tensor([[1.0, 0.0, 0.0]], dtype=torch.float32)

        result = get_world_from_local_position(pos_offset_local, pose_global)

        # After 90 degree Z rotation, [1, 0, 0] -> [0, 1, 0]
        expected = torch.tensor([[0.0, 1.0, 0.0]], dtype=torch.float32)
        self.assertTrue(
            torch.allclose(result, expected, atol=1e-5),
            f"90deg Z rotation failed: got {result}, expected {expected}",
        )

    async def test_get_world_from_local_position_90deg_x_rotation(self) -> None:
        """Test with 90 degree rotation around X axis.

        A 90-degree rotation around X axis transforms:
        - X axis -> X axis
        - Y axis -> Z axis
        - Z axis -> -Y axis

        So a local point at [0, 1, 0] should end up at [0, 0, 1] in rotated frame.
        """
        # 90 degrees around X axis: quaternion [w, x, y, z] = [cos(45°), sin(45°), 0, 0]
        angle = math.pi / 2  # 90 degrees
        qw = math.cos(angle / 2)
        qx = math.sin(angle / 2)

        # Global pose: position [0, 0, 0], quaternion [cos(45°), sin(45°), 0, 0]
        pose_global = torch.tensor([[0.0, 0.0, 0.0, qw, qx, 0.0, 0.0]], dtype=torch.float32)

        # Local point at [0, 1, 0]
        pos_offset_local = torch.tensor([[0.0, 1.0, 0.0]], dtype=torch.float32)

        result = get_world_from_local_position(pos_offset_local, pose_global)

        # After 90 degree X rotation, [0, 1, 0] -> [0, 0, 1]
        expected = torch.tensor([[0.0, 0.0, 1.0]], dtype=torch.float32)
        self.assertTrue(
            torch.allclose(result, expected, atol=1e-5),
            f"90deg X rotation failed: got {result}, expected {expected}",
        )

    async def test_get_world_from_local_position_with_translation(self) -> None:
        """Test rotation combined with translation."""
        # 90 degrees around Z axis with translation [10, 20, 30]
        angle = math.pi / 2  # 90 degrees
        qw = math.cos(angle / 2)
        qz = math.sin(angle / 2)

        pose_global = torch.tensor([[10.0, 20.0, 30.0, qw, 0.0, 0.0, qz]], dtype=torch.float32)
        pos_offset_local = torch.tensor([[1.0, 0.0, 0.0]], dtype=torch.float32)

        result = get_world_from_local_position(pos_offset_local, pose_global)

        # [1, 0, 0] rotated 90deg around Z -> [0, 1, 0], then translated by [10, 20, 30]
        expected = torch.tensor([[10.0, 21.0, 30.0]], dtype=torch.float32)
        self.assertTrue(
            torch.allclose(result, expected, atol=1e-5),
            f"Rotation+translation failed: got {result}, expected {expected}",
        )

    async def test_get_world_from_local_position_batched(self) -> None:
        """Test batched operation with multiple poses."""
        angle = math.pi / 2
        qw = math.cos(angle / 2)
        qz = math.sin(angle / 2)

        # Two different poses
        pose_global = torch.tensor(
            [
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],  # Identity
                [0.0, 0.0, 0.0, qw, 0.0, 0.0, qz],  # 90deg Z
            ],
            dtype=torch.float32,
        )

        pos_offset_local = torch.tensor(
            [
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=torch.float32,
        )

        result = get_world_from_local_position(pos_offset_local, pose_global)

        expected = torch.tensor(
            [
                [1.0, 0.0, 0.0],  # Identity: no change
                [0.0, 1.0, 0.0],  # 90deg Z: X -> Y
            ],
            dtype=torch.float32,
        )
        self.assertTrue(
            torch.allclose(result, expected, atol=1e-5),
            f"Batched operation failed: got {result}, expected {expected}",
        )

    async def test_get_world_from_local_position_180deg_rotation(self) -> None:
        """Test with 180 degree rotation around Z axis.

        This further validates the quaternion convention.
        180 degrees around Z: [1, 0, 0] -> [-1, 0, 0]
        """
        # 180 degrees around Z axis: quaternion [w, x, y, z] = [0, 0, 0, 1]
        angle = math.pi  # 180 degrees
        qw = math.cos(angle / 2)  # = 0
        qz = math.sin(angle / 2)  # = 1

        pose_global = torch.tensor([[0.0, 0.0, 0.0, qw, 0.0, 0.0, qz]], dtype=torch.float32)
        pos_offset_local = torch.tensor([[1.0, 0.0, 0.0]], dtype=torch.float32)

        result = get_world_from_local_position(pos_offset_local, pose_global)

        # After 180 degree Z rotation, [1, 0, 0] -> [-1, 0, 0]
        expected = torch.tensor([[-1.0, 0.0, 0.0]], dtype=torch.float32)
        self.assertTrue(
            torch.allclose(result, expected, atol=1e-5),
            f"180deg Z rotation failed: got {result}, expected {expected}",
        )
