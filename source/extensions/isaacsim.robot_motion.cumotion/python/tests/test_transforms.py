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

"""Test suite for transforms utilities class."""

import cumotion
import isaacsim.robot_motion.cumotion as cu_mg
import numpy as np
import omni.kit.test
import warp as wp


class TestCumotionToIsaacSimTransforms(omni.kit.test.AsyncTestCase):
    """Test suite for transforms utilities with Franka robot."""

    async def setUp(self) -> None:
        """Sets up the test environment before each test method."""

    async def tearDown(self) -> None:
        """Cleans up the test environment after each test method."""

    # ============================================================================
    # Test initialization
    # ============================================================================

    async def test_isaac_sim_to_cumotion_translations_with_identity_base_frame(self) -> None:
        """Tests Isaac Sim to cumotion translation conversion with an identity base frame.

        Verifies that when the world and base frames coincide (identity transformation),
        the translation remains unchanged during conversion.
        """
        # Identity base frame (world and base coincide):
        position_world_to_base = np.array([0.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        cumotion_translation = cu_mg.impl.utils.isaac_sim_to_cumotion_translation(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_translation, [1.0, 0.0, 0.0]))

    async def test_isaac_sim_to_cumotion_translations_with_translated_base_frame(self) -> None:
        """Tests Isaac Sim to cumotion translation conversion with a translated base frame.

        Verifies that translations are correctly adjusted when the base frame is shifted
        relative to the world frame.
        """
        # Shift the base frame back one unit in the world:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        cumotion_translation = cu_mg.impl.utils.isaac_sim_to_cumotion_translation(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_translation, [2.0, 0.0, 0.0]))

    async def test_isaac_sim_to_cumotion_translations_with_translated_and_rotated_base_frame(self) -> None:
        """Tests Isaac Sim to cumotion translation conversion with translated and rotated base frame.

        Verifies that translations are correctly transformed when the base frame has both
        translational and rotational offsets relative to the world frame. Tests rotations
        about x, y, and z axes.
        """
        # Rotate the base frame 90 degrees about the y-axis,
        # which should now mean the target point is at +2 in the z-direction:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]  # rotation of 90 degrees about the y-axis
        )

        cumotion_translation = cu_mg.impl.utils.isaac_sim_to_cumotion_translation(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_translation, [0.0, 0.0, 2.0]))

        # Rotate the base frame 90 degrees about the x-axis,
        # which should leave the points at +2 in x-direction:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]  # rotation of 90 degrees about the x-axis
        )

        cumotion_translation = cu_mg.impl.utils.isaac_sim_to_cumotion_translation(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_translation, [2.0, 0.0, 0.0]))

        # Rotate the base frame 90 degrees about the z-axis,
        # which should leave the points at -2 in y-direction:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]  # rotation of 90 degrees about the z-axis
        )

        cumotion_translation = cu_mg.impl.utils.isaac_sim_to_cumotion_translation(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_translation, [0.0, -2.0, 0.0]))

    async def test_degenerate_inputs_isaac_sim_to_cumotion_translation(self) -> None:
        """Tests that invalid inputs raise appropriate errors for Isaac Sim to cumotion translation.

        Verifies that ValueError is raised for translations with incorrect dimensions
        or batch sizes.
        """
        with self.assertRaises(ValueError):
            # attempt to pass a translation which is not of size 3:
            cumotion_translation = cu_mg.impl.utils.isaac_sim_to_cumotion_translation(
                position_world_to_target=wp.array([0.0, 0.0]),
            )

        with self.assertRaises(ValueError):
            # attempt to pass in more than one translation:
            cumotion_translation = cu_mg.impl.utils.isaac_sim_to_cumotion_translation(
                position_world_to_target=wp.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]),
            )

    # ============================================================================
    # Test cumotion_to_isaac_sim_translation
    # ============================================================================

    async def test_cumotion_to_isaac_sim_translation_with_identity_base_frame(self) -> None:
        """Tests cumotion to Isaac Sim translation conversion with an identity base frame.

        Verifies that when the world and base frames coincide (identity transformation),
        the translation remains unchanged during conversion.
        """
        # Identity base frame:
        position_world_to_base = np.array([0.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        isaac_sim_translation = cu_mg.impl.utils.cumotion_to_isaac_sim_translation(
            position_base_to_target=np.array([1.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_translation.numpy(), [1.0, 0.0, 0.0]))

    async def test_cumotion_to_isaac_sim_translation_with_translated_base_frame(self) -> None:
        """Tests cumotion to Isaac Sim translation conversion with a translated base frame.

        Verifies that translations are correctly adjusted when the base frame is shifted
        relative to the world frame.
        """
        # Shift the base frame back one unit in the world:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        isaac_sim_translation = cu_mg.impl.utils.cumotion_to_isaac_sim_translation(
            position_base_to_target=np.array([2.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_translation.numpy(), [1.0, 0.0, 0.0]))

    async def test_cumotion_to_isaac_sim_translation_with_translated_and_rotated_base_frame(self) -> None:
        """Tests cumotion to Isaac Sim translation conversion with translated and rotated base frame.

        Verifies that translations are correctly transformed when the base frame has both
        translational and rotational offsets relative to the world frame. Tests rotations
        about x, y, and z axes.
        """
        # Rotate the base frame 90 degrees about the y-axis:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]  # rotation of 90 degrees about the y-axis
        )

        isaac_sim_translation = cu_mg.impl.utils.cumotion_to_isaac_sim_translation(
            position_base_to_target=np.array([0.0, 0.0, 2.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_translation.numpy(), [1.0, 0.0, 0.0]))

        # Rotate the base frame 90 degrees about the x-axis:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]  # rotation of 90 degrees about the x-axis
        )

        isaac_sim_translation = cu_mg.impl.utils.cumotion_to_isaac_sim_translation(
            position_base_to_target=np.array([2.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_translation.numpy(), [1.0, 0.0, 0.0]))

        # Rotate the base frame 90 degrees about the z-axis:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]  # rotation of 90 degrees about the z-axis
        )

        isaac_sim_translation = cu_mg.impl.utils.cumotion_to_isaac_sim_translation(
            position_base_to_target=np.array([0.0, -2.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_translation.numpy(), [1.0, 0.0, 0.0]))

    async def test_degenerate_inputs_cumotion_to_isaac_sim_translation(self) -> None:
        """Tests that invalid inputs raise appropriate errors for cumotion to Isaac Sim translation.

        Verifies that ValueError is raised for translations with incorrect dimensions.
        """
        with self.assertRaises(ValueError):
            # attempt to pass a translation which is not of size 3:
            isaac_sim_translation = cu_mg.impl.utils.cumotion_to_isaac_sim_translation(
                position_base_to_target=np.array([0.0, 0.0]),
            )

    # ============================================================================
    # Test isaac_sim_to_cumotion_rotation
    # ============================================================================

    async def test_rotations_with_identity_base_frame(self) -> None:
        """Tests rotation conversion with identity base frame.

        Verifies that rotations are correctly converted from Isaac Sim to cumotion coordinate system
        when the base frame is identity (world and base frames coincide).
        """
        # Identity base frame with identity rotation:
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        cumotion_rotation = cu_mg.impl.utils.isaac_sim_to_cumotion_rotation(
            orientation_world_to_target=wp.array([1.0, 0.0, 0.0, 0.0]),
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(
            np.allclose(
                [cumotion_rotation.w(), cumotion_rotation.x(), cumotion_rotation.y(), cumotion_rotation.z()],
                [1.0, 0.0, 0.0, 0.0],
            )
        )

    async def test_rotations_with_rotated_base_frame(self) -> None:
        """Tests rotation conversion with rotated base frame.

        Verifies that rotations are correctly converted from Isaac Sim to cumotion coordinate system
        when the base frame is rotated 90 degrees about the y-axis relative to the world frame.
        """
        # Rotate the base frame 90 degrees about the y-axis:
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]  # rotation of 90 degrees about the y-axis
        )

        # Pass identity rotation in world frame:
        cumotion_rotation = cu_mg.impl.utils.isaac_sim_to_cumotion_rotation(
            orientation_world_to_target=wp.array([1.0, 0.0, 0.0, 0.0]),
            orientation_world_to_base=quaternion_world_to_base,
        )

        # In base frame, this should be the inverse of the base frame's rotation:
        # Expected: inverse of 90 degrees about y-axis = -90 degrees about y-axis
        expected_quat = np.array([np.cos(np.pi / 4), 0.0, -np.sin(np.pi / 4), 0.0])
        self.assertTrue(
            np.allclose(
                [cumotion_rotation.w(), cumotion_rotation.x(), cumotion_rotation.y(), cumotion_rotation.z()],
                expected_quat,
            )
        )

    async def test_rotation_composition(self) -> None:
        """Tests rotation composition in coordinate frame conversion.

        Verifies that rotation composition works correctly when converting between coordinate frames,
        specifically testing a case where the result should be 180 degrees about the y-axis.
        """
        # Base frame is rotated by -90 degrees about the y-axis
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, -np.sin(np.pi / 4), 0.0]  # 90 degrees about y-axis
        )

        # Rotation from world to target frame is 90 degrees about the y-axis
        rotation_world_to_target = wp.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])

        cumotion_rotation = cu_mg.impl.utils.isaac_sim_to_cumotion_rotation(
            orientation_world_to_target=rotation_world_to_target, orientation_world_to_base=quaternion_world_to_base
        )

        # Result should be 180 degrees about y-axis in base frame:
        self.assertTrue(
            np.allclose(
                [cumotion_rotation.w(), cumotion_rotation.x(), cumotion_rotation.y(), cumotion_rotation.z()],
                [0.0, 0.0, 1.0, 0.0],
                atol=1e-6,
            )
        )

    async def test_degenerate_inputs_rotation(self) -> None:
        """Tests error handling for invalid rotation inputs.

        Verifies that appropriate ValueError is raised when attempting to pass rotation data
        with incorrect dimensions (not size 4).
        """
        with self.assertRaises(ValueError):
            # attempt to pass a rotation which is not of size 4:
            cumotion_rotation = cu_mg.impl.utils.isaac_sim_to_cumotion_rotation(
                orientation_world_to_target=wp.array([1.0, 0.0, 0.0]),
            )

    # ============================================================================
    # Test cumotion_to_isaac_sim_rotation
    # ============================================================================

    async def test_cumotion_to_isaac_sim_rotation_with_identity_base_frame(self) -> None:
        """Tests rotation conversion from cumotion to Isaac Sim with identity base frame.

        Verifies that rotations are correctly converted from cumotion to Isaac Sim coordinate system
        when the base frame is identity (world and base frames coincide).
        """
        # Identity base frame with identity rotation:
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        isaac_sim_rotation = cu_mg.impl.utils.cumotion_to_isaac_sim_rotation(
            orientation_base_to_target=cumotion.Rotation3(1.0, 0.0, 0.0, 0.0),
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_rotation.numpy(), [1.0, 0.0, 0.0, 0.0]))

    async def test_cumotion_to_isaac_sim_rotation_with_rotated_base_frame(self) -> None:
        """Tests rotation conversion from cumotion to Isaac Sim with rotated base frame.

        Verifies that rotations are correctly converted from cumotion to Isaac Sim coordinate system
        when the base frame is rotated 90 degrees about the y-axis, testing composition of rotations.
        """
        # Rotate the base frame 90 degrees about the y-axis:
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]  # rotation of 90 degrees about the y-axis
        )

        # The target rotation (in the base frame) is a rotation of another 90 degrees.
        rotation_base_to_target = cumotion.Rotation3(np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0)
        isaac_sim_rotation = cu_mg.impl.utils.cumotion_to_isaac_sim_rotation(
            orientation_base_to_target=rotation_base_to_target, orientation_world_to_base=quaternion_world_to_base
        )

        # In world frame, this should be 180 degrees about the y-axis:
        self.assertTrue(np.allclose(isaac_sim_rotation.numpy(), [0.0, 0.0, 1.0, 0.0]))

    async def test_cumotion_to_isaac_sim_rotation_round_trip(self) -> None:
        """Tests round-trip conversion of rotations between coordinate systems.

        Verifies that converting a rotation from Isaac Sim to cumotion and back to Isaac Sim
        produces the original rotation values, ensuring conversion accuracy.
        """
        # Test that converting to cumotion and back gives the same rotation:
        quaternion_world_to_base = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])

        rotation_world_to_target_original = wp.array([np.cos(np.pi / 6), 0.0, np.sin(np.pi / 6), 0.0])

        # Convert to cumotion:
        rotation_base_to_target_cumotion = cu_mg.impl.utils.isaac_sim_to_cumotion_rotation(
            orientation_world_to_target=rotation_world_to_target_original,
            orientation_world_to_base=quaternion_world_to_base,
        )

        # Convert back to isaac sim:
        isaac_sim_rotation = cu_mg.impl.utils.cumotion_to_isaac_sim_rotation(
            orientation_base_to_target=rotation_base_to_target_cumotion,
            orientation_world_to_base=quaternion_world_to_base,
        )

        self.assertTrue(np.allclose(isaac_sim_rotation.numpy(), rotation_world_to_target_original.numpy()))

    async def test_degenerate_inputs_cumotion_to_isaac_sim_rotation(self) -> None:
        """Tests error handling for cumotion to Isaac Sim rotation conversion.

        This test is intentionally empty as the function accepts a cumotion.Rotation3 object
        which is always valid by construction.
        """
        # This test is intentionally empty as there's no degenerate input case for this function
        # The function accepts a cumotion.Rotation3 object which is always valid

    # ============================================================================
    # Test isaac_sim_to_cumotion_pose
    # ============================================================================

    async def test_pose_with_identity_base_frame(self) -> None:
        """Tests pose conversion with identity base frame.

        Verifies that poses (position and orientation) are correctly converted from Isaac Sim
        to cumotion coordinate system when the base frame is identity.
        """
        # Identity base frame:
        position_world_to_base = np.array([0.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        cumotion_pose = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            orientation_world_to_target=wp.array([1.0, 0.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_pose.translation, [1.0, 0.0, 0.0]))
        self.assertTrue(
            np.allclose(
                [
                    cumotion_pose.rotation.w(),
                    cumotion_pose.rotation.x(),
                    cumotion_pose.rotation.y(),
                    cumotion_pose.rotation.z(),
                ],
                [1.0, 0.0, 0.0, 0.0],
            )
        )

    async def test_pose_with_translated_base_frame(self) -> None:
        """Tests pose conversion with translated base frame.

        Verifies that poses are correctly converted from Isaac Sim to cumotion coordinate system
        when the base frame is translated by one unit in the negative x-direction.
        """
        # Shift the base frame back one unit in the world:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        cumotion_pose = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            orientation_world_to_target=wp.array([1.0, 0.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_pose.translation, [2.0, 0.0, 0.0]))
        self.assertTrue(
            np.allclose(
                [
                    cumotion_pose.rotation.w(),
                    cumotion_pose.rotation.x(),
                    cumotion_pose.rotation.y(),
                    cumotion_pose.rotation.z(),
                ],
                [1.0, 0.0, 0.0, 0.0],
            )
        )

    async def test_pose_with_translated_and_rotated_base_frame(self) -> None:
        """Tests isaac_sim_to_cumotion_pose with a translated and rotated base frame.

        Verifies that pose conversion correctly handles a base frame that is both translated and rotated
        relative to the world coordinate system.
        """
        # Base frame is rotated by -90 degrees about the y-axis
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, -np.sin(np.pi / 4), 0.0]  # -90 degrees about the y-axis
        )

        cumotion_pose = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
            position_world_to_target=wp.array([1.0, 0.0, 0.0]),
            orientation_world_to_target=wp.array([1.0, 0.0, 0.0, 0.0]),
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(cumotion_pose.translation, [0.0, 0.0, -2.0]))

        # Identity rotation in world frame becomes 90 degree rotation about y-axis in base frame:
        expected_quat = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])
        self.assertTrue(
            np.allclose(
                [
                    cumotion_pose.rotation.w(),
                    cumotion_pose.rotation.x(),
                    cumotion_pose.rotation.y(),
                    cumotion_pose.rotation.z(),
                ],
                expected_quat,
            )
        )

    async def test_pose_accepts_different_input_types(self) -> None:
        """Tests isaac_sim_to_cumotion_pose accepts various input data types.

        Verifies that the pose conversion function correctly handles different input formats including
        Warp arrays, NumPy arrays, and Python lists.
        """
        # Test with numpy arrays:
        cumotion_pose = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
            position_world_to_target=np.array([1.0, 0.0, 0.0]),
            orientation_world_to_target=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self.assertTrue(np.allclose(cumotion_pose.translation, [1.0, 0.0, 0.0]))

        # Test with lists:
        cumotion_pose = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
            position_world_to_target=[1.0, 0.0, 0.0], orientation_world_to_target=[1.0, 0.0, 0.0, 0.0]
        )
        self.assertTrue(np.allclose(cumotion_pose.translation, [1.0, 0.0, 0.0]))

    async def test_degenerate_inputs_pose(self) -> None:
        """Tests isaac_sim_to_cumotion_pose with invalid input dimensions.

        Verifies that appropriate ValueError exceptions are raised when position or orientation inputs
        have incorrect dimensions.
        """
        with self.assertRaises(ValueError):
            # attempt to pass a position which is not of size 3:
            cumotion_pose = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
                position_world_to_target=wp.array([0.0, 0.0]),
                orientation_world_to_target=wp.array([1.0, 0.0, 0.0, 0.0]),
            )

        with self.assertRaises(ValueError):
            # attempt to pass an orientation which is not of size 4:
            cumotion_pose = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
                position_world_to_target=wp.array([1.0, 0.0, 0.0]),
                orientation_world_to_target=wp.array([1.0, 0.0, 0.0]),
            )

    # ============================================================================
    # Test cumotion_to_isaac_sim_pose
    # ============================================================================

    async def test_cumotion_to_isaac_sim_pose_with_identity_base_frame(self) -> None:
        """Tests cumotion_to_isaac_sim_pose with an identity base frame.

        Verifies that pose conversion from Cumotion to Isaac Sim coordinate system works correctly when
        the base frame coincides with the world frame.
        """
        # Identity base frame:
        position_world_to_base = np.array([0.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        cumotion_pose = cumotion.Pose3.from_translation(np.array([1.0, 0.0, 0.0]))

        isaac_sim_position, isaac_sim_orientation = cu_mg.impl.utils.cumotion_to_isaac_sim_pose(
            pose_base_to_target=cumotion_pose,
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_position.numpy(), [1.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(isaac_sim_orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))

    async def test_cumotion_to_isaac_sim_pose_with_translated_base_frame(self) -> None:
        """Tests cumotion_to_isaac_sim_pose with a translated base frame.

        Verifies that pose conversion correctly handles a base frame that is translated relative to the
        world coordinate system.
        """
        # Shift the base frame back one unit in the world:
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array([1.0, 0.0, 0.0, 0.0])

        cumotion_pose = cumotion.Pose3.from_translation(np.array([2.0, 0.0, 0.0]))

        isaac_sim_position, isaac_sim_orientation = cu_mg.impl.utils.cumotion_to_isaac_sim_pose(
            pose_base_to_target=cumotion_pose,
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_position.numpy(), [1.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(isaac_sim_orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))

    async def test_cumotion_to_isaac_sim_pose_with_translated_and_rotated_base_frame(self) -> None:
        """Tests cumotion_to_isaac_sim_pose with a translated and rotated base frame.

        Verifies that pose conversion correctly handles a base frame that is both translated and rotated
        relative to the world coordinate system, including proper composition of transformations.
        """
        # Base frame is rotated by -90 degrees about the y-axis
        position_world_to_base = np.array([-1.0, 0.0, 0.0])
        quaternion_world_to_base = np.array(
            [np.cos(np.pi / 4), 0.0, -np.sin(np.pi / 4), 0.0]  # -90 degrees about the y-axis
        )

        # The target pose (in the base frame) has translation [0, 0, 2] and rotation of 90 degrees about y
        pose_base_to_target = cumotion.Pose3.from_translation(
            np.array([0.0, 0.0, -2.0])
        ) * cumotion.Pose3.from_rotation(cumotion.Rotation3(np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0))

        isaac_sim_position, isaac_sim_orientation = cu_mg.impl.utils.cumotion_to_isaac_sim_pose(
            pose_base_to_target=pose_base_to_target,
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )
        self.assertTrue(np.allclose(isaac_sim_position.numpy(), [1.0, 0.0, 0.0]))
        # In world frame, this should be identity rotation (the two 90 degree rotations cancel)
        self.assertTrue(np.allclose(isaac_sim_orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))

    async def test_cumotion_to_isaac_sim_pose_round_trip(self) -> None:
        """Tests round-trip conversion between Isaac Sim and Cumotion pose representations.

        Verifies that converting a pose from Isaac Sim to Cumotion and back to Isaac Sim preserves the
        original pose data with complex base frame transformations.
        """
        # Test that converting to cumotion and back gives the same pose:
        position_world_to_base = np.array([-1.0, 2.0, 0.5])
        quaternion_world_to_base_parts = [
            np.array([np.cos(np.pi / 7), 0.0, np.sin(np.pi / 7), 0.0]),
            np.array([np.cos(np.pi / 7), np.sin(np.pi / 7), 0.0, 0.0]),
            np.array([np.cos(np.pi / 7), 0.0, 0.0, -np.sin(np.pi / 7)]),
        ]

        # Compose rotations
        rotation_world_to_base = (
            cumotion.Rotation3(*quaternion_world_to_base_parts[0])
            * cumotion.Rotation3(*quaternion_world_to_base_parts[1])
            * cumotion.Rotation3(*quaternion_world_to_base_parts[2])
        )

        quaternion_world_to_base = np.array(
            [
                rotation_world_to_base.w(),
                rotation_world_to_base.x(),
                rotation_world_to_base.y(),
                rotation_world_to_base.z(),
            ]
        )

        position_world_to_target_original = wp.array([1.5, -0.5, 0.75])
        orientation_world_to_target_original = wp.array([np.cos(np.pi / 6), 0.0, np.sin(np.pi / 6), 0.0])

        # Convert to cumotion:
        pose_base_to_target_cumotion = cu_mg.impl.utils.isaac_sim_to_cumotion_pose(
            position_world_to_target=position_world_to_target_original,
            orientation_world_to_target=orientation_world_to_target_original,
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )

        # Convert back to isaac sim:
        isaac_sim_position, isaac_sim_orientation = cu_mg.impl.utils.cumotion_to_isaac_sim_pose(
            pose_base_to_target=pose_base_to_target_cumotion,
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )

        self.assertTrue(np.allclose(isaac_sim_position.numpy(), position_world_to_target_original.numpy()))
        self.assertTrue(np.allclose(isaac_sim_orientation.numpy(), orientation_world_to_target_original.numpy()))

    async def test_degenerate_inputs_cumotion_to_isaac_sim_pose(self) -> None:
        """Tests cumotion_to_isaac_sim_pose with degenerate inputs.

        This test is intentionally empty as the function accepts cumotion.Pose3 objects which are
        always valid by construction.
        """
        # This test is intentionally empty as there's no degenerate input case for this function
        # The function accepts a cumotion.Pose3 object which is always valid
