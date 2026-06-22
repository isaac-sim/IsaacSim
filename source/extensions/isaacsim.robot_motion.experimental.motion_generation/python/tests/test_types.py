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

"""Verify motion-generation state containers.

The tests cover joint, spatial, root, and complete robot states; construction by
name and index; ordering and combination semantics; read-only behavior; and
mixed NumPy and Warp backing arrays.
"""

# Import extension python module we are testing with absolute import path, as if we are an external user (i.e. a different extension)
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.robot_motion.experimental.motion_generation import (
    JointState,
    RobotState,
    RootState,
    SpatialState,
    combine_robot_states,
)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of the module will make it auto-discoverable by omni.kit.test
class TestJointState(omni.kit.test.AsyncTestCase):
    """Test class for validating JointState functionality in the isaacsim.robot_motion.experimental.motion_generation extension.

    This test class inherits from omni.kit.test.AsyncTestCase and provides comprehensive testing for JointState creation,
    validation, and behavior. It tests various construction methods including from_name, from_index, and direct constructor
    approaches. The tests verify proper handling of joint spaces, data validation, error conditions, and read-only
    properties.

    The test methods validate:
        - Joint state creation using named joints with from_name method
        - Joint state creation using indices with from_index method
        - Direct constructor validation with data arrays
        - Read-only property enforcement
        - Input validation and error handling for incorrect data types, shapes, and values
        - Proper ordering and indexing of joint data
        - Valid array population for different joint state components
    """

    # Before running each test
    async def setUp(self) -> None:
        """Sets up the test environment before each test method."""

    # After running each test
    async def tearDown(self) -> None:
        """Cleans up the test environment after each test method."""

    async def test_joint_state_from_name(self) -> None:
        """Tests creating JointState objects using joint names for various configurations.

        Verifies joint state creation with different joint spaces, partial states,
        input validation, error handling for invalid inputs, and proper ordering
        of returned joint values.
        """
        # can create a joint state:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0"],
            positions=(["joint_0"], wp.array([0.0])),
            velocities=(["joint_0"], wp.array([0.0])),
            efforts=(["joint_0"], wp.array([0.0])),
        )

        # can create a joint state with larger joint-space
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(["joint_0"], wp.array([0.0])),
            velocities=(["joint_0"], wp.array([0.0])),
            efforts=(["joint_0"], wp.array([0.0])),
        )

        # can mix which types are defined on each joint
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(["joint_0", "joint_1"], wp.array([0.0, 0.1])),
            velocities=(["joint_1"], wp.array([0.0])),
            efforts=(["joint_2"], wp.array([0.0])),
        )

        # we can build partial joint-states (mixing any type)
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(["joint_0", "joint_1"], wp.array([0.0, 0.1])),
        )
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            velocities=(["joint_1"], wp.array([0.0])),
        )
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"], efforts=(["joint_2"], wp.array([0.0]))
        )
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(["joint_0", "joint_1"], wp.array([0.0, 0.1])),
            efforts=(["joint_2"], wp.array([0.0])),
        )

        # positions are always returned in the order of the underlying joint-space:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(["joint_0", "joint_2", "joint_1"], wp.array([0.0, 2.0, 1.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 1.0, 2.0]))
        self.assertTrue(joint_state.position_names == ["joint_0", "joint_1", "joint_2"])
        self.assertTrue(joint_state.position_indices.dtype == wp.int32)
        self.assertTrue(np.allclose(joint_state.position_indices.numpy(), [0, 1, 2]))

        # Can create a joint-state with 2D array, as long as the first dimension only has size 1:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0"],
            positions=(["joint_0"], wp.array([[0.0]])),  # 2D array
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0]))
        self.assertTrue(joint_state.position_names == ["joint_0"])
        self.assertTrue(joint_state.positions.shape == (1,))

        # cannot have 2D array if first dimension is not of size 1:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0"],
                positions=(["joint_0"], wp.array([[0.0], [1.0]])),  # 2D array
            )

        # cannot provide a joint which is outside of the intended joint-space:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(["joint_4"], wp.array([0.0])),  # NOT in the joint space.
                velocities=(["joint_1"], wp.array([0.0])),
                efforts=(["joint_2"], wp.array([0.0])),
            )

        # cannot provide two joint values which define the same thing:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(["joint_0", "joint_0"], wp.array([0.0, 0.1])),  # Both trying to write to joint_0 position.
                velocities=(["joint_1"], wp.array([0.0])),
                efforts=(["joint_2"], wp.array([0.0])),
            )

        # cannot create a joint state with incorrect (non-warp) types
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(["joint_0", "joint_0"], np.array([0.0, 0.1])),  # Both trying to write to joint_0 position.
                velocities=(["joint_1"], wp.array([0.0])),
                efforts=(["joint_2"], wp.array([0.0])),
            )

        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(["joint_0", "joint_0"], wp.array([0.0, 0.1])),  # Both trying to write to joint_0 position.
                velocities=(["joint_1"], [0.0]),
                efforts=(["joint_2"], wp.array([0.0])),
            )

        # cannot build a joint state with no entries:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=[],
            )

        # cannot have length mismatch between names and array (more values than names):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0", "joint_1"],
                positions=(["joint_0", "joint_1"], wp.array([0.0, 1.0, 2.0])),  # 3 values, 2 names
            )

        # cannot have length mismatch between names and array (more names than values):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(["joint_0", "joint_1", "joint_2"], wp.array([0.0, 1.0])),  # 2 values, 3 names
            )

        # cannot have empty array (length 0):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0"],
                positions=(["joint_0"], wp.array([])),  # Empty array
            )

        # cannot have wrong dtype (int32):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0"],
                positions=(["joint_0"], wp.array([0], dtype=wp.int32)),  # int32 instead of float
            )

        # cannot have wrong dtype (int64):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0"],
                positions=(["joint_0"], wp.array([0], dtype=wp.int64)),  # int64 instead of float
            )

        # cannot have empty name list with non-empty array:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_name(
                robot_joint_space=["joint_0"],
                positions=([], wp.array([0.0])),  # Empty names, non-empty array
            )

        # can use float64 dtype explicitly:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(["joint_0", "joint_1"], wp.array([0.0, 1.0], dtype=wp.float64)),
        )
        # underlying warp array is still (always) float32.
        self.assertTrue(joint_state.positions.dtype == wp.float32)

        # velocities are always returned in the order of the underlying joint-space:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            velocities=(["joint_2", "joint_0", "joint_1"], wp.array([2.0, 0.0, 1.0])),
        )
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [0.0, 1.0, 2.0]))
        self.assertTrue(joint_state.velocity_names == ["joint_0", "joint_1", "joint_2"])
        self.assertTrue(np.allclose(joint_state.velocity_indices.numpy(), [0, 1, 2]))

        # efforts are always returned in the order of the underlying joint-space:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            efforts=(["joint_1", "joint_2", "joint_0"], wp.array([1.0, 2.0, 0.0])),
        )
        self.assertTrue(np.allclose(joint_state.efforts.numpy(), [0.0, 1.0, 2.0]))
        self.assertTrue(joint_state.effort_names == ["joint_0", "joint_1", "joint_2"])
        self.assertTrue(np.allclose(joint_state.effort_indices.numpy(), [0, 1, 2]))

        # valid array is correctly populated for positions:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(["joint_0", "joint_2"], wp.array([0.0, 2.0])),
        )
        valid_positions = joint_state.valid_array.numpy()[0, :]  # Row 0 is positions
        self.assertTrue(np.allclose(valid_positions, [True, False, True]))

        # valid array is correctly populated for velocities:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            velocities=(["joint_1"], wp.array([1.0])),
        )
        valid_velocities = joint_state.valid_array.numpy()[1, :]  # Row 1 is velocities
        self.assertTrue(np.allclose(valid_velocities, [False, True, False]))

        # valid array is correctly populated for efforts:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            efforts=(["joint_2"], wp.array([2.0])),
        )
        valid_efforts = joint_state.valid_array.numpy()[2, :]  # Row 2 is efforts
        self.assertTrue(np.allclose(valid_efforts, [False, False, True]))

        # can have same joint name in different types (positions and velocities):
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(["joint_0"], wp.array([0.0])),
            velocities=(["joint_0"], wp.array([1.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [1.0]))
        self.assertTrue(joint_state.position_names == ["joint_0"])
        self.assertTrue(joint_state.velocity_names == ["joint_0"])

        # can have same joint name in all three types:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0"],
            positions=(["joint_0"], wp.array([0.0])),
            velocities=(["joint_0"], wp.array([1.0])),
            efforts=(["joint_0"], wp.array([2.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [1.0]))
        self.assertTrue(np.allclose(joint_state.efforts.numpy(), [2.0]))

        # single-element edge case works correctly:
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0"],
            positions=(["joint_0"], wp.array([42.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [42.0]))
        self.assertTrue(joint_state.position_names == ["joint_0"])

        # large joint space works correctly:
        large_joint_space = [f"joint_{i}" for i in range(20)]
        joint_state = JointState.from_name(
            robot_joint_space=large_joint_space,
            positions=([f"joint_{i}" for i in [0, 5, 10, 15, 19]], wp.array([0.0, 5.0, 10.0, 15.0, 19.0])),
            velocities=([f"joint_{i}" for i in [1, 6, 11, 16]], wp.array([1.0, 6.0, 11.0, 16.0])),
            efforts=([f"joint_{i}" for i in [2, 7, 12, 17]], wp.array([2.0, 7.0, 12.0, 17.0])),
        )
        self.assertEqual(len(joint_state.position_names), 5)
        self.assertEqual(len(joint_state.velocity_names), 4)
        self.assertEqual(len(joint_state.effort_names), 4)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 5.0, 10.0, 15.0, 19.0]))

    async def test_joint_state_from_index(self) -> None:
        """Tests creating JointState objects using joint indices for various configurations.

        Verifies joint state creation with different joint spaces, partial states,
        input validation, error handling for invalid inputs, and proper ordering
        of returned joint values when using index-based specification.
        """
        # can create a joint state:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0"],
            positions=(wp.array([0], dtype=int), wp.array([0.0])),
            velocities=(wp.array([0], dtype=int), wp.array([0.0])),
            efforts=(wp.array([0], dtype=int), wp.array([0.0])),
        )

        # can create a joint state with larger joint-space than the defined joints.
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([0.0])),
            velocities=(wp.array([0], dtype=wp.int32), wp.array([0.0])),
            efforts=(wp.array([0], dtype=wp.int32), wp.array([0.0])),
        )

        # can mix which types are defined on each joint
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([0.0, 0.1])),
            velocities=(wp.array([1], dtype=wp.int32), wp.array([0.0])),
            efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
        )

        # we can build partial joint-states (mixing any type)
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([0.0, 0.1])),
        )
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            velocities=(wp.array([1], dtype=wp.int32), wp.array([0.0])),
        )
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
        )
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([0.0, 0.1])),
            efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
        )

        # positions are always returned in the order of the underlying joint-space:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(wp.array([0, 2, 1], dtype=wp.int32), wp.array([0.0, 2.0, 1.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 1.0, 2.0]))
        self.assertTrue(joint_state.position_names == ["joint_0", "joint_1", "joint_2"])
        self.assertTrue(joint_state.position_indices.dtype == wp.int32)
        self.assertTrue(np.allclose(joint_state.position_indices.numpy(), [0, 1, 2]))

        # can have 2D array instead of 1D for values, as long as the first dimension is of size 1:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([[0.0, 1.0]])),  # 2D array
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 1.0]))
        self.assertTrue(joint_state.position_names == ["joint_0", "joint_1"])
        self.assertTrue(np.allclose(joint_state.position_indices.numpy(), [0, 1]))
        self.assertTrue(joint_state.position_indices.dtype == wp.int32)

        # cannot have 2D array if first dimension is not of size 1:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([[0.0], [1.0]])),  # 2D array
            )

        # cannot provide an index which is outside of the intended joint-space (negative):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(wp.array([-1], dtype=wp.int32), wp.array([0.0])),
            )

        # cannot provide an index which is outside of the intended joint-space (too large):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(wp.array([3], dtype=wp.int32), wp.array([0.0])),  # Index 3 is out of range for 3 joints
            )

        # cannot provide two index values which define the same thing:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(
                    wp.array([0, 0], dtype=wp.int32),
                    wp.array([0.0, 0.1]),
                ),  # Both trying to write to index 0 position.
                velocities=(wp.array([1], dtype=wp.int32), wp.array([0.0])),
                efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
            )

        # cannot create a joint state with incorrect (non-warp) types for values
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(wp.array([0], dtype=wp.int32), np.array([0.0])),  # numpy array instead of warp
                velocities=(wp.array([1], dtype=wp.int32), wp.array([0.0])),
                efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
            )

        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([0.0])),
                velocities=(wp.array([1], dtype=wp.int32), [0.0]),  # list instead of warp array
                efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
            )

        # cannot create a joint state with incorrect (non-warp) types for indices
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(np.array([0]), wp.array([0.0])),  # numpy array instead of warp for indices
                velocities=(wp.array([1], dtype=wp.int32), wp.array([0.0])),
                efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
            )

        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=([0], wp.array([0.0])),  # list instead of warp array for indices
                velocities=(wp.array([1], dtype=wp.int32), wp.array([0.0])),
                efforts=(wp.array([2], dtype=wp.int32), wp.array([0.0])),
            )

        # cannot build a joint state with no entries:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=[],
            )

        # cannot have length mismatch between indices and array (more values than indices):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1"],
                positions=(wp.array([0, 1], dtype=wp.int32), wp.array([0.0, 1.0, 2.0])),  # 3 values, 2 indices
            )

        # cannot have length mismatch between indices and array (more indices than values):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0", "joint_1", "joint_2"],
                positions=(wp.array([0, 1, 2], dtype=wp.int32), wp.array([0.0, 1.0])),  # 2 values, 3 indices
            )

        # cannot have empty array (length 0):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([])),  # Empty array
            )

        # cannot have empty indices (length 0):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0"],
                positions=(wp.array([], dtype=wp.int32), wp.array([0.0])),  # Empty indices
            )

        # cannot have 2D array instead of 1D for indices:
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0"],
                positions=(wp.array([[0]], dtype=wp.int32), wp.array([0.0])),  # 2D indices array
            )

        # cannot have wrong dtype for values (int32):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([0], dtype=wp.int32)),  # int32 instead of float
            )

        # cannot have wrong dtype for values (int64):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([0], dtype=wp.int64)),  # int64 instead of float
            )

        # cannot have wrong dtype for indices (float32):
        with self.assertRaises(ValueError):
            joint_state = JointState.from_index(
                robot_joint_space=["joint_0"],
                positions=(wp.array([0.0], dtype=wp.float32), wp.array([0.0])),  # float32 instead of int for indices
            )

        # can use other float dtype explicitly for values, underlying datatype always
        # remains wp.float32
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([0.0, 1.0], dtype=wp.float64)),
        )
        self.assertTrue(joint_state.positions.dtype == wp.float32)
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([0.0, 1.0], dtype=float)),
        )
        self.assertTrue(joint_state.positions.dtype == wp.float32)
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([0.0, 1.0], dtype=wp.float32)),
        )
        self.assertTrue(joint_state.positions.dtype == wp.float32)

        # velocities are always returned in the order of the underlying joint-space:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            velocities=(wp.array([2, 0, 1], dtype=wp.int32), wp.array([2.0, 0.0, 1.0])),
        )
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [0.0, 1.0, 2.0]))
        self.assertTrue(joint_state.velocity_names == ["joint_0", "joint_1", "joint_2"])
        self.assertTrue(np.allclose(joint_state.velocity_indices.numpy(), [0, 1, 2]))

        # efforts are always returned in the order of the underlying joint-space:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            efforts=(wp.array([1, 2, 0], dtype=wp.int32), wp.array([1.0, 2.0, 0.0])),
        )
        self.assertTrue(np.allclose(joint_state.efforts.numpy(), [0.0, 1.0, 2.0]))
        self.assertTrue(joint_state.effort_names == ["joint_0", "joint_1", "joint_2"])
        self.assertTrue(np.allclose(joint_state.effort_indices.numpy(), [0, 1, 2]))

        # valid array is correctly populated for positions:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(wp.array([0, 2], dtype=wp.int32), wp.array([0.0, 2.0])),
        )
        valid_positions = joint_state.valid_array.numpy()[0, :]  # Row 0 is positions
        self.assertTrue(np.allclose(valid_positions, [True, False, True]))

        # valid array is correctly populated for velocities:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            velocities=(wp.array([1], dtype=wp.int32), wp.array([1.0])),
        )
        valid_velocities = joint_state.valid_array.numpy()[1, :]  # Row 1 is velocities
        self.assertTrue(np.allclose(valid_velocities, [False, True, False]))

        # valid array is correctly populated for efforts:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            efforts=(wp.array([2], dtype=wp.int32), wp.array([2.0])),
        )
        valid_efforts = joint_state.valid_array.numpy()[2, :]  # Row 2 is efforts
        self.assertTrue(np.allclose(valid_efforts, [False, False, True]))

        # can have same joint index in different types (positions and velocities):
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([0.0])),
            velocities=(wp.array([0], dtype=wp.int32), wp.array([1.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [1.0]))
        self.assertTrue(joint_state.position_names == ["joint_0"])
        self.assertTrue(joint_state.velocity_names == ["joint_0"])

        # can have same joint index in all three types:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([0.0])),
            velocities=(wp.array([0], dtype=wp.int32), wp.array([1.0])),
            efforts=(wp.array([0], dtype=wp.int32), wp.array([2.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [1.0]))
        self.assertTrue(np.allclose(joint_state.efforts.numpy(), [2.0]))

        # single-element edge case works correctly:
        joint_state = JointState.from_index(
            robot_joint_space=["joint_0"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([42.0])),
        )
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [42.0]))
        self.assertTrue(joint_state.position_names == ["joint_0"])

        # large joint space works correctly:
        large_joint_space = [f"joint_{i}" for i in range(20)]
        joint_state = JointState.from_index(
            robot_joint_space=large_joint_space,
            positions=(wp.array([0, 5, 10, 15, 19], dtype=wp.int32), wp.array([0.0, 5.0, 10.0, 15.0, 19.0])),
            velocities=(wp.array([1, 6, 11, 16], dtype=wp.int32), wp.array([1.0, 6.0, 11.0, 16.0])),
            efforts=(wp.array([2, 7, 12, 17], dtype=wp.int32), wp.array([2.0, 7.0, 12.0, 17.0])),
        )
        self.assertEqual(len(joint_state.position_names), 5)
        self.assertEqual(len(joint_state.velocity_names), 4)
        self.assertEqual(len(joint_state.effort_names), 4)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 5.0, 10.0, 15.0, 19.0]))

    async def test_joint_state_constructor(self) -> None:
        """Tests creating JointState objects using the direct constructor.

        Verifies joint state creation with raw data and valid arrays,
        validation of array shapes and types, and proper handling of
        valid array flags for determining which joints have valid data.
        """
        # can create a joint state with valid arrays:
        robot_joint_space = ["joint_0", "joint_1", "joint_2"]
        data_array = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 3], dtype=wp.bool, device="cpu")

        # Set some valid positions
        data_array.numpy()[0, 0] = 0.0
        data_array.numpy()[0, 1] = 1.0
        valid_array.numpy()[0, 0] = True
        valid_array.numpy()[0, 1] = True

        joint_state = JointState(robot_joint_space, data_array, valid_array)
        self.assertEqual(joint_state.robot_joint_space, robot_joint_space)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 1.0]))
        self.assertEqual(joint_state.position_names, ["joint_0", "joint_1"])

        # can create a joint state with all three types:
        robot_joint_space = ["joint_0", "joint_1"]
        data_array = wp.zeros(shape=[3, 2], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 2], dtype=wp.bool, device="cpu")

        # Set positions, velocities, and efforts
        data_array.numpy()[0, 0] = 0.0  # position for joint_0
        data_array.numpy()[1, 1] = 1.0  # velocity for joint_1
        data_array.numpy()[2, 0] = 2.0  # effort for joint_0

        valid_array.numpy()[0, 0] = True
        valid_array.numpy()[1, 1] = True
        valid_array.numpy()[2, 0] = True

        joint_state = JointState(robot_joint_space, data_array, valid_array)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [1.0]))
        self.assertTrue(np.allclose(joint_state.efforts.numpy(), [2.0]))
        self.assertEqual(joint_state.position_names, ["joint_0"])
        self.assertEqual(joint_state.velocity_names, ["joint_1"])
        self.assertEqual(joint_state.effort_names, ["joint_0"])

        # can create a joint state with single joint:
        robot_joint_space = ["joint_0"]
        data_array = wp.zeros(shape=[3, 1], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 1], dtype=wp.bool, device="cpu")
        valid_array.numpy()[0, 0] = True
        data_array.numpy()[0, 0] = 42.0

        joint_state = JointState(robot_joint_space, data_array, valid_array)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [42.0]))
        self.assertEqual(joint_state.position_names, ["joint_0"])

        # can create a joint state with large joint space:
        large_joint_space = [f"joint_{i}" for i in range(20)]
        data_array = wp.zeros(shape=[3, 20], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 20], dtype=wp.bool, device="cpu")

        # Set some positions
        for i in [0, 5, 10, 15, 19]:
            data_array.numpy()[0, i] = float(i)
            valid_array.numpy()[0, i] = True

        joint_state = JointState(large_joint_space, data_array, valid_array)
        self.assertEqual(len(joint_state.position_names), 5)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 5.0, 10.0, 15.0, 19.0]))

        # cannot create with data_array that is not a wp.array:
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = np.zeros(shape=[3, 1], dtype=np.float32)
            valid_array = wp.zeros(shape=[3, 1], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with valid_array that is not a wp.array:
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3, 1], dtype=wp.float32, device="cpu")
            valid_array = np.zeros(shape=[3, 1], dtype=bool)
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with data_array wrong dtype (float64):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3, 1], dtype=wp.float64, device="cpu")
            valid_array = wp.zeros(shape=[3, 1], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with data_array wrong dtype (int32):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3, 1], dtype=wp.int32, device="cpu")
            valid_array = wp.zeros(shape=[3, 1], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with valid_array wrong dtype (int32):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3, 1], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[3, 1], dtype=wp.int32, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with data_array wrong ndim (1D):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[3, 1], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with data_array wrong ndim (3D):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3, 1, 1], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[3, 1], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with valid_array wrong ndim (1D):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3, 1], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[3], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with valid_array wrong ndim (3D):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0"]
            data_array = wp.zeros(shape=[3, 1], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[3, 1, 1], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with data_array wrong shape (wrong first dimension):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0", "joint_1"]
            data_array = wp.zeros(shape=[2, 2], dtype=wp.float32, device="cpu")  # Should be [3, 2]
            valid_array = wp.zeros(shape=[3, 2], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with data_array wrong shape (wrong second dimension):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0", "joint_1"]
            data_array = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")  # Should be [3, 2]
            valid_array = wp.zeros(shape=[3, 2], dtype=wp.bool, device="cpu")
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with valid_array wrong shape (wrong first dimension):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0", "joint_1"]
            data_array = wp.zeros(shape=[3, 2], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[2, 2], dtype=wp.bool, device="cpu")  # Should be [3, 2]
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with valid_array wrong shape (wrong second dimension):
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0", "joint_1"]
            data_array = wp.zeros(shape=[3, 2], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[3, 3], dtype=wp.bool, device="cpu")  # Should be [3, 2]
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # cannot create with shape mismatch between data_array and valid_array:
        with self.assertRaises(ValueError):
            robot_joint_space = ["joint_0", "joint_1"]
            data_array = wp.zeros(shape=[3, 2], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[3, 1], dtype=wp.bool, device="cpu")  # Different shape
            joint_state = JointState(robot_joint_space, data_array, valid_array)

        # can create with empty robot_joint_space (but no valid data):
        robot_joint_space = []
        data_array = wp.zeros(shape=[3, 0], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 0], dtype=wp.bool, device="cpu")
        joint_state = JointState(robot_joint_space, data_array, valid_array)
        self.assertEqual(len(joint_state.robot_joint_space), 0)
        self.assertIsNone(joint_state.positions)
        self.assertIsNone(joint_state.velocities)
        self.assertIsNone(joint_state.efforts)

        # positions are returned in the order of the underlying joint-space:
        robot_joint_space = ["joint_0", "joint_1", "joint_2"]
        data_array = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 3], dtype=wp.bool, device="cpu")

        # Set positions in non-sequential order
        data_array.numpy()[0, 0] = 0.0
        data_array.numpy()[0, 2] = 2.0
        data_array.numpy()[0, 1] = 1.0
        valid_array.numpy()[0, :] = True

        joint_state = JointState(robot_joint_space, data_array, valid_array)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0, 1.0, 2.0]))
        self.assertEqual(joint_state.position_names, ["joint_0", "joint_1", "joint_2"])

        # valid array is correctly used to determine which joints have valid data:
        robot_joint_space = ["joint_0", "joint_1", "joint_2"]
        data_array = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 3], dtype=wp.bool, device="cpu")

        # Set data for all, but only mark some as valid
        data_array.numpy()[0, :] = [0.0, 1.0, 2.0]
        data_array.numpy()[1, :] = [3.0, 4.0, 5.0]
        data_array.numpy()[2, :] = [6.0, 7.0, 8.0]

        valid_array.numpy()[0, 0] = True  # Only joint_0 has valid position
        valid_array.numpy()[1, 1] = True  # Only joint_1 has valid velocity
        valid_array.numpy()[2, 2] = True  # Only joint_2 has valid effort

        joint_state = JointState(robot_joint_space, data_array, valid_array)
        self.assertTrue(np.allclose(joint_state.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(joint_state.velocities.numpy(), [4.0]))
        self.assertTrue(np.allclose(joint_state.efforts.numpy(), [8.0]))
        self.assertEqual(joint_state.position_names, ["joint_0"])
        self.assertEqual(joint_state.velocity_names, ["joint_1"])
        self.assertEqual(joint_state.effort_names, ["joint_2"])

    async def test_joint_state_is_read_only(self) -> None:
        """Tests that JointState objects are immutable after creation.

        Verifies that all properties of a JointState cannot be modified
        after instantiation, ensuring data integrity by preventing
        accidental modifications to joint state data.
        """
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1", "joint_2"],
            positions=(["joint_0", "joint_1"], wp.array([0.0, 0.1])),
            velocities=(["joint_1"], wp.array([0.0])),
            efforts=(["joint_2"], wp.array([0.0])),
        )

        # Cannot write to any fields:
        with self.assertRaises(AttributeError):
            joint_state.positions = wp.zeros_like(joint_state.positions)
        with self.assertRaises(AttributeError):
            joint_state.velocities = wp.zeros_like(joint_state.velocities)
        with self.assertRaises(AttributeError):
            joint_state.efforts = wp.zeros_like(joint_state.efforts)
        with self.assertRaises(AttributeError):
            joint_state.data_array = wp.zeros_like(joint_state.data_array)
        with self.assertRaises(AttributeError):
            joint_state.valid_array = wp.zeros_like(joint_state.valid_array)
        with self.assertRaises(AttributeError):
            joint_state.position_names = wp.zeros_like(joint_state.position_names)
        with self.assertRaises(AttributeError):
            joint_state.position_indices = wp.zeros_like(joint_state.position_indices)
        with self.assertRaises(AttributeError):
            joint_state.velocity_names = wp.zeros_like(joint_state.velocity_names)
        with self.assertRaises(AttributeError):
            joint_state.velocity_indices = wp.zeros_like(joint_state.velocity_indices)
        with self.assertRaises(AttributeError):
            joint_state.effort_names = wp.zeros_like(joint_state.effort_names)
        with self.assertRaises(AttributeError):
            joint_state.effort_indices = wp.zeros_like(joint_state.effort_indices)
        with self.assertRaises(AttributeError):
            joint_state.robot_joint_space = ["new", "joint", "space"]


class TestSpatialState(omni.kit.test.AsyncTestCase):
    """Test class for the SpatialState class from the isaacsim.robot_motion.experimental.motion_generation extension.

    This class contains comprehensive test cases for creating, manipulating, and validating SpatialState objects
    which represent spatial state data for robot frames including positions, orientations, linear velocities,
    and angular velocities. Tests cover both name-based and index-based creation methods, constructor validation,
    and read-only property enforcement.

    The tests verify proper handling of spatial coordinate frames, Warp array operations, data validation,
    error conditions, and edge cases to ensure the SpatialState implementation is robust and reliable for
    robot motion generation applications.
    """

    # Before running each test
    async def setUp(self) -> None:
        """Sets up test fixtures before each test method."""

    # After running each test
    async def tearDown(self) -> None:
        """Tears down test fixtures after each test method."""

    async def test_spatial_state_from_name(self) -> None:
        """Tests creating SpatialState instances using the from_name method with various configurations and validates error handling."""
        # can create a spatial state:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0"],
            positions=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
            orientations=(["frame_0"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
        )

        # can create a spatial state with larger spatial-space
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1"],
            positions=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
            orientations=(["frame_0"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
        )

        # can mix which types are defined on each frame
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(["frame_0", "frame_1"], wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
            orientations=(["frame_1"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
        )

        # we can build partial spatial-states (mixing any type)
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(["frame_0", "frame_1"], wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            orientations=(["frame_1"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(["frame_0", "frame_1"], wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
            angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
        )

        # positions are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(
                ["frame_0", "frame_2", "frame_1"],
                wp.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        self.assertTrue(spatial_state.position_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(spatial_state.position_indices.dtype == wp.int32)
        self.assertTrue(np.allclose(spatial_state.position_indices.numpy(), [0, 1, 2]))

        # cannot provide a frame which is outside of the intended spatial-space:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(["frame_4"], wp.array([[0.0, 0.0, 0.0]])),  # NOT in the spatial space.
                orientations=(["frame_1"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
            )

        # cannot create a spatial-state which defines the same frame twice:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(["frame_0", "frame_0"], wp.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])),
                orientations=(["frame_1"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
            )

        # cannot create a spatial state with incorrect (non-warp) types
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(["frame_0"], np.array([[0.0, 0.0, 0.0]])),  # numpy array instead of warp
                orientations=(["frame_1"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
            )

        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
                orientations=(["frame_1"], [[1.0, 0.0, 0.0, 0.0]]),  # list instead of warp array
                linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
            )

        # cannot build a spatial state with no entries:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=[],
            )

        # cannot have length mismatch between names and array (more values than names):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0", "frame_1"],
                positions=(
                    ["frame_0", "frame_1"],
                    wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
                ),  # 3 values, 2 names
            )

        # cannot have length mismatch between names and array (more names than values):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(
                    ["frame_0", "frame_1", "frame_2"],
                    wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
                ),  # 2 values, 3 names
            )

        # cannot have empty array (shape[0] == 0):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=(["frame_0"], wp.array([[]]).reshape([0, 3])),  # Empty array
            )

        # cannot have 1D array instead of 2D:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=(["frame_0"], wp.array([0.0, 0.0, 0.0])),  # 1D array
            )

        # cannot have 3D array instead of 2D:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=(["frame_0"], wp.array([[[0.0, 0.0, 0.0]]])),  # 3D array
            )

        # cannot have wrong dtype (int32):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=(["frame_0"], wp.array([[0, 0, 0]], dtype=wp.int32)),  # int32 instead of float
            )

        # cannot have wrong dtype (int64):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=(["frame_0"], wp.array([[0, 0, 0]], dtype=wp.int64)),  # int64 instead of float
            )

        # cannot have empty name list with non-empty array:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=([], wp.array([[0.0, 0.0, 0.0]])),  # Empty names, non-empty array
            )

        # cannot have positions with wrong shape[1] (should be 3):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=(["frame_0"], wp.array([[0.0, 0.0]])),  # shape[1] == 2, should be 3
            )

        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                positions=(["frame_0"], wp.array([[0.0, 0.0, 0.0, 0.0]])),  # shape[1] == 4, should be 3
            )

        # cannot have orientations with wrong shape[1] (should be 4):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                orientations=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),  # shape[1] == 3, should be 4
            )

        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                orientations=(["frame_0"], wp.array([[0.0, 0.0, 0.0, 0.0, 0.0]])),  # shape[1] == 5, should be 4
            )

        # cannot have linear_velocities with wrong shape[1] (should be 3):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                linear_velocities=(["frame_0"], wp.array([[0.0, 0.0]])),  # shape[1] == 2, should be 3
            )

        # cannot have angular_velocities with wrong shape[1] (should be 3):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_name(
                spatial_space=["frame_0"],
                angular_velocities=(["frame_0"], wp.array([[0.0, 0.0]])),  # shape[1] == 2, should be 3
            )

        # can use other float dtype explicitly for values, underlying datatype always
        # remains wp.float32
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1"],
            positions=(["frame_0", "frame_1"], wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=wp.float64)),
        )
        self.assertTrue(spatial_state.positions.dtype == wp.float32)
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1"],
            positions=(["frame_0", "frame_1"], wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=float)),
        )
        self.assertTrue(spatial_state.positions.dtype == wp.float32)
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1"],
            positions=(["frame_0", "frame_1"], wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=wp.float32)),
        )
        self.assertTrue(spatial_state.positions.dtype == wp.float32)

        # orientations are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            orientations=(
                ["frame_2", "frame_0", "frame_1"],
                wp.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(
                spatial_state.orientations.numpy(), [[0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0]]
            )
        )
        self.assertTrue(spatial_state.orientation_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(np.allclose(spatial_state.orientation_indices.numpy(), [0, 1, 2]))

        # linear_velocities are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            linear_velocities=(
                ["frame_1", "frame_2", "frame_0"],
                wp.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(spatial_state.linear_velocities.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        self.assertTrue(spatial_state.linear_velocity_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(np.allclose(spatial_state.linear_velocity_indices.numpy(), [0, 1, 2]))

        # angular_velocities are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            angular_velocities=(
                ["frame_1", "frame_2", "frame_0"],
                wp.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(spatial_state.angular_velocities.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        self.assertTrue(spatial_state.angular_velocity_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(np.allclose(spatial_state.angular_velocity_indices.numpy(), [0, 1, 2]))

        # valid array is correctly populated for positions:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(["frame_0", "frame_2"], wp.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])),
        )
        valid_positions = spatial_state.valid_array.numpy()[:, 0]  # Column 0 is positions
        self.assertTrue(np.allclose(valid_positions, [True, False, True]))

        # valid array is correctly populated for orientations:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            orientations=(["frame_1"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
        )
        valid_orientations = spatial_state.valid_array.numpy()[:, 1]  # Column 1 is orientations
        self.assertTrue(np.allclose(valid_orientations, [False, True, False]))

        # valid array is correctly populated for linear_velocities:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
        )
        valid_velocities = spatial_state.valid_array.numpy()[:, 2]  # Column 2 is linear_velocities
        self.assertTrue(np.allclose(valid_velocities, [False, False, True]))

        # valid array is correctly populated for angular_velocities:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            angular_velocities=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
        )
        valid_angular_velocities = spatial_state.valid_array.numpy()[:, 3]  # Column 3 is angular_velocities
        self.assertTrue(np.allclose(valid_angular_velocities, [True, False, False]))

        # can have same frame name in different types (positions and orientations):
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1"],
            positions=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
            orientations=(["frame_0"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.orientations.numpy(), [[1.0, 0.0, 0.0, 0.0]]))
        self.assertTrue(spatial_state.position_names == ["frame_0"])
        self.assertTrue(spatial_state.orientation_names == ["frame_0"])

        # can have same frame name in all four types:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0"],
            positions=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
            orientations=(["frame_0"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(["frame_0"], wp.array([[0.0, 0.0, 0.0]])),
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.orientations.numpy(), [[1.0, 0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.linear_velocities.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.angular_velocities.numpy(), [[0.0, 0.0, 0.0]]))

        # single-element edge case works correctly:
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0"],
            positions=(["frame_0"], wp.array([[42.0, 0.0, 0.0]])),
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[42.0, 0.0, 0.0]]))
        self.assertTrue(spatial_state.position_names == ["frame_0"])

        # large spatial space works correctly:
        large_spatial_space = [f"frame_{i}" for i in range(20)]
        spatial_state = SpatialState.from_name(
            spatial_space=large_spatial_space,
            positions=(
                [f"frame_{i}" for i in [0, 5, 10, 15, 19]],
                wp.array([[float(i), 0.0, 0.0] for i in [0, 5, 10, 15, 19]]),
            ),
            orientations=(
                [f"frame_{i}" for i in [1, 6, 11, 16]],
                wp.array([[1.0, 0.0, 0.0, 0.0] for _ in [1, 6, 11, 16]]),
            ),
            linear_velocities=(
                [f"frame_{i}" for i in [2, 7, 12, 17]],
                wp.array([[float(i), 0.0, 0.0] for i in [2, 7, 12, 17]]),
            ),
            angular_velocities=(
                [f"frame_{i}" for i in [3, 8, 13, 18]],
                wp.array([[float(i), 0.0, 0.0] for i in [3, 8, 13, 18]]),
            ),
        )
        self.assertEqual(len(spatial_state.position_names), 5)
        self.assertEqual(len(spatial_state.orientation_names), 4)
        self.assertEqual(len(spatial_state.linear_velocity_names), 4)
        self.assertEqual(len(spatial_state.angular_velocity_names), 4)
        self.assertTrue(
            np.allclose(
                spatial_state.positions.numpy(),
                [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [15.0, 0.0, 0.0], [19.0, 0.0, 0.0]],
            )
        )

    async def test_spatial_state_from_index(self) -> None:
        """Tests creating SpatialState instances using the from_index method with various configurations and validates error handling."""
        # can create a spatial state:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0"],
            positions=(wp.array([0], dtype=int), wp.array([[0.0, 0.0, 0.0]])),
            orientations=(wp.array([0], dtype=int), wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(wp.array([0], dtype=int), wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(wp.array([0], dtype=int), wp.array([[0.0, 0.0, 0.0]])),
        )

        # can create a spatial state with larger spatial-space than the defined frames.
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            orientations=(wp.array([0], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )

        # can mix which types are defined on each frame
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
            orientations=(wp.array([1], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )

        # we can build partial spatial-states (mixing any type)
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            orientations=(wp.array([1], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
            angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )

        # positions are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(
                wp.array([0, 2, 1], dtype=wp.int32),
                wp.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        self.assertTrue(spatial_state.position_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(spatial_state.position_indices.dtype == wp.int32)
        self.assertTrue(np.allclose(spatial_state.position_indices.numpy(), [0, 1, 2]))

        # cannot provide an index which is outside of the intended spatial-space (negative):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(wp.array([-1], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            )

        # cannot provide an index which is outside of the intended spatial-space (too large):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(
                    wp.array([3], dtype=wp.int32),
                    wp.array([[0.0, 0.0, 0.0]]),
                ),  # Index 3 is out of range for 3 frames
            )

        # cannot provide two index values which define the same thing:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(
                    wp.array([0, 0], dtype=wp.int32),
                    wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]]),
                ),  # Both trying to write to index 0 position.
                orientations=(wp.array([1], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            )

        # cannot create a spatial state with incorrect (non-warp) types for values
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(wp.array([0], dtype=wp.int32), np.array([[0.0, 0.0, 0.0]])),  # numpy array instead of warp
                orientations=(wp.array([1], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            )

        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
                orientations=(wp.array([1], dtype=wp.int32), [[1.0, 0.0, 0.0, 0.0]]),  # list instead of warp array
                linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            )

        # cannot create a spatial state with incorrect (non-warp) types for indices
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(np.array([0]), wp.array([[0.0, 0.0, 0.0]])),  # numpy array instead of warp for indices
                orientations=(wp.array([1], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            )

        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=([0], wp.array([[0.0, 0.0, 0.0]])),  # list instead of warp array for indices
                orientations=(wp.array([1], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
                linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
                angular_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            )

        # cannot build a spatial state with no entries:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=[],
            )

        # cannot have length mismatch between indices and array (more values than indices):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1"],
                positions=(
                    wp.array([0, 1], dtype=wp.int32),
                    wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
                ),  # 3 values, 2 indices
            )

        # cannot have length mismatch between indices and array (more indices than values):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0", "frame_1", "frame_2"],
                positions=(
                    wp.array([0, 1, 2], dtype=wp.int32),
                    wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
                ),  # 2 values, 3 indices
            )

        # cannot have empty array (shape[0] == 0):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([[]]).reshape([0, 3])),  # Empty array
            )

        # cannot have empty indices (length 0):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(wp.array([], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),  # Empty indices
            )

        # cannot have 1D array instead of 2D for values:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([0.0, 0.0, 0.0])),  # 1D array
            )

        # cannot have 2D array instead of 1D for indices:
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(wp.array([[0]], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),  # 2D indices array
            )

        # cannot have wrong dtype for values (int32):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(
                    wp.array([0], dtype=wp.int32),
                    wp.array([[0, 0, 0]], dtype=wp.int32),
                ),  # int32 instead of float
            )

        # cannot have wrong dtype for values (int64):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(
                    wp.array([0], dtype=wp.int32),
                    wp.array([[0, 0, 0]], dtype=wp.int64),
                ),  # int64 instead of float
            )

        # cannot have wrong dtype for indices (float32):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(
                    wp.array([0.0], dtype=wp.float32),
                    wp.array([[0.0, 0.0, 0.0]]),
                ),  # float32 instead of int for indices
            )

        # cannot have positions with wrong shape[1] (should be 3):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                positions=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0]])),  # shape[1] == 2, should be 3
            )

        # cannot have orientations with wrong shape[1] (should be 4):
        with self.assertRaises(ValueError):
            spatial_state = SpatialState.from_index(
                spatial_space=["frame_0"],
                orientations=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),  # shape[1] == 3, should be 4
            )

        # can use other float dtype explicitly for values, underlying datatype always
        # remains wp.float32
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1"],
            positions=(
                wp.array([0, 1], dtype=wp.int32),
                wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=wp.float64),
            ),
        )
        self.assertTrue(spatial_state.positions.dtype == wp.float32)
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1"],
            positions=(wp.array([0, 1], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=float)),
        )
        self.assertTrue(spatial_state.positions.dtype == wp.float32)
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1"],
            positions=(
                wp.array([0, 1], dtype=wp.int32),
                wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=wp.float32),
            ),
        )
        self.assertTrue(spatial_state.positions.dtype == wp.float32)

        # orientations are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            orientations=(
                wp.array([2, 0, 1], dtype=wp.int32),
                wp.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(
                spatial_state.orientations.numpy(), [[0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0]]
            )
        )
        self.assertTrue(spatial_state.orientation_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(np.allclose(spatial_state.orientation_indices.numpy(), [0, 1, 2]))

        # linear_velocities are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            linear_velocities=(
                wp.array([1, 2, 0], dtype=wp.int32),
                wp.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(spatial_state.linear_velocities.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        self.assertTrue(spatial_state.linear_velocity_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(np.allclose(spatial_state.linear_velocity_indices.numpy(), [0, 1, 2]))

        # angular_velocities are always returned in the order of the underlying spatial-space:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            angular_velocities=(
                wp.array([1, 2, 0], dtype=wp.int32),
                wp.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            ),
        )
        self.assertTrue(
            np.allclose(spatial_state.angular_velocities.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        self.assertTrue(spatial_state.angular_velocity_names == ["frame_0", "frame_1", "frame_2"])
        self.assertTrue(np.allclose(spatial_state.angular_velocity_indices.numpy(), [0, 1, 2]))

        # valid array is correctly populated for positions:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(wp.array([0, 2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])),
        )
        valid_positions = spatial_state.valid_array.numpy()[:, 0]  # Column 0 is positions
        self.assertTrue(np.allclose(valid_positions, [True, False, True]))

        # valid array is correctly populated for orientations:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            orientations=(wp.array([1], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
        )
        valid_orientations = spatial_state.valid_array.numpy()[:, 1]  # Column 1 is orientations
        self.assertTrue(np.allclose(valid_orientations, [False, True, False]))

        # valid array is correctly populated for linear_velocities:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            linear_velocities=(wp.array([2], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )
        valid_velocities = spatial_state.valid_array.numpy()[:, 2]  # Column 2 is linear_velocities
        self.assertTrue(np.allclose(valid_velocities, [False, False, True]))

        # valid array is correctly populated for angular_velocities:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            angular_velocities=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )
        valid_angular_velocities = spatial_state.valid_array.numpy()[:, 3]  # Column 3 is angular_velocities
        self.assertTrue(np.allclose(valid_angular_velocities, [True, False, False]))

        # can have same frame index in different types (positions and orientations):
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0", "frame_1"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            orientations=(wp.array([0], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.orientations.numpy(), [[1.0, 0.0, 0.0, 0.0]]))
        self.assertTrue(spatial_state.position_names == ["frame_0"])
        self.assertTrue(spatial_state.orientation_names == ["frame_0"])

        # can have same frame index in all four types:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            orientations=(wp.array([0], dtype=wp.int32), wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(wp.array([0], dtype=wp.int32), wp.array([[0.0, 0.0, 0.0]])),
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.orientations.numpy(), [[1.0, 0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.linear_velocities.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.angular_velocities.numpy(), [[0.0, 0.0, 0.0]]))

        # single-element edge case works correctly:
        spatial_state = SpatialState.from_index(
            spatial_space=["frame_0"],
            positions=(wp.array([0], dtype=wp.int32), wp.array([[42.0, 0.0, 0.0]])),
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[42.0, 0.0, 0.0]]))
        self.assertTrue(spatial_state.position_names == ["frame_0"])

        # large spatial space works correctly:
        large_spatial_space = [f"frame_{i}" for i in range(20)]
        spatial_state = SpatialState.from_index(
            spatial_space=large_spatial_space,
            positions=(
                wp.array([0, 5, 10, 15, 19], dtype=wp.int32),
                wp.array([[float(i), 0.0, 0.0] for i in [0, 5, 10, 15, 19]]),
            ),
            orientations=(
                wp.array([1, 6, 11, 16], dtype=wp.int32),
                wp.array([[1.0, 0.0, 0.0, 0.0] for _ in [1, 6, 11, 16]]),
            ),
            linear_velocities=(
                wp.array([2, 7, 12, 17], dtype=wp.int32),
                wp.array([[float(i), 0.0, 0.0] for i in [2, 7, 12, 17]]),
            ),
            angular_velocities=(
                wp.array([3, 8, 13, 18], dtype=wp.int32),
                wp.array([[float(i), 0.0, 0.0] for i in [3, 8, 13, 18]]),
            ),
        )
        self.assertEqual(len(spatial_state.position_names), 5)
        self.assertEqual(len(spatial_state.orientation_names), 4)
        self.assertEqual(len(spatial_state.linear_velocity_names), 4)
        self.assertEqual(len(spatial_state.angular_velocity_names), 4)
        self.assertTrue(
            np.allclose(
                spatial_state.positions.numpy(),
                [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [15.0, 0.0, 0.0], [19.0, 0.0, 0.0]],
            )
        )

    async def test_spatial_state_constructor(self) -> None:
        """Tests creating SpatialState instances using the direct constructor with various array configurations and validates error handling."""
        # can create a spatial state with valid arrays:
        spatial_space = ["frame_0", "frame_1", "frame_2"]
        position_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[3, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 4], dtype=wp.bool, device="cpu")

        # Set some valid positions
        position_data.numpy()[0, :] = [0.0, 0.0, 0.0]
        position_data.numpy()[1, :] = [1.0, 0.0, 0.0]
        valid_array.numpy()[0, 0] = True
        valid_array.numpy()[1, 0] = True

        spatial_state = SpatialState(
            spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
        )
        self.assertEqual(spatial_state.spatial_space, spatial_space)
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
        self.assertEqual(spatial_state.position_names, ["frame_0", "frame_1"])

        # can create a spatial state with all four types:
        spatial_space = ["frame_0", "frame_1"]
        position_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[2, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[2, 4], dtype=wp.bool, device="cpu")

        # Set positions, orientations, velocities, and angular_velocities
        position_data.numpy()[0, :] = [0.0, 0.0, 0.0]  # position for frame_0
        orientation_data.numpy()[1, :] = [1.0, 0.0, 0.0, 0.0]  # orientation for frame_1
        linear_velocity_data.numpy()[1, :] = [1.0, 0.0, 0.0]  # linear_velocity for frame_1
        angular_velocity_data.numpy()[0, :] = [2.0, 0.0, 0.0]  # angular_velocity for frame_0

        valid_array.numpy()[0, 0] = True  # position
        valid_array.numpy()[1, 1] = True  # orientation
        valid_array.numpy()[1, 2] = True  # linear_velocity
        valid_array.numpy()[0, 3] = True  # angular_velocity

        spatial_state = SpatialState(
            spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.orientations.numpy(), [[1.0, 0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.linear_velocities.numpy(), [[1.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.angular_velocities.numpy(), [[2.0, 0.0, 0.0]]))
        self.assertEqual(spatial_state.position_names, ["frame_0"])
        self.assertEqual(spatial_state.orientation_names, ["frame_1"])
        self.assertEqual(spatial_state.linear_velocity_names, ["frame_1"])
        self.assertEqual(spatial_state.angular_velocity_names, ["frame_0"])

        # can create a spatial state with single frame:
        spatial_space = ["frame_0"]
        position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
        valid_array.numpy()[0, 0] = True
        position_data.numpy()[0, :] = [42.0, 0.0, 0.0]

        spatial_state = SpatialState(
            spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[42.0, 0.0, 0.0]]))
        self.assertEqual(spatial_state.position_names, ["frame_0"])

        # can create a spatial state with large spatial space:
        large_spatial_space = [f"frame_{i}" for i in range(20)]
        position_data = wp.zeros(shape=[20, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[20, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[20, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[20, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[20, 4], dtype=wp.bool, device="cpu")

        # Set some positions
        for i in [0, 5, 10, 15, 19]:
            position_data.numpy()[i, :] = [float(i), 0.0, 0.0]
            valid_array.numpy()[i, 0] = True

        spatial_state = SpatialState(
            large_spatial_space,
            position_data,
            linear_velocity_data,
            orientation_data,
            angular_velocity_data,
            valid_array,
        )
        self.assertEqual(len(spatial_state.position_names), 5)
        self.assertTrue(
            np.allclose(
                spatial_state.positions.numpy(),
                [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [15.0, 0.0, 0.0], [19.0, 0.0, 0.0]],
            )
        )

        # cannot create with position_data that is not a wp.array:
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = np.zeros(shape=[1, 3], dtype=np.float32)
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with linear_velocity_data that is not a wp.array:
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = np.zeros(shape=[1, 3], dtype=np.float32)
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with orientation_data that is not a wp.array:
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = np.zeros(shape=[1, 4], dtype=np.float32)
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with angular_velocity_data that is not a wp.array:
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = np.zeros(shape=[1, 3], dtype=np.float32)
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with valid_array that is not a wp.array:
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = np.zeros(shape=[1, 4], dtype=bool)
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with position_data wrong dtype (float64):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float64, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with position_data wrong dtype (int32):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.int32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with valid_array wrong dtype (int32):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.int32, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with position_data wrong ndim (1D):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with position_data wrong ndim (3D):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3, 1], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with valid_array wrong ndim (1D):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with valid_array wrong ndim (3D):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[1, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4, 1], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with position_data wrong shape (wrong first dimension):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0", "frame_1"]
            position_data = wp.zeros(shape=[1, 3], dtype=wp.float32, device="cpu")  # Should be [2, 3]
            linear_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[2, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[2, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with position_data wrong shape (wrong second dimension):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0", "frame_1"]
            position_data = wp.zeros(shape=[2, 4], dtype=wp.float32, device="cpu")  # Should be [2, 3]
            linear_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[2, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[2, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with orientation_data wrong shape (wrong second dimension):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0", "frame_1"]
            position_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")  # Should be [2, 4]
            angular_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[2, 4], dtype=wp.bool, device="cpu")
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with valid_array wrong shape (wrong first dimension):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0", "frame_1"]
            position_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[2, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[1, 4], dtype=wp.bool, device="cpu")  # Should be [2, 4]
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # cannot create with valid_array wrong shape (wrong second dimension):
        with self.assertRaises(ValueError):
            spatial_space = ["frame_0", "frame_1"]
            position_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            linear_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            orientation_data = wp.zeros(shape=[2, 4], dtype=wp.float32, device="cpu")
            angular_velocity_data = wp.zeros(shape=[2, 3], dtype=wp.float32, device="cpu")
            valid_array = wp.zeros(shape=[2, 3], dtype=wp.bool, device="cpu")  # Should be [2, 4]
            spatial_state = SpatialState(
                spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
            )

        # can create with empty spatial_space (but no valid data):
        spatial_space = []
        position_data = wp.zeros(shape=[0, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[0, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[0, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[0, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[0, 4], dtype=wp.bool, device="cpu")
        spatial_state = SpatialState(
            spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
        )
        self.assertEqual(len(spatial_state.spatial_space), 0)
        self.assertIsNone(spatial_state.positions)
        self.assertIsNone(spatial_state.orientations)
        self.assertIsNone(spatial_state.linear_velocities)
        self.assertIsNone(spatial_state.angular_velocities)

        # positions are returned in the order of the underlying spatial-space:
        spatial_space = ["frame_0", "frame_1", "frame_2"]
        position_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[3, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 4], dtype=wp.bool, device="cpu")

        # Set positions in non-sequential order
        position_data.numpy()[0, :] = [0.0, 0.0, 0.0]
        position_data.numpy()[2, :] = [2.0, 0.0, 0.0]
        position_data.numpy()[1, :] = [1.0, 0.0, 0.0]
        valid_array.numpy()[:, 0] = True

        spatial_state = SpatialState(
            spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
        )
        self.assertTrue(
            np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        self.assertEqual(spatial_state.position_names, ["frame_0", "frame_1", "frame_2"])

        # valid array is correctly used to determine which frames have valid data:
        spatial_space = ["frame_0", "frame_1", "frame_2"]
        position_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        linear_velocity_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        orientation_data = wp.zeros(shape=[3, 4], dtype=wp.float32, device="cpu")
        angular_velocity_data = wp.zeros(shape=[3, 3], dtype=wp.float32, device="cpu")
        valid_array = wp.zeros(shape=[3, 4], dtype=wp.bool, device="cpu")

        # Set data for all, but only mark some as valid
        position_data.numpy()[:, :] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
        orientation_data.numpy()[:, :] = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
        linear_velocity_data.numpy()[:, :] = [[3.0, 0.0, 0.0], [4.0, 0.0, 0.0], [5.0, 0.0, 0.0]]
        angular_velocity_data.numpy()[:, :] = [[6.0, 0.0, 0.0], [7.0, 0.0, 0.0], [8.0, 0.0, 0.0]]

        valid_array.numpy()[0, 0] = True  # Only frame_0 has valid position
        valid_array.numpy()[1, 1] = True  # Only frame_1 has valid orientation
        valid_array.numpy()[2, 2] = True  # Only frame_2 has valid velocity
        valid_array.numpy()[0, 3] = True  # Only frame_0 has valid angular_velocity

        spatial_state = SpatialState(
            spatial_space, position_data, linear_velocity_data, orientation_data, angular_velocity_data, valid_array
        )
        self.assertTrue(np.allclose(spatial_state.positions.numpy(), [[0.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.orientations.numpy(), [[0.0, 1.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.linear_velocities.numpy(), [[5.0, 0.0, 0.0]]))
        self.assertTrue(np.allclose(spatial_state.angular_velocities.numpy(), [[6.0, 0.0, 0.0]]))
        self.assertEqual(spatial_state.position_names, ["frame_0"])
        self.assertEqual(spatial_state.orientation_names, ["frame_1"])
        self.assertEqual(spatial_state.linear_velocity_names, ["frame_2"])
        self.assertEqual(spatial_state.angular_velocity_names, ["frame_0"])

    async def test_spatial_state_is_read_only(self) -> None:
        """Tests that SpatialState fields are read-only and cannot be modified after creation."""
        spatial_state = SpatialState.from_name(
            spatial_space=["frame_0", "frame_1", "frame_2"],
            positions=(["frame_0", "frame_1"], wp.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])),
            orientations=(["frame_1"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(["frame_2"], wp.array([[0.0, 0.0, 0.0]])),
        )

        # Cannot write to any fields:
        with self.assertRaises(AttributeError):
            spatial_state.positions = wp.zeros_like(spatial_state.positions)
        with self.assertRaises(AttributeError):
            spatial_state.orientations = wp.zeros_like(spatial_state.orientations)
        with self.assertRaises(AttributeError):
            spatial_state.linear_velocities = wp.zeros_like(spatial_state.linear_velocities)
        with self.assertRaises(AttributeError):
            spatial_state.angular_velocities = wp.zeros_like(spatial_state.angular_velocities)
        with self.assertRaises(AttributeError):
            spatial_state.position_data = wp.zeros_like(spatial_state.position_data)
        with self.assertRaises(AttributeError):
            spatial_state.orientation_data = wp.zeros_like(spatial_state.orientation_data)
        with self.assertRaises(AttributeError):
            spatial_state.linear_velocity_data = wp.zeros_like(spatial_state.linear_velocity_data)
        with self.assertRaises(AttributeError):
            spatial_state.angular_velocity_data = wp.zeros_like(spatial_state.angular_velocity_data)
        with self.assertRaises(AttributeError):
            spatial_state.valid_array = wp.zeros_like(spatial_state.valid_array)
        with self.assertRaises(AttributeError):
            spatial_state.position_names = ["new", "position", "names"]
        with self.assertRaises(AttributeError):
            spatial_state.position_indices = wp.zeros_like(spatial_state.position_indices)
        with self.assertRaises(AttributeError):
            spatial_state.orientation_names = ["new", "orientation", "names"]
        with self.assertRaises(AttributeError):
            spatial_state.orientation_indices = wp.zeros_like(spatial_state.orientation_indices)
        with self.assertRaises(AttributeError):
            spatial_state.linear_velocity_names = ["new", "linear_velocity", "names"]
        with self.assertRaises(AttributeError):
            spatial_state.linear_velocity_indices = wp.zeros_like(spatial_state.linear_velocity_indices)
        with self.assertRaises(AttributeError):
            spatial_state.angular_velocity_names = ["new", "angular", "velocity", "names"]
        with self.assertRaises(AttributeError):
            spatial_state.angular_velocity_indices = wp.zeros_like(spatial_state.angular_velocity_indices)
        with self.assertRaises(AttributeError):
            spatial_state.spatial_space = ["new", "spatial", "space"]


class TestRootState(omni.kit.test.AsyncTestCase):
    """Test class for RootState functionality.

    This class contains unit tests for the RootState class, which represents the pose and velocity state
    of a robot's root/base frame. The tests verify proper construction, validation of input parameters,
    read-only property enforcement, and correct handling of various data types and edge cases.

    The test methods validate that RootState correctly handles position, orientation, linear velocity,
    and angular velocity data as Warp arrays, ensures proper dimensionality and data type validation,
    and maintains immutability of state objects after creation.
    """

    async def test_root_state(self) -> None:
        """Test RootState creation and functionality.

        Tests creating RootState objects with different combinations of parameters,
        validating data types and shapes, and verifying error handling for invalid inputs.
        """
        # can create a RootState:
        root_state = RootState(
            position=wp.array([0.1, 0.0, 0.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=wp.array([0.2, 0.0, 0.0]),
            angular_velocity=wp.array([0.3, 0.0, 0.0]),
        )
        print(f"root_state.position: {root_state.position}")
        print(f"root_state.orientation: {root_state.orientation}")
        print(f"root_state.linear_velocity: {root_state.linear_velocity}")
        print(f"root_state.angular_velocity: {root_state.angular_velocity}")
        self.assertTrue(np.allclose(root_state.position.numpy(), [0.1, 0.0, 0.0]))
        self.assertTrue(np.allclose(root_state.orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(root_state.linear_velocity.numpy(), [0.2, 0.0, 0.0]))
        self.assertTrue(np.allclose(root_state.angular_velocity.numpy(), [0.3, 0.0, 0.0]))

        # cannot create a root state with incorrect (non-warp) types:
        self.assertRaises(
            ValueError,
            RootState,
            position=[0.0, 0.0, 0.0],
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=wp.array([0.0, 0.0, 0.0]),
            angular_velocity=wp.array([0.0, 0.0, 0.0]),
        )
        self.assertRaises(
            ValueError,
            RootState,
            position=wp.array([0.0, 0.0, 0.0]),
            orientation=[1.0, 0.0, 0.0, 0.0],
            linear_velocity=wp.array([0.0, 0.0, 0.0]),
            angular_velocity=wp.array([0.0, 0.0, 0.0]),
        )
        self.assertRaises(
            ValueError,
            RootState,
            position=wp.array([0.0, 0.0, 0.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=[0.0, 0.0, 0.0],
            angular_velocity=wp.array([0.0, 0.0, 0.0]),
        )
        self.assertRaises(
            ValueError,
            RootState,
            position=wp.array([0.0, 0.0, 0.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=wp.array([0.0, 0.0, 0.0]),
            angular_velocity=[0.0, 0.0, 0.0],
        )
        self.assertRaises(
            ValueError,
            RootState,
            position=np.array([0.0, 0.0, 0.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=wp.array([0.0, 0.0, 0.0]),
            angular_velocity=wp.array([0.0, 0.0, 0.0]),
        )
        self.assertRaises(
            ValueError,
            RootState,
            position=np.array([0.0, 0.0, 0.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=wp.array([0.0, 0.0, 0.0]),
            angular_velocity=wp.array([0.0, 0.0, 0.0]),
        )

        # we can build partial root-states:
        only_position = RootState(
            position=wp.array([0.0, 0.0, 0.0]),
        )
        only_orientation = RootState(
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
        )
        only_linear_velocity = RootState(
            linear_velocity=wp.array([0.0, 0.0, 0.0]),
        )
        only_angular_velocity = RootState(
            angular_velocity=wp.array([0.0, 0.0, 0.0]),
        )

        # cannot build a root state with all None entries:
        self.assertRaises(
            ValueError,
            RootState,
            position=None,
            orientation=None,
            linear_velocity=None,
            angular_velocity=None,
        )

        # cannot build a root state with the wrong types:
        self.assertRaises(
            ValueError,
            RootState,
            position=wp.array([[]]),
            orientation=wp.array([[]]),
            linear_velocity=wp.array([[]]),
            angular_velocity=wp.array([[]]),
        )

        # cannot create with position wrong length (2 elements instead of 3):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0]),  # 2 elements, should be 3
                orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            )

        # cannot create with position wrong length (4 elements instead of 3):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0, 0.0]),  # 4 elements, should be 3
                orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            )

        # cannot create with orientation wrong length (3 elements instead of 4):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                orientation=wp.array([1.0, 0.0, 0.0]),  # 3 elements, should be 4
            )

        # cannot create with orientation wrong length (5 elements instead of 4):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                orientation=wp.array([1.0, 0.0, 0.0, 0.0, 0.0]),  # 5 elements, should be 4
            )

        # cannot create with linear_velocity wrong length (2 elements instead of 3):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                linear_velocity=wp.array([0.0, 0.0]),  # 2 elements, should be 3
            )

        # cannot create with linear_velocity wrong length (4 elements instead of 3):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                linear_velocity=wp.array([0.0, 0.0, 0.0, 0.0]),  # 4 elements, should be 3
            )

        # cannot create with angular_velocity wrong length (2 elements instead of 3):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                angular_velocity=wp.array([0.0, 0.0]),  # 2 elements, should be 3
            )

        # cannot create with angular_velocity wrong length (4 elements instead of 3):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                angular_velocity=wp.array([0.0, 0.0, 0.0, 0.0]),  # 4 elements, should be 3
            )

        # cannot create with position wrong ndim (2D array):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([[0.0, 0.0, 0.0]]),  # 2D array
                orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            )

        # cannot create with orientation wrong ndim (2D array):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                orientation=wp.array([[1.0, 0.0, 0.0, 0.0]]),  # 2D array
            )

        # cannot create with linear_velocity wrong ndim (2D array):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                linear_velocity=wp.array([[0.0, 0.0, 0.0]]),  # 2D array
            )

        # cannot create with angular_velocity wrong ndim (2D array):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                angular_velocity=wp.array([[0.0, 0.0, 0.0]]),  # 2D array
            )

        # cannot create with position wrong dtype (int32):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0, 0, 0], dtype=wp.int32),  # int32 instead of float
                orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            )

        # cannot create with position wrong dtype (int64):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0, 0, 0], dtype=wp.int64),  # int64 instead of float
                orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            )

        # cannot create with orientation wrong dtype (int32):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                orientation=wp.array([1, 0, 0, 0], dtype=wp.int32),  # int32 instead of float
            )

        # cannot create with linear_velocity wrong dtype (int32):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                linear_velocity=wp.array([0, 0, 0], dtype=wp.int32),  # int32 instead of float
            )

        # cannot create with angular_velocity wrong dtype (int32):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([0.0, 0.0, 0.0]),
                angular_velocity=wp.array([0, 0, 0], dtype=wp.int32),  # int32 instead of float
            )

        # cannot create with empty array (length 0):
        with self.assertRaises(ValueError):
            RootState(
                position=wp.array([]),  # Empty array
            )

        # can create with Python float dtype:
        root_state = RootState(
            position=wp.array([0.0, 0.0, 0.0], dtype=float),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )
        self.assertTrue(np.allclose(root_state.position.numpy(), [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(root_state.orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))

        # can create with various combinations of partial states:
        # position + orientation
        root_state = RootState(
            position=wp.array([1.0, 2.0, 3.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
        )
        self.assertTrue(np.allclose(root_state.position.numpy(), [1.0, 2.0, 3.0]))
        self.assertTrue(np.allclose(root_state.orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))
        self.assertIsNone(root_state.linear_velocity)
        self.assertIsNone(root_state.angular_velocity)

        # position + linear_velocity
        root_state = RootState(
            position=wp.array([1.0, 2.0, 3.0]),
            linear_velocity=wp.array([4.0, 5.0, 6.0]),
        )
        self.assertTrue(np.allclose(root_state.position.numpy(), [1.0, 2.0, 3.0]))
        self.assertTrue(np.allclose(root_state.linear_velocity.numpy(), [4.0, 5.0, 6.0]))
        self.assertIsNone(root_state.orientation)
        self.assertIsNone(root_state.angular_velocity)

        # orientation + angular_velocity
        root_state = RootState(
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            angular_velocity=wp.array([7.0, 8.0, 9.0]),
        )
        self.assertTrue(np.allclose(root_state.orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(root_state.angular_velocity.numpy(), [7.0, 8.0, 9.0]))
        self.assertIsNone(root_state.position)
        self.assertIsNone(root_state.linear_velocity)

        # position + orientation + linear_velocity (three fields)
        root_state = RootState(
            position=wp.array([1.0, 2.0, 3.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=wp.array([4.0, 5.0, 6.0]),
        )
        self.assertTrue(np.allclose(root_state.position.numpy(), [1.0, 2.0, 3.0]))
        self.assertTrue(np.allclose(root_state.orientation.numpy(), [1.0, 0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(root_state.linear_velocity.numpy(), [4.0, 5.0, 6.0]))
        self.assertIsNone(root_state.angular_velocity)

    async def test_root_state_is_read_only(self) -> None:
        """Test that RootState fields are read-only.

        Verifies that all fields of a RootState object cannot be modified after creation,
        ensuring immutability of the state data.
        """
        root_state = RootState(
            position=wp.array([0.1, 0.0, 0.0], dtype=wp.float32),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0], dtype=wp.float32),
            linear_velocity=wp.array([0.2, 0.0, 0.0], dtype=wp.float32),
            angular_velocity=wp.array([0.3, 0.0, 0.0], dtype=wp.float32),
        )

        # Cannot write to any fields:
        with self.assertRaises(AttributeError):
            root_state.position = wp.array([0.0, 0.0, 0.0])
        with self.assertRaises(AttributeError):
            root_state.orientation = wp.array([1.0, 0.0, 0.0, 0.0])
        with self.assertRaises(AttributeError):
            root_state.linear_velocity = wp.array([0.0, 0.0, 0.0])
        with self.assertRaises(AttributeError):
            root_state.angular_velocity = wp.array([0.0, 0.0, 0.0])


class TestRobotState(omni.kit.test.AsyncTestCase):
    """Test class for validating RobotState functionality and the combine_robot_states function.

    This test class verifies the construction of RobotState objects from various component states
    (JointState, RootState, SpatialState) and validates the combination logic for merging
    multiple RobotState instances. It ensures proper error handling for invalid combinations
    and validates that state data is correctly preserved during operations.
    """

    async def test_construct_robot_state(self) -> None:
        """Test creation of RobotState instances with various component combinations.

        Verifies that RobotState can be created with different combinations of joint states,
        root states, link states, and site states, including partial states and empty states.
        """
        # create valid component states for testing
        joint_state = JointState.from_name(
            robot_joint_space=["joint_0", "joint_1"],
            positions=(["joint_0", "joint_1"], wp.array([0.0, 1.0])),
            velocities=(["joint_0", "joint_1"], wp.array([0.0, 0.0])),
            efforts=(["joint_0", "joint_1"], wp.array([0.0, 0.0])),
        )
        root_state = RootState(
            position=wp.array([0.0, 0.0, 0.0]),
            orientation=wp.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=wp.array([0.0, 0.0, 0.0]),
            angular_velocity=wp.array([0.0, 0.0, 0.0]),
        )
        link_state = SpatialState.from_name(
            spatial_space=["body_0"],
            positions=(["body_0"], wp.array([[0.0, 0.0, 0.0]])),
            orientations=(["body_0"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(["body_0"], wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(["body_0"], wp.array([[0.0, 0.0, 0.0]])),
        )
        site_state = SpatialState.from_name(
            spatial_space=["tool_0"],
            positions=(["tool_0"], wp.array([[1.0, 0.0, 0.0]])),
            orientations=(["tool_0"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            linear_velocities=(["tool_0"], wp.array([[0.0, 0.0, 0.0]])),
            angular_velocities=(["tool_0"], wp.array([[0.0, 0.0, 0.0]])),
        )

        # can create a RobotState with all components:
        robot_state = RobotState(
            joints=joint_state,
            root=root_state,
            links=link_state,
            sites=site_state,
        )
        self.assertEqual(robot_state.joints, joint_state)
        self.assertEqual(robot_state.root, root_state)
        self.assertEqual(robot_state.links, link_state)
        self.assertEqual(robot_state.sites, site_state)

        # can create a RobotState with joints only:
        robot_state = RobotState(joints=joint_state)
        self.assertEqual(robot_state.joints, joint_state)
        self.assertIsNone(robot_state.root)
        self.assertIsNone(robot_state.links)
        self.assertIsNone(robot_state.sites)

        # can create a RobotState root only:
        robot_state = RobotState(root=root_state)
        self.assertIsNone(robot_state.joints)
        self.assertEqual(robot_state.root, root_state)
        self.assertIsNone(robot_state.links)
        self.assertIsNone(robot_state.sites)

        # can create a RobotState bodies only:
        robot_state = RobotState(links=link_state)
        self.assertIsNone(robot_state.joints)
        self.assertIsNone(robot_state.root)
        self.assertEqual(robot_state.links, link_state)
        self.assertIsNone(robot_state.sites)

        # can create a RobotState tool_frames only:
        robot_state = RobotState(sites=site_state)
        self.assertIsNone(robot_state.joints)
        self.assertIsNone(robot_state.root)
        self.assertIsNone(robot_state.links)
        self.assertEqual(robot_state.sites, site_state)

        # can create an empty RobotState:
        robot_state = RobotState()
        self.assertIsNone(robot_state.joints)
        self.assertIsNone(robot_state.root)
        self.assertIsNone(robot_state.links)
        self.assertIsNone(robot_state.sites)

    async def test_combine_robot_state(self) -> None:
        """Test combining robot states with various component types and configurations.

        Verifies successful combination of non-overlapping states within the same spaces,
        failure cases for overlapping states, and proper handling of different state types
        including joint states, root states, link states, and site states.
        """
        #########################################################
        ######### Create different control spaces ###############
        #########################################################
        joint_space_1 = ["joint_0", "joint_1", "joint_2"]
        joint_space_2 = ["joint_0", "joint_1", "joint_2", "joint_3"]

        link_space_1 = ["link_0", "link_1", "link_2"]
        link_space_2 = ["link_0", "link_1", "link_2", "link_3"]

        site_space_1 = ["site_0", "site_1", "site_2"]
        site_space_2 = ["site_0", "site_1", "site_2", "site_3"]

        #########################################################
        ### Create different sub-states to test combining with ##
        #########################################################

        joint_space_1_state_1 = RobotState(
            joints=JointState.from_name(
                joint_space_1,
                positions=(["joint_0"], wp.array([0.0])),
                velocities=(["joint_0"], wp.array([1.0])),
                efforts=(["joint_0"], wp.array([2.0])),
            )
        )

        joint_space_1_state_2 = RobotState(
            joints=JointState.from_name(
                joint_space_1,
                positions=(["joint_1"], wp.array([3.0])),
                velocities=(["joint_1"], wp.array([4.0])),
                efforts=(["joint_1"], wp.array([5.0])),
            )
        )

        joint_space_1_position_only = RobotState(
            joints=JointState.from_name(
                joint_space_1,
                positions=(["joint_0"], wp.array([0.0])),
            )
        )

        joint_space_1_efforts_only = RobotState(
            joints=JointState.from_name(
                joint_space_1,
                efforts=(["joint_2"], wp.array([6.0])),
            )
        )

        joint_space_2_state_1 = RobotState(
            joints=JointState.from_name(
                joint_space_2,
                positions=(["joint_0"], wp.array([6.0])),
                velocities=(["joint_0"], wp.array([7.0])),
                efforts=(["joint_0"], wp.array([8.0])),
            )
        )

        joint_space_2_state_2 = RobotState(
            joints=JointState.from_name(
                joint_space_2,
                positions=(["joint_1"], wp.array([9.0])),
                velocities=(["joint_1"], wp.array([10.0])),
                efforts=(["joint_1"], wp.array([11.0])),
            )
        )

        link_space_1_state_1 = RobotState(
            links=SpatialState.from_name(
                spatial_space=link_space_1,
                positions=(["link_0"], wp.array([[12.0, 13.0, 14.0]])),
                orientations=(["link_0"], wp.array([[15.0, 16.0, 17.0, 18.0]])),
                linear_velocities=(["link_0"], wp.array([[19.0, 20.0, 21.0]])),
                angular_velocities=(["link_0"], wp.array([[22.0, 23.0, 24.0]])),
            )
        )

        link_space_1_state_2 = RobotState(
            links=SpatialState.from_name(
                link_space_1,
                positions=(["link_1"], wp.array([[25.0, 26.0, 27.0]])),
                orientations=(["link_1"], wp.array([[28.0, 29.0, 30.0, 31.0]])),
                linear_velocities=(["link_1"], wp.array([[32.0, 33.0, 34.0]])),
                angular_velocities=(["link_1"], wp.array([[35.0, 36.0, 37.0]])),
            )
        )

        link_space_2_state_1 = RobotState(
            links=SpatialState.from_name(
                link_space_2,
                positions=(["link_0"], wp.array([[38.0, 39.0, 40.0]])),
                orientations=(["link_0"], wp.array([[41.0, 42.0, 43.0, 44.0]])),
                linear_velocities=(["link_0"], wp.array([[45.0, 46.0, 47.0]])),
                angular_velocities=(["link_0"], wp.array([[48.0, 49.0, 50.0]])),
            )
        )

        link_space_2_state_2 = RobotState(
            links=SpatialState.from_name(
                link_space_2,
                positions=(["link_1"], wp.array([[51.0, 52.0, 53.0]])),
                orientations=(["link_1"], wp.array([[54.0, 55.0, 56.0, 57.0]])),
                linear_velocities=(["link_1"], wp.array([[58.0, 59.0, 60.0]])),
                angular_velocities=(["link_1"], wp.array([[61.0, 62.0, 63.0]])),
            )
        )

        site_space_1_state_1 = RobotState(
            sites=SpatialState.from_name(
                site_space_1,
                positions=(["site_0"], wp.array([[64.0, 65.0, 66.0]])),
                orientations=(["site_0"], wp.array([[67.0, 68.0, 69.0, 70.0]])),
                linear_velocities=(["site_0"], wp.array([[71.0, 72.0, 73.0]])),
                angular_velocities=(["site_0"], wp.array([[74.0, 75.0, 76.0]])),
            )
        )

        site_space_1_state_2 = RobotState(
            sites=SpatialState.from_name(
                site_space_1,
                positions=(["site_1"], wp.array([[77.0, 78.0, 79.0]])),
                orientations=(["site_1"], wp.array([[80.0, 81.0, 82.0, 83.0]])),
                linear_velocities=(["site_1"], wp.array([[84.0, 85.0, 86.0]])),
                angular_velocities=(["site_1"], wp.array([[87.0, 88.0, 89.0]])),
            )
        )

        site_space_2_state_1 = RobotState(
            sites=SpatialState.from_name(
                site_space_2,
                positions=(["site_0"], wp.array([[88.0, 89.0, 90.0]])),
                orientations=(["site_0"], wp.array([[91.0, 92.0, 93.0, 94.0]])),
                linear_velocities=(["site_0"], wp.array([[95.0, 96.0, 97.0]])),
                angular_velocities=(["site_0"], wp.array([[98.0, 99.0, 100.0]])),
            )
        )

        site_space_2_state_2 = RobotState(
            sites=SpatialState.from_name(
                site_space_2,
                positions=(["site_1"], wp.array([[101.0, 102.0, 103.0]])),
                orientations=(["site_1"], wp.array([[104.0, 105.0, 106.0, 107.0]])),
                linear_velocities=(["site_1"], wp.array([[108.0, 109.0, 110.0]])),
                angular_velocities=(["site_1"], wp.array([[111.0, 112.0, 113.0]])),
            )
        )

        root_state_1 = RobotState(
            root=RootState(
                position=wp.array([120.0, 121.0, 122.0]),
                orientation=wp.array([123.0, 124.0, 125.0, 126.0]),
                linear_velocity=wp.array([127.0, 128.0, 129.0]),
                angular_velocity=wp.array([130.0, 131.0, 132.0]),
            )
        )

        root_state_2 = RobotState(
            root=RootState(
                position=wp.array([133.0, 134.0, 135.0]),
                orientation=wp.array([136.0, 137.0, 138.0, 139.0]),
                linear_velocity=wp.array([140.0, 141.0, 142.0]),
                angular_velocity=wp.array([143.0, 144.0, 145.0]),
            )
        )

        #############################################
        ### Test successful combinations of states ##
        #############################################
        # can combine two non-overlapping states defined on the same joint-space:
        combined_state = combine_robot_states(robot_state_1=joint_space_1_state_1, robot_state_2=joint_space_1_state_2)
        self.assertEqual(combined_state.joints.robot_joint_space, joint_space_1)
        self.assertTrue(np.allclose(combined_state.joints.positions.numpy(), [0.0, 3.0]))
        self.assertTrue(np.allclose(combined_state.joints.velocities.numpy(), [1.0, 4.0]))
        self.assertTrue(np.allclose(combined_state.joints.efforts.numpy(), [2.0, 5.0]))
        self.assertIsNone(combined_state.root)
        self.assertIsNone(combined_state.links)
        self.assertIsNone(combined_state.sites)

        # We can combine joint states which combine different fields on different joints:
        combined_state = combine_robot_states(
            robot_state_1=joint_space_1_position_only, robot_state_2=joint_space_1_efforts_only
        )
        self.assertEqual(combined_state.joints.robot_joint_space, joint_space_1)
        self.assertTrue(np.allclose(combined_state.joints.positions.numpy(), [0.0]))
        self.assertTrue(combined_state.joints.position_names == ["joint_0"])
        self.assertTrue(np.allclose(combined_state.joints.position_indices.numpy(), [0]))
        self.assertIsNone(combined_state.joints.velocities)  # NO VELOCITIES
        self.assertTrue(combined_state.joints.effort_names == ["joint_2"])
        self.assertTrue(np.allclose(combined_state.joints.effort_indices.numpy(), [2]))
        self.assertTrue(np.allclose(combined_state.joints.efforts.numpy(), [6.0]))
        self.assertIsNone(combined_state.root)
        self.assertIsNone(combined_state.links)
        self.assertIsNone(combined_state.sites)

        # can combine joint-states with non joint-states:
        combined_state = combine_robot_states(robot_state_1=joint_space_1_state_1, robot_state_2=root_state_1)
        self.assertEqual(combined_state.joints.robot_joint_space, joint_space_1)
        self.assertTrue(np.allclose(combined_state.joints.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(combined_state.joints.velocities.numpy(), [1.0]))
        self.assertTrue(np.allclose(combined_state.joints.efforts.numpy(), [2.0]))
        self.assertTrue(np.allclose(combined_state.root.position.numpy(), [120.0, 121.0, 122.0]))
        self.assertTrue(np.allclose(combined_state.root.orientation.numpy(), [123.0, 124.0, 125.0, 126.0]))
        self.assertTrue(np.allclose(combined_state.root.linear_velocity.numpy(), [127.0, 128.0, 129.0]))
        self.assertTrue(np.allclose(combined_state.root.angular_velocity.numpy(), [130.0, 131.0, 132.0]))
        self.assertIsNone(combined_state.links)
        self.assertIsNone(combined_state.sites)

        combined_state = combine_robot_states(robot_state_1=joint_space_1_state_1, robot_state_2=link_space_1_state_1)
        self.assertEqual(combined_state.joints.robot_joint_space, joint_space_1)
        self.assertTrue(np.allclose(combined_state.joints.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(combined_state.joints.velocities.numpy(), [1.0]))
        self.assertTrue(np.allclose(combined_state.joints.efforts.numpy(), [2.0]))
        self.assertEqual(combined_state.links.spatial_space, link_space_1)
        self.assertTrue(np.allclose(combined_state.links.positions.numpy(), [12.0, 13.0, 14.0]))
        self.assertTrue(np.allclose(combined_state.links.orientations.numpy(), [15.0, 16.0, 17.0, 18.0]))
        self.assertTrue(np.allclose(combined_state.links.linear_velocities.numpy(), [19.0, 20.0, 21.0]))
        self.assertTrue(np.allclose(combined_state.links.angular_velocities.numpy(), [22.0, 23.0, 24.0]))
        self.assertIsNone(combined_state.root)
        self.assertIsNone(combined_state.sites)

        combined_state = combine_robot_states(robot_state_1=link_space_1_state_1, robot_state_2=joint_space_1_state_1)
        self.assertEqual(combined_state.links.spatial_space, link_space_1)
        self.assertTrue(np.allclose(combined_state.links.positions.numpy(), [12.0, 13.0, 14.0]))
        self.assertTrue(np.allclose(combined_state.links.orientations.numpy(), [15.0, 16.0, 17.0, 18.0]))
        self.assertTrue(np.allclose(combined_state.links.linear_velocities.numpy(), [19.0, 20.0, 21.0]))
        self.assertTrue(np.allclose(combined_state.links.angular_velocities.numpy(), [22.0, 23.0, 24.0]))
        self.assertEqual(combined_state.joints.robot_joint_space, joint_space_1)
        self.assertTrue(np.allclose(combined_state.joints.positions.numpy(), [0.0]))
        self.assertTrue(np.allclose(combined_state.joints.velocities.numpy(), [1.0]))
        self.assertTrue(np.allclose(combined_state.joints.efforts.numpy(), [2.0]))

        # can combine two root-states, so long as they do not write to the same field:
        combined_state = combine_robot_states(
            robot_state_1=RobotState(root=RootState(position=wp.array([120.0, 121.0, 122.0]))),
            robot_state_2=RobotState(root=RootState(linear_velocity=wp.array([133.0, 134.0, 135.0]))),
        )
        self.assertTrue(np.allclose(combined_state.root.position.numpy(), [120.0, 121.0, 122.0]))
        self.assertTrue(np.allclose(combined_state.root.linear_velocity.numpy(), [133.0, 134.0, 135.0]))
        self.assertIsNone(combined_state.root.orientation)
        self.assertIsNone(combined_state.root.angular_velocity)
        self.assertIsNone(combined_state.joints)
        self.assertIsNone(combined_state.links)
        self.assertIsNone(combined_state.sites)

        #########################################
        ### Test failed combinations of states ##
        #########################################
        # cannot combine two joint-space states if they attempt to write to the same spots:
        self.assertIsNone(
            combine_robot_states(robot_state_1=joint_space_1_state_1, robot_state_2=joint_space_1_state_1)
        )

        # cannot combine two non-overlapping joint states if they are not intended for the same joint-space:
        self.assertIsNone(
            combine_robot_states(robot_state_1=joint_space_1_state_1, robot_state_2=joint_space_2_state_2)
        )

        # cannot combine two link states if they attempt to write to the same spots:
        self.assertIsNone(combine_robot_states(robot_state_1=link_space_1_state_1, robot_state_2=link_space_1_state_1))

        # cannot combine two non-overlapping link states if they are not intended for the same link-space:
        self.assertIsNone(combine_robot_states(robot_state_1=link_space_1_state_1, robot_state_2=link_space_2_state_2))

        # cannot combine two states which write to the same field of the root state:
        self.assertIsNone(combine_robot_states(robot_state_1=root_state_1, robot_state_2=root_state_2))
        self.assertIsNone(
            combine_robot_states(
                robot_state_1=RobotState(root=RootState(position=wp.array([120.0, 121.0, 122.0]))),
                robot_state_2=RobotState(root=RootState(position=wp.array([133.0, 134.0, 135.0]))),
            )
        )

        # cannot combine two states which write to the same field of the site state:
        self.assertIsNone(combine_robot_states(robot_state_1=site_space_1_state_1, robot_state_2=site_space_1_state_1))

        # cannot combine two site states which are intended for different site-spaces:
        self.assertIsNone(combine_robot_states(robot_state_1=site_space_1_state_1, robot_state_2=site_space_2_state_2))

        #########################################
        ### Test None input edge cases ##
        #########################################
        # combine_robot_states returns None if either input is None:
        self.assertIsNone(combine_robot_states(None, None))
        self.assertIsNone(combine_robot_states(None, joint_space_1_state_1))
        self.assertIsNone(combine_robot_states(joint_space_1_state_1, None))

        #########################################
        ### Test partial overlap edge cases ##
        #########################################
        # cannot combine joint states with partial overlap (same joint, different fields):
        # state_1 has positions for joint_0, state_2 has positions AND velocities for joint_0
        partial_overlap_state_1 = RobotState(
            joints=JointState.from_name(
                joint_space_1,
                positions=(["joint_0"], wp.array([0.0])),
            )
        )
        partial_overlap_state_2 = RobotState(
            joints=JointState.from_name(
                joint_space_1,
                positions=(["joint_0"], wp.array([1.0])),  # Overlaps with state_1
                velocities=(["joint_0"], wp.array([2.0])),
            )
        )
        self.assertIsNone(combine_robot_states(partial_overlap_state_1, partial_overlap_state_2))

        # cannot combine spatial states with partial overlap (same frame, different fields):
        partial_overlap_link_1 = RobotState(
            links=SpatialState.from_name(
                link_space_1,
                positions=(["link_0"], wp.array([[0.0, 0.0, 0.0]])),
            )
        )
        partial_overlap_link_2 = RobotState(
            links=SpatialState.from_name(
                link_space_1,
                positions=(["link_0"], wp.array([[1.0, 0.0, 0.0]])),  # Overlaps with link_1
                orientations=(["link_0"], wp.array([[1.0, 0.0, 0.0, 0.0]])),
            )
        )
        self.assertIsNone(combine_robot_states(partial_overlap_link_1, partial_overlap_link_2))

    async def test_different_joint_state_order(self) -> None:
        """Test that robot states with different joint space ordering cannot be combined.

        Verifies that combine_robot_states returns None when attempting to combine robot states
        where the joint spaces have the same joints but in different orders.
        """
        # cannot combine two robot states if the joint-spaces are not listed in the
        # same order:
        robot_state_1 = RobotState(
            JointState.from_name(robot_joint_space=["joint_0", "joint_1"], positions=(["joint_0"], wp.array([0.0])))
        )

        robot_state_2 = RobotState(
            JointState.from_name(robot_joint_space=["joint_1", "joint_0"], positions=(["joint_0"], wp.array([0.0])))
        )
        self.assertIsNone(combine_robot_states(robot_state_1, robot_state_2))
