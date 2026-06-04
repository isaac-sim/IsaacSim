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

# Import extension python module we are testing with absolute import path, as if we are an external user (i.e. a different extension)

"""Tests for Path functionality in the motion generation extension."""

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of the module will make it auto-discoverable by omni.kit.test
class TestPath(omni.kit.test.AsyncTestCase):
    """Test class for validating Path functionality in the motion generation extension.

    This test class validates the creation, manipulation, and indexing of Path objects from the
    isaacsim.robot_motion.experimental.motion_generation module. It tests various input formats including NumPy arrays,
    Warp arrays, and Python lists, while verifying error handling for invalid inputs.
    """

    # Before running each test
    async def setUp(self):
        """Set up test fixtures before each test method is run."""
        pass

    # After running each test
    async def tearDown(self):
        """Clean up after each test method is run."""
        pass

    async def test_path(self):
        """Test Path creation and manipulation with various input types.

        Tests creating Path objects with NumPy arrays, Warp arrays, and lists.
        Verifies waypoint counting, retrieval, indexing, and error handling for invalid inputs.
        """
        # can create a path using numpy arrays:
        path = mg.Path(waypoints=np.array([[0.0], [1.0], [2.0]]))
        self.assertEqual(path.get_waypoints_count(), 3)
        self.assertTrue(np.allclose(path.get_waypoints().numpy(), np.array([[0.0], [1.0], [2.0]])))

        # can create a path using warp arrays:
        path = mg.Path(waypoints=wp.array([[0.0], [1.0], [2.0]]))
        self.assertEqual(path.get_waypoints_count(), 3)
        self.assertTrue(np.allclose(path.get_waypoints().numpy(), np.array([[0.0], [1.0], [2.0]])))

        # cannot create a path with a non-two-dimensional array:
        self.assertRaises(ValueError, mg.Path, waypoints=np.array([0.0, 1.0, 2.0]))
        self.assertRaises(ValueError, mg.Path, waypoints=wp.array([0.0, 1.0, 2.0]))
        self.assertRaises(ValueError, mg.Path, waypoints=[[[0.0, 1.0, 2.0]]])

        # can create a path using lists:
        path = mg.Path(waypoints=[[0.0], [1.0], [2.0]])
        self.assertEqual(path.get_waypoints_count(), 3)
        self.assertTrue(np.allclose(path.get_waypoints().numpy(), np.array([[0.0], [1.0], [2.0]])))

        # can index a path, and incorrect indices raise an error:
        self.assertTrue(np.allclose(path.get_waypoint_by_index(0), np.array([0.0])))
        self.assertTrue(np.allclose(path.get_waypoint_by_index(1), np.array([1.0])))
        self.assertTrue(np.allclose(path.get_waypoint_by_index(2), np.array([2.0])))
        self.assertRaises(IndexError, path.get_waypoint_by_index, 3)
        self.assertRaises(IndexError, path.get_waypoint_by_index, -1)
