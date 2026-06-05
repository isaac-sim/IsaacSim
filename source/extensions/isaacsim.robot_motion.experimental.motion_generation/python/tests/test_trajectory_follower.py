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

"""Unit tests for the TrajectoryFollower functionality in the motion generation module."""

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of the module will make it auto-discoverable by omni.kit.test
class TestTrajectoryFollower(omni.kit.test.AsyncTestCase):
    """Test class for validating the TrajectoryFollower functionality in the motion generation module.

    This class contains comprehensive tests to verify that trajectory following works correctly,
    including trajectory creation, follower initialization, state tracking, timing validation,
    and error handling scenarios. The tests cover various edge cases such as times outside
    the trajectory bounds, proper reset behavior, and trajectory lifecycle management.

    The test validates that the TrajectoryFollower can:
    - Accept and store trajectories created from waypoint paths
    - Track trajectory state and report whether a trajectory is set
    - Follow trajectories with proper position and velocity interpolation
    - Handle timing constraints including start times and end times
    - Clean up trajectories when encountering invalid time requests
    - Require proper reset before trajectory execution
    """

    # Before running each test
    async def setUp(self):
        """Set up test fixtures before each test method is run."""
        pass

    # After running each test
    async def tearDown(self):
        """Clean up test fixtures after each test method is run."""
        pass

    async def test_trajectory_follower(self):
        """Test trajectory follower functionality including trajectory setting, state tracking, and time-based execution."""
        # create a trajectory:
        path = mg.Path(waypoints=np.array([[0.0], [1.0], [2.0]]))
        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=np.array([0.5]),
            max_accelerations=np.array([0.1]),
            robot_joint_space=["joint_0"],
            active_joints=["joint_0"],
        )

        # create a trajectory follower:
        follower = mg.TrajectoryFollower()
        follower.set_trajectory(trajectory)

        # create a dummy robot state:
        robot_state = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=["joint_0"],
                positions=(["joint_0"], wp.array([0.0])),
                velocities=(["joint_0"], wp.array([0.0])),
                efforts=(["joint_0"], wp.array([0.0])),
            )
        )

        # confirm that the trajectory follower has a trajectory:
        self.assertTrue(follower.has_trajectory())

        # Before calling forward, we must call reset.
        follower.reset(robot_state, None, 0.0)

        # by calling forward at t = 0.0, we should get the initial position:
        desired_state = follower.forward(robot_state, None, 0.0)
        self.assertIsNotNone(desired_state)
        self.assertEqual(desired_state.joints.position_names, ["joint_0"])
        self.assertTrue(np.allclose(desired_state.joints.positions.numpy(), np.array([0.0])))
        self.assertTrue(np.allclose(desired_state.joints.velocities.numpy(), np.array([0.0])))
        self.assertIsNone(desired_state.joints.efforts)

        # if we call forward at the end time of the trajectory, we should get the final position:
        desired_state = follower.forward(robot_state, None, trajectory.duration)
        self.assertIsNotNone(desired_state)
        self.assertEqual(desired_state.joints.position_names, ["joint_0"])
        self.assertTrue(np.allclose(desired_state.joints.positions.numpy(), np.array([2.0])))
        self.assertTrue(np.allclose(desired_state.joints.velocities.numpy(), np.array([0.0])))
        self.assertIsNone(desired_state.joints.efforts)

        # if I set a trajectory, and then I reset,
        # we will still have a trajectory.
        follower.set_trajectory(trajectory)
        reset_value = follower.reset(robot_state, None, 0.0)
        self.assertTrue(follower.has_trajectory())
        self.assertTrue(reset_value)

        # if I set a trajectory, and then give a time before the start time of the trajectory, we should get an error,
        # and the trajectory will be deleted. The desired state should be None.
        follower.set_trajectory(trajectory)
        follower.reset(robot_state, None, 0.0)
        desired_state = follower.forward(robot_state, None, -1.0)
        self.assertIsNone(desired_state)
        self.assertFalse(follower.has_trajectory())

        # if I give a time after the end-time of the trajectory,
        # we should get None, and the trajectory will be deleted.
        # the returned desired state is None.
        follower.set_trajectory(trajectory)
        follower.reset(robot_state, None, 0.0)
        desired_state = follower.forward(robot_state, None, trajectory.duration + 1.0)
        self.assertIsNone(desired_state)
        self.assertFalse(follower.has_trajectory())

        # if the trajectory is set with a start_time of 1.0,
        # then we should be at the end of the trajectory at
        # end_time + 1.0. There should still be a trajectory set.
        follower.set_trajectory(trajectory)
        follower.reset(robot_state, None, 1.0)
        desired_state = follower.forward(robot_state, None, trajectory.duration + 1.0)
        self.assertIsNotNone(desired_state)
        self.assertEqual(desired_state.joints.position_names, ["joint_0"])
        self.assertTrue(np.allclose(desired_state.joints.positions.numpy(), np.array([2.0])))
        self.assertTrue(np.allclose(desired_state.joints.velocities.numpy(), np.array([0.0])))
        self.assertIsNone(desired_state.joints.efforts)
        self.assertTrue(follower.has_trajectory())

        # We cannot set a trajectory and then run it, if we didn't yet
        # call `reset`:
        follower.set_trajectory(trajectory)
        desired_state = follower.forward(robot_state, None, 0.0)
        self.assertIsNone(desired_state)
