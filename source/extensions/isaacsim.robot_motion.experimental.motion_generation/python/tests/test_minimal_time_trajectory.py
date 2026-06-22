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

"""Verify minimal-time joint trajectory generation.

The tests cover single- and two-joint waypoint paths, synchronized segment
timing, stationary joints, list and Warp inputs, CPU execution, positive and
negative travel directions, drifting and non-drifting velocity profiles, and
degenerate input validation.
"""

from copy import deepcopy

# Import extension python module we are testing with absolute import path, as if we are an external user (i.e. a different extension)
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import warp as wp


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of the module will make it auto-discoverable by omni.kit.test
class TestMinimalTimeTrajectory(omni.kit.test.AsyncTestCase):
    """Test suite for minimal time trajectory generation functionality.

    This test class validates the behavior of minimal time trajectory generation for robot motion,
    including trajectory creation with various input formats, motion planning with velocity and
    acceleration constraints, and handling of edge cases. The tests cover single and multi-joint
    scenarios, drift behavior analysis, and input validation.

    The test cases verify:
    - Trajectory creation with NumPy arrays, Warp arrays, and Python lists
    - One-dimensional trajectories with and without velocity drifting
    - Multi-joint coordination ensuring synchronized waypoint arrival
    - Stationary joint handling during motion
    - CPU and CUDA device compatibility
    - Input validation and error handling for degenerate cases

    Each test method validates trajectory properties including position accuracy, velocity constraints,
    acceleration limits, and timing synchronization across multiple joints.
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures before each test method."""

    # After running each test
    async def tearDown(self) -> None:
        """Clean up after each test method."""

    async def test_can_create_trajectory(self) -> None:
        """Test that minimal time trajectories can be created with different input data types.

        Verifies trajectory creation using numpy arrays, Warp arrays, and Python lists for waypoints,
        velocities, and accelerations. Also tests that mismatched joint specifications raise ValueError.
        """
        max_velocity = 0.5
        max_acceleration = 0.1

        # we can create a trajectory using numpy arrays:
        path = mg.Path(waypoints=np.array([[0.0], [1.0], [2.0]]))

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=np.array([max_velocity]),
            max_accelerations=np.array([max_acceleration]),
            robot_joint_space=["joint_0"],
            active_joints=["joint_0"],
        )

        # we can create a trajectory using warp arrays:
        path = mg.Path(waypoints=wp.array([[0.0], [1.0], [2.0]]))

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=wp.array([max_velocity]),
            max_accelerations=wp.array([max_acceleration]),
            robot_joint_space=["joint_0"],
            active_joints=["joint_0"],
        )

        # we can create a trajectory using lists:
        path = mg.Path(waypoints=[[0.0], [1.0], [2.0]])

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=[max_velocity],
            max_accelerations=[max_acceleration],
            robot_joint_space=["joint_0"],
            active_joints=["joint_0"],
        )

        # I cannot create a trajectory using a different number of joints than the number of waypoints:
        self.assertRaises(
            ValueError,
            path.to_minimal_time_joint_trajectory,
            max_velocities=[max_velocity],
            max_accelerations=[max_acceleration],
            robot_joint_space=["joint_0", "joint_1"],
            active_joints=["joint_0", "joint_1"],
        )

    async def test_one_dimensional_trajectory_no_drifting(self) -> None:
        """Test one-dimensional trajectory generation without velocity drifting phase.

        Verifies that when maximum velocity exceeds the threshold for drift-free motion
        (v_max > sqrt(a_max*s_motion)), the trajectory consists only of acceleration and
        deceleration phases. Tests trajectory duration, velocity limits, and position/velocity
        values at key time points.
        """
        max_velocity = 0.5
        max_acceleration = 0.1

        path = mg.Path(waypoints=np.array([[0.0], [1.0], [2.0]]))

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=np.array([max_velocity]),
            max_accelerations=np.array([max_acceleration]),
            robot_joint_space=["joint_0"],
            active_joints=["joint_0"],
        )

        # There should be no drifting, because v_max > sqrt(a_max*s_motion)
        t_switching_no_drifting = np.sqrt(1 / max_acceleration)

        # Therefore, since there are two motion segments, they should each
        # have an acceleration and deceleration phase. This should mean
        # that the total trajectory time is 4*t_switching_no_drifting.
        total_trajectory_time = trajectory.duration
        self.assertTrue(abs(total_trajectory_time - 4 * t_switching_no_drifting) < 1e-8)
        # print(f"t_switching_no_drifting was found to be: {t_switching_no_drifting}")

        # now, cycle through the trajectory. We should verify that we never meet the maximum velocity.
        test_times = np.linspace(start=0.0, stop=trajectory.duration, num=500)
        last_position = 0.0
        for i, test_time in enumerate(test_times):
            desired_state = trajectory.get_target_state(test_time)
            position = desired_state.joints.positions.numpy()[0]
            velocity = desired_state.joints.velocities.numpy()[0]

            # print(f"t: {test_time}, p: {position}, v: {velocity}")

            self.assertTrue(abs(velocity) < max_velocity)
            self.assertTrue(position >= -1e-8)
            self.assertTrue(position <= 2.0 + 1e-8)
            self.assertTrue(velocity >= -1e-8)
            self.assertTrue(position >= last_position)
            last_position = deepcopy(position)

        # verify that we get the correct positions and velocities at the start-time, end-time,
        # and switching time:
        # print(f"trajectory start time: {0.0}")
        desired_state_start = trajectory.get_target_state(0.0)
        start_position = desired_state_start.joints.positions.numpy()[0]
        start_velocity = desired_state_start.joints.velocities.numpy()[0]
        print(f"start_position is: {start_position}")
        print(f"start velocity is: {start_velocity}")
        self.assertTrue(abs(start_position) < 1e-8)
        self.assertTrue(abs(start_velocity) < 1e-8)

        desired_state_end = trajectory.get_target_state(trajectory.duration)
        end_position = desired_state_end.joints.positions.numpy()[0]
        end_velocity = desired_state_end.joints.velocities.numpy()[0]
        print(f"end_position is: {end_position}")
        print(f"end_velocity is: {end_velocity}")
        self.assertTrue(abs(end_position - 2.0) < 1e-8)
        self.assertTrue(abs(end_velocity) < 1e-8)

        desired_state_middle = trajectory.get_target_state(trajectory.duration / 2)
        middle_position = desired_state_middle.joints.positions.numpy()[0]
        middle_velocity = desired_state_middle.joints.velocities.numpy()[0]

        print(f"middle_position is: {middle_position}")
        print(f"middle_velocity is: {middle_velocity}")
        self.assertTrue(abs(middle_position - 1.0) < 1e-8)
        self.assertTrue(abs(middle_velocity) < 1e-8)

    async def test_one_dimensional_trajectory_no_drifting_negative_direction(self) -> None:
        """Test one-dimensional trajectory generation in negative direction without drifting.

        Verifies trajectory behavior when moving from higher to lower position values
        without velocity drifting phase. Tests that velocity constraints and position
        bounds are maintained throughout the motion.
        """
        max_velocity = 0.5
        max_acceleration = 0.1

        path = mg.Path(waypoints=np.array([[2.0], [1.0], [0.0]]))

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=np.array([max_velocity]),
            max_accelerations=np.array([max_acceleration]),
            robot_joint_space=["joint_0"],
            active_joints=["joint_0"],
        )

        # There should be no drifting, because v_max > sqrt(a_max*s_motion)
        t_switching_no_drifting = np.sqrt(1 / max_acceleration)

        # Therefore, since there are two motion segments, they should each
        # have an acceleration and deceleration phase. This should mean
        # that the total trajectory time is 4*t_switching_no_drifting.
        total_trajectory_time = trajectory.duration
        self.assertTrue(abs(total_trajectory_time - 4 * t_switching_no_drifting) < 1e-8)
        # print(f"t_switching_no_drifting was found to be: {t_switching_no_drifting}")

        # now, cycle through the trajectory. We should verify that we never meet the maximum velocity.
        test_times = np.linspace(start=0.0, stop=trajectory.duration, num=500)
        last_position = 2.0
        for i, test_time in enumerate(test_times):
            desired_state = trajectory.get_target_state(test_time)
            position = desired_state.joints.positions.numpy()[0]
            velocity = desired_state.joints.velocities.numpy()[0]

            # print(f"t: {test_time}, p: {position}, v: {velocity}")

            self.assertTrue(abs(velocity) < max_velocity)
            self.assertTrue(position >= -1e-8)
            self.assertTrue(position <= 2.0 + 1e-8)
            self.assertTrue(velocity <= 1e-8)
            self.assertTrue(position <= last_position)
            last_position = deepcopy(position)

        # verify that we get the correct positions and velocities at the start-time, end-time,
        # and switching time:
        # print(f"trajectory start time: {0.0}")
        desired_state_start = trajectory.get_target_state(0.0)
        start_position = desired_state_start.joints.positions.numpy()[0]
        start_velocity = desired_state_start.joints.velocities.numpy()[0]
        print(f"start_position is: {start_position}")
        print(f"start velocity is: {start_velocity}")
        self.assertTrue(abs(start_position - 2.0) < 1e-8)
        self.assertTrue(abs(start_velocity) < 1e-8)

        desired_state_end = trajectory.get_target_state(trajectory.duration)
        end_position = desired_state_end.joints.positions.numpy()[0]
        end_velocity = desired_state_end.joints.velocities.numpy()[0]
        print(f"end_position is: {end_position}")
        print(f"end_velocity is: {end_velocity}")
        self.assertTrue(abs(end_position) < 1e-8)
        self.assertTrue(abs(end_velocity) < 1e-8)

        desired_state_middle = trajectory.get_target_state(trajectory.duration / 2)
        middle_position = desired_state_middle.joints.positions.numpy()[0]
        middle_velocity = desired_state_middle.joints.velocities.numpy()[0]

        print(f"middle_position is: {middle_position}")
        print(f"middle_velocity is: {middle_velocity}")
        self.assertTrue(abs(middle_position - 1.0) < 1e-8)
        self.assertTrue(abs(middle_velocity) < 1e-8)

    # Actual test, notice it is an "async" function, so "await" can be used if needed
    async def test_one_dimensional_trajectory_with_drifting(self) -> None:
        """Test one-dimensional trajectory generation with velocity drifting phase.

        Verifies that when maximum velocity is below the drift threshold
        (v_max < sqrt(a_max*s_motion)), the trajectory includes acceleration,
        constant velocity (drift), and deceleration phases. Tests drift time
        calculations and velocity behavior at switching points.
        """
        max_velocity = 0.2
        max_acceleration = 0.1

        path = mg.Path(waypoints=np.array([[0.0], [1.0], [2.0]]))

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=np.array([max_velocity]),
            max_accelerations=np.array([max_acceleration]),
            robot_joint_space=["joint_0"],
            active_joints=["joint_0"],
        )

        # There should be drifting, because v_max < sqrt(a_max*s_motion)
        t_switch = max_velocity / max_acceleration

        # find the total amount of drift time we expect:
        s_acceleration_phase = 0.5 * max_acceleration * t_switch**2

        # this is true because our total s-motion (for both motion segments)
        # is 1.0 units.
        s_drift = 1.0 - 2 * s_acceleration_phase

        t_drift = s_drift / max_velocity

        # total amount of time for motion is built of the two acceleration
        # phases, and one drift phase.
        t_motion_segment = t_drift + 2 * t_switch

        # Therefore, since there are two motion segments, they should each
        # have an acceleration and deceleration phase. This should mean
        # that the total trajectory time is 4*t_switching_no_drifting.
        total_trajectory_time = trajectory.duration
        self.assertTrue(abs(total_trajectory_time - 2 * t_motion_segment) < 1e-8)
        print(f"t_switch was found to be: {t_switch}")
        print(f"t_switch + t_drift was found to be: {t_switch + t_drift}")

        # now, cycle through the trajectory. We should verify that we never meet the maximum velocity.
        test_times = np.linspace(start=0.0, stop=trajectory.duration, num=500)
        last_position = 0.0
        for i, test_time in enumerate(test_times):
            desired_state = trajectory.get_target_state(test_time)
            position = desired_state.joints.positions.numpy()[0]
            velocity = desired_state.joints.velocities.numpy()[0]

            # print(f"t: {test_time}, p: {position}, v: {velocity}")

            self.assertTrue(position >= -1e-8)
            self.assertTrue(position <= 2.0 + 1e-8)
            self.assertTrue(position >= last_position)
            last_position = deepcopy(position)
            self.assertTrue(velocity >= -1e-8)
            self.assertTrue(velocity <= max_velocity + 1e-8)

        # verify that we get the correct positions and velocities at the start-time, end-time,
        # and switching time:
        desired_state_start = trajectory.get_target_state(0.0)
        start_position = desired_state_start.joints.positions.numpy()[0]
        start_velocity = desired_state_start.joints.velocities.numpy()[0]
        self.assertTrue(abs(start_position) < 1e-8)
        self.assertTrue(abs(start_velocity) < 1e-8)

        desired_state_end = trajectory.get_target_state(trajectory.duration)
        end_position = desired_state_end.joints.positions.numpy()[0]
        end_velocity = desired_state_end.joints.velocities.numpy()[0]
        self.assertTrue(abs(end_position - 2.0) < 1e-8)
        self.assertTrue(abs(end_velocity) < 1e-8)

        desired_state_middle = trajectory.get_target_state(trajectory.duration / 2)
        middle_position = desired_state_middle.joints.positions.numpy()[0]
        middle_velocity = desired_state_middle.joints.velocities.numpy()[0]
        self.assertTrue(abs(middle_position - 1.0) < 1e-8)
        self.assertTrue(abs(middle_velocity) < 1e-8)

        desired_state_switch_time = trajectory.get_target_state(t_switch)
        switch_time_velocity = desired_state_switch_time.joints.velocities.numpy()[0]
        self.assertTrue(abs(switch_time_velocity - max_velocity) < 1e-8)

    async def test_two_joints(self) -> None:
        """Test two-joint trajectory generation with synchronized motion.

        Verifies that both joints reach waypoints simultaneously, with trajectory timing
        limited by the slowest joint. Tests that joints maintain synchronized motion
        along straight-line paths in joint space throughout the trajectory.
        """
        # this should make sure that when two joints are defined, they both
        # get to their middle waypoints at EXACTLY the same time, which will
        # be the minimal time for the slowest joint.
        waypoints = np.array([[0.0, 0.1], [1.5, 0.2], [1.1, 2.5]])
        path = mg.Path(waypoints=waypoints)

        max_velocities = np.array([1.0, 1.0])
        max_accelerations = np.array([0.5, 0.5])

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=max_velocities,
            max_accelerations=max_accelerations,
            robot_joint_space=["joint_0", "joint_1"],
            active_joints=["joint_0", "joint_1"],
        )

        # for the FIRST motion, everything should be limited by the motion
        # of joint 1. This is because the two joints have the same maximum
        # velocity and acceleration, but joint 1 has to travel much further.

        # For joint 1, on motion 1,
        # there should be no drifting, because v_max > sqrt(a_max*s_motion)
        t_first_motion = 2 * np.sqrt(1.5 / max_accelerations[0])
        print(f"computed t_first_motion: {t_first_motion}")

        # during the SECOND motion, everything should be limited by the motion
        # of joint 2, since it has to travel much further.

        # for joint 2, on motion 2, there has to be a drifing time,
        # as v_max < sqrt(a_max * s_motion)
        t_second_motion_switch = max_velocities[1] / max_accelerations[1]
        s_drift = (
            2.5
            - 0.2  # the full s-motion
            - 2
            * (0.5 * max_accelerations[1] * t_second_motion_switch**2)  # distance we move during acceleration phases.
        )
        t_drift = s_drift / max_velocities[1]
        t_second_motion = 2 * t_second_motion_switch + t_drift
        print(f"computed t_second_motion: {t_second_motion}")

        print(f"computed trajectory time: {t_first_motion + t_second_motion}")
        print(f"actual trajectory time: {trajectory.duration}")
        # full trajectory should just take the length of the two motions put together:
        self.assertTrue(abs(trajectory.duration - t_first_motion - t_second_motion) < 1e-8)

        # Check here, the trajectory should stay on a straight line!
        # That is, q == q0*(1-t) + q1*t for t in [0,1].
        # To test this, we will make sure that every joint is at the same
        # parameter "t" throughout the entire trajectory:
        test_times = np.linspace(0.0, trajectory.duration, 100)
        for test_time in test_times:
            desired_state = trajectory.get_target_state(test_time)
            position: np.array = desired_state.joints.positions.numpy()
            velocity: np.array = desired_state.joints.velocities.numpy()
            t_values = np.zeros_like(position)
            for i, q in enumerate(position):
                if test_time <= t_first_motion:
                    t_values[i] = (q - waypoints[0, i]) / (waypoints[1, i] - waypoints[0, i])
                else:
                    t_values[i] = (q - waypoints[1, i]) / (waypoints[2, i] - waypoints[1, i])

            print(f"t: {test_time}, p: {position}, v: {velocity}")
            print(f"t_values (percent complete motion) are: {t_values}")

            # check here, all t-values should be the same!
            # i.e., every joint should always be the same percentage
            # completed their motion.
            for i_element in range(len(t_values) - 1):
                self.assertTrue(abs(t_values[i_element] - t_values[i_element + 1]) < 1e-6)

    async def test_two_joints_with_one_stationary_joint(self) -> None:
        """Test trajectory generation when one joint remains stationary.

        Verifies that a joint with identical waypoint values maintains zero position
        change and velocity throughout the entire trajectory duration.
        """
        waypoints = np.array([[0.0, 0.1], [1.5, 0.1], [1.1, 0.1]])
        path = mg.Path(waypoints=waypoints)

        max_velocities = np.array([1.0, 1.0])
        max_accelerations = np.array([0.5, 0.5])

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=max_velocities,
            max_accelerations=max_accelerations,
            robot_joint_space=["joint_0", "joint_1"],
            active_joints=["joint_0", "joint_1"],
        )

        # Check here, the second joint should be stationary for the entire trajectory.
        test_times = np.linspace(0.0, trajectory.duration, 100)
        for test_time in test_times:
            desired_state = trajectory.get_target_state(test_time)
            joint_positionts = desired_state.joints.positions.numpy()
            joint_velocities = desired_state.joints.velocities.numpy()
            self.assertTrue(np.isclose(joint_positionts[1], 0.1))
            self.assertTrue(np.isclose(joint_velocities[1], 0.0))

    async def test_two_joints_with_list_waypoints(self) -> None:
        """Test two-joint trajectory generation using Python list waypoints.

        Replicates the two-joint synchronization test using Python lists instead of
        numpy arrays for waypoints, velocities, and accelerations to verify
        compatibility with different input data types.
        """
        # This test is the same as the test_two_joints test, but with list waypoints.

        # this should make sure that when two joints are defined, they both
        # get to their middle waypoints at EXACTLY the same time, which will
        # be the minimal time for the slowest joint.
        waypoints = [[0.0, 0.1], [1.5, 0.2], [1.1, 2.5]]
        path = mg.Path(waypoints=waypoints)

        max_velocities = [1.0, 1.0]
        max_accelerations = [0.5, 0.5]

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=max_velocities,
            max_accelerations=max_accelerations,
            robot_joint_space=["joint_0", "joint_1"],
            active_joints=["joint_0", "joint_1"],
        )

        # for the FIRST motion, everything should be limited by the motion
        # of joint 1. This is because the two joints have the same maximum
        # velocity and acceleration, but joint 1 has to travel much further.

        # For joint 1, on motion 1,
        # there should be no drifting, because v_max > sqrt(a_max*s_motion)
        t_first_motion = 2 * np.sqrt(1.5 / max_accelerations[0])
        print(f"computed t_first_motion: {t_first_motion}")

        # during the SECOND motion, everything should be limited by the motion
        # of joint 2, since it has to travel much further.

        # for joint 2, on motion 2, there has to be a drifing time,
        # as v_max < sqrt(a_max * s_motion)
        t_second_motion_switch = max_velocities[1] / max_accelerations[1]
        s_drift = (
            2.5
            - 0.2  # the full s-motion
            - 2
            * (0.5 * max_accelerations[1] * t_second_motion_switch**2)  # distance we move during acceleration phases.
        )
        t_drift = s_drift / max_velocities[1]
        t_second_motion = 2 * t_second_motion_switch + t_drift
        print(f"computed t_second_motion: {t_second_motion}")

        print(f"computed trajectory time: {t_first_motion + t_second_motion}")
        print(f"actual trajectory time: {trajectory.duration}")
        # full trajectory should just take the length of the two motions put together:
        self.assertTrue(abs(trajectory.duration - t_first_motion - t_second_motion) < 1e-8)

        # Check here, the trajectory should stay on a straight line!
        # That is, q == q0*(1-t) + q1*t for t in [0,1].
        # To test this, we will make sure that every joint is at the same
        # parameter "t" throughout the entire trajectory:
        test_times = np.linspace(0.0, trajectory.duration, 100)
        for test_time in test_times:
            desired_state = trajectory.get_target_state(test_time)
            position: np.array = desired_state.joints.positions.numpy()
            velocity: np.array = desired_state.joints.velocities.numpy()
            t_values = np.zeros_like(position)
            for i, q in enumerate(position):
                if test_time <= t_first_motion:
                    t_values[i] = (q - waypoints[0][i]) / (waypoints[1][i] - waypoints[0][i])
                else:
                    t_values[i] = (q - waypoints[1][i]) / (waypoints[2][i] - waypoints[1][i])

            print(f"t: {test_time}, p: {position}, v: {velocity}")
            print(f"t_values (percent complete motion) are: {t_values}")

            # check here, all t-values should be the same!
            # i.e., every joint should always be the same percentage
            # completed their motion.
            for i_element in range(len(t_values) - 1):
                self.assertTrue(abs(t_values[i_element] - t_values[i_element + 1]) < 1e-6)

    async def test_two_joints_on_cpu_device(self) -> None:
        """Test two-joint trajectory generation using CPU device for Warp arrays.

        Verifies trajectory generation and synchronization when using Warp arrays
        specifically allocated on CPU device, ensuring device compatibility
        for motion generation computations.
        """
        # This test is the same as the test_two_joints test, but with list waypoints.

        # this should make sure that when two joints are defined, they both
        # get to their middle waypoints at EXACTLY the same time, which will
        # be the minimal time for the slowest joint.
        waypoints = wp.array([[0.0, 0.1], [1.5, 0.2], [1.1, 2.5]], device="cpu")
        path = mg.Path(waypoints=waypoints)

        # Note, here I am constructing the max velocities on the CUDA device,
        # should still work because all arrays will use the warp device of the waypoints.
        max_velocities = wp.array([1.0, 1.0])
        max_accelerations = [0.5, 0.5]

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=max_velocities,
            max_accelerations=max_accelerations,
            robot_joint_space=["joint_0", "joint_1"],
            active_joints=["joint_0", "joint_1"],
        )

        # for the FIRST motion, everything should be limited by the motion
        # of joint 1. This is because the two joints have the same maximum
        # velocity and acceleration, but joint 1 has to travel much further.

        # For joint 1, on motion 1,
        # there should be no drifting, because v_max > sqrt(a_max*s_motion)
        t_first_motion = 2 * np.sqrt(1.5 / max_accelerations[0])
        print(f"computed t_first_motion: {t_first_motion}")

        # during the SECOND motion, everything should be limited by the motion
        # of joint 2, since it has to travel much further.

        # for joint 2, on motion 2, there has to be a drifing time,
        # as v_max < sqrt(a_max * s_motion)
        t_second_motion_switch = max_velocities.numpy()[1] / max_accelerations[1]
        s_drift = (
            2.5
            - 0.2  # the full s-motion
            - 2
            * (0.5 * max_accelerations[1] * t_second_motion_switch**2)  # distance we move during acceleration phases.
        )
        t_drift = s_drift / max_velocities.numpy()[1]
        t_second_motion = 2 * t_second_motion_switch + t_drift
        print(f"computed t_second_motion: {t_second_motion}")

        print(f"computed trajectory time: {t_first_motion + t_second_motion}")
        print(f"actual trajectory time: {trajectory.duration}")
        # full trajectory should just take the length of the two motions put together:
        self.assertTrue(abs(trajectory.duration - t_first_motion - t_second_motion) < 1e-8)

        # Check here, the trajectory should stay on a straight line!
        # That is, q == q0*(1-t) + q1*t for t in [0,1].
        # To test this, we will make sure that every joint is at the same
        # parameter "t" throughout the entire trajectory:
        test_times = np.linspace(0.0, trajectory.duration, 100)
        for test_time in test_times:
            desired_state = trajectory.get_target_state(test_time)
            position: np.array = desired_state.joints.positions.numpy()
            velocity: np.array = desired_state.joints.velocities.numpy()
            t_values = np.zeros_like(position)
            for i, q in enumerate(position):
                if test_time <= t_first_motion:
                    t_values[i] = (q - waypoints.numpy()[0][i]) / (waypoints.numpy()[1][i] - waypoints.numpy()[0][i])
                else:
                    t_values[i] = (q - waypoints.numpy()[1][i]) / (waypoints.numpy()[2][i] - waypoints.numpy()[1][i])

            print(f"t: {test_time}, p: {position}, v: {velocity}")
            print(f"t_values (percent complete motion) are: {t_values}")

            # check here, all t-values should be the same!
            # i.e., every joint should always be the same percentage
            # completed their motion.
            for i_element in range(len(t_values) - 1):
                self.assertTrue(abs(t_values[i_element] - t_values[i_element + 1]) < 1e-6)

    async def test_degenerate_inputs(self) -> None:
        """Tests degenerate input handling for minimal time joint trajectory generation.

        Validates that the trajectory generation raises appropriate ValueErrors for invalid inputs including:
        duplicate waypoints, non-positive velocity/acceleration limits, incorrect array dimensions,
        mismatched joint counts, and single waypoint paths.
        """
        # waypoints must be unique:
        path = mg.Path(waypoints=np.array([[0.0, 0.1, 0.2], [0.0, 0.1, 0.2], [0.0, 0.0, 0.0]]))
        max_velocities = [1.0, 1.0, 1.0]
        max_accelerations = [1.0, 1.0, 1.0]
        joints = [f"joint{i}" for i in range(3)]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )

        path = mg.Path(waypoints=np.array([[0.0, 0.0, 0.2], [0.0, 0.1, 0.2], [0.0, 0.0, 0.0]]))

        # any maximum velocity or acceleration which is non-positive will raise.
        max_velocities = [0.0, 1.0, 1.0]
        max_accelerations = [1.0, 1.0, 1.0]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )
        max_velocities = [1.0, -1.0, 1.0]
        max_accelerations = [1.0, 1.0, 1.0]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )
        max_velocities = [1.0, 1.0, 1.0]
        max_accelerations = [1.0, 0.0, 1.0]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )
        max_velocities = [1.0, 1.0, 1.0]
        max_accelerations = [1.0, -1e-5, 1.0]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )

        # waypoints which are not 2-dimensional raise an error:
        self.assertRaises(ValueError, mg.Path, np.array([[[0.0, 0.0, 0.2], [0.0, 0.1, 0.2], [0.0, 0.0, 0.0]]]))
        self.assertRaises(ValueError, mg.Path, np.array([0.0, 0.0, 0.0]))

        # velocity or acceleration bounds which are not 1-dimensional raise a ValueError:
        path = mg.Path(waypoints=np.array([[0.0, 0.0, 0.2], [0.0, 0.1, 0.2], [0.0, 0.0, 0.0]]))
        max_velocities = [[1.0, 1.0, 1.0]]
        max_accelerations = [1.0, 1.0, 1.0]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )
        max_velocities = [1.0, 1.0, 1.0]
        max_accelerations = [[1.0, 1.0, 1.0]]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )
        max_velocities = np.array([[1.0, 1.0, 1.0]])
        max_accelerations = [1.0, 1.0, 1.0]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )
        max_velocities = [1.0, 1.0, 1.0]
        max_accelerations = np.array([[1.0, 1.0, 1.0]])
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )

        # trying to add the wrong number of joint names raises a ValueError:
        max_velocities = [1.0, 1.0, 1.0]
        max_accelerations = np.array([1.0, 1.0, 1.0])
        joints = [f"joint{i}" for i in range(4)]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )

        # Trying to create a trajectory with only a single waypoint raises an error:
        path = mg.Path(waypoints=np.array([[0.0, 0.0, 0.0]]))
        max_velocities = [1.0, 1.0, 1.0]
        max_accelerations = np.array([1.0, 1.0, 1.0])
        joints = [f"joint{i}" for i in range(3)]
        with self.assertRaises(ValueError):
            path.to_minimal_time_joint_trajectory(
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                robot_joint_space=joints,
                active_joints=joints,
            )
