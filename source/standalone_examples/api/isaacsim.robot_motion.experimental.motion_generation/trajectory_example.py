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

"""Complete example demonstrating trajectory following with the Motion Generation API.

This example shows how to:
1. Create a custom LinearTrajectory class that implements the Trajectory interface
2. Use Path.to_minimal_time_joint_trajectory() to create a minimal-time trajectory
3. Use TrajectoryFollower to execute either trajectory type

The example emphasizes that TrajectoryFollower has no opinion about which trajectory
type it follows - it works with any object that implements the Trajectory interface.
"""

# ============================================================================
# 1. Launch Simulation App
# ============================================================================
# All Isaac Sim imports must come after SimulationApp is instantiated
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Now we can import Isaac Sim modules
import argparse
from typing import Optional

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.app
import omni.timeline
import warp as wp
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async


# ============================================================================
# 2. Custom LinearTrajectory Implementation
# ============================================================================
# <start-linear-trajectory-snippet>
class LinearTrajectory(mg.Trajectory):
    """A simple trajectory that performs linear interpolation between waypoints.

    This demonstrates how to implement the Trajectory interface. The TrajectoryFollower
    has no opinion about which trajectory type it follows - it works with any object
    that implements the Trajectory interface.
    """

    # <start-linear-trajectory-init-snippet>
    def __init__(
        self,
        waypoints: np.ndarray,
        robot_joint_space: list[str],
        active_joints: list[str],
        time_per_segment: float = 1.0,
    ) -> None:
        """Initialize the linear trajectory.

        Args:
            waypoints: Array of shape (N, M) where N is number of waypoints and M is number of joints
            robot_joint_space: The full joint space of the robot
            active_joints: The joints controlled by this trajectory (must match waypoint columns)
            time_per_segment: Time to spend moving between each pair of waypoints (seconds)
        """
        if waypoints.ndim != 2:
            raise ValueError("Waypoints must be a 2D array")
        if len(waypoints) < 2:
            raise ValueError("Must have at least two waypoints")
        if len(active_joints) != waypoints.shape[1]:
            raise ValueError("Number of active joints must match waypoint columns")

        self._waypoints = waypoints
        self._robot_joint_space = robot_joint_space
        self._active_joints = active_joints
        self._time_per_segment = time_per_segment

        # Duration is time per segment times number of segments
        num_segments = len(waypoints) - 1
        self._duration = time_per_segment * num_segments

    # <end-linear-trajectory-init-snippet>

    # <start-linear-trajectory-duration-snippet>
    @property
    def duration(self) -> float:
        """Return the duration of the trajectory."""
        return self._duration

    # <end-linear-trajectory-duration-snippet>

    # <start-linear-trajectory-get-target-state-snippet>
    def get_target_state(self, time: float) -> Optional[mg.RobotState]:
        """Return the target robot state at the given time.

        Performs linear interpolation between waypoints.

        Args:
            time: Time along the trajectory (0.0 to duration)

        Returns:
            RobotState with joint positions, or None if time is out of bounds
        """
        if time < 0.0 or time > self._duration:
            return None

        # Find which segment we're in
        segment_idx = int(time / self._time_per_segment)
        segment_idx = min(segment_idx, len(self._waypoints) - 2)  # Clamp to last segment

        # Compute interpolation factor within the segment
        segment_time = time - (segment_idx * self._time_per_segment)
        alpha = segment_time / self._time_per_segment
        alpha = np.clip(alpha, 0.0, 1.0)  # Clamp to [0, 1]

        # Linear interpolation between waypoints
        start_waypoint = self._waypoints[segment_idx]
        end_waypoint = self._waypoints[segment_idx + 1]
        target_positions = start_waypoint + alpha * (end_waypoint - start_waypoint)

        # Return RobotState with joint positions
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                positions=(self._active_joints, wp.from_numpy(target_positions)),
            )
        )

    # <end-linear-trajectory-get-target-state-snippet>


# <end-linear-trajectory-snippet>


# ============================================================================
# 3. Helper Functions
# ============================================================================


def get_estimated_state_from_robot(robot: Articulation, robot_joint_space: list[str]) -> mg.RobotState:
    """Get the current estimated state of the robot.

    Args:
        robot: The robot articulation
        robot_joint_space: The full joint space of the robot

    Returns:
        RobotState with current joint positions, velocities, and efforts
    """
    # Get current joint positions, velocities, and efforts
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=robot_joint_space,
            positions=(robot_joint_space, robot.get_dof_positions()),
            velocities=(robot_joint_space, robot.get_dof_velocities()),
            efforts=(robot_joint_space, robot.get_dof_efforts()),
        )
    )


def apply_desired_state_to_robot(robot: Articulation, desired_state: mg.RobotState) -> None:
    """Apply the desired state to the robot.

    Args:
        robot: The robot articulation
        desired_state: The desired RobotState to apply
    """
    if desired_state.joints is None:
        return

    joint_state = desired_state.joints

    # Apply positions if present
    if joint_state.positions is not None:
        robot.set_dof_position_targets(joint_state.positions, dof_indices=joint_state.position_indices)

    # Apply velocities if present
    if joint_state.velocities is not None:
        robot.set_dof_velocity_targets(joint_state.velocities, dof_indices=joint_state.velocity_indices)

    # Apply efforts if present
    if joint_state.efforts is not None:
        robot.set_dof_efforts(joint_state.efforts, dof_indices=joint_state.effort_indices)


# ============================================================================
# 4. Scene Setup
# ============================================================================


async def setup_scene() -> tuple[Articulation, list[str]]:
    """Setup the simulation scene and return the robot and joint space.

    Returns:
        Tuple of (robot articulation, robot_joint_space)
    """
    # Setup scene
    await stage_utils.create_new_stage_async(template="default stage")

    # Add Franka robot
    assets_root_path = await get_assets_root_path_async()
    franka_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    robot_prim = stage_utils.add_reference_to_stage(usd_path=franka_path, path="/World/Franka")

    # await the next kit step:
    await omni.kit.app.get_app().next_update_async()

    # Set gripper variant (AlternateFinger is a common choice)
    robot_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    robot_prim.GetVariantSet("Mesh").SetVariantSelection("Performance")

    # Set camera view
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[2.0, 2.0, 1.5], target=[0.0, 0.0, 0.5])

    # Create articulation wrapper
    robot = Articulation("/World/Franka")
    await omni.kit.app.get_app().next_update_async()

    # Initialize physics
    SimulationManager.set_physics_dt(1.0 / 60.0)

    # Get robot joint space
    robot_joint_space = robot.dof_names
    print(f"\nRobot joint space: {robot_joint_space}")
    print(f"Number of DOFs: {robot.num_dofs}")

    # Start timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    await omni.kit.app.get_app().next_update_async()

    # Force the robot to an initial default position
    # Default positions: [panda_joint1-7, panda_finger_joint1-2]
    # Arm joints match robot descriptor default_q: [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75]
    # Gripper: 0.04, 0.04 = open gripper (0.0, 0.0 = closed)
    default_positions = np.array([0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75, 0.04, 0.04], dtype=np.float32)
    robot.set_dof_positions(wp.from_numpy(default_positions, dtype=wp.float32))
    print(f"  Set robot to default position")

    return robot, robot_joint_space


# ============================================================================
# 5. Trajectory Following Example
# ============================================================================


def run_trajectory_following_example(
    robot: Articulation, robot_joint_space: list[str], use_linear: bool = False
) -> None:
    """Run the trajectory following example.

    This demonstrates the complete cycle of using TrajectoryFollower:
    1. Set the trajectory using set_trajectory()
    2. Reset the controller to set the start time (must be called immediately before starting)
    3. Call forward() each time step to get the desired state

    The TrajectoryFollower has no opinion about which trajectory type it follows -
    it works with any object that implements the Trajectory interface.

    Args:
        robot: The robot articulation
        robot_joint_space: The full joint space of the robot
        use_linear: If True, use LinearTrajectory; if False, use Path.to_minimal_time_joint_trajectory()
    """
    print("\n" + "=" * 60)
    if use_linear:
        print("Trajectory Following Example: LinearTrajectory")
    else:
        print("Trajectory Following Example: Minimal-Time Trajectory")
    print("=" * 60)

    # Identify arm joints (excluding gripper)
    arm_joints = [name for name in robot_joint_space if "panda_joint" in name]
    print(f"  Arm joints ({len(arm_joints)}): {arm_joints}")

    # <start-define-waypoints-snippet>
    # Define waypoints for the arm (7 joints)
    # These waypoints define a simple motion pattern
    waypoints = np.array(
        [
            [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75],  # Default/home position
            [0.5, -0.8, 0.5, -2.0, 0.5, 1.5, 1.0],  # Waypoint 1
            [-0.5, -0.8, -0.5, -2.0, -0.5, 1.5, 0.5],  # Waypoint 2
            [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75],  # Return to home
        ],
        dtype=np.float32,
    )

    # <end-define-waypoints-snippet>

    # Create trajectory based on flag
    if use_linear:
        # <start-create-linear-trajectory-snippet>
        # Create LinearTrajectory with equal time per segment
        time_per_segment = 2.0  # seconds per segment
        trajectory = LinearTrajectory(
            waypoints=waypoints,
            robot_joint_space=robot_joint_space,
            active_joints=arm_joints,
            time_per_segment=time_per_segment,
        )
        # <end-create-linear-trajectory-snippet>
        print(f"  Using LinearTrajectory with {time_per_segment}s per segment")
        print(f"  Trajectory duration: {trajectory.duration:.3f} seconds")
    else:
        # <start-create-minimal-time-trajectory-snippet>
        # Create Path and convert to minimal-time trajectory
        path = mg.Path(waypoints)

        # Joint velocity and acceleration limits for Franka Panda
        # (not real values, just for demonstration)
        max_velocities = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # rad/s
        max_accelerations = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])  # rad/s²

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=max_velocities,
            max_accelerations=max_accelerations,
            robot_joint_space=robot_joint_space,
            active_joints=arm_joints,
        )
        # <end-create-minimal-time-trajectory-snippet>
        print(f"  Using Path.to_minimal_time_joint_trajectory()")
        print(f"  Trajectory duration: {trajectory.duration:.3f} seconds")

    # <start-trajectory-follower-cycle-snippet>
    # Create TrajectoryFollower controller
    # The TrajectoryFollower has no opinion about which trajectory type it follows -
    # it works with any object that implements the Trajectory interface
    follower = mg.TrajectoryFollower()

    # Step 1: Set the trajectory
    # This sets the trajectory and clears the start time
    follower.set_trajectory(trajectory)

    # Get initial estimated state
    estimated_state = get_estimated_state_from_robot(robot, robot_joint_space)

    # Step 2: Reset the controller to set the start time
    # This MUST be called immediately before starting to follow the trajectory
    # The reset() method sets the start time to the current simulation time
    simulation_time = 0.0
    if not follower.reset(estimated_state, None, simulation_time):
        print("  ERROR: Failed to reset trajectory follower!")
        return
    # <end-trajectory-follower-cycle-snippet>

    # Step 3: Run the control loop, calling forward() each time step
    # <start-trajectory-control-loop-snippet>
    dt = SimulationManager.get_physics_dt()
    num_steps = int((trajectory.duration + 1.0) / dt)  # Run slightly longer than trajectory duration

    for step in range(300):
        # stand still for a bit.
        simulation_app.update()

    for step in range(num_steps):
        # Update simulation
        simulation_app.update()
        simulation_time += dt

        # Get current estimated state
        estimated_state = get_estimated_state_from_robot(robot, robot_joint_space)

        # Call forward() to get desired state from trajectory
        # The TrajectoryFollower queries the trajectory at the current time
        # (relative to when reset() was called)
        desired_state = follower.forward(estimated_state, None, simulation_time)

        if desired_state is None:
            # Trajectory has ended or is out of bounds
            if step % 60 == 0:  # Print once per second
                print(f"  Trajectory ended at t={simulation_time:.3f}s")
            break

        # Apply desired state to robot
        apply_desired_state_to_robot(robot, desired_state)

        # Print status periodically
        if step % 120 == 0:  # Every 2 seconds
            if desired_state.joints is not None and desired_state.joints.positions is not None:
                positions = desired_state.joints.positions.numpy().flatten()
                print(f"\n  Step {step} (t={simulation_time:.3f}s):")
                print(f"    Desired positions: {positions[:3]}... (showing first 3 joints)")

    # Stop timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    # <end-trajectory-control-loop-snippet>

    print("\n  Trajectory following complete!")


# ============================================================================
# 6. Main Function
# ============================================================================


def main() -> None:
    """Run the complete trajectory following workflow."""
    parser = argparse.ArgumentParser(description="Trajectory following example")
    parser.add_argument(
        "--linear",
        action="store_true",
        help="Use LinearTrajectory instead of minimal-time trajectory",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Ignore: used for snippet testing only",
    )
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="Ignore: used for snippet testing only",
    )
    args, _ = parser.parse_known_args()

    print("=" * 60)
    print("Motion Generation API - Trajectory Following Example")
    print(f"  Linear: {args.linear}")
    print("=" * 60)

    # Setup scene asynchronously - this keeps the app responsive during asset loading
    print("Setting up scene (this may take a moment)...")
    robot, robot_joint_space = simulation_app.run_coroutine(setup_scene())

    # Run trajectory following example
    run_trajectory_following_example(robot, robot_joint_space, use_linear=args.linear)

    print("\n" + "=" * 60)
    print("Example Complete")
    print("=" * 60)

    # Keep window open for a moment to see results
    print("\nClosing in 2 seconds...")
    for _ in range(120):
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
