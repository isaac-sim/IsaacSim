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

"""Complete example demonstrating robot control with the Motion Generation API.

This example shows how to use a differential drive controller with optional
low-pass filtering and noise injection.
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
from isaacsim.core.experimental.objects import Cylinder
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async


# ============================================================================
# 2. Controller Implementations
# ============================================================================
# <start-low-pass-filter-snippet>
class LowPassFilterController(mg.BaseController):
    """A low-pass filter controller that filters all joint outputs (position, velocity, effort).

    This controller applies a low-pass filter to the entire underlying data array
    of the input RobotState, filtering all joint outputs simultaneously.
    """

    # <start-low-pass-filter-init-snippet>
    def __init__(self, robot_joint_space: list[str], alpha: float = 0.1) -> None:
        """Initialize the low-pass filter controller.

        Args:
            robot_joint_space: The full joint space of the robot
            alpha: Low-pass filter coefficient (0 < alpha <= 1). Smaller values = more filtering.
        """
        self.robot_joint_space = robot_joint_space
        self.alpha = alpha

        # Internal state: filtered data array (shape: (3, N))
        self.filtered_data_array = None

    # <end-low-pass-filter-init-snippet>

    def _validate_joint_spaces(self, estimated_state: mg.RobotState, setpoint_state: Optional[mg.RobotState]) -> bool:
        """Validate that joint spaces match across states.

        Args:
            estimated_state: Current estimated robot state.
            setpoint_state: Optional setpoint state (input to filter).

        Returns:
            True if all joint spaces match, False otherwise.
        """
        # Check estimated state joint space
        if estimated_state.joints is None:
            return False

        if estimated_state.joints.robot_joint_space != self.robot_joint_space:
            return False

        # Check setpoint state joint space if provided
        if setpoint_state is not None and setpoint_state.joints is not None:
            if setpoint_state.joints.robot_joint_space != self.robot_joint_space:
                return False

        return True

    # <start-low-pass-filter-reset-snippet>
    def reset(
        self, estimated_state: mg.RobotState, setpoint_state: Optional[mg.RobotState], t: float, **kwargs: object
    ) -> bool:
        """Initialize the controller.

        Resets the initial filter state to be exactly the underlying joint-data array from the estimated state.
        This prevents jerky motions - if the filter is initialized to match the exact state of the robot,
        then the robot will smoothly transition into following the filter as soon as it starts running.
        """
        # Validate joint spaces match
        if not self._validate_joint_spaces(estimated_state, setpoint_state):
            return False

        if estimated_state.joints is None:
            return False

        # Initialize filtered data array to match the estimated state's joint data array
        # This ensures smooth transition when the filter starts running
        self.filtered_data_array = wp.clone(estimated_state.joints.data_array)

        return True

    # <end-low-pass-filter-reset-snippet>

    # <start-low-pass-filter-forward-snippet>
    def forward(
        self, estimated_state: mg.RobotState, setpoint_state: Optional[mg.RobotState], t: float, **kwargs: object
    ) -> Optional[mg.RobotState]:
        """Compute filtered joint state.

        Applies low-pass filter to the entire data array of the setpoint state.
        Filters all joint outputs (position, velocity, effort) simultaneously.
        """
        # Validate joint spaces match
        if not self._validate_joint_spaces(estimated_state, setpoint_state):
            return None

        if self.filtered_data_array is None:
            return None

        # Get input data array from setpoint
        if setpoint_state is None or setpoint_state.joints is None:
            # No setpoint, return None
            return None

        input_joint_state = setpoint_state.joints
        input_data_array = input_joint_state.data_array
        input_valid_array = input_joint_state.valid_array

        # Apply low-pass filter to entire data array: filtered = alpha * input + (1 - alpha) * filtered
        input_data_np = input_data_array.numpy()
        filtered_data_np = self.filtered_data_array.numpy()
        filtered_data_np = self.alpha * input_data_np + (1 - self.alpha) * filtered_data_np
        self.filtered_data_array = wp.from_numpy(filtered_data_np, dtype=wp.float32)

        # Output filtered joint state with same valid array as input
        return mg.RobotState(
            joints=mg.JointState(
                robot_joint_space=self.robot_joint_space,
                data_array=self.filtered_data_array,
                valid_array=input_valid_array,
            )
        )

    # <end-low-pass-filter-forward-snippet>


# <end-low-pass-filter-snippet>


# <start-differential-drive-snippet>
class DifferentialDriveController(mg.BaseController):
    """A differential drive controller that tracks desired root velocity.

    This controller converts desired root linear and angular velocities
    into left and right wheel angular velocities using differential drive kinematics.

    Uses the unicycle model conversion:
        ω_R = (1/(2r)) * (2V + ωb)
        ω_L = (1/(2r)) * (2V - ωb)

    where ω is the desired angular velocity (yaw rate), V is the desired linear velocity,
    r is the radius of the wheels, and b is the distance between them.
    """

    # <start-differential-drive-init-snippet>
    def __init__(
        self,
        robot_joint_space: list[str],
        wheel_radius: float,
        wheel_base: float,
    ) -> None:
        """Initialize the differential drive controller.

        Args:
            robot_joint_space: The full joint space of the robot
            wheel_radius: Radius of left and right wheels in meters
            wheel_base: Distance between left and right wheels in meters
        """
        self.robot_joint_space = robot_joint_space
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.controlled_joints = ["left_wheel_joint", "right_wheel_joint"]

    # <end-differential-drive-init-snippet>

    def _validate_joint_spaces(self, estimated_state: mg.RobotState, setpoint_state: Optional[mg.RobotState]) -> bool:
        """Validate that joint spaces match across states.

        Args:
            estimated_state: Current estimated robot state.
            setpoint_state: Optional setpoint state.

        Returns:
            True if all joint spaces match, False otherwise.
        """
        # Check estimated state joint space
        if estimated_state.joints is None:
            return False

        if estimated_state.joints.robot_joint_space != self.robot_joint_space:
            return False

        return True

    # <start-differential-drive-reset-snippet>
    def reset(
        self, estimated_state: mg.RobotState, setpoint_state: Optional[mg.RobotState], t: float, **kwargs: object
    ) -> bool:
        """Initialize the controller.

        The DifferentialDriveController is stateless, so we don't need to do anything here.
        """
        return True

    # <end-differential-drive-reset-snippet>

    # <start-differential-drive-forward-snippet>
    def forward(
        self, estimated_state: mg.RobotState, setpoint_state: Optional[mg.RobotState], t: float, **kwargs: object
    ) -> Optional[mg.RobotState]:
        """Compute desired wheel angular velocities from root velocity setpoint.

        Converts desired root linear and angular velocities to wheel angular velocities
        using differential drive kinematics:
            ω_R = (1/(2r)) * (2V + ωb)
            ω_L = (1/(2r)) * (2V - ωb)

        where ω is angular velocity (yaw rate), V is linear velocity, r is wheel radius,
        and b is wheel base.
        """
        # First, verify that the correct inputs are there
        # We need a setpoint root state. If there is no root state, we return None.
        if setpoint_state is None or setpoint_state.root is None:
            return None

        # If the input joint state doesn't use the same robot_joint_space, we return None.
        if estimated_state.joints is None or estimated_state.joints.robot_joint_space != self.robot_joint_space:
            return None

        root_state = setpoint_state.root

        # Get desired linear and angular velocities
        # Default to zero if not provided
        if root_state.linear_velocity is not None:
            v_linear = root_state.linear_velocity.numpy()[0]  # Forward velocity (x-axis)
        else:
            v_linear = 0.0

        if root_state.angular_velocity is not None:
            v_angular = root_state.angular_velocity.numpy()[2]  # Yaw rate (z-axis)
        else:
            v_angular = 0.0

        # Convert root velocities to wheel angular velocities using differential drive kinematics
        # ω_R = (1/(2r)) * (2V + ωb)
        # ω_L = (1/(2r)) * (2V - ωb)
        inv_denominator = 1.0 / (2.0 * self.wheel_radius)
        omega_left = ((2.0 * v_linear) - (v_angular * self.wheel_base)) * inv_denominator
        omega_right = ((2.0 * v_linear) + (v_angular * self.wheel_base)) * inv_denominator

        # Output velocity commands for both wheels
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self.robot_joint_space,
                velocities=(
                    self.controlled_joints,
                    wp.array([omega_left, omega_right]),
                ),
            )
        )

    # <end-differential-drive-forward-snippet>


# <end-differential-drive-snippet>


# ============================================================================
# 3. Helper Functions
# ============================================================================


# <start-create-robotstate-from-robot-snippet>
def get_estimated_state_from_robot(robot: Articulation, robot_joint_space: list[str]) -> mg.RobotState:
    """Get the current estimated state of the robot.

    Args:
        robot: The robot articulation
        robot_joint_space: The full joint space of the robot

    Returns:
        RobotState representing the current robot state
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


# <end-create-robotstate-from-robot-snippet>


# <start-apply-desired-state-to-robot-snippet>
def apply_desired_state_to_robot(robot: Articulation, desired_state: mg.RobotState) -> None:
    """Apply a desired RobotState to the robot.

    Args:
        robot: The robot articulation
        desired_state: The desired robot state to apply
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


# <end-apply-desired-state-to-robot-snippet>
# ============================================================================
# 4. Real-time Control Loop
# ============================================================================
# <start-mobile-control-loop-snippet>
def run_differential_control_loop(
    controller: mg.BaseController,
    robot: Articulation,
    robot_joint_space: list[str],
    add_noise: bool = False,
    duration_seconds: float = 15.0,
) -> None:
    """Run a real-time control loop with a differential drive controller.

    This function provides root velocity setpoints (with optional noise) to the controller.

    Args:
        controller: The controller to run (differential drive, optionally wrapped with filter)
        robot: The robot articulation
        robot_joint_space: The full joint space of the robot
        add_noise: If True, add noise to the root velocity setpoints
        duration_seconds: How long to run the simulation
    """
    # Start timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()  # Allow physics to initialize

    # Get initial estimated state
    estimated_state = get_estimated_state_from_robot(robot, robot_joint_space)

    # Reset controller
    if not controller.reset(estimated_state, None, 0.0):
        return

    # Run simulation loop
    simulation_time = 0.0
    dt = SimulationManager.get_physics_dt()
    num_steps = int(duration_seconds / dt)

    # setpoint velocities:
    v_linear = 0.1  # m/s
    v_angular = 1.0  # rad/s

    # Noise parameters
    noise_std_linear = 0.1  # m/s
    noise_std_angular = 1.0  # rad/s
    for step in range(num_steps):
        # Update simulation
        simulation_app.update()
        simulation_time += dt

        # Get current estimated state
        estimated_state = get_estimated_state_from_robot(robot, robot_joint_space)

        if simulation_time > 2.0:

            # Add noise if requested
            v_linear_command = v_linear
            v_angular_command = v_angular
            if add_noise:
                v_linear_command += np.random.normal(0.0, noise_std_linear)
                v_angular_command += np.random.normal(0.0, noise_std_angular)

            # <start-create-robotstate-root-space-snippet>
            setpoint_state = mg.RobotState(
                root=mg.RootState(
                    linear_velocity=wp.array([v_linear_command, 0.0, 0.0]),  # Forward velocity
                    angular_velocity=wp.array([0.0, 0.0, v_angular_command]),  # Yaw rate
                )
            )
            # <end-create-robotstate-root-space-snippet>
        else:
            setpoint_state = None  # No setpoint for first 2 seconds (robot comes to stop)

        # Run controller
        desired_state = controller.forward(estimated_state, setpoint_state, simulation_time)

        # Apply desired state to robot
        if desired_state is not None:
            apply_desired_state_to_robot(robot, desired_state)

    # Stop timeline
    timeline.pause()


# <end-mobile-control-loop-snippet>


# ============================================================================
# 5. Main Example Function
# ============================================================================
# <start-sequential-controller-snippet>
def differential_drive_control(
    robot: Articulation, robot_joint_space: list[str], add_noise: bool = False, use_filter: bool = False
) -> None:
    """Example: Differential drive controller tracking root velocity.

    Args:
        robot: The robot articulation
        robot_joint_space: The full joint space of the robot
        add_noise: If True, add noise to the root velocity setpoints
        use_filter: If True, wrap the differential controller with a low-pass filter
    """
    # Create differential drive controller
    # For Jetbot, adjust these parameters as needed
    differential_controller = DifferentialDriveController(
        robot_joint_space=robot_joint_space,
        wheel_radius=0.03,  # 3 cm wheel radius
        wheel_base=0.1125,  # 11.25 cm wheel base
    )

    # Optionally wrap with low-pass filter using SequentialController
    if use_filter:
        filter_controller = LowPassFilterController(
            robot_joint_space=robot_joint_space,
            alpha=0.01,  # Low-pass filter coefficient
        )
        # SequentialController: differential controller output becomes filter input
        controller = mg.SequentialController([differential_controller, filter_controller])
    else:
        controller = differential_controller

    # Run the control loop
    run_differential_control_loop(controller, robot, robot_joint_space, add_noise=add_noise)


# <end-sequential-controller-snippet>


# ============================================================================
# 6. Main Function
# ============================================================================
async def setup_scene() -> tuple[Articulation, list[str]]:
    """Setup the simulation scene and return the robot and joint space.

    Returns:
        Tuple of (robot articulation, robot_joint_space)
    """
    # Setup scene
    await stage_utils.create_new_stage_async(template="default stage")

    # Add ground plane
    assets_root_path = await get_assets_root_path_async()

    # Add Jetbot robot
    jetbot_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    stage_utils.add_reference_to_stage(usd_path=jetbot_path, path="/World/Jetbot")
    await omni.kit.app.get_app().next_update_async()

    # Set camera view
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[0.0, 0.1, 1.5], target=[0.0, 0.1, 0.0])

    # draw a small red disk:
    disk = Cylinder(
        paths="/World/red_disk",
        radii=0.1,
        heights=0.01,
        axes="Z",
        colors=[1.0, 0.0, 0.0],
        positions=[0, 0.1, 0],
    )

    # Create articulation wrapper
    robot = Articulation("/World/Jetbot")
    await omni.kit.app.get_app().next_update_async()

    # Initialize physics
    SimulationManager.set_physics_dt(1.0 / 60.0)

    # Get robot joint space
    robot_joint_space = robot.dof_names
    print(f"\nRobot joint space: {robot_joint_space}")
    print(f"Number of DOFs: {robot.num_dofs}")

    return robot, robot_joint_space


def main() -> None:
    """Run the complete robot control workflow."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Differential drive controller example with optional noise and filtering"
    )
    parser.add_argument(
        "--noise",
        action="store_true",
        help="Add noise to the root velocity setpoints",
    )
    parser.add_argument(
        "--filter",
        action="store_true",
        help="Apply low-pass filter to the differential controller output using SequentialController",
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
    print("Motion Generation API - Differential Drive Controller Example")
    print(f"  Noise: {args.noise}")
    print(f"  Filter: {args.filter}")
    print("=" * 60)

    # Setup scene asynchronously - this keeps the app responsive during asset loading
    print("Setting up scene (this may take a moment)...")
    robot, robot_joint_space = simulation_app.run_coroutine(setup_scene())

    # Run differential drive example with optional noise and filter
    differential_drive_control(robot, robot_joint_space, add_noise=args.noise, use_filter=args.filter)

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
