# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrates trajectory generation capabilities for the UR10 robot using Lula motion planning in Isaac Sim."""

from __future__ import annotations

import os

import carb
import lula
import numpy as np
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation import (
    ArticulationTrajectory,
    LulaCSpaceTrajectoryGenerator,
    LulaKinematicsSolver,
    LulaTaskSpaceTrajectoryGenerator,
)
from isaacsim.storage.native import get_assets_root_path


class UR10TrajectoryGenerationExample:
    """Demonstrates trajectory generation capabilities for the UR10 robot using Lula motion planning.

    This class provides comprehensive examples of generating and executing robot trajectories using both
    configuration space (C-space) and task space planning approaches. It showcases the integration of
    Lula's trajectory generation capabilities with Isaac Sim's articulation system.

    The class demonstrates three main trajectory generation methods:

    1. **C-space trajectory generation**: Plans trajectories directly in joint space using predefined
       joint configurations, with support for both time-optimal and timestamped trajectories.

    2. **Task space trajectory generation**: Plans trajectories in Cartesian space by specifying
       end-effector positions and orientations, automatically handling inverse kinematics.

    3. **Advanced trajectory generation**: Combines C-space and task space planning using composite
       path specifications with various motion primitives including linear paths, arcs, and rotations.

    The example includes visualization of target poses using reference frames and provides a complete
    workflow from trajectory computation to robot execution through articulation actions.

    Key features:
    - Integration with LulaCSpaceTrajectoryGenerator and LulaTaskSpaceTrajectoryGenerator
    - Forward kinematics computation for pose visualization
    - Trajectory execution at 60 Hz physics timesteps
    - Automatic trajectory looping with pause intervals
    - Robot state management and reset functionality

    The class is designed as an example for the isaacsim.robot_motion.motion_generation.examples
    extension, demonstrating practical usage patterns for robot motion planning in Isaac Sim.
    """

    def __init__(self) -> None:
        self._c_space_trajectory_generator = None
        self._taskspace_trajectory_generator = None
        self._kinematics_solver = None

        self._action_sequence = []
        self._action_sequence_index = 0

        self._articulation = None

    def load_example_assets(self) -> list:
        """Loads the UR10 robot asset into the stage and initializes the articulation.

        Returns:
            List containing the loaded articulation asset for registration with the core.World.
        """
        # Add the Franka and target to the stage
        # The position in which things are loaded is also the position in which they

        robot_prim_path = "/ur10"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = SingleArticulation(robot_prim_path)

        # Return assets that were added to the stage so that they can be registered with the core.World
        return [self._articulation]

    def setup(self) -> None:
        """Initializes trajectory generators and kinematics solver for the UR10 robot.

        Sets up the C-space trajectory generator, task-space trajectory generator, and kinematics solver using
        configuration files from the motion_generation extension.
        """
        # Config files for supported robots are stored in the motion_generation extension under "/motion_policy_configs"
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

        # Initialize a LulaCSpaceTrajectoryGenerator object
        self._c_space_trajectory_generator = LulaCSpaceTrajectoryGenerator(
            robot_description_path=rmp_config_dir + "/universal_robots/ur10/rmpflow/ur10_robot_description.yaml",
            urdf_path=rmp_config_dir + "/universal_robots/ur10/ur10_robot.urdf",
        )

        self._taskspace_trajectory_generator = LulaTaskSpaceTrajectoryGenerator(
            robot_description_path=rmp_config_dir + "/universal_robots/ur10/rmpflow/ur10_robot_description.yaml",
            urdf_path=rmp_config_dir + "/universal_robots/ur10/ur10_robot.urdf",
        )

        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=rmp_config_dir + "/universal_robots/ur10/rmpflow/ur10_robot_description.yaml",
            urdf_path=rmp_config_dir + "/universal_robots/ur10/ur10_robot.urdf",
        )

        self._end_effector_name = "ee_link"

    def setup_cspace_trajectory(self) -> None:
        """Sets up a configuration space trajectory with predefined waypoints.

        Creates both time-optimal and timestamped C-space trajectories and visualizes the waypoints in task space
        using frame markers. Generates action sequences for both trajectories to be executed sequentially.
        """
        c_space_points = np.array(
            [
                [
                    -0.41,
                    0.5,
                    -2.36,
                    -1.28,
                    5.13,
                    -4.71,
                ],
                [
                    -1.43,
                    1.0,
                    -2.58,
                    -1.53,
                    6.0,
                    -4.74,
                ],
                [
                    -2.83,
                    0.34,
                    -2.11,
                    -1.38,
                    1.26,
                    -4.71,
                ],
                [
                    -0.41,
                    0.5,
                    -2.36,
                    -1.28,
                    5.13,
                    -4.71,
                ],
            ]
        )

        timestamps = np.array([0, 5, 10, 13])

        trajectory_time_optimal = self._c_space_trajectory_generator.compute_c_space_trajectory(c_space_points)
        trajectory_timestamped = self._c_space_trajectory_generator.compute_timestamped_c_space_trajectory(
            c_space_points, timestamps
        )

        # Visualize c-space targets in task space
        for i, point in enumerate(c_space_points):
            position, rotation = self._kinematics_solver.compute_forward_kinematics(self._end_effector_name, point)
            add_reference_to_stage(
                get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/visualized_frames/target_{i}"
            )
            frame = SingleXFormPrim(f"/visualized_frames/target_{i}", scale=np.array([0.04, 0.04, 0.04]))
            frame.set_world_pose(position, rot_matrices_to_quats(rotation))

        if trajectory_time_optimal is None or trajectory_timestamped is None:
            carb.log_warn("No trajectory could be computed")
            self._action_sequence = []
        else:
            physics_dt = 1 / 60
            self._action_sequence = []

            # Follow both trajectories in a row

            articulation_trajectory_time_optimal = ArticulationTrajectory(
                self._articulation, trajectory_time_optimal, physics_dt
            )
            self._action_sequence.extend(articulation_trajectory_time_optimal.get_action_sequence())

            articulation_trajectory_timestamped = ArticulationTrajectory(
                self._articulation, trajectory_timestamped, physics_dt
            )
            self._action_sequence.extend(articulation_trajectory_timestamped.get_action_sequence())

    def setup_taskspace_trajectory(self) -> None:
        """Sets up a task space trajectory with position and orientation targets.

        Creates a trajectory that moves the end-effector through a rectangular path in task space and visualizes
        the target positions with frame markers. Generates an action sequence for trajectory execution.
        """
        task_space_position_targets = np.array(
            [[0.3, -0.3, 0.1], [0.3, 0.3, 0.1], [0.3, 0.3, 0.5], [0.3, -0.3, 0.5], [0.3, -0.3, 0.1]]
        )

        task_space_orientation_targets = np.tile(np.array([0, 1, 0, 0]), (5, 1))

        trajectory = self._taskspace_trajectory_generator.compute_task_space_trajectory_from_points(
            task_space_position_targets, task_space_orientation_targets, self._end_effector_name
        )

        # Visualize task-space targets in task space
        for i, (position, orientation) in enumerate(zip(task_space_position_targets, task_space_orientation_targets)):
            add_reference_to_stage(
                get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", f"/visualized_frames/target_{i}"
            )
            frame = SingleXFormPrim(f"/visualized_frames/target_{i}", scale=np.array([0.04, 0.04, 0.04]))
            frame.set_world_pose(position, orientation)

        if trajectory is None:
            carb.log_warn("No trajectory could be computed")
            self._action_sequence = []
        else:
            physics_dt = 1 / 60
            articulation_trajectory = ArticulationTrajectory(self._articulation, trajectory, physics_dt)

            # Get a sequence of ArticulationActions that are intended to be passed to the robot at 1/60 second intervals
            self._action_sequence = articulation_trajectory.get_action_sequence()

    def setup_advanced_trajectory(self) -> None:
        """Sets up an advanced composite trajectory combining C-space and task-space movements.

        Demonstrates various trajectory types including linear interpolation, pure translation/rotation,
        three-point arcs, and tangent arcs. Uses CompositePathSpec to combine multiple movement primitives
        into a single complex trajectory.
        """
        # The following code demonstrates how to specify a complicated cspace and taskspace path
        # using the lula.CompositePathSpec object

        initial_c_space_robot_pose = np.array([0, 0, 0, 0, 0, 0])

        # Combine a cspace and taskspace trajectory
        composite_path_spec = lula.create_composite_path_spec(initial_c_space_robot_pose)

        #############################################################################
        # Demonstrate all the available movements in a taskspace path spec:

        # Lula has its own classes for Rotations and 6 DOF poses: Rotation3 and Pose3
        r0 = lula.Rotation3(np.pi / 2, np.array([1.0, 0.0, 0.0]))
        t0 = np.array([0.3, -0.1, 0.3])
        task_space_spec = lula.create_task_space_path_spec(lula.Pose3(r0, t0))

        # Add path linearly interpolating between r0,r1 and t0,t1
        t1 = np.array([0.3, -0.1, 0.5])
        r1 = lula.Rotation3(np.pi / 3, np.array([1, 0, 0]))
        task_space_spec.add_linear_path(lula.Pose3(r1, t1))

        # Add pure translation.  Constant rotation is assumed
        task_space_spec.add_translation(t0)

        # Add pure rotation.
        task_space_spec.add_rotation(r0)

        # Add three-point arc with constant orientation.
        t2 = np.array(
            [
                0.3,
                0.3,
                0.3,
            ]
        )
        midpoint = np.array([0.3, 0, 0.5])
        task_space_spec.add_three_point_arc(t2, midpoint, constant_orientation=True)

        # Add three-point arc with tangent orientation.
        task_space_spec.add_three_point_arc(t0, midpoint, constant_orientation=False)

        # Add three-point arc with orientation target.
        task_space_spec.add_three_point_arc_with_orientation_target(lula.Pose3(r1, t2), midpoint)

        # Add tangent arc with constant orientation. Tangent arcs are circles that connect two points
        task_space_spec.add_tangent_arc(t0, constant_orientation=True)

        # Add tangent arc with tangent orientation.
        task_space_spec.add_tangent_arc(t2, constant_orientation=False)

        # Add tangent arc with orientation target.
        task_space_spec.add_tangent_arc_with_orientation_target(lula.Pose3(r0, t0))

        ###################################################
        # Demonstrate the usage of a c_space path spec:
        c_space_spec = lula.create_c_space_path_spec(np.array([0, 0, 0, 0, 0, 0]))

        c_space_spec.add_c_space_waypoint(np.array([0, 0.5, -2.0, -1.28, 5.13, -4.71]))

        ##############################################################
        # Combine the two path specs together into a composite spec:

        # specify how to connect initial_c_space and task_space points with transition_mode option
        transition_mode = lula.CompositePathSpec.TransitionMode.FREE
        composite_path_spec.add_task_space_path_spec(task_space_spec, transition_mode)

        transition_mode = lula.CompositePathSpec.TransitionMode.FREE
        composite_path_spec.add_c_space_path_spec(c_space_spec, transition_mode)

        # Transition Modes:
        # lula.CompositePathSpec.TransitionMode.LINEAR_TASK_SPACE:
        #      Connect cspace to taskspace points linearly through task space.  This mode is only available when adding a task_space path spec.
        # lula.CompositePathSpec.TransitionMode.FREE:
        #      Put no constraints on how cspace and taskspace points are connected
        # lula.CompositePathSpec.TransitionMode.SKIP:
        #      Skip the first point of the path spec being added, using the last pose instead

        trajectory = self._taskspace_trajectory_generator.compute_task_space_trajectory_from_path_spec(
            composite_path_spec, self._end_effector_name
        )

        if trajectory is None:
            carb.log_warn("No trajectory could be computed")
            self._action_sequence = []
        else:
            physics_dt = 1 / 60
            articulation_trajectory = ArticulationTrajectory(self._articulation, trajectory, physics_dt)

            # Get a sequence of ArticulationActions that are intended to be passed to the robot at 1/60 second intervals
            self._action_sequence = articulation_trajectory.get_action_sequence()

    def update(self, step: float) -> None:
        """Updates the trajectory execution by applying the next action in the sequence.

        Teleports the robot to the initial position when starting a trajectory and applies actions sequentially.
        Automatically repeats the trajectory sequence with a 10-frame pause between cycles.

        Args:
            step: Current simulation timestep.
        """
        if len(self._action_sequence) == 0:
            return

        if self._action_sequence_index >= len(self._action_sequence):
            self._action_sequence_index += 1
            self._action_sequence_index %= (
                len(self._action_sequence) + 10
            )  # Wait 10 frames before repeating trajectories
            return

        if self._action_sequence_index == 0:
            self._teleport_robot_to_position(self._action_sequence[0])

        self._articulation.apply_action(self._action_sequence[self._action_sequence_index])

        self._action_sequence_index += 1
        self._action_sequence_index %= len(self._action_sequence) + 10  # Wait 10 frames before repeating trajectories

    def reset(self) -> None:
        """Resets the trajectory execution state and cleans up visualization frames.

        Clears the action sequence, resets the action index, and removes any frame markers from the stage.
        """
        # Delete any visualized frames
        if get_prim_at_path("/visualized_frames"):
            delete_prim("/visualized_frames")

        self._action_sequence = []
        self._action_sequence_index = 0

    def _teleport_robot_to_position(self, articulation_action: object) -> None:
        """Teleports the robot to the specified joint configuration instantly.

        Sets joint positions and velocities based on the articulation action to position the robot
        at the trajectory starting point.

        Args:
            articulation_action: Action containing joint indices and target positions for teleportation.
        """
        initial_positions = np.zeros(self._articulation.num_dof)
        initial_positions[articulation_action.joint_indices] = articulation_action.joint_positions

        self._articulation.set_joint_positions(initial_positions)
        self._articulation.set_joint_velocities(np.zeros_like(initial_positions))
