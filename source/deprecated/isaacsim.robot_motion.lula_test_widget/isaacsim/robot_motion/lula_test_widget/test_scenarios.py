# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test scenarios for Lula motion planning and control algorithms."""

from __future__ import annotations

import numpy as np
from isaacsim.core.api.objects.cone import VisualCone
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.core.api.objects.cylinder import VisualCylinder
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.numpy import rot_matrices_to_quats
from isaacsim.core.utils.prims import delete_prim, is_prim_path_valid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.articulation_trajectory import ArticulationTrajectory
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.motion_policies import RmpFlow
from isaacsim.robot_motion.motion_generation.lula.trajectory_generator import LulaTaskSpaceTrajectoryGenerator
from isaacsim.robot_motion.motion_generation.motion_policy_controller import MotionPolicyController

from .controllers import KinematicsController, TrajectoryController


class LulaTestScenarios:
    """Test scenarios for Lula motion planning and control algorithms.

    This class provides a comprehensive testing framework for robotic motion planning scenarios using Lula
    kinematics solvers, RmpFlow motion policies, and trajectory generation. It enables users to create and
    manage various test scenarios including inverse kinematics target following, obstacle avoidance with
    RmpFlow, custom trajectory execution, and sinusoidal target tracking.

    The class manages visual elements like targets, obstacles, coordinate frames, and trajectory waypoints,
    providing both interactive control and automated scenario execution. It supports real-time visualization
    of end-effector frames, collision spheres for debugging RmpFlow, and dynamic target updates.

    Key features include:
    - Inverse kinematics solving with target following
    - RmpFlow-based motion planning with obstacle avoidance
    - Custom trajectory generation and execution
    - Sinusoidal target tracking scenarios
    - Visual debugging tools for motion planning
    - Dynamic waypoint management for trajectories
    - End-effector frame visualization
    - Collision sphere visualization for RmpFlow debugging
    """

    def __init__(self) -> None:
        self._target = None
        self._obstacles = []

        self._trajectory_base_frame = None
        self._trajectory_targets = []

        self._controller = None

        self.timestep = 0

        self.lula_ik = None
        self.rmpflow = None
        self.traj_gen = None
        self.use_orientation = True

        self.scenario_name = ""

        self.rmpflow_debug_mode = False

        self._ee_frame_prim = None
        self.art_ik = None

    def visualize_ee_frame(self, articulation: object, ee_frame: str) -> None:
        """Visualizes the end effector frame for the given articulation.

        Args:
            articulation: The articulation for which to visualize the end effector frame.
            ee_frame: The end effector frame to visualize.
        """
        if self.lula_ik is None or articulation is None:
            return

        if self._ee_frame_prim is not None:
            delete_prim(self._ee_frame_prim.prim_path)

        self.art_ik = ArticulationKinematicsSolver(articulation, self.lula_ik, ee_frame)
        position, orientation = self.art_ik.compute_end_effector_pose()
        orientation = rot_matrices_to_quats(orientation)
        self._ee_frame_prim = self._create_frame_prim(position, orientation, "/Lula/end_effector")

    def stop_visualize_ee_frame(self) -> None:
        """Stops visualizing the end effector frame and cleans up associated resources."""
        if self._ee_frame_prim is not None:
            delete_prim(self._ee_frame_prim.prim_path)
        self._ee_frame_prim = None
        self.art_ik = None

    def toggle_rmpflow_debug_mode(self) -> None:
        """Toggles the RMPFlow debug mode on or off.

        When enabled, visualization of collision spheres is activated and state updates are ignored.
        When disabled, collision sphere visualization is stopped and state updates resume.
        """
        self.rmpflow_debug_mode = not self.rmpflow_debug_mode
        if self.rmpflow is None:
            return

        if self.rmpflow_debug_mode:
            self.rmpflow.set_ignore_state_updates(True)
            self.rmpflow.visualize_collision_spheres()
        else:
            self.rmpflow.set_ignore_state_updates(False)
            self.rmpflow.stop_visualizing_collision_spheres()

    def initialize_ik_solver(self, robot_description_path: str, urdf_path: str) -> None:
        """Initializes the Lula inverse kinematics solver.

        Args:
            robot_description_path: Path to the robot description file.
            urdf_path: Path to the URDF file for the robot.
        """
        self.lula_ik = LulaKinematicsSolver(robot_description_path, urdf_path)

    def get_ik_frames(self) -> list:
        """Gets all available frame names from the inverse kinematics solver.

        Returns:
            List of frame names, or empty list if IK solver is not initialized.
        """
        if self.lula_ik is None:
            return []
        return self.lula_ik.get_all_frame_names()

    def on_ik_follow_target(self, articulation: object, ee_frame_name: str) -> None:
        """Sets up an inverse kinematics scenario where the robot follows a target.

        Args:
            articulation: The articulation to control.
            ee_frame_name: Name of the end effector frame to use for tracking.
        """
        self.scenario_reset()
        if self.lula_ik is None:
            return
        art_ik = ArticulationKinematicsSolver(articulation, self.lula_ik, ee_frame_name)
        self._controller = KinematicsController("Lula Kinematics Controller", art_ik)

        self._create_target()

    def on_custom_trajectory(self, robot_description_path: str, urdf_path: str) -> None:
        """Sets up a custom trajectory scenario with waypoints forming a rectangular path.

        Args:
            robot_description_path: Path to the robot description file.
            urdf_path: Path to the URDF file for the robot.
        """
        self.scenario_reset()
        if self.lula_ik is None:
            return

        self.scenario_name = "Custom Trajectory"

        orientation = np.array([0, 1, 0, 0])
        rect_path = np.array([[0.3, -0.3, 0.1], [0.3, 0.3, 0.1], [0.3, 0.3, 0.5], [0.3, -0.3, 0.5], [0.3, -0.3, 0.1]])

        self.traj_gen = LulaTaskSpaceTrajectoryGenerator(robot_description_path, urdf_path)

        self._trajectory_base_frame = SingleXFormPrim("/Trajectory", position=np.array([0, 0, 0]))
        for i in range(4):
            frame_prim = self._create_frame_prim(rect_path[i], orientation, f"/Trajectory/Target_{i+1}")
            self._trajectory_targets.append(frame_prim)

    def create_trajectory_controller(self, articulation: object, ee_frame: str) -> None:
        """Creates a trajectory controller for following waypoints in the custom trajectory scenario.

        Args:
            articulation: The articulation to control.
            ee_frame: The end effector frame to use for trajectory following.
        """
        if self.traj_gen is None:
            return

        positions = np.empty((len(self._trajectory_targets), 3))
        orientations = np.empty((len(self._trajectory_targets), 4))

        for i, target in enumerate(self._trajectory_targets):
            positions[i], orientations[i] = target.get_world_pose()

        trajectory = self.traj_gen.compute_task_space_trajectory_from_points(positions, orientations, ee_frame)
        art_traj = ArticulationTrajectory(articulation, trajectory, 1 / 60)
        self._controller = TrajectoryController("Trajectory Controller", art_traj)

    def delete_waypoint(self) -> None:
        """Deletes the last waypoint from the custom trajectory scenario.

        Only removes waypoints if there are more than 2 remaining to maintain a valid trajectory.
        """
        if self.scenario_name == "Custom Trajectory" and len(self._trajectory_targets) > 2:
            waypoint = self._trajectory_targets[-1]
            delete_prim(waypoint.prim_path)
            self._trajectory_targets = self._trajectory_targets[:-1]

    def add_waypoint(self) -> None:
        """Adds a new waypoint to the custom trajectory scenario.

        The new waypoint is positioned at the average location of existing waypoints.
        """
        if self.scenario_name == "Custom Trajectory":
            orientation = self._trajectory_targets[-1].get_world_pose()[1]
            positions = []
            for waypoint in self._trajectory_targets:
                positions.append(waypoint.get_world_pose()[0])
            waypoint = self._create_frame_prim(
                np.mean(positions, axis=0), orientation, f"/Trajectory/Target_{len(self._trajectory_targets)+1}"
            )
            self._trajectory_targets.append(waypoint)

    def on_rmpflow_follow_target_obstacles(self, articulation: object, **rmp_config: object) -> None:
        """Sets up RmpFlow scenario for target following with obstacle avoidance.

        Creates a motion policy controller using RmpFlow for the articulation to follow a target
        while avoiding wall obstacles. The scenario includes a red target cube and two wall obstacles.

        Args:
            articulation: The articulation to control.
            **rmp_config: Configuration parameters passed to RmpFlow initialization.
        """
        self.scenario_reset()
        self.rmpflow = RmpFlow(**rmp_config)
        if self.rmpflow_debug_mode:
            self.rmpflow.set_ignore_state_updates(True)
            self.rmpflow.visualize_collision_spheres()

        self.rmpflow.set_robot_base_pose(*articulation.get_world_pose())
        art_rmp = ArticulationMotionPolicy(articulation, self.rmpflow, 1 / 60)
        self._controller = MotionPolicyController("RmpFlow Controller", art_rmp)

        self._create_target()
        self._create_wall()
        self._create_wall(position=np.array([0.4, 0, 0.1]), orientation=np.array([1, 0, 0, 0]))

        for obstacle in self._obstacles:
            self.rmpflow.add_obstacle(obstacle)

    def on_rmpflow_follow_sinusoidal_target(self, articulation: object, **rmp_config: object) -> None:
        """Sets up RmpFlow scenario for following a sinusoidal target trajectory.

        Creates a motion policy controller using RmpFlow for the articulation to follow a target
        that moves in a sinusoidal pattern when updated via scenario parameters.

        Args:
            articulation: The articulation to control.
            **rmp_config: Configuration parameters passed to RmpFlow initialization.
        """
        self.scenario_reset()
        self.scenario_name = "Sinusoidal Target"
        self.rmpflow = RmpFlow(**rmp_config)
        if self.rmpflow_debug_mode:
            self.rmpflow.set_ignore_state_updates(True)
            self.rmpflow.visualize_collision_spheres()
        self.rmpflow.set_robot_base_pose(*articulation.get_world_pose())
        art_rmp = ArticulationMotionPolicy(articulation, self.rmpflow, 1 / 60)
        self._controller = MotionPolicyController("RmpFlow Controller", art_rmp)

        self._create_target()

    def get_rmpflow(self) -> RmpFlow | None:
        """The RmpFlow motion policy instance.

        Returns:
            The current RmpFlow instance, or None if not initialized.
        """
        return self.rmpflow

    def _create_target(self, position: object = None, orientation: object = None) -> None:
        """Creates a red target cube for motion control scenarios.

        Args:
            position: World position for the target cube.
            orientation: World orientation for the target cube.
        """
        if position is None:
            position = np.array([0.5, 0, 0.5])
        if orientation is None:
            orientation = np.array([0, -1, 0, 0])
        self._target = VisualCuboid(
            "/World/Target", size=0.05, position=position, orientation=orientation, color=np.array([1.0, 0, 0])
        )

    def _create_frame_prim(self, position: object, orientation: object, parent_prim_path: str) -> SingleXFormPrim:
        """Creates a coordinate frame visualization with colored axes.

        The frame consists of X (red), Y (green), and Z (blue) axes represented by
        cylinders with cone tips.

        Args:
            position: World position for the frame.
            orientation: World orientation for the frame.
            parent_prim_path: USD prim path where the frame will be created.

        Returns:
            The SingleXFormPrim representing the coordinate frame.
        """
        frame_xform = SingleXFormPrim(parent_prim_path, position=position, orientation=orientation)

        line_len = 0.04
        line_width = 0.004
        cone_radius = 0.01
        cone_len = 0.02

        x_axis = VisualCylinder(
            parent_prim_path + "/X_line",
            translation=np.array([line_len / 2, 0, 0]),
            orientation=euler_angles_to_quat([0, np.pi / 2, 0]),
            color=np.array([1, 0, 0]),
            height=line_len,
            radius=line_width,
        )
        x_tip = VisualCone(
            parent_prim_path + "/X_tip",
            translation=np.array([line_len + cone_len / 2, 0, 0]),
            orientation=euler_angles_to_quat([0, np.pi / 2, 0]),
            color=np.array([1, 0, 0]),
            height=cone_len,
            radius=cone_radius,
        )

        y_axis = VisualCylinder(
            parent_prim_path + "/Y_line",
            translation=np.array([0, line_len / 2, 0]),
            orientation=euler_angles_to_quat([-np.pi / 2, 0, 0]),
            color=np.array([0, 1, 0]),
            height=line_len,
            radius=line_width,
        )
        y_tip = VisualCone(
            parent_prim_path + "/Y_tip",
            translation=np.array([0, line_len + cone_len / 2, 0]),
            orientation=euler_angles_to_quat([-np.pi / 2, 0, 0]),
            color=np.array([0, 1, 0]),
            height=cone_len,
            radius=cone_radius,
        )

        z_axis = VisualCylinder(
            parent_prim_path + "/Z_line",
            translation=np.array([0, 0, line_len / 2]),
            orientation=euler_angles_to_quat([0, 0, 0]),
            color=np.array([0, 0, 1]),
            height=line_len,
            radius=line_width,
        )
        z_tip = VisualCone(
            parent_prim_path + "/Z_tip",
            translation=np.array([0, 0, line_len + cone_len / 2]),
            orientation=euler_angles_to_quat([0, 0, 0]),
            color=np.array([0, 0, 1]),
            height=cone_len,
            radius=cone_radius,
        )

        return frame_xform

    def _create_wall(self, position: object = None, orientation: object = None) -> None:
        """Creates a blue wall obstacle and adds it to the obstacles list.

        Args:
            position: World position for the wall obstacle.
            orientation: World orientation for the wall obstacle.
        """
        cube_prim_path = find_unique_string_name(
            initial_name="/World/WallObstacle", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        if position is None:
            position = np.array([0.45, -0.15, 0.5])
        if orientation is None:
            orientation = euler_angles_to_quat(np.array([0, 0, np.pi / 2]))
        cube = VisualCuboid(
            prim_path=cube_prim_path,
            position=position,
            orientation=orientation,
            size=1.0,
            scale=np.array([0.1, 0.5, 0.6]),
            color=np.array([0, 0, 1.0]),
        )
        self._obstacles.append(cube)

    def set_use_orientation(self, use_orientation: bool) -> None:
        """Sets whether orientation constraints are used in motion control.

        Args:
            use_orientation: Whether to use orientation constraints in target following.
        """
        self.use_orientation = use_orientation

    def full_reset(self) -> None:
        """Performs a complete reset of all scenario data and Lula components.

        Clears the current scenario and resets the Lula IK solver, end effector visualization,
        and orientation usage setting to defaults.
        """
        self.scenario_reset()

        self.lula_ik = None
        self.use_orientation = True

        if self._ee_frame_prim is not None:
            delete_prim("/Lula")
        self._ee_frame_prim = None
        self.art_ik = None

    def scenario_reset(self) -> None:
        """Resets the current scenario by clearing targets, obstacles, and trajectories.

        Deletes all scenario-specific prims from the stage and resets internal state,
        including timestep and scenario name.
        """
        if self._target is not None:
            delete_prim(self._target.prim_path)
        if self._trajectory_base_frame is not None:
            delete_prim(self._trajectory_base_frame.prim_path)
        for obstacle in self._obstacles:
            delete_prim(obstacle.prim_path)

        self._target = None
        self._obstacles = []
        self._trajectory_targets = []
        self._trajectory_base_frame = None
        self._controller = None

        if self.rmpflow is not None:
            self.rmpflow.stop_visualizing_collision_spheres()

        self.timestep = 0
        self.scenario_name = ""

    def update_scenario(self, **scenario_params: object) -> None:
        """Updates the current scenario based on its type and provided parameters.

        For "Sinusoidal Target" scenarios, moves the target in a sinusoidal pattern
        based on frequency and radius parameters.

        Args:
            **scenario_params: Scenario-specific parameters. For sinusoidal target scenarios,
                includes w_z (vertical frequency), w_xy (horizontal frequency),
                rad_z (vertical radius), rad_xy (horizontal radius), and height (base height).
        """
        if self.scenario_name == "Sinusoidal Target":
            w_z = scenario_params["w_z"]
            w_xy = scenario_params["w_xy"]

            rad_z = scenario_params["rad_z"]
            rad_xy = scenario_params["rad_xy"]

            height = scenario_params["height"]

            z = height + rad_z * np.sin(2 * np.pi * w_z * self.timestep / 60)
            a = 2 * np.pi * w_xy * self.timestep / 60
            if (a / np.pi) % 4 > 2:
                a = -a
            x, y = rad_xy * np.cos(a), rad_xy * np.sin(a)

            target_position = np.array([x, y, z])
            target_orientation = euler_angles_to_quat(np.array([np.pi / 2, 0, np.pi / 2 + a]))

            self._target.set_world_pose(target_position, target_orientation)
        self.timestep += 1

    def get_next_action(self, **scenario_params: object) -> ArticulationAction:
        """Computes the next articulation action for the current scenario.

        Updates the end effector visualization if active, advances the scenario state, and generates the
        appropriate control action based on the current target position and orientation.

        Args:
            **scenario_params: Parameters specific to the active scenario. For "Sinusoidal Target" scenario,
                expects w_z, w_xy, rad_z, rad_xy, and height parameters.

        Returns:
            The articulation action for the next timestep. Returns an empty action if no controller is active.
        """
        if self._ee_frame_prim is not None:
            position, orientation = self.art_ik.compute_end_effector_pose()
            orientation = rot_matrices_to_quats(orientation)
            self._ee_frame_prim.set_world_pose(position, orientation)

        if self._controller is None:
            return ArticulationAction()

        self.update_scenario(**scenario_params)

        if self._target is not None:
            position, orientation = self._target.get_local_pose()
            if not self.use_orientation:
                orientation = None
            return self._controller.forward(position, orientation)
        else:
            return self._controller.forward(np.empty((3,)), None)
