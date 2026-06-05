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

"""Tutorial module demonstrating kinematics-based robot control for the Franka Panda robot using inverse kinematics computations."""

from __future__ import annotations

import os

import carb
import numpy as np
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from isaacsim.storage.native import get_assets_root_path


class FrankaKinematicsExample:
    """A tutorial example demonstrating kinematics-based robot control for the Franka Panda robot.

    This class showcases how to use the Lula kinematics solver and ArticulationKinematicsSolver to perform
    inverse kinematics computations for robot motion control. The example loads a Franka Panda robot and a
    target object, then continuously computes joint actions to move the robot's end effector toward the target
    position using inverse kinematics.

    The example demonstrates:
    - Loading robot assets and target objects into the simulation
    - Setting up kinematics solvers using robot description files and URDF
    - Computing inverse kinematics solutions in real-time
    - Applying computed joint actions to control the robot
    - Handling cases where inverse kinematics fails to converge

    The kinematics solver uses Lula's robot description and URDF files to understand the robot's kinematic
    structure and constraints. The ArticulationKinematicsSolver provides a high-level interface that bridges
    between the USD articulation and the underlying kinematics computation.
    """

    def __init__(self) -> None:
        self._kinematics_solver = None
        self._articulation_kinematics_solver = None

        self._articulation = None
        self._target = None

    def load_example_assets(self) -> tuple:
        """Loads the Franka robot and target assets into the simulation stage.

        Adds a Franka Panda robot at `/panda` and a target frame at `/World/target` to the stage.
        The target is scaled down and positioned at [0.3, 0, 0.5] with appropriate orientation.

        Returns:
            A tuple containing the articulation and target XForm prim for stage registration.
        """
        # Add the Franka and target to the stage

        robot_prim_path = "/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)

        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = XFormPrim("/World/target", scale=[0.04, 0.04, 0.04])
        self._target.set_default_state(np.array([0.3, 0, 0.5]), euler_angles_to_quats([0, np.pi, 0]))

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._articulation, self._target

    def setup(self) -> None:
        """Initializes the kinematics solvers for the Franka robot.

        Loads the robot description and URDF files, creates a LulaKinematicsSolver,
        and sets up an ArticulationKinematicsSolver for the right gripper end effector.
        """
        # Load a URDF and Lula Robot Description File for this robot:
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        kinematics_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=kinematics_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=kinematics_config_dir + "/franka/lula_franka_gen.urdf",
        )

        # Kinematics for supported robots can be loaded with a simpler equivalent
        # print("Supported Robots with a Lula Kinematics Config:", interface_config_loader.get_supported_robots_with_lula_kinematics())
        # kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
        # self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)

        print("Valid frame names at which to compute kinematics:", self._kinematics_solver.get_all_frame_names())

        end_effector_name = "right_gripper"
        self._articulation_kinematics_solver = ArticulationKinematicsSolver(
            self._articulation, self._kinematics_solver, end_effector_name
        )

    def update(self, step: float) -> None:
        """Updates the robot motion to track the target position.

        Computes inverse kinematics to move the robot's end effector to the target pose.
        Tracks any movements of the robot base and applies the computed joint actions.

        Args:
            step: Time step for the update cycle.
        """
        target_position, target_orientation = self._target.get_world_pose()

        # Track any movements of the robot base
        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()
        self._kinematics_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        action, success = self._articulation_kinematics_solver.compute_inverse_kinematics(
            target_position, target_orientation
        )

        if success:
            self._articulation.apply_action(action)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken")

        # Unused Forward Kinematics:
        # ee_position,ee_rot_mat = articulation_kinematics_solver.compute_end_effector_pose()

    def reset(self) -> None:
        """Resets the kinematics example.

        Since kinematics is stateless, this method performs no operations.
        """
        # Kinematics is stateless
