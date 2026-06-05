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

"""Tutorial example module demonstrating RMPflow motion generation with a Franka Panda robot for collision-aware end-effector target tracking."""

from __future__ import annotations

import numpy as np
from isaacsim.core.api.objects.cuboid import FixedCuboid
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation import ArticulationMotionPolicy, RmpFlow
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    get_supported_robot_policy_pairs,
    load_supported_motion_policy_config,
)
from isaacsim.storage.native import get_assets_root_path


class FrankaRmpFlowExample:
    """Tutorial example demonstrating RMPflow motion generation with a Franka Panda robot.

    This class sets up a complete robotic scene with a Franka Panda robot, target object, and obstacle,
    then uses RMPflow (Riemannian Motion Policies) to generate smooth, collision-aware motion for the robot's
    end-effector to reach target positions.

    The example loads a Franka robot and configures it with RMPflow motion generation capabilities,
    demonstrating real-time obstacle avoidance and end-effector target tracking. The robot autonomously
    generates joint trajectories that navigate around obstacles while reaching desired poses.

    Key features:
    - Automatic loading of Franka robot assets and motion policy configuration
    - Real-time collision avoidance using RMPflow algorithms
    - End-effector target tracking with smooth trajectory generation
    - Support for movable obstacles and robot base pose updates
    - Debug mode with collision sphere visualization

    Typical usage involves calling load_example_assets() to set up the scene, setup() to initialize
    RMPflow, and then repeatedly calling update() in the simulation loop to generate and apply
    motion commands.
    """

    def __init__(self) -> None:
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None

        self._dbg_mode = False

    def load_example_assets(self) -> tuple:
        """Loads the Franka robot, target frame, and obstacle assets into the stage.

        Adds a Franka Panda robot at "/panda", a target frame at "/World/target", and a blue cube obstacle.
        The assets are loaded at their default positions which serve as their initial positions.

        Returns:
            A tuple containing the articulation, target, and obstacle objects for registration with the core World.
        """
        # Add the Franka and target to the stage
        # The position in which things are loaded is also the position in which they

        robot_prim_path = "/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)

        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = XFormPrim("/World/target", scale=[0.04, 0.04, 0.04])

        self._obstacle = FixedCuboid(
            "/World/obstacle", size=0.05, position=np.array([0.4, 0.0, 0.65]), color=np.array([0.0, 0.0, 1.0])
        )

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._articulation, self._target, self._obstacle

    def setup(self) -> None:
        """Initializes the RMPflow motion generation system and connects it to the Franka robot.

        Loads the RMPflow configuration for the Franka robot, creates the RmpFlow object, adds the obstacle,
        and wraps it with ArticulationMotionPolicy to connect to the robot articulation. Sets the initial
        target pose for the end effector.
        """
        # Loading RMPflow can be done quickly for supported robots
        print("Supported Robots with a Provided RMPflow Config:", list(get_supported_robot_policy_pairs().keys()))
        rmp_config = load_supported_motion_policy_config("Franka", "RMPflow")

        # Initialize an RmpFlow object
        self._rmpflow = RmpFlow(**rmp_config)
        self._rmpflow.add_obstacle(self._obstacle)

        if self._dbg_mode:
            self._rmpflow.set_ignore_state_updates(True)
            self._rmpflow.visualize_collision_spheres()

            # Set the robot gains to be deliberately poor
            bad_proportional_gains = self._articulation.get_articulation_controller().get_gains()[0] / 50
            self._articulation.get_articulation_controller().set_gains(kps=bad_proportional_gains)

        # Use the ArticulationMotionPolicy wrapper object to connect rmpflow to the Franka robot articulation.
        self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self._rmpflow)

        self._target.set_world_pose(np.array([0.5, 0, 0.7]), euler_angles_to_quats([0, np.pi, 0]))

    def update(self, step: float) -> None:
        """Updates the RMPflow system and applies motion to the robot for the current frame.

        Sets the end effector target based on the target object's position, updates the world state
        to track obstacle and robot base movements, computes the next articulation action using RMPflow,
        and applies it to the robot.

        Args:
            step: The time elapsed for this frame in seconds.
        """
        # Step is the time elapsed on this frame
        target_position, target_orientation = self._target.get_world_pose()

        self._rmpflow.set_end_effector_target(target_position, target_orientation)

        # Track any movements of the cube obstacle
        self._rmpflow.update_world()

        # Track any movements of the robot base
        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        action = self._articulation_rmpflow.get_next_articulation_action(step)
        self._articulation.apply_action(action)

    def reset(self) -> None:
        """Resets the example to its initial state.

        In debug mode, resets the RMPflow internal state and re-visualizes collision spheres.
        Resets the target object to its initial position and orientation.
        """
        # Rmpflow is stateless unless it is explicitly told not to be
        if self._dbg_mode:
            # RMPflow was set to roll out robot state internally, assuming that all returned
            # joint targets were hit exactly.
            self._rmpflow.reset()
            self._rmpflow.visualize_collision_spheres()

        self._target.set_world_pose(np.array([0.5, 0, 0.7]), euler_angles_to_quats([0, np.pi, 0]))
