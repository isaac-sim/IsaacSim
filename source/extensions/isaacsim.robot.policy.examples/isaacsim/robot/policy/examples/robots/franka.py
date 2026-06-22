# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Example implementations of policies for Franka robots."""

import isaacsim.core.experimental.utils.transform as transform_utils
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.experimental.utils.backend import use_backend

# from isaacsim.core.utils.transformations import get_world_pose_from_relative
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path


class FrankaOpenDrawerPolicy(PolicyController):
    """The Franka Open Drawer Policy. In this policy, the robot will open the top drawer of the cabinet and hold it open.

    Args:
        prim_path: The prim path of the robot on the stage.
        cabinet: The cabinet articulation.
        root_path: The path to the articulation root of the robot.
        usd_path: The robot usd filepath in the directory.
        position: The position of the robot.
        orientation: The orientation of the robot.
    """

    def __init__(
        self,
        prim_path: str,
        cabinet: Articulation,
        root_path: str | None = None,
        usd_path: str | None = None,
        position: list[float] | None = None,
        orientation: list[float] | None = None,
    ) -> None:

        assets_root_path = get_assets_root_path()

        policy_path = assets_root_path + "/Isaac/Samples/Policies/Franka_Policies/Open_Drawer_Policy/"
        if usd_path is None:
            usd_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        super().__init__(prim_path, root_path, usd_path, position, orientation)

        self.load_policy(
            policy_path + "policy.pt",
            policy_path + "env.yaml",
        )

        torch = import_module("torch")
        self._previous_action = torch.zeros(8)
        self._action_scale = 1.0
        self._indices = wp.array([0, 1, 2, 3, 4, 5, 6, 7], dtype=wp.int32)  # index 8 is a mimic joint
        self._policy_counter = 0

        self.cabinet = cabinet

        self.franka_hand_prim = XformPrim(self.robot.link_paths[0][8])
        self.drawer_handle_top_prim = XformPrim(self.cabinet.link_paths[0][6])
        print(self.franka_hand_prim.paths)
        print(self.drawer_handle_top_prim.paths)

    def _compute_observation(self) -> object:
        """Compute the observation vector for the policy.

        The observation includes robot joint states, drawer state, relative positioning
        between end-effector and drawer handle (with their respective offsets), and previous actions.
        End-effector offset is [0, 0, 0.1034]m and drawer handle offset is [0.305, 0, 0.01]m.

        Returns:
            A 31-dimensional observation vector containing:
            - [0:9]: Robot joint positions (difference from default pose)
            - [9:18]: Robot joint velocities (difference from default velocities)
            - [18:19]: Drawer joint position
            - [19:20]: Drawer joint velocity
            - [20:23]: Relative position between drawer handle and robot end-effector
            - [23:31]: Previous action
        """
        # relative transform from the drawer handle to the drawer handle link
        """
        From env.yaml
        - prim_path: /World/envs/env_.*/Cabinet/drawer_handle_top
            name: drawer_handle_top
            offset:
                pos: !!python/tuple
                - 0.305
                - 0.0
                - 0.01
        """
        """
        From env.yaml
        - prim_path: /World/envs/env_.*/Robot/panda_hand
            name: ee_tcp
            offset:
                pos: !!python/tuple
                - 0.0
                - 0.0
                - 0.1034
        """
        torch = import_module("torch")
        with use_backend("usdrt"):
            # Get world poses of the links
            drawer_world_pos, drawer_world_orient = self.drawer_handle_top_prim.get_world_poses()
            robot_world_pos, robot_world_orient = self.franka_hand_prim.get_world_poses()

            # Get rotation matrices directly from quaternions using warp function
            R_drawer = transform_utils.quaternion_to_rotation_matrix(drawer_world_orient).reshape((3, 3))
            R_robot = transform_utils.quaternion_to_rotation_matrix(robot_world_orient).reshape((3, 3))

            # Reshape positions from (1,3) to (3,)
            drawer_world_pos = drawer_world_pos.reshape((3,))
            robot_world_pos = robot_world_pos.reshape((3,))

            # Create offset vectors as warp arrays on same device
            drawer_offset = wp.array([0.305, 0.0, 0.01], dtype=wp.float32, device=drawer_world_pos.device)
            robot_offset = wp.array([0.0, 0.0, 0.1034], dtype=wp.float32, device=robot_world_pos.device)

            # Convert to torch for matrix multiplication
            drawer_world_pos_torch = wp.to_torch(drawer_world_pos)
            robot_world_pos_torch = wp.to_torch(robot_world_pos)
            R_drawer_torch = wp.to_torch(R_drawer)
            R_robot_torch = wp.to_torch(R_robot)
            drawer_offset_torch = wp.to_torch(drawer_offset)
            robot_offset_torch = wp.to_torch(robot_offset)

            # Apply offsets in world frame using torch matmul
            drawer_world_pos = drawer_world_pos_torch + torch.matmul(R_drawer_torch, drawer_offset_torch)
            robot_world_pos = robot_world_pos_torch + torch.matmul(R_robot_torch, robot_offset_torch)

        obs = torch.zeros(31, device=torch.device(str(self.robot._device)))
        # Base lin pos
        obs[:9] = wp.to_torch(self.robot.get_dof_positions()) - self.default_pos

        # Base ang vel
        obs[9:18] = wp.to_torch(self.robot.get_dof_velocities()) - self.default_vel

        # Joint states
        obs[18:19] = wp.to_torch(self.cabinet.get_dof_positions(indices=[0], dof_indices=self.drawer_link_idx))
        obs[19:20] = wp.to_torch(self.cabinet.get_dof_velocities(indices=[0], dof_indices=self.drawer_link_idx))

        # relative distance between drawer and robot
        obs[20:23] = drawer_world_pos - robot_world_pos

        # Previous Action
        obs[23:31] = self._previous_action

        return obs

    def forward(self, dt: float) -> None:
        """Computes and applies joint position targets for the Franka arm to execute the drawer opening task.

        The control runs at a decimated rate and applies position control to the first 8 joints
        (excluding the mimic joint). Actions are scaled and added to the default pose.

        Args:
            dt: Physics timestep in seconds
        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation()
            self.action = self._compute_action(obs)
            self._previous_action = self.action.clone()

        self.robot.set_dof_position_targets(
            positions=wp.from_torch(self.default_pos[:8] + (self.action * self._action_scale)),
            dof_indices=self._indices,
        )

        self._policy_counter += 1

    def initialize(self) -> None:
        """Initializes the Franka arm articulation with position control mode and configures solver parameters.

        Sets up drawer link indices and specific physics solver settings for stable manipulation.
        """
        super().initialize(control_mode="position", set_articulation_props=False)

        self.drawer_link_idx = self.cabinet.get_dof_indices("drawer_top_joint")

        self.robot.set_solver_iteration_counts(position_counts=[32], velocity_counts=[4])
        self.robot.set_stabilization_thresholds([0])
        self.robot.set_sleep_thresholds([0])
