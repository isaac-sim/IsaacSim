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

"""H1 humanoid robot policy examples for flat terrain locomotion."""

import isaacsim.core.experimental.utils.transform as transform_utils
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path


class H1FlatTerrainPolicy(PolicyController):
    """The H1 Humanoid running Flat Terrain Policy Locomotion Policy.

    Args:
        prim_path: The prim path of the robot on the stage.
        root_path: The path to the articulation root of the robot.
        usd_path: The robot usd filepath in the directory.
        position: The position of the robot.
        orientation: The orientation of the robot.
        policy_path: Path to the policy file. If None, auto-detected from active engine.
        env_config_path: Path to the environment config file. If None, auto-detected from active engine.
    """

    def __init__(
        self,
        prim_path: str,
        root_path: str | None = None,
        usd_path: str | None = None,
        position: list[float] | None = None,
        orientation: list[float] | None = None,
        policy_path: str | None = None,
        env_config_path: str | None = None,
    ) -> None:
        assets_root_path = get_assets_root_path()
        policy_dir = assets_root_path + "/Isaac/Samples/Policies/h1"
        if usd_path is None:
            is_newton = SimulationManager.get_active_physics_engine() == "newton"
            if is_newton:
                usd_path = assets_root_path + "/Isaac/Samples/Mujoco_Menagerie/unitree_h1/h1/h1.usda"
            else:
                usd_path = assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd"

        super().__init__(prim_path, root_path, usd_path, position, orientation)

        if policy_path is None or env_config_path is None:
            active_engine = SimulationManager.get_active_physics_engine()
            is_newton = active_engine == "newton"
            if policy_path is None:
                policy_path = f"{policy_dir}/newton_policy.pt" if is_newton else f"{policy_dir}/physx_policy.pt"
            if env_config_path is None:
                env_config_path = f"{policy_dir}/newton_env.yaml" if is_newton else f"{policy_dir}/physx_env.yaml"

        self.load_policy(policy_path, env_config_path)
        self._action_scale = self.policy_env_params.get("action_scale", 0.5)
        self._previous_action = None
        self._current_action = None
        self._policy_counter = 0

    def _compute_observation(self, command: object) -> object:
        """Compute the observation vector for the policy.

        Args:
            command: The robot command velocities (v_x, v_y, w_z) in m/s and rad/s

        Returns:
            object: Observation vector of shape [12 + 3*num_dofs] containing base lin/ang velocity,
            gravity direction, commands, joint position error, joint velocities, and previous action.
        """
        torch = import_module("torch")

        lin_vel_I, ang_vel_I = self.robot.get_velocities()
        pos_IB, q_IB = self.robot.get_world_poses()

        R_IB = wp.to_torch(transform_utils.quaternion_to_rotation_matrix(q_IB))
        R_BI = R_IB.squeeze().t()
        lin_vel_b = R_BI @ wp.to_torch(lin_vel_I).t()
        ang_vel_b = R_BI @ wp.to_torch(ang_vel_I).t()
        gravity_b = R_BI @ torch.tensor([0.0, 0.0, -1.0], device=torch.device(str(self.robot._device)))

        device = torch.device(str(self.robot._device))
        num_dofs = len(self.robot.dof_names)
        obs_dim = 12 + num_dofs * 3
        obs = torch.zeros(obs_dim, device=device)
        # Base lin vel
        obs[:3] = lin_vel_b.squeeze()
        # Base ang vel
        obs[3:6] = ang_vel_b.squeeze()
        # Gravity
        obs[6:9] = gravity_b.squeeze()
        # Command
        obs[9:12] = command
        # Joint states
        current_joint_pos = wp.to_torch(self.robot.get_dof_positions())
        current_joint_vel = wp.to_torch(self.robot.get_dof_velocities())
        obs[12 : 12 + num_dofs] = current_joint_pos - self.default_pos
        obs[12 + num_dofs : 12 + 2 * num_dofs] = current_joint_vel - self.default_vel
        # Previous Action
        if self._previous_action is not None:
            obs[12 + 2 * num_dofs : 12 + 3 * num_dofs] = self._previous_action
        return obs

    def forward(self, dt: float, command: object) -> None:
        """Compute the desired joint positions and apply them to the robot.

        Policy runs at decimated rate, but control commands are applied every physics step.
        Actions are scaled and added to default positions to generate target poses.

        Args:
            dt: Timestep update in the world in seconds
            command: The robot command velocities (v_x, v_y, w_z) in m/s and rad/s
        """
        torch = import_module("torch")
        device = torch.device(str(self.robot._device))

        # Initialize action tensors on first call
        if self._previous_action is None:
            num_dofs = len(self.robot.dof_names)
            self._previous_action = torch.zeros(num_dofs, device=device)
            self._current_action = torch.zeros(num_dofs, device=device)

        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self._current_action = self._compute_action(obs)
            self._previous_action = self._current_action.clone()

            target_pos = self.default_pos + (self._current_action * self._action_scale)
            self.robot.set_dof_position_targets(positions=wp.from_torch(target_pos))

        self._policy_counter += 1

    def initialize(self) -> None:
        """Overloads the default initialize function to use default articulation root properties in the USD."""
        self._policy_counter = 0
        super().initialize(set_articulation_props=False)
