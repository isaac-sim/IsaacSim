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

"""Provides a policy controller for ANYmal quadruped robot executing flat terrain locomotion using an LSTM-based SEA network."""

import io

import omni
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.utils.transform import quaternion_to_rotation_matrix
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path


class AnymalFlatTerrainPolicy(PolicyController):
    """Policy controller for the ANYmal quadruped robot executing flat terrain locomotion.

    This controller implements a learned policy for stable walking on flat terrain,
    handling velocity commands for forward/backward motion, lateral motion, and turning.
    Uses an LSTM-based Series Elastic Actuator (SEA) network for torque control.

    Args:
        prim_path: The prim path of the robot on the stage.
        root_path: The path to the articulation root of the robot.
        usd_path: The robot usd filepath in the directory.
        position: The position of the robot.
        orientation: The orientation of the robot.
        policy_path: Path to the policy file. If None, uses default policy.
        env_config_path: Path to the environment config file. If None, uses default config.
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
        if usd_path is None:
            is_newton = SimulationManager.get_active_physics_engine() == "newton"
            if is_newton:
                usd_path = (
                    assets_root_path + "/Isaac/Samples/Mujoco_Menagerie/anybotics_anymal_c/anymal_c/anymal_c.usda"
                )
            else:
                usd_path = assets_root_path + "/Isaac/Robots/ANYbotics/anymal_c/anymal_c.usd"

        super().__init__(prim_path, root_path, usd_path, position, orientation)

        if policy_path is None:
            policy_path = assets_root_path + "/Isaac/Samples/Policies/Anymal_Policies/anymal_policy.pt"
        if env_config_path is None:
            env_config_path = assets_root_path + "/Isaac/Samples/Policies/Anymal_Policies/anymal_env.yaml"

        torch = import_module("torch")
        self._previous_action = torch.zeros(12)
        self._action_scale = 0.5
        self._policy_counter = 0

        self.load_policy(policy_path, env_config_path)

    def _compute_observation(self, command: object) -> object:
        """Compute the observation vector for the policy.

        The observation includes base linear/angular velocities, gravity direction,
        command velocities, joint positions/velocities, and previous actions.

        Args:
            command: The robot command velocities (v_x, v_y, w_z) in m/s and rad/s

        Returns:
            object: A 48-dimensional observation vector containing:
            - [0:3]: Base linear velocity in body frame
            - [3:6]: Base angular velocity in body frame
            - [6:9]: Gravity direction in body frame
            - [9:12]: Command velocities
            - [12:24]: Joint position error from default
            - [24:36]: Joint velocities
            - [36:48]: Previous action
        """
        torch = import_module("torch")

        lin_vel_I, ang_vel_I = self.robot.get_velocities()
        pos_IB, q_IB = self.robot.get_world_poses()

        R_IB = wp.to_torch(quaternion_to_rotation_matrix(q_IB))
        R_BI = R_IB.squeeze().t()
        lin_vel_b = R_BI @ wp.to_torch(lin_vel_I).t()
        ang_vel_b = R_BI @ wp.to_torch(ang_vel_I).t()
        gravity_b = R_BI @ torch.tensor([0.0, 0.0, -1.0], device=torch.device(str(self.robot._device)))

        obs = torch.zeros(48, device=torch.device(str(self.robot._device)))
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
        obs[12:24] = current_joint_pos - self.default_pos
        obs[24:36] = current_joint_vel - self.default_vel
        # Previous Action
        obs[36:48] = self._previous_action
        return obs

    def forward(self, dt: float, command: object) -> None:
        """Computes and applies joint torques for ANYmal locomotion based on the policy output.

        The control runs at a decimated rate and uses an actuator network to convert
        policy actions into joint torques. Joint order is:
        FL (hip, thigh, calf) -> FR -> RL -> RR.

        Args:
            dt: Physics timestep in seconds
            command: Robot command as linear and angular velocities (v_x, v_y, w_z)
        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.clone()

        # The learning controller uses the order of
        # FL_hip_joint FL_thigh_joint FL_calf_joint
        # FR_hip_joint FR_thigh_joint FR_calf_joint
        # RL_hip_joint RL_thigh_joint RL_calf_joint
        # RR_hip_joint RR_thigh_joint RR_calf_joint
        current_joint_pos = wp.to_torch(self.robot.get_dof_positions())
        current_joint_vel = wp.to_torch(self.robot.get_dof_velocities())

        joint_torques, _ = self._actuator_network.compute_torques(
            current_joint_pos, current_joint_vel, self._action_scale * self.action
        )
        self.robot.set_dof_efforts(wp.from_torch(joint_torques))
        self._policy_counter += 1

    def initialize(self) -> None:
        """Initialize the articulation interface and set up drive mode."""
        super().initialize(control_mode="effort")

        import warnings

        # defer the `LstmSeaNetwork` import to avoid loading torch at startup
        from isaacsim.robot.policy.examples.utils import LstmSeaNetwork

        # Suppress expected user warning from the actuation network, this is a limitation with the trained policy
        warnings.filterwarnings(
            "ignore",
            message="RNN module weights are not part of single contiguous chunk of memory.*",
            category=UserWarning,
        )

        # Actuator network
        assets_root_path = get_assets_root_path()
        file_content = omni.client.read_file(
            assets_root_path + "/Isaac/IsaacLab/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt"
        )[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        self._actuator_network = LstmSeaNetwork()
        self._actuator_network.setup(file, self.default_pos)
        self._actuator_network.reset()
