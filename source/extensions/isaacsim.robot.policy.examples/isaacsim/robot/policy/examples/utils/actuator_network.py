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

"""Provides neural network implementations for Series Elastic Actuator (SEA) torque computation in robotic control."""

import numpy as np
from isaacsim.core.deprecation_manager import import_module
from numpy import genfromtxt

torch = import_module("torch")


class LstmSeaNetwork:
    """Implements an SEA network with LSTM hidden layers.

    Initializes the network with zeroed hidden and cell states for the LSTM.
    """

    def __init__(self):

        # define the network
        self._network = None
        self._hidden_state = torch.zeros((2, 12, 8), requires_grad=False)
        self._cell_state = torch.zeros((2, 12, 8), requires_grad=False)
        # default joint position
        self._default_joint_pos = None

    def get_hidden_state(self) -> torch.Tensor:
        """Get the current hidden state of the LSTM.

        Returns:
            The hidden state tensor
        """
        if self._hidden_state is None:
            return torch.zeros((12, 8))
        else:
            return self._hidden_state[1].detach()

    def setup(self, path_or_buffer: object, default_joint_pos: torch.Tensor):
        """Set up the network by loading weights and configuring default positions.

        Args:
            path_or_buffer: Path to the JIT model file or file buffer
            default_joint_pos: Default joint positions tensor
        """
        # load the network from JIT file
        self._network = torch.jit.load(path_or_buffer)
        # set the default joint position
        self._default_joint_pos = default_joint_pos
        self._network = self._network.to(default_joint_pos.device)
        self._hidden_state = self._hidden_state.to(default_joint_pos.device)
        self._cell_state = self._cell_state.to(default_joint_pos.device)

    def reset(self):
        """Reset the LSTM hidden and cell states to zeros."""
        with torch.no_grad():
            self._hidden_state[:, :, :] = 0.0
            self._cell_state[:, :, :] = 0.0

    @torch.no_grad()
    def compute_torques(
        self, joint_pos: object, joint_vel: object, actions: object
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Compute joint torques based on current joint states and desired actions.

        Args:
            joint_pos: Current joint positions
            joint_vel: Current joint velocities
            actions: Desired joint actions

        Returns:
            tuple[torch.Tensor, torch.Tensor]: Tuple of (computed torques, hidden state)
        """
        # create sea network input obs
        actions = actions.clone()
        actuator_net_input = torch.zeros((12, 1, 2), device=actions.device)
        actuator_net_input[:, 0, 0] = actions + self._default_joint_pos - joint_pos
        actuator_net_input[:, 0, 1] = torch.clip(joint_vel, -20.0, 20.0)
        # call the network
        torques, (self._hidden_state, self._cell_state) = self._network(
            actuator_net_input, (self._hidden_state, self._cell_state)
        )
        # return the torque to apply with clipping along with hidden state
        return torques.detach().clip(-80.0, 80.0), self._hidden_state[1]


class SeaNetwork(torch.nn.Module):
    """Implements a SEA network with MLP hidden layers.

    Initializes the MLP SEA network with 6-32-32-1 architecture.
    """

    def __init__(self):

        super().__init__()
        # define layer architecture
        self._sea_network = torch.nn.Sequential(
            torch.nn.Linear(6, 32),
            torch.nn.Softsign(),
            torch.nn.Linear(32, 32),
            torch.nn.Softsign(),
            torch.nn.Linear(32, 1),
        )
        # define the delays
        self._num_delays = 2
        self._delays = [8, 3]
        # define joint histories
        self._history_size = self._delays[0]
        self._joint_pos_history = torch.zeros((12, self._history_size + 1))
        self._joint_vel_history = torch.zeros((12, self._history_size + 1))
        # define scaling for the actuator network
        self._sea_vel_scale = 0.4
        self._sea_pos_scale = 3.0
        self._sea_output_scale = 20.0
        self._action_scale = 0.5
        # default joint position
        self._default_joint_pos = None

    """
    Operations
    """

    def setup(self, weights_path: str, default_joint_pos: torch.Tensor):
        """Initialize the SEA network by loading weights and setting default joint positions.

        Args:
            weights_path: Path to the CSV file containing network weights.
            default_joint_pos: Default joint positions tensor.
        """
        # load the weights into network
        self._load_weights(weights_path)
        # set the default joint position
        self._default_joint_pos = default_joint_pos

    def reset(self):
        """Reset the joint position and velocity histories to zeros."""
        self._joint_pos_history.fill(0.0)
        self._joint_vel_history.fill(0.0)

    def compute_torques(self, joint_pos: object, joint_vel: object, actions: object) -> torch.Tensor:
        """Compute joint torques based on current joint states and desired actions.

        Args:
            joint_pos: Current joint positions.
            joint_vel: Current joint velocities.
            actions: Desired joint actions.

        Returns:
            Computed torques scaled by the output scale factor.
        """
        self._update_joint_history(joint_pos, joint_vel, actions)
        return self._compute_sea_torque()

    """
    Internal helpers.
    """

    def _load_weights(self, weights_path: str):
        """Load network weights from a CSV file and assign them to the MLP layers.

        Args:
            weights_path: Path to the CSV file containing network weights.
        """
        # load the data as torch tensor
        data = torch.from_numpy(genfromtxt(weights_path, delimiter=",", dtype=np.float32))
        # manually defines the number of neurons in MLP
        expected_num_params = 6 * 32 + 32 + 32 * 32 + 32 + 32 * 1 + 1
        assert data.numel() == expected_num_params
        # assign neuron weights to each linear layer
        idx = 0
        for layer in self._sea_network:
            if not isinstance(layer, torch.nn.Softsign):
                # layer weights
                weight = data[idx : idx + layer.in_features * layer.out_features]
                weight = weight.view(layer.out_features, layer.in_features).t()
                layer.weight = torch.nn.Parameter(weight)
                idx += layer.out_features * layer.in_features
                # layer biases
                bias = data[idx : idx + layer.out_features]
                layer.bias = torch.nn.Parameter(bias)
                idx += layer.out_features
        # set the module in eval mode
        self.eval()

    def _update_joint_history(self, joint_pos: object, joint_vel: object, actions: object) -> None:
        """Update the joint position and velocity history buffers with current values.

        Args:
            joint_pos: Current joint positions.
            joint_vel: Current joint velocities.
            actions: Desired joint actions.
        """
        joint_pos = torch.as_tensor(joint_pos)
        joint_vel = torch.as_tensor(joint_vel)
        # compute error in position
        joint_pos_error = self._action_scale * actions + self._default_joint_pos - joint_pos
        # store into history
        self._joint_pos_history[:, : self._history_size] = self._joint_pos_history[:, 1:]
        self._joint_vel_history[:, : self._history_size] = self._joint_vel_history[:, 1:]
        self._joint_pos_history[:, self._history_size] = joint_pos_error
        self._joint_vel_history[:, self._history_size] = joint_vel

    def _compute_sea_torque(self) -> torch.Tensor:
        """Compute SEA torques using the MLP network and historical joint data.

        Returns:
            Computed torques from the SEA network.
        """
        inp = torch.zeros((12, 6))
        for dof in range(12):
            inp[dof, 0] = self._sea_vel_scale * self._joint_vel_history[dof, self._history_size - self._delays[0]]
            inp[dof, 1] = self._sea_vel_scale * self._joint_vel_history[dof, self._history_size - self._delays[1]]
            inp[dof, 2] = self._sea_vel_scale * self._joint_vel_history[dof, self._history_size]
            inp[dof, 3] = self._sea_pos_scale * self._joint_pos_history[dof, self._history_size - self._delays[0]]
            inp[dof, 4] = self._sea_pos_scale * self._joint_pos_history[dof, self._history_size - self._delays[1]]
            inp[dof, 5] = self._sea_pos_scale * self._joint_pos_history[dof, self._history_size]
        return self._sea_output_scale * self._sea_network(inp)


# EOF
