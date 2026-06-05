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

"""Module for triggering domain randomization events in Isaac Sim."""

from typing import Any

from omni.replicator.core.utils import ReplicatorWrapper, create_node

from .context import initialize_context


@ReplicatorWrapper
def on_rl_frame(num_envs: int) -> Any:
    """Creates a node that triggers domain randomization on reinforcement learning frames.

    This function sets up domain randomization to occur at specific intervals during RL training,
    allowing for environment variations across multiple parallel environments.

    Args:
        num_envs: The number of environments corresponding to the number of prims
            encapsulated in the RigidPrimViews and ArticulationViews.

    Returns:
        The created OgnOnRLFrame node for domain randomization.
    """
    node = create_node("isaacsim.replicator.domain_randomization.OgnOnRLFrame")
    node.get_attribute("inputs:num_envs").set(num_envs)

    initialize_context(num_envs, node)

    return node
