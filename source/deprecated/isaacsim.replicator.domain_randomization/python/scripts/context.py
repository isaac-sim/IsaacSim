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

"""Context management for Isaac Sim domain randomization using Replicator."""

from typing import Any

import omni.graph.core as og
from omni.replicator.core.utils import utils

_context = None


class ReplicatorIsaacContext:
    """Context manager for Isaac Sim domain randomization using Replicator.

    This class manages the execution context for domain randomization in Isaac Sim environments.
    It coordinates randomization triggers across multiple environments and maintains the state
    of which environments need to be reset. The context integrates with Omni Graph to execute
    randomization through action graphs and provides tendon execution context management for
    advanced randomization scenarios.

    Args:
        num_envs: Number of simulation environments to manage.
        action_graph_entry_node: Entry node for the action graph that handles randomization execution.
    """

    def __init__(self, num_envs: Any, action_graph_entry_node: Any) -> None:
        self._num_envs = num_envs
        self._action_graph_entry_node = action_graph_entry_node
        self._reset_inds = None
        self.trigger = False

        controller = og.Controller()
        self._graph = controller.graph(utils.GRAPH_PATH)
        self._tendon_attribute_stack = [None]

    def trigger_randomization(self, reset_inds: Any) -> None:
        """Trigger randomization for specified environment indices.

        Args:
            reset_inds: Indices of environments to reset and randomize.
        """
        self.trigger = True
        self._reset_inds = reset_inds
        self._action_graph_entry_node.request_compute()
        self._graph.evaluate()

    @property
    def reset_inds(self) -> list[int]:
        """Indices of environments that were reset and randomized.

        Returns:
            The environment indices.
        """
        return self._reset_inds

    def get_tendon_exec_context(self) -> Any:
        """Get the current tendon execution context.

        Returns:
            The current tendon node from the attribute stack.
        """
        return self._tendon_attribute_stack[-1]

    def add_tendon_exec_context(self, node: Any) -> None:
        """Add a tendon execution context to the attribute stack.

        Args:
            node: The tendon node to add to the execution context stack.
        """
        self._tendon_attribute_stack.append(node)


def initialize_context(num_envs: Any, action_graph_entry_node: Any) -> None:
    """Initialize the Replicator Isaac context for domain randomization.

    Args:
        num_envs: Number of environments to manage.
        action_graph_entry_node: The action graph entry node for triggering graph evaluation.
    """
    global _context
    _context = ReplicatorIsaacContext(num_envs, action_graph_entry_node)


def get_reset_inds() -> list[int]:
    """Get the indices of environments that need to be reset.

    Returns:
        The environment indices that were marked for reset during the last randomization trigger.
    """
    return _context.reset_inds


def trigger_randomization(reset_inds: Any) -> None:
    """Trigger domain randomization for specified environments.

    Args:
        reset_inds: Indices of environments that should undergo randomization.
    """
    _context.trigger_randomization(reset_inds)
