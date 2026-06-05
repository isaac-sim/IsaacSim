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

"""Context management for domain randomization triggers and tendon execution in Isaac Sim replicator."""

import omni.graph.core as og
from omni.replicator.core.utils import utils

_context = None


class ReplicatorIsaacContext:
    """Context for managing domain randomization triggers and tendon execution.

    Args:
        num_envs: Number of environments to manage.
        action_graph_entry_node: Entry node for the action graph that handles randomization execution.
    """

    def __init__(self, num_envs, action_graph_entry_node):
        self._num_envs = num_envs
        self._action_graph_entry_node = action_graph_entry_node
        self._reset_inds = None
        self.trigger = False

        controller = og.Controller()
        self._graph = controller.graph(utils.GRAPH_PATH)
        self._tendon_attribute_stack = [None]

    def trigger_randomization(self, reset_inds):
        """Trigger randomization for the given reset indices.

        Args:
            reset_inds: The reset indices to trigger randomization for.
        """
        self.trigger = True
        self._reset_inds = reset_inds
        self._action_graph_entry_node.request_compute()
        self._graph.evaluate()

    @property
    def reset_inds(self):
        """Current reset indices.

        Returns:
            The reset indices.
        """
        return self._reset_inds

    def get_tendon_exec_context(self):
        """Get the current tendon execution context node.

        Returns:
            The current tendon execution context node.
        """
        return self._tendon_attribute_stack[-1]

    def add_tendon_exec_context(self, node):
        """Add a node to the tendon execution context stack.

        Args:
            node: The node to add to the tendon execution context stack.
        """
        self._tendon_attribute_stack.append(node)


def initialize_context(num_envs, action_graph_entry_node):
    """Initialize the global context for domain randomization.

    Args:
        num_envs: Number of environments to manage.
        action_graph_entry_node: Entry node for the action graph that handles randomization execution.
    """
    global _context
    _context = ReplicatorIsaacContext(num_envs, action_graph_entry_node)


def get_reset_inds():
    """Get the current reset indices from the global context.

    Returns:
        The current reset indices used for domain randomization.
    """
    return _context.reset_inds


def resolve_context():
    """Return the active context, falling back to the deprecated module's context."""
    if _context is not None:
        return _context
    try:
        from isaacsim.replicator.domain_randomization.scripts import context as dep_ctx

        return dep_ctx._context
    except ImportError:
        return None


def trigger_randomization(reset_inds):
    """Trigger randomization for the given reset indices.

    Args:
        reset_inds: Indices of environments to reset and apply randomization to.
    """
    _context.trigger_randomization(reset_inds)
