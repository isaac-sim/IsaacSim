# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""Template Python implementation for an Isaac Sim IPC OmniGraph node."""

# TEMPLATE-START
import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode


class OgnMyIpcNodePyState(BaseResetNode):
    """Per-instance state for the template IPC node."""

    def __init__(self) -> None:
        # Declare all attributes BEFORE calling super().__init__,
        # because BaseResetNode.__init__ calls custom_reset() immediately.
        self.handle = None  # replace with your transport handle
        self.uri = ""
        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Reset transport state when timeline or inputs change."""
        # Called on timeline stop and when inputs change.
        if self.handle is not None:
            self.handle.close()
            self.handle = None
        self.uri = ""


class OgnMyIpcNodePy:
    """Template OmniGraph node for custom IPC transports."""

    @staticmethod
    def internal_state() -> OgnMyIpcNodePyState:
        """Create per-instance state for the node.

        Returns:
            Per-instance node state.
        """
        return OgnMyIpcNodePyState()

    @staticmethod
    def compute(db: object) -> bool:
        """Evaluate one non-blocking IPC transfer step.

        Args:
            db: OmniGraph database object for the current node evaluation.

        Returns:
            True if the transfer completed successfully, otherwise False.
        """
        state = db.per_instance_state

        uri = db.inputs.uri
        if state.handle is not None and state.uri != uri:
            state.custom_reset()

        if state.handle is None:
            # Open transport from inputs (e.g. URI, config).
            # state.handle = open_my_transport(uri)
            # state.uri = uri
            # Fire execOut even on failure so downstream nodes keep running.
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return False

        # Non-blocking send or try-receive; write db.outputs on success.
        # See Performance Considerations for time-budget guidance.
        success = False  # replace with actual transfer

        # For send nodes: fire execOut every tick.
        # For receive nodes: fire execOut only when a full message arrives.
        if success:
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return success


# TEMPLATE-END
