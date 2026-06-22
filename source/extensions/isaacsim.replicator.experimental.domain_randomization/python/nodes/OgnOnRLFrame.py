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

"""Trigger domain randomization from explicit RL frame steps."""

from typing import Any

import numpy as np
import omni.graph.core as og
from isaacsim.replicator.experimental.domain_randomization.scripts import context


class OgnOnRLFrameInternalState:
    """Per-node frame counters, one counter per RL environment."""

    def __init__(self) -> Any:
        self.frame_count = None


class OgnOnRLFrame:
    """OmniGraph source node for RL-domain-randomization execution."""

    @staticmethod
    def internal_state() -> Any:
        """Create the internal state for this node instance.

        Returns:
            A new OgnOnRLFrameInternalState instance for tracking frame counts.
        """
        return OgnOnRLFrameInternalState()

    @staticmethod
    def compute(db: Any) -> bool:
        """Advance frame counters only when the randomization context fires.

        Args:
            db: OmniGraph database interface with ``inputs:num_envs`` and
                ``outputs:execOut``, ``outputs:resetInds``, and
                ``outputs:frameNum``.

        Returns:
            True after either disabling execution for an idle frame or emitting
            the current frame counts for an active randomization step.
        """
        ctx = context.resolve_context()

        if not ctx or not ctx.trigger:
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return True

        ctx.trigger = False
        state = db.per_instance_state
        reset_inds = ctx.reset_inds

        if state.frame_count is None:
            state.frame_count = np.zeros(db.inputs.num_envs)

        if reset_inds is not None and len(reset_inds) > 0:
            state.frame_count[reset_inds] = 0
            db.outputs.resetInds = reset_inds
        else:
            db.outputs.resetInds = []

        db.outputs.frameNum = state.frame_count
        state.frame_count += 1

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True
