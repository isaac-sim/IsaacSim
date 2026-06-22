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

"""Filter randomization execution by per-environment frame counts or reset indices."""

from typing import Any

import numpy as np
import omni.graph.core as og


class OgnIntervalFiltering:
    """OmniGraph gate that selects which environments should be randomized."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Emit selected indices and reset state for downstream writer nodes.

        With ``ignoreInterval`` disabled, ``inputs:frameCounts`` is filtered to
        environments whose count is positive and divisible by ``inputs:interval``.
        With ``ignoreInterval`` enabled, ``inputs:indices`` is passed through and
        ``outputs:on_reset`` is set so writers update their reset baselines.
        ``outputs:execOut`` remains enabled when no indices are selected.

        Args:
            db: Database object containing node inputs and outputs.

        Returns:
            True when selected indices are emitted, False when no indices are selected.
        """
        interval = db.inputs.interval
        frame_num = np.array(db.inputs.frameCounts)
        indices = np.array(db.inputs.indices)
        ignore_interval = db.inputs.ignoreInterval

        if (not ignore_interval and (interval is None or len(frame_num) == 0)) or (
            ignore_interval and len(indices) == 0
        ):
            db.outputs.indices = []
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return False

        if ignore_interval:
            output_inds = indices
            db.outputs.on_reset = True
        else:
            # Keep tuple from np.nonzero(); OmniGraph unwraps it for int[] outputs
            output_inds = np.nonzero(np.logical_and(frame_num % interval == 0, frame_num > 0))
            db.outputs.on_reset = False

        if len(output_inds) == 0:
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        else:
            db.outputs.indices = output_inds
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True
