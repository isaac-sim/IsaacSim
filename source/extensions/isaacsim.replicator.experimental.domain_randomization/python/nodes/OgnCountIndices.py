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

"""Count selected environment indices for downstream randomization sampling."""

from typing import Any

import numpy as np


class OgnCountIndices:
    """OmniGraph node that reports how many input indices are selected."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Set ``outputs:count`` from ``inputs:indices``.

        Replicator distributions cannot request zero samples, so an empty index
        list is reported as one sample while non-empty lists use their true
        length.

        Args:
            db: Database object containing node inputs and outputs.

        Returns:
            True after ``outputs:count`` is written.
        """
        indices = np.array(db.inputs.indices)

        # WAR because omni.replicator.core.distributions don't accept num_samples=0
        if len(indices) != 0:
            db.outputs.count = len(indices)
        else:
            db.outputs.count = 1

        return True
