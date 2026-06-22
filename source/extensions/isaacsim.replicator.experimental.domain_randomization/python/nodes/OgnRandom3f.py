# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Generate random float3 values for domain-randomization graphs."""

import random
from typing import Any


class OgnRandom3f:
    """OmniGraph node that samples each float3 component independently."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Write a random ``outputs:output`` vector within the input ranges.

        Each component is sampled uniformly between the corresponding component
        of ``inputs:minimum`` and ``inputs:maximum``.

        Args:
            db: Database object containing node inputs and outputs.

        Returns:
            True after ``outputs:output`` is written.
        """
        min_range = db.inputs.minimum
        max_range = db.inputs.maximum
        db.outputs.output = (
            random.uniform(min_range[0], max_range[0]),
            random.uniform(min_range[1], max_range[1]),
            random.uniform(min_range[2], max_range[2]),
        )

        return True
