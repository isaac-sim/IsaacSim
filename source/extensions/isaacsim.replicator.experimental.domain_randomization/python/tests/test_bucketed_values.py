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

"""Verify Gaussian material-property bucket ranges for physics-view writers."""

import numpy as np
import omni.kit.test
from isaacsim.replicator.experimental.domain_randomization.ogn.python.nodes.OgnWritePhysicsArticulationView import (
    get_bucketed_values as get_articulation_bucketed_values,
)
from isaacsim.replicator.experimental.domain_randomization.ogn.python.nodes.OgnWritePhysicsRigidPrimView import (
    get_bucketed_values as get_rigid_bucketed_values,
)


class TestBucketedValues(omni.kit.test.AsyncTestCase):
    """Test normal-distribution bucket bounds for both physics-view writers."""

    async def test_rigid_prim_gaussian_buckets_use_standard_deviation(self) -> None:
        """Derive rigid-prim bucket ranges from standard deviation, not variance."""
        samples = np.array([[-100.0] * 3, [100.0] * 3])

        for std, expected in ((0.25, [[0.5] * 3, [1.25] * 3]), (4.0, [[-7.0] * 3, [5.0] * 3])):
            actual = get_rigid_bucketed_values(
                "view", "material_properties", samples, "gaussian", np.ones(3), np.full(3, std), 4
            )
            self.assertTrue(np.allclose(actual, expected))

    async def test_articulation_gaussian_buckets_use_standard_deviation(self) -> None:
        """Derive articulation bucket ranges from standard deviation, not variance."""
        samples = np.array([[[-100.0] * 3, [100.0] * 3]])

        for std, expected in (
            (0.25, [[[0.5] * 3, [1.25] * 3]]),
            (4.0, [[[-7.0] * 3, [5.0] * 3]]),
        ):
            actual = get_articulation_bucketed_values(
                "view", "material_properties", samples, "gaussian", np.ones(3), np.full(3, std), 4
            )
            self.assertTrue(np.allclose(actual, expected))
