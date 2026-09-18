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

"""Validate lifecycle and joint-limit scenarios with the OvPhysX engine.

The D6 articulation fixture receives ``PhysxArticulationAPI`` settings, while
the prim-deletion and simulation-view invalidation scenarios use only standard
USD Physics authoring.
"""

from __future__ import annotations

import os
import sys

import _physics_setup  # noqa: F401

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _legacy_runner import (  # noqa: E402
    cpu_device,
    gpu_device,
    gpu_only,
    run_scenario,
)
from common.misc import (  # noqa: E402
    ArtJointFreeMotionToLimitMotionCommon,
    UsdPrimDeletionCommon,
)
from physx_usd_schemas import PhysxSchema  # noqa: E402

# ---------------------------------------------------------------------------
# ovphysx engine-specific extension.
# ---------------------------------------------------------------------------


class _PhysxArticulationApiMixin:
    """Apply ``PhysxArticulationAPI`` to each replicated D6 articulation root."""

    def _apply_engine_specifics(self) -> None:
        super()._apply_engine_specifics()
        for i in range(self.num_envs):
            prim = self.stage.GetPrimAtPath(f"/envs/env{i}/SimpleArticulation")
            if prim:
                PhysxSchema.PhysxArticulationAPI.Apply(prim)


class _ArtJointFreeMotionScenario(_PhysxArticulationApiMixin, ArtJointFreeMotionToLimitMotionCommon):
    pass


# ---------------------------------------------------------------------------
# Test classes.
# ---------------------------------------------------------------------------


class TestUsdPrimDeletion:
    """Keep rigid-body and articulation views valid after deactivating one ball."""

    def test_prim_deletion_ovphysx_cc(self) -> None:
        """Deactivate one ball during CPU simulation with CPU tensors."""
        run_scenario(self, UsdPrimDeletionCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_prim_deletion_ovphysx_gg(self) -> None:
        """Deactivate one ball during GPU simulation with GPU tensors."""
        run_scenario(self, UsdPrimDeletionCommon, "ovphysx", gpu_device())


class TestArtJointFreeMotionToLimitMotion:
    """Resolve a two-body articulation whose D6 translations use inverted limits."""

    def test_articulation_joint_free_motion_ovphysx_cc(self) -> None:
        """Check the three-DOF D6 topology with CPU simulation and CPU tensors."""
        run_scenario(self, _ArtJointFreeMotionScenario, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_joint_free_motion_ovphysx_gg(self) -> None:
        """Check the three-DOF D6 topology with GPU simulation and GPU tensors."""
        run_scenario(self, _ArtJointFreeMotionScenario, "ovphysx", gpu_device())
