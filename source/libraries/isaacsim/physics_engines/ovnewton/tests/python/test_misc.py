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

"""Validate lifecycle and joint-limit scenarios with the Newton engine.

The engine-neutral scenarios run directly because Newton does not require the
PhysX articulation settings used by the OvPhysX D6 fixture.
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
    ArticulationExoticRootRejectedCommon,
    ArtJointFreeMotionToLimitMotionCommon,
    UsdPrimDeletionCommon,
)


class TestUsdPrimDeletion:
    """Validate USD prim deletion with Newton."""

    def test_prim_deletion_newton_cc(self) -> None:
        """Verify prim deletion on the Newton CPU pipeline."""
        run_scenario(self, UsdPrimDeletionCommon, "newton", cpu_device())

    @gpu_only
    def test_prim_deletion_newton_gg(self) -> None:
        """Verify prim deletion on the Newton GPU pipeline."""
        run_scenario(self, UsdPrimDeletionCommon, "newton", gpu_device())


class TestArtJointFreeMotionToLimitMotion:
    """Resolve a two-body articulation whose D6 translations use inverted limits."""

    def test_articulation_joint_free_motion_newton_cc(self) -> None:
        """Check the three-DOF D6 topology on the Newton CPU pipeline."""
        run_scenario(self, ArtJointFreeMotionToLimitMotionCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_joint_free_motion_newton_gg(self) -> None:
        """Check the three-DOF D6 topology on the Newton GPU pipeline."""
        run_scenario(self, ArtJointFreeMotionToLimitMotionCommon, "newton", gpu_device())


class TestArticulationExoticRootRejected:
    """Validate rejection of unsupported articulation-root joints with Newton."""

    def test_articulation_exotic_root_rejected_newton_cc(self) -> None:
        """Verify rejection of unsupported articulation-root joints on the Newton CPU pipeline."""
        run_scenario(self, ArticulationExoticRootRejectedCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_exotic_root_rejected_newton_gg(self) -> None:
        """Verify rejection of unsupported articulation-root joints on the Newton GPU pipeline."""
        run_scenario(self, ArticulationExoticRootRejectedCommon, "newton", gpu_device())
