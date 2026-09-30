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

"""Validate articulation state reads and writes with the Newton engine."""

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
from common.articulation_get_set import (  # noqa: E402
    GetSetAppliedForcesCommon,
    GetSetDofActuationForcesCommon,
    GetSetDofPositionsCommon,
    GetSetDofPositionTargetCommon,
    GetSetDofVelocitiesCommon,
    GetSetDofVelocityTargetCommon,
    GetSetLinkGravityCommon,
    GetSetRootTransformsCommon,
    GetSetRootTransformsMixedBaseCommon,
    GetSetRootVelocitiesCommon,
    LinkStateCommon,
)


class TestArticulationGetSetRootTransforms:
    """Validate articulation-root transform reads and writes with Newton."""

    def test_articulation_get_set_root_transforms_newton_cc(self) -> None:
        """Verify articulation-root transform reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetRootTransformsCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_root_transforms_newton_gg(self) -> None:
        """Verify articulation-root transform reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetRootTransformsCommon, "newton", gpu_device())


class TestArticulationGetSetRootTransformsMixedBase:
    """Validate mixed-base articulation-root transform updates with Newton."""

    def test_articulation_get_set_root_transforms_mixed_base_newton_cc(self) -> None:
        """Verify mixed-base articulation-root transform reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetRootTransformsMixedBaseCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_root_transforms_mixed_base_newton_gg(self) -> None:
        """Verify mixed-base articulation-root transform reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetRootTransformsMixedBaseCommon, "newton", gpu_device())


class TestArticulationGetSetRootVelocities:
    """Validate articulation-root velocity reads and writes with Newton."""

    def test_articulation_get_set_root_velocities_newton_cc(self) -> None:
        """Verify articulation-root velocity reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetRootVelocitiesCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_root_velocities_newton_gg(self) -> None:
        """Verify articulation-root velocity reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetRootVelocitiesCommon, "newton", gpu_device())


class TestArticulationGetSetDofPositions:
    """Validate articulation DOF-position reads and writes with Newton."""

    def test_articulation_get_set_dof_positions_newton_cc(self) -> None:
        """Verify articulation DOF-position reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetDofPositionsCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_positions_newton_gg(self) -> None:
        """Verify articulation DOF-position reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetDofPositionsCommon, "newton", gpu_device())


class TestArticulationGetSetDofVelocities:
    """Validate articulation DOF-velocity reads and writes with Newton."""

    def test_articulation_get_set_dof_velocities_newton_cc(self) -> None:
        """Verify articulation DOF-velocity reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetDofVelocitiesCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_velocities_newton_gg(self) -> None:
        """Verify articulation DOF-velocity reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetDofVelocitiesCommon, "newton", gpu_device())


class TestArticulationGetSetDofPositionTarget:
    """Validate articulation DOF-position-target reads and writes with Newton."""

    def test_articulation_get_set_dof_position_target_newton_cc(self) -> None:
        """Verify articulation DOF-position-target reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetDofPositionTargetCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_position_target_newton_gg(self) -> None:
        """Verify articulation DOF-position-target reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetDofPositionTargetCommon, "newton", gpu_device())


class TestArticulationGetSetDofVelocityTarget:
    """Validate articulation DOF-velocity-target reads and writes with Newton."""

    def test_articulation_get_set_dof_velocity_targets_newton_cc(self) -> None:
        """Verify articulation DOF-velocity-target reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetDofVelocityTargetCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_velocity_targets_newton_gg(self) -> None:
        """Verify articulation DOF-velocity-target reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetDofVelocityTargetCommon, "newton", gpu_device())


class TestArticulationGetSetDofActuationForces:
    """Validate articulation DOF-actuation-force reads and writes with Newton."""

    def test_articulation_get_set_dof_actuation_forces_newton_cc(self) -> None:
        """Verify articulation DOF-actuation-force reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetDofActuationForcesCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_actuation_forces_newton_gg(self) -> None:
        """Verify articulation DOF-actuation-force reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetDofActuationForcesCommon, "newton", gpu_device())


class TestArticulationGetSetLinkGravity:
    """Validate per-link gravity settings with Newton."""

    def test_articulation_get_set_link_gravity_newton_cc(self) -> None:
        """Verify per-link gravity reads and writes on the Newton CPU pipeline."""
        run_scenario(self, GetSetLinkGravityCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_link_gravity_newton_gg(self) -> None:
        """Verify per-link gravity reads and writes on the Newton GPU pipeline."""
        run_scenario(self, GetSetLinkGravityCommon, "newton", gpu_device())


class TestArticulationGetSetAppliedForces:
    """Validate world-frame articulation-link wrench application with Newton."""

    def test_articulation_get_set_applied_forces_newton_cc(self) -> None:
        """Verify global articulation-link wrench application on the Newton CPU pipeline."""
        run_scenario(self, GetSetAppliedForcesCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_get_set_applied_forces_newton_gg(self) -> None:
        """Verify global articulation-link wrench application on the Newton GPU pipeline."""
        run_scenario(self, GetSetAppliedForcesCommon, "newton", gpu_device())


class TestArticulationLinkState:
    """Validate articulation link state with Newton."""

    def test_articulation_link_state_newton_cc(self) -> None:
        """Verify articulation link state on the Newton CPU pipeline."""
        run_scenario(self, LinkStateCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_link_state_newton_gg(self) -> None:
        """Verify articulation link state on the Newton GPU pipeline."""
        run_scenario(self, LinkStateCommon, "newton", gpu_device())
