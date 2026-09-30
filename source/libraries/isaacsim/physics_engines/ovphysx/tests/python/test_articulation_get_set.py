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

"""Validate articulation state reads and writes with the OvPhysX engine."""

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
    GetSetRootVelocitiesCommon,
    LinkStateCommon,
)


class TestArticulationGetSetRootTransforms:
    """Round-trip floating-base Ant root translations and orientations."""

    def test_articulation_get_set_root_transforms_ovphysx_cc(self) -> None:
        """Round-trip root transforms with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetRootTransformsCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_root_transforms_ovphysx_gg(self) -> None:
        """Round-trip root transforms with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetRootTransformsCommon, "ovphysx", gpu_device())


class TestArticulationGetSetRootVelocities:
    """Round-trip floating-base Ant spatial velocities."""

    def test_articulation_get_set_root_velocities_ovphysx_cc(self) -> None:
        """Round-trip root velocities with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetRootVelocitiesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_root_velocities_ovphysx_gg(self) -> None:
        """Round-trip root velocities with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetRootVelocitiesCommon, "ovphysx", gpu_device())


class TestArticulationGetSetDofPositions:
    """Round-trip Ant joint positions."""

    def test_articulation_get_set_dof_positions_ovphysx_cc(self) -> None:
        """Round-trip joint positions with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetDofPositionsCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_positions_ovphysx_gg(self) -> None:
        """Round-trip joint positions with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetDofPositionsCommon, "ovphysx", gpu_device())


class TestArticulationGetSetDofVelocities:
    """Round-trip Ant joint velocities."""

    def test_articulation_get_set_dof_velocities_ovphysx_cc(self) -> None:
        """Round-trip joint velocities with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetDofVelocitiesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_velocities_ovphysx_gg(self) -> None:
        """Round-trip joint velocities with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetDofVelocitiesCommon, "ovphysx", gpu_device())


class TestArticulationGetSetDofPositionTarget:
    """Round-trip Ant joint position targets."""

    def test_articulation_get_set_dof_position_target_ovphysx_cc(self) -> None:
        """Round-trip position targets with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetDofPositionTargetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_position_target_ovphysx_gg(self) -> None:
        """Round-trip position targets with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetDofPositionTargetCommon, "ovphysx", gpu_device())


class TestArticulationGetSetDofVelocityTarget:
    """Round-trip Ant joint velocity targets."""

    def test_articulation_get_set_dof_velocity_targets_ovphysx_cc(self) -> None:
        """Round-trip velocity targets with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetDofVelocityTargetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_velocity_targets_ovphysx_gg(self) -> None:
        """Round-trip velocity targets with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetDofVelocityTargetCommon, "ovphysx", gpu_device())


class TestArticulationGetSetDofActuationForces:
    """Round-trip Ant joint actuation forces."""

    def test_articulation_get_set_dof_actuation_forces_ovphysx_cc(self) -> None:
        """Round-trip actuation forces with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetDofActuationForcesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_dof_actuation_forces_ovphysx_gg(self) -> None:
        """Round-trip actuation forces with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetDofActuationForcesCommon, "ovphysx", gpu_device())


class TestArticulationGetSetLinkGravity:
    """Verify native per-link gravity flags control articulation motion."""

    def test_articulation_get_set_link_gravity_ovphysx_cc(self) -> None:
        """Check per-link gravity behavior with CPU simulation and tensors."""
        run_scenario(self, GetSetLinkGravityCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_link_gravity_ovphysx_gg(self) -> None:
        """Check per-link gravity behavior with GPU simulation and tensors."""
        run_scenario(self, GetSetLinkGravityCommon, "ovphysx", gpu_device())


class TestArticulationGetSetAppliedForces:
    """Verify world-frame force-at-position writes lift each Ant."""

    def test_articulation_get_set_applied_forces_ovphysx_cc(self) -> None:
        """Check world-frame force application with CPU simulation and CPU tensors."""
        run_scenario(self, GetSetAppliedForcesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_get_set_applied_forces_ovphysx_gg(self) -> None:
        """Check world-frame force application with GPU simulation and GPU tensors."""
        run_scenario(self, GetSetAppliedForcesCommon, "ovphysx", gpu_device())


class TestArticulationLinkState:
    """Validate full and indexed Ant link-transform and velocity reads."""

    def test_articulation_link_state_ovphysx_cc(self) -> None:
        """Check full and indexed link state with CPU simulation and CPU tensors."""
        run_scenario(self, LinkStateCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_articulation_link_state_ovphysx_gg(self) -> None:
        """Check full and indexed link state with GPU simulation and GPU tensors."""
        run_scenario(self, LinkStateCommon, "ovphysx", gpu_device())
