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

"""Validate articulation dynamics tensors with the Newton engine."""

from __future__ import annotations

import os
import sys
from typing import Any

import _physics_setup  # noqa: F401
from isaacsim.physics_engines.ovnewton import SolverType

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _legacy_runner import (  # noqa: E402
    cpu_device,
    gpu_device,
    gpu_only,
    run_scenario,
    unimplemented_placeholder,
)
from common.articulation_dynamics import (  # noqa: E402
    ArticulationRootTransformsGetCommon,
    CoriolisCentrifugalCommon,
    DofDriveTypeCommon,
    DofPositionSetFloatingBaseReadbackCommon,
    DofPositionSetReadbackCommon,
    DofPositionSetSubsetReadbackCommon,
    DofPositionTargetReadbackCommon,
    DofVelocitySetFloatingBaseReadbackCommon,
    DofVelocitySetReadbackCommon,
    DofVelocityThroughRootVelocitySetCommon,
    GravityCompensationCommon,
    JacobiansCommon,
    JointActuationForcesCommon,
    JointPerformanceEnvelopeCommon,
    MassMatricesCommon,
    RootVelocityThroughDofSetCommon,
)


class TestJacobians:
    """Validate articulation Jacobians with Newton."""

    def test_jacobians_newton_cc(self) -> None:
        """Verify jacobians on the Newton CPU pipeline."""
        run_scenario(self, JacobiansCommon, "newton", cpu_device())

    @gpu_only
    def test_jacobians_newton_gg(self) -> None:
        """Verify jacobians on the Newton GPU pipeline."""
        run_scenario(self, JacobiansCommon, "newton", gpu_device())


class TestRootVelocityThroughDofSet:
    """Validate root velocity through DOF set with Newton."""

    def test_root_velocity_through_dof_set_newton_cc(self) -> None:
        """Verify root velocity through DOF set on the Newton CPU pipeline."""
        run_scenario(self, RootVelocityThroughDofSetCommon, "newton", cpu_device())

    @gpu_only
    def test_root_velocity_through_dof_set_newton_gg(self) -> None:
        """Verify root velocity through DOF set on the Newton GPU pipeline."""
        run_scenario(self, RootVelocityThroughDofSetCommon, "newton", gpu_device())


class TestDofVelocityThroughRootVelocitySet:
    """Validate DOF velocity through root velocity set with Newton."""

    def test_dof_velocity_through_root_velocity_set_newton_cc(self) -> None:
        """Verify DOF velocity through root velocity set on the Newton CPU pipeline."""
        run_scenario(self, DofVelocityThroughRootVelocitySetCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_velocity_through_root_velocity_set_newton_gg(self) -> None:
        """Verify DOF velocity through root velocity set on the Newton GPU pipeline."""
        run_scenario(self, DofVelocityThroughRootVelocitySetCommon, "newton", gpu_device())


class TestMassMatrices:
    """Validate mass matrices with Newton."""

    def test_mass_matrices_newton_cc(self) -> None:
        """Verify mass matrices on the Newton CPU pipeline."""
        run_scenario(self, MassMatricesCommon, "newton", cpu_device())

    @gpu_only
    def test_mass_matrices_newton_gg(self) -> None:
        """Verify mass matrices on the Newton GPU pipeline."""
        run_scenario(self, MassMatricesCommon, "newton", gpu_device())


class TestCoriolisCentrifugal:
    """Validate Coriolis and centrifugal compensation forces with Newton."""

    def test_coriolis_centrifugal_newton_cc(self) -> None:
        """Verify Coriolis and centrifugal compensation with the Newton CPU pipeline."""
        run_scenario(self, CoriolisCentrifugalCommon, "newton", cpu_device())

    @gpu_only
    def test_coriolis_centrifugal_newton_gg(self) -> None:
        """Verify Coriolis and centrifugal compensation with the Newton GPU pipeline."""
        run_scenario(self, CoriolisCentrifugalCommon, "newton", gpu_device())


class TestGravityCompensation:
    """Validate gravity compensation with Newton."""

    def test_gravity_compensation_newton_cc(self) -> None:
        """Verify gravity compensation on the Newton CPU pipeline."""
        run_scenario(self, GravityCompensationCommon, "newton", cpu_device())

    @gpu_only
    def test_gravity_compensation_newton_gg(self) -> None:
        """Verify gravity compensation on the Newton GPU pipeline."""
        run_scenario(self, GravityCompensationCommon, "newton", gpu_device())


class TestJointActuationForces:
    """Validate projected joint forces after applying actuation forces."""

    class _Scenario(JointActuationForcesCommon):
        """Request joint wrench output before Newton state creation."""

        def configure_newton(self, config: Any) -> None:
            """Enable joint wrench output and inverse-dynamics force reporting."""
            config.solver_outputs.link_incoming_joint_force = True
            config.solver.solver_type = SolverType.FEATHERSTONE

    def test_joint_actuation_forces_newton_cc(self) -> None:
        """Verify joint actuation forces on the Newton CPU pipeline."""
        run_scenario(self, self._Scenario, "newton", cpu_device())

    @gpu_only
    def test_joint_actuation_forces_newton_gg(self) -> None:
        """Verify joint actuation forces on the Newton GPU pipeline."""
        run_scenario(self, self._Scenario, "newton", gpu_device())


class TestArticulationDofDriveType:
    """Validate articulation DOF drive type with Newton."""

    def test_articulation_dof_drive_type_newton_cc(self) -> None:
        """Verify articulation DOF drive type on the Newton CPU pipeline."""
        run_scenario(self, DofDriveTypeCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_dof_drive_type_newton_gg(self) -> None:
        """Verify articulation DOF drive type on the Newton GPU pipeline."""
        run_scenario(self, DofDriveTypeCommon, "newton", gpu_device())


class TestJointPerformanceEnvelope:
    """Validate joint performance envelope with Newton."""

    def test_joint_performance_envelope_newton_cc(self) -> None:
        """Verify joint performance envelope on the Newton CPU pipeline."""
        run_scenario(self, JointPerformanceEnvelopeCommon, "newton", cpu_device())

    @gpu_only
    def test_joint_performance_envelope_newton_gg(self) -> None:
        """Verify joint performance envelope on the Newton GPU pipeline."""
        run_scenario(self, JointPerformanceEnvelopeCommon, "newton", gpu_device())


class TestArticulationRootTransforms:
    """Validate articulation root transforms with Newton."""

    def test_articulation_root_transforms_newton_cc(self) -> None:
        """Verify articulation root transforms on the Newton CPU pipeline."""
        run_scenario(self, ArticulationRootTransformsGetCommon, "newton", cpu_device())

    @gpu_only
    def test_articulation_root_transforms_newton_gg(self) -> None:
        """Verify articulation root transforms on the Newton GPU pipeline."""
        run_scenario(self, ArticulationRootTransformsGetCommon, "newton", gpu_device())


class TestDofPositionTargetReadback:
    """Validate DOF position target readback with Newton."""

    def test_dof_position_target_readback_newton_cc(self) -> None:
        """Verify DOF position target readback on the Newton CPU pipeline."""
        run_scenario(self, DofPositionTargetReadbackCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_position_target_readback_newton_gg(self) -> None:
        """Verify DOF position target readback on the Newton GPU pipeline."""
        run_scenario(self, DofPositionTargetReadbackCommon, "newton", gpu_device())


class TestDofPositionSetReadback:
    """Validate DOF position set readback with Newton."""

    def test_dof_position_set_readback_newton_cc(self) -> None:
        """Verify DOF position set readback on the Newton CPU pipeline."""
        run_scenario(self, DofPositionSetReadbackCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_position_set_readback_newton_gg(self) -> None:
        """Verify DOF position set readback on the Newton GPU pipeline."""
        run_scenario(self, DofPositionSetReadbackCommon, "newton", gpu_device())


class TestDofVelocitySetReadback:
    """Validate DOF velocity set readback with Newton."""

    def test_dof_velocity_set_readback_newton_cc(self) -> None:
        """Verify DOF velocity set readback on the Newton CPU pipeline."""
        run_scenario(self, DofVelocitySetReadbackCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_velocity_set_readback_newton_gg(self) -> None:
        """Verify DOF velocity set readback on the Newton GPU pipeline."""
        run_scenario(self, DofVelocitySetReadbackCommon, "newton", gpu_device())


class TestDofPositionSetSubsetReadback:
    """Validate DOF position set subset readback with Newton."""

    def test_dof_position_set_subset_readback_newton_cc(self) -> None:
        """Verify DOF position set subset readback on the Newton CPU pipeline."""
        run_scenario(self, DofPositionSetSubsetReadbackCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_position_set_subset_readback_newton_gg(self) -> None:
        """Verify DOF position set subset readback on the Newton GPU pipeline."""
        run_scenario(self, DofPositionSetSubsetReadbackCommon, "newton", gpu_device())


class TestDofPositionSetFloatingBaseReadback:
    """Validate DOF position set floating base readback with Newton."""

    def test_dof_position_set_floating_base_readback_newton_cc(self) -> None:
        """Verify DOF position set floating base readback on the Newton CPU pipeline."""
        run_scenario(self, DofPositionSetFloatingBaseReadbackCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_position_set_floating_base_readback_newton_gg(self) -> None:
        """Verify DOF position set floating base readback on the Newton GPU pipeline."""
        run_scenario(self, DofPositionSetFloatingBaseReadbackCommon, "newton", gpu_device())


class TestDofVelocitySetFloatingBaseReadback:
    """Validate DOF velocity set floating base readback with Newton."""

    def test_dof_velocity_set_floating_base_readback_newton_cc(self) -> None:
        """Verify DOF velocity set floating base readback on the Newton CPU pipeline."""
        run_scenario(self, DofVelocitySetFloatingBaseReadbackCommon, "newton", cpu_device())

    @gpu_only
    def test_dof_velocity_set_floating_base_readback_newton_gg(self) -> None:
        """Verify DOF velocity set floating base readback on the Newton GPU pipeline."""
        run_scenario(self, DofVelocitySetFloatingBaseReadbackCommon, "newton", gpu_device())


# ---------------------------------------------------------------------------
# Placeholder shells — mirrored from the ovphysx side so the matrix is
# symmetric. Newton has no equivalent scenarios for these (they all
# depend on engine APIs that don't exist on Newton today).
# ---------------------------------------------------------------------------


@unimplemented_placeholder
class TestArticulationRootVelocities:
    """Record unimplemented Newton articulation-root velocity scenarios."""

    def test_articulation_root_velocities_newton_cc(self) -> None:
        """Record the unimplemented articulation root velocities case for Newton."""

    def test_articulation_root_velocities_newton_gg(self) -> None:
        """Record the unimplemented articulation root velocities case for Newton."""


@unimplemented_placeholder
class TestLinkAccelerations:
    """Record unimplemented Newton link-acceleration scenarios."""

    def test_link_accelerations_newton_cc(self) -> None:
        """Record the unimplemented link accelerations case for Newton."""

    def test_link_accelerations_newton_gg(self) -> None:
        """Record the unimplemented link accelerations case for Newton."""


# `joint_force_dof_projection` and `link_incoming_joint_force` placeholders
# previously lived here as `TestPortShells`; the real Newton-side tests are
# under `newton/test_force_projection.py` and `newton/test_force_sensors.py`.
# Empty stubs only added misleading SKIP rows to the matrix.
