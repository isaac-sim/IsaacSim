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

"""Validate rigid-body tensors and force application with Newton."""

from __future__ import annotations

import os
import sys

import _physics_setup  # noqa: F401
import pytest

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _legacy_runner import (  # noqa: E402
    cpu_device,
    gpu_device,
    gpu_only,
    run_scenario,
)
from _scenario import DeviceParams  # noqa: E402
from common.rigid_body import (  # noqa: E402
    ReproInertiaSetGetConsistencyCommon,
    RigidBodiesGetSetAppliedForcesCommon,
    RigidBodiesGetSetTransformsCommon,
    RigidBodiesGetSetVelocitiesCommon,
    RigidBodyAccelerationsCommon,
    RigidBodyComsSubsetRoundTripCommon,
    RigidBodyEnableDisablePhysicsCommon,
    RigidBodyForceAtPosCommon,
    RigidBodyForceCommon,
    RigidBodyPropertiesCommon,
    RigidBodyShapePropertiesCommon,
    RigidBodyTransformsCommon,
    RigidBodyVelocitiesCommon,
    RigidBodyViewCommon,
)


class TestRigidBodyView:
    """Validate rigid body view with Newton."""

    def test_rigid_body_view_newton_cc(self) -> None:
        """Verify rigid body view on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyViewCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_view_newton_gg(self) -> None:
        """Verify rigid body view on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyViewCommon, "newton", gpu_device())


class TestRigidBodyEnableDisablePhysics:
    """Validate rigid-body simulation enable and disable behavior with Newton."""

    def test_rigid_body_enable_disable_physics_newton_cc(self) -> None:
        """Verify rigid-body simulation enable/disable behavior on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyEnableDisablePhysicsCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_enable_disable_physics_newton_gg(self) -> None:
        """Verify rigid-body simulation enable/disable behavior on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyEnableDisablePhysicsCommon, "newton", gpu_device())


class TestRigidBodyProperties:
    """Validate rigid body properties with Newton."""

    def test_rigid_body_properties_newton_cc(self) -> None:
        """Verify rigid body properties on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyPropertiesCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_properties_newton_gg(self) -> None:
        """Verify rigid body properties on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyPropertiesCommon, "newton", gpu_device())


class TestRigidBodyComsSubset:
    """Validate indexed rigid-body center-of-mass updates with Newton."""

    def test_rigid_body_coms_subset_newton_cc(self) -> None:
        """Verify indexed center-of-mass updates with the Newton CPU pipeline."""
        run_scenario(self, RigidBodyComsSubsetRoundTripCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_coms_subset_newton_gg(self) -> None:
        """Verify indexed center-of-mass updates with the Newton GPU pipeline."""
        run_scenario(self, RigidBodyComsSubsetRoundTripCommon, "newton", gpu_device())


# Newton's per-shape properties surface (num-shapes / material-properties /
# contact-offsets / rest-offsets) is unwired, and Newton only partially has the data
# (single friction mu, no true contact/rest offset). Temporary gap; real wiring +
# host-cached round-trip belong in a separate Newton-completeness MR.
_GAP_NEWTON_SHAPE_PROPERTIES = "_GAP_NEWTON_SHAPE_PROPERTIES: per-shape material/offset surface unwired on Newton"


@pytest.mark.skip(reason=_GAP_NEWTON_SHAPE_PROPERTIES)
class TestRigidBodyShapeProperties:
    """Skip shape-property round trips because Newton lacks that tensor surface."""

    def test_rigid_body_shape_properties_newton_cc(self) -> None:
        """Verify rigid body shape properties on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyShapePropertiesCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_shape_properties_newton_gg(self) -> None:
        """Verify rigid body shape properties on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyShapePropertiesCommon, "newton", gpu_device())


class TestReproInertiaSetGetConsistency:
    """Validate rigid-body inertia read/write consistency with Newton."""

    def test_repro_inertia_set_get_newton_cc(self) -> None:
        """Verify rigid-body inertia read/write on the Newton CPU pipeline."""
        run_scenario(self, ReproInertiaSetGetConsistencyCommon, "newton", cpu_device())

    @gpu_only
    def test_repro_inertia_set_get_newton_gg(self) -> None:
        """Verify rigid-body inertia read/write on the Newton GPU pipeline."""
        run_scenario(self, ReproInertiaSetGetConsistencyCommon, "newton", gpu_device())


class TestRigidBodyTransforms:
    """Validate rigid body transforms with Newton."""

    def test_rigid_body_transforms_newton_cc(self) -> None:
        """Verify rigid body transforms on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyTransformsCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_transforms_newton_gc(self) -> None:
        """Verify rigid body transforms with Newton GPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyTransformsCommon, "newton", DeviceParams(True, False))

    @gpu_only
    def test_rigid_body_transforms_newton_gg(self) -> None:
        """Verify rigid body transforms on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyTransformsCommon, "newton", gpu_device())


class TestRigidBodyVelocities:
    """Validate rigid body velocities with Newton."""

    def test_rigid_body_velocities_newton_cc(self) -> None:
        """Verify rigid body velocities on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyVelocitiesCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_velocities_newton_gc(self) -> None:
        """Verify rigid body velocities with Newton GPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyVelocitiesCommon, "newton", DeviceParams(True, False))

    @gpu_only
    def test_rigid_body_velocities_newton_gg(self) -> None:
        """Verify rigid body velocities on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyVelocitiesCommon, "newton", gpu_device())


class TestRigidBodyAccelerations:
    """Validate rigid body accelerations with Newton."""

    def test_rigid_body_accelerations_newton_cc(self) -> None:
        """Verify rigid body accelerations on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyAccelerationsCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_accelerations_newton_gg(self) -> None:
        """Verify rigid body accelerations on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyAccelerationsCommon, "newton", gpu_device())


class TestRigidBodyForce:
    """Validate rigid body global force with Newton."""

    def test_rigid_body_force_newton_cc(self) -> None:
        """Verify rigid body global force on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyForceCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_force_newton_gg(self) -> None:
        """Verify rigid body global force on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyForceCommon, "newton", gpu_device())


class TestRigidBodyForceAtPos:
    """Compare world-space force-at-position behavior across two view types.

    Equivalent forces and application points must produce matching root motion
    through both view types.
    """

    def test_rigid_body_force_at_pos_newton_cc(self) -> None:
        """Compare world-space view paths on the Newton CPU pipeline."""
        run_scenario(self, RigidBodyForceAtPosCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_body_force_at_pos_newton_gg(self) -> None:
        """Compare world-space view paths on the Newton GPU pipeline."""
        run_scenario(self, RigidBodyForceAtPosCommon, "newton", gpu_device())


class TestRigidBodiesGetSetTransforms:
    """Validate multi-body transform reads and writes with Newton."""

    def test_rigid_bodies_get_set_transforms_newton_cc(self) -> None:
        """Verify multi-body transform reads and writes on the Newton CPU pipeline."""
        run_scenario(self, RigidBodiesGetSetTransformsCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_bodies_get_set_transforms_newton_gg(self) -> None:
        """Verify multi-body transform reads and writes on the Newton GPU pipeline."""
        run_scenario(self, RigidBodiesGetSetTransformsCommon, "newton", gpu_device())


class TestRigidBodiesGetSetVelocities:
    """Validate multi-body velocity reads and writes with Newton."""

    def test_rigid_bodies_get_set_velocities_newton_cc(self) -> None:
        """Verify multi-body velocity reads and writes on the Newton CPU pipeline."""
        run_scenario(self, RigidBodiesGetSetVelocitiesCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_bodies_get_set_velocities_newton_gg(self) -> None:
        """Verify multi-body velocity reads and writes on the Newton GPU pipeline."""
        run_scenario(self, RigidBodiesGetSetVelocitiesCommon, "newton", gpu_device())


class TestRigidBodiesGetSetAppliedForces:
    """Validate force-at-position application across a multi-body view with Newton."""

    def test_rigid_bodies_get_set_applied_forces_newton_cc(self) -> None:
        """Verify multi-body force-at-position application on the Newton CPU pipeline."""
        run_scenario(self, RigidBodiesGetSetAppliedForcesCommon, "newton", cpu_device())

    @gpu_only
    def test_rigid_bodies_get_set_applied_forces_newton_gg(self) -> None:
        """Verify multi-body force-at-position application on the Newton GPU pipeline."""
        run_scenario(self, RigidBodiesGetSetAppliedForcesCommon, "newton", gpu_device())
