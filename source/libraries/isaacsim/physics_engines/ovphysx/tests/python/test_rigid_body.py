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

"""Validate rigid-body tensors and force application with OvPhysX."""

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
from _scenario import DeviceParams  # noqa: E402
from common.rigid_body import (  # noqa: E402
    ReproInertiaSetGetConsistencyCommon,
    RigidBodiesGetSetAppliedForcesCommon,
    RigidBodiesGetSetTransformsCommon,
    RigidBodiesGetSetVelocitiesCommon,
    RigidBodyAccelerationsCommon,
    RigidBodyDisableGravityCommon,
    RigidBodyEnableDisablePhysicsCommon,
    RigidBodyForceAtPosCommon,
    RigidBodyForceCommon,
    RigidBodyPropertiesCommon,
    RigidBodyShapePropertiesCommon,
    RigidBodyTransformsCommon,
    RigidBodyVelocitiesCommon,
    RigidBodyViewCommon,
    RigidBodyZeroIndexSetCommon,
)


class TestRigidBodyView:
    """Validate rigid-body count and resolved prim-path metadata."""

    def test_rigid_body_view_ovphysx_cc(self) -> None:
        """Check rigid-body discovery with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyViewCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_view_ovphysx_gg(self) -> None:
        """Check rigid-body discovery with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyViewCommon, "ovphysx", gpu_device())


class TestRigidBodyEnableDisablePhysics:
    # Legacy testRigidBody.py registers this scenario CPU-only (no gc/gg).
    # The GPU variants were added during the umbrella port and are the
    # only ones that exercise the disabled-body GPU readback path; mirror
    # the legacy suite by running cc only.
    """Verify indexed simulation disabling controls which rigid bodies fall."""

    def test_rigid_body_enable_disable_physics_ovphysx_cc(self) -> None:
        """Check disable-state motion with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyEnableDisablePhysicsCommon, "ovphysx", cpu_device())


class TestRigidBodyDisableGravity:
    """Verify native gravity-disable flags control rigid-body motion."""

    def test_rigid_body_disable_gravity_ovphysx_cc(self) -> None:
        """Check gravity-disable behavior with CPU simulation and tensors."""
        run_scenario(self, RigidBodyDisableGravityCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_disable_gravity_ovphysx_gg(self) -> None:
        """Check gravity-disable behavior with GPU simulation and tensors."""
        run_scenario(self, RigidBodyDisableGravityCommon, "ovphysx", gpu_device())


class TestRigidBodyProperties:
    """Round-trip mass, center-of-mass, and inertia properties."""

    def test_rigid_body_properties_ovphysx_cc(self) -> None:
        """Round-trip rigid-body properties with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyPropertiesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_properties_ovphysx_gg(self) -> None:
        """Round-trip rigid-body properties with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyPropertiesCommon, "ovphysx", gpu_device())


class TestRigidBodyShapeProperties:
    """Round-trip material, contact-offset, and rest-offset shape data."""

    def test_rigid_body_shape_properties_ovphysx_cc(self) -> None:
        """Round-trip shape properties with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyShapePropertiesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_shape_properties_ovphysx_gg(self) -> None:
        """Round-trip shape properties with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyShapePropertiesCommon, "ovphysx", gpu_device())


class TestReproInertiaSetGetConsistency:
    """Verify that writing a retrieved inertia tensor preserves its value."""

    def test_repro_inertia_set_get_ovphysx_cc(self) -> None:
        """Round-trip authored inertia with CPU simulation and CPU tensors."""
        run_scenario(self, ReproInertiaSetGetConsistencyCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_repro_inertia_set_get_ovphysx_gg(self) -> None:
        """Round-trip authored inertia with GPU simulation and GPU tensors."""
        run_scenario(self, ReproInertiaSetGetConsistencyCommon, "ovphysx", gpu_device())


class TestRigidBodyTransforms:
    """Verify indexed transform writes persist in a gravity-free scene."""

    def test_rigid_body_transforms_ovphysx_cc(self) -> None:
        """Check transform persistence with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyTransformsCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_transforms_ovphysx_gc(self) -> None:
        """Check transform persistence with GPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyTransformsCommon, "ovphysx", DeviceParams(True, False))

    @gpu_only
    def test_rigid_body_transforms_ovphysx_gg(self) -> None:
        """Check transform persistence with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyTransformsCommon, "ovphysx", gpu_device())


class TestRigidBodyVelocities:
    """Verify indexed velocity writes persist in a gravity-free scene."""

    def test_rigid_body_velocities_ovphysx_cc(self) -> None:
        """Check velocity persistence with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyVelocitiesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_velocities_ovphysx_gc(self) -> None:
        """Check velocity persistence with GPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyVelocitiesCommon, "ovphysx", DeviceParams(True, False))

    @gpu_only
    def test_rigid_body_velocities_ovphysx_gg(self) -> None:
        """Check velocity persistence with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyVelocitiesCommon, "ovphysx", gpu_device())


class TestRigidBodyAccelerations:
    """Compare reported acceleration with gravity and finite-difference velocity."""

    def test_rigid_body_accelerations_ovphysx_cc(self) -> None:
        """Check rigid-body acceleration with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyAccelerationsCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_accelerations_ovphysx_gc(self) -> None:
        """Check rigid-body acceleration with GPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyAccelerationsCommon, "ovphysx", DeviceParams(True, False))

    @gpu_only
    def test_rigid_body_accelerations_ovphysx_gg(self) -> None:
        """Check rigid-body acceleration with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyAccelerationsCommon, "ovphysx", gpu_device())


class TestRigidBodyForce:
    """Validate the launch trajectory from a world-frame force."""

    def test_rigid_body_force_ovphysx_cc(self) -> None:
        """Check world-frame force motion with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyForceCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_force_ovphysx_gg(self) -> None:
        """Check world-frame force motion with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyForceCommon, "ovphysx", gpu_device())


class TestRigidBodyZeroIndexSet:
    """Validate that an empty indexed rigid-body write is a no-op."""

    def test_zero_index_set_ovphysx_cc(self) -> None:
        """Verify empty indexed writes with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyZeroIndexSetCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_zero_index_set_ovphysx_gg(self) -> None:
        """Verify empty indexed writes with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyZeroIndexSetCommon, "ovphysx", gpu_device())


class TestRigidBodyForceAtPos:
    """Compare world-space force-at-position behavior across two view types.

    Equivalent forces and application points must produce matching root motion
    through both view types.
    """

    def test_rigid_body_force_at_pos_ovphysx_cc(self) -> None:
        """Compare world-space view paths with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodyForceAtPosCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_body_force_at_pos_ovphysx_gg(self) -> None:
        """Compare world-space view paths with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodyForceAtPosCommon, "ovphysx", gpu_device())


class TestRigidBodiesGetSetTransforms:
    """Round-trip transforms across free bodies and articulation links."""

    def test_rigid_bodies_get_set_transforms_ovphysx_cc(self) -> None:
        """Verify multi-body transform reads and writes with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodiesGetSetTransformsCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_bodies_get_set_transforms_ovphysx_gg(self) -> None:
        """Verify multi-body transform reads and writes with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodiesGetSetTransformsCommon, "ovphysx", gpu_device())


class TestRigidBodiesGetSetVelocities:
    """Round-trip velocities across free bodies and articulation links."""

    def test_rigid_bodies_get_set_velocities_ovphysx_cc(self) -> None:
        """Verify multi-body velocity reads and writes with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodiesGetSetVelocitiesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_bodies_get_set_velocities_ovphysx_gg(self) -> None:
        """Verify multi-body velocity reads and writes with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodiesGetSetVelocitiesCommon, "ovphysx", gpu_device())


class TestRigidBodiesGetSetAppliedForces:
    """Verify an upward force-at-position write lifts every selected body.

    Every selected free body and articulation link must move upward after
    receiving the same indexed force.
    """

    def test_rigid_bodies_get_set_applied_forces_ovphysx_cc(self) -> None:
        """Check multi-body upward motion with CPU simulation and CPU tensors."""
        run_scenario(self, RigidBodiesGetSetAppliedForcesCommon, "ovphysx", cpu_device())

    @gpu_only
    def test_rigid_bodies_get_set_applied_forces_ovphysx_gg(self) -> None:
        """Check multi-body upward motion with GPU simulation and GPU tensors."""
        run_scenario(self, RigidBodiesGetSetAppliedForcesCommon, "ovphysx", gpu_device())
