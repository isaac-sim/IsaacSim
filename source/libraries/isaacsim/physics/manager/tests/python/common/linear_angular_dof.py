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

"""Exercise linear and angular articulation degree-of-freedom operations.

Rail-cart and pole-cart fixtures use ``UsdPhysics.DriveAPI`` to cover
positions, velocities, actuation forces, position targets, and velocity
targets without engine-specific schemas.
"""

from __future__ import annotations

import os
import sys
from typing import Literal

import numpy as np
import warp as wp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

import warp_utils as wp_utils  # noqa: E402
from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    Transform,
    get_asset_root,
)
from pxr import Gf, Usd, UsdPhysics  # noqa: E402


def _set_drive(
    joint_prim: Usd.Prim,
    drive_type: Literal["linear", "angular"],
    target_type: Literal["position", "velocity"],
    target_value: float,
    stiffness: float,
    damping: float,
    max_force: float,
) -> None:
    """Configure a linear or angular USD joint drive.

    Args:
        joint_prim: Joint prim to which the drive is applied.
        drive_type: Translational or rotational drive type.
        target_type: Drive target attribute to configure.
        target_value: Initial position or velocity target.
        stiffness: Drive stiffness.
        damping: Drive damping.
        max_force: Maximum force or torque produced by the drive.

    """
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, drive_type)
    if target_type == "position":
        attr = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr(target_value)
        attr.Set(target_value)
    elif target_type == "velocity":
        attr = drive.GetTargetVelocityAttr() or drive.CreateTargetVelocityAttr(target_value)
        attr.Set(target_value)
    else:
        raise ValueError(f"invalid drive target type {target_type!r}")
    (drive.GetStiffnessAttr() or drive.CreateStiffnessAttr(stiffness)).Set(stiffness)
    (drive.GetDampingAttr() or drive.CreateDampingAttr(damping)).Set(damping)
    (drive.GetMaxForceAttr() or drive.CreateMaxForceAttr(max_force)).Set(max_force)


class _LinearDofsBase(GridTestBase):
    """Build a grid of single-axis rail-cart articulations.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=42)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)
        asset_path = os.path.join(get_asset_root(), "CartRailNoPole.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("railcart"),
            Transform((0.0, 0.0, 1.0)),
            asset_path,
        )

    def on_start(self, sim: object) -> None:
        """Create the rail-cart articulation view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.railcarts = sim.create_articulation_view("/envs/*/railcart")
        self.check_articulation_view(self.railcarts, self.num_envs, 2, 1, True)
        self.all_indices = wp_utils.arange(self.railcarts.count, device=self.wp_device)
        self.on_start_impl(sim)

    def on_start_impl(self, sim: object) -> None:
        """Configure the quantity exercised by a concrete scenario.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim


class LinearDofPositionsCommon(_LinearDofsBase):
    """Verify writes and reads of linear joint positions."""

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct initial position for each rail cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.railcarts.count, -2.5, 2.5, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.railcarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.railcarts.set_data("dof-positions", self.desired, self.all_indices)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that the linear joint positions match the requested values.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 1:
            p = self.railcarts.get_data("dof-positions").numpy().squeeze()
            d = self.desired.numpy().squeeze()
            assert wp_utils.wp_allclose(p, d, rtol=1e-3, atol=1e-4), "expected positions"
            self.finish()


class LinearDofVelocitiesCommon(_LinearDofsBase):
    """Verify writes and reads of linear joint velocities."""

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct initial velocity for each rail cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.railcarts.count, -2.0, 2.0, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.railcarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.railcarts.set_data("dof-velocities", self.desired, self.all_indices)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that the linear joint velocities match the requested values.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 1:
            v = self.railcarts.get_data("dof-velocities").numpy().squeeze()
            d = self.desired.numpy().squeeze()
            assert wp_utils.wp_allclose(v, d, rtol=1e-3, atol=1e-4), "expected velocities"
            self.finish()


class LinearDofForcesCommon(_LinearDofsBase):
    """Verify linear actuation forces through a damped position controller."""

    def on_start_impl(self, sim: object) -> None:
        """Initialize joint positions and allocate the force buffer.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        self.pmin = -2.5
        self.pmax = 2.5
        flat = wp_utils.linspace(self.railcarts.count, self.pmin, self.pmax, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.railcarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.railcarts.set_data("dof-positions", self.desired, self.all_indices)
        self.forces = wp.zeros(
            (self.num_envs, self.railcarts.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        )

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Drive every cart toward zero and validate the resulting state.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        dof_pos = self.railcarts.get_data("dof-positions")
        dof_vel = self.railcarts.get_data("dof-velocities")

        if stepno == 0:
            p = dof_pos.numpy()
            assert abs(p[0, 0] - self.pmin) < 0.01, "expected min pos"
            assert abs(p[-1, 0] - self.pmax) < 0.01, "expected max pos"
        if stepno >= 99:
            p = dof_pos.numpy()
            v = dof_vel.numpy()
            assert (abs(p) < 0.01).all(), "expected positions ~0"
            assert (abs(v) < 0.01).all(), "expected velocities ~0"
            self.finish()
            return

        stiffness = 1000.0
        damping = 120.0
        # Transfer to the kernel device if get_data returned a CPU array (ovphysx GPU mode).
        dev = str(self.wp_device)
        pos_k = wp.array(dof_pos, device=dev) if str(dof_pos.device) != dev else dof_pos
        vel_k = wp.array(dof_vel, device=dev) if str(dof_vel.device) != dev else dof_vel
        wp_utils.compute_dof_forces(pos_k, vel_k, self.forces, stiffness, damping, device=self.wp_device)
        self.railcarts.set_data("dof-actuation-forces", self.forces, self.all_indices)


class LinearDofPositionTargetsCommon(_LinearDofsBase):
    """Verify position targets for linear USD joint drives.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, device_params)
        for i in range(self.num_envs):
            joint_prim = self.stage.GetPrimAtPath(f"/envs/env{i}/railcart/cartJoint")
            if joint_prim and joint_prim.IsValid():
                _set_drive(joint_prim, "linear", "position", 0.0, 2000.0, 250.0, 4000.0)

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct linear position target for each rail cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.railcarts.count, -2.5, 2.5, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.railcarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.railcarts.set_data("dof-position-targets", self.desired, self.all_indices)
        self.targets = self.desired.numpy().squeeze()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that each linear drive converges to its position target.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 60:
            p = self.railcarts.get_data("dof-positions").numpy().squeeze()
            assert wp_utils.wp_allclose(p, self.targets, rtol=1e-02, atol=1e-02), "expected positions"
            self.finish()


class LinearDofVelocityTargetsCommon(_LinearDofsBase):
    """Verify velocity targets for linear USD joint drives.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, device_params)
        for i in range(self.num_envs):
            joint_prim = self.stage.GetPrimAtPath(f"/envs/env{i}/railcart/cartJoint")
            if joint_prim and joint_prim.IsValid():
                _set_drive(joint_prim, "linear", "velocity", 0.0, 0.0, 500.0, 4000.0)

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct linear velocity target for each rail cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.railcarts.count, -2.0, 2.0, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.railcarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.railcarts.set_data("dof-velocity-targets", self.desired, self.all_indices)
        self.targets = self.desired.numpy().squeeze()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that each linear drive reaches its velocity target.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 20:
            v = self.railcarts.get_data("dof-velocities").numpy().squeeze()
            assert wp_utils.wp_allclose(v, self.targets, rtol=1e-02, atol=1e-02), "expected velocities"
            self.finish()


class _AngularDofsBase(GridTestBase):
    """Build a grid of single-axis pole-cart articulations.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=42)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)
        asset_path = os.path.join(get_asset_root(), "CartPoleNoRail.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("polecart"),
            Transform((0.0, 0.0, 1.0)),
            asset_path,
        )

    def on_start(self, sim: object) -> None:
        """Create the pole-cart articulation view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.polecarts = sim.create_articulation_view("/envs/*/polecart")
        self.check_articulation_view(self.polecarts, self.num_envs, 2, 1, True)
        self.all_indices = wp_utils.arange(self.polecarts.count, device=self.wp_device)
        self.on_start_impl(sim)

    def on_start_impl(self, sim: object) -> None:
        """Configure the quantity exercised by a concrete scenario.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim


class AngularDofPositionsCommon(_AngularDofsBase):
    """Verify writes and reads of angular joint positions."""

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct initial angle for each pole cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.polecarts.count, -1.5, 1.5, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.polecarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.polecarts.set_data("dof-positions", self.desired, self.all_indices)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that the angular joint positions match the requested values.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 1:
            p = self.polecarts.get_data("dof-positions").numpy().squeeze()
            d = self.desired.numpy().squeeze()
            assert wp_utils.wp_allclose(p, d, rtol=1e-3, atol=1e-4), "expected positions"
            self.finish()


class AngularDofVelocitiesCommon(_AngularDofsBase):
    """Verify writes and reads of angular joint velocities."""

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct initial angular velocity for each pole cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.polecarts.count, -1.0, 1.0, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.polecarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.polecarts.set_data("dof-velocities", self.desired, self.all_indices)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that the angular velocities match the requested values.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 1:
            v = self.polecarts.get_data("dof-velocities").numpy().squeeze()
            d = self.desired.numpy().squeeze()
            assert wp_utils.wp_allclose(v, d, rtol=1e-3, atol=1e-4), "expected velocities"
            self.finish()


class AngularDofForcesCommon(_AngularDofsBase):
    """`dof-actuation-forces` is plumbed through to the joint.

    Applies a constant torque to half the environments (zero to the rest) from
    rest and checks that the torqued joints gain velocity in the applied
    direction while the unforced joints stay put. This verifies the *interface* --
    a set actuation force reaches the solver, with the right sign, and its value
    is honored -- not the resulting magnitude, which varies by solver and falls
    outside the tensor API contract. A constant open-loop torque (rather than a stiff closed-loop
    PD) keeps the check stable on any solver, since a maximal-coordinate solver
    integrates raw joint forces explicitly and a stiff feedback loop diverges.
    """

    def on_start_impl(self, sim: object) -> None:
        """Initialize the angular joints and alternating torque buffer.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        n = self.polecarts.get_metadata("num-dofs")
        # Start from rest so any response is attributable to the applied torque.
        self.polecarts.set_data(
            "dof-velocities",
            wp.zeros((self.polecarts.count, n), dtype=wp.float32, device=self.wp_device),
            self.all_indices,
        )
        # Torque every other env, leave the rest unforced: this checks both that
        # the force is applied and that its value is honored (unforced == at rest).
        taus = np.zeros((self.polecarts.count, n), dtype=np.float32)
        taus[::2] = 1.0
        self.torqued = taus.reshape(-1) > 0.0
        self.forces = wp.from_numpy(taus, dtype=wp.float32, device=self.wp_device)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Apply torques and compare torqued joints with unforced joints.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        self.polecarts.set_data("dof-actuation-forces", self.forces, self.all_indices)
        if stepno >= 20:
            v = self.polecarts.get_data("dof-velocities").numpy().reshape(-1)
            # Torqued joints respond in the applied (positive) direction ...
            assert (v[self.torqued] > 0.01).all(), "a set actuation force must produce a velocity response"
            # ... and unforced joints stay at rest (the force value is honored).
            assert (np.abs(v[~self.torqued]) < 0.01).all(), "unforced joints must stay at rest"
            self.finish()


class AngularDofPositionTargetsCommon(_AngularDofsBase):
    """Verify position targets for angular USD joint drives.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, device_params)
        for i in range(self.num_envs):
            joint_prim = self.stage.GetPrimAtPath(f"/envs/env{i}/polecart/poleJoint")
            if joint_prim and joint_prim.IsValid():
                _set_drive(joint_prim, "angular", "position", 0.0, 200.0, 25.0, 1000.0)

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct angular position target for each pole cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.polecarts.count, -1.5, 1.5, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.polecarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.polecarts.set_data("dof-position-targets", self.desired, self.all_indices)
        self.targets = self.desired.numpy().squeeze()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that each angular drive converges to its position target.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 60:
            p = self.polecarts.get_data("dof-positions").numpy().squeeze()
            assert wp_utils.wp_allclose(p, self.targets, rtol=1e-01, atol=1e-01), "expected positions"
            self.finish()


class AngularDofVelocityTargetsCommon(_AngularDofsBase):
    """Verify velocity targets for angular USD joint drives.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, device_params)
        for i in range(self.num_envs):
            joint_prim = self.stage.GetPrimAtPath(f"/envs/env{i}/polecart/poleJoint")
            if joint_prim and joint_prim.IsValid():
                _set_drive(joint_prim, "angular", "velocity", 0.0, 0.0, 50.0, 1000.0)

    def on_start_impl(self, sim: object) -> None:
        """Set a distinct angular velocity target for each pole cart.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        del sim
        flat = wp_utils.linspace(self.polecarts.count, -1.0, 1.0, include_end=True, device=self.wp_device)
        self.desired = wp.from_numpy(
            flat.numpy().reshape(self.polecarts.count, 1), dtype=wp.float32, device=self.wp_device
        )
        self.polecarts.set_data("dof-velocity-targets", self.desired, self.all_indices)
        self.targets = self.desired.numpy().squeeze()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Check that each angular drive reaches its velocity target.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno == 20:
            v = self.polecarts.get_data("dof-velocities").numpy().squeeze()
            assert wp_utils.wp_allclose(v, self.targets, rtol=1e-01, atol=1e-01), "expected velocities"
            self.finish()
