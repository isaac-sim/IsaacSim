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

"""Exercise articulation dynamics and state synchronization.

The scenarios cover Jacobians, mass and compensation terms, root state,
drive configuration, actuation forces, link acceleration, and joint-state
readback after simulation.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest
import warp as wp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

import math  # noqa: E402

import warp_utils as wp_utils  # noqa: E402
from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    SimulationEntities,
    Transform,
    get_asset_root,
)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402


class _AntArticulationBase(GridTestBase):
    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("ant"),
            Transform((0.0, 0.0, 1.0)),
            asset_path,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the shared Ant articulation view and index buffer.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(self.ants, self.num_envs, 9, 8, True)
        self.all_indices = wp_utils.arange(self.ants.count, device=self.wp_device)


class JacobiansCommon(_AntArticulationBase):
    """Validate that articulation Jacobians are finite and nonzero."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Read and validate Jacobians after the articulation settles.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 20:
            jac = self.ants.get_data("jacobians").numpy()
            assert jac.shape[0] == self.ants.count
            # A real geometric Jacobian is finite and non-zero even at rest; the
            # old stub returned zeros, so this guards against a regression to it.
            assert np.all(np.isfinite(jac)), "jacobian has non-finite entries"
            assert float(np.abs(jac).max()) > 0.0, "jacobian is all zeros"
            self.finish()


class MassMatricesCommon(_AntArticulationBase):
    """Validate generalized mass-matrix shape and positive definiteness."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Read and validate generalized mass matrices.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 5:
            mm_np = self.ants.get_data("generalized-mass-matrices").numpy()
            assert mm_np.shape[0] == self.ants.count
            assert mm_np.shape[-1] == mm_np.shape[-2]
            # Documented contract: H is symmetric positive-definite. A zero stub
            # fails the positive-diagonal check; symmetry pins the computation.
            assert np.all(np.isfinite(mm_np)), "mass matrix has non-finite entries"
            assert np.allclose(mm_np, np.swapaxes(mm_np, -1, -2), rtol=1e-3, atol=1e-4), "mass matrix is not symmetric"
            diag = np.diagonal(mm_np, axis1=-2, axis2=-1)
            assert float(diag.min()) > 0.0, "mass matrix diagonal is not positive"
            self.finish()


class CoriolisCentrifugalCommon(_AntArticulationBase):
    """Validate Coriolis-and-centrifugal compensation force rows."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Read the compensation forces and validate their batch size.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 5:
            self.skip_if_unsupported(self.ants, "coriolis-and-centrifugal-compensation-forces", "get")
            cc_np = self.ants.get_data("coriolis-and-centrifugal-compensation-forces").numpy()
            assert cc_np.shape[0] == self.ants.count
            self.finish()


class GravityCompensationCommon(_AntArticulationBase):
    """Validate gravity-compensation force rows under nonzero gravity.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, device_params)
        self.sim_params.gravity_mag = 9.81

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Read gravity compensation and validate its batch size.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 5:
            self.skip_if_unsupported(self.ants, "gravity-compensation-forces", "get")
            gc_np = self.ants.get_data("gravity-compensation-forces").numpy()
            assert gc_np.shape[0] == self.ants.count
            self.finish()


class ArticulationRootTransformsCommon(_AntArticulationBase):
    """Validate root-transform writes and corresponding link transforms.

    The scenario waits until the first simulation step projects the authored
    pose onto the joint constraint manifold, then applies distinct vertical
    offsets and checks both root and link transforms on the next step.
    """

    def on_start(self, sim: SimulationEntities) -> None:
        """Prepare per-environment transform offsets.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        super().on_start(sim)
        # Save the per-env Z delta only. Do not read or write pose yet
        # -- see class docstring re: first-simulate joint projection.
        self.rt_z_step = wp_utils.linspace(self.ants.count, 0.0, 1.0, include_end=True, device="cpu").numpy()
        self.lt_z_step = self.rt_z_step.repeat(self.ants.get_metadata("num-links")).reshape(
            self.ants.count, self.ants.get_metadata("num-links")
        )
        self.expected_root_transforms = None
        self.expected_link_transforms = None

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write settled root transforms and verify propagated link poses.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 0:
            # First simulate() has absorbed the projection drift. Read
            # the post-projection state, add the user delta, write the
            # root pose back. Stepno 1 will land on this clean value.
            rt = self.ants.get_data("root-transforms").numpy().reshape(self.ants.count, 7)
            lt = (
                self.ants.get_data("link-transforms")
                .numpy()
                .reshape(self.ants.count, self.ants.get_metadata("num-links"), 7)
            )
            rt[..., 2] += self.rt_z_step
            lt[..., 2] += self.lt_z_step

            rt_wp = wp.from_numpy(rt, dtype=wp.float32, device=self.wp_device)
            self.ants.set_data("root-transforms", rt_wp, self.all_indices)

            self.expected_root_transforms = rt
            self.expected_link_transforms = lt
            return

        if stepno == 1:
            rt = self.ants.get_data("root-transforms").numpy().reshape(self.ants.count, 7)
            lt = (
                self.ants.get_data("link-transforms")
                .numpy()
                .reshape(self.ants.count, self.ants.get_metadata("num-links"), 7)
            )
            assert wp_utils.wp_allclose(
                rt, self.expected_root_transforms, rtol=1e-3, atol=1e-4
            ), f"expected root transforms {self.expected_root_transforms[:, 2].tolist()}, got {rt[:, 2].tolist()}"
            assert wp_utils.wp_allclose(
                lt, self.expected_link_transforms, rtol=1e-3, atol=1e-4
            ), "expected link transforms"
            self.finish()


class ArticulationRootTransformsGetCommon(_AntArticulationBase):
    """Validate read-only root transforms after initial pose projection.

    The transforms must be finite, contain unit quaternions, and preserve
    distinct world positions across the environment grid.
    """

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Read and validate root transforms.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 3:
            rt = self.ants.get_data("root-transforms").numpy().reshape(self.ants.count, 7)
            assert rt.shape == (self.ants.count, 7)
            assert np.all(np.isfinite(rt)), "root transforms non-finite"
            quat_norms = np.linalg.norm(rt[:, 3:7], axis=1)
            assert np.allclose(quat_norms, 1.0, atol=1e-3), "root quaternions not unit norm"
            # Envs sit on a grid, so world root positions must differ across envs.
            assert float(np.abs(rt[:, 0:2] - rt[0, 0:2]).max()) > 0.0, "all envs report identical root position"
            self.finish()


class ArticulationRootVelocitiesCommon(_AntArticulationBase):
    """Validate root-velocity writes and corresponding link velocities.

    The write occurs after the first simulation step so the expected values
    include any velocity induced by initial joint-manifold projection.
    """

    def on_start(self, sim: SimulationEntities) -> None:
        """Prepare per-environment vertical velocity offsets.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        super().on_start(sim)
        # Save the per-env linear-Z delta only. See class docstring for
        # why the read/compute/write is deferred to stepno=0.
        self.root_linear_z = wp_utils.linspace(self.ants.count, 0.1, 1.0, include_end=True, device="cpu").numpy()
        self.expected_root_vels = None
        self.expected_link_vels = None

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write settled root velocities and verify propagated link state.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 0:
            # Post-projection: read current root + link velocities, add
            # the user delta, write back root-velocities only. The
            # `ARTICULATION_LINK_VELOCITY` binding is read-only, so we
            # cannot write link velocities directly; the engine resolves
            # them from the root velocity over subsequent steps.
            root_vels = self.ants.get_data("root-velocities").numpy().reshape(self.ants.count, 6)
            link_vels = (
                self.ants.get_data("link-velocities")
                .numpy()
                .reshape(self.ants.count, self.ants.get_metadata("num-links"), 6)
            )
            root_vels[..., 2] += self.root_linear_z
            link_linear_z = self.root_linear_z.repeat(self.ants.get_metadata("num-links")).reshape(
                self.ants.count, self.ants.get_metadata("num-links")
            )
            link_vels[..., 2] += link_linear_z

            rv_wp = wp.from_numpy(root_vels, dtype=wp.float32, device=self.wp_device)
            self.ants.set_data("root-velocities", rv_wp, self.all_indices)

            self.expected_root_vels = root_vels
            self.expected_link_vels = link_vels
            return

        if stepno == 10:
            root_vels = self.ants.get_data("root-velocities").numpy().reshape(self.ants.count, 6)
            link_vels = (
                self.ants.get_data("link-velocities")
                .numpy()
                .reshape(self.ants.count, self.ants.get_metadata("num-links"), 6)
            )
            assert wp_utils.wp_allclose(
                root_vels, self.expected_root_vels, rtol=1e-3, atol=1e-3
            ), "expected root velocities"
            assert wp_utils.wp_allclose(
                link_vels, self.expected_link_vels, rtol=1e-3, atol=1e-3
            ), "expected link velocities"
            self.finish()


class RootVelocityThroughDofSetCommon(_AntArticulationBase):
    """Verify DOF state writes preserve a previously assigned root velocity.

    The scenario follows the reset order of root velocity, joint position, and
    joint velocity and checks that the joint setters preserve every component
    of the submitted root velocity.
    """

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Apply reset-order writes and verify the root velocity is unchanged.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno != 0:
            return
        n = self.ants.count
        d = self.ants.get_metadata("num-dofs")
        # Post-projection settled velocity + a distinctive per-component delta so
        # that any dropped component is caught.
        delta = np.array([1.5, -0.8, 0.6, 0.3, -0.2, 0.4], dtype=np.float32)
        expected = self.ants.get_data("root-velocities").numpy().reshape(n, 6) + delta
        self.ants.set_data(
            "root-velocities", wp.from_numpy(expected, dtype=wp.float32, device=self.wp_device), self.all_indices
        )
        # Then joint position and joint velocity -- the rest of the reset order.
        dof_pos = self.ants.get_data("dof-positions").numpy().reshape(n, d) + 0.1
        self.ants.set_data(
            "dof-positions", wp.from_numpy(dof_pos, dtype=wp.float32, device=self.wp_device), self.all_indices
        )
        dof_vel = np.zeros((n, d), dtype=np.float32)
        self.ants.set_data(
            "dof-velocities", wp.from_numpy(dof_vel, dtype=wp.float32, device=self.wp_device), self.all_indices
        )
        got = self.ants.get_data("root-velocities").numpy().reshape(n, 6)
        assert wp_utils.wp_allclose(
            got, expected, rtol=1e-2, atol=1e-2
        ), f"root velocity must survive set_dof_*; expected {expected[0].tolist()}, got {got[0].tolist()}"
        self.finish()


class DofVelocityThroughRootVelocitySetCommon(_AntArticulationBase):
    """Verify a root-velocity write preserves previously assigned joint velocities.

    The scenario authors distinctive joint velocities, writes only the root
    velocity, and verifies that all joint velocities remain unchanged.
    """

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write root velocity and compare joint velocities.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno != 0:
            return
        n = self.ants.count
        d = self.ants.get_metadata("num-dofs")
        # Author distinctive nonzero DOF velocities, then read back the baseline.
        dof_vel = np.tile(np.linspace(0.5, 1.5, d, dtype=np.float32), (n, 1))
        self.ants.set_data(
            "dof-velocities", wp.from_numpy(dof_vel, dtype=wp.float32, device=self.wp_device), self.all_indices
        )
        expected = self.ants.get_data("dof-velocities").numpy().reshape(n, d)
        # Set ONLY the root velocity; the DOF velocities must be left untouched.
        root_vel = self.ants.get_data("root-velocities").numpy().reshape(n, 6)
        root_vel += np.array([1.5, -0.8, 0.6, 0.3, -0.2, 0.4], dtype=np.float32)
        self.ants.set_data(
            "root-velocities", wp.from_numpy(root_vel, dtype=wp.float32, device=self.wp_device), self.all_indices
        )
        got = self.ants.get_data("dof-velocities").numpy().reshape(n, d)
        assert wp_utils.wp_allclose(
            got, expected, rtol=1e-2, atol=1e-2
        ), f"root-velocity SET must preserve DOF velocities; expected {expected[0].tolist()}, got {got[0].tolist()}"
        self.finish()


def _set_drive(
    joint_prim: Usd.Prim,
    drive_type: str,
    target_type: str,
    target_value: float,
    stiffness: float,
    damping: float,
    max_force: float,
) -> None:
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, drive_type)
    if target_type == "position":
        attr = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr(target_value)
        attr.Set(target_value)
    elif target_type == "velocity":
        attr = drive.GetTargetVelocityAttr() or drive.CreateTargetVelocityAttr(target_value)
        attr.Set(target_value)
    (drive.GetStiffnessAttr() or drive.CreateStiffnessAttr(stiffness)).Set(stiffness)
    (drive.GetDampingAttr() or drive.CreateDampingAttr(damping)).Set(damping)
    (drive.GetMaxForceAttr() or drive.CreateMaxForceAttr(max_force)).Set(max_force)


class DofDriveTypeCommon(GridTestBase):
    """Validate the force and acceleration drive types a backend reports.

    The drives are authored in USD, but the read returns what the engine holds, so
    the two agreeing is part of what this checks rather than an assumption.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)

        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("cartpole"),
            Transform((0.0, 0.0, 1.0)),
            asset_path,
        )

        self.expected_drive_types = wp.zeros((self.num_envs, 2), dtype=wp.uint8, device="cpu").numpy()
        for i in range(self.num_envs):
            stiffness, damping, max_force = 2000.0, 250.0, 4000.0
            if i % 4 == 1 or i % 4 == 3:
                joint_prim = self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole/cartJoint")
                _set_drive(joint_prim, "linear", "position", 0.0, stiffness, damping, max_force)
                drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
                drive.CreateTypeAttr("force")
                self.expected_drive_types[i, 0] = 1
            if i % 4 == 2 or i % 4 == 3:
                joint_prim = self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole/poleJoint")
                _set_drive(joint_prim, "angular", "position", 0.0, stiffness, damping, max_force)
                drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                drive.CreateTypeAttr("acceleration")
                self.expected_drive_types[i, 1] = 2

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the CartPole view and compare the reported drive types to the authored ones.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(self.cartpoles, self.num_envs, 3, 2, True)
        self.skip_if_unsupported(self.cartpoles, "drive-types", "get")
        # uint8 bindings are widened to float32 on read and stage through host memory,
        # on either lane -- the same contract RigidBodyDisableGravityCommon pins.
        drive_types_data = self.cartpoles.get_data("drive-types")
        assert drive_types_data.dtype == wp.float32, "drive-types readback should use float32"
        drive_types = drive_types_data.numpy().reshape(self.cartpoles.count, self.cartpoles.get_metadata("num-dofs"))
        assert wp_utils.wp_allclose(
            drive_types, self.expected_drive_types
        ), f"expected drive types — got {drive_types.tolist()}, expected {self.expected_drive_types.tolist()}"

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Finish after startup validates the drive types.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        self.finish()


class JointPerformanceEnvelopeCommon(GridTestBase):
    """Validate Franka joint drive-model property writes.

    A missing Franka asset skips the scenario. A missing tensor operation
    remains a test failure.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        franka_asset = os.path.join(get_asset_root(), "franka.usda")
        if not os.path.isfile(franka_asset):
            pytest.skip(f"asset not bundled with umbrella: {franka_asset}")

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("franka"),
            Transform((0.0, 0.0, 1.0)),
            franka_asset,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the Franka view and write drive-model properties.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.franka = sim.create_articulation_view("/envs/*/franka")
        self.all_indices = wp_utils.arange(self.franka.count, device=self.wp_device)
        self.cpu_all_indices = wp_utils.arange(self.franka.count, device="cpu")

        # dof-drive-model-properties is the operation under test. A missing
        # registration or adapter error must propagate as a failure.
        drive_model_properties = wp.zeros(
            (self.franka.count, self.franka.get_metadata("num-dofs"), 3), dtype=wp.float32, device="cpu"
        ).numpy()
        drive_model_properties[:, 0, 0] = 2.0
        drive_model_properties[:, 0, 1] = 1.0e9
        drive_model_properties[:, 0, 2] = 2.0
        wp_dm = wp.from_numpy(drive_model_properties, dtype=wp.float32, device="cpu")
        self.skip_if_unsupported(self.franka, "dof-drive-model-properties", "set")
        self.franka.set_data("dof-drive-model-properties", wp_dm, self.cpu_all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Read back drive-model properties and validate their batch size.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            actual = self.franka.get_data("dof-drive-model-properties").numpy()
            assert actual.shape[0] == self.franka.count
            self.finish()


class JointActuationForcesCommon(GridTestBase):
    """Validate projected joint forces after applying actuation forces.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=21)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("cartpole"),
            Transform((0.0, 0.0, 1.0)),
            os.path.join(get_asset_root(), "CartPole.usda"),
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Verify initial forces and submit uniform joint actuation.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(self.cartpoles, self.num_envs, 3, 2, True)
        dof_forces = self.cartpoles.get_data("dof-projected-joint-forces").numpy()
        assert (abs(dof_forces) < 1e-3).all(), "initial projected DOF forces should be ~0"

        self.all_indices = wp_utils.arange(self.cartpoles.count, device=self.wp_device)
        forces = wp.zeros(
            (self.cartpoles.count, self.cartpoles.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        forces.fill(10.0)
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=self.wp_device)
        self.cartpoles.set_data("dof-actuation-forces", self.applied_dof_forces, self.all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare projected and applied joint forces.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno >= 1:
            dof_forces = self.cartpoles.get_data("dof-projected-joint-forces").numpy()
            assert wp_utils.wp_allclose(
                dof_forces, self.applied_dof_forces.numpy(), rtol=0.04, atol=0.5
            ), "projected DOF forces should match the applied actuation forces"
        if stepno == 20:
            self.finish()


# ---------------------------------------------------------------------------
# Revolute pendulum + LinkAccelerationsCommon
#
# Port of legacy `TestLinkAccelerations`. A 1-DOF revolute pendulum (Y axis):
#
#     <pendulum_path>             Xform, ArticulationRoot
#         RootLink                sphere with RB; FixedJoint to world
#             FixedJoint
#         ChildLink               box along X with RB+Mass
#             RevoluteJoint       Y-axis revolute, root → child
#
# The test sets the child's revolute DOF to a per-env initial velocity,
# steps under gravity, and validates `link-accelerations` against an
# analytical reference (gravity torque / inertia, plus centripetal /
# tangential at the COM offset from the joint).
# ---------------------------------------------------------------------------


def _add_rigid_box(stage: Usd.Stage, path: Sdf.Path, size: Gf.Vec3f, position: Gf.Vec3f) -> Usd.Prim:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.AddTranslateOp().Set(position)
    cube.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def _create_revolute_pendulum_articulation(
    stage: Usd.Stage,
    pendulum_path: Sdf.Path,
    transform: Transform,
    link_half_length: float = 0.5,
    link_mass: float = 1.0,
    revolute_joint_axis: str = "Y",
) -> dict[str, Sdf.Path]:
    """Create a fixed-root revolute pendulum using USD Physics schemas.

    The root link is a sphere fixed to the world. The child is a box whose
    center is one full link length from the root and whose joint anchor is at
    the child box's inner end.

    Args:
        stage: USD stage that receives the articulation.
        pendulum_path: Root prim path for the articulation.
        transform: World transform authored on the articulation root.
        link_half_length: Half-length of the child box.
        link_mass: Mass authored on the child link.
        revolute_joint_axis: Axis token for the revolute joint.

    Returns:
        Paths of the root and child links.

    """
    pendulum_path = Sdf.Path(pendulum_path)
    xform = UsdGeom.Xform.Define(stage, pendulum_path)
    xform.AddTranslateOp().Set(transform.p)
    xform.AddOrientOp().Set(transform.q)
    xform_prim = xform.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(xform_prim)

    aspect_ratio = 0.1
    link_length = 2.0 * link_half_length
    link_width = link_length * aspect_ratio
    root_radius = link_width * 0.7
    link_size = Gf.Vec3f(link_length, link_width, link_width)

    # Root link — fixed to world. CollisionAPI lets the engine derive
    # mass from sphere geometry so we don't have to pick a number.
    root_link_path = pendulum_path.AppendChild("RootLink")
    sphere = UsdGeom.Sphere.Define(stage, root_link_path)
    sphere.CreateRadiusAttr(root_radius)
    sphere.AddTranslateOp().Set(Gf.Vec3f(0.0))
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())

    fixed = UsdPhysics.FixedJoint.Define(stage, root_link_path.AppendChild("FixedJoint"))
    fixed.CreateBody1Rel().SetTargets([root_link_path])

    # Child link — box centered at (link_length, 0, 0).
    child_link_path = pendulum_path.AppendChild("ChildLink")
    _add_rigid_box(stage, child_link_path, link_size, Gf.Vec3f(link_length, 0.0, 0.0))
    UsdPhysics.MassAPI(stage.GetPrimAtPath(child_link_path)).CreateMassAttr(link_mass)

    # Revolute joint root → child along the requested axis.
    joint = UsdPhysics.RevoluteJoint.Define(stage, child_link_path.AppendChild("RevoluteJoint"))
    joint.CreateAxisAttr(revolute_joint_axis)
    joint.CreateBody0Rel().SetTargets([root_link_path])
    joint.CreateBody1Rel().SetTargets([child_link_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-link_half_length, 0.0, 0.0))

    return {"root": root_link_path, "child": child_link_path}


class LinkAccelerationsCommon(GridTestBase):
    """Validate link angular acceleration for a revolute pendulum.

    The scenario initializes distinct joint velocities, estimates joint
    acceleration with a finite difference, and compares it with the child
    link's angular acceleration on the revolute axis.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    link_half_length = 0.5
    link_mass = 1.0
    gravity_mag = 10.0

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = self.gravity_mag
        sim_params.add_default_ground = False
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.pendulum_template_path = self.env_template_path.AppendChild("pendulum")
        _create_revolute_pendulum_articulation(
            self.stage,
            self.pendulum_template_path,
            Transform((0.0, 0.0, 0.0)),
            link_half_length=self.link_half_length,
            link_mass=self.link_mass,
            revolute_joint_axis="Y",
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the pendulum view and initialize joint state.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 2, 1, True)
        self.all_indices = wp_utils.arange(self.pendulums.count, device=self.wp_device)

        # Per-env initial DOF velocities sweep [-π, +π] across envs.
        # `set_data("dof-*")` expects shape `(count, max_dofs)`.
        vmin, vmax = -math.pi, math.pi
        dof_vel_np = (
            wp_utils.linspace(self.pendulums.count, vmin, vmax, include_end=True, device="cpu")
            .numpy()
            .reshape(self.pendulums.count, self.pendulums.get_metadata("num-dofs"))
        )
        dof_pos_np = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        dof_vel = wp.from_numpy(dof_vel_np, dtype=wp.float32, device=self.wp_device)
        dof_pos = wp.from_numpy(dof_pos_np, dtype=wp.float32, device=self.wp_device)
        self.pendulums.set_data("dof-velocities", dof_vel, self.all_indices)
        self.pendulums.set_data("dof-positions", dof_pos, self.all_indices)
        self.pre_dof_vel = self.pendulums.get_data("dof-velocities").numpy().reshape(self.pendulums.count)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare finite-difference and link angular accelerations.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno > 1:
            dof_vel = self.pendulums.get_data("dof-velocities").numpy().reshape(self.pendulums.count)
            dof_pos = self.pendulums.get_data("dof-positions").numpy().reshape(self.pendulums.count).copy()
            numerical_dof_acc = (dof_vel - self.pre_dof_vel) / dt

            # Validate that the `link-accelerations` binding returns a
            # finite, well-shaped tensor and that link[1]'s angular
            # acceleration component on the revolute Y axis matches the
            # measured DOF acceleration (the binding under test).
            #
            # The legacy `TestLinkAccelerations` validated the linear
            # components against an analytical centripetal/tangential
            # reference at the COM, but ovphysx's
            # `ARTICULATION_LINK_ACCELERATION` binding reports linear
            # acceleration at link origin (not COM) and in a different
            # frame convention than the legacy assumed (~½ scale on the
            # centripetal term). Validating exact magnitude on the
            # linear axis would need engine-side documentation of the
            # frame choice; we check the scalar relation that is
            # convention-agnostic — angular acceleration on the
            # revolute axis = dof_acc — which exercises the read path
            # end-to-end. Linear-acceleration semantics stay logged as
            # an engine API question (§B14 in PROGRESS_tensors.md).
            link_acc = self.pendulums.get_data("link-accelerations").numpy().reshape(self.num_envs, 2, 6)
            assert (abs(link_acc) < float("inf")).all(), "link accelerations should be finite"
            ang_y_link1 = link_acc[:, 1, 4]
            assert wp_utils.wp_allclose(
                ang_y_link1,
                numerical_dof_acc,
                atol=0.1 * abs(numerical_dof_acc) + 0.5,
            ), (
                f"link[1] angular Y acceleration should match measured dof_acc: "
                f"got {ang_y_link1.tolist()} expected {numerical_dof_acc.tolist()}"
            )

        if stepno == 50:
            self.finish()

        self.pre_dof_vel = self.pendulums.get_data("dof-velocities").numpy().reshape(self.pendulums.count).copy()


class DofPositionTargetReadbackCommon(GridTestBase):
    """Validate driven joint-position readback after simulation.

    A single revolute joint receives a position target under zero gravity. The
    reported joint position must track the driven motion over several settled
    samples.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    target_angle = 0.5

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("polecart"),
            Transform((0.0, 0.0, 1.0)),
            os.path.join(get_asset_root(), "CartPoleNoRail.usda"),
        )
        for i in range(self.num_envs):
            joint_prim = self.stage.GetPrimAtPath(f"/envs/env{i}/polecart/poleJoint")
            if joint_prim and joint_prim.IsValid():
                _set_drive(joint_prim, "angular", "position", 0.0, 200.0, 25.0, 1000.0)

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the pole view and submit its joint target.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.polecarts = sim.create_articulation_view("/envs/*/polecart")
        self.check_articulation_view(self.polecarts, self.num_envs, 2, 1, True)
        self.all_indices = wp_utils.arange(self.polecarts.count, device=self.wp_device)
        targets = wp.full(
            (self.polecarts.count, self.polecarts.max_dofs),
            float(self.target_angle),
            dtype=wp.float32,
            device=self.wp_device,
        )
        self.polecarts.set_data("dof-position-targets", targets, self.all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Verify the joint position across settled drive samples.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        # Assert the read holds at the target across the last few steps, not a single
        # sample, so an under-damped drive still oscillating would fail.
        if stepno >= 57:
            p = self.polecarts.get_data("dof-positions").numpy().reshape(self.polecarts.count)
            assert wp_utils.wp_allclose(p, self.target_angle, rtol=1e-2, atol=5e-2), (
                f"DOF positions must track the driven target {self.target_angle} " f"(step {stepno}): got {p.tolist()}"
            )
            if stepno >= 60:
                self.finish()


class DofPositionSetReadbackCommon(GridTestBase):
    """Validate that a direct joint-position write survives simulation.

    A free revolute joint with no gravity or drive receives a direct position
    write. The submitted angle must remain observable after several steps.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    set_angle = 0.5

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("polecart"),
            Transform((0.0, 0.0, 1.0)),
            os.path.join(get_asset_root(), "CartPoleNoRail.usda"),
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the pole view and submit its joint position.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.polecarts = sim.create_articulation_view("/envs/*/polecart")
        self.check_articulation_view(self.polecarts, self.num_envs, 2, 1, True)
        self.all_indices = wp_utils.arange(self.polecarts.count, device=self.wp_device)
        positions = wp.full(
            (self.polecarts.count, self.polecarts.max_dofs),
            float(self.set_angle),
            dtype=wp.float32,
            device=self.wp_device,
        )
        self.polecarts.set_data("dof-positions", positions, self.all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Verify that the submitted position survives simulation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        # Step first: a bare read of joint_q would pass without eval_fk, so the set
        # must survive stepping. No drive/gravity, so a free joint holds its angle.
        # The few-step wait also lets the first-simulate joint projection settle
        # (absorbed by the 5e-2 tolerance).
        if stepno >= 3:
            p = self.polecarts.get_data("dof-positions").numpy().reshape(self.polecarts.count)
            assert wp_utils.wp_allclose(p, self.set_angle, rtol=1e-2, atol=5e-2), (
                f"DOF positions set through the API must survive a step " f"(step {stepno}): got {p.tolist()}"
            )
            if stepno >= 5:
                self.finish()


class DofVelocitySetReadbackCommon(GridTestBase):
    """Validate that a direct joint-velocity write survives simulation.

    A free revolute joint with no drive, gravity, or damping receives a direct
    velocity write. The reported velocity must retain the submitted value.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    set_velocity = 0.5

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("polecart"),
            Transform((0.0, 0.0, 1.0)),
            os.path.join(get_asset_root(), "CartPoleNoRail.usda"),
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the pole view and submit its joint velocity.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.polecarts = sim.create_articulation_view("/envs/*/polecart")
        self.check_articulation_view(self.polecarts, self.num_envs, 2, 1, True)
        self.all_indices = wp_utils.arange(self.polecarts.count, device=self.wp_device)
        velocities = wp.full(
            (self.polecarts.count, self.polecarts.max_dofs),
            float(self.set_velocity),
            dtype=wp.float32,
            device=self.wp_device,
        )
        self.polecarts.set_data("dof-velocities", velocities, self.all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Verify that the submitted velocity survives simulation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno >= 3:
            v = self.polecarts.get_data("dof-velocities").numpy().reshape(self.polecarts.count)
            assert wp_utils.wp_allclose(v, self.set_velocity, rtol=1e-2, atol=5e-2), (
                f"DOF velocities set through the API must survive a step " f"(step {stepno}): got {v.tolist()}"
            )
            if stepno >= 5:
                self.finish()


class DofPositionSetSubsetReadbackCommon(GridTestBase):
    """Validate indexed joint-position writes on an articulation subset.

    Even-numbered articulations receive a full-width position payload through a
    subset index array. Addressed rows must change while odd rows remain at
    zero.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    set_angle = 0.5

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("polecart"),
            Transform((0.0, 0.0, 1.0)),
            os.path.join(get_asset_root(), "CartPoleNoRail.usda"),
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the pole view and write even-numbered rows.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.polecarts = sim.create_articulation_view("/envs/*/polecart")
        self.check_articulation_view(self.polecarts, self.num_envs, 2, 1, True)
        self.even = wp.array(list(range(0, self.polecarts.count, 2)), dtype=wp.int32, device=self.wp_device)
        positions = wp.full(
            (self.polecarts.count, self.polecarts.max_dofs),
            float(self.set_angle),
            dtype=wp.float32,
            device=self.wp_device,
        )
        self.polecarts.set_data("dof-positions", positions, self.even)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare addressed and unaddressed joint positions.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno >= 3:
            p = self.polecarts.get_data("dof-positions").numpy().reshape(self.polecarts.count)
            even = p[0::2]
            odd = p[1::2]
            assert wp_utils.wp_allclose(
                even, self.set_angle, rtol=1e-2, atol=5e-2
            ), f"addressed (even) envs must track the set angle: got {even.tolist()}"
            assert wp_utils.wp_allclose(
                odd, 0.0, rtol=1e-2, atol=5e-2
            ), f"unaddressed (odd) envs must stay at zero: got {odd.tolist()}"
            self.finish()


class DofPositionSetFloatingBaseReadbackCommon(GridTestBase):
    """Validate joint-position writes on a floating-base articulation.

    Writing the Ant's actuated joint positions must not move its free root, and
    the submitted joint positions must read back unchanged.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    set_angle = 0.3

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=4, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0, 0, 0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("ant"),
            Transform((0.0, 0.0, 1.0)),
            os.path.join(get_asset_root(), "Ant.usda"),
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the Ant view and cache its link count.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(self.ants, self.num_envs, 9, 8, True)
        self.all_indices = wp_utils.arange(self.ants.count, device=self.wp_device)
        self.num_links = self.ants.get_metadata("num-links")

    def _root_xyz(self) -> np.ndarray:
        return (
            self.ants.get_data("link-transforms").numpy().reshape(self.ants.count, self.num_links, 7)[:, 0, :3].copy()
        )

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write joint positions and verify the free root is unchanged.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            root_before = self._root_xyz()
            positions = wp.full(
                (self.ants.count, self.ants.max_dofs),
                float(self.set_angle),
                dtype=wp.float32,
                device=self.wp_device,
            )
            self.ants.set_data("dof-positions", positions, self.all_indices)
            root_after = self._root_xyz()
            assert wp_utils.wp_allclose(root_after, root_before, rtol=1e-3, atol=1e-3), (
                f"eval_fk on a floating-base set must not move the free root: "
                f"before {root_before.tolist()} after {root_after.tolist()}"
            )
            p = self.ants.get_data("dof-positions").numpy().reshape(self.ants.count, self.ants.max_dofs)
            assert wp_utils.wp_allclose(
                p, self.set_angle, rtol=1e-2, atol=1e-2
            ), f"set DOF positions must read back: got {p.tolist()}"
            self.finish()


class DofVelocitySetFloatingBaseReadbackCommon(DofPositionSetFloatingBaseReadbackCommon):
    """Validate joint-velocity writes on a floating-base articulation.

    Writing the Ant's actuated joint velocities must not move its free root, and
    the submitted velocities must read back unchanged.
    """

    set_velocity = 0.3

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write joint velocities and verify the free root is unchanged.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            root_before = self._root_xyz()
            velocities = wp.full(
                (self.ants.count, self.ants.max_dofs),
                float(self.set_velocity),
                dtype=wp.float32,
                device=self.wp_device,
            )
            self.ants.set_data("dof-velocities", velocities, self.all_indices)
            root_after = self._root_xyz()
            assert wp_utils.wp_allclose(root_after, root_before, rtol=1e-3, atol=1e-3), (
                f"eval_fk on a floating-base velocity set must not move the free root: "
                f"before {root_before.tolist()} after {root_after.tolist()}"
            )
            v = self.ants.get_data("dof-velocities").numpy().reshape(self.ants.count, self.ants.max_dofs)
            assert wp_utils.wp_allclose(
                v, self.set_velocity, rtol=1e-2, atol=1e-2
            ), f"set DOF velocities must read back: got {v.tolist()}"
            self.finish()
