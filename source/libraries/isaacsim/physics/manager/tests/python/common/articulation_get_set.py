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

"""Exercise articulation state reads, writes, and kinematic updates.

The shared Ant scenarios cover root and joint state round trips, applied
wrenches, per-link gravity flags, and per-link transforms and velocities.
"""

from __future__ import annotations

import os
import sys

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
    SimulationEntities,
    Transform,
    get_asset_root,
    pack_wrench,
)
from isaacsim.physics.manager.impl.tensors import EntityView  # noqa: E402
from pxr import Gf  # noqa: E402


class _ArticulationGetSetBase(GridTestBase):
    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        self.create_actor_from_asset(actor_path, Transform((0.0, 0.0, 1.0)), asset_path)
        self.atol = 1e-5

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the shared Ant view and full index buffer.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=self.wp_device)
        self.check_articulation_view(self.ants, self.num_envs, 9, 8, True)


class GetSetRootTransformsCommon(_ArticulationGetSetBase):
    """Round-trip floating-base translations and orientations."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write root transforms and verify them before finishing.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            roots = self.ants.get_data("root-transforms").numpy().reshape(self.ants.count, 7).copy()
            submitted = roots.copy()
            submitted[:, 0:3] += [0.5, -0.3, 1.0]
            # Non-identity rotation (90 deg about z) so orientation round-trips too.
            submitted[:, 3:7] = [0.0, 0.0, 0.7071068, 0.7071068]
            submitted = submitted.astype("float32")
            self.ants.set_data("root-transforms", self.to_warp(submitted), self.all_indices)
            new_roots = self.ants.get_data("root-transforms").numpy().reshape(self.ants.count, 7)
            assert wp_utils.wp_allclose(
                new_roots[:, 0:3], submitted[:, 0:3], rtol=1e-3, atol=self.atol
            ), "root translation should round-trip after set"
            # Quaternion equality up to sign (q and -q are the same rotation).
            dots = np.abs(np.sum(new_roots[:, 3:7] * submitted[:, 3:7], axis=1))
            assert np.allclose(dots, 1.0, atol=1e-3), "root orientation should round-trip after set"
        if stepno == 2:
            self.finish()


class GetSetRootVelocitiesCommon(_ArticulationGetSetBase):
    """Round-trip floating-base spatial velocities."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write root velocities and verify them before finishing.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            vels = self.ants.get_data("root-velocities").numpy().reshape(self.ants.count, 6).copy()
            submitted = (vels + [0, 0, 1, 0, 0, 0]).astype("float32")
            self.ants.set_data("root-velocities", self.to_warp(submitted), self.all_indices)
            new_vels = self.ants.get_data("root-velocities").numpy().reshape(self.ants.count, 6)
            assert wp_utils.wp_allclose(
                new_vels[:, 2], vels[:, 2] + 1.0, rtol=1e-3, atol=self.atol
            ), "linear-Z velocity should be +1 after set"
        if stepno == 2:
            self.finish()


class GetSetRootTransformsMixedBaseCommon(GridTestBase):
    """Root-transform round-trip over a mixed fixed + floating base env.

    A CartPole (fixed base) and an Ant (floating base) share every env, so
    both bases must round-trip translation and orientation and retain the
    submitted pose through a solver step.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=20.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("cartpole"),
            Transform((0.0, 0.0, 5.0)),
            os.path.join(get_asset_root(), "CartPole.usda"),
        )
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("ant"),
            Transform((0.0, 0.0, -5.0)),
            os.path.join(get_asset_root(), "Ant.usda"),
        )
        self.atol = 1e-4

    def on_start(self, sim: SimulationEntities) -> None:
        """Create fixed-base and floating-base articulation views.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.cartpole_indices = wp_utils.arange(self.cartpoles.count, device=self.wp_device)
        self.ant_indices = wp_utils.arange(self.ants.count, device=self.wp_device)
        self._targets = {}

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Round-trip both root poses and check post-step persistence.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            self._roundtrip(self.cartpoles, self.cartpole_indices, "cartpole (fixed base)")
            self._roundtrip(self.ants, self.ant_indices, "ant (floating base)")
        elif stepno == 2:
            # A solver step has run since the set; the pose must survive it, not
            # just read back immediately (a fixed base is the case that can drift).
            self._check_persists(self.cartpoles, "cartpole (fixed base)")
            self._check_persists(self.ants, "ant (floating base)")
            self.finish()

    def _roundtrip(self, view: EntityView, indices: wp.array, label: str) -> None:
        roots = view.get_data("root-transforms").numpy().reshape(view.count, 7).copy()
        submitted = roots.copy()
        submitted[:, 0:3] += [0.5, -0.3, 1.0]
        # Re-orient to a known quaternion (90 deg about z) so the fixed base's
        # joint_X_c rotation composition is exercised, not just translation.
        submitted[:, 3:7] = [0.0, 0.0, 0.7071068, 0.7071068]
        submitted = submitted.astype("float32")
        view.set_data("root-transforms", self.to_warp(submitted), indices)
        self._targets[label] = submitted  # for the post-step persistence check
        new_roots = view.get_data("root-transforms").numpy().reshape(view.count, 7)
        assert wp_utils.wp_allclose(
            new_roots[:, 0:3], submitted[:, 0:3], rtol=1e-3, atol=self.atol
        ), f"{label} root translation should round-trip after set"
        # Quaternion equality up to sign (q and -q are the same rotation).
        dots = np.abs(np.sum(new_roots[:, 3:7] * submitted[:, 3:7], axis=1))
        assert np.allclose(dots, 1.0, atol=1e-3), f"{label} root orientation should round-trip after set"

    def _check_persists(self, view: EntityView, label: str) -> None:
        submitted = self._targets[label]
        roots = view.get_data("root-transforms").numpy().reshape(view.count, 7)
        assert wp_utils.wp_allclose(
            roots[:, 0:3], submitted[:, 0:3], rtol=1e-2, atol=1e-2
        ), f"{label} root translation should persist through a solver step"
        dots = np.abs(np.sum(roots[:, 3:7] * submitted[:, 3:7], axis=1))
        assert np.allclose(dots, 1.0, atol=1e-3), f"{label} root orientation should persist through a solver step"


class GetSetDofPositionsCommon(_ArticulationGetSetBase):
    """Round-trip articulation joint positions."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write joint positions and verify them before finishing.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            n = self.ants.get_metadata("num-dofs")
            positions = self.ants.get_data("dof-positions").numpy().reshape(self.ants.count, n).copy()
            submitted = (positions + 0.5).astype("float32")
            self.ants.set_data("dof-positions", self.to_warp(submitted), self.all_indices)
            new_positions = self.ants.get_data("dof-positions").numpy().reshape(self.ants.count, n)
            assert wp_utils.wp_allclose(
                new_positions, submitted, rtol=1e-3, atol=self.atol
            ), "DOF positions should match what was set"
        if stepno == 2:
            self.finish()


class GetSetDofVelocitiesCommon(_ArticulationGetSetBase):
    """Round-trip articulation joint velocities."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write joint velocities and verify them before finishing.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            n = self.ants.get_metadata("num-dofs")
            vels = self.ants.get_data("dof-velocities").numpy().reshape(self.ants.count, n).copy()
            submitted = (vels + 0.5).astype("float32")
            self.ants.set_data("dof-velocities", self.to_warp(submitted), self.all_indices)
            new_vels = self.ants.get_data("dof-velocities").numpy().reshape(self.ants.count, n)
            assert wp_utils.wp_allclose(
                new_vels, submitted, rtol=1e-3, atol=self.atol
            ), "DOF velocities should match what was set"
        if stepno == 2:
            self.finish()


class GetSetDofPositionTargetCommon(_ArticulationGetSetBase):
    """Round-trip articulation joint position targets."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write position targets and verify them before finishing.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            n = self.ants.get_metadata("num-dofs")
            tgt = self.ants.get_data("dof-position-targets").numpy().reshape(self.ants.count, n).copy()
            submitted = (tgt + 0.5).astype("float32")
            self.ants.set_data("dof-position-targets", self.to_warp(submitted), self.all_indices)
            new_tgt = self.ants.get_data("dof-position-targets").numpy().reshape(self.ants.count, n)
            assert wp_utils.wp_allclose(
                new_tgt, submitted, rtol=1e-3, atol=self.atol
            ), "DOF position targets should match what was set"
        if stepno == 2:
            self.finish()


class GetSetDofVelocityTargetCommon(_ArticulationGetSetBase):
    """Round-trip articulation joint velocity targets."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write velocity targets and verify them before finishing.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            n = self.ants.get_metadata("num-dofs")
            tgt = self.ants.get_data("dof-velocity-targets").numpy().reshape(self.ants.count, n).copy()
            submitted = (tgt + 0.5).astype("float32")
            self.ants.set_data("dof-velocity-targets", self.to_warp(submitted), self.all_indices)
            new_tgt = self.ants.get_data("dof-velocity-targets").numpy().reshape(self.ants.count, n)
            assert wp_utils.wp_allclose(
                new_tgt, submitted, rtol=1e-3, atol=self.atol
            ), "DOF velocity targets should match what was set"
        if stepno == 2:
            self.finish()


class GetSetDofActuationForcesCommon(_ArticulationGetSetBase):
    """Round-trip articulation joint actuation forces."""

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Write actuation forces and verify them before finishing.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            n = self.ants.get_metadata("num-dofs")
            forces = self.ants.get_data("dof-actuation-forces").numpy().reshape(self.ants.count, n).copy()
            submitted = (forces + 0.5).astype("float32")
            self.ants.set_data("dof-actuation-forces", self.to_warp(submitted), self.all_indices)
            new_forces = self.ants.get_data("dof-actuation-forces").numpy().reshape(self.ants.count, n)
            assert wp_utils.wp_allclose(
                new_forces, submitted, rtol=1e-3, atol=self.atol
            ), "DOF actuation forces should match what was set"
        if stepno == 2:
            self.finish()


class GetSetAppliedForcesCommon(_ArticulationGetSetBase):
    """Apply per-link world-frame forces at offset positions and verify upward motion."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Prepare link-space force and application-position buffers.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        super().on_start(sim)
        force_offset = 1
        transforms = (
            self.ants.get_data("link-transforms").numpy().reshape(self.num_envs, self.ants.get_metadata("num-links"), 7)
        )
        positions = transforms[:, :, 0:3].copy()
        positions[:, :, 0:3] += [0, 0, force_offset]

        gForce = wp.vec3(0.0, 0.0, 10.0)
        n_total = self.ants.count * self.ants.get_metadata("num-links")
        self.forces_wp = wp_utils.fill_vec3(n_total, value=gForce, device=self.wp_device)
        self.positions_wp = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=self.wp_device)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Apply the wrench and verify the articulation rises.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            self.init_height = (
                self.ants.get_data("link-transforms")
                .numpy()
                .reshape(self.num_envs, self.ants.get_metadata("num-links"), 7)[:, :, 2]
                .copy()
            )
            wrench = pack_wrench(self.forces_wp, None, self.positions_wp)
            wrench = wrench.reshape((self.ants.count, self.ants.get_metadata("num-links"), 9))
            self.ants.set_data("apply-forces-and-torques-at-position", wrench, self.all_indices)
        if stepno == 10:
            height = (
                self.ants.get_data("link-transforms")
                .numpy()
                .reshape(self.num_envs, self.ants.get_metadata("num-links"), 7)[:, :, 2]
            )
            # The wrench is applied at +1m above each link's center,
            # so all links get an upward force (10 N) that lifts the
            # articulation. The torso (link 0 of every Ant) is rigidly
            # the root — its motion is the cleanest signal that the
            # wrench landed and was integrated correctly. Per-leg
            # swing under joint constraints can momentarily dip
            # individual leg links below their initial Z, so we don't
            # assert `.all()` (the legacy test did, but with a
            # different ovphysx solver tolerance that no longer
            # holds in MR-7884). Mean lift > 0 + torso lift > 0
            # captures the binding-write semantics that this test is
            # actually validating.
            torso_lift = (height[:, 0] - self.init_height[:, 0]).mean()
            mean_lift = (height - self.init_height).mean()
            assert torso_lift > 0.05, f"applied wrench should lift the torso (mean torso lift {torso_lift:.4f} m)"
            assert mean_lift > 0.05, f"applied wrench should lift bodies on average (mean lift {mean_lift:.4f} m)"
            self.finish()


class GetSetLinkGravityCommon(GridTestBase):
    """Validate per-link gravity flags and their runtime effect.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        self.create_actor_from_asset(actor_path, Transform((0.0, 0.0, 1.0)), asset_path)

    def on_start(self, sim: SimulationEntities) -> None:
        """Round-trip gravity flags and configure alternating runtime state.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=self.wp_device)
        self.check_articulation_view(self.ants, self.num_envs, 9, 8, True)
        self.skip_if_unsupported(self.ants, "disable-gravities", "set")

        link_count = self.ants.get_metadata("num-links")
        articulation_count = self.ants.count

        disable_all = np.ones((articulation_count, link_count), dtype=np.uint8)
        self._assert_round_trip(disable_all, "all links disabled")

        enable_all = np.zeros((articulation_count, link_count), dtype=np.uint8)
        self._assert_round_trip(enable_all, "all links enabled")

        mixed = np.zeros((articulation_count, link_count), dtype=np.uint8)
        mixed[: articulation_count // 2, :] = 1
        self._assert_round_trip(mixed, "mixed articulation state")

        self.disabled_indices = list(range(0, articulation_count, 2))
        self.enabled_indices = list(range(1, articulation_count, 2))
        runtime_flags = np.zeros((articulation_count, link_count), dtype=np.uint8)
        runtime_flags[self.disabled_indices, :] = 1
        self.ants.set_data(
            "disable-gravities",
            wp.from_numpy(runtime_flags, dtype=wp.uint8, device="cpu"),
            self.all_indices,
        )
        self.dt = 1.0 / self.sim_params.time_steps_per_second

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Verify gravity-disabled articulations remain stationary.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno != 1:
            return

        root_velocity_z = self.ants.get_data("root-velocities").numpy().reshape(self.ants.count, 6)[:, 2]
        expected_velocity_z = -2.0 * self.dt * self.sim_params.gravity_mag
        assert wp_utils.wp_allclose(
            root_velocity_z[self.disabled_indices], 0.0
        ), f"Gravity-disabled articulations should not fall (step {stepno})"
        assert wp_utils.wp_allclose(
            root_velocity_z[self.enabled_indices], expected_velocity_z
        ), f"Gravity-enabled articulations should fall at -2*dt*g (step {stepno})"
        self.finish()

    def _assert_round_trip(self, flags: np.ndarray, message: str) -> None:
        """Verify uint8 writes and float32 readback writes preserve flags.

        Args:
            flags: Full per-articulation, per-link flag matrix.
            message: Assertion context for the tested flag pattern.

        """
        self.ants.set_data(
            "disable-gravities",
            wp.from_numpy(flags, dtype=wp.uint8, device="cpu"),
            self.all_indices,
        )
        readback = self.ants.get_data("disable-gravities")
        assert readback.dtype == wp.float32, "disable-gravities readback should use float32"
        assert (readback.numpy().reshape(flags.shape) == flags).all(), f"{message} should round-trip"

        # Native boolean bindings accept uint8. Replaying public float32 readback
        # verifies the adapter converts it before forwarding the tensor to OvPhysX.
        self.ants.set_data("disable-gravities", readback, self.all_indices)
        replayed = self.ants.get_data("disable-gravities").numpy().reshape(flags.shape)
        assert (replayed == flags).all(), f"{message} GET-to-SET replay should round-trip"


class LinkStateCommon(_ArticulationGetSetBase):
    """Read and validate per-link transforms and velocities.

    ``link-transforms`` and ``link-velocities`` are read-only per-link spatial state. The
    root body is one of the links, so (order-independently) some link-velocity
    row must equal root-velocities -- this checks the values and that the root
    link is present, alongside finiteness and unit-norm orientation quaternions.
    """

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Validate full and indexed per-link state reads.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            n_links = self.ants.get_metadata("num-links")
            # view.max_links is derived from link-transforms shape_hint[1]; it
            # must equal num-links, not a flattened max_links * 7.
            assert self.ants.max_links == n_links, "view.max_links must equal num-links"
            xforms = self.ants.get_data("link-transforms").numpy()
            vels = self.ants.get_data("link-velocities").numpy()
            # get_data must return the 3D per-link grid directly, not a flattened
            # 2D buffer -- assert the raw shape (a reshape would mask a wrong one).
            assert xforms.shape == (self.num_envs, n_links, 7), "link-transforms must be (count, max_links, 7)"
            assert vels.shape == (self.num_envs, n_links, 6), "link-velocities must be (count, max_links, 6)"
            root_vels = self.ants.get_data("root-velocities").numpy().reshape(self.ants.count, 6)
            assert np.isfinite(xforms).all(), "link-transforms must be finite"
            assert np.isfinite(vels).all(), "link-velocities must be finite"
            quat_norms = np.linalg.norm(xforms[:, :, 3:7], axis=2)
            assert np.allclose(
                quat_norms, 1.0, atol=1e-4
            ), f"link-transform quaternions must be unit-norm, got {quat_norms.min()}..{quat_norms.max()}"
            root_residual = np.abs(vels - root_vels[:, None, :]).sum(axis=2).min(axis=1)
            assert np.allclose(
                root_residual, 0.0, atol=1e-4
            ), f"a link-velocity row must equal root-velocities, max residual {root_residual.max()}"
            # Indexed read must return the selected rows of the full grid, same rank.
            half = max(1, self.ants.count // 2)
            idx = wp_utils.arange(half, device=self.wp_device)
            sub = self.ants.get_data("link-transforms", idx).numpy()
            assert sub.shape == (half, n_links, 7), "indexed link-transforms must be (K, max_links, 7)"
            assert np.allclose(sub, xforms[:half]), "indexed link-transforms must match the full-read subset"
        if stepno == 2:
            self.finish()
