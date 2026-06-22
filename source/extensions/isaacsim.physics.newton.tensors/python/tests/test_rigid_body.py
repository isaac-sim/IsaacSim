# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Validate Newton rigid body tensor views against omni.physics.tensors contracts.

The tests cover view counts and paths, body mass/COM/inertia properties,
transform and velocity get-set behavior including indexed writes,
accelerations backed by ``body_qdd``, external force/torque integration, and
the required ``[linear(3), angular(3)]`` spatial vector layout.
"""

from __future__ import annotations

import numpy as np
import omni.kit.app
import omni.physics.tensors as tensors
import warp as wp
from pxr import Gf

from .test_helpers import NewtonTensorTestBase, run_on_device_configs, warp_utils

# ---------------------------------------------------------------------------
# TestRigidBodyView
#   Basic creation, count, prim paths.
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestRigidBodyView(NewtonTensorTestBase):
    """Rigid body view: creation, counts, paths."""

    async def test_rigid_body_view_counts(self) -> None:
        """Verify a wildcard ball view reports one rigid body per environment."""
        num_envs = self.setup_ball_grid()
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, num_envs)

    async def test_rigid_body_view_paths(self) -> None:
        """Verify expanded rigid body view paths include every environment ball."""
        num_envs = self.setup_ball_grid()
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.assertEqual(len(balls.prim_paths), num_envs)
        for i in range(num_envs):
            self.assertIn(f"/envs/env{i}/ball", balls.prim_paths)


# ---------------------------------------------------------------------------
# TestRigidBodyProperties
#   Masses, inverse masses, COMs, inertias, inverse inertias.
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestRigidBodyProperties(NewtonTensorTestBase):
    """Body-level properties: mass, inv mass, COM, inertia, inv inertia."""

    async def test_body_masses(self) -> None:
        """Verify rigid body masses can be written and read back for all balls."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, num_envs)
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        masses = np.ones((balls.count, 1), dtype=np.float32) * 100.0
        balls.set_masses(self.to_warp(masses), all_indices)
        self.assertTrue(np.allclose(balls.get_masses().numpy(), masses))

    async def test_body_inv_masses_shape(self) -> None:
        """Verify inverse-mass output has one entry per rigid body."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        inv_masses = balls.get_inv_masses().numpy()
        self.assertEqual(inv_masses.shape[0], num_envs)

    async def test_body_coms(self) -> None:
        """Verify center-of-mass transforms can be offset and read back."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        com = balls.get_coms().numpy().reshape(num_envs, 7)
        com[:, 0] += 0.1
        balls.set_coms(self.to_warp(com), all_indices)
        self.assertTrue(np.allclose(balls.get_coms().numpy(), com))

    async def test_body_inertias(self) -> None:
        """Verify flattened 3x3 inertia tensors can be modified and read back."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        inertias = balls.get_inertias().numpy().reshape(num_envs, 9)
        inertias[:, [0, 4, 8]] += 0.1
        balls.set_inertias(self.to_warp(inertias), all_indices)
        self.assertTrue(np.allclose(balls.get_inertias().numpy(), inertias))

    async def test_body_inv_inertias_shape(self) -> None:
        """Verify inverse inertia tensors are shaped as one 3x3 matrix per body."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        inv_inertias = balls.get_inv_inertias().numpy()
        self.assertEqual(inv_inertias.shape, (num_envs, 9))


# ---------------------------------------------------------------------------
# TestRigidBodyTransforms
#   Set transforms and read back (zero-gravity so nothing moves).
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestRigidBodyTransforms(NewtonTensorTestBase):
    """Set/get rigid body transforms."""

    async def setUp(self) -> None:
        """Disable gravity so transform writes remain stable after stepping."""
        await super().setUp()
        scene = self.stage.GetPrimAtPath("/physicsScene")
        from pxr import Gf, UsdPhysics

        physics_scene = UsdPhysics.Scene(scene)
        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, 0))
        physics_scene.CreateGravityMagnitudeAttr(0.0)

    async def test_set_get_transforms(self) -> None:
        """Verify transform writes preserve submitted poses for every rigid body."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, num_envs)
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        transforms = balls.get_transforms().numpy().reshape(balls.count, 7).copy()
        z_step = np.linspace(0.0, 1.0, balls.count, dtype=np.float32)
        transforms[:, 2] += z_step

        balls.set_transforms(self.to_warp(transforms), all_indices)
        self.step(1)

        result = balls.get_transforms().numpy().reshape(balls.count, 7)
        self.assertTrue(np.allclose(result, transforms, rtol=1e-3, atol=1e-4))

    async def test_set_get_transforms_subset(self) -> None:
        """Verify indexed transform writes move only the selected rigid bodies."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)
        self.step(1)

        original = balls.get_transforms().numpy().reshape(balls.count, 7).copy()
        modified = original.copy()
        modified[:, 2] += 1.0

        n_subset = min(balls.count, 4)
        subset_indices = self.to_warp(all_indices.numpy()[:n_subset], wp.uint32)
        balls.set_transforms(self.to_warp(modified), subset_indices)

        result = balls.get_transforms().numpy().reshape(balls.count, 7)
        self.assertTrue(np.allclose(result[:n_subset, 2], modified[:n_subset, 2], atol=1e-4))
        self.assertTrue(np.allclose(result[n_subset:], original[n_subset:], atol=1e-4))


# ---------------------------------------------------------------------------
# TestRigidBodyVelocities
#   Set velocities and read back (zero-gravity so they persist).
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestRigidBodyVelocities(NewtonTensorTestBase):
    """Set/get rigid body velocities."""

    async def setUp(self) -> None:
        """Disable gravity so velocity writes persist without external acceleration."""
        await super().setUp()
        scene = self.stage.GetPrimAtPath("/physicsScene")
        from pxr import Gf, UsdPhysics

        physics_scene = UsdPhysics.Scene(scene)
        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, 0))
        physics_scene.CreateGravityMagnitudeAttr(0.0)

    async def test_set_get_velocities(self) -> None:
        """Verify linear/angular velocity tensors round-trip for every rigid body."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, num_envs)
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        vels = np.zeros((balls.count, 6), dtype=np.float32)
        linear_z = np.linspace(0.1, 1.0, balls.count, dtype=np.float32)
        vels[:, 2] = linear_z

        balls.set_velocities(self.to_warp(vels), all_indices)

        for _ in range(10):
            self.step(1)

        result = balls.get_velocities().numpy().reshape(balls.count, 6)
        self.assertTrue(np.allclose(result, vels, rtol=1e-3, atol=1e-3))

    async def test_set_get_velocities_subset(self) -> None:
        """Verify indexed velocity writes update only the selected rigid bodies."""
        num_envs = self.setup_ball_grid(num_envs=16)
        sim = await self.create_sim()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)
        self.step(1)

        original = balls.get_velocities().numpy().reshape(balls.count, 6).copy()
        modified = original.copy()
        modified[:, 2] += 1.0

        n_subset = min(balls.count, 4)
        subset_indices = self.to_warp(all_indices.numpy()[:n_subset], wp.uint32)
        balls.set_velocities(self.to_warp(modified), subset_indices)

        result = balls.get_velocities().numpy().reshape(balls.count, 6)
        self.assertTrue(np.allclose(result[:n_subset], modified[:n_subset], atol=1e-3))
        self.assertTrue(np.allclose(result[n_subset:], original[n_subset:], atol=1e-3))


# ---------------------------------------------------------------------------
# TestRigidBodyAccelerations
#   Verify that get_accelerations returns body_qdd data when the extended
#   state attribute is requested. Requires MuJoCo solver (the only solver
#   that currently populates body_qdd).
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestRigidBodyAccelerations(NewtonTensorTestBase):
    """get_accelerations reads from body_qdd when allocated."""

    DT = 1.0 / 60.0

    async def _create_sim_with_body_qdd(self) -> "tensors.SimulationView":
        """Create a simulation view with body_qdd allocated on the state.

        Returns:
            Newton-backed simulation view with ``body_qdd`` state allocated.
        """
        await omni.kit.app.get_app().next_update_async()

        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        self.assertIsNotNone(newton_stage)
        newton_stage.initialize_newton(self.SIM_DEVICE)

        newton_stage.model.request_state_attributes("body_qdd")
        newton_stage.state_0 = newton_stage.model.state()
        newton_stage.state_1 = newton_stage.model.state()
        if newton_stage.cfg.use_cuda_graph:
            newton_stage.state_temp = newton_stage.model.state()

        import newton as nw

        nw.eval_fk(
            newton_stage.model, newton_stage.state_0.joint_q, newton_stage.state_0.joint_qd, newton_stage.state_0
        )

        self._sim = tensors.create_simulation_view("warp", backend="newton", stage_id=self._stage_id)
        self.assertIsNotNone(self._sim)
        return self._sim

    async def test_freefall_acceleration_is_gravity(self) -> None:
        """Free-falling bodies should have linear z acceleration close to -9.81."""
        num_envs = self.setup_ball_grid(num_envs=4, position=Gf.Vec3f(0, 0, 5.0))
        sim = await self._create_sim_with_body_qdd()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.step(n=5, dt=self.DT)

        accel = balls.get_accelerations().numpy().reshape(balls.count, 6)
        np.testing.assert_allclose(accel[:, 2], -9.81, rtol=0.1, atol=0.5)
        np.testing.assert_allclose(accel[:, 0:2], 0.0, atol=0.5)


# ---------------------------------------------------------------------------
# TestRigidBodyApplyForces
#   Physics tests for apply_forces and apply_forces_and_torques_at_position.
#   Verify that applied forces and torques produce linear and angular
#   velocities consistent with Newton's laws (F = m*a, tau = I*alpha) under
#   semi-implicit Euler integration. Tests are run with gravity disabled so
#   the only acceleration source is the applied wrench.
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestRigidBodyApplyForces(NewtonTensorTestBase):
    """Apply external forces and torques to rigid bodies."""

    BALL_MASS = 1.0
    BALL_RADIUS = 0.15
    DT = 1.0 / 60.0

    async def setUp(self) -> None:
        """Disable gravity so force and torque assertions isolate applied wrenches."""
        await super().setUp()
        from pxr import Gf, UsdPhysics

        scene = self.stage.GetPrimAtPath("/physicsScene")
        physics_scene = UsdPhysics.Scene(scene)
        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, 0))
        physics_scene.CreateGravityMagnitudeAttr(0.0)

    def _set_ball_masses(self, mass_value: float) -> None:
        from pxr import UsdPhysics

        for prim in self.stage.Traverse():
            if prim.GetName() != "ball":
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            mass_api = UsdPhysics.MassAPI(prim)
            if not mass_api:
                mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(float(mass_value))

    def _ball_sphere_inertia(self, mass: float, radius: float) -> float:
        return (2.0 / 5.0) * mass * radius * radius

    async def test_apply_forces_global_linear_velocity(self) -> None:
        """Upward global force on zero-velocity body: vz = (F/m) * dt after one step."""
        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS)
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        force_z = 10.0
        forces = np.zeros((balls.count, 3), dtype=np.float32)
        forces[:, 2] = force_z

        balls.apply_forces(self.to_warp(forces), all_indices, is_global=True)
        self.step(n=1, dt=self.DT)

        vels = balls.get_velocities().numpy().reshape(balls.count, 6)
        v_expected = (force_z / self.BALL_MASS) * self.DT
        np.testing.assert_allclose(vels[:, 2], v_expected, rtol=1e-2, atol=5e-3)
        np.testing.assert_allclose(vels[:, 0:2], 0.0, atol=5e-3)
        np.testing.assert_allclose(vels[:, 3:6], 0.0, atol=5e-3)

    async def test_apply_forces_subset_only_affects_selected_bodies(self) -> None:
        """Forces applied to a subset of bodies leave the other bodies at rest."""
        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS)
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        force_z = 20.0
        forces = np.zeros((balls.count, 3), dtype=np.float32)
        forces[:, 2] = force_z

        n_subset = balls.count // 2
        subset_indices = self.to_warp(all_indices.numpy()[:n_subset], wp.uint32)
        balls.apply_forces(self.to_warp(forces), subset_indices, is_global=True)
        self.step(n=1, dt=self.DT)

        vels = balls.get_velocities().numpy().reshape(balls.count, 6)
        v_expected = (force_z / self.BALL_MASS) * self.DT
        np.testing.assert_allclose(vels[:n_subset, 2], v_expected, rtol=1e-2, atol=5e-3)
        np.testing.assert_allclose(vels[n_subset:, 2], 0.0, atol=5e-3)

    async def test_apply_torques_angular_velocity(self) -> None:
        """Global torque about z produces wz = (tau/I_z) * dt after one step."""
        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS)
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        torque_z = 0.05
        forces = np.zeros((balls.count, 3), dtype=np.float32)
        torques = np.zeros((balls.count, 3), dtype=np.float32)
        torques[:, 2] = torque_z

        balls.apply_forces_and_torques_at_position(
            self.to_warp(forces),
            self.to_warp(torques),
            None,
            all_indices,
            is_global=True,
        )
        self.step(n=1, dt=self.DT)

        vels = balls.get_velocities().numpy().reshape(balls.count, 6)
        inertia = self._ball_sphere_inertia(self.BALL_MASS, self.BALL_RADIUS)
        wz_expected = (torque_z / inertia) * self.DT
        np.testing.assert_allclose(vels[:, 5], wz_expected, rtol=5e-2, atol=5e-3)
        np.testing.assert_allclose(vels[:, 0:3], 0.0, atol=5e-3)
        np.testing.assert_allclose(vels[:, 3:5], 0.0, atol=5e-3)

    async def test_apply_force_accumulates_over_steps(self) -> None:
        """Re-applying the same force each step produces velocity growing linearly with steps."""
        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS)
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        force_z = 5.0
        n_steps = 5
        forces = np.zeros((balls.count, 3), dtype=np.float32)
        forces[:, 2] = force_z

        for _ in range(n_steps):
            balls.apply_forces(self.to_warp(forces), all_indices, is_global=True)
            self.step(n=1, dt=self.DT)

        vels = balls.get_velocities().numpy().reshape(balls.count, 6)
        v_expected = (force_z / self.BALL_MASS) * self.DT * n_steps
        np.testing.assert_allclose(vels[:, 2], v_expected, rtol=2e-2, atol=5e-3)


# ---------------------------------------------------------------------------
# TestRigidBodySpatialLayout
#   Validates that get/set_velocities and apply_forces/torques use the
#   [linear(3), angular(3)] layout expected by the PhysX tensor API.
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestRigidBodySpatialLayout(NewtonTensorTestBase):
    """Velocity and force tensors must use [linear(3), angular(3)] layout."""

    BALL_MASS = 1.0
    BALL_RADIUS = 0.15
    DT = 1.0 / 60.0

    async def setUp(self) -> None:
        """Create the base rigid-body spatial-layout fixture."""
        await super().setUp()

    def _ball_sphere_inertia(self, mass: float, radius: float) -> float:
        return (2.0 / 5.0) * mass * radius * radius

    def _set_ball_masses(self, mass_value: float) -> None:
        from pxr import UsdPhysics

        for prim in self.stage.Traverse():
            if prim.GetName() != "ball":
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            mass_api = UsdPhysics.MassAPI(prim)
            if not mass_api:
                mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(float(mass_value))

    async def test_freefall_velocity_is_linear_z(self) -> None:
        """Free-falling body under gravity should only have velocity in z (slots 0-2)."""
        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS, position=Gf.Vec3f(0, 0, 5.0))
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        n_steps = 10
        for _ in range(n_steps):
            self.step(n=1, dt=self.DT)

        vels = balls.get_velocities().numpy().reshape(balls.count, 6)
        vz_expected = -9.81 * self.DT * n_steps
        np.testing.assert_allclose(vels[:, 2], vz_expected, rtol=5e-2, atol=1e-2)
        np.testing.assert_allclose(vels[:, 0:2], 0.0, atol=1e-3)
        np.testing.assert_allclose(vels[:, 3:6], 0.0, atol=1e-3)

    async def test_set_linear_velocity_produces_translation(self) -> None:
        """Setting velocity [vx,0,0,0,0,0] should translate the body in +x."""
        from pxr import UsdPhysics

        scene = UsdPhysics.Scene(self.stage.GetPrimAtPath("/physicsScene"))
        scene.CreateGravityMagnitudeAttr(0.0)

        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS, position=Gf.Vec3f(0, 0, 2.0))
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        pos_before = balls.get_transforms().numpy().reshape(balls.count, 7)[:, 0].copy()

        vx = 2.0
        vel = np.zeros((balls.count, 6), dtype=np.float32)
        vel[:, 0] = vx
        balls.set_velocities(self.to_warp(vel), all_indices)

        n_steps = 30
        for _ in range(n_steps):
            self.step(n=1, dt=self.DT)

        pos_after = balls.get_transforms().numpy().reshape(balls.count, 7)[:, 0]
        delta_x = pos_after - pos_before
        expected_dx = vx * self.DT * n_steps
        np.testing.assert_allclose(delta_x, expected_dx, rtol=5e-2, atol=1e-2)

    async def test_set_angular_velocity_produces_rotation(self) -> None:
        """Setting velocity [0,0,0,0,0,wz] should rotate the body around z."""
        from pxr import UsdPhysics

        scene = UsdPhysics.Scene(self.stage.GetPrimAtPath("/physicsScene"))
        scene.CreateGravityMagnitudeAttr(0.0)

        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS, position=Gf.Vec3f(0, 0, 2.0))
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        wz = 2.0
        vel = np.zeros((balls.count, 6), dtype=np.float32)
        vel[:, 5] = wz
        balls.set_velocities(self.to_warp(vel), all_indices)

        n_steps = 30
        for _ in range(n_steps):
            self.step(n=1, dt=self.DT)

        transforms = balls.get_transforms().numpy().reshape(balls.count, 7)
        theta = wz * self.DT * n_steps
        expected_qz = np.sin(theta / 2.0)
        np.testing.assert_allclose(np.abs(transforms[:, 5]), np.abs(expected_qz), rtol=5e-2, atol=1e-2)
        np.testing.assert_allclose(transforms[:, 3:5], 0.0, atol=1e-3)

    async def test_apply_force_layout_linear(self) -> None:
        """Force applied via apply_forces affects linear velocity (slots 0-2) only."""
        from pxr import UsdPhysics

        scene = UsdPhysics.Scene(self.stage.GetPrimAtPath("/physicsScene"))
        scene.CreateGravityMagnitudeAttr(0.0)

        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS)
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        fx = 10.0
        forces = np.zeros((balls.count, 3), dtype=np.float32)
        forces[:, 0] = fx

        balls.apply_forces(self.to_warp(forces), all_indices, is_global=True)
        self.step(n=1, dt=self.DT)

        vels = balls.get_velocities().numpy().reshape(balls.count, 6)
        vx_expected = (fx / self.BALL_MASS) * self.DT
        np.testing.assert_allclose(vels[:, 0], vx_expected, rtol=1e-2, atol=5e-3)
        np.testing.assert_allclose(vels[:, 1:3], 0.0, atol=5e-3)
        np.testing.assert_allclose(vels[:, 3:6], 0.0, atol=5e-3)

    async def test_apply_torque_layout_angular(self) -> None:
        """Torque applied via apply_forces_and_torques_at_position affects angular velocity (slots 3-5) only."""
        from pxr import UsdPhysics

        scene = UsdPhysics.Scene(self.stage.GetPrimAtPath("/physicsScene"))
        scene.CreateGravityMagnitudeAttr(0.0)

        self.setup_ball_grid(num_envs=4, radius=self.BALL_RADIUS)
        self._set_ball_masses(self.BALL_MASS)
        sim = await self.create_sim()
        self.start_playing()

        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = warp_utils.arange(balls.count, device=self.DEVICE)

        tau_z = 0.05
        forces = np.zeros((balls.count, 3), dtype=np.float32)
        torques = np.zeros((balls.count, 3), dtype=np.float32)
        torques[:, 2] = tau_z

        balls.apply_forces_and_torques_at_position(
            self.to_warp(forces),
            self.to_warp(torques),
            None,
            all_indices,
            is_global=True,
        )
        self.step(n=1, dt=self.DT)

        vels = balls.get_velocities().numpy().reshape(balls.count, 6)
        inertia = self._ball_sphere_inertia(self.BALL_MASS, self.BALL_RADIUS)
        wz_expected = (tau_z / inertia) * self.DT
        np.testing.assert_allclose(vels[:, 5], wz_expected, rtol=5e-2, atol=5e-3)
        np.testing.assert_allclose(vels[:, 0:3], 0.0, atol=5e-3)
        np.testing.assert_allclose(vels[:, 3:5], 0.0, atol=5e-3)
