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

"""Provide engine-neutral rigid-body tensor scenarios.

The scenarios build scenes with standard USD Physics schemas and validate rigid-body
view discovery, indexed simulation state, physical properties, kinematic state,
force application, and object-type queries.
"""

from __future__ import annotations

import os
import sys

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
from pxr import Gf, UsdPhysics  # noqa: E402


class RigidBodyViewCommon(GridTestBase):
    """Validate rigid-body view discovery and resolved-path metadata.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.0, 0.0, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Create a view and verify its body count and any resolved paths.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(balls, self.num_envs)

        prim_paths = balls.get_metadata("prim-paths") or []
        if prim_paths:
            assert len(prim_paths) == self.num_envs, f"expected {self.num_envs} resolved bodies, got {len(prim_paths)}"
            for i in range(self.num_envs):
                expected = f"/envs/env{i}/ball"
                assert expected in prim_paths

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Ignore physics steps because the scenario finishes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """


class RigidBodyEnableDisablePhysicsCommon(GridTestBase):
    """Validate indexed simulation disabling and its effect on motion.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8, env_spacing=2.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.num_balls = 4
        for i in range(self.num_balls):
            actor_path = self.env_template_path.AppendChild(f"ball_{i}")
            transform = Transform((0.0, 0.0, 1.0 + i * 0.5))
            ball = self.create_rigid_ball(actor_path, transform, 0.1)
            UsdPhysics.MassAPI(ball).GetMassAttr().Set(1.0)

    def on_start(self, sim: SimulationEntities) -> None:
        """Disable alternating bodies and verify the indexed state write.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.balls = sim.create_rigid_body_view("/envs/*/ball_*")
        self.check_rigid_body_view(self.balls, self.num_envs * self.num_balls)
        self.skip_if_unsupported(self.balls, "disable-simulations", "set")

        self.all_indices = wp_utils.arange(self.balls.count, device=self.wp_device)
        self.subset_inds_disabled_np = list(range(0, self.balls.count, 2))
        self.subset_inds_enabled_np = list(range(1, self.balls.count, 2))
        self.subset_indices_disabled_wp = wp.array(self.subset_inds_disabled_np, dtype=wp.int32, device=self.wp_device)
        self.subset_indices_enabled_wp = wp.array(self.subset_inds_enabled_np, dtype=wp.int32, device=self.wp_device)

        self.wp_disable_flags = wp.ones(self.balls.count, dtype=wp.uint8, device="cpu")
        self.wp_enable_flags = wp.zeros(self.balls.count, dtype=wp.uint8, device="cpu")

        self.balls.set_data("disable-simulations", self.wp_disable_flags, self.subset_indices_disabled_wp)

        current_disable_flags = self.balls.get_data("disable-simulations").numpy().flatten()
        assert (current_disable_flags[self.subset_inds_disabled_np] == 1).all()
        assert (current_disable_flags[self.subset_inds_enabled_np] == 0).all()

        self.dt = 1.0 / self.sim_params.time_steps_per_second

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Swap the disabled subset and verify only enabled bodies fall.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        current_velocities = self.balls.get_data("velocities").numpy().reshape(self.balls.count, 6)
        current_velocities_z = current_velocities[:, 2]

        if stepno == 1:
            expected_velocity_z = -2.0 * self.dt * self.sim_params.gravity_mag
            assert wp_utils.wp_allclose(
                current_velocities_z[self.subset_inds_disabled_np], 0.0
            ), f"Disabled bodies should not fall (step {stepno})"
            assert wp_utils.wp_allclose(
                current_velocities_z[self.subset_inds_enabled_np], expected_velocity_z
            ), f"Enabled bodies should fall at -2·dt·g (step {stepno})"

            self.balls.set_data("disable-simulations", self.wp_disable_flags, self.subset_indices_enabled_wp)
            self.balls.set_data("disable-simulations", self.wp_enable_flags, self.subset_indices_disabled_wp)

            current_disable_flags = self.balls.get_data("disable-simulations").numpy().flatten()
            assert (current_disable_flags[self.subset_inds_enabled_np] == 1).all()
            assert (current_disable_flags[self.subset_inds_disabled_np] == 0).all()

            self.balls.set_data("wake-up", self.all_indices)

        elif stepno == 2:
            expected_velocity_z = -self.dt * self.sim_params.gravity_mag
            assert wp_utils.wp_allclose(
                current_velocities_z[self.subset_inds_enabled_np], 0.0
            ), f"Now-disabled bodies should not fall (step {stepno})"
            assert wp_utils.wp_allclose(
                current_velocities_z[self.subset_inds_disabled_np], expected_velocity_z
            ), f"Now-enabled bodies should fall at -dt·g (step {stepno})"
            self.finish()


class RigidBodyDisableGravityCommon(GridTestBase):
    """Validate per-body gravity flags and their runtime effect.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8, env_spacing=2.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.num_balls = 4
        for i in range(self.num_balls):
            actor_path = self.env_template_path.AppendChild(f"ball_{i}")
            transform = Transform((0.0, 0.0, 1.0 + i * 0.5))
            ball = self.create_rigid_ball(actor_path, transform, 0.1)
            UsdPhysics.MassAPI(ball).GetMassAttr().Set(1.0)

    def on_start(self, sim: SimulationEntities) -> None:
        """Disable gravity for alternating bodies and verify round-trip state.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.balls = sim.create_rigid_body_view("/envs/*/ball_*")
        self.check_rigid_body_view(self.balls, self.num_envs * self.num_balls)
        self.skip_if_unsupported(self.balls, "disable-gravities", "set")

        self.disabled_indices = list(range(0, self.balls.count, 2))
        self.enabled_indices = list(range(1, self.balls.count, 2))
        disabled_indices = wp.array(self.disabled_indices, dtype=wp.int32, device=self.wp_device)

        # Generic indexed writes take a full [N] source and scatter selected rows.
        disable_flags = wp.ones(self.balls.count, dtype=wp.uint8, device="cpu")
        self.balls.set_data("disable-gravities", disable_flags, disabled_indices)

        readback = self.balls.get_data("disable-gravities")
        assert readback.dtype == wp.float32, "disable-gravities readback should use float32"
        readback_values = readback.numpy().reshape(-1)
        assert (readback_values[self.disabled_indices] == 1).all()
        assert (readback_values[self.enabled_indices] == 0).all()

        # Replay the public float32 output to cover adapter conversion back to
        # the uint8 representation required by the native boolean binding.
        self.balls.set_data("disable-gravities", readback)
        replayed_values = self.balls.get_data("disable-gravities").numpy().reshape(-1)
        assert (replayed_values == readback_values).all(), "GET-to-SET replay should preserve gravity flags"

        self.dt = 1.0 / self.sim_params.time_steps_per_second

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Verify gravity-disabled bodies remain stationary.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Simulated time interval in seconds.

        """
        if stepno != 1:
            return

        velocity_z = self.balls.get_data("velocities").numpy().reshape(self.balls.count, 6)[:, 2]
        expected_velocity_z = -2.0 * self.dt * self.sim_params.gravity_mag
        assert wp_utils.wp_allclose(
            velocity_z[self.disabled_indices], 0.0
        ), f"Gravity-disabled bodies should not fall (step {stepno})"
        assert wp_utils.wp_allclose(
            velocity_z[self.enabled_indices], expected_velocity_z
        ), f"Gravity-enabled bodies should fall at -2*dt*g (step {stepno})"
        self.finish()


class ReproInertiaSetGetConsistencyCommon(GridTestBase):
    """Verify that writing a retrieved inertia tensor preserves its value.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=3, env_spacing=2.0), SimParams(), device_params)
        actor_path = self.env_template_path.AppendChild("ball")
        ball = self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.0)))
        mass_api = UsdPhysics.MassAPI(ball)
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(0.5, 0.5, 0.5, 0.5))
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.1, 0.2, 0.3))

    def on_start(self, sim: SimulationEntities) -> None:
        """Round-trip the authored inertia tensor through the rigid-body view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = wp_utils.arange(balls.count, device=self.wp_device)

        initial = balls.get_data("inertias")
        initial_np = initial.numpy().copy()

        balls.set_data("inertias", initial, all_indices)
        new_np = balls.get_data("inertias").numpy()

        assert wp_utils.wp_allclose(initial_np, new_np), "set_inertias(get_inertias()) must be a no-op"
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Ignore physics steps because the scenario finishes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """


class RigidBodyShapePropertiesCommon(GridTestBase):
    """Validate per-shape material and collision-offset round trips.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.5)), 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Write and read material, contact-offset, and rest-offset data.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = wp_utils.arange(balls.count, device=self.wp_device)
        max_shapes = balls.get_metadata("num-shapes")

        mat_props = wp.zeros((balls.count, max_shapes, 3), dtype=wp.float32, device="cpu").numpy()
        mat_props[:, :, 0] = 0.8
        mat_props[:, :, 1] = 0.7
        mat_props[:, :, 2] = 0.6
        wp_mat = wp.from_numpy(mat_props, dtype=wp.float32, device=self.wp_device)
        balls.set_data("material-properties", wp_mat, all_indices)
        assert wp_utils.wp_allclose(
            balls.get_data("material-properties").numpy(), mat_props
        ), "material properties round-trip"

        contact_offsets = (
            wp_utils.random(balls.count * max_shapes, device=self.wp_device).numpy().reshape(balls.count, max_shapes)
        )
        wp_contacts = wp.from_numpy(contact_offsets, dtype=wp.float32, device=self.wp_device)
        balls.set_data("contact-offsets", wp_contacts, all_indices)
        assert wp_utils.wp_allclose(
            balls.get_data("contact-offsets").numpy(), contact_offsets
        ), "contact offsets round-trip"

        rest_offsets = (contact_offsets / 2).astype("float32")
        wp_rests = wp.from_numpy(rest_offsets, dtype=wp.float32, device=self.wp_device)
        balls.set_data("rest-offsets", wp_rests, all_indices)
        assert wp_utils.wp_allclose(balls.get_data("rest-offsets").numpy(), rest_offsets), "rest offsets round-trip"

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Ignore physics steps because the scenario finishes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """


class RigidBodyPropertiesCommon(GridTestBase):
    """Validate rigid-body mass, center-of-mass, and inertia access.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.5)), 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Round-trip writable properties and inspect inverse-property shapes.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        balls = sim.create_rigid_body_view("/envs/*/ball")
        all_indices = wp_utils.arange(balls.count, device=self.wp_device)

        masses = wp.full((balls.count, 1), 100.0, dtype=wp.float32, device=self.wp_device)
        balls.set_data("masses", masses, all_indices)
        assert wp_utils.wp_allclose(balls.get_data("masses"), masses)

        assert balls.get_data("inv-masses").numpy().shape[0] == self.num_envs

        com = balls.get_data("coms").numpy().reshape(self.num_envs, 7).astype("float32")
        com[:, 0] += 0.1
        wp_coms = wp.from_numpy(com, dtype=wp.float32, device=self.wp_device)
        balls.set_data("coms", wp_coms, all_indices)
        assert wp_utils.wp_allclose(balls.get_data("coms").numpy(), com)

        inertias = balls.get_data("inertias").numpy().reshape(self.num_envs, 9).astype("float32")
        inertias[:, [0, 4, 8]] += 0.1
        wp_inertias = wp.from_numpy(inertias, dtype=wp.float32, device=self.wp_device)
        balls.set_data("inertias", wp_inertias, all_indices)
        assert wp_utils.wp_allclose(balls.get_data("inertias").numpy(), inertias)

        assert balls.get_data("inv-inertias").numpy().shape == (self.num_envs, 9)

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Ignore physics steps because the scenario finishes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """


class RigidBodyComsSubsetRoundTripCommon(GridTestBase):
    """Verify center-of-mass updates through a single-body subset view.

    The selected body's model index differs from its view-local row. The scenario
    ensures both the position and orientation survive a center-of-mass write.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=4, env_spacing=2.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.5)), 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Write and read a nonidentity center-of-mass transform.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        # One-body view of the last env's ball: its model body index is > 0, so
        # the view-local cache row (0) differs from the global body index.
        balls = sim.create_rigid_body_view("/envs/env3/ball")
        assert balls.count == 1, "subset view must select exactly one body"
        one_index = wp_utils.arange(balls.count, device=self.wp_device)

        coms = balls.get_data("coms").numpy().reshape(balls.count, 7).astype("float32")
        coms[:, 0] += 0.1  # position offset
        coms[:, 3:7] = [0.5, 0.5, 0.5, 0.5]  # non-identity (unit) orientation quaternion
        wp_coms = wp.from_numpy(coms, dtype=wp.float32, device=self.wp_device)
        balls.set_data("coms", wp_coms, one_index)

        result = balls.get_data("coms").numpy().reshape(balls.count, 7)
        assert wp_utils.wp_allclose(result, coms), "subset-view coms position + cached orientation must round-trip"

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Ignore physics steps because the scenario finishes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """


class RigidBodyTransformsCommon(GridTestBase):
    """Validate indexed rigid-body transform writes.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=1.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.5)), 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Apply distinct vertical offsets to the body transforms.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.balls = sim.create_rigid_body_view("/envs/*/ball")
        self.all_indices = wp_utils.arange(self.balls.count, device=self.wp_device)

        transforms_np = self.balls.get_data("transforms").numpy().reshape(self.balls.count, 7).astype("float32")
        z_step = wp_utils.linspace(self.balls.count, 0.0, 1.0, include_end=True, device="cpu").numpy()
        transforms_np[..., 2] += z_step

        wp_transforms = wp.from_numpy(transforms_np, dtype=wp.float32, device=self.wp_device)
        self.balls.set_data("transforms", wp_transforms, self.all_indices)
        self.expected_transforms = transforms_np

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Verify the written transforms after simulation advances.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            transforms_np = self.balls.get_data("transforms").numpy().reshape(self.balls.count, 7)
            assert wp_utils.wp_allclose(
                transforms_np, self.expected_transforms, rtol=1e-3, atol=1e-4
            ), "expected transforms"
            self.finish()


class RigidBodyVelocitiesCommon(GridTestBase):
    """Validate indexed rigid-body velocity writes.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=1.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.5)), 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Assign distinct vertical velocities in the gravity-free scene.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.balls = sim.create_rigid_body_view("/envs/*/ball")
        self.all_indices = wp_utils.arange(self.balls.count, device=self.wp_device)

        vels_np = wp.zeros((self.balls.count, 6), dtype=wp.float32, device="cpu").numpy()
        vels_np[..., 2] = wp_utils.linspace(self.balls.count, 0.1, 1.0, include_end=True, device="cpu").numpy()
        wp_vels = wp.from_numpy(vels_np, dtype=wp.float32, device=self.wp_device)
        self.balls.set_data("velocities", wp_vels, self.all_indices)
        self.expected_vels = vels_np

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Verify the assigned velocities remain unchanged.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 10:
            vels_np = self.balls.get_data("velocities").numpy().reshape(self.balls.count, 6)
            assert wp_utils.wp_allclose(vels_np, self.expected_vels, rtol=1e-3, atol=1e-3), "expected velocities"
            self.finish()


class RigidBodyAccelerationsCommon(GridTestBase):
    """Validate reported accelerations against gravity and finite differences.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=4, env_spacing=1.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 1.0)
        sim_params.gravity_mag = -10
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        self.create_rigid_ball(actor_path, Transform((0.0, 0.0, 0.5)), 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Assign initial vertical velocities before sampling acceleration.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.balls = sim.create_rigid_body_view("/envs/*/ball")
        self.all_indices = wp_utils.arange(self.balls.count, device=self.wp_device)

        vels_np = wp.zeros((self.balls.count, 6), dtype=wp.float32, device="cpu").numpy()
        vels_np[..., 2] = wp_utils.linspace(self.balls.count, 0.0, 1.0, include_end=True, device="cpu").numpy()
        wp_vels = wp.from_numpy(vels_np, dtype=wp.float32, device=self.wp_device)
        self.balls.set_data("velocities", wp_vels, self.all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare acceleration data with consecutive velocity samples.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 2:
            self.old_vels = self.balls.get_data("velocities").numpy().reshape(self.balls.count, 6).copy()
        if stepno == 3:
            new_vels = self.balls.get_data("velocities").numpy().reshape(self.balls.count, 6).copy()
            self.skip_if_unsupported(self.balls, "accelerations", "get")
            acc = self.balls.get_data("accelerations").numpy().reshape(self.balls.count, 6).copy()
            acc_fd = (new_vels - self.old_vels) / dt
            assert wp_utils.wp_allclose(
                acc, acc_fd, rtol=1e-3, atol=1e-3
            ), "expected acceleration matches finite difference"
            assert wp_utils.wp_allclose(
                acc, [0, 0, -10, 0, 0, 0], rtol=1e-3, atol=1e-3
            ), "expected gravity acceleration"
            self.finish()


class RigidBodyForceCommon(GridTestBase):
    """Launch rotated bodies with a world-frame force.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        q = Gf.Quatf(Gf.Rotation(Gf.Vec3d([0, 1, 0]), 90).GetQuat())
        self.transform = Transform((0.0, 0.0, 0.5), q)
        self.create_rigid_ball(self.env_template_path.AppendChild("ball"), self.transform, 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Prepare the body view and apply the CPU-pipeline force.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.balls = sim.create_rigid_body_view("/envs/*/ball")
        global_force = wp.vec3(0.0, 0.0, 4000.0)
        self._gForce = global_force
        self._indices = wp_utils.arange(self.balls.count, device=self.wp_device)

        if not self.device_params.use_gpu_pipeline:
            # Apply immediately when the tensor pipeline does not require GPU warmup.
            forces = wp_utils.fill_vec3(self.balls.count, value=global_force, device=self.wp_device)
            wrench = pack_wrench(forces, None, None).reshape((self.balls.count, 9))
            self.balls.set_data("apply-forces-and-torques-at-position", wrench, self._indices)
        # GPU path deferred to on_physics_step(0) after warmup.

    def _apply_gpu_force_global(self) -> None:
        """Apply the world-frame force on the GPU pipeline."""
        forces = wp_utils.fill_vec3(self.balls.count, value=self._gForce, device=self.wp_device)
        wrench = pack_wrench(forces, None, None).reshape((self.balls.count, 9))
        indices = wp_utils.arange(self.balls.count, device=self.wp_device)
        self.balls.set_data("apply-forces-and-torques-at-position", wrench, indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Apply any deferred force and validate the launch trajectory.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if self.device_params.use_gpu_pipeline and stepno == 0:
            self._apply_gpu_force_global()
        z = self.balls.get_data("transforms").numpy().reshape(self.balls.count, 7)[:, 2]
        if stepno == 1:
            assert (z > 0.5).all(), "launch positions"
        elif stepno == 27:
            assert (z > 1.5).all(), "peak positions"
        elif stepno >= 60:
            assert (z < 0.2).all(), "end positions"
            self.finish()


class RigidBodyZeroIndexSetCommon(RigidBodyForceCommon):
    """Verify that a zero-row indexed velocity write changes no bodies.

    An empty index array represents an indexed write to no rows rather than an
    omitted index array that selects every row.
    """

    def on_start(self, sim: SimulationEntities) -> None:
        """Apply an empty indexed write and verify velocities are unchanged.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        balls = sim.create_rigid_body_view("/envs/*/ball")
        n = int(balls.count)
        before = balls.get_data("velocities").numpy().reshape(n, 6).copy()
        balls.set_data(
            "velocities",
            wp.zeros((0, 6), dtype=wp.float32, device=self.wp_device),
            wp.zeros(0, dtype=wp.int32, device=self.wp_device),
        )
        after = balls.get_data("velocities").numpy().reshape(n, 6)
        assert (before == after).all(), "K==0 indexed set must be a no-op"
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Ignore physics steps because the scenario finishes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """


class _RigidBodiesGetSetBase(GridTestBase):
    """Build the shared multi-body rigid-body view scene.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    body_names = ("right_ball", "left_ball", "torso")

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=4.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 20
        super().__init__(test_case, grid_params, sim_params, device_params)

        ant_asset = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        self.create_actor_from_asset(actor_path, Transform((0.0, 0.0, 1.0)), ant_asset)

        mass = 0.5
        ball = self.create_rigid_ball(actor_path.AppendChild("right_ball"), Transform((-0.5, 0.0, 0.0)), 0.1)
        UsdPhysics.MassAPI(ball).GetMassAttr().Set(mass)
        ball = self.create_rigid_ball(actor_path.AppendChild("left_ball"), Transform((0.5, 0.0, 0.0)), 0.1)
        UsdPhysics.MassAPI(ball).GetMassAttr().Set(mass)
        self.body_per_env = len(self.body_names)
        self.atol = 1e-5

        self.patterns = [f"/envs/*/ant/{name}" for name in self.body_names]


class RigidBodiesGetSetTransformsCommon(_RigidBodiesGetSetBase):
    """Validate transform writes across a heterogeneous rigid-body view."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create a view containing free bodies and articulation links.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.rb_view = sim.create_rigid_body_view(self.patterns)
        self.check_rigid_body_view(self.rb_view, self.num_envs * self.body_per_env)
        self.all_indices = wp_utils.arange(self.rb_view.count, device=self.wp_device)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Raise every selected transform and verify its vertical position.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            transforms = self.rb_view.get_data("transforms").numpy().reshape(self.rb_view.count, 7).copy()
            submitted = (transforms + [0, 0, 1.0, 0, 0, 0, 0]).astype("float32")
            self.rb_view.set_data("transforms", self.to_warp(submitted), self.all_indices)
            new_transforms = self.rb_view.get_data("transforms").numpy().reshape(self.rb_view.count, 7)
            assert wp_utils.wp_allclose(
                new_transforms[:, 2], transforms[:, 2] + 1, rtol=1e-3, atol=self.atol
            ), "set transforms +1 in Z should round-trip"
            self.finish()


class RigidBodiesGetSetVelocitiesCommon(_RigidBodiesGetSetBase):
    """Validate velocity writes across a heterogeneous rigid-body view."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create a view containing free bodies and articulation links.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.rb_view = sim.create_rigid_body_view(self.patterns)
        self.check_rigid_body_view(self.rb_view, self.num_envs * self.body_per_env)
        self.all_indices = wp_utils.arange(self.rb_view.count, device=self.wp_device)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Increase every selected body's vertical velocity and verify the write.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            vels = self.rb_view.get_data("velocities").numpy().reshape(self.rb_view.count, 6).copy()
            submitted = (vels + [0, 0, 1, 0, 0, 0]).astype("float32")
            self.rb_view.set_data("velocities", self.to_warp(submitted), self.all_indices)
            new_vels = self.rb_view.get_data("velocities").numpy().reshape(self.rb_view.count, 6)
            assert wp_utils.wp_allclose(
                new_vels[:, 2], vels[:, 2] + 1, rtol=1e-2, atol=self.atol
            ), "set velocities +1 in linear-Z should round-trip"
            self.finish()


class RigidBodiesGetSetAppliedForcesCommon(_RigidBodiesGetSetBase):
    """Validate force-at-position writes across a multi-body view."""

    body_names = ("right_ball", "left_ball", "torso", "right_back_leg", "right_back_foot")

    def on_start(self, sim: SimulationEntities) -> None:
        """Prepare upward forces and their application positions.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.rb_view = sim.create_rigid_body_view(self.patterns)
        self.check_rigid_body_view(self.rb_view, self.num_envs * self.body_per_env)
        self.all_indices = wp_utils.arange(self.rb_view.count, device=self.wp_device)

        force_offset = 1
        transforms = self.rb_view.get_data("transforms").numpy().reshape(self.num_envs, self.body_per_env, 7)
        positions = transforms[:, :, 0:3].copy()
        global_force = wp.vec3(0.0, 0.0, 150.0)
        positions[:, :, 0:3] += [0, 0, force_offset]
        self.forces_wp = wp_utils.fill_vec3(self.rb_view.count, value=global_force, device=self.wp_device)
        self.positions_wp = wp.from_numpy(positions.flatten(), dtype=wp.float32, device=self.wp_device)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Apply the forces and verify every selected body rises.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            self.init_height = (
                self.rb_view.get_data("transforms").numpy().reshape(self.num_envs, self.body_per_env, 7)[:, :, 2].copy()
            )
            n_total = self.num_envs * self.body_per_env
            wrench = pack_wrench(self.forces_wp, None, self.positions_wp).reshape((n_total, 9))
            self.rb_view.set_data("apply-forces-and-torques-at-position", wrench, self.all_indices)
        if stepno == 10:
            height = self.rb_view.get_data("transforms").numpy().reshape(self.num_envs, self.body_per_env, 7)[:, :, 2]
            assert (height > self.init_height).all(), "applied force should lift the bodies"
            self.finish()


class RigidBodyForceAtPosCommon(GridTestBase):
    """Compare force-at-position behavior across two view representations.

    The scenario selects the same nine articulation links through a rigid-body
    view and an articulation view, applies identical world-space wrenches, and
    compares the resulting root heights when path metadata identifies every root.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    body_per_env = 9

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=4.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        ant_asset = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        self.create_actor_from_asset(actor_path, Transform((0.0, -10.0, 2.0)), ant_asset)
        actor_path_2 = self.env_template_path.AppendChild("ant_2")
        self.create_actor_from_asset(actor_path_2, Transform((0.0, 10.0, 2.0)), ant_asset)

    def on_start(self, sim: SimulationEntities) -> None:
        """Create equivalent views and prepare matching force inputs.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        # Select only articulation links so both views receive the same body set.
        rb_patterns = ["/envs/*/ant/torso"]
        for leg in (
            "front_left_leg",
            "front_right_leg",
            "left_back_leg",
            "right_back_leg",
            "front_left_foot",
            "front_right_foot",
            "left_back_foot",
            "right_back_foot",
        ):
            rb_patterns.append(f"/envs/*/ant/{leg}")
        self.rb_view = sim.create_rigid_body_view(rb_patterns)
        self.arti_view = sim.create_articulation_view("/envs/*/ant_2/torso")
        self.indices = wp_utils.arange(self.rb_view.count, device=self.wp_device)
        self.arti_indices = wp_utils.arange(self.arti_view.count, device=self.wp_device)

        # Compute force-application positions from current transforms.
        force_offset = 1
        rb_transforms = self.rb_view.get_data("transforms").numpy().reshape(self.rb_view.count, 7)
        rb_positions = rb_transforms[:, 0:3].copy() + [0, 0, force_offset]
        arti_transforms = (
            self.arti_view.get_data("link-transforms")
            .numpy()
            .reshape(self.num_envs, self.arti_view.get_metadata("num-links"), 7)
        )
        arti_positions = arti_transforms[:, :, 0:3].copy() + [0, 0, force_offset]

        global_force = wp.vec3(0.0, 0.0, 100.0)
        self.forces = wp_utils.fill_vec3(self.rb_view.count, value=global_force, device=self.wp_device)
        self.arti_forces = wp_utils.fill_vec3(
            self.arti_view.count * self.arti_view.get_metadata("num-links"),
            value=global_force,
            device=self.wp_device,
        )
        self.positions = wp.from_numpy(
            rb_positions.flatten().astype("float32"),
            dtype=wp.float32,
            device=self.wp_device,
        )
        self.arti_positions = wp.from_numpy(
            arti_positions.flatten().astype("float32"),
            dtype=wp.float32,
            device=self.wp_device,
        )

        self.rb_root_indices = [
            i for i, p in enumerate(self.rb_view.get_metadata("prim-paths") or []) if p.endswith("torso")
        ]

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Apply matching wrenches and compare all identifiable root heights.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            rb_wrench = pack_wrench(self.forces, None, self.positions).reshape((self.rb_view.count, 9))
            self.rb_view.set_data("apply-forces-and-torques-at-position", rb_wrench, self.indices)
            arti_n_total = self.arti_view.count * self.arti_view.get_metadata("num-links")
            arti_wrench = pack_wrench(self.arti_forces, None, self.arti_positions).reshape(
                (self.arti_view.count, self.arti_view.get_metadata("num-links"), 9)
            )
            self.arti_view.set_data("apply-forces-and-torques-at-position", arti_wrench, self.arti_indices)
        if stepno == 10:
            arti_root = (
                self.arti_view.get_data("link-transforms")
                .numpy()
                .reshape(self.num_envs, self.arti_view.get_metadata("num-links"), 7)[:, 0]
            )
            rb_root = self.rb_view.get_data("transforms").numpy().reshape(self.rb_view.count, 7)[self.rb_root_indices]
            if len(self.rb_root_indices) == self.num_envs:
                assert wp_utils.wp_allclose(
                    arti_root[:, 2], rb_root[:, 2], rtol=1e-3, atol=1e-2
                ), "root height matches between rb_view + arti_view force application"
            self.finish()
