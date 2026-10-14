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

"""Validate Newton articulation tensor views against omni.physics.tensors contracts.

The tests cover view counts and shared metatype metadata, DOF limits and drive
properties, full and indexed state writes, body mass/COM/inertia properties,
actuation forces, and simulation-view gravity for Newton-backed articulations.
"""

from __future__ import annotations

import copy
from typing import Any

import numpy as np
import omni.kit.app
import omni.physics.tensors as tensors
import warp as wp
from isaacsim.physics.newton import NewtonConfig, configure_newton, get_newton_config
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from .test_helpers import NewtonTensorTestBase, run_on_device_configs, warp_utils

# ---------------------------------------------------------------------------
# TestArticulationViewCartpole
#   Ported from TestArticulationView (CartPole asset)
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestArticulationViewCartpole(NewtonTensorTestBase):
    """CartPole articulation view: counts, metatype, paths."""

    async def test_articulation_view_counts(self) -> None:
        """Verify cartpole articulation view counts, link count, DOF count, and homogeneity."""
        num_envs = self.setup_cartpole_grid()
        sim = await self.create_sim()

        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(cartpoles, num_envs, 3, 2)
        self.assertTrue(cartpoles.is_homogeneous)

    async def test_articulation_view_metatype(self) -> None:
        """Verify cartpole shared metatype maps expected link and DOF names to stable indices."""
        num_envs = self.setup_cartpole_grid()
        sim = await self.create_sim()

        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        mt = cartpoles.shared_metatype
        self.assertIsNotNone(mt)

        self.assertIn("rail", mt.link_indices)
        self.assertIn("cart", mt.link_indices)
        self.assertIn("pole", mt.link_indices)
        self.assertEqual(mt.link_indices["rail"], 0)
        self.assertEqual(mt.link_indices["cart"], 1)
        self.assertEqual(mt.link_indices["pole"], 2)

        self.assertIn("cartJoint", mt.dof_indices)
        self.assertIn("poleJoint", mt.dof_indices)
        self.assertEqual(mt.dof_indices["cartJoint"], 0)
        self.assertEqual(mt.dof_indices["poleJoint"], 1)

    async def test_articulation_view_paths(self) -> None:
        """Verify cartpole articulation view expands the wildcard pattern to every environment path."""
        num_envs = self.setup_cartpole_grid()
        sim = await self.create_sim()

        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.assertEqual(len(cartpoles.prim_paths), num_envs)

        for i in range(num_envs):
            self.assertIn(f"/envs/env{i}/cartpole", cartpoles.prim_paths)

    async def test_apply_force_at_position_adds_link_moment(self) -> None:
        """Apply the moment from an off-center world-frame force to articulation links."""
        num_envs = self.setup_cartpole_grid()
        sim = await self.create_sim()
        self.start_playing()

        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        all_indices = warp_utils.arange(cartpoles.count, device=self.DEVICE)
        pole_index = cartpoles.shared_metatype.link_indices["pole"]
        transforms = cartpoles.get_link_transforms().numpy().reshape(num_envs, cartpoles.max_links, 7)

        centers_of_mass = np.zeros((num_envs, cartpoles.max_links, 7), dtype=np.float32)
        centers_of_mass[:, :, 6] = 1.0
        cartpoles.set_coms(self.to_warp(centers_of_mass), all_indices)

        forces = np.zeros((num_envs, cartpoles.max_links, 3), dtype=np.float32)
        forces[:, pole_index, 2] = 50.0
        positions = transforms[:, :, :3].copy()
        positions[:, pole_index, 0] += 2.0

        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        newton_stage.state_0.body_f.zero_()
        cartpoles.apply_forces_and_torques_at_position(
            self.to_warp(forces), None, self.to_warp(positions), all_indices, is_global=True
        )

        body_labels = [str(label) for label in newton_stage.model.body_label]
        body_forces = newton_stage.state_0.body_f.numpy().reshape(-1, 6)
        pole_wrenches = np.stack([body_forces[body_labels.index(f"{path}/pole")] for path in cartpoles.prim_paths])
        expected_forces = np.tile(np.array([0.0, 0.0, 50.0]), (cartpoles.count, 1))
        expected_torques = np.tile(np.array([0.0, -100.0, 0.0]), (cartpoles.count, 1))
        np.testing.assert_allclose(pole_wrenches[:, :3], expected_forces, atol=1e-5)
        np.testing.assert_allclose(pole_wrenches[:, 3:], expected_torques, atol=1e-5)


# ---------------------------------------------------------------------------
# TestArticulationViewHumanoid
#   Ported from TestHumanoidView (Humanoid asset)
# ---------------------------------------------------------------------------
@run_on_device_configs()
class TestArticulationViewHumanoid(NewtonTensorTestBase):
    """Humanoid articulation view: metatype names, DOF types, DOF limits."""

    async def test_humanoid_metatype_names(self) -> None:
        """Verify humanoid shared metatype preserves the expected link and DOF name ordering."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, num_envs, 16, 21)

        mt = humanoids.shared_metatype
        self.assertIsNotNone(mt)

        expected_link_names = [
            "torso",
            "head",
            "left_upper_arm",
            "left_lower_arm",
            "left_hand",
            "lower_waist",
            "pelvis",
            "left_thigh",
            "left_shin",
            "left_foot",
            "right_thigh",
            "right_shin",
            "right_foot",
            "right_upper_arm",
            "right_lower_arm",
            "right_hand",
        ]
        expected_dof_names = [
            "left_upper_arm:0",
            "left_upper_arm:1",
            "left_elbow",
            "lower_waist:0",
            "lower_waist:1",
            "abdomen_x",
            "left_thigh:0",
            "left_thigh:1",
            "left_thigh:2",
            "left_knee",
            "left_foot:0",
            "left_foot:1",
            "right_thigh:0",
            "right_thigh:1",
            "right_thigh:2",
            "right_knee",
            "right_foot:0",
            "right_foot:1",
            "right_upper_arm:0",
            "right_upper_arm:1",
            "right_elbow",
        ]

        self.assertSequenceEqual(mt.link_names, expected_link_names)
        self.assertSequenceEqual(mt.dof_names, expected_dof_names)

    async def test_humanoid_dof_types(self) -> None:
        """Verify every humanoid DOF is reported as a rotational DOF."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        dof_types = humanoids.get_dof_types()
        dof_types_np = dof_types.numpy().reshape(humanoids.count, humanoids.max_dofs)[0]
        expected = np.repeat(np.uint8(tensors.DofType.Rotation), humanoids.max_dofs)
        self.assertTrue(np.array_equal(dof_types_np, expected))

    async def test_humanoid_dof_limits(self) -> None:
        """Verify humanoid DOF limits match the authored joint ranges after conversion from radians."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        mt = humanoids.shared_metatype
        dof_limits = humanoids.get_dof_limits()
        dof_limits_np = dof_limits.numpy().reshape(humanoids.count, humanoids.max_dofs, 2)[0]

        expected_dof_limits = {
            "lower_waist:0": (-45, 45),
            "lower_waist:1": (-75, 30),
            "abdomen_x": (-35, 35),
            "right_upper_arm:0": (-90, 70),
            "right_upper_arm:1": (-90, 70),
            "right_elbow": (-90, 50),
            "left_upper_arm:0": (-90, 70),
            "left_upper_arm:1": (-90, 70),
            "left_elbow": (-90, 50),
            "right_thigh:0": (-45, 15),
            "right_thigh:1": (-120, 45),
            "right_thigh:2": (-60, 35),
            "right_knee": (-160, 2),
            "right_foot:0": (-50, 50),
            "right_foot:1": (-50, 50),
            "left_thigh:0": (-45, 15),
            "left_thigh:1": (-120, 45),
            "left_thigh:2": (-60, 35),
            "left_knee": (-160, 2),
            "left_foot:0": (-50, 50),
            "left_foot:1": (-50, 50),
        }
        for dof_name, (exp_lo, exp_hi) in expected_dof_limits.items():
            self.assertIn(dof_name, mt.dof_indices)
            idx = mt.dof_indices[dof_name]
            lo = np.degrees(dof_limits_np[idx, 0])
            hi = np.degrees(dof_limits_np[idx, 1])
            self.assertAlmostEqual(lo, np.float32(exp_lo), places=4)
            self.assertAlmostEqual(hi, np.float32(exp_hi), places=4)


# ---------------------------------------------------------------------------
# TestArticulationDofProperties
#   Ported from TestArticulationDofProperties (Humanoid asset)
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestArticulationDofProperties(NewtonTensorTestBase):
    """DOF property get/set: limits, stiffness, damping, max forces, max velocities, armature."""

    async def test_dof_limits(self) -> None:
        """Verify DOF limits can be set for all humanoids and overwritten for an indexed subset."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        limits = np.zeros((humanoids.count, humanoids.max_dofs, 2))
        limits[:, :, 0] = -0.1
        limits[:, :, 1] = 0.1
        humanoids.set_dof_limits(self.to_warp(limits), all_indices)
        self.assertTrue(np.allclose(humanoids.get_dof_limits().numpy(), limits))

        limits[0, :, 0] = -0.2
        limits[0, :, 1] = 2.0
        idx = wp.from_numpy(np.array([0]), dtype=wp.int32, device=self.DEVICE)
        humanoids.set_dof_limits(self.to_warp(limits), idx)
        self.assertTrue(np.allclose(humanoids.get_dof_limits().numpy(), limits))

    async def test_dof_stiffness(self) -> None:
        """Verify DOF stiffness values round-trip for all humanoids and an indexed subset."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        stiffness = np.ones((humanoids.count, humanoids.max_dofs)) * 100
        humanoids.set_dof_stiffnesses(self.to_warp(stiffness), all_indices)
        self.assertTrue(np.allclose(humanoids.get_dof_stiffnesses().numpy(), stiffness))

        stiffness[0, :] = 50
        idx = wp.from_numpy(np.array([0]), dtype=wp.int32, device=self.DEVICE)
        humanoids.set_dof_stiffnesses(self.to_warp(stiffness), idx)
        self.assertTrue(np.allclose(humanoids.get_dof_stiffnesses().numpy(), stiffness))

    async def test_dof_damping(self) -> None:
        """Verify DOF damping values round-trip for all humanoids and an indexed subset."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        damping = np.ones((humanoids.count, humanoids.max_dofs)) * 100
        humanoids.set_dof_dampings(self.to_warp(damping), all_indices)
        self.assertTrue(np.allclose(humanoids.get_dof_dampings().numpy(), damping))

        damping[0, :] = 50
        idx = wp.from_numpy(np.array([0]), dtype=wp.int32, device=self.DEVICE)
        humanoids.set_dof_dampings(self.to_warp(damping), idx)
        self.assertTrue(np.allclose(humanoids.get_dof_dampings().numpy(), damping))

    async def test_dof_max_forces(self) -> None:
        """Verify maximum DOF force values round-trip for all humanoids and an indexed subset."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        max_force = np.ones((humanoids.count, humanoids.max_dofs)) * 1000
        humanoids.set_dof_max_forces(self.to_warp(max_force), all_indices)
        self.assertTrue(np.allclose(humanoids.get_dof_max_forces().numpy(), max_force))

        max_force[0, :] = 2000
        idx = wp.from_numpy(np.array([0]), dtype=wp.int32, device=self.DEVICE)
        humanoids.set_dof_max_forces(self.to_warp(max_force), idx)
        self.assertTrue(np.allclose(humanoids.get_dof_max_forces().numpy(), max_force))

    async def test_dof_max_velocities(self) -> None:
        """Verify maximum DOF velocity values preserve per-environment data and subset updates."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        max_vel = 1000 * np.arange(0, humanoids.count * humanoids.max_dofs).reshape(
            humanoids.count, humanoids.max_dofs
        ).astype(np.float32)
        humanoids.set_dof_max_velocities(self.to_warp(max_vel), all_indices)
        self.assertTrue(np.allclose(humanoids.get_dof_max_velocities().numpy(), max_vel))

        max_vel[0, :] = 2000
        idx = wp.from_numpy(np.array([0]), dtype=wp.int32, device=self.DEVICE)
        humanoids.set_dof_max_velocities(self.to_warp(max_vel), idx)
        self.assertTrue(np.allclose(humanoids.get_dof_max_velocities().numpy(), max_vel))

    async def test_dof_armature(self) -> None:
        """Verify DOF armature values round-trip for all humanoids and an indexed subset."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        armature = np.ones((humanoids.count, humanoids.max_dofs), dtype=np.float32)
        humanoids.set_dof_armatures(self.to_warp(armature), all_indices)
        self.assertTrue(np.allclose(humanoids.get_dof_armatures().numpy(), armature))

        armature[0, :] = 0
        idx = wp.from_numpy(np.array([0]), dtype=wp.int32, device=self.DEVICE)
        humanoids.set_dof_armatures(self.to_warp(armature), idx)
        self.assertTrue(np.allclose(humanoids.get_dof_armatures().numpy(), armature))

    async def test_dof_friction_properties(self) -> None:
        """Verify DOF friction values round-trip for all humanoids and an indexed subset."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        friction = np.ones((humanoids.count, humanoids.max_dofs, 3)) * 100
        humanoids.set_dof_friction_properties(self.to_warp(friction), all_indices)
        expected = np.zeros_like(friction)
        expected[:, :, 0] = friction[:, :, 0]
        expected[:, :, 1] = friction[:, :, 0]
        self.assertTrue(np.allclose(humanoids.get_dof_friction_properties().numpy(), expected))

        # Set every index
        friction[:, :, 0] = 50  # static
        friction[:, :, 1] = 50  # dynamic
        friction[:, :, 2] = 0  # viscous (not supported by newton)

        idx = wp.from_numpy(np.array([0]), dtype=wp.int32, device=self.DEVICE)
        humanoids.set_dof_friction_properties(self.to_warp(friction), idx)

        # Construct solution
        friction[:, :, 0] = 100  # static
        friction[:, :, 1] = 100  # dynamic
        friction[:, :, 2] = 0  # viscous (not supported by newton)

        friction[0, :, 0] = 50  # static
        friction[0, :, 1] = 50  # dynamic
        friction[0, :, 2] = 0  # viscous (not supported by newton)

        self.assertTrue(np.allclose(humanoids.get_dof_friction_properties().numpy(), friction))

    async def test_masked_dof_friction_properties(self) -> None:
        """Verify masked friction writes select rows and an empty mask performs no writes."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)
        base = np.full((num_envs, humanoids.max_dofs, 3), 100.0, dtype=np.float32)
        humanoids.set_dof_friction_properties(self.to_warp(base), all_indices)

        mask = np.zeros(humanoids.count, dtype=np.uint8)
        mask[::2] = 1
        submitted = np.full_like(base, -999.0)
        submitted[mask != 0, :, 0] = 50.0
        self.assertTrue(self.call_masked_setter(humanoids, "set_dof_friction_properties_masked", submitted, mask))

        expected = np.zeros_like(base)
        expected[:, :, :2] = 100.0
        expected[mask != 0, :, :2] = 50.0
        result = humanoids.get_dof_friction_properties().numpy().reshape(base.shape)
        np.testing.assert_allclose(result, expected)

        empty_mask = np.zeros(humanoids.count, dtype=np.uint8)
        poison = np.full_like(base, -999.0)
        self.assertTrue(self.call_masked_setter(humanoids, "set_dof_friction_properties_masked", poison, empty_mask))
        result = humanoids.get_dof_friction_properties().numpy().reshape(base.shape)
        np.testing.assert_allclose(result, expected)

    async def test_dof_friction_properties_gpu_to_cpu(self) -> None:
        """Verify all friction channels are copied from a CUDA simulation into a CPU tensor."""
        if self.SIM_DEVICE != "cuda:0" or self.DEVICE != "cpu":
            self.skipTest("This regression test targets the CUDA simulation to CPU tensor path")

        UsdGeom.Xform.Define(self.stage, "/World")
        UsdGeom.Xform.Define(self.stage, "/World/Robot")

        base = UsdGeom.Cube.Define(self.stage, "/World/Robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).GetMassAttr().Set(1.0)
        UsdPhysics.ArticulationRootAPI.Apply(base.GetPrim())

        link = UsdGeom.Cube.Define(self.stage, "/World/Robot/link")
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        UsdPhysics.MassAPI.Apply(link.GetPrim()).GetMassAttr().Set(1.0)
        UsdGeom.Xformable(link.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))

        joint = UsdPhysics.RevoluteJoint.Define(self.stage, "/World/Robot/joint")
        joint.GetBody0Rel().SetTargets([Sdf.Path("/World/Robot/base")])
        joint.GetBody1Rel().SetTargets([Sdf.Path("/World/Robot/link")])
        joint.CreateAxisAttr("Z")

        await omni.kit.app.get_app().next_update_async()
        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage
        from omni.physics.tensors.api import SimulationView
        from omni.physics.tensors.frontend_warp import FrontendWarp

        newton_stage = acquire_newton_stage()
        self.assertIsNotNone(newton_stage)
        newton_stage.initialize_newton(self.SIM_DEVICE)
        backend_view = tensors.acquire_tensor_api().create_simulation_view(self._stage_id, "newton")
        self.assertIsNotNone(backend_view)
        self._sim = SimulationView(backend_view, FrontendWarp(-1), -1)
        sim = self._sim
        articulation = sim.create_articulation_view("/World/Robot/base")

        friction = np.zeros((articulation.count, articulation.max_dofs, 3), dtype=np.float32)
        friction[:, :, 0] = (
            np.arange(articulation.count * articulation.max_dofs, dtype=np.float32).reshape(
                articulation.count, articulation.max_dofs
            )
            + 1.0
        )
        indices = wp.array(np.arange(articulation.count, dtype=np.int32), dtype=wp.int32, device="cpu")
        articulation.set_dof_friction_properties(wp.array(friction, dtype=wp.float32, device="cpu"), indices)

        expected = np.zeros_like(friction)
        expected[:, :, 0] = friction[:, :, 0]
        expected[:, :, 1] = friction[:, :, 0]
        self.assertTrue(np.allclose(articulation.get_dof_friction_properties().numpy(), expected))


# ---------------------------------------------------------------------------
# TestArticulationGetSet
#   Ported from TestArticulationGetSet* family (Ant asset)
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestArticulationGetSet(NewtonTensorTestBase):
    """Get/set for DOF positions, velocities, targets, forces, and root state."""

    ATOL = 1e-5

    async def _setup_ant(self) -> Any:
        num_envs = self.setup_ant_grid()
        sim = await self.create_sim()
        ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(ants, num_envs, 9, 8)
        self.assertTrue(ants.is_homogeneous)
        all_indices = warp_utils.arange(ants.count, device=self.DEVICE)
        return sim, ants, all_indices

    # -- DOF positions --

    async def test_get_set_dof_positions(self) -> None:
        """Verify ant DOF positions can be read, offset, written, and read back for every articulation."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        num_dof = ants.max_dofs
        positions = ants.get_dof_positions().numpy().reshape((ants.count, num_dof)).copy()

        submitted = positions + 0.5
        ants.set_dof_positions(self.to_warp(submitted), all_indices)
        result = ants.get_dof_positions().numpy().reshape((ants.count, num_dof))
        self.assertTrue(np.allclose(result, submitted, atol=self.ATOL))

    async def test_get_set_dof_positions_subset(self) -> None:
        """Verify indexed DOF position writes update only the selected ant articulations."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        num_dof = ants.max_dofs
        positions = ants.get_dof_positions().numpy().reshape((ants.count, num_dof)).copy()

        submitted = positions + 0.5
        n_subset = min(ants.count, 2)
        subset_indices = self.to_warp(all_indices.numpy()[:n_subset], wp.uint32)
        ants.set_dof_positions(self.to_warp(submitted), subset_indices)
        result = ants.get_dof_positions().numpy().reshape((ants.count, num_dof))
        self.assertTrue(np.allclose(result[:n_subset], submitted[:n_subset], atol=self.ATOL))
        self.assertTrue(np.allclose(result[n_subset:], positions[n_subset:], atol=self.ATOL))

    async def test_masked_dof_positions(self) -> None:
        """Verify partial masks select rows and an empty mask performs no writes."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        base = np.arange(1, ants.count + 1, dtype=np.float32)[:, None]
        base = np.broadcast_to(base, (ants.count, ants.max_dofs)).copy()
        ants.set_dof_positions(self.to_warp(base), all_indices)

        mask = np.zeros(ants.count, dtype=np.uint8)
        mask[::2] = 1
        submitted = np.full_like(base, -999.0)
        submitted[mask != 0] = 7.0
        self.assertTrue(self.call_masked_setter(ants, "set_dof_positions_masked", submitted, mask))

        expected = base.copy()
        expected[mask != 0] = submitted[mask != 0]
        result = ants.get_dof_positions().numpy().reshape(ants.count, ants.max_dofs)
        np.testing.assert_allclose(result, expected, atol=self.ATOL)

        empty_mask = np.zeros(ants.count, dtype=np.uint8)
        poison = np.full_like(base, -999.0)
        self.assertTrue(self.call_masked_setter(ants, "set_dof_positions_masked", poison, empty_mask))
        result = ants.get_dof_positions().numpy().reshape(ants.count, ants.max_dofs)
        np.testing.assert_allclose(result, expected, atol=self.ATOL)

    # -- DOF velocities --

    async def test_get_set_dof_velocities(self) -> None:
        """Verify ant DOF velocities can be read, offset, written, and read back."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        num_dof = ants.max_dofs
        velocities = ants.get_dof_velocities().numpy().reshape((ants.count, num_dof)).copy()

        submitted = velocities + 0.5
        ants.set_dof_velocities(self.to_warp(submitted), all_indices)
        result = ants.get_dof_velocities().numpy().reshape((ants.count, num_dof))
        self.assertTrue(np.allclose(result, submitted, atol=self.ATOL))

    # -- DOF position targets --

    async def test_get_set_dof_position_targets(self) -> None:
        """Verify ant DOF position targets round-trip through the tensor view."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        num_dof = ants.max_dofs

        submitted = np.arange(ants.count * num_dof, dtype=np.float32).reshape(ants.count, num_dof) + 0.25
        ants.set_dof_position_targets(self.to_warp(submitted), all_indices)

        # Newton 1.6 stores position targets in joint-coordinate layout. A floating
        # root consumes seven coordinates but six velocity DOFs, so validating only
        # the public set/get round trip could hide the same wrong map on both paths.
        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        model = newton_stage.model
        articulation_starts = model.articulation_start.numpy()
        target_starts = model.joint_target_q_start.numpy()
        dof_starts = model.joint_qd_start.numpy()
        articulation_labels = [str(label) for label in model.articulation_label]
        target_indices = []
        axis_indices = []
        for prim_path in ants.prim_paths:
            articulation_index = articulation_labels.index(str(prim_path))
            joint_start = int(articulation_starts[articulation_index])
            joint_end = int(articulation_starts[articulation_index + 1])
            for joint_index in range(joint_start + 1, joint_end):
                dof_count = int(dof_starts[joint_index + 1] - dof_starts[joint_index])
                target_indices.extend(int(target_starts[joint_index]) + offset for offset in range(dof_count))
                axis_indices.extend(int(dof_starts[joint_index]) + offset for offset in range(dof_count))

        self.assertEqual(len(target_indices), ants.count * num_dof)
        self.assertNotEqual(target_indices, axis_indices)
        target_buffer = newton_stage.control.joint_target_q.numpy()
        np.testing.assert_allclose(target_buffer[target_indices], submitted.reshape(-1), atol=self.ATOL)

        result = ants.get_dof_position_targets().numpy().reshape((ants.count, num_dof))
        self.assertTrue(np.allclose(result, submitted, atol=self.ATOL))

    # -- DOF velocity targets --

    async def test_get_set_dof_velocity_targets(self) -> None:
        """Verify ant DOF velocity targets round-trip through the tensor view."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        num_dof = ants.max_dofs
        targets = ants.get_dof_velocity_targets().numpy().reshape((ants.count, num_dof)).copy()

        submitted = targets + 1.0
        ants.set_dof_velocity_targets(self.to_warp(submitted), all_indices)
        result = ants.get_dof_velocity_targets().numpy().reshape((ants.count, num_dof))
        self.assertTrue(np.allclose(result, submitted, atol=self.ATOL))

    # -- DOF actuation forces --

    async def test_get_set_dof_actuation_forces(self) -> None:
        """Verify ant DOF actuation forces round-trip through the tensor view."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        num_dof = ants.max_dofs
        forces = ants.get_dof_actuation_forces().numpy().reshape((ants.count, num_dof)).copy()

        submitted = forces + 10.0
        ants.set_dof_actuation_forces(self.to_warp(submitted), all_indices)
        result = ants.get_dof_actuation_forces().numpy().reshape((ants.count, num_dof))
        self.assertTrue(np.allclose(result, submitted, atol=self.ATOL))

    # -- Root transforms --

    async def test_get_set_root_transforms(self) -> None:
        """Verify root transform writes move every ant articulation by the submitted offset."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        roots = ants.get_root_transforms().numpy().reshape((ants.count, 7)).copy()
        delta = np.array([0, 0, 1.0, 0, 0, 0, 0], dtype=np.float32)

        submitted = roots + delta
        ants.set_root_transforms(self.to_warp(submitted), all_indices)
        result = ants.get_root_transforms().numpy().reshape((ants.count, 7))
        self.assertTrue(np.allclose(result[:, 2], roots[:, 2] + 1, atol=self.ATOL))

    async def test_get_set_root_transforms_subset(self) -> None:
        """Verify indexed root transform writes move only the selected ant articulations."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        roots = ants.get_root_transforms().numpy().reshape((ants.count, 7)).copy()
        delta = np.array([0, 0, 1.0, 0, 0, 0, 0], dtype=np.float32)

        submitted = roots + delta
        n_subset = min(ants.count, 2)
        subset_indices = self.to_warp(all_indices.numpy()[:n_subset], wp.uint32)
        ants.set_root_transforms(self.to_warp(submitted), subset_indices)
        result = ants.get_root_transforms().numpy().reshape((ants.count, 7))
        self.assertTrue(np.allclose(result[:n_subset, 2], roots[:n_subset, 2] + 1, atol=self.ATOL))
        self.assertTrue(np.allclose(result[n_subset:, :], roots[n_subset:, :], atol=self.ATOL))

    # -- Root velocities --

    async def test_get_set_root_velocities(self) -> None:
        """Verify root linear and angular velocities round-trip for every ant articulation."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        root_vels = ants.get_root_velocities().numpy().reshape((ants.count, 6)).copy()

        submitted = root_vels + 0.5
        ants.set_root_velocities(self.to_warp(submitted), all_indices)
        result = ants.get_root_velocities().numpy().reshape((ants.count, 6))
        self.assertTrue(np.allclose(result, submitted, atol=self.ATOL))

    async def test_get_set_root_velocities_subset(self) -> None:
        """Verify indexed root velocity writes update only the selected ant articulations."""
        sim, ants, all_indices = await self._setup_ant()
        self.step(1)

        root_vels = ants.get_root_velocities().numpy().reshape((ants.count, 6)).copy()

        submitted = root_vels + 1.0
        n_subset = min(ants.count, 2)
        subset_indices = self.to_warp(all_indices.numpy()[:n_subset], wp.uint32)
        ants.set_root_velocities(self.to_warp(submitted), subset_indices)
        result = ants.get_root_velocities().numpy().reshape((ants.count, 6))
        self.assertTrue(np.allclose(result[:n_subset], submitted[:n_subset], atol=self.ATOL))
        self.assertTrue(np.allclose(result[n_subset:], root_vels[n_subset:], atol=self.ATOL))


# ---------------------------------------------------------------------------
# TestArticulationBodyProperties
#   Ported from TestArticulationBodyProperties (Humanoid asset)
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestArticulationBodyProperties(NewtonTensorTestBase):
    """Body-level properties: mass, inverse mass, COM, inertia."""

    async def test_body_masses(self) -> None:
        """Verify humanoid link masses can be written and read back for every articulation."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, num_envs, 16, 21)
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        masses = np.ones((humanoids.count, humanoids.max_links), dtype=np.float32) * 100.0
        humanoids.set_masses(self.to_warp(masses), all_indices)
        self.assertTrue(np.allclose(humanoids.get_masses().numpy(), masses))

    async def test_masked_body_masses(self) -> None:
        """Verify masked multi-link scatter skips every link of unselected articulations."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)
        base = np.arange(1, num_envs + 1, dtype=np.float32)[:, None]
        base = np.broadcast_to(base, (num_envs, humanoids.max_links)).copy()
        humanoids.set_masses(self.to_warp(base), all_indices)

        mask = np.zeros(humanoids.count, dtype=np.uint8)
        mask[::2] = 1
        submitted = np.full_like(base, 13.0)
        self.assertTrue(self.call_masked_setter(humanoids, "set_masses_masked", submitted, mask))

        expected = base.copy()
        expected[mask != 0] = submitted[mask != 0]
        result = humanoids.get_masses().numpy().reshape(num_envs, humanoids.max_links)
        np.testing.assert_allclose(result, expected)

    async def test_body_inv_masses_shape(self) -> None:
        """Verify inverse-mass tensors are shaped by humanoid count and maximum link count."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        inv_masses = humanoids.get_inv_masses().numpy()
        self.assertEqual(inv_masses.shape, (num_envs, humanoids.max_links))

    async def test_body_coms(self) -> None:
        """Verify humanoid center-of-mass transforms can be modified and read back."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        com = humanoids.get_coms().numpy().reshape(num_envs, humanoids.max_links, 7)
        com[:, :, 0] += 0.1
        humanoids.set_coms(self.to_warp(com), all_indices)
        self.assertTrue(np.allclose(humanoids.get_coms().numpy(), com))

    async def test_body_inertias(self) -> None:
        """Verify humanoid inertia matrices can be modified and read back."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        all_indices = warp_utils.arange(humanoids.count, device=self.DEVICE)

        inertias = humanoids.get_inertias().numpy().reshape(num_envs, humanoids.max_links, 9)
        inertias[:, :, [0, 4, 8]] += 0.1
        humanoids.set_inertias(self.to_warp(inertias), all_indices)
        self.assertTrue(np.allclose(humanoids.get_inertias().numpy(), inertias))

    async def test_body_inv_inertias_shape(self) -> None:
        """Verify inverse-inertia tensors are shaped by humanoid count, link count, and matrix entries."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()

        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        inv_inertias = humanoids.get_inv_inertias().numpy()
        self.assertEqual(inv_inertias.shape, (num_envs, humanoids.max_links, 9))


# ---------------------------------------------------------------------------
# TestDofEffortsMovement
#   Verifies that set_dof_actuation_forces actually causes joint movement.
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestDofEffortsMovement(NewtonTensorTestBase):
    """Verify that applied joint efforts produce motion after stepping."""

    async def _setup_cartpole_no_drives(self) -> Any:
        """Set up cartpoles with zero PD gains so only applied forces drive motion.

        Returns:
            Tuple of simulation view, cartpole articulation view, and all cartpole indices.
        """
        try:
            import newton

            print(f"[DIAG] newton version={getattr(newton, '__version__', '?')} path={newton.__file__}")
        except Exception as e:
            print(f"[DIAG] newton import failed: {e}")
        try:
            import mujoco_warp

            print(f"[DIAG] mujoco_warp version={getattr(mujoco_warp, '__version__', '?')} path={mujoco_warp.__file__}")
        except Exception as e:
            print(f"[DIAG] mujoco_warp import failed: {e}")

        num_envs = self.setup_cartpole_grid()
        sim = await self.create_sim()
        self.start_playing()
        self.step(1)

        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        all_indices = warp_utils.arange(cartpoles.count, device=self.DEVICE)

        zero_stiffness = np.zeros((cartpoles.count, cartpoles.max_dofs), dtype=np.float32)
        zero_damping = np.zeros((cartpoles.count, cartpoles.max_dofs), dtype=np.float32)
        cartpoles.set_dof_stiffnesses(self.to_warp(zero_stiffness), all_indices)
        cartpoles.set_dof_dampings(self.to_warp(zero_damping), all_indices)

        zero_vel = np.zeros((cartpoles.count, cartpoles.max_dofs), dtype=np.float32)
        cartpoles.set_dof_velocities(self.to_warp(zero_vel), all_indices)

        try:
            f_arr = cartpoles.get_dof_actuation_forces()
            print(
                f"[DIAG] joint_f buffer: device={f_arr.device} dtype={f_arr.dtype} "
                f"shape={f_arr.shape} ptr={hex(f_arr.ptr) if f_arr.ptr else 'None'}"
            )
        except Exception as e:
            print(f"[DIAG] joint_f buffer query failed: {e}")

        try:
            masses = cartpoles.get_masses().numpy().reshape(cartpoles.count, cartpoles.max_links)
            inertias = cartpoles.get_inertias().numpy().reshape(cartpoles.count, cartpoles.max_links, 9)
            print(f"[DIAG] masses[0]={masses[0]}")
            print(f"[DIAG] inertias[0] diag=[{inertias[0, :, 0]} {inertias[0, :, 4]} {inertias[0, :, 8]}]")
        except Exception as e:
            print(f"[DIAG] mass/inertia query failed: {e}")

        return sim, cartpoles, all_indices

    async def test_effort_causes_velocity_change(self) -> None:
        """Verify applied cartpole joint effort changes velocity when PD gains are disabled."""
        sim, cartpoles, all_indices = await self._setup_cartpole_no_drives()

        vel_before = cartpoles.get_dof_velocities().numpy().reshape(cartpoles.count, cartpoles.max_dofs).copy()

        torques = np.zeros((cartpoles.count, cartpoles.max_dofs), dtype=np.float32)
        torques[:, 0] = 50.0
        for _ in range(5):
            cartpoles.set_dof_actuation_forces(self.to_warp(torques), all_indices)
            self.step(1)

        vel_after = cartpoles.get_dof_velocities().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
        delta = np.abs(vel_after[:, 0] - vel_before[:, 0])
        for i in range(cartpoles.count):
            self.assertGreater(delta[i], 0.01, f"Env {i}: DOF 0 velocity should change after applying effort")

    async def test_effort_causes_position_change(self) -> None:
        """Verify sustained cartpole joint effort changes position when PD gains are disabled."""
        sim, cartpoles, all_indices = await self._setup_cartpole_no_drives()

        dt = self.get_sim_dt()
        print(f"[DIAG] SIM_DEVICE={self.SIM_DEVICE} DEVICE={self.DEVICE} sim_dt={dt}")

        stiff = cartpoles.get_dof_stiffnesses().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
        damp = cartpoles.get_dof_dampings().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
        max_f = cartpoles.get_dof_max_forces().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
        armature = cartpoles.get_dof_armatures().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
        print(
            f"[DIAG] stiffness[0]={stiff[0]} damping[0]={damp[0]} " f"max_force[0]={max_f[0]} armature[0]={armature[0]}"
        )

        pos_before = cartpoles.get_dof_positions().numpy().reshape(cartpoles.count, cartpoles.max_dofs).copy()
        print(f"[DIAG] pos_before:\n{pos_before}")

        torques = np.zeros((cartpoles.count, cartpoles.max_dofs), dtype=np.float32)
        torques[:, 1] = 20.0
        num_steps = max(10, int(0.1 / dt))
        print(f"[DIAG] num_steps={num_steps} torque[0]={torques[0]}")

        for s in range(num_steps):
            cartpoles.set_dof_actuation_forces(self.to_warp(torques), all_indices)
            if s < 2:
                applied_pre = cartpoles.get_dof_actuation_forces().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
                print(f"[DIAG] step {s} actuation_pre_step[0]={applied_pre[0]}")
            self.step(1)
            if s < 2:
                applied_post = cartpoles.get_dof_actuation_forces().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
                pos_now = cartpoles.get_dof_positions().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
                vel_now = cartpoles.get_dof_velocities().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
                print(
                    f"[DIAG] after step {s} actuation_post_step[0]={applied_post[0]} "
                    f"pos[0]={pos_now[0]} vel[0]={vel_now[0]}"
                )

        pos_after = cartpoles.get_dof_positions().numpy().reshape(cartpoles.count, cartpoles.max_dofs)
        delta = np.abs(pos_after[:, 1] - pos_before[:, 1])
        print(f"[DIAG] pos_after:\n{pos_after}")
        print(f"[DIAG] delta (DOF 1): {delta}")

        for i in range(cartpoles.count):
            self.assertGreater(delta[i], 0.001, f"Env {i}: DOF 1 position should change after applying effort")

    async def test_zero_effort_no_extra_motion(self) -> None:
        """Verify zero commanded effort does not introduce unbounded extra joint motion."""
        sim, cartpoles, all_indices = await self._setup_cartpole_no_drives()

        zero_vel = np.zeros((cartpoles.count, cartpoles.max_dofs), dtype=np.float32)
        cartpoles.set_dof_velocities(self.to_warp(zero_vel), all_indices)

        pos_before = cartpoles.get_dof_positions().numpy().reshape(cartpoles.count, cartpoles.max_dofs).copy()
        self.step(5)
        pos_after = cartpoles.get_dof_positions().numpy().reshape(cartpoles.count, cartpoles.max_dofs)

        for dof in range(cartpoles.max_dofs):
            gravity_delta = np.abs(pos_after[:, dof] - pos_before[:, dof])
            non_effort_dofs_max = gravity_delta.max()
            self.assertLess(
                non_effort_dofs_max,
                5.0,
                f"DOF {dof}: position change should be bounded (gravity only, no applied effort)",
            )


# ---------------------------------------------------------------------------
# TestSimulationView
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestJacobianAndMassMatrices(NewtonTensorTestBase):
    """Test Jacobian Shape and Value."""

    # Free Articulation
    async def test_humanoid_jacobian_and_mass_matrices(self) -> None:
        """Verify Jacobian and mass-matrix shapes for a floating-base humanoid."""
        num_envs = self.setup_humanoid_grid()
        sim = await self.create_sim()
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.assertEqual(humanoids.jacobian_shape, (96, 27))
        jacobian = humanoids.get_jacobians()
        self.assertEqual(jacobian.shape, (4, 16, 6, 27))
        self.assertEqual(humanoids.generalized_mass_matrix_shape, (27, 27))
        mass_matrices = humanoids.get_generalized_mass_matrices()
        self.assertEqual(mass_matrices.shape, (4, 27, 27))

    # Root-fixed Articulation
    async def test_franka_jacobian_and_mass_matrices(self) -> None:
        """Verify Jacobian and mass-matrix shapes for a fixed-base Franka."""
        num_envs = self.setup_franka_grid()
        sim = await self.create_sim()
        frankas = sim.create_articulation_view("/envs/*/franka/root_joint")
        self.assertEqual(frankas.jacobian_shape, (42, 6))
        jacobian = frankas.get_jacobians()
        self.assertEqual(jacobian.shape, (4, 7, 6, 6))
        self.assertEqual(frankas.generalized_mass_matrix_shape, (6, 6))
        mass_matrices = frankas.get_generalized_mass_matrices()
        self.assertEqual(mass_matrices.shape, (4, 6, 6))


@run_on_device_configs()
class TestJointForceOutput(NewtonTensorTestBase):
    """Test optional incoming joint wrench and projected DOF force reporting."""

    async def setUp(self) -> None:
        """Enable the solver output before creating Newton state objects."""
        self._saved_newton_config = copy.deepcopy(get_newton_config()) or NewtonConfig()
        config = copy.deepcopy(self._saved_newton_config)
        config.solver_outputs.link_incoming_joint_force = True
        configure_newton(config)
        await super().setUp()

    async def tearDown(self) -> None:
        """Restore the Newton solver-output configuration."""
        await super().tearDown()
        configure_newton(self._saved_newton_config)

    async def test_joint_force_frame_conversion_and_projection(self) -> None:
        """Transform Newton wrenches and project them onto the joint motion direction."""
        articulation_path = "/World/robot"
        articulation = UsdGeom.Xform.Define(self.stage, articulation_path)
        UsdPhysics.ArticulationRootAPI.Apply(articulation.GetPrim())

        root_path = f"{articulation_path}/root"
        root = UsdGeom.Cube.Define(self.stage, root_path)
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(1.0)
        root_joint = UsdPhysics.FixedJoint.Define(self.stage, f"{root_path}/root_joint")
        root_joint.CreateBody1Rel().SetTargets([root_path])

        child_path = f"{articulation_path}/child"
        child = UsdGeom.Cube.Define(self.stage, child_path)
        child.AddTranslateOp().Set(Gf.Vec3f(1.0, 0.5, -0.25))
        child.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.2, 1.0, -0.3), 31.0).GetQuat()))
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        child_mass = UsdPhysics.MassAPI.Apply(child.GetPrim())
        child_mass.CreateMassAttr(2.0)
        child_mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.15, -0.1, 0.2))

        child_joint = UsdPhysics.RevoluteJoint.Define(self.stage, f"{child_path}/joint")
        child_joint.CreateBody0Rel().SetTargets([root_path])
        child_joint.CreateBody1Rel().SetTargets([child_path])
        child_joint.CreateAxisAttr().Set("Y")
        child_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.5, 0.0, 0.0))
        child_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.2, 0.1, 0.05))
        joint_orientation = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.4, 0.2), -27.0).GetQuat())
        child_joint.CreateLocalRot0Attr().Set(joint_orientation)
        child_joint.CreateLocalRot1Attr().Set(joint_orientation)

        sim = await self.create_sim()
        robot = sim.create_articulation_view(articulation_path)

        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        self.assertIsNotNone(newton_stage.state_0.body_parent_f)

        body_labels = [str(label) for label in newton_stage.model.body_label]
        joint_children = newton_stage.model.joint_child.numpy()
        body_transforms = newton_stage.state_0.body_q.numpy()
        body_centers_of_mass = newton_stage.model.body_com.numpy()
        joint_child_transforms = newton_stage.model.joint_X_c.numpy()
        parent_forces = np.zeros((len(body_labels), 6), dtype=np.float32)
        force_world = np.array([2.0, -3.0, 5.0], dtype=np.float32)
        torque_at_center_of_mass_world = np.array([7.0, 11.0, -13.0], dtype=np.float32)

        def multiply_quaternion(a: np.ndarray, b: np.ndarray) -> np.ndarray:
            """Multiply xyzw quaternions."""
            return np.array(
                [
                    a[3] * b[0] + b[3] * a[0] + a[1] * b[2] - a[2] * b[1],
                    a[3] * b[1] + b[3] * a[1] + a[2] * b[0] - a[0] * b[2],
                    a[3] * b[2] + b[3] * a[2] + a[0] * b[1] - a[1] * b[0],
                    a[3] * b[3] - np.dot(a[:3], b[:3]),
                ],
                dtype=np.float32,
            )

        def rotate_vector(rotation: np.ndarray, vector: np.ndarray) -> np.ndarray:
            """Rotate a vector by an xyzw quaternion."""
            imaginary = rotation[:3]
            twice_cross = 2.0 * np.cross(imaginary, vector)
            return vector + rotation[3] * twice_cross + np.cross(imaginary, twice_cross)

        child_slot = robot.shared_metatype.link_indices["child"]
        expected = np.zeros((1, robot.max_links, 6), dtype=np.float32)
        body_index = body_labels.index(child_path)
        joint_index = int(np.flatnonzero(joint_children == body_index)[0])
        parent_forces[body_index, :3] = force_world
        parent_forces[body_index, 3:] = torque_at_center_of_mass_world

        body_transform = body_transforms[body_index]
        joint_child_transform = joint_child_transforms[joint_index]
        center_of_mass_world = body_transform[:3] + rotate_vector(body_transform[3:], body_centers_of_mass[body_index])
        joint_position_world = body_transform[:3] + rotate_vector(body_transform[3:], joint_child_transform[:3])
        torque_at_joint_world = torque_at_center_of_mass_world + np.cross(
            center_of_mass_world - joint_position_world, force_world
        )
        joint_orientation_world = multiply_quaternion(body_transform[3:], joint_child_transform[3:])
        joint_orientation_world_inverse = joint_orientation_world * np.array([-1.0, -1.0, -1.0, 1.0])
        expected[0, child_slot, :3] = rotate_vector(joint_orientation_world_inverse, force_world)
        expected[0, child_slot, 3:] = rotate_vector(joint_orientation_world_inverse, torque_at_joint_world)

        newton_stage.state_0.body_parent_f.assign(parent_forces)
        actual = robot.get_link_incoming_joint_force().numpy().reshape(1, robot.max_links, 6)
        np.testing.assert_allclose(actual, expected, rtol=1e-5, atol=1e-5)
        projected = robot.get_dof_projected_joint_forces().numpy().reshape(1, robot.max_dofs)
        np.testing.assert_allclose(projected[0, 0], expected[0, child_slot, 4], rtol=1e-5, atol=1e-5)

        newton_stage.state_0.body_parent_f.zero_()
        self.start_playing()
        self.step(1)
        solver_output = robot.get_link_incoming_joint_force().numpy().reshape(1, robot.max_links, 6)
        self.assertGreater(np.linalg.norm(solver_output[0, child_slot]), 1e-3)


# ---------------------------------------------------------------------------
# TestSimulationView
# ---------------------------------------------------------------------------


@run_on_device_configs()
class TestSimulationView(NewtonTensorTestBase):
    """Simulation-level tests: creation and gravity."""

    async def test_create_simulation_view(self) -> None:
        """Verify a Newton tensor simulation view can be created for the test stage."""
        sim = await self.create_sim()
        self.assertIsNotNone(sim)

    async def test_set_get_gravity(self) -> None:
        """Verify simulation gravity can be set and queried through the tensor view."""
        sim = await self.create_sim()
        new_gravity = Gf.Vec3f(0, 0, -10.0)
        sim.set_gravity(new_gravity)
        gravity = sim.get_gravity()
        self.assertIsNotNone(gravity)
