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

from typing import Any

import numpy as np
import omni.physics.tensors as tensors
import warp as wp
from pxr import Gf

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
        targets = ants.get_dof_position_targets().numpy().reshape((ants.count, num_dof)).copy()

        submitted = targets + 0.5
        ants.set_dof_position_targets(self.to_warp(submitted), all_indices)
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
