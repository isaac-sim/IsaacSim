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

"""Verifies the Newton articulation view implementation against loaded articulated assets and live simulation stepping. The tests cover transforms, velocities, DOF state and limits, mass and inertia properties, force application, path metadata, and validity checks."""

from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.physics.newton
import isaacsim.physics.newton.tensors
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async


async def wait_for_stage_loading() -> Any:
    """Wait until USD stage loading is complete.

    Returns:
        None.
    """
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        await omni.kit.app.get_app().next_update_async()


class TestNewtonArticulationView(omni.kit.test.AsyncTestCase):
    """Tests for Newton articulation view tensor API."""

    async def setUp(self) -> None:
        """Set up test environment."""
        self.use_gpu = True
        self.wp_device = "cuda:0" if self.use_gpu else "cpu"

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            self.skipTest("Could not find Isaac Sim assets folder")

        await stage_utils.create_new_stage_async()
        self.stage = omni.usd.get_context().get_stage()

        self.humanoid_asset = self._assets_root_path + "/Isaac/Robots/IsaacSim/Humanoid/humanoid.usd"
        stage_utils.add_reference_to_stage(usd_path=self.humanoid_asset, path="/nv_humanoid")
        await wait_for_stage_loading()

        success = SimulationManager.switch_physics_engine("newton")
        self.assertTrue(success, "Failed to switch to Newton physics backend")
        await omni.kit.app.get_app().next_update_async()

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.play()
        await omni.kit.app.get_app().next_update_async()

        self.newton_stage = isaacsim.physics.newton.acquire_stage()
        self.sim = isaacsim.physics.newton.tensors.create_simulation_view(
            frontend_name="warp", stage_id=-1, newton_stage=self.newton_stage
        )

    async def tearDown(self) -> None:
        """Clean up after test."""
        self.timeline.pause()
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().close_stage_async()

    async def test_articulation_view_creation(self) -> None:
        """Test creating articulation view and basic properties."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        self.assertIsNotNone(articulations)
        self.assertGreater(articulations.count, 0, "Should have at least one articulation")
        self.assertEqual(articulations.count, 1, "Humanoid should have exactly 1 articulation")
        self.assertGreater(articulations.max_dofs, 0, "Articulation should have DOFs")
        self.assertEqual(articulations.max_dofs, 21, f"Expected 21 DOFs, got {articulations.max_dofs}")

    async def test_metatype_info(self) -> None:
        """Test articulation metatype information."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        mt = articulations.get_metatype(0)

        self.assertIsNotNone(mt, "Metatype should not be None")
        self.assertTrue(hasattr(mt, "link_names"), "Metatype should have link_names")
        self.assertGreater(len(mt.link_names), 0, "Should have at least one link")
        self.assertTrue(hasattr(mt, "joint_names"), "Metatype should have joint_names")
        self.assertGreater(len(mt.joint_names), 0, "Should have at least one joint")
        self.assertTrue(hasattr(mt, "dof_names"), "Metatype should have dof_names")
        self.assertGreater(len(mt.dof_names), 0, "Should have at least one DOF")
        self.assertTrue(hasattr(mt, "fixed_base"), "Metatype should have fixed_base attribute")
        self.assertIsInstance(mt.fixed_base, bool, "fixed_base should be boolean")

    async def test_root_transforms(self) -> None:
        """Test getting and setting root transforms."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        initial_transforms = articulations.get_root_transforms()
        self.assertIsNotNone(initial_transforms, "Transforms should not be None")

        initial_transforms_np = (
            initial_transforms.numpy() if hasattr(initial_transforms, "numpy") else np.array(initial_transforms)
        )
        self.assertEqual(
            initial_transforms_np.shape,
            (articulations.count, 7),
            f"Transform shape should be ({articulations.count}, 7)",
        )

        modified_transforms = initial_transforms_np.copy()
        modified_transforms[0, 2] += 1.0

        transform_wp = wp.from_numpy(modified_transforms, dtype=wp.float32, device=self.wp_device)
        articulations.set_root_transforms(transform_wp, indices)

        await omni.kit.app.get_app().next_update_async()

        stepped_transforms = articulations.get_root_transforms().numpy()
        self.assertFalse(
            np.allclose(stepped_transforms[0, 2], initial_transforms_np[0, 2], atol=0.01),
            f"Z position should have changed from {initial_transforms_np[0, 2]}",
        )

    async def test_root_velocities(self) -> None:
        """Test getting and setting root velocities."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current root velocities
        initial_vel = articulations.get_root_velocities()
        self.assertIsNotNone(initial_vel, "Root velocities should not be None")

        initial_vel_np = initial_vel.numpy() if hasattr(initial_vel, "numpy") else np.array(initial_vel)
        self.assertEqual(
            initial_vel_np.shape,
            (articulations.count, 6),
            f"Root velocity shape should be ({articulations.count}, 6)",
        )

        # Modify root velocities
        modified_vel = initial_vel_np.copy()
        modified_vel[0, 0] = 1.0  # Linear x velocity
        modified_vel[0, 5] = 0.5  # Angular z velocity

        # Set modified values into simulator
        vel_wp = wp.from_numpy(modified_vel, dtype=wp.float32, device=self.wp_device)
        articulations.set_root_velocities(vel_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_vel = articulations.get_root_velocities().numpy()
        self.assertTrue(
            np.allclose(immediate_vel[0], modified_vel[0], atol=1e-5),
            "Root velocity (before step) should match set values",
        )

        # Take a step and check values are close (looser tolerance)
        await omni.kit.app.get_app().next_update_async()

        stepped_vel = articulations.get_root_velocities().numpy()
        # Velocities may be affected by damping, constraints, etc.
        self.assertTrue(
            np.allclose(stepped_vel[0, 0], modified_vel[0, 0], atol=1.0),
            f"Root velocity (after step) should be close to {modified_vel[0, 0]}, got {stepped_vel[0, 0]}",
        )

    async def test_dof_velocities(self) -> None:
        """Test getting and setting DOF velocities."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current DOF velocities
        initial_dof_vels = articulations.get_dof_velocities()
        self.assertIsNotNone(initial_dof_vels, "DOF velocities should not be None")

        initial_dof_vels_np = (
            initial_dof_vels.numpy() if hasattr(initial_dof_vels, "numpy") else np.array(initial_dof_vels)
        )
        self.assertEqual(
            initial_dof_vels_np.shape,
            (articulations.count, articulations.max_dofs),
            f"DOF velocity shape should be ({articulations.count}, {articulations.max_dofs})",
        )

        # Modify all DOF velocities
        modified_dof_vels = initial_dof_vels_np.copy()
        modified_dof_vels[0, :] = 0.5

        # Set modified values into simulator
        dof_vels_wp = wp.from_numpy(modified_dof_vels, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_velocities(dof_vels_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_dof_vels = articulations.get_dof_velocities().numpy()
        self.assertTrue(
            np.allclose(immediate_dof_vels[0], modified_dof_vels[0], atol=1e-5),
            "All DOF velocities (before step) should match set values",
        )

        # Take a step and check values are close (looser tolerance)
        await omni.kit.app.get_app().next_update_async()

        stepped_dof_vels = articulations.get_dof_velocities().numpy()
        # Velocities may be affected by damping, constraints, etc.
        self.assertTrue(
            np.allclose(stepped_dof_vels[0], modified_dof_vels[0], atol=1.0),
            "All DOF velocities (after step) should be close to set values",
        )

    async def test_dof_limits(self) -> None:
        """Test getting DOF limits."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        limits = articulations.get_dof_limits()
        self.assertIsNotNone(limits, "DOF limits should not be None")

        limits_np = limits.numpy() if hasattr(limits, "numpy") else np.array(limits)
        self.assertEqual(
            limits_np.shape,
            (articulations.count, articulations.max_dofs, 2),
            f"Limits shape should be ({articulations.count}, {articulations.max_dofs}, 2)",
        )

        self.assertTrue(
            np.all(limits_np[:, :, 0] <= limits_np[:, :, 1]),
            "Lower limits should be less than or equal to upper limits",
        )

    # @unittest.skip("Issue after upgrading to mujoco 3.5.0")
    async def test_dof_positions(self) -> None:
        """Test getting and setting DOF positions."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get DOF limits to ensure we set valid values
        limits = articulations.get_dof_limits().numpy()

        # Get current DOF positions
        initial_pos = articulations.get_dof_positions()
        self.assertIsNotNone(initial_pos, "DOF positions should not be None")

        initial_pos_np = initial_pos.numpy() if hasattr(initial_pos, "numpy") else np.array(initial_pos)
        self.assertEqual(
            initial_pos_np.shape,
            (articulations.count, articulations.max_dofs),
            f"DOF position shape should be ({articulations.count}, {articulations.max_dofs})",
        )

        # Modify DOF positions (set all to mid-range to stay within limits)
        modified_pos = initial_pos_np.copy()
        for i in range(articulations.max_dofs):
            lower = limits[0, i, 0]
            upper = limits[0, i, 1]
            modified_pos[0, i] = (lower + upper) / 2.0

        # Set modified values into simulator
        pos_wp = wp.from_numpy(modified_pos, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_positions(pos_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_pos = articulations.get_dof_positions().numpy()
        self.assertTrue(
            np.allclose(immediate_pos[0], modified_pos[0], atol=1e-5),
            "All DOF positions (before step) should match set values",
        )

        # Take a step and check values are close (looser tolerance)
        await omni.kit.app.get_app().next_update_async()

        stepped_pos = articulations.get_dof_positions().numpy()
        # Positions should remain close unless acted on by forces
        self.assertTrue(
            np.allclose(stepped_pos[0], modified_pos[0], atol=0.1),
            "All DOF positions (after step) should be close to set values",
        )

    async def test_dof_stiffness_and_damping(self) -> None:
        """Test setting and getting DOF stiffness and damping."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current stiffnesses
        initial_stiff = articulations.get_dof_stiffnesses()
        self.assertIsNotNone(initial_stiff, "Stiffnesses should not be None")

        initial_stiff_np = initial_stiff.numpy() if hasattr(initial_stiff, "numpy") else np.array(initial_stiff)
        self.assertEqual(
            initial_stiff_np.shape,
            (articulations.count, articulations.max_dofs),
            f"Stiffness shape should be ({articulations.count}, {articulations.max_dofs})",
        )

        # Modify all stiffnesses
        modified_stiff = initial_stiff_np.copy()
        modified_stiff[0, :] = 1000.0

        # Set modified values into simulator
        stiff_wp = wp.from_numpy(modified_stiff, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_stiffnesses(stiff_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_stiff = articulations.get_dof_stiffnesses().numpy()
        self.assertTrue(
            np.allclose(immediate_stiff[0], modified_stiff[0], atol=1e-5),
            "All stiffnesses (before step) should match set values",
        )

        # Take a step and check values remain unchanged (stiffness is constant)
        await omni.kit.app.get_app().next_update_async()

        stepped_stiff = articulations.get_dof_stiffnesses().numpy()
        self.assertTrue(
            np.allclose(stepped_stiff[0], modified_stiff[0], atol=1e-5),
            "All stiffnesses (after step) should remain unchanged",
        )

        # Get current dampings
        initial_damp = articulations.get_dof_dampings()
        self.assertIsNotNone(initial_damp, "Dampings should not be None")

        initial_damp_np = initial_damp.numpy() if hasattr(initial_damp, "numpy") else np.array(initial_damp)
        self.assertEqual(
            initial_damp_np.shape,
            (articulations.count, articulations.max_dofs),
            f"Damping shape should be ({articulations.count}, {articulations.max_dofs})",
        )

        # Modify all dampings
        modified_damp = initial_damp_np.copy()
        modified_damp[0, :] = 50.0

        # Set modified values into simulator
        damp_wp = wp.from_numpy(modified_damp, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_dampings(damp_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_damp = articulations.get_dof_dampings().numpy()
        self.assertTrue(
            np.allclose(immediate_damp[0], modified_damp[0], atol=1e-5),
            "All dampings (before step) should match set values",
        )

        # Take a step and check values remain unchanged (damping is constant)
        await omni.kit.app.get_app().next_update_async()

        stepped_damp = articulations.get_dof_dampings().numpy()
        self.assertTrue(
            np.allclose(stepped_damp[0], modified_damp[0], atol=1e-5),
            "All dampings (after step) should remain unchanged",
        )

    async def test_dof_position_targets(self) -> None:
        """Test setting and getting DOF position targets."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current position targets
        initial_targets = articulations.get_dof_position_targets()
        self.assertIsNotNone(initial_targets, "DOF position targets should not be None")

        initial_targets_np = initial_targets.numpy() if hasattr(initial_targets, "numpy") else np.array(initial_targets)
        self.assertEqual(
            initial_targets_np.shape,
            (articulations.count, articulations.max_dofs),
            f"Position targets shape should be ({articulations.count}, {articulations.max_dofs})",
        )

        # Modify all position targets
        modified_targets = initial_targets_np.copy()
        modified_targets[0, :] = 0.1

        # Set modified values into simulator
        targets_wp = wp.from_numpy(modified_targets, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_position_targets(targets_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_targets = articulations.get_dof_position_targets().numpy()
        self.assertTrue(
            np.allclose(immediate_targets[0], modified_targets[0], atol=1e-5),
            "All position targets (before step) should match set values",
        )

        # Take a step and check values remain unchanged (targets are constant)
        await omni.kit.app.get_app().next_update_async()

        stepped_targets = articulations.get_dof_position_targets().numpy()
        self.assertTrue(
            np.allclose(stepped_targets[0], modified_targets[0], atol=1e-5),
            "All position targets (after step) should remain unchanged",
        )

    async def test_dof_actuation_forces(self) -> None:
        """Test getting DOF actuation forces."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        forces = articulations.get_dof_actuation_forces()
        self.assertIsNotNone(forces, "Actuation forces should not be None")

        forces_np = forces.numpy() if hasattr(forces, "numpy") else np.array(forces)
        self.assertEqual(
            forces_np.shape,
            (articulations.count, articulations.max_dofs),
            f"Forces shape should be ({articulations.count}, {articulations.max_dofs})",
        )

    async def test_mass_properties(self) -> None:
        """Test getting and setting articulation masses."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current masses
        initial_masses = articulations.get_masses()
        self.assertIsNotNone(initial_masses, "Masses should not be None")

        initial_masses_np = initial_masses.numpy() if hasattr(initial_masses, "numpy") else np.array(initial_masses)
        expected_shape = (articulations.count, articulations.max_links)
        self.assertEqual(initial_masses_np.shape, expected_shape, f"Masses shape should be {expected_shape}")

        # Verify all masses are non-negative
        self.assertTrue(np.all(initial_masses_np >= 0), "All masses should be non-negative")

        # Modify all link masses
        modified_masses = initial_masses_np.copy()
        modified_masses[0, :] = initial_masses_np[0, :] * 1.5

        # Set modified values into simulator
        masses_wp = wp.from_numpy(modified_masses, dtype=wp.float32, device=self.wp_device)
        articulations.set_masses(masses_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_masses = articulations.get_masses().numpy()
        self.assertTrue(
            np.allclose(immediate_masses[0], modified_masses[0], atol=1e-5),
            "All masses (before step) should match set values",
        )

        # Take a step and check values remain unchanged (mass is constant)
        await omni.kit.app.get_app().next_update_async()

        stepped_masses = articulations.get_masses().numpy()
        self.assertTrue(
            np.allclose(stepped_masses[0], modified_masses[0], atol=1e-5),
            "All masses (after step) should remain unchanged",
        )

    async def test_inertia_properties(self) -> None:
        """Test getting and setting articulation inertias."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current inertias
        initial_inertias = articulations.get_inertias()
        self.assertIsNotNone(initial_inertias, "Inertias should not be None")

        initial_inertias_np = (
            initial_inertias.numpy() if hasattr(initial_inertias, "numpy") else np.array(initial_inertias)
        )
        self.assertEqual(len(initial_inertias_np.shape), 3, "Inertias should be 3D array")
        self.assertEqual(initial_inertias_np.shape[0], articulations.count, "First dimension should be count")

        # Verify diagonal elements (Ixx, Iyy, Izz) are non-negative
        if initial_inertias_np.shape[2] == 9:
            # Flat format: [Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz]
            Ixx = initial_inertias_np[:, :, 0]
            Iyy = initial_inertias_np[:, :, 4]
            Izz = initial_inertias_np[:, :, 8]
            self.assertTrue(np.all(Ixx >= 0), "Ixx should be non-negative")
            self.assertTrue(np.all(Iyy >= 0), "Iyy should be non-negative")
            self.assertTrue(np.all(Izz >= 0), "Izz should be non-negative")

            # Modify all link inertias
            modified_inertias = initial_inertias_np.copy()
            modified_inertias[0, :, :] = initial_inertias_np[0, :, :] * 1.2
        else:
            # 3x3 matrix format
            self.assertTrue(np.all(initial_inertias_np[:, :, 0, 0] >= 0), "Ixx should be non-negative")
            self.assertTrue(np.all(initial_inertias_np[:, :, 1, 1] >= 0), "Iyy should be non-negative")
            self.assertTrue(np.all(initial_inertias_np[:, :, 2, 2] >= 0), "Izz should be non-negative")

            # Modify all link inertias
            modified_inertias = initial_inertias_np.copy()
            modified_inertias[0, :, :, :] = initial_inertias_np[0, :, :, :] * 1.2

        # Set modified values into simulator
        inertias_wp = wp.from_numpy(modified_inertias, dtype=wp.float32, device=self.wp_device)
        articulations.set_inertias(inertias_wp, indices)

        # Check immediately before step - should match exactly (strict tolerance)
        immediate_inertias = articulations.get_inertias().numpy()
        self.assertTrue(
            np.allclose(immediate_inertias[0], modified_inertias[0], atol=1e-5),
            "All inertias (before step) should match set values",
        )

        # Take a step and check values remain unchanged (inertia is constant)
        await omni.kit.app.get_app().next_update_async()

        stepped_inertias = articulations.get_inertias().numpy()
        self.assertTrue(
            np.allclose(stepped_inertias[0], modified_inertias[0], atol=1e-5),
            "All inertias (after step) should remain unchanged",
        )

    async def test_simulation_stepping(self) -> None:
        """Test that simulation can step with articulations."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        initial_pos = articulations.get_dof_positions().numpy()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        current_pos = articulations.get_dof_positions().numpy()
        self.assertIsNotNone(current_pos, "Should still be able to get positions after stepping")
        self.assertEqual(current_pos.shape, initial_pos.shape, "Shape should remain consistent")

    async def test_articulation_properties(self) -> None:
        """Test articulation view properties match expected humanoid structure."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        # Humanoid has 22 links (torso + 21 child links)
        self.assertGreater(articulations.max_links, 15, "Humanoid should have more than 15 links")
        self.assertLess(articulations.max_links, 30, "Humanoid should have less than 30 links")

        # Humanoid has 21 DOFs
        self.assertEqual(articulations.max_dofs, 21, "Humanoid should have exactly 21 DOFs")

        # max_shapes should match collision geometry count
        self.assertGreater(articulations.max_shapes, 0, "Should have collision shapes")

        # Single articulation should be homogeneous
        self.assertTrue(articulations.is_homogeneous, "Single articulation should be homogeneous")

        # Jacobian shape: (max_links * 6, max_dofs) for fixed base or (max_links * 6, max_dofs + 6) for floating
        jacobian_shape = articulations.jacobian_shape
        self.assertEqual(jacobian_shape[0] % 6, 0, "Jacobian rows should be multiple of 6")
        self.assertGreaterEqual(jacobian_shape[1], articulations.max_dofs, "Jacobian cols should be >= max_dofs")

        # Mass matrix shape: (n, n) where n = max_dofs (fixed) or max_dofs + 6 (floating)
        mass_matrix_shape = articulations.generalized_mass_matrix_shape
        self.assertEqual(mass_matrix_shape[0], mass_matrix_shape[1], "Mass matrix should be square")
        self.assertGreaterEqual(mass_matrix_shape[0], articulations.max_dofs, "Mass matrix size should be >= max_dofs")

    async def test_paths_and_names(self) -> None:
        """Test articulation path and name properties contain expected humanoid structure."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        # Test prim_paths - should contain the torso root path
        prim_paths = articulations.prim_paths
        self.assertEqual(len(prim_paths), 1, "Should have exactly one articulation")
        self.assertIn("torso", prim_paths[0].lower(), "Root path should contain 'torso'")

        # Base path for links
        links_base = "/nv_humanoid/"

        # Test link_names - verify exact humanoid link names (16 links)
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
        link_names = articulations.link_names[0]
        self.assertEqual(list(link_names), expected_link_names, "Link names mismatch")

        # Test link_paths - verify exact paths (derived from link_names)
        expected_link_paths = [links_base + name for name in expected_link_names]
        link_paths = articulations.link_paths[0]
        self.assertEqual(list(link_paths), expected_link_paths, "Link paths mismatch")

        # Base path for joints
        joints_base = "/nv_humanoid/joints/"

        # Test joint_names - verify exact humanoid joint names
        expected_joint_names = [
            "left_upper_arm",
            "left_lower_arm",
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
        ]
        joint_names = articulations.joint_names[0]
        self.assertEqual(list(joint_names), expected_joint_names, "Joint names mismatch")

        # Test joint_paths - verify exact paths
        expected_joint_paths = [joints_base + name for name in expected_joint_names]
        joint_paths = articulations.joint_paths[0]
        self.assertEqual(list(joint_paths), expected_joint_paths, "Joint paths mismatch")

        # Test dof_names - verify exact DOF names (21 DOFs for humanoid with ball joints)
        expected_dof_names = [
            "left_upper_arm:0",
            "left_upper_arm:2",
            "left_lower_arm",
            "lower_waist:0",
            "lower_waist:1",
            "pelvis",
            "left_thigh:0",
            "left_thigh:1",
            "left_thigh:2",
            "left_shin",
            "left_foot:0",
            "left_foot:1",
            "right_thigh:0",
            "right_thigh:1",
            "right_thigh:2",
            "right_shin",
            "right_foot:0",
            "right_foot:1",
            "right_upper_arm:0",
            "right_upper_arm:2",
            "right_lower_arm",
        ]
        dof_names = articulations.dof_names[0]
        self.assertEqual(list(dof_names), expected_dof_names, "DOF names mismatch")

        # Test dof_paths - verify exact paths (derived from dof_names)
        expected_dof_paths = [joints_base + name for name in expected_dof_names]
        dof_paths = articulations.dof_paths[0]
        self.assertEqual(list(dof_paths), expected_dof_paths, "DOF paths mismatch")

    async def test_inv_masses(self) -> None:
        """Test inverse masses are consistent with masses."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        masses = articulations.get_masses().numpy()
        inv_masses = articulations.get_inv_masses().numpy()

        expected_shape = (articulations.count, articulations.max_links)
        self.assertEqual(inv_masses.shape, expected_shape, f"Inverse masses shape should be {expected_shape}")

        # Inverse masses should be 1/mass for non-zero masses
        for i in range(articulations.max_links):
            if masses[0, i] > 1e-6:
                expected_inv = 1.0 / masses[0, i]
                self.assertAlmostEqual(
                    inv_masses[0, i],
                    expected_inv,
                    places=4,
                    msg=f"inv_mass[{i}]={inv_masses[0, i]} should be 1/mass={expected_inv}",
                )

    async def test_coms(self) -> None:
        """Test centers of mass have valid quaternion orientations and can be modified."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        initial_coms = articulations.get_coms().numpy()
        expected_shape = (articulations.count, articulations.max_links, 7)
        self.assertEqual(initial_coms.shape, expected_shape, f"COMs shape should be {expected_shape}")

        # COMs format is [x, y, z, qx, qy, qz, qw] - check quaternions are normalized
        for link_idx in range(articulations.max_links):
            quat = initial_coms[0, link_idx, 3:7]  # qx, qy, qz, qw
            quat_norm = np.linalg.norm(quat)
            self.assertAlmostEqual(
                quat_norm,
                1.0,
                places=3,
                msg=f"COM quaternion for link {link_idx} should be normalized, got norm={quat_norm}",
            )

        # Modify COM positions and verify round-trip
        offset = 0.05
        modified_coms = initial_coms.copy()
        modified_coms[0, :, 0] += offset  # Shift all COMs in x

        coms_wp = wp.from_numpy(modified_coms, dtype=wp.float32, device=self.wp_device)
        articulations.set_coms(coms_wp, indices)

        updated_coms = articulations.get_coms().numpy()
        self.assertTrue(
            np.allclose(updated_coms[0, :, 0], initial_coms[0, :, 0] + offset, atol=1e-5),
            f"COM x positions should be offset by {offset}",
        )

    async def test_inv_inertias(self) -> None:
        """Test getting inverse inertias."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        inv_inertias = articulations.get_inv_inertias()
        self.assertIsNotNone(inv_inertias, "Inverse inertias should not be None")

        inv_inertias_np = inv_inertias.numpy() if hasattr(inv_inertias, "numpy") else np.array(inv_inertias)
        self.assertEqual(len(inv_inertias_np.shape), 3, "Inverse inertias should be 3D array")
        self.assertEqual(inv_inertias_np.shape[0], articulations.count, "First dimension should be count")
        self.assertEqual(inv_inertias_np.shape[1], articulations.max_links, "Second dimension should be max_links")

    async def test_dof_armatures(self) -> None:
        """Test getting and setting DOF armatures."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current armatures
        initial_armatures = articulations.get_dof_armatures()
        self.assertIsNotNone(initial_armatures, "Armatures should not be None")

        initial_armatures_np = (
            initial_armatures.numpy() if hasattr(initial_armatures, "numpy") else np.array(initial_armatures)
        )
        expected_shape = (articulations.count, articulations.max_dofs)
        self.assertEqual(initial_armatures_np.shape, expected_shape, f"Armatures shape should be {expected_shape}")

        # Modify armatures
        modified_armatures = initial_armatures_np.copy()
        modified_armatures[0, :] = 0.01

        # Set modified values
        armatures_wp = wp.from_numpy(modified_armatures, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_armatures(armatures_wp, indices)

        # Check values were set
        updated_armatures = articulations.get_dof_armatures().numpy()
        self.assertTrue(
            np.allclose(updated_armatures[0], modified_armatures[0], atol=1e-5),
            "Armatures should match set values",
        )

    async def test_dof_velocity_targets(self) -> None:
        """Test getting and setting DOF velocity targets."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current velocity targets
        initial_targets = articulations.get_dof_velocity_targets()
        self.assertIsNotNone(initial_targets, "Velocity targets should not be None")

        initial_targets_np = initial_targets.numpy() if hasattr(initial_targets, "numpy") else np.array(initial_targets)
        expected_shape = (articulations.count, articulations.max_dofs)
        self.assertEqual(initial_targets_np.shape, expected_shape, f"Velocity targets shape should be {expected_shape}")

        # Modify velocity targets
        modified_targets = initial_targets_np.copy()
        modified_targets[0, :] = 0.5

        # Set modified values
        targets_wp = wp.from_numpy(modified_targets, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_velocity_targets(targets_wp, indices)

        # Check values were set
        updated_targets = articulations.get_dof_velocity_targets().numpy()
        self.assertTrue(
            np.allclose(updated_targets[0], modified_targets[0], atol=1e-5),
            "Velocity targets should match set values",
        )

    async def test_set_dof_actuation_forces(self) -> None:
        """Test setting DOF actuation forces."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Create forces array
        forces = np.zeros((articulations.count, articulations.max_dofs), dtype=np.float32)
        forces[0, :] = 10.0

        # Set forces
        forces_wp = wp.from_numpy(forces, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_actuation_forces(forces_wp, indices)

        # Check values were set
        updated_forces = articulations.get_dof_actuation_forces().numpy()
        self.assertTrue(
            np.allclose(updated_forces[0], forces[0], atol=1e-5),
            "Actuation forces should match set values",
        )

    async def test_dof_max_forces(self) -> None:
        """Test getting and setting DOF max forces (effort limits)."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current max forces
        initial_max_forces = articulations.get_dof_max_forces()
        self.assertIsNotNone(initial_max_forces, "Max forces should not be None")

        initial_max_forces_np = (
            initial_max_forces.numpy() if hasattr(initial_max_forces, "numpy") else np.array(initial_max_forces)
        )
        expected_shape = (articulations.count, articulations.max_dofs)
        self.assertEqual(initial_max_forces_np.shape, expected_shape, f"Max forces shape should be {expected_shape}")

        # Modify max forces
        modified_max_forces = initial_max_forces_np.copy()
        modified_max_forces[0, :] = 100.0

        # Set modified values
        max_forces_wp = wp.from_numpy(modified_max_forces, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_max_forces(max_forces_wp, indices)

        # Check values were set
        updated_max_forces = articulations.get_dof_max_forces().numpy()
        self.assertTrue(
            np.allclose(updated_max_forces[0], modified_max_forces[0], atol=1e-5),
            "Max forces should match set values",
        )

    async def test_set_dof_limits(self) -> None:
        """Test setting DOF limits."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current limits
        initial_limits = articulations.get_dof_limits().numpy()

        # Modify limits (make them slightly narrower)
        modified_limits = initial_limits.copy()
        modified_limits[0, :, 0] = initial_limits[0, :, 0] + 0.01  # Raise lower limit
        modified_limits[0, :, 1] = initial_limits[0, :, 1] - 0.01  # Lower upper limit

        # Set modified values
        limits_wp = wp.from_numpy(modified_limits, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_limits(limits_wp, indices)

        # Check values were set
        updated_limits = articulations.get_dof_limits().numpy()
        self.assertTrue(
            np.allclose(updated_limits[0], modified_limits[0], atol=1e-5),
            "DOF limits should match set values",
        )

    async def test_dof_max_velocities(self) -> None:
        """Test getting and setting DOF max velocities."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Get current max velocities
        initial_max_vels = articulations.get_dof_max_velocities()
        self.assertIsNotNone(initial_max_vels, "Max velocities should not be None")

        initial_max_vels_np = (
            initial_max_vels.numpy() if hasattr(initial_max_vels, "numpy") else np.array(initial_max_vels)
        )
        expected_shape = (articulations.count, articulations.max_dofs)
        self.assertEqual(initial_max_vels_np.shape, expected_shape, f"Max velocities shape should be {expected_shape}")

        # Modify max velocities
        modified_max_vels = initial_max_vels_np.copy()
        modified_max_vels[0, :] = 10.0

        # Set modified values
        max_vels_wp = wp.from_numpy(modified_max_vels, dtype=wp.float32, device=self.wp_device)
        articulations.set_dof_max_velocities(max_vels_wp, indices)

        # Check values were set
        updated_max_vels = articulations.get_dof_max_velocities().numpy()
        self.assertTrue(
            np.allclose(updated_max_vels[0], modified_max_vels[0], atol=1e-5),
            "Max velocities should match set values",
        )

    async def test_apply_forces_and_torques(self) -> None:
        """Test applying forces and torques to articulation links."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Create force and torque arrays
        forces = np.zeros((articulations.count, articulations.max_links, 3), dtype=np.float32)
        forces[0, 0, 2] = 10.0  # Apply upward force to first link

        torques = np.zeros((articulations.count, articulations.max_links, 3), dtype=np.float32)
        torques[0, 0, 2] = 1.0  # Apply torque around z-axis

        # Apply forces and torques
        forces_wp = wp.from_numpy(forces, dtype=wp.float32, device=self.wp_device)
        torques_wp = wp.from_numpy(torques, dtype=wp.float32, device=self.wp_device)

        # This should not raise an error
        articulations.apply_forces_and_torques_at_position(forces_wp, torques_wp, None, indices, is_global=True)

        # Step simulation to apply forces
        await omni.kit.app.get_app().next_update_async()

    async def test_check_method(self) -> None:
        """Test the check method returns correct state."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")

        result = articulations.check()
        self.assertTrue(result, "check() should return True for valid articulation view")

    async def test_effort_causes_velocity_change(self) -> None:
        """Test that set_dof_actuation_forces produces joint motion."""
        articulations = self.sim.create_articulation_view("/nv_humanoid/torso*")
        indices = wp.from_numpy(np.arange(articulations.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        zero_stiffness = np.zeros((articulations.count, articulations.max_dofs), dtype=np.float32)
        zero_damping = np.zeros((articulations.count, articulations.max_dofs), dtype=np.float32)
        articulations.set_dof_stiffnesses(
            wp.from_numpy(zero_stiffness, dtype=wp.float32, device=self.wp_device), indices
        )
        articulations.set_dof_dampings(wp.from_numpy(zero_damping, dtype=wp.float32, device=self.wp_device), indices)

        zero_vel = np.zeros((articulations.count, articulations.max_dofs), dtype=np.float32)
        articulations.set_dof_velocities(wp.from_numpy(zero_vel, dtype=wp.float32, device=self.wp_device), indices)

        vel_before = articulations.get_dof_velocities().numpy().copy()

        torques = np.zeros((articulations.count, articulations.max_dofs), dtype=np.float32)
        torques[:, 0] = 50.0
        torques_wp = wp.from_numpy(torques, dtype=wp.float32, device=self.wp_device)

        for _ in range(5):
            articulations.set_dof_actuation_forces(torques_wp, indices)
            await omni.kit.app.get_app().next_update_async()

        vel_after = articulations.get_dof_velocities().numpy()
        delta = np.abs(vel_after[0, 0] - vel_before[0, 0])
        self.assertGreater(delta, 0.01, "DOF 0 velocity should change after applying effort")
