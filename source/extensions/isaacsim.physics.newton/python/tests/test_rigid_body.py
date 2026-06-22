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

"""Verifies the Newton rigid body view implementation against authored rigid bodies and imported assets. The tests cover creation, paths, transforms, velocities, accelerations, mass and inertia properties, gravity flags, force application, simulation stepping, and validity checks."""

import os
import shutil
import tempfile
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
from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


async def wait_for_stage_loading() -> Any:
    """Wait until USD stage loading is complete.

    Returns:
        None.
    """
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        await omni.kit.app.get_app().next_update_async()


_DEFAULT_VARIANT_TO_ENGINE = {"physx": "physx", "mujoco": "newton"}


async def step_physics_variants(
    prim: Any,
    timeline: Any,
    *,
    variants: Any = ("physx", "mujoco"),
    variant_to_engine: Any = None,
    num_frames: int = 30,
    settle_frames: int = 5,
) -> None:
    """Step the simulation through each ``Physics`` USD variant on ``prim``.

    For each variant the helper sets the selection on the prim's ``Physics``
    variant set, switches the runtime physics engine to the one that
    consumes that variant's schemas via
    :meth:`SimulationManager.switch_physics_engine`, plays the timeline,
    ticks ``num_frames`` simulation frames via ``next_update_async``,
    stops the timeline, and ticks ``settle_frames`` more frames so the
    stop event is fully processed before the next variant switch.  The
    assertion is implicit: nothing should crash while stepping.

    Args:
        prim: USD prim that carries the ``Physics`` variant set.
        timeline: ``omni.timeline`` timeline interface to drive.
        variants: Variant names to exercise in order.
        variant_to_engine: Optional mapping from variant name to the
            :meth:`SimulationManager.switch_physics_engine` engine name
            (``"physx"``, ``"newton"``, or ``"remotesim"``).  Defaults to
            ``{"physx": "physx", "mujoco": "newton"}``.
        num_frames: Number of frames to step while the timeline is playing.
        settle_frames: Number of frames to tick after stopping the timeline.
    """
    mapping = _DEFAULT_VARIANT_TO_ENGINE if variant_to_engine is None else variant_to_engine
    for variant in variants:
        prim.GetVariantSet("Physics").SetVariantSelection(variant)
        await omni.kit.app.get_app().next_update_async()

        engine = mapping.get(variant)
        if engine is not None:
            assert SimulationManager.switch_physics_engine(engine), f"Failed to switch to {engine} physics engine"
            await omni.kit.app.get_app().next_update_async()

        timeline.play()
        for _ in range(num_frames):
            await omni.kit.app.get_app().next_update_async()
        timeline.stop()
        for _ in range(settle_frames):
            await omni.kit.app.get_app().next_update_async()


class TestNewtonRigidBodyView(omni.kit.test.AsyncTestCase):
    """Tests for Newton rigid body view tensor API."""

    async def setUp(self) -> None:
        """Set up test environment with rigid bodies."""
        self.use_gpu = True
        self.wp_device = "cuda:0" if self.use_gpu else "cpu"

        await stage_utils.create_new_stage_async()
        self.stage = omni.usd.get_context().get_stage()

        # Create physics scene
        scene_path = "/PhysicsScene"
        scene = UsdPhysics.Scene.Define(self.stage, scene_path)
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)

        # Create multiple rigid bodies (cubes)
        self.num_bodies = 3
        self.body_paths = []
        for i in range(self.num_bodies):
            body_path = f"/World/Cube_{i}"
            self.body_paths.append(body_path)

            # Create cube geometry
            cube = UsdGeom.Cube.Define(self.stage, body_path)
            cube.GetSizeAttr().Set(0.5)
            cube.AddTranslateOp().Set(Gf.Vec3f(i * 2.0, 0.0, 1.0))

            # Add rigid body physics
            prim = self.stage.GetPrimAtPath(body_path)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim)

            # Set mass and inertia
            mass_api = UsdPhysics.MassAPI(prim)
            mass_api.CreateMassAttr(1.0)
            # Inertia for uniform cube: I = (1/6) * m * s^2
            cube_size = 0.5
            cube_mass = 1.0
            cube_inertia = (1.0 / 6.0) * cube_mass * cube_size * cube_size
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(cube_inertia, cube_inertia, cube_inertia))

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

    async def test_rigid_body_view_creation(self) -> None:
        """Test creating rigid body view and basic properties."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        self.assertIsNotNone(bodies)
        self.assertEqual(bodies.count, self.num_bodies, f"Should have {self.num_bodies} rigid bodies")

    async def test_body_paths_and_names(self) -> None:
        """Test rigid body paths and names match expected cube structure."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        # Verify body paths are correct absolute USD paths
        body_paths = bodies.body_paths
        self.assertEqual(len(body_paths), self.num_bodies, f"Should have {self.num_bodies} body paths")
        for i, path in enumerate(body_paths):
            self.assertTrue(path.startswith("/World/Cube_"), f"Path should start with /World/Cube_, got: {path}")
            self.assertIn("Cube_", path, f"Path should contain 'Cube_', got: {path}")

        # Verify body names are extracted correctly
        body_names = bodies.body_names
        self.assertEqual(len(body_names), self.num_bodies, f"Should have {self.num_bodies} body names")
        for name in body_names:
            self.assertIn("Cube_", name, f"Name should contain 'Cube_', got: {name}")

    async def test_transforms(self) -> None:
        """Test transforms have correct format and values match scene setup."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")
        indices = wp.from_numpy(np.arange(bodies.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        initial_transforms = bodies.get_transforms().numpy()
        self.assertEqual(initial_transforms.shape, (bodies.count, 7), "Transform shape should be (count, 7)")

        # Transforms are [x, y, z, qx, qy, qz, qw] - verify quaternions are normalized
        for i in range(bodies.count):
            quat = initial_transforms[i, 3:7]
            quat_norm = np.linalg.norm(quat)
            self.assertAlmostEqual(quat_norm, 1.0, places=3, msg=f"Quaternion for body {i} should be normalized")

        # Verify initial positions match scene setup (cubes at x=0, 2, 4, y=0, z=1)
        for i in range(bodies.count):
            self.assertAlmostEqual(initial_transforms[i, 1], 0.0, places=2, msg=f"Body {i} y should be ~0")
            self.assertAlmostEqual(initial_transforms[i, 2], 1.0, places=2, msg=f"Body {i} z should be ~1")

        # Modify transforms and verify round-trip
        height_offset = 2.0
        modified_transforms = initial_transforms.copy()
        modified_transforms[:, 2] += height_offset

        transform_wp = wp.from_numpy(modified_transforms, dtype=wp.float32, device=self.wp_device)
        bodies.set_transforms(transform_wp, indices)

        updated_transforms = bodies.get_transforms().numpy()
        for i in range(bodies.count):
            expected_z = initial_transforms[i, 2] + height_offset
            self.assertAlmostEqual(
                updated_transforms[i, 2],
                expected_z,
                places=4,
                msg=f"Body {i} z should be {expected_z}, got {updated_transforms[i, 2]}",
            )

    async def test_velocities(self) -> None:
        """Test velocities format and verify set values are applied."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")
        indices = wp.from_numpy(np.arange(bodies.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        initial_vel = bodies.get_velocities().numpy()
        self.assertEqual(initial_vel.shape, (bodies.count, 6), "Velocity shape should be (count, 6)")

        # Velocities are [vx, vy, vz, wx, wy, wz] - linear then angular
        # Note: Bodies may have some velocity from gravity during initial simulation step

        # Set specific velocities
        target_linear_x = 2.0
        target_angular_z = 1.0
        modified_vel = np.zeros((bodies.count, 6), dtype=np.float32)
        modified_vel[:, 0] = target_linear_x  # vx
        modified_vel[:, 5] = target_angular_z  # wz

        vel_wp = wp.from_numpy(modified_vel, dtype=wp.float32, device=self.wp_device)
        bodies.set_velocities(vel_wp, indices)

        # Verify set values immediately
        immediate_vel = bodies.get_velocities().numpy()
        for i in range(bodies.count):
            self.assertAlmostEqual(
                immediate_vel[i, 0], target_linear_x, places=4, msg=f"Body {i} vx should be {target_linear_x}"
            )
            self.assertAlmostEqual(
                immediate_vel[i, 5], target_angular_z, places=4, msg=f"Body {i} wz should be {target_angular_z}"
            )

    async def test_accelerations_without_body_qdd(self) -> None:
        """get_accelerations returns None when body_qdd is not allocated."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        accelerations = bodies.get_accelerations()
        self.assertIsNone(accelerations, "Accelerations should be None when body_qdd is not requested")

    async def test_masses(self) -> None:
        """Test masses match scene setup and can be modified."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")
        indices = wp.from_numpy(np.arange(bodies.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        initial_masses = bodies.get_masses().numpy()

        # All cubes were created with mass=1.0
        for i in range(bodies.count):
            self.assertAlmostEqual(
                initial_masses[i, 0], 1.0, places=2, msg=f"Body {i} mass should be ~1.0, got {initial_masses[i, 0]}"
            )

        # Double all masses and verify
        scale_factor = 2.0
        modified_masses = initial_masses * scale_factor
        masses_wp = wp.from_numpy(modified_masses, dtype=wp.float32, device=self.wp_device)
        bodies.set_masses(masses_wp, indices)

        updated_masses = bodies.get_masses().numpy()
        for i in range(bodies.count):
            expected_mass = initial_masses[i, 0] * scale_factor
            self.assertAlmostEqual(
                updated_masses[i, 0],
                expected_mass,
                places=4,
                msg=f"Body {i} mass should be {expected_mass}, got {updated_masses[i, 0]}",
            )

    async def test_inv_masses(self) -> None:
        """Test inverse masses are consistent with masses (inv_mass = 1/mass)."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        masses = bodies.get_masses().numpy()
        inv_masses = bodies.get_inv_masses().numpy()

        # Inverse mass should equal 1/mass for each body
        for i in range(bodies.count):
            if masses[i, 0] > 1e-6:
                expected_inv = 1.0 / masses[i, 0]
                self.assertAlmostEqual(
                    inv_masses[i, 0],
                    expected_inv,
                    places=4,
                    msg=f"inv_mass[{i}]={inv_masses[i, 0]} should be 1/mass={expected_inv}",
                )

    async def test_coms(self) -> None:
        """Test centers of mass have valid format and can be modified."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")
        indices = wp.from_numpy(np.arange(bodies.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        initial_coms = bodies.get_coms().numpy()
        self.assertEqual(initial_coms.shape, (bodies.count, 7), "COMs shape should be (count, 7)")

        # COMs format is [x, y, z, qx, qy, qz, qw]
        # For uniform cubes, COM position should be at geometric center (0, 0, 0) in local frame
        # and quaternion should be normalized
        for i in range(bodies.count):
            # Check COM position is near origin (local frame)
            com_pos = initial_coms[i, 0:3]
            self.assertTrue(
                np.allclose(com_pos, [0, 0, 0], atol=0.1), f"Body {i} COM position should be near origin, got {com_pos}"
            )

            # Check quaternion is normalized
            quat = initial_coms[i, 3:7]
            quat_norm = np.linalg.norm(quat)
            self.assertAlmostEqual(quat_norm, 1.0, places=3, msg=f"Body {i} COM quaternion should be normalized")

        # Modify COM positions and verify
        offset = np.array([0.05, 0.0, 0.0])
        modified_coms = initial_coms.copy()
        modified_coms[:, 0:3] += offset

        coms_wp = wp.from_numpy(modified_coms, dtype=wp.float32, device=self.wp_device)
        bodies.set_coms(coms_wp, indices)

        updated_coms = bodies.get_coms().numpy()
        for i in range(bodies.count):
            expected_pos = initial_coms[i, 0:3] + offset
            self.assertTrue(
                np.allclose(updated_coms[i, 0:3], expected_pos, atol=1e-5),
                f"Body {i} COM position should be {expected_pos}, got {updated_coms[i, 0:3]}",
            )

    async def test_inertias(self) -> None:
        """Test inertias have correct format and diagonal elements for uniform cubes."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")
        indices = wp.from_numpy(np.arange(bodies.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        initial_inertias = bodies.get_inertias().numpy()
        self.assertEqual(initial_inertias.shape, (bodies.count, 9), "Inertias shape should be (count, 9)")

        # Inertia is flattened 3x3: [Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz]
        # For uniform cubes, diagonal elements should be equal (Ixx = Iyy = Izz)
        # and off-diagonal elements should be ~0
        for i in range(bodies.count):
            Ixx, Ixy, Ixz = initial_inertias[i, 0], initial_inertias[i, 1], initial_inertias[i, 2]
            Iyx, Iyy, Iyz = initial_inertias[i, 3], initial_inertias[i, 4], initial_inertias[i, 5]
            Izx, Izy, Izz = initial_inertias[i, 6], initial_inertias[i, 7], initial_inertias[i, 8]

            # Diagonal should be positive
            self.assertGreater(Ixx, 0, f"Body {i} Ixx should be positive")
            self.assertGreater(Iyy, 0, f"Body {i} Iyy should be positive")
            self.assertGreater(Izz, 0, f"Body {i} Izz should be positive")

            # For uniform cube, Ixx ≈ Iyy ≈ Izz
            self.assertAlmostEqual(Ixx, Iyy, places=3, msg=f"Body {i}: Ixx should equal Iyy for uniform cube")
            self.assertAlmostEqual(Iyy, Izz, places=3, msg=f"Body {i}: Iyy should equal Izz for uniform cube")

            # Off-diagonal should be ~0 for axis-aligned cube
            self.assertAlmostEqual(Ixy, 0, places=3, msg=f"Body {i}: Ixy should be ~0")
            self.assertAlmostEqual(Ixz, 0, places=3, msg=f"Body {i}: Ixz should be ~0")

        # Scale inertias and verify round-trip
        scale_factor = 1.5
        modified_inertias = initial_inertias * scale_factor
        inertias_wp = wp.from_numpy(modified_inertias, dtype=wp.float32, device=self.wp_device)
        bodies.set_inertias(inertias_wp, indices)

        updated_inertias = bodies.get_inertias().numpy()
        self.assertTrue(
            np.allclose(updated_inertias, modified_inertias, atol=1e-5),
            "Inertias should match scaled values",
        )

    async def test_inv_inertias(self) -> None:
        """Test inverse inertias are consistent with inertias."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        inertias = bodies.get_inertias().numpy()
        inv_inertias = bodies.get_inv_inertias().numpy()

        self.assertEqual(inv_inertias.shape, (bodies.count, 9), "Inverse inertias shape should be (count, 9)")

        # For diagonal inertia tensor, inverse diagonal should be 1/I
        # Check diagonal elements: indices 0 (Ixx), 4 (Iyy), 8 (Izz)
        for i in range(bodies.count):
            for diag_idx in [0, 4, 8]:
                if inertias[i, diag_idx] > 1e-6:
                    expected_inv = 1.0 / inertias[i, diag_idx]
                    self.assertAlmostEqual(
                        inv_inertias[i, diag_idx],
                        expected_inv,
                        places=3,
                        msg=f"Body {i} inv_inertia[{diag_idx}] should be 1/inertia",
                    )

    async def test_disable_simulations(self) -> None:
        """Test getting disable simulation flags."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        disable_sims = bodies.get_disable_simulations()
        self.assertIsNotNone(disable_sims, "Disable simulations should not be None")

    async def test_disable_gravities(self) -> None:
        """Test getting disable gravity flags."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        disable_gravities = bodies.get_disable_gravities()
        self.assertIsNotNone(disable_gravities, "Disable gravities should not be None")

    async def test_apply_forces_and_torques(self) -> None:
        """Test applying forces and torques to rigid bodies."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")
        indices = wp.from_numpy(np.arange(bodies.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Create force and torque arrays
        forces = np.zeros((bodies.count, 3), dtype=np.float32)
        forces[:, 2] = 100.0  # Apply upward force

        torques = np.zeros((bodies.count, 3), dtype=np.float32)
        torques[:, 2] = 10.0  # Apply torque around z-axis

        # Apply forces and torques
        forces_wp = wp.from_numpy(forces, dtype=wp.float32, device=self.wp_device)
        torques_wp = wp.from_numpy(torques, dtype=wp.float32, device=self.wp_device)

        # This should not raise an error
        bodies.apply_forces_and_torques_at_position(forces_wp, torques_wp, None, indices, is_global=True)

        # Step simulation to apply forces
        await omni.kit.app.get_app().next_update_async()

    async def test_apply_forces_at_position(self) -> None:
        """Test applying forces at specific positions."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")
        indices = wp.from_numpy(np.arange(bodies.count, dtype=np.int32), dtype=wp.int32, device=self.wp_device)

        # Create force and position arrays
        forces = np.zeros((bodies.count, 3), dtype=np.float32)
        forces[:, 2] = 50.0  # Apply upward force

        positions = np.zeros((bodies.count, 3), dtype=np.float32)
        positions[:, 0] = 0.1  # Offset position to create torque

        # Apply forces at positions
        forces_wp = wp.from_numpy(forces, dtype=wp.float32, device=self.wp_device)
        positions_wp = wp.from_numpy(positions, dtype=wp.float32, device=self.wp_device)

        # This should not raise an error
        bodies.apply_forces_and_torques_at_position(forces_wp, None, positions_wp, indices, is_global=True)

        # Step simulation to apply forces
        await omni.kit.app.get_app().next_update_async()

    async def test_simulation_stepping(self) -> None:
        """Test bodies fall under gravity during simulation."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        initial_transforms = bodies.get_transforms().numpy()
        initial_z_positions = initial_transforms[:, 2].copy()

        # Step simulation multiple times
        num_steps = 10
        for _ in range(num_steps):
            await omni.kit.app.get_app().next_update_async()

        current_transforms = bodies.get_transforms().numpy()
        current_z_positions = current_transforms[:, 2]

        # All bodies should have fallen (z decreased) due to gravity
        for i in range(bodies.count):
            self.assertLess(
                current_z_positions[i],
                initial_z_positions[i],
                f"Body {i} should have fallen: initial z={initial_z_positions[i]}, current z={current_z_positions[i]}",
            )

        # Verify significant fall (at least 0.1m after 10 steps with gravity)
        avg_fall = np.mean(initial_z_positions - current_z_positions)
        self.assertGreater(avg_fall, 0.1, f"Average fall distance should be > 0.1m, got {avg_fall}")

    async def test_check_method(self) -> None:
        """Test the check method returns correct state."""
        bodies = self.sim.create_rigid_body_view("/World/Cube_*")

        result = bodies.check()
        self.assertTrue(result, "check() should return True for valid rigid body view")

    async def test_ur10(self) -> None:
        """Import the UR10 URDF and run a few simulation frames in both physx and mujoco variants."""
        # Stop the timeline started in setUp before swapping the stage to avoid
        # stepping a stage we're about to replace.
        self.timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Locate the UR10 URDF shipped with the URDF importer extension.
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        ext_path = ext_manager.get_extension_path(ext_id)
        urdf_path = os.path.normpath(
            os.path.abspath(os.path.join(ext_path, "data", "urdf", "robots", "ur10", "urdf", "ur10.urdf"))
        )

        # Import the URDF to a temporary location and open it as the active stage.
        tmpdir = tempfile.mkdtemp(prefix="ur10_test_")
        try:
            importer = URDFImporter()
            config = URDFImporterConfig()
            config.urdf_path = urdf_path
            config.usd_path = tmpdir
            importer.config = config
            output_path = os.path.normpath(importer.import_urdf())

            omni.usd.get_context().open_stage(output_path)
            await wait_for_stage_loading()
            self.stage = omni.usd.get_context().get_stage()

            prim = self.stage.GetPrimAtPath("/ur10")
            self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath, "UR10 root prim should exist after import")

            # Add a minimal physics scene so the timeline has something to step against.
            scene = UsdPhysics.Scene.Define(self.stage, "/physicsScene")
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr(9.81)

            # Exercise both Physics USD variants; the Newton runtime consumes either schema set,
            # the assertion here is simply that nothing crashes while stepping.
            await step_physics_variants(prim, self.timeline)

            self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self.stage), 1.0)
        finally:
            # Detach the open stage from tmpdir before deleting so USD
            # listeners don't fault on disappearing layer files.
            self.timeline.stop()
            await omni.usd.get_context().new_stage_async()
            await wait_for_stage_loading()
            shutil.rmtree(tmpdir, ignore_errors=True)

    async def test_mjcf_humanoid(self) -> None:
        """Import the humanoid MJCF and run a few simulation frames in both physx and mujoco variants."""
        # Stop the timeline started in setUp before swapping the stage to avoid
        # stepping a stage we're about to replace.
        self.timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Locate the humanoid MJCF shipped with the MJCF importer extension.
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.mjcf")
        ext_path = ext_manager.get_extension_path(ext_id)
        mjcf_path = os.path.normpath(os.path.abspath(os.path.join(ext_path, "data", "mjcf", "nv_humanoid.xml")))

        # Import the MJCF to a temporary location and open it as the active stage.
        tmpdir = tempfile.mkdtemp(prefix="humanoid_test_")
        try:
            importer = MJCFImporter()
            config = MJCFImporterConfig(mjcf_path=mjcf_path)
            config.usd_path = tmpdir
            importer.config = config
            output_path = os.path.normpath(importer.import_mjcf())

            omni.usd.get_context().open_stage(output_path)
            await wait_for_stage_loading()
            self.stage = omni.usd.get_context().get_stage()

            prim = self.stage.GetPrimAtPath("/humanoid")
            self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath, "Humanoid root prim should exist after import")

            # Add a minimal physics scene so the timeline has something to step against.
            scene = UsdPhysics.Scene.Define(self.stage, "/physicsScene")
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr(9.81)

            # Exercise both Physics USD variants; the Newton runtime consumes either schema set,
            # the assertion here is simply that nothing crashes while stepping.
            await step_physics_variants(prim, self.timeline)

            self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(self.stage), 1.0)
        finally:
            # Detach the open stage from tmpdir before deleting so USD
            # listeners don't fault on disappearing layer files.
            self.timeline.stop()
            await omni.usd.get_context().new_stage_async()
            await wait_for_stage_loading()
            shutil.rmtree(tmpdir, ignore_errors=True)
