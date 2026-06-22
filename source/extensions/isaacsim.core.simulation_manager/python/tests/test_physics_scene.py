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

"""Verifies physics scene wrappers for PhysX and Newton MuJoCo scene attributes. The tests cover construction, time step settings, gravity, solver and GPU configuration, engine API cleanup, and Newton-specific solver parameters."""

import asyncio
import dataclasses

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import omni.timeline
from isaacsim.core.simulation_manager import PhysicsScene, PhysxGpuCfg, PhysxScene, SimulationManager
from isaacsim.core.simulation_manager.impl.mjc_scene import NewtonMjcScene
from omni.kit.app import get_app
from pxr import Gf, PhysxSchema


class TestPhysicsScene(omni.kit.test.AsyncTestCase):
    """Test the base PhysicsScene class with NewtonSceneAPI attributes."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    async def test_physics_scene_constructor(self) -> None:
        """Test physics scene constructor."""
        # no PhysicsScene in stage (create new)
        self.assertListEqual(PhysicsScene.get_physics_scene_paths(), [])
        path = "/physicsScene_0"
        # Use PhysxScene to create a physics scene (PhysicsScene applies NewtonSceneAPI)
        physics_scene = PhysxScene(path)
        self.assertEqual(physics_scene.path, path)
        self.assertEqual(physics_scene.prim.GetPath().pathString, path)
        self.assertTrue(physics_scene.prim.HasAPI("NewtonSceneAPI"))

    async def test_physics_scene_constructor_applies_default_engine_api(self) -> None:
        """Test base PhysicsScene is compatible with the default PhysX engine."""
        physics_scene = PhysicsScene("/World/physicsScene")

        self.assertTrue(physics_scene.prim.HasAPI("NewtonSceneAPI"))

    async def test_dt(self) -> None:
        """Test dt."""
        physics_scene = PhysicsScene("/World/physicsScene")
        if SimulationManager.get_active_physics_engine() == "physx":
            self.assertAlmostEqual(physics_scene.get_dt(), 1.0 / 60.0, places=5)
            newton_attr = physics_scene.prim.GetAttribute("newton:timeStepsPerSecond")
            self.assertEqual(newton_attr.Get(), 1000)

            for dt in [0.01, 0.005, 0.002]:
                physics_scene.set_dt(dt)
                self.assertAlmostEqual(physics_scene.get_dt(), dt, places=5)
                physx_scene_api = PhysxSchema.PhysxSceneAPI(physics_scene.prim)
                self.assertEqual(physx_scene_api.GetTimeStepsPerSecondAttr().Get(), int(1.0 / dt))
                self.assertEqual(newton_attr.Get(), 1000)
        else:
            # default: 1000 steps/sec = 0.001 dt (from NewtonSceneAPI)
            self.assertAlmostEqual(physics_scene.get_dt(), 0.001, places=5)
            for dt in [0.01, 0.005, 0.002]:
                physics_scene.set_dt(dt)
                self.assertAlmostEqual(physics_scene.get_dt(), dt, places=5)

        # exceptions
        self.assertRaises(ValueError, physics_scene.set_dt, -1.0)
        self.assertRaises(ValueError, physics_scene.set_dt, 1.1)

    async def test_gravity_enabled(self) -> None:
        """Test gravity enabled."""
        physics_scene = PhysicsScene("/World/physicsScene")
        # default: True
        self.assertTrue(physics_scene.get_enabled_gravity())
        # toggle
        for enabled in [False, True]:
            physics_scene.set_enabled_gravity(enabled)
            self.assertEqual(physics_scene.get_enabled_gravity(), enabled)

    async def test_max_solver_iterations(self) -> None:
        """Test max solver iterations."""
        physics_scene = PhysicsScene("/World/physicsScene")
        # default: -1 (solver chooses)
        self.assertEqual(physics_scene.get_max_solver_iterations(), -1)
        # set values
        for iterations in [10, 50, 100]:
            physics_scene.set_max_solver_iterations(iterations)
            self.assertEqual(physics_scene.get_max_solver_iterations(), iterations)
        # reset to default
        physics_scene.set_max_solver_iterations(-1)
        self.assertEqual(physics_scene.get_max_solver_iterations(), -1)
        # exceptions
        self.assertRaises(ValueError, physics_scene.set_max_solver_iterations, -2)

    async def test_engine_switch_cleans_up_scene_apis(self) -> None:
        """Test that switching engines removes stale solver APIs and applies the new one."""
        if SimulationManager.get_active_physics_engine() != "newton":
            return

        physics_scene = PhysicsScene("/World/physicsScene")
        self.assertFalse(physics_scene.prim.HasAPI(PhysxSchema.PhysxSceneAPI))

        try:
            SimulationManager.switch_physics_engine("physx")
            self.assertTrue(physics_scene.prim.HasAPI(PhysxSchema.PhysxSceneAPI))
            self.assertFalse(physics_scene.prim.HasAPI("MjcSceneAPI"))

            SimulationManager.switch_physics_engine("newton")
            self.assertFalse(physics_scene.prim.HasAPI(PhysxSchema.PhysxSceneAPI))
        finally:
            if SimulationManager.get_active_physics_engine() != "newton":
                SimulationManager.switch_physics_engine("newton")


class TestPhysxScene(omni.kit.test.AsyncTestCase):
    """Test physx scene."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        # ---------------
        await stage_utils.create_new_stage_async()
        # ---------------

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        # ------------------
        # Do custom tearDown
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------
    async def test_physx_scene_constructor(self) -> None:
        """Test physx scene constructor."""
        # no PhysicsScene in stage (create new)
        self.assertListEqual(PhysicsScene.get_physics_scene_paths(), [])
        path = "/physicsScene_0"
        physx_scene = PhysxScene(path)
        self.assertEqual(physx_scene.path, path)
        self.assertEqual(physx_scene.prim.GetPath().pathString, path)
        self.assertEqual(physx_scene.physics_scene.GetPrim().GetPath().pathString, path)
        # PhysicsScene already in stage (wrap existing)
        path = "/physicsScene_1"
        prim = stage_utils.define_prim(path, type_name="PhysicsScene")
        self.assertListEqual(PhysicsScene.get_physics_scene_paths(), ["/physicsScene_0", path])
        physx_scene = PhysxScene(prim)
        self.assertEqual(physx_scene.path, path)
        self.assertEqual(physx_scene.prim.GetPath().pathString, path)
        self.assertEqual(physx_scene.physics_scene.GetPrim().GetPath().pathString, path)
        # wrap non-PhysicsScene prim
        stage_utils.define_prim("/cube", type_name="Cube")
        with self.assertRaises(ValueError):
            PhysxScene("/cube")

    async def test_get_physics_scene_paths(self) -> None:
        """Test get physics scene paths."""
        stages = [None, stage_utils.get_current_stage()]
        # no PhysicsScene in stage
        for stage in stages:
            paths = PhysicsScene.get_physics_scene_paths(stage)
            self.assertListEqual(paths, [])
        # one PhysicsScene in stage
        stage_utils.define_prim("/World/physicsScene", type_name="PhysicsScene")
        for stage in stages:
            paths = PhysicsScene.get_physics_scene_paths(stage)
            self.assertListEqual(paths, ["/World/physicsScene"])
        # several PhysicsScene(s)
        for i in range(5):
            stage_utils.define_prim(f"/World/physicsScene_{i}", type_name="PhysicsScene")
        for stage in stages:
            paths = PhysicsScene.get_physics_scene_paths(stage)
            self.assertListEqual(paths, ["/World/physicsScene"] + [f"/World/physicsScene_{i}" for i in range(5)])

    async def test_steps_per_second_and_dt(self) -> None:
        """Test steps per second and dt."""
        physx_scene = PhysxScene("/World/physicsScene")
        self.assertEqual(physx_scene.get_steps_per_second(), 60)
        self.assertAlmostEqual(physx_scene.get_dt(), 1.0 / 60.0)
        # steps per second
        for steps_per_second in [120, 30]:
            physx_scene.set_steps_per_second(steps_per_second)
            self.assertEqual(physx_scene.get_steps_per_second(), steps_per_second)
            self.assertAlmostEqual(physx_scene.get_dt(), 1.0 / steps_per_second)
        # dt
        for dt in [0.01, 0.005]:
            physx_scene.set_dt(dt)
            self.assertEqual(physx_scene.get_steps_per_second(), int(1.0 / dt))
            self.assertAlmostEqual(physx_scene.get_dt(), dt)
        # special case: zero
        physx_scene.set_steps_per_second(0)
        self.assertEqual(physx_scene.get_steps_per_second(), 0)
        self.assertEqual(physx_scene.get_dt(), 0.0)
        physx_scene.set_dt(0.0)
        self.assertEqual(physx_scene.get_steps_per_second(), 0)
        self.assertEqual(physx_scene.get_dt(), 0.0)
        # exceptions
        self.assertRaises(ValueError, physx_scene.set_steps_per_second, -1)
        self.assertRaises(ValueError, physx_scene.set_dt, -1.0)
        self.assertRaises(ValueError, physx_scene.set_dt, 1.1)

    async def test_solver_type(self) -> None:
        """Test solver type."""
        physx_scene = PhysxScene("/World/physicsScene")
        self.assertEqual(physx_scene.get_solver_type(), "TGS")
        for solver_type in ["TGS", "PGS"]:
            physx_scene.set_solver_type(solver_type)
            self.assertEqual(physx_scene.get_solver_type(), solver_type)

    async def test_enabled_gpu_dynamics(self) -> None:
        """Test enabled gpu dynamics."""
        physx_scene = PhysxScene("/World/physicsScene")
        self.assertTrue(physx_scene.get_enabled_gpu_dynamics())
        for enabled in [False, True]:
            physx_scene.set_enabled_gpu_dynamics(enabled)
            self.assertEqual(physx_scene.get_enabled_gpu_dynamics(), enabled)

    async def test_enabled_ccd(self) -> None:
        """Test enabled ccd."""
        physx_scene = PhysxScene("/World/physicsScene")
        self.assertFalse(physx_scene.get_enabled_ccd())
        for enabled in [True, False]:
            physx_scene.set_enabled_ccd(enabled)
            self.assertEqual(physx_scene.get_enabled_ccd(), enabled)

    async def test_broadphase_type(self) -> None:
        """Test broadphase type."""
        physx_scene = PhysxScene("/World/physicsScene")
        self.assertEqual(physx_scene.get_broadphase_type(), "GPU")
        for broadphase_type in ["MBP", "GPU", "SAP"]:
            physx_scene.set_broadphase_type(broadphase_type)
            self.assertEqual(physx_scene.get_broadphase_type(), broadphase_type)

    async def test_enabled_stabilization(self) -> None:
        """Test enabled stabilization."""
        physx_scene = PhysxScene("/World/physicsScene")
        self.assertFalse(physx_scene.get_enabled_stabilization())
        for enabled in [True, False]:
            physx_scene.set_enabled_stabilization(enabled)
            self.assertEqual(physx_scene.get_enabled_stabilization(), enabled)

    async def test_gpu_configuration(self) -> None:
        """Test gpu configuration."""
        physx_scene = PhysxScene("/World/physicsScene")
        config = [
            ("gpu_collision_stack_size", 67108864),
            ("gpu_found_lost_aggregate_pairs_capacity", 1024),
            ("gpu_found_lost_pairs_capacity", 262144),
            ("gpu_heap_capacity", 67108864),
            ("gpu_max_deformable_surface_contacts", 1048576),
            ("gpu_max_num_partitions", 8),
            ("gpu_max_particle_contacts", 1048576),
            ("gpu_max_rigid_contact_count", 524288),
            ("gpu_max_rigid_patch_count", 81920),
            ("gpu_max_soft_body_contacts", 1048576),
            ("gpu_temp_buffer_capacity", 16777216),
            ("gpu_total_aggregate_pairs_capacity", 1024),
        ]
        # check for configuration fields
        self.assertListEqual(
            sorted(dataclasses.asdict(PhysxGpuCfg()).keys()),
            sorted([name for name, _ in config]),
        )
        # check for configuration values
        for i, (name, value) in enumerate(config):
            self.assertEqual(
                dataclasses.asdict(physx_scene.get_gpu_configuration())[name],
                value,
                msg=f"Invalid '{name}' default value",
            )
            physx_scene.set_gpu_configuration({name: i})
            self.assertEqual(
                dataclasses.asdict(physx_scene.get_gpu_configuration())[name],
                i,
                msg=f"Invalid '{name}' expected value",
            )


class TestNewtonMjcScene(omni.kit.test.AsyncTestCase):
    """Test newton mjc scene."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    async def test_mjc_scene_constructor(self) -> None:
        """Test mjc scene constructor."""
        path = "/physicsScene"
        mjc_scene = NewtonMjcScene(path)
        self.assertEqual(mjc_scene.path, path)
        self.assertTrue(mjc_scene.prim.HasAPI("NewtonSceneAPI"))
        self.assertTrue(mjc_scene.prim.HasAPI("MjcSceneAPI"))

    async def test_dt(self) -> None:
        """Test dt."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: 0.002
        self.assertAlmostEqual(mjc_scene.get_dt(), 0.002, places=5)
        # set values
        for dt in [0.001, 0.005, 0.01]:
            mjc_scene.set_dt(dt)
            self.assertAlmostEqual(mjc_scene.get_dt(), dt, places=5)
        # exceptions
        self.assertRaises(ValueError, mjc_scene.set_dt, 0.0)
        self.assertRaises(ValueError, mjc_scene.set_dt, -1.0)

    async def test_integrator(self) -> None:
        """Test integrator."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: "euler"
        self.assertEqual(mjc_scene.get_integrator(), "euler")
        # set values
        for integrator in ["rk4", "implicit", "implicitfast", "euler"]:
            mjc_scene.set_integrator(integrator)
            self.assertEqual(mjc_scene.get_integrator(), integrator)

    async def test_solver(self) -> None:
        """Test solver."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: "newton"
        self.assertEqual(mjc_scene.get_solver(), "newton")
        # set values
        for solver in ["pgs", "cg", "newton"]:
            mjc_scene.set_solver(solver)
            self.assertEqual(mjc_scene.get_solver(), solver)

    async def test_iterations(self) -> None:
        """Test iterations."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: 100
        self.assertEqual(mjc_scene.get_iterations(), 100)
        # set values
        for iterations in [50, 200, 500]:
            mjc_scene.set_iterations(iterations)
            self.assertEqual(mjc_scene.get_iterations(), iterations)

    async def test_tolerance(self) -> None:
        """Test tolerance."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: 1e-08
        self.assertAlmostEqual(mjc_scene.get_tolerance(), 1e-08, places=10)
        # set values
        for tolerance in [1e-06, 1e-10, 1e-12]:
            mjc_scene.set_tolerance(tolerance)
            self.assertAlmostEqual(mjc_scene.get_tolerance(), tolerance, places=14)

    async def test_cone(self) -> None:
        """Test cone."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: "pyramidal"
        self.assertEqual(mjc_scene.get_cone(), "pyramidal")
        # set values
        for cone in ["elliptic", "pyramidal"]:
            mjc_scene.set_cone(cone)
            self.assertEqual(mjc_scene.get_cone(), cone)

    async def test_jacobian(self) -> None:
        """Test jacobian."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: "auto"
        self.assertEqual(mjc_scene.get_jacobian(), "auto")
        # set values
        for jacobian in ["dense", "sparse", "auto"]:
            mjc_scene.set_jacobian(jacobian)
            self.assertEqual(mjc_scene.get_jacobian(), jacobian)

    async def test_impratio(self) -> None:
        """Test impratio."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: 1.0
        self.assertAlmostEqual(mjc_scene.get_impratio(), 1.0, places=5)
        # set values
        for impratio in [0.5, 2.0, 10.0]:
            mjc_scene.set_impratio(impratio)
            self.assertAlmostEqual(mjc_scene.get_impratio(), impratio, places=5)

    async def test_wind(self) -> None:
        """Test wind."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: (0, 0, 0)
        wind = mjc_scene.get_wind()
        self.assertAlmostEqual(wind[0], 0.0, places=5)
        self.assertAlmostEqual(wind[1], 0.0, places=5)
        self.assertAlmostEqual(wind[2], 0.0, places=5)
        # set values
        test_winds = [
            Gf.Vec3d(1.0, 0.0, 0.0),
            (0.0, 5.0, 0.0),
            [0.0, 0.0, -2.0],
        ]
        for wind_val in test_winds:
            mjc_scene.set_wind(wind_val)
            wind = mjc_scene.get_wind()
            expected = Gf.Vec3d(*wind_val) if not isinstance(wind_val, Gf.Vec3d) else wind_val
            self.assertAlmostEqual(wind[0], expected[0], places=5)
            self.assertAlmostEqual(wind[1], expected[1], places=5)
            self.assertAlmostEqual(wind[2], expected[2], places=5)

    async def test_density(self) -> None:
        """Test density."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: 0.0
        self.assertAlmostEqual(mjc_scene.get_density(), 0.0, places=5)
        # set values
        for density in [1.225, 1000.0, 0.5]:
            mjc_scene.set_density(density)
            self.assertAlmostEqual(mjc_scene.get_density(), density, places=5)

    async def test_viscosity(self) -> None:
        """Test viscosity."""
        mjc_scene = NewtonMjcScene("/World/physicsScene")
        # default: 0.0
        self.assertAlmostEqual(mjc_scene.get_viscosity(), 0.0, places=5)
        # set values
        for viscosity in [0.001, 1.0, 0.0001]:
            mjc_scene.set_viscosity(viscosity)
            self.assertAlmostEqual(mjc_scene.get_viscosity(), viscosity, places=5)

    async def test_physics_scene_set_dt_play_does_not_hang(self) -> None:
        """Verify a PhysX scene with a custom PhysicsScene dt starts playback without hanging."""
        if SimulationManager.get_active_physics_engine() != "physx":
            self.skipTest("Skipping test for non-physx engine")

        self._timeline = omni.timeline.get_timeline_interface()

        PhysicsScene("/physicsScene").set_dt(1.0 / 60.0)
        await get_app().next_update_async()

        self._timeline.play()

        try:
            await asyncio.wait_for(get_app().next_update_async(), timeout=10.0)
        except asyncio.TimeoutError:
            self.fail("Timed out after playing a stage configured with PhysicsScene.set_dt().")

        self.assertTrue(self._timeline.is_playing())
