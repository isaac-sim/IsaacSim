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

"""Test joint state sensor functionality."""

import asyncio

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics.impl.joint_state_sensor import JointStateSensor, JointStateSensorReading
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdPhysics

from .common import step_simulation


class TestJointStateSensor(omni.kit.test.AsyncTestCase):
    """Test joint state sensor."""

    async def setUp(self):
        """Set up test fixtures."""
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        self._timeline = omni.timeline.get_timeline_interface()
        self.joint_state_sensor = None

    async def create_simple_articulation(self, physics_rate=60):
        """Create simple articulation."""
        await stage_utils.open_stage_async(
            self._assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
        )
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = stage_utils.get_current_stage()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / physics_rate)

        import isaacsim.core.experimental.utils.prim as prim_utils

        prim = prim_utils.get_prim_at_path("/Articulation/Arm/RevoluteJoint")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.RevoluteJoint(prim)
        joint.CreateAxisAttr("Y")

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Tear down test fixtures."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        if self.joint_state_sensor is not None:
            self.joint_state_sensor._stage_open_callback_fn()
            self.joint_state_sensor = None
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def test_reading_returns_valid_after_play(self):
        """Play sim, assert is_valid=True and dof_names is non-empty."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Expected valid reading after play")
        self.assertGreater(len(reading.dof_names), 0, "Expected non-empty dof_names")
        self.assertEqual(len(reading.dof_types), len(reading.dof_names), "dof_types length must match dof_names")
        self.assertGreater(reading.stage_meters_per_unit, 0.0, "Expected positive stage_meters_per_unit when valid")

    async def test_physics_only_step_outputs_joint_state_data(self):
        """JointStateSensor produces data when stepping physics without app/render updates."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertFalse(reading.is_valid, "Reading should be invalid before physics steps")

        try:
            self._timeline.play()
            SimulationManager.initialize_physics()
            SimulationManager.step(steps=3, update_fabric=False)

            reading = self.joint_state_sensor.get_sensor_reading()
            self.assertTrue(reading.is_valid, "Reading should be valid after physics-only steps")
            self.assertGreater(reading.time, 0.0)
            self.assertGreater(len(reading.dof_names), 0)
            self.assertEqual(len(reading.positions), len(reading.dof_names))
        finally:
            if self.joint_state_sensor is not None:
                self.joint_state_sensor.reset()
                self.joint_state_sensor._stage_open_callback_fn()
                self.joint_state_sensor = None
            if self._timeline.is_playing():
                self._timeline.stop()
                await omni.kit.app.get_app().next_update_async()

    async def test_dof_names_populated(self):
        """Verify DOF name strings are non-empty."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid)
        for name in reading.dof_names:
            self.assertIsInstance(name, str)
            self.assertGreater(len(name), 0, f"DOF name should be non-empty, got: {name!r}")

    async def test_reading_updates_over_time(self):
        """Verify sensor reading updates as simulation steps (time advances)."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        await step_simulation(0.1)
        reading1 = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading1.is_valid)

        await step_simulation(1.0)
        reading2 = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading2.is_valid)

        # Sensor is step-driven: simulation time must advance between readings
        self.assertGreater(
            reading2.time,
            reading1.time,
            "Expected sensor reading time to advance as simulation steps",
        )

    async def test_reading_changes_when_load_changes(self):
        """Verify positions, velocities, and efforts all change when we add a cube to the arm (like effort sensor test)."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        reading1 = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading1.is_valid)
        self.assertGreater(len(reading1.efforts), 0, "Expected at least one DOF effort")
        efforts1 = np.asarray(reading1.efforts, dtype=np.float32).copy()

        # Spawn a 1 kg cube 1.5 m from the joint (same as effort sensor test) to change joint load
        Cube("/new_cube", sizes=0.1, positions=[1.5, 0.0, 0.1])
        GeomPrim("/new_cube", apply_collision_apis=True)
        RigidPrim("/new_cube", masses=[1.0])

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        reading2 = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading2.is_valid)
        efforts2 = np.asarray(reading2.efforts, dtype=np.float32)

        # Efforts should change: arm only ~ -2*9.81 Nm, with cube ~ -3.5*9.81 Nm
        self.assertFalse(
            np.allclose(efforts1, efforts2, atol=1e-2),
            "Expected efforts to change after adding cube load (arm + cube)",
        )

    async def test_reading_invalid_before_play(self):
        """Assert is_valid=False before simulation starts."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertFalse(reading.is_valid)
        self.assertEqual(reading.time, 0.0)
        self.assertEqual(len(reading.dof_names), 0)
        self.assertEqual(len(reading.dof_types), 0)
        self.assertEqual(reading.stage_meters_per_unit, 0.0)

    async def test_stop_start(self):
        """Stop/restart timeline, verify readings resume."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        first_reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(first_reading.is_valid)
        self.assertGreater(len(first_reading.dof_names), 0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Release the old sensor and create a fresh one
        self.joint_state_sensor._stage_open_callback_fn()
        self.joint_state_sensor = JointStateSensor("/Articulation")

        self._timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        second_reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(second_reading.is_valid)
        self.assertEqual(len(second_reading.dof_names), len(first_reading.dof_names))

    async def test_disable_enable(self):
        """Set enabled=False/True, verify validity changes accordingly."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid)

        # Disable
        self.joint_state_sensor.enabled = False
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertFalse(reading.is_valid)

        # Re-enable
        self.joint_state_sensor.enabled = True
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid)

    async def test_all_arrays_same_length(self):
        """Assert len(positions) == len(velocities) == len(efforts) == len(dof_names) == len(dof_types)."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid)

        n = len(reading.dof_names)
        self.assertGreater(n, 0)
        self.assertEqual(len(reading.positions), n)
        self.assertEqual(len(reading.velocities), n)
        self.assertEqual(len(reading.efforts), n)
        self.assertEqual(len(reading.dof_types), n)

    async def test_reading_defaults(self):
        """Verify JointStateSensorReading default construction values."""
        reading = JointStateSensorReading()
        self.assertFalse(reading.is_valid)
        self.assertEqual(reading.time, 0.0)
        self.assertEqual(len(reading.dof_names), 0)
        self.assertEqual(len(reading.positions), 0)
        self.assertEqual(len(reading.velocities), 0)
        self.assertEqual(len(reading.efforts), 0)
        self.assertEqual(len(reading.dof_types), 0)
        self.assertEqual(reading.stage_meters_per_unit, 0.0)

    async def test_dof_types_and_stage_units(self):
        """Verify dof_types (0=revolute, 1=prismatic) and stage_meters_per_unit match stage."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid)
        # Stage was set to 1.0 m per unit in create_simple_articulation
        self.assertAlmostEqual(reading.stage_meters_per_unit, 1.0, places=5)
        # simple_articulation has revolute joints only => dof_types should be 0
        for i, dt in enumerate(reading.dof_types):
            self.assertIn(
                int(dt),
                (0, 1),
                f"dof_types[{i}] must be 0 (revolute) or 1 (prismatic), got {dt}",
            )

    async def test_reading_matches_articulation_under_control(self):
        """Apply position control, step sim, verify joint state sensor reports valid state close to commanded targets."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        articulation = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()
        articulation.set_dof_gains(
            np.ones(articulation.num_dofs) * 1e8,
            np.ones(articulation.num_dofs) * 1e8,
        )

        target_positions = np.array([0.5, 0.3], dtype=np.float64)
        articulation.set_dof_position_targets(target_positions)

        await step_simulation(2.0)

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Expected valid reading after control and step")
        self.assertEqual(len(reading.dof_names), 2, "Simple articulation has 2 DOFs")

        sensor_positions = np.asarray(reading.positions, dtype=np.float64)
        art_dof_names = articulation.dof_names

        for i, dof_name in enumerate(reading.dof_names):
            art_idx = art_dof_names.index(dof_name)
            self.assertAlmostEqual(
                sensor_positions[i],
                target_positions[art_idx],
                delta=0.2,
                msg=f"Sensor should be close to target for DOF {dof_name!r}: sensor={sensor_positions[i]}, target={target_positions[art_idx]}",
            )

    async def test_reading_velocities_under_velocity_control(self):
        """Apply velocity control, step sim, verify joint state sensor reports velocities close to commanded."""
        await self.create_simple_articulation()

        self.joint_state_sensor = JointStateSensor("/Articulation")
        self._timeline.play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        articulation = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()

        target_velocities = np.array([0.2, 0.05], dtype=np.float64)
        articulation.set_dof_velocity_targets(target_velocities)

        await step_simulation(0.5)

        reading = self.joint_state_sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Expected valid reading after velocity control and step")
        self.assertEqual(len(reading.dof_names), 2, "Simple articulation has 2 DOFs")

        sensor_velocities = np.asarray(reading.velocities, dtype=np.float64)
        art_dof_names = articulation.dof_names

        for i, dof_name in enumerate(reading.dof_names):
            art_idx = art_dof_names.index(dof_name)
            self.assertAlmostEqual(
                sensor_velocities[i],
                target_velocities[art_idx],
                delta=0.15,
                msg=f"Sensor velocity should be close to target for DOF {dof_name!r}: sensor={sensor_velocities[i]}, target={target_velocities[art_idx]}",
            )
