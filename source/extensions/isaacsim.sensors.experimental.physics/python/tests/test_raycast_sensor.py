# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test physics raycast sensor functionality."""

from __future__ import annotations

import numpy as np
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from .common import step_simulation


class TestRaycastSensor(omni.kit.test.AsyncTestCase):
    """Test physics raycast sensor on dynamic rigid bodies."""

    async def setUp(self):
        """Set up test fixtures."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Tear down test fixtures."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()

    async def test_sensor_follows_falling_rigid_body(self):
        """Verify beam origins track a rigid body falling under gravity."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.GetSizeAttr().Set(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
        ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 1))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        cube_path = "/World/FallingCube"
        start_z = 5.0
        Cube(cube_path, sizes=1.0, positions=[0.0, 0.0, start_z])
        GeomPrim(cube_path, apply_collision_apis=True)
        RigidPrim(cube_path, masses=[1.0])
        await omni.kit.app.get_app().next_update_async()

        sensor_path = "/World/FallingCube/Physics_Raycast_Sensor"
        sensor = RaycastSensor(
            Raycast.create(
                cube_path + "/Physics_Raycast_Sensor",
                min_range=0.1,
                max_range=20.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[0.0, 0.0, -1.0]],
                output_frame="WORLD",
            )
        )
        self.assertIsNotNone(sensor, "Failed to create physics raycast sensor")
        await omni.kit.app.get_app().next_update_async()

        sensor = RaycastSensor(sensor_path)

        self._timeline.play()
        await step_simulation(0.1)

        origin_z_samples = []
        for _ in range(30):
            await omni.kit.app.get_app().next_update_async()
            reading = sensor.get_sensor_reading()
            if reading.is_valid and reading.ray_count > 0:
                oz = reading.ray_origins_world[0][2]
                origin_z_samples.append(oz)

        self.assertGreaterEqual(len(origin_z_samples), 2, "Not enough valid readings collected")

        self.assertGreater(
            origin_z_samples[0],
            origin_z_samples[-1],
            "Beam origin Z should decrease as the rigid body falls",
        )

        self.assertLess(
            origin_z_samples[-1],
            start_z,
            "Beam origin should have moved below the starting height",
        )

    async def test_beam_endpoints_hit_ground(self):
        """Verify beam end points reach the ground when a sensor falls toward it."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.GetSizeAttr().Set(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
        ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 1))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        cube_path = "/World/FallingCube"
        Cube(cube_path, sizes=1.0, positions=[0.0, 0.0, 3.0])
        GeomPrim(cube_path, apply_collision_apis=True)
        RigidPrim(cube_path, masses=[1.0])
        await omni.kit.app.get_app().next_update_async()

        sensor_path = cube_path + "/Physics_Raycast_Sensor"
        sensor = RaycastSensor(
            Raycast.create(
                cube_path + "/Physics_Raycast_Sensor",
                min_range=0.6,
                max_range=20.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[0.0, 0.0, -1.0]],
                output_frame="WORLD",
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        sensor = RaycastSensor(sensor_path)

        self._timeline.play()
        # Cube rests on ground at z=0.5 (half-size). Sensor at cube center → z=0.5.
        # Let the cube fall and settle on the ground.
        await step_simulation(2.0)

        reading = sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Sensor reading should be valid")
        self.assertEqual(reading.ray_count, 1)

        depth = reading.depths[0]
        self.assertLess(depth, 20.0, "Ray should hit the ground (depth < max_range)")

        end_z = reading.ray_end_points_world[0][2]
        self.assertAlmostEqual(end_z, 0.0, delta=0.15, msg="Beam end point Z should be near ground level")

    async def test_sensor_direction_follows_rotated_body(self):
        """Verify ray directions rotate with the parent rigid body."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

        wall = UsdGeom.Cube.Define(stage, "/World/Wall")
        wall.GetSizeAttr().Set(1.0)
        wall.AddTranslateOp().Set(Gf.Vec3d(5, 0, 2))
        wall.AddScaleOp().Set(Gf.Vec3f(0.2, 10, 10))
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

        import math

        xform = UsdGeom.Xform.Define(stage, "/World/SensorMount")
        xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2))
        orient_op = xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat)
        orient_op.Set(Gf.Quatf(1, 0, 0, 0))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        xform.GetPrim().CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)

        sensor_path = "/World/SensorMount/Physics_Raycast_Sensor"
        sensor = RaycastSensor(
            Raycast.create(
                "/World/SensorMount/Physics_Raycast_Sensor",
                min_range=0.1,
                max_range=20.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                output_frame="WORLD",
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        sensor = RaycastSensor(sensor_path)

        self._timeline.play()
        await step_simulation(0.3)

        reading_fwd = sensor.get_sensor_reading()
        self.assertTrue(reading_fwd.is_valid)
        depth_fwd = reading_fwd.depths[0]
        self.assertLess(depth_fwd, 20.0, "Forward ray should hit the wall")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        yaw_90 = Gf.Quatf(math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4))
        orient_op.Set(yaw_90)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.3)

        reading_rot = sensor.get_sensor_reading()
        self.assertTrue(reading_rot.is_valid)
        depth_rot = reading_rot.depths[0]
        self.assertAlmostEqual(
            depth_rot, 20.0, delta=0.5, msg="After 90-degree yaw, ray should miss the wall (pointing along Y)"
        )


async def _create_basic_scene():
    """Create a minimal scene with physics and collision geometry."""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.GetSizeAttr().Set(1.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
    ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 1))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    wall = UsdGeom.Cube.Define(stage, "/World/Wall")
    wall.GetSizeAttr().Set(1.0)
    wall.AddTranslateOp().Set(Gf.Vec3d(5, 0, 1.5))
    wall.AddScaleOp().Set(Gf.Vec3f(0.2, 8, 3))
    UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

    UsdGeom.Xform.Define(stage, "/World/Sensors")
    await omni.kit.app.get_app().next_update_async()
    return stage


class TestRaycastSensorRuntimeData(omni.kit.test.AsyncTestCase):
    """Test RaycastSensor runtime data helpers."""

    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()

    async def test_create_authoring_sensor(self):
        """Raycast.create() + RaycastSensor() creates a new prim when the path does not exist."""
        await _create_basic_scene()

        RaycastSensor(
            Raycast.create(
                "/World/Sensors/TestSensor",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                min_range=0.5,
                max_range=50.0,
                output_frame="WORLD",
            )
        )

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/Sensors/TestSensor")
        self.assertTrue(prim.IsValid(), "RaycastSensor should create a prim")
        self.assertEqual(prim.GetTypeName(), "IsaacRaycastSensor")
        self.assertEqual(prim.GetAttribute("numRays").Get(), 1)
        self.assertAlmostEqual(prim.GetAttribute("minRange").Get(), 0.5, places=5)
        self.assertAlmostEqual(prim.GetAttribute("maxRange").Get(), 50.0, places=5)

    async def test_wrap_existing_sensor(self):
        """RaycastSensor wraps an existing prim and applies overrides."""
        await _create_basic_scene()

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/ExistingSensor",
                min_range=0.4,
                max_range=100.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        RaycastSensor(
            Raycast(
                "/World/Sensors/ExistingSensor",
                min_range=1.0,
                max_range=25.0,
                output_frame="WORLD",
            )
        )

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/Sensors/ExistingSensor")
        self.assertAlmostEqual(prim.GetAttribute("minRange").Get(), 1.0, places=5)
        self.assertAlmostEqual(prim.GetAttribute("maxRange").Get(), 25.0, places=5)
        self.assertEqual(prim.GetAttribute("outputFrameOfReference").Get(), "WORLD")

    async def test_get_data(self):
        """get_data returns dict with expected keys and valid data after simulation."""
        await _create_basic_scene()

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/FrameSensor",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                min_range=0.1,
                max_range=100.0,
                output_frame="WORLD",
                translations=np.array([[0.0, 0.0, 1.5]]),
            )
        )

        self._timeline.play()
        await step_simulation(0.3)

        frame = sensor.get_data()
        for key in ["depths", "hit_positions", "hit_normals", "hit_prim_paths", "time", "physics_step"]:
            self.assertIn(key, frame, f"Frame missing key '{key}'")

        self.assertIsInstance(frame["physics_step"], int)
        self.assertGreater(len(frame["depths"]), 0, "Should have depth data")

    async def test_get_sensor_reading(self):
        """get_sensor_reading returns a C++ reading struct."""
        await _create_basic_scene()

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/ReadingSensor",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                min_range=0.1,
                max_range=100.0,
                output_frame="WORLD",
                translations=np.array([[0.0, 0.0, 1.5]]),
            )
        )

        self._timeline.play()
        await step_simulation(0.3)

        reading = sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Reading should be valid after simulation")
        self.assertEqual(reading.ray_count, 1)

    async def test_physics_only_step_outputs_raycast_data(self):
        """RaycastSensor produces data when stepping physics without app/render updates."""
        await _create_basic_scene()

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/PhysicsOnlySensor",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                min_range=0.1,
                max_range=100.0,
                output_frame="WORLD",
                translations=np.array([[0.0, 0.0, 1.5]]),
            )
        )

        self.assertFalse(sensor.get_sensor_reading().is_valid, "Reading should be invalid before physics steps")

        try:
            self._timeline.play()
            SimulationManager.initialize_physics()
            SimulationManager.step(steps=3, update_fabric=False)

            reading = sensor.get_sensor_reading()
            self.assertTrue(reading.is_valid, "Reading should be valid after physics-only steps")
            self.assertEqual(reading.ray_count, 1)
            self.assertEqual(len(reading.depths), 1)
            self.assertLess(reading.depths[0], 100.0, "Ray should hit the wall")
        finally:
            sensor.reset()
            if self._timeline.is_playing():
                self._timeline.stop()
                await omni.kit.app.get_app().next_update_async()

    async def test_invalid_frame_before_play(self):
        """get_data returns empty arrays before simulation starts."""
        await _create_basic_scene()

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/NoPlaySensor",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
            )
        )

        frame = sensor.get_data()
        self.assertEqual(len(frame["depths"]), 0)
        self.assertEqual(frame["hit_positions"].shape, (0, 3))

    async def test_position_and_translation_conflict(self):
        """Specifying both positions and translations raises ValueError."""
        await _create_basic_scene()

        with self.assertRaises(ValueError):
            Raycast.create(
                "/World/Sensors/ConflictSensor",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                positions=np.array([[1.0, 0.0, 0.0]]),
                translations=np.array([[1.0, 0.0, 0.0]]),
            )

    async def test_mismatched_origins_directions_raises(self):
        """Mismatched ray_origins and ray_directions lengths raises ValueError."""
        await _create_basic_scene()

        with self.assertRaises(ValueError):
            Raycast.create(
                "/World/Sensors/MismatchSensor",
                ray_origins=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
            )


class TestRaycastSensorLifecycle(omni.kit.test.AsyncTestCase):
    """Test RaycastSensor lifecycle: remove, reset, timeline stop."""

    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()

    async def test_remove_sensor(self):
        """removeSensor stops future readings from being valid."""
        await _create_basic_scene()

        sensor_path = "/World/Sensors/RemoveSensor"
        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/RemoveSensor",
                min_range=0.1,
                max_range=100.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                output_frame="WORLD",
                translations=[[0.0, 0.0, 1.5]],
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        backend = RaycastSensor(sensor_path)

        self._timeline.play()
        await step_simulation(0.3)

        reading = backend.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Should have valid reading before remove")

        from isaacsim.sensors.experimental.physics.impl.extension import get_raycast_sensor_interface

        iface = get_raycast_sensor_interface()
        self.assertIsNotNone(iface)
        iface.remove_sensor(sensor_path)

        reading_after = iface.get_sensor_reading(sensor_path)
        self.assertFalse(reading_after.is_valid, "Reading should be invalid after remove")

    async def test_sensor_reset(self):
        """sensor reset() removes the C++ sensor and clears state."""
        await _create_basic_scene()

        sensor_path = "/World/Sensors/ResetSensor"
        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/ResetSensor",
                min_range=0.1,
                max_range=100.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                output_frame="WORLD",
                translations=[[0.0, 0.0, 1.5]],
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        backend = RaycastSensor(sensor_path)

        self._timeline.play()
        await step_simulation(0.3)

        reading = backend.get_sensor_reading()
        self.assertTrue(reading.is_valid)

        backend.reset()
        self.assertFalse(backend._sensor_created, "reset() should clear _sensor_created")

    async def test_sensor_on_timeline_stop(self):
        """on_timeline_stop clears interface and sensor state."""
        await _create_basic_scene()

        sensor_path = "/World/Sensors/TimelineStopSensor"
        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/TimelineStopSensor",
                min_range=0.1,
                max_range=100.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                output_frame="WORLD",
                translations=[[0.0, 0.0, 1.5]],
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        backend = RaycastSensor(sensor_path)

        self._timeline.play()
        await step_simulation(0.3)

        reading = backend.get_sensor_reading()
        self.assertTrue(reading.is_valid)

        backend.on_timeline_stop()
        self.assertFalse(backend._sensor_created)
        self.assertIsNone(backend._iface)

    async def test_create_sensor_invalid_prim(self):
        """createSensor with a non-existent prim returns false."""
        await _create_basic_scene()

        self._timeline.play()
        await step_simulation(0.1)

        from isaacsim.sensors.experimental.physics.impl.extension import get_raycast_sensor_interface

        iface = get_raycast_sensor_interface()
        if iface is None:
            self.skipTest("IRaycastSensor interface not available")

        result = iface.create_sensor("/World/NonExistent/FakeSensor")
        self.assertFalse(result, "createSensor should return false for non-existent prim")

    async def test_get_reading_nonexistent_sensor(self):
        """getSensorReading for a non-existent sensor returns invalid reading."""
        await _create_basic_scene()

        self._timeline.play()
        await step_simulation(0.1)

        from isaacsim.sensors.experimental.physics.impl.extension import get_raycast_sensor_interface

        iface = get_raycast_sensor_interface()
        if iface is None:
            self.skipTest("IRaycastSensor interface not available")

        reading = iface.get_sensor_reading("/World/NonExistent/FakeSensor")
        self.assertFalse(reading.is_valid, "Reading should be invalid for non-existent sensor")

    async def test_mismatched_origins_vs_numrays_disables_sensor(self):
        """Sensor with rayOrigins length != numRays produces invalid readings."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
        UsdGeom.Xform.Define(stage, "/World/Sensors")

        from pxr import Vt

        prim = stage.DefinePrim("/World/Sensors/BadSensor", "IsaacRaycastSensor")
        prim.CreateAttribute("numRays", Sdf.ValueTypeNames.UInt).Set(2)
        prim.CreateAttribute("rayOrigins", Sdf.ValueTypeNames.Float3Array).Set(Vt.Vec3fArray([(0, 0, 0)]))
        prim.CreateAttribute("rayDirections", Sdf.ValueTypeNames.Float3Array).Set(Vt.Vec3fArray([(1, 0, 0)]))
        await omni.kit.app.get_app().next_update_async()

        backend = RaycastSensor("/World/Sensors/BadSensor")

        self._timeline.play()
        await step_simulation(0.3)

        reading = backend.get_sensor_reading()
        self.assertFalse(reading.is_valid, "rayOrigins length != numRays should produce invalid reading")

    async def test_numrays_zero_disables_sensor(self):
        """Sensor with numRays=0 produces invalid readings."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
        UsdGeom.Xform.Define(stage, "/World/Sensors")

        prim = stage.DefinePrim("/World/Sensors/ZeroRaysSensor", "IsaacRaycastSensor")
        prim.CreateAttribute("numRays", Sdf.ValueTypeNames.UInt).Set(0)
        await omni.kit.app.get_app().next_update_async()

        backend = RaycastSensor("/World/Sensors/ZeroRaysSensor")

        self._timeline.play()
        await step_simulation(0.3)

        reading = backend.get_sensor_reading()
        self.assertFalse(reading.is_valid, "numRays=0 should produce invalid reading")

    async def test_mismatched_time_offsets_disables_sensor(self):
        """Sensor with rayTimeOffsets length != numRays produces invalid readings."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
        UsdGeom.Xform.Define(stage, "/World/Sensors")

        from pxr import Vt

        prim = stage.DefinePrim("/World/Sensors/BadOffsetsSensor", "IsaacRaycastSensor")
        prim.CreateAttribute("numRays", Sdf.ValueTypeNames.UInt).Set(2)
        prim.CreateAttribute("rayOrigins", Sdf.ValueTypeNames.Float3Array).Set(Vt.Vec3fArray([(0, 0, 0), (1, 0, 0)]))
        prim.CreateAttribute("rayDirections", Sdf.ValueTypeNames.Float3Array).Set(Vt.Vec3fArray([(1, 0, 0), (0, 1, 0)]))
        prim.CreateAttribute("rayTimeOffsets", Sdf.ValueTypeNames.FloatArray).Set(Vt.FloatArray([0.0]))
        await omni.kit.app.get_app().next_update_async()

        backend = RaycastSensor("/World/Sensors/BadOffsetsSensor")

        self._timeline.play()
        await step_simulation(0.3)

        reading = backend.get_sensor_reading()
        self.assertFalse(reading.is_valid, "rayTimeOffsets length != numRays should produce invalid reading")
