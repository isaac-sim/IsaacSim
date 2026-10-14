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

"""Verifies raycast sensor attachment, beam endpoints, body-relative directions, authoring and runtime data, lifecycle resets, invalid frames, configuration conflicts, and disabled invalid ray layouts."""

from __future__ import annotations

from typing import Any

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import PhysicsScene, SimulationManager
from isaacsim.sensors.experimental.physics import IMU, IMUSensor, Raycast, RaycastSensor
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from .common import step_simulation


class TestRaycastSensor(omni.kit.test.AsyncTestCase):
    """Test physics raycast sensor on dynamic rigid bodies."""

    async def test_default_reading_has_consistent_empty_array_shapes(self) -> None:
        """An invalid owned snapshot initializes every numeric array."""
        from isaacsim.sensors.experimental.physics import _physics_sensors

        reading = _physics_sensors.RaycastSensorReading()
        self.assertEqual(reading.depths.shape, (0,))
        self.assertEqual(reading.hit_positions.shape, (0, 3))
        self.assertEqual(reading.hit_normals.shape, (0, 3))
        self.assertEqual(reading.ray_origins_world.shape, (0, 3))
        self.assertEqual(reading.ray_end_points_world.shape, (0, 3))

    async def setUp(self) -> None:
        """Create a fresh stage and timeline for raycast-on-rigid-body tests."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop playback and invalidate physics after raycast-on-rigid-body tests."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()

    async def test_sensor_follows_falling_rigid_body(self) -> None:
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

    async def test_beam_endpoints_hit_ground(self) -> None:
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

    async def test_sensor_direction_follows_rotated_body(self) -> None:
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


async def _create_basic_scene() -> Any:
    """Create a minimal scene with physics and collision geometry.

    Returns:
        The current USD stage containing the authored test scene.
    """
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

    async def setUp(self) -> None:
        """Create a fresh stage and timeline for raycast runtime data tests."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop playback and invalidate physics after raycast runtime data tests."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()

    async def test_create_authoring_sensor(self) -> None:
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

    async def test_wrap_existing_sensor(self) -> None:
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

    async def test_get_data(self) -> None:
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

    async def test_get_sensor_reading(self) -> None:
        """get_sensor_reading returns a mutable owned snapshot without property copies."""
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
        self.assertIs(reading.depths, reading.depths)

        original_depth = float(reading.depths[0])
        reading.depths[0] = original_depth + 1.0
        self.assertEqual(reading.depths[0], original_depth + 1.0)
        reading.depths[0] = original_depth

        # Captured scalars are read-only; only the arrays a snapshot owns are mutable.
        for field in ("time", "is_valid", "ray_count"):
            with self.subTest(field=field):
                with self.assertRaises(AttributeError):
                    setattr(reading, field, 0)

        snapshot_time = reading.time
        snapshot_depths = reading.depths.copy()
        await step_simulation(0.1)
        self.assertEqual(reading.time, snapshot_time)
        np.testing.assert_array_equal(reading.depths, snapshot_depths)

    async def test_hit_prim_path_longer_than_reserved_buffer(self) -> None:
        """A hit path that outgrows the buffer reserved per ray is reported without truncation."""
        stage = await _create_basic_scene()

        # Nest the target deeply so its path outgrows the buffer each ray reserves up front,
        # taking the grow path that keeps the outgrown buffer alive for readers.
        target_path = "/World/" + "/".join(f"deeply_nested_group_{i:02d}" for i in range(12)) + "/Target"
        self.assertGreater(len(target_path), 256)

        target = UsdGeom.Cube.Define(stage, target_path)
        target.GetSizeAttr().Set(1.0)
        target.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 1.5))
        UsdPhysics.RigidBodyAPI.Apply(target.GetPrim()).CreateKinematicEnabledAttr(True)
        UsdPhysics.CollisionAPI.Apply(target.GetPrim())
        await omni.kit.app.get_app().next_update_async()

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/LongPathSensor",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                min_range=0.1,
                max_range=100.0,
                output_frame="WORLD",
                report_hit_prim_paths=True,
                translations=np.array([[0.0, 0.0, 1.5]]),
            )
        )

        self._timeline.play()
        await step_simulation(0.3)

        reading = sensor.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Reading should be valid after simulation")
        self.assertEqual(list(reading.hit_prim_paths), [target_path])

    async def test_physics_only_step_outputs_raycast_data(self) -> None:
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

    async def test_invalid_frame_before_play(self) -> None:
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

    async def test_position_and_translation_conflict(self) -> None:
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

    async def test_mismatched_origins_directions_raises(self) -> None:
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

    async def setUp(self) -> None:
        """Create a fresh stage and timeline for raycast lifecycle tests."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop playback and invalidate physics after raycast lifecycle tests."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()

    async def test_remove_sensor(self) -> None:
        """RemoveSensor stops future readings from being valid."""
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
        depths = reading.depths.copy()
        hit_positions = reading.hit_positions.copy()
        hit_normals = reading.hit_normals.copy()
        ray_origins_world = reading.ray_origins_world.copy()
        ray_end_points_world = reading.ray_end_points_world.copy()

        from isaacsim.sensors.experimental.physics.impl.extension import get_raycast_sensor_interface

        iface = get_raycast_sensor_interface()
        self.assertIsNotNone(iface)
        iface.remove_sensor(sensor_path)

        reading_after = iface.get_sensor_reading(sensor_path)
        self.assertFalse(reading_after.is_valid, "Reading should be invalid after remove")
        np.testing.assert_array_equal(reading.depths, depths)
        np.testing.assert_array_equal(reading.hit_positions, hit_positions)
        np.testing.assert_array_equal(reading.hit_normals, hit_normals)
        np.testing.assert_array_equal(reading.ray_origins_world, ray_origins_world)
        np.testing.assert_array_equal(reading.ray_end_points_world, ray_end_points_world)

    async def test_reading_view_is_live_read_only_and_invalidates_on_remove(self) -> None:
        """Owner-backed views update in place and retain last data after dynamic invalidation."""
        await _create_basic_scene()

        sensor_path = "/World/Sensors/ViewSensor"
        backend = RaycastSensor(
            Raycast.create(
                sensor_path,
                min_range=0.1,
                max_range=100.0,
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
                output_frame="WORLD",
                translations=[[0.0, 0.0, 1.5]],
            )
        )

        self._timeline.play()
        await step_simulation(0.3)

        view = backend.get_sensor_reading_view()
        self.assertTrue(view.is_valid)
        depths = view.depths
        self.assertFalse(depths.flags.writeable)
        self.assertIs(depths.base, view)
        self.assertEqual(view.hit_prim_paths, ())
        data_address = depths.__array_interface__["data"][0]
        first_time = view.time

        with self.assertRaises(ValueError):
            depths[0] = 0.0
        with self.assertRaises(ValueError):
            depths.setflags(write=True)

        await step_simulation(0.1)
        self.assertGreater(view.time, first_time)
        self.assertEqual(view.depths.__array_interface__["data"][0], data_address)

        retained_values = depths.copy()
        from isaacsim.sensors.experimental.physics.impl.extension import get_raycast_sensor_interface

        iface = get_raycast_sensor_interface()
        self.assertIsNotNone(iface)
        iface.remove_sensor(sensor_path)

        self.assertFalse(view.is_valid)
        np.testing.assert_array_equal(depths, retained_values)

    async def test_sensor_reset(self) -> None:
        """Sensor reset() removes the C++ sensor and clears state."""
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

    async def test_sensor_on_timeline_stop(self) -> None:
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

        view = backend.get_sensor_reading_view()
        self.assertTrue(view.is_valid)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(view.is_valid)

        backend.on_timeline_stop()
        self.assertFalse(backend._sensor_created)
        self.assertIsNone(backend._iface)

    async def test_create_sensor_invalid_prim(self) -> None:
        """CreateSensor with a non-existent prim returns false."""
        await _create_basic_scene()

        self._timeline.play()
        await step_simulation(0.1)

        from isaacsim.sensors.experimental.physics.impl.extension import get_raycast_sensor_interface

        iface = get_raycast_sensor_interface()
        if iface is None:
            self.skipTest("IRaycastSensor interface not available")

        result = iface.create_sensor("/World/NonExistent/FakeSensor")
        self.assertFalse(result, "createSensor should return false for non-existent prim")

    async def test_get_reading_nonexistent_sensor(self) -> None:
        """GetSensorReading for a non-existent sensor returns invalid reading."""
        await _create_basic_scene()

        self._timeline.play()
        await step_simulation(0.1)

        from isaacsim.sensors.experimental.physics.impl.extension import get_raycast_sensor_interface

        iface = get_raycast_sensor_interface()
        if iface is None:
            self.skipTest("IRaycastSensor interface not available")

        reading = iface.get_sensor_reading("/World/NonExistent/FakeSensor")
        self.assertFalse(reading.is_valid, "Reading should be invalid for non-existent sensor")

    async def test_pause_resume_with_imu_on_same_articulation(self) -> None:
        """Raycast and IMU sharing an articulation survive pause/resume."""
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        PhysicsScene("/World/PhysicsScene")

        Cube("/World/Ground", sizes=1.0, translations=[0.0, 0.0, -0.5], scales=[50.0, 50.0, 1.0])
        GeomPrim("/World/Ground", apply_collision_apis=True)

        art_path = "/Dummy"
        stage_utils.define_prim(art_path, "Xform")
        prim_utils.ensure_api(art_path, UsdPhysics.ArticulationRootAPI)

        base_path = f"{art_path}/base_link"
        Cube(base_path, sizes=0.2, positions=[0.0, 0.0, 1.0])
        GeomPrim(base_path, apply_collision_apis=True)
        RigidPrim(base_path, masses=[1.0])

        imu_link_path = f"{base_path}/base_imu_link"
        stage_utils.define_prim(imu_link_path, "Xform")

        child_path = f"{art_path}/child_link"
        Cube(child_path, sizes=0.1, positions=[0.3, 0.0, 1.0])
        GeomPrim(child_path, apply_collision_apis=True)
        RigidPrim(child_path, masses=[0.1])

        joint = UsdPhysics.RevoluteJoint(stage_utils.define_prim(f"{art_path}/joint", "PhysicsRevoluteJoint"))
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(base_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.15, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.15, 0.0, 0.0))

        raycast = RaycastSensor(
            Raycast.create(
                f"{base_path}/repro_raycast",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[0.0, 0.0, -1.0]],
                min_range=0.05,
                max_range=10.0,
                output_frame="WORLD",
                translations=[[0.2, 0.0, 0.0]],
            )
        )
        imu = IMUSensor(IMU.create(f"{imu_link_path}/repro_imu", translations=[[0.0, 0.0, 0.0]]))
        self.assertIsNotNone(raycast)
        self.assertIsNotNone(imu)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.1)

        self._timeline.pause()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.2)

        reading = raycast.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Raycast reading should recover after pause/resume")
        self.assertEqual(reading.ray_count, 1)

    async def test_mismatched_origins_vs_numrays_disables_sensor(self) -> None:
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

    async def test_numrays_zero_disables_sensor(self) -> None:
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

    async def test_mismatched_time_offsets_disables_sensor(self) -> None:
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
