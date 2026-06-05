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

"""Tests for the physics raycast sensor example."""

from typing import Any

import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor
from isaacsim.sensors.physics.examples.raycast_sensor import (
    _generate_curtain_rays,
    _generate_rotating_rays,
    _generate_solid_state_rays,
)
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


class TestRaycastSensorExample(omni.kit.test.AsyncTestCase):
    """Verify the physics raycast sensor example creates sensors and produces valid readings with hits."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def _create_scene(self) -> Any:
        """Create a minimal scene with physics, collision geometry, and sensors."""
        stage = omni.usd.get_context().get_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

        ground = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
        ground.GetSizeAttr().Set(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
        ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 0.1))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        wall = UsdGeom.Cube.Define(stage, "/World/Obstacles/Wall")
        wall.GetSizeAttr().Set(1.0)
        wall.AddTranslateOp().Set(Gf.Vec3d(5, 0, 1.5))
        wall.AddScaleOp().Set(Gf.Vec3f(0.2, 8, 3))
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

        box = UsdGeom.Cube.Define(stage, "/World/Obstacles/Box1")
        box.GetSizeAttr().Set(1.0)
        box.AddTranslateOp().Set(Gf.Vec3d(3, -3, 0.5))
        box.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())

        UsdGeom.Xform.Define(stage, "/World/Sensors")

        await omni.kit.app.get_app().next_update_async()
        return stage

    async def test_solid_state_physics_raycast_sensor(self) -> None:
        """Create solid state physics raycast sensor and verify it detects the wall obstacle."""
        stage = await self._create_scene()

        origins, directions, _ = _generate_solid_state_rays()
        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/Solid_State_Physics_Raycast_Sensor",
                min_range=0.4,
                max_range=100.0,
                ray_origins=origins,
                ray_directions=directions,
                output_frame="WORLD",
                translations=[[0.0, 0.0, 1.5]],
            )
        )
        self.assertIsNotNone(sensor, "Failed to create solid state physics raycast sensor")

        prim = stage.GetPrimAtPath("/World/Sensors/Solid_State_Physics_Raycast_Sensor")
        self.assertTrue(prim.IsValid(), "Solid state physics raycast sensor prim not found")

        self._timeline.play()
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        reader = RaycastSensor("/World/Sensors/Solid_State_Physics_Raycast_Sensor")
        reading = reader.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Solid state physics raycast sensor reading not valid")
        self.assertEqual(reading.ray_count, len(origins))

        hit_count = sum(1 for d in reading.depths if d < 100.0)
        self.assertGreater(hit_count, 0, "Solid state physics raycast sensor should detect the wall")

    async def test_rotating_physics_raycast_sensor(self) -> None:
        """Create rotating physics raycast sensor and verify it detects obstacles across a full sweep."""
        stage = await self._create_scene()

        origins, directions, time_offsets = _generate_rotating_rays()
        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/Rotating_Physics_Raycast_Sensor",
                min_range=0.4,
                max_range=100.0,
                ray_origins=origins,
                ray_directions=directions,
                ray_time_offsets=time_offsets,
                output_frame="WORLD",
                translations=[[0.0, 3.0, 1.5]],
            )
        )
        self.assertIsNotNone(sensor, "Failed to create rotating physics raycast sensor")

        prim = stage.GetPrimAtPath("/World/Sensors/Rotating_Physics_Raycast_Sensor")
        self.assertTrue(prim.IsValid(), "Rotating physics raycast sensor prim not found")

        reader = RaycastSensor("/World/Sensors/Rotating_Physics_Raycast_Sensor")

        # Run for a full sweep (~1s at 1Hz) so every azimuth column fires.
        # Accumulate hits across steps since only a subset of rays are active each step.
        self._timeline.play()
        total_hits = 0
        for _ in range(65):
            await omni.kit.app.get_app().next_update_async()
            reading = reader.get_sensor_reading()
            if reading.is_valid and reading.ray_count > 0:
                total_hits += sum(1 for d in reading.depths if d < 100.0)

        self.assertEqual(reading.ray_count, len(origins))
        self.assertGreater(total_hits, 0, "Rotating physics raycast sensor should detect obstacles during a full sweep")

    async def test_beam_curtain_physics_raycast_sensor(self) -> None:
        """Create beam curtain physics raycast sensor and verify it detects the box obstacle."""
        stage = await self._create_scene()

        origins, directions, _ = _generate_curtain_rays()
        sensor = RaycastSensor(
            Raycast.create(
                "/World/Sensors/Beam_Curtain_Physics_Raycast_Sensor",
                min_range=0.2,
                max_range=10.0,
                ray_origins=origins,
                ray_directions=directions,
                output_frame="WORLD",
                translations=[[0.0, -3.0, 1.0]],
            )
        )
        self.assertIsNotNone(sensor, "Failed to create beam curtain physics raycast sensor")

        prim = stage.GetPrimAtPath("/World/Sensors/Beam_Curtain_Physics_Raycast_Sensor")
        self.assertTrue(prim.IsValid(), "Beam curtain physics raycast sensor prim not found")

        self._timeline.play()
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        reader = RaycastSensor("/World/Sensors/Beam_Curtain_Physics_Raycast_Sensor")
        reading = reader.get_sensor_reading()
        self.assertTrue(reading.is_valid, "Beam curtain physics raycast sensor reading not valid")
        self.assertEqual(reading.ray_count, len(origins))

        hit_count = sum(1 for d in reading.depths if d < 10.0)
        self.assertGreater(hit_count, 0, "Beam curtain physics raycast sensor should detect the box")
