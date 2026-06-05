# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test lidar functionality."""

import carb
import numpy as np
import omni.isaac.RangeSensorSchema as RangeSensorSchema
import omni.kit.commands
import omni.kit.test
from isaacsim.core.api import SimulationContext, World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.sensors.physx import RotatingLidarPhysX, _range_sensor
from isaacsim.storage.native import get_assets_root_path_async
from omni.syntheticdata.tests.utils import add_semantics
from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdLux, UsdPhysics


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestLidar(omni.kit.test.AsyncTestCase):
    """Test lidar."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()

        # light
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # set up axis to z
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        # Physics scene
        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.sensors.physx")
        self._extension_path = ext_manager.get_extension_path(ext_id)

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        self._timeline.stop()

    async def sweep_parameter(self, parameter: object, min_v: float, max_v: float, step: float) -> None:
        """Perform sweep parameter operation.

        Args:
            parameter: The USD attribute to sweep.
            min_v: Minimum value for sweep.
            max_v: Maximum value for sweep.
            step: Step size for sweep.
        """
        print(parameter.GetName())
        for value in np.arange(min_v, max_v, step):
            # print(value)
            parameter.Set(float(value))
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

    async def add_cube(self, path: str, size: float, offset: object) -> UsdGeom.Cube:
        """Perform add cube operation.

        Args:
            path: USD path for the cube prim.
            size: Size of the cube.
            offset: Translation offset for the cube.

        Returns:
            The created cube geometry object.
        """
        cubeGeom = UsdGeom.Cube.Define(self._stage, path)
        cubePrim = self._stage.GetPrimAtPath(path)

        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigid_api.CreateRigidBodyEnabledAttr(True)
        UsdPhysics.CollisionAPI.Apply(cubePrim)

        return cubeGeom

    # Tests a static lidar with a cube in front of it
    async def test_static_lidar(self) -> None:
        """Test static lidar."""
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )

        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.250))

        # Run for a second
        self._timeline.play()
        await simulate_async(1.0)
        self._timeline.pause()

        # Get depth, and check that we hit the cube in front, and hit nothing in back
        depth = self._lidar.get_depth_data(lidarPath)

        self.assertLess(depth[0, 0], 2000)
        self.assertEqual(depth[450, 0], 65535)
        self._timeline.play()

    async def test_rotating_lidar_physx_physics_only_step_outputs_scan_data(self) -> None:
        """Test RotatingLidarPhysX produces frame data when stepping physics without rendering."""
        World.clear_instance()
        SimulationContext.clear_instance()
        world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
        await world.initialize_simulation_context_async()
        world.scene.add_default_ground_plane()
        lidar = world.scene.add(
            RotatingLidarPhysX(
                prim_path="/World/Lidar",
                name="physics_only_lidar",
                rotation_frequency=0.0,
                translation=np.array([0.0, 0.0, 1.0]),
            )
        )
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="cube",
                position=np.array([2.0, 0.0, 1.0]),
                scale=np.array([1.0, 1.0, 1.0]),
            )
        )
        await omni.kit.app.get_app().next_update_async()

        try:
            world.reset()
            lidar.add_depth_data_to_frame()
            lidar.add_linear_depth_data_to_frame()
            lidar.add_point_cloud_data_to_frame()

            for _ in range(3):
                world.step(render=False)

            frame = lidar.get_current_frame()
            depth = np.asarray(frame["depth"])
            linear_depth = np.asarray(frame["linear_depth"])
            point_cloud = np.asarray(frame["point_cloud"])

            self.assertGreater(depth.size, 0)
            self.assertGreater(linear_depth.size, 0)
            self.assertGreater(point_cloud.shape[0], 0)
        finally:
            world.stop()
            World.clear_instance()
            SimulationContext.clear_instance()

    # Tests a lidar on a falling cube, with a cube in front of it after it lands
    async def test_dynamic_lidar(self) -> None:
        """Test dynamic lidar."""
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )
        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add falling cube
        cubePath2 = "/World/Cube2"
        await self.add_cube(cubePath2, 0.500, Gf.Vec3f(0.0, 0.0, 2.500))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent="/World/Cube2",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.500))
        self._timeline.play()
        # get data before it falls and make sure that lidar is parented properly and does not have block infront of it
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        depth = self._lidar.get_depth_data(lidarPath)
        self.assertEqual(depth[0, 0], 65535)

        # wait for it to drop
        await simulate_async(2.0)
        self._timeline.pause()
        # Get depth, and check that we hit the cube in front, and hit nothing in back
        depth = self._lidar.get_depth_data(lidarPath)
        self.assertLess(depth[0, 0], 2000)
        self.assertEqual(depth[450, 0], 65535)

    async def test_parameter_ranges(self) -> None:
        """Test parameter ranges."""
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )
        cubePath2 = "/World/Cube2"
        await self.add_cube(cubePath2, 50.0, Gf.Vec3f(0.0, 0.0, 250.0))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Cube2/Lidar",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=True,
            yaw_offset=0.0,
        )

        self._timeline.play()
        lidar.GetHighLodAttr().Set(True)
        RangeSensorSchema.RangeSensor(lidar).GetDrawPointsAttr().Set(False)
        await self.sweep_parameter(lidar.GetRotationRateAttr(), -1024, 1024, 256)
        lidar.GetRotationRateAttr().Set(0)
        await self.sweep_parameter(lidar.GetHorizontalFovAttr(), -1024, 1024, 256)
        lidar.GetHorizontalFovAttr().Set(360)
        await self.sweep_parameter(lidar.GetVerticalFovAttr(), -1024, 1024, 256)
        lidar.GetHorizontalFovAttr().Set(120)
        lidar.GetVerticalFovAttr().Set(30)
        await self.sweep_parameter(lidar.GetHorizontalResolutionAttr(), -0.1, 1.0, 0.1)
        await self.sweep_parameter(lidar.GetVerticalResolutionAttr(), -0.1, 1.0, 0.1)
        await self.sweep_parameter(RangeSensorSchema.RangeSensor(lidar).GetMinRangeAttr(), -1024, 1024, 256)
        await self.sweep_parameter(RangeSensorSchema.RangeSensor(lidar).GetMaxRangeAttr(), -1024, 1024, 256)
        lidar.GetHighLodAttr().Set(False)

    async def test_carter_lidar(self) -> None:
        """Test carter lidar.

        Returns:
            None
        """
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        result, error = await open_stage_async(
            self._assets_root_path + "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd"
        )
        self._stage = omni.usd.get_context().get_stage()

        # Add a cube
        cubePath = "/Cube"
        await self.add_cube(cubePath, 0.750, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/NewLidar",
            parent="/carter/chassis_link",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(-0.060, 0.0, 0.370))

        # Run for two seconds

        self._timeline.play()
        await simulate_async(2.0)
        self._timeline.pause()
        # Get depth, and check that we hit the cube in front, and hit nothing in back
        depth = self._lidar.get_depth_data(lidarPath)
        self.assertLess(depth[0, 0], 2000)
        self.assertEqual(depth[450, 0], 65535)
        self._timeline.play()

    # Tests a static lidar with a cube in front of it and get semantic id for hit points
    async def test_static_lidar_semantic(self) -> None:
        """Test static lidar semantic."""
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )

        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, -1.000, 0.500))
        cubePrim = self._stage.GetPrimAtPath(cubePath)
        add_semantics(cubePrim, "cube")

        # Add another cube
        cubePath = "/World/Cube1"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 1.000, 0.500))
        cubePrim = self._stage.GetPrimAtPath(cubePath)
        add_semantics(cubePrim, "cube1")

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=True,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.250))

        # Run for a second
        self._timeline.play()
        await simulate_async(1.0)
        self._timeline.pause()

        # Get semantic data of hit points, and check that we get two non-zero semantic IDs
        semantic = self._lidar.get_semantic_data(lidarPath)
        prim_data = self._lidar.get_prim_data(lidarPath)

        # semantic IDs are deprecated, use prim paths to access semantics instead
        self.assertEqual(len(np.unique(semantic)), 0)
        self.assertEqual(len(np.unique(prim_data)), 3)
        self._timeline.play()

    # test currently not working
    # async def test_raycast_targets(self):

    #     # Add lidar
    #     lidarPath = "/World/Lidar"
    #     lidar = self.add_lidar(lidarPath)

    #     lidar.GetRotationRateAttr().Set(0.0)
    #     lidar.GetHighLodAttr().Set(True)

    #     vFOV = 45
    #     hFOV = 360
    #     vRes = 3
    #     hRes = 3

    #     vFOV = float(np.clip(vFOV, 0, 180))
    #     hFOV = float(np.clip(hFOV, 0, 360))

    #     azimuth = np.arange(-hFOV / 2, hFOV / 2, hRes)
    #     zenith = np.arange(90 - vFOV / 2, 90 + vFOV / 2, vRes)

    #     lidar.GetHorizontalFovAttr().Set(hFOV)
    #     lidar.GetVerticalFovAttr().Set(vFOV)
    #     lidar.GetHorizontalResolutionAttr().Set(hRes)
    #     lidar.GetVerticalResolutionAttr().Set(vRes)

    #     # Run for a second
    #     self._timeline.play()
    #     await simulate_async(2.0)
    #     self._timeline.pause()

    #     kitZenith = self._lidar.get_zenith_data(lidarPath)
    #     kitAzimuth = self._lidar.get_azimuth_data(lidarPath)

    #     self.assertEqual(len(list(zenith.flatten())), len(kitZenith))
    #     self.assertEqual(len(list(azimuth.flatten())), len(kitAzimuth))

    #     for (a, b) in zip(list(zenith.flatten()), kitZenith):
    #         epsilon = vRes / 10.0
    #         self.assertTrue(a <= b + epsilon)
    #         self.assertTrue(a >= b - epsilon)

    #     for (a, b) in zip(list(azimuth.flatten()), kitAzimuth):
    #         epsilon = hRes / 10.0
    #         self.assertTrue(a <= b + epsilon)
    #         self.assertTrue(a >= b - epsilon)
