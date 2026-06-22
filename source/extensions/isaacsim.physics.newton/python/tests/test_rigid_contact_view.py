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

"""Verifies Newton rigid contact view sensor and filter pattern handling. The tests cover wildcard, bracket, explicit-list, one-to-many, and multi-filter contact view configurations and validate the resolved sensor paths."""

from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.physics.newton
import isaacsim.physics.newton.tensors
import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, PhysicsSchemaTools, UsdGeom, UsdPhysics


async def wait_for_stage_loading() -> Any:
    """Wait until USD stage loading is complete.

    Returns:
        None.
    """
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        await omni.kit.app.get_app().next_update_async()


class TestRigidContactView(omni.kit.test.AsyncTestCase):
    """Test rigid contact view pattern matching functionality."""

    async def setUp(self) -> None:
        """Set up test environment with multiple cubes and ground plane."""
        self.use_gpu = True
        self.wp_device = "cuda:0" if self.use_gpu else "cpu"

        await stage_utils.create_new_stage_async()
        self.stage = omni.usd.get_context().get_stage()

        PhysicsSchemaTools.addPhysicsScene(self.stage, "/physicsScene")
        await omni.kit.app.get_app().next_update_async()

        # Create multiple cubes for testing pattern matching
        for i in range(5):
            self._create_cube(f"/World/Cube_{i}", position=(i * 2.0, 0.0, 1.0))

        # Create spheres as filter objects (same count as cubes for pattern matching)
        for i in range(5):
            self._create_sphere(f"/World/Sphere_{i}", position=(i * 2.0, 1.0, 1.0))

        # Create boxes as additional filter objects (same count as cubes for pattern matching)
        for i in range(5):
            self._create_cube(f"/World/Box_{i}", position=(i * 2.0, -1.0, 1.0), size=0.5)

        # Create ground plane
        self._create_ground_plane("/World/GroundPlane")

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

    def _create_cube(self, path: Any, position: Any = (0, 0, 0), size: Any = 1.0, mass: Any = 1.0) -> Any:
        """Create a cube with physics.

        Args:
            path: USD path where the cube should be created.
            position: Translation to apply to the cube.
            size: Cube size to author.
            mass: Mass value to author.

        Returns:
            None.
        """
        cube_geom = UsdGeom.Cube.Define(self.stage, path)
        cube_prim = self.stage.GetPrimAtPath(path)
        cube_geom.CreateSizeAttr(size)
        cube_geom.AddTranslateOp().Set(position)
        UsdPhysics.CollisionAPI.Apply(cube_prim)
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        UsdPhysics.MassAPI.Apply(cube_prim)

        # Set mass and inertia (I = (1/6) * m * s^2 for uniform cube)
        mass_api = UsdPhysics.MassAPI(cube_prim)
        mass_api.CreateMassAttr(mass)
        cube_inertia = (1.0 / 6.0) * mass * size * size
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(cube_inertia, cube_inertia, cube_inertia))

    def _create_sphere(self, path: Any, position: Any = (0, 0, 0), radius: Any = 0.5, mass: Any = 1.0) -> Any:
        """Create a sphere with physics.

        Args:
            path: USD path where the sphere should be created.
            position: Translation to apply to the sphere.
            radius: Sphere radius to author.
            mass: Mass value to author.

        Returns:
            None.
        """
        sphere_geom = UsdGeom.Sphere.Define(self.stage, path)
        sphere_prim = self.stage.GetPrimAtPath(path)
        sphere_geom.CreateRadiusAttr(radius)
        sphere_geom.AddTranslateOp().Set(position)
        UsdPhysics.CollisionAPI.Apply(sphere_prim)
        UsdPhysics.RigidBodyAPI.Apply(sphere_prim)
        UsdPhysics.MassAPI.Apply(sphere_prim)

        # Set mass and inertia (I = (2/5) * m * r^2 for uniform sphere)
        mass_api = UsdPhysics.MassAPI(sphere_prim)
        mass_api.CreateMassAttr(mass)
        sphere_inertia = (2.0 / 5.0) * mass * radius * radius
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(sphere_inertia, sphere_inertia, sphere_inertia))

    def _create_ground_plane(self, path: Any) -> Any:
        """Create a ground plane.

        Args:
            path: USD path where the ground plane should be created.

        Returns:
            None.
        """
        plane_geom = UsdGeom.Plane.Define(self.stage, path)
        plane_prim = self.stage.GetPrimAtPath(path)
        plane_geom.CreateAxisAttr("Z")
        UsdPhysics.CollisionAPI.Apply(plane_prim)

    async def test_wildcard_sensor_pattern(self) -> None:
        """Test creating contact view with wildcard sensor pattern."""
        contact_view = self.sim.create_rigid_contact_view("/World/Cube_*")

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 5, "Should match all 5 cubes")

    async def test_bracket_sensor_pattern(self) -> None:
        """Test creating contact view with bracket pattern."""
        contact_view = self.sim.create_rigid_contact_view("/World/Cube_[0-2]")

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 3, "Should match cubes 0, 1, 2")

    async def test_list_sensor_patterns(self) -> None:
        """Test creating contact view with list of sensor patterns."""
        contact_view = self.sim.create_rigid_contact_view(["/World/Cube_[0-2]", "/World/Cube_[3-4]"])

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 5, "Should match all 5 cubes via two patterns")

    async def test_explicit_sensor_list(self) -> None:
        """Test creating contact view with explicit list of sensor paths."""
        contact_view = self.sim.create_rigid_contact_view(["/World/Cube_0", "/World/Cube_2", "/World/Cube_4"])

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 3, "Should match exactly 3 specified cubes")

    async def test_no_filters(self) -> None:
        """Test creating contact view without filters (only net forces available).

        Corresponds to: view_1 = sim_view.create_rigid_contact_view("/World/Cube_*")
        """
        contact_view = self.sim.create_rigid_contact_view("/World/Cube_*")

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 5, "Should have 5 cube sensors")
        self.assertEqual(contact_view.filter_count, 0, "Should have no filters")

    async def test_single_filter_all_sensors(self) -> None:
        """Test single filter pattern that matches all sensors.

        Corresponds to: view_4 = sim_view.create_rigid_contact_view("/World/Cube_*", ["/World/GroundPlane"])

        Note: Single prim exception - GroundPlane will match with all cube sensors.
        When pattern is a string, it becomes a list of 1 element, so filter_patterns must be [[...]]
        """
        contact_view = self.sim.create_rigid_contact_view("/World/Cube_*", filter_patterns=[["/World/GroundPlane"]])

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 5, "Should have 5 cube sensors")
        self.assertGreaterEqual(contact_view.filter_count, 1, "Should have at least 1 filter")

        # Verify filter paths (list of lists, one per sensor)
        self.assertIsNotNone(contact_view.filter_paths)
        self.assertGreater(len(contact_view.filter_paths), 0, "Should have filter paths for sensors")
        # Single prim exception: GroundPlane should appear in filters for first sensor
        self.assertIn("/World/GroundPlane", contact_view.filter_paths[0])

    async def test_multiple_filters_with_wildcard(self) -> None:
        """Test multiple filter patterns with wildcards.

        Corresponds to: view_5 = sim_view.create_rigid_contact_view("/World/Cube_*", ["/World/GroundPlane", "/World/Sphere_*"])

        Note: When pattern is a string, it becomes a list, so filter_patterns must be [[...]]
        """
        contact_view = self.sim.create_rigid_contact_view(
            "/World/Cube_*", filter_patterns=[["/World/GroundPlane", "/World/Sphere_*"]]
        )

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 5, "Should have 5 cube sensors")
        self.assertGreaterEqual(contact_view.filter_count, 1, "Should have filters")

        # Verify filter paths include both GroundPlane and Spheres
        self.assertIsNotNone(contact_view.filter_paths)
        self.assertGreater(len(contact_view.filter_paths), 0, "Should have filter paths")
        # Check first sensor's filters (should have GroundPlane and a Sphere)
        first_sensor_filters = contact_view.filter_paths[0]
        self.assertIn("/World/GroundPlane", first_sensor_filters, "Should have GroundPlane")
        # At least one sphere should be in filters
        sphere_found = any("/World/Sphere_" in path for path in first_sensor_filters)
        self.assertTrue(sphere_found, "Should have sphere filters")

    async def test_list_of_filter_lists(self) -> None:
        """Test list of sensor patterns with corresponding filter lists.

        Corresponds to: view_5 = sim_view.create_rigid_contact_view(
            ["/World/Cube_[0-2]", "/World/Cube_[3-4]"],
            [["/World/GroundPlane", "/World/Sphere_[0-2]"], ["/World/GroundPlane", "/World/Sphere_[0-1]"]]
        )

        Note: Each sensor pattern has its own list of filter patterns.
        """
        contact_view = self.sim.create_rigid_contact_view(
            ["/World/Cube_[0-2]", "/World/Cube_[3-4]"],
            filter_patterns=[
                ["/World/GroundPlane", "/World/Sphere_[0-2]"],
                ["/World/GroundPlane", "/World/Sphere_[0-1]"],
            ],
        )

        self.assertIsNotNone(contact_view)
        # Cubes 0,1,2 from first pattern + Cubes 3,4 from second pattern = 5 sensors
        self.assertEqual(contact_view.sensor_count, 5, "Should have 5 cube sensors")
        self.assertGreaterEqual(contact_view.filter_count, 1, "Should have filters")

        # Verify filter paths (list of lists, one per sensor)
        self.assertIsNotNone(contact_view.filter_paths)
        self.assertGreater(len(contact_view.filter_paths), 0, "Should have filter paths")
        # Check first sensor's filters
        first_sensor_filters = contact_view.filter_paths[0]
        self.assertIn("/World/GroundPlane", first_sensor_filters, "GroundPlane should be in filters")

    async def test_complex_filter_lists_with_boxes(self) -> None:
        """Test complex filter lists with multiple object types.

        Corresponds to: view_6 = sim_view.create_rigid_contact_view(
            ["/World/Cube_[0-2]", "/World/Cube_[3-4]"],
            [["/World/GroundPlane", "/World/Sphere_[0-2]", "/World/Box_[0-2]"],
             ["/World/GroundPlane", "/World/Sphere_[0-1]", "/World/Box_[0-1]"]]
        )

        Note: Each sensor matches with GroundPlane + one Sphere + one Box.
        """
        contact_view = self.sim.create_rigid_contact_view(
            ["/World/Cube_[0-2]"], filter_patterns=[["/World/GroundPlane", "/World/Sphere_[0-2]", "/World/Box_[0-2]"]]
        )

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, 3, "Should have 3 cube sensors")
        self.assertGreaterEqual(contact_view.filter_count, 1, "Should have filters")

        # Verify filter paths include all three types (checking first sensor)
        self.assertIsNotNone(contact_view.filter_paths)
        self.assertGreater(len(contact_view.filter_paths), 0, "Should have filter paths")
        first_sensor_filters = contact_view.filter_paths[0]
        self.assertIn("/World/GroundPlane", first_sensor_filters, "Should have GroundPlane")
        sphere_found = any("/World/Sphere_" in path for path in first_sensor_filters)
        self.assertTrue(sphere_found, "Should have spheres in filters")
        box_found = any("/World/Box_" in path for path in first_sensor_filters)
        self.assertTrue(box_found, "Should have boxes in filters")

    async def test_one_to_many_relationship(self) -> None:
        """Test one-to-many relationship (each sensor tracks contacts with multiple filters).

        Corresponds to: view_7 = create_rigid_contact_view(
            [f"/World/Cube_{j}" for j in range(num_sensors)],
            filter_patterns=[[f"/World/Sphere_{j}" for j in range(num_filters)]] * num_sensors
        )

        Note: Each sensor has the same list of filters (all 5 spheres).
        """
        num_sensors = 5
        num_filters = 5

        contact_view = self.sim.create_rigid_contact_view(
            [f"/World/Cube_{j}" for j in range(num_sensors)],
            filter_patterns=[[f"/World/Sphere_{j}" for j in range(num_filters)]] * num_sensors,
        )

        self.assertIsNotNone(contact_view)
        self.assertEqual(contact_view.sensor_count, num_sensors, f"Should have {num_sensors} sensors")
        self.assertGreaterEqual(contact_view.filter_count, num_filters, f"Should have at least {num_filters} filters")

        # Verify all spheres are in filter paths (checking first sensor since all have same filters)
        self.assertIsNotNone(contact_view.filter_paths)
        self.assertEqual(len(contact_view.filter_paths), num_sensors, "Should have filter list for each sensor")
        first_sensor_filters = contact_view.filter_paths[0]
        for i in range(num_filters):
            self.assertIn(f"/World/Sphere_{i}", first_sensor_filters, f"Should have Sphere_{i} in filters")

    async def test_sensor_paths_verification(self) -> None:
        """Test that sensor paths are correctly populated."""
        contact_view = self.sim.create_rigid_contact_view(["/World/Cube_0", "/World/Cube_1", "/World/Cube_2"])

        self.assertIsNotNone(contact_view.sensor_paths)
        self.assertEqual(len(contact_view.sensor_paths), 3, "Should have 3 sensor paths")
        self.assertIn("/World/Cube_0", contact_view.sensor_paths)
        self.assertIn("/World/Cube_1", contact_view.sensor_paths)
        self.assertIn("/World/Cube_2", contact_view.sensor_paths)
