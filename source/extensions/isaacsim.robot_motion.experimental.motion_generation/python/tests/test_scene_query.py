# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test suite for scene query functionality in robot motion generation."""

import isaacsim.core.experimental.utils.stage as stage_utils

# Import extension python module we are testing with absolute import path, as if we are an external user (i.e. a different extension)
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import (
    Cube,
    Plane,
    Sphere,
)
from isaacsim.core.experimental.prims import (
    GeomPrim,
    RigidPrim,
)
from isaacsim.robot_motion.experimental.motion_generation import SceneQuery, TrackableApi
from isaacsim.storage.native import get_assets_root_path
from omni.kit.app import get_app


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of the module will make it auto-discoverable by omni.kit.test
class TestSceneQuery(omni.kit.test.AsyncTestCase):
    """Test scene query operations for detecting prims in the USD stage."""

    # Before running each test
    async def setUp(self):
        """Set up test environment before each test."""
        await stage_utils.create_new_stage_async()

        await get_app().next_update_async()

        # Initialize timeline
        self._timeline = omni.timeline.get_timeline_interface()

    # After running each test
    async def tearDown(self):
        """Clean up test environment after each test."""
        # Stop timeline if running
        if self._timeline.is_playing():
            self._timeline.stop()

        # Clean up physics callbacks if any
        if hasattr(self, "sample") and self.sample:
            self.sample.physics_cleanup()

        await get_app().next_update_async()

    async def test_scene_query_with_box_prim(self):
        """Test scene query detection of a box prim with collision API."""
        # Add a Cube to the scene:
        cube_path = "/World/Cube"
        cube = Cube(
            paths=cube_path,
            sizes=1.0,
            positions=[0.0, 0.0, 0.0],
        )

        # define a collision API for this prim:
        GeomPrim(paths=cube_path, apply_collision_apis=True)
        scene_query = SceneQuery()

        # the cube should show up in a large box:
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path in prim_list)

        # The cube should not show up if the search box is far away:
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[3.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path not in prim_list)

        # if the the cube is far away then it does not show up:
        cube.set_world_poses(positions=[2.1, 0.0, 0.0])
        await get_app().next_update_async()

        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path not in prim_list)

        # if the the cube is far away then it does not show up:
        cube.set_world_poses(positions=[0.0, 2.1, 0.0])
        await get_app().next_update_async()

        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path not in prim_list)

        # if the the cube is far away then it does not show up:
        cube.set_world_poses(positions=[0.0, 0.0, -2.1])
        await get_app().next_update_async()

        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path not in prim_list)

        # a very very small box should be included:
        cube.set_world_poses(positions=[0.0, 0.0, 0.0])
        cube.set_sizes(0.0001)
        await get_app().next_update_async()

        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path in prim_list)

        # a very very large box should be included:
        cube.set_world_poses(positions=[0.0, 0.0, 0.0])
        cube.set_sizes(10000.0)
        await get_app().next_update_async()

        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path in prim_list)

    async def test_scene_query_with_unrotated_plane_prim(self):
        """Test scene query detection of an unrotated plane prim."""
        # Add a Plane to the scene:
        plane_path = "/World/Plane"
        plane = Plane(paths=plane_path, axes="Z")

        # define a collision API for this prim:
        GeomPrim(paths=plane_path, apply_collision_apis=True)
        scene_query = SceneQuery()

        # the plane should be in a search box which goes into the negative-z:
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

        # the plane should not be in a search box which is entirely positive:
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[0.5, 0.5, 0.5],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path not in prim_list)

        # the plane should intersect if I raise it in z, until it goes above the box:
        plane.set_world_poses(positions=[0.0, 0.0, 0.5])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

        plane.set_world_poses(positions=[0.0, 0.0, 1.0001])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path not in prim_list)

        # Now, set the axis to "X".
        plane.set_axes("X")
        # No matter how high in Z, we still intersect the box:
        plane.set_world_poses(positions=[0.0, 0.0, 10000.0])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

        # Moving the plane in X, we do not intersect:
        plane.set_world_poses(positions=[-1.0001, 0.0, 0.0])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path not in prim_list)

        # Moving the search box over, we do intersect again:
        plane.set_world_poses(positions=[-1.0001, 0.0, 0.0])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[-0.5, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

        # Now, set the axis to "Y".
        plane.set_axes("Y")
        # No matter how high in Z, we still intersect the box:
        plane.set_world_poses(positions=[0.0, 0.0, 10000.0])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

        # Moving the plane in Y, we do not intersect:
        plane.set_world_poses(positions=[0, -1.0001, 0.0])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path not in prim_list)

        # Moving the search box over, we do intersect again:
        plane.set_world_poses(positions=[-1.0001, 0.0, 0.0])
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, -0.5, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

    async def test_scene_query_with_rotated_plane_prim(self):
        """Test scene query detection of a rotated plane prim."""
        # Add a Plane to the scene:
        plane_path = "/World/Plane"
        plane = Plane(paths=plane_path, axes="Z")

        # define a collision API for this prim:
        GeomPrim(paths=plane_path, apply_collision_apis=True)
        scene_query = SceneQuery()

        # rotate the plane by -45 degrees about the X-axis. This is a position
        # where we should NOT hit the search box.
        angle = -np.pi / 4
        w = np.cos(angle / 2)
        xyz = np.array([1.0, 0.0, 0.0]) * np.sin(angle / 2)

        plane.set_world_poses(positions=[0.0, -1.01, -1.01], orientations=[w, *xyz])

        # we should not hit the search box in this case:
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path not in prim_list)

        # if we set the rotation to 45 degrees, we should intersect the search box:
        angle = np.pi / 4
        w = np.cos(angle / 2)
        xyz = np.array([1.0, 0.0, 0.0]) * np.sin(angle / 2)

        plane.set_world_poses(positions=[0.0, -1.01, -1.01], orientations=[w, *xyz])

        # we should not hit the search box in this case:
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

        # If we set the axis to "Y" and then rotate -45, we SHOULD intersect the search box.
        angle = -np.pi / 4
        w = np.cos(angle / 2)
        xyz = np.array([1.0, 0.0, 0.0]) * np.sin(angle / 2)

        plane.set_world_poses(positions=[0.0, -1.01, -1.01], orientations=[w, *xyz])
        plane.set_axes("Y")

        # we should not hit the search box in this case:
        await get_app().next_update_async()
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1.0, 1.0, 1.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(plane_path in prim_list)

    async def test_different_searchable_apis(self):
        """Test scene query with different searchable API types."""
        # Add a dynamic cube, and a collision cube:
        dynamic_cube_path = "/World/DynamicCube"
        collision_cube_path = "/World/CollisionCube"
        dynamic_cube = Cube(
            paths=dynamic_cube_path,
            sizes=2.0,
        )

        collision_cube = Cube(
            paths=collision_cube_path,
            sizes=1.0,
        )

        GeomPrim(paths=collision_cube_path, apply_collision_apis=True)
        RigidPrim(paths=dynamic_cube_path, masses=[1.0])

        scene_query = SceneQuery()
        dynamic_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1] * 3,
            search_box_maximum=[1] * 3,
            tracked_api=TrackableApi.PHYSICS_RIGID_BODY,
        )

        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1] * 3,
            search_box_maximum=[1] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        self.assertTrue(dynamic_cube_path in dynamic_prims)
        self.assertTrue(dynamic_cube_path not in collision_prims)
        self.assertTrue(collision_cube_path not in dynamic_prims)
        self.assertTrue(collision_cube_path in collision_prims)

        # Test that invalid API will raise an error
        from enum import Enum

        class MyForcedApi(Enum):
            MOTION_PLANNING_COLLISION = "not_real"

        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1] * 3,
            search_box_maximum=[1] * 3,
            tracked_api=MyForcedApi.MOTION_PLANNING_COLLISION,
        )

    async def test_motion_generation_collision_api(self):
        """Test scene query with the motion generation collision API."""
        # Test the new motion generation collision API
        motion_planning_cube_path = "/World/MotionPlanningCube"
        motion_planning_cube = Cube(
            paths=motion_planning_cube_path,
            sizes=1.0,
        )

        # Apply the motion planning API to the cube
        import omni.usd
        from isaacsim.robot_motion.schema import apply_motion_planning_api

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(motion_planning_cube_path)
        apply_motion_planning_api(prim, enabled=True)

        scene_query = SceneQuery()
        motion_planning_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1] * 3,
            search_box_maximum=[1] * 3,
            tracked_api=TrackableApi.MOTION_GENERATION_COLLISION,
        )

        self.assertTrue(motion_planning_cube_path in motion_planning_prims)

        # Test that physics collision prims are not found with motion generation API
        physics_cube_path = "/World/PhysicsCube"
        physics_cube = Cube(
            paths=physics_cube_path,
            sizes=1.0,
        )
        GeomPrim(paths=physics_cube_path, apply_collision_apis=True)

        motion_planning_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1] * 3,
            search_box_maximum=[1] * 3,
            tracked_api=TrackableApi.MOTION_GENERATION_COLLISION,
        )

        self.assertTrue(motion_planning_cube_path in motion_planning_prims)
        self.assertTrue(physics_cube_path not in motion_planning_prims)

        # Test that motion planning prims are not found with physics collision API
        physics_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1] * 3,
            search_box_maximum=[1] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        self.assertTrue(physics_cube_path in physics_prims)
        self.assertTrue(motion_planning_cube_path not in physics_prims)

    async def test_search_bounds(self):
        """Test scene query with various search box bounds and positions."""
        scene_query = SceneQuery()

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[1] * 3,
            search_box_maximum=[1] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[1] * 3,
            search_box_maximum=[-1] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1, 1, -1],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1, -1, 1],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[-1, 1, 1],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[[1, 1, 1], [1, 1, 1]],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1, 1, 1, 1],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0, 0, 0],
            search_box_minimum=[-1, -1, -1],
            search_box_maximum=[1, 1, 1, 1],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # box is not well formed:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=[0, 0],
            search_box_minimum=[-1, -1],
            search_box_maximum=[1, 1],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # we can search with Numpy arrays:
        cube_path = "/Cube"
        Cube(paths=cube_path)
        GeomPrim(paths=cube_path, apply_collision_apis=True)
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=np.array([0, 0, 0]),
            search_box_minimum=np.array([-1, -1, -1]),
            search_box_maximum=np.array([1, 1, 1]),
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path in prim_list)

        # we can search with warp arrays:
        prim_list = scene_query.get_prims_in_aabb(
            search_box_origin=wp.array([0, 0, 0], dtype=wp.float32),
            search_box_minimum=wp.array([-1, -1, -1], dtype=wp.float32),
            search_box_maximum=wp.array([1, 1, 1], dtype=wp.float32),
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(cube_path in prim_list)

        # we cannot search will ill-formed numpy arrays:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=np.array([0, 0, 0, 0]),
            search_box_minimum=np.array([-1, -1, -1]),
            search_box_maximum=np.array([1, 1, 1, 1]),
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # we cannot search with ill-formed warp arrays:
        self.assertRaises(
            ValueError,
            scene_query.get_prims_in_aabb,
            search_box_origin=wp.array([0, 0], dtype=wp.float32),
            search_box_minimum=wp.array([-1, -1], dtype=wp.float32),
            search_box_maximum=wp.array([1, 1], dtype=wp.float32),
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

    async def test_robot_search(self):
        """Test scene query for finding robots in the stage."""
        # Add a robot to the stage:
        usd_path = f"{get_assets_root_path()}/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"

        for i in range(10):
            stage_utils.add_reference_to_stage(usd_path=usd_path, path=f"/World/Robot{i}")

        scene_query = SceneQuery()
        robot_paths = scene_query.get_robots_in_stage()

        self.assertTrue(len(robot_paths) == 10)

    async def test_robot_prim_tracking(self):
        """Test scene query with robot prim exclusion for collision detection."""
        usd_path = f"{get_assets_root_path()}/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"

        controlled_robot_path = "/World/ControlledRobot"
        other_robot_path = "/World/OtherRobot"
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=controlled_robot_path)
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=other_robot_path)

        # Now, we want to do a scene query where we will exclude our controlled robot:
        scene_query = SceneQuery()
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=[controlled_robot_path],
        )
        self.assertTrue(len(collision_prims) > 0)
        print(f"Collision prims that we found are: {collision_prims}")

    async def test_include_exclude_paths(self):
        """Test scene query include and exclude path filtering."""
        cube_paths = ["/World/Group1/Cube", "/World/Group2/Cube", "/World/Group3/Cube"]
        sphere_paths = ["/World/Group1/Sphere", "/World/Group2/Sphere", "/World/Group3/Sphere"]

        Cube(paths=cube_paths, sizes=5.0)
        GeomPrim(paths=cube_paths, apply_collision_apis=True)

        Sphere(paths=sphere_paths, radii=5.0)
        GeomPrim(paths=sphere_paths, apply_collision_apis=True)

        scene_query = SceneQuery()

        # If there are no inclusions/exclusions, then everything is there:
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )
        self.assertTrue(set([*cube_paths, *sphere_paths]) == set(collision_prims))

        # If I exlclude a group, it is not there:
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=["/World/Group1"],
        )
        self.assertTrue(
            set(["/World/Group2/Cube", "/World/Group2/Sphere", "/World/Group3/Cube", "/World/Group3/Sphere"])
            == set(collision_prims)
        )

        # If I exlclude a single group, it is not there:
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths="/World/Group1",
        )
        self.assertTrue(
            set(["/World/Group2/Cube", "/World/Group2/Sphere", "/World/Group3/Cube", "/World/Group3/Sphere"])
            == set(collision_prims)
        )

        # If I exlclude a group, it is not there:
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=["/World/Group2"],
        )
        self.assertTrue(
            set(["/World/Group1/Cube", "/World/Group1/Sphere", "/World/Group3/Cube", "/World/Group3/Sphere"])
            == set(collision_prims)
        )

        # If I include and then exclude the same group, the result is empty:
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=["/World/Group2"],
            include_prim_paths=["/World/Group2"],
        )
        self.assertTrue(len(collision_prims) == 0)

        # When I include a specific group, it is the only one there:
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            include_prim_paths=["/World/Group2"],
        )
        self.assertTrue(
            set(
                [
                    "/World/Group2/Cube",
                    "/World/Group2/Sphere",
                ]
            )
            == set(collision_prims)
        )

        # When I include a single group, it is the only one there:
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            include_prim_paths="/World/Group2",
        )
        self.assertTrue(
            set(
                [
                    "/World/Group2/Cube",
                    "/World/Group2/Sphere",
                ]
            )
            == set(collision_prims)
        )

    async def test_include_exclude_invalid_paths(self):
        """Test scene query behavior with invalid include and exclude paths."""
        cube_paths = ["/World/Group1/Cube", "/World/Group2/Cube"]
        Cube(paths=cube_paths, sizes=5.0)
        GeomPrim(paths=cube_paths, apply_collision_apis=True)

        scene_query = SceneQuery()

        # Including a missing prim path should return no results.
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            include_prim_paths=["/World/DoesNotExist"],
        )
        self.assertTrue(len(collision_prims) == 0)

        # Excluding a missing prim path should not filter anything.
        collision_prims = scene_query.get_prims_in_aabb(
            search_box_origin=[0] * 3,
            search_box_minimum=[-100] * 3,
            search_box_maximum=[100] * 3,
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=["/World/DoesNotExist"],
        )
        self.assertTrue(set(cube_paths) == set(collision_prims))
