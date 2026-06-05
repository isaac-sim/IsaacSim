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

"""Tests for ROS 2 simulation control services."""

import asyncio
import gc
import time

import numpy as np
import omni.kit.test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics


class TestSimControlServices(omni.kit.test.AsyncTestCase):
    """Test suite for sim control services."""

    # Before running each test
    async def setUp(self):
        """Set up test fixtures."""
        import rclpy

        await stage_utils.create_new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        if not rclpy.ok():
            rclpy.init()

    # After running each test
    async def tearDown(self):
        """Tear down test fixtures."""
        if omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await self.simulate_until_condition(
                lambda: omni.usd.get_context().get_stage_loading_status()[2] == 0,
                max_frames=3600,
            )

        self._timeline.is_stopped()
        await omni.kit.app.get_app().next_update_async()

        self._timeline = None

        gc.collect()

    async def simulate_until_condition(self, condition_func, max_frames=180, per_frame_callback=None):
        """Simulate until condition is met or maximum frames reached."""
        frames_run = 0
        while frames_run < max_frames:
            await omni.kit.app.get_app().next_update_async()
            if per_frame_callback is not None:
                per_frame_callback()
            frames_run += 1
            if condition_func():
                return True
        return False

    def create_test_stage(self):
        """Handle create test stage operation."""
        stage = omni.usd.get_context().get_stage()

        # Create World xform
        UsdGeom.Xform.Define(stage, "/World")

        # Create physics scene
        physics_scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Create distant light for scene illumination
        distant_light = UsdLux.DistantLight.Define(stage, "/World/Environment/DistantLight")
        distant_light.CreateIntensityAttr(1000)
        distant_light.CreateAngleAttr(1.0)
        distant_light.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

        # Create various xforms at different hierarchies
        UsdGeom.Xform.Define(stage, "/World/Environment")
        UsdGeom.Xform.Define(stage, "/World/Robots")
        UsdGeom.Xform.Define(stage, "/World/Sensors")

        # Create cube with RigidBody API
        cube_geom = UsdGeom.Cube.Define(stage, "/World/Objects/DynamicCube")
        cube_geom.CreateSizeAttr(1.0)
        cube_geom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 2.0))
        cube_prim = stage.GetPrimAtPath("/World/Objects/DynamicCube")
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        UsdPhysics.CollisionAPI.Apply(cube_prim)
        UsdPhysics.MeshCollisionAPI.Apply(cube_prim)

        # Create cone without RigidBody API
        cone_geom = UsdGeom.Cone.Define(stage, "/World/Objects/StaticCone")
        cone_geom.CreateHeightAttr(2.0)
        cone_geom.CreateRadiusAttr(0.5)
        cone_geom.AddTranslateOp().Set(Gf.Vec3f(2.0, 0.0, 1.0))

        return stage

    async def _call_service_async(self, service_type, service_name, request):
        """Call ROS2 services asynchronously."""
        import concurrent.futures

        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        def call_service():
            unique_id = f"{int(time.time() * 1000)}"
            node = rclpy.create_node(f"test_client_node_{unique_id}")
            try:
                client = node.create_client(service_type, service_name)

                # Wait for service
                if not client.wait_for_service(timeout_sec=60.0):
                    return None, "Service not available"

                # Make the call
                future = client.call_async(request)

                # Use executor to handle the response
                executor = SingleThreadedExecutor()
                executor.add_node(node)

                try:
                    executor.spin_until_future_complete(future, timeout_sec=60.0)
                    if future.done():
                        return future.result(), None
                    else:
                        return None, "Service call timed out"
                except Exception as e:
                    return None, f"Executor error: {e}"
                finally:
                    executor.shutdown()

            except Exception as e:
                return None, f"Service call failed: {e}"
            finally:
                node.destroy_node()

        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(call_service)
            try:
                result, error = await asyncio.get_event_loop().run_in_executor(None, future.result, 120.0)

                if error:
                    if "Service not available" in error:
                        self.skipTest(error)
                    else:
                        self.fail(error)

                return result

            except concurrent.futures.TimeoutError:
                self.fail("Service call timed out")
            except Exception as e:
                self.fail(f"Test execution failed: {e}")

    async def _call_action_async(self, action_type, action_name, goal):
        """Call ROS2 actions asynchronously."""
        import concurrent.futures

        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor

        def call_action():
            unique_id = f"{int(time.time() * 1000)}"
            node = rclpy.create_node(f"test_action_client_node_{unique_id}")
            try:
                client = ActionClient(node, action_type, action_name)

                # Wait for action server
                if not client.wait_for_server(timeout_sec=30.0):
                    return None, "Action server not available"

                # Send goal
                future = client.send_goal_async(goal)

                # Use executor to handle the response
                executor = SingleThreadedExecutor()
                executor.add_node(node)

                # Spin until goal is accepted
                try:
                    executor.spin_until_future_complete(future, timeout_sec=30.0)
                    if not future.done():
                        return None, "Goal acceptance timed out"

                    goal_handle = future.result()
                    if not goal_handle.accepted:
                        return None, "Goal was rejected"

                    # Wait for result
                    result_future = goal_handle.get_result_async()
                    executor.spin_until_future_complete(result_future, timeout_sec=30.0)

                    if result_future.done():
                        return result_future.result(), None
                    else:
                        return None, "Action execution timed out"

                except Exception as e:
                    return None, f"Executor error: {e}"
                finally:
                    executor.shutdown()

            except Exception as e:
                return None, f"Action call failed: {e}"
            finally:
                node.destroy_node()

        # Run the action call in a thread pool to avoid blocking
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(call_action)
            try:
                result, error = await asyncio.get_event_loop().run_in_executor(None, future.result, 120.0)

                if error:
                    if "Action server not available" in error:
                        self.skipTest(error)
                    else:
                        self.fail(error)

                return result

            except concurrent.futures.TimeoutError:
                self.fail("Action call timed out")
            except Exception as e:
                self.fail(f"Test execution failed: {e}")

    async def test_get_simulator_features_service(self):
        """Test get simulator features service."""
        from simulation_interfaces.srv import GetSimulatorFeatures

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        request = GetSimulatorFeatures.Request()
        result = await self._call_service_async(GetSimulatorFeatures, "/get_simulator_features", request)

        self.assertIsNotNone(result)

        # Assert specific expected values (order follows SimulatorFeatures.msg definition)
        expected_features = [0, 1, 5, 6, 10, 11, 12, 14, 20, 23, 24, 25, 26, 31, 32, 33, 40, 42, 43, 44, 45, 50]
        expected_spawn_formats = ["usd"]
        expected_custom_info = "Control Isaac Sim via ROS2 Simulation Interfaces."

        # Access the features attribute which should contain the list
        self.assertEqual(list(result.features.features), expected_features)
        self.assertEqual(list(result.features.spawn_formats), expected_spawn_formats)
        self.assertEqual(result.features.custom_info, expected_custom_info)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_simulation_state_service(self):
        """Test that GetSimulationState service returns correct state when using timeline controls.

        This test verifies the service correctly reports simulation state changes made through
        the timeline interface by testing play, pause, and stop transitions.
        """
        # fmt: off
        from simulation_interfaces.msg import SimulationState

        # fmt: on
        from simulation_interfaces.srv import GetSimulationState

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        request = GetSimulationState.Request()

        # Test initial state (should be stopped by default)
        result = await self._call_service_async(GetSimulationState, "/get_simulation_state", request)

        self.assertIsNotNone(result)
        self.assertIsNotNone(result.state)
        self.assertEqual(result.state.state, SimulationState.STATE_STOPPED)

        # Test PLAY state
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        result = await self._call_service_async(GetSimulationState, "/get_simulation_state", request)

        self.assertIsNotNone(result)
        self.assertEqual(result.state.state, SimulationState.STATE_PLAYING)

        # Test PAUSE state
        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()

        result = await self._call_service_async(GetSimulationState, "/get_simulation_state", request)

        self.assertIsNotNone(result)
        self.assertEqual(result.state.state, SimulationState.STATE_PAUSED)

        # Test STOP state
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        result = await self._call_service_async(GetSimulationState, "/get_simulation_state", request)

        self.assertIsNotNone(result)
        self.assertEqual(result.state.state, SimulationState.STATE_STOPPED)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_set_simulation_state_service(self):
        """Test that SetSimulationState service correctly controls the timeline interface.

        This test verifies the service correctly changes timeline state by setting different
        simulation states via the service and checking the timeline interface reflects the changes.
        """
        # fmt: off
        from simulation_interfaces.msg import Result, SimulationState

        # fmt: on
        from simulation_interfaces.srv import SetSimulationState

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Verify initial state is stopped
        self.assertTrue(self._timeline.is_stopped())

        # Test setting to PLAYING state
        request = SetSimulationState.Request()
        request.state.state = SimulationState.STATE_PLAYING

        result = await self._call_service_async(SetSimulationState, "/set_simulation_state", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        await omni.kit.app.get_app().next_update_async()

        # Verify timeline is now playing
        self.assertTrue(self._timeline.is_playing())
        self.assertFalse(self._timeline.is_stopped())

        # Test setting to PAUSED state
        request.state.state = SimulationState.STATE_PAUSED

        result = await self._call_service_async(SetSimulationState, "/set_simulation_state", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        await omni.kit.app.get_app().next_update_async()

        # Verify timeline is now paused (not playing and not stopped)
        self.assertFalse(self._timeline.is_playing())
        self.assertFalse(self._timeline.is_stopped())

        # Test setting to STOPPED state
        request.state.state = SimulationState.STATE_STOPPED

        result = await self._call_service_async(SetSimulationState, "/set_simulation_state", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        await omni.kit.app.get_app().next_update_async()

        # Verify timeline is now stopped
        self.assertTrue(self._timeline.is_stopped())
        self.assertFalse(self._timeline.is_playing())

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_entities_service(self):
        """Test that GetEntities service returns all entities in the stage.

        This test creates a test stage, collects all prim paths using USD traversal,
        and verifies the GetEntities service returns the expected entities.
        """
        # fmt: off

        # fmt: on
        from simulation_interfaces.srv import GetEntities

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get all prim paths from USD stage using traversal
        stage = omni.usd.get_context().get_stage()
        expected_prim_paths = []

        for prim in stage.Traverse():
            if prim.IsValid() and not prim.GetPath().isEmpty:
                if not prim.GetPath().pathString.startswith("/Render"):
                    expected_prim_paths.append(str(prim.GetPath()))

        # Sort for consistent comparison
        expected_prim_paths.sort()
        # Call the GetEntities service
        request = GetEntities.Request()
        result = await self._call_service_async(GetEntities, "/get_entities", request)

        self.assertIsNotNone(result)
        self.assertIsNotNone(result.entities)

        # Extract entity paths from service result
        service_entity_paths = []
        for entity in result.entities:
            service_entity_paths.append(entity)

        # Sort for consistent comparison
        service_entity_paths.sort()

        # Verify the service returns all expected entities
        # The service might return additional entities, but should include all expected ones
        for expected_path in expected_prim_paths:
            self.assertIn(
                expected_path,
                service_entity_paths,
                f"Expected entity path not found in service result: {expected_path}",
            )

        # Verify we got some entities (stage shouldn't be empty)
        self.assertGreater(len(service_entity_paths), 0, "Service should return at least some entities")

        # Verify specific entities we created are included
        expected_created_entities = ["/World/Objects/DynamicCube", "/World/Objects/StaticCone", "/World/physicsScene"]

        for expected_entity in expected_created_entities:
            self.assertIn(
                expected_entity, service_entity_paths, f"Expected entity not found in service result: {expected_entity}"
            )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_entity_info_service(self):
        """Test that GetEntityInfo service returns correct information for an entity.

        This test verifies the service can retrieve entity information for different types
        of entities in the scene.
        """
        # fmt: off
        from simulation_interfaces.msg import EntityCategory, Result

        # fmt: on
        from simulation_interfaces.srv import GetEntityInfo

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Test entity info for DynamicCube (has RigidBody API)
        request = GetEntityInfo.Request()
        request.entity = "/World/Objects/DynamicCube"

        result = await self._call_service_async(GetEntityInfo, "/get_entity_info", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        # Verify entity info contains expected data
        self.assertIsNotNone(result.info)
        self.assertEqual(result.info.category.category, EntityCategory.CATEGORY_OBJECT)

        # Test entity info for StaticCone (no RigidBody API)
        request.entity = "/World/Objects/StaticCone"

        result = await self._call_service_async(GetEntityInfo, "/get_entity_info", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        # Verify entity info contains expected data
        self.assertIsNotNone(result.info)
        self.assertEqual(result.info.category.category, EntityCategory.CATEGORY_OBJECT)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_entity_state_service(self):
        """Test that GetEntityState service returns correct state information.

        This test uses RigidPrim experimental API to set transform and velocity values,
        then verifies the GetEntityState service returns the correct state information.
        """
        from isaacsim.core.experimental.prims import RigidPrim, XformPrim

        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import GetEntityState

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Test entity state for DynamicCube (has RigidBody API)
        # Set specific transform and velocity values using RigidPrim API
        cube_prim = RigidPrim("/World/Objects/DynamicCube", reset_xform_op_properties=True)

        # Set specific pose and velocities
        test_position = np.array([1.5, 2.0, 3.0])
        test_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz quaternion
        test_linear_velocity = np.array([0.5, 1.0, 1.5])
        test_angular_velocity = np.array([0.1, 0.2, 0.3])

        cube_prim.set_world_poses(positions=test_position, orientations=test_orientation)
        cube_prim.set_velocities(linear_velocities=test_linear_velocity, angular_velocities=test_angular_velocity)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Query the entity state via service
        request = GetEntityState.Request()
        request.entity = "/World/Objects/DynamicCube"

        result = await self._call_service_async(GetEntityState, "/get_entity_state", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result.state)

        # Validate the returned pose matches what we set
        returned_position = result.state.pose.position
        returned_orientation = result.state.pose.orientation

        self.assertAlmostEqual(returned_position.x, test_position[0], delta=0.1)
        self.assertAlmostEqual(returned_position.y, test_position[1], delta=0.1)
        self.assertAlmostEqual(returned_position.z, test_position[2], delta=0.1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Test entity state for StaticCone (no RigidBody API)
        # Set only transform (no velocity since it's static)
        cone_position = np.array([3.0, 4.0, 1.0])
        cone_orientation = np.array([0.707, 0.0, 0.0, 0.707])  # 90 degree rotation around X

        # Use XformPrim for static objects
        cone_prim = XformPrim("/World/Objects/StaticCone", reset_xform_op_properties=True)
        cone_prim.set_world_poses(positions=cone_position, orientations=cone_orientation)

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Query the static cone entity state via service
        request.entity = "/World/Objects/StaticCone"

        result = await self._call_service_async(GetEntityState, "/get_entity_state", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result.state)

        # Validate the returned pose matches what we set for cone
        cone_returned_position = result.state.pose.position

        self.assertEqual(cone_returned_position.x, cone_position[0])
        self.assertEqual(cone_returned_position.y, cone_position[1])
        self.assertEqual(cone_returned_position.z, cone_position[2])

        # Validate that velocities are zero for static object
        cone_linear_vel = result.state.twist.linear
        cone_angular_vel = result.state.twist.angular

        self.assertEqual(cone_linear_vel.x, 0.0)
        self.assertEqual(cone_linear_vel.y, 0.0)
        self.assertEqual(cone_linear_vel.z, 0.0)
        self.assertEqual(cone_angular_vel.x, 0.0)
        self.assertEqual(cone_angular_vel.y, 0.0)
        self.assertEqual(cone_angular_vel.z, 0.0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_entities_states_service(self):
        """Test that GetEntitiesStates service returns correct state information for multiple entities.

        This test sets up both cube and cone with specific transforms and velocities,
        then filters for entities containing "Objects" and validates the returned states.
        """
        from isaacsim.core.experimental.prims import RigidPrim, XformPrim

        # fmt: off
        from simulation_interfaces.msg import EntityFilters, Result

        # fmt: on
        from simulation_interfaces.srv import GetEntitiesStates

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Set up DynamicCube (has RigidBody API)
        cube_prim = RigidPrim("/World/Objects/DynamicCube", reset_xform_op_properties=True)

        # Set specific pose and velocities for cube
        test_position = np.array([1.5, 2.0, 3.0])
        test_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz quaternion
        test_linear_velocity = np.array([0.5, 1.0, 1.5])
        test_angular_velocity = np.array([0.1, 0.2, 0.3])

        cube_prim.set_world_poses(positions=test_position, orientations=test_orientation)
        cube_prim.set_velocities(linear_velocities=test_linear_velocity, angular_velocities=test_angular_velocity)
        await omni.kit.app.get_app().next_update_async()

        # Set up StaticCone (no RigidBody API)
        cone_prim = XformPrim("/World/Objects/StaticCone", reset_xform_op_properties=True)

        # Set specific pose for cone
        cone_position = np.array([3.0, 4.0, 1.0])
        cone_orientation = np.array([0.707, 0.0, 0.0, 0.707])  # 90 degree rotation around X

        cone_prim.set_world_poses(positions=cone_position, orientations=cone_orientation)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Query entities containing "Objects" via service
        request = GetEntitiesStates.Request()
        request.filters = EntityFilters()
        request.filters.filter = "Objects"  # Filter for entities containing "Objects"

        result = await self._call_service_async(GetEntitiesStates, "/get_entities_states", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result.entities)
        self.assertIsNotNone(result.states)

        # First check - should get 3 entities with "Objects" filter
        self.assertEqual(len(result.entities), 3)
        self.assertEqual(len(result.states), 3)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Now filter more specifically for entities starting with "/World/Objects/"
        request.filters.filter = "^/World/Objects/"  # Regex to match paths starting with "/World/Objects/"

        result = await self._call_service_async(GetEntitiesStates, "/get_entities_states", request)

        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result.entities)
        self.assertIsNotNone(result.states)

        # Ensure we got both specific entities (cube and cone)
        self.assertEqual(len(result.entities), 2)
        self.assertEqual(len(result.states), 2)

        # Find cube and cone in the results
        cube_index = None
        cone_index = None

        for i, entity_path in enumerate(result.entities):
            if "DynamicCube" in entity_path:
                cube_index = i
            elif "StaticCone" in entity_path:
                cone_index = i

        self.assertIsNotNone(cube_index, "DynamicCube not found in results")
        self.assertIsNotNone(cone_index, "StaticCone not found in results")

        # Validate cube state
        cube_state = result.states[cube_index]
        cube_returned_position = cube_state.pose.position

        self.assertAlmostEqual(cube_returned_position.x, test_position[0], delta=0.1)
        self.assertAlmostEqual(cube_returned_position.y, test_position[1], delta=0.1)
        self.assertAlmostEqual(cube_returned_position.z, test_position[2], delta=0.1)

        # Validate cone state
        cone_state = result.states[cone_index]
        cone_returned_position = cone_state.pose.position

        self.assertEqual(cone_returned_position.x, cone_position[0])
        self.assertEqual(cone_returned_position.y, cone_position[1])
        self.assertEqual(cone_returned_position.z, cone_position[2])

        # Validate that cone velocities are zero for static object
        cone_linear_vel = cone_state.twist.linear
        cone_angular_vel = cone_state.twist.angular

        self.assertEqual(cone_linear_vel.x, 0.0)
        self.assertEqual(cone_linear_vel.y, 0.0)
        self.assertEqual(cone_linear_vel.z, 0.0)
        self.assertEqual(cone_angular_vel.x, 0.0)
        self.assertEqual(cone_angular_vel.y, 0.0)
        self.assertEqual(cone_angular_vel.z, 0.0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_set_entity_state_service(self):
        """Test that SetEntityState service correctly sets entity states.

        This test sets entity states for both rigid body and static objects,
        tests with simulation playing and stopped, and validates velocity handling.
        """
        from isaacsim.core.experimental.prims import RigidPrim, XformPrim

        # fmt: off
        from simulation_interfaces.msg import EntityState, Result

        # fmt: on
        from simulation_interfaces.srv import SetEntityState

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Create prim objects for direct state checking
        cube_prim = RigidPrim("/World/Objects/DynamicCube", reset_xform_op_properties=True)
        cone_prim = XformPrim("/World/Objects/StaticCone", reset_xform_op_properties=True)

        # Test with simulation playing
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Test setting DynamicCube state (has RigidBody API)
        cube_request = SetEntityState.Request()
        cube_request.entity = "/World/Objects/DynamicCube"

        # Set specific pose and velocities for cube
        cube_request.state = EntityState()
        cube_request.state.pose.position.x = 2.5
        cube_request.state.pose.position.y = 3.0
        cube_request.state.pose.position.z = 4.0
        cube_request.state.pose.orientation.w = 1.0
        cube_request.state.twist.linear.x = 1.0
        cube_request.state.twist.linear.y = 2.0
        cube_request.state.twist.linear.z = 3.0
        cube_request.state.twist.angular.x = 0.1
        cube_request.state.twist.angular.y = 0.2
        cube_request.state.twist.angular.z = 0.3

        cube_result = await self._call_service_async(SetEntityState, "/set_entity_state", cube_request)

        self.assertIsNotNone(cube_result)
        self.assertTrue(cube_result.result.result == Result.RESULT_OK)

        # Validate cube state using RigidPrim (simulation playing)
        cube_positions, cube_orientations = cube_prim.get_world_poses()
        cube_linear_vel, cube_angular_vel = cube_prim.get_velocities()

        # Test setting StaticCone state (no RigidBody API)
        cone_request = SetEntityState.Request()
        cone_request.entity = "/World/Objects/StaticCone"

        # Set pose and provide velocities (but expect them to be ignored)
        cone_request.state = EntityState()
        cone_request.state.pose.position.x = 5.0
        cone_request.state.pose.position.y = 6.0
        cone_request.state.pose.position.z = 2.0
        cone_request.state.pose.orientation.w = 0.707
        cone_request.state.pose.orientation.x = 0.0
        cone_request.state.pose.orientation.y = 0.0
        cone_request.state.pose.orientation.z = 0.707
        cone_request.state.twist.linear.x = 5.0  # Should be ignored
        cone_request.state.twist.linear.y = 6.0  # Should be ignored
        cone_request.state.twist.linear.z = 7.0  # Should be ignored
        cone_request.state.twist.angular.x = 0.5  # Should be ignored
        cone_request.state.twist.angular.y = 0.6  # Should be ignored
        cone_request.state.twist.angular.z = 0.7  # Should be ignored

        cone_result = await self._call_service_async(SetEntityState, "/set_entity_state", cone_request)

        self.assertIsNotNone(cone_result)
        self.assertTrue(cone_result.result.result == Result.RESULT_OK)

        await omni.kit.app.get_app().next_update_async()

        # Validate cone state using XformPrim (position only)
        cone_positions, cone_orientations = cone_prim.get_world_poses()

        # Check cube position - convert warp arrays to numpy
        cube_pos_np = cube_positions.numpy()
        self.assertAlmostEqual(cube_pos_np[0][0], 2.5, delta=0.2)
        self.assertAlmostEqual(cube_pos_np[0][1], 3.0, delta=0.2)
        self.assertAlmostEqual(cube_pos_np[0][2], 4.0, delta=0.2)

        # Check cube velocities - convert warp arrays to numpy
        cube_linear_vel_np = cube_linear_vel.numpy()
        cube_angular_vel_np = cube_angular_vel.numpy()
        self.assertAlmostEqual(cube_linear_vel_np[0][0], 1.0, delta=0.2)
        self.assertAlmostEqual(cube_linear_vel_np[0][1], 2.0, delta=0.2)
        self.assertAlmostEqual(cube_linear_vel_np[0][2], 3.0, delta=0.2)
        self.assertAlmostEqual(cube_angular_vel_np[0][0], 0.1, delta=0.2)
        self.assertAlmostEqual(cube_angular_vel_np[0][1], 0.2, delta=0.2)
        self.assertAlmostEqual(cube_angular_vel_np[0][2], 0.3, delta=0.2)

        # Check cone position - convert warp arrays to numpy
        cone_pos_np = cone_positions.numpy()
        self.assertEqual(cone_pos_np[0][0], 5.0)
        self.assertEqual(cone_pos_np[0][1], 6.0)
        self.assertEqual(cone_pos_np[0][2], 2.0)

        # Now test with simulation stopped
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Set different states when stopped
        cube_request.state.pose.position.x = 1.5
        cube_request.state.pose.position.y = 2.5
        cube_request.state.pose.position.z = 3.5
        cube_request.state.twist.linear.x = 0.5
        cube_request.state.twist.linear.y = 1.5
        cube_request.state.twist.linear.z = 2.5

        cube_result = await self._call_service_async(SetEntityState, "/set_entity_state", cube_request)

        self.assertIsNotNone(cube_result)
        self.assertTrue(cube_result.result.result == Result.RESULT_OK)

        cone_request.state.pose.position.x = 4.5
        cone_request.state.pose.position.y = 5.5
        cone_request.state.pose.position.z = 1.5

        cone_result = await self._call_service_async(SetEntityState, "/set_entity_state", cone_request)

        self.assertIsNotNone(cone_result)
        self.assertTrue(cone_result.result.result == Result.RESULT_OK)

        await omni.kit.app.get_app().next_update_async()

        # Validate states using experimental prims when simulation is stopped
        cube_positions_stopped, _ = cube_prim.get_world_poses()

        # Convert warp arrays to numpy for indexing
        cube_pos_stopped_np = cube_positions_stopped.numpy()
        self.assertAlmostEqual(cube_pos_stopped_np[0][0], 1.5, delta=0.1)
        self.assertAlmostEqual(cube_pos_stopped_np[0][1], 2.5, delta=0.1)
        self.assertAlmostEqual(cube_pos_stopped_np[0][2], 3.5, delta=0.1)

        cone_positions_stopped, _ = cone_prim.get_world_poses()

        # Convert warp arrays to numpy for indexing
        cone_pos_stopped_np = cone_positions_stopped.numpy()
        self.assertEqual(cone_pos_stopped_np[0][0], 4.5)
        self.assertEqual(cone_pos_stopped_np[0][1], 5.5)
        self.assertEqual(cone_pos_stopped_np[0][2], 1.5)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_spawn_entity_basic_default_position(self):
        """Test basic entity spawn with default position using USD file.

        This test first deletes an existing object, then spawns a robot entity using a USD file at the default position.
        """
        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import DeleteEntity, SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # First, verify that DynamicCube exists in the stage
        cube_prim_before = stage.GetPrimAtPath("/World/Objects/DynamicCube")
        self.assertTrue(cube_prim_before.IsValid(), "DynamicCube should exist before deletion")

        # Delete the existing DynamicCube using DeleteEntity service
        delete_request = DeleteEntity.Request()
        delete_request.entity = "/World/Objects/DynamicCube"

        delete_result = await self._call_service_async(DeleteEntity, "/delete_entity", delete_request)

        self.assertIsNotNone(delete_result)
        self.assertTrue(delete_result.result.result == Result.RESULT_OK)

        # Verify the cube no longer exists
        cube_prim_after_delete = stage.GetPrimAtPath("/World/Objects/DynamicCube")
        self.assertFalse(cube_prim_after_delete.IsValid(), "DynamicCube should not exist after deletion")

        # Basic entity spawn with default position using USD file
        request = SpawnEntity.Request()
        request.name = "BasicEntity"
        request.allow_renaming = False
        request.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        result = await self._call_service_async(SpawnEntity, "/spawn_entity", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        # Verify the entity exists in the stage
        spawned_prim = stage.GetPrimAtPath("/BasicEntity")
        self.assertTrue(spawned_prim.IsValid(), "BasicEntity should exist in the stage")

        # Verify the entity is at origin with forward orientation using experimental prims
        from isaacsim.core.experimental.prims import XformPrim

        # Create XformPrim for the spawned entity
        entity_prim = XformPrim("/BasicEntity", reset_xform_op_properties=True)

        # Get the actual world poses
        positions, orientations = entity_prim.get_world_poses()

        # Convert warp arrays to numpy for comparison
        actual_position = positions.numpy()[0]  # First (and only) entity
        actual_orientation = orientations.numpy()[0]  # First (and only) entity (w, x, y, z)

        # Expected values for origin and forward orientation
        expected_position = np.array([0.0, 0.0, 0.0])  # Origin
        expected_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion (forward orientation)

        # Verify position is at origin (with some tolerance for floating point precision)
        self.assertAlmostEqual(
            actual_position[0],
            expected_position[0],
            delta=0.01,
            msg=f"X position should be at origin: expected {expected_position[0]}, got {actual_position[0]}",
        )
        self.assertAlmostEqual(
            actual_position[1],
            expected_position[1],
            delta=0.01,
            msg=f"Y position should be at origin: expected {expected_position[1]}, got {actual_position[1]}",
        )
        self.assertAlmostEqual(
            actual_position[2],
            expected_position[2],
            delta=0.01,
            msg=f"Z position should be at origin: expected {expected_position[2]}, got {actual_position[2]}",
        )

        # Verify orientation is forward/identity
        # Check if quaternions are close or their negation is close (both represent same rotation)
        def quaternions_almost_equal(q1, q2, delta=0.01):
            return (
                abs(q1[0] - q2[0]) < delta
                and abs(q1[1] - q2[1]) < delta
                and abs(q1[2] - q2[2]) < delta
                and abs(q1[3] - q2[3]) < delta
            )

        orientation_close = quaternions_almost_equal(actual_orientation, expected_orientation)
        orientation_negated_close = quaternions_almost_equal(actual_orientation, -expected_orientation)

        self.assertTrue(
            orientation_close or orientation_negated_close,
            f"Orientation should be forward/identity: expected {expected_orientation} (or negated), got {actual_orientation}",
        )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_spawn_entity_with_position_orientation(self):
        """Test spawning entity with specific position and orientation.

        This test spawns a robot entity with explicitly set initial pose.
        """
        from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # Define expected pose values
        expected_position = np.array([1.0, 2.0, 3.0])
        expected_orientation = np.array([0.707, 0.0, 0.0, 0.707])  # w, x, y, z (90° around Z-axis)

        # Spawn with specific position and orientation
        request = SpawnEntity.Request()
        request.name = "PositionedEntity"
        request.allow_renaming = False
        request.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        # Create PoseStamped message using expected values
        request.initial_pose = PoseStamped()
        request.initial_pose.pose = Pose()
        request.initial_pose.pose.position = Point()
        request.initial_pose.pose.position.x = float(expected_position[0])
        request.initial_pose.pose.position.y = float(expected_position[1])
        request.initial_pose.pose.position.z = float(expected_position[2])
        request.initial_pose.pose.orientation = Quaternion()
        request.initial_pose.pose.orientation.w = float(expected_orientation[0])
        request.initial_pose.pose.orientation.x = float(expected_orientation[1])
        request.initial_pose.pose.orientation.y = float(expected_orientation[2])
        request.initial_pose.pose.orientation.z = float(expected_orientation[3])

        result = await self._call_service_async(SpawnEntity, "/spawn_entity", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        # Verify the entity exists and is positioned correctly
        spawned_prim = stage.GetPrimAtPath("/PositionedEntity")
        self.assertTrue(spawned_prim.IsValid(), "PositionedEntity should exist in the stage")

        # Verify the actual pose using experimental prims
        from isaacsim.core.experimental.prims import XformPrim

        # Create XformPrim for the spawned entity
        entity_prim = XformPrim("/PositionedEntity", reset_xform_op_properties=True)

        # Get the actual world poses
        positions, orientations = entity_prim.get_world_poses()

        # Convert warp arrays to numpy for comparison
        actual_position = positions.numpy()[0]  # First (and only) entity
        actual_orientation = orientations.numpy()[0]  # First (and only) entity (w, x, y, z)

        # Verify position (with some tolerance for floating point precision)
        self.assertAlmostEqual(
            actual_position[0],
            expected_position[0],
            delta=0.01,
            msg=f"X position mismatch: expected {expected_position[0]}, got {actual_position[0]}",
        )
        self.assertAlmostEqual(
            actual_position[1],
            expected_position[1],
            delta=0.01,
            msg=f"Y position mismatch: expected {expected_position[1]}, got {actual_position[1]}",
        )
        self.assertAlmostEqual(
            actual_position[2],
            expected_position[2],
            delta=0.01,
            msg=f"Z position mismatch: expected {expected_position[2]}, got {actual_position[2]}",
        )

        # Verify orientation (quaternions can be tricky due to equivalent representations and precision)
        # Check if quaternions are close or their negation is close (both represent same rotation)
        def quaternions_almost_equal(q1, q2, delta=0.01):
            return (
                abs(q1[0] - q2[0]) < delta
                and abs(q1[1] - q2[1]) < delta
                and abs(q1[2] - q2[2]) < delta
                and abs(q1[3] - q2[3]) < delta
            )

        orientation_close = quaternions_almost_equal(actual_orientation, expected_orientation)
        orientation_negated_close = quaternions_almost_equal(actual_orientation, -expected_orientation)

        self.assertTrue(
            orientation_close or orientation_negated_close,
            f"Orientation mismatch: expected {expected_orientation} (or negated), got {actual_orientation}",
        )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_spawn_entity_empty_xform(self):
        """Test empty Xform creation (no URI provided).

        This test creates an empty Xform prim without loading any USD content.
        """
        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        stage = omni.usd.get_context().get_stage()

        # Empty Xform creation (no URI)
        request = SpawnEntity.Request()
        request.name = "EmptyXform"
        request.allow_renaming = False
        request.uri = ""  # Empty URI should create just an Xform

        result = await self._call_service_async(SpawnEntity, "/spawn_entity", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        # Verify the empty Xform exists
        spawned_prim = stage.GetPrimAtPath("/EmptyXform")
        self.assertTrue(spawned_prim.IsValid(), "EmptyXform should exist in the stage")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_spawn_entity_auto_renaming(self):
        """Test spawning with auto-renaming enabled for duplicate names.

        This test spawns two entities with the same name, with auto-renaming enabled
        for the second one to verify unique name generation.
        """
        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # First spawn - original entity
        request1 = SpawnEntity.Request()
        request1.name = "DuplicateEntity"
        request1.allow_renaming = False
        request1.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        result1 = await self._call_service_async(SpawnEntity, "/spawn_entity", request1)

        self.assertIsNotNone(result1)
        self.assertTrue(result1.result.result == Result.RESULT_OK)

        # Verify first entity exists
        spawned_prim1 = stage.GetPrimAtPath("/DuplicateEntity")
        self.assertTrue(spawned_prim1.IsValid(), "First DuplicateEntity should exist in the stage")

        # Second spawn - with auto-renaming enabled (duplicate name)
        request2 = SpawnEntity.Request()
        request2.name = "DuplicateEntity"  # Same name as first
        request2.allow_renaming = True  # Should auto-rename
        request2.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        result2 = await self._call_service_async(SpawnEntity, "/spawn_entity", request2)

        self.assertIsNotNone(result2)
        self.assertTrue(result2.result.result == Result.RESULT_OK)

        spawned_prim2 = stage.GetPrimAtPath("/DuplicateEntity_1")
        self.assertTrue(spawned_prim2.IsValid(), "Original DuplicateEntity should still exist")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_spawn_entity_with_namespace(self):
        """Test spawning entity with namespace specified.

        This test spawns an entity with a namespace and verifies the isaac:namespace
        attribute is set and that ROS topics contain the namespace.
        """
        import rclpy

        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # With namespace specified
        request = SpawnEntity.Request()
        request.name = "NamespacedEntity"
        request.allow_renaming = False
        request.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"
        request.entity_namespace = "robot1"

        result = await self._call_service_async(SpawnEntity, "/spawn_entity", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)

        # Verify the entity exists at the expected path (not in namespace subdirectory)
        spawned_prim = stage.GetPrimAtPath("/NamespacedEntity")
        self.assertTrue(spawned_prim.IsValid(), "NamespacedEntity should exist at /NamespacedEntity")

        # Verify the isaac:namespace attribute is set
        if spawned_prim.HasAttribute("isaac:namespace"):
            namespace_attr = spawned_prim.GetAttribute("isaac:namespace")
            namespace_value = namespace_attr.Get()
            self.assertEqual(namespace_value, "robot1", "isaac:namespace attribute should be set to 'robot1'")
        else:
            self.fail("spawned prim should have isaac:namespace attribute")

        # Start simulation to activate ROS topics
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Check ROS topics for namespace directly
        unique_id = f"{int(time.time() * 1000)}"
        node = rclpy.create_node(f"namespace_test_node_{unique_id}")

        topic_names_and_types = node.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_names_and_types]

        # Look for topics containing the namespace
        namespace_topics = [topic for topic in topic_names if "/robot1/" in topic]
        has_namespace_topics = len(namespace_topics) > 0

        node.destroy_node()

        self.assertTrue(
            has_namespace_topics,
            f"Should find at least one topic with namespace '/robot1/'. Found topics: {namespace_topics}",
        )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_spawn_entity_error_cases(self):
        """Test SpawnEntity service error conditions.

        This test validates error handling for various invalid spawn scenarios.
        """
        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # Create an entity with unique entity name
        request_setup = SpawnEntity.Request()
        request_setup.name = "ExistingEntity"
        request_setup.allow_renaming = False
        request_setup.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        result_setup = await self._call_service_async(SpawnEntity, "/spawn_entity", request_setup)

        self.assertTrue(result_setup.result.result == Result.RESULT_OK)

        # Verify the entity exists at the expected path
        spawned_prim = stage.GetPrimAtPath("/ExistingEntity")
        self.assertTrue(spawned_prim.IsValid(), "ExistingEntity should exist at /ExistingEntity")

        # Error Case: NAME_NOT_UNIQUE when allow_renaming is false
        request_error1 = SpawnEntity.Request()
        request_error1.name = "ExistingEntity"  # Already exists
        request_error1.allow_renaming = False  # Should fail
        request_error1.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        result_error1 = await self._call_service_async(SpawnEntity, "/spawn_entity", request_error1)

        self.assertIsNotNone(result_error1)
        self.assertEqual(result_error1.result.result, SpawnEntity.Response.NAME_NOT_UNIQUE)

        # Error Case: Empty name and allow_renaming is false. Prim should be created with default prim name
        request_error2 = SpawnEntity.Request()
        request_error2.name = ""  # Empty name
        request_error2.allow_renaming = False  # Should fail
        request_error2.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        result_error2 = await self._call_service_async(SpawnEntity, "/spawn_entity", request_error2)

        self.assertIsNotNone(result_error2)
        self.assertEqual(result_error2.result.result, Result.RESULT_OK)
        # Verify the entity exists at the expected path (not in namespace subdirectory)
        spawned_prim = stage.GetPrimAtPath("/limo")
        self.assertTrue(spawned_prim.IsValid(), "limo should exist at /limo")

        # Error Case 3: RESOURCE_PARSE_ERROR for invalid USD file
        request_error3 = SpawnEntity.Request()
        request_error3.name = "InvalidEntity"
        request_error3.allow_renaming = False
        request_error3.uri = "/invalid/path/to/nonexistent.usd"  # Invalid URI

        result_error3 = await self._call_service_async(SpawnEntity, "/spawn_entity", request_error3)

        self.assertIsNotNone(result_error3)
        self.assertEqual(result_error3.result.result, SpawnEntity.Response.RESOURCE_PARSE_ERROR)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_delete_entity_service(self):
        """Test that DeleteEntity service correctly removes entities from the stage.

        This test spawns entities and then verifies they can be deleted successfully.
        """
        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import DeleteEntity, SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # First spawn an entity to delete
        spawn_request = SpawnEntity.Request()
        spawn_request.name = "EntityToDelete"
        spawn_request.allow_renaming = False
        spawn_request.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        spawn_result = await self._call_service_async(SpawnEntity, "/spawn_entity", spawn_request)

        self.assertIsNotNone(spawn_result)
        self.assertTrue(spawn_result.result.result == Result.RESULT_OK)

        # Verify the entity exists
        spawned_prim = stage.GetPrimAtPath("/EntityToDelete")
        self.assertTrue(spawned_prim.IsValid(), "EntityToDelete should exist before deletion")

        # Now delete the entity
        delete_request = DeleteEntity.Request()
        delete_request.entity = "/EntityToDelete"

        delete_result = await self._call_service_async(DeleteEntity, "/delete_entity", delete_request)

        self.assertIsNotNone(delete_result)
        self.assertTrue(delete_result.result.result == Result.RESULT_OK)

        # Verify the entity no longer exists
        deleted_prim = stage.GetPrimAtPath("/EntityToDelete")
        self.assertFalse(deleted_prim.IsValid(), "EntityToDelete should not exist after deletion")

        # Test deleting non-existent entity (should return error)
        delete_nonexistent_request = DeleteEntity.Request()
        delete_nonexistent_request.entity = "/NonExistentEntity"

        delete_nonexistent_result = await self._call_service_async(
            DeleteEntity, "/delete_entity", delete_nonexistent_request
        )
        self.assertIsNotNone(delete_nonexistent_result)
        self.assertTrue(delete_nonexistent_result.result.result == Result.RESULT_NOT_FOUND)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_reset_simulation_service(self):
        """Test that ResetSimulation service correctly resets simulation with SCOPE_DEFAULT.

        This test verifies the service removes dynamically spawned entities but keeps original stage entities.
        Only SCOPE_DEFAULT is implemented, which includes SCOPE_SPAWNED behavior.
        """
        from isaacsim.core.experimental.prims import RigidPrim

        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import ResetSimulation, SpawnEntity

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # Verify original entities exist before spawning
        cube_prim_original = stage.GetPrimAtPath("/World/Objects/DynamicCube")
        cone_prim_original = stage.GetPrimAtPath("/World/Objects/StaticCone")
        self.assertTrue(cube_prim_original.IsValid(), "Original DynamicCube should exist")
        self.assertTrue(cone_prim_original.IsValid(), "Original StaticCone should exist")

        # Spawn an entity for testing reset behavior
        spawn_request = SpawnEntity.Request()
        spawn_request.name = "SpawnedForReset"
        spawn_request.allow_renaming = False
        spawn_request.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"

        spawn_result = await self._call_service_async(SpawnEntity, "/spawn_entity", spawn_request)

        self.assertTrue(spawn_result.result.result == Result.RESULT_OK)

        # Verify spawned entity exists
        spawned_prim = stage.GetPrimAtPath("/SpawnedForReset")
        self.assertTrue(spawned_prim.IsValid(), "SpawnedForReset should exist before reset")

        # Modify cube state to test state reset behavior
        cube_prim = RigidPrim("/World/Objects/DynamicCube", reset_xform_op_properties=True)
        test_position = np.array([5.0, 5.0, 5.0])
        test_velocity = np.array([2.0, 2.0, 2.0])
        cube_prim.set_world_poses(positions=test_position)
        cube_prim.set_velocities(linear_velocities=test_velocity)

        # Start simulation to accumulate time and state changes
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Test SCOPE_DEFAULT reset (only scope actually implemented)
        reset_request = ResetSimulation.Request()
        reset_request.scope = ResetSimulation.Request.SCOPE_DEFAULT  # or SCOPE_ALL - both treated the same

        reset_result = await self._call_service_async(ResetSimulation, "/reset_simulation", reset_request)

        self.assertIsNotNone(reset_result)
        self.assertTrue(reset_result.result.result == Result.RESULT_OK)

        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()

        # Verify spawned entity was removed (has simulationInterfacesSpawned attribute)
        spawned_prim_after_reset = stage.GetPrimAtPath("/SpawnedForReset")
        self.assertFalse(spawned_prim_after_reset.IsValid(), "SpawnedForReset should be removed after reset")

        # Verify original stage entities still exist (don't have simulationInterfacesSpawned attribute)
        cube_prim_after_reset = stage.GetPrimAtPath("/World/Objects/DynamicCube")
        cone_prim_after_reset = stage.GetPrimAtPath("/World/Objects/StaticCone")
        self.assertTrue(cube_prim_after_reset.IsValid(), "Original DynamicCube should still exist after reset")
        self.assertTrue(cone_prim_after_reset.IsValid(), "Original StaticCone should still exist after reset")

        # Verify cube position was reset (should be back to original position)
        cube_positions_after_reset, _ = cube_prim.get_world_poses()
        cube_position_np = cube_positions_after_reset.numpy()[0]

        # Should be back near original position before Simulation Start (at 5,5,5)
        self.assertAlmostEqual(cube_position_np[0], 5.0, delta=0.1)
        self.assertAlmostEqual(cube_position_np[1], 5.0, delta=0.1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_step_simulation_service(self):
        """Test that StepSimulation service correctly steps simulation for finite steps.

        This test verifies the service can step simulation while paused and return to paused state.
        """
        from isaacsim.core.experimental.prims import RigidPrim

        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on
        from simulation_interfaces.srv import StepSimulation

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Start with simulation paused
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation is paused
        self.assertFalse(self._timeline.is_playing())
        self.assertFalse(self._timeline.is_stopped())

        # Set up cube with initial velocity for stepping test
        cube_prim = RigidPrim("/World/Objects/DynamicCube", reset_xform_op_properties=True)

        initial_position = np.array([0.0, 0.0, 5.0])
        initial_velocity = np.array([0.0, 0.0, -1.0])  # Falling down

        cube_prim.set_world_poses(positions=initial_position)
        cube_prim.set_velocities(linear_velocities=initial_velocity)

        await omni.kit.app.get_app().next_update_async()

        # Get initial position
        initial_positions, _ = cube_prim.get_world_poses()
        initial_z = initial_positions.numpy()[0][2]

        # Test single step (1 frame)
        step_request = StepSimulation.Request()
        step_request.steps = 1

        single_step_result = await self._call_service_async(StepSimulation, "/step_simulation", step_request)

        self.assertIsNotNone(single_step_result)
        self.assertTrue(single_step_result.result.result == Result.RESULT_OK)
        self.assertIn("1 frames", single_step_result.result.error_message)  # Should mention 1 frame

        # Verify simulation is still paused after single step
        self.assertFalse(self._timeline.is_playing())
        self.assertFalse(self._timeline.is_stopped())

        # Verify cube moved after single step
        after_single_step_positions, _ = cube_prim.get_world_poses()
        after_single_step_z = after_single_step_positions.numpy()[0][2]

        self.assertLess(after_single_step_z, initial_z, "Cube should have moved after single step")

        # Test double step
        step_request = StepSimulation.Request()
        step_request.steps = 2

        step_result = await self._call_service_async(StepSimulation, "/step_simulation", step_request)

        self.assertIsNotNone(step_result)
        self.assertTrue(step_result.result.result == Result.RESULT_OK)

        # Verify simulation is still paused after stepping
        self.assertFalse(self._timeline.is_playing())
        self.assertFalse(self._timeline.is_stopped())

        # Verify cube moved further (should have fallen more after 2 additional steps)
        after_step_positions, _ = cube_prim.get_world_poses()
        after_step_z = after_step_positions.numpy()[0][2]

        self.assertLess(after_step_z, after_single_step_z, "Cube should have fallen further after double step")

        # Test multiple steps
        step_request.steps = 5

        step_result = await self._call_service_async(StepSimulation, "/step_simulation", step_request)

        self.assertIsNotNone(step_result)
        self.assertTrue(step_result.result.result == Result.RESULT_OK)

        # Verify cube fell further
        after_multi_step_positions, _ = cube_prim.get_world_poses()
        after_multi_step_z = after_multi_step_positions.numpy()[0][2]

        self.assertLess(after_multi_step_z, after_step_z, "Cube should have fallen further after multiple steps")

        # Test step simulation when not paused (should fail)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        step_request.steps = 1

        step_result_playing = await self._call_service_async(StepSimulation, "/step_simulation", step_request)

        self.assertIsNotNone(step_result_playing)
        self.assertTrue(step_result_playing.result.result == Result.RESULT_INCORRECT_STATE)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_simulate_steps_action(self):
        """Test that SimulateSteps action correctly steps simulation with feedback.

        This test verifies the action can step simulation while providing progress feedback.
        """
        from isaacsim.core.experimental.prims import RigidPrim
        from simulation_interfaces.action import SimulateSteps

        # fmt: off
        from simulation_interfaces.msg import Result

        # fmt: on

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Start with simulation paused
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation is paused
        self.assertFalse(self._timeline.is_playing())
        self.assertFalse(self._timeline.is_stopped())

        # Set up cube for action testing
        cube_prim = RigidPrim("/World/Objects/DynamicCube", reset_xform_op_properties=True)

        initial_position = np.array([0.0, 0.0, 5.0])
        initial_velocity = np.array([0.0, 0.0, -1.0])  # Falling down

        cube_prim.set_world_poses(positions=initial_position)
        cube_prim.set_velocities(linear_velocities=initial_velocity)

        await omni.kit.app.get_app().next_update_async()

        # Get initial position
        initial_positions, _ = cube_prim.get_world_poses()
        initial_z = initial_positions.numpy()[0][2]

        # Test SimulateSteps action with feedback
        goal = SimulateSteps.Goal()
        goal.steps = 5

        action_result = await self._call_action_async(SimulateSteps, "/simulate_steps", goal)

        self.assertIsNotNone(action_result)
        self.assertTrue(action_result.result.result.result == Result.RESULT_OK)

        # Verify simulation is still paused after action completion
        self.assertFalse(self._timeline.is_playing())
        self.assertFalse(self._timeline.is_stopped())

        # Verify cube moved due to simulation steps
        after_action_positions, _ = cube_prim.get_world_poses()
        after_action_z = after_action_positions.numpy()[0][2]

        self.assertLess(after_action_z, initial_z, "Cube should have fallen after SimulateSteps action")

        # Test action when simulation is already playing (should fail)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        goal.steps = 3

        action_result_playing = await self._call_action_async(SimulateSteps, "/simulate_steps", goal)

        self.assertIsNotNone(action_result_playing)
        self.assertTrue(action_result_playing.result.result.result == Result.RESULT_INCORRECT_STATE)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_load_world_service(self):
        """Test that LoadWorld service correctly loads USD world files.

        This test verifies the service can load valid USD files, handles invalid paths,
        and validates unsupported formats.
        """
        # fmt: off
        from simulation_interfaces.msg import Result
        from simulation_interfaces.srv import LoadWorld

        # fmt: on

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        # Test loading a valid USD world file
        request = LoadWorld.Request()
        request.uri = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"

        result = await self._call_service_async(LoadWorld, "/load_world", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result.world)
        self.assertEqual(result.world.world_resource.uri, request.uri)
        self.assertEqual(result.world.name, "default_environment")

        # Verify the world was actually loaded
        stage = omni.usd.get_context().get_stage()
        self.assertIsNotNone(stage, "Stage should be loaded after LoadWorld")

        root_layer = stage.GetRootLayer()

        stage_uri = root_layer.identifier
        self.assertEqual(result.world.world_resource.uri, stage_uri)

        # Test loading with direct URI (without asset root path prefix)
        request_no_prefix = LoadWorld.Request()
        request_no_prefix.uri = "/Isaac/Environments/Grid/default_environment.usd"

        result_no_prefix = await self._call_service_async(LoadWorld, "/load_world", request_no_prefix)

        self.assertIsNotNone(result_no_prefix)
        self.assertTrue(result_no_prefix.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result_no_prefix.world)
        self.assertEqual(result_no_prefix.world.name, "default_environment")
        self.assertEqual(result_no_prefix.world.world_resource.uri, assets_root_path + request_no_prefix.uri)

        # Verify the world was actually loaded
        stage = omni.usd.get_context().get_stage()
        self.assertIsNotNone(stage, "Stage should be loaded after LoadWorld")

        root_layer = stage.GetRootLayer()

        stage_uri = root_layer.identifier
        self.assertEqual(result_no_prefix.world.world_resource.uri, stage_uri)

        # Test loading invalid file path (should fail)
        request_invalid = LoadWorld.Request()
        request_invalid.uri = "/invalid/path/to/nonexistent.usd"

        result_invalid = await self._call_service_async(LoadWorld, "/load_world", request_invalid)

        self.assertIsNotNone(result_invalid)
        self.assertEqual(result_invalid.result.result, LoadWorld.Response.RESOURCE_PARSE_ERROR)

        # Test unsupported format (should fail)
        request_unsupported = LoadWorld.Request()
        request_unsupported.uri = "/some/path/file.obj"  # Not a USD format

        result_unsupported = await self._call_service_async(LoadWorld, "/load_world", request_unsupported)

        self.assertIsNotNone(result_unsupported)
        self.assertEqual(result_unsupported.result.result, LoadWorld.Response.UNSUPPORTED_FORMAT)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_current_world_service(self):
        """Test that GetCurrentWorld service returns correct world information.

        This test verifies the service returns appropriate responses for loaded worlds,
        in-memory stages, and when no world is loaded.
        """
        # fmt: off
        from simulation_interfaces.msg import Result
        from simulation_interfaces.srv import GetCurrentWorld, LoadWorld

        # fmt: on
        # Test with in-memory world (created test stage)
        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        request = GetCurrentWorld.Request()
        result = await self._call_service_async(GetCurrentWorld, "/get_current_world", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result.world)
        # In-memory world should have empty URI and specific name
        self.assertEqual(result.world.world_resource.uri, "")
        self.assertEqual(result.world.name, "untitled_world")

        # Test with loaded world file
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        # Load a world first
        load_request = LoadWorld.Request()
        load_request.uri = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"

        load_result = await self._call_service_async(LoadWorld, "/load_world", load_request)
        self.assertIsNotNone(load_result)
        self.assertTrue(load_result.result.result == Result.RESULT_OK)

        # Now test GetCurrentWorld with loaded world
        result_loaded = await self._call_service_async(GetCurrentWorld, "/get_current_world", request)

        self.assertIsNotNone(result_loaded)
        self.assertTrue(result_loaded.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result_loaded.world)
        # Should have the loaded URI and extracted name
        self.assertEqual(result_loaded.world.world_resource.uri, load_request.uri)
        self.assertEqual(result_loaded.world.name, "default_environment")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_unload_world_service(self):
        """Test that UnloadWorld service correctly unloads worlds and creates empty stage.

        This test verifies the service can unload a loaded world, creating an anonymous
        in-memory stage, and handles the case when no world is loaded.
        """
        from simulation_interfaces.msg import Result
        from simulation_interfaces.srv import LoadWorld, UnloadWorld

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Get the assets root path
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        # First load a world so we have something to unload
        load_request = LoadWorld.Request()
        load_request.uri = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"

        load_result = await self._call_service_async(LoadWorld, "/load_world", load_request)

        self.assertIsNotNone(load_result)
        self.assertTrue(load_result.result.result == Result.RESULT_OK)

        # Verify the world was loaded with a real file URI
        stage = omni.usd.get_context().get_stage()
        self.assertIsNotNone(stage, "Stage should be loaded after LoadWorld")

        root_layer = stage.GetRootLayer()
        stage_uri_before = root_layer.identifier
        self.assertFalse(stage_uri_before.startswith("anon:"), "Loaded stage should not be anonymous")
        self.assertEqual(stage_uri_before, load_request.uri, "Stage URI should match loaded file")

        # Now test UnloadWorld service
        unload_request = UnloadWorld.Request()
        unload_result = await self._call_service_async(UnloadWorld, "/unload_world", unload_request)

        self.assertIsNotNone(unload_result)
        self.assertTrue(unload_result.result.result == Result.RESULT_OK)

        # Verify the stage is now anonymous (in-memory)
        stage_after = omni.usd.get_context().get_stage()
        self.assertIsNotNone(stage_after, "Stage should still exist after unload")

        root_layer_after = stage_after.GetRootLayer()
        stage_uri_after = root_layer_after.identifier
        self.assertTrue(
            stage_uri_after.startswith("anon:"), f"Unloaded stage should be anonymous. Got: {stage_uri_after}"
        )

        # Test unloading when no specific world is loaded (should still work)
        unload_result2 = await self._call_service_async(UnloadWorld, "/unload_world", unload_request)

        self.assertIsNotNone(unload_result2)
        self.assertTrue(unload_result2.result.result == Result.RESULT_OK)

        # Verify stage is still anonymous
        stage_final = omni.usd.get_context().get_stage()
        root_layer_final = stage_final.GetRootLayer()
        stage_uri_final = root_layer_final.identifier
        self.assertTrue(
            stage_uri_final.startswith("anon:"),
            f"Stage should remain anonymous after second unload. Got: {stage_uri_final}",
        )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_available_worlds_service(self):
        """Test that GetAvailableWorlds service discovers and filters world files correctly.

        This test verifies basic world discovery, tag filtering with ANY/ALL modes,
        additional sources, and error handling.
        """
        # fmt: off
        from simulation_interfaces.msg import Result, TagsFilter
        from simulation_interfaces.srv import GetAvailableWorlds

        # fmt: on

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # Test basic world discovery without filters
        request = GetAvailableWorlds.Request()

        result = await self._call_service_async(GetAvailableWorlds, "/get_available_worlds", request)

        self.assertIsNotNone(result)
        self.assertTrue(result.result.result == Result.RESULT_OK)
        self.assertIsNotNone(result.worlds)
        self.assertGreater(len(result.worlds), 0, "Should find at least some world files")

        # Verify each world has required fields
        for world in result.worlds:
            self.assertIsNotNone(world.world_resource.uri)
            self.assertIsNotNone(world.name)
            self.assertTrue(world.name != "", "World name should not be empty")

        # Test tag filtering with ANY mode (default)
        request_any = GetAvailableWorlds.Request()
        request_any.filter = TagsFilter()
        request_any.filter.tags = ["warehouse", "simple"]  # Should find files containing either term
        request_any.filter.filter_mode = TagsFilter.FILTER_MODE_ANY

        result_any = await self._call_service_async(GetAvailableWorlds, "/get_available_worlds", request_any)

        self.assertIsNotNone(result_any)
        self.assertTrue(result_any.result.result == Result.RESULT_OK)

        # Should find at least some files (warehouse.usd should match)
        if len(result_any.worlds) > 0:
            # Verify EVERY result contains at least one of our filter terms (ANY mode)
            for world in result_any.worlds:
                uri_lower = world.world_resource.uri.lower()
                has_warehouse = "warehouse" in uri_lower
                has_simple = "simple" in uri_lower
                self.assertTrue(
                    has_warehouse or has_simple,
                    f"File '{world.world_resource.uri}' should match at least one filter pattern ['warehouse', 'simple'] in ANY mode",
                )

        # Test tag filtering with ALL mode
        request_all = GetAvailableWorlds.Request()
        request_all.filter = TagsFilter()
        request_all.filter.tags = ["warehouse", "simple"]  # Should find files containing both terms
        request_all.filter.filter_mode = TagsFilter.FILTER_MODE_ALL

        result_all = await self._call_service_async(GetAvailableWorlds, "/get_available_worlds", request_all)

        self.assertIsNotNone(result_all)
        self.assertTrue(result_all.result.result == Result.RESULT_OK)
        # ALL mode should generally return fewer or equal results than ANY mode
        self.assertLessEqual(len(result_all.worlds), len(result_any.worlds))

        # Verify EVERY result contains ALL of our filter terms (ALL mode)
        if len(result_all.worlds) > 0:
            for world in result_all.worlds:
                uri_lower = world.world_resource.uri.lower()
                has_warehouse = "warehouse" in uri_lower
                has_simple = "simple" in uri_lower
                self.assertTrue(
                    has_warehouse and has_simple,
                    f"File '{world.world_resource.uri}' should match ALL filter patterns ['warehouse', 'simple'] in ALL mode",
                )

        # Test with additional sources
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path:
            request_additional = GetAvailableWorlds.Request()
            request_additional.additional_sources = [assets_root_path + "/Isaac/Sensors/SICK/picoScan150"]

            result_additional = await self._call_service_async(
                GetAvailableWorlds, "/get_available_worlds", request_additional
            )

            self.assertIsNotNone(result_additional)
            self.assertTrue(result_additional.result.result == Result.RESULT_OK)
            self.assertGreater(len(result_additional.worlds), 0, "Should find worlds in additional sources")

        # Test continue_on_error flag with invalid additional source
        request_error = GetAvailableWorlds.Request()
        request_error.additional_sources = ["/invalid/nonexistent/path"]
        request_error.continue_on_error = True

        result_error = await self._call_service_async(GetAvailableWorlds, "/get_available_worlds", request_error)

        self.assertIsNotNone(result_error)
        # Should succeed despite invalid path due to continue_on_error=True
        self.assertTrue(result_error.result.result == Result.RESULT_OK)

        # Test offline_only flag (should not affect local files)
        request_offline = GetAvailableWorlds.Request()
        request_offline.offline_only = True

        result_offline = await self._call_service_async(GetAvailableWorlds, "/get_available_worlds", request_offline)

        self.assertIsNotNone(result_offline)
        self.assertTrue(result_offline.result.result == Result.RESULT_OK)

        # Verify no remote URLs in offline-only results
        remote_protocols = ["omniverse://", "https://", "http://", "ftp://", "sftp://"]
        for world in result_offline.worlds:
            is_remote = any(world.world_resource.uri.startswith(protocol) for protocol in remote_protocols)
            self.assertFalse(
                is_remote, f"Offline-only should not return remote URLs. Found: {world.world_resource.uri}"
            )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_spawn_entities_batch_service(self):
        """Test SpawnEntities batch spawn service.

        Verifies that multiple entities can be spawned in a single call, that per-entity
        results are returned, and that partial failures are reported correctly.
        """
        from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
        from isaacsim.core.experimental.prims import XformPrim

        # fmt: off
        from simulation_interfaces.msg import Resource, Result
        from simulation_interfaces.msg import SpawnEntity as SpawnEntityMsg

        # fmt: on
        from simulation_interfaces.srv import SpawnEntities

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        stage = omni.usd.get_context().get_stage()

        # --- Test 1: batch spawn two valid entities (one from URI, one empty xform) ---
        request = SpawnEntities.Request()

        spawn_req1 = SpawnEntityMsg()
        spawn_req1.name = "BatchRobot"
        spawn_req1.allow_renaming = False
        spawn_req1.entity_resource = Resource()
        spawn_req1.entity_resource.uri = assets_root_path + "/Isaac/Samples/ROS2/Robots/limo_ROS.usd"
        spawn_req1.initial_pose = PoseStamped()
        spawn_req1.initial_pose.pose = Pose(
            position=Point(x=1.0, y=2.0, z=0.0),
            orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        )

        spawn_req2 = SpawnEntityMsg()
        spawn_req2.name = "BatchXform"
        spawn_req2.allow_renaming = False
        spawn_req2.entity_resource = Resource()
        spawn_req2.entity_resource.uri = ""
        spawn_req2.initial_pose = PoseStamped()

        request.spawn_requests = [spawn_req1, spawn_req2]

        result = await self._call_service_async(SpawnEntities, "/spawn_entities", request)

        self.assertIsNotNone(result)
        self.assertEqual(result.result.result, Result.RESULT_OK)
        self.assertEqual(len(result.results), 2)

        self.assertEqual(result.results[0].result.result, Result.RESULT_OK)
        self.assertEqual(result.results[0].entity_name, "/BatchRobot")
        self.assertEqual(result.results[1].result.result, Result.RESULT_OK)
        self.assertEqual(result.results[1].entity_name, "/BatchXform")

        self.assertTrue(stage.GetPrimAtPath("/BatchRobot").IsValid(), "BatchRobot should exist in the stage")
        self.assertTrue(stage.GetPrimAtPath("/BatchXform").IsValid(), "BatchXform should exist in the stage")

        # Verify position of first entity
        prim1 = XformPrim("/BatchRobot", reset_xform_op_properties=True)
        positions, _ = prim1.get_world_poses()
        pos_np = positions.numpy()[0]
        self.assertAlmostEqual(pos_np[0], 1.0, delta=0.01)
        self.assertAlmostEqual(pos_np[1], 2.0, delta=0.01)
        self.assertAlmostEqual(pos_np[2], 0.0, delta=0.01)

        # --- Test 2: mixed success/failure (one valid xform, one invalid URI) ---
        request2 = SpawnEntities.Request()

        spawn_ok = SpawnEntityMsg()
        spawn_ok.name = "BatchValid"
        spawn_ok.allow_renaming = False
        spawn_ok.entity_resource = Resource()
        spawn_ok.entity_resource.uri = ""
        spawn_ok.initial_pose = PoseStamped()

        spawn_bad = SpawnEntityMsg()
        spawn_bad.name = "BatchInvalid"
        spawn_bad.allow_renaming = False
        spawn_bad.entity_resource = Resource()
        spawn_bad.entity_resource.uri = "/invalid/nonexistent.usd"
        spawn_bad.initial_pose = PoseStamped()

        request2.spawn_requests = [spawn_ok, spawn_bad]

        result2 = await self._call_service_async(SpawnEntities, "/spawn_entities", request2)

        self.assertIsNotNone(result2)
        self.assertEqual(result2.result.result, SpawnEntities.Response.ENTITIES_SPAWN_FAILED)
        self.assertEqual(len(result2.results), 2)

        self.assertEqual(result2.results[0].result.result, Result.RESULT_OK)
        self.assertEqual(result2.results[0].entity_name, "/BatchValid")
        self.assertNotEqual(result2.results[1].result.result, Result.RESULT_OK)

        self.assertTrue(stage.GetPrimAtPath("/BatchValid").IsValid(), "BatchValid should exist despite partial failure")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_entity_bounds_service(self):
        """Test GetEntityBounds service returns correct bounding boxes.

        Verifies TYPE_BOX axis-aligned bounds for known geometry (cube and cone),
        and that querying a nonexistent entity returns RESULT_NOT_FOUND.
        """
        # fmt: off
        from simulation_interfaces.msg import Bounds, Result

        # fmt: on
        from simulation_interfaces.srv import GetEntityBounds

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # --- Test bounds for DynamicCube (unit cube at (0, 0, 2)) ---
        request = GetEntityBounds.Request()
        request.entity = "/World/Objects/DynamicCube"

        result = await self._call_service_async(GetEntityBounds, "/get_entity_bounds", request)

        self.assertIsNotNone(result)
        self.assertEqual(result.result.result, Result.RESULT_OK)
        self.assertEqual(result.bounds.type, Bounds.TYPE_BOX)
        self.assertEqual(len(result.bounds.points), 2)

        min_pt = result.bounds.points[0]
        max_pt = result.bounds.points[1]

        # Unit cube (size=1.0) translated to (0, 0, 2) should span roughly (-0.5,-0.5,1.5) to (0.5,0.5,2.5)
        self.assertAlmostEqual(min_pt.x, -0.5, delta=0.01)
        self.assertAlmostEqual(min_pt.y, -0.5, delta=0.01)
        self.assertAlmostEqual(min_pt.z, 1.5, delta=0.01)
        self.assertAlmostEqual(max_pt.x, 0.5, delta=0.01)
        self.assertAlmostEqual(max_pt.y, 0.5, delta=0.01)
        self.assertAlmostEqual(max_pt.z, 2.5, delta=0.01)

        # --- Test bounds for StaticCone (height=2, radius=0.5 at (2, 0, 1)) ---
        request.entity = "/World/Objects/StaticCone"

        result_cone = await self._call_service_async(GetEntityBounds, "/get_entity_bounds", request)

        self.assertIsNotNone(result_cone)
        self.assertEqual(result_cone.result.result, Result.RESULT_OK)
        self.assertEqual(result_cone.bounds.type, Bounds.TYPE_BOX)
        self.assertEqual(len(result_cone.bounds.points), 2)

        cone_min = result_cone.bounds.points[0]
        cone_max = result_cone.bounds.points[1]

        # Cone with height=2, radius=0.5 at (2, 0, 1): spans roughly (1.5,-0.5,0) to (2.5,0.5,2)
        self.assertAlmostEqual(cone_min.x, 1.5, delta=0.01)
        self.assertAlmostEqual(cone_min.y, -0.5, delta=0.01)
        self.assertAlmostEqual(cone_min.z, 0.0, delta=0.01)
        self.assertAlmostEqual(cone_max.x, 2.5, delta=0.01)
        self.assertAlmostEqual(cone_max.y, 0.5, delta=0.01)
        self.assertAlmostEqual(cone_max.z, 2.0, delta=0.01)

        # --- Test nonexistent entity ---
        request.entity = "/World/NonExistentPrim"

        result_missing = await self._call_service_async(GetEntityBounds, "/get_entity_bounds", request)

        self.assertIsNotNone(result_missing)
        self.assertEqual(result_missing.result.result, Result.RESULT_NOT_FOUND)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_spawnables_service(self):
        """Test GetSpawnables service discovers available USD assets.

        Verifies that default sources return spawnables, that each spawnable has
        required fields, and that additional custom sources are searched.
        """
        # fmt: off
        from simulation_interfaces.msg import Bounds, Result

        # fmt: on
        from simulation_interfaces.srv import GetSpawnables

        self.create_test_stage()
        await omni.kit.app.get_app().next_update_async()

        # --- Test default sources (Robots + Props) ---
        request = GetSpawnables.Request()

        result = await self._call_service_async(GetSpawnables, "/get_spawnables", request)

        self.assertIsNotNone(result)
        self.assertEqual(result.result.result, Result.RESULT_OK)
        self.assertGreater(len(result.spawnables), 0, "Should find at least some spawnables")

        for spawnable in result.spawnables:
            self.assertTrue(spawnable.uri != "", "Spawnable should have a non-empty URI")
            self.assertTrue(spawnable.description != "", "Spawnable should have a non-empty description")
            self.assertEqual(spawnable.spawn_bounds.type, Bounds.TYPE_EMPTY)
            self.assertNotIn(".thumbs", spawnable.uri, "Thumbnail assets should be excluded from spawnables")

        # --- Test with additional sources ---
        assets_root_path = await get_assets_root_path_async()
        self.assertIsNotNone(assets_root_path, "Could not find Isaac Sim assets folder")

        request_with_sources = GetSpawnables.Request()
        request_with_sources.sources = [assets_root_path + "/Isaac/Samples/ROS2/Robots"]

        result_with_sources = await self._call_service_async(GetSpawnables, "/get_spawnables", request_with_sources)

        self.assertIsNotNone(result_with_sources)
        self.assertEqual(result_with_sources.result.result, Result.RESULT_OK)
        self.assertGreater(len(result_with_sources.spawnables), 0, "Should find spawnables from additional sources")

        # Verify at least one spawnable is a USD file
        has_usd = any(
            spawnable.uri.endswith((".usd", ".usda", ".usdc", ".usdz")) for spawnable in result_with_sources.spawnables
        )
        self.assertTrue(has_usd, "Should find at least one USD spawnable")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
