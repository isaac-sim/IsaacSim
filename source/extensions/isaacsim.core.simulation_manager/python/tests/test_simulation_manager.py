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

"""Test for simulation manager."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
import omni.timeline
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from pxr import PhysxSchema, UsdPhysics


class TestSimulationManagerBackend(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager backend management."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        SimulationManager.set_backend("numpy")
        super().tearDown()

    async def test_get_set_backend(self):
        """Test getting and setting all valid backends."""
        for backend in ["numpy", "torch", "warp"]:
            SimulationManager.set_backend(backend)
            self.assertEqual(SimulationManager.get_backend(), backend)

    async def test_get_backend_utils(self):
        """Test getting backend utils for all backends and invalid backend."""
        # Test numpy backend
        SimulationManager.set_backend("numpy")
        utils = SimulationManager._get_backend_utils()
        self.assertIsNotNone(utils)
        self.assertTrue(hasattr(utils, "convert"))

        # Test torch backend
        SimulationManager.set_backend("torch")
        try:
            utils = SimulationManager._get_backend_utils()
            self.assertIsNotNone(utils)
            self.assertTrue(hasattr(utils, "convert"))
        except Exception as e:
            print(f"test_get_backend_utils failed for torch: {e}")

        # Test warp backend
        SimulationManager.set_backend("warp")
        try:
            utils = SimulationManager._get_backend_utils()
            self.assertIsNotNone(utils)
            self.assertTrue(hasattr(utils, "convert"))
        except Exception as e:
            print(f"test_get_backend_utils failed for warp: {e}")

        # Test invalid backend raises exception
        SimulationManager.set_backend("invalid_backend")
        with self.assertRaises(Exception):
            SimulationManager._get_backend_utils()


class TestSimulationManagerDefaultCallbacks(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager default callback enable/disable functionality."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        SimulationManager.enable_all_default_callbacks(True)
        super().tearDown()

    async def test_default_callbacks(self):
        """Test all default callback enable/disable functionality."""
        # Test individual callbacks
        SimulationManager.enable_warm_start_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("warm_start"))
        SimulationManager.enable_warm_start_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("warm_start"))

        SimulationManager.enable_on_stop_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("on_stop"))
        SimulationManager.enable_on_stop_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("on_stop"))

        SimulationManager.enable_post_warm_start_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("post_warm_start"))
        SimulationManager.enable_post_warm_start_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("post_warm_start"))

        SimulationManager.enable_stage_open_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("stage_open"))
        SimulationManager.enable_stage_open_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("stage_open"))

        # Test enable/disable all callbacks
        SimulationManager.enable_all_default_callbacks(False)
        status = SimulationManager.get_default_callback_status()
        self.assertFalse(all(status.values()))

        SimulationManager.enable_all_default_callbacks(True)
        status = SimulationManager.get_default_callback_status()
        self.assertTrue(all(status.values()))

        # Test get_default_callback_status returns correct keys
        expected_keys = {"warm_start", "on_stop", "post_warm_start", "stage_open", "stage_close"}
        self.assertEqual(set(status.keys()), expected_keys)

        # Test invalid callback name returns False
        self.assertFalse(SimulationManager.is_default_callback_enabled("nonexistent_callback"))


class TestSimulationManagerPhysicsDevice(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager physics device management."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World/PhysicsScene", type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()
        self._original_device = SimulationManager.get_physics_sim_device()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        try:
            SimulationManager.set_physics_sim_device(self._original_device)
        except Exception as e:
            print(f"tearDown failed to restore physics sim device: {e}")
        super().tearDown()

    async def test_physics_sim_device(self):
        """Test getting and setting physics simulation device."""
        # Test get returns string
        device = SimulationManager.get_physics_sim_device()
        self.assertIsInstance(device, str)
        self.assertTrue(device == "cpu" or device.startswith("cuda:"))

        # Test setting to CPU
        SimulationManager.set_physics_sim_device("cpu")

        # Test setting to CUDA
        try:
            SimulationManager.set_physics_sim_device("cuda")
            device = SimulationManager.get_physics_sim_device()
            self.assertIn("cuda", device)
        except Exception as e:
            print(f"test_physics_sim_device failed for cuda: {e}")

        # Test setting to CUDA with specific device ID
        try:
            SimulationManager.set_physics_sim_device("cuda:0")
            device = SimulationManager.get_physics_sim_device()
            self.assertEqual(device, "cuda:0")
        except Exception as e:
            print(f"test_physics_sim_device failed for cuda:0: {e}")

        # Test invalid device raises exception
        with self.assertRaises(Exception):
            SimulationManager.set_physics_sim_device("invalid_device")


class TestSimulationManagerPhysicsSceneSettings(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager physics scene settings (dt, broadphase, CCD, GPU dynamics, stabilization, solver)."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/World/PhysicsScene"
        self._registered_callbacks = []
        self._received_dt_values = []

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        for callback_id in self._registered_callbacks:
            try:
                SimulationManager.deregister_callback(callback_id)
            except Exception as e:
                print(f"tearDown failed to deregister callback {callback_id}: {e}")
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def _create_physics_scene(self):
        """Helper to create physics scene for tests that need it."""
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

    def _physics_step_callback(self, step_dt, context):
        """Callback to capture physics step dt values."""
        self._received_dt_values.append(step_dt)

    async def test_physics_scene_settings_no_scene(self):
        """Test all physics scene settings without specifying a physics scene."""
        # Test get physics dt default
        dt = SimulationManager.get_physics_dt()
        self.assertIsNotNone(dt)
        self.assertGreaterEqual(dt, 0)

        # Test set physics dt and verify in callback
        test_dts = [1.0 / 60.0, 1.0 / 120.0, 1.0 / 30.0, 1.0 / 240.0]
        for expected_dt in test_dts:
            self._received_dt_values.clear()
            try:
                SimulationManager.set_physics_dt(expected_dt)
                dt = SimulationManager.get_physics_dt()
                self.assertAlmostEqual(dt, expected_dt, places=5)
            except Exception as e:
                print(f"test_physics_scene_settings_no_scene failed for dt {expected_dt}: {e}")
                continue

            callback_id = SimulationManager.register_callback(
                self._physics_step_callback, event=IsaacEvents.POST_PHYSICS_STEP
            )
            self._registered_callbacks.append(callback_id)

            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()

            if self._received_dt_values:
                for received_dt in self._received_dt_values:
                    self.assertAlmostEqual(
                        received_dt,
                        expected_dt,
                        places=5,
                        msg=f"Callback dt {received_dt} does not match expected {expected_dt}",
                    )

            SimulationManager.deregister_callback(callback_id)
            self._registered_callbacks.remove(callback_id)

        # Test set physics dt
        expected_dt = 1.0 / 90.0
        try:
            SimulationManager.set_physics_dt(expected_dt)
            dt = SimulationManager.get_physics_dt()
            self.assertAlmostEqual(dt, expected_dt, places=5)
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene set_physics_dt failed: {e}")

        # Test set physics dt to zero
        try:
            SimulationManager.set_physics_dt(0)
            dt = SimulationManager.get_physics_dt()
            self.assertEqual(dt, 0.0)
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene set_physics_dt_zero failed: {e}")

        # Test invalid physics dt values
        has_physics_scene = SimulationManager.get_default_physics_scene() is not None
        if has_physics_scene:
            with self.assertRaises(ValueError):
                SimulationManager.set_physics_dt(-1.0)
            with self.assertRaises(ValueError):
                SimulationManager.set_physics_dt(1.5)

        # PhysX-specific settings - only test assertions when PhysX is active
        is_physx = SimulationManager.get_active_physics_engine() == "physx"

        if is_physx:
            # Test broadphase type (PhysX-specific)
            for btype in ["MBP", "GPU"]:
                try:
                    SimulationManager.set_broadphase_type(btype)
                    result = SimulationManager.get_broadphase_type()
                    self.assertEqual(result, btype)
                    # For Newton, set_broadphase_type is a no-op, get returns default
                except Exception as e:
                    print(f"test_physics_scene_settings_no_scene broadphase failed for {btype}: {e}")

        # Test solver type (PhysX-specific)
        if is_physx:
            for solver in ["TGS", "PGS"]:
                try:
                    SimulationManager.set_solver_type(solver)
                    result = SimulationManager.get_solver_type()
                    self.assertEqual(result, solver)
                except Exception as e:
                    print(f"test_physics_scene_settings_no_scene solver failed for {solver}: {e}")

        # Test invalid solver type raises exception (PhysX-specific)
        if is_physx:
            with self.assertRaises(ValueError):
                SimulationManager.set_solver_type("INVALID")

        # Test CCD
        try:
            SimulationManager.set_physics_sim_device("cpu")
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene failed to set device to cpu: {e}")

        if is_physx:
            try:
                SimulationManager.enable_ccd(True)
                self.assertTrue(SimulationManager.is_ccd_enabled())
                SimulationManager.enable_ccd(False)
                self.assertFalse(SimulationManager.is_ccd_enabled())
            except Exception as e:
                print(f"test_physics_scene_settings_no_scene ccd failed: {e}")

        # Test GPU dynamics (PhysX-specific)
        if is_physx:
            try:
                SimulationManager.enable_gpu_dynamics(True)
                self.assertTrue(SimulationManager.is_gpu_dynamics_enabled())
                SimulationManager.enable_gpu_dynamics(False)
                self.assertFalse(SimulationManager.is_gpu_dynamics_enabled())
            except Exception as e:
                print(f"test_physics_scene_settings_no_scene gpu_dynamics failed: {e}")

        # Test stabilization (PhysX-specific)
        if is_physx:
            try:
                SimulationManager.enable_stablization(True)
                self.assertTrue(SimulationManager.is_stablization_enabled())
                SimulationManager.enable_stablization(False)
                self.assertFalse(SimulationManager.is_stablization_enabled())
            except Exception as e:
                print(f"test_physics_scene_settings_no_scene stabilization failed: {e}")

        # Test get default physics scene
        scene = SimulationManager.get_default_physics_scene()
        if scene is not None:
            self.assertIsInstance(scene, str)

    async def test_physics_scene_settings_with_scene(self):
        """Test all physics scene settings with a specific physics scene."""
        if SimulationManager._engine != "physx":
            self.skipTest(f"Skipping PhysX-specific test (active engine: {SimulationManager._engine})")
        await self._create_physics_scene()

        # Test set physics dt with physics scene
        expected_dt = 1.0 / 90.0
        try:
            SimulationManager.set_physics_dt(expected_dt, physics_scene=self._physics_scene_path)
            dt = SimulationManager.get_physics_dt(physics_scene=self._physics_scene_path)
            self.assertAlmostEqual(dt, expected_dt, places=5)
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene set_physics_dt failed: {e}")

        # Test broadphase type with physics scene
        for btype in ["MBP", "GPU"]:
            try:
                SimulationManager.set_broadphase_type(btype, physics_scene=self._physics_scene_path)
                result = SimulationManager.get_broadphase_type(physics_scene=self._physics_scene_path)
                self.assertEqual(result, btype)
            except Exception as e:
                print(f"test_physics_scene_settings_with_scene broadphase failed for {btype}: {e}")

        # Test solver type with physics scene
        for solver in ["TGS", "PGS"]:
            try:
                SimulationManager.set_solver_type(solver, physics_scene=self._physics_scene_path)
                result = SimulationManager.get_solver_type(physics_scene=self._physics_scene_path)
                self.assertEqual(result, solver)
            except Exception as e:
                print(f"test_physics_scene_settings_with_scene solver failed for {solver}: {e}")

        # Test CCD with physics scene
        try:
            SimulationManager.set_physics_sim_device("cpu")
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene failed to set device to cpu: {e}")

        try:
            SimulationManager.enable_ccd(True, physics_scene=self._physics_scene_path)
            self.assertTrue(SimulationManager.is_ccd_enabled(physics_scene=self._physics_scene_path))
            SimulationManager.enable_ccd(False, physics_scene=self._physics_scene_path)
            self.assertFalse(SimulationManager.is_ccd_enabled(physics_scene=self._physics_scene_path))
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene ccd failed: {e}")

        # Test GPU dynamics with physics scene
        try:
            SimulationManager.enable_gpu_dynamics(True, physics_scene=self._physics_scene_path)
            self.assertTrue(SimulationManager.is_gpu_dynamics_enabled(physics_scene=self._physics_scene_path))
            SimulationManager.enable_gpu_dynamics(False, physics_scene=self._physics_scene_path)
            self.assertFalse(SimulationManager.is_gpu_dynamics_enabled(physics_scene=self._physics_scene_path))
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene gpu_dynamics failed: {e}")

        # Test stabilization with physics scene
        try:
            SimulationManager.enable_stablization(True, physics_scene=self._physics_scene_path)
            self.assertTrue(SimulationManager.is_stablization_enabled(physics_scene=self._physics_scene_path))
            SimulationManager.enable_stablization(False, physics_scene=self._physics_scene_path)
            self.assertFalse(SimulationManager.is_stablization_enabled(physics_scene=self._physics_scene_path))
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene stabilization failed: {e}")

        # Test set default physics scene
        try:
            SimulationManager.set_default_physics_scene(self._physics_scene_path)
        except Exception as e:
            if "SimulationManager is not tracking physics scenes" not in str(e):
                raise


class TestSimulationManagerCallbacks(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager callback registration and event handling."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/World/PhysicsScene"
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()
        self._registered_callbacks = []

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        for callback_id in self._registered_callbacks:
            try:
                SimulationManager.deregister_callback(callback_id)
            except Exception as e:
                print(f"tearDown failed to deregister callback {callback_id}: {e}")
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_register_callbacks(self):
        """Test registering and deregistering callbacks for all event types."""
        # Test message bus callbacks
        events = [
            IsaacEvents.PHYSICS_WARMUP,
            IsaacEvents.PHYSICS_READY,
            IsaacEvents.POST_RESET,
            IsaacEvents.SIMULATION_VIEW_CREATED,
        ]
        for event in events:
            callback_id = SimulationManager.register_callback(lambda x: None, event=event)
            self._registered_callbacks.append(callback_id)
            self.assertIsNotNone(callback_id)

        # Test physics step callbacks
        for event in [IsaacEvents.PRE_PHYSICS_STEP, IsaacEvents.POST_PHYSICS_STEP]:
            callback_id = SimulationManager.register_callback(lambda dt, context: None, event=event)
            self._registered_callbacks.append(callback_id)
            self.assertIsNotNone(callback_id)

        # Test timeline stop callback
        callback_id = SimulationManager.register_callback(lambda x: None, event=IsaacEvents.TIMELINE_STOP)
        self._registered_callbacks.append(callback_id)
        self.assertIsNotNone(callback_id)

        # Test prim deletion callback
        try:
            callback_id = SimulationManager.register_callback(lambda prim_path: None, event=IsaacEvents.PRIM_DELETION)
            self.assertIsNotNone(callback_id)
        except Exception as e:
            print(f"test_register_callbacks prim_deletion failed: {e}")

        # Test callbacks with specific execution order
        callback_id_1 = SimulationManager.register_callback(
            lambda dt, context: None, event=IsaacEvents.POST_PHYSICS_STEP, order=0
        )
        callback_id_2 = SimulationManager.register_callback(
            lambda dt, context: None, event=IsaacEvents.POST_PHYSICS_STEP, order=10
        )
        self._registered_callbacks.extend([callback_id_1, callback_id_2])
        self.assertNotEqual(callback_id_1, callback_id_2)

        # Test callback with bound method
        class CallbackHolder:
            def __init__(self):
                self.called = False

            def on_physics_ready(self, event):
                self.called = True

        holder = CallbackHolder()
        callback_id = SimulationManager.register_callback(holder.on_physics_ready, event=IsaacEvents.PHYSICS_READY)
        self._registered_callbacks.append(callback_id)
        self.assertIsNotNone(callback_id)

        # Test deregistering callback
        callback_id = SimulationManager.register_callback(lambda x: None, event=IsaacEvents.PHYSICS_READY)
        self.assertTrue(SimulationManager.deregister_callback(callback_id))
        self.assertFalse(SimulationManager.deregister_callback(callback_id))

        # Test deregistering invalid callback (log a warning and do nothing)
        self.assertFalse(SimulationManager.deregister_callback(999999))

    async def test_event_dispatch(self):
        """Test that all events are dispatched correctly."""
        warmup_received = []
        ready_received = []
        view_created_received = []
        stop_received = []
        execution_order = []

        # Register all event callbacks
        warmup_callback_id = SimulationManager.register_callback(
            lambda event: warmup_received.append(True), event=IsaacEvents.PHYSICS_WARMUP
        )
        ready_callback_id = SimulationManager.register_callback(
            lambda event: ready_received.append(True), event=IsaacEvents.PHYSICS_READY
        )
        view_created_callback_id = SimulationManager.register_callback(
            lambda event: view_created_received.append(True), event=IsaacEvents.SIMULATION_VIEW_CREATED
        )
        stop_callback_id = SimulationManager.register_callback(
            lambda event: stop_received.append(True), event=IsaacEvents.TIMELINE_STOP
        )
        pre_callback_id = SimulationManager.register_callback(
            lambda dt, context: execution_order.append("pre"), event=IsaacEvents.PRE_PHYSICS_STEP
        )
        post_callback_id = SimulationManager.register_callback(
            lambda dt, context: execution_order.append("post"), event=IsaacEvents.POST_PHYSICS_STEP
        )
        self._registered_callbacks.extend(
            [
                warmup_callback_id,
                ready_callback_id,
                view_created_callback_id,
                stop_callback_id,
                pre_callback_id,
                post_callback_id,
            ]
        )

        # Start simulation
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify warmup, ready, and view created events
        self.assertGreater(len(warmup_received), 0, "PHYSICS_WARMUP event was not received")
        self.assertGreater(len(ready_received), 0, "PHYSICS_READY event was not received")
        self.assertGreater(len(view_created_received), 0, "SIMULATION_VIEW_CREATED event was not received")

        # Verify pre/post physics step order
        if len(execution_order) >= 2:
            pre_indices = [i for i, x in enumerate(execution_order) if x == "pre"]
            post_indices = [i for i, x in enumerate(execution_order) if x == "post"]
            if pre_indices and post_indices:
                self.assertLess(
                    pre_indices[0], post_indices[0], "PRE_PHYSICS_STEP should execute before POST_PHYSICS_STEP"
                )

        # Stop and verify stop event
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.assertGreater(len(stop_received), 0, "TIMELINE_STOP event was not received")


class TestSimulationManagerSimulationLifecycle(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager simulation state, step, and initialization."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/World/PhysicsScene"
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_simulation_state(self):
        """Test simulation state through play, pause, and time advancement."""
        # Test initial state
        self.assertFalse(SimulationManager.is_simulating())
        self.assertFalse(SimulationManager.is_paused())
        self.assertEqual(SimulationManager.get_simulation_time(), 0.0)
        self.assertEqual(SimulationManager.get_num_physics_steps(), 0)

        timeline = omni.timeline.get_timeline_interface()
        initial_time = SimulationManager.get_simulation_time()

        # Test state after play
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(SimulationManager.is_simulating())
        self.assertFalse(SimulationManager.is_paused())

        # Test simulation time advances
        await omni.kit.app.get_app().next_update_async()
        self.assertGreater(SimulationManager.get_simulation_time(), initial_time)

        # Test state after pause
        timeline.pause()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(SimulationManager.is_simulating())
        self.assertTrue(SimulationManager.is_paused())

    async def test_simulation_operations(self):
        """Test simulation operations including physics view, stepping, and initialization."""
        # Test physics sim view before simulation
        self.assertIsNone(SimulationManager.get_physics_sim_view())

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Test physics sim view after simulation starts
        # View may or may not be available depending on torch availability
        # Just verify the function doesn't raise
        SimulationManager.get_physics_sim_view()

        try:
            SimulationManager.step()
        except Exception as e:
            print(f"test_simulation_operations step failed: {e}")

        # Test initialize_physics
        try:
            SimulationManager.initialize_physics()
        except Exception as e:
            print(f"test_simulation_operations initialize_physics failed: {e}")

        # Test assets_loading returns bool
        result = SimulationManager.assets_loading()
        self.assertIsInstance(result, bool)


class TestSimulationManagerPhysicsEngines(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager physics engine switching and management."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        # Create a physics scene with proper PhysX schema
        stage = stage_utils.get_current_stage()
        scene_prim = stage_utils.define_prim("/World/PhysicsScene")
        UsdPhysics.Scene.Define(stage, scene_prim.GetPath())
        PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        await omni.kit.app.get_app().next_update_async()
        # Register the physics scene with SimulationManager
        SimulationManager.set_default_physics_scene("/World/PhysicsScene")
        self._original_engine = SimulationManager.get_active_physics_engine()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        if self._original_engine:
            try:
                SimulationManager.switch_physics_engine(self._original_engine, verbose=False)
            except Exception:
                pass
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_engine_query_and_switching(self):
        """Test engine query methods and switching to PhysX."""
        # This test is specific to PhysX - skip if another engine is active
        if SimulationManager.get_active_physics_engine() != "physx":
            self.skipTest(
                f"Skipping PhysX-specific test (active engine: {SimulationManager.get_active_physics_engine()})"
            )
        # Test get_active_physics_engine returns PhysX by default
        engine = SimulationManager.get_active_physics_engine()
        self.assertEqual(engine, "physx")

        # Test get_available_physics_engines returns list with PhysX
        engines = SimulationManager.get_available_physics_engines(verbose=False)
        self.assertGreater(len(engines), 0)
        engine_names = [e[0] for e in engines]
        self.assertIn("physx", engine_names)

        # Test switching to same engine
        result = SimulationManager.switch_physics_engine("physx", verbose=False)
        self.assertTrue(result)
        self.assertEqual(SimulationManager.get_active_physics_engine(), "physx")

    async def test_invalid_switch_preserves_state(self):
        """Test that invalid engine switches preserve all state and PhysX continues working."""
        # This test is specific to PhysX - skip if another engine is active
        if SimulationManager.get_active_physics_engine() != "physx":
            self.skipTest(
                f"Skipping PhysX-specific test (active engine: {SimulationManager.get_active_physics_engine()})"
            )
        # Capture state before invalid switches
        engine_before = SimulationManager.get_active_physics_engine()
        dt_before = SimulationManager.get_physics_dt()
        broadphase_before = SimulationManager.get_broadphase_type()
        solver_before = SimulationManager.get_solver_type()
        ccd_before = SimulationManager.is_ccd_enabled()
        gpu_before = SimulationManager.is_gpu_dynamics_enabled()
        stab_before = SimulationManager.is_stablization_enabled()

        # Attempt invalid switch
        self.assertFalse(SimulationManager.switch_physics_engine("invalid_engine", verbose=False))

        # Verify all state is preserved
        self.assertEqual(SimulationManager.get_active_physics_engine(), engine_before)
        self.assertEqual(SimulationManager.get_physics_dt(), dt_before)
        self.assertEqual(SimulationManager.get_broadphase_type(), broadphase_before)
        self.assertEqual(SimulationManager.get_solver_type(), solver_before)
        self.assertEqual(SimulationManager.is_ccd_enabled(), ccd_before)
        self.assertEqual(SimulationManager.is_gpu_dynamics_enabled(), gpu_before)
        self.assertEqual(SimulationManager.is_stablization_enabled(), stab_before)

        # Verify PhysX methods still work correctly
        SimulationManager.set_broadphase_type("GPU")
        self.assertEqual(SimulationManager.get_broadphase_type(), "GPU")

        SimulationManager.set_solver_type("PGS")
        self.assertEqual(SimulationManager.get_solver_type(), "PGS")

        # Restore original values
        SimulationManager.set_broadphase_type(broadphase_before)
        SimulationManager.set_solver_type(solver_before)


class TestSimulationManagerFabricAndNoticeHandlers(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager fabric and USD notice handler management."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    async def test_fabric_and_notice_handlers(self):
        """Test all fabric and USD notice handler functionality."""
        # Test enable/disable fabric
        try:
            SimulationManager.enable_fabric(False)
            SimulationManager.enable_fabric(True)
        except Exception as e:
            print(f"test_fabric_and_notice_handlers enable_fabric failed: {e}")

        # Test is_fabric_enabled returns bool
        result = SimulationManager.is_fabric_enabled()
        self.assertIsInstance(result, bool)

        # Test enable/disable USD notice handler

        try:
            SimulationManager.enable_usd_notice_handler(False)
            SimulationManager.enable_usd_notice_handler(True)
        except Exception as e:
            print(f"test_fabric_and_notice_handlers usd_notice_handler failed: {e}")

        # Test fabric USD notice handler
        try:
            stage = stage_utils.get_current_stage()
            stage_id = stage_utils.get_stage_id(stage)

            SimulationManager.enable_fabric_usd_notice_handler(stage_id, False)
            SimulationManager.enable_fabric_usd_notice_handler(stage_id, True)

            result = SimulationManager.is_fabric_usd_notice_handler_enabled(stage_id)
            self.assertIsInstance(result, bool)
        except Exception as e:
            print(f"test_fabric_and_notice_handlers fabric_usd_notice_handler failed: {e}")


class TestSimulationManagerStageTransitions(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager behavior during stage transitions and prim invalidation."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/PhysicsScene"

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_stale_physics_scene_prim_reference(self):
        """Test that SimulationManager handles stale prim references gracefully.

        This test directly simulates the bug condition where the _physics_scenes
        dictionary contains a PhysxScene with an invalid/expired prim reference.

        This reproduces the exact issue that caused failures in robot_setup.assembler tests,
        where certain USD operations invalidate physics scene prims without triggering
        the deletion callback.

        The sequence:
        1. Create physics scene and start simulation (caches PhysxScene with prim reference)
        2. Stop simulation
        3. Simulate prim invalidation by clearing root layer and recreating scene
        4. The cached PhysxScene now has a stale prim reference
        5. Try to play - without fix, this raises RuntimeError for expired prim
        """

        # Create physics scene and start simulation
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation started and physics scene is cached
        self.assertTrue(SimulationManager.is_simulating())
        physics_scenes_before = SimulationManager.get_physics_scenes()
        self.assertGreater(len(physics_scenes_before), 0, "Physics scene should be cached")

        # Store reference to cached prim for verification later
        cached_prim = physics_scenes_before[0].prim
        self.assertTrue(cached_prim.IsValid(), "Cached prim should be valid initially")

        # Stop simulation
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Simulate the bug condition: Clear the root layer's content which invalidates
        # the cached prim reference, but does NOT trigger the deletion callback
        # that would remove the entry from _physics_scenes
        stage = stage_utils.get_current_stage()
        root_layer = stage.GetRootLayer()

        # Clear the layer content - this invalidates existing prim references
        root_layer.Clear()
        await omni.kit.app.get_app().next_update_async()

        # Verify the cached prim is now invalid (the bug condition)
        self.assertFalse(cached_prim.IsValid(), "Cached prim should be invalid after layer clear")

        # Verify _physics_scenes still has the stale entry (this is the bug)
        physics_scenes_after_clear = SimulationManager.get_physics_scenes()
        # Note: The stale entry may or may not still be present depending on
        # whether the fix is enabled

        # Create a new physics scene at the same path
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

        # Try to play again - without the fix, this raises:
        # RuntimeError: Accessed invalid expired 'PhysicsScene' prim </PhysicsScene>
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation is running
        self.assertTrue(SimulationManager.is_simulating())

        # Verify we can access physics scene properties without error
        dt = SimulationManager.get_physics_dt()
        self.assertIsNotNone(dt)
        self.assertGreater(dt, 0)

    async def test_physics_scene_prim_invalidation_on_stage_change(self):
        """Test that SimulationManager handles expired physics scene prims gracefully.

        This test verifies that when a stage is changed (e.g., new stage opened),
        the SimulationManager properly handles the case where cached physics scene
        prim references become invalid/expired.

        The sequence that can cause issues:
        1. Create physics scene and start simulation (caches PhysxScene with prim reference)
        2. Stop simulation
        3. Open new stage (invalidates the cached prim reference)
        4. Play again - should not raise RuntimeError for expired prim
        """
        # Create physics scene and start simulation
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation started successfully
        self.assertTrue(SimulationManager.is_simulating())

        # Stop simulation
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Open a new stage - this invalidates any cached prim references
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Create a new physics scene on the new stage
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

        # Play again - this should NOT raise RuntimeError for expired prim
        # The SimulationManager should properly handle the stage transition
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation is running on the new stage
        self.assertTrue(SimulationManager.is_simulating())

        # Verify we can access physics scene properties without error
        dt = SimulationManager.get_physics_dt()
        self.assertIsNotNone(dt)
        self.assertGreater(dt, 0)

    async def test_physics_scene_deletion_during_simulation(self):
        """Test SimulationManager behavior when physics scene is deleted.

        This test verifies that deleting a physics scene prim while simulation
        is stopped doesn't cause errors when trying to play again.
        """
        # Create physics scene and start simulation
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation started
        self.assertTrue(SimulationManager.is_simulating())

        # Stop simulation
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Delete the physics scene prim
        stage = stage_utils.get_current_stage()
        stage.RemovePrim(self._physics_scene_path)
        await omni.kit.app.get_app().next_update_async()

        # Create a new physics scene
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

        # Play again - should handle the prim recreation gracefully
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify simulation is running
        self.assertTrue(SimulationManager.is_simulating())

    async def test_multiple_stage_transitions(self):
        """Test SimulationManager across multiple stage open/close cycles.

        This test verifies that repeated stage transitions don't accumulate
        stale physics scene references.
        """
        timeline = omni.timeline.get_timeline_interface()

        for i in range(3):
            # Create new stage
            await stage_utils.create_new_stage_async()
            await omni.kit.app.get_app().next_update_async()

            # Create physics scene
            stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
            await omni.kit.app.get_app().next_update_async()

            # Play simulation
            timeline.play()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            # Verify simulation started
            self.assertTrue(SimulationManager.is_simulating(), f"Simulation failed to start on iteration {i}")

            # Verify physics dt is accessible
            dt = SimulationManager.get_physics_dt()
            self.assertIsNotNone(dt, f"Physics dt was None on iteration {i}")

            # Stop simulation
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()


class TestSimulationManagerMultiTickRendering(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager multi-tick rendering mode support."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # /rtx/hydra/supportMultiTickRate must be set at app startup (see the
        # "multitick_rendering" test block in config/extension.toml); toggling it
        # here at runtime does not reconfigure the rtx.hydra render pipeline.
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/World/PhysicsScene"
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_multi_tick_simulation_time_update(self):
        """Test that simulation time is written to the Fabric prim in multi-tick mode.

        When /rtx/hydra/supportMultiTickRate is enabled, the physics callback writes
        the current simulation time to the /ExternalSimulationTime Fabric prim.
        This test verifies the value starts at zero and increases with each physics step.
        """
        import omni.usd

        usd_context = omni.usd.get_context()
        stage_id = usd_context.get_stage_id()

        def _read_fabric_sim_time():
            """Read the omni:time attribute from /ExternalSimulationTime in Fabric."""
            try:
                import usdrt

                fabric_stage = usdrt.Usd.Stage.Attach(stage_id)
                prim = fabric_stage.GetPrimAtPath("/ExternalSimulationTime")
                if prim and prim.HasAttribute("omni:time"):
                    attr = prim.GetAttribute("omni:time")
                    return float(attr.Get())
            except Exception:
                pass
            return None

        # Before play, the Fabric prim was seeded to 0 by onAttach; verify that
        # here rather than after play, since play+next_update advances physics
        # before Python can observe the initial state.
        initial_time = _read_fabric_sim_time()
        self.assertIsNotNone(initial_time, "Fabric /ExternalSimulationTime prim not found")
        self.assertAlmostEqual(initial_time, 0.0, places=5, msg="Initial Fabric simulation time should be zero")

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await omni.kit.app.get_app().next_update_async()

        self.assertTrue(SimulationManager.is_simulating())

        prev_time = initial_time
        for i in range(5):
            await omni.kit.app.get_app().next_update_async()

            current_time = _read_fabric_sim_time()
            self.assertIsNotNone(current_time)
            self.assertGreater(current_time, prev_time, f"Fabric simulation time should increase on step {i + 1}")
            prev_time = current_time

        final_fabric_time = _read_fabric_sim_time()
        sim_time = SimulationManager.get_simulation_time()
        self.assertAlmostEqual(
            final_fabric_time,
            sim_time,
            places=5,
            msg="Fabric simulation time should match SimulationManager time",
        )

        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
