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

"""Tests for base benchmark services and recorder registration."""

import isaacsim.core.experimental.utils.app as app_utils
import omni
from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmarkAsync
from isaacsim.benchmark.services.datarecorders.interface import (
    InputContext,
    MeasurementData,
    MeasurementDataRecorder,
    MeasurementDataRecorderRegistry,
)
from isaacsim.benchmark.services.metrics.measurements import (
    BooleanMeasurement,
    DictMeasurement,
    ListMeasurement,
    SingleMeasurement,
)


class TestBaseIsaacBenchmarkAsync(BaseIsaacBenchmarkAsync):
    """Async tests for base benchmark behavior."""

    async def setUp(self) -> None:
        """Set up the async benchmark test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await super().setUp(backend_type="LocalLogMetrics")

    async def tearDown(self) -> None:
        """Tear down the async benchmark test."""
        await super().tearDown()

    async def test_base_isaac_benchmark(self) -> None:
        """Test basic benchmark flow with two phases."""
        self.benchmark_name = "test_base_isaac_benchmark"
        await self.set_phase("loading", False, True)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()
        app_utils.play()

        await self.set_phase("benchmark")
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()

    async def test_store_custom_measurement(self) -> None:
        """Test storing multiple custom measurements across phases."""
        self.benchmark_name = "test_custom_measurements"
        await self.set_phase("loading", False, True)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()
        app_utils.play()
        await omni.kit.app.get_app().next_update_async()

        # Store different types of measurements
        measurement = BooleanMeasurement(name="bool_measure", bvalue=True)
        await self.store_custom_measurement("phase_1", measurement)

        measurement = SingleMeasurement(name="single_measure_1", value=1.23, unit="ms")
        await self.store_custom_measurement("phase_1", measurement)

        measurement = DictMeasurement(name="dict_measure", value={"key": "value"})
        await self.store_custom_measurement("phase_2", measurement)

        measurement = SingleMeasurement(name="single_measure_2", value=4.56, unit="ms")
        await self.store_custom_measurement("phase_3", measurement)

        measurement = ListMeasurement(name="list_measure", value=[1, 2, 3])
        await self.store_custom_measurement("phase_4", measurement)
        await omni.kit.app.get_app().next_update_async()

    async def test_default_recorders_initialized(self) -> None:
        """Test that default recorders load correctly."""
        self.benchmark_name = "test_default_recorders"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Initialize with default recorders
        await super().setUp(backend_type="LocalLogMetrics")

        # Verify all default recorders are loaded
        recorder_names = [r.__class__.__name__ for r in self.recorders]
        expected_recorders = [
            "AppFrametimeRecorder",
            "PhysicsFrametimeRecorder",
            "RuntimeRecorder",
            "MemoryRecorder",
            "CPUContinuousRecorder",
            "HardwareSpecRecorder",
        ]
        for expected in expected_recorders:
            self.assertIn(expected, recorder_names, f"Expected recorder {expected} not found")

        await omni.kit.app.get_app().next_update_async()

    async def test_custom_recorders_selection(self) -> None:
        """Test selecting a custom subset of recorders."""
        self.benchmark_name = "test_custom_selection"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Initialize with only runtime and memory recorders
        custom_recorders = ["runtime", "memory"]
        await super().setUp(backend_type="LocalLogMetrics", recorders=custom_recorders)

        # Verify only selected recorders are loaded
        recorder_names = [r.__class__.__name__ for r in self.recorders]
        self.assertEqual(len(recorder_names), 2, "Should have exactly 2 recorders")
        self.assertIn("RuntimeRecorder", recorder_names)
        self.assertIn("MemoryRecorder", recorder_names)

        await omni.kit.app.get_app().next_update_async()

    async def test_gpu_frametime_recorder(self) -> None:
        """Test that GPU frametime recorder can be enabled."""
        self.benchmark_name = "test_gpu_frametime"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Add gpu_frametime to default recorders
        recorders_with_gpu = DEFAULT_RECORDERS + ["gpu_frametime"]
        await super().setUp(backend_type="LocalLogMetrics", recorders=recorders_with_gpu)

        # Verify GPU frametime recorder is loaded
        recorder_names = [r.__class__.__name__ for r in self.recorders]
        self.assertIn("GPUFrametimeRecorder", recorder_names)

        await omni.kit.app.get_app().next_update_async()

    async def test_register_and_use_custom_recorder(self) -> None:
        """Test registering and using a custom recorder via the plugin system."""

        # Define a simple custom recorder
        @MeasurementDataRecorderRegistry.register("test_custom")
        class TestCustomRecorder(MeasurementDataRecorder):
            """Custom recorder for testing registry integration.

            Args:
                context: Input context for the recorder. Defaults to None.
            """

            def __init__(self, context: InputContext | None = None) -> None:
                self.context = context
                self._value = 0

            def start_collecting(self) -> None:
                """Start collecting custom data."""
                self._value = 42

            def stop_collecting(self) -> None:
                """Stop collecting custom data."""

            def get_data(self) -> MeasurementData:
                """Return the collected custom measurement.

                Returns:
                    Collected measurement data.
                """
                return MeasurementData(
                    measurements=[SingleMeasurement(name="Test Value", value=self._value, unit="units")]
                )

        self.benchmark_name = "test_custom_recorder"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Use custom recorder
        await super().setUp(backend_type="LocalLogMetrics", recorders=["test_custom", "runtime"])

        # Verify custom recorder loaded
        recorder_names = [r.__class__.__name__ for r in self.recorders]
        self.assertIn("TestCustomRecorder", recorder_names)

        # Run phase with the custom recorder
        await self.set_phase("benchmark")
        await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()

        await omni.kit.app.get_app().next_update_async()

    async def test_list_available_recorders(self) -> None:
        """Test listing available recorders from the registry."""
        available = MeasurementDataRecorderRegistry.list_available()

        # Verify core recorders are available
        core_recorders = [
            "app_frametime",
            "physics_frametime",
            "runtime",
            "memory",
            "cpu_continuous",
            "hardware",
            "gpu_frametime",
            "render_frametime",
        ]
        for recorder_name in core_recorders:
            self.assertIn(recorder_name, available, f"Recorder {recorder_name} not in registry")

    async def test_recorder_lifecycle(self) -> None:
        """Test that recorders start and stop correctly during phase transitions."""
        self.benchmark_name = "test_lifecycle"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        await super().setUp(backend_type="LocalLogMetrics", recorders=["runtime", "app_frametime"])

        # Loading phase - only runtime should start
        await self.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)
        active_names = [r.__class__.__name__ for r in self._active_recorders]
        self.assertIn("RuntimeRecorder", active_names)
        self.assertNotIn("AppFrametimeRecorder", active_names)
        await self.store_measurements()

        # Benchmark phase - both should start
        await self.set_phase("benchmark", start_recording_frametime=True, start_recording_runtime=True)
        active_names = [r.__class__.__name__ for r in self._active_recorders]
        self.assertIn("RuntimeRecorder", active_names)
        self.assertIn("AppFrametimeRecorder", active_names)

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()

        await omni.kit.app.get_app().next_update_async()

    async def test_recorder_context(self) -> None:
        """Test that recorders have access to InputContext."""
        self.benchmark_name = "test_context"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        await super().setUp(backend_type="LocalLogMetrics", recorders=["runtime"])

        # Verify recorders have context with expected fields
        for recorder in self.recorders:
            if hasattr(recorder, "context") and recorder.context:
                self.assertIsNotNone(recorder.context.artifact_prefix)
                self.assertIsNotNone(recorder.context.kit_version)
                self.assertIsNotNone(recorder.context.phase)

        await omni.kit.app.get_app().next_update_async()

    async def test_phase_transitions(self) -> None:
        """Test multiple phase transitions work correctly."""
        self.benchmark_name = "test_phases"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        await super().setUp(backend_type="LocalLogMetrics")

        # Test multiple phase transitions
        phases = ["loading", "warmup", "benchmark", "cooldown"]
        for phase in phases:
            await self.set_phase(phase)
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            await self.store_measurements()

        await omni.kit.app.get_app().next_update_async()

    async def test_stateless_recorders(self) -> None:
        """Test stateless recorders (memory, hardware) work correctly."""
        self.benchmark_name = "test_stateless"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        await super().setUp(backend_type="LocalLogMetrics", recorders=["memory", "hardware"])

        recorder_names = [r.__class__.__name__ for r in self.recorders]
        self.assertIn("MemoryRecorder", recorder_names)
        self.assertIn("HardwareSpecRecorder", recorder_names)

        await self.set_phase("benchmark")
        await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()

        await omni.kit.app.get_app().next_update_async()

    async def test_empty_recorders_list(self) -> None:
        """Test that an empty recorders list uses defaults."""
        self.benchmark_name = "test_empty_list"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        await super().setUp(backend_type="LocalLogMetrics", recorders=None)

        # Verify default recorders loaded
        self.assertGreater(len(self.recorders), 0, "Should have default recorders")
        recorder_names = [r.__class__.__name__ for r in self.recorders]
        self.assertIn("RuntimeRecorder", recorder_names)

        await omni.kit.app.get_app().next_update_async()

    async def test_report_generation(self) -> None:
        """Test that report generation flag works correctly."""
        self.benchmark_name = "test_report"
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        await super().setUp(backend_type="LocalLogMetrics", report_generation=True)
        self.assertTrue(self.report, "Report generation should be enabled")

        await self.set_phase("benchmark")
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()

        await omni.kit.app.get_app().next_update_async()

    async def test_backend_types(self) -> None:
        """Test different backend types can be initialized."""
        backend_types = ["LocalLogMetrics", "JSONFileMetrics", "OmniPerfKPIFile"]

        for backend_type in backend_types:
            self.benchmark_name = f"test_backend_{backend_type}"
            await omni.usd.get_context().new_stage_async()
            await omni.kit.app.get_app().next_update_async()

            await super().setUp(backend_type=backend_type, recorders=["runtime"])

            await self.set_phase("test")
            await omni.kit.app.get_app().next_update_async()
            await self.store_measurements()

            await omni.kit.app.get_app().next_update_async()
