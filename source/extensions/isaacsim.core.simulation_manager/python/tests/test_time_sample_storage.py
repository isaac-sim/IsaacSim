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

"""Verifies SimulationManager time sample storage through monotonic sampling, interpolation, range queries, logging, and structured access. The tests cover buffer overflow, high-frequency stepping, stop-play behavior, pause-resume behavior, and stored-time lookups."""

from typing import Any

import carb
import omni.kit.app
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.utils.stage import create_new_stage_async, get_current_stage
from isaacsim.core.simulation_manager import SimulationManager


class TestTimeSampleStorage(omni.kit.test.AsyncTestCase):
    """Test time sample storage."""

    async def setUp(self) -> None:
        """Set up test environment."""
        super().setUp()
        await create_new_stage_async()
        # Set stage FPS to match carb setting for deterministic time sample testing
        settings = carb.settings.get_settings()
        sim_period_denom = settings.get("/app/settings/fabricDefaultSimPeriodDenominator") or 60
        get_current_stage(backend="usd").SetTimeCodesPerSecond(sim_period_denom)
        # Set physics dt to 1/60 for consistent timestep across engines
        SimulationManager.set_physics_dt(1.0 / 60.0)
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up after test."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    def _verify_samples_monotonic(self, samples: Any, message: Any = "Samples should be monotonic") -> None:
        """Manually verify that samples are monotonically increasing in time.

        This replaces the removed validateSamplesMonotonic C++ function.

        Args:
            samples: Time sample entries to validate.
            message: Assertion message prefix to use when monotonicity checks fail.
        """
        for i in range(1, len(samples)):
            prev_entry = samples[i - 1]
            curr_entry = samples[i]

            # Access the structured data
            prev_time = prev_entry.time.to_float()
            curr_time = curr_entry.time.to_float()
            prev_sim_time = prev_entry.data.sim_time
            curr_sim_time = curr_entry.data.sim_time
            prev_sim_time_monotonic = prev_entry.data.sim_time_monotonic
            curr_sim_time_monotonic = curr_entry.data.sim_time_monotonic

            self.assertLess(prev_time, curr_time, f"{message}: Rational time not monotonic at sample {i}")
            self.assertLessEqual(prev_sim_time, curr_sim_time, f"{message}: Sim time not monotonic at sample {i}")
            self.assertLessEqual(
                prev_sim_time_monotonic,
                curr_sim_time_monotonic,
                f"{message}: Monotonic sim time not monotonic at sample {i}",
            )

    async def test_time_samples_monotonic_increase(self) -> None:
        """Test that stored samples are monotonically increasing."""
        timeline = omni.timeline.get_timeline_interface()

        # Initially should be empty
        self.assertEqual(SimulationManager._simulation_manager_interface.get_sample_count(), 0)

        # Start simulation and let it run
        timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        # Should now have samples
        sample_count = SimulationManager._simulation_manager_interface.get_sample_count()
        self.assertGreater(sample_count, 0, "Should have stored samples")

        # Get all samples and manually verify monotonic increase
        samples = SimulationManager._simulation_manager_interface.get_all_samples()
        self.assertEqual(len(samples), sample_count)

        # Manually verify monotonic increase
        self._verify_samples_monotonic(samples)

        for i in range(1, len(samples)):
            prev_entry = samples[i - 1]
            curr_entry = samples[i]

            # Access the structured data
            prev_time = prev_entry.time.to_float()
            curr_time = curr_entry.time.to_float()
            prev_sim_time = prev_entry.data.sim_time
            curr_sim_time = curr_entry.data.sim_time

            self.assertLess(prev_time, curr_time, f"Rational time not monotonic at sample {i}")
            self.assertLessEqual(prev_sim_time, curr_sim_time, f"Sim time not monotonic at sample {i}")

            # Verify all entries are valid
            self.assertTrue(prev_entry.valid, f"Sample {i-1} should be valid")
            self.assertTrue(curr_entry.valid, f"Sample {i} should be valid")

    async def test_high_frequency_simulation_buffer_behavior(self) -> None:
        """Test high write rate doesn't cause issues and buffer behaves correctly."""
        SimulationManager.set_physics_dt(1 / 1000)  # Physics at 1000 Hz
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Track frame times to verify 3-frame history access
        frame_times = []

        # Run simulation for many steps (more than buffer capacity)
        buffer_capacity = SimulationManager._simulation_manager_interface.get_buffer_capacity()
        for step in range(buffer_capacity + 40):
            await omni.kit.app.get_app().next_update_async()

            # Capture current frame time every few steps
            if step % 5 == 0:
                current_frame_time = SimulationManager._simulation_manager_interface.get_current_time()
                if current_frame_time.numerator != -1:  # Valid time (not kInvalidRationalTime)
                    frame_times.append((step, current_frame_time))

            # Check periodically that storage remains valid
            if step % 25 == 0:
                sample_count = SimulationManager._simulation_manager_interface.get_sample_count()
                self.assertLessEqual(sample_count, buffer_capacity, f"Should not exceed buffer capacity at step {step}")
                # Verify monotonic increase manually
                samples_at_step = SimulationManager._simulation_manager_interface.get_all_samples()
                self._verify_samples_monotonic(samples_at_step, f"Should remain monotonic at step {step}")

                # If we have at least 4 frame captures (≥ 3 frames ago), test historical access
                if len(frame_times) >= 4:
                    # Try to read time from 3 frames ago
                    three_frames_ago = frame_times[-4][1]  # 4th from end = 3 frames ago
                    sim_time_historical = SimulationManager._simulation_manager_interface.get_simulation_time_at_time(
                        three_frames_ago
                    )
                    self.assertIsNotNone(
                        sim_time_historical, f"Should be able to read time from 3 frames ago at step {step}"
                    )

        # Should not exceed buffer capacity due to circular buffer
        final_count = SimulationManager._simulation_manager_interface.get_sample_count()
        self.assertLessEqual(final_count, buffer_capacity, "Should not exceed buffer capacity")

        # Should still have samples and be monotonic
        self.assertGreater(final_count, 0)
        final_samples = SimulationManager._simulation_manager_interface.get_all_samples()
        self._verify_samples_monotonic(final_samples)

        # Test that we can still read at various times
        samples = SimulationManager._simulation_manager_interface.get_all_samples()
        for i, sample in enumerate(samples[::5]):  # Test every 5th sample
            numerator = sample.time.numerator
            denominator = sample.time.denominator
            sim_time = SimulationManager._simulation_manager_interface.get_simulation_time_at_time(
                (numerator, denominator)
            )
            self.assertIsNotNone(sim_time, f"Should be able to read sample {i*5}")

        # Final verification: ensure we captured frame times and can access historical data
        self.assertGreaterEqual(len(frame_times), 4, "Should have captured multiple frame times")

        # Verify we can read from the oldest captured frame time
        if frame_times:
            oldest_frame_time = frame_times[0][1]
            sim_time_oldest = SimulationManager._simulation_manager_interface.get_simulation_time_at_time(
                oldest_frame_time
            )
            # Note: This may be None if the sample was evicted from the circular buffer
            # That's expected behavior for very old samples beyond buffer capacity

    async def test_stop_play_behavior(self) -> None:
        """Test behavior during stop/play cycles."""
        timeline = omni.timeline.get_timeline_interface()

        # Start simulation
        timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        samples_after_play = SimulationManager._simulation_manager_interface.get_all_samples()
        count_after_play = len(samples_after_play)
        self.assertGreater(count_after_play, 0)
        self._verify_samples_monotonic(samples_after_play)

        # Get time range
        time_range = SimulationManager._simulation_manager_interface.get_sample_range()
        self.assertIsNotNone(time_range, "Should have time range when samples exist")

        # Verify time range makes sense
        earliest_time, latest_time = time_range
        earliest_seconds = earliest_time.to_float()
        latest_seconds = latest_time.to_float()
        self.assertLess(earliest_seconds, latest_seconds, "Latest time should be greater than earliest")

        # Stop simulation
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Storage should be cleared on stop
        count_after_stop = SimulationManager._simulation_manager_interface.get_sample_count()
        self.assertEqual(count_after_stop, 0, "Storage should be cleared on stop")

        # Time range should be None when empty
        time_range_after_stop = SimulationManager._simulation_manager_interface.get_sample_range()
        self.assertIsNone(time_range_after_stop, "Should have no time range when empty")

        # Start again
        timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        samples_after_restart = SimulationManager._simulation_manager_interface.get_all_samples()
        count_after_restart = len(samples_after_restart)
        self.assertGreater(count_after_restart, 0)
        self._verify_samples_monotonic(samples_after_restart)

    async def test_pause_resume_behavior(self) -> None:
        """Test that pause/resume doesn't break sample consistency."""
        timeline = omni.timeline.get_timeline_interface()

        # Start and run
        timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        samples_before_pause = SimulationManager._simulation_manager_interface.get_all_samples()
        count_before_pause = len(samples_before_pause)
        SimulationManager._simulation_manager_interface.log_statistics()

        # Pause
        timeline.pause()
        await omni.kit.app.get_app().next_update_async()

        # Sample count shouldn't change during pause
        count_during_pause = SimulationManager._simulation_manager_interface.get_sample_count()
        self.assertEqual(count_during_pause, count_before_pause)
        samples_during_pause = SimulationManager._simulation_manager_interface.get_all_samples()
        self._verify_samples_monotonic(samples_during_pause)

        # Resume and continue
        timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        samples_after_resume = SimulationManager._simulation_manager_interface.get_all_samples()
        SimulationManager._simulation_manager_interface.log_statistics()
        count_after_resume = len(samples_after_resume)
        self.assertGreater(count_after_resume, count_before_pause)
        self._verify_samples_monotonic(samples_after_resume)

    async def test_interpolation_with_stored_samples(self) -> None:
        """Test interpolation using actual stored samples."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Run simulation to get some samples
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        samples = SimulationManager._simulation_manager_interface.get_all_samples()

        self.assertGreater(len(samples), 2, "Need at least 2 samples for interpolation test")

        # Test interpolation between first and last sample
        first_sample = samples[0]
        last_sample = samples[-1]

        # Calculate midpoint time
        first_time = first_sample.time.to_float()
        last_time = last_sample.time.to_float()
        mid_time = (first_time + last_time) / 2

        # Convert back to rational using current sim period denominator
        settings = carb.settings.get_settings()
        sim_period_denom = settings.get("/app/settings/fabricDefaultSimPeriodDenominator") or 60
        mid_numerator = int(mid_time * sim_period_denom)
        mid_denominator = sim_period_denom

        # Test interpolation
        interpolated_sim_time = SimulationManager._simulation_manager_interface.get_simulation_time_at_time(
            (mid_numerator, mid_denominator)
        )
        self.assertIsNotNone(interpolated_sim_time)

        # Should be between first and last simulation times
        first_sim_time = first_sample.data.sim_time
        last_sim_time = last_sample.data.sim_time
        self.assertGreaterEqual(interpolated_sim_time, first_sim_time)
        self.assertLessEqual(interpolated_sim_time, last_sim_time)

    async def test_time_sample_range(self) -> None:
        """Test time sample range functionality."""
        # Should be None when empty
        self.assertIsNone(SimulationManager._simulation_manager_interface.get_sample_range())

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Run enough frames to accumulate a meaningful time range but stay
        # within the circular buffer capacity so no samples are evicted.
        physics_fps = 60
        expected_frame_delta = 1.0 / physics_fps
        buffer_capacity = SimulationManager._simulation_manager_interface.get_buffer_capacity()
        num_frames = buffer_capacity - 5

        for f in range(num_frames):
            await omni.kit.app.get_app().next_update_async()

        # Should have range now
        time_range = SimulationManager._simulation_manager_interface.get_sample_range()
        self.assertIsNotNone(time_range)

        earliest_time, latest_time = time_range
        earliest_seconds = earliest_time.to_float()
        latest_seconds = latest_time.to_float()

        self.assertLess(
            earliest_seconds,
            latest_seconds,
            f"Latest time {latest_seconds} should be greater than earliest {earliest_seconds}",
        )

        # Verify range matches actual samples
        samples = SimulationManager._simulation_manager_interface.get_all_samples()
        self.assertGreater(len(samples), 1, "Should have multiple samples")

        first_sample_time = samples[0].time.to_float()
        last_sample_time = samples[-1].time.to_float()

        # Derive the expected total delta from the actual number of stored
        # samples (each consecutive pair is one physics step apart).
        expected_total_delta = (len(samples) - 1) * expected_frame_delta

        # Verify range delta matches expected timing
        time_delta = latest_seconds - earliest_seconds
        self.assertAlmostEqual(
            time_delta,
            expected_total_delta,
            places=2,
            msg=f"Time range {earliest_seconds:.4f} - {latest_seconds:.4f} ({len(samples)} samples) should span {expected_total_delta:.4f}s",
        )

        # Verify delta between consecutive samples matches physics dt
        for i in range(1, len(samples)):
            prev_time = samples[i - 1].time.to_float()
            curr_time = samples[i].time.to_float()
            actual_delta = curr_time - prev_time
            self.assertAlmostEqual(
                actual_delta,
                expected_frame_delta,
                places=4,
                msg=f"Frame {i} time delta should be ~{expected_frame_delta:.4f}s (1/{physics_fps}), got {actual_delta:.4f}s",
            )

            prev_sim_time = samples[i - 1].data.sim_time
            curr_sim_time = samples[i].data.sim_time
            sim_delta = curr_sim_time - prev_sim_time
            self.assertAlmostEqual(
                sim_delta,
                expected_frame_delta,
                places=4,
                msg=f"Frame {i} sim_time delta should be ~{expected_frame_delta:.4f}s (1/{physics_fps}), got {sim_delta:.4f}s",
            )

        self.assertAlmostEqual(earliest_seconds, first_sample_time, places=6)
        self.assertAlmostEqual(latest_seconds, last_sample_time, places=6)

        # Verify simulation time difference matches expected total delta
        first_sample_sim_time = samples[0].data.sim_time
        last_sample_sim_time = samples[-1].data.sim_time
        sim_time_delta = last_sample_sim_time - first_sample_sim_time
        self.assertAlmostEqual(
            sim_time_delta,
            expected_total_delta,
            places=2,
            msg=f"Simulation time should span {expected_total_delta:.4f}s ({len(samples)} samples)",
        )

    async def test_logging_functionality(self) -> None:
        """Test that logging doesn't crash and works with different storage states."""
        # Should work when empty
        SimulationManager._simulation_manager_interface.log_statistics()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        print("Playing timeline")
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        print("Done playing timeline")
        # Should work with samples
        SimulationManager._simulation_manager_interface.log_statistics()

        # Should work after stop (when cleared)
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        print("Stopped")
        SimulationManager._simulation_manager_interface.log_statistics()
        await omni.kit.app.get_app().next_update_async()

    async def test_structured_data_access(self) -> None:
        """Test that the Entry and TimeData objects work correctly."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        samples = SimulationManager._simulation_manager_interface.get_all_samples()
        self.assertGreater(len(samples), 0)

        for i, sample in enumerate(samples):
            # Test time access
            self.assertIsInstance(sample.time.numerator, int)
            self.assertIsInstance(sample.time.denominator, int)

            # Test that to_float() works
            time_float = sample.time.to_float()
            self.assertIsInstance(time_float, float)
            self.assertGreaterEqual(time_float, 0.0)

            # Test data access - all should be numbers
            self.assertIsInstance(sample.data.sim_time, float)
            self.assertIsInstance(sample.data.sim_time_monotonic, float)
            self.assertIsInstance(sample.data.system_time, float)

            # Test valid flag
            self.assertTrue(sample.valid, f"Sample {i} should be valid")

    async def test_buffer_overflow_oldest_eviction(self) -> None:
        """Test that buffer overflow correctly evicts oldest samples."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Run for exactly buffer capacity + some extra
        buffer_capacity = SimulationManager._simulation_manager_interface.get_buffer_capacity()
        for _ in range(buffer_capacity + 10):
            await omni.kit.app.get_app().next_update_async()

        samples = SimulationManager._simulation_manager_interface.get_all_samples()
        sample_count = len(samples)

        # Should not exceed capacity
        self.assertLessEqual(sample_count, buffer_capacity, "Should not exceed buffer capacity")

        # Should still be monotonic (oldest samples evicted)
        self._verify_samples_monotonic(samples)

        # The remaining samples should be the most recent ones
        # (we can't easily test this without knowing exact timing,
        #  but the monotonic check verifies basic correctness)

    async def test_get_simulation_time_at_time(self) -> None:
        """Test that get_simulation_time_at_time works correctly."""
        invalid_time = SimulationManager._simulation_manager_interface.get_simulation_time_at_time((0, 0))
        self.assertAlmostEqual(invalid_time, 0.0)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        invalid_time = SimulationManager._simulation_manager_interface.get_simulation_time_at_time((0, 0))
        self.assertAlmostEqual(invalid_time, 3 * 1 / 60)  # two initial steps done during play + 1 more step
