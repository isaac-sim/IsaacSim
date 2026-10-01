# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for retry-safe SRTX continuous capture state."""

from types import SimpleNamespace

import omni.kit.test
from isaacsim.ros2.nodes.impl.ros2_common import SrtxCaptureState


class TestSrtxCaptureRetryState(omni.kit.test.AsyncTestCase):
    """Verify transient SRTX start failures do not poison cached state."""

    async def test_failed_start_does_not_cache_output_path(self) -> None:
        """A failed start should allow the same output path to be retried."""
        state = SrtxCaptureState()
        state._refresh_stage = lambda: None

        srtx = SimpleNamespace(stop_calls=[], start_calls=[], fail_start=True)

        def stop_continuous_capture(sensor_set_name: str) -> None:
            srtx.stop_calls.append(sensor_set_name)

        def start_continuous_capture(sensor_set_name: str, output_paths: list[str]) -> None:
            srtx.start_calls.append((sensor_set_name, list(output_paths)))
            if srtx.fail_start:
                srtx.fail_start = False
                raise RuntimeError("transient SRTX start failure")

        srtx.stop_continuous_capture = stop_continuous_capture
        srtx.start_continuous_capture = start_continuous_capture

        with self.assertRaisesRegex(RuntimeError, "transient SRTX start failure"):
            state.start_or_extend(srtx, "sensor-set", "/Render/Product/LdrColor")

        self.assertNotIn("sensor-set", state._output_paths)

        state.start_or_extend(srtx, "sensor-set", "/Render/Product/LdrColor")

        self.assertEqual(state._output_paths["sensor-set"], ["/Render/Product/LdrColor"])
        self.assertEqual(len(srtx.start_calls), 2)
