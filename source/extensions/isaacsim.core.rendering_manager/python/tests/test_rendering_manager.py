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

import asyncio

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.rendering_manager import RenderingEvent, RenderingManager, ViewportManager

_SETTING_RATE_LIMIT_ENABLED = "/app/runLoops/main/rateLimitEnabled"


class TestRenderingManager(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # ---------------
        self._callback_call = [0, 0]
        self._callback_call_stack = []
        self._fire_order = []
        self._rate_limit_enabled = carb.settings.get_settings().get_as_bool(_SETTING_RATE_LIMIT_ENABLED)
        await stage_utils.create_new_stage_async()
        # ---------------

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        # ------------------
        carb.settings.get_settings().set_bool(_SETTING_RATE_LIMIT_ENABLED, self._rate_limit_enabled)
        stage_utils.close_stage()
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------

    def _callback(self, *args, **kwargs):
        self._callback_call[0] += 1
        self._callback_call_stack.append(self._callback_call[:])
        self._fire_order.append("class")

    # --------------------------------------------------------------------

    async def test_render(self):
        for _ in range(10):
            await RenderingManager.render_async()

    async def test_dt(self):
        # get default vale
        for enabled in [False, True]:
            carb.settings.get_settings().set_bool(_SETTING_RATE_LIMIT_ENABLED, enabled)
            expected_dt = 1 / 120 if enabled else 1 / 60
            current_dt = RenderingManager.get_dt()
            self.assertAlmostEqual(
                expected_dt,
                current_dt,
                msg=f"Expected {1 / expected_dt} Hz. Got {1 / current_dt} Hz (rateLimitEnabled: {enabled})",
            )
        # set custom value
        for i, enabled in enumerate([False, True]):
            custom_dt = 1 / (99 + 10 * i)
            RenderingManager.set_dt(custom_dt)
            current_dt = RenderingManager.get_dt()
            self.assertAlmostEqual(
                custom_dt,
                current_dt,
                msg=f"Expected {1 / custom_dt} Hz. Got {1 / current_dt} Hz (rateLimitEnabled: {enabled})",
            )

    async def test_callback(self):
        # NEW_FRAME events are pumped by Hydra's GPU completion check; with the default
        # rendering pipeline a single render_async() may yield 0, 1, or more NEW_FRAME
        # dispatches. A strict 1:1 mapping only holds under /app/hydraEngine/waitIdle=true
        # plus a late /app/updateOrder/checkForHydraRenderComplete (e.g. the
        # isaacsim.exp.base.zero_delay experience), neither of which is set by the
        # default test runner. This test therefore asserts the documented
        # RenderingManager contracts directly, with no dependence on event cadence:
        #   - register_callback returns monotonically increasing ids
        #   - registered callbacks fire while subscribed and never after deregistration
        #   - co-registered callbacks fire on the same NEW_FRAME dispatches, in
        #     registration order (lockstep)
        #   - deregister_all_callbacks removes every subscription
        #   - deregistering a stale id is a safe no-op

        def callback(*args, **kwargs):
            self._callback_call[1] += 1
            self._callback_call_stack.append(self._callback_call[:])
            self._fire_order.append("local")

        status, frames = await ViewportManager.wait_for_viewport_async()
        self.assertTrue(status, f"Viewport not ready after {frames} frames")
        self.assertListEqual(self._callback_call, [0, 0])
        await asyncio.sleep(0.5)  # let previous (unregistered) events pass, since we are not waiting idly

        RENDERS_PER_PHASE = 5

        async def run_renders():
            for _ in range(RENDERS_PER_PHASE):
                await RenderingManager.render_async()

        def snapshot():
            return self._callback_call[0], self._callback_call[1], len(self._fire_order)

        # phase A: only the local callback is registered
        local_id = RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=callback)
        self.assertEqual(local_id, 0)
        c0, l0, f0 = snapshot()
        await run_renders()
        c1, l1, f1 = snapshot()
        self.assertGreater(l1 - l0, 0, "local callback should have fired in phase A")
        self.assertEqual(c1, c0, "class callback must not fire before it is registered")
        self.assertTrue(
            all(k == "local" for k in self._fire_order[f0:f1]),
            f"phase A fires must be local-only, got {self._fire_order[f0:f1]}",
        )

        # phase B: register class on top; both fire in lockstep, in registration order
        class_id = RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=self._callback)
        self.assertEqual(class_id, 1)
        c0, l0, f0 = snapshot()
        await run_renders()
        c1, l1, f1 = snapshot()
        self.assertGreater(l1 - l0, 0, "local callback should fire in phase B")
        self.assertEqual(
            l1 - l0,
            c1 - c0,
            "co-registered callbacks must fire on the same NEW_FRAME dispatches",
        )
        phase_b = self._fire_order[f0:f1]
        self.assertEqual(len(phase_b) % 2, 0, f"phase B fires must be paired, got {phase_b}")
        for i in range(0, len(phase_b), 2):
            self.assertEqual(
                phase_b[i : i + 2],
                ["local", "class"],
                f"phase B pair {i // 2} must fire in registration order, got {phase_b[i : i + 2]}",
            )

        # phase C: deregister local; only class continues to fire
        RenderingManager.deregister_callback(local_id)
        c0, l0, f0 = snapshot()
        await run_renders()
        c1, l1, f1 = snapshot()
        self.assertEqual(l1, l0, "deregistered local callback must not fire")
        self.assertGreater(c1 - c0, 0, "class callback should still fire in phase C")
        self.assertTrue(
            all(k == "class" for k in self._fire_order[f0:f1]),
            f"phase C fires must be class-only, got {self._fire_order[f0:f1]}",
        )

        # phase D: deregister class as well; no callbacks fire
        RenderingManager.deregister_callback(class_id)
        c0, l0, f0 = snapshot()
        await run_renders()
        c1, l1, f1 = snapshot()
        self.assertEqual((c1, l1, f1), (c0, l0, f0), "no callbacks should fire when none are registered")

        # phase E: re-register class; id increments past previously-issued ids
        class_id_2 = RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=self._callback)
        self.assertEqual(class_id_2, 2)
        c0, l0, f0 = snapshot()
        await run_renders()
        c1, l1, f1 = snapshot()
        self.assertEqual(l1, l0, "previously-deregistered local callback must remain silent")
        self.assertGreater(c1 - c0, 0, "re-registered class callback should fire in phase E")
        self.assertTrue(
            all(k == "class" for k in self._fire_order[f0:f1]),
            f"phase E fires must be class-only, got {self._fire_order[f0:f1]}",
        )

        # phase F: deregister_all_callbacks removes every subscription
        RenderingManager.deregister_all_callbacks()
        c0, l0, f0 = snapshot()
        await run_renders()
        c1, l1, f1 = snapshot()
        self.assertEqual((c1, l1, f1), (c0, l0, f0), "deregister_all_callbacks should silence all callbacks")

        # phase G: deregistering already-deregistered ids logs a warning but does not raise
        RenderingManager.deregister_callback(local_id)
        RenderingManager.deregister_callback(class_id_2)
        c0, l0, f0 = snapshot()
        await run_renders()
        c1, l1, f1 = snapshot()
        self.assertEqual((c1, l1, f1), (c0, l0, f0), "stale deregister_callback must be a no-op")
