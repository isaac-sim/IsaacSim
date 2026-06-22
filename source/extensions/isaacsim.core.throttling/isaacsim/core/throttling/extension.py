# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


"""Timeline-driven rendering and run loop throttling for Isaac Sim."""

import asyncio
import sys

import carb
import carb.eventdispatcher
import omni.ext
import omni.kit.app
import omni.timeline

ASYNC_TOGGLE_SETTING = "/exts/isaacsim.core.throttling/enable_async"
MANUAL_TOGGLE_SETTING = "/exts/isaacsim.core.throttling/enable_manualmode"
ASYNC_RENDERING_SETTING = "/app/asyncRendering"
ASYNC_RENDERING_LOW_LATENCY_SETTING = "/app/asyncRenderingLowLatency"

_extension_instance = None


class Extension(omni.ext.IExt):
    """Coordinate throttling settings with timeline playback state.

    The extension listens for timeline play, stop, and pause events. During playback it hides viewport gizmos,
    disables eco mode, optionally puts the main Kit loop in manual mode, and defers disabling async rendering
    until after the play callback returns. When playback stops or pauses, it restores editing-oriented settings
    and re-enables async rendering after a short frame delay unless Replicator still owns the setting (a capture
    pipeline is attached).
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize timeline subscriptions and throttling state.

        The extension stores the main loop interface when available, registers timeline event observers, and
        applies startup async rendering/manual loop settings from the extension configuration.

        Args:
            ext_id: Enabled extension identifier provided by Kit.
        """
        # Initialize loop runner
        self._loop_runner = None
        try:
            from omni.kit.loop import _loop as omni_loop

            self._loop_runner = omni_loop.acquire_loop_interface()
        except Exception:
            pass

        # frame counting for async toggle
        self._frame_count = 0
        self._waiting_for_async_reenable = False
        self._frame_update_subscription = None
        self._frame_delay = 10  # default 10 frame delay
        self._disable_async_rendering_task = None

        # Enable the developer throttling settings when extension starts
        carb.settings.get_settings().set("/app/show_developer_preference_section", True)

        self._replicator_capture_warning_logged = False

        global _extension_instance
        _extension_instance = self

        self.timeline_event_sub_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PLAY,
            on_event=self._on_play,
            observer_name="isaacsim.core.throttling.Extension._on_play",
        )
        self.timeline_event_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._on_stop,
            observer_name="isaacsim.core.throttling.Extension._on_stop",
        )
        self.timeline_event_sub_pause = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PAUSE,
            on_event=self._on_pause,
            observer_name="isaacsim.core.throttling.Extension._on_pause",
        )

        _settings = carb.settings.get_settings()

        if _settings.get(ASYNC_TOGGLE_SETTING):
            # Enable async rendering at startup
            self._set_async_rendering(True)

        if _settings.get(MANUAL_TOGGLE_SETTING):
            self._set_loop_manual_mode(False)

    def _set_loop_manual_mode(self, manual_mode: bool) -> None:
        """Set the main loop runner's manual mode when the loop interface is available.

        Args:
            manual_mode: Whether the main run loop should advance manually.
        """
        if self._loop_runner is not None:
            try:
                self._loop_runner.set_manual_mode(manual_mode)
            except Exception:
                pass

    def _set_async_rendering(self, enabled: bool) -> None:
        """Set both async rendering settings to the same state.

        Args:
            enabled: Whether async rendering should be enabled.
        """
        _settings = carb.settings.get_settings()
        _settings.set(ASYNC_RENDERING_SETTING, enabled)
        _settings.set(ASYNC_RENDERING_LOW_LATENCY_SETTING, enabled)

    def _on_frame_update(self, event: carb.events.IEvent) -> None:
        """Track update frames before async rendering is re-enabled.

        Re-enabling waits for the configured frame delay and only occurs while the timeline remains stopped or
        paused. Replicator ownership is checked at the end of the delay so async rendering is left untouched while a
        capture pipeline is attached.

        Args:
            event: Kit update event.
        """
        if not self._waiting_for_async_reenable:
            return
        self._frame_count += 1

        if self._frame_count >= self._frame_delay:
            # Check timeline did not play within the frame delay
            timeline = omni.timeline.get_timeline_interface()
            if not timeline.is_playing():
                # toggle async rendering
                _settings = carb.settings.get_settings()
                if _settings.get(ASYNC_TOGGLE_SETTING) and not self._is_replicator_managing_async_rendering():
                    self._set_async_rendering(True)

            self._stop_frame_counting()

    def _start_frame_counting(self) -> None:
        """Start the delayed async-rendering re-enable window."""
        self._frame_count = 0
        self._waiting_for_async_reenable = True

        # Create subscription on-demand to save on overhead
        if self._frame_update_subscription is None:
            self._frame_update_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._on_frame_update,
                observer_name="IsaacSimThrottling._on_frame_update",
            )

    def _stop_frame_counting(self) -> None:
        """Stop waiting to re-enable async rendering after pause or stop."""
        self._waiting_for_async_reenable = False
        self._frame_count = 0
        self._frame_update_subscription = None

    def _is_replicator_managing_async_rendering(self) -> bool:
        """Return whether Replicator currently owns the async rendering setting.

        Replicator drives ``/app/asyncRendering`` for the duration of a capture session: it disables it for
        synchronous captures and restores it on a delayed timer that also clears a stale asset-loading flag to work
        around a Kit issue where toggling async rendering can emit an ``ASSETS_LOADING`` event with no matching
        ``ASSETS_LOADED`` (NVBug-6169678). A configured capture pipeline (one or more attached annotators) means
        Replicator owns the setting even while the orchestrator is briefly ``STOPPED`` between steps, so the
        throttling extension must not toggle async rendering during that window or it re-introduces the one-sided
        ``ASSETS_LOADING`` event and the subsequent 30s asset-loading stall on the next Replicator step.

        Returns:
            True if a Replicator capture pipeline is configured or actively running; otherwise False.
        """
        rep = sys.modules.get("omni.replicator.core")
        if rep is None:
            return False

        try:
            annotator_registry = getattr(rep, "AnnotatorRegistry", None)
            if annotator_registry is None:
                annotators_module = sys.modules.get("omni.replicator.core.scripts.annotators")
                annotator_registry = getattr(annotators_module, "AnnotatorRegistry", None)
            # Attached annotators mean a capture pipeline is set up and Replicator owns async rendering, including
            # the stopped window between steps. Defer to Replicator regardless of the orchestrator run status.
            if annotator_registry is not None and annotator_registry.has_attached_annotators():
                return True

            # Fall back to run status so an actively running orchestrator still defers throttling even when the
            # annotator registry cannot be queried.
            status = rep.orchestrator.get_status()
            inactive_statuses = (rep.orchestrator.Status.STOPPED, rep.orchestrator.Status.STOPPING)
            return status not in inactive_statuses
        except Exception as exc:
            if not self._replicator_capture_warning_logged:
                carb.log_warn(f"Unable to query Replicator capture status for async rendering throttling: {exc}")
                self._replicator_capture_warning_logged = True
            return False

    async def _disable_async_rendering_after_update_async(self) -> None:
        """Disable async rendering one Kit update after timeline play begins if playback is still active."""
        try:
            await omni.kit.app.get_app().next_update_async()
        except asyncio.CancelledError:
            return

        _settings = carb.settings.get_settings()
        timeline = omni.timeline.get_timeline_interface()
        if _settings.get(ASYNC_TOGGLE_SETTING) and timeline.is_playing():
            self._set_async_rendering(False)

    def _defer_async_rendering_disable(self) -> None:
        """Schedule one pending task to disable async rendering after the current update."""
        task = getattr(self, "_disable_async_rendering_task", None)
        if task is not None and not task.done():
            return

        self._disable_async_rendering_task = asyncio.ensure_future(self._disable_async_rendering_after_update_async())

    def _cancel_deferred_async_rendering_disable(self) -> None:
        """Cancel a pending async-rendering disable task from a stale play event."""
        task = getattr(self, "_disable_async_rendering_task", None)
        if task is not None and not task.done():
            task.cancel()
        self._disable_async_rendering_task = None

    def _on_play(self, event: carb.eventdispatcher.Event) -> None:
        """Apply playback throttling when the timeline starts.

        Async rendering is disabled on the next update rather than inside the play event callback, which avoids
        deadlocks with play-on-load startup paths that begin simulation from within the callback. If playback stops
        before that update, the deferred task leaves async rendering unchanged.

        Args:
            event: Timeline play event.
        """
        _settings = carb.settings.get_settings()
        _settings.set("/rtx/ecoMode/enabled", False)
        _settings.set("/exts/omni.kit.hydra_texture/gizmos/enabled", False)
        self._stop_frame_counting()

        if _settings.get(ASYNC_TOGGLE_SETTING):
            self._defer_async_rendering_disable()

        if _settings.get(MANUAL_TOGGLE_SETTING):
            self._set_loop_manual_mode(True)

    def _on_stop(self, event: carb.eventdispatcher.Event) -> None:
        """Restore editing settings and begin async-rendering re-enable delay after stop.

        Args:
            event: Timeline stop event.
        """
        _settings = carb.settings.get_settings()
        _settings.set("/rtx/ecoMode/enabled", True)
        _settings.set("/exts/omni.kit.hydra_texture/gizmos/enabled", True)
        self._cancel_deferred_async_rendering_disable()

        if _settings.get(ASYNC_TOGGLE_SETTING):
            # Start frame delay counting to give replicator time to finish
            self._start_frame_counting()

        if _settings.get(MANUAL_TOGGLE_SETTING):
            self._set_loop_manual_mode(False)

    def _on_pause(self, event: carb.eventdispatcher.Event) -> None:
        """Restore editing settings and begin async-rendering re-enable delay after pause.

        Args:
            event: Timeline pause event.
        """
        _settings = carb.settings.get_settings()
        _settings.set("/rtx/ecoMode/enabled", True)
        _settings.set("/exts/omni.kit.hydra_texture/gizmos/enabled", True)
        self._cancel_deferred_async_rendering_disable()

        if _settings.get(ASYNC_TOGGLE_SETTING):
            # Start frame delay counting to give replicator time to finish
            self._start_frame_counting()

        if _settings.get(MANUAL_TOGGLE_SETTING):
            self._set_loop_manual_mode(False)

    def on_shutdown(self) -> None:
        """Release timeline observers, update subscriptions, and pending async tasks.

        Shutdown also clears frame-delay bookkeeping so reloading the extension starts from a clean state.
        """
        global _extension_instance
        _extension_instance = None

        self.timeline_event_sub_play = None
        self.timeline_event_sub_stop = None
        self.timeline_event_sub_pause = None

        self._stop_frame_counting()
        self._cancel_deferred_async_rendering_disable()


def get_instance() -> Extension | None:
    """Return the active throttling extension instance, if the extension is loaded.

    Returns:
        The active extension instance, or None if it is not loaded.
    """
    return _extension_instance
