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

"""Opt-in composable that drives :class:`EpisodeRecorder` lifecycle from timeline events.

:class:`EpisodeRecorder` itself is timeline-agnostic by design. Callers who want "timeline
PLAY -> start_episode, STOP -> end_episode" attach this controller.

Timeline STOP always drains an active episode because stopping the timeline invalidates
physics views the recorder relies on; only the PLAY -> start_episode side can be gated off
via :attr:`auto_start_on_play` so that the user can trigger recording manually while the
timeline is already playing.
"""

from __future__ import annotations

from typing import Any

import carb

from .recorder import EpisodeRecorder


class TimelineDrivenEpisodeController:
    """Start / end episodes in response to timeline PLAY / STOP events.

    Args:
        recorder: The :class:`EpisodeRecorder` to drive.
        auto_start_on_play: When ``True`` (default), timeline PLAY / RESUME starts a new
            episode if the session is open and no episode is active. When ``False``, only
            timeline STOP is honored; recording must be started by an explicit UI click or
            :func:`dispatch_episode_command` call. This flag can be toggled at runtime via
            :meth:`set_auto_start_on_play`.
    """

    def __init__(
        self,
        recorder: EpisodeRecorder,
        *,
        auto_start_on_play: bool = True,
    ) -> None:
        self._recorder = recorder
        self._auto_start_on_play = bool(auto_start_on_play)
        self._callback_ids: list[int] = []
        self._enabled: bool = False

    @property
    def is_enabled(self) -> bool:
        """Return whether enabled."""
        return self._enabled

    @property
    def auto_start_on_play(self) -> bool:
        """Run the auto start on play operation."""
        return self._auto_start_on_play

    def set_auto_start_on_play(self, value: bool) -> None:
        """Toggle the PLAY -> start_episode behavior at runtime.

        Args:
            value: Value to process.
        """
        self._auto_start_on_play = bool(value)

    def enable(self) -> None:
        """Subscribe to simulation timeline events. Idempotent.

        Both PLAY / RESUME and STOP callbacks are always registered; the auto-start flag
        is evaluated inside the handler so that toggling the setting from the UI takes
        effect immediately without needing to re-enable the controller.
        """
        if self._enabled:
            return
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        self._callback_ids.append(
            SimulationManager.register_callback(self._on_started, event=SimulationEvent.SIMULATION_STARTED)
        )
        self._callback_ids.append(
            SimulationManager.register_callback(self._on_started, event=SimulationEvent.SIMULATION_RESUMED)
        )
        self._callback_ids.append(
            SimulationManager.register_callback(self._on_stopped, event=SimulationEvent.SIMULATION_STOPPED)
        )
        self._enabled = True

    def disable(self) -> None:
        """Unsubscribe. Idempotent."""
        if not self._enabled:
            return
        from isaacsim.core.simulation_manager import SimulationManager

        for cb_id in list(self._callback_ids):
            try:
                SimulationManager.deregister_callback(cb_id)
            except Exception:
                pass
        self._callback_ids.clear()
        self._enabled = False

    def _on_started(self, event: Any) -> None:
        if not self._auto_start_on_play:
            return
        if not self._recorder.is_session_open:
            return
        if self._recorder.is_recording or self._recorder.is_paused:
            return
        try:
            self._recorder.start_episode()
        except Exception as exc:
            carb.log_error(f"[TimelineDrivenEpisodeController] start_episode failed: {exc}")

    def _on_stopped(self, event: Any) -> None:
        if self._recorder.state != "episode_active":
            return
        try:
            self._recorder.end_episode()
        except Exception as exc:
            carb.log_error(f"[TimelineDrivenEpisodeController] end_episode failed: {exc}")
