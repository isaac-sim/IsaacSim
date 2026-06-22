# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Standalone dockable window hosting the :class:`EpisodeRecorderPanel`."""

from __future__ import annotations

from typing import Any

import carb.eventdispatcher
import omni.kit.app
import omni.ui as ui
import omni.usd

from .episode_recorder_panel import EpisodeRecorderPanel


class EpisodeRecorderWindow(ui.Window):
    """Main window for the Episode Recorder UI extension.

    Owns a single :class:`EpisodeRecorderPanel` and subscribes to stage
    close / app quit events so the session is torn down cleanly when the
    user closes the stage or exits Kit.

    Args:
        title: Window title to show in the Kit UI.
    """

    def __init__(self, title: str) -> None:
        super().__init__(title, dockPreference=ui.DockPreference.MAIN)
        self.deferred_dock_in("Property", ui.DockPolicy.DO_NOTHING)

        self._panel = EpisodeRecorderPanel()

        self._sub_shutdown = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_POST_QUIT,
            on_event=self._on_editor_quit_event,
            observer_name="isaacsim.replicator.episode_recorder.ui._on_editor_quit_event",
            order=0,
        )
        self._stage_event_sub = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop(
                self._on_stage_event,
                name="isaacsim.replicator.episode_recorder.ui._on_stage_event",
            )
        )

        with self.frame:
            with ui.ScrollingFrame():
                self._panel.build()

    def _on_editor_quit_event(self, _event: object) -> None:
        self._panel.destroy()

    def _on_stage_event(self, event: Any) -> None:
        if event.type == int(omni.usd.StageEventType.CLOSING):
            self._panel.on_stage_closed()

    def destroy(self) -> None:
        """Destroy the panel and release event subscriptions."""
        if self._panel is not None:
            self._panel.destroy()
            self._panel = None
        self._sub_shutdown = None
        self._stage_event_sub = None
        super().destroy()
