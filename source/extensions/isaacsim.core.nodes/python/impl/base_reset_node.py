# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Base class for nodes that automatically reset when the timeline stops."""

import carb.eventdispatcher
import carb.events
import omni.timeline
import omni.usd


class BaseResetNode:
    """Base class for nodes that automatically reset when stop is pressed.

    Args:
        initialize: Whether the node should be initialized on creation.
    """

    def __init__(self, initialize: bool = False) -> None:
        self.initialized = initialize

        timeline = omni.timeline.get_timeline_interface()

        self.timeline_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self.on_stop_play,
            observer_name="isaacsim.core.nodes.BaseResetNode.on_stop_play",
        )

    def on_stop_play(self, event: carb.eventdispatcher.Event) -> None:
        """Timeline stop event callback - reset node state.

        Args:
            event: The timeline stop event from the event dispatcher.
        """
        self.custom_reset()
        self.initialized = False

    # Defined by subclass
    def custom_reset(self) -> None:
        """Custom reset logic to be implemented by subclasses.

        This method is called when the timeline stops to perform node-specific reset operations.
        """

    def reset(self) -> None:
        """Cleans up the node by clearing event subscriptions and initialization state."""
        self.timeline_event_sub = None
        self.initialized = None
