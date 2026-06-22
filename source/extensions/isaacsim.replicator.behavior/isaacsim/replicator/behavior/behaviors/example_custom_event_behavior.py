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

"""Example behavior script controlled via custom event dispatching."""

from __future__ import annotations

from typing import Any

import carb
import carb.events
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS, EXTENSION_NAME
from isaacsim.replicator.behavior.utils.behavior_utils import (
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_exposed_variables,
)
from omni.behavior.scripting.core import BehaviorScript
from pxr import Sdf, Usd


class ExampleCustomEventBehavior(BehaviorScript):
    """Example behavior script controlled via custom event dispatching."""

    BEHAVIOR_NS = "exampleCustomEventBehavior"
    EVENT_NAME_IN = f"{EXTENSION_NAME}.{BEHAVIOR_NS}.in"
    EVENT_NAME_OUT = f"{EXTENSION_NAME}.{BEHAVIOR_NS}.out"
    ALLOWED_FUNCTIONS = ["setup", "update", "reset"]

    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "includeChildren",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": "Include valid prim children to the behavior.",
        },
        {
            "attr_name": "event:input",
            "attr_type": Sdf.ValueTypeNames.String,
            "default_value": f"{EVENT_NAME_IN}",
            "doc": (
                "Event to subscribe to for controlling the behavior.\n"
                "NOTE: Changing this value will not have any effect since the event subscription is done on init."
            ),
            "lock": True,
        },
        {
            "attr_name": "event:output",
            "attr_type": Sdf.ValueTypeNames.String,
            "default_value": f"{EVENT_NAME_OUT}",
            "doc": "Event name to publish to on behavior update.",
        },
    ]

    def on_init(self) -> None:
        """Called when the script is assigned to a prim."""
        self._event_name_out = self.EVENT_NAME_OUT
        self._valid_prims = []

        # App event stream, used to listen to incoming control events, and to publish the state of the behavior script
        self._event_stream = carb.eventdispatcher.get_eventdispatcher()

        # Subscribe to the event stream to listen for incoming control events
        self._event_sub = self._event_stream.observe_event(
            event_name=self.EVENT_NAME_IN,
            on_event=self._on_event,
            observer_name="ExampleCustomEventBehavior._event_sub",
        )

        # Expose the variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def on_destroy(self) -> None:
        """Called when the script is unassigned from a prim."""
        # Unsubscribe from the event stream
        self._reset()
        self._event_sub = None

        # Exposed variables should be removed if the script is no longer assigned to the prim
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def setup(self) -> None:
        """Set up the behavior and publish a setup completion event."""
        print(f"[ExampleCustomEventBehavior][{self.prim_path}] setup()")
        self._setup()
        self._event_stream.dispatch_event(
            event_name=self._event_name_out, payload={"prim_path": str(self.prim_path), "function_name": "setup"}
        )

    def update(self) -> None:
        """Apply the behavior and publish an update completion event."""
        print(f"[ExampleCustomEventBehavior][{self.prim_path}] update()")
        self._apply_behavior()
        self._event_stream.dispatch_event(
            event_name=self._event_name_out, payload={"prim_path": str(self.prim_path), "function_name": "update"}
        )

    def reset(self) -> None:
        """Reset the behavior and publish a reset completion event."""
        print(f"[ExampleCustomEventBehavior][{self.prim_path}] reset()")
        self._reset()
        self._event_stream.dispatch_event(
            event_name=self._event_name_out, payload={"prim_path": str(self.prim_path), "function_name": "reset"}
        )

    def _on_event(self, event: carb.events.IEvent) -> None:
        # If the specific prim_path is provided, but does not match the prim_path of this script, return
        if event.payload.get("prim_path", None) and event.payload.get("prim_path") != self.prim_path:
            return

        # Check if the function_name is valid, if so call the function
        if (function_name := event.payload.get("function_name", None)) and function_name in self.ALLOWED_FUNCTIONS:
            getattr(self, function_name)()

    def _setup(self) -> None:
        # Fetch the exposed attributes
        self._include_children = self._get_exposed_variable("includeChildren")
        self._event_name_out = self._get_exposed_variable("event:output")

        # Get the prims to apply the behavior to
        if self._include_children:
            self._valid_prims = [prim for prim in Usd.PrimRange(self.prim) if prim.IsValid()]
        elif self.prim.IsValid():
            self._valid_prims = [self.prim]
        else:
            self._valid_prims = []
            carb.log_warn(f"[{self.prim_path}] No valid prims found.")

    def _reset(self) -> None:
        self._valid_prims.clear()

    def _apply_behavior(self) -> None:
        if not self._valid_prims:
            print(f"[ExampleCustomEventBehavior][{self.prim_path}] No valid prims found.")
            return
        for prim in self._valid_prims:
            print(f"[ExampleCustomEventBehavior][{self.prim_path}] Applying behavior to prim {prim.GetPath()}")

    def _get_exposed_variable(self, attr_name: str) -> Any:
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)
