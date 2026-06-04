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

"""Example standalone behavior script that applies a dummy behavior to prims."""

from __future__ import annotations

from typing import Any

import carb
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS
from isaacsim.replicator.behavior.utils.behavior_utils import (
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_exposed_variables,
)
from omni.behavior.scripting.core import BehaviorScript
from pxr import Sdf, Usd


class ExampleBehavior(BehaviorScript):
    """Example behavior script that applies a dummy behavior to prims.

    The behavior can be applied to multiple prims at once.
    """

    BEHAVIOR_NS = "exampleBehavior"

    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 0,
            "doc": "Interval for updating the behavior. Value 0 means every frame.",
        },
        {
            "attr_name": "includeChildren",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": "Include valid prim children to the behavior.",
        },
    ]

    def on_init(self) -> None:
        """Called when the script is assigned to a prim."""
        self._interval = 0
        self._update_counter = 0
        self._valid_prims = []

        # Expose the variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def on_destroy(self) -> None:
        """Called when the script is unassigned from a prim."""
        self._reset()
        # Exposed variables should be removed if the script is no longer assigned to the prim
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def on_play(self) -> None:
        """Handle play event by setting up and optionally applying the behavior."""
        print(f"[ExampleBehavior][{self.prim_path}] on_play()")
        self._setup()
        # Make sure the initial behavior is applied if the interval is larger than 0
        if self._interval > 0:
            self._apply_behavior()

    def on_stop(self) -> None:
        """Handle stop event by resetting the behavior state."""
        print(f"[ExampleBehavior][{self.prim_path}] on_stop()")
        self._reset()

    def on_update(self, current_time: float, delta_time: float) -> None:
        """Called on per frame update events that occur when `playing`.

        Args:
            current_time: The current simulation time.
            delta_time: The time elapsed since the last update.
        """
        if delta_time <= 0:
            return
        if self._interval <= 0:
            self._apply_behavior()
        else:
            self._update_counter += 1
            if self._update_counter >= self._interval:
                self._apply_behavior()
                self._update_counter = 0

    def _setup(self) -> None:
        # Fetch the exposed attributes
        self._include_children = self._get_exposed_variable("includeChildren")
        self._interval = self._get_exposed_variable("interval")

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
        self._interval = 0
        self._update_counter = 0

    def _apply_behavior(self) -> None:
        for prim in self._valid_prims:
            print(f"[ExampleBehavior][{self.prim_path}] Applying behavior to prim {prim.GetPath()}")

    def _get_exposed_variable(self, attr_name: str) -> Any:
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)
