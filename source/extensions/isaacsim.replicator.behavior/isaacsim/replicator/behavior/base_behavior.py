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

"""Base behavior script with common functionality for exposing variables as USD attributes."""

from __future__ import annotations

from typing import Any, NoReturn

from isaacsim.replicator.behavior.utils.behavior_utils import (
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_exposed_variables,
)
from omni.behavior.scripting.core import BehaviorScript
from pxr import Sdf

from .global_variables import EXPOSED_ATTR_NS


class BaseBehavior(BehaviorScript):
    """Base class for behavior scripts, providing common functionality for exposing variables as USD attributes.

    Attributes:
        BEHAVIOR_NS (str): Namespace for the behavior, to be defined by subclasses.
        VARIABLES_TO_EXPOSE (List[Dict]): List of variables to expose, defined by subclasses.
    """

    BEHAVIOR_NS = "baseBehavior"
    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 0,
            "doc": "Interval for updating the behavior. Value 0 means every frame.",
        },
    ]

    def on_init(self) -> None:
        """Called when the script is assigned to a prim."""
        # Workaround to prevent base class instantiation by the ScriptManager
        if self.__class__.__name__ == "BaseBehavior":
            return

        self._update_counter = 0
        self._interval = 0

        # Expose the variables as USD attributes. The property window refresh (if a UI is loaded) is
        # triggered via a carb event dispatched from inside create_exposed_variables().
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def on_destroy(self) -> None:
        """Called when the script is unassigned from a prim."""
        # Exposed variables should be removed if the script is no longer assigned to the prim
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def on_play(self) -> None:
        """Called when `play` is pressed."""
        self._interval = self._get_exposed_variable("interval")
        print(f"[BaseBehavior][{self.prim_path}] on_play(); interval: {self._interval}")

    def on_stop(self) -> None:
        """Called when `stop` is pressed."""
        print(f"[BaseBehavior][{self.prim_path}] on_stop()")
        self._interval = 0
        self._update_counter = 0

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

    def _apply_behavior(self) -> NoReturn:
        """Pure virtual method that must be implemented by subclasses."""
        raise NotImplementedError

    def _get_exposed_variable(self, attr_name: str) -> Any:
        """Get the value of an exposed attribute.

        Args:
            attr_name: The name of the attribute to retrieve.

        Returns:
            The value of the exposed attribute.
        """
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)
