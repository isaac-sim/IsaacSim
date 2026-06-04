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

"""Behavior script that randomizes light properties such as intensity and color."""

from __future__ import annotations

from typing import Any

import carb
import numpy as np
import omni.kit.commands
import omni.usd
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS
from isaacsim.replicator.behavior.utils.behavior_utils import (
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_exposed_variables,
)
from omni.behavior.scripting.core import BehaviorScript
from pxr import Gf, Sdf, Usd, UsdLux


class LightRandomizer(BehaviorScript):
    """Behavior script that randomizes light properties such as intensity and color for light prim(s)."""

    BEHAVIOR_NS = "lightRandomizer"
    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "includeChildren",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": "Include valid prim children to the behavior.",
        },
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 0,
            "doc": "Interval for updating the behavior. Value 0 means every frame.",
        },
        {
            "attr_name": "range:minColor",
            "attr_type": Sdf.ValueTypeNames.Color3f,
            "default_value": Gf.Vec3f(0.1, 0.1, 0.1),
            "doc": "Minimum RGB values for the light color randomization.",
        },
        {
            "attr_name": "range:maxColor",
            "attr_type": Sdf.ValueTypeNames.Color3f,
            "default_value": Gf.Vec3f(0.9, 0.9, 0.9),
            "doc": "Maximum RGB values for the light color randomization.",
        },
        {
            "attr_name": "range:intensity",
            "attr_type": Sdf.ValueTypeNames.Float2,
            "default_value": Gf.Vec2f(1000.0, 20000.0),
            "doc": "Range for the light intensity as (min, max).",
        },
        {
            "attr_name": "seed",
            "attr_type": Sdf.ValueTypeNames.Int,
            "default_value": -1,
            "doc": "Random seed for reproducible randomization. Use -1 for non-deterministic behavior.",
        },
    ]

    def on_init(self) -> None:
        """Called when the script is assigned to a prim."""
        self._rng = None
        self._update_counter = 0
        self._interval = 0
        self._valid_prims = []
        self._initial_attributes = {}

        # Expose the variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def on_destroy(self) -> None:
        """Called when the script is unassigned from a prim."""
        self._reset()
        # Exposed variables should be removed if the script is no longer assigned to the prim
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def on_play(self) -> None:
        """Called when `play` is pressed."""
        self._setup()
        # Make sure the initial behavior is applied if the interval is larger than 0
        if self._interval > 0:
            self._apply_behavior()

    def on_stop(self) -> None:
        """Called when `stop` is pressed."""
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
        # Fetch the exposed attributes (re-read on every setup so runtime edits take effect on the next apply)
        include_children = self._get_exposed_variable("includeChildren")
        self._interval = self._get_exposed_variable("interval")
        self._min_color = self._get_exposed_variable("range:minColor")
        self._max_color = self._get_exposed_variable("range:maxColor")
        self._intensity_range = self._get_exposed_variable("range:intensity")
        seed = self._get_exposed_variable("seed")

        # Skip the one-shot setup if already initialized (e.g. a play/pause/play loop). Re-caching here
        # would store the current randomized color/intensity as the "initial" and break restoration on stop.
        if self._valid_prims:
            return

        # Initialize the random number generator (use seed if valid, otherwise non-deterministic)
        if self._rng is None:
            self._rng = np.random.default_rng(seed if seed >= 0 else None)

        # Get the valid prims (light prims)
        if include_children:
            self._valid_prims = [prim for prim in Usd.PrimRange(self.prim) if prim.HasAPI(UsdLux.LightAPI)]
        elif self.prim.HasAPI(UsdLux.LightAPI):
            self._valid_prims = [self.prim]
        else:
            self._valid_prims = []
            carb.log_warn(f"[{self.prim_path}] No valid light prims found.")

        # Cache original attributes to restore after randomization
        self._initial_attributes = {}
        for prim in self._valid_prims:
            self._cache_initial_attributes(prim)

    def _reset(self) -> None:
        # Restore original attributes
        for prim, attrs in self._initial_attributes.items():
            for attr_name, attr_value in attrs.items():
                if attr_value is None:
                    continue
                prim.GetAttribute(attr_name).Set(attr_value)

        # Clear cached values
        self._valid_prims.clear()
        self._initial_attributes.clear()
        self._update_counter = 0
        self._rng = None

    def _apply_behavior(self) -> None:
        for prim in self._valid_prims:
            rand_color = (
                self._rng.uniform(self._min_color[0], self._max_color[0]),
                self._rng.uniform(self._min_color[1], self._max_color[1]),
                self._rng.uniform(self._min_color[2], self._max_color[2]),
            )
            prim.GetAttribute("inputs:color").Set(rand_color)

            rand_intensity = self._rng.uniform(self._intensity_range[0], self._intensity_range[1])
            prim.GetAttribute("inputs:intensity").Set(rand_intensity)

    def _cache_initial_attributes(self, prim: Usd.Prim) -> None:
        if not prim.HasAttribute("inputs:intensity"):
            prim.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float)
        if not prim.HasAttribute("inputs:color"):
            prim.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f)
        intensity = prim.GetAttribute("inputs:intensity").Get()
        color = prim.GetAttribute("inputs:color").Get()
        if intensity is None:
            carb.log_warn(
                f"[LightRandomizer] {prim.GetPath()}.inputs:intensity has no authored value and will be skipped on reset."
            )
        if color is None:
            carb.log_warn(
                f"[LightRandomizer] {prim.GetPath()}.inputs:color has no authored value and will be skipped on reset."
            )
        self._initial_attributes[prim] = {
            "inputs:intensity": intensity,
            "inputs:color": color,
        }

    def _get_exposed_variable(self, attr_name: str) -> Any:
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)

    def set_rng(self, rng: np.random.Generator | None = None) -> None:
        """Set the random number generator, overriding the USD seed attribute.

        Args:
            rng: Numpy random generator. If None, creates a new default generator.
        """
        self._rng = rng if rng is not None else np.random.default_rng()
