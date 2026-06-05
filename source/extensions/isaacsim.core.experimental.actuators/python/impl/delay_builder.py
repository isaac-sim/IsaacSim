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

"""Builder for `newton.actuators.Delay` from a parsed `NewtonActuatorDelayAPI` prim."""

from __future__ import annotations

from typing import Any

import warp as wp
from newton.actuators import Delay


def build_delay(kwargs: dict[str, Any], n: int, device: wp.Device) -> Delay:
    """Build a `Delay` sized for an `n`-instance actuator batch.

    The schema authors a single scalar `newton:delaySteps`; the same delay is applied to
    every physical actuator this `Actuator` represents. `max_delay` is the same scalar value
    (the circular buffer depth equals the per-DOF delay).

    Args:
        kwargs: Scalar kwargs from `ActuatorParsed.component_specs` (expects `delay_steps`).
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device for the `delay_steps` array.

    Returns:
        A configured `Delay` instance.
    """
    resolved = Delay.resolve_arguments(kwargs)
    steps = int(resolved["delay_steps"])
    return Delay(
        delay_steps=wp.array([steps] * n, dtype=wp.int32, device=device),
        max_delay=steps,
    )
