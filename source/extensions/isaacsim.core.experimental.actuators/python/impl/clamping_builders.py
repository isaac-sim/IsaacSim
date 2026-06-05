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

"""Builders for `newton.actuators` clamping components from parsed `NewtonActuator` prims.

One builder per concrete clamping class. Each one validates schema values via the class's
own `resolve_arguments` classmethod, then constructs the Newton component with explicitly
typed arguments — per-DOF numeric parameters are placed into length-`n` `wp.array`s;
shared table-valued parameters (`ClampingPositionBased` lookup tables) pass through as-is
because Newton already broadcasts them across every physical actuator.

Callers typically use the `build_clamping` dispatcher, which routes on the component class.
"""

from __future__ import annotations

from typing import Any

import warp as wp
from newton.actuators import (
    Clamping,
    ClampingDCMotor,
    ClampingMaxEffort,
    ClampingPositionBased,
)


def _float_array(value: float, n: int, device: wp.Device) -> wp.array:
    """Build a length-`n` `wp.float32` array filled with `value`."""
    return wp.array([float(value)] * n, dtype=wp.float32, device=device)


def build_clamping_max_effort(kwargs: dict[str, Any], n: int, device: wp.Device) -> ClampingMaxEffort:
    """Build a `ClampingMaxEffort` from scalar schema kwargs.

    Args:
        kwargs: Scalar kwargs from `ActuatorParsed.component_specs`.
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device for the limit array.

    Returns:
        A configured `ClampingMaxEffort` instance.
    """
    resolved = ClampingMaxEffort.resolve_arguments(kwargs)
    return ClampingMaxEffort(max_effort=_float_array(resolved["max_effort"], n, device))


def build_clamping_dc_motor(kwargs: dict[str, Any], n: int, device: wp.Device) -> ClampingDCMotor:
    """Build a `ClampingDCMotor` from scalar schema kwargs.

    Args:
        kwargs: Scalar kwargs from `ActuatorParsed.component_specs`.
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device for the parameter arrays.

    Returns:
        A configured `ClampingDCMotor` instance.
    """
    resolved = ClampingDCMotor.resolve_arguments(kwargs)
    return ClampingDCMotor(
        saturation_effort=_float_array(resolved["saturation_effort"], n, device),
        velocity_limit=_float_array(resolved["velocity_limit"], n, device),
        max_motor_effort=_float_array(resolved["max_motor_effort"], n, device),
    )


def build_clamping_position_based(kwargs: dict[str, Any], n: int, device: wp.Device) -> ClampingPositionBased:
    """Build a `ClampingPositionBased` from schema kwargs.

    The lookup table (`lookup_positions` + `lookup_efforts`, or `lookup_table_path`) is
    shared across every physical actuator represented by the owning `Actuator`; Newton's
    `ClampingPositionBased.finalize` allocates the device arrays internally at `Actuator`
    construction time. `n` and `device` are accepted for signature uniformity and are not
    used here.

    Args:
        kwargs: Schema kwargs from `ActuatorParsed.component_specs`.
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device (unused).

    Returns:
        A configured `ClampingPositionBased` instance.
    """
    resolved = ClampingPositionBased.resolve_arguments(kwargs)
    return ClampingPositionBased(
        lookup_table_path=resolved.get("lookup_table_path"),
        lookup_positions=tuple(resolved["lookup_positions"]) if "lookup_positions" in resolved else None,
        lookup_efforts=tuple(resolved["lookup_efforts"]) if "lookup_efforts" in resolved else None,
    )


_CLAMPING_BUILDERS = {
    ClampingMaxEffort: build_clamping_max_effort,
    ClampingDCMotor: build_clamping_dc_motor,
    ClampingPositionBased: build_clamping_position_based,
}


def build_clamping(cls: type[Clamping], kwargs: dict[str, Any], n: int, device: wp.Device) -> Clamping:
    """Dispatch to the builder for the given clamping class.

    Args:
        cls: Concrete clamping class from `ActuatorParsed.component_specs`.
        kwargs: Scalar or table kwargs from `ActuatorParsed.component_specs`.
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device for any per-DOF arrays.

    Returns:
        A configured clamping component of type `cls`.

    Raises:
        ValueError: If `cls` has no registered builder.
    """
    builder = _CLAMPING_BUILDERS.get(cls)
    if builder is None:
        raise ValueError(
            f"No builder registered for clamping class {cls.__name__!r}. "
            f"Supported: {sorted(c.__name__ for c in _CLAMPING_BUILDERS)}."
        )
    return builder(kwargs, n, device)
