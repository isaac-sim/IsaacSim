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

"""Builders for `newton.actuators` controllers from parsed `NewtonActuator` prims.

One builder per concrete controller class. Each one validates schema values via the
class's own `resolve_arguments` classmethod, then constructs the Newton component with
explicitly typed arguments — per-DOF numeric parameters are placed into length-`n`
`wp.array`s; shared scalar parameters (e.g. neural `model_path`) pass through unchanged.

Callers typically use the `build_controller` dispatcher, which routes on `controller_class`.
"""

from __future__ import annotations

from typing import Any

import warp as wp
from newton.actuators import (
    Controller,
    ControllerNeuralLSTM,
    ControllerNeuralMLP,
    ControllerPD,
    ControllerPID,
)


def _float_array(value: float, n: int, device: wp.Device) -> wp.array:
    """Build a length-`n` `wp.float32` array filled with `value`."""
    return wp.array([float(value)] * n, dtype=wp.float32, device=device)


def build_controller_pd(kwargs: dict[str, Any], n: int, device: wp.Device) -> ControllerPD:
    """Build a `ControllerPD` from scalar schema kwargs.

    Args:
        kwargs: Scalar kwargs from `ActuatorParsed.controller_kwargs`.
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device for the gain arrays.

    Returns:
        A configured `ControllerPD` instance.
    """
    resolved = ControllerPD.resolve_arguments(kwargs)
    return ControllerPD(
        kp=_float_array(resolved["kp"], n, device),
        kd=_float_array(resolved["kd"], n, device),
        const_effort=_float_array(resolved["const_effort"], n, device),
    )


def build_controller_pid(kwargs: dict[str, Any], n: int, device: wp.Device) -> ControllerPID:
    """Build a `ControllerPID` from scalar schema kwargs.

    Args:
        kwargs: Scalar kwargs from `ActuatorParsed.controller_kwargs`.
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device for the gain arrays.

    Returns:
        A configured `ControllerPID` instance.
    """
    resolved = ControllerPID.resolve_arguments(kwargs)
    return ControllerPID(
        kp=_float_array(resolved["kp"], n, device),
        ki=_float_array(resolved["ki"], n, device),
        kd=_float_array(resolved["kd"], n, device),
        integral_max=_float_array(resolved["integral_max"], n, device),
        const_effort=_float_array(resolved["const_effort"], n, device),
    )


def build_controller_neural_mlp(kwargs: dict[str, Any], n: int, device: wp.Device) -> ControllerNeuralMLP:
    """Build a `ControllerNeuralMLP` from schema kwargs.

    The underlying controller shares a single checkpoint across every physical actuator
    represented by the owning `Actuator`; `n` and `device` are accepted for signature
    uniformity with the other builders and may be used in the future if runtime state
    sizing becomes necessary here.

    Args:
        kwargs: Schema kwargs from `ActuatorParsed.controller_kwargs` (expects `model_path`).
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device (unused).

    Returns:
        A configured `ControllerNeuralMLP` instance.
    """
    resolved = ControllerNeuralMLP.resolve_arguments(kwargs)
    return ControllerNeuralMLP(model_path=str(resolved["model_path"]))


def build_controller_neural_lstm(kwargs: dict[str, Any], n: int, device: wp.Device) -> ControllerNeuralLSTM:
    """Build a `ControllerNeuralLSTM` from schema kwargs.

    Args:
        kwargs: Schema kwargs from `ActuatorParsed.controller_kwargs` (expects `model_path`).
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device (unused).

    Returns:
        A configured `ControllerNeuralLSTM` instance.
    """
    resolved = ControllerNeuralLSTM.resolve_arguments(kwargs)
    return ControllerNeuralLSTM(model_path=str(resolved["model_path"]))


_CONTROLLER_BUILDERS = {
    ControllerPD: build_controller_pd,
    ControllerPID: build_controller_pid,
    ControllerNeuralMLP: build_controller_neural_mlp,
    ControllerNeuralLSTM: build_controller_neural_lstm,
}


def build_controller(cls: type[Controller], kwargs: dict[str, Any], n: int, device: wp.Device) -> Controller:
    """Dispatch to the builder for the given controller class.

    Args:
        cls: Concrete controller class from `ActuatorParsed.controller_class`.
        kwargs: Scalar kwargs from `ActuatorParsed.controller_kwargs`.
        n: Number of physical actuators the owning `Actuator` represents.
        device: Warp device for any per-DOF arrays.

    Returns:
        A configured controller of type `cls`.

    Raises:
        ValueError: If `cls` has no registered builder.
    """
    builder = _CONTROLLER_BUILDERS.get(cls)
    if builder is None:
        raise ValueError(
            f"No builder registered for controller class {cls.__name__!r}. "
            f"Supported: {sorted(c.__name__ for c in _CONTROLLER_BUILDERS)}."
        )
    return builder(kwargs, n, device)
