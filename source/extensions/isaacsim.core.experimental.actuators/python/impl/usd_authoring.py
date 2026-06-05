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

"""USD authoring utilities for Newton actuators.

This module provides dataclass config types and a single entry-point function,
`add_actuator`, for authoring `NewtonActuator` prims into an articulation's USD
subtree.  Every component type supported by the Newton actuators parser has a
corresponding config class so that valid combinations can be expressed in pure
Python and committed to a stage without constructing a live `ArticulationActuators`.

Typical usage::

    from isaacsim.core.experimental.actuators import (
        add_actuator,
        PDControlConfig,
        MaxEffortClampingConfig,
        DelayConfig,
    )

    add_actuator(
        "/World/Robot",
        target_names="RevoluteJoint",
        name="elbow_actuator",
        controller=PDControlConfig(kp=500.0, kd=50.0),
        clamping=MaxEffortClampingConfig(max_effort=100.0),
        delay=DelayConfig(delay_steps=2),
    )
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import isaacsim.core.experimental.utils.stage as stage_utils
from pxr import Sdf, Usd, UsdPhysics

__all__ = [
    "PDControlConfig",
    "PIDControlConfig",
    "NeuralControlConfig",
    "MaxEffortClampingConfig",
    "DCMotorClampingConfig",
    "PositionBasedClampingConfig",
    "DelayConfig",
    "add_actuator",
]

# ──── Controller config dataclasses ──────────────────────────────────────────


@dataclass
class PDControlConfig:
    """Configuration for a PD position-velocity controller.

    Corresponds to `NewtonPDControlAPI` in the Newton USD schema.

    Args:
        kp: Proportional (position) gain.
        kd: Derivative (velocity) gain.
        const_effort: Constant bias effort added to the controller output.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import PDControlConfig

        >>> cfg = PDControlConfig(kp=500.0, kd=50.0)
    """

    kp: float
    kd: float
    const_effort: float = 0.0


@dataclass
class PIDControlConfig:
    """Configuration for a PID position-velocity controller.

    Corresponds to `NewtonPIDControlAPI` in the Newton USD schema.

    Args:
        kp: Proportional (position) gain.
        ki: Integral gain.
        kd: Derivative (velocity) gain.
        integral_max: Anti-windup clamp applied symmetrically to the integral term.
            Pass `math.inf` to disable anti-windup.
        const_effort: Constant bias effort added to the controller output.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import PIDControlConfig

        >>> cfg = PIDControlConfig(kp=500.0, ki=10.0, kd=50.0, integral_max=200.0)
    """

    kp: float
    ki: float
    kd: float
    integral_max: float = math.inf
    const_effort: float = 0.0


@dataclass
class NeuralControlConfig:
    """Configuration for a neural-network controller (MLP or LSTM).

    The concrete class (``ControllerNeuralMLP`` or ``ControllerNeuralLSTM``) is
    selected at parse time by inspecting the ``model_type`` metadata stored inside
    the checkpoint at `model_path`.

    Corresponds to `NewtonNeuralControlAPI` in the Newton USD schema.

    Args:
        model_path: Path to the pre-trained model checkpoint (e.g. a TorchScript
            ``.pt`` file).  The file must contain a ``model_type`` metadata entry
            with value ``"mlp"`` or ``"lstm"``.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import NeuralControlConfig

        >>> cfg = NeuralControlConfig(model_path="/path/to/policy.pt")
    """

    model_path: str


# ──── Clamping config dataclasses ─────────────────────────────────────────────


@dataclass
class MaxEffortClampingConfig:
    """Configuration for symmetric max-effort clamping.

    Clamps actuator output to the range ``[-max_effort, +max_effort]``.

    Corresponds to `NewtonMaxEffortClampingAPI` in the Newton USD schema.

    Args:
        max_effort: Maximum output effort limit (force or torque).

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import MaxEffortClampingConfig

        >>> cfg = MaxEffortClampingConfig(max_effort=100.0)
    """

    max_effort: float


@dataclass
class DCMotorClampingConfig:
    """Configuration for DC motor four-quadrant effort clamping.

    Models a motor whose available output effort decreases linearly with joint velocity:

        ``effort_max(vel) = min(saturation_effort * (1 - vel / velocity_limit), max_motor_effort)``

    Corresponds to `NewtonDCMotorClampingAPI` in the Newton USD schema.

    Args:
        saturation_effort: Peak motor effort at stall (zero velocity).
        velocity_limit: Maximum no-load joint velocity at which the motor produces zero effort
            in the direction of motion.  Pass `math.inf` to disable the velocity-dependent
            saturation.
        max_motor_effort: Hard upper bound on the effort-speed envelope.  Pass `math.inf` to
            leave uncapped.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import DCMotorClampingConfig

        >>> cfg = DCMotorClampingConfig(saturation_effort=120.0, velocity_limit=10.0, max_motor_effort=100.0)
    """

    saturation_effort: float
    velocity_limit: float
    max_motor_effort: float


@dataclass
class PositionBasedClampingConfig:
    """Configuration for position-dependent effort clamping via a lookup table.

    The maximum output effort is interpolated from a table of ``(position, effort)``
    pairs.  Positions outside the table range are clamped to the nearest endpoint.

    Corresponds to `NewtonPositionBasedClampingAPI` in the Newton USD schema.

    Args:
        lookup_positions: Sorted (monotonically non-decreasing) joint positions
            [rad or m] for the lookup table.  Must be the same length as
            `lookup_efforts` and non-empty.
        lookup_efforts: Non-negative maximum output efforts corresponding to each
            entry in `lookup_positions`.  Must be the same length as
            `lookup_positions`.

    Raises:
        ValueError: If `lookup_positions` is empty.
        ValueError: If `lookup_positions` and `lookup_efforts` have different lengths.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import PositionBasedClampingConfig

        >>> cfg = PositionBasedClampingConfig(
        ...     lookup_positions=[0.0, 0.5, 1.0],
        ...     lookup_efforts=[100.0, 80.0, 50.0],
        ... )
    """

    lookup_positions: list[float]
    lookup_efforts: list[float]


# ──── Delay config dataclass ──────────────────────────────────────────────────


@dataclass
class DelayConfig:
    """Configuration for command-input delay.

    Delays commanded position and velocity targets by a fixed number of physics
    timesteps to model communication or processing lag.

    Corresponds to `NewtonActuatorDelayAPI` in the Newton USD schema.

    Args:
        delay_steps: Number of physics timesteps to delay command inputs.  A value
            of ``0`` disables delay.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import DelayConfig

        >>> cfg = DelayConfig(delay_steps=3)
    """

    delay_steps: int


# ──── Private helpers ─────────────────────────────────────────────────────────


def _collect_joint_paths(stage: Usd.Stage, articulation_root: Sdf.Path) -> dict[str, list[Sdf.Path]]:
    """Map each joint name (last path segment) to all matching full `Sdf.Path`s.

    Args:
        stage: Active USD stage.
        articulation_root: Root path of the articulation to search within.

    Returns:
        Mapping from joint name to a list of `Sdf.Path`s under `articulation_root`.
    """
    root_prim = stage.GetPrimAtPath(articulation_root)
    if not root_prim.IsValid():
        return {}
    result: dict[str, list[Sdf.Path]] = {}
    for prim in Usd.PrimRange(root_prim):
        if prim.GetPath() == articulation_root:
            continue
        if UsdPhysics.Joint(prim):
            seg = prim.GetPath().name
            result.setdefault(seg, []).append(prim.GetPath())
    return result


def _resolve_target_paths(
    stage: Usd.Stage,
    articulation_root: Sdf.Path,
    target_names: list[str],
) -> list[Sdf.Path]:
    """Resolve DOF names to absolute `Sdf.Path`s, raising on unknown or ambiguous names.

    Args:
        stage: Active USD stage.
        articulation_root: Root path of the articulation to search within.
        target_names: List of joint names (last path segment) to resolve.

    Returns:
        Ordered list of resolved `Sdf.Path`s, one per entry in `target_names`.

    Raises:
        ValueError: If any name has no matching joint under `articulation_root`.
        ValueError: If any name matches multiple joints (ambiguous).
    """
    joint_map = _collect_joint_paths(stage, articulation_root)
    resolved: list[Sdf.Path] = []
    for target_name in target_names:
        matches = joint_map.get(target_name, [])
        if not matches:
            available = sorted(joint_map)
            raise ValueError(
                f"Target name {target_name!r} does not match any joint DOF under "
                f"{str(articulation_root)!r}. Available joint names: {available}."
            )
        if len(matches) > 1:
            raise ValueError(
                f"Target name {target_name!r} is ambiguous under {str(articulation_root)!r}: "
                f"matches {[str(p) for p in matches]}. "
                "Use the full USD path string directly to disambiguate."
            )
        resolved.append(matches[0])
    return resolved


def _apply_controller(
    prim: Usd.Prim,
    cfg: PDControlConfig | PIDControlConfig | NeuralControlConfig,
) -> None:
    """Apply the controller API schema and author all configured attributes onto `prim`.

    Args:
        prim: The `NewtonActuator` prim to modify.
        cfg: Controller configuration to apply.

    Raises:
        TypeError: If `cfg` is not a recognised controller config type.
    """
    if isinstance(cfg, PDControlConfig):
        prim.AddAppliedSchema("NewtonPDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(cfg.kp)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(cfg.kd)
        prim.CreateAttribute("newton:constEffort", Sdf.ValueTypeNames.Float).Set(cfg.const_effort)
    elif isinstance(cfg, PIDControlConfig):
        prim.AddAppliedSchema("NewtonPIDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(cfg.kp)
        prim.CreateAttribute("newton:ki", Sdf.ValueTypeNames.Float).Set(cfg.ki)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(cfg.kd)
        prim.CreateAttribute("newton:integralMax", Sdf.ValueTypeNames.Float).Set(cfg.integral_max)
        prim.CreateAttribute("newton:constEffort", Sdf.ValueTypeNames.Float).Set(cfg.const_effort)
    elif isinstance(cfg, NeuralControlConfig):
        prim.AddAppliedSchema("NewtonNeuralControlAPI")
        prim.CreateAttribute("newton:modelPath", Sdf.ValueTypeNames.Asset).Set(cfg.model_path)
    else:
        raise TypeError(f"Unknown controller config type {type(cfg).__name__!r}.")


def _apply_clamping(
    prim: Usd.Prim,
    cfg: MaxEffortClampingConfig | DCMotorClampingConfig | PositionBasedClampingConfig,
) -> None:
    """Apply the clamping API schema and author all configured attributes onto `prim`.

    Args:
        prim: The `NewtonActuator` prim to modify.
        cfg: Clamping configuration to apply.

    Raises:
        ValueError: If `cfg` is `PositionBasedClampingConfig` and the lookup table
            data is invalid (empty or length mismatch).
        TypeError: If `cfg` is not a recognised clamping config type.
    """
    if isinstance(cfg, MaxEffortClampingConfig):
        prim.AddAppliedSchema("NewtonMaxEffortClampingAPI")
        prim.CreateAttribute("newton:maxEffort", Sdf.ValueTypeNames.Float).Set(cfg.max_effort)
    elif isinstance(cfg, DCMotorClampingConfig):
        prim.AddAppliedSchema("NewtonDCMotorClampingAPI")
        prim.CreateAttribute("newton:saturationEffort", Sdf.ValueTypeNames.Float).Set(cfg.saturation_effort)
        prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(cfg.velocity_limit)
        prim.CreateAttribute("newton:maxMotorEffort", Sdf.ValueTypeNames.Float).Set(cfg.max_motor_effort)
    elif isinstance(cfg, PositionBasedClampingConfig):
        if not cfg.lookup_positions:
            raise ValueError("PositionBasedClampingConfig: `lookup_positions` must not be empty.")
        if len(cfg.lookup_positions) != len(cfg.lookup_efforts):
            raise ValueError(
                f"PositionBasedClampingConfig: `lookup_positions` length ({len(cfg.lookup_positions)}) "
                f"must match `lookup_efforts` length ({len(cfg.lookup_efforts)})."
            )
        prim.AddAppliedSchema("NewtonPositionBasedClampingAPI")
        prim.CreateAttribute("newton:lookupPositions", Sdf.ValueTypeNames.FloatArray).Set(cfg.lookup_positions)
        prim.CreateAttribute("newton:lookupEfforts", Sdf.ValueTypeNames.FloatArray).Set(cfg.lookup_efforts)
    else:
        raise TypeError(f"Unknown clamping config type {type(cfg).__name__!r}.")


def _apply_delay(prim: Usd.Prim, cfg: DelayConfig) -> None:
    """Apply the delay API schema and author the `delay_steps` attribute onto `prim`.

    Args:
        prim: The `NewtonActuator` prim to modify.
        cfg: Delay configuration to apply.
    """
    prim.AddAppliedSchema("NewtonActuatorDelayAPI")
    prim.CreateAttribute("newton:delaySteps", Sdf.ValueTypeNames.Int).Set(cfg.delay_steps)


# ──── Public API ──────────────────────────────────────────────────────────────


def add_actuator(
    articulation_root: str | Sdf.Path,
    target_names: str | list[str],
    name: str,
    controller: PDControlConfig | PIDControlConfig | NeuralControlConfig,
    *,
    clamping: list[MaxEffortClampingConfig | DCMotorClampingConfig | PositionBasedClampingConfig] | None = None,
    delay: DelayConfig | None = None,
    overwrite_existing: bool = False,
) -> Usd.Prim:
    """Author a `NewtonActuator` prim targeting one or more DOFs of an articulation.

    The prim is created at ``{articulation_root}/Actuators/{name}`` on the current
    USD stage.  All provided config objects are translated to the corresponding Newton
    USD API schemas and their attributes are authored immediately.

    Target DOFs are identified by name — the last segment of their full USD path
    (e.g. ``"RevoluteJoint"`` for ``/World/Robot/Arm/RevoluteJoint``).  The stage is
    traversed at authoring time to validate that each name resolves to exactly one
    joint prim under `articulation_root`.

    Args:
        articulation_root: Root USD path of the articulation (e.g. ``"/World/Robot"``).
        target_names: Name or list of names of the target joint DOFs (last path segment).
        name: Name for the `NewtonActuator` prim, used as the final path segment under
            ``{articulation_root}/Actuators/``.
        controller: Controller configuration.  Must be one of `PDControlConfig`,
            `PIDControlConfig`, or `NeuralControlConfig`.
        clamping: Optional list of clamping configurations.  Each entry must be one of
            `MaxEffortClampingConfig`, `DCMotorClampingConfig`, or
            `PositionBasedClampingConfig`.  Each clamping type may appear at most once.
        delay: Optional command-input delay configuration.
        overwrite_existing: When ``True``, silently replace any existing prim at the
            computed path; otherwise raise `ValueError` if a prim already exists there.

    Returns:
        The newly created `NewtonActuator` `Usd.Prim`.

    Raises:
        ValueError: If `overwrite_existing` is ``False`` and a prim already exists at
            the computed path.
        ValueError: If any entry in `target_names` does not match exactly one joint
            DOF under `articulation_root`.
        ValueError: If `clamping` contains duplicate config types.
        ValueError: If `clamping` contains a `PositionBasedClampingConfig` with invalid
            lookup table data.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import (
        ...     add_actuator,
        ...     PDControlConfig,
        ...     MaxEffortClampingConfig,
        ...     DCMotorClampingConfig,
        ...     DelayConfig,
        ... )

        >>> prim = add_actuator(  # doctest: +NO_CHECK
        ...     "/World/Robot",
        ...     target_names="RevoluteJoint",
        ...     name="elbow_actuator",
        ...     controller=PDControlConfig(kp=500.0, kd=50.0),
        ...     clamping=[
        ...         MaxEffortClampingConfig(max_effort=100.0),
        ...         DCMotorClampingConfig(saturation_effort=120.0, velocity_limit=10.0, max_motor_effort=100.0),
        ...     ],
        ...     delay=DelayConfig(delay_steps=2),
        ... )
    """
    root = Sdf.Path(str(articulation_root))
    names = [target_names] if isinstance(target_names, str) else list(target_names)
    prim_path = root.AppendPath(f"Actuators/{name}")

    stage = stage_utils.get_current_stage(backend="usd")

    existing = stage.GetPrimAtPath(prim_path)
    if existing.IsValid():
        if not overwrite_existing:
            raise ValueError(
                f"A prim already exists at {str(prim_path)!r}. " "Pass overwrite_existing=True to replace it."
            )
        stage.RemovePrim(prim_path)

    if clamping:
        seen_types: set[type] = set()
        for c in clamping:
            if type(c) in seen_types:
                raise ValueError(
                    f"Duplicate clamping config type {type(c).__name__!r}. "
                    "Each clamping type may appear at most once per actuator."
                )
            seen_types.add(type(c))

    target_paths = _resolve_target_paths(stage, root, names)

    prim = stage.DefinePrim(str(prim_path), "NewtonActuator")
    _apply_controller(prim, controller)
    for clamp_cfg in clamping or []:
        _apply_clamping(prim, clamp_cfg)
    if delay is not None:
        _apply_delay(prim, delay)

    prim.CreateRelationship("newton:targets").SetTargets(target_paths)

    return prim
