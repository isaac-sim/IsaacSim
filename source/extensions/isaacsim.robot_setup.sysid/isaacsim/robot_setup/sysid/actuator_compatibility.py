# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Backend-neutral simulation and actuator capability resolution.

The run specification keeps the Kit runtime, physics engine, solver, and
actuator implementation as separate axes. This module provides one source of
truth for preflight, runtime construction, reports, and UI capability queries.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

PHYSICS_BACKEND_AUTO = "auto"
PHYSICS_BACKEND_PHYSX = "physx"
PHYSICS_BACKEND_NEWTON = "newton"

ACTUATOR_RUNTIME_AUTO = "auto"
ACTUATOR_RUNTIME_IMPLICIT = "implicit_drive"
ACTUATOR_RUNTIME_EXPLICIT = "newton_explicit"
ACTUATOR_RUNTIME_MIXED = "mixed"

EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY = "authored_only"
EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED = "promote_selected"

ACTUATOR_OWNER_IMPLICIT = "implicit"
ACTUATOR_OWNER_EXPLICIT = "explicit"

ACTUATOR_DELAY_PARAMETER = "actuator_command_delay_seconds"


@dataclass(frozen=True)
class ResolvedSimulationCompatibility:
    """Normalized execution axes and advertised actuator features."""

    runtime: str
    physics_backend: str
    physics_solver: str
    actuator_runtime: str
    supports_pd: bool
    supports_pid: bool
    supports_delay: bool
    supports_effort_clamp: bool
    supports_neural_execution: bool
    supports_graph_capture: bool

    @property
    def uses_explicit_actuators(self) -> bool:
        """Return whether any DOF uses an explicit actuator."""
        return self.actuator_runtime in (
            ACTUATOR_RUNTIME_EXPLICIT,
            ACTUATOR_RUNTIME_MIXED,
        )


@dataclass(frozen=True)
class ActuatorDofPlan:
    """Resolved control ownership for one trajectory DOF."""

    dof_index: int
    owner: str
    reason: str

    @property
    def explicit(self) -> bool:
        """Return whether this DOF is owned by an explicit actuator."""
        return self.owner == ACTUATOR_OWNER_EXPLICIT


def resolve_simulation_compatibility(
    simulation: Any,
) -> ResolvedSimulationCompatibility:
    """Resolve the simulation execution axes.

    Args:
        simulation: Simulation configuration or compatible object.

    Returns:
        The normalized runtime, physics, and actuator capabilities.
    """
    # Normalized the same way as `get_backend_capabilities` so the two never
    # disagree about which engine a given run-spec selects.
    engine = str(getattr(simulation, "engine", "isaac_sim") or "isaac_sim").strip().lower()
    runtime = "standalone" if engine == "newton" else "isaac_sim"

    requested_backend = (
        str(getattr(simulation, "physics_backend", PHYSICS_BACKEND_AUTO) or PHYSICS_BACKEND_AUTO).strip().lower()
    )
    if requested_backend == PHYSICS_BACKEND_AUTO:
        physics_backend = PHYSICS_BACKEND_NEWTON if runtime == "standalone" else PHYSICS_BACKEND_PHYSX
    else:
        physics_backend = requested_backend

    requested_actuators = (
        str(getattr(simulation, "actuator_runtime", ACTUATOR_RUNTIME_AUTO) or ACTUATOR_RUNTIME_AUTO).strip().lower()
    )
    solver = str(getattr(getattr(simulation, "newton", None), "solver", "mujoco") or "mujoco").strip().lower()
    if requested_actuators == ACTUATOR_RUNTIME_AUTO:
        if runtime == "standalone" and solver == "featherstone_diff":
            # The differentiable bridge evaluates mapped-joint PD effort in an
            # application-side kernel.  It is an explicit PD pipeline and must
            # not be advertised as the mixed MuJoCo path.
            actuator_runtime = ACTUATOR_RUNTIME_EXPLICIT
        elif runtime == "standalone":
            # Standalone MuJoCo promotes only DOFs that need stateful actuator
            # behavior; ordinary drive-default joints remain implicit.
            actuator_runtime = ACTUATOR_RUNTIME_MIXED
        else:
            actuator_runtime = ACTUATOR_RUNTIME_IMPLICIT
    else:
        actuator_runtime = requested_actuators

    explicit = actuator_runtime in (ACTUATOR_RUNTIME_EXPLICIT, ACTUATOR_RUNTIME_MIXED)
    differentiable = runtime == "standalone" and solver == "featherstone_diff"
    return ResolvedSimulationCompatibility(
        runtime=runtime,
        physics_backend=physics_backend,
        physics_solver=solver,
        actuator_runtime=actuator_runtime,
        supports_pd=True,
        supports_pid=explicit and not differentiable,
        supports_delay=explicit and not differentiable,
        supports_effort_clamp=explicit and not differentiable,
        supports_neural_execution=explicit and not differentiable,
        supports_graph_capture=(physics_backend == PHYSICS_BACKEND_NEWTON and solver == "mujoco"),
    )


def parameter_requires_explicit_actuator(parameter_type: object) -> bool:
    """Return whether an implicit USD drive cannot express a parameter.

    Args:
        parameter_type: Enum-like or serialized parameter type.

    Returns:
        Whether the parameter requires an explicit actuator.
    """
    value = getattr(parameter_type, "value", parameter_type)
    return str(value) in {
        "joint_integral_gain",
        ACTUATOR_DELAY_PARAMETER,
    }


def is_actuator_delay_parameter(parameter_type: object) -> bool:
    """Return whether a value denotes physical actuator delay.

    Args:
        parameter_type: Enum-like or serialized parameter type.

    Returns:
        Whether the value identifies an actuator delay parameter.
    """
    value = getattr(parameter_type, "value", parameter_type)
    return str(value) == ACTUATOR_DELAY_PARAMETER


def resolve_actuator_dof_plan(
    actuator_runtime: str,
    num_dof: int,
    *,
    authored_explicit_dofs: Iterable[int] = (),
    parameter_entries: Iterable[object] = (),
) -> tuple[ActuatorDofPlan, ...]:
    """Resolve the single control owner for every trajectory DOF.

    ``newton_explicit`` owns every mapped DOF, ``implicit_drive`` owns none,
    and ``mixed`` owns authored explicit DOFs plus DOFs selected for stateful
    actuator parameters such as PID integral gain or physical command delay.

    Args:
        actuator_runtime: Resolved actuator runtime identifier.
        num_dof: Number of trajectory degrees of freedom.
        authored_explicit_dofs: DOF indices with authored explicit actuators.
        parameter_entries: Parameter entries selected for the solve.

    Returns:
        One control-ownership plan for each trajectory DOF.
    """
    count = max(0, int(num_dof))
    runtime = str(actuator_runtime).lower()
    if runtime not in {
        ACTUATOR_RUNTIME_IMPLICIT,
        ACTUATOR_RUNTIME_EXPLICIT,
        ACTUATOR_RUNTIME_MIXED,
    }:
        raise ValueError(f"Actuator runtime must be resolved before planning; got {actuator_runtime!r}.")

    explicit: set[int] = set()
    reasons: dict[int, str] = {}
    if runtime == ACTUATOR_RUNTIME_EXPLICIT:
        explicit.update(range(count))
        reasons.update(dict.fromkeys(range(count), "explicit_runtime"))
    elif runtime == ACTUATOR_RUNTIME_MIXED:
        for value in authored_explicit_dofs:
            index = int(value)
            if 0 <= index < count:
                explicit.add(index)
                reasons[index] = "authored_explicit_actuator"
        for entry in parameter_entries:
            ptype = getattr(entry, "param_type", "")
            pvalue = getattr(ptype, "value", ptype)
            if not parameter_requires_explicit_actuator(str(pvalue)):
                continue
            index = int(getattr(entry, "dof_index", -1))
            indices = range(count) if index < 0 else (index,)
            for selected in indices:
                if 0 <= int(selected) < count:
                    explicit.add(int(selected))
                    reasons[int(selected)] = f"parameter:{pvalue}"

    return tuple(
        ActuatorDofPlan(
            dof_index=index,
            owner=(ACTUATOR_OWNER_EXPLICIT if index in explicit else ACTUATOR_OWNER_IMPLICIT),
            reason=reasons.get(
                index,
                ("implicit_runtime" if runtime == ACTUATOR_RUNTIME_IMPLICIT else "mixed_default"),
            ),
        )
        for index in range(count)
    )
