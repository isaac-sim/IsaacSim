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

"""Parameter and residual capabilities for every SysID simulation backend."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .actuator_compatibility import (
    PHYSICS_BACKEND_NEWTON,
    PHYSICS_BACKEND_PHYSX,
    resolve_simulation_compatibility,
)
from .parameter_types import SysIdParameterEntry, SysIdParameterType
from .run_spec import (
    NEWTON_SOLVER_FEATHERSTONE_DIFF,
    NEWTON_SOLVER_MUJOCO,
    SIMULATION_ENGINE_ISAAC_SIM,
    SIMULATION_ENGINE_NEWTON,
)

RESIDUAL_POSITION = "position"
RESIDUAL_VELOCITY = "velocity"
RESIDUAL_TORQUE = "torque"
RESIDUAL_END_EFFECTOR_POSE = "end_effector_pose"
RESIDUAL_CONTACT_FORCE = "contact_force"

_ALL_PARAMETERS = frozenset(SysIdParameterType)
_STATE_RESIDUALS = frozenset({RESIDUAL_POSITION, RESIDUAL_VELOCITY})
_JOINT_RESIDUALS = _STATE_RESIDUALS | {RESIDUAL_TORQUE}


@dataclass(frozen=True)
class SysIdBackendCapabilities:
    """One backend's parameter and simulated-residual contract.

    Args:
        key: Stable identifier for the backend combination.
        label: Human-readable backend name.
        parameter_types: Parameter types accepted by the backend.
        residual_channels: Simulated residual channels produced by the backend.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_setup.sysid.backend_capabilities import SysIdBackendCapabilities
        >>> from isaacsim.robot_setup.sysid.parameter_types import SysIdParameterType

        >>> capabilities = SysIdBackendCapabilities(
        ...     key="example",
        ...     label="Example backend",
        ...     parameter_types=frozenset({SysIdParameterType.JOINT_DAMPING}),
        ...     residual_channels=frozenset({"position"}),
        ... )
    """

    key: str
    #: Stable identifier for the backend combination.

    label: str
    #: Human-readable backend name.

    parameter_types: frozenset[SysIdParameterType]
    #: Parameter types accepted by the backend.

    residual_channels: frozenset[str]
    #: Simulated residual channels produced by the backend.

    def unsupported_parameter_types(self, entries: list[SysIdParameterEntry]) -> list[str]:
        """Return unsupported selected parameter names.

        Args:
            entries: Parameter entries selected for a run.

        Returns:
            Sorted unsupported parameter names.

        Example:

        .. code-block:: python

            >>> from isaacsim.robot_setup.sysid.parameter_types import SysIdParameterEntry, SysIdParameterType

            >>> entries = [SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, dof_index=0)]
            >>> capabilities.unsupported_parameter_types(entries)  # doctest: +NO_CHECK
            ['joint_friction']
        """
        return sorted({entry.param_type.value for entry in entries if entry.param_type not in self.parameter_types})

    def supports_residual(self, channel: str) -> bool:
        """Return whether the backend produces a simulated residual channel.

        Args:
            channel: Normalized residual channel name.

        Returns:
            Whether the channel is available from this backend.

        Example:

        .. code-block:: python

            >>> capabilities.supports_residual("position")  # doctest: +NO_CHECK
            True
        """
        return str(channel) in self.residual_channels


ISAAC_SIM_PHYSX_CAPABILITIES = SysIdBackendCapabilities(
    key="isaac_sim_physx",
    label="Isaac Sim + PhysX",
    parameter_types=_ALL_PARAMETERS,
    residual_channels=_JOINT_RESIDUALS | {RESIDUAL_END_EFFECTOR_POSE, RESIDUAL_CONTACT_FORCE},
)

ISAAC_SIM_NEWTON_CAPABILITIES = SysIdBackendCapabilities(
    key="isaac_sim_newton",
    label="Isaac-hosted Newton",
    parameter_types=_ALL_PARAMETERS - {SysIdParameterType.JOINT_FRICTION},
    # Newton's hosted tensor view does not currently expose projected joint
    # forces or incoming link forces. Its actuation-force readback is the
    # commanded effort, not a replacement for either measured quantity.
    residual_channels=_STATE_RESIDUALS | {RESIDUAL_END_EFFECTOR_POSE},
)

NEWTON_MUJOCO_CAPABILITIES = SysIdBackendCapabilities(
    key="newton_mujoco",
    label="Standalone Newton MuJoCo",
    parameter_types=frozenset(
        {
            SysIdParameterType.JOINT_FRICTION,
            SysIdParameterType.JOINT_STIFFNESS,
            SysIdParameterType.JOINT_DAMPING,
            SysIdParameterType.JOINT_INTEGRAL_GAIN,
            SysIdParameterType.JOINT_ARMATURE,
            SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS,
            SysIdParameterType.LINK_MASS,
            SysIdParameterType.LINK_COM_OFFSET_X,
            SysIdParameterType.LINK_COM_OFFSET_Y,
            SysIdParameterType.LINK_COM_OFFSET_Z,
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
        }
    ),
    residual_channels=_JOINT_RESIDUALS,
)

NEWTON_FEATHERSTONE_CAPABILITIES = SysIdBackendCapabilities(
    key="newton_featherstone_diff",
    label="Differentiable Newton Featherstone",
    parameter_types=frozenset(
        {
            SysIdParameterType.JOINT_FRICTION,
            SysIdParameterType.JOINT_STIFFNESS,
            SysIdParameterType.JOINT_DAMPING,
            SysIdParameterType.LINK_MASS,
            SysIdParameterType.LINK_COM_OFFSET_X,
            SysIdParameterType.LINK_COM_OFFSET_Y,
            SysIdParameterType.LINK_COM_OFFSET_Z,
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
        }
    ),
    residual_channels=_JOINT_RESIDUALS,
)


def get_backend_capabilities(
    *,
    engine: str,
    physics_backend: str,
    newton_solver: str = NEWTON_SOLVER_MUJOCO,
) -> SysIdBackendCapabilities | None:
    """Resolve capabilities from normalized execution-axis values.

    Args:
        engine: Simulation execution host.
        physics_backend: Physics backend selected in the host.
        newton_solver: Standalone Newton solver selection.

    Returns:
        Matching capabilities, or None for an unknown combination.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_setup.sysid.backend_capabilities import get_backend_capabilities

        >>> capabilities = get_backend_capabilities(engine="isaac_sim", physics_backend="physx")
        >>> print(capabilities.label if capabilities is not None else None)  # doctest: +NO_CHECK
        Isaac Sim + PhysX
    """
    engine = str(engine).strip().lower()
    physics_backend = str(physics_backend).strip().lower()
    newton_solver = str(newton_solver).strip().lower()
    if engine == SIMULATION_ENGINE_ISAAC_SIM:
        if physics_backend == PHYSICS_BACKEND_PHYSX:
            return ISAAC_SIM_PHYSX_CAPABILITIES
        if physics_backend == PHYSICS_BACKEND_NEWTON:
            return ISAAC_SIM_NEWTON_CAPABILITIES
        return None
    if engine == SIMULATION_ENGINE_NEWTON:
        if newton_solver == NEWTON_SOLVER_MUJOCO:
            return NEWTON_MUJOCO_CAPABILITIES
        if newton_solver == NEWTON_SOLVER_FEATHERSTONE_DIFF:
            return NEWTON_FEATHERSTONE_CAPABILITIES
    return None


def resolve_backend_capabilities(simulation: Any) -> SysIdBackendCapabilities | None:
    """Resolve capabilities from a simulation run-spec section.

    Args:
        simulation: Simulation configuration or compatible object.

    Returns:
        Matching capabilities, or None when the execution axes are invalid.

    Example:

    .. code-block:: python

        >>> from types import SimpleNamespace

        >>> from isaacsim.robot_setup.sysid.backend_capabilities import resolve_backend_capabilities

        >>> simulation = SimpleNamespace(engine="isaac_sim", physics_backend="physx")
        >>> capabilities = resolve_backend_capabilities(simulation)
        >>> print(capabilities.label if capabilities is not None else None)  # doctest: +NO_CHECK
        Isaac Sim + PhysX
    """
    compatibility = resolve_simulation_compatibility(simulation)
    return get_backend_capabilities(
        engine=str(getattr(simulation, "engine", "")),
        physics_backend=compatibility.physics_backend,
        newton_solver=compatibility.physics_solver,
    )
