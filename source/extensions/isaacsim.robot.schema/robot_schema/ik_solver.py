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

"""IK solver interface and registry.

Defines the abstract :class:`IKSolver` base class that every IK solver
must implement, and the :class:`IKSolverRegistry` for runtime
registration and lookup of solver implementations.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

import numpy as np

from .math import Transform, VecN, quat_conj, quat_mul

if TYPE_CHECKING:
    from .kinematic_chain import KinematicChain

# ---------------------------------------------------------------------------
# Shared pose-error utility
# ---------------------------------------------------------------------------


def pose_error(Td: Transform, T: Transform) -> VecN:
    """Compute 6-DOF pose error between desired and actual transforms.

    Args:
        Td: Desired end-effector transform.
        T: Actual (current) transform.

    Returns:
        6-vector [rot_x, rot_y, rot_z, pos_x, pos_y, pos_z].

    """
    dp = Td.t - T.t
    dq = quat_mul(Td.q, quat_conj(T.q))
    rot = dq[1:] * np.sign(dq[0]) * 2.0
    return np.concatenate([rot, dp])


# ---------------------------------------------------------------------------
# Abstract solver interface
# ---------------------------------------------------------------------------


class IKSolver(ABC):
    """Abstract base class for inverse-kinematics solvers.

    Subclasses must implement :meth:`solve`.  Solver-specific parameters
    (damping, iteration count, etc.) are passed as keyword arguments.
    """

    @abstractmethod
    def solve(
        self,
        chain: KinematicChain,
        target: Transform,
        q0: VecN | None = None,
        **kwargs: Any,
    ) -> VecN:
        """Solve IK for the given kinematic chain.

        Args:
            chain: Kinematic chain providing joints and FK computation.
            target: Desired end-effector pose in chain-local coordinates.
            q0: Initial joint configuration guess. When None the solver
                must start from the zero configuration.
            **kwargs: Solver-specific parameters.

        Returns:
            Joint values that (approximately) achieve the target.

        """
        ...


# ---------------------------------------------------------------------------
# Solver registry
# ---------------------------------------------------------------------------


class IKSolverRegistry:
    """Global registry of IK solver implementations.

    Solvers register themselves via :meth:`register` (typically at module
    import time).  Consumers obtain instances via :meth:`get`.
    """

    _solvers: dict[str, type[IKSolver]] = {}
    _default: str = ""

    @classmethod
    def register(cls, name: str, solver_cls: type[IKSolver], *, default: bool = False) -> None:
        """Register an IK solver class under the given name.

        Args:
            name: Registry key for the solver.
            solver_cls: Solver class to register.
            default: If True, use as the default solver when name is None.

        """
        cls._solvers[name] = solver_cls
        if default or not cls._default:
            cls._default = name

    @classmethod
    def get(cls, name: str | None = None) -> IKSolver:
        """Return a new instance of the solver registered under the given name.

        Args:
            name: Registry key, or None for the default solver.

        Returns:
            New solver instance.

        Raises:
            KeyError: When the requested solver name is not registered.

        """
        name = name or cls._default
        if name not in cls._solvers:
            raise KeyError(f"IK solver '{name}' not registered. " f"Available: {list(cls._solvers.keys())}")
        return cls._solvers[name]()

    @classmethod
    def available(cls) -> list[str]:
        """Return the names of all registered solvers.

        Returns:
            List of registered solver names.

        """
        return list(cls._solvers.keys())

    @classmethod
    def default_name(cls) -> str:
        """Return the name of the default solver.

        Returns:
            Default solver name.

        """
        return cls._default
