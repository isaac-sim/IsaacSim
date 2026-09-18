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

"""Convert the per-DOF state of an articulation into a joint state reading.

The articulation is on a meter-and-kilogram stage, so the resulting values are already SI.
"""

from __future__ import annotations

import enum
from collections.abc import Sequence
from dataclasses import dataclass

import warp as wp

#: Stage meters per linear unit, which a reading carries as the shipped one does. This module
#: assumes a meter stage, so it is always one and a consumer has nothing to scale.
METERS_PER_UNIT = 1.0


class DofType(enum.IntEnum):
    """The two kinds of degree of freedom.

    Values match the shipped reading's ``uint8`` ``dof_types``, so a code is ``DofType(code)`` and a USD name is
    ``DofType[name.upper()]``.
    """

    #: About an axis. Positions are radians and efforts are newton-meters.
    ROTATION = 0
    #: Along an axis. Positions are meters and efforts are newtons.
    TRANSLATION = 1


@dataclass(frozen=True)
class Dof:
    """One degree of freedom of an articulation, in the order the engine reports it.

    Args:
        name: DOF name, as the articulation reports it.
        type: A `DofType`, or the code of one.

    Raises:
        ValueError: If the type is neither.

    """

    name: str
    type: DofType

    def __post_init__(self) -> None:
        """Convert the type, which also rejects anything that is not a DOF type."""
        object.__setattr__(self, "type", DofType(self.type))


@dataclass
class ArticulationState:
    """Raw per-DOF state of an articulation, as an entity's DOF getters return it.

    Args:
        positions: DOF positions, radians or meters.
        velocities: DOF velocities, per second.
        efforts: Measured joint efforts, or ``None`` when the backend does not report them, which
            the reading reports as zeros, as the shipped sensor does.

    """

    positions: wp.array
    velocities: wp.array
    efforts: wp.array | None = None


@dataclass
class JointStateReading:
    """One reading, carrying the fields the shipped ``JointStateSensorReading`` carries.

    Args:
        is_valid: Whether the reading holds data. Always true here, since a reading is made from a
            state rather than accumulated, and the shipped sensor's own invalid reading is the one
            it hands back before a simulation is running.
        time: Simulation time of the state, in seconds.
        dof_names: DOF names, in the reported order.
        positions: Positions, rad or m.
        velocities: Velocities, rad/s or m/s.
        efforts: Measured joint efforts, N*m or N. Zeros when the backend reports none.
        dof_types: Per-DOF `DofType`, whose members are the ``uint8`` codes the shipped reading
            carries.
        stage_meters_per_unit: Stage meters per linear unit, always ``1.0`` for the meter stage this
            module assumes. A consumer scales a translation DOF by it and leaves a rotation alone.

    """

    is_valid: bool
    time: float
    dof_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    dof_types: tuple[DofType, ...]
    stage_meters_per_unit: float


class JointStateSensor:
    """Reports an articulation's degrees of freedom untouched, as the shipped sensor does.

    Args:
        dofs: Every DOF of the articulation, in the order the raw state arrays are indexed.

    Raises:
        ValueError: If `dofs` is empty.

    """

    def __init__(self, dofs: Sequence[Dof]) -> None:
        if len(dofs) == 0:
            raise ValueError("An articulation with no DOFs has no joint state to report.")
        self._dof_count = len(dofs)
        self._names = tuple(dof.name for dof in dofs)
        self._types = tuple(dof.type for dof in dofs)

    @property
    def dof_names(self) -> tuple[str, ...]:
        """DOF names in the order readings report them."""
        return self._names

    @property
    def dof_types(self) -> tuple[DofType, ...]:
        """Per-DOF `DofType`, in the order readings report them."""
        return self._types

    def _check_length(self, name: str, values: Sequence[float]) -> None:
        """Check that a raw state array holds one value per DOF.

        Args:
            name: Field name, for the error message.
            values: Array to check.

        Raises:
            ValueError: If the length does not match the articulation's DOF count.

        """
        if len(values) != self._dof_count:
            raise ValueError(f"`{name}` holds {len(values)} values, expected {self._dof_count}, one per DOF.")

    def read(self, time: float, state: ArticulationState) -> JointStateReading:
        """Select and label one raw articulation state.

        Args:
            time: Simulation time of the state, in seconds.
            state: Raw per-DOF state, in stage units and articulation order.

        Returns:
            The reading, over the DOFs this sensor reports.

        Raises:
            ValueError: If an array does not hold one value per DOF of the articulation.

        """
        self._check_length("positions", state.positions)
        self._check_length("velocities", state.velocities)
        if state.efforts is not None:
            self._check_length("efforts", state.efforts)

        # A Warp array cannot be read an item at a time, so each is copied to the host whole.
        efforts = state.efforts
        return JointStateReading(
            is_valid=True,
            time=float(time),
            dof_names=self._names,
            positions=tuple(state.positions.list()),
            velocities=tuple(state.velocities.list()),
            # Zeros when the backend reports none, which is what the shipped sensor substitutes.
            efforts=(0.0,) * self._dof_count if efforts is None else tuple(efforts.list()),
            dof_types=self._types,
            stage_meters_per_unit=METERS_PER_UNIT,
        )
