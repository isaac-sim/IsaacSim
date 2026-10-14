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

"""Correct Featherstone mass assembly for OVNewton's independently ordered bodies.

Newton 1.6.0 indexes body inertia by joint index in its private mass kernel.
OVNewton preserves its own body ordering, so those indices need not coincide.
Keep the upstream step implementation and replace only that kernel launch in
this solver's private globals. Remove this adapter once the supported Newton
cohort consistently follows joint_child during mass assembly.
"""

from __future__ import annotations

import inspect
from types import FunctionType
from typing import Any

import warp as wp
from newton.solvers import SolverFeatherstone


@wp.kernel
def _assemble_mass(
    starts: wp.array(dtype=int),
    ends: wp.array(dtype=int),
    offsets: wp.array(dtype=int),
    inertias: wp.array(dtype=wp.spatial_matrix),
    children: wp.array(dtype=int),
    mass: wp.array(dtype=float),
) -> None:
    articulation = wp.tid()
    start = starts[articulation]
    count = ends[articulation] - start
    stride = count * 6
    offset = offsets[articulation]
    for link in range(count):
        inertia = inertias[children[start + link]]
        for row in range(6):
            for column in range(6):
                mass[offset + (link * 6 + row) * stride + link * 6 + column] = inertia[row, column]


class _WarpMassLaunch:
    """Forward Warp operations while correcting one solver-local kernel launch."""

    def __init__(self, warp_module: Any, mass_kernel: Any, children: wp.array) -> None:
        self._warp = warp_module
        self._mass_kernel = mass_kernel
        self._children = children

    def __getattr__(self, name: str) -> Any:
        return getattr(self._warp, name)

    def launch(self, kernel: Any, *args: Any, **kwargs: Any) -> Any:
        """Preserve ordinary launches and add the body map to mass assembly."""
        if kernel is self._mass_kernel:
            inputs = kwargs.get("inputs")
            if args or inputs is None or len(inputs) != 4 or len(kwargs.get("outputs", ())) != 1:
                raise RuntimeError("Unsupported Newton Featherstone mass launch; update the body-order adapter")
            kernel = _assemble_mass
            kwargs["inputs"] = [*inputs, self._children]
        return self._warp.launch(kernel, *args, **kwargs)


class FeatherstoneSolver(SolverFeatherstone):
    """Preserve upstream integration with mass blocks ordered by child bodies."""

    def __init__(self, model: Any, **kwargs: Any) -> None:
        upstream_step = SolverFeatherstone.step
        mass_kernel = upstream_step.__globals__.get("eval_rigid_mass")
        expected_parameters = (
            "articulation_start",
            "articulation_end",
            "articulation_M_start",
            "body_I_s",
            "M",
        )
        if (
            mass_kernel is None
            or tuple(inspect.signature(mass_kernel.func).parameters) != expected_parameters
            or "eval_rigid_mass" not in upstream_step.__code__.co_names
        ):
            raise RuntimeError("Unsupported Newton Featherstone mass interface; update the body-order adapter")
        super().__init__(model, **kwargs)
        deterministic = kwargs.get("deterministic")
        self._set_module_options(
            {
                "deterministic": wp.config.deterministic if deterministic is None else deterministic,
                "deterministic_max_records": 0,
            },
            module=_assemble_mass.module,
        )
        step_globals = dict(upstream_step.__globals__)
        step_globals["wp"] = _WarpMassLaunch(step_globals["wp"], mass_kernel, model.joint_child)
        self._step_with_body_order = FunctionType(
            upstream_step.__code__,
            step_globals,
            upstream_step.__name__,
            upstream_step.__defaults__,
            upstream_step.__closure__,
        )
        self._step_with_body_order.__kwdefaults__ = upstream_step.__kwdefaults__

    def step(self, state_in: Any, state_out: Any, control: Any, contacts: Any, dt: float) -> None:
        """Run the upstream step with the per-model mass assembly correction."""
        self._step_with_body_order(self, state_in, state_out, control, contacts, dt)
