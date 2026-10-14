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

"""Autograd-preserving theta decoding for differentiable simulation bridges.

Differentiable sibling of :func:`parameter_apply.decode_theta_to_apply_state`. The
numpy decoder extracts scalars via ``theta_row[i].item()``, which severs the torch
autograd graph; this module keeps every decoded quantity as a torch tensor whose
``grad_fn`` traces back to the theta batch, so gradient-based optimizers can
backpropagate rollout costs through the parameter mapping.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from .backend_capabilities import NEWTON_FEATHERSTONE_CAPABILITIES
from .inertia_param import LOG_CHOLESKY_DIM, log_cholesky_to_inertia_matrix_torch
from .parameter_types import GLOBAL_LINK_INDEX, SysIdParameterEntry, SysIdParameterType

_COM_AXIS_BY_TYPE = {
    SysIdParameterType.LINK_COM_OFFSET_X: 0,
    SysIdParameterType.LINK_COM_OFFSET_Y: 1,
    SysIdParameterType.LINK_COM_OFFSET_Z: 2,
}

_DIFFERENTIABLE_PARAMETER_TYPES = NEWTON_FEATHERSTONE_CAPABILITIES.parameter_types


@dataclass
class ThetaDecodeBaselines:
    """Baseline physical values the theta entries scale or offset.

    Args:
        stiffness: Shape ``(N,)`` baseline joint drive stiffness (already floored to > 0).
        damping: Shape ``(N,)`` baseline joint drive damping.
        friction: Shape ``(N,)`` baseline joint friction torque magnitude.
        link_mass: Shape ``(L,)`` baseline link masses.
        link_com: Shape ``(L, 3)`` baseline link center-of-mass offsets.
        link_inertia: Shape ``(L, 3, 3)`` baseline link inertia tensors.
        link_inertia_lc: Shape ``(L, 6)`` Log-Cholesky encoding of ``link_inertia``.
    """

    stiffness: torch.Tensor
    damping: torch.Tensor
    friction: torch.Tensor
    link_mass: torch.Tensor
    link_com: torch.Tensor
    link_inertia: torch.Tensor
    link_inertia_lc: torch.Tensor

    @property
    def num_dof(self) -> int:
        """Number of joint DOFs covered by the joint baselines."""
        return int(self.stiffness.shape[0])

    @property
    def num_links(self) -> int:
        """Number of links covered by the link baselines."""
        return int(self.link_mass.shape[0])


@dataclass
class ThetaBatchTensors:
    """Decoded absolute parameter tensors, graph-connected to the theta batch."""

    ke: torch.Tensor  #: Shape ``(B, N)`` joint drive stiffness.
    kd: torch.Tensor  #: Shape ``(B, N)`` joint drive damping.
    friction: torch.Tensor  #: Shape ``(B, N)`` joint friction torque magnitude.
    mass: torch.Tensor  #: Shape ``(B, L)`` link masses.
    com: torch.Tensor  #: Shape ``(B, L, 3)`` link center-of-mass offsets.
    inertia: torch.Tensor  #: Shape ``(B, L, 3, 3)`` link inertia tensors.


def validate_differentiable_entries(
    param_entries: list[SysIdParameterEntry],
) -> list[str]:
    """Return the (sorted) parameter type values that the differentiable decode cannot handle.

    Args:
        param_entries: Parameter metadata to inspect.

    Returns:
        Unsupported parameter type names, empty when every row is differentiable.
    """
    return sorted(
        {entry.param_type.value for entry in param_entries if entry.param_type not in _DIFFERENTIABLE_PARAMETER_TYPES}
    )


def decode_theta_batch_to_tensors(
    theta_batch: torch.Tensor,
    param_entries: list[SysIdParameterEntry],
    baselines: ThetaDecodeBaselines,
) -> ThetaBatchTensors:
    """Decode a theta batch into absolute physical parameter tensors, keeping autograd.

    Semantics mirror :func:`parameter_apply.decode_theta_to_apply_state`: joint drive and
    friction entries are multiplicative scales on the baselines (clamped non-negative),
    ``LINK_MASS`` scales all links when global (``link_index == -1``) or one link otherwise,
    COM offsets are additive deltas, and Log-Cholesky inertia entries overwrite individual
    components of the baseline encoding — only links touched by an inertia entry are
    re-decoded, untouched links keep the baseline inertia tensor verbatim.

    Args:
        theta_batch: Shape ``(B, M)`` parameter rows (may require grad).
        param_entries: Column metadata for the ``M`` theta entries.
        baselines: Baseline physical values on the same device/dtype as ``theta_batch``.

    Returns:
        Decoded tensors with batch dimension ``B``.

    Raises:
        ValueError: If ``theta_batch`` is not 2D, its column count does not match
            ``param_entries``, or an entry type is not differentiable-decodable.
    """
    if theta_batch.dim() != 2:
        raise ValueError(f"theta_batch must have shape (B, M), got {tuple(theta_batch.shape)}.")
    if theta_batch.shape[1] != len(param_entries):
        raise ValueError(
            f"theta_batch has {theta_batch.shape[1]} columns but {len(param_entries)} parameter entries were given."
        )
    unsupported = validate_differentiable_entries(param_entries)
    if unsupported:
        raise ValueError(f"Parameter types not supported by the differentiable decode: {unsupported}")

    batch = theta_batch
    num_batch = batch.shape[0]
    num_dof = baselines.num_dof
    num_links = baselines.num_links
    device = batch.device
    dtype = batch.dtype

    # All accumulation below is out-of-place (mask blends): in-place index writes
    # after reading the same slice invalidate tensors saved for backward.
    ke_scale = torch.ones(num_batch, num_dof, device=device, dtype=dtype)
    kd_scale = torch.ones(num_batch, num_dof, device=device, dtype=dtype)
    friction_scale = torch.ones(num_batch, num_dof, device=device, dtype=dtype)
    mass_scale = torch.ones(num_batch, num_links, device=device, dtype=dtype)
    com_delta = torch.zeros(num_batch, num_links, 3, device=device, dtype=dtype)

    inertia_lc = baselines.link_inertia_lc.to(device=device, dtype=dtype)
    inertia_lc = inertia_lc.unsqueeze(0).expand(num_batch, num_links, LOG_CHOLESKY_DIM)
    touched_inertia_links: set[int] = set()

    def dof_mask(dof_index: int) -> torch.Tensor:
        mask = torch.zeros(1, num_dof, device=device, dtype=dtype)
        mask[0, dof_index] = 1.0
        return mask

    def link_mask(link_index: int) -> torch.Tensor:
        mask = torch.zeros(1, num_links, device=device, dtype=dtype)
        mask[0, link_index] = 1.0
        return mask

    for column, entry in enumerate(param_entries):
        values = batch[:, column]
        ptype = entry.param_type

        if ptype in (
            SysIdParameterType.JOINT_FRICTION,
            SysIdParameterType.JOINT_STIFFNESS,
            SysIdParameterType.JOINT_DAMPING,
        ):
            if not 0 <= entry.dof_index < num_dof:
                raise ValueError(f"{ptype.value} dof_index {entry.dof_index} is outside [0, {num_dof}).")
            # Multiplicative accumulate on one column: scale *= value there, *= 1 elsewhere.
            factor = 1.0 + dof_mask(entry.dof_index) * (values.clamp_min(0.0).unsqueeze(1) - 1.0)
            if ptype == SysIdParameterType.JOINT_FRICTION:
                friction_scale = friction_scale * factor
            elif ptype == SysIdParameterType.JOINT_STIFFNESS:
                ke_scale = ke_scale * factor
            else:
                kd_scale = kd_scale * factor
            continue

        if ptype == SysIdParameterType.LINK_MASS:
            scaled = values.clamp_min(0.0).unsqueeze(1)
            if entry.link_index == GLOBAL_LINK_INDEX:
                mass_scale = scaled.expand(num_batch, num_links)
            elif 0 <= entry.link_index < num_links:
                mask = link_mask(entry.link_index)
                mass_scale = mass_scale * (1.0 - mask) + mask * scaled
            else:
                raise ValueError(
                    f"{ptype.value} link_index {entry.link_index} is outside [0, {num_links}) "
                    f"and is not GLOBAL_LINK_INDEX."
                )
            continue

        if ptype in _COM_AXIS_BY_TYPE:
            axis = _COM_AXIS_BY_TYPE[ptype]
            mask = torch.zeros(1, num_links, 3, device=device, dtype=dtype)
            if entry.link_index == GLOBAL_LINK_INDEX:
                mask[0, :, axis] = 1.0
            elif 0 <= entry.link_index < num_links:
                mask[0, entry.link_index, axis] = 1.0
            else:
                raise ValueError(
                    f"{ptype.value} link_index {entry.link_index} is outside [0, {num_links}) "
                    f"and is not GLOBAL_LINK_INDEX."
                )
            com_delta = com_delta + mask * values.reshape(num_batch, 1, 1)
            continue

        if ptype == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
            if not 0 <= entry.component_index < LOG_CHOLESKY_DIM:
                raise ValueError(
                    f"{ptype.value} component_index {entry.component_index} is outside " f"[0, {LOG_CHOLESKY_DIM})."
                )
            mask = torch.zeros(1, num_links, LOG_CHOLESKY_DIM, device=device, dtype=dtype)
            if entry.link_index == GLOBAL_LINK_INDEX:
                mask[0, :, entry.component_index] = 1.0
                touched_inertia_links.update(range(num_links))
            elif 0 <= entry.link_index < num_links:
                mask[0, entry.link_index, entry.component_index] = 1.0
                touched_inertia_links.add(entry.link_index)
            else:
                raise ValueError(
                    f"{ptype.value} link_index {entry.link_index} is outside [0, {num_links}) "
                    f"and is not GLOBAL_LINK_INDEX."
                )
            inertia_lc = inertia_lc * (1.0 - mask) + mask * values.reshape(num_batch, 1, 1)
            continue

    ke = baselines.stiffness.to(device=device, dtype=dtype).unsqueeze(0) * ke_scale
    kd = baselines.damping.to(device=device, dtype=dtype).unsqueeze(0) * kd_scale
    friction = baselines.friction.to(device=device, dtype=dtype).unsqueeze(0) * friction_scale
    mass = baselines.link_mass.to(device=device, dtype=dtype).unsqueeze(0) * mass_scale
    com = baselines.link_com.to(device=device, dtype=dtype).unsqueeze(0) + com_delta

    # Only re-decode links an inertia entry actually touched; untouched links keep the
    # authored baseline tensor verbatim (a decode round-trip would perturb it numerically).
    inertia = baselines.link_inertia.to(device=device, dtype=dtype).unsqueeze(0)
    inertia = inertia.expand(num_batch, num_links, 3, 3)
    if touched_inertia_links:
        touched_mask = torch.zeros(1, num_links, 1, 1, device=device, dtype=dtype)
        touched_mask[0, sorted(touched_inertia_links)] = 1.0
        decoded = log_cholesky_to_inertia_matrix_torch(inertia_lc)
        inertia = inertia * (1.0 - touched_mask) + decoded * touched_mask

    return ThetaBatchTensors(ke=ke, kd=kd, friction=friction, mass=mass, com=com, inertia=inertia)
