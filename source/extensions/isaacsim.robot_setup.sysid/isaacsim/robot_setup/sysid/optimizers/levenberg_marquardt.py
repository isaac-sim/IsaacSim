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

"""Levenberg-Marquardt system identification optimizer."""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Optional

import carb
import torch

from ..env_bridge import SysIdEnvironmentBridgeError
from ..lm_solver import (
    build_bounded_perturbed_theta_batch,
    clamp_parameters,
    compute_jacobian,
    compute_residual_cost,
    levenberg_marquardt_step,
)
from ..optimizer_base import OptimizationIterationStatus, OptimizerConfig
from ..parameter_space import ParameterSpace
from ..parameter_types import SysIdParameterType
from ..regressor import estimate_analytical_jacobian
from ..rollout_optimizer_base import RolloutOptimizerBase
from ..runtime import yield_control

_CONSTANT_ANALYTICAL_PARAMETER_TYPES = frozenset(
    {
        SysIdParameterType.LINK_MASS,
        SysIdParameterType.JOINT_FRICTION,
        SysIdParameterType.JOINT_STIFFNESS,
        SysIdParameterType.JOINT_DAMPING,
    }
)


def _residual_is_torque_only(config: OptimizerConfig) -> bool:
    """Return True when the residual contains only the torque channel.

    The analytical regressor is in the torque (inverse-dynamics) domain, so it is
    only a valid Levenberg-Marquardt Jacobian when the residual is exclusively
    torque. The default residual (position + velocity) lives in a different space.

    Args:
        config: Optimizer and trajectory configuration.

    Returns:
        Whether the configured residual contains torque terms exclusively.
    """
    residual_cfg = config.residual_weight_config
    if residual_cfg is None:
        return False
    return (
        float(residual_cfg.torque_weight) > 0.0
        and float(residual_cfg.position_weight) == 0.0
        and float(residual_cfg.velocity_weight) == 0.0
        and float(residual_cfg.end_effector_pose_weight) == 0.0
        and float(residual_cfg.contact_force_weight) == 0.0
    )


def _parameters_support_constant_analytical_jacobian(config: OptimizerConfig) -> bool:
    """Return whether every selected parameter has a constant torque Jacobian.

    Args:
        config: Optimizer configuration containing the selected parameter entries.

    Returns:
        Whether all selected entries support the analytical torque-domain Jacobian.
    """
    return all(entry.param_type in _CONSTANT_ANALYTICAL_PARAMETER_TYPES for entry in config.param_entries)


def _scale_analytical_jacobian_for_residual(
    jacobian: torch.Tensor,
    *,
    sample_weights: torch.Tensor | None,
    torque_weight: float,
    num_steps: int,
    num_dof: int,
) -> torch.Tensor:
    """Apply the exact row scaling used by ``WeightedResidualEngine``.

    Args:
        jacobian: Unweighted torque Jacobian with one row per step and DOF.
        sample_weights: Optional per-step residual multipliers.
        torque_weight: Non-negative torque residual weight.
        num_steps: Number of trajectory steps represented by the rows.
        num_dof: Number of joint torque residuals per step.

    Returns:
        Jacobian with sample and torque residual scaling applied row-wise.
    """
    expected_rows = int(num_steps) * int(num_dof)
    if jacobian.shape[0] != expected_rows:
        raise ValueError(
            f"Analytical Jacobian rows ({jacobian.shape[0]}) must equal " f"num_steps * num_dof ({expected_rows})."
        )
    step_scale = torch.full(
        (num_steps,),
        math.sqrt(float(torque_weight)),
        device=jacobian.device,
        dtype=jacobian.dtype,
    )
    if sample_weights is not None:
        weights = sample_weights.to(device=jacobian.device, dtype=jacobian.dtype).reshape(-1)
        if weights.shape[0] != num_steps:
            raise ValueError(f"Analytical sample weights ({weights.shape[0]}) must match rollout steps ({num_steps}).")
        step_scale = step_scale * weights
    row_scale = step_scale.repeat_interleave(num_dof)
    return jacobian * row_scale.unsqueeze(1)


class LevenbergMarquardtOptimizer(RolloutOptimizerBase):
    """Orchestrates parallel rollouts and LM updates via :class:`SysIdEnvironmentBridge`."""

    def __init__(self) -> None:
        super().__init__()
        self._parameter_space: Optional[ParameterSpace] = None

    def set_parameter_space(self, parameter_space: ParameterSpace | None) -> None:
        """Attach parameter registry used for inertia reparam and analytical Jacobians.

        Args:
            parameter_space: Optional registry used to enforce coupled constraints.
        """
        self._parameter_space = parameter_space

    async def run_async(
        self,
        config: OptimizerConfig,
        on_iteration: Optional[Callable[[OptimizationIterationStatus], None]] = None,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> OptimizationIterationStatus:
        """Run LM optimization asynchronously, yielding UI updates between iterations.

        Args:
            config: Optimizer and trajectory configuration.
            on_iteration: Optional iteration callback.
            on_progress: Optional progress callback.

        Returns:
            Final LM iteration status, including parameters, cost, and convergence state.
        """
        self.reset_cancel()
        bridge = self._bridge
        device = bridge.device

        m = len(config.param_entries)
        if m < 1:
            raise ValueError("Select at least one parameter to optimize.")

        theta_min = config.theta_min.to(device=device, dtype=torch.float32)
        theta_max = config.theta_max.to(device=device, dtype=torch.float32)
        theta = clamp_parameters(config.theta_initial.to(device=device, dtype=torch.float32), theta_min, theta_max)
        if self._parameter_space is not None:
            theta = self._parameter_space.reparameterize_inertia_theta(theta, config.param_entries)
            theta = clamp_parameters(theta, theta_min, theta_max)
        damping = float(config.damping_initial)
        epsilon = float(config.epsilon)
        use_analytical = config.use_analytical_jacobian
        if config.backend_config is not None:
            use_analytical = use_analytical or config.backend_config.use_analytical_jacobian
        if use_analytical and not _residual_is_torque_only(config):
            # The analytical regressor is d(torque)/d(theta); it is only a valid
            # Jacobian for a torque-domain residual. The default position/velocity
            # residual lives in a different space, so fall back to finite differences
            # rather than solving against a mismatched operator.
            carb.log_warn(
                "SysId: analytical Jacobian requested but the residual is not torque-only; "
                "falling back to finite-difference Jacobian."
            )
            use_analytical = False
        if use_analytical and not _parameters_support_constant_analytical_jacobian(config):
            unsupported = sorted(
                {
                    entry.param_type.value
                    for entry in config.param_entries
                    if entry.param_type not in _CONSTANT_ANALYTICAL_PARAMETER_TYPES
                }
            )
            carb.log_warn(
                "SysId: analytical Jacobian requested for nonlinear or unsupported parameter types "
                f"({', '.join(unsupported)}); falling back to finite differences."
            )
            use_analytical = False
        training_segments = self._training_segments(config)
        if len(training_segments) > 1:
            use_analytical = False

        status = OptimizationIterationStatus(
            iteration=0,
            cost=0.0,
            damping=damping,
            accepted=True,
            theta=theta.detach().cpu().tolist(),
        )

        max_iter = config.max_iterations
        for iteration in range(1, max_iter + 1):
            self._raise_if_cancelled()

            iter_start = (iteration - 1) / max(max_iter, 1)
            iter_end = iteration / max(max_iter, 1)
            jacobian_end = iter_start + (iter_end - iter_start) * 0.70
            if on_progress:
                phase = "nominal rollout" if use_analytical else "Jacobian rollouts"
                on_progress(f"Iteration {iteration}/{max_iter}: {phase}", iter_start)
                await yield_control()

            dynamics_context = None
            if use_analytical:
                context_getter = getattr(bridge, "get_analytical_dynamics_context", None)
                dynamics_context = (
                    context_getter()
                    if callable(context_getter)
                    else getattr(bridge, "analytical_dynamics_context", None)
                )
                if dynamics_context is None:
                    raise SysIdEnvironmentBridgeError(
                        "Analytical Jacobian was requested, but the active bridge does not expose a compatible "
                        "fixed-base dynamics context. Disable analytical Jacobian or use a bridge that implements "
                        "get_analytical_dynamics_context()."
                    )

            column_steps = None
            if use_analytical:
                theta_batch = theta.unsqueeze(0)
            else:
                theta_batch, column_steps = build_bounded_perturbed_theta_batch(
                    theta,
                    epsilon,
                    theta_min,
                    theta_max,
                    parameter_space=self._parameter_space,
                    param_entries=config.param_entries,
                )
            _, residuals, plot_jacobian = await self.compute_costs_for_theta_batch(
                config,
                theta_batch,
                on_progress=self._progress_for_phase(on_progress, iter_start, jacobian_end),
            )

            r_nom = residuals[0]
            r_pert = None if use_analytical else residuals[1:]
            if not bool(torch.all(torch.isfinite(r_nom)).item()):
                raise SysIdEnvironmentBridgeError(
                    "Levenberg-Marquardt nominal rollout produced non-finite residuals. "
                    "Check trajectory values, parameter bounds, and simulation stability."
                )
            cost_before = float(compute_residual_cost(r_nom).item())
            if not math.isfinite(cost_before):
                raise SysIdEnvironmentBridgeError("Levenberg-Marquardt nominal rollout produced a non-finite cost.")

            analytical = None
            if use_analytical:
                analytical_trajectory = training_segments[0].trajectory
                analytical_steps = min(
                    analytical_trajectory.commands.shape[0],
                    max(1, config.max_rollout_steps),
                )
                analytical = estimate_analytical_jacobian(
                    analytical_trajectory.positions[:analytical_steps],
                    analytical_trajectory.velocities[:analytical_steps],
                    analytical_trajectory.times[:analytical_steps],
                    config.param_entries,
                    num_links=config.num_links,
                    baseline_link_inertia_lc=(
                        self._parameter_space.baseline.link_inertia_lc if self._parameter_space else None
                    ),
                    residual_dim=r_nom.shape[0],
                    dynamics_context=dynamics_context,
                    commands=analytical_trajectory.commands[:analytical_steps],
                    baseline_joint_friction=(
                        self._parameter_space.baseline.joint_friction if self._parameter_space else None
                    ),
                    baseline_joint_stiffness=(
                        self._parameter_space.baseline.joint_stiffness if self._parameter_space else None
                    ),
                    baseline_joint_damping=(
                        self._parameter_space.baseline.joint_damping if self._parameter_space else None
                    ),
                )
                if analytical is not None:
                    residual_cfg = config.residual_weight_config
                    if residual_cfg is None:
                        raise ValueError("Torque-only analytical mode requires residual weight configuration.")
                    analytical_sample_weights = self._cached_sample_weights(
                        residual_cfg,
                        analytical_trajectory,
                        bridge.device,
                        analytical_steps,
                    )
                    analytical = _scale_analytical_jacobian_for_residual(
                        analytical.to(device=bridge.device, dtype=r_nom.dtype),
                        sample_weights=analytical_sample_weights,
                        torque_weight=float(residual_cfg.torque_weight),
                        num_steps=analytical_steps,
                        num_dof=int(analytical_trajectory.positions.shape[1]),
                    )
                if analytical is None:
                    if on_progress:
                        on_progress(
                            f"Iteration {iteration}/{max_iter}: finite-difference fallback",
                            iter_start,
                        )
                        await yield_control()
                    fallback_full_batch, column_steps = build_bounded_perturbed_theta_batch(
                        theta,
                        epsilon,
                        theta_min,
                        theta_max,
                        parameter_space=self._parameter_space,
                        param_entries=config.param_entries,
                    )
                    fallback_batch = fallback_full_batch[1:]
                    _, r_pert, _ = await self.compute_costs_for_theta_batch(
                        config,
                        fallback_batch,
                        on_progress=self._progress_for_phase(on_progress, iter_start, jacobian_end),
                        include_plot_data=False,
                    )

            jacobian = compute_jacobian(
                r_nom,
                r_pert,
                epsilon,
                analytical_jacobian=analytical,
                parameter_count=m,
                column_steps=column_steps,
            )
            if not bool(torch.all(torch.isfinite(jacobian)).item()):
                raise SysIdEnvironmentBridgeError(
                    "Levenberg-Marquardt Jacobian contains non-finite values. "
                    "Check perturbed rollout stability and finite-difference epsilon."
                )
            try:
                delta = levenberg_marquardt_step(jacobian, r_nom, damping)
            except RuntimeError as exc:
                raise SysIdEnvironmentBridgeError(
                    "Levenberg-Marquardt could not compute a stable parameter step."
                ) from exc
            if not bool(torch.all(torch.isfinite(delta)).item()):
                raise SysIdEnvironmentBridgeError("Levenberg-Marquardt produced a non-finite parameter step.")
            theta_trial = clamp_parameters(theta + delta, theta_min, theta_max)
            if self._parameter_space is not None:
                theta_trial = self._parameter_space.reparameterize_inertia_theta(theta_trial, config.param_entries)
                theta_trial = clamp_parameters(theta_trial, theta_min, theta_max)

            if on_progress:
                on_progress(f"Iteration {iteration}/{max_iter}: trial rollout", jacobian_end)
                await yield_control()

            trial_batch = theta_trial.unsqueeze(0)
            trial_cost, _, plot_trial = await self.compute_costs_for_theta_batch(
                config,
                trial_batch,
                on_progress=self._progress_for_phase(on_progress, jacobian_end, iter_end),
            )
            cost_trial = float(trial_cost[0].item())

            accepted = math.isfinite(cost_trial) and cost_trial < cost_before
            if accepted:
                theta = theta_trial
                damping = max(damping / 10.0, 1e-12)
                plot_final = plot_trial
                final_cost = cost_trial
            else:
                damping = damping * 10.0
                plot_final = plot_jacobian
                final_cost = cost_before

            status = OptimizationIterationStatus(
                iteration=iteration,
                cost=final_cost,
                damping=damping,
                accepted=accepted,
                theta=theta.detach().cpu().tolist(),
                rollout=plot_final,
            )

            await yield_control()
            if on_iteration is not None:
                on_iteration(status)
            if on_progress:
                on_progress(f"Iteration {iteration}/{max_iter}: complete", iter_end)
            await yield_control()

        self._raise_if_cancelled()
        if on_progress:
            on_progress("Optimization finished", 1.0)

        return status


__all__ = ["LevenbergMarquardtOptimizer"]
