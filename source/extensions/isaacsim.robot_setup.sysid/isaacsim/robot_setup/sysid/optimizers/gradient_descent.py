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

"""First-order (Adam) optimizer over differentiable rollout costs.

Unlike Levenberg-Marquardt, which needs the full residual Jacobian (one rollout per
parameter with finite differences), this backend consumes the exact cost gradient a
differentiable bridge produces in a single backward pass, so the per-iteration cost
is independent of the parameter count. Bounds are enforced by projection (clamp)
after each Adam step. Multiple random restarts run as extra batch rows so a
batch-capable bridge evaluates them in one vectorized rollout.

Each restart owns its Adam param group: when a step walks a row into a region
where the rollout cost turns non-finite (e.g. across the explicit integrator's
damping stability boundary), that row's step is rejected — the last finite theta
is restored, the row's learning rate is halved, and its Adam momentum is dropped
— while the other rows continue unaffected. The run only aborts when no restart
ever produces a finite cost at its initial theta.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

import carb
import torch

from ..env_bridge import SysIdEnvironmentBridgeError
from ..optimizer_base import OptimizationIterationStatus, OptimizerConfig
from ..optimizer_config import OptimizerBackendConfig
from ..parameter_space import ParameterSpace
from ..rollout_optimizer_base import RolloutOptimizerBase
from ..runtime import yield_control

_NON_DIFFERENTIABLE_BRIDGE_MESSAGE = (
    "Gradient descent requires a bridge with autograd rollouts (set "
    "simulation.newton.solver='featherstone_diff'); use CMA-ES, Bayesian, or "
    "Levenberg-Marquardt with the current bridge instead."
)


class GradientDescentOptimizer(RolloutOptimizerBase):
    """Adam over rollout costs with projected box bounds and optional multi-start."""

    progress_label = "Gradient descent"

    def __init__(self) -> None:
        super().__init__()
        self._parameter_space: Optional[ParameterSpace] = None

    def set_parameter_space(self, parameter_space: ParameterSpace | None) -> None:
        """Store the parameter registry for API parity with the other backends.

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
        """Run Adam iterations asynchronously, yielding UI updates between rollouts.

        Args:
            config: Optimizer and trajectory configuration.
            on_iteration: Optional iteration callback.
            on_progress: Optional progress callback.

        Returns:
            Final optimization status.
        """
        self.reset_cancel()
        device = self._bridge.device
        num_params = len(config.param_entries)
        if num_params < 1:
            raise ValueError("Select at least one parameter to optimize.")

        backend_cfg = config.backend_config or OptimizerBackendConfig()
        learning_rate = max(float(backend_cfg.gd_learning_rate), 1e-12)
        num_restarts = max(1, int(backend_cfg.gd_num_restarts))

        theta_min = config.theta_min.to(device=device, dtype=torch.float32)
        theta_max = config.theta_max.to(device=device, dtype=torch.float32)
        lower = torch.minimum(theta_min, theta_max)
        upper = torch.maximum(theta_min, theta_max)

        def repair_theta(row: torch.Tensor) -> torch.Tensor:
            repaired = torch.clamp(row, lower, upper)
            if self._parameter_space is not None:
                repaired = self._parameter_space.reparameterize_inertia_theta(repaired, config.param_entries)
                repaired = torch.clamp(repaired, lower, upper)
            return repaired

        seeded_rows = [repair_theta(config.theta_initial.to(device=device, dtype=torch.float32))]
        if config.theta_fallback is not None:
            # Guard against a harmful analytical-presolve seed: the pre-presolve
            # initials run as their own restart row, so the per-iteration argmin
            # keeps whichever start is actually better at rollout cost.
            seeded_rows.append(repair_theta(config.theta_fallback.to(device=device, dtype=torch.float32)))
        num_restarts = max(num_restarts, len(seeded_rows))
        theta_init = torch.empty(num_restarts, num_params, device=device, dtype=torch.float32)
        for row_index, row in enumerate(seeded_rows):
            theta_init[row_index] = row
        if num_restarts > len(seeded_rows):
            span = (upper - lower).unsqueeze(0)
            random_count = num_restarts - len(seeded_rows)
            gd_seed = int(backend_cfg.gd_seed)
            if gd_seed < 0:
                raise ValueError("gd_seed must be non-negative.")
            restart_generator = torch.Generator(device=device)
            restart_generator.manual_seed(gd_seed)
            theta_init[len(seeded_rows) :] = (
                lower.unsqueeze(0)
                + torch.rand(
                    random_count,
                    num_params,
                    device=device,
                    generator=restart_generator,
                )
                * span
            )
        if self._parameter_space is not None:
            for row_index in range(num_restarts):
                theta_init[row_index] = repair_theta(theta_init[row_index])
        # One leaf tensor and Adam param group per restart so a row that steps into a
        # non-finite region can reject the step and back off its own learning rate
        # without slowing the healthy rows.
        theta_rows = [theta_init[row_index].detach().clone().requires_grad_(True) for row_index in range(num_restarts)]
        adam = torch.optim.Adam([{"params": [row], "lr": learning_rate} for row in theta_rows])
        row_learning_rate = [learning_rate] * num_restarts
        min_learning_rate = learning_rate * 1e-6
        last_finite_theta = [row.detach().clone() for row in theta_rows]
        row_was_finite = [False] * num_restarts

        best_cost = float("inf")
        best_theta = theta_rows[0].detach().clone()
        best_plot_data = None
        status = OptimizationIterationStatus(
            iteration=0,
            cost=best_cost,
            damping=learning_rate,
            accepted=True,
            theta=best_theta.cpu().tolist(),
        )

        max_iterations = config.max_iterations
        for iteration in range(1, max_iterations + 1):
            self._raise_if_cancelled()

            iter_start = (iteration - 1) / max(max_iterations, 1)
            iter_end = iteration / max(max_iterations, 1)
            if on_progress:
                on_progress(
                    f"{self.progress_label} iteration {iteration}/{max_iterations}",
                    iter_start,
                )
                await yield_control()

            # `torch.stack` copies the leaves, so `theta_batch` keeps this iteration's
            # values even after `adam.step()` mutates the rows in place.
            theta_batch = torch.stack(theta_rows)
            serial_backward = bool(
                getattr(self._bridge, "requires_serial_backward", False)
                and (config.training_segments or config.training_chunks)
            )
            adam.zero_grad()
            if serial_backward:
                (
                    costs_detached,
                    finite,
                    plot_data,
                ) = await self.compute_segmented_costs_and_backward(
                    config,
                    theta_batch,
                    on_progress=self._progress_for_phase(on_progress, iter_start, iter_end),
                )
                costs = costs_detached
            else:
                costs, _, plot_data = await self.compute_costs_for_theta_batch(
                    config,
                    theta_batch,
                    on_progress=self._progress_for_phase(on_progress, iter_start, iter_end),
                )
                if iteration == 1 and not costs.requires_grad:
                    raise SysIdEnvironmentBridgeError(_NON_DIFFERENTIABLE_BRIDGE_MESSAGE)
                costs_detached = costs.detach()
                finite = torch.isfinite(costs_detached)
            if iteration == 1 and config.theta_fallback is not None:
                seed_cost = float(costs_detached[0].item())
                fallback_cost = float(costs_detached[1].item())
                if not (seed_cost <= fallback_cost):
                    carb.log_warn(
                        "SysId gradient descent: the analytical presolve seed starts worse than the "
                        f"pre-presolve initials at rollout cost ({seed_cost:.6g} vs {fallback_cost:.6g}); "
                        "the initials run as their own restart row and the better start wins."
                    )
                else:
                    carb.log_info(
                        "SysId gradient descent: initial rollout cost "
                        f"{seed_cost:.6g} at the analytical presolve seed vs {fallback_cost:.6g} at the "
                        "pre-presolve initials."
                    )
            any_finite = bool(torch.any(finite))
            if not any_finite and not any(row_was_finite):
                raise SysIdEnvironmentBridgeError(
                    "Gradient descent received non-finite costs for every restart at its initial "
                    "parameters. If this is the differentiable Newton bridge, the explicit "
                    "integrator likely went unstable: increase "
                    "simulation.newton.featherstone_substeps and/or author joint armature "
                    "(reflected rotor inertia) on the robot - see the preceding SysId warning for "
                    "the stability estimate. Otherwise check the trajectory and parameter bounds."
                )

            accepted = False
            row_index = 0
            if any_finite:
                row_index = int(torch.argmin(torch.where(finite, costs_detached, torch.inf)).item())
                row_cost = float(costs_detached[row_index].item())
                accepted = row_cost < best_cost
                if accepted:
                    best_cost = row_cost
                    best_theta = repair_theta(theta_batch[row_index].detach()).clone()

                if not serial_backward:
                    # Backward before any extra rollout: a second forward on the same bridge
                    # could overwrite state the pending backward pass still needs.
                    costs[finite].sum().backward()
                finite_gradient = finite.clone()
                for restart_index, row in enumerate(theta_rows):
                    if not bool(finite[restart_index]):
                        row.grad = None
                    elif row.grad is None or not bool(torch.all(torch.isfinite(row.grad))):
                        finite_gradient[restart_index] = False
                        row.grad = None
                adam.step()
            else:
                finite_gradient = finite.clone()

            with torch.no_grad():
                for restart_index in range(num_restarts):
                    if bool(finite[restart_index]):
                        row_was_finite[restart_index] = True
                        last_finite_theta[restart_index] = theta_batch[restart_index].detach().clone()
                        if not bool(finite_gradient[restart_index]):
                            theta_rows[restart_index].copy_(last_finite_theta[restart_index])
                            row_learning_rate[restart_index] = max(
                                row_learning_rate[restart_index] * 0.5, min_learning_rate
                            )
                            adam.param_groups[restart_index]["lr"] = row_learning_rate[restart_index]
                            adam.state.pop(theta_rows[restart_index], None)
                            carb.log_warn(
                                f"SysId gradient descent: restart {restart_index} produced a non-finite "
                                f"gradient at iteration {iteration}; rejected the step and reduced its "
                                f"learning rate to {row_learning_rate[restart_index]:.3g}."
                            )
                    elif row_was_finite[restart_index]:
                        # The last accepted step walked this row into a non-finite region
                        # (e.g. across the explicit integrator's damping stability
                        # boundary). Reject it: restore the last finite theta, halve the
                        # row's learning rate, and drop its Adam momentum so the next
                        # step does not immediately re-cross the boundary.
                        theta_rows[restart_index].copy_(last_finite_theta[restart_index])
                        row_learning_rate[restart_index] = max(
                            row_learning_rate[restart_index] * 0.5, min_learning_rate
                        )
                        adam.param_groups[restart_index]["lr"] = row_learning_rate[restart_index]
                        adam.state.pop(theta_rows[restart_index], None)
                        carb.log_warn(
                            f"SysId gradient descent: restart {restart_index} produced a non-finite "
                            f"cost at iteration {iteration}; rejected the step and reduced its "
                            f"learning rate to {row_learning_rate[restart_index]:.3g}."
                        )
                for row in theta_rows:
                    row.copy_(repair_theta(row))

            if accepted:
                if row_index == 0 and plot_data is not None:
                    best_plot_data = plot_data
                else:
                    _, _, best_plot_data = await self.compute_costs_for_theta_batch(
                        config,
                        best_theta.unsqueeze(0),
                        include_plot_data=True,
                    )

            status = OptimizationIterationStatus(
                iteration=iteration,
                cost=best_cost,
                damping=row_learning_rate[row_index],
                accepted=accepted,
                theta=best_theta.cpu().tolist(),
                rollout=best_plot_data,
            )
            await yield_control()
            if on_iteration is not None:
                on_iteration(status)
            if on_progress:
                on_progress(
                    f"{self.progress_label} iteration {iteration}/{max_iterations}: complete",
                    iter_end,
                )
            await yield_control()

        self._raise_if_cancelled()
        if best_cost < float("inf"):
            if on_progress:
                on_progress(f"{self.progress_label} final best rollout", 0.99)
                await yield_control()
            final_costs, _, final_plot = await self.compute_costs_for_theta_batch(
                config,
                best_theta.unsqueeze(0).detach(),
                on_progress=self._progress_for_phase(on_progress, 0.99, 1.0),
            )
            final_cost = float(final_costs[0].item())
            if torch.isfinite(final_costs[0]):
                best_cost = final_cost
                best_plot_data = final_plot
            status = OptimizationIterationStatus(
                iteration=status.iteration,
                cost=best_cost,
                damping=row_learning_rate[row_index],
                accepted=status.accepted,
                theta=best_theta.cpu().tolist(),
                rollout=best_plot_data,
            )

        self._raise_if_cancelled()
        if on_progress:
            on_progress("Optimization finished", 1.0)
        return status


__all__ = ["GradientDescentOptimizer"]
