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


"""Bayesian optimization over rollout costs."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

import carb
import numpy as np
import torch

from ..env_bridge import SysIdEnvironmentBridgeError
from ..optimizer_base import OptimizationIterationStatus, OptimizerConfig
from ..optimizer_config import OptimizerBackendConfig
from ..parameter_space import ParameterSpace
from ..rollout_optimizer_base import RolloutOptimizerBase
from ..runtime import yield_control
from .cost_utils import finite_cost_mask
from .gp_core import expected_improvement_numpy, gp_posterior, standardize_observations
from .parameter_transform import BoxParameterTransform


def _select_diverse_acquisition_batch(
    candidates_norm: np.ndarray,
    scores: np.ndarray,
    batch_size: int,
) -> np.ndarray:
    """Greedily choose high-score candidates while suppressing near-duplicates.

    Args:
        candidates_norm: Candidate rows in normalized box coordinates.
        scores: Acquisition scores aligned with candidate rows.
        batch_size: Maximum number of candidates to select or evaluate.

    Returns:
        Selected normalized candidate rows.
    """
    candidates = np.asarray(candidates_norm, dtype=np.float64)
    scores = np.asarray(scores, dtype=np.float64).reshape(-1)
    if candidates.ndim != 2:
        raise ValueError(f"Expected candidate array with shape (N, M), got {candidates.shape}.")
    count = int(candidates.shape[0])
    if count == 0:
        raise ValueError("At least one BO acquisition candidate is required.")

    target = max(1, min(int(batch_size), count))
    selected: list[int] = []
    available = np.ones(count, dtype=bool)
    min_distance = np.ones(count, dtype=np.float64)
    positive_scores = np.maximum(scores, 0.0)

    for _ in range(target):
        adjusted = positive_scores * (0.1 + min_distance)
        adjusted[~available] = -np.inf
        index = int(np.argmax(adjusted))
        if not np.isfinite(adjusted[index]):
            break
        selected.append(index)
        available[index] = False

        distances = np.linalg.norm(candidates - candidates[index], axis=1)
        min_distance = np.minimum(min_distance, distances)

    if not selected:
        selected = [int(np.argmax(scores))]
    return candidates[selected]


class BayesianOptimizer(RolloutOptimizerBase):
    """Gaussian-process Bayesian optimization with expected-improvement acquisition."""

    def __init__(self) -> None:
        super().__init__()
        self._parameter_space: Optional[ParameterSpace] = None

    def set_parameter_space(self, parameter_space: ParameterSpace | None) -> None:  # noqa: D102
        self._parameter_space = parameter_space

    async def run_async(  # noqa: D102
        self,
        config: OptimizerConfig,
        on_iteration: Optional[Callable[[OptimizationIterationStatus], None]] = None,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> OptimizationIterationStatus:
        self.reset_cancel()
        device = self._bridge.device
        m = len(config.param_entries)
        if m < 1:
            raise ValueError("Select at least one parameter to optimize.")

        backend_cfg = config.backend_config or OptimizerBackendConfig()
        theta_min = config.theta_min.to(device=device, dtype=torch.float32)
        theta_max = config.theta_max.to(device=device, dtype=torch.float32)
        theta = config.theta_initial.to(device=device, dtype=torch.float32).clone()

        transform = BoxParameterTransform.from_bounds(theta_min, theta_max)
        initial_norm = np.clip(transform.physical_to_normalized(theta.detach().cpu().numpy()), 0.0, 1.0)
        fallback_norm = None
        if config.theta_fallback is not None:
            # Guard against a harmful analytical-presolve seed: the pre-presolve
            # initials are observed alongside the seed in the first batch, so the
            # incumbent-best (and the GP) see both starts.
            fallback_norm = np.clip(
                transform.physical_to_normalized(
                    config.theta_fallback.to(device=device, dtype=torch.float32).detach().cpu().numpy()
                ),
                0.0,
                1.0,
            )
        bo_seed = int(backend_cfg.bo_seed)
        if bo_seed < 0:
            raise ValueError("bo_seed must be non-negative.")
        rng = np.random.default_rng(bo_seed)
        x_obs_list: list[np.ndarray] = []
        y_obs_list: list[float] = []

        best_theta = transform.normalized_to_theta_batch(
            initial_norm,
            device=device,
            parameter_space=self._parameter_space,
            param_entries=config.param_entries,
        )[0]
        best_cost = float("inf")
        best_plot_data = None
        status = OptimizationIterationStatus(
            iteration=0,
            cost=best_cost,
            damping=0.0,
            accepted=True,
            theta=best_theta.detach().cpu().tolist(),
        )

        initial_count = max(1, int(backend_cfg.bo_initial_samples))
        max_evaluations = config.max_iterations
        batch_size = max(1, int(backend_cfg.bo_batch_size))
        acquisition_candidate_count = max(batch_size, int(backend_cfg.bo_candidate_count))

        async def evaluate_points_with_progress(
            points_norm: np.ndarray,
            progress_cb: Optional[Callable[[str, float], None]],
        ) -> tuple[np.ndarray, Optional[object]]:
            theta_batch = transform.normalized_to_theta_batch(
                points_norm,
                device=device,
                parameter_space=self._parameter_space,
                param_entries=config.param_entries,
            )
            costs, _, plot = await self.compute_costs_for_theta_batch(config, theta_batch, on_progress=progress_cb)
            return costs.detach().cpu().numpy(), plot

        eval_index = 0
        evaluated_initial = False
        while eval_index < max_evaluations:
            self._raise_if_cancelled()

            remaining = max_evaluations - eval_index
            current_batch_size = min(batch_size, remaining)
            if not evaluated_initial:
                rows = [initial_norm.copy()]
                if fallback_norm is not None and current_batch_size > 1:
                    rows.append(fallback_norm.copy())
                evaluated_initial = True
                random_count = max(0, current_batch_size - len(rows))
                if random_count:
                    rows.extend(rng.random((random_count, m)))
                batch_norm = np.asarray(rows, dtype=np.float64)
            elif len(x_obs_list) < initial_count:
                random_count = min(current_batch_size, initial_count - len(x_obs_list))
                batch_norm = rng.random((random_count, m))
            else:
                candidates_norm = rng.random((acquisition_candidate_count, m))
                x_train = np.asarray(x_obs_list, dtype=np.float64)
                y_train = np.asarray(y_obs_list, dtype=np.float64)
                y_standard, y_mean, y_scale = standardize_observations(y_train)
                try:
                    mu, sigma = gp_posterior(x_train, y_standard, candidates_norm)
                    best_standard = (best_cost - y_mean) / y_scale
                    ei = expected_improvement_numpy(mu, sigma, best_standard)
                    batch_norm = _select_diverse_acquisition_batch(candidates_norm, ei, current_batch_size)
                except np.linalg.LinAlgError as exc:
                    carb.log_warn(f"GP posterior failed ({exc}); evaluating random candidates instead.")
                    batch_norm = candidates_norm[:current_batch_size]

            batch_count = int(batch_norm.shape[0])
            eval_start_index = eval_index + 1
            eval_index += batch_count
            eval_start = (eval_start_index - 1) / max(max_evaluations, 1)
            eval_end = eval_index / max(max_evaluations, 1)
            eval_label = (
                f"{eval_start_index}/{max_evaluations}"
                if batch_count == 1
                else f"{eval_start_index}-{eval_index}/{max_evaluations}"
            )
            if on_progress:
                on_progress(f"Bayesian opt evaluations {eval_label}", eval_start)
                await yield_control()

            costs_np, batch_plot_data = await evaluate_points_with_progress(
                batch_norm,
                self._progress_for_phase(on_progress, eval_start, eval_end),
            )
            finite_mask = finite_cost_mask(costs_np)
            accepted = False
            accepted_index = -1
            for row_index, (row_norm, row_cost, is_finite) in enumerate(zip(batch_norm, costs_np, finite_mask)):
                if not is_finite:
                    continue
                cost = float(row_cost)
                x_obs_list.append(row_norm.copy())
                y_obs_list.append(cost)
                if cost < best_cost:
                    accepted = True
                    accepted_index = row_index
                    best_cost = cost
                    best_theta = transform.normalized_to_theta_batch(
                        row_norm,
                        device=device,
                        parameter_space=self._parameter_space,
                        param_entries=config.param_entries,
                    )[0]
            if accepted:
                if accepted_index == 0 and batch_plot_data is not None:
                    best_plot_data = batch_plot_data
                else:
                    _, _, best_plot_data = await self.compute_costs_for_theta_batch(
                        config,
                        best_theta.unsqueeze(0),
                        include_plot_data=True,
                    )

            status = OptimizationIterationStatus(
                iteration=eval_index,
                cost=best_cost,
                damping=0.0,
                accepted=accepted,
                theta=best_theta.detach().cpu().tolist(),
                rollout=best_plot_data,
            )
            await yield_control()
            if on_iteration is not None:
                on_iteration(status)
            if on_progress:
                on_progress(f"Bayesian opt evaluations {eval_label}: complete", eval_end)
            await yield_control()

        self._raise_if_cancelled()
        if not np.isfinite(best_cost):
            raise SysIdEnvironmentBridgeError(
                "Bayesian optimization did not produce a finite rollout cost. "
                "Check trajectory values, parameter bounds, and simulation stability."
            )
        if best_cost < float("inf"):
            if on_progress:
                on_progress("Bayesian opt final best rollout", 0.99)
                await yield_control()
            final_costs, _, final_plot = await self.compute_costs_for_theta_batch(
                config,
                best_theta.unsqueeze(0),
                on_progress=self._progress_for_phase(on_progress, 0.99, 1.0),
            )
            final_cost = float(final_costs[0].item())
            if np.isfinite(final_cost):
                best_cost = final_cost
                best_plot_data = final_plot
            status = OptimizationIterationStatus(
                iteration=status.iteration,
                cost=best_cost,
                damping=0.0,
                accepted=status.accepted,
                theta=best_theta.detach().cpu().tolist(),
                rollout=best_plot_data,
            )

        self._raise_if_cancelled()
        if on_progress:
            on_progress("Optimization finished", 1.0)
        return status
