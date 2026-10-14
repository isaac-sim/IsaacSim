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

"""Shared optimizer protocol and configuration."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol, runtime_checkable

import torch

from .env_bridge import SysIdEnvironmentBridge
from .optimizer_config import OptimizerBackendConfig
from .parameter_types import SysIdParameterEntry
from .residual_config import ResidualWeightConfig
from .rollout_plot_data import SysIdRolloutPlotData
from .trajectory_csv import TrajectoryDataset
from .trajectory_segments import TrajectoryChunk


@dataclass
class OptimizationIterationStatus:
    """Status reported to the UI after each optimizer iteration."""

    iteration: int
    cost: float
    damping: float
    accepted: bool
    theta: list[float]
    rollout: Optional[SysIdRolloutPlotData] = None


@dataclass
class OptimizerConfig:
    """Solver and trajectory configuration shared by all optimizer backends."""

    trajectory: TrajectoryDataset
    param_entries: list[SysIdParameterEntry]
    theta_initial: torch.Tensor
    theta_min: torch.Tensor
    theta_max: torch.Tensor
    epsilon: float
    damping_initial: float
    max_iterations: int
    max_rollout_steps: int = 300
    use_analytical_jacobian: bool = False
    num_links: int = 1
    residual_weight_config: ResidualWeightConfig | None = None
    backend_config: OptimizerBackendConfig | None = None
    training_segments: list[TrajectoryChunk] | None = None
    training_chunks: list[TrajectoryDataset] | None = None
    training_chunk_weights: list[float] | None = None
    training_chunk_labels: list[str] | None = None
    # Pre-presolve initial parameters, set only when an analytical presolve seed
    # replaced ``theta_initial``. Backends evaluate it alongside the seed so a
    # poor seed cannot make the solve worse than running without the presolve.
    theta_fallback: torch.Tensor | None = None
    # Immutable values authored by the user/recipe at launch.  This differs
    # from ``theta_initial`` when analytical presolve supplies the optimizer
    # seed, and is the only valid baseline for "untuned" validation.
    theta_authored_initial: torch.Tensor | None = None


# Backward-compatible aliases for existing call sites.
SysIdIterationStatus = OptimizationIterationStatus
SysIdOptimizerConfig = OptimizerConfig


@runtime_checkable
class Optimizer(Protocol):
    """Common interface for SysID optimization backends."""

    def set_bridge(self, bridge: SysIdEnvironmentBridge) -> None:
        """Attach a simulation backend.

        Args:
            bridge: Simulation environment bridge to attach.
        """
        ...

    def get_bridge(self) -> SysIdEnvironmentBridge:
        """Return the active simulation backend.

        Returns:
            Active simulation environment bridge.
        """
        ...

    def cleanup_bridge(self, remove_clones: bool = True) -> None:
        """Release resources held by the active bridge.

        Args:
            remove_clones: Whether bridge cleanup also removes cloned environments.
        """
        ...

    async def cleanup_bridge_async(self, remove_clones: bool = True) -> None:
        """Release resources held by the active bridge, yielding Kit updates when needed.

        Args:
            remove_clones: Whether bridge cleanup also removes cloned environments.
        """
        ...

    def request_cancel(self) -> None:
        """Request graceful stop of the async optimization loop."""
        ...

    def reset_cancel(self) -> None:
        """Clear a prior cancellation request."""
        ...

    async def evaluate_nominal_cost(self, config: OptimizerConfig, theta: torch.Tensor) -> float:
        """Evaluate cost at nominal parameters only (single-env rollout).

        Args:
            config: Optimizer and trajectory configuration.
            theta: Nominal parameter vector to evaluate.

        Returns:
            Scalar rollout cost at ``theta``.
        """
        ...

    async def run_async(
        self,
        config: OptimizerConfig,
        on_iteration: Optional[Callable[[OptimizationIterationStatus], None]] = None,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> OptimizationIterationStatus:
        """Run optimization asynchronously, yielding UI updates between iterations.

        Args:
            config: Optimizer and trajectory configuration.
            on_iteration: Optional iteration callback.
            on_progress: Optional progress callback.

        Returns:
            Final iteration status, including parameters, cost, and convergence state.
        """
        ...
