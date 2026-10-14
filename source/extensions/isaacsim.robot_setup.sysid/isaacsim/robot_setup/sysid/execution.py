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

"""High-level execution API shared by UI and headless workflows."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from .errors import SysIdRuntimeUnavailableError

if TYPE_CHECKING:
    from .optimizer_base import OptimizationIterationStatus
    from .run_controller import SysIdPreparedRun
    from .run_session import SysIdRunResult, SysIdRunSession
    from .run_spec import SysIdRunSpec


async def run_sysid(
    spec: SysIdRunSpec,
    *,
    on_progress: Callable[[str, float], None] | None = None,
    on_iteration: Callable[[OptimizationIterationStatus], None] | None = None,
    prepared_run: SysIdPreparedRun | None = None,
    provenance_sidecar_path: str | None = None,
    instance_local_writeback: bool = False,
) -> SysIdRunResult:
    """Execute a complete SysID run from a run specification.

    Isaac Sim execution uses the active Kit stage, opening ``stage.input_path``
    when supplied. Newton execution can instead open that USD directly without
    using the Kit stage.

    Args:
        spec: SysID run specification.
        on_progress: Optional progress callback.
        on_iteration: Optional iteration callback.
        prepared_run: Optional prepared runtime state to reuse.
        provenance_sidecar_path: Optional override for the provenance sidecar destination.
        instance_local_writeback: Author accepted parameters as overrides in the open stage root.

    Returns:
        Completed optimization result and generated artifacts.
    """
    if prepared_run is None:
        from .run_controller import SysIdRunController
        from .runtime import ensure_required_runtime_extensions_async

        await ensure_required_runtime_extensions_async(spec.simulation)
        engine = str(spec.simulation.engine)
        stage = await _resolve_stage(spec, engine)
        controller = SysIdRunController(spec)
        trajectory = controller.load_trajectory()
        prepared_run = controller.prepare(stage, trajectory=trajectory, timeline_playing=None)
    elif str(prepared_run.simulation_engine) != str(spec.simulation.engine):
        raise ValueError(
            "prepared_run simulation engine does not match spec.simulation.engine: "
            f"{prepared_run.simulation_engine!r} != {spec.simulation.engine!r}"
        )
    session = create_sysid_session(
        spec,
        prepared_run,
        provenance_sidecar_path=provenance_sidecar_path,
        instance_local_writeback=instance_local_writeback,
    )
    return await session.run_async(on_iteration=on_iteration, on_progress=on_progress)


def create_sysid_session(
    spec: SysIdRunSpec,
    prepared: SysIdPreparedRun,
    *,
    provenance_sidecar_path: str | None = None,
    instance_local_writeback: bool = False,
) -> SysIdRunSession:
    """Create the compatibility session used by advanced UI and CLI callers.

    Args:
        spec: SysID run specification.
        prepared: Prepared SysID run state.
        provenance_sidecar_path: Optional override for the provenance sidecar destination.
        instance_local_writeback: Author accepted parameters as overrides in the open stage root.

    Returns:
        Configured one-shot run session.
    """
    from .run_session import SysIdRunSession

    outputs = spec.outputs
    return SysIdRunSession(
        prepared,
        apply_params_to_usd=outputs.apply_parameters_to_usd,
        write_usd_provenance=outputs.write_usd_provenance,
        export_provenance_sidecar=outputs.export_provenance_sidecar,
        provenance_sidecar_path=(
            outputs.provenance_path if provenance_sidecar_path is None else provenance_sidecar_path
        ),
        remove_clones_after_run=outputs.remove_clones_after_run,
        robot_path=spec.simulation.robot_prim_path,
        validation_animation_enabled=outputs.render_validation_animation,
        validation_animation_path=outputs.validation_animation_path,
        validation_animation_fps=outputs.validation_animation_fps,
        validation_animation_max_frames=outputs.validation_animation_max_frames,
        validation_relative_change_threshold=outputs.validation_relative_change_threshold,
        validation_absolute_change_threshold=outputs.validation_absolute_change_threshold,
        compute_parameter_confidence=outputs.compute_parameter_confidence,
        parameter_confidence_rollout_budget=outputs.parameter_confidence_rollout_budget,
        instance_local_writeback=instance_local_writeback,
    )


async def _resolve_stage(spec: SysIdRunSpec, engine: str) -> Any:
    from .run_spec import SIMULATION_ENGINE_ISAAC_SIM, SIMULATION_ENGINE_NEWTON

    stage_path = str(spec.stage.input_path or "")
    if engine == SIMULATION_ENGINE_ISAAC_SIM:
        from .runtime import get_current_stage, open_stage_async, require_kit_runtime

        require_kit_runtime("The Isaac Sim SysID engine")
        if stage_path:
            return await open_stage_async(stage_path)
        stage = get_current_stage()
        if stage is None:
            raise SysIdRuntimeUnavailableError(
                "The Isaac Sim SysID engine requires an active USD stage or stage.input_path."
            )
        return stage
    if engine != SIMULATION_ENGINE_NEWTON:
        raise ValueError(f"Unsupported SysID simulation engine: {engine}")

    from .runtime import get_current_stage, kit_runtime_available, open_stage_async

    if stage_path and kit_runtime_available():
        return await open_stage_async(stage_path)
    if not stage_path:
        if kit_runtime_available():
            stage = get_current_stage()
            if stage is not None:
                return stage
        raise ValueError("Newton SysID outside Kit requires stage.input_path.")
    try:
        from pxr import Usd
    except ModuleNotFoundError as exc:
        if str(exc.name or "") not in {"pxr", "pxr.Usd"}:
            raise
        raise SysIdRuntimeUnavailableError(
            "Standalone Newton SysID requires OpenUSD from the Isaac Sim Python environment."
        ) from exc
    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {stage_path}")
    return stage
