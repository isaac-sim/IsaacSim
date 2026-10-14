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

"""Recipe application: map workflow presets onto run specs with data-derived defaults.

Presets (:mod:`preset_loader`) capture per-robot-class opinions (which parameters,
which optimizer, which residual channels). This module applies them onto a
:class:`SysIdRunSpec` and derives the settings that should follow from the data
rather than from the user — most importantly residual channel weights, which are
meaningless for channels the telemetry does not contain.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from .actuator_compatibility import (
    ACTUATOR_RUNTIME_AUTO,
    ACTUATOR_RUNTIME_EXPLICIT,
    ACTUATOR_RUNTIME_IMPLICIT,
    ACTUATOR_RUNTIME_MIXED,
    EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
    EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
    PHYSICS_BACKEND_AUTO,
    PHYSICS_BACKEND_NEWTON,
    PHYSICS_BACKEND_PHYSX,
)
from .backend_capabilities import resolve_backend_capabilities
from .preset_loader import ParameterPreset
from .residual_config import ResidualWeightConfig
from .run_spec import (
    NEWTON_SOLVER_FEATHERSTONE_DIFF,
    NEWTON_SOLVER_MUJOCO,
    SIMULATION_ENGINE_ISAAC_SIM,
    SIMULATION_ENGINE_NEWTON,
    ParameterRunSpec,
    ResidualsRunSpec,
    SysIdRunSpec,
)
from .torque_semantics import EFFORT_SEMANTICS_LINK_SIDE, resolve_effort_semantics
from .trajectory_csv import TrajectoryDataset

_VALID_ENGINES = {SIMULATION_ENGINE_ISAAC_SIM, SIMULATION_ENGINE_NEWTON}
_VALID_NEWTON_SOLVERS = {NEWTON_SOLVER_MUJOCO, NEWTON_SOLVER_FEATHERSTONE_DIFF}
_VALID_FEEDFORWARD_MODES = {"", "none", "gravity", "inverse_dynamics"}
_VALID_ACTUATOR_RUNTIMES = {
    ACTUATOR_RUNTIME_AUTO,
    ACTUATOR_RUNTIME_IMPLICIT,
    ACTUATOR_RUNTIME_EXPLICIT,
    ACTUATOR_RUNTIME_MIXED,
}
_VALID_PHYSICS_BACKENDS = {PHYSICS_BACKEND_AUTO, PHYSICS_BACKEND_PHYSX, PHYSICS_BACKEND_NEWTON}
_VALID_EXPLICIT_ACTUATOR_POLICIES = {
    EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
    EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
}
_SOLVER_INT_KEYS = (
    "max_iterations",
    "max_rollout_steps",
    "cma_population_size",
    "cma_batch_size",
    "bo_initial_samples",
    "bo_candidate_count",
    "bo_batch_size",
    "gd_num_restarts",
    "cma_seed",
    "bo_seed",
    "gd_seed",
)
_SOLVER_FLOAT_KEYS = ("damping_initial", "epsilon", "cma_sigma", "gd_learning_rate")
_SOLVER_BOOL_KEYS = ("use_analytical_jacobian", "use_analytical_presolve")
_SOLVER_STR_KEYS = ("optimizer",)

# Default channel weights used when a present channel is enabled automatically.
RECIPE_DEFAULT_TORQUE_WEIGHT = 0.2
RECIPE_DEFAULT_EE_POSE_WEIGHT = 0.5
RECIPE_DEFAULT_CONTACT_WEIGHT = 0.5

_SPACE_FLAG_NAMES = (
    "include_basic",
    "include_com_offsets",
    "include_inertia_log_cholesky",
    "include_joint_limit_scales",
    "include_per_link_mass",
    "include_command_delay",
)


def derive_residual_weights(
    trajectory: TrajectoryDataset,
    base: ResidualWeightConfig | None = None,
    *,
    auto_enable: bool = True,
) -> ResidualWeightConfig:
    """Derive residual channel weights from compatible telemetry channels.

    Position and velocity keep the base weights (or the 1.0/1.0 defaults). For the
    optional channels (torque, end-effector pose, contact force). A torque
    channel is compatible only when it is declared as link-side joint torque:

    - channel absent in the telemetry: weight forced to 0.0 regardless of base
      (a positive weight for a missing channel is never meaningful);
    - channel present with a positive base weight: base weight kept;
    - channel present with a zero base weight (or no base): enabled at the
      ``RECIPE_DEFAULT_*`` value when ``auto_enable`` is True, else kept at 0.

    ``sample_weights`` and ``end_effector_link_index`` pass through from the base.

    Args:
        trajectory: Loaded telemetry used to detect channel presence.
        base: Starting weights (for example from a preset); defaults when None.
        auto_enable: Whether present-but-unweighted channels are switched on.

    Returns:
        A new weight configuration; the base is not modified.
    """
    cfg = base if base is not None else ResidualWeightConfig()

    def resolve(present: bool, weight: float, default: float) -> float:
        if not present:
            return 0.0
        if weight > 0.0:
            return float(weight)
        return float(default) if auto_enable else 0.0

    usable_torque = (
        trajectory.torques is not None and resolve_effort_semantics(trajectory) == EFFORT_SEMANTICS_LINK_SIDE
    )
    return replace(
        cfg,
        torque_weight=resolve(usable_torque, cfg.torque_weight, RECIPE_DEFAULT_TORQUE_WEIGHT),
        end_effector_pose_weight=resolve(
            trajectory.end_effector_poses is not None, cfg.end_effector_pose_weight, RECIPE_DEFAULT_EE_POSE_WEIGHT
        ),
        contact_force_weight=resolve(
            trajectory.contact_forces is not None, cfg.contact_force_weight, RECIPE_DEFAULT_CONTACT_WEIGHT
        ),
    )


def recipe_parameter_type_values(preset: ParameterPreset) -> set[str]:
    """Parameter types a recipe selects by default.

    Joint drives and friction (plus the global mass scale) are always included;
    advanced families follow the preset's registry flags. Calibration-only types
    that need explicit intent (integral gain, command delay) are never
    auto-selected.

    Args:
        preset: Value supplied for ``preset``.

    Returns:
        Result produced by the operation.
    """
    values = {"joint_friction", "joint_stiffness", "joint_damping", "joint_armature", "link_mass"}
    if preset.include_com_offsets:
        values.update({"link_com_offset_x", "link_com_offset_y", "link_com_offset_z"})
    if preset.include_inertia_log_cholesky:
        values.add("link_inertia_log_cholesky")
    if preset.include_joint_limit_scales:
        values.update({"joint_limit_lower_scale", "joint_limit_upper_scale"})
    return values


def apply_recipe_to_run_spec(
    spec: SysIdRunSpec,
    preset: ParameterPreset,
    *,
    trajectory: TrajectoryDataset | None = None,
) -> list[str]:
    """Apply a preset/recipe to a run spec in place.

    Copies the parameter-space flags, maps the preset optimizer settings onto the
    solver spec, and applies residual weights — auto-derived from the telemetry when
    a trajectory is provided (see :func:`derive_residual_weights`).

    Args:
        spec: Run spec mutated in place.
        preset: Loaded preset/recipe.
        trajectory: Optional telemetry enabling data-derived residual weights.

    Returns:
        Human-readable notes describing derived or skipped settings (includes the
        preset's own notes).
    """
    notes: list[str] = []

    flags_changed = _apply_space_flags(spec, preset)
    if flags_changed and spec.parameters.selected:
        spec.parameters.selected = []
        notes.append("Parameter space flags changed; re-select parameters before running.")

    _apply_backend_config(spec, preset)
    notes.extend(_apply_simulation_section(spec, preset))
    notes.extend(_apply_solver_section(spec, preset))
    _apply_chunking_section(spec, preset)
    notes.extend(_apply_residual_weights(spec, preset, trajectory))
    notes.extend(str(note) for note in preset.notes)
    return notes


def infer_telemetry_source_type(telemetry_path: str) -> str:
    """Infer the telemetry source type from a path (csv, mcap, ros2_bag, or lerobot).

    Args:
        telemetry_path: Value supplied for ``telemetry_path``.

    Returns:
        Result produced by the operation.
    """
    path = Path(str(telemetry_path))
    suffix = path.suffix.lower()
    if suffix == ".csv":
        return "csv"
    if suffix == ".mcap":
        return "mcap"
    if path.is_dir() and ((path / "metadata.yaml").is_file() or list(path.glob("*.db3"))):
        return "ros2_bag"
    if suffix in (".db3", ".yaml", ".yml"):
        return "ros2_bag"
    if path.is_dir() and (path / "meta" / "info.json").is_file():
        return "lerobot"
    raise ValueError(f"Could not infer telemetry source type from '{telemetry_path}'.")


def synthesize_run_spec_from_quickstart(
    *,
    robot_prim_path: str,
    telemetry_path: str,
    recipe: ParameterPreset,
    source_env_path: str = "",
    mapping_path: str = "",
    stage_path: str = "",
    trajectory: TrajectoryDataset | None = None,
    num_joints: int | None = None,
    num_links: int | None = None,
) -> tuple[SysIdRunSpec, list[str]]:
    """Build a complete run spec from robot + telemetry + recipe (quick-start mode).

    Populates ``parameters.selected`` from the recipe's registry with the registry
    default bounds when the robot's joint/link counts are known, filtering families
    that the selected simulation backend rejects.

    Returns:
        ``(spec, notes)`` — notes include recipe application notes plus quick-start
        caveats (synthesized bounds are registry defaults; edit the saved spec to tune).

    Args:
        robot_prim_path: Value supplied for ``robot_prim_path``.
        telemetry_path: Value supplied for ``telemetry_path``.
        recipe: Value supplied for ``recipe``.
        source_env_path: Value supplied for ``source_env_path``.
        mapping_path: Value supplied for ``mapping_path``.
        stage_path: Value supplied for ``stage_path``.
        trajectory: Telemetry trajectory used by the operation.
        num_joints: Value supplied for ``num_joints``.
        num_links: Value supplied for ``num_links``.
    """
    spec = SysIdRunSpec()
    spec.stage.input_path = str(stage_path)
    spec.simulation.robot_prim_path = str(robot_prim_path)
    spec.telemetry.source_path = str(telemetry_path)
    spec.telemetry.source_type = infer_telemetry_source_type(telemetry_path)
    spec.telemetry.mapping_path = str(mapping_path)

    notes = apply_recipe_to_run_spec(spec, recipe, trajectory=trajectory)

    if source_env_path:
        spec.simulation.source_env_path = str(source_env_path)
    else:
        spec.simulation.parallel_clones = False
        notes.append("Parallel clones disabled; provide a clone source environment to enable them.")

    if num_joints is not None and num_joints >= 1:
        space = recipe.build_parameter_space(num_joints=int(num_joints), num_links=max(1, int(num_links or 1)))
        selected_types = recipe_parameter_type_values(recipe)
        entries = [item for item in space.specs if item.param_type.value in selected_types]
        capabilities = resolve_backend_capabilities(spec.simulation)
        if capabilities is not None:
            rejected = capabilities.unsupported_parameter_types([item.entry for item in entries])
            if rejected:
                notes.append(f"Dropped parameters unsupported by {capabilities.label}: {', '.join(rejected)}.")
                entries = [item for item in entries if item.param_type.value not in set(rejected)]
        else:
            notes.append(
                "Backend capabilities are unknown for the selected execution axes; parameter support was not "
                "checked and unsupported parameters may fail at solve time."
            )
        spec.parameters.selected = [
            ParameterRunSpec.from_entry(
                item.entry,
                initial=item.default_initial,
                min_value=item.default_min,
                max_value=item.default_max,
            )
            for item in entries
        ]
        notes.append("Synthesized parameter bounds are registry defaults; edit the saved run spec to tune them.")
    else:
        notes.append("Joint/link counts unknown; select parameters before solving.")
    return spec, notes


def _apply_space_flags(spec: SysIdRunSpec, preset: ParameterPreset) -> bool:
    changed = False
    for flag in _SPACE_FLAG_NAMES:
        value = bool(getattr(preset, flag))
        if bool(getattr(spec.parameters.space, flag)) != value:
            setattr(spec.parameters.space, flag, value)
            changed = True
    return changed


def _apply_backend_config(spec: SysIdRunSpec, preset: ParameterPreset) -> None:
    cfg = preset.build_optimizer_backend_config()
    solver = spec.solver
    solver.optimizer = cfg.backend.value
    solver.cma_population_size = cfg.cma_population_size
    solver.cma_sigma = float(cfg.cma_sigma)
    solver.cma_batch_size = cfg.cma_batch_size
    solver.cma_seed = int(cfg.cma_seed)
    solver.bo_initial_samples = int(cfg.bo_initial_samples)
    solver.bo_candidate_count = int(cfg.bo_candidate_count)
    solver.bo_batch_size = int(cfg.bo_batch_size)
    solver.bo_seed = int(cfg.bo_seed)
    solver.gd_learning_rate = float(cfg.gd_learning_rate)
    solver.gd_num_restarts = int(cfg.gd_num_restarts)
    solver.gd_seed = int(cfg.gd_seed)
    solver.use_analytical_jacobian = bool(cfg.use_analytical_jacobian)
    solver.use_analytical_presolve = bool(cfg.use_analytical_presolve)


def _apply_simulation_section(spec: SysIdRunSpec, preset: ParameterPreset) -> list[str]:
    section = dict(preset.simulation or {})
    if not section:
        return []
    notes: list[str] = []
    engine = section.get("engine")
    if engine is not None:
        if str(engine) in _VALID_ENGINES:
            spec.simulation.engine = str(engine)
        else:
            notes.append(f"Skipped unknown simulation engine '{engine}'.")
    newton_section = section.get("newton")
    if isinstance(newton_section, dict):
        solver = newton_section.get("solver")
        if solver is not None:
            if str(solver) in _VALID_NEWTON_SOLVERS:
                spec.simulation.newton.solver = str(solver)
            else:
                notes.append(f"Skipped unknown Newton solver '{solver}'.")
        feedforward = newton_section.get("feedforward")
        if feedforward is not None:
            if str(feedforward).strip().lower() in _VALID_FEEDFORWARD_MODES:
                spec.simulation.newton.feedforward = str(feedforward).strip().lower()
            else:
                notes.append(f"Skipped unknown Newton feedforward mode '{feedforward}'.")
        if "featherstone_substeps" in newton_section:
            spec.simulation.newton.featherstone_substeps = max(1, int(newton_section["featherstone_substeps"]))
        if "mujoco_substeps" in newton_section:
            spec.simulation.newton.mujoco_substeps = max(1, int(newton_section["mujoco_substeps"]))
        if "device" in newton_section:
            spec.simulation.newton.device = str(newton_section["device"])
        if "cuda_graph_capture" in newton_section:
            spec.simulation.newton.cuda_graph_capture = bool(newton_section["cuda_graph_capture"])
    if "parallel_clones" in section:
        spec.simulation.parallel_clones = bool(section["parallel_clones"])
    if "offline_stepping" in section:
        spec.simulation.offline_stepping = bool(section["offline_stepping"])
    actuator_runtime = section.get("actuator_runtime")
    if actuator_runtime is not None:
        if str(actuator_runtime) in _VALID_ACTUATOR_RUNTIMES:
            spec.simulation.actuator_runtime = str(actuator_runtime)
        else:
            notes.append(f"Skipped unknown actuator runtime '{actuator_runtime}'.")
    physics_backend = section.get("physics_backend")
    if physics_backend is not None:
        if str(physics_backend) in _VALID_PHYSICS_BACKENDS:
            spec.simulation.physics_backend = str(physics_backend)
        else:
            notes.append(f"Skipped unknown physics backend '{physics_backend}'.")
    actuator_policy = section.get("explicit_actuator_policy")
    if actuator_policy is not None:
        if str(actuator_policy) in _VALID_EXPLICIT_ACTUATOR_POLICIES:
            spec.simulation.explicit_actuator_policy = str(actuator_policy)
        else:
            notes.append(f"Skipped unknown explicit actuator policy '{actuator_policy}'.")
    return notes


def _apply_solver_section(spec: SysIdRunSpec, preset: ParameterPreset) -> list[str]:
    """Apply the dedicated ``solver`` section (wins over ``optimizer_backend`` fields).

    Args:
        spec: SysID run specification.
        preset: Value supplied for ``preset``.

    Returns:
        Result produced by the operation.
    """
    section = dict(preset.solver or {})
    if not section:
        return []
    notes: list[str] = []
    for key, value in section.items():
        if key in _SOLVER_INT_KEYS:
            setattr(spec.solver, key, int(value) if value is not None else None)
        elif key in _SOLVER_FLOAT_KEYS:
            setattr(spec.solver, key, float(value))
        elif key in _SOLVER_BOOL_KEYS:
            setattr(spec.solver, key, bool(value))
        elif key in _SOLVER_STR_KEYS:
            setattr(spec.solver, key, str(value))
        else:
            notes.append(f"Skipped unknown solver setting '{key}'.")
    return notes


def _apply_chunking_section(spec: SysIdRunSpec, preset: ParameterPreset) -> None:
    """Record the auto-split policy; chunks materialize at prepare time (idempotent).

    Args:
        spec: SysID run specification.
        preset: Value supplied for ``preset``.
    """
    section = dict(preset.chunking or {})
    if str(section.get("mode", "")).lower() != "auto":
        return
    spec.telemetry.auto_split = True
    if "train_fraction" in section:
        spec.telemetry.auto_split_train_fraction = float(section["train_fraction"])
    if "min_chunk_seconds" in section:
        spec.telemetry.auto_split_min_chunk_seconds = float(section["min_chunk_seconds"])


def _apply_residual_weights(
    spec: SysIdRunSpec,
    preset: ParameterPreset,
    trajectory: TrajectoryDataset | None,
) -> list[str]:
    base = preset.build_residual_weight_config()
    if trajectory is None:
        spec.residuals = ResidualsRunSpec.from_config(base)
        return ["Residual weights applied without telemetry; load telemetry to auto-derive channel weights."]

    # Absent channels are always zeroed (never meaningful); present-but-unweighted
    # channels are only switched on in "auto" mode.
    derived = derive_residual_weights(trajectory, base, auto_enable=preset.residual_weights_mode == "auto")
    notes: list[str] = []
    torque_semantics = resolve_effort_semantics(trajectory)
    for channel, base_weight, derived_weight in (
        ("torque", base.torque_weight, derived.torque_weight),
        ("end-effector pose", base.end_effector_pose_weight, derived.end_effector_pose_weight),
        ("contact force", base.contact_force_weight, derived.contact_force_weight),
    ):
        if channel == "torque" and trajectory.torques is not None and torque_semantics != EFFORT_SEMANTICS_LINK_SIDE:
            notes.append(
                "Torque residual disabled: telemetry declares "
                f"torque_semantics='{torque_semantics}', not link-side joint torque."
            )
        elif base_weight > 0.0 and derived_weight == 0.0:
            notes.append(f"{channel.capitalize()} residual disabled: telemetry has no {channel} channel.")
        elif base_weight == 0.0 and derived_weight > 0.0:
            notes.append(f"{channel.capitalize()} residual enabled at weight {derived_weight:g}: channel present.")
    spec.residuals = ResidualsRunSpec.from_config(derived)
    return notes
