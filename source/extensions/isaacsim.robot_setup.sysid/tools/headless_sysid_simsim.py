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

"""Sim-sim validation: identify KNOWN parameter perturbations from synthetic telemetry.

Three phases in one headless session:

1. **Generate** — roll the robot out with a known ``truth`` parameter vector
   (perturbed from nominal) using a generator bridge, at the base run spec's
   command excitation, inject sensor noise, and write CSV telemetry through the
   standard ingestion contract.
2. **Identify** — run the untouched production pipeline (preflight, prepare,
   solve) on the synthetic telemetry, starting from NOMINAL initials.
3. **Score** — compare recovered parameters against the truth, gate the
   controller-independent plant parameters, and correlate recovery error with
   the post-solve confidence verdicts.

Position-domain fitting is self-consistent under many model defects (axis/unit
errors, joint permutations); recovering *known* parameters is the only check
that fails loudly on the whole class. Run this after pipeline changes and
before trusting a new robot/telemetry configuration.

``--generation-bridge mujoco`` generates the truth telemetry with the
sampling-based Newton-MuJoCo bridge (implicit integrator, no substeps —
roughly 30x faster than high-substep Featherstone) while identification stays
on the differentiable Featherstone bridge. Generating and identifying with
different solver families breaks the same-family "inverse crime" of the
default mode: integrator- or solver-specific artifacts can no longer cancel
between generator and estimator. The MuJoCo bridge rolls only joint-level
parameters (friction/stiffness/damping/armature); inertial truth
perturbations are skipped (kept at nominal) and marked in the truth table.

Example:
    python tools/headless_sysid_simsim.py \
        --run-spec C:/path/to/sysid_run_spec.json \
        --output-dir C:/path/to/simsim_out
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

# Reuse the solve tool's Kit bootstrap helpers (same directory).
sys.path.insert(0, str(Path(__file__).resolve().parent))
import headless_sysid_solve as _solve_tool  # noqa: E402

# Deterministic truth perturbations per parameter family (multiplier deltas or
# absolute offsets, cycled per DOF/link). All values stay inside typical bounds.
_TRUTH_STIFFNESS = [0.85, 1.15, 0.90, 1.10, 0.95, 1.05, 0.88]
_TRUTH_DAMPING = [1.20, 1.30, 1.15, 1.25, 1.10, 1.20, 1.35]
_TRUTH_FRICTION = [1.40, 0.80, 1.20, 0.90, 1.30, 1.10, 0.70]
_TRUTH_MASS_SCALE = 1.08
_TRUTH_COM_OFFSET = 0.012  # meters, sign alternates per entry
_TRUTH_LC_DELTA = 0.20  # log-Cholesky delta on top of the spec initial

_DEFAULT_GATES = {
    "link_mass": 0.03,  # relative
    "joint_friction": 0.15,  # relative
    "joint_stiffness": 0.30,  # relative
    "joint_damping": 0.30,  # relative
    "link_com_offset": 0.005,  # absolute meters
}

# Under full inverse-dynamics feedforward the correction gains, friction, and CoM
# offsets are only weakly excited (the feedforward supplies the motion torque, so
# the cost reaches the noise floor almost independently of them — measured on the
# Franka baseline: gains 50-90% off at noise-floor cost). Gate only what the
# structure can identify; everything else is report-only.
_ID_FF_GATES = {
    "link_mass": 0.06,  # relative; nominal-model feedforward biases recovery low
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-spec", required=True, help="Base SysIdRunSpec JSON (robot, stage, commands source).")
    parser.add_argument("--output-dir", required=True, help="Directory for synthetic telemetry + reports.")
    parser.add_argument(
        "--result-json", default="", help="Score report path (default: <output-dir>/simsim_report.json)."
    )
    parser.add_argument(
        "--generation-bridge",
        choices=("featherstone_diff", "mujoco"),
        default="featherstone_diff",
        help="Bridge that generates the truth telemetry. 'mujoco' uses the sampling-based "
        "Newton-MuJoCo bridge (implicit integrator, no substeps, ~30x faster) so identification "
        "with the Featherstone diff bridge is cross-solver — no same-family inverse crime.",
    )
    parser.add_argument(
        "--generator-substeps", type=int, default=64, help="Featherstone substeps for generation (diff bridge only)."
    )
    parser.add_argument(
        "--generator-feedforward", default="", help="Generator feedforward mode (default: identify mode)."
    )
    parser.add_argument("--identify-feedforward", default="", help="Identification feedforward (default: base spec).")
    parser.add_argument("--iterations", type=int, default=0, help="Override solver iterations (0 = base spec).")
    parser.add_argument("--noise-position", type=float, default=2e-4, help="Position noise sigma (rad).")
    parser.add_argument("--noise-velocity", type=float, default=2e-3, help="Velocity noise sigma (rad/s).")
    parser.add_argument("--noise-torque", type=float, default=0.3, help="Torque noise sigma (Nm).")
    parser.add_argument("--rate-divisor", type=int, default=1, help="Keep every Nth sample (rate ablation).")
    parser.add_argument("--seed", type=int, default=7, help="Noise RNG seed.")
    parser.add_argument("--no-enable-extension", action="store_true")
    parser.add_argument("--skip-timeline-start", action="store_true")
    parser.add_argument("--timeline-warmup-updates", type=int, default=0)
    parser.add_argument("--rate-log-interval", type=float, default=30.0)
    return parser.parse_args()


def _build_truth_theta(spec, entries, theta_min, theta_max) -> tuple[list[float], list[dict[str, Any]]]:  # noqa: ANN001
    """Deterministic, bounds-respecting truth vector plus a per-entry table.

    Args:
        spec: SysID run specification.
        entries: Parameter entries handled by the operation.
        theta_min: Lower optimizer bounds.
        theta_max: Upper optimizer bounds.

    Returns:
        Truth parameter vector and its per-entry manifest.
    """  # noqa: DOC106, DOC107
    import numpy as np

    mins = theta_min.detach().cpu().numpy().astype(np.float64).reshape(-1)
    maxs = theta_max.detach().cpu().numpy().astype(np.float64).reshape(-1)
    counters: dict[str, int] = {}
    truth: list[float] = []
    table: list[dict[str, Any]] = []
    for index, (entry, item) in enumerate(zip(entries, spec.parameters.selected)):
        family = entry.param_type.value
        k = counters.get(family, 0)
        counters[family] = k + 1
        initial = float(item.initial)
        if family == "joint_stiffness":
            value = _TRUTH_STIFFNESS[k % len(_TRUTH_STIFFNESS)]
        elif family == "joint_damping":
            value = _TRUTH_DAMPING[k % len(_TRUTH_DAMPING)]
        elif family == "joint_friction":
            value = _TRUTH_FRICTION[k % len(_TRUTH_FRICTION)]
        elif family == "link_mass":
            value = _TRUTH_MASS_SCALE
        elif family.startswith("link_com_offset"):
            value = _TRUTH_COM_OFFSET * (1.0 if k % 2 == 0 else -1.0)
        elif family == "link_inertia_log_cholesky":
            value = initial + (_TRUTH_LC_DELTA if k % 3 == 0 else 0.0)
        else:
            value = initial
        value = float(np.clip(value, mins[index], maxs[index]))
        truth.append(value)
        table.append(
            {
                "index": index,
                "param_type": family,
                "dof_index": int(entry.dof_index),
                "link_index": int(entry.link_index),
                "component_index": int(entry.component_index),
                "initial": initial,
                "truth": value,
            }
        )
    return truth, table


async def _generate(
    spec: Any,
    stage: Any,
    trajectory: Any,
    args: argparse.Namespace,
    out_dir: Path,
    bridge_holder: list[Any],
) -> dict[str, Any]:
    """Roll out the truth model over the spec's chunk windows and write CSV telemetry.

    Args:
        spec: SysID run specification.
        stage: USD stage used by the operation.
        trajectory: Telemetry trajectory used by the operation.
        args: Parsed sim-to-sim command arguments.
        out_dir: Directory receiving generated telemetry and metadata.
        bridge_holder: Caller-owned list used to guarantee bridge cleanup.

    Returns:
        Generated telemetry paths and truth parameter metadata.
    """  # noqa: DOC107
    import numpy as np
    import torch
    from isaacsim.robot_setup.sysid.run_controller import (
        SysIdRunController,
        build_parameter_space,
        optimizer_vectors,
    )
    from isaacsim.robot_setup.sysid.trajectory_segments import build_trajectory_chunks

    parameter_space, link_paths, joint_paths = build_parameter_space(spec, stage, trajectory)
    entries, theta_initial, theta_min, theta_max = optimizer_vectors(spec)
    truth, table = _build_truth_theta(spec, entries, theta_min, theta_max)

    generator_mode = (args.generator_feedforward or "").strip() or str(
        getattr(spec.simulation.newton, "feedforward", "none") or "none"
    )
    generation_bridge = str(getattr(args, "generation_bridge", "featherstone_diff") or "featherstone_diff")
    rollout_entries = entries
    if generation_bridge == "mujoco":
        from isaacsim.robot_setup.sysid.newton_sysid_bridge import (
            _NEWTON_SUPPORTED_PARAMETERS,
        )
        from isaacsim.robot_setup.sysid.run_spec import NEWTON_SOLVER_MUJOCO

        # The MuJoCo bridge rolls only joint-level parameters; inertial truth
        # perturbations cannot be applied, so their truth IS the nominal initial
        # (recorded in the table so scoring stays honest).
        kept = [i for i, entry in enumerate(entries) if entry.param_type in _NEWTON_SUPPORTED_PARAMETERS]
        for index in range(len(entries)):
            if index not in kept:
                truth[index] = float(table[index]["initial"])
                table[index]["truth"] = truth[index]
                table[index]["generator_applied"] = False
        if len(kept) < len(entries):
            skipped = sorted({entries[i].param_type.value for i in range(len(entries)) if i not in kept})
            print(
                f"[generate] mujoco bridge: {len(entries) - len(kept)} truth perturbations kept at "
                f"nominal (unsupported families: {', '.join(skipped)})",
                flush=True,
            )
        rollout_entries = [entries[i] for i in kept]
        rollout_truth = [truth[i] for i in kept]
        generator_config = replace(
            spec.simulation.newton,
            solver=NEWTON_SOLVER_MUJOCO,
            feedforward=generator_mode,
        )
    else:
        from isaacsim.robot_setup.sysid.run_spec import NEWTON_SOLVER_FEATHERSTONE_DIFF

        rollout_truth = truth
        generator_config = replace(
            spec.simulation.newton,
            solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
            featherstone_substeps=int(args.generator_substeps),
            feedforward=generator_mode,
        )

    generator_spec = replace(
        spec,
        simulation=replace(spec.simulation, newton=generator_config),
    )
    generator_controller = SysIdRunController(generator_spec)
    bridge = generator_controller._create_bridge(
        stage=stage,
        parameter_space=parameter_space,
        link_paths=link_paths,
        joint_paths=joint_paths,
        residual_cfg=generator_spec.residuals.to_config(),
    )
    bridge_holder.append(bridge)

    rng = np.random.default_rng(int(args.seed))
    chunks = build_trajectory_chunks(trajectory, spec.telemetry.chunks)
    num_dof = int(trajectory.num_joints)
    frames: list[dict[str, np.ndarray]] = []
    truth_tensor = torch.tensor([rollout_truth], dtype=torch.float32)
    from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset

    for chunk in chunks:
        chunk_traj = chunk.trajectory
        steps = int(chunk_traj.commands.shape[0])
        commands = torch.as_tensor(chunk_traj.commands, dtype=torch.float32)
        # Fixed-point generation: a real feedforward controller reads back the
        # robot's OWN state, while the bridge reads the configured trajectory.
        # Iterate until the feedforward/velocity-target source equals the
        # produced telemetry — the exact assumption the estimator makes later.
        # Bridge row k is the state at measured sample k (row 0 = the initial
        # state seeded from the source trajectory), so the produced telemetry
        # needs no row-0 pinning to stay aligned with the estimator.
        source = chunk_traj
        result = None
        for _ in range(3):
            bridge.set_trajectory(source)
            result = await bridge.run_rollout_async(truth_tensor, rollout_entries, commands, steps)
            iter_positions = result.positions[0].detach().cpu().numpy().astype(np.float64)
            iter_velocities = result.velocities[0].detach().cpu().numpy().astype(np.float64)
            source = TrajectoryDataset(
                times=np.asarray(chunk_traj.times[: iter_positions.shape[0]], dtype=np.float64),
                positions=iter_positions,
                velocities=iter_velocities,
                commands=np.asarray(chunk_traj.commands[: iter_positions.shape[0]], dtype=np.float64),
            )
        steps = int(result.positions.shape[1])
        positions = result.positions[0].detach().cpu().numpy().astype(np.float64)
        velocities = result.velocities[0].detach().cpu().numpy().astype(np.float64)
        torques = result.torques[0].detach().cpu().numpy().astype(np.float64)
        if not (np.all(np.isfinite(positions)) and np.all(np.isfinite(torques))):
            raise RuntimeError(f"Truth rollout produced non-finite telemetry on chunk '{chunk.spec.name}'.")
        positions = positions + rng.normal(0.0, args.noise_position, positions.shape)
        velocities = velocities + rng.normal(0.0, args.noise_velocity, velocities.shape)
        torques = torques + rng.normal(0.0, args.noise_torque, torques.shape)
        frames.append(
            {
                "times": np.asarray(chunk_traj.times[:steps], dtype=np.float64),
                "positions": positions,
                "velocities": velocities,
                "commands": np.asarray(chunk_traj.commands[:steps], dtype=np.float64),
                "torques": torques,
            }
        )
        print(
            f"[generate] chunk '{chunk.spec.name}': {steps} steps, " f"|tau| mean {np.abs(torques).mean():.2f} Nm",
            flush=True,
        )

    divisor = max(1, int(args.rate_divisor))
    times = np.concatenate([frame["times"][::divisor] for frame in frames])
    columns: dict[str, np.ndarray] = {"time": times}
    joint_names = [f"joint_{i}" for i in range(num_dof)]
    metadata = getattr(trajectory, "metadata", None)
    extra = getattr(metadata, "extra", None)
    if isinstance(extra, dict):
        nested = extra.get("topic_mapping")
        if isinstance(nested, dict) and nested.get("joint_names"):
            joint_names = [str(n) for n in nested["joint_names"][:num_dof]]
    column_prefix = {"positions": "position", "velocities": "velocity", "commands": "command", "torques": "torque"}
    for signal, prefix in column_prefix.items():
        stacked = np.concatenate([frame[signal][::divisor] for frame in frames], axis=0)
        for j in range(num_dof):
            columns[f"{prefix}_{j}"] = stacked[:, j]

    csv_path = out_dir / "simsim_telemetry.csv"
    header = ",".join(columns.keys())
    matrix = np.stack(list(columns.values()), axis=1)
    # %.15g keeps sub-millisecond spacing on epoch-seconds timestamps.
    np.savetxt(csv_path, matrix, delimiter=",", header=header, comments="", fmt="%.15g")

    mapping_path = out_dir / "simsim_column_map.json"
    mapping = {
        "time_column": "time",
        "position_columns": [f"position_{j}" for j in range(num_dof)],
        "velocity_columns": [f"velocity_{j}" for j in range(num_dof)],
        "command_columns": [f"command_{j}" for j in range(num_dof)],
        "torque_columns": [f"torque_{j}" for j in range(num_dof)],
        "torque_semantics": "link_side",
        "joint_names": joint_names,
    }
    mapping_path.write_text(json.dumps(mapping, indent=2), encoding="utf-8")

    manifest = {
        "truth": table,
        "generator": {
            "bridge": generation_bridge,
            "substeps": int(args.generator_substeps) if generation_bridge == "featherstone_diff" else None,
            "feedforward": generator_mode,
            "noise": {
                "position": args.noise_position,
                "velocity": args.noise_velocity,
                "torque": args.noise_torque,
            },
            "rate_divisor": divisor,
            "seed": int(args.seed),
        },
    }
    (out_dir / "simsim_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"[generate] telemetry: {csv_path} ({times.shape[0]} samples)", flush=True)
    return {"csv_path": csv_path, "mapping_path": mapping_path, "truth": truth, "truth_table": table}


async def _cleanup_generation_bridges(bridges: list[Any]) -> None:
    """Release rollout buffers and clone resources owned by generation bridges.

    Args:
        bridges: Generation bridges to release in reverse creation order.
    """
    for bridge in reversed(bridges):
        release = getattr(bridge, "release_rollout_memory", None)
        if callable(release):
            release()
        cleanup_async = getattr(bridge, "cleanup_async", None)
        if callable(cleanup_async):
            await cleanup_async(remove_clones=True)
            continue
        cleanup = getattr(bridge, "cleanup", None)
        if callable(cleanup):
            cleanup(remove_clones=True)


def _synthesize_identify_spec(spec, generated: dict[str, Any], args, out_dir: Path):  # noqa: ANN001, ANN202
    """Clone the base spec, pointing telemetry at the synthetic CSV.

    Args:
        spec: SysID run specification.
        generated: Generated telemetry paths and truth metadata.
        args: Parsed sim-to-sim command arguments.
        out_dir: Directory receiving the synthesized run specification.

    Returns:
        Run specification configured to identify the generated telemetry.
    """  # noqa: DOC107
    from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec

    payload = spec.to_dict()
    payload["telemetry"]["source_type"] = "csv"
    payload["telemetry"]["source_path"] = str(generated["csv_path"])
    payload["telemetry"]["mapping_path"] = str(generated["mapping_path"])
    payload["sampling"]["mode"] = "off"
    payload["outputs"]["apply_parameters_to_usd"] = False
    payload["outputs"]["write_usd_provenance"] = False
    payload["outputs"]["render_validation_animation"] = False
    if args.identify_feedforward.strip():
        payload["simulation"]["newton"]["feedforward"] = args.identify_feedforward.strip()
    if int(args.iterations) > 0:
        payload["solver"]["max_iterations"] = int(args.iterations)
    identify_spec = SysIdRunSpec.from_dict(payload)
    spec_path = out_dir / "simsim_identify_spec.json"
    spec_path.write_text(json.dumps(identify_spec.to_dict(), indent=2), encoding="utf-8")
    print(f"[identify] spec: {spec_path}", flush=True)
    return identify_spec


async def _identify(identify_spec, stage, args) -> dict[str, Any]:  # noqa: ANN001
    """Run the untouched production pipeline on the synthetic telemetry.

    Args:
        identify_spec: Run specification for the identification pass.
        stage: USD stage used by the operation.
        args: Parsed sim-to-sim command arguments.

    Returns:
        Identification result and report payload.
    """  # noqa: DOC106, DOC107
    from isaacsim.robot_setup.sysid.execution import run_sysid
    from isaacsim.robot_setup.sysid.run_controller import SysIdRunController

    controller = SysIdRunController(identify_spec)
    trajectory = controller.load_trajectory()
    print(f"[identify] synthetic trajectory: {int(trajectory.times.shape[0])} samples", flush=True)
    preflight = controller.preflight(
        stage=stage,
        trajectory=trajectory,
        timeline_playing=(None if args.skip_timeline_start else True),
    )
    for line in preflight.summary_lines():
        print(f"[preflight] {line}", flush=True)
    preflight.raise_for_errors()
    prepared = controller.prepare(
        stage,
        trajectory=trajectory,
        timeline_playing=(None if args.skip_timeline_start else True),
    )
    check_report = getattr(prepared, "check_report", None)
    if check_report is not None:
        for line in check_report.summary_lines():
            print(f"[check] {line}", flush=True)

    def on_iteration(status) -> None:  # noqa: ANN001
        accepted = "accepted" if status.accepted else "rejected"
        print(f"[iter {status.iteration}] cost={status.cost:.6e} {accepted}", flush=True)

    result = await run_sysid(identify_spec, on_iteration=on_iteration, prepared_run=prepared)
    return {"result": result, "prepared": prepared}


def _score(identified: dict[str, Any], truth_table: list[dict[str, Any]], gates: dict[str, float]) -> dict[str, Any]:
    result = identified["result"]
    recovered = [float(v) for v in result.selected_status.theta]
    confidence = getattr(result, "parameter_confidence", {}) or {}
    verdict_by_index = {
        int(entry.get("index", position)): str(entry.get("verdict", "unknown"))
        for position, entry in enumerate(confidence.get("entries", []))
        if isinstance(entry, dict)
    }

    rows: list[dict[str, Any]] = []
    failures: list[str] = []
    for item in truth_table:
        index = int(item["index"])
        truth = float(item["truth"])
        value = recovered[index] if index < len(recovered) else float("nan")
        family = str(item["param_type"])
        absolute_error = abs(value - truth)
        relative_error = absolute_error / max(abs(truth), 1e-9)
        gate_key = "link_com_offset" if family.startswith("link_com_offset") else family
        gate = gates.get(gate_key)
        gate_kind = "absolute" if gate_key == "link_com_offset" else "relative"
        error_for_gate = absolute_error if gate_kind == "absolute" else relative_error
        passed = None if gate is None else bool(error_for_gate <= gate)
        if passed is False:
            failures.append(
                f"{family}[{index}] truth={truth:.4g} recovered={value:.4g} "
                f"{gate_kind} err={error_for_gate:.3g} > gate {gate:.3g}"
            )
        rows.append(
            {
                **item,
                "recovered": value,
                "absolute_error": absolute_error,
                "relative_error": relative_error,
                "gate": gate,
                "gate_kind": gate_kind if gate is not None else None,
                "passed": passed,
                "confidence_verdict": verdict_by_index.get(index, "unknown"),
            }
        )

    families: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        families.setdefault(row["param_type"], []).append(row)
    print("\n[score] recovery by family:", flush=True)
    for family, group in families.items():
        gated = [row for row in group if row["passed"] is not None]
        max_rel = max(row["relative_error"] for row in group)
        max_abs = max(row["absolute_error"] for row in group)
        status = "PASS" if all(row["passed"] is not False for row in gated) else "FAIL"
        marker = status if gated else "report"
        print(
            f"  {family:28s} n={len(group):2d} max_rel={max_rel:8.3g} max_abs={max_abs:8.3g} [{marker}]",
            flush=True,
        )

    # Confidence-verdict correlation: well_constrained entries should recover better.
    by_verdict: dict[str, list[float]] = {}
    for row in rows:
        by_verdict.setdefault(row["confidence_verdict"], []).append(row["relative_error"])
    verdict_summary = {
        verdict: {"count": len(errors), "median_relative_error": float(sorted(errors)[len(errors) // 2])}
        for verdict, errors in by_verdict.items()
    }
    print("[score] median relative error by confidence verdict:", flush=True)
    for verdict, stats in verdict_summary.items():
        print(f"  {verdict:20s} n={stats['count']:2d} median_rel={stats['median_relative_error']:.3g}", flush=True)
    noise_floor = confidence.get("noise_floor") or {}
    if noise_floor.get("at_noise_floor"):
        ratio = noise_floor.get("cost_to_floor_ratio")
        ratio_text = f" (cost/floor ratio {ratio:.2g})" if isinstance(ratio, (int, float)) else ""
        print(
            f"[score] confidence verdicts flagged at noise floor{ratio_text} — "
            "verdict/recovery correlation is not expected to hold on this run.",
            flush=True,
        )

    ok = not failures
    for failure in failures:
        print(f"[score] GATE FAIL: {failure}", flush=True)
    print(f"[score] overall: {'PASS' if ok else 'FAIL'}", flush=True)
    return {
        "ok": ok,
        "gates": gates,
        "rows": rows,
        "verdict_summary": verdict_summary,
        "noise_floor": noise_floor,
        "failures": failures,
        "final_cost": float(result.selected_status.cost),
    }


async def _run(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = Path(args.output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.no_enable_extension:
        await _solve_tool._enable_extension()
    _solve_tool._ensure_extension_package_visible()

    from isaacsim.robot_setup.sysid.run_controller import SysIdRunController

    spec = _solve_tool._load_spec(Path(args.run_spec))
    stage = await _solve_tool._open_stage(spec.stage.input_path)
    controller = SysIdRunController(spec)
    trajectory = controller.load_trajectory()
    print(f"[simsim] base trajectory: {int(trajectory.times.shape[0])} samples", flush=True)
    if not args.skip_timeline_start:
        await _solve_tool._start_timeline(args.timeline_warmup_updates)

    generation_bridges: list[Any] = []
    try:
        generated = await _generate(spec, stage, trajectory, args, out_dir, generation_bridges)
    finally:
        await _cleanup_generation_bridges(generation_bridges)
    # The generation bridge's rollout buffers (per-substep states/controls, and
    # captured graphs when the shape fits the launch budget) are several GiB on
    # a 16 GB GPU and are only reclaimed once reference cycles are collected.
    # Free them before identification so the solve + confidence stages get the
    # full card (measured: confidence OOMs otherwise).
    import gc

    gc.collect()
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:
        pass
    identify_spec = _synthesize_identify_spec(spec, generated, args, out_dir)
    identified = await _identify(identify_spec, stage, args)
    identify_mode = str(getattr(identify_spec.simulation.newton, "feedforward", "") or "").strip().lower()
    gates = dict(_ID_FF_GATES if identify_mode == "inverse_dynamics" else _DEFAULT_GATES)
    report = _score(identified, generated["truth_table"], gates)
    report["manifest"] = json.loads((out_dir / "simsim_manifest.json").read_text(encoding="utf-8"))

    result_path = Path(args.result_json) if args.result_json else out_dir / "simsim_report.json"
    _solve_tool._write_json(result_path, report)
    print(f"[simsim] report: {result_path}", flush=True)
    return report


def main() -> int:  # noqa: D103
    args = _parse_args()
    try:
        from isaacsim import SimulationApp
    except Exception:
        code = _solve_tool._rerun_with_bundled_python()
        if code is not None:
            return code
        print(
            "[simsim] ERROR: could not import isaacsim.SimulationApp and no bundled Isaac Sim Python "
            "launcher was found.",
            flush=True,
        )
        return 2

    app = SimulationApp({"headless": True})
    exit_code = 1
    try:
        loop = asyncio.get_event_loop()
        task = loop.create_task(_run(args))
        print("[simsim] Kit update pump started.", flush=True)
        while not task.done():
            app.update()
        print("[simsim] Kit update pump stopped.", flush=True)
        report = task.result()
        exit_code = 0 if report.get("ok") else 3
    except Exception as exc:
        print(f"[simsim] ERROR: {exc}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        # Fast shutdown terminates the process inside `close()`, so the status must be passed here.
        app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
