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

"""Run a System Identification solve from a headless Isaac Sim session.

Example:
    python tools/headless_sysid_solve.py \
        --run-spec C:/path/to/sysid_run_spec.json \
        --result-json C:/path/to/sysid_result.json \
        --save-stage

The run spec is the same JSON contract used by the extension API:
``isaacsim.robot_setup.sysid.SysIdRunSpec``.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import json
import os
import subprocess
import sys
import tempfile
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-spec",
        default="",
        help="Path to a SysIdRunSpec JSON file (omit when using the --robot/--telemetry quick start).",
    )
    parser.add_argument(
        "--robot",
        default="",
        help="Quick start: robot articulation prim path (pairs with --telemetry and --recipe).",
    )
    parser.add_argument(
        "--telemetry",
        default="",
        help="Quick start: telemetry source path (CSV/MCAP/ROS 2 bag/LeRobot).",
    )
    parser.add_argument(
        "--recipe",
        default="manipulator",
        help="Quick start: built-in recipe/preset name (default: manipulator).",
    )
    parser.add_argument(
        "--source-env",
        default="",
        help=(
            "Quick start: source environment prim for parallel clones, such as /World/envs/env_0. "
            "Parallel cloning is disabled when omitted."
        ),
    )
    parser.add_argument(
        "--mapping",
        default="",
        help="Quick start: optional topic/column mapping JSON or YAML path.",
    )
    parser.add_argument(
        "--save-run-spec",
        default="",
        help=(
            "Quick start: output path for the synthesized run spec JSON "
            "(default: sysid_run_spec_synthesized.json next to the result)."
        ),
    )
    parser.add_argument(
        "--stage",
        default="",
        help="USD stage to open before solving. Overrides stage.input_path from the run spec.",
    )
    parser.add_argument(
        "--result-json",
        default="",
        help="Optional path for a compact solve summary JSON.",
    )
    parser.add_argument(
        "--checkpoint-json",
        default="",
        help=(
            "Optional path for a crash-safe progress checkpoint JSON. If omitted and --result-json is set, "
            "<result_stem>_checkpoint.json is written next to the result."
        ),
    )
    parser.add_argument(
        "--save-stage",
        action="store_true",
        help="Save the open stage after optimized parameters and provenance are authored.",
    )
    parser.add_argument(
        "--no-enable-extension",
        action="store_true",
        help="Do not ask Kit's extension manager to enable isaacsim.robot_setup.sysid.",
    )
    parser.add_argument(
        "--timeline-warmup-updates",
        type=int,
        default=0,
        help="Number of Kit updates to wait after pressing Play before solving.",
    )
    parser.add_argument(
        "--skip-timeline-start",
        action="store_true",
        help="Diagnostic only: skip pressing Play before preflight/prepare.",
    )
    parser.add_argument(
        "--rate-log-interval",
        type=float,
        default=2.0,
        help="Minimum seconds between solve throughput proxy log lines.",
    )
    return parser.parse_args()


def _extension_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _extension_python_package_root() -> Path:
    return _extension_root() / "isaacsim"


def _isaac_sim_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _bundled_python_launchers() -> list[Path]:
    root = _isaac_sim_root()
    if os.name == "nt":
        platforms = ("windows-x86_64",)
        launcher = "python.bat"
    else:
        platforms = ("linux-x86_64", "linux-aarch64")
        launcher = "python.sh"
    candidates = [root / "_build" / platform / "release" / launcher for platform in platforms]
    candidates.append(root / launcher)
    candidates.extend(root / "source" / "scripts" / "python" / platform / launcher for platform in platforms)
    return candidates


def _rerun_with_bundled_python() -> int | None:
    for launcher in _bundled_python_launchers():
        if launcher.is_file():
            print(f"[sysid] Re-running with Isaac Sim Python: {launcher}", flush=True)
            command = [str(launcher), str(Path(__file__).resolve()), *sys.argv[1:]]
            return subprocess.call(command, cwd=os.getcwd())
    return None


def _load_spec_and_payload(path: Path):  # noqa: ANN202
    from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec
    from isaacsim.robot_setup.sysid.run_spec_paths import resolve_run_spec_local_paths

    resolved_path = path.expanduser().resolve()
    with resolved_path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    spec = SysIdRunSpec.from_dict(payload)
    resolve_run_spec_local_paths(spec, resolved_path.parent)
    return spec, payload


def _load_spec(path: Path):  # noqa: ANN202
    return _load_spec_and_payload(path)[0]


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.replace(path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def _checkpoint_output_path(result_json: str, checkpoint_json: str) -> Path | None:
    if checkpoint_json:
        return Path(checkpoint_json).expanduser().resolve()
    if not result_json:
        return None
    result_path = Path(result_json).expanduser().resolve()
    return result_path.with_name(f"{result_path.stem}_checkpoint.json")


def _status_to_dict(status, *, source: str = "") -> dict[str, Any]:  # noqa: ANN001
    payload = {
        "iteration": int(status.iteration),
        "cost": float(status.cost),
        "damping": float(status.damping),
        "accepted": bool(status.accepted),
        "theta": [float(value) for value in status.theta],
    }
    if source:
        payload["source"] = source
    return payload


def _preflight_to_dict(preflight) -> dict[str, Any]:  # noqa: ANN001
    if preflight is None:
        return {"ok": None, "issues": []}
    return {
        "ok": bool(preflight.ok),
        "issues": [
            {
                "code": issue.code,
                "severity": issue.severity,
                "message": issue.message,
            }
            for issue in preflight.issues
        ],
    }


def _checkpoint_payload(
    *,
    spec,  # noqa: ANN001
    prepared,  # noqa: ANN001
    schema_issues,  # noqa: ANN001
    telemetry_quality,  # noqa: ANN001
    last_status,  # noqa: ANN001
    best_known_status,  # noqa: ANN001
    progress_fraction: float,
    checkpoint_status: str,
    error: BaseException | None = None,
) -> dict[str, Any]:
    from isaacsim.robot_setup.sysid.analysis import parameter_rows
    from isaacsim.robot_setup.sysid.schema_validation import schema_issues_to_dicts

    best_theta = None
    if best_known_status is not None:
        best_theta = [float(value) for value in best_known_status.theta]
    elif prepared is not None:
        best_theta = [float(value) for value in prepared.config.theta_initial.detach().cpu().reshape(-1).tolist()]
    payload = {
        "ok": error is None,
        "checkpoint_schema_version": 1,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "last_status": checkpoint_status,
        "progress_fraction": max(0.0, min(1.0, float(progress_fraction))),
        "backend": getattr(prepared, "backend_label", ""),
        "simulation_engine": getattr(prepared, "simulation_engine", spec.simulation.engine),
        "robot_prim_path": spec.simulation.robot_prim_path,
        "telemetry_source": spec.telemetry.source_path,
        "parameter_rows": parameter_rows(prepared.config) if prepared is not None else [],
        "best_known_theta": best_theta or [],
        "last_iteration_status": _status_to_dict(last_status) if last_status is not None else None,
        "best_known_status": _status_to_dict(best_known_status) if best_known_status is not None else None,
        "preflight": _preflight_to_dict(getattr(prepared, "preflight", None)),
        "schema_issues": schema_issues_to_dicts(schema_issues or []),
        "telemetry_quality": telemetry_quality.to_dict() if telemetry_quality is not None else {},
        "train_chunks": _chunk_summaries(getattr(prepared, "train_chunks", []) if prepared is not None else []),
        "validation_chunks": _chunk_summaries(
            getattr(prepared, "validation_chunks", []) if prepared is not None else []
        ),
        "resampling_diagnostics": (
            [
                diagnostic.to_dict() if hasattr(diagnostic, "to_dict") else dict(diagnostic)
                for diagnostic in (getattr(prepared, "resampling_diagnostics", []) or [])
            ]
            if prepared is not None
            else []
        ),
        "error": None,
    }
    if error is not None:
        payload["error"] = {
            "type": type(error).__name__,
            "message": str(error),
            "repr": repr(error),
        }
    return payload


def _chunk_summaries(chunks) -> list[dict[str, Any]]:  # noqa: ANN001
    return [
        {
            "name": chunk.spec.display_name(index),
            "role": chunk.spec.role,
            "excitation": chunk.spec.excitation,
            "start": float(chunk.spec.start),
            "end": float(chunk.spec.end),
            "sample_count": int(chunk.sample_count),
        }
        for index, chunk in enumerate(chunks)
    ]


def _result_to_dict(
    spec: Any,
    prepared: Any,
    result: Any,
    *,
    validation_artifacts: dict[str, Any] | None = None,
    schema_issues: list[Any] | None = None,
    telemetry_quality: Any | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build this tool's result payload through the shared run-report contract.

    The payload is validated against the packaged run-report schema before it is
    returned, so ``--result-json`` can never advertise a schema version it does
    not satisfy. Tool-specific fields live under ``extra``, which the schema
    leaves open.

    Args:
        spec: Run specification that produced the result.
        prepared: Prepared run state.
        result: Completed solve result.
        validation_artifacts: Written validation artifact paths.
        schema_issues: Run-spec schema issues already computed for this run.
        telemetry_quality: Telemetry assessment already made for this run. When
            omitted it falls back to the one the prepared run carries.
        extra: Tool-specific payload merged into the report's ``extra`` object.

    Returns:
        A JSON-safe run report conforming to the packaged run-report schema.

    Raises:
        SysIdSchemaError: If the assembled report violates the run-report schema.
    """
    from isaacsim.robot_setup.sysid.run_report import build_sysid_run_report
    from isaacsim.robot_setup.sysid.schema_validation import (
        raise_for_schema_errors,
        validate_sysid_run_report_payload,
    )

    if telemetry_quality is None:
        telemetry_quality = getattr(prepared, "telemetry_quality", None)
    report = build_sysid_run_report(
        spec=spec,
        prepared=prepared,
        result=result,
        schema_issues=schema_issues,
        telemetry_quality=telemetry_quality,
        validation_artifacts=validation_artifacts or _empty_validation_artifacts(result),
        extra=extra,
    )
    payload = report.to_dict()
    raise_for_schema_errors(validate_sysid_run_report_payload(payload), context="Run report")
    return payload


def _artifact_output_dir(result_json: str) -> Path:
    if result_json:
        return Path(result_json).expanduser().resolve().parent
    return Path.cwd().resolve()


def _empty_validation_artifacts(result) -> dict[str, Any]:  # noqa: ANN001
    return {
        "source": getattr(result, "validation_artifact_source", "configured_chunks"),
        "joint_angle_delta_png": "",
        "joint_angle_delta_skipped_reason": "",
        "render_png": getattr(result, "validation_render_path", ""),
        "render_skipped_reason": getattr(result, "validation_render_skipped_reason", ""),
        "trajectory_animation_mp4": "",
        "trajectory_animation_gif": "",
        "trajectory_animation_png": "",
        "trajectory_animation_skipped_reason": "",
    }


def _write_validation_artifacts(result, output_dir: Path) -> dict[str, Any]:  # noqa: ANN001
    from isaacsim.robot_setup.sysid.run_report import write_validation_artifacts

    return write_validation_artifacts(result, output_dir)


class _SolveRateLogger:
    """Logs a wall-clock throughput proxy from optimizer progress fractions.

    Args:
        prepared: Constructor value for ``prepared``.
        interval_seconds: Constructor value for ``interval_seconds``.
    """

    def __init__(self, prepared: Any, *, interval_seconds: float = 2.0) -> None:
        self._interval = max(0.1, float(interval_seconds))
        self._start_time = time.perf_counter()
        self._last_log_time = self._start_time
        self._last_fraction = 0.0
        self._max_fraction = 0.0
        max_iterations = max(1, int(prepared.config.max_iterations))
        self._planned_samples = max_iterations * sum(
            min(int(chunk.sample_count), int(prepared.config.max_rollout_steps)) for chunk in prepared.train_chunks
        )
        self._planned_sim_seconds = max_iterations * sum(
            float(chunk.duration_seconds) for chunk in prepared.train_chunks
        )

    def maybe_log(self, fraction: float, *, force: bool = False) -> None:
        now = time.perf_counter()
        fraction = max(0.0, min(1.0, float(fraction)))
        if fraction < self._max_fraction:
            fraction = self._max_fraction
        elapsed = max(1e-9, now - self._start_time)
        if not force and now - self._last_log_time < self._interval:
            self._max_fraction = max(self._max_fraction, fraction)
            return

        delta_fraction = max(0.0, fraction - self._last_fraction)
        delta_time = max(1e-9, now - self._last_log_time)
        samples_per_sec = (delta_fraction * self._planned_samples) / delta_time
        avg_samples_per_sec = (fraction * self._planned_samples) / elapsed
        realtime_factor = (delta_fraction * self._planned_sim_seconds) / delta_time
        avg_realtime_factor = (fraction * self._planned_sim_seconds) / elapsed
        print(
            "[rate] "
            f"elapsed={elapsed:.1f}s progress={fraction:.3f} "
            f"samples/s={samples_per_sec:.1f} avg_samples/s={avg_samples_per_sec:.1f} "
            f"rtf={realtime_factor:.2f} avg_rtf={avg_realtime_factor:.2f}",
            flush=True,
        )
        self._last_log_time = now
        self._last_fraction = fraction
        self._max_fraction = fraction


async def _next_updates(count: int = 1) -> None:
    import omni.kit.app

    app = omni.kit.app.get_app()
    for _ in range(max(1, count)):
        await app.next_update_async()


async def _enable_extension() -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    try:
        print("[sysid] enabling isaacsim.robot_setup.sysid extension...", flush=True)
        add_path = getattr(manager, "add_path", None)
        if callable(add_path):
            add_path(str(_extension_root()))
        manager.set_extension_enabled_immediate("isaacsim.robot_setup.sysid", True)
        await _next_updates(2)
        print("[sysid] extension enable complete.", flush=True)
    except Exception as exc:
        print(f"[sysid] WARN: could not enable extension manager entry: {exc}", flush=True)


def _ensure_extension_package_visible() -> None:
    sys.path.insert(0, str(_extension_root()))
    import isaacsim

    package_root = str(_extension_python_package_root())
    package_path = getattr(isaacsim, "__path__", None)
    if package_path is not None and package_root not in package_path:
        package_path.append(package_root)
    importlib.invalidate_caches()
    print(f"[sysid] extension package path visible: {package_root}", flush=True)


async def _open_stage(stage_path: str):  # noqa: ANN202
    import omni.usd

    ctx = omni.usd.get_context()
    if stage_path:
        print(f"[sysid] opening stage: {stage_path}", flush=True)
        opener = getattr(ctx, "open_stage_async", None)
        if callable(opener):
            result = await opener(stage_path)
            if isinstance(result, tuple):
                ok = bool(result[0])
            else:
                ok = bool(result)
            if not ok:
                raise RuntimeError(f"Failed to open USD stage: {stage_path}")
        else:
            ok = ctx.open_stage(stage_path)
            if ok is False:
                raise RuntimeError(f"Failed to open USD stage: {stage_path}")
            await _next_updates(10)
        print("[sysid] stage opened.", flush=True)

    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open. Provide --stage or stage.input_path in the run spec.")
    return stage


async def _start_timeline(warmup_updates: int = 3) -> None:
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        print("[sysid] starting timeline...", flush=True)
        timeline.play()
        print("[sysid] timeline play requested.", flush=True)
        if warmup_updates > 0:
            print(f"[sysid] waiting {warmup_updates} timeline warmup update(s)...", flush=True)
            await _next_updates(warmup_updates)
        print("[sysid] timeline is playing.", flush=True)
    else:
        print("[sysid] timeline already playing.", flush=True)


async def _save_stage() -> bool:
    import omni.usd

    ctx = omni.usd.get_context()
    await _next_updates(1)
    return bool(ctx.save_stage())


async def _synthesize_quickstart_spec(args: argparse.Namespace, artifact_dir: Path):  # noqa: ANN202
    """Build and save a run spec from --robot/--telemetry/--recipe, returning (spec, stage, trajectory).

    Args:
        args: Parsed headless-solve command arguments.
        artifact_dir: Directory receiving the generated run specification.

    Returns:
        Generated run specification, opened stage, and loaded trajectory.
    """
    from isaacsim.robot_setup.sysid.articulation_utils import (
        collect_robot_link_and_joint_paths,
    )
    from isaacsim.robot_setup.sysid.preset_loader import load_builtin_parameter_preset
    from isaacsim.robot_setup.sysid.recipes import (
        infer_telemetry_source_type,
        synthesize_run_spec_from_quickstart,
    )
    from isaacsim.robot_setup.sysid.run_controller import SysIdRunController
    from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec

    if not (args.robot and args.telemetry):
        raise ValueError("Quick start requires both --robot and --telemetry (or use --run-spec).")
    if not args.stage:
        raise ValueError("Quick start requires --stage (the robot USD to open).")

    stage = await _open_stage(args.stage)
    recipe = load_builtin_parameter_preset(args.recipe)
    print(f"[recipe] using '{recipe.name}': {recipe.description}", flush=True)

    # Load the trajectory once through a minimal telemetry-only spec.
    probe_spec = SysIdRunSpec()
    probe_spec.telemetry.source_path = str(args.telemetry)
    probe_spec.telemetry.source_type = infer_telemetry_source_type(args.telemetry)
    probe_spec.telemetry.mapping_path = str(args.mapping)
    trajectory = SysIdRunController(probe_spec).load_trajectory()
    links, _joints = collect_robot_link_and_joint_paths(stage, args.robot)
    spec, notes = synthesize_run_spec_from_quickstart(
        robot_prim_path=args.robot,
        telemetry_path=args.telemetry,
        recipe=recipe,
        source_env_path=args.source_env,
        mapping_path=args.mapping,
        stage_path=args.stage,
        trajectory=trajectory,
        num_joints=int(trajectory.num_joints),
        num_links=max(1, len(links)),
    )
    for note in notes:
        print(f"[recipe] {note}", flush=True)

    save_path = Path(args.save_run_spec) if args.save_run_spec else artifact_dir / "sysid_run_spec_synthesized.json"
    _write_json(save_path, spec.to_dict())
    print(f"[recipe] synthesized run spec saved: {save_path}", flush=True)
    return spec, stage, trajectory


async def _run(args: argparse.Namespace) -> dict[str, Any]:
    artifact_dir = _artifact_output_dir(args.result_json)
    quickstart = bool(args.robot or args.telemetry)

    if not args.no_enable_extension:
        await _enable_extension()
    else:
        print("[sysid] skipping extension-manager enable.", flush=True)
    _ensure_extension_package_visible()

    print("[sysid] importing System Identification API...", flush=True)
    from isaacsim.robot_setup.sysid.headless_job import (
        HeadlessSysIdJobRequest,
        run_headless_sysid_job,
    )

    if quickstart:
        spec, stage, trajectory = await _synthesize_quickstart_spec(args, artifact_dir)
        raw_spec_payload = None
    else:
        run_spec_path = Path(args.run_spec)
        print(f"[sysid] loading RunSpec: {run_spec_path}", flush=True)
        spec, raw_spec_payload = _load_spec_and_payload(run_spec_path)
        stage = None
        trajectory = None
    if args.skip_timeline_start:
        print("[sysid] skipping timeline start; solve may fail if physics tensors are unavailable.", flush=True)

    prepared = None
    schema_issues = []
    telemetry_quality = None
    check_report = None
    rate_logger = None
    checkpoint_path = _checkpoint_output_path(args.result_json, args.checkpoint_json)
    last_iteration_status = None
    best_known_status = None
    progress_fraction = 0.0

    def on_prepared(prepared_run: Any, preflight: Any, quality: Any, issues: list[Any]) -> None:
        nonlocal check_report, prepared, rate_logger, schema_issues, telemetry_quality
        prepared = prepared_run
        schema_issues = issues
        telemetry_quality = quality
        rate_logger = _SolveRateLogger(prepared, interval_seconds=args.rate_log_interval)
        for issue in schema_issues:
            print(f"[schema:{issue.severity}] {issue.path}: {issue.message}", flush=True)
        raw_trajectory = prepared.raw_trajectory
        print(
            f"[sysid] trajectory loaded: samples={int(raw_trajectory.times.shape[0])}, "
            f"joints={raw_trajectory.num_joints}",
            flush=True,
        )
        print(f"[telemetry] {telemetry_quality.compact_summary()}", flush=True)
        for issue in telemetry_quality.issues:
            print(f"[telemetry:{issue.severity}] {issue.code}: {issue.message}", flush=True)
        for line in preflight.summary_lines():
            print(f"[preflight] {line}", flush=True)
        print(
            "[sysid] "
            f"backend={prepared.backend_label}, "
            f"parameters={len(prepared.param_entries)}, "
            f"train_chunks={len(prepared.train_chunks)}, "
            f"validation_chunks={len(prepared.validation_chunks)}",
            flush=True,
        )
        check_report = getattr(prepared, "check_report", None)
        if check_report is not None:
            for line in check_report.summary_lines():
                print(f"[check] {line}", flush=True)
        for diagnostic in getattr(prepared, "resampling_diagnostics", []) or []:
            print(
                "[sampling] "
                f"enabled={diagnostic.enabled} reason={diagnostic.reason} "
                f"raw={diagnostic.raw_sample_count} resampled={diagnostic.resampled_sample_count} "
                f"target_dt={diagnostic.target_dt} capped={diagnostic.capped}",
                flush=True,
            )

    def write_checkpoint(checkpoint_status: str, *, error: BaseException | None = None) -> None:
        if checkpoint_path is None:
            return
        payload = _checkpoint_payload(
            spec=spec,
            prepared=prepared,
            schema_issues=schema_issues,
            telemetry_quality=telemetry_quality,
            last_status=last_iteration_status,
            best_known_status=best_known_status,
            progress_fraction=progress_fraction,
            checkpoint_status=checkpoint_status,
            error=error,
        )
        try:
            _write_json(checkpoint_path, payload)
        except Exception as checkpoint_exc:
            print(f"[sysid] WARN: failed to write checkpoint {checkpoint_path}: {checkpoint_exc}", flush=True)

    def on_progress(message: str, fraction: float) -> None:
        nonlocal progress_fraction
        progress_fraction = max(progress_fraction, max(0.0, min(1.0, float(fraction))))
        print(f"[progress] {fraction:.3f} {message}", flush=True)
        if rate_logger is not None:
            rate_logger.maybe_log(fraction)

    def on_iteration(status: Any) -> None:
        nonlocal best_known_status, last_iteration_status, progress_fraction
        last_iteration_status = status
        if prepared is None:
            raise RuntimeError("SysID iteration received before run preparation completed.")
        progress_fraction = max(
            progress_fraction,
            min(1.0, float(status.iteration) / max(1, prepared.config.max_iterations)),
        )
        if status.accepted:
            best_known_status = status
        accepted = "accepted" if status.accepted else "rejected"
        print(
            f"[iter {status.iteration}] cost={status.cost:.6e} " f"lambda={status.damping:.3e} {accepted}",
            flush=True,
        )
        if rate_logger is not None:
            rate_logger.maybe_log(min(1.0, status.iteration / max(1, prepared.config.max_iterations)), force=True)
        if status.accepted:
            write_checkpoint("accepted_iteration")

    try:
        outcome = await run_headless_sysid_job(
            HeadlessSysIdJobRequest(
                spec=spec,
                stage_path=args.stage,
                stage=stage,
                trajectory=trajectory,
                raw_spec_payload=raw_spec_payload,
                enable_extension=False,
                start_timeline=not args.skip_timeline_start,
                timeline_warmup_updates=args.timeline_warmup_updates,
                save_stage=args.save_stage,
            ),
            on_iteration=on_iteration,
            on_progress=on_progress,
            on_prepared=on_prepared,
        )
    except Exception as exc:
        write_checkpoint("exception", error=exc)
        raise
    prepared = outcome.prepared
    result = outcome.result
    telemetry_quality = outcome.telemetry_quality
    schema_issues = outcome.schema_issues
    best_known_status = result.selected_status
    progress_fraction = 1.0
    write_checkpoint("completed")
    if rate_logger is not None:
        rate_logger.maybe_log(1.0, force=True)

    saved_stage = outcome.stage_saved
    if args.save_stage:
        print(f"[sysid] save_stage={saved_stage}", flush=True)

    validation_artifacts = _write_validation_artifacts(result, artifact_dir)
    tool_extra: dict[str, Any] = {
        "tool": "headless_sysid_solve",
        "stage_saved": saved_stage,
        "telemetry_source": spec.telemetry.source_path,
    }
    if check_report is not None:
        tool_extra["check_report"] = check_report.to_dict()
    summary = _result_to_dict(
        spec,
        prepared,
        result,
        validation_artifacts=validation_artifacts,
        schema_issues=schema_issues,
        telemetry_quality=telemetry_quality,
        extra=tool_extra,
    )
    confidence = getattr(result, "parameter_confidence", {}) or {}
    for entry in confidence.get("entries", []):
        rng = entry.get("ten_percent_range")
        rng_text = f" ±{rng:.4g}" if isinstance(rng, (int, float)) else ""
        marker = "; at noise floor" if entry.get("at_noise_floor") else ""
        print(
            f"[confidence] {entry.get('name', '?')}: {entry.get('value', float('nan')):.5g}{rng_text} "
            f"({entry.get('verdict', 'unknown')}{marker})",
            flush=True,
        )
    noise_floor = confidence.get("noise_floor") or {}
    if noise_floor.get("at_noise_floor"):
        ratio = noise_floor.get("cost_to_floor_ratio")
        ratio_text = f" (cost/floor ratio {ratio:.2g})" if isinstance(ratio, (int, float)) else ""
        print(
            f"[confidence] WARNING: solve cost is at the estimated measurement-noise floor{ratio_text} — "
            "verdicts unreliable; validate against ground truth or load-side torque.",
            flush=True,
        )
    if args.result_json:
        _write_json(Path(args.result_json), summary)
        print(f"[sysid] result_json={args.result_json}", flush=True)
    return summary


def main() -> int:  # noqa: D103
    args = _parse_args()
    quickstart = bool(args.robot or args.telemetry)
    if quickstart == bool(args.run_spec):
        print(
            "[sysid] ERROR: provide either --run-spec, or the quick start pair --robot + --telemetry.",
            file=sys.stderr,
            flush=True,
        )
        return 2
    sys.path.insert(0, str(_extension_root()))

    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        code = _rerun_with_bundled_python()
        if code is not None:
            return code
        searched = "\n  ".join(str(candidate) for candidate in _bundled_python_launchers())
        print(
            "[sysid] ERROR: could not import isaacsim.SimulationApp and no bundled Isaac Sim Python "
            f"launcher was found. Run this script with the Isaac Sim launcher. Searched:\n  {searched}",
            file=sys.stderr,
            flush=True,
        )
        print(f"[sysid] import detail: {exc}", file=sys.stderr, flush=True)
        return 1

    simulation_app = SimulationApp(
        {
            "headless": True,
            "create_new_stage": False,
            "disable_viewport_updates": True,
        }
    )
    exit_code = 1
    try:
        loop = asyncio.get_event_loop()
        task = loop.create_task(_run(args))
        print("[sysid] Kit update pump started.", flush=True)
        while not task.done():
            simulation_app.update()
        print("[sysid] Kit update pump stopped.", flush=True)
        summary = task.result()
        print(json.dumps(summary, indent=2), flush=True)
        exit_code = 0
    except Exception as exc:
        print(f"[sysid] ERROR: {exc}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        # Fast shutdown terminates the process inside `close()`, so the status must be passed here.
        simulation_app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
