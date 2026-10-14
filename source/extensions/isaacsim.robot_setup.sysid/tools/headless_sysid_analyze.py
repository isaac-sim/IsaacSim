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

"""Run baseline and parameter-sensitivity diagnostics from a headless Isaac Sim session."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from headless_sysid_solve import (
    _artifact_output_dir,
    _enable_extension,
    _ensure_extension_package_visible,
    _load_spec,
    _next_updates,
    _open_stage,
    _rerun_with_bundled_python,
    _start_timeline,
    _write_json,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-spec", required=True, help="Path to a SysIdRunSpec JSON file.")
    parser.add_argument("--stage", default="", help="USD stage to open before diagnostics.")
    parser.add_argument("--result-json", default="", help="Path for the analysis JSON.")
    parser.add_argument("--mode", choices=("baseline", "sensitivity", "all"), default="all")
    parser.add_argument("--relative-epsilon", type=float, default=0.05)
    parser.add_argument("--absolute-epsilon", type=float, default=1e-4)
    parser.add_argument("--rate-log-interval", type=float, default=30.0)
    parser.add_argument("--no-enable-extension", action="store_true")
    parser.add_argument("--timeline-warmup-updates", type=int, default=0)
    parser.add_argument("--skip-timeline-start", action="store_true")
    return parser.parse_args()


async def _run(args: argparse.Namespace) -> dict[str, Any]:
    run_spec_path = Path(args.run_spec)
    artifact_dir = _artifact_output_dir(args.result_json or "sysid_analysis_result.json")
    if not args.no_enable_extension:
        await _enable_extension()
    _ensure_extension_package_visible()

    from isaacsim.robot_setup.sysid.analysis import (
        ANALYSIS_SCHEMA_VERSION,
        build_analysis_recommendations,
        build_perturbations,
        build_sensitivity_entry,
        evaluate_chunk_set,
        parameter_rows,
        perturb_theta,
    )
    from isaacsim.robot_setup.sysid.run_controller import SysIdRunController
    from isaacsim.robot_setup.sysid.run_report import (
        validation_summary,
        write_validation_artifacts,
    )
    from isaacsim.robot_setup.sysid.schema_validation import (
        raise_for_schema_errors,
        schema_issues_to_dicts,
        validate_sysid_run_spec_payload,
    )
    from isaacsim.robot_setup.sysid.telemetry_quality import (
        build_telemetry_quality_report,
    )

    print(f"[sysid-analysis] loading RunSpec: {run_spec_path}", flush=True)
    spec = _load_spec(run_spec_path)
    schema_issues = validate_sysid_run_spec_payload(spec.to_dict())
    for issue in schema_issues:
        print(f"[schema:{issue.severity}] {issue.path}: {issue.message}", flush=True)
    raise_for_schema_errors(schema_issues, context="Run specification")

    stage = await _open_stage(args.stage or spec.stage.input_path)
    controller = SysIdRunController(spec)
    print(f"[sysid-analysis] loading trajectory: {spec.telemetry.source_path}", flush=True)
    trajectory = controller.load_trajectory()
    if args.skip_timeline_start:
        print("[sysid-analysis] skipping timeline start; diagnostics may fail if physics tensors are unavailable.")
    else:
        await _start_timeline(args.timeline_warmup_updates)

    preflight = controller.preflight(
        stage=stage,
        trajectory=trajectory,
        timeline_playing=(None if args.skip_timeline_start else True),
        allow_empty_parameters=True,
    )
    for line in preflight.summary_lines():
        print(f"[preflight] {line}", flush=True)
    try:
        preflight.raise_for_errors()
        prepared = controller.prepare(
            stage,
            trajectory=trajectory,
            timeline_playing=(None if args.skip_timeline_start else True),
            allow_empty_parameters=True,
        )
    except Exception:
        # Preparation failures are often caused by the telemetry, so still surface its summary.
        with contextlib.suppress(Exception):
            summary = build_telemetry_quality_report(trajectory, spec.telemetry.chunks).compact_summary()
            print(f"[telemetry] {summary}", flush=True)
        raise
    telemetry_quality = prepared.telemetry_quality
    print(f"[telemetry] {telemetry_quality.compact_summary()}", flush=True)
    optimizer = prepared.optimizer
    try:
        print(
            "[sysid-analysis] "
            f"backend={prepared.backend_label}, parameters={len(prepared.param_entries)}, "
            f"train_chunks={len(prepared.train_chunks)}, validation_chunks={len(prepared.validation_chunks)}",
            flush=True,
        )
        for diagnostic in prepared.resampling_diagnostics or []:
            print(
                "[sampling] "
                f"enabled={diagnostic.enabled} reason={diagnostic.reason} "
                f"raw={diagnostic.raw_sample_count} resampled={diagnostic.resampled_sample_count} "
                f"target_dt={diagnostic.target_dt} capped={diagnostic.capped}",
                flush=True,
            )
        theta_initial = [float(value) for value in prepared.config.theta_initial.detach().cpu().reshape(-1).tolist()]
        progress = _ProgressPrinter(interval_seconds=float(args.rate_log_interval))
    except BaseException:
        await _cleanup_optimizer(optimizer, spec.outputs.remove_clones_after_run)
        raise

    def on_progress(message: str, fraction: float) -> None:
        progress.maybe_print(message, fraction)

    try:
        baseline_train = await evaluate_chunk_set(
            optimizer,
            prepared.config,
            prepared.train_chunks,
            theta_initial,
            label="Baseline train",
            on_progress=on_progress,
        )
        baseline_validation = await evaluate_chunk_set(
            optimizer,
            prepared.config,
            prepared.validation_chunks,
            theta_initial,
            label="Baseline validation",
            on_progress=on_progress,
        )
        baseline = {
            "theta": theta_initial,
            "train": baseline_train.to_dict(),
            "validation": baseline_validation.to_dict(),
        }
        artifacts = write_validation_artifacts(
            SimpleNamespace(
                validation_rollouts=baseline_validation.rollouts,
                validation_metrics=baseline_validation.metrics,
                validation_artifact_source="configured_chunks",
                validation_render_path="",
                validation_render_skipped_reason="",
                validation_animation_enabled=spec.outputs.render_validation_animation,
                validation_animation_path=spec.outputs.validation_animation_path,
                validation_animation_fps=spec.outputs.validation_animation_fps,
                validation_animation_max_frames=spec.outputs.validation_animation_max_frames,
            ),
            artifact_dir,
        )
        sensitivity: list[dict[str, Any]] = []
        if args.mode in {"sensitivity", "all"} and prepared.config.param_entries:
            perturbations = build_perturbations(
                prepared.config,
                relative_epsilon=float(args.relative_epsilon),
                absolute_epsilon=float(args.absolute_epsilon),
            )
            total = max(1, len(perturbations))
            for index, perturbation in enumerate(perturbations):
                print(
                    f"[sysid-analysis] sensitivity {index + 1}/{total}: {perturbation.name}",
                    flush=True,
                )
                minus_train = minus_validation = plus_train = plus_validation = None
                if perturbation.minus is not None:
                    theta_minus = perturb_theta(theta_initial, perturbation, perturbation.minus)
                    minus_train = (
                        await evaluate_chunk_set(
                            optimizer,
                            prepared.config,
                            prepared.train_chunks,
                            theta_minus,
                            label=f"{perturbation.name} minus train",
                            on_progress=on_progress,
                        )
                    ).metrics
                    minus_validation = (
                        await evaluate_chunk_set(
                            optimizer,
                            prepared.config,
                            prepared.validation_chunks,
                            theta_minus,
                            label=f"{perturbation.name} minus validation",
                            on_progress=on_progress,
                        )
                    ).metrics
                if perturbation.plus is not None:
                    theta_plus = perturb_theta(theta_initial, perturbation, perturbation.plus)
                    plus_train = (
                        await evaluate_chunk_set(
                            optimizer,
                            prepared.config,
                            prepared.train_chunks,
                            theta_plus,
                            label=f"{perturbation.name} plus train",
                            on_progress=on_progress,
                        )
                    ).metrics
                    plus_validation = (
                        await evaluate_chunk_set(
                            optimizer,
                            prepared.config,
                            prepared.validation_chunks,
                            theta_plus,
                            label=f"{perturbation.name} plus validation",
                            on_progress=on_progress,
                        )
                    ).metrics
                sensitivity.append(
                    build_sensitivity_entry(
                        perturbation,
                        baseline_train=baseline_train.metrics,
                        baseline_validation=baseline_validation.metrics,
                        minus_train=minus_train,
                        minus_validation=minus_validation,
                        plus_train=plus_train,
                        plus_validation=plus_validation,
                    )
                )

        result = {
            "ok": True,
            "analysis_schema_version": ANALYSIS_SCHEMA_VERSION,
            "mode": args.mode,
            "run_spec": spec.to_dict(),
            "schema_issues": schema_issues_to_dicts(schema_issues),
            "telemetry_quality": telemetry_quality.to_dict(),
            "backend": prepared.backend_label,
            "robot_prim_path": spec.simulation.robot_prim_path,
            "parameters": parameter_rows(prepared.config),
            "baseline": baseline if args.mode in {"baseline", "all", "sensitivity"} else {},
            "sensitivity": sensitivity,
            "recommendations": build_analysis_recommendations(baseline=baseline, sensitivity=sensitivity),
            "artifacts": artifacts,
            "resampling_diagnostics": [
                diagnostic.to_dict() if hasattr(diagnostic, "to_dict") else dict(diagnostic)
                for diagnostic in (prepared.resampling_diagnostics or [])
            ],
            "summaries": {
                "train": validation_summary(baseline_train.metrics),
                "validation": validation_summary(baseline_validation.metrics),
            },
        }
        output_path = (
            Path(args.result_json).expanduser().resolve()
            if args.result_json
            else (artifact_dir / "sysid_analysis_result.json").resolve()
        )
        _write_json(output_path, result)
        print(f"[sysid-analysis] result_json={output_path}", flush=True)
        return result
    finally:
        await _cleanup_optimizer(optimizer, spec.outputs.remove_clones_after_run)


async def _cleanup_optimizer(optimizer: Any, remove_clones: bool) -> None:
    """Release one analysis optimizer's bridge through either cleanup contract.

    Args:
        optimizer: Optimizer owning the runtime bridge.
        remove_clones: Whether cleanup removes cloned environments.
    """
    cleanup_async = getattr(optimizer, "cleanup_bridge_async", None)
    if callable(cleanup_async):
        await cleanup_async(remove_clones=remove_clones)
        return
    cleanup = getattr(optimizer, "cleanup_bridge", None)
    if callable(cleanup):
        cleanup(remove_clones=remove_clones)


class _ProgressPrinter:
    def __init__(self, *, interval_seconds: float) -> None:
        import time

        self._time = time
        self._interval = max(0.1, float(interval_seconds))
        self._last = 0.0

    def maybe_print(self, message: str, fraction: float) -> None:
        now = self._time.perf_counter()
        if now - self._last < self._interval and float(fraction) < 1.0:
            return
        self._last = now
        print(f"[progress] {float(fraction):.3f} {message}", flush=True)


def main() -> int:  # noqa: D103
    args = _parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        code = _rerun_with_bundled_python()
        if code is not None:
            return code
        print(f"[sysid-analysis] ERROR: could not import isaacsim.SimulationApp: {exc}", file=sys.stderr, flush=True)
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
        print("[sysid-analysis] Kit update pump started.", flush=True)
        while not task.done():
            simulation_app.update()
        print("[sysid-analysis] Kit update pump stopped.", flush=True)
        print(json.dumps(task.result(), indent=2), flush=True)
        exit_code = 0
    except Exception as exc:
        print(f"[sysid-analysis] ERROR: {exc}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        try:
            import omni.timeline

            omni.timeline.get_timeline_interface().stop()
            loop = asyncio.get_event_loop()
            if loop.is_running():
                pass
            else:
                loop.run_until_complete(_next_updates(2))
        except Exception:
            pass
        # Fast shutdown terminates the process inside `close()`, so the status must be passed here.
        simulation_app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
