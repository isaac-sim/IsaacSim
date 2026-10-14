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

"""Apply an already evaluated SysID theta to a copied USD stage.

This diagnostic helper is intentionally not a solver: it loads a SysIdRunSpec,
extracts a selected theta from a result JSON, writes those parameter values onto
a copy of the input USD, and saves the copy for visual/interactive inspection.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import shutil
import sys
from pathlib import Path
from typing import Any

from headless_sysid_solve import (
    _enable_extension,
    _ensure_extension_package_visible,
    _extension_root,
    _load_spec,
    _next_updates,
    _open_stage,
    _rerun_with_bundled_python,
    _save_stage,
    _start_timeline,
    _write_json,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-spec", required=True, help="SysIdRunSpec JSON used to define selected parameters.")
    parser.add_argument("--result-json", required=True, help="Result/report JSON containing selected_status.theta.")
    parser.add_argument("--stage", default="", help="Input USD. Overrides stage.input_path from the run spec.")
    parser.add_argument("--output-stage", required=True, help="New USD path to write. The source USD is not modified.")
    parser.add_argument("--summary-json", default="", help="Optional JSON summary/provenance sidecar path.")
    parser.add_argument(
        "--status-key",
        default="selected_status",
        choices=("selected_status", "final_status", "last_accepted_status"),
        help="Which status payload from result JSON supplies theta.",
    )
    parser.add_argument(
        "--no-enable-extension",
        action="store_true",
        help="Do not ask Kit's extension manager to enable isaacsim.robot_setup.sysid.",
    )
    parser.add_argument("--timeline-warmup-updates", type=int, default=3)
    parser.add_argument("--overwrite", action="store_true", help="Allow replacing an existing output USD.")
    return parser.parse_args()


def _load_result_status(result_json: Path, status_key: str) -> tuple[dict[str, Any], dict[str, Any]]:
    with result_json.expanduser().open("r", encoding="utf-8") as stream:
        result_payload = json.load(stream)
    status = result_payload.get(status_key)
    if not isinstance(status, dict):
        raise ValueError(f"Result JSON does not contain object key '{status_key}'.")
    theta = status.get("theta")
    if not isinstance(theta, list) or not theta:
        raise ValueError(f"Result JSON key '{status_key}.theta' is missing or empty.")
    return result_payload, status


def _select_physics_variant_for_session(stage: Any, robot_prim_path: str, solver_layer_name: str) -> bool:
    """Activate a configured Physics variant without authoring it to the asset.

    Args:
        stage: USD stage whose anonymous session layer receives the selection.
        robot_prim_path: Robot prim that owns the ``Physics`` variant set.
        solver_layer_name: Solver variant to activate.

    Returns:
        Whether the requested solver variant is active.
    """
    if stage is None or not robot_prim_path or not solver_layer_name:
        return False

    from pxr import Usd

    robot = stage.GetPrimAtPath(robot_prim_path)
    if not robot.IsValid():
        return False
    physics_variants = robot.GetVariantSets().GetVariantSet("Physics")
    if solver_layer_name not in physics_variants.GetVariantNames():
        return False
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        physics_variants.SetVariantSelection(solver_layer_name)
    return physics_variants.GetVariantSelection() == solver_layer_name


def _copy_stage(
    source: Path,
    destination: Path,
    *,
    overwrite: bool,
    robot_prim_path: str = "",
    solver_layer_name: str = "",
) -> tuple[Path | None, Path | None]:
    """Copy a flat stage, or create safe overrides for a multi-physics stage.

    Args:
        source: Input USD stage path.
        destination: Output USD stage path.
        overwrite: Whether an existing output may be replaced.
        robot_prim_path: Robot prim used to detect multi-physics composition.
        solver_layer_name: Optional active solver layer name.

    Returns:
        Paths to output-owned physics layers, or ``None`` entries for a flat copy.
    """
    if source.expanduser().resolve() == destination.expanduser().resolve():
        raise ValueError("Input and output USD paths must be different.")
    if not source.is_file():
        raise FileNotFoundError(f"Input USD does not exist: {source}")

    from isaacsim.robot_setup.sysid.usd_write_layer import create_physics_override_stage, is_multiphysics_stage
    from pxr import Usd

    inspection_stage = Usd.Stage.Open(str(source)) if robot_prim_path else None
    _select_physics_variant_for_session(inspection_stage, robot_prim_path, solver_layer_name)
    if inspection_stage is not None and is_multiphysics_stage(inspection_stage, robot_prim_path):
        return create_physics_override_stage(
            source,
            destination,
            overwrite=overwrite,
            solver_layer_name=solver_layer_name,
        )

    if destination.exists() and not overwrite:
        raise FileExistsError(f"Output USD already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return None, None


async def _run(args: argparse.Namespace) -> dict[str, Any]:
    if not args.no_enable_extension:
        await _enable_extension()
    else:
        print("[sysid] skipping extension-manager enable.", flush=True)
    _ensure_extension_package_visible()

    from isaacsim.robot_setup.sysid.actuator_compatibility import resolve_simulation_compatibility
    from isaacsim.robot_setup.sysid.optimizer_base import OptimizationIterationStatus
    from isaacsim.robot_setup.sysid.provenance import (
        ProvenanceWriter,
        build_provenance_record,
    )
    from isaacsim.robot_setup.sysid.run_controller import (
        SysIdRunController,
        write_optimized_parameters_to_usd,
    )

    spec = _load_spec(Path(args.run_spec))
    source_stage = Path(args.stage or spec.stage.input_path).expanduser().resolve()
    output_stage = Path(args.output_stage).expanduser().resolve()
    result_payload, status_payload = _load_result_status(Path(args.result_json), args.status_key)
    theta = [float(value) for value in status_payload["theta"]]

    compatibility = resolve_simulation_compatibility(spec.simulation)
    solver_layer_name = ""
    if compatibility.physics_backend == "physx":
        solver_layer_name = "physx"
    elif compatibility.physics_backend == "newton" and compatibility.physics_solver == "mujoco":
        solver_layer_name = "mujoco"

    physics_layer_path, solver_layer_path = _copy_stage(
        source_stage,
        output_stage,
        overwrite=bool(args.overwrite),
        robot_prim_path=spec.simulation.robot_prim_path,
        solver_layer_name=solver_layer_name,
    )
    stage = await _open_stage(str(output_stage))
    if physics_layer_path is not None:
        _select_physics_variant_for_session(stage, spec.simulation.robot_prim_path, solver_layer_name)
    from pxr import Sdf

    physics_write_layer = None
    solver_write_layer = None
    if physics_layer_path is not None:
        root_layer = stage.GetRootLayer()
        active_write_layer = Sdf.Layer.FindOrOpenRelativeToLayer(root_layer, root_layer.subLayerPaths[0])
        if solver_layer_path is None:
            physics_write_layer = active_write_layer
        elif active_write_layer is not None:
            solver_write_layer = active_write_layer
            physics_write_layer = Sdf.Layer.FindOrOpenRelativeToLayer(
                solver_write_layer,
                solver_write_layer.subLayerPaths[0],
            )
        if physics_write_layer is None:
            raise RuntimeError("Could not open the output physics write layer.")
    await _start_timeline(max(0, int(args.timeline_warmup_updates)))

    controller = SysIdRunController(spec)
    print(f"[sysid] loading trajectory: {spec.telemetry.source_path}", flush=True)
    trajectory = controller.load_trajectory()
    prepared = controller.prepare(stage, trajectory=trajectory, timeline_playing=True)
    try:
        if len(theta) != len(prepared.config.param_entries):
            raise ValueError(
                f"Theta length ({len(theta)}) does not match selected parameter count "
                f"({len(prepared.config.param_entries)})."
            )

        bridge = prepared.optimizer.get_bridge() if hasattr(prepared.optimizer, "get_bridge") else prepared.bridge
        write_kwargs = {}
        if physics_write_layer is not None:
            active_solver_write_layer = solver_write_layer if solver_write_layer is not None else physics_write_layer
            write_kwargs = {
                "neutral_write_layer": physics_write_layer,
                "solver_write_layer": active_solver_write_layer,
            }
        parameters_written = write_optimized_parameters_to_usd(
            stage,
            prepared.parameter_space,
            prepared.config,
            theta,
            max(1, len(prepared.link_paths)),
            bridge,
            physics_backend=compatibility.physics_backend,
            **write_kwargs,
        )
        if not parameters_written:
            raise RuntimeError("No selected parameter was written to the copied USD stage.")

        final_status = OptimizationIterationStatus(
            iteration=int(status_payload.get("iteration", 0)),
            cost=float(status_payload.get("cost", 0.0)),
            damping=float(status_payload.get("damping", 0.0)),
            accepted=bool(status_payload.get("accepted", True)),
            theta=theta,
        )
        validation_metrics = result_payload.get("validation_metrics", [])
        if not isinstance(validation_metrics, list):
            validation_metrics = []
        record = build_provenance_record(
            config=prepared.config,
            final_status=final_status,
            optimizer_backend=str(result_payload.get("backend", prepared.backend_label)),
            robot_prim_path=spec.simulation.robot_prim_path,
            validation_metrics=validation_metrics,
            simulation_engine=str(getattr(prepared, "simulation_engine", "")),
            newton_config=getattr(prepared, "newton_config", None),
        )
        writer = ProvenanceWriter()
        writer.write_to_stage(stage, spec.simulation.robot_prim_path, record)

        if physics_write_layer is not None and not physics_write_layer.Save():
            raise RuntimeError(f"Failed to save the output physics layer: {physics_write_layer.realPath}")
        if solver_write_layer is not None and not solver_write_layer.Save():
            raise RuntimeError(f"Failed to save the output solver layer: {solver_write_layer.realPath}")
        await _next_updates(2)
        saved = await _save_stage()
        if not saved:
            raise RuntimeError(f"Failed to save the copied USD stage: {output_stage}")

        summary = {
            "ok": True,
            "source_stage": str(source_stage),
            "output_stage": str(output_stage),
            "run_spec": str(Path(args.run_spec).expanduser().resolve()),
            "result_json": str(Path(args.result_json).expanduser().resolve()),
            "status_key": args.status_key,
            "robot_prim_path": spec.simulation.robot_prim_path,
            "parameters_written": True,
            "provenance_written_to_usd": True,
            "stage_saved": True,
            "theta": theta,
            "source_tuning_assessment": result_payload.get("tuning_assessment", {}),
            "inspection_note": (
                "This USD contains the physical theta values only. Command timing delay diagnostics are not "
                "physical USD parameters and must be reproduced by the playback/validation setup."
            ),
        }
        if args.summary_json:
            sidecar = Path(args.summary_json).expanduser().resolve()
            summary["summary_json"] = str(sidecar)
            summary["provenance"] = record.to_dict()
            _write_json(sidecar, summary)
        return summary
    finally:
        cleanup_async = getattr(prepared.optimizer, "cleanup_bridge_async", None)
        if callable(cleanup_async):
            await cleanup_async(remove_clones=True)
        else:
            cleanup = getattr(prepared.optimizer, "cleanup_bridge", None)
            if callable(cleanup):
                cleanup(remove_clones=True)


def main() -> int:  # noqa: D103
    args = _parse_args()
    sys.path.insert(0, str(_extension_root()))
    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        code = _rerun_with_bundled_python()
        if code is not None:
            return code
        print(
            "[sysid] ERROR: could not import isaacsim.SimulationApp and no bundled Isaac Sim Python "
            "launcher was found.",
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
        exit_code = 0 if summary.get("ok") else 1
    except Exception as exc:
        print(f"[sysid] ERROR: {exc}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        # Fast shutdown terminates the process inside `close()`, so the status must be passed here.
        simulation_app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
