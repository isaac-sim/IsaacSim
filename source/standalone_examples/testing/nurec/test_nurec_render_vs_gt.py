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

"""Render-vs-GT regression test for the SPG-runtime PPISP rendering path.

For each case in the config (particle + volume), calls `render_and_score` — which
derives the GT timestamps from `gt_root`, renders the NuRec USD at those rig keyframes, and
scores the frames against GT — then gates the aggregate PSNR/SSIM against the per-case
thresholds.

The NuRec USDs are referenced as Isaac-relative paths (``/Isaac/Samples/...``) in the
config; ``resolve_path`` expands them via ``get_assets_root_path()`` at runtime so they
resolve against the internal Nucleus or the production S3 mirror automatically.

Usage:
    ./python.sh source/standalone_examples/testing/nurec/test_nurec_render_vs_gt.py \
        [--config config/nurec_eval_test.yaml] [--case particle] [--save-images]
"""

from __future__ import annotations

import argparse
import os
import sys

_DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "nurec_eval_test.yaml")

parser = argparse.ArgumentParser(description="Render SPG NuRec USDs at GT timestamps and score vs GT.")
parser.add_argument("--config", default=_DEFAULT_CONFIG, help="regression config (cases + thresholds)")
parser.add_argument("--case", default=None, help="run only this case (default: all cases in the config)")
parser.add_argument(
    "--stage",
    default=None,
    help="render this NuRec USD directly (local path or omniverse:// URL); needs --gt-root, bypasses the config cases",
)
parser.add_argument("--gt-root", default=None, help="GT tree for --stage, laid out as <gt_root>/<camera>/<ts_ns>.png")
parser.add_argument(
    "--cameras", nargs="+", default=None, help="cameras for --stage (default: every camera under --gt-root)"
)
parser.add_argument("--save-images", action="store_true", help="save GT|rendered|diff panels")
parser.add_argument(
    "--save-plots", action="store_true", help="save per-metric plots (PSNR/SSIM over index, histogram, pose heatmap)"
)
parser.add_argument(
    "--out-dir",
    default=None,
    help="base output dir for renders/panels/plots (default: per-case output_parent_dir from the config)",
)
parser.add_argument("--min-psnr", type=float, default=None, help="override per-case PSNR gate")
parser.add_argument("--min-ssim", type=float, default=None, help="override per-case SSIM gate")
parser.add_argument("--num-samples", type=int, default=None, help="override per-case GT-frame subsample count")
args, _ = parser.parse_known_args()
if args.stage and not args.gt_root:
    parser.error("--stage requires --gt-root")

# Config paths in the regression file are relative to its own directory.
cfg_dir = os.path.dirname(os.path.abspath(os.path.expanduser(args.config)))

# Enable the rendering utilities (which pulls omni.rtx.spg) at launch; render single-GPU.
extra_args = ["--enable", "isaacsim.replicator.nurec_utils", "--/renderer/multiGpu/enabled=false"]
print(f"[test_nurec_render_vs_gt] config={args.config} extra_args={extra_args}", flush=True)

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True, "multi_gpu": False, "extra_args": extra_args})

import carb
from isaacsim.replicator.nurec_utils.eval import render_and_score
from isaacsim.replicator.nurec_utils.metrics.scoped_timer import ScopedTimer
from isaacsim.replicator.nurec_utils.rendering_setup import enable_omni_rtx_spg, load_config, resolve_path, select_cases

simulation_app.update()
enable_omni_rtx_spg(simulation_app)

if args.stage:
    # Ad-hoc single NuRec USD + GT (no config), for manual runs. Absolute paths so the
    # config-relative resolution in run_case is a no-op.
    cases = [
        {
            "name": "adhoc",
            "stage": args.stage,
            "gt_root": os.path.abspath(os.path.expanduser(args.gt_root)),
            "cameras": args.cameras,
            "output_parent_dir": os.path.abspath(os.path.expanduser(args.out_dir or "_out_adhoc")),
            "keyframe_tolerance_us": 1000.0,  # match a GT timestamp to the nearest rig keyframe within 1 ms
        }
    ]
else:
    # Merge the regression config onto the shipped defaults so it inherits the carb-override sets.
    cfg = load_config(args.config)
    cases = select_cases(cfg, args.case)


def run_case(case: dict) -> int | None:
    """Render + score one case, returning the gate result.

    Args:
        case: A flattened case config (stage, gt_root, cameras, thresholds, ...).

    Returns:
        0 (passed the gate), 1 (failed the gate), or 2 (nothing rendered/scored, or a
        launch prerequisite was unmet for the SPG stage).
    """
    name = case.get("name") or "default"
    stage_path = resolve_path(case["stage"], cfg_dir)
    gt_root = resolve_path(case["gt_root"], cfg_dir)
    cameras = set(case["cameras"]) if case.get("cameras") else None
    num_samples = args.num_samples if args.num_samples is not None else case.get("num_samples")
    warmup_steps = int(case.get("rendering", {}).get("warmup_steps", 800))
    keyframe_tol_us = float(case.get("keyframe_tolerance_us", 1.0))
    thresholds = case.get("thresholds") or {}
    min_psnr = args.min_psnr if args.min_psnr is not None else thresholds.get("min_psnr")
    min_ssim = args.min_ssim if args.min_ssim is not None else thresholds.get("min_ssim")
    base_dir = (
        os.path.abspath(os.path.expanduser(args.out_dir))
        if args.out_dir
        else resolve_path(case["output_parent_dir"], cfg_dir)
    )
    out_dir = os.path.join(base_dir, f"{name}_spg")

    carb.log_warn(f"==================== case: {name} ====================")
    summary = render_and_score(
        simulation_app,
        stage_path,
        gt_root,
        out_dir,
        cameras=cameras,
        num_samples=num_samples,
        keyframe_tol_us=keyframe_tol_us,
        warmup_steps=warmup_steps,
        config_path=args.config,
        write_panels=args.save_images,
        write_plots=args.save_plots,
        timing_label=name,
    )
    if summary is None:
        carb.log_error(f"[{name}] nothing rendered/scored.")
        return 2

    overall = summary["overall"]
    passed = (min_psnr is None or overall["psnr_mean"] >= min_psnr) and (
        min_ssim is None or overall["ssim_mean"] >= min_ssim
    )
    carb.log_warn(
        f"[{name}] {overall['n']} frames PSNR={overall['psnr_mean']:.2f} dB SSIM={overall['ssim_mean']:.4f} | "
        f"gate min_psnr={min_psnr} min_ssim={min_ssim} -> {'PASS' if passed else 'FAIL'} | out={out_dir}"
    )
    return 0 if passed else 1


results = {(c.get("name") or "default"): run_case(c) for c in cases}
carb.log_warn(f"=== results (0=pass 1=fail 2=no-score): {results} ===")
exit_code = 0 if all(v == 0 for v in results.values()) else 1

ScopedTimer.print_table()
simulation_app.close()
sys.exit(exit_code)
