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

"""Copy-vs-bind regression test for the SPG RenderProduct clone.

For each case, render each camera's authored RenderProduct (bind) and a session-layer clone (copy) at
N random GT keyframes, then assert:
  - sanity:   the clone scores against ground truth above the per-case PSNR/SSIM gates (it renders the
              scene correctly through its copied PPISP graph);
  - faithful: the clone matches the bind within RTPT noise (mean absolute pixel diff), since both render
              the same camera through the same graph.
The clone is a distinct RenderProduct with its own LdrColor AOV (proven structurally by the render.py
unit test), so it cannot alias the authored RP's output. GT layout: <gt_root>/<camera>/<ts_ns>.png.

    ./python.sh source/standalone_examples/testing/nurec/test_copy_rp_render.py [--config ...] [--case particle]
"""

from __future__ import annotations

import argparse
import os
import random
import sys
from enum import Enum


class CaseResult(Enum):
    """Outcome of a copy-vs-bind case."""

    PASS = 0
    FAIL = 1
    SKIP = 2


_DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "nurec_copy_test.yaml")

parser = argparse.ArgumentParser(description="Render an authored SPG RenderProduct and a clone of it, and compare.")
parser.add_argument("--config", default=_DEFAULT_CONFIG, help="copy-test config (cases + thresholds)")
parser.add_argument("--case", default=None, help="run only this case (default: all cases in the config)")
parser.add_argument("--num-samples", type=int, default=None, help="override GT keyframes sampled per camera")
parser.add_argument("--min-psnr", type=float, default=None, help="override per-case clone-vs-GT PSNR gate")
parser.add_argument("--min-ssim", type=float, default=None, help="override per-case clone-vs-GT SSIM gate")
parser.add_argument("--save-dir", default=None, help="if set, save gt/bind/clone/diff PNGs per scored frame here")
args, _ = parser.parse_known_args()

# Enable the rendering utilities (which pulls omni.rtx.spg) at launch; render single-GPU.
extra_args = ["--enable", "isaacsim.replicator.nurec_utils", "--/renderer/multiGpu/enabled=false"]
print(f"[test_copy_rp_render] config={args.config} extra_args={extra_args}", flush=True)

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True, "multi_gpu": False, "extra_args": extra_args})

import carb
import numpy as np
from isaacsim.core.experimental.utils.stage import open_stage
from isaacsim.replicator.nurec_utils.eval import gt_path, read_gt_timestamps, resolve_gt_root
from isaacsim.replicator.nurec_utils.metrics.psnr_ssim import image_diff, load_rgb, match_shape, mean_abs_diff, score
from isaacsim.replicator.nurec_utils.render import CameraRenderer, RenderTargetFactory, discover_render_products
from isaacsim.replicator.nurec_utils.rendering_setup import (
    enable_omni_rtx_spg,
    load_config,
    resolve_path,
    select_cases,
    setup_for_rendering,
)

simulation_app.update()
enable_omni_rtx_spg(simulation_app)

cfg = load_config(args.config)
cases = select_cases(cfg, args.case)

SEED = int(cfg.get("seed", 0))
WARMUP_STEPS = int(cfg.get("warmup_steps", 800))
TOL_US = float(cfg.get("keyframe_tolerance_us", 1000.0))
SAVE_DIR = os.path.expanduser(args.save_dir) if args.save_dir else None
# Config paths in the regression file are relative to its own directory.
cfg_dir = os.path.dirname(os.path.abspath(os.path.expanduser(args.config)))


def _to_rgb(image: np.ndarray) -> np.ndarray:
    """Drop the alpha channel from an LdrColor render so it matches the 3-channel GT images."""
    arr = np.asarray(image)
    if arr.ndim == 3 and arr.shape[-1] == 4:
        return arr[..., :3]
    return arr


def _save_images(out_dir: str, stem: str, gt: np.ndarray, bind: np.ndarray, clone: np.ndarray) -> None:
    """Save gt/bind/clone and an amplified clone-vs-bind diff as PNGs under out_dir for visual inspection."""
    from PIL import Image

    os.makedirs(out_dir, exist_ok=True)
    Image.fromarray(gt).save(os.path.join(out_dir, f"{stem}_gt.png"))
    Image.fromarray(bind).save(os.path.join(out_dir, f"{stem}_bind.png"))
    Image.fromarray(clone).save(os.path.join(out_dir, f"{stem}_clone.png"))
    Image.fromarray(image_diff(bind, clone)).save(os.path.join(out_dir, f"{stem}_diff_clone_bind.png"))


def _render_target_keyframes(stage, name: str, target, ts_list: list[int]) -> dict:
    """Render one RenderTarget at each keyframe; returns ``{ts_ns: rgb image or None}``, closing the renderer.

    Renders a single bound RenderProduct at a time: an authored RenderProduct and a clone of it cannot be
    bound simultaneously, so the bind and clone passes each run alone.
    """
    renderer = CameraRenderer.open(stage, name, simulation_app, target, warmup_steps=WARMUP_STEPS)
    images: dict = {}
    try:
        for ts_ns in ts_list:
            result = renderer.render_at_keyframe(ts_ns, TOL_US)
            images[ts_ns] = _to_rgb(result[0]) if result is not None else None
    finally:
        renderer.close()
    return images


def run_case(case: dict) -> CaseResult:
    """Render bind + clone for each camera at N random GT keyframes; score the clone and compare to bind.

    Returns:
        `CaseResult.PASS` (clone passed the GT gate and matched the bind), `CaseResult.FAIL` (failed a
        gate / mismatch), or `CaseResult.SKIP` (not an SPG stage, no GT timestamps, or no authored
        RenderProducts for the requested cameras).
    """
    name = case.get("name") or "default"
    carb.log_warn(f"==================== copy-rp case: {name} ====================")
    ok, stage = open_stage(resolve_path(case["stage"], cfg_dir))
    if not ok or stage is None:
        carb.log_error(f"[{name}] failed to open stage: {case['stage']}")
        return CaseResult.SKIP

    success, nurec, spg, problems = setup_for_rendering(stage, args.config)
    if not (success and spg):
        carb.log_error(f"[{name}] not a renderable SPG stage: nurec={nurec} spg={spg} problems={problems}")
        return CaseResult.SKIP

    thresholds = case.get("thresholds") or {}
    min_psnr = args.min_psnr if args.min_psnr is not None else thresholds.get("min_psnr")
    min_ssim = args.min_ssim if args.min_ssim is not None else thresholds.get("min_ssim")
    max_abs_diff_px = float(thresholds.get("max_abs_clone_bind_diff_px", 2.0))
    num_samples = args.num_samples if args.num_samples is not None else cfg.get("num_samples")
    cameras = set(case["cameras"]) if case.get("cameras") else None

    gt_root = resolve_gt_root(resolve_path(case["gt_root"], cfg_dir))
    per_camera_ts = read_gt_timestamps(gt_root, cameras)
    if not per_camera_ts:
        carb.log_error(f"[{name}] no GT timestamps under {case['gt_root']} for cameras {cameras}")
        return CaseResult.SKIP

    render_products = discover_render_products(stage)
    factory = RenderTargetFactory(has_spg=True)
    rows: list[dict] = []
    failures: list[str] = []

    # Create every clone before the first render. omni.rtx.spg registers a PPISP render product when it
    # populates the stage at a hydra sync; a clone must already exist at that point to be developed, or it
    # renders raw radiance. So build all clones, then sync once, then render.
    work = []
    for camera, all_ts in per_camera_ts.items():
        if camera not in render_products:
            failures.append(f"{camera}: no authored RenderProduct on the stage")
            continue
        rp_path = render_products[camera]
        camera_path = str(stage.GetPrimAtPath(rp_path).GetRelationship("camera").GetTargets()[0])
        ts_list = sorted(random.Random(SEED).sample(all_ts, min(num_samples, len(all_ts)))) if num_samples else all_ts
        clone = factory.clone(stage, f"{camera}_copy", src_rp_path=rp_path, camera_path=camera_path)
        work.append((camera, rp_path, ts_list, clone))
    simulation_app.update()

    # An authored RenderProduct and a clone of it cannot be bound at the same time, so render the clone
    # first (its render is the first bound-RP sync, with the clone present), then the authored RenderProduct.
    for camera, rp_path, ts_list, clone in work:
        clone_images = _render_target_keyframes(stage, f"{camera}_copy", clone, ts_list)
        bind_images = _render_target_keyframes(
            stage, f"{camera}_bind", factory.create(stage, f"{camera}_bind", rp_path=rp_path), ts_list
        )

        for ts_ns in ts_list:
            bind_image, copy_image = bind_images.get(ts_ns), clone_images.get(ts_ns)
            if bind_image is None or copy_image is None:
                failures.append(
                    f"{camera}@{ts_ns}: render returned None (bind={bind_image is not None}, copy={copy_image is not None})"
                )
                continue
            gt_image = load_rgb(gt_path(gt_root, camera, ts_ns))
            clone_vs_gt = score(gt_image, match_shape(copy_image, gt_image))
            bind_vs_gt = score(gt_image, match_shape(bind_image, gt_image))
            abs_clone_bind_diff_px = mean_abs_diff(bind_image, copy_image)
            rows.append(
                {
                    "psnr": clone_vs_gt["psnr"],
                    "ssim": clone_vs_gt["ssim"],
                    "abs_clone_bind_diff_px": abs_clone_bind_diff_px,
                }
            )
            carb.log_warn(
                f"[{name}] {camera}@{ts_ns}: clone_vs_gt psnr={clone_vs_gt['psnr']:.2f} ssim={clone_vs_gt['ssim']:.4f} "
                f"| bind_vs_gt psnr={bind_vs_gt['psnr']:.2f} ssim={bind_vs_gt['ssim']:.4f} "
                f"| abs_clone_bind_diff_px={abs_clone_bind_diff_px:.3f}"
            )
            if SAVE_DIR:
                _save_images(SAVE_DIR, f"{name}_{camera}_{ts_ns}", gt_image, bind_image, copy_image)
            if abs_clone_bind_diff_px > max_abs_diff_px:
                failures.append(
                    f"{camera}@{ts_ns}: clone differs from bind (abs_clone_bind_diff_px={abs_clone_bind_diff_px:.3f})"
                )

    if not rows:
        carb.log_error(f"[{name}] nothing rendered/scored")
        return CaseResult.SKIP

    psnr_mean = float(np.mean([r["psnr"] for r in rows]))
    ssim_mean = float(np.mean([r["ssim"] for r in rows]))
    worst_abs_diff_px = float(np.max([r["abs_clone_bind_diff_px"] for r in rows]))
    sanity_ok = (min_psnr is None or psnr_mean >= min_psnr) and (min_ssim is None or ssim_mean >= min_ssim)
    carb.log_warn(
        f"[{name}] {len(rows)} frames clone_vs_gt PSNR={psnr_mean:.2f} SSIM={ssim_mean:.4f} "
        f"(gate {min_psnr}/{min_ssim}) | worst abs_clone_bind_diff_px={worst_abs_diff_px:.3f} (gate {max_abs_diff_px})"
    )
    if failures or not sanity_ok:
        for failure in failures:
            carb.log_error(f"[{name}] FAIL {failure}")
        if not sanity_ok:
            carb.log_error(f"[{name}] FAIL clone-vs-GT below gate (PSNR={psnr_mean:.2f} SSIM={ssim_mean:.4f})")
        return CaseResult.FAIL
    carb.log_warn(f"[{name}] PASS: clone renders correctly vs GT and matches bind")
    return CaseResult.PASS


results = {(case.get("name") or "default"): run_case(case) for case in cases}
carb.log_warn(f"=== copy-rp results: { {name: r.name for name, r in results.items()} } ===")
exit_code = 0 if all(r is CaseResult.PASS for r in results.values()) else 1
simulation_app.close()
sys.exit(exit_code)
