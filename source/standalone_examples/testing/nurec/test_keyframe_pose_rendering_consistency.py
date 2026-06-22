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

"""Keyframe-vs-pose rendering consistency, measured against ground truth (GT).

For each GT frame of a NuRec USD it scores the SPG render two ways against the *same* GT
image and checks the two agree:

  1. **keyframe** — `render_at_keyframe(ts)` replays the sensor-rig keyframe; score vs GT.
  2. **pose**     — extract that keyframe's world pose (`camera_world_pose`), round-trip it
     through a TUM file, `render_at_pose(pose)`, and score vs GT.

If `render_at_pose` poses the camera the same way the rig does, the two PSNR/SSIM-vs-GT
numbers should match. The gate is on the **delta** (|poses-vs-GT − keyframe-vs-GT|): pose
rendering should reproduce the keyframe's agreement with GT, modulo RTPT jitter. Absolute
render-vs-GT quality is covered by `test_nurec_render_vs_gt.py`.

The stage and gt-root default to Isaac-relative paths (``/Isaac/Samples/...``) that are
expanded via ``get_assets_root_path()`` at runtime; pass ``--stage`` / ``--gt-root`` to
override with a local path or an omniverse:// URL.

Usage:
    ./python.sh source/standalone_examples/testing/nurec/test_keyframe_pose_rendering_consistency.py \
        [--stage <NuRec USD-or-omniverse-url>] [--gt-root <gt>] [--cameras front_stereo_camera_left] \
        [--num-samples 3] [--save-images]
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from typing import Any

_STAGE = "/Isaac/Samples/NuRec/Galileo/NRE_26_06/PPISP_ON/particle_spg.usdz"
_GT_ROOT = "/Isaac/Samples/NuRec/Galileo/CI_Test_GT_Images"

parser = argparse.ArgumentParser(description="Check render_at_pose matches render_at_keyframe against GT.")
parser.add_argument(
    "--stage",
    default=_STAGE,
    help="NuRec SPG USD (Isaac-relative /Isaac/... path, local path)",
)
parser.add_argument(
    "--gt-root",
    default=_GT_ROOT,
    help="GT tree laid out as <gt>/<camera>/<ts_ns>.png (Isaac-relative /Isaac/... path, local path)",
)
parser.add_argument(
    "--cameras",
    nargs="+",
    default=["front_stereo_camera_left"],
    help="camera names to check",
)
parser.add_argument("--num-samples", type=int, default=3, help="GT frames to sample per camera")
parser.add_argument(
    "--keyframe-tolerance-us",
    type=float,
    default=1000.0,
    help="ts -> rig-keyframe match window (us)",
)
parser.add_argument("--out-dir", default=None, help="optional dir for the TUM files + saved frames")
parser.add_argument(
    "--max-psnr-delta",
    type=float,
    default=1.0,
    help="max |poses-vs-GT - keyframe-vs-GT| PSNR (dB)",
)
parser.add_argument(
    "--max-ssim-delta",
    type=float,
    default=0.03,
    help="max |poses-vs-GT - keyframe-vs-GT| SSIM",
)
parser.add_argument("--save-images", action="store_true", help="save keyframe/pose/diff frames")
args, _ = parser.parse_known_args()

stage_path = os.path.expanduser(args.stage)
gt_root = os.path.expanduser(args.gt_root)
out_dir = (
    os.path.abspath(os.path.expanduser(args.out_dir))
    if args.out_dir
    else os.path.join(tempfile.gettempdir(), "nurec_pose_consistency")
)

# Enable the rendering utilities (which pulls omni.rtx.spg) at launch; render single-GPU.
# setup_for_rendering loads the carb overrides from the shipped config and applies them per stage.
extra_args = [
    "--enable",
    "isaacsim.replicator.nurec_utils",
    "--/renderer/multiGpu/enabled=false",
]
print(
    f"[pose_consistency] stage={stage_path} gt_root={gt_root} cameras={args.cameras}",
    flush=True,
)

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True, "multi_gpu": False, "extra_args": extra_args})

import carb
from isaacsim.replicator.nurec_utils import render as r
from isaacsim.replicator.nurec_utils.eval import (
    gt_path,
    read_gt_timestamps,
    resolve_gt_root,
)
from isaacsim.replicator.nurec_utils.metrics.psnr_ssim import (
    image_diff,
    load_rgb,
    match_shape,
    score,
)
from isaacsim.replicator.nurec_utils.rendering_setup import (
    enable_omni_rtx_spg,
    resolve_path,
    setup_for_rendering,
)
from isaacsim.replicator.nurec_utils.usd_utils import (
    camera_world_pose,
    open_stage,
    read_tum,
    save_image,
    subsample,
    write_tum,
)

simulation_app.update()
enable_omni_rtx_spg(simulation_app)

stage_path = resolve_path(stage_path, os.getcwd())
gt_root = resolve_path(gt_root, os.getcwd())

failures: list[str] = []


def _dataset_name(gt_root: str) -> str:
    """Return a stable label for one GT root, used for logs and output folders.

    Args:
        gt_root: GT tree path.

    Returns:
        The parent directory name when `gt_root` is a `gt` folder, else the basename.
    """
    base = os.path.basename(gt_root)
    return os.path.basename(os.path.dirname(gt_root)) if base == "gt" else base


def check_camera(
    stage: Any,
    cam: str,
    rp_path: str,
    gt_root: str,
    per_camera_ts: dict[str, list[int]],
) -> None:
    """Score keyframe- and pose-rendered frames against the same GT and gate the delta.

    Args:
        stage: The open USD stage.
        cam: Camera logical name (for logs / output paths).
        rp_path: The authored RenderProduct prim path for the camera.
        gt_root: GT tree to score against.
        per_camera_ts: Timestamp map derived from `gt_root`.
    """
    dataset = _dataset_name(gt_root)
    dataset_out_dir = os.path.join(out_dir, dataset)
    ts_list = subsample(per_camera_ts.get(cam, []), int(args.num_samples))
    if not ts_list:
        carb.log_error(f"[{dataset}/{cam}] no GT timestamps under {gt_root}; cannot run pose consistency.")
        failures.append(f"{dataset}/{cam}: no GT")
        return
    target = factory.create(stage, cam, rp_path=rp_path)
    cap = r.CameraRenderer.open(stage, cam, simulation_app, target, force_identity_exposure=has_spg)
    try:
        # Phase 1: render at each GT timestamp's keyframe, score vs GT, extract the pose.
        refs: list[tuple[int, list[float], Any, dict, Any]] = []  # (ts_ns, pose, gt, kf_score, kf_rgb)
        for ts_ns in ts_list:
            gt = load_rgb(gt_path(gt_root, cam, ts_ns))
            result = cap.render_at_keyframe(ts_ns, tol_us=args.keyframe_tolerance_us)
            if result is None:
                carb.log_warn(f"[{dataset}/{cam}] ts_ns={ts_ns}: no keyframe within tolerance; skipping.")
                continue
            kf_rgb, matched_tc = result
            kf_sc = score(gt, match_shape(kf_rgb, gt))
            pose = camera_world_pose(stage, cap.camera_path, matched_tc)
            refs.append((ts_ns, pose, gt, kf_sc, kf_rgb))

        # TUM round-trip: write the extracted poses, read them back.
        tum_path = os.path.join(dataset_out_dir, f"{cam}.tum")
        write_tum(tum_path, [(ts_ns, pose) for ts_ns, pose, _, _, _ in refs])
        tum_entries = read_tum(tum_path)

        # Phase 2: render at each (round-tripped) pose, score vs the same GT, gate the delta.
        scored = 0
        for (ts_ns, _, gt, kf_sc, kf_rgb), (_, pose) in zip(refs, tum_entries):
            posed = cap.render_at_pose(pose)
            if posed is None:
                carb.log_warn(f"[{dataset}/{cam}] pose ts_ns={ts_ns}: no frame; skipping.")
                continue
            pose_sc = score(gt, match_shape(posed, gt))
            scored += 1
            d_psnr = pose_sc["psnr"] - kf_sc["psnr"]
            d_ssim = pose_sc["ssim"] - kf_sc["ssim"]
            ok = abs(d_psnr) <= args.max_psnr_delta and abs(d_ssim) <= args.max_ssim_delta
            carb.log_warn(
                f"[{dataset}/{cam}] ts_ns={ts_ns} | keyframe-vs-GT PSNR={kf_sc['psnr']:.2f} SSIM={kf_sc['ssim']:.4f} "
                f"| pose-vs-GT PSNR={pose_sc['psnr']:.2f} SSIM={pose_sc['ssim']:.4f} "
                f"| dPSNR={d_psnr:+.2f} dSSIM={d_ssim:+.4f} -> {'OK' if ok else 'FAIL'}"
            )
            if not ok:
                failures.append(f"{dataset}/{cam}:{ts_ns} dPSNR={d_psnr:+.2f} dSSIM={d_ssim:+.4f}")
            if args.save_images:
                stem = f"{cam}_{ts_ns}"
                save_image(os.path.join(dataset_out_dir, "keyframe", f"{stem}.png"), kf_rgb)
                save_image(os.path.join(dataset_out_dir, "pose", f"{stem}.png"), posed)
                save_image(
                    os.path.join(dataset_out_dir, "diff", f"{stem}.png"),
                    image_diff(kf_rgb, posed),
                )

        # A camera with GT timestamps that scored nothing (no keyframe matched, or no frame
        # produced) is a failure, not a silent pass.
        if not scored:
            carb.log_error(
                f"[{dataset}/{cam}] scored 0 frames (no keyframe matched within {args.keyframe_tolerance_us}us)."
            )
            failures.append(f"{dataset}/{cam}: 0 frames scored")
    finally:
        cap.close()


stage = open_stage(stage_path)
if stage is None:
    carb.log_error(f"Failed to open stage: {stage_path}")
    simulation_app.close()
    sys.exit(2)
simulation_app.update()
success, _, has_spg, _ = setup_for_rendering(stage)
if not success:
    simulation_app.close()
    sys.exit(2)
factory = r.RenderTargetFactory(has_spg)

rps = r.discover_render_products(stage)
rps = {c: rp for c, rp in rps.items() if c in args.cameras}
if not rps:
    carb.log_error(f"No RenderProducts under /Render for {args.cameras}.")
    simulation_app.close()
    sys.exit(2)
gt_root = resolve_gt_root(gt_root)
per_camera_ts = read_gt_timestamps(gt_root, set(args.cameras))
for cam, rp_path in rps.items():
    check_camera(stage, cam, rp_path, gt_root, per_camera_ts)

carb.log_warn(
    f"=== pose consistency vs GT: {'ALL OK' if not failures else f'{len(failures)} FAILURE(S): {failures}'} ==="
)
simulation_app.close()
sys.exit(1 if failures else 0)
