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

"""Score rendered NuRec frames against a ground-truth (GT) image tree.

The GT tree is laid out as `<gt_root>/<camera>/<ts_ns>.<ext>`, one image per camera and
nanosecond timestamp. `read_gt_timestamps` and `gt_path` read that layout, `evaluate` scores
a render manifest against it (PSNR / SSIM / image-diff, with optional panels and plots), and
`render_and_score` renders the GT timestamps then scores them. Used by the nurec render-vs-GT
and pose-consistency tests.

Scoring is torch-based, so the caller must already have a `SimulationApp` running.
"""

from __future__ import annotations

import atexit
import csv
import json
import os
import posixpath
from typing import Any

import carb
import numpy as np
from isaacsim.replicator.nurec_utils.manifest import load_manifest
from isaacsim.replicator.nurec_utils.metrics.psnr_ssim import (
    image_diff,
    load_rgb,
    match_shape,
    score,
)
from isaacsim.replicator.nurec_utils.metrics.scoped_timer import ScopedTimer
from isaacsim.replicator.nurec_utils.render import render_keyframes
from isaacsim.replicator.nurec_utils.usd_utils import is_remote_path, subsample

# (csv/json key, plot label) for the headline metrics.
HEADLINES = [("psnr", "PSNR (dB)"), ("ssim", "SSIM"), ("mean_abs_diff", "MAD")]
_IMAGE_EXTS = (".png", ".jpg", ".jpeg")

# Maps a remote `gt_root` URL to its local mirror directory.
_GT_MIRROR_CACHE: dict[str, str] = {}


@atexit.register
def _cleanup_gt_mirrors() -> None:
    """Remove the temporary directories that remote `gt_root`s were mirrored into."""
    import shutil

    for path in _GT_MIRROR_CACHE.values():
        shutil.rmtree(path, ignore_errors=True)


def _log_timer(label: str, timer: ScopedTimer) -> None:
    if timer.elapsed_time_ms is not None:
        carb.log_info(f"[nurec timing] {label}: {timer.elapsed_time_ms:.3f} ms")


def resolve_gt_root(gt_root: str) -> str:
    """Return a local directory for `gt_root`.

    A local path is returned as an absolute path. A remote URL (e.g. `omniverse://`) is mirrored
    into a temporary directory whose path is returned. Requires a running SimulationApp when
    `gt_root` is remote.

    Args:
        gt_root: A local path or a remote URL (e.g. `omniverse://`).

    Returns:
        An absolute local directory: the resolved path, or the temporary mirror for a remote root.
    """
    if not is_remote_path(gt_root):
        return os.path.abspath(os.path.expanduser(gt_root))
    if gt_root in _GT_MIRROR_CACHE:
        carb.log_info(f"[nurec timing] omniverse GT mirror: cache hit {gt_root} -> {_GT_MIRROR_CACHE[gt_root]}")
        return _GT_MIRROR_CACHE[gt_root]
    import tempfile

    import omni.client

    local_dir = tempfile.mkdtemp(prefix="nurec_gt_")
    carb.log_info(f"[nurec] mirroring GT tree {gt_root} -> {local_dir}")
    with ScopedTimer("omniverse_gt_mirror") as timer:
        n_files = _mirror_tree(omni.client, gt_root.rstrip("/"), local_dir)
    carb.log_info(f"[nurec] mirrored {n_files} GT file(s) from {gt_root}")
    _log_timer(f"omniverse GT mirror {gt_root} -> {local_dir}", timer)
    _GT_MIRROR_CACHE[gt_root] = local_dir
    return local_dir


def _mirror_tree(client: Any, url: str, local_dir: str) -> int:
    """Download the folder `url` into `local_dir`, recursing into subfolders; return the file count.

    Args:
        client: The `omni.client` module.
        url: The remote folder URL to mirror.
        local_dir: The local directory to write into.

    Returns:
        The number of files downloaded.
    """
    res, entries = client.list(url)
    if res != client.Result.OK:
        raise RuntimeError(f"omni.client.list failed for {url}: {res}")
    count = 0
    for entry in entries:
        name = entry.relative_path
        child_url = posixpath.join(url, name)
        if int(entry.flags) & int(client.ItemFlags.CAN_HAVE_CHILDREN):
            os.makedirs(os.path.join(local_dir, name), exist_ok=True)
            count += _mirror_tree(client, child_url, os.path.join(local_dir, name))
        else:
            res_read, _, content = client.read_file(child_url)
            if res_read != client.Result.OK:
                raise RuntimeError(f"omni.client.read_file failed for {child_url}: {res_read}")
            with open(os.path.join(local_dir, name), "wb") as f:
                f.write(bytes(memoryview(content)))
            count += 1
    return count


def read_gt_timestamps(gt_root: str, cameras: set[str] | None = None) -> dict[str, list[int]]:
    """Derive per-camera timestamps from a GT tree `<gt_root>/<camera>/<ts_ns>.<ext>`.

    Args:
        gt_root: Root of the GT tree.
        cameras: When given, keep only these camera subdirectories.

    Returns:
        A dict mapping each camera to its sorted nanosecond timestamps.
    """
    gt_root = resolve_gt_root(gt_root)
    out: dict[str, list[int]] = {}
    if not os.path.isdir(gt_root):
        return out
    for cam in sorted(os.listdir(gt_root)):
        if cameras is not None and cam not in cameras:
            continue
        cam_dir = os.path.join(gt_root, cam)
        if not os.path.isdir(cam_dir):
            continue
        ts = [
            int(stem)
            for stem, ext in (os.path.splitext(n) for n in os.listdir(cam_dir))
            if ext.lower() in _IMAGE_EXTS and stem.isdigit()
        ]
        if ts:
            out[cam] = sorted(ts)
    return out


def gt_path(gt_root: str, camera: str, ts_ns: int) -> str:
    """Return the GT image path for a camera + timestamp, resolving its extension.

    Finds `<gt_root>/<camera>/<ts_ns>.<ext>` by stem (`.png` / `.jpg` / `.jpeg`); falls back
    to a `.png` path when no matching file is present.

    Args:
        gt_root: Root of the GT tree.
        camera: Camera name.
        ts_ns: Timestamp in nanoseconds.

    Returns:
        The GT image path.
    """
    for ext in _IMAGE_EXTS:
        candidate = os.path.join(gt_root, camera, f"{ts_ns}{ext}")
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(gt_root, camera, f"{ts_ns}.png")


def _aggregate(rows: list[dict]) -> dict:
    out: dict = {"n": len(rows)}
    for k, _ in HEADLINES:
        vals = [r[k] for r in rows]
        out[f"{k}_mean"] = float(np.mean(vals))
        out[f"{k}_median"] = float(np.median(vals))
    return out


def _write_plots(rows: list[dict], plots_dir: str) -> None:
    from isaacsim.replicator.nurec_utils.metrics.plotting import (
        plot_histogram,
        plot_metric_over_index,
        plot_pose_heatmap,
    )

    have_pos = bool(rows) and all(r.get("position") is not None for r in rows)
    for key, label in HEADLINES:
        vals = [r[key] for r in rows]
        plot_metric_over_index(vals, label, os.path.join(plots_dir, f"{key}_over_index.png"))
        plot_histogram(vals, label, os.path.join(plots_dir, f"{key}_hist.png"))
        if have_pos:
            plot_pose_heatmap(
                [r["position"] for r in rows],
                vals,
                label,
                os.path.join(plots_dir, f"{key}_pose_heatmap.png"),
            )


def _write_panels(rows: list[dict], cameras: list[str], out_dir: str) -> None:
    """Write a `GT | rendered | diff` panel per frame, over its camera's PSNR/SSIM trend.

    Args:
        rows: The scored frame rows (each with camera, ts_ns, psnr, ssim, gt, rendered).
        cameras: The camera names to write panels for.
        out_dir: Output directory; panels are written under `<out_dir>/panels/<camera>/`.
    """
    from isaacsim.replicator.nurec_utils.metrics.plotting import save_comparison_panel

    for cam in cameras:
        cam_rows = sorted((r for r in rows if r["camera"] == cam), key=lambda r: int(r["ts_ns"]))
        timestamps = [int(r["ts_ns"]) for r in cam_rows]
        psnr_series = [r["psnr"] for r in cam_rows]
        ssim_series = [r["ssim"] for r in cam_rows]
        for r in cam_rows:
            gt_img = load_rgb(r["gt"])
            rendered = match_shape(load_rgb(r["rendered"]), gt_img)
            stem = os.path.splitext(os.path.basename(r["rendered"]))[0]
            save_comparison_panel(
                gt_img,
                rendered,
                image_diff(gt_img, rendered),
                r["psnr"],
                r["ssim"],
                os.path.join(out_dir, "panels", cam, f"{stem}.png"),
                title=f"{cam}  {stem}",
                timestamps=timestamps,
                psnr_series=psnr_series,
                ssim_series=ssim_series,
                current_ts=int(r["ts_ns"]),
            )


def evaluate(
    manifest_path: str,
    gt_root: str,
    out_dir: str | None = None,
    write_panels: bool = True,
    write_plots: bool = True,
) -> dict | None:
    """Score every frame in a render manifest against its GT and write metrics/panels/plots.

    Each rendered frame is matched to its GT by `gt_path(gt_root, camera, ts_ns)`. Assumes a
    `SimulationApp` is already running (torch must be importable); the caller owns the app.

    Args:
        manifest_path: Path to the render manifest.json to score.
        gt_root: Root of the GT tree to resolve GT images against.
        out_dir: Output directory; defaults to `<manifest dir>/eval`.
        write_panels: Whether to write per-frame `GT | rendered | diff` panels.
        write_plots: Whether to write per-metric time/histogram/pose plots.

    Returns:
        A summary dict with "manifest", "overall", "by_camera", and "n_skipped", or None when
        no frame could be scored against GT.
    """
    manifest_path = os.path.abspath(os.path.expanduser(manifest_path))
    gt_root = resolve_gt_root(gt_root)
    _, pairs = load_manifest(manifest_path)
    out_dir = (
        os.path.abspath(os.path.expanduser(out_dir))
        if out_dir
        else os.path.join(os.path.dirname(manifest_path), "eval")
    )
    os.makedirs(out_dir, exist_ok=True)

    rows: list[dict] = []
    skipped = 0
    for pair in pairs:
        cam = pair.get("camera", "")
        rendered_path = pair.get("rendered", "")
        gt = gt_path(gt_root, cam, pair.get("ts_ns"))
        if not (os.path.isfile(gt) and os.path.isfile(rendered_path)):
            skipped += 1
            continue
        gt_img = load_rgb(gt)
        rendered = match_shape(load_rgb(rendered_path), gt_img)
        sc = score(gt_img, rendered)
        row = {
            "camera": cam,
            "ts_ns": pair.get("ts_ns", ""),
            **sc,
            "rendered": rendered_path,
            "gt": gt,
        }
        if pair.get("position") is not None:
            row["position"] = pair["position"]
        rows.append(row)

        stem = os.path.splitext(os.path.basename(rendered_path))[0]
        carb.log_info(f"  {cam} {stem} psnr={sc['psnr']:6.2f} ssim={sc['ssim']:.4f} mad={sc['mean_abs_diff']:6.2f}")

    if not rows:
        return None

    overall = _aggregate(rows)
    by_camera = {c: _aggregate([r for r in rows if r["camera"] == c]) for c in sorted({r["camera"] for r in rows})}

    with open(os.path.join(out_dir, "metrics.csv"), "w", newline="") as f:
        cols = ["camera", "ts_ns", "psnr", "ssim", "mean_abs_diff", "rendered", "gt"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")  # position is a list -> excluded
        w.writeheader()
        w.writerows(rows)
    summary = {
        "manifest": manifest_path,
        "overall": overall,
        "by_camera": by_camera,
        "n_skipped": skipped,
    }
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    if write_panels:
        _write_panels(rows, list(by_camera), out_dir)

    if write_plots:
        _write_plots(rows, os.path.join(out_dir, "plots", "overall"))
        for cam in by_camera:
            _write_plots(
                [r for r in rows if r["camera"] == cam],
                os.path.join(out_dir, "plots", cam),
            )

    carb.log_info(
        f"[{overall['n']} frames] PSNR={overall['psnr_mean']:.2f} dB SSIM={overall['ssim_mean']:.4f} "
        f"MAD={overall['mean_abs_diff_mean']:.2f} (skipped {skipped}) -> {out_dir}"
    )
    return summary


def render_and_score(
    simulation_app: Any,
    stage_path: str,
    gt_root: str,
    out_dir: str,
    *,
    cameras: set[str] | None = None,
    num_samples: int | None = None,
    keyframe_tol_us: float = 1.0,
    warmup_steps: int = 800,
    config_path: str | None = None,
    write_panels: bool = True,
    write_plots: bool = True,
    timing_label: str | None = None,
) -> dict | None:
    """Render the GT timestamps for `stage_path`, then score the result against GT.

    Derives the timestamps from `gt_root`, renders them with `render_keyframes`, and scores via
    `evaluate`. Assumes a `SimulationApp` is running with `omni.rtx.spg` enabled (the caller
    owns boot/teardown).

    Args:
        simulation_app: The already-booted SimulationApp.
        stage_path: Path/URL of the NuRec USD to render.
        gt_root: Root of the GT tree (supplies the timestamps and the scoring images).
        out_dir: Output directory; frames go under `out_dir/render`, scores under `out_dir/eval`.
        cameras: Restrict to these cameras; None uses every camera found under `gt_root`.
        num_samples: Subsample this many timestamps per camera (None = all).
        keyframe_tol_us: Tolerance (us) for matching a timestamp to a rig keyframe.
        warmup_steps: RTPT accumulation ticks per frame.
        config_path: Optional YAML overriding the shipped render config (carb overrides).
        write_panels: Whether to write per-frame comparison panels.
        write_plots: Whether to write per-metric plots.
        timing_label: Optional label used for scoped timing output.

    Returns:
        The `evaluate` summary dict, or None when nothing was rendered or scored.
    """
    label = timing_label or os.path.basename(stage_path.rstrip("/")) or "nurec"
    with ScopedTimer(f"{label}.gt_timestamps") as timer:
        per_camera_ts = read_gt_timestamps(gt_root, cameras)
    _log_timer(f"{label} GT timestamp discovery", timer)
    if not per_camera_ts:
        carb.log_error(f"no GT timestamps under {gt_root}.")
        return None
    if num_samples:
        per_camera_ts = {cam: subsample(ts, int(num_samples)) for cam, ts in per_camera_ts.items()}

    with ScopedTimer(f"{label}.render_keyframes_total") as timer:
        manifest_path = render_keyframes(
            simulation_app,
            stage_path,
            os.path.join(out_dir, "render"),
            per_camera_ts,
            warmup_steps=warmup_steps,
            keyframe_tol_us=keyframe_tol_us,
            config_path=config_path,
            timing_label=label,
        )
    _log_timer(f"{label} render_keyframes total", timer)
    if manifest_path is None:
        return None
    with ScopedTimer(f"{label}.evaluate") as timer:
        summary = evaluate(
            manifest_path,
            gt_root,
            out_dir=os.path.join(out_dir, "eval"),
            write_panels=write_panels,
            write_plots=write_plots,
        )
    _log_timer(f"{label} evaluate", timer)
    return summary
