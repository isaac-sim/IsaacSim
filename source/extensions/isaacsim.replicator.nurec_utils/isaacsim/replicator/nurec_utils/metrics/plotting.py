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

"""Plotting helpers for evaluation output: time/index plots, histograms, pose heatmaps, panels.

Uses matplotlib's headless Agg backend (no display).
"""

from __future__ import annotations

import os
from collections.abc import Iterable

import matplotlib.pyplot as plt
import numpy as np

plt.switch_backend("Agg")  # headless backend, no display


def plot_metric_over_index(values: Iterable[float], metric_name: str, out_path: str) -> None:
    """Plot a metric over frame index (the time-ordered trajectory) as a line.

    Args:
        values: The metric value per frame, in trajectory order.
        metric_name: Label for the metric (used on the y-axis and title).
        out_path: Path to write the PNG to (parent dirs are created).
    """
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(list(values), marker=".", linestyle="-", linewidth=0.8, markersize=2)
    ax.set_xlabel("frame index")
    ax.set_ylabel(metric_name)
    ax.set_title(f"{metric_name} over frames")
    ax.grid(alpha=0.3)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_histogram(values: Iterable[float], metric_name: str, out_path: str) -> None:
    """Plot the distribution of a metric across frames as a histogram.

    Args:
        values: The metric values to histogram.
        metric_name: Label for the metric (used on the x-axis and title).
        out_path: Path to write the PNG to (parent dirs are created).
    """
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(list(values), bins=30)
    ax.set_xlabel(metric_name)
    ax.set_ylabel("count")
    ax.set_title(f"{metric_name} distribution")
    ax.grid(alpha=0.3)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_pose_heatmap(
    positions: Iterable[tuple[float, float, float]],
    values: Iterable[float],
    metric_name: str,
    out_path: str,
    axes: str = "xy",
    cmap: str = "viridis",
    vmin: float | None = None,
    vmax: float | None = None,
) -> None:
    """Plot a top-down scatter of camera positions colored by `values` (e.g. PSNR or SSIM).

    Args:
        positions: Per-frame camera world positions as (x, y, z).
        values: The metric value per position, used as the point color.
        metric_name: Label for the metric (used on the colorbar and title).
        out_path: Path to write the PNG to (parent dirs are created).
        axes: Projection plane to scatter on — "xy", "xz", or "yz" — pick the dataset's
            ground plane.
        cmap: Matplotlib colormap name.
        vmin: Lower bound of the color scale; pass the same across datasets for comparable
            heatmaps.
        vmax: Upper bound of the color scale; pass the same across datasets for comparable
            heatmaps.
    """
    pos = np.asarray(list(positions), dtype=np.float64)
    vals = np.asarray(list(values), dtype=np.float64)
    idx_map = {"x": 0, "y": 1, "z": 2}
    ax_i, ax_j = idx_map[axes[0]], idx_map[axes[1]]

    fig, ax = plt.subplots(figsize=(7, 6))
    sc = ax.scatter(pos[:, ax_i], pos[:, ax_j], c=vals, cmap=cmap, s=18, edgecolors="none", vmin=vmin, vmax=vmax)
    ax.set_xlabel(f"{axes[0]} (m)")
    ax.set_ylabel(f"{axes[1]} (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.set_title(f"{metric_name} along trajectory ({axes.upper()} plane)")
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label(metric_name)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def save_comparison_panel(
    gt: np.ndarray,
    rendered: np.ndarray,
    diff: np.ndarray,
    psnr_value: float,
    ssim_value: float,
    out_path: str,
    title: str | None = None,
    diff_scale: float = 4.0,
    timestamps: list[int] | None = None,
    psnr_series: list[float] | None = None,
    ssim_series: list[float] | None = None,
    current_ts: int | None = None,
) -> None:
    """Write a `GT | rendered | diff` panel, optionally over a camera's PSNR/SSIM trend.

    When `timestamps` and the metric series are given, a second row plots this camera's PSNR and
    SSIM against elapsed seconds from its first frame, with a mean line and a marker at this frame.

    Args:
        gt: The ground-truth image.
        rendered: The rendered image.
        diff: The (amplified) difference image.
        psnr_value: PSNR in dB for this frame, shown in the caption.
        ssim_value: SSIM for this frame, shown in the caption.
        out_path: Path to write the PNG to (parent dirs are created).
        title: Optional figure title (suptitle).
        diff_scale: Amplification factor the diff image was scaled by, shown in its panel label.
        timestamps: This camera's frame timestamps (ns), time-sorted; enables the trend row.
        psnr_series: This camera's PSNR per frame, aligned to `timestamps`.
        ssim_series: This camera's SSIM per frame, aligned to `timestamps`.
        current_ts: Timestamp (ns) of this frame, marked on the trend plots.
    """
    diff_label = f"|GT - rendered| x {diff_scale:g}"
    caption = f"PSNR = {psnr_value:.2f} dB     SSIM = {ssim_value:.3f}"
    have_trend = bool(timestamps) and psnr_series is not None and ssim_series is not None

    if not have_trend:
        fig, img_axes = plt.subplots(1, 3, figsize=(15, 5.5))
        for ax, image, label in zip(img_axes, (gt, rendered, diff), ("GT", "rendered", diff_label)):
            ax.imshow(image)
            ax.set_title(label)
            ax.axis("off")
        fig.text(0.5, 0.04, caption, ha="center", va="center", fontsize=12)
        if title:
            fig.suptitle(title, fontsize=10)
        fig.tight_layout(rect=(0, 0.06, 1, 0.97 if title else 1.0))
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        fig.savefig(out_path, dpi=120)
        plt.close(fig)
        return

    # Dark theme. Size the whole figure from the input image aspect so each GT|rendered|diff panel shows
    # at native aspect (no stretch, no wasted bars); the bottom trend row and overall size follow from it.
    bg, fg = "black", "white"
    img_aspect = gt.shape[0] / gt.shape[1]  # height / width of the source images
    fig_w, left, right = 16.0, 0.04, 0.99
    img_row_h = fig_w * (right - left) / 3.0 * img_aspect  # height of one native-aspect panel across a third
    plots_row_h = max(2.6, 0.7 * img_row_h)  # trend row; scales with image height, floored for readability
    top_pad, mid_gap, bot_pad = 0.95, 0.55, 0.6  # inches for suptitle / inter-row gap / x-axis labels
    fig_h = top_pad + img_row_h + mid_gap + plots_row_h + bot_pad

    fig = plt.figure(figsize=(fig_w, fig_h), facecolor=bg)
    outer = fig.add_gridspec(
        2,
        1,
        height_ratios=[img_row_h, plots_row_h],
        left=left,
        right=right,
        top=1 - top_pad / fig_h,
        bottom=bot_pad / fig_h,
        hspace=mid_gap / ((img_row_h + plots_row_h) / 2.0),
    )
    top = outer[0].subgridspec(1, 3, wspace=0.04)
    bottom = outer[1].subgridspec(1, 2, wspace=0.18)

    img_axes = [fig.add_subplot(top[0, k]) for k in range(3)]
    for ax, image, label in zip(img_axes, (gt, rendered, diff), ("GT", "rendered", diff_label)):
        ax.imshow(image)
        ax.set_box_aspect(img_aspect)  # lock the axes box to the image aspect -> fills with no stretch or bars
        ax.set_title(label, fontsize=12, pad=4, color=fg)
        ax.axis("off")

    t0 = timestamps[0]
    xs = [(t - t0) / 1e9 for t in timestamps]  # seconds since this camera's first frame
    plot_axes = [fig.add_subplot(bottom[0, 0]), fig.add_subplot(bottom[0, 1])]
    for ax, series, label in zip(plot_axes, (psnr_series, ssim_series), ("PSNR (dB)", "SSIM")):
        ax.set_facecolor(bg)
        ax.plot(xs, series, marker=".", linestyle="-", linewidth=1.4, markersize=7, color="deepskyblue")
        mean = sum(series) / len(series)
        ax.axhline(mean, linestyle="--", linewidth=1.1, color="gray", label=f"mean {mean:.3g}")
        if current_ts is not None and current_ts in timestamps:
            i = timestamps.index(current_ts)
            ax.axvline(xs[i], linestyle="-", linewidth=1.6, color="crimson", label="this frame")
            ax.plot(xs[i], series[i], marker="o", color="crimson", markersize=10, zorder=5)
        ax.set_xlabel("time (s) from first frame", color=fg, fontsize=11)
        ax.set_ylabel(label, color=fg, fontsize=11)
        ax.tick_params(colors=fg, labelsize=10)
        for spine in ax.spines.values():
            spine.set_color(fg)
        ax.grid(alpha=0.25, color=fg)
        legend = ax.legend(fontsize=10, loc="best", facecolor=bg, edgecolor=fg)
        for text in legend.get_texts():
            text.set_color(fg)
    fig.suptitle(f"{title}\n{caption}".strip(), fontsize=14, color=fg)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)
