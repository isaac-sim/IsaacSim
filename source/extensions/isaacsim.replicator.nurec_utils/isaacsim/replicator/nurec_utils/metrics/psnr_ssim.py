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

"""PSNR / SSIM / image-diff helpers for scoring rendered frames against ground truth.

PSNR and SSIM follow the torchmetrics conventions: inputs normalized to ``[0, 1]``; PSNR with
``data_range=1.0`` and identical frames capped at 100 dB; SSIM with an 11x11 Gaussian window
(``sigma=1.5``).

The torch-backed functions must be called after a ``SimulationApp`` exists; ``mean_abs_diff``
and ``image_diff`` are pure numpy and work anywhere.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from PIL import Image

if TYPE_CHECKING:
    import torch

# torchmetrics/NuRec SSIM defaults.
_SSIM_KERNEL_SIZE = 11
_SSIM_SIGMA = 1.5
_SSIM_K1 = 0.01
_SSIM_K2 = 0.03
# data_range for [0, 1]-normalized images.
_DATA_RANGE = 1.0
# Finite cap for PSNR of identical frames (MSE -> 0): 10*log10(1 / 1e-10) = 100 dB.
_PSNR_MAX_DB = 100.0


def load_rgb(path: str) -> np.ndarray:
    """Load an image as HxWx3 uint8 (drops alpha).

    Args:
        path: Path to the image file.

    Returns:
        The image as an HxWx3 uint8 array.
    """
    return np.asarray(Image.open(path).convert("RGB"))


def match_shape(rendered: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """Resize `rendered` to `gt`'s HxW if they differ (RP resolution vs GT resolution).

    Args:
        rendered: The rendered image to resize.
        gt: The ground-truth image whose HxW is the target.

    Returns:
        `rendered` unchanged when shapes already match, else a bilinearly resized copy.
    """
    if rendered.shape[:2] == gt.shape[:2]:
        return rendered
    h, w = gt.shape[:2]
    return np.asarray(Image.fromarray(rendered).resize((w, h), Image.Resampling.BILINEAR))


def score(gt: np.ndarray, rendered: np.ndarray) -> dict:
    """Compute PSNR / SSIM / mean-abs-diff for one (gt, rendered) pair.

    Args:
        gt: The ground-truth image (HxWx3 uint8).
        rendered: The rendered image (HxWx3 uint8), same shape as `gt`.

    Returns:
        Dict with "psnr", "ssim", and "mean_abs_diff" keys.
    """
    return {
        "psnr": psnr(gt, rendered),
        "ssim": ssim(gt, rendered),
        "mean_abs_diff": mean_abs_diff(gt, rendered),
    }


def _device() -> torch.device:
    """Return the torch device to score on: CUDA when available (under a booted app), else CPU.

    Returns:
        The selected torch device.
    """
    import torch

    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def _to_tensor(img: np.ndarray) -> torch.Tensor:
    """Convert a uint8 HxWx3 (or HxW) numpy image to a float32 [1, C, H, W] tensor in [0, 1].

    Args:
        img: The uint8 image array; grayscale (HxW) gains a channel axis.

    Returns:
        The image as a float32 tensor of shape [1, C, H, W] on the scoring device.
    """
    import torch

    arr = np.asarray(img)
    if arr.ndim == 2:  # grayscale -> add channel
        arr = arr[..., None]
    # Writable, contiguous buffer for `torch.from_numpy`.
    buf = np.ascontiguousarray(arr)
    if buf is arr:
        buf = buf.copy()
    t = torch.from_numpy(buf).to(_device(), dtype=torch.float32) / 255.0
    return t.permute(2, 0, 1).unsqueeze(0)  # HWC -> [1, C, H, W]


def psnr(gt: np.ndarray, rendered: np.ndarray) -> float:
    """Compute peak signal-to-noise ratio in dB on [0, 1]-normalized inputs.

    Equivalent to torchmetrics ``PeakSignalNoiseRatio(data_range=1.0)``. Identical frames
    return the 100 dB cap rather than positive infinity.

    Args:
        gt: The ground-truth image (HxWx3 uint8).
        rendered: The rendered image (HxWx3 uint8), same shape as `gt`.

    Returns:
        The PSNR value in decibels (higher is better).
    """
    import torch

    a, b = _to_tensor(gt), _to_tensor(rendered)
    mse = torch.mean((a - b) ** 2)
    if mse.item() == 0.0:
        return _PSNR_MAX_DB
    return float((10.0 * torch.log10((_DATA_RANGE**2) / mse)).item())


def _gaussian_window(
    channels: int,
    kernel_size: int,
    sigma: float,
    device: torch.device,
    dtype: torch.dtype,
) -> torch.Tensor:
    """Build a depthwise (per-channel) 2-D Gaussian kernel that sums to 1.

    Args:
        channels: Number of input channels (the kernel is replicated per channel).
        kernel_size: Side length of the square kernel.
        sigma: Standard deviation of the Gaussian.
        device: Device to allocate the kernel on.
        dtype: Floating-point dtype of the kernel.

    Returns:
        The kernel of shape [channels, 1, kernel_size, kernel_size].
    """
    import torch

    coords = torch.arange(kernel_size, device=device, dtype=dtype) - (kernel_size - 1) / 2.0
    g1d = torch.exp(-(coords**2) / (2.0 * sigma**2))
    g1d = g1d / g1d.sum()
    g2d = g1d[:, None] * g1d[None, :]
    return g2d.expand(channels, 1, kernel_size, kernel_size).contiguous()


def ssim(gt: np.ndarray, rendered: np.ndarray) -> float:
    """Compute structural similarity (1.0 = identical) on [0, 1]-normalized inputs.

    Matches torchmetrics ``StructuralSimilarityIndexMeasure(data_range=1.0, kernel_size=11)``
    (Gaussian window sigma=1.5, k1=0.01, k2=0.03, population covariance, valid conv).

    Args:
        gt: The ground-truth image (HxWx3 uint8).
        rendered: The rendered image (HxWx3 uint8), same shape as `gt`.

    Returns:
        The mean SSIM over the image, in [-1, 1] (higher is better).
    """
    import torch
    import torch.nn.functional as F

    a, b = _to_tensor(gt), _to_tensor(rendered)
    channels = a.shape[1]
    kernel = _gaussian_window(channels, _SSIM_KERNEL_SIZE, _SSIM_SIGMA, a.device, a.dtype)

    c1 = (_SSIM_K1 * _DATA_RANGE) ** 2
    c2 = (_SSIM_K2 * _DATA_RANGE) ** 2

    # One grouped conv over [a, b, a*a, b*b, a*b] (valid convolution, like torchmetrics).
    stacked = torch.cat([a, b, a * a, b * b, a * b], dim=0)
    out = F.conv2d(stacked, kernel, groups=channels)
    mu_a, mu_b, a_sq, b_sq, ab = out.split(a.shape[0], dim=0)

    mu_a_sq, mu_b_sq, mu_ab = mu_a**2, mu_b**2, mu_a * mu_b
    sigma_a_sq = a_sq - mu_a_sq
    sigma_b_sq = b_sq - mu_b_sq
    sigma_ab = ab - mu_ab

    ssim_map = ((2 * mu_ab + c1) * (2 * sigma_ab + c2)) / ((mu_a_sq + mu_b_sq + c1) * (sigma_a_sq + sigma_b_sq + c2))
    return float(ssim_map.mean().item())


def mean_abs_diff(gt: np.ndarray, rendered: np.ndarray) -> float:
    """Compute the mean absolute per-pixel difference in 8-bit levels (pure numpy).

    Args:
        gt: The ground-truth image (HxWx3 uint8).
        rendered: The rendered image (HxWx3 uint8), same shape as `gt`.

    Returns:
        The mean absolute difference in 8-bit levels (lower is better).
    """
    return float(np.abs(gt.astype(np.float32) - rendered.astype(np.float32)).mean())


def image_diff(gt: np.ndarray, rendered: np.ndarray, amplify: float = 4.0) -> np.ndarray:
    """Compute a per-pixel absolute difference image, amplified for visibility (pure numpy).

    Args:
        gt: The ground-truth image (HxWx3 uint8).
        rendered: The rendered image (HxWx3 uint8), same shape as `gt`.
        amplify: Factor the absolute difference is multiplied by before clamping.

    Returns:
        The amplified difference as a uint8 array of the same shape as the inputs.
    """
    diff = np.abs(gt.astype(np.float32) - rendered.astype(np.float32))
    return np.clip(diff * amplify, 0, 255).astype(np.uint8)
