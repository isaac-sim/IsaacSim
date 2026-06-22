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

"""Correctness checks for the torch-based PSNR/SSIM metrics.

Builds a fixed synthetic image pair (deterministic gradients + seeded noise) and asserts the
metrics against known golden values, verified against skimage's Wang-Gaussian reference.
"""

from __future__ import annotations

import numpy as np
import omni.kit.test
from isaacsim.replicator.nurec_utils.metrics.psnr_ssim import psnr, score, ssim

# Golden values for (gt, noisy), verified against skimage's Wang-Gaussian SSIM / PSNR.
GOLDEN_PSNR_NOISY = 24.7727
GOLDEN_SSIM_NOISY = 0.25092
PSNR_TOL = 1e-2
SSIM_TOL = 1e-3


class TestTorchMetrics(omni.kit.test.AsyncTestCase):
    """Torch PSNR/SSIM against identical-frame invariants and known golden values."""

    async def setUp(self) -> None:
        """Build a fixed synthetic GT image and a seeded-noise copy (both deterministic)."""
        yy, xx = np.mgrid[0:256, 0:320]
        base = (
            127
            + 80 * np.sin(xx / 30.0)[..., None] * np.array([1.0, 0.6, 0.3])
            + 40 * np.cos(yy / 25.0)[..., None] * np.array([0.2, 1.0, 0.5])
        )
        self.gt = np.clip(base, 0, 255).astype(np.uint8)
        rng = np.random.default_rng(0)
        self.noisy = np.clip(self.gt.astype(np.int16) + rng.integers(-25, 26, self.gt.shape), 0, 255).astype(np.uint8)

    async def test_identical_frame_invariants(self) -> None:
        """An identical frame scores the PSNR cap (100 dB) and SSIM 1.0."""
        self.assertAlmostEqual(psnr(self.gt, self.gt), 100.0, places=5)
        self.assertAlmostEqual(ssim(self.gt, self.gt), 1.0, places=4)

    async def test_noisy_matches_golden(self) -> None:
        """The noisy pair reproduces the verified golden PSNR/SSIM."""
        self.assertAlmostEqual(psnr(self.gt, self.noisy), GOLDEN_PSNR_NOISY, delta=PSNR_TOL)
        self.assertAlmostEqual(ssim(self.gt, self.noisy), GOLDEN_SSIM_NOISY, delta=SSIM_TOL)

    async def test_score_keys(self) -> None:
        """score() returns exactly the PSNR / SSIM / mean-abs-diff keys."""
        self.assertEqual(set(score(self.gt, self.noisy)), {"psnr", "ssim", "mean_abs_diff"})
