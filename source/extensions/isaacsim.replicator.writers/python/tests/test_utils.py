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

from unittest import mock

import numpy as np
import omni.kit.test
from isaacsim.replicator.writers.scripts.utils import invert_fisheye_polynomial, project_pinhole


def _evaluate_fisheye_polynomial(r: float, poly_coeffs: list[float]) -> float:
    a, b, c, d, e, f = poly_coeffs
    return a + b * r + c * r**2 + d * r**3 + e * r**4 + f * r**5


class TestPinholeUtils(omni.kit.test.AsyncTestCase):
    """Test pinhole utility behavior."""

    async def test_project_pinhole_returns_screen_center_when_homogeneous_divisor_is_zero(self) -> None:
        """Test that points on the projection plane do not crash during normalization."""
        camera_point = np.array([1.0, 0.0, 5.0, 0.0])
        camera_params = {
            "cameraProjection": np.identity(4).flatten(),
            "renderProductResolution": [1920, 1080],
        }

        self.assertEqual(project_pinhole(camera_point, camera_params), (960, 540))


class TestFisheyePolynomialUtils(omni.kit.test.AsyncTestCase):
    """Test fisheye polynomial utility behavior."""

    async def test_invert_fisheye_polynomial_logs_warning_when_not_converged(self) -> None:
        """Test that non-convergence emits a warning before returning the final iterate."""
        theta = 0.5
        poly_coeffs = [0.0, 0.001, 100.0, -200.0, 100.0, -10.0]

        with mock.patch("carb.log_warn") as log_warn:
            r = invert_fisheye_polynomial(theta, poly_coeffs, max_iterations=10)

        residual = abs(_evaluate_fisheye_polynomial(r, poly_coeffs) - theta)

        self.assertGreater(residual, 1e6)
        log_warn.assert_called_once()
        self.assertIn("Newton-Raphson did not converge", log_warn.call_args.args[0])

    async def test_invert_fisheye_polynomial_does_not_warn_when_converged(self) -> None:
        """Test that a convergent inversion does not emit a warning."""
        theta = 0.5
        poly_coeffs = [0.0, 1.0, 0.1, 0.0, 0.0, 0.0]

        with mock.patch("carb.log_warn") as log_warn:
            r = invert_fisheye_polynomial(theta, poly_coeffs, max_iterations=10)

        residual = abs(_evaluate_fisheye_polynomial(r, poly_coeffs) - theta)

        self.assertLess(residual, 1e-6)
        log_warn.assert_not_called()
