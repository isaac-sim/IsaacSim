# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for pycoverage compatibility patches applied to NumPy and SciPy functions."""

from __future__ import annotations

import numpy as np
import omni.kit.test


class TestPyCoveragePatches(omni.kit.test.AsyncTestCase):
    """Tests verifying that pycoverage compatibility patches work correctly.

    These patches are applied at import time by isaacsim.test.utils.__init__
    when ``/exts/omni.kit.test/pyCoverageEnabled`` is set. The tests below
    exercise the patched code paths to ensure they produce correct results
    regardless of whether coverage is active.
    """

    # ------------------------------------------------------------------
    # scipy.stats import (CopyMode patch)
    # ------------------------------------------------------------------
    async def test_import_scipy_stats(self) -> None:
        """Importing scipy.stats must not raise (covers _CopyMode patch)."""
        import scipy.stats  # noqa: F401

    async def test_scipy_stats_truncnorm_accessible(self) -> None:
        """scipy.stats.truncnorm must be accessible after import."""
        import scipy.stats as stats

        self.assertTrue(callable(stats.truncnorm), "truncnorm should be a callable distribution")

    # ------------------------------------------------------------------
    # numpy _CopyMode np.array wrapper patch
    # ------------------------------------------------------------------
    async def test_numpy_array_with_copymode(self) -> None:
        """np.array must accept _CopyMode values in the copy parameter."""
        import numpy._globals as npg

        _CopyMode = getattr(npg, "_CopyMode", None)
        if _CopyMode is None:
            self.skipTest("numpy._globals._CopyMode not available")

        # _CopyMode.IF_NEEDED should behave like copy=None (copy if needed).
        # Without the patch this raises ValueError.
        arr = np.array(1.0, copy=_CopyMode.IF_NEEDED, dtype=np.float64)
        self.assertEqual(arr, 1.0)

        # _CopyMode.ALWAYS should behave like copy=True (always copy).
        src = np.array([1, 2, 3])
        arr = np.array(src, copy=_CopyMode.ALWAYS)
        np.testing.assert_array_equal(arr, src)
        self.assertFalse(np.shares_memory(arr, src))

        # _CopyMode.NEVER should behave like copy=False (no copy).
        arr = np.array(src, copy=_CopyMode.NEVER)
        np.testing.assert_array_equal(arr, src)

    # ------------------------------------------------------------------
    # numpy amax / amin / sum / prod patches
    # ------------------------------------------------------------------
    async def test_numpy_amax(self) -> None:
        """np.amax must return correct results after patching."""
        a = np.array([1, 3, 2])
        self.assertEqual(np.amax(a), 3)

        b = np.array([[1, 5], [3, 2]])
        np.testing.assert_array_equal(np.amax(b, axis=1), np.array([5, 3]))

    async def test_numpy_amin(self) -> None:
        """np.amin must return correct results after patching."""
        a = np.array([4, 1, 7])
        self.assertEqual(np.amin(a), 1)

        b = np.array([[4, 1], [7, 2]])
        np.testing.assert_array_equal(np.amin(b, axis=1), np.array([1, 2]))

    async def test_numpy_sum(self) -> None:
        """np.sum must return correct results after patching."""
        a = np.array([1, 2, 3])
        self.assertEqual(np.sum(a), 6)

        b = np.array([[1, 2], [3, 4]])
        np.testing.assert_array_equal(np.sum(b, axis=1), np.array([3, 7]))

    async def test_numpy_prod(self) -> None:
        """np.prod must return correct results after patching."""
        a = np.array([2, 3, 4])
        self.assertEqual(np.prod(a), 24)

        b = np.array([[2, 3], [4, 5]])
        np.testing.assert_array_equal(np.prod(b, axis=1), np.array([6, 20]))
