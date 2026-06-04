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

"""Tests for Warp rotation utility functions."""

from unittest import mock

import isaacsim.core.utils.warp as warp_utils
import numpy as np
import omni.kit.test
import warp as wp


class TestWarpRotationUtils(omni.kit.test.AsyncTestCase):
    """Test cases for Warp rotation utilities."""

    async def test_quaternion_format_conversions_keep_cpu_data_on_cpu(self) -> None:
        """Test quaternion format conversions do not force CPU arrays through CUDA."""
        xyzw = wp.array([1.0, 2.0, 3.0, 4.0], dtype=wp.float32, device="cpu")

        with mock.patch.object(warp_utils, "move_data", side_effect=AssertionError("unexpected device migration")):
            wxyz = warp_utils.xyzw2wxyz(xyzw)
            xyzw_round_trip = warp_utils.wxyz2xyzw(wxyz)

        self.assertEqual(str(wxyz.device), "cpu")
        self.assertEqual(str(xyzw_round_trip.device), "cpu")
        np.testing.assert_allclose(wxyz.numpy(), np.array([4.0, 1.0, 2.0, 3.0], dtype=np.float32))
        np.testing.assert_allclose(xyzw_round_trip.numpy(), xyzw.numpy())
