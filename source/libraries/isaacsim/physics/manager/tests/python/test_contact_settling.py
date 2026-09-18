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

"""Verify per-sensor settling behavior used by batched contact scenarios."""

from __future__ import annotations

import numpy as np
from common.contacts import _PerSensorSettlingTracker


class TestPerSensorSettlingTracker:
    """Check that spatially isolated sensors can settle on different steps."""

    def test_latches_distinct_settling_windows(self) -> None:
        """Accept sensors whose three-step stable windows do not overlap."""
        tracker = _PerSensorSettlingTracker(2)

        tracker.update(np.array([True, False]))
        tracker.update(np.array([True, True]))
        tracker.update(np.array([True, True]))
        assert not tracker.is_complete

        tracker.update(np.array([False, True]))
        assert tracker.is_complete

    def test_reports_only_sensors_that_never_settled(self) -> None:
        """Include unsettled indices, longest streaks, and final values in diagnostics."""
        tracker = _PerSensorSettlingTracker(3)
        tracker.update(np.array([True, True, False]))
        tracker.update(np.array([True, True, False]))
        tracker.update(np.array([True, False, False]))

        values = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
        assert tracker.format_unsettled(values) == (
            "unsettled sensor indices=[1, 2], longest streaks=[2, 0], final values=[[3.0, 4.0], [5.0, 6.0]]"
        )
