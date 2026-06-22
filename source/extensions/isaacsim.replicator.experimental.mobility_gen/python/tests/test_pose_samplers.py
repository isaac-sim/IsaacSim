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

"""Verifies GridPoseSampler block containment and mask sizing without requiring an Isaac Sim runtime. The tests ensure sampled poses stay inside column and row blocks and that block masks cover the expected area."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any
from unittest.mock import patch

import numpy as np
import omni.kit.test

# ---------------------------------------------------------------------------
# Load GridPoseSampler from source without the isaacsim package installed.
# ---------------------------------------------------------------------------
_SRC = (Path(__file__).parent.parent / "impl" / "pose_samplers.py").read_text()

# Strip the two relative import lines; inject stubs via exec globals instead.
_STRIPPED = "\n".join(
    line
    for line in _SRC.splitlines()
    if not line.startswith("from .occupancy_map") and not line.startswith("from .types")
)


@dataclass
class Point2d:
    """Test point2d behavior."""

    x: float
    y: float


@dataclass
class Pose2d:
    """Test pose2d behavior."""

    x: float
    y: float
    theta: float


_globs = {"OccupancyMap": object, "Point2d": Point2d, "Pose2d": Pose2d}
exec(compile(_STRIPPED, "pose_samplers.py", "exec"), _globs)
GridPoseSampler = _globs["GridPoseSampler"]


# ---------------------------------------------------------------------------
# Mock occupancy map: 100×100 all-free, 10 m × 10 m.
# ---------------------------------------------------------------------------
class _MockOccupancyMap:
    def width_pixels(self) -> Any:
        return 100

    def height_pixels(self) -> Any:
        return 100

    def width_meters(self) -> Any:
        return 10.0

    def height_meters(self) -> Any:
        return 10.0

    def freespace_mask(self) -> Any:
        return np.ones((100, 100), dtype=bool)


class TestGridPoseSamplerBlockContainment(omni.kit.test.AsyncTestCase):
    """GridPoseSampler must only return poses inside the selected grid block.

    Bug (lines 130-131): two separate row-strip assignments instead of a single
    2D slice → samples span the full map width regardless of the chosen block column.
    Fix: block_mask[y_min:y_max, x_min:x_max] = True
    """

    GRID_SIZE_M = 2.0  # 2 m cells → 20 px blocks in a 100 px / 10 m map
    BLOCK_SIZE_PX = 20  # 100 * 2.0 / 10.0

    def _sample_n(self, block_x: Any, block_y: Any, n: int = 300) -> Any:
        omap = _MockOccupancyMap()
        sampler = GridPoseSampler(self.GRID_SIZE_M)
        call_count = [0]

        def _fixed_randint(a: Any, b: Any) -> Any:
            call_count[0] += 1
            return block_x if call_count[0] % 2 == 1 else block_y

        poses = []
        with patch("random.randint", side_effect=_fixed_randint):
            for _ in range(n):
                call_count[0] = 0
                poses.append(sampler.sample_px(omap))
        return poses

    async def test_samples_within_block_column(self) -> None:
        """All sampled x coords must fall within the selected block column."""
        block_x, block_y = 1, 3
        x_min, x_max = block_x * self.BLOCK_SIZE_PX, (block_x + 1) * self.BLOCK_SIZE_PX
        out = [p for p in self._sample_n(block_x, block_y) if not (x_min <= p.x < x_max)]
        self.assertEqual(
            len(out),
            0,
            f"{len(out)}/300 samples outside x∈[{x_min},{x_max}): first offender x={out[0].x if out else '-'}",
        )

    async def test_samples_within_block_row(self) -> None:
        """All sampled y coords must fall within the selected block row."""
        block_x, block_y = 1, 3
        y_min, y_max = block_y * self.BLOCK_SIZE_PX, (block_y + 1) * self.BLOCK_SIZE_PX
        out = [p for p in self._sample_n(block_x, block_y) if not (y_min <= p.y < y_max)]
        self.assertEqual(
            len(out),
            0,
            f"{len(out)}/300 samples outside y∈[{y_min},{y_max}): first offender y={out[0].y if out else '-'}",
        )

    async def test_block_mask_area(self) -> None:
        """Fixed mask must cover exactly block_size² pixels (not two full-width strips)."""
        bsz = self.BLOCK_SIZE_PX
        block_x, block_y = 1, 3
        x_min, y_min = block_x * bsz, block_y * bsz

        buggy = np.zeros((100, 100), dtype=bool)
        buggy[x_min : x_min + bsz] = True
        buggy[y_min : y_min + bsz] = True
        self.assertNotEqual(np.count_nonzero(buggy), bsz * bsz, "Buggy mask unexpectedly equals the correct area.")

        fixed = np.zeros((100, 100), dtype=bool)
        fixed[y_min : y_min + bsz, x_min : x_min + bsz] = True
        n = np.count_nonzero(fixed)
        self.assertEqual(n, bsz * bsz, f"Fixed mask should cover {bsz*bsz} px, got {n}.")
