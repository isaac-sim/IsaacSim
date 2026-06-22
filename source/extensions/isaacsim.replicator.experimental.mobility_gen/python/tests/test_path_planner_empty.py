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

"""Verifies path planner handling when breadth-first search finds no reachable endpoints. The tests ensure empty visited sets raise or report friendly errors while nonempty inputs still produce valid samples."""

from pathlib import Path

import numpy as np
import omni.kit.test

# ---------------------------------------------------------------------------
# Load GeneratePathsOutput from source — strip the C++ binding import since
# we only need the pure-Python dataclass and its methods.
# ---------------------------------------------------------------------------
_SRC = (Path(__file__).parent.parent / "impl" / "path_planner.py").read_text()

_STRIPPED = "\n".join(
    line for line in _SRC.splitlines() if "from ..bindings" not in line and "_path_planner.unroll_path" not in line
)

_globs = {}
exec(compile(_STRIPPED, "path_planner.py", "exec"), _globs)
GeneratePathsOutput = _globs["GeneratePathsOutput"]


def _make_output(visited: np.ndarray) -> "GeneratePathsOutput":
    """Construct a GeneratePathsOutput with the given visited grid.

    Args:
        visited: Visited-cell grid.

    Returns:
        A path planner output object using ``visited`` and empty predecessor arrays.
    """
    shape = visited.shape
    return GeneratePathsOutput(
        visited=visited,
        distance_to_start=np.zeros(shape, dtype=np.int64),
        prev_i=np.full(shape, -1, dtype=np.int64),
        prev_j=np.full(shape, -1, dtype=np.int64),
    )


class TestSampleRandomEndPointEmptyBFS(omni.kit.test.AsyncTestCase):
    """sample_random_end_point must not crash when BFS finds no reachable cells.

    Bug: random.randint(0, len(i) - 1) with len(i)==0 gives randint(0, -1)
    which raises ValueError.  Fix: raise a descriptive error (or return None)
    before calling randint.
    """

    async def test_empty_visited_raises_value_error(self) -> None:
        """With no visited cells, sample_random_end_point currently raises ValueError."""
        output = _make_output(np.zeros((5, 5), dtype=np.int64))
        with self.assertRaises((ValueError, Exception)):
            output.sample_random_end_point()

    async def test_nonempty_visited_succeeds(self) -> None:
        """With visited cells present, sample_random_end_point must return a valid index."""
        visited = np.zeros((5, 5), dtype=np.int64)
        visited[2, 3] = 1
        visited[1, 1] = 1
        output = _make_output(visited)
        row, col = output.sample_random_end_point()
        self.assertIn((row, col), [(2, 3), (1, 1)])

    async def test_empty_visited_returns_friendly_error(self) -> None:
        """After the fix, empty BFS should raise a clear RuntimeError, not ValueError."""
        output = _make_output(np.zeros((5, 5), dtype=np.int64))
        with self.assertRaises(RuntimeError):
            output.sample_random_end_point()
