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

"""Kit-free smoke test for the ``isaacsim.robot_motion.experimental.motion_generation`` library.

This test intentionally imports **only** the public library and the standard library. It runs outside of Omniverse
Kit (no ``omni.kit.app``, no ``SimulationApp``) to prove that the relocated motion generation package is importable
in a pure Kit-free environment, that its frozen public API is unchanged, and that no ``carb`` / ``omni.*`` module is
pulled in as a side effect of importing it. The heavier behavioral tests continue to run under the carrier extension
in Kit.
"""

from __future__ import annotations

import subprocess
import sys

import pytest

# The public API is frozen: this list must stay identical to the Kit extension it was relocated from.
_FROZEN_PUBLIC_API = [
    "BaseController",
    "SelectableController",
    "CombinedController",
    "ChainedController",
    "ObstacleConfiguration",
    "ObstacleRepresentation",
    "ObstacleStrategy",
    "Path",
    "SceneQuery",
    "TrackableApi",
    "Trajectory",
    "TrajectoryFollower",
    "JointState",
    "RobotState",
    "RootState",
    "SpatialState",
    "combine_robot_states",
    "WorldBinding",
    "WorldInterface",
]


def test_public_import_path_and_frozen_all() -> None:
    """The public import path resolves and exposes the frozen ``__all__`` names in any order."""
    import isaacsim.robot_motion.experimental.motion_generation as motion_generation

    assert set(motion_generation.__all__) == set(_FROZEN_PUBLIC_API)
    for symbol in _FROZEN_PUBLIC_API:
        assert hasattr(motion_generation, symbol), f"missing public symbol: {symbol}"


def test_import_is_kit_free() -> None:
    """Importing the library in a fresh interpreter must not import ``carb`` or any ``omni.*`` module.

    Run in a subprocess so this assertion is not polluted by modules another test (or the test runner) already imported
    into this interpreter.
    """
    program = (
        "import sys\n"
        "import isaacsim.robot_motion.experimental.motion_generation  # noqa: F401\n"
        "leaked = sorted(m for m in sys.modules if m == 'carb' or m.split('.')[0] == 'omni')\n"
        "assert not leaked, 'Kit modules leaked into a Kit-free import: %r' % leaked\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        "Kit-free import check failed.\n" f"stdout:\n{result.stdout}\n" f"stderr:\n{result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
