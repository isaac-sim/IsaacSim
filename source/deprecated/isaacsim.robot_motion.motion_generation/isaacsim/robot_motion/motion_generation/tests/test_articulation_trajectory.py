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

"""Tests for articulation trajectory bounds handling."""

from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

try:
    import omni.kit.test

    _TEST_CASE_BASE = omni.kit.test.AsyncTestCase
    _OMNI_KIT_TEST_AVAILABLE = True
except ImportError:
    _TEST_CASE_BASE = unittest.IsolatedAsyncioTestCase
    _OMNI_KIT_TEST_AVAILABLE = False


def _package(name: str) -> types.ModuleType:
    package = types.ModuleType(name)
    package.__path__ = []
    return package


def _load_articulation_trajectory_module() -> types.ModuleType:
    package_name = "_articulation_trajectory_test_package"
    module_name = f"{package_name}.articulation_trajectory"

    carb_module = types.ModuleType("carb")
    carb_module.log_error = mock.MagicMock()

    articulations_module = types.ModuleType("isaacsim.core.api.articulations")
    articulations_module.ArticulationSubset = type("ArticulationSubset", (), {})

    prims_module = types.ModuleType("isaacsim.core.prims")
    prims_module.SingleArticulation = type("SingleArticulation", (), {})

    types_module = types.ModuleType("isaacsim.core.utils.types")
    types_module.ArticulationAction = type("ArticulationAction", (), {})

    trajectory_module = types.ModuleType(f"{package_name}.trajectory")
    trajectory_module.Trajectory = type("Trajectory", (), {})

    stubs = {
        "carb": carb_module,
        "isaacsim": _package("isaacsim"),
        "isaacsim.core": _package("isaacsim.core"),
        "isaacsim.core.api": _package("isaacsim.core.api"),
        "isaacsim.core.api.articulations": articulations_module,
        "isaacsim.core.prims": prims_module,
        "isaacsim.core.utils": _package("isaacsim.core.utils"),
        "isaacsim.core.utils.types": types_module,
        package_name: _package(package_name),
        f"{package_name}.trajectory": trajectory_module,
    }

    source_path = Path(__file__).resolve().parents[1] / "articulation_trajectory.py"
    spec = importlib.util.spec_from_file_location(module_name, source_path)
    module = importlib.util.module_from_spec(spec)

    with mock.patch.dict(sys.modules, stubs):
        sys.modules[module_name] = module
        spec.loader.exec_module(module)

    return module


class TestArticulationTrajectory(_TEST_CASE_BASE):
    """Test articulation trajectory behavior without requiring Isaac runtime modules."""

    async def _setup_case(self) -> None:
        self.module = _load_articulation_trajectory_module()
        self.articulation_trajectory = object.__new__(self.module.ArticulationTrajectory)

        self.trajectory = mock.MagicMock()
        self.trajectory.start_time = 0.0
        self.trajectory.end_time = 2.5
        self.trajectory.get_joint_targets.return_value = ([0.0], [0.0])
        self.articulation_trajectory._trajectory = self.trajectory

        self.active_joints_view = mock.MagicMock()
        self.active_joints_view.make_articulation_action.return_value = object()
        self.articulation_trajectory._active_joints_view = self.active_joints_view

    if _OMNI_KIT_TEST_AVAILABLE:

        async def setUp(self) -> None:
            """Set up a mocked articulation trajectory test case."""
            await self._setup_case()

    else:

        async def asyncSetUp(self) -> None:
            """Set up a mocked articulation trajectory test case."""
            await self._setup_case()

    async def test_get_action_at_time_raises_for_times_outside_trajectory_bounds(self) -> None:
        """Verify out-of-bounds timestamps raise before querying joint targets."""
        cases = (
            (-0.5, "before the start time"),
            (10.0, "after the end time"),
        )

        for invalid_time, expected_message in cases:
            with self.subTest(invalid_time=invalid_time):
                self.trajectory.get_joint_targets.reset_mock()
                self.active_joints_view.make_articulation_action.reset_mock()
                self.module.carb.log_error.reset_mock()

                with self.assertRaisesRegex(ValueError, expected_message):
                    self.articulation_trajectory.get_action_at_time(invalid_time)

                self.module.carb.log_error.assert_called_once()
                self.trajectory.get_joint_targets.assert_not_called()
                self.active_joints_view.make_articulation_action.assert_not_called()


if __name__ == "__main__":
    unittest.main()
