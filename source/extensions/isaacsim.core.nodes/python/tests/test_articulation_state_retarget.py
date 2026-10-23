# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for IsaacArticulationState robot retargeting."""

from types import SimpleNamespace

import numpy as np
import omni.kit.test
from isaacsim.core.nodes.ogn.python.nodes.OgnIsaacArticulationState import OgnIsaacArticulationState


class _FakeState:
    def __init__(self) -> None:
        self.initialized = True
        self.robot_prim = "/RobotA"
        self.dof_names = []
        self.dof_indices = np.array([], dtype=np.int64)
        self.initialized_paths = []
        self.pick_calls = 0

    def initialize_articulation(self) -> None:
        self.initialized_paths.append(self.robot_prim)
        self.dof_names = None
        self.dof_indices = None
        self.initialized = True

    def pick_dofs(self, dof_names, dof_indices) -> None:
        self.pick_calls += 1
        self.dof_names = dof_names
        self.dof_indices = dof_indices

    def get_dof_names(self):
        return []

    def get_articulation_state(self):
        empty = np.array([], dtype=np.float32)
        return empty, empty, empty, empty, empty


class TestArticulationStateRetarget(omni.kit.test.AsyncTestCase):
    """Verify changing the robot input refreshes the cached articulation state."""

    async def test_robot_path_change_reinitializes_articulation_and_selection(self) -> None:
        """A new robotPath should rebuild the handle and re-resolve DOF selection."""
        state = _FakeState()
        db = SimpleNamespace(
            per_instance_state=state,
            inputs=SimpleNamespace(robotPath="/RobotB", targetPrim=[], jointNames=[], jointIndices=[]),
            outputs=SimpleNamespace(),
            log_error=lambda _message: None,
            log_warn=lambda _message: None,
        )

        self.assertTrue(OgnIsaacArticulationState.compute(db))
        self.assertEqual(state.robot_prim, "/RobotB")
        self.assertEqual(state.initialized_paths, ["/RobotB"])
        self.assertEqual(state.pick_calls, 1)
