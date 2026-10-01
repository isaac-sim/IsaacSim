# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for articulation-controller robot and index input refresh."""

from types import MethodType, SimpleNamespace

import numpy as np
import omni.kit.test
from isaacsim.core.nodes.ogn.python.nodes.OgnIsaacArticulationController import (
    OgnIsaacArticulationController,
    OgnIsaacArticulationControllerInternalState,
)


class _FakeArticulation:
    def __init__(self) -> None:
        self.position_targets = []

    def set_dof_position_targets(self, values, dof_indices=None) -> None:
        self.position_targets.append((np.asarray(values), np.asarray(dof_indices)))

    def set_dof_velocity_targets(self, values, dof_indices=None) -> None:
        pass

    def set_dof_efforts(self, values, dof_indices=None) -> None:
        pass


class _FakeDb:
    def __init__(self, state, *, robot_path: str, joint_indices=None, position_command=None) -> None:
        self.per_instance_state = state
        self.inputs = SimpleNamespace(
            robotPath=robot_path,
            targetPrim=[],
            jointNames=[],
            jointIndices=[] if joint_indices is None else joint_indices,
            positionCommand=[] if position_command is None else position_command,
            velocityCommand=[],
            effortCommand=[],
        )
        self.errors = []

    def log_error(self, message) -> None:
        self.errors.append(str(message))


class TestArticulationControllerInputRefresh(omni.kit.test.AsyncTestCase):
    """Verify controller target changes and explicit DOF zero are honored."""

    @staticmethod
    def _state() -> OgnIsaacArticulationControllerInternalState:
        state = OgnIsaacArticulationControllerInternalState.__new__(OgnIsaacArticulationControllerInternalState)
        state.initialized = True
        state.prim_path = "/RobotA"
        state.articulation = _FakeArticulation()
        state.joint_names = None
        state.joint_indices = None
        state.joint_picked = False
        state.command_error_message = None
        state.node = None
        return state

    async def test_explicit_zero_joint_index_remains_selected(self) -> None:
        """jointIndices=[0] must not fall through to the all-DOFs path."""
        state = self._state()
        state.prim_path = "/Robot"
        db = _FakeDb(state, robot_path="/Robot", joint_indices=[0], position_command=[0.25])

        self.assertTrue(OgnIsaacArticulationController.compute(db))
        self.assertEqual(db.errors, [])
        self.assertEqual(len(state.articulation.position_targets), 1)
        values, indices = state.articulation.position_targets[0]
        self.assertTrue(np.array_equal(values, np.array([0.25])))
        self.assertTrue(np.array_equal(indices, np.array([0])))

    async def test_robot_path_change_reinitializes_controller(self) -> None:
        """Changing robotPath should rebuild the articulation handle for the new robot."""
        state = self._state()
        state.joint_picked = True
        initialized_paths = []

        def initialize_controller(inner_state) -> None:
            initialized_paths.append(inner_state.prim_path)
            inner_state.articulation = _FakeArticulation()
            inner_state.initialized = True

        state.initialize_controller = MethodType(initialize_controller, state)
        db = _FakeDb(state, robot_path="/RobotB")

        self.assertTrue(OgnIsaacArticulationController.compute(db))
        self.assertEqual(db.errors, [])
        self.assertEqual(initialized_paths, ["/RobotB"])
        self.assertEqual(state.prim_path, "/RobotB")
        self.assertTrue(state.joint_picked)
