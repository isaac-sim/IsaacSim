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

"""Compatibility checks for public IK constructor signatures."""

import inspect
from unittest.mock import Mock

import numpy as np
import omni.kit.test
from isaacsim.replicator.teleop.controllers.lm_ik import LMIKController
from isaacsim.replicator.teleop.controllers.pink_ik import PinkIKController
from isaacsim.replicator.teleop.controllers.position_ik import PositionBasedIKController
from isaacsim.replicator.teleop.controllers.robot_ik import IKValidationResult
from isaacsim.replicator.teleop.controllers.velocity_ik import VelocityBasedIKController


class TestIKLegacyAPI(omni.kit.test.AsyncTestCase):
    """Keep legacy argument order while permitting non-contiguous arm DOFs."""

    async def test_count_argument(self) -> None:
        """Existing positional and keyword callers retain first-N semantics."""
        for cls in (PositionBasedIKController, VelocityBasedIKController, LMIKController):
            for controller in (cls(Mock(), Mock(), 0, 3), cls(Mock(), Mock(), 0, num_arm_dofs=3)):
                np.testing.assert_array_equal(controller._arm_dof_indices, [0, 1, 2])

    async def test_explicit_partition(self) -> None:
        """New optional indices preserve the specified order and reject mismatches."""
        for cls in (PositionBasedIKController, VelocityBasedIKController, LMIKController):
            controller = cls(Mock(), Mock(), 0, 3, arm_dof_indices=[0, 2, 4])
            np.testing.assert_array_equal(controller._arm_dof_indices, [0, 2, 4])
            with self.assertRaises(ValueError):
                cls(Mock(), Mock(), 0, 3, arm_dof_indices=[0, 2])

    async def test_pink_signature(self) -> None:
        """PINK's existing arguments bind without requiring a backend or a stage."""
        signature = inspect.signature(PinkIKController)
        bound = signature.bind(Mock(), Mock(), 0, 6, "tool", "/World/Robot")
        self.assertEqual(bound.arguments["num_arm_dofs"], 6)
        self.assertEqual(bound.arguments["ee_link_name"], "tool")
        self.assertEqual(signature.parameters["arm_dof_indices"].kind, inspect.Parameter.KEYWORD_ONLY)

    async def test_validation_result_argument_order(self) -> None:
        """Added diagnostics do not shift existing positional dataclass fields."""
        result = IKValidationResult(True, "ok", "/Robot", ["base"], ["joint"], 1, 1)
        self.assertEqual(result.link_names, ["base"])
        self.assertEqual(result.dof_names, ["joint"])
        self.assertEqual(result.arm_dofs, 1)

    async def test_lm_limits_follow_partition(self) -> None:
        """The lower/upper tuple is indexed by arm DOF, not by a prefix."""
        lower, upper = Mock(), Mock()
        lower.numpy.return_value = np.array([[-1.0, -2.0, -3.0]])
        upper.numpy.return_value = np.array([[1.0, 2.0, 3.0]])
        robot = Mock()
        robot.get_dof_limits.return_value = (lower, upper)
        controller = LMIKController(robot, Mock(), 0, 2, arm_dof_indices=[2, 0])
        np.testing.assert_array_equal(controller._lo, [-3.0, -1.0])
        np.testing.assert_array_equal(controller._hi, [3.0, 1.0])
