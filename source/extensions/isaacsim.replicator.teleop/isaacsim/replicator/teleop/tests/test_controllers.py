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

"""Test controller instantiation."""

from __future__ import annotations

import omni.kit.app
import omni.kit.test
from isaacsim.replicator.teleop import (
    FloatingRigidBodyController,
    GraspController,
    LocomotionController,
    RobotIKController,
)


class TestControllerInstantiation(omni.kit.test.AsyncTestCase):
    """Verify all controllers can be created without a stage or VR."""

    async def setUp(self) -> None:
        """Set up test environment."""
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test environment."""
        await omni.kit.app.get_app().next_update_async()

    async def test_floating_controller(self) -> None:
        """Verify FloatingRigidBodyController can be created without a stage."""
        ctrl = FloatingRigidBodyController()
        self.assertIsNotNone(ctrl)

    async def test_ik_controller(self) -> None:
        """Verify RobotIKController can be created without a stage."""
        ctrl = RobotIKController()
        self.assertIsNotNone(ctrl)

    async def test_grasp_controller(self) -> None:
        """Verify GraspController can be created without a stage."""
        ctrl = GraspController()
        self.assertIsNotNone(ctrl)

    async def test_locomotion_controller(self) -> None:
        """Verify LocomotionController can be created without a stage."""
        ctrl = LocomotionController()
        self.assertIsNotNone(ctrl)
        self.assertAlmostEqual(ctrl.linear_step, LocomotionController.DEFAULT_LINEAR_STEP)
        self.assertAlmostEqual(ctrl.angular_step, LocomotionController.DEFAULT_ANGULAR_STEP)
