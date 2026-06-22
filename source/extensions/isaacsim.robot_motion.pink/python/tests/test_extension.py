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

"""Smoke tests that the PINK motion extension loads and that its Pinocchio, Pink, public API, and transform utility imports are available."""

import omni.kit.test


class TestExtension(omni.kit.test.AsyncTestCase):
    """Verify that the PINK extension loads and core dependencies are importable."""

    async def setUp(self) -> None:
        """Prepare the Extension test fixture."""
        super().setUp()

    async def tearDown(self) -> None:
        """Clean up the Extension test fixture."""
        super().tearDown()

    async def test_pinocchio_importable(self) -> None:
        """Verify pinocchio is importable and can create a model."""
        import pinocchio as pin

        model = pin.Model()
        self.assertIsNotNone(model)
        self.assertEqual(model.nq, 0)

    async def test_pink_importable(self) -> None:
        """Verify pink is importable and Configuration class is available."""
        import pink

        self.assertIsNotNone(pink.Configuration)
        self.assertIsNotNone(pink.solve_ik)

    async def test_extension_public_api(self) -> None:
        """Verify all public symbols from isaacsim.robot_motion.pink are importable."""
        from isaacsim.robot_motion.pink import PinkIKController, PinkRobot, load_pink_robot, load_pink_supported_robot

        self.assertIsNotNone(PinkRobot)
        self.assertIsNotNone(PinkIKController)
        self.assertIsNotNone(load_pink_robot)
        self.assertIsNotNone(load_pink_supported_robot)

    async def test_transform_utils_importable(self) -> None:
        """Verify transform utilities are importable."""
        from isaacsim.robot_motion.pink.impl.utils import (
            isaac_sim_position_quaternion_to_se3,
            map_joint_positions_to_pinocchio,
            map_pinocchio_velocity_to_joint_state,
            se3_to_isaac_sim_position_quaternion,
        )

        self.assertIsNotNone(isaac_sim_position_quaternion_to_se3)
        self.assertIsNotNone(se3_to_isaac_sim_position_quaternion)
        self.assertIsNotNone(map_joint_positions_to_pinocchio)
        self.assertIsNotNone(map_pinocchio_velocity_to_joint_state)
