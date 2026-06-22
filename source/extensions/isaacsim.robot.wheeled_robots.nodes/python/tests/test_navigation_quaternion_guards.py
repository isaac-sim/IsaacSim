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

"""Regression coverage for navigation OmniGraph nodes when goal or pose inputs contain default zero quaternions. The tests verify those inputs are treated as zero yaw instead of producing invalid rotations."""

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test


class TestNavigationQuaternionGuards(ogts.OmniGraphTestCase):
    """Validate navigation nodes tolerate default invalid quaternion inputs."""

    async def setUp(self) -> None:
        """Prepare the Navigation Quaternion Guards test fixture."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Clean up the Navigation Quaternion Guards test fixture."""
        app_utils.stop()
        await app_utils.update_app_async()
        await omni.kit.stage_templates.new_stage_async()

    async def test_check_goal_uses_zero_yaw_for_default_zero_quaternion(self) -> None:
        """CheckGoal2D should not fail when its orientation input remains at the OGN zero default."""
        graph, [check_goal_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("CheckGoal2D", "isaacsim.robot.wheeled_robots.CheckGoal2D"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CheckGoal2D.inputs:currentPosition", [0.0, 0.0, 0.0]),
                    ("CheckGoal2D.inputs:target", [0.0, 0.0, 0.0]),
                    ("CheckGoal2D.inputs:targetChanged", True),
                    ("CheckGoal2D.inputs:thresholds", [0.1, 0.1]),
                ],
            },
        )

        await og.Controller.evaluate(graph)

        reached_goal = og.Controller(og.Controller.attribute("outputs:reachedGoal", check_goal_node)).get()
        self.assertEqual(list(reached_goal), [True, True])

    async def test_quintic_path_planner_uses_zero_yaw_for_default_zero_quaternions(self) -> None:
        """QuinticPathPlanner should plan with zero yaw when current and target quaternions are zero."""
        graph, [planner_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("QuinticPathPlanner", "isaacsim.robot.wheeled_robots.QuinticPathPlanner"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("QuinticPathPlanner.inputs:currentPosition", [0.0, 0.0, 0.0]),
                    ("QuinticPathPlanner.inputs:targetPosition", [1.0, 0.0, 0.0]),
                    ("QuinticPathPlanner.inputs:maxAccel", 1000.0),
                    ("QuinticPathPlanner.inputs:maxJerk", 1000.0),
                ],
            },
        )

        await og.Controller.evaluate(graph)

        target = og.Controller(og.Controller.attribute("outputs:target", planner_node)).get()
        path_arrays = og.Controller(og.Controller.attribute("outputs:pathArrays", planner_node)).get()
        self.assertTrue(np.all(np.isfinite(target)))
        self.assertTrue(np.all(np.isfinite(path_arrays)))
        self.assertGreater(len(path_arrays), 0)

    async def test_stanley_control_uses_zero_yaw_for_default_zero_quaternion(self) -> None:
        """StanleyControlPID should not fail when its orientation input remains at the OGN zero default."""
        graph, [stanley_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("StanleyControlPID", "isaacsim.robot.wheeled_robots.StanleyControlPID"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("StanleyControlPID.inputs:currentPosition", [0.0, 0.0, 0.0]),
                    ("StanleyControlPID.inputs:currentSpeed", [0.0, 0.0, 0.0]),
                    ("StanleyControlPID.inputs:reachedGoal", [False, False]),
                    ("StanleyControlPID.inputs:target", [0.0, 0.0, 0.5]),
                    ("StanleyControlPID.inputs:thresholds", [0.1, 0.1]),
                    ("StanleyControlPID.inputs:wheelBase", 0.4132),
                    ("StanleyControlPID.inputs:step", 0.16666666667),
                ],
            },
        )

        await og.Controller.evaluate(graph)

        linear_velocity = og.Controller(og.Controller.attribute("outputs:linearVelocity", stanley_node)).get()
        angular_velocity = og.Controller(og.Controller.attribute("outputs:angularVelocity", stanley_node)).get()
        self.assertTrue(np.isfinite(linear_velocity))
        self.assertTrue(np.isfinite(angular_velocity))
