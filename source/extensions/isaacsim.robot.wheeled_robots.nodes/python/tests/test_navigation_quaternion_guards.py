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
import omni.usd
import usdrt
from pxr import Gf, UsdGeom


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

    async def test_check_goal_compares_current_yaw_to_target_yaw(self) -> None:
        """CheckGoal2D should compare the wrapped yaw error against the orientation threshold."""
        graph, [check_goal_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("CheckGoal2D", "isaacsim.robot.wheeled_robots.CheckGoal2D"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CheckGoal2D.inputs:currentPosition", [0.0, 0.0, 0.0]),
                    ("CheckGoal2D.inputs:thresholds", [0.1, 0.05]),
                ],
            },
        )

        target_attribute = og.Controller.attribute("inputs:target", check_goal_node)
        target_changed_attribute = og.Controller.attribute("inputs:targetChanged", check_goal_node)
        orientation_attribute = og.Controller.attribute("inputs:currentOrientation", check_goal_node)
        reached_goal_attribute = og.Controller.attribute("outputs:reachedGoal", check_goal_node)
        cases = [
            (0.8, 0.8, True),
            (0.8, -1.0, False),
            (np.pi - 0.01, -np.pi + 0.01, True),
        ]

        for target_yaw, current_yaw, expected in cases:
            og.Controller.set(target_attribute, [0.0, 0.0, target_yaw])
            og.Controller.set(target_changed_attribute, True)
            og.Controller.set(orientation_attribute, [0.0, 0.0, np.sin(current_yaw / 2.0), np.cos(current_yaw / 2.0)])
            await og.Controller.evaluate(graph)

            reached_goal = og.Controller(reached_goal_attribute).get()
            self.assertEqual(list(reached_goal), [True, expected])

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

    async def test_quintic_path_planner_uses_signed_yaw_from_target_prim(self) -> None:
        """QuinticPathPlanner should preserve the sign of a targetPrim's rotation, not just its magnitude."""
        stage = omni.usd.get_context().get_stage()
        goal_prim = UsdGeom.Xform.Define(stage, "/World/Goal")
        goal_prim.AddTranslateOp().Set(Gf.Vec3d(1.0, 0.0, 0.0))
        rotate_op = goal_prim.AddRotateXYZOp()

        graph, [planner_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("QuinticPathPlanner", "isaacsim.robot.wheeled_robots.QuinticPathPlanner"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("QuinticPathPlanner.inputs:currentPosition", [0.0, 0.0, 0.0]),
                    ("QuinticPathPlanner.inputs:currentOrientation", [0.0, 0.0, 0.0, 1.0]),
                    ("QuinticPathPlanner.inputs:maxAccel", 1000.0),
                    ("QuinticPathPlanner.inputs:maxJerk", 1000.0),
                    (
                        "QuinticPathPlanner.inputs:targetPrim",
                        [usdrt.Sdf.Path("/World/Goal")],
                    ),
                ],
            },
        )

        target_attribute = og.Controller.attribute("outputs:target", planner_node)

        for goal_yaw_deg, expected_sign in ((30.0, 1.0), (-30.0, -1.0)):
            # perturb the goal far enough (>0.05 rad) from any prior target to force a re-plan
            rotate_op.Set(Gf.Vec3d(0.0, 0.0, 179.0))
            await og.Controller.evaluate(graph)

            rotate_op.Set(Gf.Vec3d(0.0, 0.0, goal_yaw_deg))
            await og.Controller.evaluate(graph)

            planned_yaw = og.Controller(target_attribute).get()[2]
            self.assertAlmostEqual(planned_yaw, np.radians(goal_yaw_deg), places=3)
            self.assertEqual(np.sign(planned_yaw), expected_sign)

    async def test_quintic_path_planner_wraps_target_change_across_pi(self) -> None:
        """QuinticPathPlanner should compare target yaw changes using the shortest wrapped angle."""
        stage = omni.usd.get_context().get_stage()
        goal_prim = UsdGeom.Xform.Define(stage, "/World/GoalWrap")
        goal_prim.AddTranslateOp().Set(Gf.Vec3d(1.0, 0.0, 0.0))
        rotate_op = goal_prim.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3d(0.0, 0.0, 179.0))

        graph, [planner_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraphWrap"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("QuinticPathPlanner", "isaacsim.robot.wheeled_robots.QuinticPathPlanner"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("QuinticPathPlanner.inputs:currentPosition", [0.0, 0.0, 0.0]),
                    ("QuinticPathPlanner.inputs:currentOrientation", [0.0, 0.0, 0.0, 1.0]),
                    ("QuinticPathPlanner.inputs:maxAccel", 1000.0),
                    ("QuinticPathPlanner.inputs:maxJerk", 1000.0),
                    ("QuinticPathPlanner.inputs:targetPrim", [usdrt.Sdf.Path("/World/GoalWrap")]),
                ],
            },
        )

        await og.Controller.evaluate(graph)
        target_changed = og.Controller.attribute("outputs:targetChanged", planner_node)
        self.assertTrue(og.Controller(target_changed).get())

        rotate_op.Set(Gf.Vec3d(0.0, 0.0, -179.0))
        await og.Controller.evaluate(graph)
        self.assertFalse(og.Controller(target_changed).get())

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
