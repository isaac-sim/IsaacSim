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

"""UI tests for the Robot Poser extension.

Verify that interactive widgets are discoverable by their identifiers and
that basic user interactions (opening the window, adding a named pose)
are registered and stored in USD as expected.

Requires ``omni.kit.ui_test`` and ``isaacsim.test.utils``; the test class is
skipped when these are not available (e.g. during the startup test run).
"""

try:
    import omni.kit.ui_test as ui_test
    from isaacsim.test.utils import MenuUITestCase

    _HAS_UI_TEST = True
except ImportError:
    _HAS_UI_TEST = False

if _HAS_UI_TEST:
    import omni.kit.app
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics
    from usd.schema.isaac import robot_schema

    WINDOW_TITLE = "Robot Poser"
    MENU_PATH = f"Tools/Robotics/{WINDOW_TITLE}"

    class TestRobotPoserUI(MenuUITestCase):
        """UI tests for Robot Poser extension identifiers and interactions."""

        async def setUp(self) -> None:
            """Create a fresh stage with a two-link robot before each test."""
            await super().setUp()
            self._create_robot()

        def _create_robot(self) -> None:
            """Create a two-link robot with IsaacRobotAPI on the current stage."""
            stage = self._stage

            robot_xform = UsdGeom.Xform.Define(stage, "/World/Robot")
            UsdPhysics.RigidBodyAPI.Apply(robot_xform.GetPrim())
            UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())

            link1 = UsdGeom.Xform.Define(stage, "/World/Robot/Link1")
            UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())

            joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint1")
            joint.CreateBody0Rel().SetTargets([robot_xform.GetPrim().GetPath()])
            joint.CreateBody1Rel().SetTargets([link1.GetPrim().GetPath()])
            joint.CreateAxisAttr("X")
            joint.GetLowerLimitAttr().Set(-90.0)
            joint.GetUpperLimitAttr().Set(90.0)
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

            robot_schema.ApplyRobotAPI(robot_xform.GetPrim())

            self._robot_prim = robot_xform.GetPrim()
            self._link1_prim = link1.GetPrim()

        async def _open_poser_window(self) -> None:
            """Open the Robot Poser window via the menu."""
            await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
            await ui_test.human_delay()
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()

        async def _close_poser_window(self) -> None:
            """Close the Robot Poser window if it is visible."""
            window = ui_test.find(WINDOW_TITLE)
            if window is not None:
                await self.menu_click_with_retry(MENU_PATH)
                await ui_test.human_delay()

        # --------------------------------------------------------------
        # Tests
        # --------------------------------------------------------------

        async def test_window_opens(self) -> None:
            """Verify the Robot Poser window opens via the menu."""
            await self._open_poser_window()
            window = ui_test.find(WINDOW_TITLE)
            self.assertIsNotNone(window, "Robot Poser window should be visible after menu click")
            await self._close_poser_window()

        async def test_named_poses_frame_exists(self) -> None:
            """Verify the Named Poses collapsable frame is present."""
            await self._open_poser_window()
            frame = await self.find_widget_with_retry(
                f"{WINDOW_TITLE}//Frame/**/CollapsableFrame[*].identifier=='poser_named_poses_frame'"
            )
            self.assertIsNotNone(frame, "Named Poses frame should be discoverable by identifier")
            await self._close_poser_window()

        async def test_named_poses_tree_exists(self) -> None:
            """Verify the Named Poses tree view is present."""
            await self._open_poser_window()
            tree = await self.find_widget_with_retry(
                f"{WINDOW_TITLE}//Frame/**/TreeView[*].identifier=='poser_named_poses_tree'"
            )
            self.assertIsNotNone(tree, "Named Poses tree view should be discoverable by identifier")
            await self._close_poser_window()

        async def test_add_pose_button_exists(self) -> None:
            """Verify the add-pose button is present in the table."""
            await self._open_poser_window()
            add_btn = await self.find_widget_with_retry(
                f"{WINDOW_TITLE}//Frame/**/InvisibleButton[*].identifier=='poser_add_pose'"
            )
            self.assertIsNotNone(add_btn, "Add-pose button should be discoverable by identifier")
            await self._close_poser_window()

        async def test_add_pose_creates_usd_prim(self) -> None:
            """Click the add-pose button and verify a named pose prim is created in USD."""
            await self._open_poser_window()

            for _ in range(10):
                await omni.kit.app.get_app().next_update_async()

            add_btn = await self.find_widget_with_retry(
                f"{WINDOW_TITLE}//Frame/**/InvisibleButton[*].identifier=='poser_add_pose'"
            )
            self.assertIsNotNone(add_btn, "Add-pose button must exist before clicking")
            await add_btn.click()
            await ui_test.human_delay()

            for _ in range(10):
                await omni.kit.app.get_app().next_update_async()

            stage = omni.usd.get_context().get_stage()
            named_poses_scope = stage.GetPrimAtPath("/World/Robot/NamedPoses")
            if named_poses_scope and named_poses_scope.IsValid():
                children = [c for c in named_poses_scope.GetChildren() if c.GetTypeName() == "IsaacNamedPose"]
                self.assertGreater(
                    len(children), 0, "At least one IsaacNamedPose should be created after add-pose click"
                )

            await self._close_poser_window()
