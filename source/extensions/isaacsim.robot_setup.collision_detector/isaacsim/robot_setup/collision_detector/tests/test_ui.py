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

"""UI tests for the Robot Self-Collision Detector extension."""

import omni.kit.app
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.usd
from isaacsim.test.utils import MenuUITestCase
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema

from ..window import RobotSelfCollisionWindow

WINDOW_NAME = "Robot Self-Collision Detector"
_Q = f"{WINDOW_NAME}//Frame"


class TestCollisionDetectorUI(MenuUITestCase):
    """Verify that UI controls are wired correctly and state propagates as expected."""

    async def setUp(self) -> None:
        """Build a test robot and open the collision detector window."""
        await super().setUp()
        self._stage = omni.usd.get_context().get_stage()
        self._build_test_robot()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        self._window = RobotSelfCollisionWindow()
        await ui_test.human_delay(50)

    async def tearDown(self) -> None:
        """Close the collision detector window and clean up."""
        if self._window:
            self._window.destroy()
            self._window = None
        await super().tearDown()

    # ------------------------------------------------------------------
    # Robot fixture (3-link, links 1 & 3 overlap)
    # ------------------------------------------------------------------

    def _build_test_robot(self) -> None:
        """Build a three-link serial robot with overlapping colliders on links 1 and 3."""
        stage = self._stage
        UsdGeom.Xform.Define(stage, "/World")
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

        link1 = self._add_link("/World/Robot/Link1", Gf.Vec3d(0, 0, 0))
        UsdPhysics.ArticulationRootAPI.Apply(link1)
        PhysxSchema.PhysxArticulationAPI.Apply(link1)
        PhysxSchema.PhysxArticulationAPI(link1).CreateEnabledSelfCollisionsAttr().Set(True)

        link2 = self._add_link("/World/Robot/Link2", Gf.Vec3d(1, 0, 0))
        link3 = self._add_link("/World/Robot/Link3", Gf.Vec3d(2, 0, 0))

        self._add_revolute_joint("/World/Robot/Joint1", link1, link2)
        self._add_revolute_joint("/World/Robot/Joint2", link2, link3)

        robot_root = stage.GetPrimAtPath("/World/Robot")
        robot_schema.ApplyRobotAPI(robot_root)

    def _add_link(self, path: str, position: Gf.Vec3d) -> Usd.Prim:
        """Add a rigid-body link with a cube collider at the given position.

        Args:
            path: USD prim path for the link.
            position: World-space position.

        Returns:
            The created link prim.
        """
        xform = UsdGeom.Xform.Define(self._stage, path)
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        xform.AddTranslateOp().Set(position)
        cube = UsdGeom.Cube.Define(self._stage, f"{path}/Collision")
        cube.CreateSizeAttr(10.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return prim

    def _add_revolute_joint(self, path: str, body0: Usd.Prim, body1: Usd.Prim) -> UsdPhysics.RevoluteJoint:
        """Add a revolute joint connecting two rigid bodies.

        Args:
            path: USD prim path for the joint.
            body0: First body prim.
            body1: Second body prim.

        Returns:
            The created revolute joint.
        """
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, path)
        joint.CreateBody0Rel().SetTargets([body0.GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPath()])
        joint.CreateAxisAttr("X")
        return joint

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_robot_combo_populated(self) -> None:
        """Robot combo box shows the robot found on stage."""
        combo = await self.find_widget_with_retry(f"{_Q}/**/ComboBox[*].identifier=='collision_detector_robot_combo'")
        self.assertIsNotNone(combo)
        selected_idx = combo.model.get_item_value_model().get_value_as_int()
        self.assertEqual(selected_idx, 0)

    async def test_check_collisions_populates_tree(self) -> None:
        """Clicking 'Check Collisions' populates the TreeView with collision pairs."""
        btn = await self.find_widget_with_retry(f"{_Q}/**/ZStack[*].identifier=='collision_detector_check_collisions'")
        await btn.click()
        await ui_test.human_delay(50)

        tree = await self.find_widget_with_retry(f"{_Q}/**/TreeView[*].identifier=='collision_detector_tree_view'")
        self.assertIsNotNone(tree)
        items = tree.model.get_item_children(None)
        self.assertGreater(len(items), 0, "Expected at least one collision pair after check")

    async def test_env_collisions_checkbox_toggles_state(self) -> None:
        """Toggling the env-collisions checkbox updates the internal flag."""
        cb = await self.find_widget_with_retry(f"{_Q}/**/CheckBox[*].identifier=='collision_detector_env_collisions'")
        self.assertIsNotNone(cb)

        initial_value = cb.model.get_value_as_bool()
        self.assertFalse(initial_value, "Env collisions should default to False")

        await cb.click()
        await ui_test.human_delay()
        self.assertTrue(cb.model.get_value_as_bool(), "Env collisions should be True after click")

        await cb.click()
        await ui_test.human_delay()
        self.assertFalse(cb.model.get_value_as_bool(), "Env collisions should be False after second click")

    async def test_filtered_checkbox_toggles(self) -> None:
        """Clicking a filtered-pair checkbox updates the item's filtered state."""
        btn = await self.find_widget_with_retry(f"{_Q}/**/ZStack[*].identifier=='collision_detector_check_collisions'")
        await btn.click()
        await ui_test.human_delay(50)

        tree = await self.find_widget_with_retry(f"{_Q}/**/TreeView[*].identifier=='collision_detector_tree_view'")
        items = tree.model.get_item_children(None)
        self.assertGreater(len(items), 0, "Need collision pairs to test filtered toggle")

        first_item = items[0]
        self.assertFalse(first_item.data.filtered, "Pair should start unfiltered")

        filtered_cb = await self.find_widget_with_retry(
            f"{_Q}/**/CheckBox[*].identifier=='collision_detector_filtered_cb'"
        )
        self.assertIsNotNone(filtered_cb)
        await filtered_cb.click()
        await ui_test.human_delay()

        self.assertTrue(first_item.data.filtered, "Pair should be filtered after checkbox click")

        prim_a = self._stage.GetPrimAtPath(first_item.data.body_a_path)
        prim_b = self._stage.GetPrimAtPath(first_item.data.body_b_path)
        has_filter = prim_a.HasAPI(UsdPhysics.FilteredPairsAPI) or prim_b.HasAPI(UsdPhysics.FilteredPairsAPI)
        self.assertTrue(has_filter, "FilteredPairsAPI should be authored on stage after toggle")

    async def test_header_labels_exist(self) -> None:
        """Column header labels are present with the expected identifiers."""
        header_a = await self.find_widget_with_retry(f"{_Q}/**/Label[*].identifier=='collision_detector_header_body_a'")
        header_b = await self.find_widget_with_retry(f"{_Q}/**/Label[*].identifier=='collision_detector_header_body_b'")
        header_f = await self.find_widget_with_retry(
            f"{_Q}/**/Label[*].identifier=='collision_detector_header_filtered'"
        )
        self.assertIsNotNone(header_a)
        self.assertIsNotNone(header_b)
        self.assertIsNotNone(header_f)
        self.assertEqual(header_a.widget.text, "Rigid Body  A")
        self.assertEqual(header_b.widget.text, "Rigid Body  B")
        self.assertEqual(header_f.widget.text, "Filtered Pair")

    async def test_body_labels_show_after_check(self) -> None:
        """Body name labels appear in the tree after running collision check."""
        btn = await self.find_widget_with_retry(f"{_Q}/**/ZStack[*].identifier=='collision_detector_check_collisions'")
        await btn.click()
        await ui_test.human_delay(50)

        body_a_labels = ui_test.find_all(f"{_Q}/**/Label[*].identifier=='collision_detector_body_a'")
        body_b_labels = ui_test.find_all(f"{_Q}/**/Label[*].identifier=='collision_detector_body_b'")
        self.assertGreater(len(body_a_labels), 0, "Expected body-A labels after collision check")
        self.assertGreater(len(body_b_labels), 0, "Expected body-B labels after collision check")
