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

"""Tests for the collision detector widget's detection logic."""

import contextlib

import omni.kit.app
import omni.kit.test
import omni.ui as ui
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema

from .. import widget as widget_module
from ..widget import CollisionDetectorWidget, RigidBodyPairData, _iter_body_prims


@contextlib.contextmanager
def _exhausted_group_pool():
    """Make ``register_selection_group`` report an exhausted pool.

    The engine offers no way to release a selection group, so genuinely
    draining the 255-entry pool would poison every later test in the process.
    """
    real_get_context = omni.usd.get_context

    class _ExhaustedContext:
        def __init__(self, real) -> None:
            self._real = real

        def __getattr__(self, name):
            return getattr(self._real, name)

        def register_selection_group(self) -> int:
            return 0

    omni.usd.get_context = lambda *args, **kwargs: _ExhaustedContext(real_get_context(*args, **kwargs))
    try:
        yield
    finally:
        omni.usd.get_context = real_get_context


class TestCollisionDetection(omni.kit.test.AsyncTestCase):
    """Verify self-collision detection on a four-link serial robot.

    The fixture builds a robot whose links 1-3 have overlapping cube
    colliders while link 4 is placed far away.  Revolute joints form a
    serial chain (Link1-Link2-Link3-Link4), so adjacent pairs are
    automatically filtered by the physics engine.
    """

    async def setUp(self) -> None:
        """Create a fresh stage and build the test robot."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        self._robot_prim = self._build_four_link_robot()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Wait for pending loads and clean up."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    # ------------------------------------------------------------------
    # Robot fixture
    # ------------------------------------------------------------------

    def _build_four_link_robot(self) -> Usd.Prim:
        """Build a serial robot with four links and three revolute joints.

        Links 1-3 sit at nearby positions with large cubes that overlap.
        Link 4 is offset far enough that its cube does not touch the others.

        Returns:
            The robot root prim at ``/World/Robot``.
        """
        stage = self._stage

        UsdGeom.Xform.Define(stage, "/World")

        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(0.0)

        # -- Link 1 (articulation root) -----------------------------------
        link1 = self._add_link("/World/Robot/Link1", Gf.Vec3d(0, 0, 0))
        UsdPhysics.ArticulationRootAPI.Apply(link1)
        PhysxSchema.PhysxArticulationAPI.Apply(link1)
        PhysxSchema.PhysxArticulationAPI(link1).CreateEnabledSelfCollisionsAttr().Set(True)

        # -- Links 2-4 ----------------------------------------------------
        link2 = self._add_link("/World/Robot/Link2", Gf.Vec3d(1, 0, 0))
        link3 = self._add_link("/World/Robot/Link3", Gf.Vec3d(2, 0, 0))
        link4 = self._add_link("/World/Robot/Link4", Gf.Vec3d(200, 0, 0))

        # -- Revolute joints (serial chain) --------------------------------
        self._add_revolute_joint("/World/Robot/Joint1", link1, link2, Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 0, 0))
        self._add_revolute_joint("/World/Robot/Joint2", link2, link3, Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 0, 0))
        self._add_revolute_joint("/World/Robot/Joint3", link3, link4, Gf.Vec3f(198, 0, 0), Gf.Vec3f(0, 0, 0))

        # -- Robot schema --------------------------------------------------
        robot_root = stage.GetPrimAtPath("/World/Robot")
        robot_schema.ApplyRobotAPI(robot_root)

        return robot_root

    def _add_link(self, path: str, position: Gf.Vec3d) -> Usd.Prim:
        xform = UsdGeom.Xform.Define(self._stage, path)
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        xform.AddTranslateOp().Set(position)

        cube = UsdGeom.Cube.Define(self._stage, f"{path}/Collision")
        cube.CreateSizeAttr(10.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return prim

    def _add_revolute_joint(
        self, path: str, body0: Usd.Prim, body1: Usd.Prim, local_pos0: Gf.Vec3f, local_pos1: Gf.Vec3f
    ) -> UsdPhysics.RevoluteJoint:
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, path)
        joint.CreateBody0Rel().SetTargets([body0.GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPath()])
        joint.CreateAxisAttr("X")
        joint.CreateLocalPos0Attr().Set(local_pos0)
        joint.CreateLocalPos1Attr().Set(local_pos1)
        return joint

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _detect(self, include_env: bool = False) -> list[RigidBodyPairData]:
        """Create a widget, run detection, destroy the widget, return pairs.

        Args:
            include_env: When True, enable environment collision detection.

        Returns:
            Detected collision-pair data for the test robot.
        """
        with ui.Frame():
            widget = CollisionDetectorWidget()

        widget._include_env_collisions = include_env
        pairs = widget._detect_collisions(self._robot_prim, self._stage)
        widget.destroy()
        return pairs

    @staticmethod
    def _find_pair(pairs: list[RigidBodyPairData], name_a: str, name_b: str) -> RigidBodyPairData | None:
        target = {name_a, name_b}
        for p in pairs:
            if {p.body_a_name, p.body_b_name} == target:
                return p
        return None

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_nonadjacent_overlap_detected(self) -> None:
        """Link1 and Link3 overlap and are not joint-adjacent, so they collide."""
        pairs = self._detect()
        pair = self._find_pair(pairs, "Link1", "Link3")
        self.assertIsNotNone(pair, f"Expected Link1-Link3 pair in {[(p.body_a_name, p.body_b_name) for p in pairs]}")
        self.assertFalse(pair.filtered)

    async def test_adjacent_pairs_filtered_by_joints(self) -> None:
        """Link1-Link2 and Link2-Link3 are joint-adjacent and must not appear."""
        pairs = self._detect()
        names = [(p.body_a_name, p.body_b_name) for p in pairs]
        self.assertIsNone(self._find_pair(pairs, "Link1", "Link2"), f"Link1-Link2 should be filtered: {names}")
        self.assertIsNone(self._find_pair(pairs, "Link2", "Link3"), f"Link2-Link3 should be filtered: {names}")

    async def test_nonoverlapping_link_nonadjacent_pairs(self) -> None:
        """Link4 is non-adjacent to Link1 and Link2, so those pairs are reported."""
        pairs = self._detect()
        self.assertIsNone(
            self._find_pair(pairs, "Link1", "Link4"),
            "Link1-Link4 are non-adjacent and not colliding and should be filtered",
        )
        self.assertIsNone(
            self._find_pair(pairs, "Link2", "Link4"),
            "Link2-Link4 are non-adjacent and not colliding and should be filtered",
        )
        self.assertIsNone(
            self._find_pair(pairs, "Link3", "Link4"),
            "Link3-Link4 are joint-adjacent and should be filtered",
        )
        self.assertIsNotNone(
            self._find_pair(pairs, "Link1", "Link3"),
            "Link1-Link3 are non-adjacent and should be reported",
        )

    async def test_filtered_pair_detected(self) -> None:
        """A FilteredPairsAPI relationship between Link1 and Link3 is listed with filtered=True."""
        link1 = self._stage.GetPrimAtPath("/World/Robot/Link1")
        filtered_api = UsdPhysics.FilteredPairsAPI.Apply(link1)
        rel = filtered_api.CreateFilteredPairsRel()
        rel.AddTarget(Sdf.Path("/World/Robot/Link3"))
        await omni.kit.app.get_app().next_update_async()

        pairs = self._detect()
        pair = self._find_pair(pairs, "Link1", "Link3")
        self.assertIsNotNone(pair, "Link1-Link3 must still be listed after adding a filter")
        self.assertTrue(pair.filtered, "Link1-Link3 pair should have filtered=True")

    # ------------------------------------------------------------------
    # Environment collision tests
    # ------------------------------------------------------------------

    def _add_env_obstacle(self, path: str = "/World/Obstacle", position: Gf.Vec3d = Gf.Vec3d(0, 0, 0)) -> Usd.Prim:
        """Add a non-robot rigid body with a collision cube.

        Args:
            path: USD path for the obstacle.
            position: World-space position.

        Returns:
            The obstacle prim.
        """
        return self._add_link(path, position)

    async def test_env_collision_excluded_by_default(self) -> None:
        """Environment obstacles are excluded when include_env_collisions is False."""
        self._add_env_obstacle()
        await omni.kit.app.get_app().next_update_async()

        pairs = self._detect(include_env=False)
        pair = self._find_pair(pairs, "Link1", "Obstacle")
        self.assertIsNone(pair, "Robot-environment pair should not appear with env collisions disabled")

    async def test_env_collision_included_when_enabled(self) -> None:
        """Environment obstacles overlapping robot links appear when env collisions are enabled."""
        self._add_env_obstacle()
        await omni.kit.app.get_app().next_update_async()

        pairs = self._detect(include_env=True)
        pair = self._find_pair(pairs, "Link1", "Obstacle")
        self.assertIsNotNone(
            pair,
            f"Expected Link1-Obstacle pair with env collisions enabled: {[(p.body_a_name, p.body_b_name) for p in pairs]}",
        )
        self.assertFalse(pair.filtered)

    async def test_env_collision_still_includes_self_collisions(self) -> None:
        """Self-collision pairs are still reported when env collisions are enabled."""
        self._add_env_obstacle()
        await omni.kit.app.get_app().next_update_async()

        pairs = self._detect(include_env=True)
        pair = self._find_pair(pairs, "Link1", "Link3")
        self.assertIsNotNone(pair, "Self-collision pair Link1-Link3 must still appear with env collisions enabled")

    async def test_env_filtered_pair_included_when_enabled(self) -> None:
        """A FilteredPairsAPI between a robot link and an env body appears when env collisions are on."""
        obstacle = self._add_env_obstacle()
        filtered_api = UsdPhysics.FilteredPairsAPI.Apply(self._stage.GetPrimAtPath("/World/Robot/Link1"))
        rel = filtered_api.CreateFilteredPairsRel()
        rel.AddTarget(obstacle.GetPath())
        await omni.kit.app.get_app().next_update_async()

        pairs = self._detect(include_env=True)
        pair = self._find_pair(pairs, "Link1", "Obstacle")
        self.assertIsNotNone(pair, "Filtered robot-env pair must appear with env collisions enabled")
        self.assertTrue(pair.filtered)

    async def test_env_filtered_pair_excluded_by_default(self) -> None:
        """A FilteredPairsAPI between a robot link and an env body is excluded by default."""
        obstacle = self._add_env_obstacle()
        filtered_api = UsdPhysics.FilteredPairsAPI.Apply(self._stage.GetPrimAtPath("/World/Robot/Link1"))
        rel = filtered_api.CreateFilteredPairsRel()
        rel.AddTarget(obstacle.GetPath())
        await omni.kit.app.get_app().next_update_async()

        pairs = self._detect(include_env=False)
        pair = self._find_pair(pairs, "Link1", "Obstacle")
        self.assertIsNone(pair, "Filtered robot-env pair should not appear with env collisions disabled")

    # ------------------------------------------------------------------
    # Nested rigid body traversal
    # ------------------------------------------------------------------

    def _add_nested_rigid_body(self, parent_path: str) -> str:
        """Add a child rigid body with its own collider under an existing link.

        Args:
            parent_path: Path of the parent rigid body.

        Returns:
            Path of the nested rigid body prim.
        """
        nested_path = f"{parent_path}/Nested"
        nested = UsdGeom.Xform.Define(self._stage, nested_path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(nested)
        cube = UsdGeom.Cube.Define(self._stage, f"{nested_path}/Collision")
        cube.CreateSizeAttr(10.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return nested_path

    async def test_iter_body_prims_prunes_nested_bodies(self) -> None:
        """_iter_body_prims yields the root but skips nested rigid bodies and their children."""
        nested_path = self._add_nested_rigid_body("/World/Robot/Link1")
        await omni.kit.app.get_app().next_update_async()

        root = self._stage.GetPrimAtPath("/World/Robot/Link1")
        visited = {str(p.GetPath()) for p in _iter_body_prims(root)}
        self.assertIn("/World/Robot/Link1", visited)
        self.assertIn("/World/Robot/Link1/Collision", visited)
        self.assertNotIn(nested_path, visited, "Nested rigid body prim must be pruned")
        self.assertNotIn(f"{nested_path}/Collision", visited, "Nested rigid body children must be pruned")

    async def test_collect_gprims_halts_at_nested_rigid_body(self) -> None:
        """Gprims under a nested rigid body are not attributed to the parent body."""
        nested_path = self._add_nested_rigid_body("/World/Robot/Link1")
        await omni.kit.app.get_app().next_update_async()

        parent_gprims = CollisionDetectorWidget._collect_gprims(self._stage, "/World/Robot/Link1")
        self.assertIn("/World/Robot/Link1/Collision", parent_gprims)
        self.assertNotIn(
            f"{nested_path}/Collision",
            parent_gprims,
            "Nested rigid body geometry must not be collected under the parent body",
        )

    async def test_collect_gprims_includes_nested_body_when_queried_directly(self) -> None:
        """A nested rigid body still owns its own geometry when queried directly."""
        nested_path = self._add_nested_rigid_body("/World/Robot/Link1")
        await omni.kit.app.get_app().next_update_async()

        nested_gprims = CollisionDetectorWidget._collect_gprims(self._stage, nested_path)
        self.assertEqual(nested_gprims, {f"{nested_path}/Collision"})

    async def test_row_selection_highlights_own_geometry_not_nested_bodies(self) -> None:
        """Selecting a pair highlights each body's own geometry, not nested child bodies.

        The highlight is driven by per-body selection groups (``_group_prims``),
        not by a stage selection. A body's group must contain its own colliders
        but never the geometry of a nested child rigid body, and the stage
        selection is left empty so it cannot fight the group highlight.
        """
        nested_path = self._add_nested_rigid_body("/World/Robot/Link1")
        await omni.kit.app.get_app().next_update_async()

        with ui.Frame():
            widget = CollisionDetectorWidget()
        try:
            pairs = [
                RigidBodyPairData(
                    body_a_name="Link1",
                    body_b_name="Link4",
                    body_a_path="/World/Robot/Link1",
                    body_b_path="/World/Robot/Link4",
                )
            ]
            widget._pairs = pairs
            widget._build_body_color_map()
            widget._tree_model.set_data(pairs)
            item = widget._tree_model._all_items[0]
            widget._on_tree_selection_changed([item])

            link1_group = widget._group_prims.get("/World/Robot/Link1", set())
            self.assertIn(
                "/World/Robot/Link1/Collision",
                link1_group,
                "Link1's own collider must be in its highlight group",
            )
            self.assertNotIn(
                f"{nested_path}/Collision",
                link1_group,
                "Nested rigid body geometry must not be added to the parent body's group",
            )
            self.assertNotIn(
                nested_path,
                widget._group_prims,
                "A nested body that is not part of the pair must not get its own group",
            )
            selected = omni.usd.get_context().get_selection().get_selected_prim_paths()
            self.assertEqual(selected, [], "Row selection must not author a stage selection")
        finally:
            widget.destroy()

    async def test_focal_click_reuses_the_group_holding_the_body_colour(self) -> None:
        """A focal click must resolve a body to the group already carrying its colour.

        Row selection paints a body's colour onto one group; the focal click
        only re-applies a group and never sets a colour itself. Both paths must
        therefore resolve a body to the same group, or the focal highlight
        shows a stale colour or no colour at all.
        """
        with ui.Frame():
            widget = CollisionDetectorWidget()
        try:
            pairs = [
                RigidBodyPairData(
                    body_a_name="Link1",
                    body_b_name="Link3",
                    body_a_path="/World/Robot/Link1",
                    body_b_path="/World/Robot/Link3",
                ),
                RigidBodyPairData(
                    body_a_name="Link2",
                    body_b_name="Link4",
                    body_a_path="/World/Robot/Link2",
                    body_b_path="/World/Robot/Link4",
                ),
            ]
            widget._pairs = pairs
            widget._build_body_color_map()
            widget._tree_model.set_data(pairs)
            row = next(i for i in widget._tree_model._all_items if i.data.body_a_name == "Link2")

            widget._on_tree_selection_changed([row])
            row_gid = widget._active_groups["/World/Robot/Link2"]

            widget._select_collision_prim(row, "/World/Robot/Link2")
            focal_gid = widget._active_groups["/World/Robot/Link2"]

            self.assertEqual(row_gid, focal_gid, "Focal click must reuse the group holding Link2's colour")
        finally:
            widget.destroy()


class TestSelectionGroupAllocation(omni.kit.test.AsyncTestCase):
    """Verify selection-group allocation never leaks IDs or targets the default group.

    The viewport exposes 255 usable selection groups and no way to hand one
    back, so every identifier the widget takes is gone for the life of the
    process.  Group 0 is the viewport's own default and must never be written.
    """

    async def setUp(self) -> None:
        """Start each test from an empty registration list."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        widget_module._registered_group_ids.clear()

    async def tearDown(self) -> None:
        """Leave the registration list empty for the next test."""
        widget_module._registered_group_ids.clear()
        await omni.kit.app.get_app().next_update_async()

    async def test_deregister_keeps_ids_for_reuse(self) -> None:
        """Deregistering must reuse identifiers, because the engine cannot reclaim them."""
        before = [widget_module._get_group_id(i) for i in range(4)]
        widget_module.deregister_selection_groups()
        after = [widget_module._get_group_id(i) for i in range(4)]
        self.assertEqual(before, after, "Re-acquiring after deregister must not consume fresh engine IDs")

    async def test_repeated_deregister_cycles_do_not_grow_the_pool(self) -> None:
        """Repeated deregister cycles must not drain the finite engine pool.

        Deliberately stays well under the 255-entry pool so that a regression
        fails this assertion rather than exhausting the pool for later tests.
        """
        widget_module._get_group_id(4)
        first_cycle_ids = list(widget_module._registered_group_ids)
        for _ in range(20):
            widget_module.deregister_selection_groups()
            widget_module._get_group_id(4)
        self.assertEqual(
            widget_module._registered_group_ids,
            first_cycle_ids,
            "20 deregister cycles must not allocate any new engine selection group",
        )

    async def test_exhausted_pool_reports_no_group_rather_than_default(self) -> None:
        """With nothing registered, exhaustion must report no group instead of group 0."""
        with _exhausted_group_pool():
            self.assertIsNone(widget_module._get_group_id(0))

    async def test_exhausted_pool_does_not_collapse_bodies_onto_one_group(self) -> None:
        """Overflow past the registered groups must report no group, not reuse the last one."""
        widget_module._get_group_id(2)
        registered = list(widget_module._registered_group_ids)
        with _exhausted_group_pool():
            self.assertIsNone(widget_module._get_group_id(7))
        self.assertEqual(widget_module._registered_group_ids, registered)
