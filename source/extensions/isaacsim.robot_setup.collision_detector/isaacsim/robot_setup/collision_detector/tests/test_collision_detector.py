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

import omni.kit.app
import omni.kit.test
import omni.ui as ui
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema

from ..widget import CollisionDetectorWidget, RigidBodyPairData


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
