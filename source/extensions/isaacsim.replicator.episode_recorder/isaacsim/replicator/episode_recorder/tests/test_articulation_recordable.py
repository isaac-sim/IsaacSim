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

"""Regression tests for the USD-only :class:`ArticulationRecordable`."""

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.episode_recorder import ArticulationRecordable, ReplayPolicy
from pxr import UsdPhysics


class _FakeXformWrapper:
    """Stand-in for :class:`XformPrim` that records the calls made against it."""

    def __init__(self, paths: list[str]) -> None:
        self.paths = list(paths)
        self.set_world_poses_calls: list[tuple[np.ndarray, np.ndarray]] = []
        self.reset_calls = 0

    def set_world_poses(self, *, positions: np.ndarray, orientations: np.ndarray) -> None:
        self.set_world_poses_calls.append((np.asarray(positions).copy(), np.asarray(orientations).copy()))

    def reset_xform_op_properties(self) -> None:
        self.reset_calls += 1


class ArticulationRecordableTests(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_discovers_xformable_links_under_root(self) -> None:
        stage = omni.usd.get_context().get_stage()
        stage_utils.define_prim("/World", "Xform")
        robot = stage_utils.define_prim("/World/Robot", "Xform")
        UsdPhysics.ArticulationRootAPI.Apply(robot)
        stage_utils.define_prim("/World/Robot/base", "Xform")
        stage_utils.define_prim("/World/Robot/link_1", "Xform")
        stage_utils.define_prim("/World/Robot/link_2", "Xform")

        rec = ArticulationRecordable(group="state/robot", prim_path="/World/Robot")
        rec.on_session_open(stage)

        self.assertGreaterEqual(rec.num_links, 4)  # root + 3 links at minimum
        self.assertEqual(rec.link_paths[0], "/World/Robot")
        self.assertIn("/World/Robot/base", rec.link_paths)
        self.assertIn("/World/Robot/link_1", rec.link_paths)
        self.assertIn("/World/Robot/link_2", rec.link_paths)

        channels = rec.describe_channels()
        self.assertEqual(channels["positions"].shape, (rec.num_links, 3))
        self.assertEqual(channels["orientations"].shape, (rec.num_links, 4))

        manifest = rec.to_manifest()
        self.assertEqual(manifest["type"], "articulation")
        self.assertEqual(manifest["link_paths"], rec.link_paths)
        self.assertTrue(manifest["include_root"])

    async def test_include_root_preserved_when_rigid_links_exist(self) -> None:
        stage = omni.usd.get_context().get_stage()
        stage_utils.define_prim("/World", "Xform")
        robot = stage_utils.define_prim("/World/Robot", "Xform")
        UsdPhysics.ArticulationRootAPI.Apply(robot)
        rigid_link = stage_utils.define_prim("/World/Robot/rigid_link", "Xform")
        stage_utils.define_prim("/World/Robot/visual_link", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(rigid_link)

        rec = ArticulationRecordable(group="state/robot", prim_path="/World/Robot")
        rec.on_session_open(stage)

        self.assertEqual(rec.link_paths[0], "/World/Robot")
        self.assertIn("/World/Robot/rigid_link", rec.link_paths)
        self.assertNotIn("/World/Robot/visual_link", rec.link_paths)
        self.assertEqual(len(rec.link_paths), len(set(rec.link_paths)))

    async def test_joint_as_root_falls_back_to_xformable_ancestor(self) -> None:
        """PhysX fixed-base convention: ArticulationRootAPI on a UsdPhysicsJoint whose
        parent is the link-Xform subtree. Discovery must walk up one level."""
        stage = omni.usd.get_context().get_stage()
        stage_utils.define_prim("/World", "Xform")
        body = stage_utils.define_prim("/World/Robot", "Xform")
        stage_utils.define_prim("/World/Robot/link_a", "Xform")
        stage_utils.define_prim("/World/Robot/link_b", "Xform")
        joint = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/root_joint")
        UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())

        rec = ArticulationRecordable(group="state/robot", prim_path="/World/Robot/root_joint")
        rec.on_session_open(stage)

        self.assertGreater(rec.num_links, 0)
        self.assertIn("/World/Robot", rec.link_paths)
        self.assertIn("/World/Robot/link_a", rec.link_paths)
        self.assertIn("/World/Robot/link_b", rec.link_paths)
        self.assertNotIn("/World/Robot/root_joint", rec.link_paths)
        self.assertEqual(body.GetPath().pathString, rec.link_paths[0])

    async def test_no_xformable_under_root_is_noop(self) -> None:
        """An articulation with no Xformable descendants must not crash the session."""
        stage = omni.usd.get_context().get_stage()
        stage_utils.define_prim("/World", "Xform")
        joint = UsdPhysics.FixedJoint.Define(stage, "/World/orphan_joint")
        UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())

        rec = ArticulationRecordable(group="state/orphan", prim_path="/World/orphan_joint")
        rec.on_session_open(stage)

        self.assertEqual(rec.num_links, 0)
        self.assertIsNone(rec._wrapper)
        # sample() and apply() should be no-ops, not raise.
        frame = rec.sample()
        self.assertEqual(frame["positions"].shape, (0, 3))
        self.assertEqual(frame["orientations"].shape, (0, 4))
        rec.apply(frame, policy=ReplayPolicy(strictness="best_effort"))

    async def test_from_manifest_preserves_link_paths(self) -> None:
        entry = {
            "type": "articulation",
            "group": "state/robot",
            "prim_path": "/World/Robot",
            "include_root": True,
            "link_paths": ["/World/Robot", "/World/Robot/link_1"],
        }
        rec = ArticulationRecordable.from_manifest(entry)
        self.assertEqual(rec.prim_path, "/World/Robot")
        self.assertEqual(rec.link_paths, ["/World/Robot", "/World/Robot/link_1"])
        self.assertEqual(rec.num_links, 2)

    async def test_session_close_invalidates_wrapper(self) -> None:
        rec = ArticulationRecordable(
            group="state/robot",
            prim_path="/World/Robot",
            link_paths=["/World/Robot", "/World/Robot/link_1"],
        )
        rec._wrapper = _FakeXformWrapper(rec.link_paths)
        rec._xform_ops_reset = True

        rec.on_session_close()

        self.assertIsNone(rec._wrapper)
        self.assertFalse(rec._xform_ops_reset)

    async def test_apply_writes_world_poses(self) -> None:
        rec = ArticulationRecordable(
            group="state/robot",
            prim_path="/World/Robot",
            link_paths=["/World/Robot", "/World/Robot/link_1"],
        )
        fake = _FakeXformWrapper(rec.link_paths)
        rec._wrapper = fake

        positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (2, 1))
        rec.apply(
            {"positions": positions, "orientations": orientations},
            policy=ReplayPolicy(strictness="best_effort"),
        )

        self.assertEqual(len(fake.set_world_poses_calls), 1)
        call_positions, call_orientations = fake.set_world_poses_calls[0]
        np.testing.assert_array_equal(call_positions, positions)
        np.testing.assert_array_equal(call_orientations, orientations)

    async def test_apply_self_heals_missing_xform_ops_once(self) -> None:
        rec = ArticulationRecordable(
            group="state/robot",
            prim_path="/World/Robot",
            link_paths=["/World/Robot", "/World/Robot/link_1"],
        )

        class _FailOnceWrapper(_FakeXformWrapper):
            def __init__(self, paths: list[str]) -> None:
                super().__init__(paths)
                self._failed_once = False

            def set_world_poses(self, *, positions, orientations):  # type: ignore[override]
                if not self._failed_once:
                    self._failed_once = True
                    raise RuntimeError(
                        "Undefined 'xformOp:translate' property. " "Call reset_xform_op_properties() on the prim."
                    )
                super().set_world_poses(positions=positions, orientations=orientations)

        fake = _FailOnceWrapper(rec.link_paths)
        rec._wrapper = fake

        positions = np.zeros((2, 3), dtype=np.float32)
        orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (2, 1))
        rec.apply(
            {"positions": positions, "orientations": orientations},
            policy=ReplayPolicy(strictness="best_effort"),
        )

        self.assertEqual(fake.reset_calls, 1)
        self.assertTrue(rec._xform_ops_reset)
        self.assertEqual(len(fake.set_world_poses_calls), 1)
