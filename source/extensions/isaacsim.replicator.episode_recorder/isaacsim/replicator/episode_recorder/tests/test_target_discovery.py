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

"""Tests for the ``isaacsim.replicator.episode_recorder.target_discovery`` USD walkers."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.replicator.episode_recorder import target_discovery
from pxr import UsdPhysics


async def _build_scene_with_articulation_and_rigids() -> None:
    """Build a scene with one articulation, one loose rigid cube, and one plain xform."""
    stage_utils.define_prim("/World", "Xform")
    stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")

    robot = stage_utils.define_prim("/World/Robot", "Xform")
    UsdPhysics.ArticulationRootAPI.Apply(robot)

    Cube("/World/Robot/Link", sizes=0.25, positions=[0.0, 0.0, 0.25])
    GeomPrim("/World/Robot/Link", apply_collision_apis=True)
    RigidPrim("/World/Robot/Link", masses=[1.0])

    Cube("/World/Cube", sizes=0.25, positions=[1.0, 0.0, 0.5])
    GeomPrim("/World/Cube", apply_collision_apis=True)
    RigidPrim("/World/Cube", masses=[1.0])

    stage_utils.define_prim("/World/Marker", "Xform")
    XformPrim("/World/Marker", positions=[-1.0, 0.0, 0.0], reset_xform_op_properties=True)


class TestTargetDiscovery(omni.kit.test.AsyncTestCase):
    """Exercise each discovery helper against a fixture stage."""

    async def setUp(self) -> None:
        """Set up the test fixture."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down the test fixture."""
        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_discover_articulations_finds_robot(self) -> None:
        """Run the discover articulations finds robot test."""
        await _build_scene_with_articulation_and_rigids()

        arts = target_discovery.discover_articulations_under("/World")
        self.assertEqual(set(arts.values()), {"/World/Robot"})
        for name in arts:
            self.assertTrue(name.replace("_", "").isalnum(), f"Name '{name}' is not HDF5-safe.")

    async def test_discover_rigid_bodies_excludes_articulation_descendants(self) -> None:
        """Run the discover rigid bodies excludes articulation descendants test."""
        await _build_scene_with_articulation_and_rigids()

        rigids = target_discovery.discover_rigid_bodies_under("/World")
        self.assertIn("/World/Cube", rigids.values())
        for path in rigids.values():
            self.assertFalse(path.startswith("/World/Robot/"))

    async def test_discover_xforms_filters_articulations_by_default(self) -> None:
        """Run the discover xforms filters articulations by default test."""
        await _build_scene_with_articulation_and_rigids()

        xforms = target_discovery.discover_xforms_under("/World")
        values = set(xforms.values())
        self.assertIn("/World/Marker", values)
        self.assertIn("/World/Cube", values)
        self.assertNotIn("/World/Robot", values)

    async def test_discover_all_under_bundles_articulations_and_rigids(self) -> None:
        """Run the discover all under bundles articulations and rigids test."""
        await _build_scene_with_articulation_and_rigids()

        arts, prims = target_discovery.discover_all_under("/World")
        self.assertEqual(set(arts.values()), {"/World/Robot"})
        self.assertIn("/World/Cube", prims.values())
        self.assertFalse(set(arts.keys()) & set(prims.keys()))

    async def test_discover_all_under_includes_loose_xforms_when_requested(self) -> None:
        """Run the discover all under includes loose xforms when requested test."""
        await _build_scene_with_articulation_and_rigids()

        arts, prims = target_discovery.discover_all_under("/World", include_loose_xforms=True)
        self.assertIn("/World/Marker", prims.values())

    async def test_max_depth_restricts_walk(self) -> None:
        """Run the max depth restricts walk test."""
        await _build_scene_with_articulation_and_rigids()

        nested_xforms = target_discovery.discover_xforms_under("/World", max_depth=1)
        for path in nested_xforms.values():
            self.assertLessEqual(path.count("/"), 2, f"{path} is deeper than max_depth=1")

    async def test_invalid_root_raises(self) -> None:
        """Run the invalid root raises test."""
        with self.assertRaises(ValueError):
            target_discovery.discover_articulations_under("/World/DoesNotExist")
