# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for the URDF collision-visualization option."""

import omni.kit.test
from isaacsim.asset.exporter.urdf.converter.link_reader import read_link
from pxr import Usd, UsdGeom, UsdPhysics


class TestCollisionVisualizationOption(omni.kit.test.TestCase):
    def _create_link_with_collision_cube(self) -> Usd.Prim:
        stage = Usd.Stage.CreateInMemory()
        link = UsdGeom.Xform.Define(stage, "/Robot/link").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(link)
        collision = UsdGeom.Cube.Define(stage, "/Robot/link/Collision").GetPrim()
        UsdPhysics.CollisionAPI.Apply(collision)
        return link

    def test_collision_geometry_is_not_visual_by_default(self) -> None:
        link = read_link(self._create_link_with_collision_cube())

        self.assertEqual(len(link.collisions), 1)
        self.assertEqual(len(link.visuals), 0)

    def test_collision_geometry_is_visual_when_requested(self) -> None:
        link = read_link(self._create_link_with_collision_cube(), visualize_collision_meshes=True)

        self.assertEqual(len(link.collisions), 1)
        self.assertEqual(len(link.visuals), 1)
        self.assertEqual(link.visuals[0].name, link.collisions[0].name)
