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

"""Tests for `GeometriesRoutingRule._compute_geometry_hash` unit-invariance."""

from __future__ import annotations

import os
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.perf.geometries import (
    _GEOMETRY_HASH_QUANTIZE_MAX_METERS,
    GeometriesRoutingRule,
)
from pxr import Gf, Usd, UsdGeom


def _build_unit_triangle_stage(path: str, meters_per_unit: float) -> Usd.Stage:
    """Create a stage with a single triangle mesh at `meters_per_unit`.

    The triangle is authored in **stage units** but represents the same
    physical geometry: a right triangle with legs of 1 m.

    Args:
        path: File path where the stage root layer is created.
        meters_per_unit: Stage meters-per-unit metadata value.

    Returns:
        Stage containing the authored triangle mesh.

    """
    stage = Usd.Stage.CreateNew(path)
    stage.SetMetadata("metersPerUnit", meters_per_unit)
    stage.SetMetadata("upAxis", "Z")

    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())

    mesh = UsdGeom.Mesh.Define(stage, "/root/triangle")
    leg_m = 1.0
    leg = leg_m / meters_per_unit
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(0.0, 0.0, 0.0),
            Gf.Vec3f(leg, 0.0, 0.0),
            Gf.Vec3f(0.0, leg, 0.0),
        ]
    )
    mesh.CreateExtentAttr([Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(leg, leg, 0.0)])
    stage.GetRootLayer().Save()
    return stage


class TestGeometryHashUnits(omni.kit.test.AsyncTestCase):
    """`_compute_geometry_hash` should be invariant to `metersPerUnit`."""

    async def setUp(self) -> None:
        """Allocate a scratch directory for the per-test stage files."""
        self._tmpdir = tempfile.mkdtemp(prefix="isaacsim_geometry_hash_")

    async def tearDown(self) -> None:
        """Remove the scratch directory created in `setUp`."""
        import shutil

        shutil.rmtree(self._tmpdir, ignore_errors=True)

    async def test_equivalent_meshes_across_units_hash_identically(self) -> None:
        """The same 1 m triangle authored in m, cm, mm hashes to the same digest."""
        hashes: dict[float, str] = {}
        for mpu in (1.0, 0.01, 0.001):
            path = os.path.join(self._tmpdir, f"tri_{int(1 / mpu)}.usda")
            stage = _build_unit_triangle_stage(path, mpu)
            rule = GeometriesRoutingRule(
                source_stage=stage,
                package_root=self._tmpdir,
                destination_path="payloads",
                args={},
            )
            prim = stage.GetPrimAtPath("/root/triangle")
            hashes[mpu] = rule._compute_geometry_hash(prim)
        self.assertEqual(
            hashes[1.0],
            hashes[0.01],
            msg="Meters vs. centimeters produced divergent hashes",
        )
        self.assertEqual(
            hashes[1.0],
            hashes[0.001],
            msg="Meters vs. millimeters produced divergent hashes",
        )

    async def test_quantum_never_exceeds_cap(self) -> None:
        """For a huge mesh the grid stays at or below the 1 cm ceiling (in meters)."""
        path = os.path.join(self._tmpdir, "huge.usda")
        stage = Usd.Stage.CreateNew(path)
        stage.SetMetadata("metersPerUnit", 1.0)
        stage.SetMetadata("upAxis", "Z")
        root = UsdGeom.Xform.Define(stage, "/root")
        stage.SetDefaultPrim(root.GetPrim())
        mesh = UsdGeom.Mesh.Define(stage, "/root/huge")
        # 10 km mesh: the pre-fix code would have picked a 10 m grid.
        extent_half = 10000.0
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        mesh.CreatePointsAttr(
            [
                Gf.Vec3f(0.0, 0.0, 0.0),
                Gf.Vec3f(extent_half, 0.0, 0.0),
                Gf.Vec3f(0.0, extent_half, 0.0),
            ]
        )
        mesh.CreateExtentAttr([Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(extent_half, extent_half, 0.0)])

        # Two meshes with points that differ only by ~5 cm must still hash
        # differently because the clamp keeps the grid at 1 cm.
        path_near = os.path.join(self._tmpdir, "huge_near.usda")
        stage_near = Usd.Stage.CreateNew(path_near)
        stage_near.SetMetadata("metersPerUnit", 1.0)
        stage_near.SetMetadata("upAxis", "Z")
        root_near = UsdGeom.Xform.Define(stage_near, "/root")
        stage_near.SetDefaultPrim(root_near.GetPrim())
        mesh_near = UsdGeom.Mesh.Define(stage_near, "/root/huge")
        mesh_near.CreateFaceVertexCountsAttr([3])
        mesh_near.CreateFaceVertexIndicesAttr([0, 1, 2])
        mesh_near.CreatePointsAttr(
            [
                Gf.Vec3f(0.0, 0.0, 0.0),
                Gf.Vec3f(extent_half + 0.05, 0.0, 0.0),  # 5 cm shift
                Gf.Vec3f(0.0, extent_half, 0.0),
            ]
        )
        mesh_near.CreateExtentAttr([Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(extent_half + 0.05, extent_half, 0.0)])

        rule_first = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )
        h1 = rule_first._compute_geometry_hash(stage.GetPrimAtPath("/root/huge"))
        rule_second = GeometriesRoutingRule(
            source_stage=stage_near,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )
        h2 = rule_second._compute_geometry_hash(stage_near.GetPrimAtPath("/root/huge"))
        self.assertNotEqual(
            h1,
            h2,
            msg=(
                f"Clamp at {_GEOMETRY_HASH_QUANTIZE_MAX_METERS} m should keep "
                "5 cm differences distinguishable on large meshes."
            ),
        )
