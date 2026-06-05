# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for bounds computation utilities."""

import omni.kit.commands
import omni.kit.test
from isaacsim.core.utils.bounds import (
    compute_aabb,
    compute_combined_aabb,
    compute_obb,
    compute_obb_corners,
    create_bbox_cache,
    recompute_extents,
)
from isaacsim.core.utils.stage import get_current_stage
from pxr import UsdGeom


class TestBounds(omni.kit.test.AsyncTestCase):
    """Test cases for Bounds."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    async def test_recompute_extents(self) -> None:
        """Test recompute extents."""
        stage = get_current_stage()
        cubeGeom = UsdGeom.Cube.Define(stage, "/cube_shape")
        size = 123
        offset = (100, 200, 300)
        cubeGeom.AddTranslateOp().Set(offset)
        cubeGeom.CreateSizeAttr(size)
        await omni.kit.app.get_app().next_update_async()
        cache = create_bbox_cache()
        recompute_extents(cubeGeom)
        # it shouldn't error even if we cast it to a base prim
        recompute_extents(cubeGeom.GetPrim())
        await omni.kit.app.get_app().next_update_async()
        aabb = compute_aabb(cache, "/cube_shape")
        self.assertListEqual(aabb.tolist(), [38.5, 138.5, 238.5, 161.5, 261.5, 361.5])

        result, cube_path = omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube")
        await omni.kit.app.get_app().next_update_async()
        cube_prim = stage.GetPrimAtPath(cube_path)
        cube_mesh = UsdGeom.Mesh(cube_prim)
        points = cube_mesh.GetPointsAttr().Get()

        for i in range(len(points)):
            points[i] = points[i] * 1.5
        cube_mesh.GetPointsAttr().Set(points)

        # recompute extents after changing points
        await omni.kit.app.get_app().next_update_async()
        recompute_extents(cube_prim)
        await omni.kit.app.get_app().next_update_async()

        aabb = compute_aabb(cache, cube_path)
        self.assertListEqual(aabb.tolist(), [-0.75, -0.75, -0.75, 0.75, 0.75, 0.75])
        combined_aabb = compute_combined_aabb(cache, ["/cube_shape", cube_path])
        self.assertListEqual(combined_aabb.tolist(), [-0.75, -0.75, -0.75, 161.5, 261.5, 361.5])
        # this should be the same as including children when calculating bbox for entire scene
        aabb_with_children = compute_aabb(cache, "/", include_children=True)
        self.assertListEqual(combined_aabb.tolist(), aabb_with_children.tolist())

    async def test_nested_recompute_extents(self) -> None:
        """Test nested recompute extents."""
        stage = get_current_stage()
        cubeA = UsdGeom.Cube.Define(stage, "/nested_cube")
        cubeB = UsdGeom.Cube.Define(stage, "/nested_cube/nested_cube")
        cubeC = UsdGeom.Cube.Define(stage, "/nested_cube/nested_cube/nested_cube")
        size = 123
        offset = (100, 200, 300)
        cubeA.AddTranslateOp().Set(offset)
        cubeA.CreateSizeAttr(size)

        cubeB.AddTranslateOp().Set(offset)
        cubeB.CreateSizeAttr(size)

        cubeC.AddTranslateOp().Set(offset)
        cubeC.CreateSizeAttr(size)

        await omni.kit.app.get_app().next_update_async()
        recompute_extents(cubeA, include_children=False)
        await omni.kit.app.get_app().next_update_async()
        cache = create_bbox_cache(use_extents_hint=False)
        await omni.kit.app.get_app().next_update_async()

        aabb = compute_aabb(cache, "/nested_cube")
        self.assertListEqual(aabb.tolist(), [38.5, 138.5, 238.5, 161.5, 261.5, 361.5])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube")
        self.assertListEqual(aabb.tolist(), [138.5, 338.5, 538.5, 261.5, 461.5, 661.5])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube/nested_cube")
        self.assertListEqual(aabb.tolist(), [238.5, 538.5, 838.5, 361.5, 661.5, 961.5])
        recompute_extents(cubeA, include_children=True)
        cache = create_bbox_cache()

        aabb = compute_aabb(cache, "/nested_cube", include_children=True)
        self.assertListEqual(aabb.tolist(), [38.5, 138.5, 238.5, 361.5, 661.5, 961.5])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube", include_children=True)
        self.assertListEqual(aabb.tolist(), [138.5, 338.5, 538.5, 361.5, 661.5, 961.5])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube/nested_cube", include_children=True)
        self.assertListEqual(aabb.tolist(), [238.5, 538.5, 838.5, 361.5, 661.5, 961.5])

        # This should not fail
        recompute_extents(cubeA.GetPrim(), include_children=True)

    async def test_obb_default(self) -> None:
        """Test obb default."""
        stage = get_current_stage()
        UsdGeom.Cube.Define(stage, "/cube_shape")

        await omni.kit.app.get_app().next_update_async()
        cache = create_bbox_cache()

        centroid, axes, half_extent = compute_obb(cache, "/cube_shape")
        centroid_expected = [0.0, 0.0, 0.0]
        for a, b in zip(centroid.tolist(), centroid_expected):
            self.assertAlmostEqual(a, b)
        axes_expected = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        for a, b in zip(axes.flatten().tolist(), axes_expected):
            self.assertAlmostEqual(a, b)
        half_extent_expected = [1.0, 1.0, 1.0]
        for a, b in zip(half_extent, half_extent_expected):
            self.assertAlmostEqual(a, b)

        corners = compute_obb_corners(cache, "/cube_shape")
        corners_expected = [
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            1.0,
            -1.0,
            1.0,
            -1.0,
            -1.0,
            1.0,
            1.0,
            1.0,
            -1.0,
            -1.0,
            1.0,
            -1.0,
            1.0,
            1.0,
            1.0,
            -1.0,
            1.0,
            1.0,
            1.0,
        ]
        for a, b in zip(corners.flatten().tolist(), corners_expected):
            self.assertAlmostEqual(a, b)

    async def test_obb_transformed(self) -> None:
        """Test obb transformed."""
        stage = get_current_stage()
        cubeGeom = UsdGeom.Cube.Define(stage, "/cube_shape")

        cubeGeom.CreateSizeAttr(3)
        # NOTE when authoring size, height, radius, etc. attributes, extents need to be manually updated
        recompute_extents(cubeGeom.GetPrim())
        cubeGeom.AddScaleOp().Set((1, -2, 1.5))
        cubeGeom.AddTranslateOp().Set((-1, 1.1, 5))
        cubeGeom.AddRotateXYZOp().Set((0, 45, 30))

        await omni.kit.app.get_app().next_update_async()
        cache = create_bbox_cache()

        centroid, axes, half_extent = compute_obb(cache, "/cube_shape")
        centroid_expected = [-1.0, -2.2, 7.5]
        for a, b in zip(centroid.tolist(), centroid_expected):
            self.assertAlmostEqual(a, b)
        axes_expected = [
            0.6123724356957945,
            -0.7071067811865474,
            -1.0606601717798214,
            -0.49999999999999994,
            -1.7320508075688774,
            0.0,
            0.6123724356957946,
            -0.7071067811865475,
            1.0606601717798212,
        ]
        for a, b in zip(axes.flatten().tolist(), axes_expected):
            self.assertAlmostEqual(a, b)

        half_extent_expected = [1.5, 1.5, 1.5]
        for a, b in zip(half_extent, half_extent_expected):
            self.assertAlmostEqual(a, b)

        corners = compute_obb_corners(cache, "/cube_shape")
        corners_expected = [
            -2.087117307087383,
            2.5193965549129578,
            7.5,
            -0.24999999999999978,
            0.3980762113533156,
            10.681980515339465,
            -3.587117307087383,
            -2.6767558677936742,
            7.5,
            -1.7499999999999998,
            -4.798076211353316,
            10.681980515339465,
            -0.2500000000000002,
            0.39807621135331606,
            4.318019484660535,
            1.5871173070873834,
            -1.7232441322063263,
            7.5,
            -1.75,
            -4.798076211353316,
            4.318019484660535,
            0.08711730708738363,
            -6.919396554912958,
            7.5,
        ]
        for a, b in zip(corners.flatten().tolist(), corners_expected):
            self.assertAlmostEqual(a, b)

    async def test_obb_nested(self) -> None:
        """Test obb nested."""
        stage = get_current_stage()
        cubeA = UsdGeom.Cube.Define(stage, "/nested_cube")
        cubeB = UsdGeom.Cube.Define(stage, "/nested_cube/nested_cube")
        cubeC = UsdGeom.Cube.Define(stage, "/nested_cube/nested_cube/nested_cube")

        # NOTE when authoring size, height, radius, etc. attributes, extents need to be manually updated
        cubeA.CreateSizeAttr(1)
        recompute_extents(cubeA.GetPrim())
        cubeA.AddScaleOp().Set((0.5, 0.5, -0.5))
        cubeA.AddTranslateOp().Set((-2, -2, -2))
        cubeA.AddRotateXYZOp().Set((30, 60, 90))

        cubeB.CreateSizeAttr(1.5)
        recompute_extents(cubeB.GetPrim())
        cubeB.AddScaleOp().Set((1, -1, 1))
        cubeB.AddTranslateOp().Set((1, 1, 1))
        cubeB.AddRotateXYZOp().Set((45, 0, 0))

        cubeC.CreateSizeAttr(3)
        recompute_extents(cubeC.GetPrim())
        cubeC.AddScaleOp().Set((2, 2, 2))
        cubeC.AddTranslateOp().Set((3, 3, 3))
        cubeC.AddRotateXYZOp().Set((0, 0, 0))

        await omni.kit.app.get_app().next_update_async()
        cache = create_bbox_cache()

        centroidA, axesA, half_extentA = compute_obb(cache, "/nested_cube")
        centroidA_expected = [-1.0, -1.0, 1.0]
        for a, b in zip(centroidA.tolist(), centroidA_expected):
            self.assertAlmostEqual(a, b)
        axesA_expected = [
            5.551115123125784e-17,
            0.25000000000000006,
            0.4330127018922193,
            -0.4330127018922193,
            0.2165063509461097,
            -0.125,
            0.25000000000000006,
            0.37499999999999994,
            -0.21650635094610973,
        ]
        for a, b in zip(axesA.flatten().tolist(), axesA_expected):
            self.assertAlmostEqual(a, b)
        half_extentA_expected = [0.5, 0.5, 0.5]
        for a, b in zip(half_extentA, half_extentA_expected):
            self.assertAlmostEqual(a, b)
        cornersA = compute_obb_corners(cache, "/nested_cube")
        cornersA_expected = [
            -0.9084936490538904,
            -1.420753175473055,
            0.9542468245269453,
            -0.6584936490538904,
            -1.045753175473055,
            0.7377404735808355,
            -1.3415063509461096,
            -1.204246824526945,
            0.8292468245269453,
            -1.0915063509461096,
            -0.8292468245269451,
            0.6127404735808355,
            -0.9084936490538904,
            -1.170753175473055,
            1.3872595264191645,
            -0.6584936490538904,
            -0.7957531754730549,
            1.1707531754730547,
            -1.3415063509461096,
            -0.9542468245269451,
            1.2622595264191645,
            -1.0915063509461096,
            -0.5792468245269451,
            1.0457531754730547,
        ]
        for a, b in zip(cornersA.flatten().tolist(), cornersA_expected):
            self.assertAlmostEqual(a, b)

        centroidB, axesB, half_extentB = compute_obb(cache, "/nested_cube/nested_cube")
        centroidB_expected = [-0.3169872981077806, -0.5915063509461097, 1.3415063509461096]
        for a, b in zip(centroidB.tolist(), centroidB_expected):
            self.assertAlmostEqual(a, b)
        axesB_expected = [
            5.551115123125784e-17,
            0.25000000000000006,
            0.4330127018922193,
            0.48296291314453416,
            0.11207193402100665,
            -0.06470476127563027,
            -0.1294095225512604,
            0.4182581518689039,
            -0.2414814565722671,
        ]
        for a, b in zip(axesB.flatten().tolist(), axesB_expected):
            self.assertAlmostEqual(a, b)
        half_extentB_expected = [0.75, 0.75, 0.75]
        for a, b in zip(half_extentB, half_extentB_expected):
            self.assertAlmostEqual(a, b)
        cornersB = compute_obb_corners(cache, "/nested_cube/nested_cube")
        cornersB_expected = [
            -0.5821523410527359,
            -1.1767539153635427,
            1.246386487912868,
            -0.7762666248796265,
            -0.5493666875601868,
            0.8841643030544675,
            0.1422920286640653,
            -1.0086460143320328,
            1.1493293459994227,
            -0.05182225516282532,
            -0.38125878652867695,
            0.7871071611410221,
            -0.5821523410527359,
            -0.8017539153635426,
            1.895905540751197,
            -0.7762666248796265,
            -0.1743666875601867,
            1.5336833558927965,
            0.1422920286640654,
            -0.6336460143320326,
            1.7988483988377517,
            -0.05182225516282521,
            -0.00625878652867673,
            1.436626213979351,
        ]
        for a, b in zip(cornersB.flatten().tolist(), cornersB_expected):
            self.assertAlmostEqual(a, b)

        centroidC, axesC, half_extentC = compute_obb(cache, "/nested_cube/nested_cube/nested_cube")
        centroidC_expected = [1.8043330454518622, 4.090474164393354, 2.1024652552120413]
        for a, b in zip(centroidC.tolist(), centroidC_expected):
            self.assertAlmostEqual(a, b)
        axesC_expected = [
            1.1102230246251568e-16,
            0.5000000000000001,
            0.8660254037844386,
            0.9659258262890683,
            0.2241438680420133,
            -0.12940952255126054,
            -0.2588190451025208,
            0.8365163037378078,
            -0.4829629131445342,
        ]
        for a, b in zip(axesC.flatten().tolist(), axesC_expected):
            self.assertAlmostEqual(a, b)
        half_extentC_expected = [1.5, 1.5, 1.5]
        for a, b in zip(half_extentC, half_extentC_expected):
            self.assertAlmostEqual(a, b)
        cornersC = compute_obb_corners(cache, "/nested_cube/nested_cube/nested_cube")
        cornersC_expected = [
            0.7436728736720407,
            1.749483906723623,
            1.7219858030790753,
            -0.03278426163552173,
            4.2590328179370465,
            0.2730970636454728,
            3.6414503525392456,
            2.4219155108496624,
            1.3337572354252938,
            2.8649932172316834,
            4.931464422063086,
            -0.11513150400830874,
            0.7436728736720412,
            3.249483906723623,
            4.320062014432391,
            -0.032784261635521283,
            5.7590328179370465,
            2.871173274998789,
            3.641450352539246,
            3.9219155108496624,
            3.93183344677861,
            2.864993217231684,
            6.431464422063086,
            2.4829447073450073,
        ]
        for a, b in zip(cornersC.flatten().tolist(), cornersC_expected):
            self.assertAlmostEqual(a, b)
