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

"""Test merge mesh utils functionality."""

import carb
import omni
import omni.kit.test
from isaacsim.asset.importer.utils.impl import importer_utils, merge_mesh_utils
from pxr import Usd, UsdGeom


class TestMergeMeshUtils(omni.kit.test.AsyncTestCase):
    """Test helpers in :mod:`isaacsim.asset.importer.utils.impl.merge_mesh_utils`."""

    async def test_merge_mesh_reduces_mesh_count(self) -> None:
        """Merge multiple meshes when Scene Optimizer is available."""
        try:
            import omni.scene.optimizer.core  # noqa: F401
        except ImportError:
            self.skipTest("Scene Optimizer modules are not available")

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")

        def define_cube_mesh(path: str) -> UsdGeom.Mesh:
            mesh = UsdGeom.Mesh.Define(stage, path)
            points = [
                (-0.5, -0.5, -0.5),
                (0.5, -0.5, -0.5),
                (0.5, 0.5, -0.5),
                (-0.5, 0.5, -0.5),
                (-0.5, -0.5, 0.5),
                (0.5, -0.5, 0.5),
                (0.5, 0.5, 0.5),
                (-0.5, 0.5, 0.5),
            ]
            face_vertex_counts = [4, 4, 4, 4, 4, 4]
            face_vertex_indices = [
                0,
                1,
                2,
                3,
                4,
                5,
                6,
                7,
                0,
                1,
                5,
                4,
                2,
                3,
                7,
                6,
                1,
                2,
                6,
                5,
                3,
                0,
                4,
                7,
            ]
            mesh.CreatePointsAttr(points)
            mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
            mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
            mesh.CreateSubdivisionSchemeAttr("none")
            return mesh

        define_cube_mesh("/World/A")
        define_cube_mesh("/World/B")
        define_cube_mesh("/World/C")
        UsdGeom.Cube.Define(stage, "/World/Cube")

        await omni.kit.app.get_app().next_update_async()

        def mesh_count() -> int:
            return sum(1 for prim in stage.Traverse() if prim.GetTypeName() in importer_utils.USD_GEOMETRY_TYPES)

        before_count = mesh_count()
        self.assertEqual(before_count, 4)
        self.assertTrue(stage.GetPrimAtPath("/World/Cube").IsValid())

        merge_mesh_utils.clean_mesh_operation(stage)
        merge_mesh_utils.generate_mesh_uv_normals_operation(stage)
        merge_mesh_utils.merge_mesh(stage, ["/World/A", "/World/B", "/World/C", "/World/Cube"])

        await omni.kit.app.get_app().next_update_async()

        after_count = mesh_count()
        # meshes are merged together, cube is separate
        carb.log_info(f"before_count: {before_count}, after_count: {after_count}")
        self.assertEqual(after_count, 2)
