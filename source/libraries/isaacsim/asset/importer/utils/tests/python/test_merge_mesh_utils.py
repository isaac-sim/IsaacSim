# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify optimizer failure propagation and standalone mesh merging."""

from __future__ import annotations

import unittest
from unittest.mock import Mock, patch

from isaacsim.asset.importer.utils.impl import merge_mesh_utils
from pxr import Usd, UsdGeom, UsdPhysics


class TestMergeMeshUtils(unittest.TestCase):
    """Exercise importer mesh operations on real USD stages."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        body = UsdGeom.Xform.Define(self.stage, "/Body")
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        self.mesh_paths = ["/Body/A", "/Body/B"]
        self.points = []
        for index, path in enumerate(self.mesh_paths):
            points = [(index * 2, 0, 0), (index * 2 + 1, 0, 0), (index * 2, 1, 0)]
            self.points.extend(points)
            mesh = UsdGeom.Mesh.Define(self.stage, path)
            mesh.CreatePointsAttr(points)
            mesh.CreateFaceVertexCountsAttr([3])
            mesh.CreateFaceVertexIndicesAttr([0, 1, 2])

    def test_optimizer_failures_include_operation_and_message(self) -> None:
        """Every reported optimizer failure must become an actionable exception."""
        message = "ABI mismatch: expected USD stage type"
        cases = [
            ("meshCleanup", merge_mesh_utils.clean_mesh_operation, [self.stage], [(False, message, None)]),
            (
                "generateNormals",
                merge_mesh_utils.generate_mesh_uv_normals_operation,
                [self.stage],
                [(False, message, None)],
            ),
            (
                "generateProjectionUVs",
                merge_mesh_utils.generate_mesh_uv_normals_operation,
                [self.stage],
                [(True, None, None), (False, message, None)],
            ),
            ("merge", merge_mesh_utils.merge_mesh, [self.stage, self.mesh_paths], [(False, message, None)]),
        ]
        for operation, function, arguments, results in cases:
            with self.subTest(operation=operation):
                core = Mock()
                core.executeOperation.side_effect = results
                with patch.object(merge_mesh_utils, "_create_optimizer_context", return_value=(core, object())):
                    with self.assertRaises(RuntimeError) as error:
                        function(*arguments)
                self.assertIn(operation, str(error.exception))
                self.assertIn(message, str(error.exception))
                self.assertEqual(core.executeOperation.call_count, len(results))
                self.assertTrue(all(self.stage.GetPrimAtPath(path) for path in self.mesh_paths))

    def test_failed_normals_stops_uv_generation(self) -> None:
        """Do not run UV generation after normal generation fails."""
        core = Mock()
        core.executeOperation.return_value = (False, "Cannot generate normals", None)
        with patch.object(merge_mesh_utils, "_create_optimizer_context", return_value=(core, object())):
            with self.assertRaisesRegex(RuntimeError, "generateNormals.*Cannot generate normals"):
                merge_mesh_utils.generate_mesh_uv_normals_operation(self.stage)
        self.assertEqual([call.args[0] for call in core.executeOperation.call_args_list], ["generateNormals"])

    def test_successful_operation_sequence(self) -> None:
        """Successful cleanup and normal/UV operations preserve execution order and stage context."""
        core = Mock()
        core.executeOperation.return_value = (True, None, None)
        context = object()
        with patch.object(
            merge_mesh_utils, "_create_optimizer_context", return_value=(core, context)
        ) as create_context:
            self.assertIsNone(merge_mesh_utils.clean_mesh_operation(self.stage))
            self.assertIsNone(merge_mesh_utils.generate_mesh_uv_normals_operation(self.stage))
        self.assertEqual(
            [call.args[0] for call in core.executeOperation.call_args_list],
            ["meshCleanup", "generateNormals", "generateProjectionUVs"],
        )
        self.assertTrue(all(call.args[1] is context for call in core.executeOperation.call_args_list))
        self.assertEqual(create_context.call_count, 2)
        create_context.assert_called_with(self.stage)

    def test_missing_optimizer_raises(self) -> None:
        """An unavailable optimizer cannot be reported as a successful operation."""
        cases = [
            (merge_mesh_utils.clean_mesh_operation, [self.stage]),
            (merge_mesh_utils.generate_mesh_uv_normals_operation, [self.stage]),
            (merge_mesh_utils.merge_mesh, [self.stage, self.mesh_paths]),
            (merge_mesh_utils.merge_meshes_operation, [self.stage]),
        ]
        with patch.object(merge_mesh_utils, "_HAS_USD_OPTIMIZE", False):
            for function, arguments in cases:
                with self.subTest(operation=function.__name__):
                    with self.assertRaisesRegex(RuntimeError, "usd_optimize is not installed"):
                        function(*arguments)

    def test_failed_group_merge_does_not_return_success_count(self) -> None:
        """A failed merge must not return the number of planned mesh groups."""
        core = Mock()
        core.executeOperation.return_value = (False, "Stage rejected", None)
        with patch.object(merge_mesh_utils, "_create_optimizer_context", return_value=(core, object())):
            with self.assertRaisesRegex(RuntimeError, "merge.*Stage rejected"):
                merge_mesh_utils.merge_meshes_operation(self.stage)
        core.executeOperation.assert_called_once()
        self.assertEqual(set(core.executeOperation.call_args.args[2]["meshPrimPaths"]), set(self.mesh_paths))

    def test_empty_mesh_list_is_no_op_without_optimizer(self) -> None:
        """An empty list needs no optimizer and performs no work."""
        with patch.object(merge_mesh_utils, "_HAS_USD_OPTIMIZE", False):
            with patch.object(merge_mesh_utils, "_create_optimizer_context") as create_context:
                self.assertIsNone(merge_mesh_utils.merge_mesh(self.stage, []))
                create_context.assert_not_called()

    def test_no_mesh_groups_returns_zero_without_optimizer(self) -> None:
        """A stage with no eligible meshes requires no optimizer."""
        with patch.object(merge_mesh_utils, "_HAS_USD_OPTIMIZE", False):
            self.assertEqual(merge_mesh_utils.merge_meshes_operation(Usd.Stage.CreateInMemory()), 0)

    def test_successful_merge_authors_combined_geometry(self) -> None:
        """The standalone optimizer must really merge triangles and preserve their points."""
        self.assertEqual(merge_mesh_utils.merge_meshes_operation(self.stage), 1)
        meshes = [UsdGeom.Mesh(prim) for prim in self.stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
        self.assertEqual(len(meshes), 1)
        self.assertEqual(sorted(tuple(point) for point in meshes[0].GetPointsAttr().Get()), sorted(self.points))
        self.assertEqual(list(meshes[0].GetFaceVertexCountsAttr().Get()), [3, 3])
        indices = list(meshes[0].GetFaceVertexIndicesAttr().Get())
        self.assertEqual(len(indices), 6)
        self.assertEqual(set(indices), set(range(6)))
        self.assertFalse(any(self.stage.GetPrimAtPath(path) for path in self.mesh_paths))


if __name__ == "__main__":
    unittest.main()
