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

"""Validate Mesh wrapper access to authored geometry and display attributes.

The suite wraps USD mesh prims, verifies collection and geom metadata, and
round-trips variable-length points, normals, face topology, crease and corner
data, subdivision settings, and display colors through indexed get/set calls.
"""

from typing import Any, Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.commands
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Mesh
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
)
from pxr import UsdGeom


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs: Any) -> None:
    """Create a fresh stage and author existing mesh prims for wrap-mode tests.

    Args:
        max_num_prims: Maximum number of prims to prepare on the stage.
        operation: Operation mode selected by parametrization.
        **kwargs: Additional arguments supplied by parametrization.
    """
    # create new stage
    await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            omni.kit.commands.execute(
                "CreateMeshPrimWithDefaultXformCommand", prim_type="Sphere", prim_path=f"/World/A_{i}"
            )


class TestMesh(omni.kit.test.AsyncTestCase):
    """Exercise Mesh geometry, topology, subdivision, and color APIs."""

    async def setUp(self) -> None:
        """Initialize the async fixture; parametrized cases create their own stages."""
        super().setUp()

    async def tearDown(self) -> None:
        """Finalize the async fixture without additional mesh cleanup."""
        super().tearDown()

    # --------------------------------------------------------------------

    def custom_sample(self, *, num_prims: Any, batch_range: Any, data_shape: Any, dtype: Any) -> Any:
        """Build per-prim variable-length geometry samples in supported input types.

        Args:
            num_prims: Number of prims in the parametrized collection.
            batch_range: Range used to choose per-prim sample counts.
            data_shape: Shape appended to each generated sample.
            dtype: Data type used for generated samples.

        Returns:
            Generated input values and expected values for each supported input form.
        """
        data = []
        for index in [2, 5, 8]:  # full shape list, np.ndarray, wp.array
            values, expected_values = [], []
            # generate a different set of values (with different batch size) for each prim
            for _ in range(num_prims):
                shape = (np.random.randint(low=batch_range[0], high=batch_range[1]), *data_shape)
                _samples = draw_sample(shape=shape, dtype=dtype)
                values.append(_samples[index][0])
                expected_values.append(_samples[index][1])
            data.append((values, expected_values))
        return data

    def custom_face_test_set(self, num_prims: Any, num_points: Any) -> Any:
        """Build valid face vertex, count, and hole-index inputs for mesh tests.

        Args:
            num_prims: Number of prims in the parametrized collection.
            num_points: Number of points available for generated topology.

        Returns:
            Generated face vertex indices, vertex counts, and hole indices with expected values.
        """
        data = []
        for _type in [list, np.ndarray, wp.array]:
            vertex_indices, vertex_counts, hole_indices = [], [], []
            expected_vertex_indices, expected_vertex_counts, expected_hole_indices = [], [], []
            for _ in range(num_prims):
                num_faces = np.random.randint(low=5, high=10)
                # draw random values
                counts = np.random.randint(low=2, high=5, size=num_faces).astype(np.int32)
                indices = np.random.randint(low=0, high=num_points, size=int(np.sum(counts))).astype(np.int32)
                h_indices = np.random.randint(
                    low=0, high=num_faces, size=np.random.randint(low=0, high=num_faces)
                ).astype(np.int32)
                # convert to expected type
                if _type == list:
                    vertex_indices.append(indices.tolist())
                    vertex_counts.append(counts.tolist())
                    hole_indices.append(h_indices.tolist())
                elif _type == np.ndarray:
                    vertex_indices.append(indices.copy())
                    vertex_counts.append(counts.copy())
                    hole_indices.append(h_indices.copy())
                elif _type == wp.array:
                    vertex_indices.append(wp.array(indices))
                    vertex_counts.append(wp.array(counts))
                    hole_indices.append(wp.array(h_indices))
                expected_vertex_indices.append(indices)
                expected_vertex_counts.append(counts)
                expected_hole_indices.append(h_indices)
            data.append(
                (
                    (vertex_indices, expected_vertex_indices),
                    (vertex_counts, expected_vertex_counts),
                    (hole_indices, expected_hole_indices),
                )
            )
        return data

    def custom_crease_test_set(self, num_prims: Any, num_points: Any) -> Any:
        """Build valid crease index, length, and sharpness inputs for mesh tests.

        Args:
            num_prims: Number of prims in the parametrized collection.
            num_points: Number of points available for generated topology.

        Returns:
            Generated crease indices, crease lengths, and sharpnesses with expected values.
        """
        data = []
        for _type in [list, np.ndarray, wp.array]:
            crease_indices, crease_lengths, crease_sharpnesses = [], [], []
            expected_crease_indices, expected_crease_lengths, expected_crease_sharpnesses = [], [], []
            for _ in range(num_prims):
                num_creases = np.random.randint(low=5, high=10)
                # draw random values
                lengths = np.random.randint(low=2, high=5, size=num_creases).astype(np.int32)
                indices = np.random.randint(low=0, high=num_points, size=int(np.sum(lengths))).astype(np.int32)
                if np.random.rand() < 0.5:
                    sharpnesses = np.random.rand(num_creases).astype(np.float32)
                else:
                    sharpnesses = np.random.rand(int(np.sum(lengths - 1))).astype(np.float32)
                # convert to expected type
                if _type == list:
                    crease_indices.append(indices.tolist())
                    crease_lengths.append(lengths.tolist())
                    crease_sharpnesses.append(sharpnesses.tolist())
                elif _type == np.ndarray:
                    crease_indices.append(indices.copy())
                    crease_lengths.append(lengths.copy())
                    crease_sharpnesses.append(sharpnesses.copy())
                elif _type == wp.array:
                    crease_indices.append(wp.array(indices))
                    crease_lengths.append(wp.array(lengths))
                    crease_sharpnesses.append(wp.array(sharpnesses))
                expected_crease_indices.append(indices)
                expected_crease_lengths.append(lengths)
                expected_crease_sharpnesses.append(sharpnesses)
            data.append(
                (
                    (crease_indices, expected_crease_indices),
                    (crease_lengths, expected_crease_lengths),
                    (crease_sharpnesses, expected_crease_sharpnesses),
                )
            )
        return data

    def custom_corner_test_set(self, num_prims: Any, num_points: Any) -> Any:
        """Build valid corner index and sharpness inputs for mesh tests.

        Args:
            num_prims: Number of prims in the parametrized collection.
            num_points: Number of points available for generated topology.

        Returns:
            Generated corner indices and sharpnesses with expected values.
        """
        data = []
        for _type in [list, np.ndarray, wp.array]:
            corner_indices, corner_sharpnesses = [], []
            expected_corner_indices, expected_corner_sharpnesses = [], []
            for _ in range(num_prims):
                num_corners = np.random.randint(low=5, high=10)
                # draw random values
                indices = np.random.randint(low=0, high=num_points, size=num_corners).astype(np.int32)
                sharpnesses = np.random.rand(num_corners).astype(np.float32)
                # convert to expected type
                if _type == list:
                    corner_indices.append(indices.tolist())
                    corner_sharpnesses.append(sharpnesses.tolist())
                elif _type == np.ndarray:
                    corner_indices.append(indices.copy())
                    corner_sharpnesses.append(sharpnesses.copy())
                elif _type == wp.array:
                    corner_indices.append(wp.array(indices))
                    corner_sharpnesses.append(wp.array(sharpnesses))
                expected_corner_indices.append(indices)
                expected_corner_sharpnesses.append(sharpnesses)
            data.append(
                (
                    (corner_indices, expected_corner_indices),
                    (corner_sharpnesses, expected_corner_sharpnesses),
                )
            )
        return data

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_properties_and_getters(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test properties and getters.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        # test cases (properties)
        # - geoms
        self.assertEqual(len(prim.geoms), num_prims, f"Invalid geoms len ({num_prims} prims)")
        for usd_prim, geom in zip(prim.prims, prim.geoms):
            self.assertTrue(geom.GetPrim().IsA(UsdGeom.Mesh), f"Invalid geom type: {geom.GetPrim().GetTypeName()}")
            self.assertTrue(
                usd_prim.IsValid() and usd_prim.IsA(UsdGeom.Mesh), f"Invalid prim type: {usd_prim.GetTypeName()}"
            )
        # - num_faces
        self.assertEqual(len(prim.num_faces), num_prims, f"Invalid num_faces len ({num_prims} prims)")
        for geom, num_faces in zip(prim.geoms, prim.num_faces):
            self.assertEqual(geom.GetFaceCount(), num_faces, f"Invalid num_faces: {geom.GetFaceCount()}")

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_points(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test points.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in self.custom_sample(
                num_prims=expected_count, batch_range=(5, 10), data_shape=(3,), dtype=wp.float32
            ):
                prim.set_points(v0, indices=indices)
                output = prim.get_points(indices=indices)
                for i in range(len(expected_v0)):
                    check_array(output[i], shape=(expected_v0[i].shape[0], 3), dtype=wp.float32, device=device)
                    check_allclose(expected_v0[i], output[i], given=(v0[i],))

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_normals(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test normals.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in self.custom_sample(
                num_prims=expected_count, batch_range=(5, 10), data_shape=(3,), dtype=wp.float32
            ):
                prim.set_normals(v0, indices=indices)
                output = prim.get_normals(indices=indices)
                for i in range(len(expected_v0)):
                    check_array(output[i], shape=(expected_v0[i].shape[0], 3), dtype=wp.float32, device=device)
                    check_allclose(expected_v0[i], output[i], given=(v0[i],))

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_face_specs(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test face specs.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        num_points = prim.get_points()[0].shape[0]
        if not num_points:  # empty mesh
            num_points = 50
        varying_linear_interp_choices = ["none", "cornersOnly", "cornersPlus1", "cornersPlus2", "boundaries", "all"]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for ((v0, expected_v0), (v1, expected_v1), (v2, expected_v2)), (vv, expected_vv) in zip(
                self.custom_face_test_set(num_prims=expected_count, num_points=num_points),
                draw_choice(shape=(expected_count,), choices=varying_linear_interp_choices),
            ):
                prim.set_face_specs(v0, v1, vv, v2, indices=indices)
                output = prim.get_face_specs(indices=indices)
                check_lists(expected_vv, output[2])  # varying_linear_interpolations is index 2
                for i in range(len(expected_v0)):  # hole_indices is index 3
                    check_array(output[0][i], shape=(expected_v0[i].shape[0],), dtype=wp.int32, device=device)
                    check_array(output[1][i], shape=(expected_v1[i].shape[0],), dtype=wp.int32, device=device)
                    check_array(output[3][i], shape=(expected_v2[i].shape[0],), dtype=wp.int32, device=device)
                    check_allclose(
                        (expected_v0[i], expected_v1[i], expected_v2[i]),
                        (output[0][i], output[1][i], output[3][i]),
                        given=(v0[i], v1[i], v2[i]),
                    )

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_crease_specs(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test crease specs.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        num_points = prim.get_points()[0].shape[0]
        if not num_points:  # empty mesh
            num_points = 50
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1), (v2, expected_v2) in self.custom_crease_test_set(
                num_prims=expected_count, num_points=num_points
            ):
                prim.set_crease_specs(v0, v1, v2, indices=indices)
                output = prim.get_crease_specs(indices=indices)
                for i in range(len(expected_v0)):
                    check_array(output[0][i], shape=(expected_v0[i].shape[0],), dtype=wp.int32, device=device)
                    check_array(output[1][i], shape=(expected_v1[i].shape[0],), dtype=wp.int32, device=device)
                    check_array(output[2][i], shape=(expected_v2[i].shape[0],), dtype=wp.float32, device=device)
                    check_allclose(
                        (expected_v0[i], expected_v1[i], expected_v2[i]),
                        (output[0][i], output[1][i], output[2][i]),
                        given=(v0[i], v1[i], v2[i]),
                    )

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_corner_specs(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test corner specs.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        num_points = prim.get_points()[0].shape[0]
        if not num_points:  # empty mesh
            num_points = 50
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in self.custom_corner_test_set(
                num_prims=expected_count, num_points=num_points
            ):
                prim.set_corner_specs(v0, v1, indices=indices)
                output = prim.get_corner_specs(indices=indices)
                for i in range(len(expected_v0)):
                    check_array(output[0][i], shape=(expected_v0[i].shape[0],), dtype=wp.int32, device=device)
                    check_array(output[1][i], shape=(expected_v0[i].shape[0],), dtype=wp.float32, device=device)
                    check_allclose((expected_v0[i], expected_v1[i]), (output[0][i], output[1][i]), given=(v0[i], v1[i]))

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_subdivision_specs(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test subdivision specs.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        subdivision_scheme_choices = ["catmullClark", "loop", "bilinear", "none"]
        interpolate_boundary_choices = ["none", "edgeOnly", "edgeAndCorner"]
        triangle_subdivision_rule_choices = ["catmullClark", "smooth"]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1), (v2, expected_v2) in zip(
                draw_choice(shape=(expected_count,), choices=subdivision_scheme_choices),
                draw_choice(shape=(expected_count,), choices=interpolate_boundary_choices),
                draw_choice(shape=(expected_count,), choices=triangle_subdivision_rule_choices),
            ):
                prim.set_subdivision_specs(v0, v1, v2, indices=indices)
                output = prim.get_subdivision_specs(indices=indices)
                check_lists(expected_v0, output[0])
                check_lists(expected_v1, output[1])
                check_lists(expected_v2, output[2])

    @parametrize(backends=["usd"], prim_class=Mesh, populate_stage_func=populate_stage)
    async def test_display_colors(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test display colors.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        choices = [
            (0.1, 0.2, 0.3),  # RGB tuple
            "#aBc",  # case-insensitive short hex RGB
            "#0A1b2C",  # case-insensitive hex RGB
            "0.5",  # grayscale
            "k",  # basic color
            "AquaMarine",  # case-insensitive X11/CSS4 color with no spaces
            "xkcd:eggShell",  # case-insensitive  xkcd color
            "tab:Green",  # case-insensitive tableau color
            "C2",  # CN color specification
            "none",  # special value (fully transparent)
        ]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=choices):
                prim.set_display_colors(v0, indices=indices)
