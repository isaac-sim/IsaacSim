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

"""Test mesh behavior."""

from typing import Any

import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import Mesh

from .fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

N = 5

# A single triangle: 3 vertices, 1 face.
TRIANGLE_POINTS = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
TRIANGLE_NORMALS = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]], dtype=np.float32)
TRIANGLE_VERTEX_INDICES = np.array([0, 1, 2], dtype=np.int32)
TRIANGLE_VERTEX_COUNTS = np.array([3], dtype=np.int32)


def _get_paths(stage: Any, populate: Any) -> Any:
    paths = [f"/World/Prim{i}" for i in range(N)]
    if populate:
        for path in paths:
            stage.define_prim(path, "Mesh")
        paths = "/World/Prim.*"
    return paths


def test_num_faces(capsys: Any, stage: Any) -> None:
    """Test num faces.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    prims = Mesh([f"/World/Prim{i}" for i in range(N)])
    prims.set_face_specs(
        vertex_indices=[TRIANGLE_VERTEX_INDICES],
        vertex_counts=[TRIANGLE_VERTEX_COUNTS],
    )
    assert prims.num_faces == [1] * N


def _get_display_colors(prims: Any, indices: Any = None) -> Any:
    """Read the authored display colors, one RGB triple per prim.

    The wrappers only write display colors, so the primvar is read back as a plain attribute: one array of
    colors per prim, which the setter always authors with a single entry.

    Args:
        prims: Wrapper to read from.
        indices: Indices of prims to read. If not defined, all wrapped prims are read.

    Returns:
        Display colors (shape ``(N, 3)``).
    """
    output = prims.get_attribute_values("primvars:displayColor", indices=indices)
    isaacsim_test.check_array(output, shape=(len(output), 1, 3), dtype=wp.float32)
    return output.numpy().reshape(-1, 3)


@pytest.mark.parametrize("populate", [True, False])
def test_points(capsys: Any, stage: Any, populate: Any) -> None:
    """Test points.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    prims.set_points([TRIANGLE_POINTS])
    output = prims.get_points()
    assert len(output) == N
    for arr in output:
        isaacsim_test.check_allclose(arr, TRIANGLE_POINTS)


@pytest.mark.parametrize("populate", [True, False])
def test_normals(capsys: Any, stage: Any, populate: Any) -> None:
    """Test normals.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    prims.set_normals([TRIANGLE_NORMALS])
    output = prims.get_normals()
    assert len(output) == N
    for arr in output:
        isaacsim_test.check_allclose(arr, TRIANGLE_NORMALS)


@pytest.mark.parametrize("populate", [True, False])
def test_face_specs(capsys: Any, stage: Any, populate: Any) -> None:
    """Test face specs.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    prims.set_face_specs(
        vertex_indices=[TRIANGLE_VERTEX_INDICES],
        vertex_counts=[TRIANGLE_VERTEX_COUNTS],
    )
    vertex_indices, vertex_counts, _, _ = prims.get_face_specs()
    assert len(vertex_indices) == N
    assert len(vertex_counts) == N
    for vi in vertex_indices:
        isaacsim_test.check_allclose(vi, TRIANGLE_VERTEX_INDICES)
    for vc in vertex_counts:
        isaacsim_test.check_allclose(vc, TRIANGLE_VERTEX_COUNTS)


@pytest.mark.parametrize("populate", [True, False])
def test_crease_specs(capsys: Any, stage: Any, populate: Any) -> None:
    """Test crease specs.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    crease_indices = np.array([0, 1], dtype=np.int32)
    crease_lengths = np.array([2], dtype=np.int32)
    crease_sharpnesses = np.array([1.0], dtype=np.float32)
    prims.set_crease_specs([crease_indices], [crease_lengths], [crease_sharpnesses])
    out_indices, out_lengths, out_sharpnesses = prims.get_crease_specs()
    assert len(out_indices) == N
    for ci in out_indices:
        isaacsim_test.check_allclose(ci, crease_indices)
    for cl in out_lengths:
        isaacsim_test.check_allclose(cl, crease_lengths)
    for cs in out_sharpnesses:
        isaacsim_test.check_allclose(cs, crease_sharpnesses)


@pytest.mark.parametrize("populate", [True, False])
def test_corner_specs(capsys: Any, stage: Any, populate: Any) -> None:
    """Test corner specs.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    corner_indices = np.array([0], dtype=np.int32)
    corner_sharpnesses = np.array([2.0], dtype=np.float32)
    prims.set_corner_specs([corner_indices], [corner_sharpnesses])
    out_indices, out_sharpnesses = prims.get_corner_specs()
    assert len(out_indices) == N
    for ci in out_indices:
        isaacsim_test.check_allclose(ci, corner_indices)
    for cs in out_sharpnesses:
        isaacsim_test.check_allclose(cs, corner_sharpnesses)


@pytest.mark.parametrize("populate", [True, False])
def test_subdivision_specs(capsys: Any, stage: Any, populate: Any) -> None:
    """Test subdivision specs.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    prims.set_subdivision_specs(subdivision_schemes="catmullClark")
    schemes, _, _ = prims.get_subdivision_specs()
    assert schemes == ["catmullClark"] * N


@pytest.mark.parametrize("populate", [True, False])
def test_display_colors(capsys: Any, stage: Any, populate: Any) -> None:
    """Test display colors.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    # a single color token is broadcast to all the prims
    prims.set_display_colors("#ff0000")
    isaacsim_test.check_allclose(np.tile([1.0, 0.0, 0.0], (N, 1)), _get_display_colors(prims))
    # numeric RGB triples, one per prim, and only for the selected prims
    values = np.linspace(0.0, 1.0, N * 3, dtype=np.float32).reshape(N, 3)
    prims.set_display_colors(values)
    isaacsim_test.check_allclose(values, _get_display_colors(prims))
    prims.set_display_colors("green", indices=[2])
    isaacsim_test.check_allclose(np.array([[0.0, 0.5019608, 0.0]]), _get_display_colors(prims, indices=[2]))
    # one color token per prim
    prims.set_display_colors(["#00ff00", "k", "tab:blue", "0.5", "aquamarine"])
    expected = np.array(
        [[0, 1, 0], [0, 0, 0], [0.12156863, 0.46666667, 0.70588235], [0.5, 0.5, 0.5], [0.49803922, 1, 0.83137255]]
    )
    isaacsim_test.check_allclose(expected, _get_display_colors(prims))
    # a single grayscale component is broadcast to the three channels, one per prim or shared by all of them
    prims.set_display_colors(0.25)
    isaacsim_test.check_allclose(np.full((N, 3), 0.25), _get_display_colors(prims))
    grays = np.linspace(0.0, 1.0, N, dtype=np.float32).reshape(N, 1)
    prims.set_display_colors(grays)
    isaacsim_test.check_allclose(np.tile(grays, (1, 3)), _get_display_colors(prims))
    # colors that cannot be broadcast to one RGB triple per prim are rejected
    with pytest.raises(ValueError):
        prims.set_display_colors(["red", "green"])
    with pytest.raises(ValueError):
        prims.set_display_colors(np.zeros((3, N), dtype=np.float32))
    # unrecognized color tokens are rejected
    with pytest.raises(ValueError):
        prims.set_display_colors("not-a-color")
    # an empty selection has no prim to color
    with pytest.raises(ValueError):
        prims.set_display_colors("red", indices=[])


@pytest.mark.parametrize("populate", [True, False])
def test_update_extents(capsys: Any, stage: Any, populate: Any) -> None:
    """Test update extents.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = Mesh(_get_paths(stage, populate))
    prims.set_points([TRIANGLE_POINTS])
    prims.update_extents()


def test_are_of_type(capsys: Any, stage: Any) -> None:
    """Test are of type.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    type_names = ["Mesh", "Cube", "Scope"]
    for index, type_name in enumerate(type_names):
        stage.define_prim(f"/World/Prim{index}", type_name)
    paths = [f"/World/Prim{index}" for index in range(len(type_names))]
    # boolean flags are reported per prim, in the order the paths were given
    output = Mesh.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(len(type_names), 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([1, 0, 0], dtype=np.bool_).reshape(-1, 1), output)
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(Mesh.are_of_type("/World/Prim.*"), shape=(len(type_names), 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        Mesh.are_of_type("/World/NonExistent")


def test_legacy_display_color_getter(stage: Any) -> None:
    """Preserve the M1 placeholder call with and without the indices keyword.

    Args:
        stage: Stage used by the test.
    """
    prims = Mesh(["/World/LegacyColor"])
    for result in (prims.get_display_colors(), prims.get_display_colors(indices=[0])):
        assert isinstance(result, np.ndarray)
        assert result.shape == ()
        assert result.dtype == np.dtype(np.int32)
        np.testing.assert_array_equal(result, np.asarray(0, dtype=np.int32))
    del prims
    result[...] = 7
    assert result.item() == 7
