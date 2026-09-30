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

"""Test ground plane behavior."""

import itertools
from typing import Any

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import Mesh, Plane
from isaacsim.foundation.prims import GroundPlane

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

N = 5

# Orientations (quaternion xyzw) that map the mesh +Z normal onto each supported axis.
AXIS_ORIENTATIONS = {
    "X": [0.0, 0.70710678, 0.0, 0.70710678],
    "Y": [-0.70710678, 0.0, 0.0, 0.70710678],
    "Z": [0.0, 0.0, 0.0, 1.0],
}

"""
Utility functions.
"""

_group_counter = itertools.count()


def _get_new_paths() -> Any:
    """Get paths to prims that do not exist yet.

    Each call returns a fresh group of paths, since the stage fixture is shared across the
    examples generated for a single test case, and creation only happens on nonexistent paths.

    Returns:
        The resulting value.
    """
    group = next(_group_counter)
    return [f"/World/Group{group}/Prim{i}" for i in range(N)]


def _match_quaternion_sign(reference: Any, quats: Any) -> Any:
    """Flip quaternions (per row) whose sign is the antipodal equivalent of the reference.

    Quaternions ``q`` and ``-q`` represent the same rotation, so a plain element-wise
    comparison against a reference orientation is only valid once signs are aligned.

    Args:
        reference: Reference quaternions, shape ``(N, 4)``.
        quats: Quaternions to align to the reference sign, shape ``(N, 4)``.

    Returns:
        The resulting value.
    """
    signs = np.sign(np.sum(np.asarray(reference) * np.asarray(quats), axis=-1, keepdims=True))
    signs[signs == 0] = 1.0
    return np.asarray(quats) * signs


def _get_paths(stage: Any, populate: Any) -> Any:
    """Get the paths to wrap or create.

    The children of the populated prims are intentionally not named ``Plane`` and ``Mesh``,
    since ground planes authored by other tools are matched by USD type rather than by name.

    Args:
        stage: Stage used by the test.
        populate: Whether to populate the test stage.

    Returns:
        The resulting value.
    """
    paths = _get_new_paths()
    if populate:
        for path in paths:
            stage.define_prim(path, "Xform")
            stage.define_prim(f"{path}/Collision", "Plane")
            stage.define_prim(f"{path}/Render", "Mesh")
    return paths


"""
Test cases.
"""


@pytest.mark.parametrize("populate", [True, False])
def test_composite_structure(capsys: Any, stage: Any, populate: Any) -> None:
    """Test composite structure.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = GroundPlane(_get_paths(stage, populate))
    assert len(prims.paths) == N
    isaacsim_test.check_equal(prims.get_type_name(), ["Xform"] * N)
    # children wrappers
    assert isinstance(prims.planes, Plane)
    assert isinstance(prims.meshes, Mesh)
    isaacsim_test.check_equal(prims.planes.get_type_name(), ["Plane"] * N)
    isaacsim_test.check_equal(prims.meshes.get_type_name(), ["Mesh"] * N)
    # children are wrapped in the order of the ground planes
    for path, plane_path, mesh_path in zip(prims.paths, prims.planes.paths, prims.meshes.paths):
        assert plane_path.startswith(f"{path}/")
        assert mesh_path.startswith(f"{path}/")


def test_wrap_missing_children(capsys: Any, stage: Any) -> None:
    """Test wrapping prims without the required children.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    # single child
    stage.define_prim("/World/OneChild", "Xform")
    stage.define_prim("/World/OneChild/Collision", "Plane")
    with pytest.raises(ValueError):
        GroundPlane("/World/OneChild")
    # two children, no Mesh
    stage.define_prim("/World/NoMesh", "Xform")
    stage.define_prim("/World/NoMesh/Collision", "Plane")
    stage.define_prim("/World/NoMesh/Other", "Cube")
    with pytest.raises(ValueError):
        GroundPlane("/World/NoMesh")
    # two children, no Plane
    stage.define_prim("/World/NoPlane", "Xform")
    stage.define_prim("/World/NoPlane/Render", "Mesh")
    stage.define_prim("/World/NoPlane/Other", "Cube")
    with pytest.raises(ValueError):
        GroundPlane("/World/NoPlane")


@hypothesis.given(
    sizes=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(N, 1), elements=isaacsim_test.finite_float_elements(min_value=0.001, max_value=1000.0)
    ),
    axes=hypothesis.strategies.lists(hypothesis.strategies.sampled_from(["X", "Y", "Z"]), min_size=N, max_size=N),
    colors=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(N, 3), elements=isaacsim_test.finite_float_elements(min_value=0.0, max_value=1.0)
    ),
)
def test_creation(capsys: Any, stage: Any, sizes: Any, axes: Any, colors: Any) -> None:
    """Test creation.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        sizes: Sizes values.
        axes: Axes values.
        colors: Colors values.
    """
    # default values: size 100.0 (full extent, in stage units) and stage up-axis
    prims = GroundPlane(_get_paths(stage, populate=False))
    isaacsim_test.check_allclose(np.full((N, 1), 100.0), prims.planes.get_widths())
    isaacsim_test.check_allclose(np.full((N, 1), 100.0), prims.planes.get_lengths())
    isaacsim_test.check_equal(prims.planes.get_axes(), [stage.get_up_axis()] * N)
    # the Plane children drive collision only, so they are authored as guides
    isaacsim_test.check_equal(prims.planes.get_attribute_values("purpose"), ["guide"] * N)
    assert all(prims.planes.has_api("PhysicsCollisionAPI").numpy())
    # given values
    prims = GroundPlane(_get_paths(stage, populate=False), sizes=sizes, axes=axes, colors=colors)
    isaacsim_test.check_allclose(sizes, prims.planes.get_widths())
    isaacsim_test.check_allclose(sizes, prims.planes.get_lengths())
    isaacsim_test.check_allclose(np.tile(sizes, (1, 3)), prims.meshes.get_local_scales())
    isaacsim_test.check_equal(prims.planes.get_axes(), axes)
    expected_orientations = np.array([AXIS_ORIENTATIONS[axis] for axis in axes], dtype=np.float32)
    actual_orientations = prims.meshes.get_local_poses()[1]
    isaacsim_test.check_allclose(
        expected_orientations, _match_quaternion_sign(expected_orientations, actual_orientations)
    )
    # single axis and color token, applied to all the ground planes
    prims = GroundPlane(_get_paths(stage, populate=False), axes="Y", colors="red")
    isaacsim_test.check_equal(prims.planes.get_axes(), ["Y"] * N)
    expected_y_orientations = np.tile(AXIS_ORIENTATIONS["Y"], (N, 1))
    actual_y_orientations = prims.meshes.get_local_poses()[1]
    isaacsim_test.check_allclose(
        expected_y_orientations, _match_quaternion_sign(expected_y_orientations, actual_y_orientations)
    )
    # unsupported axis
    with pytest.raises(ValueError):
        GroundPlane(_get_paths(stage, populate=False), axes="W")


@hypothesis.given(
    contact_offsets=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(N, 1), elements=isaacsim_test.finite_float_elements()
    ),
    rest_offsets=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(N, 1), elements=isaacsim_test.finite_float_elements()
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_offsets(capsys: Any, stage: Any, populate: Any, contact_offsets: Any, rest_offsets: Any) -> None:
    """Test offsets.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        contact_offsets: Contact offsets values.
        rest_offsets: Rest offsets values.
    """
    prims = GroundPlane(_get_paths(stage, populate))
    # get values
    output_contact_offsets, output_rest_offsets = prims.get_offsets()
    isaacsim_test.check_array(output_contact_offsets, shape=(N, 1), dtype=wp.float32)
    isaacsim_test.check_array(output_rest_offsets, shape=(N, 1), dtype=wp.float32)
    # round-trip values
    prims.set_offsets(contact_offsets, rest_offsets)
    output_contact_offsets, output_rest_offsets = prims.get_offsets()
    isaacsim_test.check_array(output_contact_offsets, shape=(N, 1), dtype=wp.float32)
    isaacsim_test.check_array(output_rest_offsets, shape=(N, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(contact_offsets, output_contact_offsets)
    isaacsim_test.check_allclose(rest_offsets, output_rest_offsets)


@hypothesis.given(
    radii=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(N, 1), elements=isaacsim_test.finite_float_elements()),
)
@pytest.mark.parametrize("minimum", [True, False])
@pytest.mark.parametrize("populate", [True, False])
def test_torsional_patch_radii(capsys: Any, stage: Any, populate: Any, minimum: Any, radii: Any) -> None:
    """Test torsional patch radii.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        minimum: Whether to process the minimum torsional patch radii.
        radii: Radii values.
    """
    prims = GroundPlane(_get_paths(stage, populate))
    # get values
    output = prims.get_torsional_patch_radii(minimum=minimum)
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.float32)
    # round-trip values
    prims.set_torsional_patch_radii(radii, minimum=minimum)
    output = prims.get_torsional_patch_radii(minimum=minimum)
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(radii, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(N, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_enabled_collisions(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test enabled collisions.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = GroundPlane(_get_paths(stage, populate))
    # get values
    output = prims.get_enabled_collisions()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_collisions(values)
    output = prims.get_enabled_collisions()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


def test_are_of_type(stage: Any) -> None:
    """Test are of type.

    Args:
        stage: Stage used by the test.
    """
    # a ground plane is identified by its composite structure (Plane + Mesh children), not by a schema
    ground_plane_path = _get_new_paths()[0]
    GroundPlane(ground_plane_path)
    # a plain Xform has no children, and an Xform with unrelated children does not match either
    stage.define_prim("/World/Empty", "Xform")
    stage.define_prim("/World/Wrong", "Xform")
    stage.define_prim("/World/Wrong/A", "Cube")
    stage.define_prim("/World/Wrong/B", "Cube")
    # the same composite structure under a non-transformable prim does not match
    stage.define_prim("/World/Scope", "Scope")
    stage.define_prim("/World/Scope/P", "Plane")
    stage.define_prim("/World/Scope/M", "Mesh")
    paths = [ground_plane_path, "/World/Empty", "/World/Wrong", "/World/Scope"]

    output = GroundPlane.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(4, 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([[1], [0], [0], [0]], dtype=np.bool_), output)
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(GroundPlane.are_of_type("/World/(Empty|Wrong)"), shape=(2, 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        GroundPlane.are_of_type("/World/NonExistent")
