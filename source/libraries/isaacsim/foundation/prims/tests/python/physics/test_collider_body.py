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

"""Test collider body behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import Prim
from isaacsim.foundation.prims import ColliderBody

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture


def _get_paths(stage: Any) -> Any:
    for path in [f"/World/Prim{i}" for i in range(5)]:
        stage.define_prim(path, "Cube")
    return "/World/Prim.*"


def test_collision_apis(stage: Any) -> None:
    """Test collision apis.

    Args:
        stage: Stage used by the test.
    """
    prims = ColliderBody(_get_paths(stage))
    prims.apply_collision_apis()
    assert all(prims.has_api("PhysicsCollisionAPI").numpy())
    assert all(prims.has_api("PhysicsMeshCollisionAPI").numpy())
    assert all(prims.has_api("PhysxCollisionAPI").numpy())
    prims.remove_collision_apis()
    assert not any(prims.has_api("PhysicsCollisionAPI").numpy())
    assert not any(prims.has_api("PhysicsMeshCollisionAPI").numpy())
    assert not any(prims.has_api("PhysxCollisionAPI").numpy())


@hypothesis.given(
    contact_offsets=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(5, 1), elements=isaacsim_test.finite_float_elements()
    ),
    rest_offsets=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(5, 1), elements=isaacsim_test.finite_float_elements()
    ),
)
def test_offsets(stage: Any, contact_offsets: Any, rest_offsets: Any) -> None:
    """Test offsets.

    Args:
        stage: Stage used by the test.
        contact_offsets: Contact offsets values.
        rest_offsets: Rest offsets values.
    """
    prims = ColliderBody(_get_paths(stage))
    # get values
    output_contact_offsets, output_rest_offsets = prims.get_offsets()
    isaacsim_test.check_array(output_contact_offsets, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_array(output_rest_offsets, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_offsets(contact_offsets, rest_offsets)
    output_contact_offsets, output_rest_offsets = prims.get_offsets()
    isaacsim_test.check_array(output_contact_offsets, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_array(output_rest_offsets, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(contact_offsets, output_contact_offsets)
    isaacsim_test.check_allclose(rest_offsets, output_rest_offsets)


@hypothesis.given(
    radii=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1), elements=isaacsim_test.finite_float_elements()),
)
@pytest.mark.parametrize("minimum", [True, False])
def test_torsional_patch_radii(stage: Any, minimum: Any, radii: Any) -> None:
    """Test torsional patch radii.

    Args:
        stage: Stage used by the test.
        minimum: Minimum minimum value.
        radii: Radii values.
    """
    prims = ColliderBody(_get_paths(stage))
    # get values
    output = prims.get_torsional_patch_radii(minimum=minimum)
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_torsional_patch_radii(radii, minimum=minimum)
    output = prims.get_torsional_patch_radii(minimum=minimum)
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(radii, output)


@hypothesis.given(
    values=hypothesis.strategies.lists(
        hypothesis.strategies.sampled_from(
            [
                "none",
                "convexDecomposition",
                "convexHull",
                "boundingSphere",
                "boundingCube",
                "meshSimplification",
                "sdf",
                "sphereFill",
            ]
        ),
        min_size=5,
        max_size=5,
    ),
)
def test_collision_approximations(stage: Any, values: Any) -> None:
    """Test collision approximations.

    Args:
        stage: Stage used by the test.
        values: Values exercised by the test.
    """
    prims = ColliderBody(_get_paths(stage))
    # get values
    prims.get_collision_approximations()
    # round-trip values
    prims.set_collision_approximations(values)
    output = prims.get_collision_approximations()
    isaacsim_test.check_equal(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
def test_enabled_collisions(stage: Any, values: Any) -> None:
    """Test enabled collisions.

    Args:
        stage: Stage used by the test.
        values: Values exercised by the test.
    """
    prims = ColliderBody(_get_paths(stage))
    # get values
    output = prims.get_enabled_collisions()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_collisions(values)
    output = prims.get_enabled_collisions()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


def test_are_of_type(stage: Any) -> None:
    """Test are of type.

    Args:
        stage: Stage used by the test.
    """
    stage.define_prim("/World/Body", "Cube")
    stage.define_prim("/World/Other", "Cube")
    paths = ["/World/Body", "/World/Other"]
    # the Collision API is not applied yet
    output = ColliderBody.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(2, 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([[0], [0]], dtype=np.bool_), output)
    # the constructor applies it to the prims it wraps
    ColliderBody("/World/Body")
    isaacsim_test.check_equal(np.array([[1], [0]], dtype=np.bool_), ColliderBody.are_of_type(paths))
    # a non-transformable prim never matches, even with the schema applied
    stage.define_prim("/World/Scope", "Scope")
    Prim("/World/Scope").apply_api("PhysicsCollisionAPI")
    isaacsim_test.check_equal(np.array([[0]], dtype=np.bool_), ColliderBody.are_of_type("/World/Scope"))
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(ColliderBody.are_of_type("/World/(Body|Other)"), shape=(2, 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        ColliderBody.are_of_type("/World/NonExistent")
