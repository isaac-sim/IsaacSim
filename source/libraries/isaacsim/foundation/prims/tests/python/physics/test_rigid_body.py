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

"""Test rigid body behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import Prim
from isaacsim.foundation.prims import RigidBody

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Utility functions.
"""


def _get_paths(stage: Any) -> Any:
    for path in [f"/World/Prim{i}" for i in range(5)]:
        stage.define_prim(path, "Cube")
    return "/World/Prim.*"


"""
Test cases.
"""


@hypothesis.given(
    masses=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(5, 1), elements=isaacsim_test.finite_float_elements()
    ),
)
def test_masses(capsys: Any, stage: Any, masses: Any) -> None:
    """Test masses.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        masses: Masses values.
    """
    prims = RigidBody(_get_paths(stage))
    # get values
    output = prims.get_masses()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_masses(masses)
    output = prims.get_masses()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(masses, output)
    # inverse masses
    output = prims.get_masses(inverse=True)
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(1.0 / (masses + 1e-8), output)


@hypothesis.given(
    densities=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(5, 1), elements=isaacsim_test.finite_float_elements()
    ),
)
def test_densities(capsys: Any, stage: Any, densities: Any) -> None:
    """Test densities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        densities: Densities values.
    """
    prims = RigidBody(_get_paths(stage))
    # get values
    output = prims.get_densities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_densities(densities)
    output = prims.get_densities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(densities, output)


@hypothesis.given(
    thresholds=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(5, 1), elements=isaacsim_test.finite_float_elements()
    ),
)
def test_sleep_thresholds(capsys: Any, stage: Any, thresholds: Any) -> None:
    """Test sleep thresholds.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        thresholds: Thresholds values.
    """
    prims = RigidBody(_get_paths(stage))
    # get values
    output = prims.get_sleep_thresholds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_sleep_thresholds(thresholds)
    output = prims.get_sleep_thresholds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(thresholds, output)


@hypothesis.given(
    linear_velocities=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(5, 3), elements=isaacsim_test.finite_float_elements()
    ),
    angular_velocities=hypothesis.extra.numpy.arrays(
        dtype=np.float32, shape=(5, 3), elements=isaacsim_test.finite_float_elements()
    ),
)
def test_velocities(capsys: Any, stage: Any, linear_velocities: Any, angular_velocities: Any) -> None:
    """Test velocities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        linear_velocities: Linear velocities values.
        angular_velocities: Angular velocities values.
    """
    prims = RigidBody(_get_paths(stage))
    # get values
    output_linear_velocities, output_angular_velocities = prims.get_velocities()
    isaacsim_test.check_array(output_linear_velocities, shape=(5, 3), dtype=wp.float32)
    isaacsim_test.check_array(output_angular_velocities, shape=(5, 3), dtype=wp.float32)
    # round-trip values
    prims.set_velocities(linear_velocities, angular_velocities)
    output_linear_velocities, output_angular_velocities = prims.get_velocities()
    isaacsim_test.check_array(output_linear_velocities, shape=(5, 3), dtype=wp.float32)
    isaacsim_test.check_array(output_angular_velocities, shape=(5, 3), dtype=wp.float32)
    isaacsim_test.check_allclose(linear_velocities, output_linear_velocities)
    isaacsim_test.check_allclose(angular_velocities, output_angular_velocities)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
def test_enabled_rigid_bodies(capsys: Any, stage: Any, values: Any) -> None:
    """Test enabled rigid bodies.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        values: Values exercised by the test.
    """
    prims = RigidBody(_get_paths(stage))
    # get values
    output = prims.get_enabled_rigid_bodies()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_rigid_bodies(values)
    output = prims.get_enabled_rigid_bodies()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
def test_enabled_gravities(capsys: Any, stage: Any, values: Any) -> None:
    """Test enabled gravities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        values: Values exercised by the test.
    """
    prims = RigidBody(_get_paths(stage))
    # get values
    output = prims.get_enabled_gravities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_gravities(values)
    output = prims.get_enabled_gravities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


def test_physics_apis(capsys: Any, stage: Any) -> None:
    """Test physics apis.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    prims = RigidBody(_get_paths(stage))
    prims.apply_physics_apis()
    assert all(prims.has_api("PhysicsRigidBodyAPI").numpy())
    assert all(prims.has_api("PhysxRigidBodyAPI").numpy())
    prims.remove_physics_apis()
    assert not any(prims.has_api("PhysicsRigidBodyAPI").numpy())
    assert not any(prims.has_api("PhysxRigidBodyAPI").numpy())


def test_are_of_type(stage: Any) -> None:
    """Test are of type.

    Args:
        stage: Stage used by the test.
    """
    stage.define_prim("/World/Body", "Cube")
    stage.define_prim("/World/Other", "Cube")
    paths = ["/World/Body", "/World/Other"]
    # the Rigid Body API is not applied yet
    output = RigidBody.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(2, 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([[0], [0]], dtype=np.bool_), output)
    # the constructor applies it to the prims it wraps
    RigidBody("/World/Body")
    isaacsim_test.check_equal(np.array([[1], [0]], dtype=np.bool_), RigidBody.are_of_type(paths))
    # a non-transformable prim never matches, even with the schema applied
    stage.define_prim("/World/Scope", "Scope")
    Prim("/World/Scope").apply_api("PhysicsRigidBodyAPI")
    isaacsim_test.check_equal(np.array([[0]], dtype=np.bool_), RigidBody.are_of_type("/World/Scope"))
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(RigidBody.are_of_type("/World/(Body|Other)"), shape=(2, 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        RigidBody.are_of_type("/World/NonExistent")
