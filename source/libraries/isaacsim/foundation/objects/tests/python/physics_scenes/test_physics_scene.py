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

"""Test physics scene behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import PhysicsScene

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Utility functions.
"""


def _get_paths(stage: Any, populate: Any) -> Any:
    paths = [f"/World/Prim{i}" for i in range(5)]
    if populate:
        for path in paths:
            stage.define_prim(path, "PhysicsScene")
        paths = "/World/Prim.*"
    return paths


"""
Test cases.
"""


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 3),
        elements=isaacsim_test.finite_float_elements(min_value=-1e3, max_value=1e3),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_gravities(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test gravities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysicsScene(_get_paths(stage, populate))
    # get values
    output = prims.get_gravities()
    isaacsim_test.check_array(output, shape=(5, 3), dtype=wp.float32)
    # round-trip values
    prims.set_gravities(values)
    output = prims.get_gravities()
    isaacsim_test.check_array(output, shape=(5, 3), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output, atol=1e-3)
    # a single vector is applied to every selected prim
    prims.set_gravities(np.array([0.0, 0.0, -9.81], dtype=np.float32))
    isaacsim_test.check_allclose(np.tile([0.0, 0.0, -9.81], (5, 1)).astype(np.float32), prims.get_gravities())
    # a zero-length vector keeps the stage's down direction
    prims.set_gravities(np.zeros((1, 3), dtype=np.float32))
    isaacsim_test.check_allclose(np.zeros((5, 3), dtype=np.float32), prims.get_gravities())
    isaacsim_test.check_allclose(
        np.tile([0.0, 0.0, -1.0], (5, 1)).astype(np.float32), prims.get_attribute_values("physics:gravityDirection")
    )
    # a value count that is not a multiple of three is rejected
    with pytest.raises(ValueError):
        prims.set_gravities(np.zeros((5,), dtype=np.float32))


@pytest.mark.parametrize("populate", [True, False])
def test_gravities_sentinels(capsys: Any, stage: Any, populate: Any) -> None:
    """Test the USD sentinels that gravity resolves against.

    A zero direction and a negative magnitude are two independent requests, and the standard
    gravity they resolve to is expressed in stage units.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = PhysicsScene(_get_paths(stage, populate))
    # an unauthored gravity falls along the stage's down direction
    isaacsim_test.check_allclose(np.tile([0.0, 0.0, -9.81], (5, 1)).astype(np.float32), prims.get_gravities())
    stage.set_up_axis("Y")
    isaacsim_test.check_allclose(np.tile([0.0, -9.81, 0.0], (5, 1)).astype(np.float32), prims.get_gravities())
    stage.set_up_axis("Z")
    # a magnitude authored on its own falls along the stage's down direction too
    prims.set_attribute_values("physics:gravityMagnitude", np.array([4.0], dtype=np.float32))
    isaacsim_test.check_allclose(np.tile([0.0, 0.0, -4.0], (5, 1)).astype(np.float32), prims.get_gravities())
    # any negative magnitude, not only the -inf default, requests the standard gravity
    prims.set_attribute_values("physics:gravityMagnitude", np.array([-1.0], dtype=np.float32))
    prims.set_attribute_values("physics:gravityDirection", np.array([1.0, 0.0, 0.0], dtype=np.float32))
    isaacsim_test.check_allclose(np.tile([9.81, 0.0, 0.0], (5, 1)).astype(np.float32), prims.get_gravities())
    # the standard gravity is expressed in stage units
    stage.set_units(meters_per_unit=0.01)
    isaacsim_test.check_allclose(np.tile([981.0, 0.0, 0.0], (5, 1)).astype(np.float32), prims.get_gravities())
    # an authored magnitude is stored in stage units too, so it needs no conversion
    prims.set_gravities(np.array([0.0, 0.0, -981.0], dtype=np.float32))
    isaacsim_test.check_allclose(
        np.full((5, 1), 981.0, dtype=np.float32), prims.get_attribute_values("physics:gravityMagnitude")
    )


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=hypothesis.strategies.sampled_from([1.0, 0.5, 1 / 30.0, 1 / 60.0, 1 / 120.0, 1 / 1000.0]),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_dts(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test delta times.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysicsScene(_get_paths(stage, populate))
    # get values
    output = prims.get_dts()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values: each sampled delta time is the exact inverse of an integer frequency
    prims.set_dts(dts=values)
    output = prims.get_dts()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)
    # the delta time is the reciprocal view of the step frequency
    isaacsim_test.check_allclose(np.round(1.0 / values).astype(np.int32), prims.get_time_steps_per_seconds())
    # a subset of the wrapped prims can be addressed
    prims.set_dts(np.full((5, 1), 1 / 60.0, dtype=np.float32))
    prims.set_dts(np.array([1 / 120.0], dtype=np.float32), indices=np.array([1, 3]))
    isaacsim_test.check_allclose(
        np.array([[1 / 60.0], [1 / 120.0], [1 / 60.0], [1 / 120.0], [1 / 60.0]], dtype=np.float32), prims.get_dts()
    )
    isaacsim_test.check_allclose(np.full((2, 1), 1 / 120.0, dtype=np.float32), prims.get_dts(indices=np.array([1, 3])))
    # out-of-range delta times are rejected
    for value in [0.0, -0.1, 1.1, 1e-12]:
        with pytest.raises(ValueError):
            prims.set_dts(np.array([value], dtype=np.float32))


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.int32,
        shape=(5, 1),
        elements=hypothesis.strategies.integers(min_value=1, max_value=10000),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_time_steps_per_seconds(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test time steps per second.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysicsScene(_get_paths(stage, populate))
    # get values
    output = prims.get_time_steps_per_seconds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    # round-trip values
    prims.set_time_steps_per_seconds(values)
    output = prims.get_time_steps_per_seconds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    isaacsim_test.check_equal(values, output)
    # a single value is applied to every selected prim
    prims.set_time_steps_per_seconds(np.array([240], dtype=np.int32))
    isaacsim_test.check_equal(np.full((5, 1), 240, dtype=np.int32), prims.get_time_steps_per_seconds())


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_enabled_gravities(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test enabled gravities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysicsScene(_get_paths(stage, populate))
    # get values
    output = prims.get_enabled_gravities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_gravities(values)
    output = prims.get_enabled_gravities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)
    # a single value is applied to every selected prim
    prims.set_enabled_gravities(np.array([False]))
    isaacsim_test.check_equal(np.zeros((5, 1), dtype=np.bool_), prims.get_enabled_gravities())


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.int32,
        shape=(5, 1),
        elements=hypothesis.strategies.integers(min_value=-1, max_value=1000),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_max_solver_iterations(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test maximum solver iterations.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysicsScene(_get_paths(stage, populate))
    # get values
    output = prims.get_max_solver_iterations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    # round-trip values
    prims.set_max_solver_iterations(values)
    output = prims.get_max_solver_iterations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    isaacsim_test.check_equal(values, output)
    # a single value is applied to every selected prim
    prims.set_max_solver_iterations(np.array([32], dtype=np.int32))
    isaacsim_test.check_equal(np.full((5, 1), 32, dtype=np.int32), prims.get_max_solver_iterations())


def test_constructor(capsys: Any, stage: Any) -> None:
    """Test the construction of the wrapper.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    # non-existing paths are created as Physics Scene prims carrying the Newton scene schema
    prims = PhysicsScene(["/World/Prim0", "/World/Prim1"])
    isaacsim_test.check_equal(["PhysicsScene", "PhysicsScene"], prims.get_type_name())
    isaacsim_test.check_equal(np.ones((2, 1), dtype=np.bool_), prims.has_api("NewtonSceneAPI"))
    # existing prims of another type are rejected
    stage.define_prim("/World/Scope", "Scope")
    with pytest.raises(RuntimeError):
        PhysicsScene("/World/Scope")


def test_are_of_type(capsys: Any, stage: Any) -> None:
    """Test are of type.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    type_names = ["PhysicsScene", "Scope", "Xform"]
    for index, type_name in enumerate(type_names):
        stage.define_prim(f"/World/Prim{index}", type_name)
    paths = [f"/World/Prim{index}" for index in range(len(type_names))]
    # boolean flags are reported per prim, in the order the paths were given
    output = PhysicsScene.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(len(type_names), 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([1, 0, 0], dtype=np.bool_).reshape(-1, 1), output)
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(PhysicsScene.are_of_type("/World/Prim.*"), shape=(len(type_names), 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        PhysicsScene.are_of_type("/World/NonExistent")


def test_get_physics_scene_paths(capsys: Any, stage: Any) -> None:
    """Test get physics scene paths.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    # an empty stage holds no physics scene
    isaacsim_test.check_equal([], PhysicsScene.get_physics_scene_paths())
    # scenes are found at any depth, and prims of other types are ignored
    stage.define_prim("/World/Scope", "Scope")
    stage.define_prim("/World/Scope/Nested", "PhysicsScene")
    stage.define_prim("/RootScene", "PhysicsScene")
    isaacsim_test.check_equal(
        sorted(["/RootScene", "/World/Scope/Nested"]), sorted(PhysicsScene.get_physics_scene_paths())
    )
