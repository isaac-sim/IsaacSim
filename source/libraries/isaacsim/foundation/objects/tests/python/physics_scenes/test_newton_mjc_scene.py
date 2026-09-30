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

"""Test Newton MuJoCo physics scene behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import NewtonMjcScene, PhysicsScene

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Utility functions.
"""


def _get_prims(stage: Any, populate: Any, count: int = 5) -> Any:
    """Build a NewtonMjcScene wrapper, skipping the test when the MuJoCo USD schemas are missing.

    The ``MjcSceneAPI`` schema ships with the MuJoCo USD schema plugin, which is not registered in
    every build. Without it the wrapper cannot be constructed at all, so the test is skipped rather
    than failed.

    Args:
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        count: Number of prims to wrap.

    Returns:
        The wrapper over the test prims.
    """
    paths = [f"/World/Prim{i}" for i in range(count)]
    if populate:
        for path in paths:
            stage.define_prim(path, "PhysicsScene")
        paths = "/World/Prim.*"
    try:
        return NewtonMjcScene(paths)
    except ValueError as error:
        pytest.skip(f"the MuJoCo USD schemas are not registered in this build: {error}")


def _check_tokens(prims: Any, setter: str, getter: str, default: str, values: list, invalid: str) -> None:
    """Check the round-trip of a token-valued setting.

    Args:
        prims: Wrapper over the test prims.
        setter: Name of the setter exercised by the test.
        getter: Name of the getter exercised by the test.
        default: Expected value before anything is written.
        values: Accepted tokens exercised by the test.
        invalid: Token that must be rejected.
    """
    # get values
    isaacsim_test.check_equal([default] * 5, getattr(prims, getter)())
    # a single token is applied to every selected prim
    for value in values:
        getattr(prims, setter)(value)
        isaacsim_test.check_equal([value] * 5, getattr(prims, getter)())
    # one token per prim
    per_prim = [values[index % len(values)] for index in range(5)]
    getattr(prims, setter)(per_prim)
    isaacsim_test.check_equal(per_prim, getattr(prims, getter)())
    # unsupported tokens are rejected
    with pytest.raises(ValueError):
        getattr(prims, setter)(invalid)


"""
Test cases.
"""


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float64,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(min_value=1e-6, max_value=1.0),
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
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_dts()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    # round-trip values: unlike the other solvers, the MuJoCo delta time is stored as a duration
    prims.set_dts(dts=values)
    output = prims.get_dts()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose(values, output)
    # the solver-agnostic step frequency is kept in sync
    isaacsim_test.check_equal(
        np.round(1.0 / values).astype(np.int32), prims.get_attribute_values("newton:timeStepsPerSecond")
    )
    # out-of-range delta times are rejected
    for value in [0.0, -0.1, 1.1]:
        with pytest.raises(ValueError):
            prims.set_dts(np.array([value], dtype=np.float64))


@pytest.mark.parametrize("populate", [True, False])
def test_time_steps_per_seconds(capsys: Any, stage: Any, populate: Any) -> None:
    """Test time steps per second.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_time_steps_per_seconds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    # round-trip values
    values = np.array([[1], [30], [60], [500], [1000]], dtype=np.int32)
    prims.set_time_steps_per_seconds(values)
    output = prims.get_time_steps_per_seconds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    isaacsim_test.check_equal(values, output)
    # the MuJoCo attribute holds the reciprocal duration, and the solver-agnostic one the frequency
    isaacsim_test.check_allclose(1.0 / values.astype(np.float64), prims.get_attribute_values("mjc:option:timestep"))
    isaacsim_test.check_equal(values, prims.get_attribute_values("newton:timeStepsPerSecond"))
    # non-positive step frequencies are rejected
    for value in [0, -1]:
        with pytest.raises(ValueError):
            prims.set_time_steps_per_seconds(np.array([value], dtype=np.int32))
    # without 'MjcSceneAPI' there is no MuJoCo attribute to read, so the base value is reported
    PhysicsScene(prims.paths).set_time_steps_per_seconds(np.array([333], dtype=np.int32))
    prims.remove_api("MjcSceneAPI")
    isaacsim_test.check_equal(np.full((5, 1), 333, dtype=np.int32), prims.get_time_steps_per_seconds())


@pytest.mark.parametrize("populate", [True, False])
def test_integrators(capsys: Any, stage: Any, populate: Any) -> None:
    """Test integrators.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    _check_tokens(
        _get_prims(stage, populate),
        "set_integrators",
        "get_integrators",
        "euler",
        ["rk4", "implicit", "implicitfast"],
        "verlet",
    )


@pytest.mark.parametrize("populate", [True, False])
def test_solvers(capsys: Any, stage: Any, populate: Any) -> None:
    """Test constraint solver algorithms.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    _check_tokens(_get_prims(stage, populate), "set_solvers", "get_solvers", "newton", ["pgs", "cg"], "PGS")


@pytest.mark.parametrize("populate", [True, False])
def test_cones(capsys: Any, stage: Any, populate: Any) -> None:
    """Test friction cone types.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    _check_tokens(_get_prims(stage, populate), "set_cones", "get_cones", "pyramidal", ["elliptic"], "conic")


@pytest.mark.parametrize("populate", [True, False])
def test_jacobians(capsys: Any, stage: Any, populate: Any) -> None:
    """Test constraint Jacobian types.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    _check_tokens(_get_prims(stage, populate), "set_jacobians", "get_jacobians", "auto", ["dense", "sparse"], "banded")


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.int32,
        shape=(5, 1),
        elements=hypothesis.strategies.integers(min_value=1, max_value=1000),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_iterations(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test solver iterations.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_iterations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    # round-trip values
    prims.set_iterations(values)
    output = prims.get_iterations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.int32)
    isaacsim_test.check_equal(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float64,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_tolerances(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test solver tolerances.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_tolerances()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_tolerances(values)
    output = prims.get_tolerances()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float64,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_impedance_ratios(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test impedance ratios.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_impedance_ratios()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_impedance_ratios(values)
    output = prims.get_impedance_ratios()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float64,
        shape=(5, 3),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_winds(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test wind velocities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_winds()
    isaacsim_test.check_array(output, shape=(5, 3), dtype=wp.float64)
    # round-trip values
    prims.set_winds(values)
    output = prims.get_winds()
    isaacsim_test.check_array(output, shape=(5, 3), dtype=wp.float64)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float64,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_densities(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test medium densities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_densities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_densities(values)
    output = prims.get_densities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float64,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_viscosities(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test medium viscosities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = _get_prims(stage, populate)
    # get values
    output = prims.get_viscosities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_viscosities(values)
    output = prims.get_viscosities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose(values, output)


def test_constructor(capsys: Any, stage: Any) -> None:
    """Test the construction of the wrapper.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    # non-existing paths are created as Physics Scene prims carrying both scene schemas
    prims = _get_prims(stage, populate=False, count=2)
    isaacsim_test.check_equal(["PhysicsScene", "PhysicsScene"], prims.get_type_name())
    isaacsim_test.check_equal(np.ones((2, 1), dtype=np.bool_), prims.has_api("MjcSceneAPI"))
    isaacsim_test.check_equal(np.ones((2, 1), dtype=np.bool_), prims.has_api("NewtonSceneAPI"))
    # existing prims of another type are rejected
    stage.define_prim("/World/Scope", "Scope")
    with pytest.raises(RuntimeError):
        NewtonMjcScene("/World/Scope")
