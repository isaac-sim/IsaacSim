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

"""Test PhysX physics scene behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import PhysicsScene, PhysxGpuCfg, PhysxScene

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Utility functions.
"""

GPU_CONFIGURATION_FIELDS = [
    "gpu_collision_stack_size",
    "gpu_found_lost_aggregate_pairs_capacity",
    "gpu_found_lost_pairs_capacity",
    "gpu_heap_capacity",
    "gpu_max_deformable_surface_contacts",
    "gpu_max_deformable_volume_contacts",
    "gpu_max_num_partitions",
    "gpu_max_particle_contacts",
    "gpu_max_rigid_contact_count",
    "gpu_max_rigid_patch_count",
    "gpu_temp_buffer_capacity",
    "gpu_total_aggregate_pairs_capacity",
]


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
    prims = PhysxScene(_get_paths(stage, populate))
    # get values
    output = prims.get_dts()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values: each sampled delta time is the exact inverse of an integer frequency
    prims.set_dts(dts=values)
    output = prims.get_dts()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)
    # the delta time is the reciprocal view of the PhysX step frequency
    isaacsim_test.check_allclose(np.round(1.0 / values).astype(np.int32), prims.get_time_steps_per_seconds())
    # the solver-agnostic delta time is kept in sync
    isaacsim_test.check_allclose(
        np.round(1.0 / values).astype(np.int32), prims.get_attribute_values("newton:timeStepsPerSecond")
    )
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
    prims = PhysxScene(_get_paths(stage, populate))
    # get values
    output = prims.get_time_steps_per_seconds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.uint32)
    # round-trip values
    prims.set_time_steps_per_seconds(values)
    output = prims.get_time_steps_per_seconds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.uint32)
    isaacsim_test.check_equal(values.astype(np.uint32), output)
    # the solver-agnostic step frequency is kept in sync
    isaacsim_test.check_equal(values, prims.get_attribute_values("newton:timeStepsPerSecond"))
    # without 'PhysxSceneAPI' there is no PhysX attribute to read, so the base value is reported
    PhysicsScene(prims.paths).set_time_steps_per_seconds(np.array([333], dtype=np.int32))
    prims.remove_api("PhysxSceneAPI")
    isaacsim_test.check_equal(np.full((5, 1), 333, dtype=np.int32), prims.get_time_steps_per_seconds())


@pytest.mark.parametrize("populate", [True, False])
def test_solver_types(capsys: Any, stage: Any, populate: Any) -> None:
    """Test solver types.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = PhysxScene(_get_paths(stage, populate))
    # get values
    isaacsim_test.check_equal(["TGS"] * 5, prims.get_solver_types())
    # a single token is applied to every selected prim
    prims.set_solver_types("PGS")
    isaacsim_test.check_equal(["PGS"] * 5, prims.get_solver_types())
    # one token per prim
    prims.set_solver_types(["TGS", "PGS", "TGS", "PGS", "TGS"])
    isaacsim_test.check_equal(["TGS", "PGS", "TGS", "PGS", "TGS"], prims.get_solver_types())
    # unsupported tokens are rejected
    with pytest.raises(ValueError):
        prims.set_solver_types("tgs")


@pytest.mark.parametrize("populate", [True, False])
def test_broadphase_types(capsys: Any, stage: Any, populate: Any) -> None:
    """Test broadphase types.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = PhysxScene(_get_paths(stage, populate))
    # get values
    isaacsim_test.check_equal(["GPU"] * 5, prims.get_broadphase_types())
    # a single token is applied to every selected prim
    prims.set_broadphase_types("MBP")
    isaacsim_test.check_equal(["MBP"] * 5, prims.get_broadphase_types())
    # one token per prim
    prims.set_broadphase_types(["SAP", "MBP", "GPU", "SAP", "MBP"])
    isaacsim_test.check_equal(["SAP", "MBP", "GPU", "SAP", "MBP"], prims.get_broadphase_types())
    # unsupported tokens are rejected
    with pytest.raises(ValueError):
        prims.set_broadphase_types("NONE")


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_enabled_gpu_dynamics(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test enabled GPU dynamics.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysxScene(_get_paths(stage, populate))
    # get values
    output = prims.get_enabled_gpu_dynamics()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_ccds(np.ones((5, 1), dtype=np.bool_))
    prims.set_enabled_gpu_dynamics(values)
    output = prims.get_enabled_gpu_dynamics()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)
    # GPU dynamics does not support CCD, so CCD survives only where GPU dynamics was left disabled
    isaacsim_test.check_equal(np.logical_not(values), prims.get_enabled_ccds())


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_enabled_ccds(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test enabled CCDs.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysxScene(_get_paths(stage, populate))
    # get values
    output = prims.get_enabled_ccds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_gpu_dynamics(np.ones((5, 1), dtype=np.bool_))
    prims.set_enabled_ccds(values)
    output = prims.get_enabled_ccds()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)
    # CCD is not supported by GPU dynamics, so GPU dynamics survives only where CCD was left disabled
    isaacsim_test.check_equal(np.logical_not(values), prims.get_enabled_gpu_dynamics())


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_enabled_stabilizations(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test enabled stabilizations.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = PhysxScene(_get_paths(stage, populate))
    # get values
    output = prims.get_enabled_stabilizations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_stabilizations(values)
    output = prims.get_enabled_stabilizations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


@pytest.mark.parametrize("populate", [True, False])
def test_gpu_configuration(capsys: Any, stage: Any, populate: Any) -> None:
    """Test the GPU configuration.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
    """
    prims = PhysxScene(_get_paths(stage, populate))
    # get values: every field is populated
    before = prims.get_gpu_configuration()
    for field in GPU_CONFIGURATION_FIELDS:
        values = getattr(before, field)
        assert values is not None, f"'{field}' was not populated"
        isaacsim_test.check_array(values, shape=(5, 1))
    # an unset field is left untouched
    prims.set_gpu_configuration(PhysxGpuCfg(gpu_max_num_partitions=np.full((5, 1), 16, dtype=np.int32)))
    after = prims.get_gpu_configuration()
    isaacsim_test.check_allclose(np.full((5, 1), 16, dtype=np.int32), after.gpu_max_num_partitions)
    for field in GPU_CONFIGURATION_FIELDS:
        if field != "gpu_max_num_partitions":
            isaacsim_test.check_allclose(getattr(before, field), getattr(after, field))
    # round-trip every field
    written = {
        field: np.full((5, 1), 1024 + index, dtype=np.int64) for index, field in enumerate(GPU_CONFIGURATION_FIELDS)
    }
    prims.set_gpu_configuration(PhysxGpuCfg(**written))
    after = prims.get_gpu_configuration()
    for field, values in written.items():
        isaacsim_test.check_allclose(values, getattr(after, field))


def test_constructor(capsys: Any, stage: Any) -> None:
    """Test the construction of the wrapper.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    # non-existing paths are created as Physics Scene prims carrying both scene schemas
    prims = PhysxScene(["/World/Prim0", "/World/Prim1"])
    isaacsim_test.check_equal(["PhysicsScene", "PhysicsScene"], prims.get_type_name())
    isaacsim_test.check_equal(np.ones((2, 1), dtype=np.bool_), prims.has_api("PhysxSceneAPI"))
    isaacsim_test.check_equal(np.ones((2, 1), dtype=np.bool_), prims.has_api("NewtonSceneAPI"))
    # existing prims of another type are rejected
    stage.define_prim("/World/Scope", "Scope")
    with pytest.raises(RuntimeError):
        PhysxScene("/World/Scope")
