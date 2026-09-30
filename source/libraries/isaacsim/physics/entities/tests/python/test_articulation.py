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

"""Test articulation behavior."""

from typing import Any

import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.physics.entities import ArticulationEntity

from .fixtures import engine  # noqa: F401 - imported so pytest can discover the fixture

PATHS = [f"/World/Prim{i}" for i in range(5)]

N = len(PATHS)  # articulations
L = 12  # links
D = 7  # DOFs
J = 9  # joints
T = 3  # fixed tendons
S = 4  # shapes per articulation

"""
Test cases.
"""


def test_metadata(capsys: Any, engine: Any) -> None:
    """Test metadata.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    assert prims.num_prims == N
    assert prims.num_dofs == D
    assert prims.num_joints == J
    assert prims.num_links == L
    assert prims.num_shapes == S
    assert prims.num_fixed_tendons == T
    assert prims.dof_names == [f"joint_{i}" for i in range(D)]
    assert prims.joint_names == [f"joint_{i}" for i in range(J)]
    assert prims.link_names == [f"link_{i}" for i in range(L)]
    assert prims.jacobian_matrix_shape == (L - 1, 6, D)
    assert prims.mass_matrix_shape == (D, D)


def test_unavailable_metadata(capsys: Any, engine: Any) -> None:
    """Test unavailable metadata.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    for name in ["dof_paths", "dof_types", "joint_paths", "joint_types", "link_paths"]:
        with pytest.raises(RuntimeError):
            getattr(prims, name)


def test_indices(capsys: Any, engine: Any) -> None:
    """Test indices.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    # single name
    isaacsim_test.check_array(prims.get_dof_indices("joint_0"), shape=(1,), dtype=wp.int32)
    isaacsim_test.check_array(prims.get_joint_indices("joint_0"), shape=(1,), dtype=wp.int32)
    isaacsim_test.check_array(prims.get_link_indices("link_0"), shape=(1,), dtype=wp.int32)
    # multiple names
    isaacsim_test.check_array(prims.get_dof_indices(["joint_1", "joint_2"]), shape=(2,), dtype=wp.int32)
    isaacsim_test.check_array(prims.get_joint_indices(["joint_1", "joint_2"]), shape=(2,), dtype=wp.int32)
    isaacsim_test.check_array(prims.get_link_indices(["link_1", "link_2"]), shape=(2,), dtype=wp.int32)
    # invalid names
    with pytest.raises(ValueError):
        prims.get_dof_indices("invalid")
    with pytest.raises(ValueError):
        prims.get_joint_indices("invalid")
    with pytest.raises(ValueError):
        prims.get_link_indices("invalid")


def test_dof_limits(capsys: Any, engine: Any) -> None:
    """Test dof limits.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    lower, upper = prims.get_dof_limits()
    isaacsim_test.check_array([lower, upper], shape=(N, D), dtype=wp.float32)
    # round-trip
    prims.set_dof_limits(lower=np.ones((N, D), dtype=np.float32), upper=np.ones((N, D), dtype=np.float32))
    lower, upper = prims.get_dof_limits()
    isaacsim_test.check_array([lower, upper], shape=(N, D), dtype=wp.float32)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_dof_limits()


def test_dof_friction_properties(capsys: Any, engine: Any) -> None:
    """Test dof friction properties.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    static_frictions, dynamic_frictions, viscous_frictions = prims.get_dof_friction_properties()
    isaacsim_test.check_array([static_frictions, dynamic_frictions, viscous_frictions], shape=(N, D), dtype=wp.float32)
    # round-trip
    prims.set_dof_friction_properties(
        static_frictions=np.ones((N, D), dtype=np.float32),
        dynamic_frictions=np.ones((N, D), dtype=np.float32),
        viscous_frictions=np.ones((N, D), dtype=np.float32),
    )
    static_frictions, dynamic_frictions, viscous_frictions = prims.get_dof_friction_properties()
    isaacsim_test.check_array([static_frictions, dynamic_frictions, viscous_frictions], shape=(N, D), dtype=wp.float32)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_dof_friction_properties()


def test_dof_armatures(capsys: Any, engine: Any) -> None:
    """Test dof armatures.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_dof_armatures()
    isaacsim_test.check_array(output, shape=(N, D), dtype=wp.float32)
    # round-trip
    prims.set_dof_armatures(np.ones((N, D), dtype=np.float32))
    output = prims.get_dof_armatures()
    isaacsim_test.check_array(output, shape=(N, D), dtype=wp.float32)


@pytest.mark.parametrize(
    "getter,setter",
    [
        ("get_dof_positions", "set_dof_positions"),
        ("get_dof_position_targets", "set_dof_position_targets"),
        ("get_dof_velocities", "set_dof_velocities"),
        ("get_dof_velocity_targets", "set_dof_velocity_targets"),
        ("get_dof_efforts", "set_dof_efforts"),
        ("get_dof_max_efforts", "set_dof_max_efforts"),
        ("get_dof_max_velocities", "set_dof_max_velocities"),
    ],
)
def test_dof_state(capsys: Any, engine: Any, getter: Any, setter: Any) -> None:
    """Test dof state.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.
        getter: Callback that reads the property.
        setter: Callback that writes the property.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = getattr(prims, getter)()
    isaacsim_test.check_array(output, shape=(N, D), dtype=wp.float32)
    # round-trip
    getattr(prims, setter)(np.ones((N, D), dtype=np.float32))
    output = getattr(prims, getter)()
    isaacsim_test.check_array(output, shape=(N, D), dtype=wp.float32)


def test_dof_projected_joint_forces(capsys: Any, engine: Any) -> None:
    """Test dof projected joint forces.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_dof_projected_joint_forces()
    isaacsim_test.check_array(output, shape=(N, D), dtype=wp.float32)


def test_dof_gains(capsys: Any, engine: Any) -> None:
    """Test dof gains.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    stiffnesses, dampings = prims.get_dof_gains()
    isaacsim_test.check_array([stiffnesses, dampings], shape=(N, D), dtype=wp.float32)
    # round-trip
    prims.set_dof_gains(stiffnesses=np.ones((N, D), dtype=np.float32), dampings=np.ones((N, D), dtype=np.float32))
    stiffnesses, dampings = prims.get_dof_gains()
    isaacsim_test.check_array([stiffnesses, dampings], shape=(N, D), dtype=wp.float32)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_dof_gains()


def test_switch_dof_control_mode(capsys: Any, engine: Any) -> None:
    """Test switch dof control mode.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    # position mode keeps the current gains
    with pytest.raises(AssertionError):
        prims.switch_dof_control_mode("position")
    # invalid mode
    with pytest.raises(ValueError):
        prims.switch_dof_control_mode("invalid")


def test_world_poses(capsys: Any, engine: Any) -> None:
    """Test world poses.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    positions, orientations = prims.get_world_poses()
    isaacsim_test.check_array(positions, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, 4), dtype=wp.float32)
    # round-trip
    prims.set_world_poses(positions=np.ones((N, 3), dtype=np.float32), orientations=np.ones((N, 4), dtype=np.float32))
    positions, orientations = prims.get_world_poses()
    isaacsim_test.check_array(positions, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, 4), dtype=wp.float32)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_world_poses()


def test_velocities(capsys: Any, engine: Any) -> None:
    """Test velocities.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    linear_velocities, angular_velocities = prims.get_velocities()
    isaacsim_test.check_array([linear_velocities, angular_velocities], shape=(N, 3), dtype=wp.float32)
    # round-trip
    prims.set_velocities(
        linear_velocities=np.ones((N, 3), dtype=np.float32), angular_velocities=np.ones((N, 3), dtype=np.float32)
    )
    linear_velocities, angular_velocities = prims.get_velocities()
    isaacsim_test.check_array([linear_velocities, angular_velocities], shape=(N, 3), dtype=wp.float32)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_velocities()


def test_link_incoming_joint_force(capsys: Any, engine: Any) -> None:
    """Test link incoming joint force.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    forces, torques = prims.get_link_incoming_joint_force()
    isaacsim_test.check_array([forces, torques], shape=(N, L, 3), dtype=wp.float32)


def test_jacobian_matrices(capsys: Any, engine: Any) -> None:
    """Test jacobian matrices.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_jacobian_matrices()
    isaacsim_test.check_array(output, shape=(N, (L - 1) * 6, D), dtype=wp.float32)


def test_mass_matrices(capsys: Any, engine: Any) -> None:
    """Test mass matrices.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_mass_matrices()
    isaacsim_test.check_array(output, shape=(N, D, D), dtype=wp.float32)


def test_compensation_forces(capsys: Any, engine: Any) -> None:
    """Test compensation forces.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_dof_coriolis_and_centrifugal_compensation_forces()
    isaacsim_test.check_array(output, shape=(N, D), dtype=wp.float32)
    output = prims.get_dof_gravity_compensation_forces()
    isaacsim_test.check_array(output, shape=(N, D), dtype=wp.float32)


def test_link_masses(capsys: Any, engine: Any) -> None:
    """Test link masses.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_link_masses()
    isaacsim_test.check_array(output, shape=(N, L), dtype=wp.float32)
    # round-trip
    prims.set_link_masses(np.ones((N, L), dtype=np.float32))
    output = prims.get_link_masses()
    isaacsim_test.check_array(output, shape=(N, L), dtype=wp.float32)
    # inverse masses
    output = prims.get_link_masses(inverse=True)
    isaacsim_test.check_array(output, shape=(N, L), dtype=wp.float32)


def test_link_inertias(capsys: Any, engine: Any) -> None:
    """Test link inertias.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_link_inertias()
    isaacsim_test.check_array(output, shape=(N, L, 9), dtype=wp.float32)
    # round-trip
    prims.set_link_inertias(np.ones((N, L, 9), dtype=np.float32))
    output = prims.get_link_inertias()
    isaacsim_test.check_array(output, shape=(N, L, 9), dtype=wp.float32)
    # inverse inertias
    output = prims.get_link_inertias(inverse=True)
    isaacsim_test.check_array(output, shape=(N, L, 9), dtype=wp.float32)


def test_link_coms(capsys: Any, engine: Any) -> None:
    """Test link coms.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    positions, orientations = prims.get_link_coms()
    isaacsim_test.check_array(positions, shape=(N, L, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, L, 4), dtype=wp.float32)
    # round-trip
    prims.set_link_coms(
        positions=np.ones((N, L, 3), dtype=np.float32), orientations=np.ones((N, L, 4), dtype=np.float32)
    )
    positions, orientations = prims.get_link_coms()
    isaacsim_test.check_array(positions, shape=(N, L, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, L, 4), dtype=wp.float32)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_link_coms()


def test_link_enabled_gravities(capsys: Any, engine: Any) -> None:
    """Test link enabled gravities.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = prims.get_link_enabled_gravities()
    isaacsim_test.check_array(output, shape=(N, L), dtype=wp.uint8)
    # round-trip
    prims.set_link_enabled_gravities(np.zeros((N, L), dtype=np.uint8))
    output = prims.get_link_enabled_gravities()
    isaacsim_test.check_array(output, shape=(N, L), dtype=wp.uint8)


@pytest.mark.parametrize(
    "getter",
    [
        "get_fixed_tendon_stiffnesses",
        "get_fixed_tendon_dampings",
        "get_fixed_tendon_limit_stiffnesses",
        "get_fixed_tendon_rest_lengths",
        "get_fixed_tendon_offsets",
    ],
)
def test_fixed_tendon_properties(capsys: Any, engine: Any, getter: Any) -> None:
    """Test fixed tendon properties.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.
        getter: Callback that reads the property.

    """
    prims = ArticulationEntity("engine", PATHS)
    output = getattr(prims, getter)()
    isaacsim_test.check_array(output, shape=(N, T), dtype=wp.float32)


def test_fixed_tendon_limits(capsys: Any, engine: Any) -> None:
    """Test fixed tendon limits.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    lower_limits, upper_limits = prims.get_fixed_tendon_limits()
    isaacsim_test.check_array([lower_limits, upper_limits], shape=(N, T), dtype=wp.float32)


def test_set_fixed_tendon_properties(capsys: Any, engine: Any) -> None:
    """Test set fixed tendon properties.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = ArticulationEntity("engine", PATHS)
    prims.set_fixed_tendon_properties(
        stiffnesses=np.ones((N, T), dtype=np.float32),
        dampings=np.ones((N, T), dtype=np.float32),
        limit_stiffnesses=np.ones((N, T), dtype=np.float32),
        lower_limits=np.ones((N, T), dtype=np.float32),
        upper_limits=np.ones((N, T), dtype=np.float32),
        rest_lengths=np.ones((N, T), dtype=np.float32),
        offsets=np.ones((N, T), dtype=np.float32),
    )
    lower_limits, upper_limits = prims.get_fixed_tendon_limits()
    isaacsim_test.check_array([lower_limits, upper_limits], shape=(N, T), dtype=wp.float32)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_fixed_tendon_properties()


@pytest.mark.parametrize("getter", ["get_dof_drive_model_properties", "get_dof_drive_types"])
def test_unsupported_dof_drive_queries(capsys: Any, engine: Any, getter: Any) -> None:
    """Test unsupported dof drive queries.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.
        getter: Callback that reads the property.

    """
    prims = ArticulationEntity("engine", PATHS)
    with pytest.raises(Exception):
        getattr(prims, getter)()
