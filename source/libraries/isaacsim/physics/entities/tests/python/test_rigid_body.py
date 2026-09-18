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

import isaacsim_test
import numpy as np
import warp as wp
from isaacsim.physics.entities import RigidBodyEntity

from .fixtures import engine  # noqa: F401 - imported so pytest can discover the fixture

PATHS = [f"/World/Prim{i}" for i in range(5)]
N = len(PATHS)
NUM_SHAPES = 100

"""
Test cases.
"""


def test_metadata(capsys: Any, engine: Any) -> None:
    """Test metadata.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    assert prims.num_prims == len(PATHS)
    assert prims.num_shapes == NUM_SHAPES


def test_world_poses(capsys: Any, engine: Any) -> None:
    """Test world poses.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    positions, orientations = prims.get_world_poses()
    isaacsim_test.check_array(positions, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, 4), dtype=wp.float32)
    # round-trip
    prims.set_world_poses(
        positions=np.ones((N, 3), dtype=np.float32),
        orientations=np.ones((N, 4), dtype=np.float32),
    )
    positions, orientations = prims.get_world_poses()
    isaacsim_test.check_array(positions, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, 4), dtype=wp.float32)


def test_velocities(capsys: Any, engine: Any) -> None:
    """Test velocities.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    linear_velocities, angular_velocities = prims.get_velocities()
    isaacsim_test.check_array(linear_velocities, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(angular_velocities, shape=(N, 3), dtype=wp.float32)
    # round-trip
    prims.set_velocities(
        linear_velocities=np.ones((N, 3), dtype=np.float32),
        angular_velocities=np.ones((N, 3), dtype=np.float32),
    )
    linear_velocities, angular_velocities = prims.get_velocities()
    isaacsim_test.check_array(linear_velocities, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(angular_velocities, shape=(N, 3), dtype=wp.float32)


def test_apply_forces(capsys: Any, engine: Any) -> None:
    """Test apply forces.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    prims.apply_forces(np.ones((N, 3), dtype=np.float32))


def test_apply_forces_and_torques_at_positions(capsys: Any, engine: Any) -> None:
    """Test apply forces and torques at positions.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    prims.apply_forces_and_torques_at_positions(
        forces=np.ones((N, 3), dtype=np.float32),
        torques=np.ones((N, 3), dtype=np.float32),
        positions=np.ones((N, 3), dtype=np.float32),
    )


def test_masses(capsys: Any, engine: Any) -> None:
    """Test masses.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    # get values
    output = prims.get_masses()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.float32)
    # round-trip values
    prims.set_masses(np.ones((N, 1), dtype=np.float32))
    output = prims.get_masses()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.float32)
    # inverse masses
    output = prims.get_masses(inverse=True)
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.float32)


def test_inertias(capsys: Any, engine: Any) -> None:
    """Test inertias.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    output = prims.get_inertias()
    isaacsim_test.check_array(output, shape=(N, 9), dtype=wp.float32)
    # round-trip
    prims.set_inertias(np.ones((N, 9), dtype=np.float32))
    output = prims.get_inertias()
    isaacsim_test.check_array(output, shape=(N, 9), dtype=wp.float32)
    # inverse inertias
    output = prims.get_inertias(inverse=True)
    isaacsim_test.check_array(output, shape=(N, 9), dtype=wp.float32)


def test_coms(capsys: Any, engine: Any) -> None:
    """Test coms.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    positions, orientations = prims.get_coms()
    isaacsim_test.check_array(positions, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, 4), dtype=wp.float32)
    # round-trip
    prims.set_coms(
        positions=np.ones((N, 3), dtype=np.float32),
        orientations=np.ones((N, 4), dtype=np.float32),
    )
    positions, orientations = prims.get_coms()
    isaacsim_test.check_array(positions, shape=(N, 3), dtype=wp.float32)
    isaacsim_test.check_array(orientations, shape=(N, 4), dtype=wp.float32)


def test_enabled_rigid_bodies(capsys: Any, engine: Any) -> None:
    """Test enabled rigid bodies.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    output = prims.get_enabled_rigid_bodies()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.uint8)
    # round-trip
    prims.set_enabled_rigid_bodies(np.zeros((N, 1), dtype=np.uint8))
    output = prims.get_enabled_rigid_bodies()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.uint8)


def test_enabled_gravities(capsys: Any, engine: Any) -> None:
    """Test enabled gravities.

    Args:
        capsys: Pytest output-capture fixture.
        engine: Physics engine supplied by the test fixture.

    """
    prims = RigidBodyEntity("engine", PATHS)
    output = prims.get_enabled_gravities()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.uint8)
    # round-trip
    prims.set_enabled_gravities(np.zeros((N, 1), dtype=np.uint8))
    output = prims.get_enabled_gravities()
    isaacsim_test.check_array(output, shape=(N, 1), dtype=wp.uint8)
