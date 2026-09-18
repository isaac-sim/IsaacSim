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

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.prims import Articulation

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

USD_PATH = isaacsim_test.resolve_resource_path("ant.usda")

# the referenced articulation ('ant.usda') is composed of 8 revolute joints (all of them DOFs) and 9 links
NUM_PRIMS = 5
NUM_DOFS = 8
NUM_LINKS = 9

"""
Utility functions.
"""


def _get_paths(stage: Any) -> Any:
    for path in [f"/World/Prim{i}" for i in range(NUM_PRIMS)]:
        stage.add_reference(USD_PATH, path)
    return "/World/Prim.*"


def _float_arrays(shape: Any, *, min_value: Any = -1e6, max_value: Any = 1e6) -> Any:
    return hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=shape,
        elements=isaacsim_test.finite_float_elements(min_value=min_value, max_value=max_value),
    )


"""
Test cases.
"""


def test_metadata(stage: Any) -> None:
    """Test metadata.

    Args:
        stage: Stage used by the test.
    """
    prims = Articulation(_get_paths(stage))
    # articulation roots
    assert len(prims.root_paths) == NUM_PRIMS
    # DOFs
    assert prims.num_dofs == NUM_DOFS
    assert len(prims.dof_names) == NUM_DOFS
    assert len(prims.dof_types) == NUM_DOFS
    assert prims.dof_types == ["rotation"] * NUM_DOFS
    assert [len(paths) for paths in prims.dof_paths] == [NUM_DOFS] * NUM_PRIMS
    # joints
    assert prims.num_joints == NUM_DOFS
    assert len(prims.joint_names) == NUM_DOFS
    assert prims.joint_types == ["revolute"] * NUM_DOFS
    assert [len(paths) for paths in prims.joint_paths] == [NUM_DOFS] * NUM_PRIMS
    # links
    assert prims.num_links == NUM_LINKS
    assert len(prims.link_names) == NUM_LINKS
    assert [len(paths) for paths in prims.link_paths] == [NUM_LINKS] * NUM_PRIMS


def test_indices_by_name(stage: Any) -> None:
    """Test indices by name.

    Args:
        stage: Stage used by the test.
    """
    prims = Articulation(_get_paths(stage))
    # all names, in order
    for names, indices in [
        (prims.dof_names, prims.get_dof_indices(prims.dof_names)),
        (prims.joint_names, prims.get_joint_indices(prims.joint_names)),
        (prims.link_names, prims.get_link_indices(prims.link_names)),
    ]:
        isaacsim_test.check_array(indices, shape=(len(names),), dtype=wp.int32)
        isaacsim_test.check_equal(indices.numpy(), np.arange(len(names), dtype=np.int32))
    # single name
    isaacsim_test.check_equal(prims.get_dof_indices(prims.dof_names[2]).numpy(), np.array([2], dtype=np.int32))
    isaacsim_test.check_equal(prims.get_link_indices([prims.link_names[3]]).numpy(), np.array([3], dtype=np.int32))
    # unknown names
    with pytest.raises(ValueError):
        prims.get_dof_indices("undefined")
    with pytest.raises(ValueError):
        prims.get_joint_indices("undefined")
    with pytest.raises(ValueError):
        prims.get_link_indices("undefined")


@hypothesis.given(
    lower=_float_arrays((NUM_PRIMS, NUM_DOFS)),
    upper=_float_arrays((NUM_PRIMS, NUM_DOFS)),
)
def test_dof_limits(stage: Any, lower: Any, upper: Any) -> None:
    """Test dof limits.

    Args:
        stage: Stage used by the test.
        lower: Lower values.
        upper: Upper values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output_lower, output_upper = prims.get_dof_limits()
    isaacsim_test.check_array(output_lower, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_array(output_upper, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_limits(lower, upper)
    output_lower, output_upper = prims.get_dof_limits()
    isaacsim_test.check_allclose(lower, output_lower)
    isaacsim_test.check_allclose(upper, output_upper)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_dof_limits()


@hypothesis.given(
    static_frictions=_float_arrays((NUM_PRIMS, NUM_DOFS)),
    dynamic_frictions=_float_arrays((NUM_PRIMS, NUM_DOFS)),
    viscous_frictions=_float_arrays((NUM_PRIMS, NUM_DOFS)),
)
def test_dof_friction_properties(
    stage: Any, static_frictions: Any, dynamic_frictions: Any, viscous_frictions: Any
) -> None:
    """Test dof friction properties.

    Args:
        stage: Stage used by the test.
        static_frictions: Static frictions values.
        dynamic_frictions: Dynamic frictions values.
        viscous_frictions: Viscous frictions values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_friction_properties()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_friction_properties(static_frictions, dynamic_frictions, viscous_frictions)
    output = prims.get_dof_friction_properties()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_allclose([static_frictions, dynamic_frictions, viscous_frictions], output)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_dof_friction_properties()


@hypothesis.given(
    speed_effort_gradients=_float_arrays((NUM_PRIMS, NUM_DOFS)),
    maximum_actuator_velocities=_float_arrays((NUM_PRIMS, NUM_DOFS)),
    velocity_dependent_resistances=_float_arrays((NUM_PRIMS, NUM_DOFS)),
)
def test_dof_drive_model_properties(
    stage: Any, speed_effort_gradients: Any, maximum_actuator_velocities: Any, velocity_dependent_resistances: Any
) -> None:
    """Test dof drive model properties.

    Args:
        stage: Stage used by the test.
        speed_effort_gradients: Speed effort gradients values.
        maximum_actuator_velocities: Maximum actuator velocities values.
        velocity_dependent_resistances: Velocity dependent resistances values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_drive_model_properties()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_drive_model_properties(
        speed_effort_gradients, maximum_actuator_velocities, velocity_dependent_resistances
    )
    output = prims.get_dof_drive_model_properties()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_allclose(
        [speed_effort_gradients, maximum_actuator_velocities, velocity_dependent_resistances], output
    )
    # undefined values
    with pytest.raises(ValueError):
        prims.set_dof_drive_model_properties()


def test_dof_drive_model_properties_without_envelope(stage: Any) -> None:
    """Test dof drive model properties without envelope.

    Args:
        stage: Stage used by the test.
    """
    prims = Articulation(_get_paths(stage))
    # DOFs without the performance envelope applied report zero without mutating the stage
    output = prims.get_dof_drive_model_properties()
    isaacsim_test.check_allclose([np.zeros((NUM_PRIMS, NUM_DOFS), dtype=np.float32)] * 3, output)


@hypothesis.given(armatures=_float_arrays((NUM_PRIMS, NUM_DOFS)))
def test_dof_armatures(stage: Any, armatures: Any) -> None:
    """Test dof armatures.

    Args:
        stage: Stage used by the test.
        armatures: Armatures values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_armatures()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_armatures(armatures)
    output = prims.get_dof_armatures()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_allclose(armatures, output)


@hypothesis.given(
    types=hypothesis.strategies.lists(
        hypothesis.strategies.lists(
            hypothesis.strategies.sampled_from(["force", "acceleration"]),
            min_size=NUM_DOFS,
            max_size=NUM_DOFS,
        ),
        min_size=NUM_PRIMS,
        max_size=NUM_PRIMS,
    ),
)
def test_dof_drive_types(stage: Any, types: Any) -> None:
    """Test dof drive types.

    Args:
        stage: Stage used by the test.
        types: Value type identifiers.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_drive_types()
    assert [len(row) for row in output] == [NUM_DOFS] * NUM_PRIMS
    # round-trip values
    prims.set_dof_drive_types(types)
    output = prims.get_dof_drive_types()
    isaacsim_test.check_equal(types, output)
    # broadcast a single type to all the prims/DOFs
    prims.set_dof_drive_types("acceleration")
    output = prims.get_dof_drive_types()
    isaacsim_test.check_equal([["acceleration"] * NUM_DOFS] * NUM_PRIMS, output)
    # broadcast a single row to all the prims
    prims.set_dof_drive_types([types[0]])
    output = prims.get_dof_drive_types()
    isaacsim_test.check_equal([types[0]] * NUM_PRIMS, output)
    # broadcast a single type per prim to all the DOFs
    prims.set_dof_drive_types([[row[0]] for row in types])
    output = prims.get_dof_drive_types()
    isaacsim_test.check_equal([[row[0]] * NUM_DOFS for row in types], output)
    # non-broadcastable values
    with pytest.raises(ValueError):
        prims.set_dof_drive_types(types[:2])
    with pytest.raises(ValueError):
        prims.set_dof_drive_types([row[:2] for row in types])


@hypothesis.given(max_velocities=_float_arrays((NUM_PRIMS, NUM_DOFS)))
def test_dof_max_velocities(stage: Any, max_velocities: Any) -> None:
    """Test dof max velocities.

    Args:
        stage: Stage used by the test.
        max_velocities: Max velocities values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_max_velocities()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_max_velocities(max_velocities)
    output = prims.get_dof_max_velocities()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_allclose(max_velocities, output)


@hypothesis.given(max_efforts=_float_arrays((NUM_PRIMS, NUM_DOFS)))
def test_dof_max_efforts(stage: Any, max_efforts: Any) -> None:
    """Test dof max efforts.

    Args:
        stage: Stage used by the test.
        max_efforts: Maximum efforts value.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_max_efforts()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_max_efforts(max_efforts)
    output = prims.get_dof_max_efforts()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_allclose(max_efforts, output)


@hypothesis.given(
    stiffnesses=_float_arrays((NUM_PRIMS, NUM_DOFS)),
    dampings=_float_arrays((NUM_PRIMS, NUM_DOFS)),
)
def test_dof_gains(stage: Any, stiffnesses: Any, dampings: Any) -> None:
    """Test dof gains.

    Args:
        stage: Stage used by the test.
        stiffnesses: Stiffnesses values.
        dampings: Dampings values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output_stiffnesses, output_dampings = prims.get_dof_gains()
    isaacsim_test.check_array(output_stiffnesses, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_array(output_dampings, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_gains(stiffnesses, dampings)
    output_stiffnesses, output_dampings = prims.get_dof_gains()
    isaacsim_test.check_allclose(stiffnesses, output_stiffnesses)
    isaacsim_test.check_allclose(dampings, output_dampings)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_dof_gains()


@hypothesis.given(
    stiffnesses=_float_arrays((NUM_PRIMS, NUM_DOFS), min_value=1.0, max_value=1e6),
    dampings=_float_arrays((NUM_PRIMS, NUM_DOFS), min_value=1.0, max_value=1e6),
)
def test_switch_dof_control_mode(stage: Any, stiffnesses: Any, dampings: Any) -> None:
    """Test switch dof control mode.

    Args:
        stage: Stage used by the test.
        stiffnesses: Stiffnesses values.
        dampings: Dampings values.
    """
    prims = Articulation(_get_paths(stage))
    zeros = np.zeros((NUM_PRIMS, NUM_DOFS), dtype=np.float32)
    # author the gains that the 'position' control mode is expected to restore
    prims.set_dof_gains(stiffnesses, dampings)
    # velocity control: null stiffnesses, default dampings
    prims.switch_dof_control_mode("velocity")
    output_stiffnesses, output_dampings = prims.get_dof_gains()
    isaacsim_test.check_allclose(zeros, output_stiffnesses)
    isaacsim_test.check_allclose(dampings, output_dampings)
    # effort control: null gains
    prims.switch_dof_control_mode("effort")
    output_stiffnesses, output_dampings = prims.get_dof_gains()
    isaacsim_test.check_allclose(zeros, output_stiffnesses)
    isaacsim_test.check_allclose(zeros, output_dampings)
    # position control: default gains
    prims.switch_dof_control_mode("position")
    output_stiffnesses, output_dampings = prims.get_dof_gains()
    isaacsim_test.check_allclose(stiffnesses, output_stiffnesses)
    isaacsim_test.check_allclose(dampings, output_dampings)
    # invalid control mode
    with pytest.raises(ValueError):
        prims.switch_dof_control_mode("undefined")


@hypothesis.given(positions=_float_arrays((NUM_PRIMS, NUM_DOFS)))
def test_dof_position_targets(stage: Any, positions: Any) -> None:
    """Test dof position targets.

    Args:
        stage: Stage used by the test.
        positions: Positions values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_position_targets()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_position_targets(positions)
    output = prims.get_dof_position_targets()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_allclose(positions, output)


@hypothesis.given(velocities=_float_arrays((NUM_PRIMS, NUM_DOFS)))
def test_dof_velocity_targets(stage: Any, velocities: Any) -> None:
    """Test dof velocity targets.

    Args:
        stage: Stage used by the test.
        velocities: Velocities values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_dof_velocity_targets()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    # round-trip values
    prims.set_dof_velocity_targets(velocities)
    output = prims.get_dof_velocity_targets()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_DOFS), dtype=wp.float32)
    isaacsim_test.check_allclose(velocities, output)


@hypothesis.given(masses=_float_arrays((NUM_PRIMS, NUM_LINKS)))
def test_link_masses(stage: Any, masses: Any) -> None:
    """Test link masses.

    Args:
        stage: Stage used by the test.
        masses: Masses values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_link_masses()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_LINKS), dtype=wp.float32)
    # round-trip values
    prims.set_link_masses(masses)
    output = prims.get_link_masses()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_LINKS), dtype=wp.float32)
    isaacsim_test.check_allclose(masses, output)
    # inverse masses
    output = prims.get_link_masses(inverse=True)
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_LINKS), dtype=wp.float32)
    isaacsim_test.check_allclose(1.0 / (masses + 1e-8), output)


@hypothesis.given(values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(NUM_PRIMS, NUM_LINKS)))
def test_link_enabled_gravities(stage: Any, values: Any) -> None:
    """Test link enabled gravities.

    Args:
        stage: Stage used by the test.
        values: Values exercised by the test.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_link_enabled_gravities()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_LINKS), dtype=wp.bool)
    # round-trip values
    prims.set_link_enabled_gravities(values)
    output = prims.get_link_enabled_gravities()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, NUM_LINKS), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


@hypothesis.given(
    position_counts=hypothesis.extra.numpy.arrays(
        dtype=np.int32, shape=(NUM_PRIMS, 1), elements=hypothesis.strategies.integers(min_value=0, max_value=255)
    ),
    velocity_counts=hypothesis.extra.numpy.arrays(
        dtype=np.int32, shape=(NUM_PRIMS, 1), elements=hypothesis.strategies.integers(min_value=0, max_value=255)
    ),
)
def test_solver_iteration_counts(stage: Any, position_counts: Any, velocity_counts: Any) -> None:
    """Test solver iteration counts.

    Args:
        stage: Stage used by the test.
        position_counts: Position counts values.
        velocity_counts: Velocity counts values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output_position_counts, output_velocity_counts = prims.get_solver_iteration_counts()
    isaacsim_test.check_array(output_position_counts, shape=(NUM_PRIMS, 1), dtype=wp.int32)
    isaacsim_test.check_array(output_velocity_counts, shape=(NUM_PRIMS, 1), dtype=wp.int32)
    # round-trip values
    prims.set_solver_iteration_counts(position_counts, velocity_counts)
    output_position_counts, output_velocity_counts = prims.get_solver_iteration_counts()
    isaacsim_test.check_equal(position_counts, output_position_counts)
    isaacsim_test.check_equal(velocity_counts, output_velocity_counts)
    # undefined values
    with pytest.raises(ValueError):
        prims.set_solver_iteration_counts()


@hypothesis.given(thresholds=_float_arrays((NUM_PRIMS, 1)))
def test_stabilization_thresholds(stage: Any, thresholds: Any) -> None:
    """Test stabilization thresholds.

    Args:
        stage: Stage used by the test.
        thresholds: Thresholds values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_stabilization_thresholds()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, 1), dtype=wp.float32)
    # round-trip values
    prims.set_stabilization_thresholds(thresholds)
    output = prims.get_stabilization_thresholds()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(thresholds, output)


@hypothesis.given(values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(NUM_PRIMS, 1)))
def test_enabled_self_collisions(stage: Any, values: Any) -> None:
    """Test enabled self collisions.

    Args:
        stage: Stage used by the test.
        values: Values exercised by the test.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_enabled_self_collisions()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_self_collisions(values)
    output = prims.get_enabled_self_collisions()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


@hypothesis.given(thresholds=_float_arrays((NUM_PRIMS, 1)))
def test_sleep_thresholds(stage: Any, thresholds: Any) -> None:
    """Test sleep thresholds.

    Args:
        stage: Stage used by the test.
        thresholds: Thresholds values.
    """
    prims = Articulation(_get_paths(stage))
    # get values
    output = prims.get_sleep_thresholds()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, 1), dtype=wp.float32)
    # round-trip values
    prims.set_sleep_thresholds(thresholds)
    output = prims.get_sleep_thresholds()
    isaacsim_test.check_array(output, shape=(NUM_PRIMS, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(thresholds, output)


@hypothesis.given(armatures=_float_arrays((2, 3)))
def test_indexed_dofs(stage: Any, armatures: Any) -> None:
    """Test indexed dofs.

    Args:
        stage: Stage used by the test.
        armatures: Armatures values.
    """
    prims = Articulation(_get_paths(stage))
    indices = [0, 2]
    dof_indices = [1, 3, 5]
    prims.set_dof_armatures(np.zeros((NUM_PRIMS, NUM_DOFS), dtype=np.float32))
    # get/set the selected (prim, DOF) pairs only
    prims.set_dof_armatures(armatures, indices=indices, dof_indices=dof_indices)
    output = prims.get_dof_armatures(indices=indices, dof_indices=dof_indices)
    isaacsim_test.check_array(output, shape=(2, 3), dtype=wp.float32)
    isaacsim_test.check_allclose(armatures, output)
    # the non-selected pairs are left untouched
    expected = np.zeros((NUM_PRIMS, NUM_DOFS), dtype=np.float32)
    expected[np.ix_(indices, dof_indices)] = armatures
    isaacsim_test.check_allclose(expected, prims.get_dof_armatures())
    # negative indices count from the end
    output = prims.get_dof_armatures(indices=[-NUM_PRIMS], dof_indices=[-NUM_DOFS])
    isaacsim_test.check_allclose(expected[:1, :1], output)
    # out-of-range indices
    with pytest.raises(IndexError):
        prims.get_dof_armatures(dof_indices=[NUM_DOFS])
    with pytest.raises(IndexError):
        prims.get_link_masses(link_indices=[NUM_LINKS])


@hypothesis.given(masses=_float_arrays((NUM_PRIMS, 1)))
def test_value_broadcasting(stage: Any, masses: Any) -> None:
    """Test value broadcasting.

    Args:
        stage: Stage used by the test.
        masses: Masses values.
    """
    prims = Articulation(_get_paths(stage))
    # broadcast a single value to all the prims/links
    prims.set_link_masses(masses[:1, :1])
    isaacsim_test.check_allclose(np.broadcast_to(masses[:1, :1], (NUM_PRIMS, NUM_LINKS)), prims.get_link_masses())
    # broadcast a single value per prim to all the links
    prims.set_link_masses(masses)
    isaacsim_test.check_allclose(np.broadcast_to(masses, (NUM_PRIMS, NUM_LINKS)), prims.get_link_masses())


def test_are_of_type(stage: Any) -> None:
    """Test are of type.

    Args:
        stage: Stage used by the test.
    """
    stage.define_prim("/World/Robots", "Xform")
    stage.add_reference(USD_PATH, "/World/Robots/Prim0")
    stage.define_prim("/World/Other", "Cube")
    stage.define_prim("/World/Scope", "Scope")
    # the articulation root (/World/Robots/Prim0/torso) is only searched for on the prim itself and on its
    # direct children, so /World/Robots (two levels above the root) does not match; prims that are not
    # transformable never match
    paths = ["/World/Robots/Prim0/torso", "/World/Robots/Prim0", "/World/Robots", "/World/Other", "/World/Scope"]
    output = Articulation.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([[1], [1], [0], [0], [0]], dtype=np.bool_), output)
    # every flagged path is accepted by the constructor and resolves to the same articulation root
    assert Articulation("/World/Robots/Prim0/torso").root_paths == Articulation("/World/Robots/Prim0").root_paths
    # the constructor's search is unbounded, so it still accepts ancestors that are_of_type does not flag
    assert Articulation("/World/Robots").root_paths == Articulation("/World/Robots/Prim0").root_paths
    # non-existing prims
    with pytest.raises(RuntimeError):
        Articulation.are_of_type("/World/NonExistent")
