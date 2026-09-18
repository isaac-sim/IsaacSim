# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test physics manager behavior."""

from collections.abc import Iterator
from typing import Any

import isaacsim.physics.registration as registration
import pytest
from isaacsim.physics.manager import PhysicsEvent, PhysicsManager

from .utils import MockSimulator, setup_simulation_fns


@pytest.fixture(scope="function")
def physics_engines(request: Any) -> Iterator[None]:
    """Register and unregister the requested number of dummy physics engines.

    Args:
        request: Pytest fixture request.

    Yields:
        Control while the physics engines are registered.

    """

    def _register_physics_engines(num_engines: Any) -> list:
        simulation_ids = []
        for i in range(num_engines):
            # create a mock simulator for each engine
            simulator = MockSimulator()
            simulators.append(simulator)
            # create, setup and register the simulation
            simulation = registration.Simulation()
            setup_simulation_fns(simulation, simulator)
            simulation_id = registration.register_simulation(simulation, f"physics_engine_{i}")
            simulator._simulation_id = simulation_id
            simulation_ids.append(simulation_id)
        return simulation_ids

    def _unregister_physics_engines(simulation_ids: list) -> None:
        for simulation_id in simulation_ids:
            registration.unregister_simulation(simulation_id)
        simulators.clear()

    # physics engines lifecycle management
    simulators = []
    simulation_ids = _register_physics_engines(request.param)
    yield
    _unregister_physics_engines(simulation_ids)


"""
Test cases.
"""


def test_physics_event_enum(capsys: Any) -> None:
    """Test physics event enum.

    Args:
        capsys: Pytest output-capture fixture.

    """
    items = [item for item in dir(PhysicsEvent) if not item.startswith("_")]
    assert len(items) == 5, "Number of physics events does not match the expected ones"


def test_physics_manager_singleton(capsys: Any) -> None:
    # check that the constructor is not defined
    """Test physics manager singleton.

    Args:
        capsys: Pytest output-capture fixture.

    """
    with pytest.raises(TypeError):
        PhysicsManager()
    # check that the singleton instance is always the same
    physics_manager = PhysicsManager.get_instance()
    assert physics_manager is not None, "Singleton instance is None"
    assert physics_manager is PhysicsManager.get_instance(), "Singleton instance is not the same"


@pytest.mark.parametrize("physics_engines", [1], indirect=True)
def test_physics_manager_step(capsys: Any, physics_engines: Any) -> None:
    """Test physics manager step.

    Args:
        capsys: Pytest output-capture fixture.
        physics_engines: Physics engines supplied by the test fixture.

    """

    def callback(step: Any, steps: Any) -> Any:
        assert step, "Step should not be 0 in any case"
        return step != 3

    def pre_step_callback(payload: Any) -> None:
        nonlocal pre_step_count
        pre_step_count += 1

    def post_step_callback(payload: Any) -> None:
        nonlocal post_step_count
        post_step_count += 1

    physics_manager = PhysicsManager.get_instance()

    with pytest.raises(RuntimeError):
        physics_manager.step()
    physics_manager.switch_physics_engine("physics_engine_0")
    physics_manager.initialize(0, 1)

    pre_step_count = 0
    post_step_count = 0
    physics_manager.register_callback(pre_step_callback, PhysicsEvent.PHYSICS_PRE_STEP)
    physics_manager.register_callback(post_step_callback, PhysicsEvent.PHYSICS_POST_STEP)

    # default number of steps (1)
    assert physics_manager.step() == 1, "Step should return 1"
    assert pre_step_count == 1, "Pre-step count should be 1"
    assert post_step_count == 1, "Post-step count should be 1"
    # several steps at once
    assert physics_manager.step(steps=5) == 5, "Step should return 5"
    assert pre_step_count == 6, "Pre-step count should be 6 (1 + 5)"
    assert post_step_count == 6, "Post-step count should be 6 (1 + 5)"
    # several steps at once (with early stopping)
    assert physics_manager.step(steps=5, callback=callback) == 3, "Step should return 3"
    assert pre_step_count == 9, "Pre-step count should be 9 (1 + 5 + 3)"
    assert post_step_count == 9, "Post-step count should be 9 (1 + 5 + 3)"


@pytest.mark.parametrize("physics_engines", [3], indirect=True)
def test_physics_manager_physics_engine_bindings(capsys: Any, physics_engines: Any) -> None:
    """Test physics manager physics engine bindings.

    Args:
        capsys: Pytest output-capture fixture.
        physics_engines: Physics engines supplied by the test fixture.

    """

    def _check_active_engine(status: tuple[bool]) -> None:
        engines = physics_manager.get_registered_physics_engines()
        assert len(engines) == len(status), "Number of engines does not match the expected ones"
        current_status = tuple([engine[1] for engine in sorted(engines, key=lambda x: x[0])])
        assert (
            status == current_status
        ), f"Engines activation status does not match. Expected: {status}, current: {current_status}"

    # FIXME: ovphysx is registered globally, so it will be counted as one of the engines (the first one)
    physics_manager = PhysicsManager.get_instance()
    _check_active_engine((False, True, True, True))

    # switch to valid engines
    assert physics_manager.switch_physics_engine("physics_engine_0"), "Switching engine should succeed"
    _check_active_engine((False, True, False, False))
    assert physics_manager.switch_physics_engine("physics_engine_1"), "Switching engine should succeed"
    _check_active_engine((False, False, True, False))
    assert physics_manager.switch_physics_engine("physics_engine_2"), "Switching engine should succeed"
    _check_active_engine((False, False, False, True))

    # switch to an invalid engine
    assert (
        physics_manager.switch_physics_engine("__invalid_physics_engine__") is False
    ), "Switching to an unregistered physics engine should fail"
    _check_active_engine((False, False, False, True))
