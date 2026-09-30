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

"""Test control behavior."""

from __future__ import annotations

from collections.abc import Callable

import isaacsim.physics.ovsim as ovsim
import pytest

authoring = ovsim.control.authoring
simulation = ovsim.control.simulation


@pytest.mark.parametrize(
    ("operation", "args"),
    (
        pytest.param(authoring.create_stage, (), id="create-stage"),
        pytest.param(authoring.open_stage, ("scene.usda",), id="open-stage"),
        pytest.param(authoring.save_stage, ("scene.usda",), id="save-stage"),
        pytest.param(authoring.import_stage_from_string, ("#usda 1.0",), id="import-stage"),
        pytest.param(authoring.export_stage_to_string, (), id="export-stage"),
        pytest.param(authoring.close_stage, (), id="close-stage"),
        pytest.param(authoring.add_reference_to_stage, ("asset.usda", "/World/Asset"), id="add-reference"),
        pytest.param(authoring.define_prim, ("/World/Cube",), id="define-prim"),
        pytest.param(authoring.move_prim, ("/World/Cube", "/World/MovedCube"), id="move-prim"),
        pytest.param(authoring.remove_prim, ("/World/Cube",), id="remove-prim"),
        pytest.param(
            authoring.create_prim_attribute,
            ("/World/Cube", "enabled", "bool"),
            id="create-attribute",
        ),
        pytest.param(
            authoring.remove_prim_attribute,
            ("/World/Cube", "enabled"),
            id="remove-attribute",
        ),
        pytest.param(authoring.set_parameter, ("physics", "example", True), id="set-parameter"),
        pytest.param(authoring.get_parameter, ("physics", "example"), id="get-parameter"),
    ),
)
def test_authoring_operations_report_unavailable(operation: Callable[..., object], args: tuple[object, ...]) -> None:
    """Report unsupported stage-authoring operations consistently.

    Args:
        operation: Authoring operation under test.
        args: Positional arguments for the operation.

    """
    with pytest.raises(RuntimeError, match="not implemented"):
        operation(*args)


def test_simulation_compatibility_lifecycle_is_noop() -> None:
    """Keep compatibility lifecycle operations safe for manual stepping clients."""
    simulation.play()
    simulation.pause()
    simulation.stop()


def test_simulation_pointer_parameter_round_trip() -> None:
    """Store and retrieve the native OVStage pointer parameter."""
    simulation.set_parameter("physics", "ovstage-stage-ptr", 0)

    assert simulation.get_parameter("physics", "ovstage-stage-ptr") == 0


@pytest.mark.parametrize(
    ("provider", "parameter_name", "value"),
    (
        pytest.param("rendering", "ovstage-stage-ptr", 0, id="provider"),
        pytest.param("physics", "unknown", 0, id="parameter"),
        pytest.param("physics", "ovstage-stage-ptr", "not-a-pointer", id="value-type"),
    ),
)
def test_simulation_parameter_validation(provider: str, parameter_name: str, value: object) -> None:
    """Reject unsupported simulation parameter requests.

    Args:
        provider: Provider name under test.
        parameter_name: Parameter name under test.
        value: Parameter value under test.

    """
    with pytest.raises(ValueError):
        simulation.set_parameter(provider, parameter_name, value)


@pytest.mark.parametrize(
    ("provider", "parameter_name"),
    (
        pytest.param("rendering", "ovstage-stage-ptr", id="provider"),
        pytest.param("physics", "unknown", id="parameter"),
    ),
)
def test_simulation_parameter_lookup_validation(provider: str, parameter_name: str) -> None:
    """Reject unsupported simulation parameter lookups.

    Args:
        provider: Provider name under test.
        parameter_name: Parameter name under test.

    """
    with pytest.raises(ValueError):
        simulation.get_parameter(provider, parameter_name)
