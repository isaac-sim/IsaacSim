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

"""Validate how the OvPhysX entity factories read their construction options.

The option values a caller supplies reach the engine only once a simulation is live, so the rejection
paths are exercised from a running scenario. The values that are accepted are covered by the contact and
SDF scenarios, which pass their filters and capacities through the same options.
"""

from __future__ import annotations

import os
import sys

import _physics_setup
import pytest

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _legacy_runner import cpu_device, run_scenario  # noqa: E402
from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    SimulationEntities,
    Transform,
)


class _RejectsMalformedOptionsScenario(GridTestBase):
    """Scenario that checks how the engine answers option values it cannot use.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=2), SimParams(), device_params)
        self.create_rigid_ball(self.env_template_path.AppendChild("ball"), Transform((0.0, 0.0, 0.5)), 0.15)

    def on_start(self, sim: SimulationEntities) -> None:
        """Check that malformed options are refused and absent ones fall back to the defaults.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        import isaacsim.physics.manager.impl.tensors as t

        # A wrong-typed option is a caller mistake, not a reason to silently use the default.
        with pytest.raises(Exception):
            t.create_entity("ovphysx", "rigid-contact", "/envs/*/ball", {"max-contact-data-count": "many"})
        with pytest.raises(Exception):
            t.create_entity("ovphysx", "rigid-contact", "/envs/*/ball", {"filter-patterns": 7})
        with pytest.raises(Exception):
            t.create_entity("ovphysx", "sdf-shape", "/envs/*/ball", {"num-points": 1.5})

        # A count the engine's int cannot hold would otherwise wrap to a small or negative capacity.
        with pytest.raises(Exception):
            t.create_entity("ovphysx", "rigid-contact", "/envs/*/ball", {"max-contact-data-count": 2**40})
        with pytest.raises(Exception):
            t.create_entity("ovphysx", "rigid-contact", "/envs/*/ball", {"max-contact-data-count": -1})

        # An option this entity does not read is ignored rather than rejected.
        view = t.create_entity("ovphysx", "rigid-body", "/envs/*/ball", {"num-points": 4})
        assert view is not None
        assert view.count == 2

        # Omitting the options entirely still builds a usable view.
        assert t.create_entity("ovphysx", "rigid-contact", "/envs/*/ball") is not None

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Accept the unused per-step callback required by the scenario harness.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """


class TestEntityOptions:
    """Validate OvPhysX construction-option handling."""

    def test_rejects_malformed_options_ovphysx_cc(self) -> None:
        """Check option rejection against a live CPU simulation."""
        run_scenario(self, _RejectsMalformedOptionsScenario, "ovphysx", cpu_device())


class TestTensorDeviceOrdinal:
    """Validate the backend-wide tensor-device declaration."""

    def test_declared_ordinal_round_trips(self) -> None:
        """Check that the declared ordinal is reported back and restored afterwards."""
        original = _physics_setup.get_tensor_device_ordinal()
        try:
            _physics_setup.set_tensor_device_ordinal(2)
            assert _physics_setup.get_tensor_device_ordinal() == 2
            _physics_setup.set_tensor_device_ordinal(-1)
            assert _physics_setup.get_tensor_device_ordinal() == -1
        finally:
            _physics_setup.set_tensor_device_ordinal(original)
