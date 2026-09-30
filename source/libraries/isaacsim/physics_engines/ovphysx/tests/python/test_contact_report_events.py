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

"""Validate the ovphysx contact-report event callback.

This is the ``SimulationFunctions`` callback surface -- subscribers registered
through ``subscribe_physics_contact_report_events`` receive per-contact event
headers, contact points and friction anchors after a step. It is distinct from
the contact tensor views (``net-contact-forces`` / ``contact-data``), which are
covered by ``test_contacts.py``.

The scenario rests a box on the ground plane with ``PhysxContactReportAPI``
applied, so every step produces contacts.
"""

from __future__ import annotations

import os
import sys

import _physics_setup  # noqa: F401  -- registers the ovphysx backend
from physx_usd_schemas import PhysxSchema
from pxr import Gf

_TENSORS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

import isaacsim.physics.manager as physics_manager  # noqa: E402
from _legacy_runner import cpu_device  # noqa: E402
from _scenario import GridParams, GridTestBase, RunnerInMemory, SimParams, Transform  # noqa: E402


class _ContactReportScenario(GridTestBase):
    """Rest a reporting box on the ground and record contact-report callbacks.

    Args:
        test_case: Test case that owns the scenario.
        device_params: Simulation device parameters.

    """

    def __init__(self, test_case: object, device_params: object) -> None:
        super().__init__(test_case, GridParams(4, 1.0), SimParams(), device_params)
        self.maxsteps = 60

        actor_path = self.env_template_path.AppendChild("box")
        self.create_rigid_box(actor_path, Transform((0.0, 0.0, 0.15)), Gf.Vec3f(0.3, 0.3, 0.3))
        self._contact_body_paths = [actor_path]
        self._apply_engine_specifics()

        self.events: list[tuple[int, int, int]] = []
        self.subscription = None

    def _apply_engine_specifics(self) -> None:
        """Enable PhysX contact reporting on every replicated box."""
        for template_path in self._contact_body_paths:
            relative = str(template_path)[len(str(self.env_template_path)) :].lstrip("/")
            for env_index in range(self.num_envs):
                prim = self.stage.GetPrimAtPath(f"/envs/env{env_index}/{relative}")
                if not prim:
                    continue
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0)
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)

    def record(self, headers: object, data: object, anchors: object) -> None:
        """Record the sizes delivered to one contact-report callback.

        Args:
            headers: Contact report headers.
            data: Contact report payload data.
            anchors: Contact friction-anchor data.

        """
        self.events.append((len(headers), len(data), len(anchors)))

    def subscribe(self) -> None:
        """Subscribe to contact-report events for this scenario."""
        self.subscription = physics_manager.subscribe_physics_contact_report_events(self.record)

    def unsubscribe(self) -> None:
        """Release the subscription if one is held."""
        if self.subscription is not None:
            self.subscription.unsubscribe()
            self.subscription = None

    def on_start(self, sim: object) -> None:
        """Subscribe before the first step.

        Args:
            sim: Active simulation view.

        """
        self.subscribe()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """No per-step assertions; the tests inspect ``events`` afterwards.

        Args:
            sim: Active simulation view.
            stepno: Zero-based simulation step number.
            dt: Simulation time step.

        """


def _contact_events(scenario: _ContactReportScenario) -> list[tuple[int, int, int]]:
    """Return the recorded callbacks that carried at least one pair and one contact point.

    Args:
        scenario: Physics scenario to execute.

    Returns:
        The resulting value.

    """
    return [event for event in scenario.events if event[0] > 0 and event[1] > 0]


class TestOvPhysxContactReportEvents:
    """Contact-report callbacks must deliver real contact data on both step paths."""

    def test_async_step_delivers_contact_data(self) -> None:
        """A box resting on the ground reports contacts through the async path."""
        scenario = _ContactReportScenario(self, cpu_device())
        runner = RunnerInMemory(scenario, engine="ovphysx")
        runner.start()
        try:
            runner.simulate()
        finally:
            scenario.unsubscribe()
            runner.stop()

        assert scenario.events, "no contact-report callback fired"
        assert _contact_events(scenario), f"every callback carried zero contacts: {scenario.events[:5]}"

    def test_sync_step_delivers_contact_data(self) -> None:
        """The synchronous step path must notify contact-report subscribers too."""
        scenario = _ContactReportScenario(self, cpu_device())
        runner = RunnerInMemory(scenario, engine="ovphysx")
        runner.start()
        try:
            dt = 1.0 / scenario.sim_params.time_steps_per_second
            current_time = 0.0
            for _ in range(scenario.maxsteps):
                physics_manager.simulate(dt, current_time)
                current_time += dt
        finally:
            scenario.unsubscribe()
            runner.stop()

        assert scenario.events, "the synchronous step path fired no contact-report callback"
        assert _contact_events(scenario), f"every callback carried zero contacts: {scenario.events[:5]}"

    def test_fetch_without_pending_step_reports_nothing(self) -> None:
        """A fetch that drains no step must not deliver a contact-report event."""
        scenario = _ContactReportScenario(self, cpu_device())
        runner = RunnerInMemory(scenario, engine="ovphysx")
        runner.start()
        try:
            scenario.events.clear()
            physics_manager.fetch_results()
        finally:
            scenario.unsubscribe()
            runner.stop()

        assert not scenario.events, f"fetch with nothing pending delivered {scenario.events}"
