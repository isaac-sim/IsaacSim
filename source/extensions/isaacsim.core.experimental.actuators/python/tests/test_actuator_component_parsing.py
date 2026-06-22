# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies that Newton actuator controller, clamping, and delay components authored in USD can be discovered and executed. Covers PD, PID, max effort, DC motor, position-based clamping, and delay component parsing."""

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.actuators import ArticulationActuators
from isaacsim.storage.native import get_assets_root_path
from newton.actuators import (
    ClampingDCMotor,
    ClampingMaxEffort,
    ClampingPositionBased,
    ControllerPD,
    ControllerPID,
    Delay,
)
from pxr import Sdf, Usd

_SIMPLE_ART_REL_PATH = "Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
_ART_ROOT = "/World/A_0"
_REVOLUTE_JOINT_PATH = "/World/A_0/Arm/RevoluteJoint"

_SMOKE_STEPS = 10


class TestActuatorComponentParsing(omni.kit.test.AsyncTestCase):
    """Smoke tests: one per Newton actuator component type (controller / clamping / delay).

    Every test verifies:

    1. The component type instantiated from the USD description matches the expected class.
    2. The simulation runs for `_SMOKE_STEPS` physics frames without raising an exception.
    """

    async def setUp(self) -> None:
        """Prepare the actuator component parsing test stage."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
        usd_path = f"{get_assets_root_path()}/{_SIMPLE_ART_REL_PATH}"
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=_ART_ROOT)
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        """Clean up the actuator component parsing test stage."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _author_pd_prim(self, path: str, target: Sdf.Path) -> Usd.Prim:
        """Define a `NewtonActuator` prim with a baseline `NewtonPDControlAPI`.

        Args:
            path: USD path at which to define the `NewtonActuator` prim.
            target: Absolute `Sdf.Path` of the joint to actuate.

        Returns:
            The newly defined prim, suitable for adding additional applied schemas.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        prim = stage.DefinePrim(path, "NewtonActuator")
        prim.AddAppliedSchema("NewtonPDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(100.0)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(10.0)
        prim.CreateRelationship("newton:targets").SetTargets([target])
        return prim

    async def _run_smoke_test(self, actuated: ArticulationActuators) -> None:
        """Play physics and step `_SMOKE_STEPS` frames.

        Args:
            actuated: The `ArticulationActuators` under test.  Must have `auto_step_pre_physics=True`.
        """
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        for _ in range(_SMOKE_STEPS):
            await omni.kit.app.get_app().next_update_async()

    # ------------------------------------------------------------------
    # Controllers
    # ------------------------------------------------------------------

    async def test_controller_pd_parses_and_runs(self) -> None:
        """Verify that a `NewtonPDControlAPI` prim yields a `ControllerPD` and runs without error."""
        target = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_prim(f"{_ART_ROOT}/Actuators/act", target)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self.assertIsInstance(actuated.actuators[0].controller, ControllerPD)
            await self._run_smoke_test(actuated)
        finally:
            actuated.close()

    async def test_controller_pid_parses_and_runs(self) -> None:
        """Verify that a `NewtonPIDControlAPI` prim yields a `ControllerPID` and runs without error."""
        target = Sdf.Path(_REVOLUTE_JOINT_PATH)
        stage = stage_utils.get_current_stage(backend="usd")
        prim = stage.DefinePrim(f"{_ART_ROOT}/Actuators/act", "NewtonActuator")
        prim.AddAppliedSchema("NewtonPIDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(100.0)
        prim.CreateAttribute("newton:ki", Sdf.ValueTypeNames.Float).Set(10.0)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(5.0)
        prim.CreateRelationship("newton:targets").SetTargets([target])

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self.assertIsInstance(actuated.actuators[0].controller, ControllerPID)
            await self._run_smoke_test(actuated)
        finally:
            actuated.close()

    # TODO: Add tests for `ControllerNeuralMLP` and `ControllerNeuralLSTM` once test checkpoint
    # files are available in the repo.  Both are parsed via `NewtonNeuralControlAPI` — the parser
    # reads `newton:modelPath` and inspects the `model_type` key in the checkpoint's metadata dict
    # to choose between the two classes.

    # ------------------------------------------------------------------
    # Clamping
    # ------------------------------------------------------------------

    async def test_clamping_max_effort_parses_and_runs(self) -> None:
        """Verify that a `NewtonMaxEffortClampingAPI` prim yields a `ClampingMaxEffort` and runs without error."""
        target = Sdf.Path(_REVOLUTE_JOINT_PATH)
        prim = self._author_pd_prim(f"{_ART_ROOT}/Actuators/act", target)
        prim.AddAppliedSchema("NewtonMaxEffortClampingAPI")
        prim.CreateAttribute("newton:maxEffort", Sdf.ValueTypeNames.Float).Set(50.0)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self.assertIsInstance(actuated.actuators[0].clamping[0], ClampingMaxEffort)
            await self._run_smoke_test(actuated)
        finally:
            actuated.close()

    async def test_clamping_dc_motor_parses_and_runs(self) -> None:
        """Verify that a `NewtonDCMotorClampingAPI` prim yields a `ClampingDCMotor` and runs without error."""
        target = Sdf.Path(_REVOLUTE_JOINT_PATH)
        prim = self._author_pd_prim(f"{_ART_ROOT}/Actuators/act", target)
        prim.AddAppliedSchema("NewtonDCMotorClampingAPI")
        prim.CreateAttribute("newton:saturationEffort", Sdf.ValueTypeNames.Float).Set(100.0)
        prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(10.0)
        prim.CreateAttribute("newton:maxMotorEffort", Sdf.ValueTypeNames.Float).Set(80.0)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self.assertIsInstance(actuated.actuators[0].clamping[0], ClampingDCMotor)
            await self._run_smoke_test(actuated)
        finally:
            actuated.close()

    async def test_clamping_position_based_parses_and_runs(self) -> None:
        """Verify that a `NewtonPositionBasedClampingAPI` prim yields a `ClampingPositionBased` and runs without error."""
        target = Sdf.Path(_REVOLUTE_JOINT_PATH)
        prim = self._author_pd_prim(f"{_ART_ROOT}/Actuators/act", target)
        prim.AddAppliedSchema("NewtonPositionBasedClampingAPI")
        prim.CreateAttribute("newton:lookupPositions", Sdf.ValueTypeNames.FloatArray).Set([0.0, 0.5, 1.0])
        prim.CreateAttribute("newton:lookupEfforts", Sdf.ValueTypeNames.FloatArray).Set([50.0, 40.0, 20.0])

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self.assertIsInstance(actuated.actuators[0].clamping[0], ClampingPositionBased)
            await self._run_smoke_test(actuated)
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Delay
    # ------------------------------------------------------------------

    async def test_delay_parses_and_runs(self) -> None:
        """Verify that a `NewtonActuatorDelayAPI` prim yields a `Delay` and runs without error."""
        target = Sdf.Path(_REVOLUTE_JOINT_PATH)
        prim = self._author_pd_prim(f"{_ART_ROOT}/Actuators/act", target)
        prim.AddAppliedSchema("NewtonActuatorDelayAPI")
        prim.CreateAttribute("newton:delaySteps", Sdf.ValueTypeNames.Int).Set(3)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self.assertIsInstance(actuated.actuators[0].delay, Delay)
            await self._run_smoke_test(actuated)
        finally:
            actuated.close()
