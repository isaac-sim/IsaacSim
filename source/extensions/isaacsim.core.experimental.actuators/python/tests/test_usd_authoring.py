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

"""Verifies USD authoring helpers for Newton actuator components. Covers controller, clamping, and delay prim creation, authored attributes and relationships, target validation, overwrite behavior, discovery round trips, and invalid configuration errors."""

from __future__ import annotations

import math
import os
import tempfile

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.experimental.actuators import (
    ArticulationActuators,
    DCMotorClampingConfig,
    DelayConfig,
    MaxEffortClampingConfig,
    NeuralControlConfig,
    PDControlConfig,
    PIDControlConfig,
    PositionBasedClampingConfig,
    add_actuator,
)
from isaacsim.storage.native import get_assets_root_path
from newton.actuators import (
    ClampingDCMotor,
    ClampingMaxEffort,
    ClampingPositionBased,
    ControllerNeuralLSTM,
    ControllerNeuralMLP,
    ControllerPD,
    ControllerPID,
    Delay,
)
from pxr import Usd

_SIMPLE_ART_REL_PATH = "Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
_ART_ROOT = "/World/A_0"
_REVOLUTE_JOINT_NAME = "RevoluteJoint"
_PRISMATIC_JOINT_NAME = "PrismaticJoint"


# ----------------------------------------------------------------------------
# Optional torch support — used only by the neural-controller round-trip tests.
# These tests skip cleanly when torch is not available.
# ----------------------------------------------------------------------------
try:
    import torch as _torch

    _HAS_TORCH = True

    class _TinyLSTMModule(_torch.nn.Module):
        """Minimal wrapper around a single ``torch.nn.LSTM`` for the LSTM test.

        ``ControllerNeuralLSTM`` requires ``network.lstm`` to be a
        ``torch.nn.LSTM`` with ``input_size=2``, ``batch_first=True``,
        ``num_layers >= 1``, non-bidirectional, and ``proj_size == 0``.
        This module satisfies those constraints with the smallest network
        that will load.
        """

        def __init__(self) -> None:
            super().__init__()
            self.lstm = _torch.nn.LSTM(input_size=2, hidden_size=4, num_layers=1, batch_first=True)
            self.head = _torch.nn.Linear(4, 1)

except ImportError:  # pragma: no cover - torch is an optional test dep
    _HAS_TORCH = False
    _torch = None  # type: ignore[assignment]
    _TinyLSTMModule = None  # type: ignore[assignment]


class TestUsdAuthoring(omni.kit.test.AsyncTestCase):
    """Tests for `add_actuator` and the companion config dataclasses."""

    async def setUp(self) -> None:
        """Prepare the USD authoring test stage."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
        usd_path = f"{get_assets_root_path()}/{_SIMPLE_ART_REL_PATH}"
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=_ART_ROOT)

    async def tearDown(self) -> None:
        """Clean up the USD authoring test stage."""
        super().tearDown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _stage(self) -> Usd.Stage:
        return stage_utils.get_current_stage(backend="usd")

    def _get_prim(self, actuator_name: str) -> Usd.Prim:
        return self._stage().GetPrimAtPath(f"{_ART_ROOT}/Actuators/{actuator_name}")

    # ------------------------------------------------------------------
    # Controllers — round-trip: author → discover correct type
    # ------------------------------------------------------------------

    async def test_pd_controller_authored_and_discovered(self) -> None:
        """Verify that a `PDControlConfig` yields a `ControllerPD` in `ArticulationActuators`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=200.0, kd=20.0),
        )

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertEqual(len(actuated.actuators), 1)
            self.assertIsInstance(actuated.actuators[0].controller, ControllerPD)
        finally:
            actuated.close()

    async def test_pid_controller_authored_and_discovered(self) -> None:
        """Verify that a `PIDControlConfig` yields a `ControllerPID` in `ArticulationActuators`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PIDControlConfig(kp=200.0, ki=5.0, kd=20.0),
        )

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertEqual(len(actuated.actuators), 1)
            self.assertIsInstance(actuated.actuators[0].controller, ControllerPID)
        finally:
            actuated.close()

    def _save_neural_checkpoint(self, tmp_dir: str, model_type: str) -> str:
        """Save a tiny torch checkpoint at ``tmp_dir`` with the given ``model_type``.

        Uses the dict-checkpoint format (``torch.save({"model": net, "metadata": {...}}, ...)``)
        which is the simplest path that ``newton.actuators.utils.load_checkpoint`` accepts.
        The networks are intentionally minimal — these tests verify the actuator can
        be *built* (i.e. parsed and constructed); they do not exercise ``compute()``.

        Args:
            tmp_dir: Directory where the checkpoint file is written.
            model_type: Neural controller type to encode in checkpoint metadata.

        Returns:
            Absolute path to the saved checkpoint file.
        """
        if model_type == "mlp":
            net = _torch.nn.Sequential(_torch.nn.Linear(2, 1))
            filename = "mlp.pt"
        elif model_type == "lstm":
            net = _TinyLSTMModule()
            filename = "lstm.pt"
        else:
            raise ValueError(f"unsupported model_type: {model_type!r}")
        model_path = os.path.join(tmp_dir, filename)
        _torch.save({"model": net, "metadata": {"model_type": model_type}}, model_path)
        return model_path

    async def test_neural_mlp_controller_authored_and_discovered(self) -> None:
        """Verify that a `NeuralControlConfig` pointing at an MLP checkpoint yields a `ControllerNeuralMLP`.

        Exercises authoring → discovery → controller-class resolution for the
        neural-MLP path.  The `model_type` key in the checkpoint metadata is
        what dispatches `NewtonNeuralControlAPI` to `ControllerNeuralMLP`.
        """
        if not _HAS_TORCH:
            self.skipTest("torch not installed")
        with tempfile.TemporaryDirectory() as tmp_dir:
            model_path = self._save_neural_checkpoint(tmp_dir, model_type="mlp")
            add_actuator(
                _ART_ROOT,
                target_names=_REVOLUTE_JOINT_NAME,
                name="act",
                controller=NeuralControlConfig(model_path=model_path),
            )

            actuated = ArticulationActuators(_ART_ROOT)
            try:
                self.assertEqual(len(actuated.actuators), 1)
                self.assertIsInstance(actuated.actuators[0].controller, ControllerNeuralMLP)
            finally:
                actuated.close()

    async def test_neural_lstm_controller_authored_and_discovered(self) -> None:
        """Verify that a `NeuralControlConfig` pointing at an LSTM checkpoint yields a `ControllerNeuralLSTM`.

        Same dispatch path as the MLP case but with ``model_type="lstm"`` in
        the metadata; the network has the structural constraints
        ``ControllerNeuralLSTM`` checks at construction (``network.lstm`` is a
        ``torch.nn.LSTM`` with ``input_size=2``, ``batch_first=True``, etc.).
        """
        if not _HAS_TORCH:
            self.skipTest("torch not installed")
        with tempfile.TemporaryDirectory() as tmp_dir:
            model_path = self._save_neural_checkpoint(tmp_dir, model_type="lstm")
            add_actuator(
                _ART_ROOT,
                target_names=_REVOLUTE_JOINT_NAME,
                name="act",
                controller=NeuralControlConfig(model_path=model_path),
            )

            actuated = ArticulationActuators(_ART_ROOT)
            try:
                self.assertEqual(len(actuated.actuators), 1)
                self.assertIsInstance(actuated.actuators[0].controller, ControllerNeuralLSTM)
            finally:
                actuated.close()

    # ------------------------------------------------------------------
    # Clamping — round-trip: author → discover correct type
    # ------------------------------------------------------------------

    async def test_max_effort_clamping_authored_and_discovered(self) -> None:
        """Verify that `MaxEffortClampingConfig` yields a `ClampingMaxEffort` in `ArticulationActuators`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            clamping=[MaxEffortClampingConfig(max_effort=50.0)],
        )

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertIsInstance(actuated.actuators[0].clamping[0], ClampingMaxEffort)
        finally:
            actuated.close()

    async def test_dc_motor_clamping_authored_and_discovered(self) -> None:
        """Verify that `DCMotorClampingConfig` yields a `ClampingDCMotor` in `ArticulationActuators`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            clamping=[DCMotorClampingConfig(saturation_effort=120.0, velocity_limit=10.0, max_motor_effort=100.0)],
        )

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertIsInstance(actuated.actuators[0].clamping[0], ClampingDCMotor)
        finally:
            actuated.close()

    async def test_position_based_clamping_authored_and_discovered(self) -> None:
        """Verify that `PositionBasedClampingConfig` yields a `ClampingPositionBased` in `ArticulationActuators`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            clamping=[
                PositionBasedClampingConfig(
                    lookup_positions=[0.0, 0.5, 1.0],
                    lookup_efforts=[50.0, 40.0, 20.0],
                )
            ],
        )

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertIsInstance(actuated.actuators[0].clamping[0], ClampingPositionBased)
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Delay — round-trip: author → discover correct type
    # ------------------------------------------------------------------

    async def test_delay_authored_and_discovered(self) -> None:
        """Verify that `DelayConfig` yields a `Delay` in `ArticulationActuators`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            delay=DelayConfig(delay_steps=3),
        )

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertIsInstance(actuated.actuators[0].delay, Delay)
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Authored USD attribute values
    # ------------------------------------------------------------------

    async def test_pd_values_authored_on_prim(self) -> None:
        """Verify that authored `kp`, `kd`, and `const_effort` match the config on the prim."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=300.0, kd=30.0, const_effort=5.0),
        )
        prim = self._get_prim("act")
        self.assertTrue(prim.IsValid())
        self.assertAlmostEqual(prim.GetAttribute("newton:kp").Get(), 300.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:kd").Get(), 30.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:constEffort").Get(), 5.0)

    async def test_pid_values_authored_on_prim(self) -> None:
        """Verify that authored PID gains, `integral_max`, and `const_effort` match the config on the prim."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PIDControlConfig(kp=300.0, ki=10.0, kd=30.0, integral_max=200.0, const_effort=2.0),
        )
        prim = self._get_prim("act")
        self.assertAlmostEqual(prim.GetAttribute("newton:kp").Get(), 300.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:ki").Get(), 10.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:kd").Get(), 30.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:integralMax").Get(), 200.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:constEffort").Get(), 2.0)

    async def test_pid_default_integral_max_authored_on_prim(self) -> None:
        """Verify that `PIDControlConfig` authors `inf` for `integral_max` when callers omit the field."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PIDControlConfig(kp=100.0, ki=5.0, kd=10.0),
        )
        prim = self._get_prim("act")
        self.assertTrue(math.isinf(prim.GetAttribute("newton:integralMax").Get()))

    async def test_max_effort_value_authored_on_prim(self) -> None:
        """Verify that `MaxEffortClampingConfig.max_effort` is authored on the prim."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            clamping=[MaxEffortClampingConfig(max_effort=75.0)],
        )
        prim = self._get_prim("act")
        self.assertAlmostEqual(prim.GetAttribute("newton:maxEffort").Get(), 75.0)

    async def test_dc_motor_values_authored_on_prim(self) -> None:
        """Verify that all `DCMotorClampingConfig` fields are authored on the prim."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            clamping=[DCMotorClampingConfig(saturation_effort=120.0, velocity_limit=8.0, max_motor_effort=90.0)],
        )
        prim = self._get_prim("act")
        self.assertAlmostEqual(prim.GetAttribute("newton:saturationEffort").Get(), 120.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:velocityLimit").Get(), 8.0)
        self.assertAlmostEqual(prim.GetAttribute("newton:maxMotorEffort").Get(), 90.0)

    async def test_position_based_values_authored_on_prim(self) -> None:
        """Verify that `PositionBasedClampingConfig` arrays are authored verbatim on the prim."""
        positions = [0.0, 0.5, 1.0]
        efforts = [50.0, 40.0, 20.0]
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            clamping=[PositionBasedClampingConfig(lookup_positions=positions, lookup_efforts=efforts)],
        )
        prim = self._get_prim("act")
        self.assertEqual(list(prim.GetAttribute("newton:lookupPositions").Get()), positions)
        self.assertEqual(list(prim.GetAttribute("newton:lookupEfforts").Get()), efforts)

    async def test_delay_value_authored_on_prim(self) -> None:
        """Verify that `DelayConfig.delay_steps` is authored on the prim."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            delay=DelayConfig(delay_steps=5),
        )
        prim = self._get_prim("act")
        self.assertEqual(prim.GetAttribute("newton:delaySteps").Get(), 5)

    # ------------------------------------------------------------------
    # Prim path and target relationship
    # ------------------------------------------------------------------

    async def test_prim_created_at_expected_path(self) -> None:
        """Verify that the `NewtonActuator` prim is created at `{art_root}/Actuators/{name}`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="my_actuator",
            controller=PDControlConfig(kp=100.0, kd=10.0),
        )
        prim = self._stage().GetPrimAtPath(f"{_ART_ROOT}/Actuators/my_actuator")
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "NewtonActuator")

    async def test_targets_relationship_authored(self) -> None:
        """Verify that the `newton:targets` relationship points to the resolved joint `Sdf.Path`."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
        )
        prim = self._get_prim("act")
        targets = prim.GetRelationship("newton:targets").GetTargets()
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0].name, _REVOLUTE_JOINT_NAME)

    async def test_multiple_targets_authored(self) -> None:
        """Verify that a list `target_names` authors all resolved paths as targets."""
        add_actuator(
            _ART_ROOT,
            target_names=[_REVOLUTE_JOINT_NAME, _PRISMATIC_JOINT_NAME],
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
        )
        prim = self._get_prim("act")
        targets = prim.GetRelationship("newton:targets").GetTargets()
        target_names = {t.name for t in targets}
        self.assertEqual(target_names, {_REVOLUTE_JOINT_NAME, _PRISMATIC_JOINT_NAME})

    # ------------------------------------------------------------------
    # Error handling
    # ------------------------------------------------------------------

    async def test_unknown_target_name_raises(self) -> None:
        """Verify that a target name not matching any joint raises `ValueError`."""
        with self.assertRaises(ValueError):
            add_actuator(
                _ART_ROOT,
                target_names="NonExistentJoint",
                name="act",
                controller=PDControlConfig(kp=100.0, kd=10.0),
            )

    async def test_overwrite_false_raises_on_existing_prim(self) -> None:
        """Verify that `add_actuator` raises `ValueError` when a prim already exists and overwriting is disabled."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
        )
        with self.assertRaises(ValueError):
            add_actuator(
                _ART_ROOT,
                target_names=_REVOLUTE_JOINT_NAME,
                name="act",
                controller=PDControlConfig(kp=200.0, kd=20.0),
            )

    async def test_overwrite_true_replaces_prim(self) -> None:
        """Verify that enabling overwrite replaces the existing prim with the new config."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
        )
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PIDControlConfig(kp=200.0, ki=5.0, kd=20.0),
            overwrite_existing=True,
        )
        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertIsInstance(
                actuated.actuators[0].controller,
                ControllerPID,
                "Overwritten prim must use the new controller type",
            )
        finally:
            actuated.close()

    async def test_position_based_empty_lookup_raises(self) -> None:
        """Verify that `PositionBasedClampingConfig` with empty `lookup_positions` raises `ValueError`."""
        with self.assertRaises(ValueError):
            add_actuator(
                _ART_ROOT,
                target_names=_REVOLUTE_JOINT_NAME,
                name="act",
                controller=PDControlConfig(kp=100.0, kd=10.0),
                clamping=[PositionBasedClampingConfig(lookup_positions=[], lookup_efforts=[])],
            )

    async def test_position_based_length_mismatch_raises(self) -> None:
        """Verify that `PositionBasedClampingConfig` with mismatched array lengths raises `ValueError`."""
        with self.assertRaises(ValueError):
            add_actuator(
                _ART_ROOT,
                target_names=_REVOLUTE_JOINT_NAME,
                name="act",
                controller=PDControlConfig(kp=100.0, kd=10.0),
                clamping=[
                    PositionBasedClampingConfig(
                        lookup_positions=[0.0, 0.5, 1.0],
                        lookup_efforts=[50.0, 20.0],  # length mismatch
                    )
                ],
            )

    async def test_multiple_clampings_authored_and_discovered(self) -> None:
        """Verify that several distinct clamping types on one actuator are all discovered."""
        add_actuator(
            _ART_ROOT,
            target_names=_REVOLUTE_JOINT_NAME,
            name="act",
            controller=PDControlConfig(kp=100.0, kd=10.0),
            clamping=[
                MaxEffortClampingConfig(max_effort=50.0),
                DCMotorClampingConfig(saturation_effort=120.0, velocity_limit=10.0, max_motor_effort=100.0),
            ],
        )

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            clamping_types = {type(c) for c in actuated.actuators[0].clamping}
            self.assertIn(ClampingMaxEffort, clamping_types)
            self.assertIn(ClampingDCMotor, clamping_types)
        finally:
            actuated.close()

    async def test_duplicate_clamping_type_raises(self) -> None:
        """Verify that passing the same clamping type twice raises `ValueError`."""
        with self.assertRaises(ValueError):
            add_actuator(
                _ART_ROOT,
                target_names=_REVOLUTE_JOINT_NAME,
                name="act",
                controller=PDControlConfig(kp=100.0, kd=10.0),
                clamping=[
                    MaxEffortClampingConfig(max_effort=50.0),
                    MaxEffortClampingConfig(max_effort=80.0),
                ],
            )
