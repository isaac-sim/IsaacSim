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

"""Regression tests for portable SysID run-spec serialization."""

from __future__ import annotations

import omni.kit.test
from isaacsim.robot_setup.sysid.run_spec import (
    NewtonSimulationRunSpec,
    OutputsRunSpec,
    ParameterRunSpec,
    ParametersRunSpec,
    SimulationRunSpec,
    SysIdRunSpec,
    TelemetryRunSpec,
    TimeWindowRunSpec,
)
from isaacsim.robot_setup.sysid.trajectory_segments import TelemetryChunkRunSpec


class RunSpecSerializationTests(omni.kit.test.AsyncTestCase):
    """Protect canonical run-spec serialization behavior."""

    async def test_canonical_payload_round_trips(self) -> None:
        """Preserve the canonical serialized payload across a decode/encode cycle."""
        spec = SysIdRunSpec(
            telemetry=TelemetryRunSpec(
                source_path="run.csv",
                time_window=TimeWindowRunSpec(start=1.0, end=2.0),
                chunks=[
                    TelemetryChunkRunSpec(
                        name="Train",
                        role="train",
                        excitation="chirp",
                        start=1.0,
                        end=2.0,
                        weight=1.5,
                    )
                ],
            ),
            simulation=SimulationRunSpec(
                robot_prim_path="/World/Robot",
                newton=NewtonSimulationRunSpec(feedforward="gravity"),
            ),
            parameters=ParametersRunSpec(
                selected=[
                    ParameterRunSpec(
                        param_type="joint_friction",
                        dof_index=0,
                        initial=1.0,
                        min=0.0,
                        max=5.0,
                    )
                ]
            ),
            outputs=OutputsRunSpec(
                validation_relative_change_threshold=0.05,
                validation_absolute_change_threshold=1.0e-6,
            ),
        )
        canonical_payload = spec.to_dict()

        restored = SysIdRunSpec.from_dict(canonical_payload)

        self.assertEqual(restored.to_dict(), canonical_payload)
        self.assertEqual(restored.outputs.validation_relative_change_threshold, 0.05)
        self.assertEqual(restored.outputs.validation_absolute_change_threshold, 1.0e-6)

    async def test_command_alignment_round_trips(self) -> None:
        """Preserve telemetry command alignment across serialization."""
        restored = SysIdRunSpec.from_dict({"telemetry": {"command_alignment_seconds": 0.125}})

        self.assertEqual(restored.telemetry.command_alignment_seconds, 0.125)
        self.assertEqual(restored.telemetry.resolved_command_alignment_seconds(), 0.125)

    async def test_rejects_non_mapping_nested_sections(self) -> None:
        """Reject malformed nested sections instead of replacing them with defaults."""
        cases = (
            ({"telemetry": []}, "telemetry"),
            ({"parameters": []}, "parameters"),
            ({"telemetry_quality": []}, "telemetry_quality"),
            ({"simulation": {"newton": []}}, "newton"),
        )
        for payload, expected in cases:
            with self.subTest(payload=payload):
                with self.assertRaisesRegex(ValueError, expected):
                    SysIdRunSpec.from_dict(payload)

    async def test_rejects_non_mapping_list_entries(self) -> None:
        """Reject malformed chunk and selected-parameter rows."""
        cases = (
            (
                {"telemetry": {"chunks": [{"start": 0.0, "end": 1.0}, "bad"]}},
                r"chunks\[1\]",
            ),
            (
                {"parameters": {"selected": [{"param_type": "joint_friction"}, 7]}},
                r"selected\[1\]",
            ),
            ({"parameters": {"selected": {}}}, "parameters.selected"),
        )
        for payload, expected in cases:
            with self.subTest(payload=payload):
                with self.assertRaisesRegex(ValueError, expected):
                    SysIdRunSpec.from_dict(payload)
