# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for HolonomicRobotUsdSetup retargeting."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import omni.kit.test

MODULE_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnHolonomicRobotUsdSetup.py"
SPEC = importlib.util.spec_from_file_location("_holonomic_setup_retarget", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class _FakeRobotParams:
    def __init__(self, marker: float) -> None:
        self.wheel_radius = [marker]
        self.wheel_positions = [[marker, 0.0, 0.0]]
        self.wheel_orientations = [[1.0, 0.0, 0.0, 0.0]]
        self.mecanum_angles = [marker]
        self.wheel_dof_names = [f"wheel_{int(marker)}"]
        self.wheel_axis = [1.0, 0.0, 0.0]
        self.up_axis = [0.0, 0.0, 1.0]


class TestHolonomicSetupRetarget(omni.kit.test.AsyncTestCase):
    """Verify path changes rebuild cached holonomic robot parameters."""

    async def test_changed_prim_paths_reinitialize_robot_params(self) -> None:
        calls = []

        def fake_setup(*, robot_prim_path: str, com_prim_path: str):
            calls.append((robot_prim_path, com_prim_path))
            marker = 1.0 if robot_prim_path == "/RobotA" else 2.0
            return _FakeRobotParams(marker)

        state = MODULE.OgnHolonomicRobotUsdSetupInternalState()
        db = SimpleNamespace(
            per_instance_state=state,
            inputs=SimpleNamespace(
                usePath=True,
                robotPrimPath="/RobotA",
                comPrimPath="/RobotA/com",
                robotPrim=[],
                comPrim=[],
            ),
            outputs=SimpleNamespace(),
            log_error=lambda _message: None,
        )

        with patch.object(MODULE, "HolonomicRobotUsdSetup", side_effect=fake_setup):
            self.assertTrue(MODULE.OgnHolonomicRobotUsdSetup.compute(db))
            self.assertEqual(db.outputs.wheelRadius, [1.0])

            db.inputs.robotPrimPath = "/RobotB"
            db.inputs.comPrimPath = "/RobotB/com"
            self.assertTrue(MODULE.OgnHolonomicRobotUsdSetup.compute(db))

        self.assertEqual(calls, [("/RobotA", "/RobotA/com"), ("/RobotB", "/RobotB/com")])
        self.assertEqual(db.outputs.wheelRadius, [2.0])
        self.assertEqual(db.outputs.wheelDofNames, ["wheel_2"])
