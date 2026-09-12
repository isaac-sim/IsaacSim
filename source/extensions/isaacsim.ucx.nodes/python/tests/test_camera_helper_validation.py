# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Retry-state tests for UCXCameraHelper."""

import importlib.util
from pathlib import Path
from unittest.mock import MagicMock, patch

import omni.kit.test
from pxr import Usd

MODULE_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnUCXCameraHelper.py"
SPEC = importlib.util.spec_from_file_location("_ucx_camera_helper_validation", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
OgnUCXCameraHelper = MODULE.OgnUCXCameraHelper


class TestUCXCameraHelperValidation(omni.kit.test.AsyncTestCase):
    """Verify transient setup failures leave the helper retryable."""

    @staticmethod
    def _db() -> MagicMock:
        db = MagicMock()
        db.per_instance_state.initialized = False
        db.inputs.renderProductPath = "/Render/Product"
        db.inputs.resetSimulationTimeOnStop = True
        db.inputs.frameSkipCount = 0
        db.inputs.useSystemTime = False
        db.inputs.port = 13337
        db.inputs.tag = 10
        db.inputs.sendCudaBuffer = True
        return db

    @patch.object(MODULE.omni.usd, "get_context")
    async def test_missing_stage_leaves_state_uninitialized(self, mock_get_context: MagicMock) -> None:
        db = self._db()
        mock_get_context.return_value.get_stage.return_value = None

        self.assertFalse(OgnUCXCameraHelper.compute(db))
        self.assertFalse(db.per_instance_state.initialized)

    @patch.object(MODULE.rep.writers, "get", side_effect=RuntimeError("temporary writer failure"))
    @patch.object(MODULE.omni.syntheticdata.SyntheticData, "convert_sensor_type_to_rendervar", return_value="LdrColor")
    @patch.object(MODULE.omni.usd, "get_context")
    async def test_writer_failure_leaves_state_uninitialized(
        self, mock_get_context: MagicMock, _mock_convert: MagicMock, _mock_writer_get: MagicMock
    ) -> None:
        db = self._db()
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Render/Product")
        mock_get_context.return_value.get_stage.return_value = stage

        self.assertFalse(OgnUCXCameraHelper.compute(db))
        self.assertFalse(db.per_instance_state.initialized)
