# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validation tests for ROS2CameraInfoHelper prerequisites."""

import importlib.util
from pathlib import Path
from unittest.mock import MagicMock, patch

import omni.kit.test
from pxr import Usd

MODULE_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnROS2CameraInfoHelper.py"
SPEC = importlib.util.spec_from_file_location("_camera_info_helper_validation", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
OgnROS2CameraInfoHelper = MODULE.OgnROS2CameraInfoHelper


class TestROS2CameraInfoHelperValidation(omni.kit.test.AsyncTestCase):
    """Verify missing stage/render products are handled as retryable prerequisites."""

    @staticmethod
    def _db() -> MagicMock:
        db = MagicMock()
        db.inputs.enabled = True
        db.inputs.renderProductPath = "/Render/Left"
        db.inputs.renderProductPathRight = ""
        db.per_instance_state.initialized = False
        return db

    @patch.object(MODULE.omni.usd, "get_context")
    async def test_missing_stage_returns_false(self, mock_get_context: MagicMock) -> None:
        db = self._db()
        mock_get_context.return_value.get_stage.return_value = None

        self.assertFalse(OgnROS2CameraInfoHelper.compute(db))

    @patch.object(MODULE, "read_camera_info")
    @patch.object(MODULE.carb.settings, "get_settings")
    @patch.object(MODULE.omni.usd, "get_context")
    async def test_missing_left_render_product_retries_before_camera_info(
        self, mock_get_context: MagicMock, mock_get_settings: MagicMock, mock_read_camera_info: MagicMock
    ) -> None:
        db = self._db()
        stage = Usd.Stage.CreateInMemory()
        mock_get_context.return_value.get_stage.return_value = stage
        mock_get_settings.return_value.get_as_bool.return_value = False

        self.assertFalse(OgnROS2CameraInfoHelper.compute(db))
        mock_read_camera_info.assert_not_called()

    @patch.object(MODULE, "read_camera_info")
    @patch.object(MODULE.carb.settings, "get_settings")
    @patch.object(MODULE.omni.usd, "get_context")
    async def test_missing_right_render_product_retries_before_camera_info(
        self, mock_get_context: MagicMock, mock_get_settings: MagicMock, mock_read_camera_info: MagicMock
    ) -> None:
        db = self._db()
        db.inputs.renderProductPathRight = "/Render/Right"
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Render/Left")
        mock_get_context.return_value.get_stage.return_value = stage
        mock_get_settings.return_value.get_as_bool.return_value = False

        self.assertFalse(OgnROS2CameraInfoHelper.compute(db))
        mock_read_camera_info.assert_not_called()
