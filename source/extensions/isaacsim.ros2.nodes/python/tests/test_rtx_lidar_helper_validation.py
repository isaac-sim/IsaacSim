# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validation tests for ROS2RtxLidarHelper prerequisites."""

import importlib.util
from pathlib import Path
from unittest.mock import MagicMock, patch

import omni.kit.test
from pxr import Usd

MODULE_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnROS2RtxLidarHelper.py"
SPEC = importlib.util.spec_from_file_location("_rtx_lidar_helper_validation", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
OgnROS2RtxLidarHelper = MODULE.OgnROS2RtxLidarHelper


class TestROS2RtxLidarHelperValidation(omni.kit.test.AsyncTestCase):
    """Verify missing stage/render products are rejected before camera lookup."""

    @staticmethod
    def _db() -> MagicMock:
        db = MagicMock()
        db.inputs.enabled = True
        db.inputs.renderProductPath = "/Render/MissingProduct"
        db.per_instance_state.initialized = False
        return db

    @patch.object(MODULE.ViewportManager, "get_camera")
    @patch.object(MODULE.omni.usd, "get_context")
    async def test_missing_stage_retries_before_camera_lookup(
        self, mock_get_context: MagicMock, mock_get_camera: MagicMock
    ) -> None:
        db = self._db()
        mock_get_context.return_value.get_stage.return_value = None

        self.assertFalse(OgnROS2RtxLidarHelper.compute(db))
        mock_get_camera.assert_not_called()

    @patch.object(MODULE.ViewportManager, "get_camera")
    @patch.object(MODULE.omni.usd, "get_context")
    async def test_missing_render_product_retries_before_camera_lookup(
        self, mock_get_context: MagicMock, mock_get_camera: MagicMock
    ) -> None:
        db = self._db()
        mock_get_context.return_value.get_stage.return_value = Usd.Stage.CreateInMemory()

        self.assertFalse(OgnROS2RtxLidarHelper.compute(db))
        mock_get_camera.assert_not_called()
