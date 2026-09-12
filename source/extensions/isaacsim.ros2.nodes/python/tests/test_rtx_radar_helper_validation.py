# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for ROS2 RTX radar helper lifecycle validation."""

import importlib.util
from pathlib import Path
from unittest.mock import MagicMock, patch

import omni.kit.test

RADAR_HELPER_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnROS2RtxRadarHelper.py"
_spec = importlib.util.spec_from_file_location("_test_ros2_rtx_radar_helper", RADAR_HELPER_PATH)
_radar_helper = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_radar_helper)
OgnROS2RtxRadarHelper = _radar_helper.OgnROS2RtxRadarHelper


class TestROS2RtxRadarHelperValidation(omni.kit.test.AsyncTestCase):
    """Validate early lifecycle and render-product guards."""

    async def test_disabling_initialized_helper_resets_state(self) -> None:
        """Disabling an active helper must detach/reset its writer state."""
        db = MagicMock()
        db.inputs.enabled = False
        db.per_instance_state.initialized = True

        self.assertTrue(OgnROS2RtxRadarHelper.compute(db))

        db.per_instance_state.custom_reset.assert_called_once_with()

    async def test_missing_render_product_retries_before_camera_lookup(self) -> None:
        """An invalid USD prim should be rejected before resolving the radar camera."""
        db = MagicMock()
        db.inputs.enabled = True
        db.inputs.renderProductPath = "/Render/MissingProduct"
        db.per_instance_state.initialized = False

        invalid_prim = MagicMock()
        invalid_prim.IsValid.return_value = False

        with patch.object(_radar_helper.omni.usd, "get_context") as mock_get_context, patch.object(
            _radar_helper.ViewportManager, "get_camera"
        ) as mock_get_camera:
            mock_get_context.return_value.get_stage.return_value.GetPrimAtPath.return_value = invalid_prim

            self.assertFalse(OgnROS2RtxRadarHelper.compute(db))

            mock_get_camera.assert_not_called()
