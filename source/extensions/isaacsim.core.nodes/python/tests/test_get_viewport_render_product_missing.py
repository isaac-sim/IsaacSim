# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for missing viewport lookup state."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import omni.kit.test


_MODULE_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnIsaacGetViewportRenderProduct.py"
_SPEC = importlib.util.spec_from_file_location("_ogn_get_viewport_render_product_test", _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(_MODULE)


class TestGetViewportRenderProductMissing(omni.kit.test.AsyncTestCase):
    """Verify a missing viewport follows the documented False-return path."""

    async def test_missing_viewport_does_not_raise_for_uninitialized_cache(self) -> None:
        state = _MODULE.OgnIsaacGetViewportRenderProductInternalState()
        db = SimpleNamespace(
            per_instance_state=state,
            inputs=SimpleNamespace(viewport="MissingViewport"),
            outputs=SimpleNamespace(),
        )

        with patch.object(_MODULE, "get_viewport_from_window_name", return_value=None), patch.object(
            _MODULE.carb, "log_warn"
        ) as log_warn:
            result = _MODULE.OgnIsaacGetViewportRenderProduct.compute(db)

        self.assertFalse(result)
        self.assertIsNone(state.viewport)
        self.assertFalse(state.initialized)
        log_warn.assert_called_once_with("viewport name MissingViewport not found")
