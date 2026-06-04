# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utilities for testing Isaac Sim applications including image comparison, file validation, and UI testing."""

from __future__ import annotations

from typing import Any

import carb.settings

_COVERAGE_PATCH_APPLIED = False


def _is_pycoverage_enabled() -> bool:
    """Check if Python coverage is enabled in the test settings.

    Returns:
        True if coverage is enabled via the /exts/omni.kit.test/pyCoverageEnabled setting.
    """
    settings = carb.settings.get_settings()
    if settings is None:
        return False
    try:
        return bool(settings.get_as_bool("/exts/omni.kit.test/pyCoverageEnabled"))
    except Exception:
        return False


def _apply_numpy_copymode_coverage_patch() -> None:
    """Wrap ``np.array`` to translate ``_CopyMode`` enum values for ``copy``.

    When coverage is enabled, scipy's ``array_api_compat`` layer passes
    ``_CopyMode.IF_NEEDED`` to ``np.array(copy=...)``.  The bundled numpy
    does not handle the ``_CopyMode`` enum natively: it tries
    ``bool(copy)`` which either raises ``ValueError`` (for ``IF_NEEDED``)
    or maps to the wrong semantics (``False`` means *never* copy).

    This patch intercepts ``np.array`` calls and translates ``_CopyMode``
    values to the plain-Python equivalents numpy understands:

    * ``_CopyMode.ALWAYS``     → ``True``  (always copy)
    * ``_CopyMode.IF_NEEDED``  → ``None``  (copy only when necessary)
    * ``_CopyMode.NEVER``      → ``False`` (never copy, raise if impossible)
    """
    try:
        import numpy as np
        import numpy._globals as npg

        _CopyMode = getattr(npg, "_CopyMode", None)
        if _CopyMode is None:
            return

        if getattr(np, "_isaacsim_array_cov_patch_applied", False):
            return

        _COPYMODE_MAP = {
            _CopyMode.ALWAYS: True,
            _CopyMode.IF_NEEDED: None,
            _CopyMode.NEVER: False,
        }

        _original_array = np.array

        def _patched_array(*args: Any, **kwargs: Any) -> Any:
            if "copy" in kwargs and isinstance(kwargs["copy"], _CopyMode):
                kwargs["copy"] = _COPYMODE_MAP.get(kwargs["copy"], kwargs["copy"])
            return _original_array(*args, **kwargs)

        np.array = _patched_array
        np._isaacsim_array_cov_patch_applied = True
    except (ImportError, AttributeError):
        pass


def _apply_numpy_coverage_patch() -> None:
    """Apply patches to NumPy methods to handle coverage.py's _NoValueType sentinels.

    This patches NumPy's core methods (_amax, _amin, _sum, _prod) to properly handle
    coverage.py's sentinel values that can cause TypeError exceptions during array operations.
    """
    global _COVERAGE_PATCH_APPLIED
    if _COVERAGE_PATCH_APPLIED:
        return

    import numpy as np
    import numpy._core._methods as npm

    if getattr(npm, "_isaacsim_cov_patch_applied", False):
        _COVERAGE_PATCH_APPLIED = True
        return

    original_amax = npm._amax
    original_amin = npm._amin
    original_sum = npm._sum
    original_prod = npm._prod

    def _is_no_value_type(obj: Any) -> bool:
        """Check if an object is coverage.py's _NoValueType sentinel.

        Args:
            obj: The object to check.

        Returns:
            True if the object is a _NoValueType sentinel.
        """
        return hasattr(obj, "__class__") and obj.__class__.__name__ == "_NoValueType"

    def _coverage_amax(
        a: Any, axis: Any = None, out: Any = None, keepdims: Any = False, initial: Any = None, where: Any = True
    ) -> Any:
        """Handle coverage.py _NoValueType sentinels in max operations.

        Args:
            a: Input array.
            axis: Axis along which to operate.
            out: Output array.
            keepdims: Whether to keep dimensions.
            initial: Initial value.
            where: Condition for elements to include.

        Returns:
            Maximum value(s) of the array.
        """
        try:
            result = original_amax(a, axis, out, keepdims, initial, where)
            if _is_no_value_type(result):
                if axis is None:
                    return max(a.flat) if a.size > 0 else 0
                return np.array([max(row) for row in np.atleast_2d(a)])
            return result
        except TypeError as exc:
            if "_NoValueType" in str(exc):
                if axis is None:
                    return max(a.flat) if a.size > 0 else 0
                return np.array([max(row) for row in np.atleast_2d(a)])
            raise

    def _coverage_amin(
        a: Any, axis: Any = None, out: Any = None, keepdims: Any = False, initial: Any = None, where: Any = True
    ) -> Any:
        """Handle coverage.py _NoValueType sentinels in min operations.

        Args:
            a: Input array.
            axis: Axis along which to operate.
            out: Output array.
            keepdims: Whether to keep dimensions.
            initial: Initial value.
            where: Condition for elements to include.

        Returns:
            Minimum value(s) of the array.
        """
        try:
            result = original_amin(a, axis, out, keepdims, initial, where)
            if _is_no_value_type(result):
                if axis is None:
                    return min(a.flat) if a.size > 0 else 0
                return np.array([min(row) for row in np.atleast_2d(a)])
            return result
        except TypeError as exc:
            if "_NoValueType" in str(exc):
                if axis is None:
                    return min(a.flat) if a.size > 0 else 0
                return np.array([min(row) for row in np.atleast_2d(a)])
            raise

    def _coverage_sum(
        a: Any,
        axis: Any = None,
        dtype: Any = None,
        out: Any = None,
        keepdims: Any = False,
        initial: Any = 0,
        where: Any = True,
    ) -> Any:
        """Handle coverage.py _NoValueType sentinels in sum operations.

        Args:
            a: Input array.
            axis: Axis along which to operate.
            dtype: Data type of the output.
            out: Output array.
            keepdims: Whether to keep dimensions.
            initial: Initial value.
            where: Condition for elements to include.

        Returns:
            Sum of array elements.
        """
        try:
            result = original_sum(a, axis, dtype, out, keepdims, initial, where)
            if _is_no_value_type(result):
                if axis is None:
                    return sum(a.flat)
                return np.array([sum(row) for row in np.atleast_2d(a)])
            return result
        except TypeError as exc:
            if "_NoValueType" in str(exc):
                if axis is None:
                    return sum(a.flat)
                return np.array([sum(row) for row in np.atleast_2d(a)])
            raise

    def _coverage_prod(
        a: Any,
        axis: Any = None,
        dtype: Any = None,
        out: Any = None,
        keepdims: Any = False,
        initial: Any = 1,
        where: Any = True,
    ) -> Any:
        """Handle coverage.py _NoValueType sentinels in prod operations.

        Args:
            a: Input array.
            axis: Axis along which to operate.
            dtype: Data type of the output.
            out: Output array.
            keepdims: Whether to keep dimensions.
            initial: Initial value.
            where: Condition for elements to include.

        Returns:
            Product of array elements.
        """
        try:
            result = original_prod(a, axis, dtype, out, keepdims, initial, where)
            if _is_no_value_type(result):
                if axis is None:
                    result = 1
                    for val in a.flat:
                        result *= val
                    return result
                return np.array([np.prod(row) for row in np.atleast_2d(a)])
            return result
        except TypeError as exc:
            if "_NoValueType" in str(exc):
                if axis is None:
                    result = 1
                    for val in a.flat:
                        result *= val
                    return result
                return np.array([np.prod(row) for row in np.atleast_2d(a)])
            raise

    npm._amax = _coverage_amax
    npm._amin = _coverage_amin
    npm._sum = _coverage_sum
    npm._prod = _coverage_prod
    setattr(npm, "_isaacsim_cov_patch_applied", True)
    _COVERAGE_PATCH_APPLIED = True


def _apply_novaluetype_numeric_patch() -> None:
    """Give ``numpy._globals._NoValueType`` numeric dunder methods.

    When coverage.py is active its tracing can leak ``_NoValue`` sentinels into
    code paths that call ``int()``, ``float()``, or use the value as an index.
    Adding ``__int__``, ``__float__``, and ``__index__`` to the sentinel class lets
    those calls return 0 instead of raising ``TypeError``.
    """
    try:
        import numpy._globals as npg

        _NVT = getattr(npg, "_NoValueType", None)
        if _NVT is not None and not hasattr(_NVT, "__int__"):
            _NVT.__int__ = lambda self: 0
            _NVT.__float__ = lambda self: 0.0
            _NVT.__index__ = lambda self: 0
    except (ImportError, AttributeError):
        pass


if _is_pycoverage_enabled():
    _apply_numpy_copymode_coverage_patch()
    _apply_numpy_coverage_patch()
    _apply_novaluetype_numeric_patch()

from .button_utils import *
from .file_validation import *
from .image_capture import *
from .image_comparison import *
from .image_io import *
from .layout_utils import *
from .menu_ui_test import *
from .menu_utils import *
from .stage_utils import *
from .timed_async_test import *
from .usd_utils import *
from .viewport_utils import *

__all__ = [
    "get_widget_screen_center",
    "deferred_click",
    "deferred_click_widget",
    "discover_template_buttons",
    "validate_folder_contents",
    "get_folder_file_summary",
    "validate_file_list",
    "capture_annotator_data_async",
    "capture_rgb_data_async",
    "capture_depth_data_async",
    "capture_viewport_annotator_data_async",
    "capture_app_screenshot_async",
    "capture_viewport_screenshot_async",
    "capture_frame_sequence_async",
    "compute_difference_metrics",
    "print_difference_statistics",
    "compare_arrays_within_tolerances",
    "compare_images_within_tolerances",
    "compare_images_in_directories",
    "save_rgb_image",
    "save_depth_image",
    "save_annotator_data",
    "read_image_as_array",
    "close_windows",
    "ensure_dock_height",
    "ensure_dock_height_async",
    "ensure_window_visible",
    "ensure_window_visible_async",
    "reset_to_default_layout",
    "reset_to_default_layout_async",
    "MenuUITestCase",
    "find_widget_with_retry",
    "find_enabled_widget_with_retry",
    "wait_for_widget_enabled",
    "scroll_to_widget",
    "menu_click_with_retry",
    "list_menu_paths",
    "perform_widget_action",
    "get_all_menu_paths",
    "count_menu_items",
    "navigate_menu_visual",
    "poll_until",
    "poll_until_async",
    "wait_for_prim",
    "wait_for_prim_async",
    "wait_for_stage_prims",
    "wait_for_stage_prims_async",
    "project_world_to_screen",
    "TimedAsyncTestCase",
    "compare_usd_files",
    "check",
]
