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

"""Exact-entry numeric fields for the System Identification workflow."""

from __future__ import annotations

import re
from collections.abc import Callable
from math import inf

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import UIWidgetWrapper
from isaacsim.gui.components.ui_utils import add_line_rect_flourish, format_tt
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH

_FORMAT_PRECISION_PATTERN = re.compile(r"%\.(\d+)[fFgGeE]")
_DEFAULT_PRECISION = 2


def _precision_from_format(format_spec: str) -> int:
    """Return the decimal count encoded in a printf-style float format.

    ``ui.FloatField`` accepts ``precision`` but silently discards ``format``, so
    a format string can only be honored by extracting its decimal count.

    Args:
        format_spec: Printf-style format such as ``"%.4f"``.

    Returns:
        Decimal count, or the default when the format encodes none.

    Example:

    .. code-block:: python

        >>> _precision_from_format("%.4f")
        4
    """
    match = _FORMAT_PRECISION_PATTERN.search(str(format_spec or ""))
    return int(match.group(1)) if match else _DEFAULT_PRECISION


class ExactFloatField(UIWidgetWrapper):
    """Labeled float input that does not change when the user drags it.

    Args:
        label: Short descriptive label.
        tooltip: Help shown over the label and input.
        default_value: Initial value.
        step: Retained for API compatibility with the standard field wrapper.
        format: Printf-style float format; only its decimal count is applied,
            because ``ui.FloatField`` exposes ``precision`` rather than ``format``.
        lower_limit: Optional inclusive lower bound.
        upper_limit: Optional inclusive upper bound.
        on_value_changed_fn: Callback invoked when the value changes.
        on_end_edit_fn: Callback invoked when editing finishes.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_setup.sysid.ui.numeric_inputs import ExactFloatField

        >>> field = ExactFloatField("Gain", default_value=1.5, lower_limit=0.0, upper_limit=10.0)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: float = 0.0,
        step: float = 0.01,
        format: str = "%.2f",
        lower_limit: float | None = None,
        upper_limit: float | None = None,
        on_value_changed_fn: Callable[[float], None] | None = None,
        on_end_edit_fn: Callable[[float], None] | None = None,
    ) -> None:
        self._lower_limit = float(lower_limit) if lower_limit is not None else None
        self._upper_limit = float(upper_limit) if upper_limit is not None else None
        self._on_value_changed_fn = on_value_changed_fn
        self._on_end_edit_fn = on_end_edit_fn
        self._suppress_callbacks = False
        # `ui.FloatField` has no drag gesture, so `step` is inert here. It
        # also has no `format` property, so the format string is reduced to
        # the decimal count that `precision` accepts.
        _ = step

        frame = ui.Frame()
        with frame:
            with ui.HStack():
                self._label = ui.Label(
                    label,
                    width=LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip=format_tt(tooltip),
                )
                self._float_field = ui.FloatField(
                    name="Field",
                    width=ui.Fraction(1),
                    height=LABEL_HEIGHT,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip=format_tt(tooltip),
                    precision=_precision_from_format(format),
                )
                add_line_rect_flourish(False)

        super().__init__(frame)
        self._float_field.model.set_min(self.get_lower_limit())
        self._float_field.model.set_max(self.get_upper_limit())
        self.set_value(default_value)
        # Subscribe to both events unconditionally so the callback setters can
        # change which one is delivered without rebuilding the subscription.
        self._float_field.model.add_value_changed_fn(self._on_value_changed)
        self._float_field.model.add_end_edit_fn(self._on_end_edit)

    @property
    def label(self) -> ui.Label:
        """Return the descriptive label.

        Example:

        .. code-block:: python

            >>> print(field.label.text)  # doctest: +NO_CHECK
            Gain
        """
        return self._label

    @property
    def float_field(self) -> ui.FloatField:
        """Return the underlying exact-entry widget.

        Example:

        .. code-block:: python

            >>> field.float_field.model.get_value_as_float()  # doctest: +NO_CHECK
            1.5
        """
        return self._float_field

    def get_value(self) -> float:
        """Return the current value.

        Returns:
            Current float value.

        Example:

        .. code-block:: python

            >>> field.get_value()  # doctest: +NO_CHECK
            1.5
        """
        return float(self._float_field.model.get_value_as_float())

    def set_value(self, value: float) -> None:
        """Set the current value, clipped to configured bounds.

        Args:
            value: Value to set.

        Example:

        .. code-block:: python

            >>> field.set_value(2.0)
            >>> field.get_value()  # doctest: +NO_CHECK
            2.0
        """
        self._write_model_value(self._clamp(float(value)))

    def get_lower_limit(self) -> float:
        """Return the configured lower bound.

        Returns:
            Configured lower bound, or negative infinity when unbounded.

        Example:

        .. code-block:: python

            >>> field.get_lower_limit()  # doctest: +NO_CHECK
            0.0
        """
        return -inf if self._lower_limit is None else self._lower_limit

    def get_upper_limit(self) -> float:
        """Return the configured upper bound.

        Returns:
            Configured upper bound, or positive infinity when unbounded.

        Example:

        .. code-block:: python

            >>> field.get_upper_limit()  # doctest: +NO_CHECK
            10.0
        """
        return inf if self._upper_limit is None else self._upper_limit

    def set_lower_limit(self, value: float) -> None:
        """Set the inclusive lower bound.

        Args:
            value: New lower bound.

        Example:

        .. code-block:: python

            >>> field.set_lower_limit(1.0)
            >>> field.get_lower_limit()  # doctest: +NO_CHECK
            1.0
        """
        self._lower_limit = float(value)
        self._float_field.model.set_min(self._lower_limit)
        self._reclamp_to_limits()

    def set_upper_limit(self, value: float) -> None:
        """Set the inclusive upper bound.

        Args:
            value: New upper bound.

        Example:

        .. code-block:: python

            >>> field.set_upper_limit(8.0)
            >>> field.get_upper_limit()  # doctest: +NO_CHECK
            8.0
        """
        self._upper_limit = float(value)
        self._float_field.model.set_max(self._upper_limit)
        self._reclamp_to_limits()

    def set_on_value_changed_fn(self, callback: Callable[[float], None] | None) -> None:
        """Replace the value-changed callback.

        Args:
            callback: Callback to store.

        Example:

        .. code-block:: python

            >>> field.set_on_value_changed_fn(print)
        """
        self._on_value_changed_fn = callback

    def set_on_end_edit_fn(self, callback: Callable[[float], None] | None) -> None:
        """Replace the end-edit callback.

        Args:
            callback: Callback to store.

        Example:

        .. code-block:: python

            >>> field.set_on_end_edit_fn(print)
        """
        self._on_end_edit_fn = callback

    def _clamp(self, value: float) -> float:
        return min(self.get_upper_limit(), max(self.get_lower_limit(), value))

    def _write_model_value(self, value: float) -> None:
        """Write the model without notifying the owner of a programmatic change."""
        previous = self._suppress_callbacks
        self._suppress_callbacks = True
        try:
            self._float_field.model.set_value(value)
        finally:
            self._suppress_callbacks = previous

    def _reclamp_to_limits(self) -> None:
        """Re-apply the bounds, notifying the owner if that moved the value."""
        current = self.get_value()
        clamped = self._clamp(current)
        if clamped == current:
            return
        self._write_model_value(clamped)
        self._dispatch(clamped, end_edit=self._on_end_edit_fn is not None)

    def _dispatch(self, value: float, *, end_edit: bool) -> None:
        if end_edit:
            callback = self._on_end_edit_fn
        elif self._on_end_edit_fn is not None:
            # An end-edit consumer opted out of per-keystroke notifications.
            callback = None
        else:
            callback = self._on_value_changed_fn
        if callback is not None:
            callback(value)

    def _on_value_changed(self, model: object) -> None:
        self._on_model_changed(model, end_edit=False)

    def _on_end_edit(self, model: object) -> None:
        self._on_model_changed(model, end_edit=True)

    def _on_model_changed(self, model: object, *, end_edit: bool) -> None:
        if self._suppress_callbacks:
            return
        raw = float(model.get_value_as_float())
        value = self._clamp(raw)
        if value != raw:
            self._write_model_value(value)
        self._dispatch(value, end_edit=end_edit)


class ExactIntField(UIWidgetWrapper):
    """Labeled integer input that does not change when the user drags it.

    Args:
        label: Short descriptive label.
        tooltip: Help shown over the label and input.
        default_value: Initial value.
        lower_limit: Optional inclusive lower bound.
        upper_limit: Optional inclusive upper bound.
        on_value_changed_fn: Callback invoked when the value changes.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_setup.sysid.ui.numeric_inputs import ExactIntField

        >>> field = ExactIntField("Iterations", default_value=20, lower_limit=1, upper_limit=100)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: int = 0,
        lower_limit: int | None = None,
        upper_limit: int | None = None,
        on_value_changed_fn: Callable[[int], None] | None = None,
    ) -> None:
        self._lower_limit = int(lower_limit) if lower_limit is not None else None
        self._upper_limit = int(upper_limit) if upper_limit is not None else None
        self._on_value_changed_fn = on_value_changed_fn
        self._suppress_callbacks = False

        frame = ui.Frame()
        with frame:
            with ui.HStack():
                self._label = ui.Label(
                    label,
                    width=LABEL_WIDTH,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip=format_tt(tooltip),
                )
                self._int_field = ui.IntField(
                    name="Field",
                    width=ui.Fraction(1),
                    height=LABEL_HEIGHT,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip=format_tt(tooltip),
                )
                add_line_rect_flourish(False)

        super().__init__(frame)
        self._int_field.model.set_min(self.get_lower_limit())
        self._int_field.model.set_max(self.get_upper_limit())
        self.set_value(default_value)
        self._int_field.model.add_value_changed_fn(self._on_model_changed)

    @property
    def label(self) -> ui.Label:
        """Return the descriptive label.

        Example:

        .. code-block:: python

            >>> print(field.label.text)  # doctest: +NO_CHECK
            Iterations
        """
        return self._label

    @property
    def int_field(self) -> ui.IntField:
        """Return the underlying exact-entry widget.

        Example:

        .. code-block:: python

            >>> field.int_field.model.get_value_as_int()  # doctest: +NO_CHECK
            20
        """
        return self._int_field

    def get_value(self) -> int:
        """Return the current value.

        Returns:
            Current integer value.

        Example:

        .. code-block:: python

            >>> field.get_value()  # doctest: +NO_CHECK
            20
        """
        return int(self._int_field.model.get_value_as_int())

    def set_value(self, value: int) -> None:
        """Set the current value, clipped to configured bounds.

        Args:
            value: Value to set.

        Example:

        .. code-block:: python

            >>> field.set_value(25)
            >>> field.get_value()  # doctest: +NO_CHECK
            25
        """
        self._write_model_value(self._clamp(int(value)))

    def get_lower_limit(self) -> int:
        """Return the configured lower bound.

        Returns:
            Configured lower bound, or the minimum signed 64-bit value when unbounded.

        Example:

        .. code-block:: python

            >>> field.get_lower_limit()  # doctest: +NO_CHECK
            1
        """
        return -(2**63) if self._lower_limit is None else self._lower_limit

    def get_upper_limit(self) -> int:
        """Return the configured upper bound.

        Returns:
            Configured upper bound, or the maximum signed 64-bit value when unbounded.

        Example:

        .. code-block:: python

            >>> field.get_upper_limit()  # doctest: +NO_CHECK
            100
        """
        return 2**63 - 1 if self._upper_limit is None else self._upper_limit

    def set_lower_limit(self, value: int) -> None:
        """Set the inclusive lower bound.

        Args:
            value: New lower bound.

        Example:

        .. code-block:: python

            >>> field.set_lower_limit(5)
            >>> field.get_lower_limit()  # doctest: +NO_CHECK
            5
        """
        self._lower_limit = int(value)
        self._int_field.model.set_min(self._lower_limit)
        self._reclamp_to_limits()

    def set_upper_limit(self, value: int) -> None:
        """Set the inclusive upper bound.

        Args:
            value: New upper bound.

        Example:

        .. code-block:: python

            >>> field.set_upper_limit(80)
            >>> field.get_upper_limit()  # doctest: +NO_CHECK
            80
        """
        self._upper_limit = int(value)
        self._int_field.model.set_max(self._upper_limit)
        self._reclamp_to_limits()

    def set_on_value_changed_fn(self, callback: Callable[[int], None] | None) -> None:
        """Replace the value-changed callback.

        Args:
            callback: Callback to store.

        Example:

        .. code-block:: python

            >>> field.set_on_value_changed_fn(print)
        """
        self._on_value_changed_fn = callback

    def _clamp(self, value: int) -> int:
        return min(self.get_upper_limit(), max(self.get_lower_limit(), value))

    def _write_model_value(self, value: int) -> None:
        """Write the model without notifying the owner of a programmatic change."""
        previous = self._suppress_callbacks
        self._suppress_callbacks = True
        try:
            self._int_field.model.set_value(value)
        finally:
            self._suppress_callbacks = previous

    def _reclamp_to_limits(self) -> None:
        """Re-apply the bounds, notifying the owner if that moved the value."""
        current = self.get_value()
        clamped = self._clamp(current)
        if clamped == current:
            return
        self._write_model_value(clamped)
        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(clamped)

    def _on_model_changed(self, model: object) -> None:
        if self._suppress_callbacks:
            return
        raw = int(model.get_value_as_int())
        value = self._clamp(raw)
        if value != raw:
            self._write_model_value(value)
        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(value)


__all__ = ["ExactFloatField", "ExactIntField"]
