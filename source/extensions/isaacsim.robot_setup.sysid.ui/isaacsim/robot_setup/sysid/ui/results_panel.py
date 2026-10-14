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

"""Results panel: live charts, parameter deltas, residual summary, validation metrics."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Any, Optional

import omni.ui as ui

from .live_charts import SysIdLiveChartsWidget
from .styles import (
    ACCENT_COLOR,
    CARD_BACKGROUND_COLOR,
    CARD_BORDER_COLOR,
    INFO_BACKGROUND_COLOR,
    PAGE_DETAIL_STYLE,
    PAGE_TITLE_STYLE,
    SPACE_MD,
    SPACE_SM,
    SPACE_XS,
    SUCCESS_BACKGROUND_COLOR,
    WARNING_BACKGROUND_COLOR,
)

# Body copy inside cards. Slightly brighter than the muted secondary grey so the
# card content reads as primary information while section chrome stays quiet.
_BODY_TEXT_STYLE = {"font_size": 12, "color": 0xFFC9C7C1}
_CARD_TITLE_STYLE = {"font_size": 13, "color": 0xFFE6E6E6}
_CARD_RADIUS = 8

# Verdict chip palette (background tint, foreground text), keyed by verdict cue.
_CHIP_WELL = (SUCCESS_BACKGROUND_COLOR, 0xFF8FCFA8)
_CHIP_WEAK = (0xFF2A3138, 0xFF85B7EB)
_CHIP_NOISE = (WARNING_BACKGROUND_COLOR, 0xFFEF9F27)
_CHIP_UNKNOWN = (INFO_BACKGROUND_COLOR, 0xFF9E9E9E)


def _verdict_chip_colors(verdict: str, at_noise_floor: bool) -> tuple[int, int]:
    """Map a confidence verdict to a (background, foreground) chip color pair.

    Args:
        verdict: Human-readable verdict text.
        at_noise_floor: Whether the parameter sits at the measurement-noise floor.

    Returns:
        Background and foreground ARGB colors for the verdict chip.
    """
    if at_noise_floor:
        return _CHIP_NOISE
    lowered = verdict.lower()
    if "well" in lowered:
        return _CHIP_WELL
    if "weak" in lowered:
        return _CHIP_WEAK
    return _CHIP_UNKNOWN


class ResultsPanel:
    """Build and manage the Results collapsible section.

    Args:
        on_export_run_report: Optional callback fired when the user exports a run report.
    """

    def __init__(
        self,
        on_export_run_report: Callable[[], None] | None = None,
    ) -> None:
        self._on_export_run_report = on_export_run_report

        self._live_charts = SysIdLiveChartsWidget()
        self._live_charts_frame: Optional[ui.Frame] = None
        self._empty_state_label: Optional[ui.Label] = None
        self._has_run_started = False
        self._container_frame = None
        self._open_callback: Callable[[], None] | None = None

        self._parameter_delta_label: Optional[ui.Label] = None
        self._confidence_rows: Optional[ui.VStack] = None
        self._residual_summary_label: Optional[ui.Label] = None
        self._validation_results_label: Optional[ui.Label] = None
        self._artifact_summary_label: Optional[ui.Label] = None
        self._export_run_report_btn: Optional[ui.Button] = None

        # Run-state chip in the header ("complete" once results are populated).
        self._state_chip_bg: Optional[ui.Rectangle] = None
        self._state_chip_label: Optional[ui.Label] = None

        # Header + card frames that hide together with the post-run content.
        self._post_run_decorations: list = []

        self.wrapped_ui_elements: list = []

    # ------------------------------------------------------------------ build

    @contextmanager
    def _card(self, title: str, *, background: int = CARD_BACKGROUND_COLOR) -> Iterator[None]:
        """Yield inside a padded, bordered card whose header carries an accent bar.

        The card frame is tracked in ``_post_run_decorations`` so it hides with the
        rest of the post-run content while a solve is running.

        Args:
            title: Section title shown next to the accent bar.
            background: Card fill color (override for semantic cards).
        """
        frame = ui.Frame()
        self._post_run_decorations.append(frame)
        with frame, ui.ZStack():
            ui.Rectangle(
                style={
                    "background_color": background,
                    "border_color": CARD_BORDER_COLOR,
                    "border_width": 1,
                    "border_radius": _CARD_RADIUS,
                }
            )
            with ui.VStack(spacing=SPACE_SM):
                ui.Spacer(height=SPACE_SM)
                with ui.HStack(spacing=0):
                    ui.Spacer(width=SPACE_MD)
                    with ui.VStack(spacing=SPACE_SM):
                        if title:
                            with ui.HStack(spacing=SPACE_SM, height=16):
                                ui.Rectangle(
                                    width=3,
                                    height=14,
                                    style={"background_color": ACCENT_COLOR, "border_radius": 2},
                                )
                                ui.Label(title, style=_CARD_TITLE_STYLE)
                        yield
                    ui.Spacer(width=SPACE_MD)
                ui.Spacer(height=SPACE_SM)

    def build(self) -> None:
        # A CollapsableFrame holds a single child, so everything must live under one
        # VStack. Without it only the last widget (the export button) would render.
        """Handle build."""
        self._post_run_decorations = []
        with ui.VStack(spacing=SPACE_MD):
            # Before the first in-app run, show a note instead of an empty chart:
            # headless-tool results are not imported into this page automatically.
            self._empty_state_label = ui.Label(
                "No in-app optimization has run in this session. Results from headless tools are written to "
                "their result JSON and artifact directory; they are not imported into this page automatically.",
                word_wrap=True,
                style=_BODY_TEXT_STYLE,
            )
            self._live_charts_frame = ui.Frame(visible=False)
            with self._live_charts_frame:
                self._live_charts.build()
            self.wrapped_ui_elements.extend(self._live_charts.wrapped_ui_elements)

            self._build_header()

            with self._card("Parameter deltas"):
                self._parameter_delta_label = ui.Label(
                    "Run optimization to compare initial and final values.",
                    word_wrap=True,
                    style=_BODY_TEXT_STYLE,
                )

            with self._card("Parameter confidence"):
                self._confidence_rows = ui.VStack(spacing=SPACE_XS)
                with self._confidence_rows:
                    ui.Label(
                        "Run optimization to see per-parameter sensitivity " "(value ± range where cost rises 10%).",
                        word_wrap=True,
                        style=_BODY_TEXT_STYLE,
                    )

            with self._card("Residual summary"):
                self._residual_summary_label = ui.Label(
                    "Active residual channels and convergence notes appear after a run.",
                    word_wrap=True,
                    style=_BODY_TEXT_STYLE,
                )

            with self._card("Validation", background=INFO_BACKGROUND_COLOR):
                self._validation_results_label = ui.Label(
                    "Untuned and tuned parameters replay on the same validation chunks.",
                    word_wrap=True,
                    style=_BODY_TEXT_STYLE,
                )

            with self._card("Exported artifacts"):
                self._artifact_summary_label = ui.Label(
                    "Export a run report to see its JSON and validation-artifact location.",
                    word_wrap=True,
                    style=_BODY_TEXT_STYLE,
                )

            self._build_export_row()

        # Before the first run, show only the empty-state note: hide the placeholder
        # cards, header, state chip, and export row until reset_for_run flips
        # _has_run_started. Without this they render alongside the empty-state note.
        self.set_controls_enabled(False)

    def _build_header(self) -> None:
        """Build the results title, one-line status, and run-state chip."""
        header = ui.Frame()
        self._post_run_decorations.append(header)
        with header, ui.HStack(spacing=SPACE_SM, height=40):
            with ui.VStack(spacing=2):
                ui.Label("Results", style=PAGE_TITLE_STYLE)
                ui.Label("Optimization output and validation.", style=PAGE_DETAIL_STYLE)
            ui.Spacer(width=ui.Fraction(1))
            with ui.VStack(width=96):
                ui.Spacer(height=SPACE_XS)
                with ui.ZStack(height=20):
                    self._state_chip_bg = ui.Rectangle(
                        style={"background_color": SUCCESS_BACKGROUND_COLOR, "border_radius": 10}
                    )
                    self._state_chip_label = ui.Label(
                        "complete",
                        alignment=ui.Alignment.CENTER,
                        style={"font_size": 11, "color": 0xFF8FCFA8},
                    )

    def _build_export_row(self) -> None:
        """Build the right-aligned, secondary-styled export button."""
        row = ui.Frame()
        self._post_run_decorations.append(row)
        with row, ui.HStack(height=30):
            ui.Spacer(width=ui.Fraction(1))
            self._export_run_report_btn = ui.Button(
                "Export run report",
                width=150,
                height=28,
                clicked_fn=self._on_export_clicked,
                tooltip="Write a JSON summary of the latest SysID run next to the telemetry source.",
                style={
                    "": {
                        "background_color": CARD_BACKGROUND_COLOR,
                        "border_color": 0xFF4A473F,
                        "border_width": 1,
                        "border_radius": 3,
                        "padding": 5,
                    },
                    ":hovered": {"background_color": 0xFF3A3833},
                    "Button.Label": {"color": 0xFFE6E6E6, "font_size": 13},
                },
            )

    def set_container_frame(self, frame: Any) -> None:
        """Set container frame.

        Args:
            frame: Frame that owns the panel.
        """
        self._container_frame = frame

    def set_open_callback(self, callback: Callable[[], None] | None) -> None:
        """Set the callback used to reveal the Results workflow page.

        Args:
            callback: Callback value.
        """
        self._open_callback = callback

    # ---------------------------------------------------------------- live chart proxies

    def reset_for_run(self, num_joints: int) -> None:
        """Handle reset for run.

        Args:
            num_joints: Num joints value.
        """
        self._open_results_frame()
        if self._empty_state_label is not None:
            self._empty_state_label.visible = False
        self._has_run_started = True
        if self._live_charts_frame is not None:
            self._live_charts_frame.visible = True
        self._live_charts.reset_for_run(num_joints)

    def set_activity(self, label: str, fraction: float) -> None:
        """Set activity.

        Args:
            label: Label value.
            fraction: Fraction value.
        """
        self._live_charts.set_activity(label, fraction)

    def add_cost_point(self, iteration: int, cost: float) -> None:
        """Handle add cost point.

        Args:
            iteration: Iteration value.
            cost: Cost value.
        """
        self._live_charts.add_cost_point(iteration, cost)

    def show_final_joint_errors(self, rollout: Any) -> None:
        """Handle show final joint errors.

        Args:
            rollout: Rollout data to display.
        """
        self._live_charts.show_final_joint_errors(rollout)

    def set_controls_enabled(self, enabled: bool) -> None:
        """Set controls enabled.

        Args:
            enabled: Whether the option is enabled.
        """
        self._live_charts.set_controls_enabled(enabled)
        post_run_visible = bool(enabled) and self._has_run_started
        for decoration in self._post_run_decorations:
            if decoration is not None:
                decoration.visible = post_run_visible
        if self._export_run_report_btn is not None:
            self._export_run_report_btn.enabled = post_run_visible

    def refresh_live_progress_layout(self) -> None:
        """Handle refresh live progress layout."""
        self._open_results_frame()
        self._live_charts.refresh_live_progress_layout()

    # ---------------------------------------------------------------- text labels

    def set_parameter_deltas(self, text: str) -> None:
        """Set parameter deltas.

        Args:
            text: Text to display.
        """
        if self._parameter_delta_label is not None:
            self._parameter_delta_label.text = text

    def set_parameter_confidence(self, payload: dict | None) -> None:
        """Render post-solve per-parameter sensitivity rows with verdict chips.

        Args:
            payload: Payload value.
        """
        rows = self._confidence_rows
        if rows is None:
            return
        rows.clear()
        entries = (payload or {}).get("entries") if payload else None
        if not entries:
            reason = (payload or {}).get("skipped_reason", "")
            text = (
                f"Confidence estimation skipped: {reason}"
                if reason
                else "Run optimization to see per-parameter sensitivity " "(value ± range where cost rises 10%)."
            )
            with rows:
                ui.Label(text, word_wrap=True, style=_BODY_TEXT_STYLE)
            return

        with rows:
            for entry in entries:
                rng = entry.get("ten_percent_range")
                rng_text = f" ±{rng:.4g}" if isinstance(rng, (int, float)) else ""
                verdict = str(entry.get("verdict", "unknown")).replace("_", " ")
                at_noise = bool(entry.get("at_noise_floor"))
                value = entry.get("value", float("nan"))
                background, foreground = _verdict_chip_colors(verdict, at_noise)
                chip_text = f"{verdict}; noise floor" if at_noise else verdict
                with ui.HStack(spacing=SPACE_SM, height=20):
                    ui.Label(
                        f"{entry.get('name', '?')}  {value:.5g}{rng_text}",
                        style=_BODY_TEXT_STYLE,
                    )
                    ui.Spacer(width=ui.Fraction(1))
                    with ui.ZStack(width=132):
                        ui.Rectangle(style={"background_color": background, "border_radius": 9})
                        ui.Label(
                            chip_text,
                            alignment=ui.Alignment.CENTER,
                            style={"font_size": 11, "color": foreground},
                        )

            footnotes = []
            noise_floor = (payload or {}).get("noise_floor") or {}
            if noise_floor.get("at_noise_floor"):
                footnotes.append(
                    "Solve cost is at the estimated measurement-noise floor — verdicts unreliable; "
                    "validate against ground truth or load-side torque."
                )
            source = (payload or {}).get("chunk_source", "")
            if source:
                footnotes.append(f"Evaluated on {source.replace('_', ' ')}.")
            for note in footnotes:
                ui.Label(note, word_wrap=True, style={"font_size": 11, "color": 0xFF8A8A8A})

    def set_residual_summary(self, text: str) -> None:
        """Set residual summary.

        Args:
            text: Text to display.
        """
        if self._residual_summary_label is not None:
            self._residual_summary_label.text = text

    def set_validation_results(self, text: str) -> None:
        """Set validation results.

        Args:
            text: Text to display.
        """
        if self._validation_results_label is not None:
            self._validation_results_label.text = text

    def set_artifact_summary(self, text: str) -> None:
        """Show where the latest in-app run report and artifacts were exported.

        Args:
            text: Text to display.
        """
        if self._artifact_summary_label is not None:
            self._artifact_summary_label.text = text

    def set_export_btn_enabled(self, enabled: bool) -> None:
        """Set export btn enabled.

        Args:
            enabled: Whether the option is enabled.
        """
        if self._export_run_report_btn is not None:
            self._export_run_report_btn.enabled = enabled

    # ---------------------------------------------------------------- cleanup

    def cleanup(self) -> None:
        """Release resources."""
        self._live_charts.cleanup()
        for element in self.wrapped_ui_elements:
            if hasattr(element, "cleanup"):
                element.cleanup()
        self.wrapped_ui_elements.clear()
        self._post_run_decorations = []
        self._parameter_delta_label = None
        self._confidence_rows = None
        self._residual_summary_label = None
        self._validation_results_label = None
        self._artifact_summary_label = None
        self._export_run_report_btn = None
        self._state_chip_bg = None
        self._state_chip_label = None
        self._live_charts_frame = None
        self._empty_state_label = None
        self._has_run_started = False
        self._container_frame = None
        self._open_callback = None

    # --------------------------------------------------------------- private

    def _open_results_frame(self) -> None:
        if self._open_callback is not None:
            self._open_callback()
            return
        if self._container_frame is None:
            return
        self._container_frame.collapsed = False

    def _on_export_clicked(self, *_args: Any) -> None:
        if self._on_export_run_report:
            self._on_export_run_report()
