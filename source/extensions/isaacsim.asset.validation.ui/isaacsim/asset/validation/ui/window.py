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

"""Dockable window for profile-driven SimReady asset validation."""

from __future__ import annotations

import asyncio
import webbrowser
from collections.abc import Callable
from pathlib import Path
from typing import Any

import carb.settings
import omni.kit.app
import omni.kit.clipboard
import omni.kit.window.popup_dialog
import omni.ui as ui
import omni.usd
from omni.asset_validator.core import FixStatus, Issue
from omni.kit.window.filepicker import FilePickerDialog
from pxr import Sdf, Usd

from .controller import RequirementResult, RequirementStatus, ValidationController
from .profile_resolver import ProfileResolutionError, ProfileResolver
from .report import build_report, location_text, write_report
from .result_delegate import ResultsTreeDelegate
from .result_model import FindingItem, RequirementItem, ResultsTreeModel

WINDOW_TITLE = "SimReady Asset Validation"
_DOCS_URL_SETTING = "/exts/isaacsim.asset.validation.ui/docsUrl"
_PROFILE_FILTER_SETTING = "/exts/isaacsim.asset.validation.ui/profileFilter"
_PROFILE_NAMES_SETTING = "/exts/isaacsim.asset.validation.ui/profileNames"
_DOCS_ROOT = "capabilities"


def _requirement_docs_url(path: str, base: str) -> str:
    """Build the published documentation URL for a requirement.

    Requirement metadata stores paths relative to a tier ``capabilities`` directory, while the
    published documentation site serves them under a ``capabilities`` root.

    Args:
        path: Requirement documentation path from the requirements registry.
        base: Documentation site base URL.

    Returns:
        Absolute documentation URL, or the base URL when no path is available.
    """
    base = base.rstrip("/")
    relative = path.strip("/")
    if not relative:
        return base
    if not relative.startswith(f"{_DOCS_ROOT}/"):
        relative = f"{_DOCS_ROOT}/{relative}"
    return f"{base}/{relative}"


def _profile_name_overrides(configured: Any) -> dict[str, str]:
    """Normalize the profileNames setting into a profile-id to label map.

    Args:
        configured: Raw ``profileNames`` setting value.

    Returns:
        Mapping of profile IDs to display-name overrides.
    """
    if not isinstance(configured, dict):
        return {}
    return {str(profile_id): str(label) for profile_id, label in configured.items() if str(profile_id) and str(label)}


def _pretty_profile_name(profile_id: str, overrides: dict[str, str] | None = None) -> str:
    """Convert a profile identifier into a readable selector label.

    Args:
        profile_id: Bundled SimReady profile identifier.
        overrides: Optional display-name overrides keyed by profile ID.

    Returns:
        Configured override when present, otherwise a generated pretty name.
    """
    if overrides and profile_id in overrides:
        return overrides[profile_id]
    parts = profile_id.split("-")
    if len(parts) > 1 and parts[-1] in {"Isaac", "Core"}:
        return f"{' '.join(parts[:-1])} ({parts[-1]})"
    return " ".join(parts)


def _filtered_profile_ids(available_ids: tuple[str, ...], configured: Any) -> tuple[str, ...]:
    """Exclude configured profile IDs from the available selector options.

    Args:
        available_ids: All discovered profile IDs.
        configured: Raw ``profileFilter`` setting listing IDs to exclude.

    Returns:
        Profile IDs remaining after exclusion.
    """
    if isinstance(configured, str):
        excluded = {configured}
    elif isinstance(configured, (list, tuple)):
        excluded = {str(profile_id) for profile_id in configured}
    else:
        excluded = set()
    return tuple(profile_id for profile_id in available_ids if profile_id not in excluded)


class ProgressModel(ui.AbstractValueModel):
    """Progress model that renders its value as a percentage."""

    def __init__(self) -> None:
        super().__init__()
        self._value = 0.0

    def set_value(self, value: float) -> None:
        """Set the progress fraction.

        Args:
            value: Progress between 0.0 and 1.0.
        """
        value = float(value)
        if value != self._value:
            self._value = value
            self._value_changed()

    def get_value_as_float(self) -> float:
        """Get the progress fraction.

        Returns:
            Progress between 0.0 and 1.0.
        """
        return self._value

    def get_value_as_string(self) -> str:
        """Get the progress value formatted as a percentage.

        Returns:
            Progress percentage text.
        """
        return f"{self._value * 100:.0f}%"


class SimReadyValidationWindow(ui.Window):
    """Window for selecting a SimReady profile and validating a USD asset."""

    def __init__(self) -> None:
        super().__init__(
            WINDOW_TITLE, width=860, height=720, visible=True, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._resolver = ProfileResolver()
        self._controller = ValidationController()
        self._dock_task: asyncio.Task | None = None
        self._unsubscribe: Callable[[], None] | None = None
        self._ui_ready = False
        self._file_picker: FilePickerDialog | None = None
        self._confirm_dialog: Any | None = None
        self._selected_result: RequirementResult | None = None
        self._selected_issue: Issue | None = None
        settings = carb.settings.get_settings()
        configured_profiles = settings.get(_PROFILE_FILTER_SETTING)
        name_overrides = _profile_name_overrides(settings.get(_PROFILE_NAMES_SETTING))
        self._profile_ids = _filtered_profile_ids(self._resolver.profile_ids, configured_profiles)
        self._profile_labels = tuple(
            _pretty_profile_name(profile_id, name_overrides) for profile_id in self._profile_ids
        )
        self._versions: tuple[str, ...] = ()
        self._results_model = ResultsTreeModel()
        self._results_delegate = ResultsTreeDelegate()
        self._progress_model = ProgressModel()
        self._status_model = ui.SimpleStringModel("Select a profile and asset.")
        self._source_model = ui.SimpleStringModel("")
        self._search_model = ui.SimpleStringModel("")
        self._summary_model = ui.SimpleStringModel("No results.")
        self._failures_model = ui.SimpleBoolModel(False)
        self._actionable_model = ui.SimpleBoolModel(False)
        self._optional_models: dict[str, ui.SimpleBoolModel] = {}
        self._optional_subscriptions: list[Any] = []
        self._label_subscriptions: list[Any] = []
        self._validate_button: ui.Button | None = None
        self._cancel_button: ui.Button | None = None
        self._export_button: ui.Button | None = None
        self._report_picker: FilePickerDialog | None = None
        self._build_ui()
        self._ui_ready = True
        self._unsubscribe = self._controller.subscribe(self._on_controller_changed)
        if self._profile_ids:
            self._select_profile(self._profile_ids[0])
        self._use_current_stage()
        self._dock_task = asyncio.ensure_future(self._dock_to_viewport())

    async def _dock_to_viewport(self) -> None:
        """Dock the window to the left of the viewport once the workspace is ready."""
        app = omni.kit.app.get_app()
        await app.next_update_async()
        viewport = ui.Workspace.get_window("Viewport")
        window = ui.Workspace.get_window(WINDOW_TITLE)
        if viewport is not None and window is not None:
            window.dock_in(viewport, ui.DockPosition.LEFT, 0.33)
        await app.next_update_async()

    def destroy(self) -> None:
        """Release controller, picker, dialogs, and UI resources."""
        self._ui_ready = False
        if self._dock_task is not None and not self._dock_task.done():
            self._dock_task.cancel()
        self._dock_task = None
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        self._controller.cleanup()
        self._close_file_picker()
        self._close_report_picker()
        if self._confirm_dialog is not None:
            self._confirm_dialog.hide()
            self._confirm_dialog = None
        self._optional_subscriptions.clear()
        self._label_subscriptions.clear()
        super().destroy()

    def _build_ui(self) -> None:
        with self.frame:
            with ui.VStack(spacing=8):
                self._build_source_panel()
                self._build_profile_panel()
                self._build_command_bar()
                self._build_summary_bar()
                with ui.ScrollingFrame(
                    height=ui.Fraction(1),
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                ):
                    self._results_tree = ui.TreeView(
                        self._results_model,
                        delegate=self._results_delegate,
                        root_visible=False,
                        header_visible=True,
                        columns_resizable=True,
                        column_widths=[ui.Fraction(0.5), ui.Pixel(180), ui.Fraction(0.5)],
                        identifier="simready_validation_results",
                    )
                    self._results_tree.set_selection_changed_fn(self._on_selection_changed)
                self._details_frame = ui.CollapsableFrame("Selected result", collapsed=False, height=0)
                self._details_frame.set_build_fn(self._build_details)
                ui.Spacer(height=8)

    def _build_source_panel(self) -> None:
        with ui.CollapsableFrame("Asset", collapsed=False, height=0):
            with ui.VStack(spacing=5):
                with ui.HStack(height=26, spacing=6):
                    ui.Button(
                        "Use Current Stage",
                        width=140,
                        clicked_fn=self._use_current_stage,
                        identifier="simready_use_stage",
                    )
                    ui.Button("Browse USD...", width=110, clicked_fn=self._open_file_picker)
                    ui.StringField(self._source_model, height=24, identifier="simready_asset_path")
                    ui.Button("Use Path", width=80, clicked_fn=self._use_path)

    def _build_profile_panel(self) -> None:
        with ui.CollapsableFrame("SimReady profile", collapsed=False, height=0):
            with ui.VStack(spacing=5):
                with ui.HStack(height=26, spacing=6):
                    ui.Label("Profile", width=60)
                    self._profile_combo = ui.ComboBox(
                        0, *self._profile_labels, height=24, identifier="simready_profile"
                    )
                    self._profile_combo.model.add_item_changed_fn(self._on_profile_changed)
                    ui.Label("Version", width=55)
                    self._version_frame = ui.Frame(width=150, height=24)
                    self._version_frame.set_build_fn(self._build_version_combo)
                self._optional_frame = ui.Frame(height=0)
                self._optional_frame.set_build_fn(self._build_optional_features)

    def _build_command_bar(self) -> None:
        with ui.HStack(height=30, spacing=6):
            self._validate_button = ui.Button(
                "Validate",
                width=100,
                clicked_fn=self._on_validate,
                identifier="simready_validate",
            )
            self._cancel_button = ui.Button(
                "Cancel",
                width=80,
                clicked_fn=self._controller.cancel,
                visible=False,
                identifier="simready_cancel",
            )
            ui.ProgressBar(self._progress_model, height=20)
            self._bind_label(self._status_model, width=ui.Fraction(1), word_wrap=True)
            self._export_button = ui.Button(
                "Export Report...",
                width=130,
                clicked_fn=self._open_report_picker,
                enabled=False,
                identifier="simready_export_report",
            )

    def _build_summary_bar(self) -> None:
        with ui.VStack(spacing=4, height=0):
            self._bind_label(self._summary_model, height=22)
            with ui.HStack(height=26, spacing=8):
                ui.Label("Filter", width=40)
                ui.StringField(self._search_model, height=22, identifier="simready_result_filter")
                self._search_model.add_value_changed_fn(lambda _: self._apply_filters())
                ui.CheckBox(self._failures_model, width=18)
                ui.Label("Failures only", width=90)
                ui.CheckBox(self._actionable_model, width=18)
                ui.Label("Actionable only", width=105)
                self._failures_model.add_value_changed_fn(lambda _: self._apply_filters())
                self._actionable_model.add_value_changed_fn(lambda _: self._apply_filters())

    def _bind_label(self, model: ui.SimpleStringModel, **kwargs: Any) -> ui.Label:
        label = ui.Label(model.as_string, **kwargs)

        def _on_changed(changed_model: ui.SimpleStringModel) -> None:
            label.text = changed_model.as_string

        self._label_subscriptions.append(model.subscribe_value_changed_fn(_on_changed))
        return label

    def _build_version_combo(self) -> None:
        selected = 0
        self._version_combo = ui.ComboBox(selected, *self._versions, height=24, identifier="simready_version")
        self._version_combo.model.add_item_changed_fn(self._on_version_changed)

    def _build_optional_features(self) -> None:
        self._optional_models.clear()
        self._optional_subscriptions.clear()
        profile = self._controller.profile
        if profile is None:
            return
        optional_features = [feature for feature in profile.features if feature.optional]
        with ui.VStack(spacing=3):
            ui.Label(
                f"{len(profile.features) - len(optional_features)} required feature(s), "
                f"{len(optional_features)} optional feature(s)",
                style={"color": 0xFFAAAAAA},
            )
            for feature in optional_features:
                with ui.HStack(height=22):
                    model = ui.SimpleBoolModel(False)
                    self._optional_models[feature.definition.feature_id] = model
                    ui.CheckBox(model, width=18)
                    ui.Label(
                        f"{feature.definition.display_name} ({len(feature.requirements)} implemented rules)",
                        tooltip=feature.definition.feature_id,
                    )
                    subscription = model.add_value_changed_fn(
                        lambda changed, feature_id=feature.definition.feature_id: self._controller.set_optional_feature_enabled(
                            feature_id, changed.as_bool
                        )
                    )
                    self._optional_subscriptions.append(subscription)

    def _build_details(self) -> None:
        with ui.VStack(spacing=6, height=0):
            result = self._selected_result
            if result is None:
                ui.Label("Select a requirement or finding.", style={"color": 0xFF999999})
                ui.Spacer(height=8)
                return
            with ui.ScrollingFrame(
                height=ui.Pixel(150),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            ):
                self._build_details_body(result)
            self._build_details_actions()
            ui.Spacer(height=8)

    def _build_details_body(self, result: RequirementResult) -> None:
        issue = self._selected_issue
        with ui.VStack(spacing=5, height=0):
            ui.Label(f"{result.code}: {result.name}", word_wrap=True, style={"font_size": 16})
            self._build_location_banner(issue)
            ui.Label(
                f"Status: {result.status.value} | Feature: {result.feature_name}"
                f"{' (optional)' if result.optional_feature else ''}",
                word_wrap=True,
                style={"color": 0xFFAAAAAA},
            )
            ui.Label(
                f"Category: {result.category or 'None'} | Validator: {result.rule_name or 'Not registered'} | "
                f"Findings: {len(result.issues)}",
                word_wrap=True,
                style={"color": 0xFFAAAAAA},
            )
            ui.Label(result.guidance or "No requirement guidance is available.", word_wrap=True)
            if issue is not None:
                ui.Separator()
                severity = issue.severity.name.title() if issue.severity is not None else "Unknown"
                ui.Label(f"{severity}: {issue.message}", word_wrap=True)
                if issue.suggestions:
                    ui.Label(f"Suggested action: {issue.suggestion.message}", word_wrap=True)

    def _build_location_banner(self, issue: Issue | None) -> None:
        location = location_text(issue.at) if issue is not None else ""
        fixable = self._can_apply_fix()
        with ui.HStack(height=24, spacing=8):
            ui.Label("Location", width=70, style={"color": 0xFFAAAAAA})
            ui.Label(
                location or "Whole asset (no specific prim reported)",
                word_wrap=True,
                tooltip=location,
                style={"font_size": 15, "color": 0xFF6FD3FF if location else 0xFFAAAAAA},
            )
            ui.Label(
                "Auto fix available" if fixable else "No auto fix",
                width=130,
                alignment=ui.Alignment.RIGHT_CENTER,
                style={"color": 0xFF70C070 if fixable else 0xFF909090},
            )

    def _build_details_actions(self) -> None:
        with ui.HStack(height=28, spacing=6):
            ui.Button("Open requirement docs", clicked_fn=self._open_requirement_docs)
            ui.Button("Copy details", clicked_fn=self._copy_details)
            ui.Button(
                "Select affected prim",
                clicked_fn=self._select_affected_prim,
                enabled=self._can_select_affected_prim(),
                identifier="simready_select_prim",
            )
            ui.Button(
                "Apply fix",
                clicked_fn=self._confirm_fix,
                enabled=self._can_apply_fix(),
                identifier="simready_apply_fix",
            )

    def _on_profile_changed(self, model: ui.AbstractItemModel, _: ui.AbstractItem) -> None:
        index = model.get_item_value_model().as_int
        if 0 <= index < len(self._profile_ids):
            self._select_profile(self._profile_ids[index])

    def _on_version_changed(self, model: ui.AbstractItemModel, _: ui.AbstractItem) -> None:
        index = model.get_item_value_model().as_int
        if self._controller.profile is None or not (0 <= index < len(self._versions)):
            return
        self._resolve_profile(self._controller.profile.definition.profile_id, self._versions[index])

    def _select_profile(self, profile_id: str) -> None:
        self._versions = self._resolver.versions(profile_id)
        self._version_frame.rebuild()
        if self._versions:
            self._resolve_profile(profile_id, self._versions[0])

    def _resolve_profile(self, profile_id: str, version: str) -> None:
        try:
            self._controller.set_profile(self._resolver.resolve(profile_id, version))
            self._optional_frame.rebuild()
        except ProfileResolutionError as exc:
            self._status_model.set_value(str(exc))

    def _use_current_stage(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._controller.set_asset(None)
            self._source_model.set_value("")
            self._status_model.set_value("No USD stage is open.")
            return
        self._controller.set_asset(stage)
        identifier = stage.GetRootLayer().identifier
        self._source_model.set_value(identifier)
        self._status_model.set_value("Using the current stage, including unsaved edits.")

    def _use_path(self) -> None:
        path = self._source_model.get_value_as_string().strip()
        self._controller.set_asset(path or None)
        self._status_model.set_value(f"Using asset path: {path}" if path else "Enter a USD asset path.")

    def _open_file_picker(self) -> None:
        self._close_file_picker()
        self._file_picker = FilePickerDialog(
            "Select USD Asset",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=self._on_file_selected,
            enable_file_bar=False,
            file_extension_options=[
                (".usd", "USD File"),
                (".usda", "USD Ascii File"),
                (".usdc", "USD Binary File"),
                (".usdz", "USD Package"),
            ],
        )
        self._file_picker.show()

    def _on_file_selected(self, file_name: str, dir_name: str) -> None:
        path = str(Path(dir_name).joinpath(file_name))
        self._source_model.set_value(path)
        self._controller.set_asset(path)
        self._status_model.set_value(f"Using asset path: {path}")
        self._close_file_picker()

    def _close_file_picker(self) -> None:
        if self._file_picker is None:
            return
        picker = self._file_picker
        self._file_picker = None
        picker.hide()

        async def destroy_picker() -> None:
            await omni.kit.app.get_app().next_update_async()
            picker.destroy()

        asyncio.ensure_future(destroy_picker())

    def _open_report_picker(self) -> None:
        if not self._controller.results:
            self._status_model.set_value("Run validation before exporting a report.")
            return
        self._close_report_picker()
        self._report_picker = FilePickerDialog(
            "Export Validation Report",
            allow_multi_selection=False,
            apply_button_label="Save",
            click_apply_handler=self._on_report_path_selected,
            current_filename=self._default_report_name(),
            file_extension_options=[(".json", "JSON Report")],
        )
        self._report_picker.show()

    def _on_report_path_selected(self, file_name: str, dir_name: str) -> None:
        self._close_report_picker()
        if not file_name:
            self._status_model.set_value("Enter a report file name.")
            return
        try:
            destination = write_report(Path(dir_name) / file_name, self._build_report())
        except OSError as exc:
            self._status_model.set_value(f"Report export failed: {exc}")
            return
        self._status_model.set_value(f"Report exported to {destination}")

    def _build_report(self) -> dict[str, Any]:
        asset = self._controller.asset
        asset_label = (
            asset.GetRootLayer().identifier
            if isinstance(asset, Usd.Stage)
            else str(asset or self._source_model.get_value_as_string())
        )
        return build_report(
            self._controller.results,
            self._controller.profile,
            asset_label,
            [feature.definition.feature_id for feature in self._controller.selected_features],
        )

    def _default_report_name(self) -> str:
        profile = self._controller.profile
        stem = profile.definition.profile_id if profile is not None else "simready"
        return f"{stem}_validation_report.json"

    def _close_report_picker(self) -> None:
        if self._report_picker is None:
            return
        picker = self._report_picker
        self._report_picker = None
        picker.hide()

        async def destroy_picker() -> None:
            await omni.kit.app.get_app().next_update_async()
            picker.destroy()

        asyncio.ensure_future(destroy_picker())

    def _on_validate(self) -> None:
        try:
            self._controller.start_validation()
        except RuntimeError as exc:
            self._status_model.set_value(str(exc))

    def _on_controller_changed(self) -> None:
        if not self._ui_ready:
            return
        self._progress_model.set_value(self._controller.progress)
        if self._validate_button is not None:
            self._validate_button.enabled = self._controller.can_validate
        if self._cancel_button is not None:
            self._cancel_button.visible = self._controller.running
        if self._export_button is not None:
            self._export_button.enabled = bool(self._controller.results)
        self._results_model.update(self._controller.results)
        self._expand_features()
        self._update_summary()
        if self._controller.error_message:
            self._status_model.set_value(self._controller.error_message)
        elif self._controller.running:
            self._status_model.set_value(f"Validating... {self._controller.progress:.0%}")
        elif self._controller.results:
            self._status_model.set_value("Validation complete.")

    def _update_summary(self) -> None:
        counts = dict.fromkeys(RequirementStatus, 0)
        for result in self._controller.results:
            counts[result.status] += 1
        self._summary_model.set_value(
            " | ".join(f"{status.value}: {counts[status]}" for status in RequirementStatus)
            if self._controller.results
            else "No results."
        )

    def _apply_filters(self) -> None:
        self._results_model.set_filters(
            self._search_model.get_value_as_string(),
            self._failures_model.as_bool,
            self._actionable_model.as_bool,
        )
        self._expand_features()

    def _expand_features(self) -> None:
        for feature_item in self._results_model.roots:
            self._results_tree.set_expanded(feature_item, True, False)

    def _on_selection_changed(self, items: list[Any]) -> None:
        self._selected_result = None
        self._selected_issue = None
        if items:
            item = items[0]
            if isinstance(item, RequirementItem):
                self._selected_result = item.result
                self._selected_issue = item.result.issues[0] if item.result.issues else None
            elif isinstance(item, FindingItem) and isinstance(item.parent, RequirementItem):
                self._selected_result = item.parent.result
                self._selected_issue = item.issue
        self._details_frame.rebuild()

    def _open_requirement_docs(self) -> None:
        if self._selected_result is None:
            return
        base = carb.settings.get_settings().get(_DOCS_URL_SETTING)
        webbrowser.open(_requirement_docs_url(self._selected_result.documentation_path, base))

    def _copy_details(self) -> None:
        if self._selected_result is None:
            return
        result = self._selected_result
        issue_text = "\n".join(
            f"- {issue.severity.name}: {issue.message} ({location_text(issue.at) or 'Asset'})"
            for issue in result.issues
        )
        text = (
            f"{result.code} {result.name}\n"
            f"Status: {result.status.value}\n"
            f"Rule: {result.category}/{result.rule_name}\n"
            f"Guidance: {result.guidance}\n{issue_text}"
        )
        omni.kit.clipboard.copy(text)

    def _affected_prim_path(self) -> Sdf.Path | None:
        issue = self._selected_issue
        if issue is None or not isinstance(self._controller.asset, Usd.Stage):
            return None
        at = issue.at
        candidates = at if isinstance(at, (list, tuple)) else [at]
        for candidate in candidates:
            path = getattr(candidate, "path", None)
            if isinstance(path, Sdf.Path):
                return path.GetPrimPath() if path.IsPropertyPath() else path
        return None

    def _can_select_affected_prim(self) -> bool:
        return self._affected_prim_path() is not None

    def _select_affected_prim(self) -> None:
        prim_path = self._affected_prim_path()
        if prim_path is None:
            return
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(prim_path)], True)

    def _can_apply_fix(self) -> bool:
        issue = self._selected_issue
        return (
            isinstance(self._controller.asset, Usd.Stage)
            and issue is not None
            and bool(issue.suggestions)
            and bool(issue.all_fix_sites)
        )

    def _confirm_fix(self) -> None:
        if not self._can_apply_fix() or self._selected_issue is None:
            return
        issue = self._selected_issue

        def apply(dialog: Any) -> None:
            dialog.hide()
            self._confirm_dialog = None
            asyncio.ensure_future(self._apply_fix(issue))

        def cancel(dialog: Any) -> None:
            dialog.hide()
            self._confirm_dialog = None

        self._confirm_dialog = omni.kit.window.popup_dialog.MessageDialog(
            title="Apply validator fix",
            message=(
                f"{issue.suggestion.message}\n\n"
                "The fix will modify the current stage and re-run validation. The stage will not be saved automatically."
            ),
            ok_label="Apply",
            cancel_label="Cancel",
            ok_handler=apply,
            cancel_handler=cancel,
        )
        self._confirm_dialog.show()

    async def _apply_fix(self, issue: Issue) -> None:
        try:
            result = await self._controller.apply_fix_async(issue)
            self._status_model.set_value(
                "Fix applied." if result.status is FixStatus.SUCCESS else f"Fix failed: {result.status.name}"
            )
        except RuntimeError as exc:
            self._status_model.set_value(str(exc))
