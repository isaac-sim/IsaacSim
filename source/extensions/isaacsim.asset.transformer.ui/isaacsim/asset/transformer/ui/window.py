# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Main window for the Asset Transformer extension UI."""

__all__ = ["AssetTransformerWindow"]

import asyncio
import json
import os
from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import Any

import carb
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile, RuleRegistry, RuleSpec
from omni.kit.menu.utils import MenuHelperWindow
from omni.kit.widget.filebrowser import FileBrowserItem
from omni.kit.window.filepicker import FilePickerDialog

from .action_list import ActionListFrame
from .action_models import (
    ActionItemValueModel,
    ActionListItem,
    ActionListModel,
    RuleActionItem,
)
from .constants import (
    ACTION_SET_INFO_TEXT,
    ADD_ICON_URL,
    AUTOLOAD_LABEL_TEXT,
    EXECUTE_ICON_URL,
    FOLDER_ICON_URL,
    HEADER_TEXT_ACTIONS,
    HEADER_TEXT_INPUT,
    HEADER_TEXT_REVIEW,
    HELP_ICON_URL,
    INDENT_SIZE,
    INFO_ICON_URL,
    INPUT_FILE_INFO_TEXT,
    MAX_RECENT_PRESETS,
    REMOVE_ICON_URL,
    REVIEW_INFO_TEXT,
    SAVE_ICON_URL,
    SETTING_RECENT_PRESETS,
    TRIANGLE_SIZE,
)
from .style import STYLE


class Sections(StrEnum):
    """Identifiers for the three collapsible sections of the window."""

    INPUT = "input"
    """Identifier for the input file selection section."""
    ACTIONS = "actions"
    """Identifier for the transformation actions configuration section."""
    REVIEW = "review"
    """Identifier for the review and execute section."""


@dataclass
class InputFileSelectionData:
    """Storage for widgets related to input file selection.

    Holds only the widgets that need to be read or written at runtime.

    Args:
        input_source_label: Label widget for the input source field.
        input_source_field: String field widget displaying the input source path.
        output_dir_field: String field widget for the output directory path.
        select_source_button: Button widget to open file picker for source selection.
        select_output_button: Button widget to open file picker for output directory selection.
        autoload_option: Checkbox widget to enable/disable auto-loading of transformed assets.
    """

    input_source_label: ui.Label
    input_source_field: ui.StringField
    output_dir_field: ui.StringField
    select_source_button: ui.Button
    select_output_button: ui.Button
    autoload_option: ui.CheckBox


def _filter_file_picker(ext: list[str], item: FileBrowserItem) -> bool:
    """Return True if *item* matches one of the given file extensions.

    Args:
        ext: Allowed extensions including the leading dot (e.g. ``[".usd"]``).
        item: File browser item to test.

    Returns:
        True if the item's extension is in *ext*.
    """
    if not item or item.is_folder:
        return False

    found_extension = os.path.splitext(item.path)[1]
    return True if found_extension in ext else False


class BoldCollapsableFrame(ui.CollapsableFrame):
    """Collapsible frame with a custom bold header and directional triangle.

    Args:
        show_help_button: Reserved for future use.
        **kwargs: Keyword arguments forwarded to ``ui.CollapsableFrame``.
    """

    # TODO: This class _may_ not be necessary with proper styling, look into this
    # Especially since I moved the docs button to a top-level floaring button...

    def __init__(self, show_help_button: bool = False, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.set_build_header_fn(self._build_header)
        self.explicit_hover = True

    def _build_header(self, is_collapsed: bool, title: str) -> None:
        """Build the custom header row with a directional triangle and label.

        Args:
            is_collapsed: Whether the frame is currently collapsed.
            title: Section identifier used to resolve the display text.
        """
        header_text: str
        match title:
            case Sections.INPUT:
                header_text = HEADER_TEXT_INPUT
            case Sections.ACTIONS:
                header_text = HEADER_TEXT_ACTIONS
            case Sections.REVIEW:
                header_text = HEADER_TEXT_REVIEW
            case _:
                carb.log_error("Unknown header text chosen")
                header_text = "[ERROR]"

        with ui.HStack():
            ui.Spacer(width=TRIANGLE_SIZE)
            with ui.VStack(name="center_content", width=TRIANGLE_SIZE):
                ui.Spacer()
                ui.Triangle(
                    height=TRIANGLE_SIZE,
                    width=TRIANGLE_SIZE,
                    alignment=(ui.Alignment.RIGHT_CENTER if is_collapsed else ui.Alignment.CENTER_BOTTOM),
                    style_type_name_override="CollapsableFrame.Triangle",
                )
                ui.Spacer()

            ui.Spacer(width=TRIANGLE_SIZE)
            ui.Label(header_text, style_type_name_override="CollapsableFrame.Header")


class AssetTransformerWindow(MenuHelperWindow):
    """Main window for the Asset Transformer tool.

    Provides UI sections for selecting input/output files, configuring
    transformation actions, and executing the transformation pipeline.

    Args:
        visible: Whether the window is visible on creation.
        **kwargs: Additional keyword arguments forwarded to ``MenuHelperWindow``.
    """

    WINDOW_TITLE: str = "Asset Transformer"
    """The title displayed in the window header."""

    # Default window dimensions
    DEFAULT_WIDTH = 600
    """The default pixel width of the window."""
    DEFAULT_HEIGHT = 710
    """The default pixel height of the window."""

    def __init__(self, visible: bool = True, **kwargs: Any) -> None:
        # Set default size if not provided via kwargs
        kwargs.setdefault("width", self.DEFAULT_WIDTH)
        kwargs.setdefault("height", self.DEFAULT_HEIGHT)
        super().__init__(self.WINDOW_TITLE, visible=visible, **kwargs)

        self.frame.set_build_fn(self._build_ui)

        self._file_data: InputFileSelectionData | None = None
        self._file_type_radio_collection: ui.RadioCollection | None = None
        self._file_picker: FilePickerDialog | None = None

        self._clear_all_button: ui.Button | None = None
        self._execute_button: ui.Button | None = None

        self._action_list_model = ActionListModel()
        self._profile = RuleProfile(profile_name="asset_transformer", version="1.0")
        self._rule_registry = RuleRegistry()

        # Profile-level editable models
        self._profile_name_model = ui.SimpleStringModel(self._profile.profile_name)
        self._profile_version_model = ui.SimpleStringModel(self._profile.version or "")
        self._profile_interface_model = ui.SimpleStringModel(self._profile.interface_asset_name or "")
        self._profile_base_name_model = ui.SimpleStringModel(self._profile.base_name or "")
        self._profile_flatten_model = ui.SimpleBoolModel(self._profile.flatten_source)

        self._profile_name_model.add_value_changed_fn(
            lambda m: setattr(self._profile, "profile_name", m.get_value_as_string())
        )
        self._profile_version_model.add_value_changed_fn(
            lambda m: setattr(self._profile, "version", m.get_value_as_string() or None)
        )
        self._profile_interface_model.add_value_changed_fn(
            lambda m: setattr(self._profile, "interface_asset_name", m.get_value_as_string() or None)
        )
        self._profile_base_name_model.add_value_changed_fn(
            lambda m: setattr(self._profile, "base_name", m.get_value_as_string() or None)
        )
        self._profile_flatten_model.add_value_changed_fn(
            lambda m: setattr(self._profile, "flatten_source", m.get_value_as_bool())
        )
        self._manager = AssetTransformerManager()

        # Update button states when action list changes
        self._action_list_model.add_item_changed_fn(lambda _, __: self._update_button_states())
        self._action_list_model.add_item_changed_fn(lambda _, __: self._sync_profile_rules())

        self._profile_name_label: ui.Label | None = None
        self._last_preset_dir: str | None = None
        self._confirmation_dialog: ui.Window | None = None

        # Subscribe to stage events to update stage field when stage changes
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub = self._usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="AssetTransformer stage event"
        )

        self._ensure_default_recent_presets()
        self._sync_profile_rules()

    def _ensure_default_recent_presets(self) -> None:
        """Seed the recent-presets persistent setting with the default preset if empty."""
        if self._get_recent_presets():
            return
        self._seed_default_preset()

    def _seed_default_preset(self) -> None:
        """Populate the recent presets list with the built-in default preset."""
        try:
            ext_manager = omni.kit.app.get_app().get_extension_manager()
            rules_path = ext_manager.get_extension_path_by_module("isaacsim.asset.transformer.rules")
            if rules_path:
                default_preset = os.path.join(rules_path, "data", "isaacsim_structure.json")
                if os.path.isfile(default_preset):
                    self._save_recent_presets([{"path": default_preset, "name": "Isaac Sim Structure"}])
                    carb.log_info(f"Seeded recent presets with default: {default_preset}")
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Could not seed default preset: {exc}")

    def _run_actions(self) -> None:
        """Execute the configured transformation actions on the input stage."""
        assert self._file_data is not None

        # Unconditional precheck: if "Active Stage" is selected and the stage
        # has unsaved state (new or dirty), ask the user how to proceed
        # before doing anything else. Yes saves and re-runs, No proceeds
        # with the on-disk version, Cancel aborts.
        if self._is_active_stage_unsaved():
            carb.log_warn("Asset Transformer: stage is unsaved, prompting Save before running")
            self._show_save_prompt(
                on_yes=lambda: self._save_active_stage_and_run(on_saved=self._run_actions_after_precheck),
                on_no=self._run_actions_after_precheck,
            )
            return

        self._run_actions_after_precheck()

    def _run_actions_after_precheck(self) -> None:
        """Run the transformer pipeline after the unsaved-stage precheck has resolved.

        Called from two paths: directly from :meth:`_run_actions` when the
        stage is already saved, and from the Save prompt callbacks (Yes after
        a successful save, No to use the on-disk version as-is).
        """
        assert self._file_data is not None

        input_stage_path, input_error = self._resolve_input_stage_path()
        if not input_stage_path:
            self._show_error_dialog("Cannot Run Asset Transformer", input_error or "Input stage is not available.")
            return

        profile = self._build_profile_from_actions()
        if not profile.rules:
            self._show_error_dialog(
                "No Actions Configured",
                "Add at least one action via the Actions section before clicking Execute Actions.",
            )
            return
        if not profile.output_package_root:
            self._show_error_dialog(
                "Output Directory Not Set",
                "Set an output directory in the Input section before clicking Execute Actions.",
            )
            return

        try:
            report = self._manager.run(
                input_stage_path,
                profile,
                package_root=profile.output_package_root,
            )
            carb.log_info(f"Execution completed: {len(report.results)} rules, output={report.package_root}")
            # Save the report in the package root folder as "transform_report.json"

            report_path = Path(report.package_root).joinpath("transform_report.json")
            try:
                with open(report_path, "w", encoding="utf-8") as f:
                    json.dump(report.to_dict() if hasattr(report, "to_dict") else report.__dict__, f, indent=2)
                carb.log_info(f"Report saved at {report_path}")
            except Exception as e:
                carb.log_warn(f"Could not save report at {report_path}: {e}")
        except Exception as exc:  # noqa: BLE001
            carb.log_error(f"Failed to execute actions: {exc}")
            self._show_error_dialog("Asset Transformer Failed", f"Failed to execute actions:\n{exc}")
            return

        # Surface per-rule failures from the execution report. Individual rule
        # failures (e.g. misconfigured rules raising ValueError) are captured
        # by the manager into RuleExecutionResult and do not propagate as
        # exceptions, so the only signal to the user is the report itself.
        failed = [r for r in report.results if not r.success]
        if failed:
            # Cap how many failures we display so a profile with many
            # misconfigured rules does not produce a modal that overflows the
            # screen; full details remain in the persisted report.
            max_lines = 10
            lines = [f"  - {r.rule.name}: {r.error or 'rule reported failure'}" for r in failed[:max_lines]]
            if len(failed) > max_lines:
                lines.append(f"  ... and {len(failed) - max_lines} more (see report for full details)")
            self._show_error_dialog(
                "Some Actions Failed",
                f"{len(failed)} of {len(report.results)} actions failed:\n" + "\n".join(lines),
            )

        if self._file_data.autoload_option.model.get_value_as_bool():
            output_path = report.output_stage_path
            if output_path and Path(output_path).exists():
                carb.log_info(f"Loading restructured file: {output_path}")
                omni.usd.get_context().open_stage(output_path)
            else:
                carb.log_warn(f"Output file does not exist, skipping autoload: {output_path}")

    def _show_help(self) -> None:
        """Open the Asset Transformer documentation in a web browser."""
        import webbrowser

        # TODO: Update to the correct URL for Asset Transformer documentation
        doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/index.html"
        try:
            webbrowser.open(doc_link, new=2)
        except Exception as e:
            carb.log_warn(f"Could not open browser with url: {doc_link}, {e}")

    def _build_ui(self) -> None:
        """Build the core UI, delegating to per-section helpers."""
        with self.frame:
            with ui.ScrollingFrame(
                style=STYLE,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            ):
                with ui.ZStack(height=0):
                    with ui.HStack():
                        ui.Spacer(width=INDENT_SIZE)
                        with ui.VStack(name="indent", spacing=4):
                            with BoldCollapsableFrame(title=Sections.INPUT, height=0):
                                with ui.HStack():
                                    ui.Spacer(width=4)
                                    self._build_io_panel()
                                    ui.Spacer(width=4)

                            with BoldCollapsableFrame(
                                title=Sections.ACTIONS,
                                height=0,
                            ):
                                with ui.HStack():
                                    ui.Spacer(width=4)
                                    self._build_actions_set_panel()
                                    ui.Spacer(width=4)
                            with BoldCollapsableFrame(
                                title=Sections.REVIEW,
                                height=0,
                            ):
                                with ui.HStack(height=50):
                                    ui.Spacer(width=4)
                                    self._build_review_and_execute_panel()
                                    ui.Spacer(width=4)
                    with ui.VStack(name="no_margin"):
                        ui.Spacer(height=2)
                        with ui.HStack(
                            height=0,
                            opaque_for_mouse_events=True,
                            name="no_margin",
                        ):
                            ui.Spacer()
                            with ui.HStack(content_clipping=True, width=0):
                                ui.Button(
                                    name="image_button",
                                    width=28,
                                    height=28,
                                    clicked_fn=self._show_help,
                                    image_url=HELP_ICON_URL,
                                    content_clipping=True,
                                )
                            ui.Spacer(width=10)
                        ui.Spacer()

        # Set initial button states after UI is built
        self._update_button_states()

    def _build_info_section(self, info_text: str) -> None:
        """Build the informational label row shown at the top of each section.

        Args:
            info_text: The help text to display beside the info icon.
        """
        with ui.HStack(height=0):
            ui.Image(INFO_ICON_URL, width=20, height=20)
            ui.Spacer(width=6)
            ui.Label(info_text, name="info", word_wrap=True, height=0)

    def _build_io_panel(self) -> None:
        """Build the input/output file selection panel."""
        self._file_type_radio_collection = ui.RadioCollection()

        # Actual layout
        with ui.VStack(name="indent", spacing=4):
            self._build_info_section(INPUT_FILE_INFO_TEXT)
            ui.Label("File", name="header")
            with ui.HStack():
                ui.RadioButton(
                    width=0,
                    image_width=20,
                    radio_collection=self._file_type_radio_collection,
                    text="Active Stage",
                )
                ui.Spacer()
                ui.RadioButton(
                    width=0,
                    image_width=20,
                    radio_collection=self._file_type_radio_collection,
                    text="Pick File",
                )
                ui.Spacer()
            with ui.HStack():
                input_source_label = ui.Label("Stage", width=ui.Percent(33))
                input_source_field = ui.StringField(read_only=True)
                select_source_button = ui.Button(
                    name="image_button",
                    image_url=FOLDER_ICON_URL,
                    image_width=20,
                    width=0,
                    clicked_fn=self._select_input_file,
                    tooltip="Select input file",
                )
            with ui.HStack():
                ui.Label("Output Directory", width=ui.Percent(33))
                output_dir_field = ui.StringField()
                select_output_button = ui.Button(
                    name="image_button",
                    image_url=FOLDER_ICON_URL,
                    image_width=20,
                    width=0,
                    clicked_fn=self._select_output_dir,
                    tooltip="Select output directory",
                )
            with ui.HStack():
                ui.Label(AUTOLOAD_LABEL_TEXT, width=ui.Percent(33))
                autoload_option = ui.CheckBox(align=ui.Alignment.LEFT_CENTER)
            ui.Spacer(height=8)

        # Cache widgets and data that might need to change
        self._file_data = InputFileSelectionData(
            input_source_label=input_source_label,
            input_source_field=input_source_field,
            output_dir_field=output_dir_field,
            select_source_button=select_source_button,
            select_output_button=select_output_button,
            autoload_option=autoload_option,
        )

        autoload_option.model.set_value(True)
        self._file_data.select_source_button.visible = False
        self._file_type_radio_collection.model.set_value(0)
        self._file_type_radio_collection.model.add_value_changed_fn(lambda model: self._update_file_input_type(model))
        # Initialize stage field with current stage info
        self._update_stage_field()

        # Update profile and buttons when output file changes
        def _on_output_dir_changed(_: ui.AbstractValueModel) -> None:
            self._update_profile_output_paths()
            self._update_button_states()

        output_dir_field.model.add_value_changed_fn(_on_output_dir_changed)

    def _on_stage_event(self, event: carb.events.IEvent) -> None:
        """Handle USD stage events to refresh the stage input field.

        Args:
            event: The stage event payload.
        """
        if event.type == int(omni.usd.StageEventType.OPENED) or event.type == int(omni.usd.StageEventType.CLOSED):
            self._update_stage_field()

    def _update_stage_field(self) -> None:
        """Update the stage input field with the current stage path.

        Shows the file path if the stage is saved, or indicator text if
        unsaved or no stage is open.
        """
        if self._file_data is None:
            return

        # Only update if "Organize File From Stage" is selected
        if self._file_type_radio_collection and self._file_type_radio_collection.model.get_value_as_int() != 0:
            return

        stage = self._usd_context.get_stage()
        if stage is None:
            self._file_data.input_source_field.model.set_value("<No stage open>")
            return

        root_layer = stage.GetRootLayer()
        identifier = root_layer.identifier

        if identifier.startswith("anon:"):
            # Unsaved stage - show indicator
            self._file_data.input_source_field.model.set_value("<Unsaved stage>")
        else:
            # Saved stage - show the file path
            real_path = root_layer.realPath
            if real_path:
                self._file_data.input_source_field.model.set_value(real_path)
            else:
                # Use identifier as fallback (might be a URL)
                self._file_data.input_source_field.model.set_value(identifier)

    def _resolve_input_stage_path(self) -> tuple[str | None, str | None]:
        """Resolve the input stage file path from either the open stage or the text field.

        Returns:
            A ``(path, error_message)`` tuple. On success, ``path`` is the
            resolved file path and ``error_message`` is ``None``. On failure,
            ``path`` is ``None`` and ``error_message`` is a user-facing string
            describing the specific failure so the caller can surface it in
            the UI in addition to writing it to the developer log.
        """
        assert self._file_data is not None

        use_stage = self._file_type_radio_collection and self._file_type_radio_collection.model.get_value_as_int() == 0
        if use_stage:
            stage = self._usd_context.get_stage()
            if stage is None:
                msg = "No stage is open."
                carb.log_error(msg)
                return None, msg
            root_layer = stage.GetRootLayer()
            if root_layer is None:
                msg = "Current stage has no root layer."
                carb.log_error(msg)
                return None, msg
            if root_layer.realPath:
                return root_layer.realPath, None
            identifier = root_layer.identifier
            if identifier and not identifier.startswith("anon:"):
                return identifier, None
            msg = (
                "The active stage must be saved before running the Asset Transformer.\n"
                "Save the stage via File > Save and try again."
            )
            carb.log_error("Stage must be saved to run the transformer")
            return None, msg

        input_path = self._file_data.input_source_field.model.get_value_as_string().strip()
        if not input_path:
            msg = "Input file is not set. Choose a USD file in the Input section."
            carb.log_error("Input file is not set")
            return None, msg
        if not Path(input_path).exists():
            msg = f"Input file does not exist:\n{input_path}"
            carb.log_error(f"Input file does not exist: {input_path}")
            return None, msg
        return input_path, None

    def _is_active_stage_unsaved(self) -> bool:
        """Return True when the active stage has any state the transformer cannot see on disk.

        Two cases count as "unsaved" for our purposes, because the
        transformer reads the stage from disk and so will silently ignore
        anything not yet written out:

        1. The stage has never been written to disk — ``is_new_stage()`` is
           True. Kit's :func:`omni.kit.window.file.save` will route this to
           Save As, opening the file-picker dialog.
        2. The stage was previously saved but has pending in-memory edits —
           ``omni.usd.get_dirty_layers(stage, recursive=True)`` returns a
           non-empty list. Kit's ``save()`` will save the dirty layers in
           place (no dialog) so the transformer sees the latest state.

        Only meaningful when "Active Stage" is selected; the "From File"
        mode operates on a user-chosen file regardless of the active stage's
        save state.

        Returns:
            True if the active stage must be saved before the transformer runs.
        """
        use_stage = self._file_type_radio_collection and self._file_type_radio_collection.model.get_value_as_int() == 0
        if not use_stage:
            return False
        ctx = self._usd_context
        if ctx.get_stage_state() != omni.usd.StageState.OPENED:
            return False
        if ctx.is_new_stage():
            return True
        stage = ctx.get_stage()
        if stage is None:
            return False
        dirty_layers = omni.usd.get_dirty_layers(stage, True)
        return bool(dirty_layers)

    def _save_active_stage_and_run(self, on_saved: Callable[[], None]) -> None:
        """Invoke Kit's standard save flow; on success invoke *on_saved*.

        Delegates entirely to :func:`omni.kit.window.file.save`. That helper
        already handles the "no path yet" case by routing to Save As (which
        opens the Save Stage file picker), as well as in-place save for stages
        that already have a backing file. We pass a completion callback that
        re-invokes the original Execute Actions click once the stage is on
        disk, so the user does not have to click Execute a second time.

        Args:
            on_saved: Callable invoked after the stage is successfully
                saved. Typically :meth:`_run_actions` so the original Execute
                Actions click resumes once the precondition is satisfied.
        """
        # Imported lazily so importing this module does not require
        # ``omni.kit.window.file`` to be loaded eagerly (it is a runtime
        # dependency declared in ``extension.toml``).
        import omni.kit.window.file

        def _on_save_done(success: bool, url: str) -> None:
            if not success:
                # User cancelled the file picker or save failed. Surface this
                # in the UI as well as the developer log so the user is not
                # left wondering why "Execute Actions" appeared to do nothing.
                carb.log_warn(f"Save cancelled or failed (url={url!r}); not re-running Asset Transformer")
                self._show_error_dialog(
                    "Save Cancelled",
                    "Save was cancelled or failed; the Asset Transformer was not run.\n"
                    "Click Execute Actions again once the stage has been saved.",
                )
                return
            carb.log_info(f"Stage saved to {url!r}; resuming Asset Transformer run")
            on_saved()

        omni.kit.window.file.save(_on_save_done)

    def _resolve_output_package_root(self) -> str | None:
        """Read the output directory from the UI field.

        Returns:
            The output directory path, or None if empty.
        """
        assert self._file_data is not None
        output_dir = self._file_data.output_dir_field.model.get_value_as_string().strip()
        if not output_dir:
            return None
        return output_dir

    def _resolve_base_output_path(self) -> str | None:
        """Resolve the base output path (delegates to the package root).

        Returns:
            The output package root, or None.
        """
        return self._resolve_output_package_root()

    def _update_profile_output_paths(self) -> None:
        """Sync the profile's output package root from the UI field."""
        self._profile.output_package_root = self._resolve_output_package_root()

    def _sync_profile_rules(self) -> None:
        """Rebuild the profile's rule list from the current action list model."""
        rules: list[RuleSpec] = []
        for item in self._action_list_model.get_item_children():
            assert isinstance(item, ActionListItem)
            action = item.action_model.get_action()
            if hasattr(action, "get_rule_spec"):
                rules.append(action.get_rule_spec())
        self._profile.rules = rules

    def _update_profile_name_label(self) -> None:
        """Sync the profile UI models from the current ``self._profile``.

        Note: the output directory field is intentionally *not* synced here.
        Presets do not persist ``output_package_root``, so overwriting the
        field would clear a user-supplied value every time a preset is loaded.
        """
        self._profile_name_model.set_value(self._profile.profile_name or "")
        self._profile_version_model.set_value(self._profile.version or "")
        self._profile_interface_model.set_value(self._profile.interface_asset_name or "")
        self._profile_base_name_model.set_value(self._profile.base_name or "")
        self._profile_flatten_model.set_value(self._profile.flatten_source)

    def _build_profile_from_actions(self) -> RuleProfile:
        """Build and return a fully populated ``RuleProfile`` from the current UI state.

        Returns:
            The profile with rules and output paths synced from the UI.
        """
        self._sync_profile_rules()
        self._update_profile_output_paths()
        return self._profile

    def _update_file_input_type(self, model: ui.AbstractValueModel) -> None:
        """Respond to a change in the input-type radio button.

        Args:
            model: The radio collection model whose value changed.
        """
        assert self._file_data is not None

        selection_idx = int(model.get_value_as_int())
        use_stage = selection_idx == 0

        if use_stage:
            self._file_data.input_source_label.text = "Stage"
            self._file_data.select_source_button.visible = False
            self._file_data.input_source_field.read_only = True
            self._update_stage_field()
        else:
            self._file_data.input_source_label.text = "Input File"
            self._file_data.select_source_button.visible = True
            self._file_data.input_source_field.read_only = False
            self._file_data.input_source_field.model.set_value("")

    def _close_file_picker(self) -> None:
        """Close the current file picker dialog.

        The dialog is hidden immediately so a replacement picker can be shown
        in the same frame.  Destruction is deferred to the next frame to avoid
        the ``Container::destroy during draw`` error.
        """
        if self._file_picker is None:
            return

        picker = self._file_picker
        self._file_picker = None
        picker.hide()

        async def _deferred_destroy() -> None:
            await omni.kit.app.get_app().next_update_async()
            picker.destroy()

        asyncio.ensure_future(_deferred_destroy())

    @staticmethod
    def _estimate_dialog_height(message: str, dialog_width: int = 450) -> int:
        """Estimate the dialog height needed to display *message* without clipping.

        Args:
            message: Body text (may contain newlines and long paths).
            dialog_width: Pixel width of the dialog.

        Returns:
            Pixel height that accommodates the wrapped text plus buttons.
        """
        usable_chars_per_line = max(1, (dialog_width - 40) // 8)
        wrapped_lines = 0
        for line in message.split("\n"):
            wrapped_lines += max(1, -(-len(line) // usable_chars_per_line))
        text_height = wrapped_lines * 20
        chrome = 16 + 8 + 30 + 8  # top spacer + gap + button row + bottom spacer
        return max(160, text_height + chrome + 30)

    def _dismiss_active_modal(self) -> None:
        """Properly close any modal currently occupying ``self._confirmation_dialog``.

        Hides the dialog immediately, clears the slot, and schedules
        :func:`ui.Window.destroy` for the next frame to avoid
        ``Container::destroy during draw`` errors. Any user-supplied
        ``on_*`` callbacks of the displaced modal are NOT invoked --
        replacement is treated as a silent cancel so the caller of the
        new modal can rely on its own callbacks firing exclusively.

        Existing modals previously leaked when a new one stomped the slot
        (the window object stayed alive because no ``_deferred_destroy``
        ever ran); this helper makes the displacement explicit and safe.
        """
        previous = self._confirmation_dialog
        if previous is None:
            return
        self._confirmation_dialog = None
        previous.visible = False
        carb.log_warn(
            f"AssetTransformerWindow modal slot displaced while {previous.title!r} was open; "
            "the previous dialog is being dismissed without invoking its callbacks."
        )

        async def _destroy_next_frame() -> None:
            await omni.kit.app.get_app().next_update_async()
            previous.destroy()

        asyncio.ensure_future(_destroy_next_frame())

    def _build_modal_window(self, title: str, message: str, width: int = 450) -> tuple[Any, Callable[[], None]]:
        """Create a modal window plus a deferred-destroy closure.

        Shared scaffolding for the three modal helpers. Returns the
        ``ui.Window`` (caller fills in the frame contents) and a
        ``_deferred_destroy`` callable to wire to the buttons.

        Args:
            title: Window title.
            message: Body text -- used only to size the window height.
            width: Window width in pixels.

        Returns:
            ``(dialog, deferred_destroy)``.
        """
        self._dismiss_active_modal()
        dialog = ui.Window(
            title,
            width=width,
            height=self._estimate_dialog_height(message, dialog_width=width),
            flags=(ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_MODAL | ui.WINDOW_FLAGS_NO_SAVED_SETTINGS),
        )
        self._confirmation_dialog = dialog

        def _deferred_destroy() -> None:
            if self._confirmation_dialog is dialog:
                self._confirmation_dialog = None
            dialog.visible = False

            async def _destroy_next_frame() -> None:
                await omni.kit.app.get_app().next_update_async()
                dialog.destroy()

            asyncio.ensure_future(_destroy_next_frame())

        return dialog, _deferred_destroy

    def _show_save_prompt(self, on_yes: Callable[[], None], on_no: Callable[[], None]) -> None:
        """Show a modal Yes / No / Cancel dialog asking whether to save before continuing.

        Mirrors the standard "Save before continuing?" pattern: ``Yes`` saves
        and continues, ``No`` continues without saving, ``Cancel`` aborts.
        Reuses the shared modal slot via :meth:`_build_modal_window`, which
        properly destroys (not just hides) any prior modal.

        Args:
            on_yes: Callback invoked when the user clicks Yes.
            on_no: Callback invoked when the user clicks No.
        """
        message = "Save before continuing?"
        dialog, _deferred_destroy = self._build_modal_window("Save Stage?", message, width=360)

        def _yes() -> None:
            _deferred_destroy()
            on_yes()

        def _no() -> None:
            _deferred_destroy()
            on_no()

        with dialog.frame:
            with ui.VStack(spacing=8):
                ui.Spacer(height=8)
                ui.Label(message, word_wrap=True, alignment=ui.Alignment.CENTER, height=0)
                ui.Spacer()
                with ui.HStack(height=30):
                    ui.Spacer()
                    ui.Button("Yes", width=ui.Pixel(80), clicked_fn=_yes)
                    ui.Spacer(width=8)
                    ui.Button("No", width=ui.Pixel(80), clicked_fn=_no)
                    ui.Spacer(width=8)
                    ui.Button("Cancel", width=ui.Pixel(80), clicked_fn=_deferred_destroy)
                    ui.Spacer()
                ui.Spacer(height=8)

        dialog.visible = True

    def _show_error_dialog(self, title: str, message: str) -> None:
        """Show a modal error dialog with a single OK button.

        Used to surface preflight failures (unsaved stage, missing output
        directory, etc.) and exception/result errors from
        :meth:`_run_actions` into the Asset Transformer window. Without this
        the only signal is a ``carb.log_error`` entry in the developer
        console, which is invisible to anyone watching only the tool window.

        Args:
            title: Window title (typically a short error category).
            message: Body text explaining the failure and corrective action.
        """
        dialog, _deferred_destroy = self._build_modal_window(title, message)

        with dialog.frame:
            with ui.VStack(spacing=8):
                ui.Spacer(height=8)
                ui.Label(message, word_wrap=True, alignment=ui.Alignment.CENTER, height=0)
                ui.Spacer()
                with ui.HStack(height=30):
                    ui.Spacer()
                    ui.Button("OK", width=ui.Pixel(100), clicked_fn=_deferred_destroy)
                    ui.Spacer()
                ui.Spacer(height=8)

        dialog.visible = True

    def _show_confirmation_dialog(
        self,
        title: str,
        message: str,
        on_confirm: Callable[[], None],
        confirm_label: str = "Confirm",
        cancel_label: str = "Cancel",
    ) -> None:
        """Show a modal confirmation dialog with two buttons.

        The window height is computed from the message content so that long
        file paths do not cause the buttons to be clipped.

        Args:
            title: Window title.
            message: Body text displayed to the user.
            on_confirm: Callable invoked when the user presses the confirm button.
            confirm_label: Text shown on the confirm button. Defaults to ``"Confirm"``.
            cancel_label: Text shown on the cancel button. Defaults to ``"Cancel"``.
        """
        dialog, _deferred_destroy = self._build_modal_window(title, message)

        def _confirm() -> None:
            _deferred_destroy()
            on_confirm()

        def _cancel() -> None:
            _deferred_destroy()

        with dialog.frame:
            with ui.VStack(spacing=8):
                ui.Spacer(height=8)
                ui.Label(message, word_wrap=True, alignment=ui.Alignment.CENTER, height=0)
                ui.Spacer()
                with ui.HStack(height=30):
                    ui.Spacer()
                    ui.Button(confirm_label, width=ui.Pixel(120), clicked_fn=_confirm)
                    ui.Spacer(width=8)
                    ui.Button(cancel_label, width=ui.Pixel(120), clicked_fn=_cancel)
                    ui.Spacer()
                ui.Spacer(height=8)

        dialog.visible = True

    def _show_picker_and_navigate(self, directory: str | None) -> None:
        """Show the current file picker then navigate to *directory*.

        The dialog is shown first so that its internal browser widget is
        initialised.  Navigation is deferred to the next application frame
        because the widget is not ready to accept ``navigate_to`` calls in
        the same frame as ``show()``.

        A trailing separator is appended to *directory* when absent so the
        file picker treats the last path component as a directory.

        Args:
            directory: Absolute path to navigate to.  When *None* or not a
                valid directory only ``show()`` is called.
        """
        if self._file_picker is None:
            return

        self._file_picker.show()

        if not directory or not os.path.isdir(directory):
            return

        if not directory.endswith(os.sep):
            directory += os.sep

        picker = self._file_picker

        async def _deferred_navigate() -> None:
            await omni.kit.app.get_app().next_update_async()
            if picker is not self._file_picker:
                return
            picker.set_current_directory(directory)
            picker.navigate_to(directory)
            picker.refresh_current_directory()

        asyncio.ensure_future(_deferred_navigate())

    def _resolve_input_dir(self) -> str | None:
        """Return the parent directory of the current input asset.

        When the radio is set to *Active Stage*, the directory is resolved
        directly from the open USD stage so that even if the text field has
        not been populated yet the correct folder is returned.  In *Pick
        File* mode the value of the input text field is used instead.

        Returns:
            The parent directory path, or None.
        """
        if self._file_data is None:
            return None

        use_stage = (
            self._file_type_radio_collection is not None
            and self._file_type_radio_collection.model.get_value_as_int() == 0
        )
        if use_stage:
            stage = self._usd_context.get_stage()
            if stage is not None:
                root_layer = stage.GetRootLayer()
                real_path = root_layer.realPath if root_layer else None
                if real_path:
                    parent = str(Path(real_path).parent)
                    if os.path.isdir(parent):
                        return parent
            return None

        current = self._file_data.input_source_field.model.get_value_as_string().strip()
        if not current or current.startswith("<"):
            return None
        parent = str(Path(current).parent)
        return parent if os.path.isdir(parent) else None

    def _on_input_file_selected(self, file_name: str, dir_name: str) -> None:
        """Handle input file selection from the file picker.

        Args:
            file_name: Selected file name.
            dir_name: Directory containing the selected file.
        """
        assert self._file_data is not None
        self._file_data.input_source_field.model.set_value(str(Path(dir_name).joinpath(file_name)))
        self._close_file_picker()

    def _on_output_dir_selected(self, file_name: str, dir_name: str) -> None:
        """Handle output directory selection from the file picker.

        When the user selects a folder in the browser cards the
        ``file_name`` argument contains that folder name.  If the combined
        ``dir_name / file_name`` resolves to a directory it is used as the
        output path; otherwise ``dir_name`` alone is used.

        If the resolved directory is non-empty a confirmation dialog is
        shown before applying the selection.

        Args:
            file_name: Name entered / selected in the file bar (may be a
                subfolder name).
            dir_name: The directory the picker is currently navigated to.
        """
        assert self._file_data is not None
        self._close_file_picker()

        # Prefer the combined path when the user selected a subfolder
        selected_dir = dir_name
        if file_name:
            candidate = str(Path(dir_name).joinpath(file_name))
            if os.path.isdir(candidate):
                selected_dir = candidate

        def _apply() -> None:
            assert self._file_data is not None
            self._file_data.output_dir_field.model.set_value(selected_dir)

        if os.path.isdir(selected_dir) and os.listdir(selected_dir):
            self._show_confirmation_dialog(
                "Directory Not Empty",
                f"The selected directory is not empty:\n{selected_dir}\n\nDo you want to use it anyway?",
                _apply,
            )
        else:
            _apply()

    def _on_preset_load_selected(self, file_name: str, dir_name: str) -> None:
        """Handle preset file selection for loading.

        Args:
            file_name: Selected preset file name.
            dir_name: Directory containing the preset file.
        """
        self._close_file_picker()
        filepath = str(Path(dir_name).joinpath(file_name))
        self._load_preset_from_path(filepath)

    def _load_preset_from_path(self, filepath: str) -> None:
        """Load a preset profile from a JSON file path.

        Args:
            filepath: Absolute path to the JSON preset file.
        """
        carb.log_info(f"Loading preset from {filepath}")

        try:
            with open(filepath, encoding="utf-8") as f:
                preset_data = json.load(f)

            profile = RuleProfile.from_dict(preset_data)
            loaded_rules = list(profile.rules)
            self._profile = profile

            self._action_list_model.clear_all()
            for rule_spec in loaded_rules:
                action = RuleActionItem(rule_spec, registry=self._rule_registry)
                self._action_list_model.append_child_item(None, ActionItemValueModel(action))

            self._add_recent_preset(filepath, profile.profile_name)
            self._last_preset_dir = str(Path(filepath).parent)
            self._update_profile_name_label()
            carb.log_info(
                f"Loaded {len(profile.rules)} rules from preset"
                f" (output_package_root={profile.output_package_root!r})"
            )

        except FileNotFoundError:
            carb.log_error(f"Preset file not found: {filepath}")
            self._remove_recent_preset(filepath)
        except json.JSONDecodeError as e:
            carb.log_error(f"Failed to parse preset JSON: {e}")
        except ValueError as e:
            carb.log_error(f"Invalid preset profile: {e}")
        except Exception as e:
            carb.log_error(f"Failed to load preset: {e}")

    @staticmethod
    def _get_recent_presets() -> list[dict[str, str]]:
        """Read recent presets from persistent settings.

        Returns:
            List of dicts with ``"path"`` and ``"name"`` keys.
        """
        settings = carb.settings.get_settings()
        raw = settings.get(SETTING_RECENT_PRESETS)
        if not raw or not isinstance(raw, str):
            return []
        try:
            entries = json.loads(raw)
            if isinstance(entries, list):
                return [e for e in entries if isinstance(e, dict) and "path" in e and "name" in e]
        except json.JSONDecodeError:
            pass
        return []

    @staticmethod
    def _save_recent_presets(entries: list[dict[str, str]]) -> None:
        """Write recent presets to persistent settings.

        Args:
            entries: List of dicts with ``"path"`` and ``"name"`` keys.
        """
        settings = carb.settings.get_settings()
        settings.set(SETTING_RECENT_PRESETS, json.dumps(entries[:MAX_RECENT_PRESETS]))

    def _add_recent_preset(self, filepath: str, profile_name: str) -> None:
        """Add a preset to the recent list, moving it to the front if already present.

        Args:
            filepath: Absolute path to the preset file.
            profile_name: Display name for the preset.
        """
        entries = self._get_recent_presets()
        # Remove existing entry for this path
        entries = [e for e in entries if e["path"] != filepath]
        # Insert at front
        entries.insert(0, {"path": filepath, "name": profile_name})
        # Trim to max
        self._save_recent_presets(entries[:MAX_RECENT_PRESETS])

    def _remove_recent_preset(self, filepath: str) -> None:
        """Remove a preset from the recent list and re-seed defaults if the list becomes empty.

        Args:
            filepath: Absolute path of the preset to remove.
        """
        entries = self._get_recent_presets()
        entries = [e for e in entries if e["path"] != filepath]
        self._save_recent_presets(entries)
        if not entries:
            self._seed_default_preset()

    def _on_preset_save_selected(self, file_name: str, dir_name: str) -> None:
        """Handle preset file selection for saving.

        If the target file already exists and is non-empty a confirmation
        dialog is shown before overwriting.

        Args:
            file_name: File name chosen in the picker.
            dir_name: Directory chosen in the picker.
        """
        self._close_file_picker()

        filepath = Path(dir_name).joinpath(file_name)

        # Ensure .json extension
        if filepath.suffix.lower() != ".json":
            filepath = filepath.with_suffix(".json")

        if filepath.is_file() and filepath.stat().st_size > 0:
            self._show_confirmation_dialog(
                "File Already Exists",
                f"The file already exists:\n{filepath}\n\nDo you want to overwrite it?",
                lambda: self._do_save_preset(filepath),
            )
        else:
            self._do_save_preset(filepath)

    def _do_save_preset(self, filepath: Path) -> None:
        """Write the current profile to *filepath* as a JSON preset.

        Args:
            filepath: Destination path (must end in ``.json``).
        """
        carb.log_info(f"Saving preset to {filepath}")

        try:
            profile = self._build_profile_from_actions()
            preset_data = profile.to_dict()
            # Never persist output_package_root -- it is a runtime-only value
            # chosen by the user at execution time, not part of the preset.
            preset_data["output_package_root"] = None

            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(preset_data, f, indent=2, ensure_ascii=True)

            self._last_preset_dir = str(filepath.parent)
            self._add_recent_preset(str(filepath), profile.profile_name)
            self._update_profile_name_label()
            carb.log_info(f"Saved {len(profile.rules)} rules to preset")

        except PermissionError:
            carb.log_error(f"Permission denied writing to: {filepath}")
        except Exception as e:
            carb.log_error(f"Failed to save preset: {e}")

    def _select_input_file(self) -> None:
        """Open a file picker dialog to select an input USD file.

        The picker navigates to the parent directory of the current input
        file value, when available.
        """
        self._close_file_picker()
        self._file_picker = FilePickerDialog(
            "Select Input File",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=self._on_input_file_selected,
            enable_file_bar=False,
            file_extension_options=[
                (".usd", "USD File"),
                (".usda", "USD Ascii File"),
                (".usdc", "USD Binary File"),
                (".usdz", "USD Packaged Archive File"),
            ],
            item_filter_fn=lambda item: _filter_file_picker([".usd", ".usda", ".usdc", ".usdz"], item),
        )
        self._show_picker_and_navigate(self._resolve_input_dir())

    def _select_output_dir(self) -> None:
        """Open a file picker dialog to select the output directory.

        Navigates to the current output directory if set. Falls back to the
        parent directory of the input asset when no output directory is
        configured yet.
        """
        self._close_file_picker()
        self._file_picker = FilePickerDialog(
            "Select Output Directory",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=self._on_output_dir_selected,
            enable_file_bar=True,
        )
        self._file_picker.set_filebar_label_name("Folder Name")
        start_dir: str | None = None
        if self._file_data is not None:
            output = self._file_data.output_dir_field.model.get_value_as_string().strip()
            if output and os.path.isdir(output):
                start_dir = output
        if not start_dir:
            start_dir = self._resolve_input_dir()
        self._show_picker_and_navigate(start_dir)

    def _load_preset(self) -> None:
        """Show a popup menu with recent presets and a Browse option."""
        recent = self._get_recent_presets()

        if not recent:
            self._load_preset_from_file_picker()
            return

        self._preset_menu = ui.Menu("Load Preset")
        with self._preset_menu:
            for entry in recent:
                ui.MenuItem(
                    entry["name"],
                    tooltip=entry["path"],
                    triggered_fn=lambda p=entry["path"]: self._load_preset_from_path(p),
                )
            ui.Separator()
            ui.MenuItem(
                "Browse...",
                triggered_fn=self._load_preset_from_file_picker,
            )
        self._preset_menu.show()

    def _load_preset_from_file_picker(self) -> None:
        """Open a file picker dialog to browse for and load a preset JSON file."""
        self._close_file_picker()
        self._file_picker = FilePickerDialog(
            "Select Preset",
            allow_multi_selection=False,
            apply_button_label="Load",
            click_apply_handler=self._on_preset_load_selected,
            enable_file_bar=False,
            file_extension_options=[(".json", "JSON Files (*.json, *.JSON)")],
            item_filter_fn=lambda item: _filter_file_picker([".json"], item),
        )
        self._show_picker_and_navigate(self._resolve_last_preset_dir())

    def _save_preset(self) -> None:
        """Open a file picker dialog to save the current profile as a JSON preset.

        Navigates to the directory of the last-used preset file. Falls back
        to the user's ``Documents`` folder (or home directory) when no recent
        preset path is available.
        """
        self._close_file_picker()
        self._file_picker = FilePickerDialog(
            "Save Preset",
            allow_multi_selection=False,
            apply_button_label="Save",
            click_apply_handler=self._on_preset_save_selected,
            file_extension_options=[(".json", "JSON Files (*.json, *.JSON)")],
            item_filter_fn=lambda item: _filter_file_picker([".json"], item),
        )
        target_dir = self._resolve_last_preset_dir()
        if not target_dir:
            docs = str(Path.home() / "Documents")
            target_dir = docs if os.path.isdir(docs) else str(Path.home())
        self._show_picker_and_navigate(target_dir)

    def _resolve_last_preset_dir(self) -> str | None:
        """Return the directory of the last-used preset, if it still exists.

        Falls back to the parent directory of the most recent preset entry
        from persistent settings.

        Returns:
            The directory path, or None.
        """
        if self._last_preset_dir and os.path.isdir(self._last_preset_dir):
            return self._last_preset_dir
        recent = self._get_recent_presets()
        if recent:
            parent = str(Path(recent[0]["path"]).parent)
            if os.path.isdir(parent):
                return parent
        return None

    def _build_actions_set_panel(self) -> None:
        """Build the actions configuration section with preset and profile controls."""
        with ui.VStack(name="indent"):
            self._build_info_section(ACTION_SET_INFO_TEXT)
            ui.Spacer(height=4)
            # Load/Save and Clear All buttons
            with ui.HStack(height=0, name="no_margin"):
                ui.Button(
                    text="Load Preset  ",
                    image_url=FOLDER_ICON_URL,
                    image_width=24,
                    width=0,
                    clicked_fn=self._load_preset,
                )
                ui.Spacer(width=4)
                ui.Button(
                    text="Save Preset  ",
                    image_url=SAVE_ICON_URL,
                    image_width=24,
                    width=0,
                    clicked_fn=self._save_preset,
                )
                ui.Spacer(width=ui.Fraction(1))
                self._clear_all_button = ui.Button(
                    text="Clear All Actions  ",
                    image_url=REMOVE_ICON_URL,
                    image_width=24,
                    width=0,
                    clicked_fn=self._clear_all_actions,
                )
            ui.Spacer(height=4)
            # -- Profile-level parameters -----------------------------------
            with ui.CollapsableFrame("Profile Settings", collapsed=True, height=0):
                with ui.VStack(spacing=4, height=0):
                    ui.Spacer(height=2)
                    with ui.HStack(height=0, spacing=8):
                        ui.Label("Profile Name", width=ui.Percent(30))
                        ui.StringField(model=self._profile_name_model)
                    with ui.HStack(height=0, spacing=8):
                        ui.Label("Version", width=ui.Percent(30))
                        ui.StringField(model=self._profile_version_model)
                    with ui.HStack(height=0, spacing=8):
                        ui.Label("Interface Asset", width=ui.Percent(30))
                        ui.StringField(model=self._profile_interface_model)
                    with ui.HStack(height=0, spacing=8):
                        ui.Label("Base Name", width=ui.Percent(30))
                        ui.StringField(model=self._profile_base_name_model)
                    with ui.HStack(height=0, spacing=8):
                        ui.Label("Flatten Source", width=ui.Percent(30))
                        ui.CheckBox(height=0, model=self._profile_flatten_model)
                        ui.Spacer()
                    ui.Spacer(height=2)
            ui.Spacer(height=4)
            self._action_list_frame = ActionListFrame(self._action_list_model)

            # NOTE: This is a ZStack hack:
            # I couldn't get the icon and text to be centered but next to each other in
            # the button normally, so I'm just putting them atop the button looking how
            # they should.
            with ui.ZStack():
                ui.Button(
                    text=" ",
                    name="add_action",
                    clicked_fn=self._add_action,
                )
                with ui.HStack():
                    ui.Spacer()
                    ui.Image(ADD_ICON_URL, width=24, height=24)
                    ui.Label("Add Action", width=0)
                    ui.Spacer()
            ui.Spacer(height=8)

    def _add_action(self) -> None:
        """Append a new default action to the action list."""
        action_count = len(self._action_list_model.get_item_children(None))
        default_rule_type = ""
        rule_types = self._rule_registry.list_rule_types()
        if rule_types:
            default_rule_type = rule_types[0]
            default_name = default_rule_type.split(".")[-1]
        else:
            default_name = f"Rule {action_count + 1}"
            carb.log_warn("No registered rules found for new action")

        spec = RuleSpec(
            name=default_name,
            type=default_rule_type,
            destination=None,
            params={},
            enabled=True,
        )
        action = RuleActionItem(spec, registry=self._rule_registry)
        self._action_list_model.append_child_item(None, ActionItemValueModel(action))

    def _clear_all_actions(self) -> None:
        """Remove all actions from the action list."""
        self._action_list_model.clear_all()

    def _update_button_states(self) -> None:
        """Update the enabled state of *Clear All* and *Execute* buttons.

        *Clear All* is enabled when actions exist. *Execute* is enabled when
        at least one action is enabled and the output directory is set.
        """
        has_actions = len(self._action_list_model.get_item_children()) > 0
        has_enabled_actions = self._action_list_model.has_enabled_actions()
        has_output_file = (
            self._file_data is not None
            and self._file_data.output_dir_field is not None
            and self._file_data.output_dir_field.model.get_value_as_string().strip() != ""
        )

        if self._clear_all_button is not None:
            self._clear_all_button.enabled = has_actions

        if self._execute_button is not None:
            self._execute_button.enabled = has_enabled_actions and has_output_file

    def _build_review_and_execute_panel(self) -> None:
        """Build the review and execute section with the Execute button."""
        with ui.VStack(name="indent", spacing=4):
            self._build_info_section(REVIEW_INFO_TEXT)

            with ui.HStack(height=50):
                self._execute_button = ui.Button(
                    name="execute_action",
                    text="Execute Actions",
                    alignment=ui.Alignment.CENTER,
                    image_url=EXECUTE_ICON_URL,
                    image_width=30,
                    clicked_fn=self._run_actions,
                )

            ui.Spacer(height=8)
