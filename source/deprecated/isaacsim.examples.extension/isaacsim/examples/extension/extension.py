# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""An Isaac Sim extension that provides a user interface for generating different types of extension templates to help users create custom extensions."""

import asyncio
import gc
from collections.abc import Callable

import omni
import omni.kit.actions.core
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import CollapsableFrame, ScrollingWindow, TextBlock
from isaacsim.gui.components.ui_utils import btn_builder, get_style, setup_ui_headers, str_builder
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, refresh_menu_items, remove_menu_items

from .template_generator import TemplateGenerator

EXTENSION_NAME = "Generate Extension Templates"


class Extension(omni.ext.IExt):
    """An Isaac Sim extension for generating code templates to help users create custom extensions.

    This extension provides a user interface for generating different types of extension templates,
    including configuration tooling, loaded scenario, scripting, and UI component library templates.
    Users can specify the extension path, title, and description to create starter code for building
    standalone UI-based extensions in Isaac Sim.

    The extension creates a dockable window with forms for each template type, allowing users to:
    - Select the target directory for the generated extension
    - Provide an extension title and description
    - Generate the template files with proper structure and boilerplate code

    The generated templates serve as starting points for extension development, providing the necessary
    file structure, imports, and basic functionality patterns commonly used in Isaac Sim extensions.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: The extension identifier.
        """
        # Events
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_NAME, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        self._action_id = "show_generate_extension_templates"

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            self._action_id,
            lambda *_: self._menu_callback(),
            description=f"Open {EXTENSION_NAME} window",
        )

        self._menu_items = [
            MenuItemDescription(
                name=EXTENSION_NAME,
                onclick_action=(ext_id, self._action_id),
                ticked=True,
                ticked_fn=self._is_visible,
            )
        ]
        add_menu_items(self._menu_items, "Utilities")

        self._template_generator = TemplateGenerator()

    def on_shutdown(self) -> None:
        """Clean up extension resources and UI components."""
        self._models = {}
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_id, self._action_id)
        remove_menu_items(self._menu_items, "Utilities")
        if self._window:
            self._window = None
        gc.collect()

    def _is_visible(self) -> bool:
        return self._window.visible if self._window else False

    def _on_window(self, visible: bool) -> None:
        """Handle window visibility changes.

        Args:
            visible: The visibility state of the window.
        """
        refresh_menu_items("Utilities")
        if self._window.visible:
            self._build_ui()

    def _menu_callback(self) -> None:
        """Toggle the extension window visibility when menu item is clicked."""
        self._window.visible = not self._window.visible

    def _build_ui(self) -> None:
        """Build the main user interface for the extension template generator."""
        # if not self._window:
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                self._build_info_ui()

                self._build_template_ui(
                    "Configuration Tooling Template", self._template_generator.generate_configuration_tooling_template
                )

                self._build_template_ui(
                    "Loaded Scenario Template", self._template_generator.generate_loaded_scenario_template
                )

                self._build_template_ui("Scripting Template", self._template_generator.generate_scripting_template)

                self._build_template_ui(
                    "UI Component Library", self._template_generator.generate_component_library_template
                )

                self._build_status_panel()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()

            def dock(space: object, name: str, location: object, pos: float = 0.5) -> object:
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _build_info_ui(self) -> None:
        """Build the information section of the UI with title, documentation link, and overview."""
        title = EXTENSION_NAME
        doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/utilities/extension_template_generator.html"

        overview = (
            "Generate Extension Templates to get started building and programming standalone UI-based extensions in "
            + "Isaac Sim."
        )

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_status_panel(self) -> None:
        """Build the status panel UI component for displaying generation status messages."""
        self._status_frame = CollapsableFrame("Status Frame", collapsed=True, visible=False)
        with self._status_frame:
            self._status_block = TextBlock("Status", "", num_lines=3, include_copy_button=False)

    def _build_template_ui(self, template_name: str, generate_fun: Callable) -> None:
        """Build the UI for a specific extension template with input fields and generation controls.

        Args:
            template_name: Name of the extension template.
            generate_fun: Function to call for generating the template.
        """
        frame = ui.CollapsableFrame(
            title=template_name,
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        path_field = template_name + "_path"
        title_field = template_name + "_title"
        generate_btn = template_name + "_generate"
        description_field = template_name + "_description"

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def control_generate_btn(model: object = None) -> None:
                    path = self._models[path_field].get_value_as_string()
                    title = self._models[title_field].get_value_as_string()

                    if path != "" and path[-1] != "/" and path[-1] != "\\" and title.strip(" ") != "":
                        self._models[generate_btn].enabled = True
                        self.write_status(f"Ready to Generate {template_name}")
                    else:
                        self._models[generate_btn].enabled = False
                        self.write_status(
                            "Cannot Generate Extension Template Without a Title and Valid Path.  The Path must not end in a '/'."
                        )

                self._models[path_field] = str_builder(
                    label="Extension Path",
                    tooltip="Directory where the extension template will be populated.  The path must not end in a slash",
                    use_folder_picker=True,
                    item_filter_fn=lambda item: item.is_folder,
                    folder_dialog_title="Select Path",
                    folder_button_title="Select",
                )
                self._models[path_field].add_value_changed_fn(control_generate_btn)

                self._models[title_field] = str_builder(
                    label="Extension Title",
                    default_val="",
                    tooltip="Title of Extension that will show up on Isaac Sim Toolbar",
                )
                self._models[title_field].add_value_changed_fn(control_generate_btn)

                self._models[description_field] = str_builder(
                    label="Extension Description", default_val="", tooltip="Short description of extension"
                )

                def on_generate_extension(model: object = None, val: object = None) -> None:
                    path = self._models[path_field].get_value_as_string()
                    title = self._models[title_field].get_value_as_string()
                    description = self._models[description_field].get_value_as_string()
                    generate_fun(path, title, description)

                    self.write_status(f"Created new extension '{title}' at {path} from {template_name}")

                self._models[generate_btn] = btn_builder(
                    label="Generate Extension",
                    text="Generate Extension",
                    tooltip=f"Generate {template_name}",
                    on_clicked_fn=on_generate_extension,
                )
                self._models[generate_btn].enabled = False

    def write_status(self, status: str, collapsed: bool = False, visible: bool = True) -> None:
        """Update the status panel with a message and control its visibility.

        Args:
            status: The status message to display.
            collapsed: Whether the status frame should be collapsed.
            visible: Whether the status frame should be visible.
        """
        self._status_block.set_text(status)
        self._status_frame.collapsed = collapsed
        self._status_frame.visible = visible
