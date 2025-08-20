# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio
import os.path
import sys
import webbrowser

import carb.settings
import carb.tokens
import omni.appwindow
import omni.client
import omni.ext
import omni.kit.app
import omni.kit.commands
import omni.kit.stage_templates as stage_templates
import omni.kit.ui
import omni.ui as ui
from isaacsim.core.version import get_version
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, build_submenu_dict
from omni.kit.window.title import get_main_window_title

DOCS_URL = "https://docs.omniverse.nvidia.com"
REFERENCE_GUIDE_URL = DOCS_URL + "/isaacsim"
ASSETS_GUIDE_URL = DOCS_URL + "/isaacsim/latest/install_workstation.html"
MANUAL_URL = "https://docs.omniverse.nvidia.com/py/isaacsim/index.html"
FORUMS_URL = "https://forums.developer.nvidia.com/c/omniverse/simulation/69"
KIT_MANUAL_URL = DOCS_URL + "/py/kit/index.html"

from pathlib import Path

DATA_PATH = Path(__file__).parent.parent.parent.parent.parent
EXT_FOLDER = "isaacsim.app.setup"


async def _load_layout(layout_file: str, keep_windows_open=False):
    try:
        from omni.kit.quicklayout import QuickLayout

        # few frames delay to avoid the conflict with the layout of omni.kit.mainwindow
        for i in range(3):
            await omni.kit.app.get_app().next_update_async()
        QuickLayout.load_file(layout_file, keep_windows_open)

    except Exception as exc:
        pass

        QuickLayout.load_file(layout_file)


class CreateSetupExtension(omni.ext.IExt):
    """Create Final Configuration"""

    def on_startup(self, ext_id: str):
        """setup the window layout, menu, final configuration of the extensions etc"""
        self._settings = carb.settings.get_settings()
        self._ext_manager = omni.kit.app.get_app().get_extension_manager()

        # Adjust the Window Title to show the Isaac Sim Version
        window_title = get_main_window_title()
        app_version_core, app_version_prerel, _, _, _, _, _, _ = get_version()
        window_title.set_app_version(app_version_core)
        self.app_title = self._settings.get("/app/window/title")
        omni.kit.app.get_app().print_and_log(f"{self.app_title} Version: {app_version_core}-{app_version_prerel}")

        self.__setup_window_task = asyncio.ensure_future(self.__dock_windows())
        self.__setup_property_window = asyncio.ensure_future(self.__property_window())
        asyncio.ensure_future(self.__enable_ros_bridge())
        self.__menu_update()
        self.__add_app_icon(ext_id)
        self.create_new_stage = self._settings.get("/isaac/startup/create_new_stage")
        if self.create_new_stage:
            self.__await_new_scene = asyncio.ensure_future(self.__new_stage())

        # Increase hang detection timeout
        omni.client.set_hang_detection_time_ms(10000)

        # Force navmesh baking false
        self._settings.set("/persistent/exts/omni.anim.navigation.core/navMesh/config/autoRebakeOnChanges", False)

    async def __new_stage(self):

        from omni.kit.viewport.utility import get_active_viewport, next_viewport_frame_async

        await omni.kit.app.get_app().next_update_async()
        if omni.usd.get_context().can_open_stage():
            stage_templates.new_stage(template=None)
        await omni.kit.app.get_app().next_update_async()
        await next_viewport_frame_async(get_active_viewport())
        await omni.kit.app.get_app().next_update_async()

        # Let users know when app is ready for use and live-streaming
        omni.kit.app.get_app().print_and_log(f"{self.app_title} App is loaded.")

        # Record startup time as time at which app is ready for use
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if ext_manager.is_extension_enabled("isaacsim.benchmark.services"):
            from isaacsim.benchmark.services import BaseIsaacBenchmark

            benchmark = BaseIsaacBenchmark(
                benchmark_name="app_startup",
                workflow_metadata={
                    "metadata": [
                        {"name": "mode", "data": "async"},
                    ]
                },
            )
            benchmark.set_phase("startup", start_recording_frametime=False, start_recording_runtime=False)
            benchmark.store_measurements()
            benchmark.stop()

        await omni.kit.app.get_app().next_update_async()

    def _start_app(self, app_id, console=True, custom_args=None):
        """start another Kit app with the same settings"""
        import platform
        import subprocess
        import sys

        kit_exe_path = os.path.join(os.path.abspath(carb.tokens.get_tokens_interface().resolve("${kit}")), "kit")
        if sys.platform == "win32":
            kit_exe_path += ".exe"

        app_path = carb.tokens.get_tokens_interface().resolve("${app}")
        kit_file_path = os.path.join(app_path, app_id)

        run_args = [kit_exe_path]
        run_args += [kit_file_path]
        if custom_args:
            run_args.extend(custom_args)

        # Pass all exts folders
        exts_folders = self._settings.get("/app/exts/folders")
        if exts_folders:
            for folder in exts_folders:
                run_args.extend(["--ext-folder", folder])

        kwargs = {"close_fds": False}
        if platform.system().lower() == "windows":
            if console:
                kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE | subprocess.CREATE_NEW_PROCESS_GROUP
            else:
                kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

        subprocess.Popen(run_args, **kwargs)

    def _show_ui_docs(self):
        """show the omniverse ui documentation as an external Application"""
        self._start_app("isaacsim.exp.uidoc.kit")

    def _show_selector(self):
        """show the app selector as an external Application"""
        self._start_app(
            "isaacsim.selector.kit",
            console=False,
            custom_args={"--/persistent/ext/isaacsim.app.selector/auto_start=false"},
        )

    async def __dock_windows(self):
        await omni.kit.app.get_app().next_update_async()

        assets = ui.Workspace.get_window("Isaac Sim Assets")
        content = ui.Workspace.get_window("Content")
        stage = ui.Workspace.get_window("Stage")
        layer = ui.Workspace.get_window("Layer")
        console = ui.Workspace.get_window("Console")

        await omni.kit.app.get_app().next_update_async()
        if layer:
            layer.dock_order = 1
        if stage:
            stage.dock_order = 0
            stage.focus()

        await omni.kit.app.get_app().next_update_async()
        if console:
            console.dock_order = 2
        if content:
            content.dock_order = 1
        if assets:
            assets.dock_order = 0
            assets.focus()

    async def __property_window(self):
        await omni.kit.app.get_app().next_update_async()
        import omni.kit.window.property as property_window_ext

        property_window = property_window_ext.get_window()
        property_window.set_scheme_delegate_layout(
            "Create Layout", ["path_prim", "material_prim", "xformable_prim", "shade_prim", "camera_prim"]
        )

    def _open_browser(self, path):
        import platform
        import subprocess

        if platform.system().lower() == "windows":
            webbrowser.open(path)
        else:
            # use native system level open, handles snap based browsers better
            subprocess.Popen(["xdg-open", path])

    def _open_web_file(self, path):
        filepath = os.path.abspath(path)
        if os.path.exists(filepath):
            self._open_browser("file://" + filepath)
        else:
            carb.log_warn("Failed to open " + filepath)

    def __menu_update(self):

        self.HELP_REFERENCE_GUIDE_MENU = "Help/Isaac Sim Online Guide"
        self.HELP_SCRIPTING_MANUAL = "Help/Isaac Sim Scripting Manual"
        self.HELP_FORUMS_URL = "Help/Isaac Sim Online Forums"
        self.HELP_UI_DOCS = "Help/Omni UI Docs"
        self.HELP_KIT_MANUAL = "Help/Kit Programming Manual"
        self.UI_SELECTOR_MENU_PATH = "Help/Isaac Sim App Selector"

        self._current_layout_priority = 20

        def add_layout_menu_entry(name, parameter, key):
            import inspect

            menu_path = f"Layouts/{name}"

            if inspect.isfunction(parameter):
                onclick_fn = lambda *_: asyncio.ensure_future(parameter())
            else:
                onclick_fn = lambda *_: asyncio.ensure_future(
                    _load_layout(f"{DATA_PATH}/{EXT_FOLDER}/layouts/{parameter}.json")
                )

            return MenuItemDescription(
                name=menu_path, onclick_fn=onclick_fn, hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, key)
            )

        # create Quick Load & Quick Save
        from omni.kit.quicklayout import QuickLayout

        async def quick_save():
            QuickLayout.quick_save(None, None)

        async def quick_load():
            QuickLayout.quick_load(None, None)

        menu_dict = build_submenu_dict(
            [
                MenuItemDescription(
                    name=self.HELP_REFERENCE_GUIDE_MENU, onclick_fn=lambda *_: self._open_browser(REFERENCE_GUIDE_URL)
                ),
                MenuItemDescription(
                    name=self.HELP_SCRIPTING_MANUAL, onclick_fn=lambda *_: self._open_browser(MANUAL_URL)
                ),
                MenuItemDescription(name=self.HELP_FORUMS_URL, onclick_fn=lambda *_: self._open_browser(FORUMS_URL)),
                MenuItemDescription(
                    name=self.HELP_KIT_MANUAL, onclick_fn=lambda *_: self._open_browser(KIT_MANUAL_URL)
                ),
                MenuItemDescription(name=self.HELP_UI_DOCS, onclick_fn=lambda *_: self._show_ui_docs()),
                MenuItemDescription(name=self.UI_SELECTOR_MENU_PATH, onclick_fn=lambda *_: self._show_selector()),
                add_layout_menu_entry("Default", "default", carb.input.KeyboardInput.KEY_1),
                add_layout_menu_entry("Visual Scripting", "visualScripting", carb.input.KeyboardInput.KEY_4),
                add_layout_menu_entry("Replicator", "sdg", carb.input.KeyboardInput.KEY_5),
                add_layout_menu_entry("Quick Save", quick_save, carb.input.KeyboardInput.KEY_7),
                add_layout_menu_entry("Quick Load", quick_load, carb.input.KeyboardInput.KEY_8),
            ]
        )
        for group in menu_dict:
            add_menu_items(menu_dict[group], group)

    def __add_app_icon(self, ext_id):

        extension_path = self._ext_manager.get_extension_path(ext_id)
        if sys.platform == "win32":
            pass
        else:
            user_apps_folder = os.path.expanduser("~/.local/share/applications")
            if os.path.exists(user_apps_folder):
                with open(os.path.expanduser("~/.local/share/applications/IsaacSim.desktop"), "w") as file:
                    omni.kit.app.get_app().print_and_log("Writing Isaac Sim icon file")
                    file.write(
                        f"""[Desktop Entry]
Version=1.0
Name=Isaac Sim
Icon={extension_path}/data/omni.isaac.sim.png
Terminal=false
Type=Application
StartupWMClass=IsaacSim"""
                    )

    async def __enable_ros_bridge(self):
        try:
            ros_bridge_name = self._settings.get("isaac/startup/ros_bridge_extension")
            if ros_bridge_name is not None and len(ros_bridge_name):
                await omni.kit.app.get_app().next_update_async()
                self._ext_manager.set_extension_enabled_immediate(ros_bridge_name, True)
                await omni.kit.app.get_app().next_update_async()
        except Exception as e:
            carb.log_warn(f"isaacsim.app.setup shutdown before ros bridge enabled")
