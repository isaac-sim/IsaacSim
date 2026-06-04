# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the exposed variables property widget UI."""

import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.kit.window.property
import omni.usd
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS
from isaacsim.replicator.behavior.ui.exposed_variables_widget import ExposedVariablesPropertyWidget
from isaacsim.replicator.behavior.ui.global_variables import WIDGET_NAME
from omni.kit.window.property.property_widget import PropertyWidget
from pxr import Sdf

BEHAVIOR_SCRIPTS_EXTENSION_NAME = "isaacsim.replicator.behavior"
BEHAVIOR_SCRIPT_PATH = "/isaacsim/replicator/behavior/behaviors/example_behavior.py"


class TestExposedVariablesWidgetUI(omni.kit.test.AsyncTestCase):
    """Test the exposed variables property widget UI registration and building."""

    async def setup(self) -> None:
        """Set up test fixtures."""

    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    async def test_widget_registered(self) -> None:
        """Verify the exposed variables widget is registered in the property window."""
        await omni.usd.get_context().new_stage_async()

        # Get the widget from the property window
        property_window = omni.kit.window.property.get_window()
        widgets_top = property_window._widgets_top
        widgets_top_prim_type = widgets_top.get("prim")
        exposed_variables_widget = widgets_top_prim_type.get(WIDGET_NAME)

        # Check if the widget is registered
        self.assertTrue(exposed_variables_widget, f"Widget {WIDGET_NAME} not found in property window")

        # Check if the widget is an instance of PropertyWidget
        self.assertTrue(isinstance(exposed_variables_widget, PropertyWidget))

        # Check if the namespace is in the filter list
        widget_filter = exposed_variables_widget._attribute_namespace_filter
        self.assertTrue(
            EXPOSED_ATTR_NS in widget_filter,
            f"Namespace {EXPOSED_ATTR_NS} not found in filter list: {widget_filter}",
        )

    async def test_widget_built(self) -> None:
        """Verify the widget is built for prims with behavior scripts and not for those without."""
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        prim_path = "/World/MyPrim"
        prim = stage.DefinePrim(prim_path, "Xform")
        prim_path_no_behaviors = "/World/MyPrimNoBehaviors"
        prim_no_behaviors = stage.DefinePrim(prim_path_no_behaviors, "Xform")

        # Add scripting API to the prim
        omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[prim_path])
        scripts_attr = prim.GetAttribute("omni:scripting:scripts")
        self.assertTrue(scripts_attr, f"No 'omni:scripting:scripts' attribute found on prim: {prim_path}")

        # Add example behavior script to the prim
        extension_manager = omni.kit.app.get_app().get_extension_manager()
        extension_id = extension_manager.get_enabled_extension_id(BEHAVIOR_SCRIPTS_EXTENSION_NAME)
        extension_path = extension_manager.get_extension_path(extension_id)
        script_path = f"{extension_path}{BEHAVIOR_SCRIPT_PATH}"

        # Add the behavior script to the prim
        current_scripts = scripts_attr.Get()
        if current_scripts is None:
            current_scripts = []
        current_scripts.append(Sdf.AssetPath(script_path))
        scripts_attr.Set(current_scripts)

        # NOTE, at least 3 updates are needed to ensure the script is loaded and the exposed vars are set
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        # Select the prim with the script and check if the widget is built
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[prim_path])

        # NOTE: at least 2 updates are needed to ensure the widget is built
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()

        # Check if the widget is in the built widgets list
        property_window = omni.kit.window.property.get_window()
        built_widgets = property_window._built_widgets
        exposed_variables_widget = next(
            (widget for widget in built_widgets if isinstance(widget, ExposedVariablesPropertyWidget)), None
        )
        self.assertTrue(exposed_variables_widget, f"ExposedVariablesPropertyWidget not found in the built widgets")

        # Select the prim with the script and check if the widget is built
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[prim_path_no_behaviors])

        # NOTE: at least 2 updates are needed to ensure the widget is built
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()

        # Make sure the widget is not built for a prim without behaviors
        built_widgets = property_window._built_widgets
        exposed_variables_widget = next(
            (widget for widget in built_widgets if isinstance(widget, ExposedVariablesPropertyWidget)), None
        )
        self.assertFalse(
            exposed_variables_widget,
            f"ExposedVariablesPropertyWidget found in the built widgets for prim without behaviors",
        )
