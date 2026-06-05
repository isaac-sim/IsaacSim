# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides an extension demonstrating keyboard input control through Omnigraph programming in Isaac Sim."""

import os

import omni.ext
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.omnigraph_keyboard import OmnigraphKeyboard


class OmnigraphKeyboardExtension(omni.ext.IExt):
    """Extension demonstrating keyboard input control through Omnigraph programming in Isaac Sim.

    This example shows how to modify a cube's size using keyboard inputs processed through
    Omnigraph nodes. Users can grow or shrink a cube by pressing 'a' or 'd' keys respectively.
    The extension integrates with the Visual Scripting Window to display the underlying
    Omnigraph implementation.

    The extension registers itself with the examples browser under the "Input Devices" category
    and provides an interactive UI for running the demonstration.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Omnigraph Keyboard extension.

        Sets up the UI template and registers the example with the examples browser. The example demonstrates how to control cube size using keyboard input through Omnigraph programming.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Omnigraph Keyboard"
        self.category = "Input Devices"

        overview = "This Example shows how to change the size of a cube using the keyboard through omnigraph progrmaming in Isaac Sim."
        overview += "\n\tKeybord Input:"
        overview += "\n\t\ta: Grow"
        overview += "\n\t\td: Shrink"
        overview += "\n\nPress the 'Open in IDE' button to view the source code."
        overview += "\nOpen Visual Scripting Window to see Omnigraph"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Omnigraph Keyboard Example",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/examples.html",
            "overview": overview,
            "sample": OmnigraphKeyboard(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Cleans up the Omnigraph Keyboard extension.

        Deregisters the example from the examples browser.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
