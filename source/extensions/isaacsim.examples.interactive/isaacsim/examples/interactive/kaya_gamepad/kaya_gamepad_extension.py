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

"""Extension for controlling a NVIDIA Kaya robot using gamepad input in Isaac Sim interactive examples."""

import os

import omni.ext
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.kaya_gamepad import KayaGamepad


class KayaGamepadExtension(omni.ext.IExt):
    """Extension that provides an interactive example for controlling a NVIDIA Kaya robot using a gamepad.

    This extension demonstrates how to integrate gamepad input with Isaac Sim to control robotic systems. It creates
    a user interface that allows users to connect a gamepad and drive the Kaya robot in simulation. The extension
    registers itself with the examples browser, making it accessible through the standard Isaac Sim examples interface.

    The example includes comprehensive documentation and source code access, enabling users to understand and modify
    the gamepad control implementation for their own robotic applications.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Kaya Gamepad extension and registers it with the examples browser.

        Sets up the UI template for the NVIDIA Kaya robot gamepad control example and makes it available
        in the examples browser under the Input Devices category.

        Args:
            ext_id: The extension identifier string.
        """
        self.example_name = "Kaya Gamepad"
        self.category = "Input Devices"

        overview = "This Example shows how to drive a NVIDIA Kaya robot using a Gamepad in Isaac Sim."
        overview += "\n\nConnect a gamepad to the robot, and the press PLAY to begin simulating."
        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "NVIDIA Kaya Gamepad Example",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/examples.html",
            "overview": overview,
            "sample": KayaGamepad(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Cleans up the extension by deregistering the Kaya Gamepad example from the examples browser."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
