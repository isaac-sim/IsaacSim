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

"""Extension that provides a Hello World example demonstrating basic Isaac Sim scripting concepts through interactive tutorials."""

import os

import omni.ext
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.hello_world import HelloWorld


class HelloWorldExtension(omni.ext.IExt):
    """Extension providing a Hello World example for Isaac Sim.

    This extension demonstrates basic Isaac Sim scripting concepts through an interactive example that teaches
    users fundamental operations in asynchronous mode. The extension creates a UI template with comprehensive
    documentation and registers itself with the examples browser for easy access.

    The Hello World example serves as an entry point for users to learn Isaac Sim's core API functionality
    through practical demonstrations and hands-on interaction.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the Hello World extension.

        Sets up the example UI template and registers it with the examples browser.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Hello World"
        self.category = "General"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Hello World Example",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_hello_world.html",
            "overview": "This Example introduces the user on how to do cool stuff with Isaac Sim through scripting in asynchronous mode.",
            "sample": HelloWorld(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Clean up the Hello World extension.

        Deregisters the example from the examples browser.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
