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

"""Extension for registering the Go2 locomotion example in the examples browser."""

import os

import omni.ext
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.robot.policy.examples.interactive.go2 import Go2Example


class Go2ExampleExtension(omni.ext.IExt):
    """Register the Go2 locomotion example in the examples browser."""

    def on_startup(self, ext_id: str):
        """Register the Go2 example on extension startup.

        Args:
            ext_id: Extension identifier.
        """
        self.example_name = "Go2"
        self.category = "Policy"

        overview = "This Example shows a Unitree Go2 running a flat terrain policy trained in Isaac Lab. "
        overview += "Use the Physics Engine menu in the viewport to switch between PhysX and Newton before loading. "
        overview += "\n\n\tKeybord Input:"
        overview += "\n\t\tup arrow / numpad 8: Move Forward"
        overview += "\n\t\tdown arrow/ numpad 2: Move Reverse"
        overview += "\n\t\tleft arrow/ numpad 4: Move Left"
        overview += "\n\t\tright arrow / numpad 6: Move Right"
        overview += "\n\t\tN / numpad 7: Spin Counterclockwise"
        overview += "\n\t\tM / numpad 9: Spin Clockwise"
        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Go2: Unitree Go2",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html",
            "overview": overview,
            "sample": Go2Example(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # Register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self):
        """Deregister the Go2 example on extension shutdown."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
