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

"""Extension that provides a Unitree H1 humanoid robot policy example."""

import os

import omni.ext
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.robot.policy.examples.interactive.humanoid import HumanoidExample


class HumanoidExampleExtension(omni.ext.IExt):
    """Extension that provides a Unitree H1 humanoid robot policy example.

    This extension demonstrates a Unitree H1 humanoid robot running a flat terrain locomotion policy
    trained in Isaac Lab. The example showcases policy deployment for humanoid robots in Isaac Sim.

    The extension registers itself with the examples browser under the "Policy" category and provides
    a user interface for interacting with the humanoid robot simulation. Users can control the robot
    using keyboard inputs to move forward and rotate.

    Keyboard controls:
        - Up arrow / numpad 8: Move forward
        - Left arrow / numpad 4: Spin counterclockwise
        - Right arrow / numpad 6: Spin clockwise
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Humanoid example extension.

        Registers the Unitree H1 humanoid robot example with the examples browser and creates the UI template
        with keyboard controls for forward movement and rotation.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Humanoid"
        self.category = "Policy"

        overview = "This Example shows a Unitree H1 running a flat terrain policy trained in Isaac Lab. "
        overview += "Use the Physics Engine menu in the viewport to switch between PhysX and Newton before loading. "
        overview += "\n\n\tKeyboard Input:"
        overview += "\n\t\tup arrow / numpad 8: Move Forward"
        overview += "\n\t\tleft arrow / numpad 4: Spin Counterclockwise"
        overview += "\n\t\tright arrow / numpad 6: Spin Clockwise"
        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Humanoid: Unitree H1",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html",
            "overview": overview,
            "sample": HumanoidExample(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # Register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Cleans up the extension by deregistering the Humanoid example from the examples browser."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
