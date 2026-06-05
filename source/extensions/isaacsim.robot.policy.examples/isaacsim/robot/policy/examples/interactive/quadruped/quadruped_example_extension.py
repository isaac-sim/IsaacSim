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

"""Extension demonstrating quadruped robot control using a Boston Dynamics Spot with a trained policy."""

import os

import omni.ext
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.robot.policy.examples.interactive.quadruped import QuadrupedExample


class QuadrupedExampleExtension(omni.ext.IExt):
    """Extension that demonstrates quadruped robot control using a Boston Dynamics Spot with a trained policy.

    This extension provides an interactive example showing a Boston Dynamics Spot robot running a flat terrain
    policy that was trained in Isaac Lab. The demonstration includes keyboard controls for navigating the robot
    in different directions and rotational movements.

    The extension registers itself with the examples browser under the "Policy" category and provides a
    comprehensive UI with documentation links and source code access. Users can control the quadruped using
    various keyboard inputs including arrow keys and numpad controls for forward, reverse, lateral, and
    rotational movements.

    Keyboard controls include:
    - Up arrow/numpad 8: Move forward
    - Down arrow/numpad 2: Move reverse
    - Left arrow/numpad 4: Move left
    - Right arrow/numpad 6: Move right
    - N/numpad 7: Spin counterclockwise
    - M/numpad 9: Spin clockwise

    The extension integrates with the Isaac Sim examples browser system and provides access to detailed
    documentation about policy deployment in Isaac Lab.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the quadruped example extension.

        Sets up the UI template with keyboard controls for Boston Dynamics Spot robot movement
        and registers the example with the examples browser.

        Args:
            ext_id: Extension identifier string.
        """
        self.example_name = "Quadruped"
        self.category = "Policy"

        overview = "This Example shows a Boston Dynamics Spot running a flat terrain policy trained in Isaac Lab. "
        overview += "Use the Physics Engine menu in the viewport to switch between PhysX and Newton before loading. "
        overview += "\n\n\tKeyboard Input:"
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
            "title": "Quadruped: Boston Dynamics Spot",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html",
            "overview": overview,
            "sample": QuadrupedExample(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self) -> None:
        """Clean up the quadruped example extension.

        Deregisters the example from the examples browser when the extension shuts down.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return
