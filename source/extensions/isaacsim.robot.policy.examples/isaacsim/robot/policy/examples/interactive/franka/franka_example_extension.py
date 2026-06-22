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

"""Extension that demonstrates a Franka Panda robot performing a drawer opening task using a trained policy."""

import os

import omni.ext
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.robot.policy.examples.interactive.franka import FrankaExample


class FrankaExampleExtension(omni.ext.IExt):
    """Extension that demonstrates a Franka Panda robot performing a drawer opening task using a trained policy.

    This extension provides an interactive example showcasing a Franka Panda robot executing a drawer opening
    policy that was trained in Isaac Lab. The robot attempts to open a drawer in front of it and maintain
    it in an open position. The simulation automatically resets every 10 seconds (simulation time), allowing
    the robot to repeatedly demonstrate the task.

    The extension integrates with the Isaac Sim examples browser, registering itself under the "Policy"
    category as "Franka". It provides a complete UI interface through the BaseSampleUITemplate, including
    documentation links and overview information for users.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Franka example extension and registers it with the examples browser.

        Sets up the Franka Panda drawer opening policy example with UI components and documentation.
        The example demonstrates a trained Isaac Lab policy where the Franka robot attempts to open
        a drawer and hold it open, resetting every 10 seconds of simulation time.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Franka"
        self.category = "Policy"

        overview = "This Example shows a Franka Panda open drawer policy trained in Isaac Lab. "
        overview += "The Franka will attempt to open the drawer in front of it and hold it open. "
        overview += "The scene will reset every 10s (sim time) and the Franka will try again."

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Manipulator: Franka",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html",
            "overview": overview,
            "sample": FrankaExample(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # Register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Cleans up the extension by deregistering the Franka example from the examples browser."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
