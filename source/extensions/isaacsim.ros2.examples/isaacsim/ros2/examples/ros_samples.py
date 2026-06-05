# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""ROS 2 sample scene extension for navigation examples."""

import asyncio
import os
import weakref

import carb
import omni.ext
import omni.ui as ui
import omni.usd
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import setup_ui_headers
from isaacsim.storage.native import get_assets_root_path


class Extension(omni.ext.IExt):
    """Extension providing ROS 2 navigation sample scenes."""

    # Example names and categories as class constants for easier maintenance
    NOVA_CARTER_NAME = "Nova Carter"
    NOVA_CARTER_JOINT_STATES_NAME = "Nova Carter Joint States"
    IW_HUB_NAME = "iw_hub"
    SAMPLE_SCENE_NAME = "Sample Scene"
    PERCEPTOR_SCENE_NAME = "Perceptor Scene"
    HOSPITAL_SCENE_NAME = "Hospital Scene"
    OFFICE_SCENE_NAME = "Office Scene"

    # Categories
    ROS2_NAVIGATION_CATEGORY = "ROS2/Navigation"
    ROS2_ISAAC_ROS_CATEGORY = "ROS2/Isaac ROS"
    ROS2_MULTIPLE_ROBOTS_CATEGORY = "ROS2/Navigation/Multiple Robots"

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and register all examples.

        Args:
            ext_id: The extension identifier.
        """
        self._ext_id = ext_id
        self._registered_examples = []  # Track registered examples for cleanup

        # Register Nova Carter example
        self._register_example(
            name=self.NOVA_CARTER_NAME,
            file_path="/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd",
            category=self.ROS2_NAVIGATION_CATEGORY,
        )

        # Register Nova Carter Joint States example
        self._register_example(
            name=self.NOVA_CARTER_JOINT_STATES_NAME,
            file_path="/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation_joint_states.usd",
            category=self.ROS2_NAVIGATION_CATEGORY,
        )

        # Register iw_hub example
        self._register_example(
            name=self.IW_HUB_NAME,
            file_path="/Isaac/Samples/ROS2/Scenario/iw_hub_warehouse_navigation.usd",
            category=self.ROS2_NAVIGATION_CATEGORY,
        )

        # Register Sample Scene example
        self._register_example(
            name=self.SAMPLE_SCENE_NAME,
            file_path="/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd",
            category=self.ROS2_ISAAC_ROS_CATEGORY,
        )

        # Register Perceptor Scene example
        self._register_example(
            name=self.PERCEPTOR_SCENE_NAME,
            file_path="/Isaac/Samples/ROS2/Scenario/perceptor_navigation.usd",
            category=self.ROS2_ISAAC_ROS_CATEGORY,
        )

        # Register Hospital Scene example
        self._register_example(
            name=self.HOSPITAL_SCENE_NAME,
            file_path="/Isaac/Samples/ROS2/Scenario/multiple_robot_carter_hospital_navigation.usd",
            category=self.ROS2_MULTIPLE_ROBOTS_CATEGORY,
        )

        # Register Office Scene example
        self._register_example(
            name=self.OFFICE_SCENE_NAME,
            file_path="/Isaac/Samples/ROS2/Scenario/multiple_robot_carter_office_navigation.usd",
            category=self.ROS2_MULTIPLE_ROBOTS_CATEGORY,
        )

    def _register_example(self, name: str, file_path: str, category: str) -> None:
        """Register a single example and track it for cleanup.

        Args:
            name: Display name of the example.
            file_path: USD file path for the example scene.
            category: Menu category for the example.
        """
        get_browser_instance().register_example(
            name=name,
            ui_hook=lambda a=weakref.proxy(self), n=name, f=file_path: a.build_ui(n, f),
            category=category,
        )
        # Track the registered example for proper cleanup
        self._registered_examples.append((name, category))

    def build_ui(self, name: str, file_path: str) -> None:
        """Build the UI for the example.

        Args:
            name: Display name for the example.
            file_path: USD file path for the example scene.
        """
        # check if ros2 bridge is enabled before proceeding
        extension_enabled = omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.ros2.bridge")
        if not extension_enabled:
            msg = "ROS2 Bridge is not enabled. Please enable the extension to use this feature."
            carb.log_error(msg)
        else:
            overview = "This sample demonstrates how to use ROS2 Navigation packages with Isaac Sim. \n\n The Environment Loaded already contains the OmniGraphs needed to connect with ROS2."
            self._main_stack = ui.VStack(spacing=5, height=0)
            with self._main_stack:
                setup_ui_headers(
                    self._ext_id,
                    file_path=os.path.abspath(__file__),
                    title=name,
                    overview=overview,
                    info_collapsed=False,
                )
                ui.Button(
                    "Load Sample Scene", clicked_fn=lambda a=weakref.proxy(self): a._on_environment_setup(file_path)
                )

    def _on_environment_setup(self, stage_path: str) -> None:
        """Load the specified USD stage asynchronously.

        Args:
            stage_path: Path to the USD stage to load.
        """

        async def load_stage(path: str) -> None:
            await omni.usd.get_context().open_stage_async(path)

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        scenario_path = self._assets_root_path + stage_path

        asyncio.ensure_future(load_stage(scenario_path))

    def on_shutdown(self) -> None:
        """Clean up by deregistering all registered examples."""
        for name, category in self._registered_examples:
            get_browser_instance().deregister_example(name=name, category=category)
