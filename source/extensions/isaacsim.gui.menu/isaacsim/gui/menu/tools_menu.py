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

"""Tools menu layout for Isaac Sim."""

import omni.kit.menu.utils
from omni.kit.menu.utils import LayoutSourceSearch, MenuItemDescription, MenuLayout, add_menu_items


class ToolsMenuExtension:
    """Build and manage the Tools menu.

    Args:
        ext_id: Extension identifier provided by the extension manager.
    """

    def __init__(self, ext_id: str) -> None:
        self.__menu_layout = [
            MenuLayout.Menu(
                "Tools",
                [
                    MenuLayout.SubMenu(
                        "Action and Event Data Generation",
                        [
                            MenuLayout.Seperator("Actor Simulation and SDG"),
                            MenuLayout.Item("Actor SDG"),
                            MenuLayout.Item("Command Injection"),
                            MenuLayout.Item("Command Setting"),
                            MenuLayout.Seperator("Object Simulation and SDG"),
                            MenuLayout.Item("Object SDG"),
                            MenuLayout.Item("Distribution Visualizer"),
                            MenuLayout.Seperator("VLM Scene Captioning"),
                            MenuLayout.Item("VLM Scene Captioning"),
                            MenuLayout.Seperator("Physical Space Event Generation"),
                            MenuLayout.Item("Event Scene Tagger"),
                            MenuLayout.Item("Event Config File"),
                        ],
                        source_search=LayoutSourceSearch.LOCAL_ONLY,
                    ),
                    MenuLayout.SubMenu(
                        "Animation",
                        [
                            MenuLayout.Item("Retargeting", source="Window/Animation/Retargeting"),  # ??
                            MenuLayout.Item(
                                "Simplify Animation Curve", source="Tools/Animation/Curve Processing/Simplify Curve"
                            ),
                            MenuLayout.Item("Stage Recorder", source="Window/Animation/Stage Recorder"),
                            MenuLayout.Item(
                                "Timesamples to Curves", source="Tools/Animation/Convert/USD TimeSample to Curves"
                            ),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Physics",
                        [
                            MenuLayout.Item(
                                "Collision Groups Filtering Matrix",
                                source="Window/Physics/Collision Groups Filtering Matrix",
                            ),
                            MenuLayout.Item("Physics API Editor"),
                            MenuLayout.Item("Physics Inspector"),
                            MenuLayout.Item("PhysX Character Controller", source="Window/Physics/Character Controller"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Replicator",
                        [
                            MenuLayout.Item(name="Replicator YAML"),
                            MenuLayout.Item(name="Semantics Schema Editor"),
                            MenuLayout.Item(name="Synthetic Data Recorder"),
                            MenuLayout.Seperator("Orchestrator"),
                            MenuLayout.Item(name="Preview"),
                            MenuLayout.Item(name="Start"),
                            MenuLayout.Item(name="Step"),
                            MenuLayout.Item(name="Pause"),
                            MenuLayout.Item(name="Resume"),
                            MenuLayout.Item(name="Stop"),
                            MenuLayout.Seperator("Grasping"),
                            MenuLayout.Item(name="Grasping"),
                            MenuLayout.Seperator("Teleop"),
                            MenuLayout.Item(name="Teleop"),
                            MenuLayout.Item(name="Episode Recorder"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Robotics",
                        [
                            MenuLayout.SubMenu(
                                "Asset Editors",
                                [
                                    MenuLayout.Item("Gain Tuner"),
                                    MenuLayout.Item("Mesh Merge Tool"),
                                    MenuLayout.Item("Robot Assembler"),
                                    MenuLayout.Item("Robot Self-Collision Detector"),
                                ],
                            ),
                            MenuLayout.SubMenu(
                                "OmniGraph Controllers",
                                [
                                    MenuLayout.Item(name="Differential Controller"),
                                    MenuLayout.Item(name="Joint Position"),
                                    MenuLayout.Item(name="Joint Velocity"),
                                    MenuLayout.Item(name="Open Loop Gripper"),
                                ],
                            ),
                            MenuLayout.Item("ROS 2 OmniGraphs"),
                            MenuLayout.Seperator("Navigation"),
                            MenuLayout.Item("Heightmap Importer"),
                            MenuLayout.Item("Occupancy Map"),
                            MenuLayout.Seperator("Manipulation"),
                            MenuLayout.Item("Grasp Editor"),
                            MenuLayout.Item("Lula Robot Description Editor"),
                            MenuLayout.Item("Lula Test Widget"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Sensors",
                        [
                            MenuLayout.Item("Camera Calibration"),
                            MenuLayout.Item("Camera Placement"),
                            MenuLayout.Item("Camera Inspector"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "USD",
                        [
                            MenuLayout.Item("USD Paths", source="Window/USD Paths"),
                            MenuLayout.Item("Variant Presenter", source="Tools/Variants/Variant Presenter"),
                            MenuLayout.Item("Variant Editor", source="Tools/Variants/Variant Editor"),
                        ],
                    ),
                    MenuLayout.Seperator("Toolbars"),
                    MenuLayout.Item("Main ToolBar", source="Window/Main ToolBar"),
                    MenuLayout.Item("Physics Toolbar", "Window/Physics/Physics Authoring Toolbar"),
                ],
            )
        ]
        omni.kit.menu.utils.add_layout(self.__menu_layout)

        physics_inspector = MenuItemDescription(
            name="Physics Inspector",
            onclick_action=("omni.physx.supportui", "show_physics_inspector"),
        )

        add_menu_items([physics_inspector], "Tools")

    def shutdown(self) -> None:
        """Remove menu layouts.

        Example:
            .. code-block:: python

                menu = ToolsMenuExtension("ext.id")
                menu.shutdown()
        """
        omni.kit.menu.utils.remove_layout(self.__menu_layout)
