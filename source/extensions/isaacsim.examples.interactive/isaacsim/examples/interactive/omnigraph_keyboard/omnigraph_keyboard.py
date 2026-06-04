# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Interactive sample demonstrating keyboard control of a cube's size using Omni Graph."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.storage.native import get_assets_root_path


class OmnigraphKeyboard(BaseSample):
    """Interactive sample demonstrating keyboard control of a cube's size using Omni Graph.

    This sample creates a cyan cube in the scene and sets up an Omni Graph that responds to keyboard input.
    The cube's size can be dynamically modified during simulation using the 'A' and 'D' keys:

    - 'A' key: Increases the cube's size
    - 'D' key: Decreases the cube's size

    The implementation showcases how to:
    - Create geometric objects using the experimental API
    - Apply materials to objects for visual customization
    - Build an Omni Graph for real-time keyboard interaction
    - Connect multiple graph nodes to process input and modify object properties
    - Handle scene lifecycle events (reset, clear) to maintain object state

    The graph architecture includes keyboard state readers, type converters, mathematical operations,
    and prim attribute readers/writers that work together to translate key presses into size changes.
    The cube is placed on a ground plane environment and the camera is positioned to provide an optimal
    viewing angle for the interactive demonstration.

    This example is useful for understanding how to create interactive simulations that respond to
    user input through Omni Graph, demonstrating the integration between USD prims, materials,
    and graph-based logic systems.
    """

    def __init__(self) -> None:
        super().__init__()
        self._gamepad_gains = (40.0, 40.0, 2.0)
        self._gamepad_deadzone = 0.15
        self._cube = None
        self._initial_position = [0, 0, 1.0]
        self._initial_size = 1.0

    def setup_scene(self) -> None:
        """Sets up the scene with a controllable cube and keyboard input graph.

        Creates a cyan cube that can be resized using A and D keys through an Omni Graph action graph.
        Also adds a ground plane environment and configures the camera view.
        """
        # Create cube using experimental API
        self._cube = Cube(
            "/Cube", positions=self._initial_position, sizes=[self._initial_size], reset_xform_op_properties=True
        )

        # Apply cyan color material
        material = PreviewSurfaceMaterial("/Looks/cyan_material")
        material.set_input_values("diffuseColor", [0.0, 1.0, 1.0])  # RGB for cyan
        self._cube.apply_visual_materials(material)

        # Add ground plane environment for physics simulation
        ground_plane = stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[5, 5, 3], target=[0, 0, 0])

        # setup graph
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("A", "omni.graph.nodes.ReadKeyboardState"),
                    ("D", "omni.graph.nodes.ReadKeyboardState"),
                    ("ToDouble1", "omni.graph.nodes.ToDouble"),
                    ("ToDouble2", "omni.graph.nodes.ToDouble"),
                    ("Negate", "omni.graph.nodes.Multiply"),
                    ("DeltaAdd", "omni.graph.nodes.Add"),
                    ("SizeAdd", "omni.graph.nodes.Add"),
                    ("NegOne", "omni.graph.nodes.ConstantInt"),
                    ("ScaleDown", "omni.graph.nodes.Multiply"),
                    ("ScaleFactor", "omni.graph.nodes.ConstantDouble"),
                    ("CubeWrite", "omni.graph.nodes.WritePrimAttribute"),  # write prim property translate
                    ("CubeRead", "omni.graph.nodes.ReadPrimAttribute"),
                ],
                keys.SET_VALUES: [
                    ("A.inputs:key", "A"),
                    ("D.inputs:key", "D"),
                    ("OnTick.inputs:onlyPlayback", True),  # only tick when simulator is playing
                    ("NegOne.inputs:value", -1),
                    ("ScaleFactor.inputs:value", 0.1),
                    ("CubeWrite.inputs:name", "size"),
                    ("CubeWrite.inputs:primPath", "/Cube"),
                    ("CubeWrite.inputs:usePath", True),
                    ("CubeRead.inputs:name", "size"),
                    ("CubeRead.inputs:primPath", "/Cube"),
                    ("CubeRead.inputs:usePath", True),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "CubeWrite.inputs:execIn"),
                    ("A.outputs:isPressed", "ToDouble1.inputs:value"),
                    ("D.outputs:isPressed", "ToDouble2.inputs:value"),
                    ("ToDouble2.outputs:converted", "Negate.inputs:a"),
                    ("NegOne.inputs:value", "Negate.inputs:b"),
                    ("ToDouble1.outputs:converted", "DeltaAdd.inputs:a"),
                    ("Negate.outputs:product", "DeltaAdd.inputs:b"),
                    ("DeltaAdd.outputs:sum", "ScaleDown.inputs:a"),
                    ("CubeRead.outputs:value", "SizeAdd.inputs:b"),
                    ("SizeAdd.outputs:sum", "CubeWrite.inputs:value"),
                    ("ScaleFactor.inputs:value", "ScaleDown.inputs:b"),
                    ("ScaleDown.outputs:product", "SizeAdd.inputs:a"),
                ],
            },
        )

    async def setup_post_load(self) -> None:
        """Called after the scene is loaded."""

    async def setup_pre_reset(self) -> None:
        """Called before world reset."""

    async def setup_post_reset(self) -> None:
        """Called after world reset to restore cube to initial state."""
        if self._cube:
            # Reset cube position and size to initial values
            self._cube.set_world_poses(positions=self._initial_position)
            self._cube.set_sizes(sizes=[self._initial_size])

    async def setup_post_clear(self) -> None:
        """Called after clearing the scene."""
        self._cube = None
