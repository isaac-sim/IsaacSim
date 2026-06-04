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

"""Extension that provides an interactive tutorial for getting started with Isaac Sim basics."""

import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.getting_started.getting_started import GettingStarted
from isaacsim.gui.components.ui_utils import btn_builder


class GettingStartedExtension(omni.ext.IExt):
    """Extension that provides an interactive tutorial for getting started with Isaac Sim basics.

    This extension implements Part I of the Isaac Sim tutorials, offering a step-by-step guided experience
    for learning fundamental concepts. It registers with the Examples Browser under the "Tutorials" category
    and provides an interactive UI for hands-on learning.

    The extension covers essential Isaac Sim operations including adding ground planes, light sources,
    visual objects, physics-enabled objects, and applying physics and collision properties to existing
    objects. Each tutorial step is presented as an interactive button that executes the corresponding
    operation when clicked.

    The tutorial follows a logical progression where users learn to:
    - Add basic scene elements (ground plane and lighting)
    - Create visual objects without physics
    - Create physics-enabled objects
    - Apply physics and collision properties to existing objects

    The extension integrates with the Examples Browser system and provides documentation links to the
    official Isaac Sim quickstart guide. It includes safety features such as disabling completed steps
    to prevent duplication and enabling subsequent steps as prerequisites are met.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension starts up.

        Initializes the Getting Started tutorial extension by setting up the UI and registering
        it with the examples browser.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Part I: Basics"
        self.category = "Tutorials"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Getting Started",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim.html",
            "overview": " Select the tutorial in the Example Browser again to restart the tutorial.\n\n 'Reset' Button is disabled. to Restart, click on the thumbnail in the browser instead. \n\n This Example follows the 'Getting Started' tutorials from the documentation\n\n Press the 'Open in IDE' button to view the source code.",
            "sample": GettingStarted(),
        }

        ui_handle = GettingStartedUI(**ui_kwargs)

        # register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self) -> None:
        """Called when the extension shuts down.

        Cleans up by deregistering the Getting Started tutorial from the examples browser.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)

        return


class GettingStartedUI(BaseSampleUITemplate):
    """UI interface for the Getting Started with Isaac Sim tutorial.

    This class provides an interactive UI that guides users through fundamental Isaac Sim concepts including
    adding ground planes, light sources, visual objects, and physics-enabled objects to a scene. The tutorial
    follows the official Getting Started documentation and demonstrates basic scene construction and physics
    setup through a series of interactive buttons.

    The UI includes collapsible frames with buttons for:
    - Adding ground plane to establish a base for the scene
    - Adding light sources for proper scene illumination
    - Creating visual cubes without physics properties
    - Creating physics-enabled cubes with collision detection
    - Adding physics and collision properties to existing objects

    Buttons are intelligently enabled/disabled based on tutorial progression to guide users through the
    proper sequence of operations. The interface integrates with the Isaac Sim examples browser system
    for easy access and navigation.

    Args:
        *args: Variable length argument list passed to the parent BaseSampleUITemplate.
        **kwargs: Additional keyword arguments passed to the parent BaseSampleUITemplate. Expected
            arguments include ext_id, file_path, title, doc_link, overview, and sample instance.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

    def build_window(self) -> None:
        """Builds the main window for the Getting Started tutorial."""

    def post_reset_button_event(self) -> None:
        """Handles actions after the reset button is pressed."""

    def post_load_button_event(self) -> None:
        """Handles actions after the load button is pressed."""

    def post_clear_button_event(self) -> None:
        """Handles actions after the clear button is pressed."""

    def build_extra_frames(self) -> None:
        """Builds additional UI frames for the Getting Started tutorial.

        Creates a collapsible frame containing interactive tutorial elements for learning Isaac Sim basics.
        """
        extra_stacks = self.get_extra_frames_handle()
        self.task_ui_elements = {}
        with extra_stacks:
            with ui.CollapsableFrame(
                title="Getting Started with Isaac Sim",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                # style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_getting_started_ui()

    def build_getting_started_ui(self) -> None:
        """Builds the main UI elements for the Getting Started tutorial.

        Creates interactive buttons for adding ground plane, light source, visual cubes, and physics cubes to the scene. Each button demonstrates different aspects of Isaac Sim scene creation and physics properties.
        """
        with ui.VStack(spacing=5):
            dict = {
                "label": "Add Ground Plane",
                "type": "button",
                "text": "Add Ground Plane",
                "tooltip": "Add a ground plane to the scene",
                "on_clicked_fn": self._add_ground_plane,
            }
            self.task_ui_elements["Add Ground Plane"] = btn_builder(**dict)

            dict = {
                "label": "Add a Light Source",
                "type": "button",
                "text": "Add Light Source",
                "tooltip": "Add a light source to the scene",
                "on_clicked_fn": self._add_light_source,
            }

            self.task_ui_elements["Add Light Source"] = btn_builder(**dict)

            dict = {
                "label": "Add Visual Cube",
                "type": "button",
                "text": "Add Visual Cube",
                "tooltip": "Add a visual cube to scene. A visual cube has no physics properties.",
                "on_clicked_fn": self._add_visual_cube,
            }
            self.task_ui_elements["Add Visual Cube"] = btn_builder(**dict)

            dict = {
                "label": "Add Physics Cube",
                "type": "button",
                "text": "Add Physics Cube",
                "tooltip": "Add a cube with physics and collision properties",
                "on_clicked_fn": self._add_physics_cube,
            }
            self.task_ui_elements["Add Physics Cube"] = btn_builder(**dict)

            dict = {
                "label": "Add Physics Properties",
                "type": "button",
                "text": "Add Physics Properties",
                "tooltip": "Append physics properties to existing objects",
                "on_clicked_fn": self._add_physics_properties,
            }
            self.task_ui_elements["Add Physics Properties"] = btn_builder(**dict)
            self.task_ui_elements["Add Physics Properties"].enabled = False

            dict = {
                "label": "Add Collision Properties",
                "type": "button",
                "text": "Add Collision Properties",
                "tooltip": "Append collision properties to existing objects",
                "on_clicked_fn": self._add_collision_properties,
            }
            self.task_ui_elements["Add Collision Properties"] = btn_builder(**dict)
            self.task_ui_elements["Add Collision Properties"].enabled = False

    def _add_visual_cube(self) -> None:
        """Adds visual cubes to the scene without physics properties.

        Creates two visual cubes with different colors (yellow and green) positioned at different locations in the scene. Disables the visual cube button and enables the physics properties button.
        """
        from isaacsim.core.experimental.objects import Cube
        from pxr import Gf

        # Create visual cube (no physics properties)
        cube = Cube("/visual_cube", sizes=0.3, positions=[0, 0.5, 1.0])
        # Set display color (yellow)
        cube.geoms[0].GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 1.0, 0.0)])

        # Create static visual cube
        cube_static = Cube("/visual_cube_static", sizes=0.3, positions=[0.5, 0, 0.5])
        # Set display color (green)
        cube_static.geoms[0].GetDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])

        self.task_ui_elements["Add Visual Cube"].enabled = False
        # enable the add physics properties button
        self.task_ui_elements["Add Physics Properties"].enabled = True

    def _add_physics_cube(self) -> None:
        """Adds a cube with physics and collision properties to the scene.

        Creates a cyan-colored cube with rigid body physics and collision APIs applied. The cube is positioned to demonstrate dynamic physics behavior when the simulation runs.
        """
        from isaacsim.core.experimental.objects import Cube
        from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
        from pxr import Gf

        # Create cube with physics and collision properties
        cube = Cube("/dynamic_cube", sizes=0.3, positions=[0, -0.5, 1.5])
        # Set display color (cyan)
        cube.geoms[0].GetDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 1.0)])

        # Apply rigid body physics
        RigidPrim("/dynamic_cube")

        # Apply collision APIs
        GeomPrim("/dynamic_cube", apply_collision_apis=True)

        self.task_ui_elements["Add Physics Cube"].enabled = False

    def _add_ground_plane(self) -> None:
        """Adds a ground plane to the scene.

        Creates a ground plane at the world origin to provide a surface for physics objects to interact with. Disables the ground plane button after creation.
        """
        from isaacsim.core.experimental.objects import GroundPlane

        GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
        self.task_ui_elements["Add Ground Plane"].enabled = False

    def _add_light_source(self) -> None:
        """Adds a distant light source to the scene.

        Creates a distant light with intensity of 300 to illuminate the scene. Disables the light source button after creation.
        """
        from isaacsim.core.experimental.objects import DistantLight

        light = DistantLight("/DistantLight")
        light.set_intensities(300)
        self.task_ui_elements["Add Light Source"].enabled = False

    def _add_physics_properties(self) -> None:
        """Adds physics properties to an existing visual cube in the scene.

        Applies rigid body physics to the '/visual_cube' prim and updates the UI button states to enable collision properties and disable physics properties buttons.
        """
        from isaacsim.core.experimental.prims import RigidPrim

        # Add physics properties to existing object
        RigidPrim("/visual_cube")
        self.task_ui_elements["Add Collision Properties"].enabled = True
        self.task_ui_elements["Add Physics Properties"].enabled = False

    def _add_collision_properties(self) -> None:
        """Adds collision properties to an existing visual cube in the scene.

        Applies collision APIs to the '/visual_cube' prim using GeomPrim and disables the collision properties button in the UI.
        """
        from isaacsim.core.experimental.prims import GeomPrim

        GeomPrim("/visual_cube", apply_collision_apis=True)

        self.task_ui_elements["Add Collision Properties"].enabled = False
