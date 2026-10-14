# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension for generating heightmap terrain from heightmap and occupancy map images."""

from __future__ import annotations

import asyncio
import gc
import os
import weakref

import carb
import omni.ext
import omni.kit.app
import omni.kit.commands
import omni.ui as ui
import omni.usd
from isaacsim.asset.importer.heightmap.importer import HeightmapImporter, generated_paths, is_importer_generated
from isaacsim.gui.components import btn_builder, cb_builder, float_builder
from isaacsim.gui.components.menu import make_menu_item_description
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from PIL import Image
from pxr import Usd, UsdGeom

__all__ = [
    "HeightmapImporter",
]

EXTENSION_NAME = "Heightmap Importer"

# UI Constants
DEFAULT_WINDOW_WIDTH = 600
DEFAULT_WINDOW_HEIGHT = 400
PREVIEW_WINDOW_WIDTH = 300
PREVIEW_WINDOW_HEIGHT = 300
DEFAULT_CELL_SIZE = 0.05
MIN_CELL_SIZE = 0.001
MAX_CELL_SIZE = 10.0

# Generating into the current stage is the default so that pressing Generate cannot
# discard unsaved scene content the user has no reason to expect losing.
DEFAULT_ADD_TO_CURRENT_STAGE = True

# A terrain generated on its own needs a floor and a light to be visible at all, so both
# stay on by default. Scenes that supply their own can switch them off.
DEFAULT_CREATE_GROUND_PLANE = True
DEFAULT_CREATE_LIGHTING = True

# Off by default so that nothing already on the stage is replaced without being asked for,
# terrain from a previous run included. Generating twice leaves two terrains, not one.
DEFAULT_OVERWRITE_EXISTING = False

# Image processing constants
MAX_IMAGE_DIMENSION = 10000  # Prevent memory issues with very large images


class Extension(omni.ext.IExt):
    """Extension for generating heightmap terrain from heightmap/occupancy map images.

    This extension provides a UI to load PNG images representing heightmaps or occupancy
    maps and generate 3D heightmap terrain in the USD stage using point instancers for
    efficient rendering of large numbers of cubes.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and create the UI window and menu items.

        Args:
            ext_id: The extension identifier string.
        """
        self._window = omni.ui.Window(
            EXTENSION_NAME, width=DEFAULT_WINDOW_WIDTH, height=DEFAULT_WINDOW_HEIGHT, visible=False
        )
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        menu_entry = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription("Robotics", sub_menu=menu_entry)]
        add_menu_items(self._menu_items, "Tools")
        self._filepicker = None
        self._visualize_window = None
        self._generation_task = None
        with self._window.frame:
            with ui.HStack(spacing=10):
                with ui.VStack(spacing=5, height=0):
                    self._cell_size = float_builder(label="Cell Size", default_val=DEFAULT_CELL_SIZE)
                    self._add_to_current_stage = cb_builder(
                        label="Add to Current Stage",
                        default_val=DEFAULT_ADD_TO_CURRENT_STAGE,
                        tooltip=(
                            "Add the generated terrain to the current stage. "
                            "When disabled, Generate replaces the stage and discards all existing content."
                        ),
                    )
                    self._overwrite_existing = cb_builder(
                        label="Overwrite Existing",
                        default_val=DEFAULT_OVERWRITE_EXISTING,
                        tooltip=(
                            "Replace whatever is already at the generated paths. When disabled "
                            "nothing on the stage is touched: each Generate builds a new terrain "
                            "at the next free path, and the ground plane and light are skipped if "
                            "theirs are taken."
                        ),
                    )
                    self._create_ground_plane = cb_builder(
                        label="Create Ground Plane",
                        default_val=DEFAULT_CREATE_GROUND_PLANE,
                        tooltip=(
                            "Add a ground plane sized to the generated terrain. "
                            "Disable it for a scene that already has a floor."
                        ),
                    )
                    self._create_lighting = cb_builder(
                        label="Create Lighting",
                        default_val=DEFAULT_CREATE_LIGHTING,
                        tooltip=(
                            "Add a distant light for the generated terrain. "
                            "Disable it to keep the lighting the scene already has."
                        ),
                    )
                    self._load_button = btn_builder(label="Image", text="Load", on_clicked_fn=self._load_image_dialog)
                    self._generate_button = btn_builder(
                        label="Heightmap", text="Generate", on_clicked_fn=self._generate
                    )
                    self._generate_button.enabled = False

        # Initialize image-related attributes
        self._image: Image.Image | None = None
        self._image_width: int = 0
        self._image_height: int = 0
        self._rgb_byte_provider: ui.ByteImageProvider | None = None
        self._importer: HeightmapImporter | None = None

    def _menu_callback(self) -> None:
        """Toggle the visibility of the extension window."""
        self._window.visible = not self._window.visible

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is shut down."""
        # Cancelled first, before anything it holds a reference to is torn down. Generation
        # awaits a frame before it touches the stage, so a task still in flight would come
        # back after the extension is gone and write terrain into the stage on its way out.
        # Its inputs were all read up front, so nothing it needs is missing and nothing
        # would stop it.
        if self._generation_task is not None:
            self._generation_task.cancel()
            self._generation_task = None

        remove_menu_items(self._menu_items, "Tools")

        # Clean up image resources
        if self._image is not None:
            try:
                self._image.close()
            except Exception:
                pass
            self._image = None

        # Clean up byte provider
        self._rgb_byte_provider = None

        # Clean up windows
        if self._visualize_window is not None:
            try:
                self._visualize_window.destroy()
            except Exception as e:
                carb.log_warn(f"Failed to destroy visualization window: {e}")
            finally:
                self._visualize_window = None

        if self._filepicker is not None:
            try:
                self._filepicker.hide()
            except Exception:
                pass
            self._filepicker = None

        self._window = None

        # Clear UI component references
        self._load_button = None
        self._generate_button = None
        self._cell_size = None
        self._add_to_current_stage = None
        self._overwrite_existing = None
        self._create_ground_plane = None
        self._create_lighting = None
        self._importer = None

        gc.collect()

    def _load_image_dialog(self) -> None:
        """Open a file picker dialog to select a PNG image file.

        The dialog filters to show only PNG files that can be used as heightmaps
        or occupancy maps for heightmap generation.
        """
        from omni.kit.widget.filebrowser import FileBrowserItem
        from omni.kit.window.filepicker import FilePickerDialog

        def _on_filter_png_files(item: FileBrowserItem) -> bool:
            """Filter file browser items to show only PNG files and folders.

            Args:
                item: The file browser item to filter.

            Returns:
                True if the item should be shown, False otherwise.
            """
            if not item or item.is_folder:
                return True
            # Show only files with listed extensions (case-insensitive)
            return os.path.splitext(item.path)[1].lower() == ".png"

        self._filepicker = FilePickerDialog(
            "Load .png image",
            allow_multi_selection=False,
            apply_button_label="Load",
            click_apply_handler=self._load_image,
            item_filter_options=[".png Files (*.png, *.PNG)"],
            item_filter_fn=_on_filter_png_files,
        )
        self._filepicker.show()

    def _load_image(self, filename: str, folder_path: str) -> None:
        """Load a PNG image file and display it in a visualization window.

        Args:
            filename: The filename of the image to load.
            folder_path: The folder path containing the image file.
        """
        # Use os.path.join for proper cross-platform path construction
        image_path = os.path.join(folder_path, filename)
        self._filepicker.hide()

        # Validate path
        if not image_path or not filename:
            carb.log_warn("File path can't be empty.")
            self._generate_button.enabled = False
            return

        if not os.path.exists(image_path):
            carb.log_error(f"Image file does not exist: {image_path}")
            self._generate_button.enabled = False
            return

        # Clean up previous visualization window if it exists
        if self._visualize_window is not None:
            try:
                self._visualize_window.destroy()
            except Exception as e:
                carb.log_warn(f"Failed to destroy previous visualization window: {e}")
            finally:
                self._visualize_window = None

        # Load image with error handling
        try:
            carb.log_info(f"Opening file at {image_path}")
            self._image = Image.open(image_path).convert("RGBA")
            self._image_width, self._image_height = self._image.size
        except FileNotFoundError:
            carb.log_error(f"Image file not found: {image_path}")
            self._image = None
            self._generate_button.enabled = False
            return
        except (IOError, OSError) as e:
            carb.log_error(f"Failed to read image file: {e}")
            self._image = None
            self._generate_button.enabled = False
            return
        except Image.UnidentifiedImageError:
            carb.log_error(f"File is not a valid image: {image_path}")
            self._image = None
            self._generate_button.enabled = False
            return
        except Exception as e:
            carb.log_error(f"Unexpected error loading image: {e}")
            self._image = None
            self._generate_button.enabled = False
            return

        # Validate image dimensions
        if self._image_width == 0 or self._image_height == 0:
            carb.log_error(f"Image has invalid dimensions: {self._image_width}x{self._image_height}")
            self._image = None
            self._generate_button.enabled = False
            return

        if self._image_width > MAX_IMAGE_DIMENSION or self._image_height > MAX_IMAGE_DIMENSION:
            carb.log_error(
                f"Image too large: {self._image_width}x{self._image_height}. "
                f"Maximum dimension is {MAX_IMAGE_DIMENSION}."
            )
            self._image = None
            self._generate_button.enabled = False
            return

        carb.log_info(f"Image Size: {self._image_width} x {self._image_height}")

        # Create visualization window to preview the loaded image
        self._visualize_window = ui.Window("Image Preview", width=PREVIEW_WINDOW_WIDTH, height=PREVIEW_WINDOW_HEIGHT)
        with self._visualize_window.frame:
            self._rgb_byte_provider = ui.ByteImageProvider()
            self._rgb_byte_provider.set_bytes_data(
                list(self._image.tobytes("raw", "RGBA")), [self._image_width, self._image_height]
            )
            with ui.VStack():
                ui.ImageWithProvider(self._rgb_byte_provider)

        self._generate_button.enabled = True

    def _generate(self) -> None:
        """Trigger the asynchronous heightmap generation process.

        Validates that an image is loaded and cell size is valid before proceeding.
        """
        if self._generation_task is not None and not self._generation_task.done():
            carb.log_warn("Heightmap generation is already running. Wait for it to finish.")
            return

        # Validate that an image is loaded
        if self._image is None:
            carb.log_error("No image loaded. Please load an image first.")
            return

        # Validate cell size
        cell_scale = self._cell_size.get_value_as_float()
        if cell_scale <= 0:
            carb.log_error(f"Cell size must be positive, got {cell_scale}")
            return

        if cell_scale < MIN_CELL_SIZE:
            carb.log_warn(
                f"Cell size {cell_scale} is very small (min recommended: {MIN_CELL_SIZE}). "
                "This may result in many instances and performance issues."
            )

        if cell_scale > MAX_CELL_SIZE:
            carb.log_warn(
                f"Cell size {cell_scale} is very large (max recommended: {MAX_CELL_SIZE}). "
                "This may result in sparse cell placement."
            )

        # Every input is read here, while the widgets are known to exist. Generation waits a
        # frame before it touches the stage, and by then the panel may have been shut down.
        self._generation_task = asyncio.ensure_future(
            self._create_heightmap(
                image=self._image,
                cell_scale=cell_scale,
                add_to_current_stage=self._add_to_current_stage.get_value_as_bool(),
                overwrite_existing=self._overwrite_existing.get_value_as_bool(),
                create_ground_plane=self._create_ground_plane.get_value_as_bool(),
                create_lighting=self._create_lighting.get_value_as_bool(),
            )
        )

    def _warn_on_generated_path_collisions(
        self, stage: Usd.Stage, create_ground_plane: bool, create_lighting: bool
    ) -> None:
        """Warn about existing prims that generation is about to replace.

        `Add to Current Stage` keeps the stage, but the importer writes to its own fixed
        paths and removes whatever already occupies them, subtrees included. Only the paths
        this run will actually write to are reported, and only when they hold something other
        than a previous run's output — otherwise every regenerate would warn about terrain the
        importer authored itself.

        Only called when `Overwrite Existing` is on, so this records a replacement the user
        asked for rather than advising them to avoid one. Naming the option is what makes the
        record useful: it is the thing to turn off to have the terrain step aside instead.

        Args:
            stage: USD stage the terrain will be generated into.
            create_ground_plane: Whether this run will write the ground plane.
            create_lighting: Whether this run will write the distant light.
        """
        paths = generated_paths(create_ground_plane, create_lighting)
        collisions = [
            path for path in paths if stage.GetPrimAtPath(path).IsValid() and not is_importer_generated(stage, path)
        ]
        if not collisions:
            return

        # Stated as intent rather than as an accomplished fact. Generation can still refuse
        # a path it cannot clear, and an empty occupancy map skips the ground plane, so the
        # replacement announced here does not always happen.
        carb.log_warn(
            f"Generating terrain will replace existing content at {', '.join(collisions)}, "
            "which cannot be undone. Turn off 'Overwrite Existing' to build a new terrain "
            "alongside it instead of over it."
        )

    def _warn_on_stage_up_axis_mismatch(self, stage: Usd.Stage) -> None:
        """Warn when terrain will not stand upright in the stage it is added to.

        Cell Size is converted into the stage's units, so a non-metric stage needs no
        caveat. Orientation is not converted: terrain is always built in the XY plane.

        Args:
            stage: USD stage the terrain will be generated into.
        """
        up_axis = UsdGeom.GetStageUpAxis(stage)
        if up_axis == UsdGeom.Tokens.z:
            return

        carb.log_warn(
            f"Stage is {up_axis}-up but terrain is generated in the XY plane, so it will lie "
            f"against the wrong axis, and the generated ground plane will face {up_axis} and so "
            "sit perpendicular to it. Rotate the generated terrain to suit."
        )

    async def _create_heightmap(
        self,
        image: Image.Image,
        cell_scale: float,
        add_to_current_stage: bool,
        overwrite_existing: bool,
        create_ground_plane: bool,
        create_lighting: bool,
    ) -> None:
        """Create a 3D heightmap terrain from the loaded heightmap image.

        Terrain is added to the current stage unless `Add to Current Stage` is
        disabled, in which case the stage is replaced with a new one first. Adding to the
        current stage leaves the stage's own metadata alone, and replaces what is at the
        generated paths only when `Overwrite Existing` is on, which is what the warning
        reports. Generation itself is delegated to the HeightmapImporter class.

        Every option is taken as an argument rather than read from a widget, because this
        awaits before it touches the stage and the panel can be shut down or retuned in the
        meantime.

        Args:
            image: Heightmap image to generate terrain from.
            cell_scale: Size of each cell in meters.
            add_to_current_stage: Whether to keep the current stage instead of replacing it.
            overwrite_existing: Whether to replace whatever is at the generated paths,
                rather than leaving the stage untouched and generating alongside it.
            create_ground_plane: Whether to add a ground plane under the terrain.
            create_lighting: Whether to add a distant light for the terrain.
        """
        if not add_to_current_stage:
            carb.log_warn(
                "Replacing the current stage with a new one. All existing stage content will be "
                "discarded, including unsaved changes. Enable 'Add to Current Stage' to generate "
                "terrain alongside the existing content."
            )
            await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Get stage
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            carb.log_error("Failed to get USD stage. Cannot create heightmap.")
            return

        if add_to_current_stage:
            # Only worth reporting when the run is actually allowed to replace that content.
            # Otherwise the terrain is about to be built beside it, and announcing a
            # replacement that is not going to happen would misdescribe the run.
            if overwrite_existing:
                self._warn_on_generated_path_collisions(stage, create_ground_plane, create_lighting)
            self._warn_on_stage_up_axis_mismatch(stage)

        # Create importer and generate heightmap
        try:
            self._importer = HeightmapImporter(stage)
            num_cells = self._importer.create_heightmap(
                image,
                cell_scale,
                create_ground_plane=create_ground_plane,
                create_lighting=create_lighting,
                configure_stage_metadata=not add_to_current_stage,
                overwrite_existing=overwrite_existing,
            )
            carb.log_info(f"Successfully created heightmap with {num_cells} cells!")
        except ValueError as e:
            carb.log_error(f"Invalid parameters for heightmap generation: {e}")
        except RuntimeError as e:
            carb.log_error(f"Failed to create heightmap: {e}")
        except Exception as e:
            carb.log_error(f"Unexpected error during heightmap generation: {e}")
