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

import asyncio
import gc
import os
import weakref
from typing import Optional

import carb
import omni.ext
import omni.kit.commands
import omni.ui as ui
import omni.usd
from isaacsim.asset.importer.heightmap.importer import HeightmapImporter
from isaacsim.gui.components.menu import make_menu_item_description
from isaacsim.gui.components.ui_utils import btn_builder, float_builder
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from PIL import Image

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
        with self._window.frame:
            with ui.HStack(spacing=10):
                with ui.VStack(spacing=5, height=0):
                    self._cell_size = float_builder(label="Cell Size", default_val=DEFAULT_CELL_SIZE)
                    self._load_button = btn_builder(label="Image", text="Load", on_clicked_fn=self._load_image_dialog)
                    self._generate_button = btn_builder(
                        label="Heightmap", text="Generate", on_clicked_fn=self._generate
                    )
                    self._generate_button.enabled = False

        # Initialize image-related attributes
        self._image: Optional[Image.Image] = None
        self._image_width: int = 0
        self._image_height: int = 0
        self._rgb_byte_provider: Optional[ui.ByteImageProvider] = None
        self._importer: Optional[HeightmapImporter] = None

    def _menu_callback(self) -> None:
        """Toggle the visibility of the extension window."""
        self._window.visible = not self._window.visible

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is shut down."""
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

        asyncio.ensure_future(self._create_heightmap())

    async def _create_heightmap(self) -> None:
        """Create a 3D heightmap terrain from the loaded heightmap image.

        This method creates a new USD stage and delegates heightmap generation
        to the HeightmapImporter class.
        """
        # Re-validate image still exists (async race condition protection)
        if self._image is None:
            carb.log_error("Image was unloaded before heightmap generation completed.")
            return

        # Wait for new stage before creating objects for heightmap
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Get stage
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            carb.log_error("Failed to get USD stage. Cannot create heightmap.")
            return

        cell_scale = self._cell_size.get_value_as_float()

        # Create importer and generate heightmap
        try:
            self._importer = HeightmapImporter(stage)
            num_cells = self._importer.create_heightmap(
                self._image, cell_scale, create_ground_plane=True, create_lighting=True
            )
            carb.log_info(f"Successfully created heightmap with {num_cells} cells!")
        except ValueError as e:
            carb.log_error(f"Invalid parameters for heightmap generation: {e}")
        except RuntimeError as e:
            carb.log_error(f"Failed to create heightmap: {e}")
        except Exception as e:
            carb.log_error(f"Unexpected error during heightmap generation: {e}")
