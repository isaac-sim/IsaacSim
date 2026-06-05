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

"""Heightmap importer utilities."""

from typing import Optional

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.usd
from isaacsim.core.experimental.objects import GroundPlane
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

# Heightmap generation constants
DEFAULT_CUBE_HEIGHT = 2.0
DEFAULT_CUBE_COLOR = (0.5, 0.5, 0.5)
OCCUPIED_PIXEL_THRESHOLD = 127  # Pixels darker than this value are considered occupied
GROUND_PLANE_MARGIN = 1.0
DEFAULT_LIGHT_INTENSITY = 2000

# Image processing constants
LARGE_IMAGE_THRESHOLD = 1000000  # Show progress message for images > 1MP

# USD Scene paths
WORLD_PATH = "/World"
GROUND_PLANE_PATH = "/World/groundPlane"
LIGHT_PATH = "/World/defaultLight"
OCCUPANCY_MAP_PATH = "/World/occupancyMap"
INSTANCES_PATH = "/World/occupancyMap/occupiedInstances"
CUBE_PROTOTYPE_PATH = "/World/occupancyMap/occupiedInstances/occupiedCube"


class HeightmapImporter:
    """Converts heightmap/occupancy map images into 3D terrain.

    This class handles the conversion of 2D heightmap images into 3D terrain
    in USD stages using point instancers for efficient rendering.

    Args:
        stage: USD stage to create heightmap in. If None, uses the current stage.
    """

    def __init__(self, stage: Optional[any] = None) -> None:
        self._stage = stage

    def create_heightmap(
        self, image: Image.Image, cell_scale: float, create_ground_plane: bool = True, create_lighting: bool = True
    ) -> int:
        """Create a 3D heightmap terrain from a heightmap image.

        This method generates heightmap terrain using point instancers for efficient
        rendering. Dark pixels in the image (below threshold) are converted to
        cube instances in 3D space.

        Args:
            image: PIL Image representing the heightmap/occupancy map.
            cell_scale: The scale of each cell in meters.
            create_ground_plane: Whether to create a ground plane for the heightmap.
            create_lighting: Whether to create lighting for the scene.

        Returns:
            The number of cells created in the heightmap.

        Raises:
            ValueError: If image is None or cell_scale is invalid.
            RuntimeError: If USD stage is not available.

        Example:

        .. code-block:: python

            >>> from PIL import Image
            >>> import omni.usd
            >>>
            >>> importer = HeightmapImporter()
            >>> image = Image.open("heightmap.png")
            >>>
            >>> num_cells = importer.create_heightmap(
            ...     image,
            ...     cell_scale=0.05,
            ... )
            ...
            >>> print(f"Created {num_cells} cells")
            Created 1234 cells
        """
        # Validate inputs
        if image is None:
            raise ValueError("Image cannot be None")

        if not isinstance(image, Image.Image):
            raise ValueError(f"Image must be a PIL Image, got {type(image).__name__}")

        if cell_scale <= 0:
            raise ValueError(f"Cell scale must be positive, got {cell_scale}")

        # Get or validate stage
        if self._stage is None:
            self._stage = omni.usd.get_context().get_stage()

        if self._stage is None:
            raise RuntimeError("No USD stage available")

        image_width, image_height = image.size

        # Provide feedback for large images
        total_pixels = image_width * image_height
        if total_pixels > LARGE_IMAGE_THRESHOLD:
            carb.log_info(f"Processing large image ({total_pixels:,} pixels), this may take a moment...")

        # Setup stage properties
        carb.log_info("Setting up stage properties...")
        self._setup_stage_properties()

        # Create lighting if requested
        if create_lighting:
            carb.log_info("Setting up lighting...")
            self._create_lighting()

        # Generate heightmap instances
        carb.log_info("Generating heightmap instances...")
        num_cells = self._create_heightmap_instances(image, cell_scale)

        # Create ground plane sized to the heightmap's actual XY bounds (after instances exist)
        if create_ground_plane:
            carb.log_info("Creating ground plane...")
            self._create_ground_plane()

        carb.log_info(f"Heightmap generation complete! Created {num_cells} cells.")

        return num_cells

    def _setup_stage_properties(self) -> None:
        """Configure basic stage properties like units and up-axis."""
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        stage_utils.set_stage_up_axis("Z")
        world_prim = self._stage.GetPrimAtPath(WORLD_PATH)
        if not world_prim.IsValid():
            UsdGeom.Xform.Define(self._stage, WORLD_PATH)
            world_prim = self._stage.GetPrimAtPath(WORLD_PATH)
        self._stage.SetDefaultPrim(world_prim)

    def _create_ground_plane(self) -> None:
        """Create a ground plane sized to fit the heightmap's XY bounds.

        Must be called after :py:meth:`_create_heightmap_instances` so the bounding box
        computed from the occupancy map prim reflects the generated geometry.
        """
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bounds = bbox_cache.ComputeWorldBound(self._stage.GetPrimAtPath(OCCUPANCY_MAP_PATH)).GetRange()
        if bounds.IsEmpty():
            carb.log_warn("Occupancy map has empty bounds; skipping ground plane creation.")
            return
        min_pt, max_pt = bounds.GetMin(), bounds.GetMax()

        # Use the longer XY dimension so a square plane fully covers the heightmap without stretching.
        size = max(max_pt[0] - min_pt[0], max_pt[1] - min_pt[1]) + 2 * GROUND_PLANE_MARGIN
        center_x = (min_pt[0] + max_pt[0]) / 2.0
        center_y = (min_pt[1] + max_pt[1]) / 2.0

        GroundPlane(
            GROUND_PLANE_PATH,
            sizes=size,
            positions=[[center_x, center_y, 0.0]],
            colors=[1.0, 1.0, 1.0],
        )

    def _create_lighting(self) -> None:
        """Create a distant light for the scene."""
        light_prim = UsdLux.DistantLight.Define(self._stage, Sdf.Path(LIGHT_PATH))
        light_prim.CreateIntensityAttr(DEFAULT_LIGHT_INTENSITY)

    def _create_heightmap_instances(self, image: Image.Image, cell_scale: float) -> int:
        """Create the point instancer with cube instances for occupied cells.

        Args:
            image: PIL Image representing the heightmap/occupancy map.
            cell_scale: The scale of each cell in meters.

        Returns:
            The number of cells created.
        """
        cell_offset = cell_scale / 2.0

        self._create_parent_transform(cell_offset)
        point_instancer = self._create_point_instancer()
        cube_prototype = self._create_cube_prototype(cell_scale)
        occupied_positions = self._generate_occupied_positions(image, cell_scale, cell_offset)

        self._configure_point_instancer(point_instancer, cube_prototype, occupied_positions)

        return len(occupied_positions)

    def _create_parent_transform(self, cell_offset: float) -> UsdGeom.Xform:
        """Create the parent transform for the occupancy map.

        Args:
            cell_offset: The offset to center cells at their grid positions.

        Returns:
            The USD Xform primitive for the parent transform.
        """
        if self._stage.GetPrimAtPath(OCCUPANCY_MAP_PATH):
            self._stage.RemovePrim(OCCUPANCY_MAP_PATH)
        parent_xform = UsdGeom.Xform(self._stage.DefinePrim(OCCUPANCY_MAP_PATH, "Xform"))
        parent_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, cell_offset))
        return parent_xform

    def _create_point_instancer(self) -> UsdGeom.PointInstancer:
        """Create the point instancer that will contain all cube instances.

        Returns:
            The USD PointInstancer primitive.
        """
        if self._stage.GetPrimAtPath(INSTANCES_PATH):
            self._stage.RemovePrim(INSTANCES_PATH)
        point_instancer = UsdGeom.PointInstancer(self._stage.DefinePrim(INSTANCES_PATH, "PointInstancer"))
        point_instancer.AddScaleOp().Set(Gf.Vec3f(1))
        point_instancer.AddTranslateOp().Set(Gf.Vec3f(0, 0, (DEFAULT_CUBE_HEIGHT / 2.0)))
        return point_instancer

    def _configure_point_instancer(
        self, point_instancer: UsdGeom.PointInstancer, cube_prototype: UsdGeom.Cube, positions: list[Gf.Vec3f]
    ) -> None:
        """Configure the point instancer with positions and prototype.

        Args:
            point_instancer: The point instancer to configure.
            cube_prototype: The cube prototype to instance.
            positions: List of positions for each instance.
        """
        if not positions:
            carb.log_warn("No occupied positions found. Heightmap will be empty.")
            return

        point_instancer.CreatePositionsAttr().Set(positions)
        point_instancer.CreatePrototypesRel().SetTargets([cube_prototype.GetPath()])
        point_instancer.CreateProtoIndicesAttr().Set([0] * len(positions))

    def _create_cube_prototype(self, cell_scale: float) -> UsdGeom.Cube:
        """Create the cube prototype that will be instanced for each occupied cell.

        Args:
            cell_scale: The scale of each cell in meters.

        Returns:
            The USD cube primitive to be used as a prototype.
        """
        if self._stage.GetPrimAtPath(CUBE_PROTOTYPE_PATH):
            self._stage.RemovePrim(CUBE_PROTOTYPE_PATH)

        cube = UsdGeom.Cube(self._stage.DefinePrim(CUBE_PROTOTYPE_PATH, "Cube"))
        cube.AddScaleOp().Set(Gf.Vec3f(cell_scale, cell_scale, DEFAULT_CUBE_HEIGHT))
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorPrimvar().Set([DEFAULT_CUBE_COLOR])
        UsdPhysics.CollisionAPI.Apply(self._stage.GetPrimAtPath(CUBE_PROTOTYPE_PATH))

        return cube

    def _generate_occupied_positions(self, image: Image.Image, cell_scale: float, cell_offset: float) -> list[Gf.Vec3f]:
        """Generate 3D positions for all occupied cells in the image.

        Uses NumPy for efficient vectorized processing of large images.

        Args:
            image: PIL Image representing the heightmap/occupancy map.
            cell_scale: The scale of each cell in meters.
            cell_offset: The offset to center cells at their grid positions.

        Returns:
            List of Gf.Vec3f positions for each occupied cell.
        """
        try:
            # Convert PIL Image to numpy array for fast processing
            img_array = np.array(image)

            # Handle 2D grayscale (mode 'L', 'I', 'F') and 3D+ (RGB/RGBA) arrays
            if img_array.ndim == 2:
                channel = img_array
            elif img_array.ndim >= 3 and img_array.shape[2] >= 1:
                channel = img_array[:, :, 0]
            else:
                raise ValueError(
                    f"Image has invalid shape: {img_array.shape}. Expected 2D grayscale or 3D+ with channels."
                )

            # Find occupied pixels (below threshold)
            occupied_mask = channel < OCCUPIED_PIXEL_THRESHOLD

            # Get coordinates of occupied pixels (y, x order from numpy)
            y_coords, x_coords = np.where(occupied_mask)

            # Vectorized position calculation
            world_x = (x_coords * cell_scale) + cell_offset
            world_y = -((y_coords * cell_scale) + cell_offset)

            # Create list of Vec3f positions
            occupied_positions = [Gf.Vec3f(float(x), float(y), 0.0) for x, y in zip(world_x, world_y)]

            return occupied_positions
        except IndexError as e:
            raise ValueError(f"Failed to access image array channels: {e}") from e
        except Exception as e:
            if isinstance(e, ValueError):
                raise
            raise ValueError(f"Failed to generate occupied positions: {e}") from e
