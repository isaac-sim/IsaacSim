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

"""Unit tests for HeightmapImporter class."""

from __future__ import annotations

import asyncio
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, Mock, patch

import omni.kit.test
import omni.usd
from isaacsim.asset.importer.heightmap.importer import (
    GENERATED_MARKER_ATTR,
    GROUND_PLANE_MARGIN,
    GROUND_PLANE_PATH,
    LIGHT_PATH,
    OCCUPANCY_MAP_PATH,
    OCCUPIED_PIXEL_THRESHOLD,
    WORLD_PATH,
    HeightmapImporter,
    cube_prototype_path,
    instances_path,
    is_importer_generated,
)
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

if TYPE_CHECKING:
    # Only for annotations. Importing it at runtime here would load the extension module
    # before the test cases that construct it choose to, which each one does itself.
    from isaacsim.asset.importer.heightmap.extension import Extension

PRESERVED_PRIM_PATH = "/World/PreservedCube"
COLLIDING_PRIM_PATH = f"{OCCUPANCY_MAP_PATH}/UserAuthoredCube"
# The path a run steps aside to when something else holds the base one. Spelled out rather
# than built from the importer's own suffix logic, so a change to that logic is caught here
# instead of being mirrored into the expectation.
SUFFIXED_TERRAIN_PATH = "/World/occupancyMap_01"


def world_bound(stage: Usd.Stage, path: str) -> Gf.Range3d:
    """Return the world-space range of the prim at ``path``.

    All purposes are included because the ground plane's geometry is not authored with the
    default purpose, so restricting to it collapses the plane to a point.

    Args:
        stage: Stage to measure on.
        path: Prim path to measure.

    Returns:
        World-space range of the prim, empty if it has no extent.
    """
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.proxy,
            UsdGeom.Tokens.guide,
            UsdGeom.Tokens.render,
        ],
    )
    return bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(path)).GetRange()


def world_child_names(stage: Usd.Stage) -> set[str]:
    """Return the names of the prims directly under ``/World``.

    Everything the importer generates is a direct child of ``/World``, so comparing this
    set is how a test says no extra prim appeared anywhere a duplicate could have landed.

    Args:
        stage: Stage to inspect.

    Returns:
        Names of the direct children of ``/World``.
    """
    return {child.GetName() for child in stage.GetPrimAtPath(WORLD_PATH).GetChildren()}


async def press_generate(extension: Extension) -> None:
    """Drive the Generate button and wait for the generation it schedules.

    Tests go through the button rather than calling the coroutine directly, so that the
    input snapshotting and the re-entrancy guard are exercised too.

    Args:
        extension: Heightmap importer extension to drive.
    """
    extension._generate()
    if extension._generation_task is not None:
        await extension._generation_task


class TestHeightmapImporter(omni.kit.test.AsyncTestCase):
    """Test suite for HeightmapImporter class."""

    def setUp(self) -> None:
        """Set up test fixtures."""
        self.mock_stage = MagicMock()
        self.importer = HeightmapImporter(self.mock_stage)

    def test_initialization_with_stage(self) -> None:
        """Test initialization with a provided stage."""
        importer = HeightmapImporter(self.mock_stage)
        self.assertEqual(importer._stage, self.mock_stage)

    def test_initialization_without_stage(self) -> None:
        """Test initialization without a provided stage."""
        importer = HeightmapImporter()
        self.assertIsNone(importer._stage)

    @patch("isaacsim.asset.importer.heightmap.importer.omni.usd.get_context")
    def test_create_heightmap_with_none_image(self, mock_get_context: object) -> None:
        """Test that create_heightmap raises ValueError when image is None.

        Args:
            mock_get_context: Mock for omni.usd.get_context.
        """
        with self.assertRaises(ValueError) as context:
            self.importer.create_heightmap(None, 0.05)
        self.assertIn("Image cannot be None", str(context.exception))

    def test_create_heightmap_with_invalid_cell_scale(self) -> None:
        """Test that create_heightmap raises ValueError with invalid cell scale."""
        # Create a simple test image
        test_image = Image.new("RGBA", (10, 10), color=(255, 255, 255, 255))

        with self.assertRaises(ValueError) as context:
            self.importer.create_heightmap(test_image, 0.0)
        self.assertIn("Cell scale must be positive", str(context.exception))

        with self.assertRaises(ValueError) as context:
            self.importer.create_heightmap(test_image, -0.5)
        self.assertIn("Cell scale must be positive", str(context.exception))

    def test_create_heightmap_rejects_non_pil_images_before_stage_writes(self) -> None:
        """Test invalid image objects raise ValueError without modifying the stage."""

        class FakeImage:
            size = (64, 64)

        invalid_images = ["/tmp/heightmap.png", 42, b"\x89PNG", FakeImage()]

        for invalid_image in invalid_images:
            with self.subTest(invalid_image=invalid_image):
                stage = MagicMock()
                importer = HeightmapImporter(stage)

                with self.assertRaises(ValueError) as context:
                    importer.create_heightmap(invalid_image, 0.05)

                self.assertIn("PIL Image", str(context.exception))
                stage.GetPrimAtPath.assert_not_called()
                stage.DefinePrim.assert_not_called()
                stage.RemovePrim.assert_not_called()

    @patch("isaacsim.asset.importer.heightmap.importer.omni.usd.get_context")
    def test_create_heightmap_no_stage_available(self, mock_get_context: object) -> None:
        """Test that create_heightmap raises RuntimeError when stage is not available.

        Args:
            mock_get_context: Mock for omni.usd.get_context.
        """
        # Setup importer without a stage
        importer = HeightmapImporter(None)
        mock_context = Mock()
        mock_context.get_stage.return_value = None
        mock_get_context.return_value = mock_context

        test_image = Image.new("RGBA", (10, 10), color=(255, 255, 255, 255))

        with self.assertRaises(RuntimeError) as context:
            importer.create_heightmap(test_image, 0.05)
        self.assertIn("No USD stage available", str(context.exception))

    def test_generate_occupied_positions_with_simple_image(self) -> None:
        """Test position generation with a simple test image."""
        # Create a 3x3 image with specific pattern
        test_image = Image.new("RGBA", (3, 3), color=(255, 255, 255, 255))
        pixels = test_image.load()
        # Make corners black (below threshold)
        pixels[0, 0] = (0, 0, 0, 255)  # Top-left
        pixels[2, 0] = (0, 0, 0, 255)  # Top-right
        pixels[0, 2] = (0, 0, 0, 255)  # Bottom-left
        pixels[2, 2] = (0, 0, 0, 255)  # Bottom-right

        cell_scale = 1.0
        cell_offset = 0.5
        positions = self.importer._generate_occupied_positions(test_image, cell_scale, cell_offset)

        # Should have 4 positions
        self.assertEqual(len(positions), 4)

        # Verify positions are Gf.Vec3f
        for pos in positions:
            self.assertIsInstance(pos, Gf.Vec3f)

        # Verify positions are calculated correctly
        # Expected positions for the four corners:
        # (0, 0) -> x=0.5, y=-0.5
        # (2, 0) -> x=2.5, y=-0.5
        # (0, 2) -> x=0.5, y=-2.5
        # (2, 2) -> x=2.5, y=-2.5
        expected_positions = {
            (0.5, -0.5),
            (2.5, -0.5),
            (0.5, -2.5),
            (2.5, -2.5),
        }

        actual_positions = {(float(pos[0]), float(pos[1])) for pos in positions}

        # Compare sets (order doesn't matter)
        for exp_pos in expected_positions:
            # Find matching position within tolerance
            found = any(
                abs(exp_pos[0] - act_pos[0]) < 1e-5 and abs(exp_pos[1] - act_pos[1]) < 1e-5
                for act_pos in actual_positions
            )
            self.assertTrue(found, f"Expected position {exp_pos} not found in actual positions")

    def test_generate_occupied_positions_with_threshold(self) -> None:
        """Test that only pixels below threshold are converted to positions."""
        # Create a 2x2 image
        test_image = Image.new("RGBA", (2, 2), color=(255, 255, 255, 255))
        pixels = test_image.load()
        # Below threshold
        pixels[0, 0] = (OCCUPIED_PIXEL_THRESHOLD - 1, 0, 0, 255)
        # At threshold (should not be included)
        pixels[1, 0] = (OCCUPIED_PIXEL_THRESHOLD, 0, 0, 255)
        # Above threshold (should not be included)
        pixels[0, 1] = (OCCUPIED_PIXEL_THRESHOLD + 1, 0, 0, 255)
        # Well below threshold
        pixels[1, 1] = (0, 0, 0, 255)

        positions = self.importer._generate_occupied_positions(test_image, 1.0, 0.5)

        # Only 2 positions should be created (below threshold)
        self.assertEqual(len(positions), 2)

    def test_generate_occupied_positions_with_all_white_image(self) -> None:
        """Test that an all-white image produces no positions."""
        test_image = Image.new("RGBA", (5, 5), color=(255, 255, 255, 255))
        positions = self.importer._generate_occupied_positions(test_image, 1.0, 0.5)
        self.assertEqual(len(positions), 0)

    def test_generate_occupied_positions_with_all_black_image(self) -> None:
        """Test that an all-black image produces positions for all pixels."""
        test_image = Image.new("RGBA", (5, 5), color=(0, 0, 0, 255))
        positions = self.importer._generate_occupied_positions(test_image, 1.0, 0.5)
        # Should create 25 positions (5x5)
        self.assertEqual(len(positions), 25)

    def test_generate_occupied_positions_with_different_scales(self) -> None:
        """Test position generation with different cell scales."""
        test_image = Image.new("RGBA", (2, 2), color=(0, 0, 0, 255))

        # Test with small scale
        positions_small = self.importer._generate_occupied_positions(test_image, 0.1, 0.05)
        self.assertEqual(len(positions_small), 4)
        self.assertAlmostEqual(positions_small[0][0], 0.05, places=5)

        # Test with large scale
        positions_large = self.importer._generate_occupied_positions(test_image, 10.0, 5.0)
        self.assertEqual(len(positions_large), 4)
        self.assertAlmostEqual(positions_large[0][0], 5.0, places=5)

    def test_generate_occupied_positions_z_coordinate(self) -> None:
        """Test that all positions have z-coordinate of 0.0."""
        test_image = Image.new("RGBA", (3, 3), color=(0, 0, 0, 255))
        positions = self.importer._generate_occupied_positions(test_image, 1.0, 0.5)

        for pos in positions:
            self.assertEqual(pos[2], 0.0)

    def test_generate_occupied_positions_with_grayscale_image(self) -> None:
        """A single-channel image is valid input and every dark pixel still counts."""
        test_image = Image.new("L", (5, 5), color=0)

        positions = self.importer._generate_occupied_positions(test_image, 1.0, 0.5)

        # All pixels are 0 (below threshold 127), so all 25 cells are occupied
        self.assertEqual(len(positions), 25)

    def test_generate_occupied_positions_raises_for_invalid_image_shape(self) -> None:
        """Test invalid image data is reported to callers instead of becoming an empty heightmap."""

        class InvalidImage:
            pass

        with self.assertRaises(ValueError) as context:
            self.importer._generate_occupied_positions(InvalidImage(), 1.0, 0.5)

        self.assertIn("invalid shape", str(context.exception))

    @patch("isaacsim.asset.importer.heightmap.importer.np.array")
    def test_generate_occupied_positions_propagates_image_processing_errors(self, mock_np_array: object) -> None:
        """Test unexpected image processing failures are propagated to the caller.

        Args:
            mock_np_array: Mock for np.array.
        """
        mock_np_array.side_effect = RuntimeError("boom")
        test_image = Image.new("RGBA", (2, 2), color=(255, 255, 255, 255))

        with self.assertRaises(ValueError) as context:
            self.importer._generate_occupied_positions(test_image, 1.0, 0.5)

        self.assertIn("Failed to generate occupied positions", str(context.exception))

    def _stub_bbox_cache(self, mock_usd_geom: object, min_pt: tuple, max_pt: tuple) -> None:
        """Configure ``UsdGeom.BBoxCache`` to return the given world-space extents.

        Args:
            mock_usd_geom: Mock for the UsdGeom module.
            min_pt: Minimum bbox point.
            max_pt: Maximum bbox point.
        """
        mock_bbox_range = MagicMock()
        mock_bbox_range.IsEmpty.return_value = False
        mock_bbox_range.GetMin.return_value = Gf.Vec3d(*min_pt)
        mock_bbox_range.GetMax.return_value = Gf.Vec3d(*max_pt)
        mock_world_bound = MagicMock()
        mock_world_bound.GetRange.return_value = mock_bbox_range
        mock_bbox_cache = MagicMock()
        mock_bbox_cache.ComputeWorldBound.return_value = mock_world_bound
        mock_usd_geom.BBoxCache.return_value = mock_bbox_cache

    @patch("isaacsim.asset.importer.heightmap.importer.GroundPlane")
    @patch("isaacsim.asset.importer.heightmap.importer.Usd")
    @patch("isaacsim.asset.importer.heightmap.importer.UsdGeom")
    def test_create_ground_plane_sizes_to_square_bounds(
        self, mock_usd_geom: object, _mock_usd: object, mock_ground_plane: object
    ) -> None:
        """Ground plane size for a square heightmap is dim + 2 * margin, centered on bbox.

        Args:
            mock_usd_geom: Mock for the UsdGeom module.
            _mock_usd: Mock for the Usd module.
            mock_ground_plane: Mock for the GroundPlane class.
        """
        # 10x10 heightmap occupying X:[0,10], Y:[-10,0]
        self._stub_bbox_cache(mock_usd_geom, min_pt=(0.0, -10.0, 0.0), max_pt=(10.0, 0.0, 1.0))

        self.importer._create_ground_plane(OCCUPANCY_MAP_PATH)

        mock_usd_geom.BBoxCache.return_value.ComputeWorldBound.assert_called_once()
        # The world-bound was computed against the occupancy map prim, not some other prim.
        self.mock_stage.GetPrimAtPath.assert_any_call(OCCUPANCY_MAP_PATH)

        mock_ground_plane.assert_called_once()
        _, kwargs = mock_ground_plane.call_args
        self.assertAlmostEqual(kwargs["sizes"], 10.0 + 2 * GROUND_PLANE_MARGIN)
        self.assertEqual(kwargs["positions"], [[5.0, -5.0, 0.0]])
        # No non-uniform scaling — that distorts the wireframe material.
        self.assertNotIn("scales", kwargs)

    @patch("isaacsim.asset.importer.heightmap.importer.GroundPlane")
    @patch("isaacsim.asset.importer.heightmap.importer.Usd")
    @patch("isaacsim.asset.importer.heightmap.importer.UsdGeom")
    def test_create_ground_plane_uses_max_dim_for_wide_bounds(
        self, mock_usd_geom: object, _mock_usd: object, mock_ground_plane: object
    ) -> None:
        """A wider-than-tall heightmap sizes the (square) ground plane to the longer dimension.

        Args:
            mock_usd_geom: Mock for the UsdGeom module.
            _mock_usd: Mock for the Usd module.
            mock_ground_plane: Mock for the GroundPlane class.
        """
        # X width = 20, Y height = 5 — wider than tall.
        self._stub_bbox_cache(mock_usd_geom, min_pt=(0.0, -5.0, 0.0), max_pt=(20.0, 0.0, 1.0))

        self.importer._create_ground_plane(OCCUPANCY_MAP_PATH)

        _, kwargs = mock_ground_plane.call_args
        self.assertAlmostEqual(kwargs["sizes"], 20.0 + 2 * GROUND_PLANE_MARGIN)
        self.assertEqual(kwargs["positions"], [[10.0, -2.5, 0.0]])

    @patch("isaacsim.asset.importer.heightmap.importer.GroundPlane")
    @patch("isaacsim.asset.importer.heightmap.importer.Usd")
    @patch("isaacsim.asset.importer.heightmap.importer.UsdGeom")
    def test_create_ground_plane_uses_max_dim_for_tall_bounds(
        self, mock_usd_geom: object, _mock_usd: object, mock_ground_plane: object
    ) -> None:
        """A taller-than-wide heightmap sizes the (square) ground plane to the longer dimension.

        Args:
            mock_usd_geom: Mock for the UsdGeom module.
            _mock_usd: Mock for the Usd module.
            mock_ground_plane: Mock for the GroundPlane class.
        """
        # X width = 4, Y height = 30 — taller than wide.
        self._stub_bbox_cache(mock_usd_geom, min_pt=(0.0, -30.0, 0.0), max_pt=(4.0, 0.0, 1.0))

        self.importer._create_ground_plane(OCCUPANCY_MAP_PATH)

        _, kwargs = mock_ground_plane.call_args
        self.assertAlmostEqual(kwargs["sizes"], 30.0 + 2 * GROUND_PLANE_MARGIN)
        self.assertEqual(kwargs["positions"], [[2.0, -15.0, 0.0]])

    @patch("isaacsim.asset.importer.heightmap.importer.GroundPlane")
    @patch("isaacsim.asset.importer.heightmap.importer.Usd")
    @patch("isaacsim.asset.importer.heightmap.importer.UsdGeom")
    def test_create_ground_plane_handles_off_origin_bounds(
        self, mock_usd_geom: object, _mock_usd: object, mock_ground_plane: object
    ) -> None:
        """Ground plane center follows the bbox midpoint even when bounds don't start at the origin.

        Args:
            mock_usd_geom: Mock for the UsdGeom module.
            _mock_usd: Mock for the Usd module.
            mock_ground_plane: Mock for the GroundPlane class.
        """
        # Heightmap shifted into positive Y as well: X:[10, 20], Y:[5, 15]
        self._stub_bbox_cache(mock_usd_geom, min_pt=(10.0, 5.0, 0.0), max_pt=(20.0, 15.0, 1.0))

        self.importer._create_ground_plane(OCCUPANCY_MAP_PATH)

        _, kwargs = mock_ground_plane.call_args
        self.assertEqual(kwargs["positions"], [[15.0, 10.0, 0.0]])
        self.assertAlmostEqual(kwargs["sizes"], 10.0 + 2 * GROUND_PLANE_MARGIN)

    @patch("isaacsim.asset.importer.heightmap.importer.carb")
    @patch("isaacsim.asset.importer.heightmap.importer.GroundPlane")
    @patch("isaacsim.asset.importer.heightmap.importer.Usd")
    @patch("isaacsim.asset.importer.heightmap.importer.UsdGeom")
    def test_create_ground_plane_skips_creation_for_empty_bounds(
        self,
        mock_usd_geom: object,
        _mock_usd: object,
        mock_ground_plane: object,
        mock_carb: object,
    ) -> None:
        """If the occupancy map has no occupied cells, the bbox is empty and the plane is skipped.

        Args:
            mock_usd_geom: Mock for the UsdGeom module.
            _mock_usd: Mock for the Usd module.
            mock_ground_plane: Mock for the GroundPlane class.
            mock_carb: Mock for the carb module.
        """
        mock_world_bound = MagicMock()
        empty_range = MagicMock()
        empty_range.IsEmpty.return_value = True
        mock_world_bound.GetRange.return_value = empty_range
        mock_usd_geom.BBoxCache.return_value.ComputeWorldBound.return_value = mock_world_bound

        self.importer._create_ground_plane(OCCUPANCY_MAP_PATH)

        mock_ground_plane.assert_not_called()
        mock_carb.log_warn.assert_called_once()


class TestHeightmapImporterEndToEnd(omni.kit.test.AsyncTestCase):
    """Run the whole of ``create_heightmap`` against a real stage.

    Deliberately unmocked. Patching out ``Usd`` to drive this method hides every decision
    that reads the stage back, so the assertions stop depending on the behavior they name
    and start depending on how the mocks were wired that day.
    """

    async def setUp(self) -> None:
        """Open a fresh stage."""
        await omni.usd.get_context().new_stage_async()

    async def tearDown(self) -> None:
        """Leave a clean stage behind."""
        await omni.usd.get_context().new_stage_async()

    async def test_a_full_run_builds_terrain_furniture_and_stage_metadata(self) -> None:
        """One cube per dark pixel, both pieces of furniture, and a metric Z-up stage."""
        image = Image.new("RGBA", (5, 5), color=(255, 255, 255, 255))
        pixels = image.load()
        for xy in ((0, 0), (1, 1), (2, 2)):
            pixels[xy] = (0, 0, 0, 255)
        stage = omni.usd.get_context().get_stage()
        importer = HeightmapImporter(stage)

        num_cells = importer.create_heightmap(image, cell_scale=0.05, create_ground_plane=True, create_lighting=True)

        self.assertEqual(num_cells, 3)
        self.assertEqual(importer.terrain_path, OCCUPANCY_MAP_PATH)
        self.assertEqual(world_child_names(stage), {"occupancyMap", "groundPlane", "defaultLight"})
        instancer = UsdGeom.PointInstancer(stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH)))
        self.assertEqual(len(instancer.GetPositionsAttr().Get()), 3)
        self.assertTrue(
            stage.GetPrimAtPath(cube_prototype_path(OCCUPANCY_MAP_PATH)).HasAPI(UsdPhysics.CollisionAPI),
            "the cells have no collision, so nothing can stand on the terrain",
        )
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)

    async def test_declining_the_furniture_leaves_the_terrain_on_its_own(self) -> None:
        """Nothing but terrain, and no ground plane sized against it either."""
        image = Image.new("RGBA", (3, 3), color=(0, 0, 0, 255))
        stage = omni.usd.get_context().get_stage()

        HeightmapImporter(stage).create_heightmap(
            image, cell_scale=1.0, create_ground_plane=False, create_lighting=False
        )

        self.assertEqual(world_child_names(stage), {"occupancyMap"})


class TestHeightmapExtension(omni.kit.test.AsyncTestCase):
    """Test suite for the heightmap extension UI."""

    @patch("omni.kit.window.filepicker.FilePickerDialog")
    def test_load_image_dialog_shows_filepicker(self, mock_file_picker_dialog: object) -> None:
        """Test that the load image file picker is visible after construction.

        Args:
            mock_file_picker_dialog: Mock for the file picker dialog constructor.
        """
        from isaacsim.asset.importer.heightmap.extension import Extension

        extension = Extension()
        extension._load_image_dialog()

        file_picker = mock_file_picker_dialog.return_value
        self.assertIs(extension._filepicker, file_picker)
        file_picker.show.assert_called_once_with()


class TestHeightmapExtensionStageHandling(omni.kit.test.AsyncTestCase):
    """Test which stage the Generate button writes terrain into.

    These tests use a real stage rather than a ``MagicMock``. The behavior under test is
    whether the existing stage survives generation, which a mock stage cannot observe.
    """

    async def setUp(self) -> None:
        """Open a fresh stage and build an extension with a loaded image."""
        from isaacsim.asset.importer.heightmap.extension import Extension

        await omni.usd.get_context().new_stage_async()
        self.extension = Extension()
        self.extension.on_startup("isaacsim.asset.importer.heightmap.tests")
        self.extension._image = Image.new("RGBA", (4, 4), color=(0, 0, 0, 255))
        self.extension._cell_size.set_value(1.0)

    async def tearDown(self) -> None:
        """Shut the extension down so its menu items and windows are released."""
        self.extension.on_shutdown()
        await omni.usd.get_context().new_stage_async()

    async def test_add_to_current_stage_is_enabled_by_default(self) -> None:
        """Generate must not discard scene content unless the user opts in."""
        self.assertTrue(self.extension._add_to_current_stage.get_value_as_bool())

    async def test_generate_preserves_existing_prims_when_adding_to_current_stage(self) -> None:
        """With the option enabled, terrain is added and existing prims survive."""
        usd_context = omni.usd.get_context()
        UsdGeom.Cube.Define(usd_context.get_stage(), PRESERVED_PRIM_PATH)
        stage_id = usd_context.get_stage_id()

        self.extension._add_to_current_stage.set_value(True)
        await press_generate(self.extension)

        stage = usd_context.get_stage()
        self.assertTrue(stage.GetPrimAtPath(PRESERVED_PRIM_PATH).IsValid())
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())
        # A preserved prim on a replaced stage would be a false pass, so pin the stage too.
        self.assertEqual(usd_context.get_stage_id(), stage_id)

    async def test_generate_replaces_stage_when_adding_to_current_stage_disabled(self) -> None:
        """With the option disabled, generation starts from a new stage."""
        usd_context = omni.usd.get_context()
        UsdGeom.Cube.Define(usd_context.get_stage(), PRESERVED_PRIM_PATH)

        self.extension._add_to_current_stage.set_value(False)
        await press_generate(self.extension)

        stage = usd_context.get_stage()
        self.assertFalse(stage.GetPrimAtPath(PRESERVED_PRIM_PATH).IsValid())
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

    async def test_generate_replaces_content_under_the_importers_own_paths(self) -> None:
        """Keeping the stage does not protect the paths the importer writes to.

        The option preserves the stage, not every prim on it. Content parented under
        the terrain namespace is still removed, which is what makes regenerating work.
        """
        usd_context = omni.usd.get_context()
        UsdGeom.Cube.Define(usd_context.get_stage(), COLLIDING_PRIM_PATH)

        self.extension._add_to_current_stage.set_value(True)
        self.extension._overwrite_existing.set_value(True)
        await press_generate(self.extension)

        stage = usd_context.get_stage()
        self.assertFalse(stage.GetPrimAtPath(COLLIDING_PRIM_PATH).IsValid())
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

    async def test_collision_warning_names_the_path_being_replaced(self) -> None:
        """Content that generation will overwrite has to be called out by path."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, COLLIDING_PRIM_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_generated_path_collisions(stage, True, True)

        mock_log_warn.assert_called_once()
        self.assertIn(OCCUPANCY_MAP_PATH, mock_log_warn.call_args[0][0])

    async def test_no_collision_warning_when_generated_paths_are_free(self) -> None:
        """A prim that survives generation must not be reported as at risk."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, PRESERVED_PRIM_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_generated_path_collisions(stage, True, True)

        mock_log_warn.assert_not_called()

    async def test_collision_warning_skips_paths_this_run_will_not_write(self) -> None:
        """A ground plane is only at risk if this run is going to create one."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, GROUND_PLANE_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_generated_path_collisions(stage, False, False)

        mock_log_warn.assert_not_called()

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_generated_path_collisions(stage, True, False)

        mock_log_warn.assert_called_once()
        self.assertIn(GROUND_PLANE_PATH, mock_log_warn.call_args[0][0])

    async def test_regenerating_does_not_warn_about_the_importers_own_output(self) -> None:
        """Warning on every regenerate would train users to ignore the one that matters."""
        self.extension._add_to_current_stage.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        for path in (OCCUPANCY_MAP_PATH, GROUND_PLANE_PATH, LIGHT_PATH):
            self.assertTrue(stage.GetPrimAtPath(path).IsValid(), f"{path} was not generated")

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_generated_path_collisions(stage, True, True)

        mock_log_warn.assert_not_called()

    async def test_content_parented_under_generated_terrain_is_still_reported(self) -> None:
        """Marking generated output must not silence the warning for the user's own prims."""
        self.extension._add_to_current_stage.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, COLLIDING_PRIM_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_generated_path_collisions(stage, True, True)

        mock_log_warn.assert_called_once()
        self.assertIn(OCCUPANCY_MAP_PATH, mock_log_warn.call_args[0][0])

    async def test_a_deactivated_user_prim_is_not_mistaken_for_generated_output(self) -> None:
        """The default traversal skips inactive prims, making an empty subtree look clean."""
        stage = omni.usd.get_context().get_stage()
        prim = UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH).GetPrim()
        prim.SetActive(False)

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_a_deactivated_child_of_generated_terrain_is_still_reported(self) -> None:
        """Deactivating a prim does not make it disposable."""
        self.extension._add_to_current_stage.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        child = UsdGeom.Cube.Define(stage, COLLIDING_PRIM_PATH).GetPrim()
        child.SetActive(False)

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_an_unloaded_payload_is_not_mistaken_for_generated_output(self) -> None:
        """Content behind an unloaded payload cannot be checked, so it cannot be claimed."""
        stage = omni.usd.get_context().get_stage()
        payload_layer = Sdf.Layer.CreateAnonymous(".usda")
        payload_stage = Usd.Stage.Open(payload_layer)
        UsdGeom.Xform.Define(payload_stage, "/Payload")
        payload_stage.SetDefaultPrim(payload_stage.GetPrimAtPath("/Payload"))

        prim = UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH).GetPrim()
        prim.CreateAttribute(GENERATED_MARKER_ATTR, Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        prim.GetPayloads().AddPayload(payload_layer.identifier)
        stage.Unload(OCCUPANCY_MAP_PATH)
        self.assertFalse(prim.IsLoaded())

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_an_arc_whose_target_is_missing_is_not_claimed_either(self) -> None:
        """An arc that resolves to nothing is the hardest kind to notice, not the easiest.

        A payload whose file is absent still answers ``IsLoaded()`` with True, and a
        reference, inherit or specialize pointing nowhere contributes no children at all.
        So the prim looks like a bare marked prim, and deleting it takes the arc with it --
        the user's work, which starts composing content the moment the target appears.
        """
        with tempfile.TemporaryDirectory() as directory:
            missing = str(Path(directory) / "missing.usda")
            arcs = {
                "payload": lambda prim: prim.GetPayloads().AddPayload(missing),
                "reference": lambda prim: prim.GetReferences().AddReference(missing),
                "inherit": lambda prim: prim.GetInherits().AddInherit("/NoSuchClass"),
                "specialize": lambda prim: prim.GetSpecializes().AddSpecialize("/NoSuchClass"),
            }
            for name, add_arc in arcs.items():
                with self.subTest(arc=name):
                    # A new stage per arc, so one iteration cannot carry state into the next.
                    await omni.usd.get_context().new_stage_async()
                    stage = omni.usd.get_context().get_stage()
                    prim = UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH).GetPrim()
                    prim.CreateAttribute(GENERATED_MARKER_ATTR, Sdf.ValueTypeNames.Bool, custom=True).Set(True)
                    add_arc(prim)
                    # The state that makes this invisible to a traversal, spelled out.
                    self.assertEqual(len(list(prim.GetChildren())), 0)

                    self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_an_unloaded_payload_on_a_descendant_is_not_claimed_either(self) -> None:
        """A payload hung off a marked descendant hides as much as one on the root.

        The prim carrying it is the importer's own and marked, so nothing in the traversal
        looks suspicious. Only the payload itself says content is being hidden, and it has
        to be noticed wherever in the subtree it is attached.
        """
        stage = omni.usd.get_context().get_stage()
        payload_layer = Sdf.Layer.CreateAnonymous(".usda")
        payload_stage = Usd.Stage.Open(payload_layer)
        UsdGeom.Xform.Define(payload_stage, "/Payload")
        payload_stage.SetDefaultPrim(payload_stage.GetPrimAtPath("/Payload"))

        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()
        self.assertTrue(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

        descendant = stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH))
        self.assertTrue(descendant.IsValid())
        descendant.GetPayloads().AddPayload(payload_layer.identifier)
        stage.Unload(descendant.GetPath())
        self.assertFalse(descendant.IsLoaded())
        # The root is what gets asked about, and it carries no payload of its own.
        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).HasAuthoredPayloads())

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_a_user_prim_under_a_deactivated_marked_prim_is_not_claimed(self) -> None:
        """Deactivating one of our own prims must not hide what the user put beneath it.

        Composition stops at an inactive prim, so a traversal reaches the marked prim and
        nothing under it. Every prim it manages to visit is then ours, which is exactly
        what a subtree of ours with the user's content hidden inside looks like.
        """
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()

        marked = stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH))
        UsdGeom.Cube.Define(stage, f"{marked.GetPath()}/UserCube")
        marked.SetActive(False)

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_a_user_prim_behind_an_instanceable_marked_prim_is_not_claimed(self) -> None:
        """An instance boundary hides its contents from traversal just as a payload does."""
        referenced = Sdf.Layer.CreateAnonymous(".usda")
        referenced_stage = Usd.Stage.Open(referenced)
        UsdGeom.Xform.Define(referenced_stage, "/Ref")
        UsdGeom.Cube.Define(referenced_stage, "/Ref/UserCube")
        referenced_stage.SetDefaultPrim(referenced_stage.GetPrimAtPath("/Ref"))

        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()

        marked = stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH))
        marked.GetReferences().AddReference(referenced.identifier)
        marked.SetInstanceable(True)
        self.assertTrue(marked.IsInstance())

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_a_user_prim_in_an_unselected_variant_is_not_claimed(self) -> None:
        """An unselected variant still holds content that removing the subtree would take."""
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()

        marked = stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH))
        variant_set = marked.GetVariantSets().AddVariantSet("userContent")
        variant_set.AddVariant("present")
        variant_set.AddVariant("absent")
        variant_set.SetVariantSelection("present")
        with variant_set.GetVariantEditContext():
            UsdGeom.Cube.Define(stage, f"{marked.GetPath()}/UserCube")
        variant_set.SetVariantSelection("absent")
        self.assertFalse(stage.GetPrimAtPath(f"{marked.GetPath()}/UserCube").IsValid())

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_generate_warns_about_a_collision_it_is_about_to_overwrite(self) -> None:
        """The warning is the only notice the user gets, so Generate must emit it."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)
        self.extension._add_to_current_stage.set_value(True)
        self.extension._overwrite_existing.set_value(True)

        with patch("carb.log_warn") as mock_log_warn:
            await press_generate(self.extension)

        messages = [call[0][0] for call in mock_log_warn.call_args_list]
        self.assertTrue(
            any(OCCUPANCY_MAP_PATH in message and "will replace existing content" in message for message in messages),
            f"no collision warning among {messages}",
        )

    async def test_overwriting_warns_about_a_users_prim_buried_under_our_terrain(self) -> None:
        """Overwriting discards the subtree, so a prim of theirs anywhere in it is at risk.

        The terrain root is the importer's own and looks entirely unremarkable. Only
        walking the subtree finds the prim that is about to be deleted, and the warning is
        the only thing standing between the user and losing it.
        """
        self.extension._add_to_current_stage.set_value(True)
        self.extension._overwrite_existing.set_value(True)
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, COLLIDING_PRIM_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            await press_generate(self.extension)

        messages = [call[0][0] for call in mock_log_warn.call_args_list]
        self.assertTrue(
            any(OCCUPANCY_MAP_PATH in message and "will replace existing content" in message for message in messages),
            f"the buried prim was deleted without warning: {messages}",
        )

    async def test_an_empty_heightmap_clears_a_floor_left_by_a_previous_run(self) -> None:
        """A floor outliving the terrain it was sized for is stale geometry with no owner."""
        self.extension._add_to_current_stage.set_value(True)
        # Only a run that replaces the terrain can orphan the floor. A run that builds
        # alongside leaves the first terrain standing, and the floor still belongs to it.
        self.extension._overwrite_existing.set_value(True)
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())

        self.extension._image = Image.new("RGBA", (4, 4), color=(255, 255, 255, 255))
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertFalse(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())

    async def test_an_empty_second_run_leaves_the_standing_scene_alone(self) -> None:
        """A run that finds nothing to build must still not disturb what is already there.

        The floor belongs to the first run's terrain, which is still standing. What spares
        it is the furniture skip rather than the empty-bounds clear: a ground plane is never
        attempted when its path is taken, so that branch is not reached at all. The guard
        inside it is covered by ``test_a_path_the_run_skips_is_not_claimed_as_generated``.
        """
        self.extension._add_to_current_stage.set_value(True)
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())

        self.extension._image = Image.new("RGBA", (4, 4), color=(255, 255, 255, 255))
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())
        self.assertTrue(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())
        # Without this the test passes on a run that simply rebuilt over the first terrain.
        self.assertEqual(self.extension._importer.terrain_path, f"{OCCUPANCY_MAP_PATH}_01")

    async def test_a_path_the_run_skips_is_not_claimed_as_generated(self) -> None:
        """Marking a path nothing was written to would brand the user's prim as ours."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, GROUND_PLANE_PATH)
        # An all-white image occupies no cells, so the ground plane is skipped.
        self.extension._image = Image.new("RGBA", (4, 4), color=(255, 255, 255, 255))

        self.extension._add_to_current_stage.set_value(True)
        # Opted in, so the run writes to that path and the marking is what has to spare the
        # cube. Stepping aside would leave it untouched for the wrong reason and pass anyway.
        self.extension._overwrite_existing.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertEqual(stage.GetPrimAtPath(GROUND_PLANE_PATH).GetTypeName(), "Cube")
        self.assertFalse(is_importer_generated(stage, GROUND_PLANE_PATH))

    async def test_generate_is_ignored_while_a_previous_run_is_in_flight(self) -> None:
        """Overlapping runs would write terrain from two different settings."""
        self.extension._generate()
        first_task = self.extension._generation_task
        self.assertIsNotNone(first_task)

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._generate()

        self.assertIs(self.extension._generation_task, first_task)
        mock_log_warn.assert_called_once()
        self.assertIn("already running", mock_log_warn.call_args[0][0])

        await first_task

    async def test_shutdown_cancels_a_generation_still_in_flight(self) -> None:
        """A task resuming after shutdown would build terrain into the stage regardless.

        Not a widget problem: every input was snapshotted when the button was pressed, so
        the coroutine needs nothing from the panel and nothing would stop it.
        """
        self.extension._generate()
        task = self.extension._generation_task

        self.extension.on_shutdown()

        # Without the cancel the task resumes and generates into a stage nobody asked.
        with self.assertRaises(asyncio.CancelledError):
            await task

        # Re-created so tearDown's shutdown has something valid to tear down.
        self.extension.on_startup("isaacsim.asset.importer.heightmap.tests")

    async def test_generate_replaces_a_user_prim_at_the_ground_plane_path(self) -> None:
        """A non-ground-plane prim on that path must not abort generation."""
        usd_context = omni.usd.get_context()
        UsdGeom.Cube.Define(usd_context.get_stage(), GROUND_PLANE_PATH)

        self.extension._add_to_current_stage.set_value(True)
        self.extension._overwrite_existing.set_value(True)
        await press_generate(self.extension)

        stage = usd_context.get_stage()
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())
        # The cube is gone and a real ground plane stands in its place.
        self.assertNotEqual(stage.GetPrimAtPath(GROUND_PLANE_PATH).GetTypeName(), "Cube")
        self.assertTrue(is_importer_generated(stage, GROUND_PLANE_PATH))


class TestHeightmapExtensionAutoNaming(omni.kit.test.AsyncTestCase):
    """Test that a run without permission to overwrite changes nothing already on the stage.

    Stepping aside is what lets a user keep what they had and still get terrain, without
    having to go and rename anything first. It applies to terrain this importer built as
    much as to the user's own content: an option about overwriting that still overwrites
    some things would be the more surprising rule. The terrain is the deliverable, so it
    has to land somewhere; the ground plane and light are conveniences, so they are skipped
    rather than duplicated.
    """

    async def setUp(self) -> None:
        """Open a fresh stage and build an extension with a loaded image."""
        from isaacsim.asset.importer.heightmap.extension import Extension

        await omni.usd.get_context().new_stage_async()
        self.extension = Extension()
        self.extension.on_startup("isaacsim.asset.importer.heightmap.tests")
        self.extension._image = Image.new("RGBA", (4, 4), color=(0, 0, 0, 255))
        self.extension._cell_size.set_value(1.0)
        self.extension._add_to_current_stage.set_value(True)

    async def tearDown(self) -> None:
        """Shut the extension down so its menu items and windows are released."""
        self.extension.on_shutdown()
        await omni.usd.get_context().new_stage_async()

    async def test_overwrite_existing_is_disabled_by_default(self) -> None:
        """The non-destructive answer has to be what the user gets without deciding."""
        self.assertFalse(self.extension._overwrite_existing.get_value_as_bool())

    async def test_terrain_steps_aside_instead_of_replacing_a_users_prim(self) -> None:
        """The user keeps their prim and still gets terrain, with nothing to undo."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertEqual(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(), "Cube")
        self.assertTrue(is_importer_generated(stage, SUFFIXED_TERRAIN_PATH))

    async def test_the_terrain_is_whole_at_the_path_it_stepped_aside_to(self) -> None:
        """Terrain whose instancer stayed at the old path would be an empty transform."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        instancer = stage.GetPrimAtPath(instances_path(SUFFIXED_TERRAIN_PATH))
        self.assertEqual(instancer.GetTypeName(), "PointInstancer")
        self.assertTrue(UsdGeom.PointInstancer(instancer).GetPositionsAttr().Get())
        self.assertEqual(stage.GetPrimAtPath(cube_prototype_path(SUFFIXED_TERRAIN_PATH)).GetTypeName(), "Cube")

    async def test_stepping_aside_is_reported_so_the_terrain_can_be_found(self) -> None:
        """Terrain at a path the user did not choose is lost unless they are told."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            await press_generate(self.extension)

        messages = [call[0][0] for call in mock_log_warn.call_args_list]
        self.assertTrue(
            any(SUFFIXED_TERRAIN_PATH in message for message in messages),
            f"nothing said where the terrain went: {messages}",
        )

    async def test_stepping_aside_does_not_claim_to_be_replacing_anything(self) -> None:
        """Announcing a replacement that did not happen is worse than saying nothing."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            await press_generate(self.extension)

        messages = [call[0][0] for call in mock_log_warn.call_args_list]
        self.assertFalse(
            any("will replace existing content" in message for message in messages),
            f"a run that replaced nothing still announced a replacement: {messages}",
        )

    async def test_the_ground_plane_is_sized_to_the_terrain_that_was_built(self) -> None:
        """A plane sized from the abandoned path would collapse to nothing."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        terrain = world_bound(stage, SUFFIXED_TERRAIN_PATH)
        plane = world_bound(stage, GROUND_PLANE_PATH)
        self.assertFalse(terrain.IsEmpty())
        self.assertFalse(plane.IsEmpty())
        for axis in (0, 1):
            self.assertLessEqual(plane.GetMin()[axis], terrain.GetMin()[axis])
            self.assertGreaterEqual(plane.GetMax()[axis], terrain.GetMax()[axis])

    async def test_each_generate_adds_a_terrain_rather_than_replacing_the_last(self) -> None:
        """Reusing the path would overwrite, which is the one thing the option rules out."""
        await press_generate(self.extension)
        self.extension._cell_size.set_value(2.0)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertTrue(is_importer_generated(stage, OCCUPANCY_MAP_PATH))
        self.assertTrue(is_importer_generated(stage, SUFFIXED_TERRAIN_PATH))

    async def test_the_terrain_already_built_is_left_exactly_as_it_was(self) -> None:
        """Landing at a free path is no good if the run edits the old one on the way past."""
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()
        first = UsdGeom.PointInstancer(stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH)))
        positions_before = list(first.GetPositionsAttr().Get())

        self.extension._cell_size.set_value(2.0)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        first = UsdGeom.PointInstancer(stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH)))
        self.assertEqual(list(first.GetPositionsAttr().Get()), positions_before)

    async def test_terrains_keep_accumulating_past_the_first_suffix(self) -> None:
        """A run that only ever tries one suffix would start overwriting at the third press."""
        for cell_size in (1.0, 2.0, 3.0):
            self.extension._cell_size.set_value(cell_size)
            await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        for path in (OCCUPANCY_MAP_PATH, SUFFIXED_TERRAIN_PATH, f"{OCCUPANCY_MAP_PATH}_02"):
            self.assertTrue(is_importer_generated(stage, path), f"no terrain at {path}")

    async def test_a_users_prim_parented_under_a_terrain_survives_the_next_run(self) -> None:
        """Regenerating discards the whole subtree, so the whole subtree has to be spared."""
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, COLLIDING_PRIM_PATH)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(COLLIDING_PRIM_PATH).IsValid())
        self.assertTrue(is_importer_generated(stage, SUFFIXED_TERRAIN_PATH))

    async def test_our_own_ground_plane_is_kept_rather_than_rebuilt(self) -> None:
        """The floor is content on the stage too, so the option has to spare it as well.

        Measured rather than merely checked for existence: a run that cleared the floor and
        built a new one would leave a valid prim at the same path either way. Only its size
        says which happened, and the second run asks for cells four times as wide.
        """
        UsdGeom.SetStageUpAxis(omni.usd.get_context().get_stage(), UsdGeom.Tokens.z)
        await press_generate(self.extension)
        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())
        plane_before = world_bound(stage, GROUND_PLANE_PATH)
        self.assertFalse(plane_before.IsEmpty())

        self.extension._cell_size.set_value(4.0)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        plane_after = world_bound(stage, GROUND_PLANE_PATH)
        self.assertEqual(plane_after.GetMin(), plane_before.GetMin())
        self.assertEqual(plane_after.GetMax(), plane_before.GetMax())

    async def test_a_users_ground_plane_is_kept_and_no_second_one_is_added(self) -> None:
        """A second plane would sit coincident with theirs and z-fight with it."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, GROUND_PLANE_PATH)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertEqual(stage.GetPrimAtPath(GROUND_PLANE_PATH).GetTypeName(), "Cube")
        self.assertTrue(is_importer_generated(stage, OCCUPANCY_MAP_PATH))
        # A plane put anywhere else would show up here; there is no second one.
        self.assertEqual(world_child_names(stage), {"groundPlane", "defaultLight", "occupancyMap"})

    async def test_a_users_light_is_kept_and_no_second_one_is_added(self) -> None:
        """A second distant light would stack onto the lighting they already authored."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, LIGHT_PATH)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertEqual(stage.GetPrimAtPath(LIGHT_PATH).GetTypeName(), "Xform")
        self.assertEqual(world_child_names(stage), {"groundPlane", "defaultLight", "occupancyMap"})
        self.assertTrue(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_skipping_the_furniture_says_so(self) -> None:
        """A missing floor with no explanation reads as the option having failed."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, GROUND_PLANE_PATH)

        with patch("carb.log_warn") as mock_log_warn:
            await press_generate(self.extension)

        messages = [call[0][0] for call in mock_log_warn.call_args_list]
        self.assertTrue(
            any(GROUND_PLANE_PATH in message and "keeps what it had" in message for message in messages),
            f"nothing explained the missing ground plane: {messages}",
        )

    async def test_the_light_left_in_place_is_not_claimed_as_generated(self) -> None:
        """Marked as ours, the next run would feel free to overwrite what it just spared."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, LIGHT_PATH)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertFalse(is_importer_generated(stage, LIGHT_PATH))

    async def test_opting_in_replaces_the_content_instead_of_stepping_aside(self) -> None:
        """The opt-in has to actually overwrite, or the fixed path is unreachable."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        self.extension._overwrite_existing.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertNotEqual(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(), "Cube")
        self.assertTrue(is_importer_generated(stage, OCCUPANCY_MAP_PATH))
        self.assertFalse(stage.GetPrimAtPath(SUFFIXED_TERRAIN_PATH).IsValid())

    async def test_replacing_the_stage_leaves_the_base_path_free(self) -> None:
        """A new stage holds nothing, so there is nothing to step aside from."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        self.extension._add_to_current_stage.set_value(False)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertTrue(is_importer_generated(stage, OCCUPANCY_MAP_PATH))


class TestHeightmapImporterOverwriteApi(omni.kit.test.AsyncTestCase):
    """Test the ``overwrite_existing`` argument on the importer API itself."""

    async def setUp(self) -> None:
        """Open a fresh stage and a 2x2 fully occupied image."""
        await omni.usd.get_context().new_stage_async()
        self.image = Image.new("RGBA", (2, 2), color=(0, 0, 0, 255))

    async def tearDown(self) -> None:
        """Leave a clean stage behind."""
        await omni.usd.get_context().new_stage_async()

    def _generate(self, stage: Usd.Stage, **kwargs: bool) -> HeightmapImporter:
        """Generate terrain into ``stage``, leaving its metadata alone.

        Returns:
            The importer, so the caller can read where it put the terrain.
        """
        importer = HeightmapImporter(stage)
        importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False, **kwargs)
        return importer

    def _payload_layer_defining_the_terrain_path(self) -> Sdf.Layer:
        """Build a layer whose default prim holds user content at the terrain path.

        Payloaded onto ``/World``, this puts a prim of the user's at exactly the path the
        importer wants, reachable only once the payload is loaded.

        Returns:
            An anonymous layer suitable for payloading onto ``/World``.
        """
        layer = Sdf.Layer.CreateAnonymous(".usda")
        stage = Usd.Stage.Open(layer)
        UsdGeom.Xform.Define(stage, "/Env")
        theirs = UsdGeom.Cube.Define(stage, "/Env/occupancyMap").GetPrim()
        theirs.CreateAttribute("user:keepMe", Sdf.ValueTypeNames.String, custom=True).Set("keep-me")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/Env"))
        return layer

    async def test_content_behind_an_unloaded_payload_on_world_is_not_written_over(self) -> None:
        """An unloaded payload makes the path look empty, and it is anything but.

        Writing a local spec there does not delete their prim, but it stacks this
        importer's terrain on top of it: loading the payload afterwards composes their
        content and ours into one prim. Nothing visible on the stage says which paths the
        payload claims, so the only safe answer is to stop and say so.
        """
        stage = omni.usd.get_context().get_stage()
        payload = self._payload_layer_defining_the_terrain_path()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        world.GetPayloads().AddPayload(payload.identifier)
        stage.Unload(WORLD_PATH)
        # The path the importer would otherwise consider free, and why it looks that way.
        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid(), "setup failed to hide the path")

        with self.assertRaises(RuntimeError):
            self._generate(stage, overwrite_existing=False)

        self.assertFalse(
            stage.GetRootLayer().GetPrimAtPath(OCCUPANCY_MAP_PATH),
            "a local spec was authored over the prim the payload defines",
        )

    async def test_content_in_an_unselected_variant_on_world_is_not_written_over(self) -> None:
        """The variant is one selection away from colliding with whatever we wrote."""
        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        variants = world.GetVariantSets().AddVariantSet("layout")
        variants.AddVariant("withTheirTerrain")
        variants.AddVariant("empty")
        variants.SetVariantSelection("withTheirTerrain")
        with variants.GetVariantEditContext():
            UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)
        variants.SetVariantSelection("empty")
        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid(), "setup failed to hide the path")

        with self.assertRaises(RuntimeError):
            self._generate(stage, overwrite_existing=False)

        variants.SetVariantSelection("withTheirTerrain")
        self.assertEqual(
            stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(),
            "Cube",
            "their cube came back as something else, so the variant was written into",
        )

    async def test_content_an_unselected_variant_references_is_not_written_over(self) -> None:
        """A variant need not author the path itself to end up owning it.

        Nothing in this stage's layers mentions the terrain path: the variant carries a
        reference, and the path only appears once that reference is composed, which happens
        only when the variant is selected. Reading specs alone finds nothing to object to,
        so an arc on a variant above a generated path has to be taken at its word.
        """
        # Held for the length of the test, because an anonymous layer lives only as long as
        # something refers to it and the reference would otherwise dangle.
        their_asset = Sdf.Layer.CreateAnonymous("their_layout.usda")
        asset_stage = Usd.Stage.Open(their_asset)
        UsdGeom.Cube.Define(asset_stage, OCCUPANCY_MAP_PATH)
        asset_stage.SetDefaultPrim(asset_stage.GetPrimAtPath(WORLD_PATH))

        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        variants = world.GetVariantSets().AddVariantSet("layout")
        variants.AddVariant("theirs")
        variants.AddVariant("empty")
        variants.SetVariantSelection("theirs")
        with variants.GetVariantEditContext():
            world.GetReferences().AddReference(their_asset.identifier, Sdf.Path(WORLD_PATH))
        variants.SetVariantSelection("empty")
        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid(), "setup failed to hide the path")

        with self.assertRaisesRegex(RuntimeError, "hides everything beneath it"):
            self._generate(stage, overwrite_existing=False)

        variants.SetVariantSelection("theirs")
        self.assertEqual(
            stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(),
            "Cube",
            "their cube came back as something else, so the variant was written into",
        )

    async def test_a_deactivated_world_is_refused_whether_or_not_overwriting_is_allowed(self) -> None:
        """Overwriting sees through concealment no better than stepping aside does.

        Permission to replace content is not permission to replace content sight unseen,
        so a run that was told it may overwrite has to stop here too.
        """
        for overwrite_existing in (False, True):
            with self.subTest(overwrite_existing=overwrite_existing):
                await omni.usd.get_context().new_stage_async()
                stage = omni.usd.get_context().get_stage()
                UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim().SetActive(False)

                # Matched on the message, because USD raises a RuntimeError of its own
                # when written to through a deactivated prim. Asserting the type alone
                # would pass whether the run was refused or merely crashed.
                with self.assertRaisesRegex(RuntimeError, "hides everything beneath it"):
                    self._generate(stage, overwrite_existing=overwrite_existing)

    async def test_a_marker_inherited_from_a_class_does_not_make_a_prim_ours(self) -> None:
        """Only a marker this importer wrote on the prim itself counts as ownership.

        A composed value is not evidence of authorship: the user can inherit one from a
        class, and reading the attribute alone cannot tell that apart from a prim this
        importer stamped.
        """
        stage = omni.usd.get_context().get_stage()
        self._generate(stage)

        marked_class = stage.CreateClassPrim("/MarkedClass")
        marked_class.CreateAttribute(GENERATED_MARKER_ATTR, Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        theirs = UsdGeom.Cube.Define(stage, f"{OCCUPANCY_MAP_PATH}/TheirCube").GetPrim()
        theirs.GetInherits().AddInherit("/MarkedClass")
        self.assertTrue(theirs.GetAttribute(GENERATED_MARKER_ATTR).Get(), "setup failed to inherit the marker")

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_configuring_metadata_rewrites_a_stage_that_is_neither_metric_nor_z_up(self) -> None:
        """A new stage is already metric and Z-up, so only a hostile one proves the rewrite.

        Kit pins ``app.stage.upAxis`` to Z, which means a test that replaces the stage
        cannot tell the flag being honored from the default it would have got anyway.
        """
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)

        HeightmapImporter(stage).create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=True)

        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)
        self.assertEqual(stage.GetDefaultPrim().GetPath().pathString, WORLD_PATH)

    async def test_leaving_metadata_alone_keeps_a_stage_that_is_neither_metric_nor_z_up(self) -> None:
        """The other half of the same stage: opting out must change none of the three."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        original_default = stage.GetDefaultPrim()

        HeightmapImporter(stage).create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 0.01)
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.y)
        self.assertEqual(stage.GetDefaultPrim(), original_default)

    async def test_output_from_a_run_that_failed_partway_is_still_marked_as_ours(self) -> None:
        """Debris a failed run leaves behind must not be mistaken for the user's content.

        The light is written before the terrain, so a failure building the terrain leaves
        it standing. Marking only after every step succeeded would leave it unmarked and
        indistinguishable from a light of the user's, for good: later runs would refuse to
        build over it and would warn about replacing it.
        """
        stage = omni.usd.get_context().get_stage()
        importer = HeightmapImporter(stage)

        with patch.object(importer, "_create_heightmap_instances", side_effect=RuntimeError("boom")):
            with self.assertRaises(RuntimeError):
                importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

        self.assertTrue(stage.GetPrimAtPath(LIGHT_PATH).IsValid(), "the light should have been written")
        self.assertTrue(is_importer_generated(stage, LIGHT_PATH))

    async def test_a_run_that_failed_reports_no_terrain_path_of_its_own(self) -> None:
        """A path that was never built is not somewhere the caller can go looking."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)
        importer = HeightmapImporter(stage)

        with patch.object(importer, "_create_heightmap_instances", side_effect=RuntimeError("boom")):
            with self.assertRaises(RuntimeError):
                importer.create_heightmap(
                    self.image, cell_scale=1.0, configure_stage_metadata=False, overwrite_existing=False
                )

        # Had the resolved path been published before the terrain existed, this would be
        # occupancyMap_01: a path the caller would find empty.
        self.assertIsNone(importer.terrain_path)

    async def test_a_run_whose_ground_plane_failed_is_not_reported_as_successful(self) -> None:
        """The terrain standing is not the whole run, so it is not yet worth reporting."""
        stage = omni.usd.get_context().get_stage()
        importer = HeightmapImporter(stage)

        with patch.object(importer, "_create_ground_plane", side_effect=RuntimeError("boom")):
            with self.assertRaises(RuntimeError):
                importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

        self.assertIsNone(importer.terrain_path)

    async def test_a_bad_image_leaves_no_half_built_terrain_behind(self) -> None:
        """Debris from a failed read would be claimed by nobody and cleared by nothing.

        The image is the only fallible part of building terrain, so reading it before any
        prim is authored is what keeps a failure from leaving a rootless instancer that
        later runs have to treat as the user's.
        """
        stage = omni.usd.get_context().get_stage()
        importer = HeightmapImporter(stage)

        with patch.object(importer, "_generate_occupied_positions", side_effect=ValueError("bad image")):
            with self.assertRaises(ValueError):
                importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())
        self.assertFalse(stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH)).IsValid())

    async def test_a_bad_image_does_not_cost_the_terrain_that_was_already_there(self) -> None:
        """Clearing the path before reading the image would spend it on a read that fails.

        Overwriting removes the previous terrain to make room for the new one. Do that
        first and a bad image leaves the user with neither: cleaning up after the failure
        cannot bring back what was cleared to make room.
        """
        stage = omni.usd.get_context().get_stage()
        self._generate(stage, create_ground_plane=False, create_lighting=False)
        original = stage.GetPrimAtPath(instances_path(OCCUPANCY_MAP_PATH)).GetPath()

        importer = HeightmapImporter(stage)
        with patch.object(importer, "_generate_occupied_positions", side_effect=ValueError("bad image")):
            with self.assertRaises(ValueError):
                importer.create_heightmap(
                    self.image,
                    cell_scale=1.0,
                    configure_stage_metadata=False,
                    overwrite_existing=True,
                    create_ground_plane=False,
                    create_lighting=False,
                )

        self.assertTrue(stage.GetPrimAtPath(original).IsValid(), "the terrain they already had is gone")

    async def test_authoring_that_fails_partway_leaves_no_unmarked_terrain_behind(self) -> None:
        """Reading the image first is not enough, because authoring can fail too.

        The marker goes on once the whole subtree exists, so prims written before a failure
        carry none and later runs read them as the user's: building alongside them rather
        than over them, and warning before replacing them.
        """
        stage = omni.usd.get_context().get_stage()
        importer = HeightmapImporter(stage)

        with patch.object(importer, "_configure_point_instancer", side_effect=RuntimeError("usd refused")):
            with self.assertRaisesRegex(RuntimeError, "usd refused"):
                importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

        # The real proof: the next run gets the path it wanted, rather than stepping around
        # debris it would have had to treat as content of the user's.
        self.assertEqual(self._generate(stage, overwrite_existing=False).terrain_path, OCCUPANCY_MAP_PATH)

    async def test_the_api_replaces_existing_content_by_default(self) -> None:
        """The documented behavior of these fixed paths is that they are written to."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        importer = self._generate(stage)

        self.assertNotEqual(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(), "Cube")
        self.assertEqual(importer.terrain_path, OCCUPANCY_MAP_PATH)

    async def test_the_api_can_be_asked_to_step_aside_instead(self) -> None:
        """A script generating into a scene it did not author needs the safe option too."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)

        importer = self._generate(stage, overwrite_existing=False)

        self.assertEqual(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(), "Cube")
        self.assertEqual(importer.terrain_path, SUFFIXED_TERRAIN_PATH)
        self.assertTrue(is_importer_generated(stage, SUFFIXED_TERRAIN_PATH))

    async def test_a_second_call_reports_the_new_path_not_the_old_one(self) -> None:
        """``terrain_path`` is how a script finds what it just built, so it has to move."""
        stage = omni.usd.get_context().get_stage()

        first = self._generate(stage, overwrite_existing=False)
        second = self._generate(stage, overwrite_existing=False)

        self.assertEqual(first.terrain_path, OCCUPANCY_MAP_PATH)
        self.assertEqual(second.terrain_path, SUFFIXED_TERRAIN_PATH)

    async def test_an_unclearable_path_is_stepped_around_rather_than_refused(self) -> None:
        """A suffixed path has no spec in any layer, so the conflict simply does not apply."""
        stage = omni.usd.get_context().get_stage()
        sublayer = Sdf.Layer.CreateAnonymous()
        stage.GetRootLayer().subLayerPaths.append(sublayer.identifier)
        with Usd.EditContext(stage, sublayer):
            UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH)

        importer = self._generate(stage, overwrite_existing=False)

        self.assertEqual(importer.terrain_path, SUFFIXED_TERRAIN_PATH)
        self.assertTrue(is_importer_generated(stage, SUFFIXED_TERRAIN_PATH))

    async def test_a_muted_layer_spec_is_stepped_around_too(self) -> None:
        """The prim does not resolve while muted, but unmuting would stack it underneath."""
        stage = omni.usd.get_context().get_stage()
        sublayer = Sdf.Layer.CreateAnonymous()
        stage.GetRootLayer().subLayerPaths.append(sublayer.identifier)
        with Usd.EditContext(stage, sublayer):
            UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH)
        stage.MuteLayer(sublayer.identifier)
        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

        importer = self._generate(stage, overwrite_existing=False)

        self.assertEqual(importer.terrain_path, SUFFIXED_TERRAIN_PATH)

    async def test_a_muted_layer_hiding_the_spec_indirectly_is_stepped_around_too(self) -> None:
        """Muting the layer that *holds* the spec is the one shape a direct lookup catches.

        Mute something that sublayers it in, or that references it, or that only defines it
        inside a variant, and reading the muted layer's own specs finds nothing. The spec
        is still there, and unmuting brings it back under whatever was written on top.
        """

        def layer_defining_the_terrain_path() -> Sdf.Layer:
            layer = Sdf.Layer.CreateAnonymous(".usda")
            inner = Usd.Stage.Open(layer)
            UsdGeom.Cube.Define(inner, OCCUPANCY_MAP_PATH)
            return layer

        def nested_sublayer() -> Sdf.Layer:
            child = layer_defining_the_terrain_path()
            parent = Sdf.Layer.CreateAnonymous(".usda")
            parent.subLayerPaths.append(child.identifier)
            kept_alive.append(child)
            return parent

        def behind_a_reference() -> Sdf.Layer:
            asset = Sdf.Layer.CreateAnonymous(".usda")
            inner = Usd.Stage.Open(asset)
            UsdGeom.Xform.Define(inner, "/Env")
            UsdGeom.Cube.Define(inner, "/Env/occupancyMap")
            inner.SetDefaultPrim(inner.GetPrimAtPath("/Env"))
            holder = Sdf.Layer.CreateAnonymous(".usda")
            holder_stage = Usd.Stage.Open(holder)
            UsdGeom.Xform.Define(holder_stage, WORLD_PATH).GetPrim().GetReferences().AddReference(asset.identifier)
            kept_alive.append(asset)
            return holder

        def inside_a_variant() -> Sdf.Layer:
            layer = Sdf.Layer.CreateAnonymous(".usda")
            inner = Usd.Stage.Open(layer)
            world = UsdGeom.Xform.Define(inner, WORLD_PATH).GetPrim()
            variants = world.GetVariantSets().AddVariantSet("layout")
            variants.AddVariant("withMap")
            variants.SetVariantSelection("withMap")
            with variants.GetVariantEditContext():
                UsdGeom.Cube.Define(inner, OCCUPANCY_MAP_PATH)
            variants.SetVariantSelection("")
            return layer

        # Anonymous layers survive only while something references them.
        kept_alive: list[Sdf.Layer] = []

        def variant_inside_a_nested_sublayer() -> Sdf.Layer:
            # The one shape neither composition nor the root layer's specs can reach:
            # composing the muted layer picks the selected variant, and the spec that
            # matters is in a layer further down the stack.
            child = inside_a_variant()
            parent = Sdf.Layer.CreateAnonymous(".usda")
            parent.subLayerPaths.append(child.identifier)
            kept_alive.append(child)
            return parent

        shapes = {
            "nested sublayer": nested_sublayer,
            "behind a reference": behind_a_reference,
            "inside a variant": inside_a_variant,
            "variant inside a nested sublayer": variant_inside_a_nested_sublayer,
        }
        for label, build in shapes.items():
            with self.subTest(shape=label):
                # A new stage per shape, so one iteration cannot carry state into the next.
                await omni.usd.get_context().new_stage_async()
                stage = omni.usd.get_context().get_stage()
                hidden = build()
                stage.GetRootLayer().subLayerPaths.append(hidden.identifier)
                stage.MuteLayer(hidden.identifier)
                self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

                importer = self._generate(stage, overwrite_existing=False)

                self.assertEqual(importer.terrain_path, SUFFIXED_TERRAIN_PATH)
                self.assertFalse(
                    stage.GetRootLayer().GetPrimAtPath(OCCUPANCY_MAP_PATH),
                    "a spec was authored where unmuting would stack the hidden one beneath it",
                )

    async def test_running_out_of_candidate_paths_raises_rather_than_overwriting(self) -> None:
        """Failing closed matters most in the case nobody designed for."""
        stage = omni.usd.get_context().get_stage()
        with patch("isaacsim.asset.importer.heightmap.importer.MAX_TERRAIN_PATH_CANDIDATES", 2):
            UsdGeom.Cube.Define(stage, OCCUPANCY_MAP_PATH)
            UsdGeom.Cube.Define(stage, SUFFIXED_TERRAIN_PATH)

            with self.assertRaises(RuntimeError):
                self._generate(stage, overwrite_existing=False)

        self.assertEqual(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(), "Cube")
        self.assertEqual(stage.GetPrimAtPath(SUFFIXED_TERRAIN_PATH).GetTypeName(), "Cube")
        self.assertFalse(stage.GetPrimAtPath(LIGHT_PATH).IsValid())


class TestHeightmapExtensionSceneFurniture(omni.kit.test.AsyncTestCase):
    """Test what generation adds to a stage besides the terrain itself.

    A scene can already have a floor and its own lighting, and generating terrain into
    it should not force a second ground plane, a distant light, or different stage units
    on it. Real stages are used because the assertions are about stage contents.
    """

    async def setUp(self) -> None:
        """Open a fresh stage and build an extension with a loaded image."""
        from isaacsim.asset.importer.heightmap.extension import Extension

        await omni.usd.get_context().new_stage_async()
        self.extension = Extension()
        self.extension.on_startup("isaacsim.asset.importer.heightmap.tests")
        self.extension._image = Image.new("RGBA", (4, 4), color=(0, 0, 0, 255))
        self.extension._cell_size.set_value(1.0)

    async def tearDown(self) -> None:
        """Shut the extension down so its menu items and windows are released."""
        self.extension.on_shutdown()
        await omni.usd.get_context().new_stage_async()

    async def test_ground_plane_and_lighting_are_enabled_by_default(self) -> None:
        """Terrain generated on its own needs a floor and a light to be visible."""
        self.assertTrue(self.extension._create_ground_plane.get_value_as_bool())
        self.assertTrue(self.extension._create_lighting.get_value_as_bool())

    async def test_generate_adds_ground_plane_and_light_when_enabled(self) -> None:
        """Control for the opt-out tests: both are created when left enabled."""
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())
        self.assertTrue(stage.GetPrimAtPath(LIGHT_PATH).IsValid())

    async def test_generate_skips_ground_plane_when_disabled(self) -> None:
        """A scene with its own floor must not get a second one."""
        self.extension._create_ground_plane.set_value(False)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertFalse(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

    async def test_generate_skips_lighting_when_disabled(self) -> None:
        """A scene with authored lighting must not get a distant light added."""
        self.extension._create_lighting.set_value(False)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertFalse(stage.GetPrimAtPath(LIGHT_PATH).IsValid())
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

    async def test_generate_leaves_existing_stage_metadata_alone(self) -> None:
        """Rewriting units or up-axis would rescale or reorient the whole scene."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)

        self.extension._add_to_current_stage.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 0.01)
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.y)

    async def test_cell_size_is_in_meters_when_added_to_a_centimeter_stage(self) -> None:
        """Keeping the stage's units has to mean converting into them, not ignoring them.

        The panel turns "Add to Current Stage" into ``configure_stage_metadata=False``, and
        the worry is that the same switch also skips the unit conversion, leaving a
        1-meter cell authored as 1 stage unit — a centimeter here, a hundredth of the size
        asked for. Measured end to end through the button rather than at the API, because
        it is the wiring between the two that the claim is about.
        """
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        self.extension._add_to_current_stage.set_value(True)
        self.extension._create_ground_plane.set_value(False)
        self.extension._cell_size.set_value(1.0)

        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 0.01)
        terrain = world_bound(stage, OCCUPANCY_MAP_PATH)
        # A 4x4 fully occupied image at 1-meter cells spans 4 meters, which is 400 units on
        # this stage. Authored without conversion it would span 4 units, or 4 centimeters.
        width_in_meters = (terrain.GetMax()[0] - terrain.GetMin()[0]) * 0.01
        self.assertAlmostEqual(width_in_meters, 4.0, places=4)

    async def test_generate_sets_stage_metadata_when_replacing_the_stage(self) -> None:
        """Terrain is built for meters and Z-up, so its own stage is set up that way."""
        self.extension._add_to_current_stage.set_value(False)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)

    async def test_mismatched_stage_up_axis_is_reported(self) -> None:
        """Orientation is not converted, so a Y-up scene has to be called out."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_stage_up_axis_mismatch(stage)

        mock_log_warn.assert_called_once()
        self.assertIn("Y-up", mock_log_warn.call_args[0][0])

    async def test_matching_stage_up_axis_is_not_reported(self) -> None:
        """A Z-up stage needs no caveat, whatever its units."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        with patch("carb.log_warn") as mock_log_warn:
            self.extension._warn_on_stage_up_axis_mismatch(stage)

        mock_log_warn.assert_not_called()

    async def test_regenerating_at_a_larger_cell_size_resizes_the_ground_plane(self) -> None:
        """Terrain that outgrows its floor would hang over the edge and drop colliders."""
        # The ground plane is built for the stage's up-axis while terrain is always built in
        # XY, so only a Z-up stage puts the two in the same plane to compare.
        UsdGeom.SetStageUpAxis(omni.usd.get_context().get_stage(), UsdGeom.Tokens.z)
        self.extension._add_to_current_stage.set_value(True)
        # Resizing is only on the table for a run that replaces the terrain in place. A run
        # that builds a second one alongside leaves the first floor with the first terrain.
        self.extension._overwrite_existing.set_value(True)

        self.extension._cell_size.set_value(0.25)
        await press_generate(self.extension)

        self.extension._cell_size.set_value(4.0)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        terrain = world_bound(stage, OCCUPANCY_MAP_PATH)
        plane = world_bound(stage, GROUND_PLANE_PATH)
        self.assertFalse(terrain.IsEmpty())
        self.assertFalse(plane.IsEmpty())
        for axis in (0, 1):
            self.assertLessEqual(plane.GetMin()[axis], terrain.GetMin()[axis])
            self.assertGreaterEqual(plane.GetMax()[axis], terrain.GetMax()[axis])

    async def test_generate_replaces_an_existing_light_rather_than_retyping_it(self) -> None:
        """Retyping in place would leave the user's children and attributes behind."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, LIGHT_PATH)
        UsdGeom.Cube.Define(stage, f"{LIGHT_PATH}/UserChild")

        self.extension._add_to_current_stage.set_value(True)
        self.extension._overwrite_existing.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(LIGHT_PATH).IsValid())
        self.assertFalse(stage.GetPrimAtPath(f"{LIGHT_PATH}/UserChild").IsValid())

    async def test_generate_does_not_set_a_default_prim_on_a_stage_it_kept(self) -> None:
        """`configure_stage_metadata` off has to mean no metadata is written at all."""
        stage = omni.usd.get_context().get_stage()
        stage.ClearDefaultPrim()

        self.extension._add_to_current_stage.set_value(True)
        await press_generate(self.extension)

        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())
        self.assertFalse(stage.HasDefaultPrim())


class TestHeightmapImporterStageUnits(omni.kit.test.AsyncTestCase):
    """Test that a cell size in meters produces terrain of that physical size.

    ``cell_scale`` is documented in meters, so on a stage authored in other units it has
    to be converted or the terrain comes out the wrong size by the unit ratio.
    """

    async def setUp(self) -> None:
        """Open a fresh stage and a 2x2 fully occupied image."""
        await omni.usd.get_context().new_stage_async()
        self.image = Image.new("RGBA", (2, 2), color=(0, 0, 0, 255))

    async def tearDown(self) -> None:
        """Leave a clean stage behind."""
        await omni.usd.get_context().new_stage_async()

    def _terrain_width_in_units(self, stage: Usd.Stage) -> float:
        """Return the generated terrain's X extent in stage units."""
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bounds = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH)).GetRange()
        return bounds.GetMax()[0] - bounds.GetMin()[0]

    def _terrain_width_in_meters(self, stage: Usd.Stage) -> float:
        """Return the generated terrain's X extent converted back into meters."""
        return self._terrain_width_in_units(stage) * UsdGeom.GetStageMetersPerUnit(stage)

    async def test_cell_size_is_in_meters_on_a_metric_stage(self) -> None:
        """Baseline: two 1-meter cells span 2 meters on a metric stage."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        HeightmapImporter(stage).create_heightmap(
            self.image, cell_scale=1.0, create_ground_plane=False, create_lighting=False
        )

        self.assertAlmostEqual(self._terrain_width_in_meters(stage), 2.0, places=4)

    async def test_cell_size_is_still_in_meters_on_a_centimeter_stage(self) -> None:
        """The same request must produce the same physical size on a centimeter stage."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        HeightmapImporter(stage).create_heightmap(
            self.image,
            cell_scale=1.0,
            create_ground_plane=False,
            create_lighting=False,
            configure_stage_metadata=False,
        )

        self.assertAlmostEqual(self._terrain_width_in_meters(stage), 2.0, places=4)

    async def test_ground_plane_margin_is_in_meters_on_a_centimeter_stage(self) -> None:
        """The margin is a meter constant, so it has to be converted like the cell size."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        HeightmapImporter(stage).create_heightmap(
            self.image,
            cell_scale=1.0,
            create_ground_plane=True,
            create_lighting=False,
            configure_stage_metadata=False,
        )

        # The ground plane's geometry is not authored with the default purpose, so restricting
        # to it would collapse the plane to a point.
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            includedPurposes=[
                UsdGeom.Tokens.default_,
                UsdGeom.Tokens.proxy,
                UsdGeom.Tokens.guide,
                UsdGeom.Tokens.render,
            ],
        )
        plane = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(GROUND_PLANE_PATH)).GetRange()
        terrain = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH)).GetRange()

        # GROUND_PLANE_MARGIN is 1 meter, so the plane should overhang the terrain by 1 meter.
        overhang_in_meters = (plane.GetMax()[0] - terrain.GetMax()[0]) * 0.01
        self.assertAlmostEqual(overhang_in_meters, GROUND_PLANE_MARGIN, places=3)

    async def _assert_falls_back_to_metric(self, meters_per_unit: float) -> None:
        """Assert terrain is authored at its metric size despite unusable stage units."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, meters_per_unit)

        num_cells = HeightmapImporter(stage).create_heightmap(
            self.image,
            cell_scale=1.0,
            create_ground_plane=False,
            create_lighting=False,
            configure_stage_metadata=False,
        )

        self.assertEqual(num_cells, 4)
        # Falling back to metric means a factor of 1.0, so two 1.0 cells span 2.0 units.
        self.assertAlmostEqual(self._terrain_width_in_units(stage), 2.0, places=4)

    async def test_zero_stage_units_fall_back_to_metric(self) -> None:
        """A stage claiming zero meters per unit must not divide by zero."""
        await self._assert_falls_back_to_metric(0.0)

    async def test_nan_stage_units_fall_back_to_metric(self) -> None:
        """NaN fails every comparison, so a naive guard would author NaN geometry."""
        await self._assert_falls_back_to_metric(float("nan"))


class TestHeightmapImporterLayerHandling(omni.kit.test.AsyncTestCase):
    """Test generation onto a stage whose terrain paths are defined in another layer.

    ``RemovePrim`` only clears specs in the layer being edited, so a prim defined in a
    sublayer survives it. Generating anyway would stack a second set of transform ops onto
    the surviving prim, which raises deep inside USD after the stage has already been
    written to.

    These runs all insist on the fixed paths, which is what makes a refusal the only way
    out. A run allowed to pick its own path steps around the conflict instead, covered in
    :py:class:`TestHeightmapImporterOverwriteApi`.
    """

    async def setUp(self) -> None:
        """Open a fresh stage and a 2x2 fully occupied image."""
        await omni.usd.get_context().new_stage_async()
        self.image = Image.new("RGBA", (2, 2), color=(0, 0, 0, 255))

    async def tearDown(self) -> None:
        """Leave a clean stage behind."""
        await omni.usd.get_context().new_stage_async()

    async def test_a_generated_path_defined_in_another_layer_is_refused(self) -> None:
        """Refusing up front is the only outcome that leaves the stage untouched."""
        stage = omni.usd.get_context().get_stage()
        sublayer = Sdf.Layer.CreateAnonymous()
        stage.GetRootLayer().subLayerPaths.append(sublayer.identifier)
        with Usd.EditContext(stage, sublayer):
            UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH)

        with self.assertRaises(RuntimeError):
            HeightmapImporter(stage).create_heightmap(
                self.image,
                cell_scale=1.0,
                create_ground_plane=True,
                create_lighting=True,
                configure_stage_metadata=False,
            )

        # Bailing out mid-generation would leave the light and ground plane behind with no undo.
        self.assertFalse(stage.GetPrimAtPath(LIGHT_PATH).IsValid())
        self.assertFalse(stage.GetPrimAtPath(GROUND_PLANE_PATH).IsValid())

    async def test_a_generated_path_authored_in_a_variant_is_refused(self) -> None:
        """A variant spec shares the edit layer but is still beyond RemovePrim's reach."""
        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        variant_set = world.GetVariantSets().AddVariantSet("terrain")
        variant_set.AddVariant("A")
        variant_set.SetVariantSelection("A")
        with variant_set.GetVariantEditContext():
            UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH)
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

        # Matched on the message: the visibility guard refuses with a RuntimeError too, and
        # this test is about the unremovable spec rather than about anything being hidden.
        with self.assertRaisesRegex(RuntimeError, "which the layer being edited cannot remove"):
            HeightmapImporter(stage).create_heightmap(
                self.image,
                cell_scale=1.0,
                create_ground_plane=True,
                create_lighting=True,
                configure_stage_metadata=False,
            )

        # The light is created before the occupancy map, so a late refusal would destroy it.
        self.assertFalse(stage.GetPrimAtPath(LIGHT_PATH).IsValid())

    async def test_a_marker_arriving_through_a_reference_on_world_does_not_make_a_prim_ours(self) -> None:
        """The arc is on `/World`, which the ownership walk never looks at.

        Treating any arc on a prim as grounds for disowning it does not help here: the
        reference sits above the generated subtree, so every prim the walk visits looks
        clean while the marker it reads was authored in somebody else's layer.
        """
        referenced = Sdf.Layer.CreateAnonymous(".usda")
        inner = Usd.Stage.Open(referenced)
        UsdGeom.Xform.Define(inner, "/Env")
        theirs = UsdGeom.Cube.Define(inner, "/Env/occupancyMap").GetPrim()
        theirs.CreateAttribute(GENERATED_MARKER_ATTR, Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        inner.SetDefaultPrim(inner.GetPrimAtPath("/Env"))

        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        world.GetReferences().AddReference(referenced.identifier)
        # The state that makes this dangerous: their prim reads as marked, and carries no
        # arc of its own for the walk to notice.
        occupancy_map = stage.GetPrimAtPath(OCCUPANCY_MAP_PATH)
        self.assertTrue(occupancy_map.GetAttribute(GENERATED_MARKER_ATTR).Get())
        self.assertFalse(occupancy_map.HasAuthoredReferences())

        self.assertFalse(is_importer_generated(stage, OCCUPANCY_MAP_PATH))

    async def test_an_instanceable_world_is_refused_rather_than_failing_in_usd(self) -> None:
        """Authoring under an instance proxy is an error USD reports in its own words.

        Left to USD the user gets a composition stack trace; refused here they get the
        path of the prim to fix.
        """
        referenced = Sdf.Layer.CreateAnonymous(".usda")
        inner = Usd.Stage.Open(referenced)
        UsdGeom.Xform.Define(inner, "/Env")
        inner.SetDefaultPrim(inner.GetPrimAtPath("/Env"))

        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        world.GetReferences().AddReference(referenced.identifier)
        world.SetInstanceable(True)
        self.assertTrue(world.IsInstance(), "setup failed to make /World an instance")

        with self.assertRaisesRegex(RuntimeError, "hides everything beneath it"):
            HeightmapImporter(stage).create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

    async def test_a_loaded_payload_on_world_does_not_block_generation(self) -> None:
        """Refusing every payloaded environment would rule out most real stages.

        A payload that resolved and loaded has its content composed into the namespace, so
        the generated paths can be inspected like any other. Only an unloaded one hides
        them, and the two must not be treated alike.
        """
        payload_layer = Sdf.Layer.CreateAnonymous(".usda")
        payload_stage = Usd.Stage.Open(payload_layer)
        UsdGeom.Xform.Define(payload_stage, "/Env")
        UsdGeom.Cube.Define(payload_stage, "/Env/occupancyMap")
        payload_stage.SetDefaultPrim(payload_stage.GetPrimAtPath("/Env"))

        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        world.GetPayloads().AddPayload(payload_layer.identifier)
        self.assertTrue(world.IsLoaded(), "setup failed to load the payload")
        # Their prim is visible, so the run can see it and step aside rather than refuse.
        self.assertTrue(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

        importer = HeightmapImporter(stage)
        importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False, overwrite_existing=False)

        self.assertEqual(importer.terrain_path, f"{OCCUPANCY_MAP_PATH}_01")
        self.assertEqual(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).GetTypeName(), "Cube")

    async def test_a_variant_set_that_leaves_the_generated_paths_alone_does_not_block_generation(self) -> None:
        """A lookdev set on `/World` hides nothing this run is about to write.

        Refusing on the mere presence of an unselected variant would rule out exactly the
        authored stages this option exists to generate into, for no gain: whatever those
        variants define, it is not at the paths the importer wants.
        """
        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        variants = world.GetVariantSets().AddVariantSet("timeOfDay")
        for name in ("day", "night"):
            variants.AddVariant(name)
            variants.SetVariantSelection(name)
            with variants.GetVariantEditContext():
                UsdGeom.Xform.Define(stage, f"{WORLD_PATH}/Sky")
        variants.SetVariantSelection("day")

        importer = HeightmapImporter(stage)
        importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

        self.assertEqual(importer.terrain_path, OCCUPANCY_MAP_PATH)

    async def test_a_variant_set_whose_only_variant_is_selected_hides_nothing(self) -> None:
        """Refusing every stage that merely has variants would rule out most authored ones.

        The selected variant's content is composed and visible, so there is nothing a run
        could write over unseen. Only the variants left unselected are unreachable.
        """
        stage = omni.usd.get_context().get_stage()
        world = UsdGeom.Xform.Define(stage, WORLD_PATH).GetPrim()
        variant_set = world.GetVariantSets().AddVariantSet("terrain")
        variant_set.AddVariant("A")
        variant_set.SetVariantSelection("A")

        importer = HeightmapImporter(stage)
        importer.create_heightmap(self.image, cell_scale=1.0, configure_stage_metadata=False)

        self.assertEqual(importer.terrain_path, OCCUPANCY_MAP_PATH)

    async def test_a_generated_path_defined_in_a_muted_layer_is_refused(self) -> None:
        """Muting hides the spec from the stage, but unmuting stacks it under new terrain."""
        stage = omni.usd.get_context().get_stage()
        sublayer = Sdf.Layer.CreateAnonymous()
        stage.GetRootLayer().subLayerPaths.append(sublayer.identifier)
        with Usd.EditContext(stage, sublayer):
            UsdGeom.Xform.Define(stage, OCCUPANCY_MAP_PATH)
        stage.MuteLayer(sublayer.identifier)
        # Muted, so the prim does not resolve at all and a stage-level check sees nothing.
        self.assertFalse(stage.GetPrimAtPath(OCCUPANCY_MAP_PATH).IsValid())

        with self.assertRaises(RuntimeError):
            HeightmapImporter(stage).create_heightmap(
                self.image,
                cell_scale=1.0,
                create_ground_plane=True,
                create_lighting=True,
                configure_stage_metadata=False,
            )

        self.assertFalse(stage.GetPrimAtPath(LIGHT_PATH).IsValid())


class TestHeightmapImporterIntegration(omni.kit.test.AsyncTestCase):
    """Integration tests for HeightmapImporter with realistic scenarios."""

    def setUp(self) -> None:
        """Set up test fixtures."""
        self.mock_stage = MagicMock()
        self.importer = HeightmapImporter(self.mock_stage)

    def test_checkerboard_pattern(self) -> None:
        """Test position generation with a checkerboard pattern."""
        # Create 4x4 checkerboard
        test_image = Image.new("RGBA", (4, 4), color=(255, 255, 255, 255))
        pixels = test_image.load()
        for y in range(4):
            for x in range(4):
                if (x + y) % 2 == 0:
                    pixels[x, y] = (0, 0, 0, 255)

        positions = self.importer._generate_occupied_positions(test_image, 1.0, 0.5)

        # Checkerboard should have 8 black squares
        self.assertEqual(len(positions), 8)

    def test_stripe_pattern(self) -> None:
        """Test position generation with vertical stripes."""
        # Create 6x4 image with vertical stripes
        test_image = Image.new("RGBA", (6, 4), color=(255, 255, 255, 255))
        pixels = test_image.load()
        for y in range(4):
            for x in range(0, 6, 2):  # Every other column
                pixels[x, y] = (0, 0, 0, 255)

        positions = self.importer._generate_occupied_positions(test_image, 1.0, 0.5)

        # Should have 12 positions (3 columns × 4 rows)
        self.assertEqual(len(positions), 12)

    def test_large_image_performance(self) -> None:
        """Test that large images can be processed without errors."""
        # Create a 100x100 image
        test_image = Image.new("RGBA", (100, 100), color=(255, 255, 255, 255))
        pixels = test_image.load()
        # Make diagonal black
        for i in range(100):
            pixels[i, i] = (0, 0, 0, 255)

        positions = self.importer._generate_occupied_positions(test_image, 0.05, 0.025)

        # Should have 100 positions (diagonal)
        self.assertEqual(len(positions), 100)

        # Verify that positions increase along diagonal
        sorted_positions = sorted(positions, key=lambda p: (p[0], -p[1]))
        for i in range(len(sorted_positions) - 1):
            # X should increase
            self.assertLess(sorted_positions[i][0], sorted_positions[i + 1][0])
