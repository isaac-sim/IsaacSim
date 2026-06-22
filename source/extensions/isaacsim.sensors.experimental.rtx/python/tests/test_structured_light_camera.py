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

"""Verify StructuredLightCamera projector authoring, timing, pose handling, CameraSensor RGB, and orchestrator capture."""

from __future__ import annotations

from fractions import Fraction
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera, StructuredLightCamera
from pxr import UsdGeom, UsdLux

# ------------------------------------------------------------------------------------
# Fixtures and helpers
# ------------------------------------------------------------------------------------

# Per-test camera resolution (height, width) for CameraSensor-based tests.
_RESOLUTION = (240, 320)

# Default pattern paths used by most tests; values are placeholder filesystem paths
# — the tests never actually read pixel data from them.
_PLACEHOLDER_DIRECTION_TEXTURE = Path("/tmp/sl_direction.exr")


def _make_placeholder_patterns(num: int, temp_dir: str) -> list[Path]:
    """Create ``num`` minimal 1×1 PNG files and return their paths.

    Some USD asset-resolver code paths probe for file existence at set time;
    writing a minimal valid PNG keeps those probes quiet during tests.

    Args:
        num: Number of placeholder patterns to create.
        temp_dir: Directory where placeholder patterns are written.

    Returns:
        Paths to the placeholder pattern files.
    """
    import struct
    import zlib

    def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", crc)

    patterns: list[Path] = []
    for i in range(num):
        p = Path(temp_dir) / f"pattern_{i:02d}.png"
        with open(p, "wb") as fh:
            fh.write(b"\x89PNG\r\n\x1a\n")
            fh.write(_png_chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0)))
            fh.write(_png_chunk(b"IDAT", zlib.compress(b"\x00" + bytes([i * 40, 255 - i * 40, 128]))))
            fh.write(_png_chunk(b"IEND", b""))
        patterns.append(p)
    return patterns


# ------------------------------------------------------------------------------------
# Shared base class
# ------------------------------------------------------------------------------------


class _StructuredLightCameraTestBase(omni.kit.test.AsyncTestCase):
    """Common setup/teardown and camera-lifetime tracking.

    Every :class:`StructuredLightCamera` instance constructed via :meth:`_make_camera`
    is released in ``tearDown`` so its ``GLOBAL_EVENT_UPDATE`` observer does not leak
    into subsequent tests running in the same Kit process.
    """

    async def setUp(self) -> None:
        super().setUp()
        self._temp_dir = None
        self._cameras: list[StructuredLightCamera] = []

    async def tearDown(self) -> None:
        # Release every StructuredLightCamera this test created so their
        # GLOBAL_EVENT_UPDATE observers do not leak into subsequent tests.
        for cam in self._cameras:
            try:
                cam.destroy()
            except Exception:
                pass
        self._cameras.clear()
        # Stop any Replicator Orchestrator activity kicked off by play()/step().
        try:
            await rep.orchestrator.stop_async()
        except Exception:
            pass
        try:
            omni.timeline.get_timeline_interface().stop()
            await omni.kit.app.get_app().next_update_async()
        except Exception:
            pass
        try:
            stage_utils.close_stage()
        except Exception:
            pass
        if self._temp_dir is not None:
            try:
                self._temp_dir.cleanup()
            except Exception:
                pass
        super().tearDown()

    def _make_camera(self, *args: Any, **kwargs: Any) -> StructuredLightCamera:
        """Construct a StructuredLightCamera and register it for automatic cleanup.

        Args:
            *args: Positional arguments forwarded to StructuredLightCamera.
            **kwargs: Keyword arguments forwarded to StructuredLightCamera.

        Returns:
            Constructed structured-light camera.
        """
        instance = StructuredLightCamera(*args, **kwargs)
        self._cameras.append(instance)
        return instance


# ------------------------------------------------------------------------------------
# Group A — Authoring API
# ------------------------------------------------------------------------------------


class TestStructuredLightCameraAuthoring(_StructuredLightCameraTestBase):
    """Authoring-only tests: no CameraSensor, no Replicator, no rendering."""

    async def setUp(self) -> None:
        """Create placeholder projector patterns for structured-light authoring tests."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        self._temp_dir = TemporaryDirectory()
        self._patterns = _make_placeholder_patterns(5, self._temp_dir.name)

    # -- initialization / prim structure --

    async def test_initialization(self) -> None:
        """Initialize a structured-light camera and verify default projector timing metadata."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        self.assertEqual(cam.get_num_patterns(), 5)
        self.assertEqual(
            cam.get_projector_timestamps(),
            [(0, 1), (1, 30), (1, 15), (1, 10), (2, 15)],
        )
        self.assertEqual(cam.get_projector_cycle_period(), (1, 6))
        self.assertEqual(cam.get_projector_prim_path(), "/World/camera/projectors")
        self.assertEqual(cam.get_projector_direction_texture(), _PLACEHOLDER_DIRECTION_TEXTURE)

    async def test_rect_light_creation(self) -> None:
        """Create one projector RectLight prim for each structured-light pattern."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        rect_lights = cam.get_rect_light_prims()
        self.assertEqual(len(rect_lights), 5)
        for i, prim in enumerate(rect_lights):
            self.assertTrue(prim.IsValid())
            self.assertEqual(str(prim.GetPath()), f"/World/camera/projectors/RectLight_{i:02d}")
            self.assertTrue(prim.IsA(UsdLux.RectLight))

    async def test_shaping_api_applied(self) -> None:
        """Apply UsdLux ShapingAPI to every structured-light projector RectLight."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        for prim in cam.get_rect_light_prims():
            self.assertTrue(prim.HasAPI(UsdLux.ShapingAPI))

    async def test_projector_attributes(self) -> None:
        """Author projector texture and primary-ray visibility attributes on each RectLight."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        for i, prim in enumerate(cam.get_rect_light_prims()):
            self.assertTrue(prim.GetAttribute("isProjector").Get())
            self.assertFalse(prim.GetAttribute("visibleInPrimaryRay").Get())
            texture_path = prim.GetAttribute("inputs:texture:file").Get()
            self.assertIn(f"pattern_{i:02d}.png", str(texture_path.path))

    async def test_direction_texture_set_on_all_rect_lights(self) -> None:
        """Author the projector direction texture on every structured-light RectLight."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        for prim in cam.get_rect_light_prims():
            direction_attr = prim.GetAttribute("projector:directionTexture:file")
            self.assertTrue(direction_attr.IsValid())
            direction_path = direction_attr.Get()
            self.assertEqual(str(direction_path.path), str(_PLACEHOLDER_DIRECTION_TEXTURE))

    async def test_url_asset_paths_preserved(self) -> None:
        # Asset-resolver paths (e.g. omniverse://) must be forwarded verbatim, not
        # run through pathlib.absolute() which would corrupt the scheme.
        """Preserve omniverse:// projector pattern and direction-texture asset paths verbatim."""
        url_patterns = [f"omniverse://server.example/patterns/image_{i:02d}.png" for i in range(3)]
        url_direction = "omniverse://server.example/direction.exr"
        cam = self._make_camera(
            "/World/camera_url",
            projector_light_patterns=url_patterns,
            projector_direction_texture=url_direction,
        )
        for i, prim in enumerate(cam.get_rect_light_prims()):
            texture_path = prim.GetAttribute("inputs:texture:file").Get()
            self.assertEqual(str(texture_path.path), url_patterns[i])
            direction_path = prim.GetAttribute("projector:directionTexture:file").Get()
            self.assertEqual(str(direction_path.path), url_direction)

    async def test_nonexistent_patterns_and_direction_texture_accepted(self) -> None:
        # USD handles missing-file validation at render time; authoring must not probe.
        """Accept missing projector texture paths without probing files during authoring."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=[Path("/nonexistent/p0.png")],
            projector_direction_texture=Path("/nonexistent/direction.exr"),
        )
        self.assertEqual(cam.get_num_patterns(), 1)

    # -- timestamps / cycle period --

    async def test_projector_timestamps_get_set(self) -> None:
        """Store structured-light projector timestamps as reduced rational pairs."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        # Fractions are stored in reduced form: (2, 1000) -> (1, 500) etc.
        cam.set_projector_timestamps([(0, 1), (1, 1000), (2, 1000), (3, 1000), (4, 1000)])
        self.assertEqual(
            cam.get_projector_timestamps(),
            [(0, 1), (1, 1000), (1, 500), (3, 1000), (1, 250)],
        )

    async def test_projector_timestamps_validation(self) -> None:
        # Non-zero first entry → ValueError.
        """Reject invalid structured-light projector pattern schedules and assets."""
        with self.assertRaises(ValueError):
            StructuredLightCamera(
                "/World/bad0",
                projector_light_patterns=self._patterns,
                projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
                projector_timestamps=[(1, 100), (2, 100), (3, 100), (4, 100), (5, 100)],
            )
        # Decreasing → ValueError.
        with self.assertRaises(ValueError):
            StructuredLightCamera(
                "/World/bad1",
                projector_light_patterns=self._patterns,
                projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
                projector_timestamps=[(0, 1), (2, 100), (1, 100), (3, 100), (4, 100)],
            )
        # Wrong length → ValueError.
        with self.assertRaises(ValueError):
            StructuredLightCamera(
                "/World/bad2",
                projector_light_patterns=self._patterns,
                projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
                projector_timestamps=[(0, 1), (1, 100)],
            )
        # Float value in tuple → ValueError (wrapped TypeError).
        with self.assertRaises(ValueError):
            StructuredLightCamera(
                "/World/bad3",
                projector_light_patterns=self._patterns,
                projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
                projector_timestamps=[(0, 1), (0.5, 1), (1, 1), (2, 1), (3, 1)],  # type: ignore[list-item]
            )
        # Empty patterns → ValueError.
        with self.assertRaises(ValueError):
            StructuredLightCamera(
                "/World/bad4",
                projector_light_patterns=[],
                projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            )
        # No direction texture → ValueError.
        with self.assertRaises(ValueError):
            StructuredLightCamera(
                "/World/bad5",
                projector_light_patterns=self._patterns,
                projector_direction_texture=None,
            )

    async def test_single_pattern_accepted(self) -> None:
        """Accept a single structured-light projector pattern and infer its cycle period."""
        cam = self._make_camera(
            "/World/camera_single",
            projector_light_patterns=[self._patterns[0]],
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        self.assertEqual(cam.get_num_patterns(), 1)
        self.assertEqual(cam.get_projector_cycle_period(), (1, 30))

    async def test_cycle_period_inferred(self) -> None:
        """Infer projector cycle period from the last timestamp plus first interval."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            projector_timestamps=[(0, 1), (1, 1000), (3, 1000), (6, 1000), (10, 1000)],
        )
        # Inferred cycle = timestamps[-1] + (timestamps[1] - timestamps[0]) = 10/1000 + 1/1000 = 11/1000
        self.assertEqual(cam.get_projector_cycle_period(), (11, 1000))

    async def test_cycle_period_explicit(self) -> None:
        """Preserve an explicitly supplied structured-light projector cycle period."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            projector_timestamps=[(0, 1), (1, 100), (2, 100), (3, 100), (4, 100)],
            projector_cycle_period=(1, 10),
        )
        self.assertEqual(cam.get_projector_cycle_period(), (1, 10))

    async def test_explicit_cycle_period_preserved_on_set_timestamps(self) -> None:
        """Keep an explicit cycle period and prior timestamps when schedule updates fail."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            projector_timestamps=[(0, 1), (1, 1000), (2, 1000), (3, 1000), (4, 1000)],
            projector_cycle_period=(1, 10),
        )
        # Compatible new timestamps — explicit cycle period preserved.
        cam.set_projector_timestamps([(0, 1), (1, 10_000), (2, 10_000), (3, 10_000), (4, 10_000)])
        self.assertEqual(cam.get_projector_cycle_period(), (1, 10))
        expected_ts_before_raise = cam.get_projector_timestamps()
        # Incompatible new timestamps (last >= cycle) — raises, prior state preserved.
        with self.assertRaises(ValueError):
            cam.set_projector_timestamps([(0, 1), (1, 8), (1, 7), (1, 6), (1, 5)])
        self.assertEqual(
            cam.get_projector_timestamps(),
            expected_ts_before_raise,
            "failed set_projector_timestamps must leave prior schedule intact",
        )
        self.assertEqual(
            cam.get_projector_cycle_period(),
            (1, 10),
            "failed set_projector_timestamps must leave explicit cycle period intact",
        )

    async def test_implicit_cycle_period_reinferred_on_set_timestamps(self) -> None:
        """Reinfer the projector cycle period when timestamps are changed without an explicit period."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        cam.set_projector_timestamps([(0, 1), (1, 1000), (2, 1000), (3, 1000), (4, 1000)])
        # New cycle = 4/1000 + 1/1000 = 5/1000 = 1/200
        self.assertEqual(cam.get_projector_cycle_period(), (1, 200))

    async def test_set_projector_cycle_period(self) -> None:
        """Set, validate, and reinfer the structured-light projector cycle period."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        cam.set_projector_cycle_period((1, 5))
        self.assertEqual(cam.get_projector_cycle_period(), (1, 5))
        # Invalid — period <= last timestamp.
        with self.assertRaises(ValueError):
            cam.set_projector_cycle_period((1, 10_000))
        # None → re-infer from current timestamps.
        cam.set_projector_cycle_period(None)
        self.assertEqual(cam.get_projector_cycle_period(), (1, 6))

    # -- pattern selection / visibility --

    async def test_initial_pattern_visibility(self) -> None:
        """Make only the first projector pattern visible after structured-light camera creation."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        self.assertEqual(cam.get_active_pattern_index(), 0)
        for i, prim in enumerate(cam.get_rect_light_prims()):
            vis = UsdGeom.Imageable(prim).ComputeVisibility()
            expected = UsdGeom.Tokens.inherited if i == 0 else UsdGeom.Tokens.invisible
            self.assertEqual(vis, expected, f"pattern {i}: expected {expected}, got {vis}")

    async def test_manual_pattern_switching(self) -> None:
        """Switch the active structured-light projector pattern manually and update visibility."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        cam.set_active_pattern_manual(3)
        self.assertEqual(cam.get_active_pattern_index(), 3)
        for i, prim in enumerate(cam.get_rect_light_prims()):
            vis = UsdGeom.Imageable(prim).ComputeVisibility()
            expected = UsdGeom.Tokens.inherited if i == 3 else UsdGeom.Tokens.invisible
            self.assertEqual(vis, expected)

    async def test_pattern_index_bounds(self) -> None:
        """Reject manual structured-light projector pattern indices outside the available range."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        with self.assertRaises(IndexError):
            cam.set_active_pattern_manual(5)
        with self.assertRaises(IndexError):
            cam.set_active_pattern_manual(-1)

    async def test_time_based_pattern_selection(self) -> None:
        # Use tight but unambiguous timestamps so each slot is easy to hit.
        """Select structured-light projector patterns by simulation time within a cycle."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            projector_timestamps=[(0, 1), (1, 1000), (2, 1000), (3, 1000), (4, 1000)],
            projector_cycle_period=(5, 1000),
        )
        for t, expected in [
            (0.0, 0),
            (0.0005, 0),
            (0.001, 1),
            (0.0025, 2),
            (0.0045, 4),
            (0.005, 0),  # wraps to cycle start
            (0.0055, 0),
            (0.006, 1),  # second cycle
        ]:
            idx = cam._pattern_index_at_time(t)
            self.assertEqual(idx, expected, f"t={t}s expected {expected} got {idx}")

    async def test_pattern_index_at_cycle_boundary(self) -> None:
        """Handle structured-light projector pattern selection exactly at cycle boundaries."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            projector_timestamps=[(0, 1), (1, 1000), (2, 1000), (3, 1000), (4, 1000)],
            projector_cycle_period=(5, 1000),
        )
        # Exactly at cycle_period — phase is 0, pattern 0 active.
        self.assertEqual(cam._pattern_index_at_time(float(Fraction(5, 1000))), 0)
        # Just before last pattern boundary.
        self.assertEqual(cam._pattern_index_at_time(float(Fraction(3, 1000) - Fraction(1, 10**9))), 2)
        self.assertEqual(cam._pattern_index_at_time(float(Fraction(3, 1000))), 3)

    # -- projector pose --

    async def test_default_projector_path(self) -> None:
        """Use a projectors child prim under the camera when no projector path is provided."""
        cam = self._make_camera(
            "/World/my_camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        self.assertEqual(cam.get_projector_prim_path(), "/World/my_camera/projectors")

    async def test_default_projector_pose_is_identity_when_child_of_camera(self) -> None:
        # When the projector is a child of the camera and no explicit pose is
        # supplied, its local transform must be identity so USD composition inherits
        # the camera's world pose.
        """Keep child projector local pose at identity so it inherits the camera pose."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            positions=np.array([[5.0, 6.0, 7.0]]),
        )
        proj_prim = stage_utils.get_current_stage().GetPrimAtPath(cam.get_projector_prim_path())
        xformable = UsdGeom.Xformable(proj_prim)
        ops = {op.GetOpType(): op for op in xformable.GetOrderedXformOps()}
        translate = ops[UsdGeom.XformOp.TypeTranslate].Get()
        self.assertAlmostEqual(float(translate[0]), 0.0, places=5)
        self.assertAlmostEqual(float(translate[1]), 0.0, places=5)
        self.assertAlmostEqual(float(translate[2]), 0.0, places=5)

    async def test_default_projector_pose_copies_camera_pose_when_sibling(self) -> None:
        # When the projector lives at a path outside the camera's subtree, its
        # world pose must match the camera's world pose.
        """Copy the camera world pose to a sibling structured-light projector prim."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            projector_prim_path="/World/sibling_projector",
            positions=np.array([[1.0, 2.0, 3.0]]),
        )
        proj_prim = stage_utils.get_current_stage().GetPrimAtPath(cam.get_projector_prim_path())
        xformable = UsdGeom.Xformable(proj_prim)
        ops = {op.GetOpType(): op for op in xformable.GetOrderedXformOps()}
        translate = ops[UsdGeom.XformOp.TypeTranslate].Get()
        self.assertAlmostEqual(float(translate[0]), 1.0, places=5)
        self.assertAlmostEqual(float(translate[1]), 2.0, places=5)
        self.assertAlmostEqual(float(translate[2]), 3.0, places=5)

    async def test_explicit_projector_position_orientation(self) -> None:
        """Author an explicit structured-light projector position and orientation."""
        position = np.array([10.0, 20.0, 30.0])
        orientation = np.array([1.0, 0.0, 0.0, 0.0])
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
            projector_prim_path="/World/explicit_projector",
            projector_position=position,
            projector_orientation=orientation,
        )
        proj_prim = stage_utils.get_current_stage().GetPrimAtPath(cam.get_projector_prim_path())
        xformable = UsdGeom.Xformable(proj_prim)
        ops = {op.GetOpType(): op for op in xformable.GetOrderedXformOps()}
        translate = ops[UsdGeom.XformOp.TypeTranslate].Get()
        self.assertAlmostEqual(float(translate[0]), 10.0, places=5)
        self.assertAlmostEqual(float(translate[1]), 20.0, places=5)
        self.assertAlmostEqual(float(translate[2]), 30.0, places=5)

    # -- construction-time safety --

    async def test_rect_light_type_collision_rolls_back_camera_prim(self) -> None:
        # Pre-create a caller-authored RectLight_00 of the wrong type under a
        # known projector path. Construction must raise and cleanup must undo
        # ONLY the prims it created — pre-existing caller data must survive.
        """Roll back created camera prims without deleting caller-authored projector children."""
        stage_utils.define_prim("/World/external_projector", "Xform")
        stage_utils.define_prim("/World/external_projector/RectLight_00", "Xform")
        stage = stage_utils.get_current_stage()
        self.assertFalse(stage.GetPrimAtPath("/World/rollback_cam").IsValid())
        with self.assertRaises(RuntimeError):
            StructuredLightCamera(
                "/World/rollback_cam",
                projector_light_patterns=self._patterns,
                projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
                projector_prim_path="/World/external_projector",
            )
        # The fresh Camera prim must have been cleaned up.
        self.assertFalse(stage.GetPrimAtPath("/World/rollback_cam").IsValid())
        # The pre-existing caller-authored prims must survive unchanged.
        ext_proj = stage.GetPrimAtPath("/World/external_projector")
        self.assertTrue(ext_proj.IsValid(), "caller-authored projector Xform was deleted by rollback")
        self.assertEqual(ext_proj.GetTypeName(), "Xform")
        # The Xform should not have been mutated with projector xformOps.
        self.assertEqual(
            len(UsdGeom.Xformable(ext_proj).GetOrderedXformOps()),
            0,
            "caller-authored projector Xform had xformOps written by a failed construction",
        )
        existing_rect_light = stage.GetPrimAtPath("/World/external_projector/RectLight_00")
        self.assertTrue(existing_rect_light.IsValid(), "caller-authored RectLight_00 was deleted by rollback")
        self.assertEqual(existing_rect_light.GetTypeName(), "Xform")

    async def test_inheritance_from_rtx_camera(self) -> None:
        """Expose StructuredLightCamera as an RtxCamera with a valid Camera prim and wrapper."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        self.assertIsInstance(cam, RtxCamera)
        prim = cam.prims[0]
        self.assertEqual(prim.GetTypeName(), "Camera")
        self.assertTrue(prim.HasAPI("OmniSensorAPI"))
        self.assertIsNotNone(cam.camera)

    # -- lifecycle --

    async def test_destroy_nulls_subscription(self) -> None:
        """Destroy the structured-light app-update subscription idempotently."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        self.assertIsNotNone(cam._app_update_sub)
        cam.destroy()
        self.assertIsNone(cam._app_update_sub)
        # Idempotent.
        cam.destroy()
        self.assertIsNone(cam._app_update_sub)

    async def test_post_reset(self) -> None:
        """Reset structured-light pattern state, warning state, and cached simulation time."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=_PLACEHOLDER_DIRECTION_TEXTURE,
        )
        cam.set_active_pattern_manual(3)
        self.assertEqual(cam.get_active_pattern_index(), 3)
        cam._warned_coarse_dt = True  # simulate a prior warning
        cam.post_reset()
        self.assertEqual(cam.get_active_pattern_index(), 0)
        self.assertFalse(cam._warned_coarse_dt)
        self.assertIsNone(cam._prev_sim_time)


# ------------------------------------------------------------------------------------
# Group B — CameraSensor integration
# ------------------------------------------------------------------------------------


class TestStructuredLightCameraWithCameraSensor(_StructuredLightCameraTestBase):
    """Integration tests wrapping a StructuredLightCamera in :class:`CameraSensor`."""

    async def setUp(self) -> None:
        """Create a lit scene and placeholder structured-light assets for CameraSensor tests."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        stage_utils.define_prim("/World", "Xform")
        # Add a DomeLight so primary rays hit a lit scene; projector lights have
        # visibleInPrimaryRay=False and with an empty direction texture would not
        # contribute any measurable illumination on their own.
        UsdLux.DomeLight.Define(stage_utils.get_current_stage(), "/World/DomeLight")
        Cube("/World/target", sizes=2.0, positions=[0.0, 0.0, 0.0], colors=[1.0, 1.0, 1.0])
        self._temp_dir = TemporaryDirectory()
        self._patterns = _make_placeholder_patterns(3, self._temp_dir.name)
        self._direction_texture = Path(self._temp_dir.name) / "direction.exr"
        self._direction_texture.write_bytes(b"")  # empty placeholder — USD tolerates it

    async def test_camera_sensor_creation(self) -> None:
        """Wrap a StructuredLightCamera in CameraSensor with RGB annotator and fixed resolution."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=self._direction_texture,
            positions=np.array([[3.0, 3.0, 2.0]]),
            orientations=np.array([[1.0, 0.0, 0.0, 0.0]]),
        )
        sensor = CameraSensor(cam, resolution=_RESOLUTION, annotators=["rgb"])
        try:
            # ``CameraSensor.camera`` returns the underlying experimental Camera wrapper
            # sourced from ``authoring_object.camera``.
            self.assertIsNotNone(sensor.camera)
            self.assertEqual(sensor.resolution, _RESOLUTION)
        finally:
            del sensor
            await omni.kit.app.get_app().next_update_async()

    async def test_camera_sensor_rgb_shape(self) -> None:
        """Capture RGB data from a StructuredLightCamera-backed CameraSensor."""
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=self._direction_texture,
            positions=np.array([[3.0, 3.0, 2.0]]),
        )
        ViewportManager.set_camera_view(cam.paths[0], eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.0])
        sensor = CameraSensor(cam, resolution=_RESOLUTION, annotators=["rgb"])
        try:
            app_utils.play(commit=True)
            data = None
            for _ in range(30):
                await app_utils.update_app_async()
                data, _ = sensor.get_data("rgb")
                if data is not None:
                    break
            self.assertIsNotNone(data, "No RGB data after 30 app updates")
            self.assertEqual(tuple(data.shape), (*_RESOLUTION, 3))
        finally:
            del sensor


# ------------------------------------------------------------------------------------
# Group C — Replicator Orchestrator capture
# ------------------------------------------------------------------------------------


class TestStructuredLightCameraOrchestrator(_StructuredLightCameraTestBase):
    """End-to-end tests using ``rep.orchestrator.step_async`` with pattern cycling."""

    # 5 patterns with variable intervals spanning 0–4 ms.
    _TIMESTAMPS: list[tuple[int, int]] = [
        (0, 1),
        (7, 10_000),
        (17, 10_000),
        (26, 10_000),
        (4, 1_000),
    ]

    async def setUp(self) -> None:
        """Create a lit scene and timed structured-light assets for orchestrator capture tests."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        stage_utils.define_prim("/World", "Xform")
        UsdLux.DomeLight.Define(stage_utils.get_current_stage(), "/World/DomeLight")
        Cube("/World/target", sizes=2.0, positions=[0.0, 0.0, 0.0], colors=[1.0, 1.0, 1.0])
        self._temp_dir = TemporaryDirectory()
        self._patterns = _make_placeholder_patterns(len(self._TIMESTAMPS), self._temp_dir.name)
        self._direction_texture = Path(self._temp_dir.name) / "direction.exr"
        self._direction_texture.write_bytes(b"")

    async def _make_camera_and_sensor(self) -> tuple[StructuredLightCamera, CameraSensor]:
        cam = self._make_camera(
            "/World/camera",
            projector_light_patterns=self._patterns,
            projector_direction_texture=self._direction_texture,
            projector_timestamps=self._TIMESTAMPS,
            positions=np.array([[3.0, 3.0, 2.0]]),
        )
        ViewportManager.set_camera_view(cam.paths[0], eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.0])
        sensor = CameraSensor(cam, resolution=_RESOLUTION, annotators=["rgb"])
        return cam, sensor

    async def test_orchestrator_single_step_capture(self) -> None:
        """Capture one orchestrator RGB step and verify the first projector pattern stays active."""
        cam, sensor = await self._make_camera_and_sensor()
        try:
            await rep.orchestrator.step_async(rt_subframes=2, delta_time=0.0)
            data, _ = sensor.get_data("rgb")
            # One extra update in case orchestrator wait_for_render returned before
            # the annotator's Python-side buffer refreshed.
            for _ in range(3):
                if data is not None:
                    break
                await app_utils.update_app_async()
                data, _ = sensor.get_data("rgb")
            self.assertIsNotNone(data, "No RGB data after orchestrator.step")
            self.assertEqual(tuple(data.shape), (*_RESOLUTION, 3))
            # First step at delta_time=0 keeps us at t=0 → pattern 0 active.
            self.assertEqual(cam.get_active_pattern_index(), 0)
        finally:
            del sensor

    async def test_orchestrator_pattern_cycle(self) -> None:
        """Advance orchestrator steps through each projector interval and verify pattern cycling."""
        cam, sensor = await self._make_camera_and_sensor()
        try:
            fractions = [Fraction(*ts) for ts in self._TIMESTAMPS]
            intervals = [fractions[0]] + [fractions[i] - fractions[i - 1] for i in range(1, len(fractions))]
            cumulative = Fraction(0)
            for idx, interval in enumerate(intervals):
                await rep.orchestrator.step_async(rt_subframes=2, delta_time=float(interval))
                cumulative += interval
                # Poll up to a few updates for the app-update callback to observe the
                # new timeline value and switch patterns. One update is usually enough
                # but CI has been observed to need more headroom.
                active = cam.get_active_pattern_index()
                for _ in range(5):
                    if active == idx:
                        break
                    await app_utils.update_app_async()
                    active = cam.get_active_pattern_index()
                self.assertEqual(
                    active,
                    idx,
                    msg=f"step {idx}: expected pattern {idx}, got {active}; interval={interval}",
                )
                # Verify the timeline advanced by the expected amount (float compared
                # with a generous tolerance — the orchestrator's commit path may snap
                # to the timeline's internal timebase).
                current = omni.timeline.get_timeline_interface().get_current_time()
                self.assertAlmostEqual(
                    current,
                    float(cumulative),
                    places=6,
                    msg=f"step {idx}: timeline at {current}s, expected {float(cumulative)}s",
                )
                data, _ = sensor.get_data("rgb")
                self.assertIsNotNone(data, f"step {idx}: missing RGB data")
        finally:
            del sensor
