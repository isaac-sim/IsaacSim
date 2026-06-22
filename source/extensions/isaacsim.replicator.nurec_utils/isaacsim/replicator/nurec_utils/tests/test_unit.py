# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for the NuRec rendering utilities' pure-Python helpers.

Covers render-config loading and case selection, keyframe subsampling, time/pose math,
manifest IO, and the numpy metric helpers. The torch-based PSNR/SSIM are checked separately.
"""

from __future__ import annotations

import os
import tempfile

import numpy as np
import omni.kit.test
from isaacsim.replicator.nurec_utils import usd_utils
from isaacsim.replicator.nurec_utils.manifest import load_manifest, write_manifest
from isaacsim.replicator.nurec_utils.metrics.psnr_ssim import image_diff, load_rgb, match_shape, mean_abs_diff
from isaacsim.replicator.nurec_utils.rendering_setup import build_extra_args, load_config, resolve_path, select_cases
from isaacsim.replicator.nurec_utils.usd_utils import (
    KeyframeIndex,
    frame_poses,
    frame_timestamp_us,
    subsample,
    subsample_evenly,
    subsample_first,
)


class TestResolvePath(omni.kit.test.AsyncTestCase):
    """Path resolution helper."""

    async def test_relative_joins_base(self) -> None:
        """Relative paths join the base dir."""
        self.assertEqual(resolve_path("a/b.usda", "/base/cfg"), os.path.normpath(os.path.join("/base/cfg", "a/b.usda")))

    async def test_absolute_unchanged(self) -> None:
        """Absolute paths pass through unchanged."""
        self.assertEqual(resolve_path("/x/y.usda", "/base"), "/x/y.usda")

    async def test_user_expansion(self) -> None:
        """A leading ~ expands to the home directory."""
        self.assertEqual(resolve_path("~/z.usda", "/base"), os.path.join(os.path.expanduser("~"), "z.usda"))


class TestSelectCases(omni.kit.test.AsyncTestCase):
    """Multi-case config flattening."""

    async def test_single_stage_returns_self(self) -> None:
        """A single-stage config yields itself."""
        cfg = {"stage": "s.usda", "gt_root": "g"}
        self.assertEqual(select_cases(cfg), [cfg])

    async def test_multi_case_merges_over_top_level(self) -> None:
        """Each case merges its fields over the shared top-level keys."""
        cfg = {"gt_root": "g", "cases": [{"name": "p", "stage": "p.usda"}, {"name": "v", "stage": "v.usda"}]}
        cases = select_cases(cfg)
        self.assertEqual([c["name"] for c in cases], ["p", "v"])
        self.assertEqual(cases[0]["gt_root"], "g")  # shared key inherited
        self.assertEqual(cases[0]["stage"], "p.usda")  # case key
        self.assertNotIn("cases", cases[0])

    async def test_filter_by_name(self) -> None:
        """A case name filters to the matching case."""
        cfg = {"cases": [{"name": "p"}, {"name": "v"}]}
        self.assertEqual([c["name"] for c in select_cases(cfg, "v")], ["v"])

    async def test_unknown_case_raises(self) -> None:
        """An unknown case name raises SystemExit."""
        with self.assertRaises(SystemExit):
            select_cases({"cases": [{"name": "p"}]}, "nope")


class TestSubsample(omni.kit.test.AsyncTestCase):
    """Keyframe subsampling."""

    async def setUp(self) -> None:
        """Create a 0..9 sequence."""
        self.seq = list(range(10))

    async def test_evenly_endpoints_inclusive(self) -> None:
        """Even subsampling includes both endpoints."""
        self.assertEqual(subsample_evenly(self.seq, 3), [0, 4, 9])

    async def test_evenly_one_is_middle(self) -> None:
        """A single even sample is the middle element."""
        self.assertEqual(subsample_evenly(self.seq, 1), [5])

    async def test_evenly_n_ge_len_returns_all(self) -> None:
        """Asking for >= len returns the whole sequence."""
        self.assertEqual(subsample_evenly(self.seq, 99), self.seq)

    async def test_first(self) -> None:
        """First-N subsampling takes the leading items."""
        self.assertEqual(subsample_first(self.seq, 3), [0, 1, 2])

    async def test_subsample_all_when_n_falsy(self) -> None:
        """A falsy count keeps the whole sequence."""
        self.assertEqual(subsample(self.seq, None), self.seq)
        self.assertEqual(subsample(self.seq, 0), self.seq)

    async def test_subsample_modes(self) -> None:
        """The mode selects the even or first strategy."""
        self.assertEqual(subsample(self.seq, 3, "even"), [0, 4, 9])
        self.assertEqual(subsample(self.seq, 3, "first"), [0, 1, 2])

    async def test_subsample_bad_mode_raises(self) -> None:
        """An unknown mode raises ValueError."""
        with self.assertRaises(ValueError):
            subsample(self.seq, 3, "bogus")

    async def test_empty(self) -> None:
        """Empty input yields empty output."""
        self.assertEqual(subsample_evenly([], 3), [])
        self.assertEqual(subsample_first([], 3), [])


class TestTimestamps(omni.kit.test.AsyncTestCase):
    """Time-code to timestamp conversion."""

    async def test_frame_timestamp_formula(self) -> None:
        """Timestamp is offset + (tc / tcps) * 1e6."""
        self.assertEqual(frame_timestamp_us(24.0, 24.0, 1_000_000), 1_000_000 + 1_000_000)

    async def test_zero_tcps_returns_offset(self) -> None:
        """Zero tcps returns just the offset."""
        self.assertEqual(frame_timestamp_us(5.0, 0.0, 42), 42)


class TestFramePoses(omni.kit.test.AsyncTestCase):
    """Per-frame timestamp + pose bundling."""

    async def test_bundles_timestamp_and_pose(self) -> None:
        """frame_poses bundles ts_us/ts_ns/position per time code."""
        # Patch `camera_world_position` to avoid needing pxr / a real stage.
        orig = usd_utils.camera_world_position
        usd_utils.camera_world_position = lambda stage, cam, tc: [tc, 0.0, 0.0]
        try:
            out = frame_poses(None, "/cam", [0.0, 24.0], 24.0, 1_000_000)
        finally:
            usd_utils.camera_world_position = orig
        self.assertEqual([o["ts_us"] for o in out], [1_000_000, 2_000_000])
        self.assertEqual(out[1]["ts_ns"], 2_000_000_000)
        self.assertEqual(out[0]["position"], [0.0, 0.0, 0.0])
        self.assertEqual(out[1]["time_code"], 24.0)


class TestKeyframeIndex(omni.kit.test.AsyncTestCase):
    """Nearest-within-tolerance timestamp -> rig keyframe time code."""

    async def test_match_within_and_outside_tolerance(self) -> None:
        """Match returns the nearest keyframe's time code within tolerance, else None."""
        ki = KeyframeIndex([1_000_000, 2_000_000], [10.0, 20.0])  # ts_ns -> time_code
        self.assertEqual(ki.match(1_000_000, tol_us=1.0), 10.0)  # exact
        self.assertEqual(ki.match(1_000_300, tol_us=1.0), 10.0)  # 300ns < 1us tolerance
        self.assertEqual(ki.match(1_999_900, tol_us=1.0), 20.0)  # nearest is 2_000_000
        self.assertIsNone(ki.match(1_500_000, tol_us=1.0))  # 0.5ms from either -> None

    async def test_empty(self) -> None:
        """An empty index never matches."""
        self.assertIsNone(KeyframeIndex([], []).match(1, 1.0))


class TestBuildExtraArgs(omni.kit.test.AsyncTestCase):
    """SimulationApp launch-arg assembly (fixed launch constants)."""

    async def test_launch_constants(self) -> None:
        """build_extra_args returns exactly the SPG launch constants — no runtime settings."""
        args = build_extra_args()
        self.assertIn("--enable", args)  # `--enable omni.rtx.spg` is two adjacent tokens
        self.assertEqual(args[args.index("--enable") + 1], "omni.rtx.spg")
        self.assertIn("--/renderer/multiGpu/enabled=false", args)
        # Pre-hydra-sync settings (spg/enabled, disableNuRecPostProcessings, ...) are applied
        # post-boot by the carb-override applier, not passed as launch args.
        self.assertNotIn("--/rtx/spg/enabled=true", args)


class TestLoadConfig(omni.kit.test.AsyncTestCase):
    """Default-config loading + --config overlay."""

    async def test_override_merges_one_level_deep(self) -> None:
        """--config replaces top-level keys and merges nested dicts per key (siblings kept)."""
        with tempfile.TemporaryDirectory() as tmp:
            override = os.path.join(tmp, "ov.yaml")
            with open(override, "w") as f:
                f.write("keyframe_tolerance_us: 1000\nspg_pre_hydra_sync_overrides:\n  /rtx/spg/enabled: false\n")
            cfg = load_config(override)
            self.assertEqual(cfg["keyframe_tolerance_us"], 1000)  # top-level replaced
            spg = cfg["spg_pre_hydra_sync_overrides"]
            self.assertEqual(spg["/rtx/spg/enabled"], False)  # per-key override (the value we wrote)
            # Merge keeps unmentioned siblings: assert the key survives, not its shipped value
            # (asserting the value would just duplicate the default config).
            self.assertIn("/omni/rtx/nre/compositing/disableNuRecPostProcessings", spg)  # sibling kept


class TestMetrics(omni.kit.test.AsyncTestCase):
    """Pure-numpy metric helpers (PSNR/SSIM are torch, tested elsewhere)."""

    async def setUp(self) -> None:
        """Create a base image and a one-pixel-perturbed copy."""
        rng = np.random.default_rng(0)
        self.a = rng.integers(0, 256, size=(32, 32, 3), dtype=np.uint8)
        self.b = self.a.copy()
        self.b[0, 0, 0] = 255 - self.b[0, 0, 0]

    async def test_mean_abs_diff(self) -> None:
        """Mean abs diff is 0 for identical images and positive otherwise."""
        self.assertEqual(mean_abs_diff(self.a, self.a), 0.0)
        self.assertGreater(mean_abs_diff(self.a, self.b), 0.0)

    async def test_image_diff_shape_dtype(self) -> None:
        """Diff image keeps input shape/dtype and is all-zero for identical inputs."""
        d = image_diff(self.a, self.a)
        self.assertEqual(d.shape, self.a.shape)
        self.assertEqual(d.dtype, np.uint8)
        self.assertEqual(int(d.max()), 0)

    async def test_image_diff_amplify_clamps(self) -> None:
        """Amplification stays clamped to uint8 and is nonzero when inputs differ."""
        d = image_diff(self.a, self.b, amplify=100.0)
        self.assertLessEqual(int(d.max()), 255)
        self.assertGreater(int(d.max()), 0)

    async def test_match_shape(self) -> None:
        """match_shape returns the input unchanged on match, else resizes to GT."""
        self.assertIs(match_shape(self.a, self.a), self.a)
        small = self.a[:16, :8]
        out = match_shape(self.a, small)
        self.assertEqual(out.shape[:2], small.shape[:2])

    async def test_load_rgb_roundtrip(self) -> None:
        """load_rgb reads back a saved image as the same uint8 HxWx3 array."""
        from PIL import Image

        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "x.png")
            Image.fromarray(self.a).save(p)
            loaded = load_rgb(p)
            self.assertEqual(loaded.shape, self.a.shape)
            self.assertTrue(np.array_equal(loaded, self.a))


class TestManifest(omni.kit.test.AsyncTestCase):
    """Manifest write/load with relative-path storage."""

    async def test_roundtrip_relative_paths(self) -> None:
        """Rendered paths store relative and resolve back to absolute; the manifest is GT-free."""
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = os.path.join(tmp, "run")
            os.makedirs(out_dir)
            rendered = os.path.join(out_dir, "cam0", "123.png")
            pairs = [{"camera": "cam0", "rendered": rendered, "ts_ns": 123, "position": [1.0, 2.0, 3.0]}]
            manifest_path = write_manifest(out_dir, "/abs/stage.usda", ["cam0"], pairs)

            import json

            with open(manifest_path) as mf:
                raw = json.load(mf)
            self.assertEqual(raw["pairs"][0]["rendered"], "cam0/123.png")  # relative to out_dir
            self.assertNotIn("gt", raw["pairs"][0])  # render manifest carries no GT
            self.assertNotIn("gt_root", raw)

            header, loaded = load_manifest(manifest_path)
            self.assertEqual(header["stage"], "/abs/stage.usda")
            self.assertEqual(header["cameras"], ["cam0"])
            self.assertEqual(loaded[0]["rendered"], rendered)  # resolved back to absolute
            self.assertEqual(loaded[0]["ts_ns"], 123)
            self.assertEqual(loaded[0]["position"], [1.0, 2.0, 3.0])
