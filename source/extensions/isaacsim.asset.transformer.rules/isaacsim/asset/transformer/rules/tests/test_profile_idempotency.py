# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Idempotency test: the Isaac Sim profile should produce stable output across consecutive runs."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.manager import AssetTransformerManager
from isaacsim.asset.transformer.models import RuleProfile
from pxr import Sdf

from .common import _TEST_DATA_DIR, _UR10E_SHOULDER_USD

_PROFILE_JSON = os.path.join(os.path.dirname(_TEST_DATA_DIR), "isaacsim_structure.json")


def _normalise_layer(layer: Sdf.Layer) -> str:
    """Export *layer* to USDA text with only the ``doc`` field stripped.

    The ``doc`` field accumulates "Generated from Composed Stage of root
    layer ..." entries that embed absolute paths, so it legitimately
    differs between consecutive runs.  Everything else must match.

    Args:
        layer: The USD layer to normalize.

    Returns:
        The normalized USDA text.
    """
    layer.documentation = ""
    return layer.ExportToString()


def _collect_layers(root_dir: str) -> dict[str, str]:
    """Return ``{relative_path: normalised_usda}`` for every USD file under *root_dir*.

    Args:
        root_dir: The root directory to search for USD files.

    Returns:
        A mapping from relative file path to normalised USDA text.
    """
    result: dict[str, str] = {}
    for dirpath, _, filenames in os.walk(root_dir):
        for fn in sorted(filenames):
            if not fn.endswith((".usda", ".usd")):
                continue
            abs_path = os.path.join(dirpath, fn)
            layer = Sdf.Layer.FindOrOpen(abs_path)
            if layer is not None:
                result[os.path.relpath(abs_path, root_dir)] = _normalise_layer(layer)
    return result


def _diff_detail(rel: str, a: str, b: str, context: int = 2) -> str:
    """Return a human-readable description of the first difference between *a* and *b*.

    Args:
        rel: Relative path for the file being compared.
        a: The first text to compare.
        b: The second text to compare.
        context: Number of context lines around the difference.

    Returns:
        A human-readable description of the first difference.
    """
    lines_a = a.splitlines()
    lines_b = b.splitlines()
    line_no = 0
    for i, (la, lb) in enumerate(zip(lines_a, lines_b), 1):
        if la != lb:
            line_no = i
            break
    if line_no == 0:
        if len(lines_a) != len(lines_b):
            line_no = min(len(lines_a), len(lines_b)) + 1
        else:
            return f"{rel}: content differs (unknown location)"
    idx = line_no - 1
    run1_line = lines_a[idx] if idx < len(lines_a) else "<EOF>"
    run2_line = lines_b[idx] if idx < len(lines_b) else "<EOF>"
    start = max(0, idx - context)
    end = min(max(len(lines_a), len(lines_b)), idx + context + 1)
    ctx_a = "\n".join(f"    {j}: {lines_a[j]!r}" for j in range(start, min(end, len(lines_a))))
    ctx_b = "\n".join(f"    {j}: {lines_b[j]!r}" for j in range(start, min(end, len(lines_b))))
    return (
        f"{rel}: content differs (first mismatch ~line {line_no})\n"
        f"  run1 line {line_no}: {run1_line!r}\n"
        f"  run2 line {line_no}: {run2_line!r}\n"
        f"  run1 context:\n{ctx_a}\n"
        f"  run2 context:\n{ctx_b}"
    )


class TestProfileIdempotency(omni.kit.test.AsyncTestCase):
    """Run the full Isaac Sim profile twice and assert stable output."""

    async def setUp(self) -> None:
        """Initialise temp-directory tracking and success flag."""
        self._dirs: list[str] = []
        self._success = False

    async def tearDown(self) -> None:
        """Remove temp directories on success; keep them on failure for inspection."""
        if self._success:
            for d in self._dirs:
                shutil.rmtree(d, ignore_errors=True)

    def _tmpdir(self) -> str:
        """Create a unique temp directory and track it for cleanup on success.

        Returns:
            Absolute path to the new directory.
        """
        d = tempfile.mkdtemp(prefix="asset_transformer_rules_test_profile_idempotency_")
        self._dirs.append(d)
        return d

    async def test_double_transform_produces_identical_output(self) -> None:
        """Transform ur10e_shoulder twice; the second output must match the first."""
        errors: list[str] = []

        with open(_PROFILE_JSON, encoding="utf-8") as fh:
            profile = RuleProfile.from_json(fh.read())

        # -- Run 1: original asset -------------------------------------------
        out1 = self._tmpdir()
        report1 = AssetTransformerManager().run(
            input_stage=_UR10E_SHOULDER_USD,
            profile=profile,
            package_root=out1,
        )
        for r in report1.results:
            if not r.success:
                errors.append(f"Run 1 rule '{r.rule.name}' failed: {r.error}")
        if errors:
            self.fail("\n".join(errors))

        run1_output = report1.output_stage_path
        self.assertTrue(
            run1_output and os.path.isfile(run1_output),
            f"Run 1 did not produce an output stage: {run1_output}",
        )

        # -- Run 2: re-transform run-1 output --------------------------------
        out2 = self._tmpdir()
        report2 = AssetTransformerManager().run(
            input_stage=run1_output,
            profile=profile,
            package_root=out2,
        )
        for r in report2.results:
            if not r.success:
                errors.append(f"Run 2 rule '{r.rule.name}' failed: {r.error}")
        if errors:
            self.fail("\n".join(errors))

        # -- Structural comparison -------------------------------------------
        files1 = _collect_layers(out1)
        files2 = _collect_layers(out2)

        only1 = sorted(set(files1) - set(files2))
        only2 = sorted(set(files2) - set(files1))
        if only1:
            errors.append(f"Files only in run 1: {only1}")
        if only2:
            errors.append(f"Files only in run 2: {only2}")

        for rel in sorted(set(files1) & set(files2)):
            t1 = files1[rel]
            t2 = files2[rel]
            if t1 != t2:
                errors.append(_diff_detail(rel, t1, t2))

        # -- Targeted check: no stale payload on any payload-layer default prim
        for out_dir, label in [(out1, "run 1"), (out2, "run 2")]:
            for dirpath, _, filenames in os.walk(out_dir):
                for fn in filenames:
                    if not fn.endswith((".usda", ".usd")):
                        continue
                    abs_path = os.path.join(dirpath, fn)
                    layer = Sdf.Layer.FindOrOpen(abs_path)
                    if layer is None or not layer.defaultPrim:
                        continue
                    prim_spec = layer.GetPrimAtPath(f"/{layer.defaultPrim}")
                    if prim_spec is None or not prim_spec.hasPayloads:
                        continue
                    rel = os.path.relpath(abs_path, out_dir)
                    # Interface layers legitimately carry payload arcs.
                    if rel == os.path.basename(abs_path):
                        continue
                    errors.append(f"{label}: {rel} default prim '/{layer.defaultPrim}' has stale payload arc(s)")

        if errors:
            self.fail("\n".join(errors))

        self._success = True
