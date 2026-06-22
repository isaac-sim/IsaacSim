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

"""Tests for XRDF/Lula I/O modules and the full export pipeline.

This file covers two layers:

* Pure unit tests for :mod:`xrdf_io`, :mod:`lula_io`, and :mod:`yaml_utils` that
  don't require a USD stage. These exercise the public dataclasses and helpers.

* An integration test that loads a real robot USD, populates an
  :class:`EditorState` with collision spheres, exports both URDF and XRDF to
  temporary files, and verifies the result loads cleanly through
  :mod:`isaacsim.robot_motion.cumotion`.
"""

from __future__ import annotations

import asyncio
import os
import tempfile

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.cumotion as cu_mg
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
import yaml
from isaacsim.asset.exporter.urdf.converter import UsdToUrdfConverter
from isaacsim.robot_setup.xrdf_editor import (
    EditorState,
    articulation_discovery,
    is_xrdf_file,
    is_yaml_file,
)
from isaacsim.robot_setup.xrdf_editor.constants import (
    COLLISION_KEY_V1,
    COLLISION_KEY_V2,
    DEFAULT_ACCELERATION_LIMIT,
    DEFAULT_GEOMETRY_GROUP_NAME,
    DEFAULT_JERK_LIMIT,
    XRDF_FORMAT,
    XRDF_VERSION_1,
    XRDF_VERSION_2,
)
from isaacsim.robot_setup.xrdf_editor.lula_io import (
    LulaWriteInputs,
    read_lula_robot_description_file,
    write_lula_robot_description_file,
)
from isaacsim.robot_setup.xrdf_editor.xrdf_io import (
    XrdfWriteInputs,
    build_xrdf_dict,
    collision_key_for_version,
    is_valid_xrdf_file,
    merge_passthrough_dict,
    read_xrdf_file,
    write_xrdf_file,
)
from isaacsim.robot_setup.xrdf_editor.yaml_utils import (
    recursive_cast_to_float,
    safe_load_yaml,
)
from isaacsim.storage.native import get_assets_root_path

_ROBOT_USD = "Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"


def _articulation_base_path(stage: object) -> str | None:
    """Find the first articulation BASE path on ``stage`` and return it.

    Uses :func:`articulation_discovery.find_all_articulation_base_paths` so the
    returned path matches what :class:`EditorState` expects (the maximal
    subtree containing the robot, e.g. ``/ur10e``), not the path of the
    ArticulationRootAPI-bearing prim (which on UR10e is ``/ur10e/root_joint``).

    Args:
        stage: Stage to inspect.

    Returns:
        First articulation base path, or None if none exists.
    """
    paths = articulation_discovery.find_all_articulation_base_paths(stage)
    return paths[0] if paths else None


def _disable_instanceable(stage: object) -> None:
    """Recursively turn off ``instanceable=True`` on every prim of ``stage``.

    UR10e's USD references sub-assets with ``instanceable=True``, but
    :func:`sphere_generation.find_link_meshes` treats instanceable meshes as
    opaque (it can't read their points) and records empty mesh lists for the
    enclosing link. The integration test needs real per-link mesh inventories
    to populate the exported XRDF, so we disable instancing first.

    A single pass isn't enough because hiding the children of an instanceable
    prim only becomes visible after the parent is flipped, so we loop until
    no more instanceable prims remain.

    Args:
        stage: Stage to update.
    """
    while True:
        changed = False
        for prim in stage.Traverse():
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                changed = True
        if not changed:
            return


class TestXrdfIoPure(omni.kit.test.AsyncTestCase):
    """Pure unit tests for the I/O helpers — no USD stage required."""

    # ------------------------------------------------------------------
    # Extension checkers
    # ------------------------------------------------------------------
    async def test_is_xrdf_file_extensions(self) -> None:
        """Test is xrdf file extensions."""
        self.assertTrue(is_xrdf_file("robot.xrdf"))
        self.assertTrue(is_xrdf_file("robot.yaml"))
        self.assertTrue(is_xrdf_file("robot.yml"))
        self.assertTrue(is_xrdf_file("/tmp/Foo.YAML"))
        self.assertFalse(is_xrdf_file("robot.urdf"))
        self.assertFalse(is_xrdf_file("robot.txt"))

    async def test_is_yaml_file_extensions(self) -> None:
        """Test is yaml file extensions."""
        self.assertTrue(is_yaml_file("robot.yaml"))
        self.assertTrue(is_yaml_file("robot.yml"))
        self.assertFalse(is_yaml_file("robot.xrdf"))
        self.assertFalse(is_yaml_file("robot.urdf"))

    # ------------------------------------------------------------------
    # YAML helpers
    # ------------------------------------------------------------------
    async def test_recursive_cast_to_float_handles_strings_and_lists(self) -> None:
        """Test recursive cast to float handles strings and lists."""
        d = {
            "scalar": "1.5",
            "non_numeric": "hello",
            "list": ["2.0", "world", "-3.5"],
            "nested": {"value": "0.25"},
        }
        recursive_cast_to_float(d)
        self.assertEqual(d["scalar"], 1.5)
        self.assertEqual(d["non_numeric"], "hello")
        self.assertEqual(d["list"], [2.0, "world", -3.5])
        self.assertEqual(d["nested"]["value"], 0.25)

    async def test_safe_load_yaml_round_trip(self) -> None:
        """Test safe load yaml round trip."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "doc.yaml")
            with open(path, "w") as f:
                yaml.safe_dump({"a": 1.0, "b": [2.0, 3.0], "c": {"d": "0.5"}}, f)
            loaded = safe_load_yaml(path)
            self.assertEqual(loaded["a"], 1.0)
            self.assertEqual(loaded["b"], [2.0, 3.0])
            self.assertEqual(loaded["c"]["d"], 0.5)

    async def test_safe_load_yaml_handles_missing_path_gracefully(self) -> None:
        """Test safe load yaml handles missing path gracefully."""
        with tempfile.TemporaryDirectory() as tmpdir:
            empty_path = os.path.join(tmpdir, "empty.yaml")
            with open(empty_path, "w") as f:
                f.write("")
            self.assertEqual(safe_load_yaml(empty_path), {})

    async def test_safe_load_yaml_handles_non_mapping_root(self) -> None:
        """A YAML file with a list/scalar root must return ``{}``, not crash.

        Regression: ``recursive_cast_to_float`` assumes a dict and previously
        crashed with ``AttributeError`` when handed a list or scalar root.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            for name, body in (
                ("list_root.yaml", "[1, 2, 3]\n"),
                ("scalar_root.yaml", "just-a-string\n"),
            ):
                path = os.path.join(tmpdir, name)
                with open(path, "w") as f:
                    f.write(body)
                self.assertEqual(safe_load_yaml(path), {}, f"{name} should yield an empty dict")

    # ------------------------------------------------------------------
    # XRDF version helpers
    # ------------------------------------------------------------------
    async def test_collision_key_for_version(self) -> None:
        """Test collision key for version."""
        self.assertEqual(collision_key_for_version(XRDF_VERSION_1), COLLISION_KEY_V1)
        self.assertEqual(collision_key_for_version(XRDF_VERSION_2), COLLISION_KEY_V2)
        with self.assertRaises(ValueError):
            collision_key_for_version(99.0)

    # ------------------------------------------------------------------
    # Round-trip without spheres / merge
    # ------------------------------------------------------------------
    def _build_minimal_inputs(self, path: str, format_version: float = XRDF_VERSION_2) -> XrdfWriteInputs:
        dof_names = ["j0", "j1", "j2"]
        return XrdfWriteInputs(
            path=path,
            format_version=format_version,
            articulation_base_path="/World/robot",
            dof_names=dof_names,
            active_joints_mask=np.array([True, True, False]),
            joint_positions=np.array([0.1, -0.2, 0.0]),
            acceleration_limits=np.array([1.0, 2.0, 3.0]),
            jerk_limits=np.array([100.0, 200.0, 300.0]),
            ordered_links=["base", "link1", "link2"],
            ignore_dict={"base": ["link1"], "link1": ["link2"]},
            sphere_dict_writer=None,  # No real spheres
        )

    async def test_build_xrdf_dict_basic_structure_v2(self) -> None:
        """Test build xrdf dict basic structure v2."""
        inputs = self._build_minimal_inputs("/tmp/never-written.xrdf", XRDF_VERSION_2)
        result = build_xrdf_dict(inputs)
        self.assertEqual(result["format"], XRDF_FORMAT)
        self.assertEqual(result["format_version"], 2.0)
        self.assertIn(COLLISION_KEY_V2, result)
        self.assertNotIn(COLLISION_KEY_V1, result)
        self.assertEqual(result["cspace"]["joint_names"], ["j0", "j1"])  # only active joints
        self.assertEqual(result["cspace"]["acceleration_limits"], [1.0, 2.0])
        self.assertEqual(result["cspace"]["jerk_limits"], [100.0, 200.0])
        self.assertEqual(set(result["default_joint_positions"].keys()), {"j0", "j1", "j2"})

    async def test_build_xrdf_dict_basic_structure_v1(self) -> None:
        """Test build xrdf dict basic structure v1."""
        inputs = self._build_minimal_inputs("/tmp/never-written.xrdf", XRDF_VERSION_1)
        result = build_xrdf_dict(inputs)
        self.assertEqual(result["format_version"], 1.0)
        self.assertIn(COLLISION_KEY_V1, result)
        self.assertNotIn(COLLISION_KEY_V2, result)

    async def test_invalid_format_version_falls_back_to_v2(self) -> None:
        """Test invalid format version falls back to v2."""
        inputs = self._build_minimal_inputs("/tmp/never-written.xrdf", 5.5)
        result = build_xrdf_dict(inputs)
        self.assertEqual(result["format_version"], XRDF_VERSION_2)
        self.assertIn(COLLISION_KEY_V2, result)

    async def test_build_xrdf_dict_merge_existing_without_self_collision(self) -> None:
        """Merging an XRDF that has world_collision + geometry but no self_collision.

        Regression: ``build_xrdf_dict`` used to raise ``KeyError`` accessing
        ``parsed_file["self_collision"]`` after merging, because
        ``merge_passthrough_dict`` only creates ``self_collision`` when it
        already exists in the source. The merge path now mirrors the collision
        geometry group into ``self_collision`` before populating ``ignore``.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            merge_path = os.path.join(tmpdir, "partial.xrdf")
            existing = {
                "format": XRDF_FORMAT,
                "format_version": XRDF_VERSION_2,
                COLLISION_KEY_V2: {"geometry": "preexisting_group"},
                "geometry": {"preexisting_group": {"spheres": {}}},
            }
            with open(merge_path, "w") as f:
                yaml.safe_dump(existing, f)

            inputs = self._build_minimal_inputs(os.path.join(tmpdir, "out.xrdf"), XRDF_VERSION_2)
            inputs.merge_existing = merge_path
            inputs.articulation_frames = {"base", "link1", "link2"}

            result = build_xrdf_dict(inputs)

            self.assertIn("self_collision", result)
            self.assertEqual(result["self_collision"]["geometry"], "preexisting_group")
            self.assertEqual(result["self_collision"]["ignore"], inputs.ignore_dict)
            self.assertEqual(result[COLLISION_KEY_V2]["geometry"], "preexisting_group")
            self.assertNotIn(DEFAULT_GEOMETRY_GROUP_NAME, result["geometry"])

    async def test_write_and_read_round_trip(self) -> None:
        """Test write and read round trip."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "robot.xrdf")
            inputs = self._build_minimal_inputs(path)
            write_xrdf_file(inputs)
            self.assertTrue(os.path.exists(path))

            self.assertTrue(is_valid_xrdf_file(path))

            result = read_xrdf_file(
                path,
                inputs.dof_names,
                default_acceleration_limit=DEFAULT_ACCELERATION_LIMIT,
                default_jerk_limit=DEFAULT_JERK_LIMIT,
            )
            np.testing.assert_array_equal(result.active_joints_mask, inputs.active_joints_mask)
            np.testing.assert_allclose(
                result.acceleration_limits[: len(inputs.dof_names)][result.active_joints_mask],
                inputs.acceleration_limits[: len(inputs.dof_names)][inputs.active_joints_mask],
                atol=1e-3,
            )
            np.testing.assert_allclose(
                result.jerk_limits[: len(inputs.dof_names)][result.active_joints_mask],
                inputs.jerk_limits[: len(inputs.dof_names)][inputs.active_joints_mask],
                atol=1e-3,
            )
            np.testing.assert_allclose(result.joint_positions, inputs.joint_positions, atol=1e-3)

    async def test_is_valid_xrdf_file_rejects_non_xrdf(self) -> None:
        """Test is valid xrdf file rejects non xrdf."""
        with tempfile.TemporaryDirectory() as tmpdir:
            not_xrdf = os.path.join(tmpdir, "robot.yaml")
            with open(not_xrdf, "w") as f:
                yaml.safe_dump({"hello": "world"}, f)
            self.assertFalse(is_valid_xrdf_file(not_xrdf))

            missing = os.path.join(tmpdir, "does_not_exist.xrdf")
            self.assertFalse(is_valid_xrdf_file(missing))

    async def test_read_xrdf_file_raises_on_missing_format(self) -> None:
        """Test read xrdf file raises on missing format."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "bad.xrdf")
            with open(path, "w") as f:
                yaml.safe_dump({"format_version": 2.0, "cspace": {}}, f)
            with self.assertRaises(ValueError):
                read_xrdf_file(
                    path,
                    ["j0"],
                    default_acceleration_limit=DEFAULT_ACCELERATION_LIMIT,
                    default_jerk_limit=DEFAULT_JERK_LIMIT,
                )

    # ------------------------------------------------------------------
    # Full-payload reads for v1 / v2 / Lula YAML
    # ------------------------------------------------------------------
    @staticmethod
    def _full_xrdf_payload(format_version: float, *, collision_key: str) -> dict:
        """Build a complete XRDF dict with collision, geometry, and self-collision.

        Args:
            format_version: XRDF format version to encode.
            collision_key: Top-level collision key to include.

        Returns:
            Complete XRDF payload.
        """
        return {
            "format": XRDF_FORMAT,
            "format_version": float(format_version),
            "default_joint_positions": {"j0": 0.5, "j1": -0.25, "j2": 1.0},
            "cspace": {
                "joint_names": ["j0", "j1"],  # j2 is intentionally inactive
                "acceleration_limits": [7.0, 8.0],
                "jerk_limits": [700.0, 800.0],
            },
            collision_key: {"geometry": "test_group"},
            "geometry": {
                "test_group": {
                    "spheres": {
                        "link0": [{"center": [0.0, 0.0, 0.1], "radius": 0.05}],
                        "link1": [
                            {"center": [0.1, 0.0, 0.0], "radius": 0.04},
                            {"center": [0.2, 0.0, 0.0], "radius": 0.03},
                        ],
                    }
                }
            },
            "self_collision": {
                "geometry": "test_group",
                "ignore": {"link0": ["link1"], "link1": ["link2"]},
            },
        }

    def _assert_full_xrdf_round_trip(self, format_version: float, collision_key: str) -> None:
        """Run shared assertions for the v1 and v2 read-from-disk round-trip tests.

        Args:
            format_version: XRDF format version to test.
            collision_key: Top-level collision key expected for the version.
        """
        dof_names = ["j0", "j1", "j2"]
        payload = self._full_xrdf_payload(format_version, collision_key=collision_key)

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "robot.xrdf")
            with open(path, "w") as f:
                yaml.safe_dump(payload, f)

            self.assertTrue(is_valid_xrdf_file(path))

            result = read_xrdf_file(
                path,
                dof_names,
                default_acceleration_limit=DEFAULT_ACCELERATION_LIMIT,
                default_jerk_limit=DEFAULT_JERK_LIMIT,
            )

        # --- Projected state (the XrdfReadResult fields) ---
        np.testing.assert_array_equal(result.active_joints_mask, np.array([True, True, False]))

        np.testing.assert_allclose(
            result.acceleration_limits[result.active_joints_mask], np.array([7.0, 8.0]), atol=1e-6
        )
        np.testing.assert_allclose(
            result.acceleration_limits[~result.active_joints_mask],
            np.array([DEFAULT_ACCELERATION_LIMIT]),
            atol=1e-6,
        )

        np.testing.assert_allclose(result.jerk_limits[result.active_joints_mask], np.array([700.0, 800.0]), atol=1e-6)
        np.testing.assert_allclose(
            result.jerk_limits[~result.active_joints_mask], np.array([DEFAULT_JERK_LIMIT]), atol=1e-6
        )

        np.testing.assert_allclose(result.joint_positions, np.array([0.5, -0.25, 1.0]), atol=1e-6)

        # --- Raw parsed_file payload survives verbatim (no projection / mutation) ---
        parsed = result.parsed_file
        self.assertEqual(parsed["format"], XRDF_FORMAT)
        self.assertEqual(parsed["format_version"], float(format_version))
        self.assertIn(collision_key, parsed)
        self.assertEqual(parsed[collision_key]["geometry"], "test_group")

        spheres = parsed["geometry"]["test_group"]["spheres"]
        self.assertEqual(set(spheres.keys()), {"link0", "link1"})
        self.assertEqual(len(spheres["link0"]), 1)
        self.assertEqual(len(spheres["link1"]), 2)
        np.testing.assert_allclose(spheres["link0"][0]["center"], [0.0, 0.0, 0.1], atol=1e-6)
        self.assertAlmostEqual(spheres["link0"][0]["radius"], 0.05, places=6)
        np.testing.assert_allclose(spheres["link1"][1]["center"], [0.2, 0.0, 0.0], atol=1e-6)
        self.assertAlmostEqual(spheres["link1"][1]["radius"], 0.03, places=6)

        self.assertEqual(
            parsed["self_collision"]["ignore"],
            {"link0": ["link1"], "link1": ["link2"]},
        )
        self.assertEqual(parsed["self_collision"]["geometry"], "test_group")

    async def test_read_xrdf_file_v1_returns_full_payload(self) -> None:
        """Read a v1 XRDF from disk and verify every section round-trips."""
        self._assert_full_xrdf_round_trip(XRDF_VERSION_1, COLLISION_KEY_V1)

    async def test_read_xrdf_file_v2_returns_full_payload(self) -> None:
        """Read a v2 XRDF from disk and verify every section round-trips."""
        self._assert_full_xrdf_round_trip(XRDF_VERSION_2, COLLISION_KEY_V2)

    async def test_read_lula_robot_description_file_returns_full_payload(self) -> None:
        """Read a Lula robot-description YAML and verify the projected state.

        Lula's reader returns a narrower :class:`LulaReadResult` (no parsed
        dict), so the assertions cover the projected state and the
        ``cspace_to_urdf_rules`` fixed-joint semantics: a fixed joint must
        end up inactive but with its position pinned to the rule's value.
        """
        dof_names = ["j0", "j1", "j2"]
        # j0 + j1 active via cspace; j2 fixed via cspace_to_urdf_rules.
        payload = {
            "api_version": 1.0,
            "cspace": ["j0", "j1"],
            "default_q": [0.5, -0.25],
            "acceleration_limits": [7.0, 8.0],
            "jerk_limits": [700.0, 800.0],
            "cspace_to_urdf_rules": [
                {"name": "j2", "rule": "fixed", "value": 1.0},
            ],
            "collision_spheres": [
                {"link0": [{"center": [0.0, 0.0, 0.1], "radius": 0.05}]},
                {"link1": [{"center": [0.1, 0.0, 0.0], "radius": 0.04}]},
            ],
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "robot.yaml")
            with open(path, "w") as f:
                yaml.safe_dump(payload, f)

            result = read_lula_robot_description_file(
                path,
                dof_names,
                default_acceleration_limit=DEFAULT_ACCELERATION_LIMIT,
                default_jerk_limit=DEFAULT_JERK_LIMIT,
            )

        np.testing.assert_array_equal(result.active_joints_mask, np.array([True, True, False]))

        np.testing.assert_allclose(result.acceleration_limits[:2], np.array([7.0, 8.0]), atol=1e-6)
        np.testing.assert_allclose(result.acceleration_limits[2:], np.array([DEFAULT_ACCELERATION_LIMIT]), atol=1e-6)

        np.testing.assert_allclose(result.jerk_limits[:2], np.array([700.0, 800.0]), atol=1e-6)
        np.testing.assert_allclose(result.jerk_limits[2:], np.array([DEFAULT_JERK_LIMIT]), atol=1e-6)

        # Fixed joints from cspace_to_urdf_rules: inactive, but position pinned.
        np.testing.assert_allclose(result.joint_positions, np.array([0.5, -0.25, 1.0]), atol=1e-6)

    # ------------------------------------------------------------------
    # Defensive guards: non-mapping YAML roots, malformed merge inputs
    # ------------------------------------------------------------------
    async def test_is_valid_xrdf_file_rejects_non_mapping_yaml_root(self) -> None:
        """A YAML file whose root is a list or scalar must be rejected, not crash."""
        with tempfile.TemporaryDirectory() as tmpdir:
            for name, body in (
                ("list_root.xrdf", "[1, 2, 3]\n"),
                ("scalar_root.xrdf", "just-a-string\n"),
                ("empty.xrdf", ""),
            ):
                path = os.path.join(tmpdir, name)
                with open(path, "w") as f:
                    f.write(body)
                # Must return False rather than raise AttributeError on .get().
                self.assertFalse(is_valid_xrdf_file(path), f"{name} should not be a valid XRDF")

    async def test_read_xrdf_file_raises_clean_error_on_non_mapping_root(self) -> None:
        """A list/scalar-rooted YAML must surface a ValueError (no AttributeError)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "list_root.xrdf")
            with open(path, "w") as f:
                f.write("[1, 2, 3]\n")
            with self.assertRaises(ValueError):
                read_xrdf_file(
                    path,
                    ["j0"],
                    default_acceleration_limit=DEFAULT_ACCELERATION_LIMIT,
                    default_jerk_limit=DEFAULT_JERK_LIMIT,
                )

    async def test_merge_passthrough_preserves_self_collision_ignore_when_geometry_renamed(self) -> None:
        """Regression: changing the geometry group name during merge must keep ``ignore``.

        Previously merge_passthrough_dict replaced the whole self_collision block
        with ``{"geometry": ...}`` when self_collision.geometry differed from
        collision.geometry, dropping the ``ignore`` (and ``buffer_distance``)
        rules that the helper is explicitly supposed to preserve.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "renamed_geometry.xrdf")
            content = {
                "format": XRDF_FORMAT,
                "format_version": XRDF_VERSION_2,
                COLLISION_KEY_V2: {"geometry": "world_group"},
                "geometry": {"world_group": {"spheres": {}}},
                "self_collision": {
                    "geometry": "self_group",  # Different name from world_group
                    "ignore": {"link0": ["link1"], "link1": ["link2"]},
                    "buffer_distance": {"link0": 0.01},
                },
            }
            with open(path, "w") as f:
                yaml.safe_dump(content, f)

            result = merge_passthrough_dict(path, articulation_frames=set())

        self.assertIn("self_collision", result)
        # Geometry pointer was rewritten to match collision_key.geometry.
        self.assertEqual(result["self_collision"]["geometry"], "world_group")
        # ...but preserved fields survived the rewrite.
        self.assertEqual(
            result["self_collision"]["ignore"],
            {"link0": ["link1"], "link1": ["link2"]},
        )
        self.assertEqual(result["self_collision"]["buffer_distance"], {"link0": 0.01})

    async def test_merge_passthrough_normalises_missing_geometry_group(self) -> None:
        """A file with collision.geometry pointing at a non-existent group must not crash."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "dangling_group.xrdf")
            content = {
                "format": XRDF_FORMAT,
                "format_version": XRDF_VERSION_2,
                # collision_key references a group name not present in the
                # top-level geometry dict.
                COLLISION_KEY_V2: {"geometry": "missing_group"},
                "geometry": {"some_other_group": {"spheres": {}}},
                "self_collision": {
                    "geometry": "missing_group",
                    "ignore": {"link0": ["link1"]},
                },
            }
            with open(path, "w") as f:
                yaml.safe_dump(content, f)

            result = merge_passthrough_dict(path, articulation_frames=set())

        # Half-populated geometry was dropped so build_xrdf_dict() can re-seed.
        self.assertNotIn(COLLISION_KEY_V2, result)
        self.assertNotIn("geometry", result)
        self.assertNotIn("self_collision", result)

    async def test_merge_passthrough_normalises_missing_top_level_geometry(self) -> None:
        """A file with collision_key but no top-level geometry dict must not crash."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "no_top_geometry.xrdf")
            content = {
                "format": XRDF_FORMAT,
                "format_version": XRDF_VERSION_2,
                COLLISION_KEY_V2: {"geometry": "some_group"},
                # No top-level "geometry" key at all.
            }
            with open(path, "w") as f:
                yaml.safe_dump(content, f)

            result = merge_passthrough_dict(path, articulation_frames=set())

        self.assertNotIn(COLLISION_KEY_V2, result)
        self.assertNotIn("geometry", result)

    async def test_merge_passthrough_mirrors_self_collision_geometry_when_missing(self) -> None:
        """``self_collision`` present without a ``geometry`` sub-key must get one mirrored from collision.

        Regression: previously the rewrite-in-place branch only fired when
        ``self_collision.geometry`` already existed, so a partially-malformed
        merge source kept ``self_collision`` without a geometry pointer.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "self_collision_no_geometry.xrdf")
            content = {
                "format": XRDF_FORMAT,
                "format_version": XRDF_VERSION_2,
                COLLISION_KEY_V2: {"geometry": "shared_group"},
                "geometry": {"shared_group": {"spheres": {}}},
                # self_collision has ignore rules but no geometry pointer.
                "self_collision": {"ignore": {"link0": ["link1"]}},
            }
            with open(path, "w") as f:
                yaml.safe_dump(content, f)

            result = merge_passthrough_dict(path, articulation_frames=set())

        self.assertIn("self_collision", result)
        self.assertEqual(result["self_collision"]["geometry"], "shared_group")
        # Preserved field survived the mirror.
        self.assertEqual(result["self_collision"]["ignore"], {"link0": ["link1"]})

    async def test_merge_passthrough_renames_v1_collision_to_v2(self) -> None:
        """A merge source using the v1 ``collision`` key must be normalised to ``world_collision``.

        Internally :func:`merge_passthrough_dict` works in v2-space; the v1 key
        rename branch had no coverage even though every supported file format
        flows through it.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "v1.xrdf")
            content = {
                "format": XRDF_FORMAT,
                "format_version": XRDF_VERSION_1,
                COLLISION_KEY_V1: {"geometry": "v1_group"},
                "geometry": {"v1_group": {"spheres": {"link0": [{"center": [0.0, 0.0, 0.0], "radius": 0.01}]}}},
                "self_collision": {"geometry": "v1_group", "ignore": {"link0": ["link1"]}},
            }
            with open(path, "w") as f:
                yaml.safe_dump(content, f)

            result = merge_passthrough_dict(path, articulation_frames=set())

        # v1 key was renamed; v2 key now holds the geometry pointer.
        self.assertIn(COLLISION_KEY_V2, result)
        self.assertNotIn(COLLISION_KEY_V1, result)
        self.assertEqual(result[COLLISION_KEY_V2]["geometry"], "v1_group")
        # Geometry payload and self_collision survived intact.
        self.assertEqual(
            result["geometry"]["v1_group"]["spheres"],
            {"link0": [{"center": [0.0, 0.0, 0.0], "radius": 0.01}]},
        )
        self.assertEqual(result["self_collision"]["ignore"], {"link0": ["link1"]})

    async def test_merge_passthrough_buffer_distance_reconciliation(self) -> None:
        """world_collision buffer distances must be zeroed and the offset moved to self_collision.

        Covers four behaviours that previously had no test:

        * frames inside ``articulation_frames`` are zeroed in world_collision
          and decremented in self_collision (preserving relative buffer);
        * frames inside ``articulation_frames`` with no self_collision entry
          get an inverted entry written *back into the parsed dict* (not into
          a throw-away local copy);
        * frames outside ``articulation_frames`` are left untouched in both;
        * self_collision without any pre-existing ``buffer_distance`` block
          still receives the new offsets (regression for a ``.get(..., {})``
          / ``setdefault(...)`` mismatch).
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            # --- Case A: self_collision already has a buffer_distance dict ---
            path_a = os.path.join(tmpdir, "with_existing_bd.xrdf")
            with open(path_a, "w") as f:
                yaml.safe_dump(
                    {
                        "format": XRDF_FORMAT,
                        "format_version": XRDF_VERSION_2,
                        COLLISION_KEY_V2: {
                            "geometry": "g",
                            "buffer_distance": {
                                "link_in": 0.10,
                                "link_in_only_world": 0.05,
                                "link_external": 0.20,
                            },
                        },
                        "geometry": {"g": {"spheres": {}}},
                        "self_collision": {
                            "geometry": "g",
                            "buffer_distance": {"link_in": 0.30, "link_only_self": 0.40},
                        },
                    },
                    f,
                )

            result_a = merge_passthrough_dict(path_a, articulation_frames={"link_in", "link_in_only_world"})

            world_bd_a = result_a[COLLISION_KEY_V2]["buffer_distance"]
            self_bd_a = result_a["self_collision"]["buffer_distance"]

            # Frames inside articulation_frames are zeroed in world_collision.
            self.assertAlmostEqual(world_bd_a["link_in"], 0.0, places=6)
            self.assertAlmostEqual(world_bd_a["link_in_only_world"], 0.0, places=6)
            # Frames outside articulation_frames are untouched in world_collision.
            self.assertAlmostEqual(world_bd_a["link_external"], 0.20, places=6)

            # self_collision gets the inverted offset for in-set frames.
            self.assertAlmostEqual(self_bd_a["link_in"], 0.30 - 0.10, places=6)
            # in-set frame missing from self_collision gets -world value.
            self.assertAlmostEqual(self_bd_a["link_in_only_world"], -0.05, places=6)
            # Pre-existing self-only entries survive unchanged.
            self.assertAlmostEqual(self_bd_a["link_only_self"], 0.40, places=6)
            # Out-of-set frames are not written into self_collision.
            self.assertNotIn("link_external", self_bd_a)

            # --- Case B: self_collision has NO buffer_distance dict ---
            # Regression for the ``.get(..., {})`` discard bug.
            path_b = os.path.join(tmpdir, "no_existing_bd.xrdf")
            with open(path_b, "w") as f:
                yaml.safe_dump(
                    {
                        "format": XRDF_FORMAT,
                        "format_version": XRDF_VERSION_2,
                        COLLISION_KEY_V2: {
                            "geometry": "g",
                            "buffer_distance": {"link_in": 0.07},
                        },
                        "geometry": {"g": {"spheres": {}}},
                        "self_collision": {"geometry": "g"},  # no buffer_distance
                    },
                    f,
                )

            result_b = merge_passthrough_dict(path_b, articulation_frames={"link_in"})

            self.assertAlmostEqual(result_b[COLLISION_KEY_V2]["buffer_distance"]["link_in"], 0.0, places=6)
            self.assertIn(
                "buffer_distance",
                result_b["self_collision"],
                "self_collision.buffer_distance must be created when new offsets are written",
            )
            self.assertAlmostEqual(result_b["self_collision"]["buffer_distance"]["link_in"], -0.07, places=6)

    # ------------------------------------------------------------------
    # Mimic-joint filtering
    # ------------------------------------------------------------------
    async def test_build_xrdf_dict_omits_mimic_joints_from_default_positions(self) -> None:
        """Mimic-follower joints must not appear in ``default_joint_positions``.

        Regression: cuMotion's ``load_robot_from_memory`` rejects independent
        defaults for joints with ``PhysxSchema.PhysxMimicJointAPI`` applied
        because it derives their positions from the URDF ``<mimic>`` element.
        """
        inputs = self._build_minimal_inputs("/tmp/never-written.xrdf", XRDF_VERSION_2)
        inputs.mimic_joint_names = {"j1"}

        result = build_xrdf_dict(inputs)

        self.assertEqual(set(result["default_joint_positions"].keys()), {"j0", "j2"})
        self.assertNotIn("j1", result["default_joint_positions"])

    async def test_build_xrdf_dict_omits_mimic_joints_from_cspace(self) -> None:
        """Mimic-follower joints must not appear in ``cspace.joint_names`` even when active.

        cuMotion treats every joint in ``cspace.joint_names`` as directly
        controllable. A mimic follower cannot be controlled directly, so it
        must be filtered out regardless of the user's active/fixed toggle.
        """
        inputs = self._build_minimal_inputs("/tmp/never-written.xrdf", XRDF_VERSION_2)
        # Mark `j1` as active in the mask AND as a mimic joint; the mimic
        # filter must win and exclude `j1` from cspace entirely.
        inputs.active_joints_mask = np.array([True, True, True])
        inputs.mimic_joint_names = {"j1"}

        result = build_xrdf_dict(inputs)

        cspace = result["cspace"]
        self.assertEqual(cspace["joint_names"], ["j0", "j2"])
        # acceleration / jerk lists must shrink in lock-step with joint_names.
        self.assertEqual(len(cspace["acceleration_limits"]), len(cspace["joint_names"]))
        self.assertEqual(len(cspace["jerk_limits"]), len(cspace["joint_names"]))
        # The per-joint values that survived must be `j0` (index 0) and `j2`
        # (index 2) from the original input arrays.
        self.assertAlmostEqual(cspace["acceleration_limits"][0], 1.0, places=6)
        self.assertAlmostEqual(cspace["acceleration_limits"][1], 3.0, places=6)
        self.assertAlmostEqual(cspace["jerk_limits"][0], 100.0, places=6)
        self.assertAlmostEqual(cspace["jerk_limits"][1], 300.0, places=6)

    async def test_build_xrdf_dict_with_empty_mimic_set_preserves_behaviour(self) -> None:
        """Passing an empty ``mimic_joint_names`` must reproduce the pre-mimic behaviour."""
        inputs = self._build_minimal_inputs("/tmp/never-written.xrdf", XRDF_VERSION_2)
        inputs.mimic_joint_names = set()

        result = build_xrdf_dict(inputs)

        self.assertEqual(set(result["default_joint_positions"].keys()), {"j0", "j1", "j2"})
        self.assertEqual(result["cspace"]["joint_names"], ["j0", "j1"])

    async def test_write_lula_robot_description_omits_mimic_joints(self) -> None:
        """Mimic-follower joints must appear in neither ``cspace`` nor ``cspace_to_urdf_rules``.

        Lula derives the mimic-follower's position from the URDF ``<mimic>``
        relationship; emitting an explicit ``fixed`` rule here would override
        the URDF and break the gear ratio.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "robot.yaml")
            # Three DOFs: j0 active, j1 mimic (active in mask but should still
            # be filtered out), j2 fixed.

            class _NoSpheres:
                def save_spheres(self, _base_path: object, _f: object) -> None:
                    return None

            inputs = LulaWriteInputs(
                path=path,
                articulation_base_path="/World/robot",
                dof_names=["j0", "j1", "j2"],
                active_joints_mask=np.array([True, True, False]),
                joint_positions=np.array([0.1, 0.5, -0.25]),
                acceleration_limits=np.array([1.0, 2.0, 3.0]),
                jerk_limits=np.array([100.0, 200.0, 300.0]),
                collision_sphere_editor=_NoSpheres(),
                mimic_joint_names={"j1"},
            )
            write_lula_robot_description_file(inputs)

            with open(path) as f:
                contents = f.read()

        self.assertIn("- j0", contents)
        self.assertNotIn("- j1", contents)
        # `j2` is a fixed-rule entry, not a cspace entry — different syntax.
        self.assertIn("name: j2", contents)
        # `j1` must not appear as either a cspace entry OR a fixed rule.
        self.assertNotIn("name: j1", contents)

    async def test_find_mimic_joint_names_detects_physx_and_newton_schemas(self) -> None:
        """``find_mimic_joint_names`` must detect both legacy and current mimic schemas.

        * ``PhysxSchema.PhysxMimicJointAPI`` (multi-apply): authored by
          assets imported before Isaac Sim 5.0 / available on bundled USD
          robots that have not yet been re-imported.
        * ``NewtonMimicAPI`` (single-apply): the schema the current URDF
          importer authors exclusively (CHANGELOG ``isaacsim.asset.importer.urdf``
          3.10.0). Detected by string name to avoid importing the binding.

        Without the second branch any robot imported via the modern URDF path
        would silently slip through the mimic filter and trigger the cuMotion
        ``"joint is a mimic joint controlled by an auxiliary c-space coordinate"``
        load failure that motivated this filter in the first place.
        """
        from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/robot")
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/World/robot"))
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath("/World/robot"))

        # Plain revolute joint — not mimic, must be excluded from the result.
        plain = UsdPhysics.RevoluteJoint.Define(stage, "/World/robot/plain_joint")
        plain.CreateAxisAttr("X")

        # Mimic joint via the legacy PhysX multi-apply schema. Both axis
        # instances (rotX/rotZ) should be enough for HasAPI() to fire.
        physx_mimic = UsdPhysics.RevoluteJoint.Define(stage, "/World/robot/physx_mimic_joint")
        physx_mimic.CreateAxisAttr("Z")
        physx_mimic_api = PhysxSchema.PhysxMimicJointAPI.Apply(physx_mimic.GetPrim(), "rotZ")
        physx_mimic_api.GetReferenceJointRel().AddTarget(plain.GetPrim().GetPath())
        physx_mimic_api.GetGearingAttr().Set(1.0)

        # Mimic joint via the current Newton single-apply schema. The schema
        # may not be registered as a Python type in every build, so we apply
        # it by string name (matching the helper's detection logic).
        newton_mimic = UsdPhysics.RevoluteJoint.Define(stage, "/World/robot/newton_mimic_joint")
        newton_mimic.CreateAxisAttr("Z")
        newton_mimic.GetPrim().ApplyAPI("NewtonMimicAPI")
        newton_mimic.GetPrim().CreateAttribute("newton:mimicCoef1", Sdf.ValueTypeNames.Float).Set(1.0)
        newton_mimic.GetPrim().CreateAttribute("newton:mimicCoef0", Sdf.ValueTypeNames.Float).Set(0.0)
        newton_mimic.GetPrim().CreateRelationship("newton:mimicJoint").AddTarget(plain.GetPrim().GetPath())

        # Joint living outside the articulation subtree must be ignored even if
        # it carries the mimic API.
        stray = UsdPhysics.RevoluteJoint.Define(stage, "/World/stray_mimic_joint")
        stray.CreateAxisAttr("X")
        PhysxSchema.PhysxMimicJointAPI.Apply(stray.GetPrim(), "rotX")

        names = articulation_discovery.find_mimic_joint_names(stage, "/World/robot")
        self.assertEqual(names, {"physx_mimic_joint", "newton_mimic_joint"})

        # Defensive arg handling: None stage / empty path return empty set.
        self.assertEqual(articulation_discovery.find_mimic_joint_names(None, "/World/robot"), set())
        self.assertEqual(articulation_discovery.find_mimic_joint_names(stage, ""), set())
        self.assertEqual(articulation_discovery.find_mimic_joint_names(stage, "/does/not/exist"), set())

    async def test_write_lula_robot_description_raises_when_only_mimic_active(self) -> None:
        """If every active joint is a mimic follower, the writer must raise.

        After filtering, no DOFs remain in cspace, which is the same failure
        mode as exporting with no active joints at all — the existing guard
        rail in :func:`write_lula_robot_description_file` already covers it.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "robot.yaml")

            class _NoSpheres:
                def save_spheres(self, _base_path: object, _f: object) -> None:
                    return None

            inputs = LulaWriteInputs(
                path=path,
                articulation_base_path="/World/robot",
                dof_names=["j0"],
                active_joints_mask=np.array([True]),
                joint_positions=np.array([0.5]),
                acceleration_limits=np.array([1.0]),
                jerk_limits=np.array([100.0]),
                collision_sphere_editor=_NoSpheres(),
                mimic_joint_names={"j0"},
            )
            with self.assertRaises(ValueError):
                write_lula_robot_description_file(inputs)


class TestXrdfPipelineIntegration(omni.kit.test.AsyncTestCase):
    """End-to-end pipeline test: USD -> URDF + XRDF -> cuMotion load."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up test fixtures."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        if self._timeline and self._timeline.is_playing():
            self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def _open_robot(self, robot_path: str) -> object | None:
        assets_root = get_assets_root_path()
        if assets_root is None:
            return None
        full_path = os.path.join(assets_root + "/", robot_path)
        await stage_utils.open_stage_async(full_path)
        return stage_utils.get_current_stage()

    async def _ensure_articulation_initialized(self) -> None:
        """Play the timeline briefly so physics initialises the articulation view."""
        self._timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

    async def test_full_xrdf_export_and_cumotion_load(self) -> None:
        """Open UR10e, populate state, export URDF + XRDF, then load via cuMotion."""
        stage = await self._open_robot(_ROBOT_USD)
        if stage is None:
            self.skipTest("Could not access asset server to open UR10e")
            return

        _disable_instanceable(stage)
        await omni.kit.app.get_app().next_update_async()

        await self._ensure_articulation_initialized()

        articulation_base_path = _articulation_base_path(stage)
        self.assertIsNotNone(articulation_base_path, "UR10e USD should contain an articulation root")

        # NOTE: UR10e does not author `urdf:limit:effort` on its joints, so the
        # exported URDF lacks the `effort` attribute on `<limit>` elements.
        # `load_cumotion_robot` now normalizes the URDF in-memory before handing
        # it to urdfdom (see `isaacsim.robot_motion.cumotion.urdf_normalize`),
        # so the test no longer needs to synthesize effort limits on the USD.

        state = EditorState()
        try:
            state.select_articulation(articulation_base_path)
            self.assertIsNotNone(state.articulation)
            self.assertGreater(state.num_dof, 0)

            # Mark every DOF as active with default limits so the cspace block is populated.
            state.active_joints[:] = True
            state.acceleration_limits[:] = DEFAULT_ACCELERATION_LIMIT
            state.jerk_limits[:] = DEFAULT_JERK_LIMIT

            # Add collision spheres to at least one link so the exported XRDF carries geometry.
            link_subpaths = list(state.link_to_meshes.keys())
            self.assertTrue(link_subpaths, "Expected at least one link with meshes under UR10e")
            target_link_path = state.articulation_base_path + link_subpaths[0]
            for translation in (
                np.array([0.0, 0.0, 0.05]),
                np.array([0.0, 0.0, 0.10]),
                np.array([0.0, 0.0, 0.15]),
            ):
                state.collision_sphere_editor.add_sphere(target_link_path, translation, 0.05)

            with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmpdir:
                urdf_path = os.path.join(tmpdir, "robot.urdf")
                xrdf_path = os.path.join(tmpdir, "robot.xrdf")

                # Export URDF directly via the converter (matches xrdf_export's role).
                converter = UsdToUrdfConverter(
                    stage=stage,
                    root_prim_path=articulation_base_path,
                    mesh_dir_name="meshes",
                    mesh_path_prefix="./",
                )
                converter.convert(urdf_path)
                self.assertTrue(os.path.exists(urdf_path), "URDF was not produced")

                # Export XRDF via the refactored API.
                state.export_xrdf(xrdf_path, format_version=XRDF_VERSION_2)
                self.assertTrue(os.path.exists(xrdf_path), "XRDF was not produced")
                self.assertTrue(is_valid_xrdf_file(xrdf_path))

                # Sanity-check the XRDF carries our spheres + DOFs.
                parsed = safe_load_yaml(xrdf_path)
                self.assertEqual(parsed["format"], XRDF_FORMAT)
                self.assertEqual(parsed["format_version"], XRDF_VERSION_2)
                self.assertIn(COLLISION_KEY_V2, parsed)
                cspace_names = parsed["cspace"]["joint_names"]
                self.assertEqual(set(cspace_names), set(state.dof_names))

                # Finally, load the configuration through cuMotion.
                robot = cu_mg.load_cumotion_robot(
                    directory=tmpdir, urdf_filename="robot.urdf", xrdf_filename="robot.xrdf"
                )
                self.assertIsNotNone(robot)
                self.assertEqual(set(robot.controlled_joint_names), set(state.dof_names))
        finally:
            state.on_shutdown()
