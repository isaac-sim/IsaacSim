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

"""Verify portable gripper definitions without contacting an assets server."""

from __future__ import annotations

import shutil
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import AsyncMock, patch

import omni.kit.test
import yaml
from isaacsim.replicator.teleop import resolve_floating_gripper_asset_path_async as resolve_asset
from isaacsim.replicator.teleop import scripted_floating_gripper as grippers
from isaacsim.replicator.teleop._asset_paths import _resolve_definition_grasp_config_path

_STORAGE = "isaacsim.storage.native"


class TestGripperAssetPaths(omni.kit.test.AsyncTestCase):
    """Cover resolution precedence, offline use, and failure diagnostics."""

    async def test_assets_root_fragments_keep_legacy_meaning(self) -> None:
        """Both fragment spellings resolve under the explicitly supplied root."""
        with patch(f"{_STORAGE}.get_assets_root_path_async", new_callable=AsyncMock) as root:
            for asset in ("/Isaac/Robots/gripper.usd", "Isaac/Robots/gripper.usd"):
                for base in ("omniverse://server/pack/", "https://host/pack/"):
                    result = await resolve_asset(asset, assets_root=base, verify=False)
                    self.assertEqual(result, base + "Isaac/Robots/gripper.usd")
            root.assert_not_awaited()

    async def test_configured_root_is_resolved_on_demand(self) -> None:
        """Fragments use the public storage root and verification APIs."""
        with (
            patch(
                f"{_STORAGE}.get_assets_root_path_async", new=AsyncMock(return_value="omniverse://server/pack")
            ) as root,
            patch(f"{_STORAGE}.path_exists", new=AsyncMock(return_value=True)) as exists,
        ):
            result = await resolve_asset("/Isaac/Robots/gripper.usd")
            self.assertEqual(result, "omniverse://server/pack/Isaac/Robots/gripper.usd")
            root.assert_awaited_once()
            exists.assert_awaited_once_with(result)

    async def test_local_asset_pack_root(self) -> None:
        """A fragment also resolves under a local asset-pack directory."""
        with TemporaryDirectory() as directory:
            asset = Path(directory) / "Isaac/Robots/gripper.usda"
            asset.parent.mkdir(parents=True)
            asset.touch()
            result = await resolve_asset("/Isaac/Robots/gripper.usda", assets_root=directory, verify=False)
            self.assertEqual(Path(result), asset)

    async def test_uris_preserved_without_assets_root(self) -> None:
        """Explicit URIs never trigger asset-root discovery, including custom providers."""
        with (
            patch(f"{_STORAGE}.get_assets_root_path_async", new=AsyncMock(side_effect=AssertionError("Root queried"))),
            patch(f"{_STORAGE}.path_exists", new=AsyncMock(return_value=True)) as exists,
        ):
            for uri in (
                "omniverse://server/gripper.usda",
                "https://host/gripper.usd",
                "file:///data/a.usda",
                "custom+usd://a/b",
            ):
                self.assertEqual(await resolve_asset(uri, base_dir="/unused"), uri)
                exists.assert_awaited_with(uri)

    async def test_absolute_local_file_needs_no_assets_root(self) -> None:
        """Existing local files remain absolute even when the configured root is unavailable."""
        with TemporaryDirectory() as directory:
            asset = Path(directory) / "gripper with spaces.usda"
            asset.touch()
            with (
                patch(
                    f"{_STORAGE}.get_assets_root_path_async", new=AsyncMock(side_effect=AssertionError("Root queried"))
                ),
                patch(f"{_STORAGE}.path_exists", new=AsyncMock(return_value=True)) as exists,
            ):
                self.assertEqual(await resolve_asset(str(asset)), str(asset))
                exists.assert_awaited_once_with(str(asset))

    async def test_definition_relative_file_takes_precedence(self) -> None:
        """An existing sibling wins over a supplied assets root."""
        with TemporaryDirectory() as directory:
            asset = Path(directory) / "overlays/gripper.usda"
            asset.parent.mkdir()
            asset.touch()
            with patch(
                f"{_STORAGE}.get_assets_root_path_async", new=AsyncMock(side_effect=AssertionError("Root queried"))
            ):
                self.assertEqual(
                    await resolve_asset(
                        "overlays/gripper.usda", base_dir=directory, assets_root="https://unused", verify=False
                    ),
                    str(asset),
                )

    async def test_missing_relative_file_falls_back_to_assets_root(self) -> None:
        """Missing definition-local files retain the fragment fallback."""
        with TemporaryDirectory() as directory:
            self.assertEqual(
                await resolve_asset(
                    "Isaac/Robots/g.usd", base_dir=directory, assets_root="https://host/pack", verify=False
                ),
                "https://host/pack/Isaac/Robots/g.usd",
            )

    async def test_missing_absolute_file_keeps_fragment_fallback(self) -> None:
        """An unavailable absolute file follows the documented assets-root convention."""
        with TemporaryDirectory() as directory:
            missing = str(Path(directory) / "missing.usda")
            resolved = await resolve_asset(missing, assets_root="https://host/pack", verify=False)
            self.assertEqual(resolved, "https://host/pack/" + Path(missing).as_posix().lstrip("/"))

    async def test_file_uri_failure_does_not_fall_back(self) -> None:
        """A file URI explicitly requires its authored local location."""
        with (
            patch(f"{_STORAGE}.get_assets_root_path_async", new=AsyncMock(side_effect=AssertionError("Root queried"))),
            patch(f"{_STORAGE}.path_exists", new=AsyncMock(return_value=False)),
        ):
            with self.assertRaisesRegex(ValueError, "file:///missing/gripper.usda"):
                await resolve_asset("file:///missing/gripper.usda")

    async def test_missing_asset_error_lists_both_locations(self) -> None:
        """Errors identify the local candidate and assets-root candidate."""
        with TemporaryDirectory() as directory, patch(f"{_STORAGE}.path_exists", new=AsyncMock(return_value=False)):
            with self.assertRaises(ValueError) as error:
                await resolve_asset("missing.usda", base_dir=directory, assets_root="https://host/pack")
            self.assertIn(str(Path(directory) / "missing.usda"), str(error.exception))
            self.assertIn("https://host/pack/missing.usda", str(error.exception))

    async def test_relative_path_without_base_does_not_use_cwd(self) -> None:
        """No source directory means fragment semantics, even if a CWD file exists."""
        with patch("pathlib.Path.is_file", return_value=True) as local_check:
            self.assertEqual(
                await resolve_asset("overlay.usda", assets_root="https://host/pack", verify=False),
                "https://host/pack/overlay.usda",
            )
            local_check.assert_not_called()
        with patch(f"{_STORAGE}.path_exists", new=AsyncMock(return_value=False)):
            with self.assertRaisesRegex(ValueError, "No definition directory"):
                await resolve_asset("missing.usda", assets_root="https://host/pack")

    async def test_empty_path_fails_before_any_io(self) -> None:
        """Empty input is rejected without querying storage."""
        with patch(f"{_STORAGE}.get_assets_root_path_async", new_callable=AsyncMock) as root:
            with self.assertRaisesRegex(ValueError, "empty"):
                await resolve_asset("  ")
            root.assert_not_awaited()

    async def test_unavailable_root_reports_authored_path(self) -> None:
        """A root failure identifies the fragment that required it."""
        with patch(f"{_STORAGE}.get_assets_root_path_async", new=AsyncMock(return_value=None)):
            with self.assertRaisesRegex(RuntimeError, "/Isaac/Robots/g.usd"):
                await resolve_asset("/Isaac/Robots/g.usd")

    async def test_verify_false_skips_remote_stat(self) -> None:
        """Callers may resolve a location without an availability probe."""
        with patch(f"{_STORAGE}.path_exists", new=AsyncMock(side_effect=AssertionError("Stat called"))):
            self.assertEqual(await resolve_asset("https://host/g.usd", verify=False), "https://host/g.usd")

    async def test_windows_drive_path_is_not_a_uri(self) -> None:
        """Emulate native Windows file checks without requiring a Windows host."""
        with (
            patch(f"{_STORAGE}.is_absolute_path", return_value=True),
            patch("pathlib.Path.is_file", return_value=True),
            patch(f"{_STORAGE}.get_assets_root_path_async", new=AsyncMock(side_effect=AssertionError("Root queried"))),
        ):
            for path in (r"C:\grippers\overlay.usda", r"\\server\share\overlay.usda"):
                self.assertEqual(await resolve_asset(path, verify=False), path)


class TestPortableGripperDefinitions(omni.kit.test.AsyncTestCase):
    """Verify loader and builder integration without adding a definition registry."""

    def _write_definition_pair(self, directory: Path) -> None:
        """Create a definition, overlay and grasp map beside one another."""
        source = grippers._get_floating_grippers_dir() / "xarm.yaml"
        definition = yaml.safe_load(source.read_text(encoding="utf-8"))
        definition["asset_path"] = "overlay.usda"
        definition["grasp_controller"]["config_path"] = "maps/grasp.yaml"
        (directory / "maps").mkdir(parents=True)
        (directory / "xarm.yaml").write_text(yaml.safe_dump(definition), encoding="utf-8")
        (directory / "overlay.usda").write_text("#usda 1.0\n", encoding="utf-8")
        (directory / "maps/grasp.yaml").write_text("joints: []\n", encoding="utf-8")

    def test_definition_overlay_and_grasp_map_move_together(self) -> None:
        """Reloaded paths follow the definition after its directory moves."""
        with TemporaryDirectory() as temporary:
            original = Path(temporary) / "original"
            self._write_definition_pair(original)
            relocated = Path(temporary) / "relocated"
            for directory in (original, relocated):
                if directory == relocated:
                    shutil.move(str(original), str(relocated))
                with patch.object(grippers, "_get_floating_grippers_dir", return_value=directory):
                    spec = grippers.get_floating_gripper_spec("xarm")
                self.assertEqual(spec["asset_path"], str(directory / "overlay.usda"))
                self.assertEqual(spec["grasp_controller"]["config_path"], str(directory / "maps/grasp.yaml"))

    def test_builtin_and_absolute_grasp_maps_are_preserved(self) -> None:
        """Existing built-in and absolute map locations do not change."""
        with TemporaryDirectory() as temporary:
            for path in ("builtin://xarm_grasp", str(Path(temporary) / "map.yaml")):
                self.assertEqual(_resolve_definition_grasp_config_path(path, Path(temporary)), path)

    async def test_loader_opens_local_overlay_without_assets_root(self) -> None:
        """A local definition composes its overlay even when asset-root discovery would fail."""
        with TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self._write_definition_pair(directory)
            (directory / "overlay.usda").write_text(
                '#usda 1.0\n(defaultPrim = "gripper")\n'
                'def Xform "gripper" {\n'
                '    def Xform "xarm_gripper_base_link" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}\n'
                '    def PhysicsFixedJoint "root_joint" {}\n'
                "}\n",
                encoding="utf-8",
            )
            await grippers.stage_utils.create_new_stage_async(template="empty")
            try:
                with (
                    patch.object(grippers, "_get_floating_grippers_dir", return_value=directory),
                    patch(
                        f"{_STORAGE}.get_assets_root_path_async",
                        new=AsyncMock(side_effect=AssertionError("Root queried")),
                    ),
                ):
                    paths = await grippers.load_floating_grippers_async(
                        {"right": "xarm"}, {"right": ((0, 0, 1), (1, 0, 0, 0))}
                    )
                stage = grippers.stage_utils.get_current_stage()
                self.assertTrue(stage.GetPrimAtPath(paths["right"]["base_link"]).IsValid())
                self.assertTrue(stage.GetPrimAtPath(paths["right"]["tcp"]).IsValid())
            finally:
                grippers.stage_utils.close_stage()
                await grippers.app_utils.update_app_async()

    def test_missing_relative_grasp_map_reports_its_location(self) -> None:
        """A missing sibling map is not interpreted against the process CWD."""
        with TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError) as error:
                _resolve_definition_grasp_config_path("missing.yaml", Path(temporary))
            self.assertIn(str(Path(temporary) / "missing.yaml"), str(error.exception))

    async def test_loader_resolves_assets_before_changing_stage(self) -> None:
        """An unavailable second asset must not leave a partially loaded first gripper."""
        spec = grippers.get_floating_gripper_spec("xarm")
        with (
            patch.object(grippers, "get_floating_gripper_spec", return_value=spec),
            patch.object(
                grippers,
                "resolve_floating_gripper_asset_path_async",
                new=AsyncMock(side_effect=["first.usda", ValueError("Unavailable")]),
            ),
            patch.object(grippers.stage_utils, "define_prim") as define,
        ):
            with self.assertRaisesRegex(ValueError, "Unavailable"):
                await grippers.load_floating_grippers_async(
                    {"left": "xarm", "right": "xarm"},
                    dict.fromkeys(("left", "right"), ((0, 0, 0), (1, 0, 0, 0))),
                )
            define.assert_not_called()
