# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the URDF package scanner and OptionWidget.populate_packages."""

import os
import pathlib
import tempfile

import omni.kit.app
import omni.kit.test
import omni.ui as ui
from isaacsim.asset.importer.urdf import URDFImporterConfig
from isaacsim.asset.importer.urdf.ui.impl.option_widget import OptionWidget
from isaacsim.asset.importer.urdf.ui.impl.package_scanner import (
    _try_directory_walk,
    _try_meshes_folder,
    scan_urdf_packages,
)


def _write_urdf(path: pathlib.Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _touch(path: pathlib.Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.touch()


class TestScanUrdfPackages(omni.kit.test.AsyncTestCase):
    """Unit tests for :func:`scan_urdf_packages`.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...

    """

    def setUp(self) -> None:
        """Set up test fixtures."""
        self._tmp = tempfile.TemporaryDirectory()
        self._root = pathlib.Path(self._tmp.name)

    def tearDown(self) -> None:
        """Tear down test fixtures."""
        self._tmp.cleanup()

    # ------------------------------------------------------------------
    # File existence guards
    # ------------------------------------------------------------------

    async def test_nonexistent_file_returns_empty(self) -> None:
        """Scan a path that does not exist on disk.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.package_scanner import scan_urdf_packages
            >>> scan_urdf_packages("/nonexistent/path/robot.urdf")
            []

        """
        result = scan_urdf_packages("/nonexistent/does/not/exist.urdf")
        self.assertEqual(result, [])

    async def test_no_package_refs_returns_empty(self) -> None:
        """URDF with only relative mesh paths yields no packages.

        Example:

        .. code-block:: python

            >>> scan_urdf_packages.__module__
            'isaacsim.asset.importer.urdf.ui.impl.package_scanner'

        """
        urdf_path = self._root / "robot" / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<robot name="r"><link name="base"><visual><geometry>'
            '<mesh filename="meshes/base.dae"/>'
            "</geometry></visual></link></robot>",
        )
        result = scan_urdf_packages(str(urdf_path))
        self.assertEqual(result, [])

    # ------------------------------------------------------------------
    # Resolution: directory walk
    # ------------------------------------------------------------------

    async def test_directory_walk_resolves_package(self) -> None:
        """Package root found by walking up from the URDF directory.

        Layout::

            root/
              my_robot/
                meshes/
                  base.dae          <-- referenced mesh exists here
                urdf/
                  robot.urdf        <-- package://my_robot/meshes/base.dae

        Example:

        .. code-block:: python

            >>> scan_urdf_packages.__module__
            'isaacsim.asset.importer.urdf.ui.impl.package_scanner'

        """
        pkg_dir = self._root / "my_robot"
        mesh_file = pkg_dir / "meshes" / "base.dae"
        _touch(mesh_file)
        urdf_path = pkg_dir / "urdf" / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<robot name="r"><link name="base"><visual><geometry>'
            '<mesh filename="package://my_robot/meshes/base.dae"/>'
            "</geometry></visual></link></robot>",
        )

        result = scan_urdf_packages(str(urdf_path))
        self.assertEqual(len(result), 1)
        name, path = result[0]
        self.assertEqual(name, "my_robot")
        self.assertEqual(pathlib.Path(path), pkg_dir)

    async def test_directory_walk_multi_level(self) -> None:
        """Package root found several levels above the URDF directory.

        Layout::

            root/
              ws/
                src/
                  my_robot/
                    meshes/
                      arm.stl
                    urdf/
                      robot.urdf
        """
        pkg_dir = self._root / "ws" / "src" / "my_robot"
        _touch(pkg_dir / "meshes" / "arm.stl")
        urdf_path = pkg_dir / "urdf" / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<mesh filename="package://my_robot/meshes/arm.stl"/>',
        )

        result = scan_urdf_packages(str(urdf_path))
        self.assertEqual(len(result), 1)
        name, path = result[0]
        self.assertEqual(name, "my_robot")
        self.assertTrue(pathlib.Path(path).is_dir())

    # ------------------------------------------------------------------
    # Resolution: meshes-folder heuristic
    # ------------------------------------------------------------------

    async def test_heuristic_file_in_ancestor(self) -> None:
        """Referenced file found in an ancestor of the URDF directory.

        Layout::

            root/
              models/
                wheel.obj
              urdf/
                robot.urdf        <-- package://pkg/models/wheel.obj
        """
        pkg_dir = self._root
        (pkg_dir / "models").mkdir(parents=True, exist_ok=True)
        (pkg_dir / "models" / "wheel.obj").touch()
        urdf_path = pkg_dir / "urdf" / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<mesh filename="package://pkg/models/wheel.obj"/>',
        )

        result = scan_urdf_packages(str(urdf_path))
        self.assertEqual(len(result), 1)
        name, path = result[0]
        self.assertEqual(name, "pkg")
        self.assertEqual(pathlib.Path(path), pkg_dir)

    async def test_heuristic_file_in_pkg_subdir(self) -> None:
        """Referenced file found under ``{pkg_name}/`` subdirectory of an ancestor.

        Layout::

            root/
              pkg_a/
                other/
                  shape.dae
              urdf/
                robot.urdf
        """
        pkg_dir = self._root
        (pkg_dir / "pkg_a" / "other").mkdir(parents=True, exist_ok=True)
        (pkg_dir / "pkg_a" / "other" / "shape.dae").touch()
        urdf_path = pkg_dir / "urdf" / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<mesh filename="package://pkg_a/other/shape.dae"/>',
        )

        result = scan_urdf_packages(str(urdf_path))
        self.assertEqual(len(result), 1)
        name, path = result[0]
        self.assertEqual(name, "pkg_a")
        self.assertEqual(pathlib.Path(path), pkg_dir / "pkg_a")

    # ------------------------------------------------------------------
    # Unresolved package
    # ------------------------------------------------------------------

    async def test_unresolved_package_empty_path(self) -> None:
        """Package reference with no matching filesystem content gives empty path.

        Example:

        .. code-block:: python

            >>> scan_urdf_packages.__module__
            'isaacsim.asset.importer.urdf.ui.impl.package_scanner'

        """
        urdf_path = self._root / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<mesh filename="package://nonexistent_pkg/meshes/foo.dae"/>',
        )
        result = scan_urdf_packages(str(urdf_path))
        self.assertEqual(len(result), 1)
        name, path = result[0]
        self.assertEqual(name, "nonexistent_pkg")
        self.assertEqual(path, "")

    # ------------------------------------------------------------------
    # Deduplication
    # ------------------------------------------------------------------

    async def test_duplicate_refs_deduplicated(self) -> None:
        """Multiple references to the same package produce one row.

        Example:

        .. code-block:: python

            >>> scan_urdf_packages.__module__
            'isaacsim.asset.importer.urdf.ui.impl.package_scanner'

        """
        urdf_path = self._root / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<mesh filename="package://my_pkg/meshes/a.dae"/>' '<mesh filename="package://my_pkg/meshes/b.dae"/>',
        )
        result = scan_urdf_packages(str(urdf_path))
        names = [r[0] for r in result]
        self.assertEqual(names.count("my_pkg"), 1)

    # ------------------------------------------------------------------
    # Multiple distinct packages
    # ------------------------------------------------------------------

    async def test_multiple_packages(self) -> None:
        """Each distinct ``package://`` name yields its own row.

        Example:

        .. code-block:: python

            >>> scan_urdf_packages.__module__
            'isaacsim.asset.importer.urdf.ui.impl.package_scanner'

        """
        urdf_path = self._root / "robot.urdf"
        _write_urdf(
            urdf_path,
            '<mesh filename="package://pkg_a/meshes/a.dae"/>' '<mesh filename="package://pkg_b/meshes/b.dae"/>',
        )
        result = scan_urdf_packages(str(urdf_path))
        names = {r[0] for r in result}
        self.assertIn("pkg_a", names)
        self.assertIn("pkg_b", names)
        self.assertEqual(len(names), 2)

    # ------------------------------------------------------------------
    # Real URDF fixtures
    # ------------------------------------------------------------------

    async def test_real_carter_urdf(self) -> None:
        """carter.urdf uses ``package://carter/meshes/...`` resolved via directory walk.

        Example:

        .. code-block:: python

            >>> scan_urdf_packages.__module__
            'isaacsim.asset.importer.urdf.ui.impl.package_scanner'

        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        urdf_ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        urdf_ext_path = ext_manager.get_extension_path(urdf_ext_id)
        carter_urdf = os.path.normpath(
            os.path.join(urdf_ext_path, "data", "urdf", "robots", "carter", "urdf", "carter.urdf")
        )
        if not os.path.isfile(carter_urdf):
            self.skipTest(f"carter.urdf not found at {carter_urdf}")

        result = scan_urdf_packages(carter_urdf)
        names = {r[0] for r in result}
        self.assertIn("carter", names)

        carter_path = next(path for name, path in result if name == "carter")
        self.assertTrue(pathlib.Path(carter_path).is_dir(), f"Resolved path not a directory: {carter_path}")
        self.assertTrue((pathlib.Path(carter_path) / "meshes").is_dir())

    async def test_real_kaya_urdf(self) -> None:
        """kaya.urdf uses bare ``package://meshes/...`` (no package prefix) resolved via heuristic.

        Example:

        .. code-block:: python

            >>> scan_urdf_packages.__module__
            'isaacsim.asset.importer.urdf.ui.impl.package_scanner'

        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        urdf_ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        urdf_ext_path = ext_manager.get_extension_path(urdf_ext_id)
        kaya_urdf = os.path.normpath(os.path.join(urdf_ext_path, "data", "urdf", "robots", "kaya", "urdf", "kaya.urdf"))
        if not os.path.isfile(kaya_urdf):
            self.skipTest(f"kaya.urdf not found at {kaya_urdf}")

        result = scan_urdf_packages(kaya_urdf)
        self.assertTrue(len(result) > 0, "Expected at least one package entry from kaya.urdf")

        # The meshes/ folder is a sibling of the urdf/ dir; heuristic should resolve it
        for name, path in result:
            if path:
                self.assertTrue(pathlib.Path(path).is_dir(), f"Resolved path for '{name}' not a directory: {path}")


class TestDirectoryWalkHelper(omni.kit.test.AsyncTestCase):
    """Unit tests for the :func:`_try_directory_walk` helper.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...

    """

    def setUp(self) -> None:
        """Set up test fixtures."""
        self._tmp = tempfile.TemporaryDirectory()
        self._root = pathlib.Path(self._tmp.name)

    def tearDown(self) -> None:
        """Tear down test fixtures."""
        self._tmp.cleanup()

    async def test_empty_rel_paths_returns_none(self) -> None:
        """Empty relative paths list yields None.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.package_scanner import _try_directory_walk
            >>> import pathlib
            >>> _try_directory_walk(pathlib.Path("/tmp"), []) is None
            True

        """
        result = _try_directory_walk(self._root, [])
        self.assertIsNone(result)

    async def test_finds_file_at_start_dir(self) -> None:
        """File at the starting directory resolves immediately.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.package_scanner import _try_directory_walk
            >>> import pathlib

        """
        target = self._root / "meshes" / "part.dae"
        _touch(target)
        result = _try_directory_walk(self._root, ["meshes/part.dae"])
        self.assertEqual(pathlib.Path(result), self._root)

    async def test_returns_none_when_not_found(self) -> None:
        """Returns None when no ancestor contains any of the relative paths.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.package_scanner import _try_directory_walk
            >>> import pathlib

        """
        result = _try_directory_walk(self._root, ["does/not/exist.dae"])
        self.assertIsNone(result)


class TestMeshesFolderHelper(omni.kit.test.AsyncTestCase):
    """Unit tests for the :func:`_try_meshes_folder` helper.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...

    """

    def setUp(self) -> None:
        """Set up test fixtures."""
        self._tmp = tempfile.TemporaryDirectory()
        self._root = pathlib.Path(self._tmp.name)

    def tearDown(self) -> None:
        """Tear down test fixtures."""
        self._tmp.cleanup()

    async def test_directory_with_rel_paths_found(self) -> None:
        """Directory containing a referenced file is picked up.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.package_scanner import _try_meshes_folder
            >>> import pathlib

        """
        (self._root / "models").mkdir()
        (self._root / "models" / "part.obj").touch()
        result = _try_meshes_folder(self._root, "any_pkg", ["models/part.obj"])
        self.assertEqual(pathlib.Path(result), self._root)

    async def test_pkg_subdir_with_file(self) -> None:
        """``{pkg_name}/`` subfolder containing a referenced file is resolved.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.package_scanner import _try_meshes_folder
            >>> import pathlib

        """
        (self._root / "my_pkg" / "data").mkdir(parents=True)
        (self._root / "my_pkg" / "data" / "arm.dae").touch()
        result = _try_meshes_folder(self._root, "my_pkg", ["data/arm.dae"])
        self.assertEqual(pathlib.Path(result), self._root / "my_pkg")

    async def test_returns_none_when_absent(self) -> None:
        """Returns None when no ``meshes/`` folder can be found.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.package_scanner import _try_meshes_folder
            >>> import pathlib

        """
        result = _try_meshes_folder(self._root, "nonexistent_pkg", [])
        self.assertIsNone(result)


class TestOptionWidgetPopulatePackages(omni.kit.test.AsyncTestCase):
    """Tests for :meth:`OptionWidget.populate_packages`.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...

    """

    async def setUp(self) -> None:
        """Create a minimal OptionWidget inside a hidden window.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf import URDFImporterConfig
            >>> URDFImporterConfig()
            <...>

        """
        self._window = ui.Window("_test_pkg_scanner_window", visible=False)
        self._models: dict = {}
        self._config = URDFImporterConfig()
        with self._window.frame:
            self._widget = OptionWidget(self._models, self._config)
            self._widget.build_options()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Destroy the test window.

        Example:

        .. code-block:: python

            >>> import asyncio
            >>> asyncio.sleep(0)  # doctest: +SKIP

        """
        self._window.destroy()
        await omni.kit.app.get_app().next_update_async()

    async def test_populate_replaces_empty_row(self) -> None:
        """Calling populate_packages replaces the initial empty row.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.option_widget import OptionWidget

        """
        self._widget.populate_packages([("my_pkg", "/opt/ros/my_pkg")])
        await omni.kit.app.get_app().next_update_async()

        rows = self._widget.get_ros_package_map()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["name"], "my_pkg")
        self.assertEqual(rows[0]["path"], "/opt/ros/my_pkg")

    async def test_populate_multiple_packages(self) -> None:
        """All provided packages appear in the model after populate_packages.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.option_widget import OptionWidget

        """
        packages = [("pkg_a", "/path/a"), ("pkg_b", "/path/b"), ("pkg_c", "")]
        self._widget.populate_packages(packages)
        await omni.kit.app.get_app().next_update_async()

        result = self._widget.get_ros_package_map()
        # pkg_c has empty path and get_ros_package_map only includes entries with non-empty name
        names = [r["name"] for r in result]
        self.assertIn("pkg_a", names)
        self.assertIn("pkg_b", names)
        self.assertIn("pkg_c", names)

    async def test_populate_empty_list_is_noop(self) -> None:
        """populate_packages with an empty list leaves the table unchanged.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.option_widget import OptionWidget

        """
        # Add a row manually before the noop call
        self._widget._ros_package_model.add_row("existing", "/existing/path")
        rows_before = self._widget._ros_package_model.get_rows()

        self._widget.populate_packages([])
        await omni.kit.app.get_app().next_update_async()

        rows_after = self._widget._ros_package_model.get_rows()
        self.assertEqual(rows_before, rows_after)

    async def test_populate_path_survives_get_ros_package_map(self) -> None:
        """Resolved paths are faithfully returned by get_ros_package_map.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf.ui.impl.option_widget import OptionWidget

        """
        self._widget.populate_packages([("robot_pkg", "/home/user/ws/src/robot_pkg")])
        await omni.kit.app.get_app().next_update_async()

        pkg_map = self._widget.get_ros_package_map()
        self.assertEqual(len(pkg_map), 1)
        self.assertEqual(pkg_map[0], {"name": "robot_pkg", "path": "/home/user/ws/src/robot_pkg"})
