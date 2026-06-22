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

"""URDF Exporter UI extension. Provides File menu integration and export options dialog."""

from __future__ import annotations

import gc
import logging
import os
import re
from typing import Any

import omni
import omni.ext
import omni.ui as ui
import omni.usd
from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.kit.window.file_exporter import ExportOptionsDelegate, get_file_exporter
from pxr import Gf, Sdf, Usd, UsdPhysics

from .option_widget import OptionWidget

_logger = logging.getLogger(__name__)

EXTENSION_TITLE = "URDF Exporter"

_extension_instance: Extension | None = None


def get_instance() -> Extension | None:
    """Get the current extension instance.

    Returns:
        Current extension instance, or None if the extension is not started.
    """
    return _extension_instance


class Extension(omni.ext.IExt):
    """URDF Exporter UI extension.

    Adds a File menu item and provides an export options dialog
    that delegates to UsdToUrdfConverter from the core extension.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and add File menu entry.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        global _extension_instance
        _extension_instance = self
        self._export_options = None

        self._menu_items = [MenuItemDescription(name=EXTENSION_TITLE, onclick_fn=self._show_dialog)]
        add_menu_items(self._menu_items, "File")

    def _show_dialog(self) -> None:
        file_exporter = get_file_exporter()
        if not file_exporter:
            return

        self._export_options = UrdfExporterDelegate()

        filename_url = _derive_filename_url_from_stage()

        file_exporter.show_window(
            title="Export As ...",
            file_extension_types=[(".urdf", "URDF format")],
            export_handler=self._export_options.export,
            filename_url=filename_url,
        )

        file_exporter.add_export_options_frame("Export Options", self._export_options)

    def _hide_dialog(self) -> None:
        file_exporter = get_file_exporter()
        if file_exporter:
            file_exporter.hide_window()

    def on_shutdown(self) -> None:
        """Tear down the extension and remove menu items."""
        if self._export_options:
            self._export_options.cleanup()
        self._hide_dialog()
        remove_menu_items(self._menu_items, "File")

        global _extension_instance
        _extension_instance = None
        gc.collect()


class UrdfExporterDelegate(ExportOptionsDelegate):
    """File exporter delegate for URDF export."""

    def __init__(self) -> None:
        super().__init__(
            build_fn=self._build_ui_impl,
            destroy_fn=self._destroy_impl,
        )
        self._widget = None
        self._option_widget = OptionWidget()

    def _build_ui_impl(self) -> None:
        self._widget = ui.Frame()
        with self._widget:
            self._option_widget.build()

    def export(self, filename: str, dirname: str, extension: str = "", selections: list[str] | None = None) -> None:
        """Export the current stage to URDF.

        Args:
            filename: Output filename without the directory path.
            dirname: Output directory path.
            extension: File extension selected by the file exporter.
            selections: Selected USD prim paths from the file exporter, or None when no selections are provided.
        """
        result = self._do_export(dirname, filename)
        if result:
            print("Export to URDF successful")
        else:
            print("Error: Failed to export to URDF")

    def _do_export(self, export_dir: str, export_filename: str) -> bool:
        opts = self._option_widget

        root = opts.root_prim_path
        if not root:
            stage = omni.usd.get_context().get_stage()
            default_prim = stage.GetDefaultPrim()
            if default_prim:
                root = default_prim.GetPath().pathString
            print("Root prim not specified. Using the Default Prim on Stage. ", root)

        if not export_dir and not export_filename:
            _stage_dir, _export_filename = _get_stage_source_dir_and_stem()
            if not export_dir:
                if not _stage_dir:
                    _logger.error("Cannot determine output directory from stage source. Specify an output directory.")
                    return False
                export_dir = _stage_dir
            if not export_filename:
                if not _export_filename:
                    _logger.error("Cannot determine output filename from stage source. Specify a filename.")
                    return False
                export_filename = _export_filename

        export_path = os.path.join(export_dir, f"{export_filename}.urdf")
        stage = omni.usd.get_context().get_stage()

        if opts.use_physx_inertia:
            _write_physx_inertia(stage)

        mesh_prefix = opts.mesh_path_prefix

        if mesh_prefix == "package://":
            pkg_name = opts.package_name
            if not pkg_name:
                pkg_name = os.path.splitext(os.path.basename(export_path))[0]
            sanitized = re.sub(r"[^a-z0-9_]", "_", pkg_name.lower())
            sanitized = re.sub(r"_+", "_", sanitized).strip("_")
            if len(sanitized) < 2:
                sanitized += "_pkg"
            mesh_prefix = f"package://{sanitized}/"

        try:
            converter = UsdToUrdfConverter(
                stage=stage,
                root_prim_path=root,
                mesh_dir_name=opts.mesh_dir_name,
                mesh_path_prefix=mesh_prefix,
                visualize_collision_meshes=opts.visualize_collision_meshes,
            )
            converter.convert(export_path)
            print(f"Converted USD to URDF: {export_path}")
            return True
        except Exception as e:
            _logger.error(f"Failed to export URDF: {e}")
            import traceback

            traceback.print_exc()
            return False

    def _destroy_impl(self) -> None:
        if self._widget:
            self._widget.destroy()
        self._widget = None

    def cleanup(self) -> None:
        """Release UI resources."""
        self._option_widget.cleanup()
        self._destroy_impl()


def _write_physx_inertia(stage: Usd.Stage) -> None:
    """Query PhysX for inertia data and write to MassAPI where not authored.

    Args:
        stage: Stage containing rigid bodies to query.
    """
    try:
        import numpy as np
        from omni.physx import get_physx_property_query_interface
        from omni.physx.bindings._physx import PhysxPropertyQueryMode, PhysxPropertyQueryResult
        from pxr import PhysicsSchemaTools, UsdUtils
    except ImportError:
        _logger.warning("PhysX query interface not available, skipping inertia pre-computation")
        return

    inertia_layer = Sdf.Layer.CreateAnonymous("inertia_temp.usda")
    root_layer = stage.GetRootLayer()
    root_layer.subLayerPaths.append(inertia_layer.identifier)
    stage.SetEditTarget(Usd.EditTarget(inertia_layer))

    try:
        stage_cache = UsdUtils.StageCache().Get()
        stage_id = stage_cache.GetId(stage).ToLongInt()

        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            if not prim.HasAPI(UsdPhysics.MassAPI):
                continue

            mass_api = UsdPhysics.MassAPI(prim)
            mass_attr = mass_api.GetMassAttr()
            if mass_attr and mass_attr.HasAuthoredValue():
                diag_attr = mass_api.GetDiagonalInertiaAttr()
                if diag_attr and diag_attr.HasAuthoredValue():
                    continue

            prim_path = str(prim.GetPath())
            prim_id = PhysicsSchemaTools.sdfPathToInt(prim_path)
            inertia_result = {}

            def rigid_body_fn(rigid_info: Any, path: str = prim_path) -> None:
                if rigid_info.result == PhysxPropertyQueryResult.VALID:
                    inertia_result["mass"] = rigid_info.mass
                    inertia_result["com"] = np.array(rigid_info.center_of_mass)
                    inertia_result["inertia"] = np.array(rigid_info.inertia)
                    inertia_result["axes"] = np.array(
                        [
                            rigid_info.principal_axes[3],
                            rigid_info.principal_axes[0],
                            rigid_info.principal_axes[1],
                            rigid_info.principal_axes[2],
                        ]
                    )

            get_physx_property_query_interface().query_prim(
                stage_id=stage_id,
                prim_id=prim_id,
                query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
                rigid_body_fn=rigid_body_fn,
            )

            if "mass" in inertia_result:
                mass_api.GetMassAttr().Set(float(inertia_result["mass"]))
                com = inertia_result["com"]
                mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(float(com[0]), float(com[1]), float(com[2])))
                diag = inertia_result["inertia"]
                mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(float(diag[0]), float(diag[1]), float(diag[2])))
                axes = inertia_result["axes"]
                w, x, y, z = float(axes[0]), float(axes[1]), float(axes[2]), float(axes[3])
                mass_api.GetPrincipalAxesAttr().Set(Gf.Quatf(w, (x, y, z)))
    finally:
        root_layer.subLayerPaths.remove(inertia_layer.identifier)
        stage.SetEditTarget(stage.GetRootLayer())


def _get_stage_source_dir_and_stem() -> tuple[str, str]:
    """Get the directory and stem from the current stage's source layer.

    For ``omniverse://`` URLs, uses ``omni.client`` to parse the path.

    Returns:
        Tuple containing the source directory and filename stem. Both values are empty when no source can be
        determined.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return "", ""

    layer = stage.GetRootLayer()
    identifier = layer.identifier
    real_path = layer.realPath

    if identifier and identifier.startswith("omniverse://"):
        try:
            import omni.client as omni_client

            result = omni_client.break_url(identifier)
            path = result.path or ""
            basename = path.rsplit("/", 1)[-1] if "/" in path else path
            dirname = identifier[: identifier.rfind("/")]
        except (ImportError, AttributeError):
            path_part = identifier.split("://", 1)[-1]
            basename = path_part.rsplit("/", 1)[-1] if "/" in path_part else path_part
            dirname = identifier[: identifier.rfind("/")]
        stem = os.path.splitext(basename)[0] if basename else ""
        return dirname, stem

    source = real_path or identifier
    if source and not source.startswith("anon"):
        dirname = os.path.dirname(os.path.abspath(source))
        stem = os.path.splitext(os.path.basename(source))[0]
        return dirname, stem

    return "", ""


def _derive_filename_url_from_stage() -> str | None:
    """Build a ``filename_url`` for the file exporter dialog.

    Returns ``<directory>/<stem>`` (no extension) so the dialog pre-fills
    both directory and filename.  Returns ``None`` when the stage has no
    determinable source.

    Returns:
        Filename URL for the file exporter dialog, or None when the stage has no determinable source.
    """
    dirname, stem = _get_stage_source_dir_and_stem()
    if not stem:
        return None
    if dirname:
        return os.path.join(dirname, stem)
    return stem
