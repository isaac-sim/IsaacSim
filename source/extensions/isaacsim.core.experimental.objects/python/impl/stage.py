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

"""Stage module."""

from __future__ import annotations

from typing import Literal

import carb
import omni.kit.app
import omni.kit.stage_templates
import omni.usd
import omni.usd.commands
from omni.metrics.assembler.core import get_metrics_assembler_interface
from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils


class Stage:
    """Wrap or create a USD stage, exposing stage-level operations identified by stage ID.

    The class is instance-free with respect to USD objects: it holds only an integer stage ID
    and works exclusively with prim paths as strings.

    Args:
        stage_id: ID of an existing stage to attach to. If ``None``, an invalid stage is created.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Stage
        >>>
        >>> # attach to the current context stage
        >>> stage = Stage()
        >>> stage.is_valid()
        False
        >>> stage = stage.create_stage()
        >>> stage.is_valid()
        True
        >>>
        >>> # attach to a specific stage by ID
        >>> stage = Stage(stage_id=stage.get_stage_id())
        >>> stage.is_valid()
        True
    """

    def __init__(self, stage_id: int | None = None) -> None:
        self._stage_id = stage_id if stage_id is not None else -1
        if self._stage_id != -1:
            cache = UsdUtils.StageCache.Get()
            stage = cache.Find(Usd.StageCache.Id.FromLongInt(self._stage_id))
            if stage is None:
                raise ValueError(f"Stage with ID ({self._stage_id}) not found")

    def __str__(self) -> str:
        return self.generate_string_representation(mode="tree")

    def _insert_or_get_id(self, stage: Usd.Stage) -> int:
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            stage_id = cache.Insert(stage).ToLongInt()
        return stage_id

    def _get_stage(self) -> Usd.Stage | None:
        if self._stage_id < 0:
            return None
        return UsdUtils.StageCache.Get().Find(Usd.StageCache.Id.FromLongInt(self._stage_id))

    def _setup_gridroom(self) -> None:
        """Populate the stage with a grid-like room scene."""
        # defer imports to avoid circular dependencies at module load time
        from .ground_plane import GroundPlane
        from .lights.sphere import SphereLight

        self.define_prim("/World", "Xform")
        self.define_prim("/World/Environment", "Xform")
        GroundPlane("/World/GroundPlane", templates="wireframe-blue")
        sphere_light = SphereLight("/World/Environment/SphereLight", radii=0.25, positions=(0.0, 0.0, 2.5))
        sphere_light.set_colors("white")
        sphere_light.set_intensities(100000)
        sphere_light.set_color_temperatures(6500)
        shaping_api = UsdLux.ShapingAPI.Apply(sphere_light.prims[0])
        shaping_api.GetShapingConeAngleAttr().Set(180)

    """
    Methods.
    """

    def get_stage_id(self) -> int:
        """Get the stage ID.

        Backends: :guilabel:`usd`.

        Returns:
            The stage ID.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage()
            >>> stage.get_stage_id()  # doctest: +NO_CHECK
            9223006
        """
        return self._stage_id

    def is_valid(self) -> bool:
        """Check whether the stage referenced by the stored ID is still valid.

        Backends: :guilabel:`usd`.

        Returns:
            Whether the stage is valid.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage()
            >>> stage.is_valid()
            True
        """
        return self._get_stage() is not None

    def open_stage(self, usd_path: str) -> Stage:
        """Open a USD file and attach the resulting stage to this instance.

        Backends: :guilabel:`usd`.

        Args:
            usd_path: USD file path to open.

        Returns:
            Self, for method chaining.

        Raises:
            ValueError: If the file is not a valid USD file (shallow check).

        Example:

        .. code-block:: python

            >>> from isaacsim.storage.native import get_assets_root_path
            >>>
            >>> stage = Stage().open_stage(  # doctest: +NO_CHECK
            ...     get_assets_root_path() + "/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
            ... )
        """
        if not Usd.Stage.IsSupportedFile(usd_path):
            raise ValueError(f"The file ({usd_path}) is not USD open-able")
        usd_context = omni.usd.get_context()
        usd_context.disable_save_to_recent_files()
        usd_context.open_stage(usd_path)
        usd_context.enable_save_to_recent_files()
        self._stage_id = self._insert_or_get_id(usd_context.get_stage())
        return self

    def create_stage(self, *, template: str | None = None) -> Stage:
        """Create a new USD stage and attach it to this instance.

        Backends: :guilabel:`usd`.

        .. note::

            At least the following templates should be available.
            Other templates might be available depending on app customizations.

            .. list-table:: Isaac Sim templates
                :header-rows: 1

                * - Template
                  - Description
                * - ``"gridroom"``
                  - Stage with a blue gridroom scene.

            .. list-table:: Kit templates
                :header-rows: 1

                * - Template
                  - Description
                * - ``"default stage"``
                  - Stage with a gray gridded plane, dome and distant lights, and the ``/World`` Xform prim.
                * - ``"empty"``
                  - Empty stage with the ``/World`` Xform prim.
                * - ``"sunlight"``
                  - Stage with a distant light and the ``/World`` Xform prim.

        Args:
            template: The template to use. If ``None``, creates an empty stage.

        Returns:
            Self, for method chaining.

        Raises:
            ValueError: When the template is not found.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage(template="gridroom")  # doctest: +NO_CHECK
        """
        # create 'empty' stage
        if template in [None, "gridroom"]:
            omni.usd.get_context().new_stage()
            if template == "gridroom":
                self._stage_id = self._insert_or_get_id(omni.usd.get_context().get_stage())
                self._setup_gridroom()
        # create stage from template
        else:
            templates = [name for item in omni.kit.stage_templates.get_stage_template_list() for name in item]
            if template not in templates:
                raise ValueError(f"Template '{template}' not found. Available templates: {templates}")
            omni.kit.stage_templates.new_stage(template=template)
        self._stage_id = self._insert_or_get_id(omni.usd.get_context().get_stage())
        return self

    def save_stage(self, usd_path: str) -> bool:
        """Save the stage to a USD file.

        Backends: :guilabel:`usd`.

        Args:
            usd_path: USD file path to save to.

        Returns:
            Whether the stage was saved successfully.

        Raises:
            ValueError: If the file path is not a valid USD path.

        Example:

        .. code-block:: python

            >>> import os, tempfile
            >>> stage = Stage().create_stage()
            >>> stage.save_stage(os.path.join(tempfile.gettempdir(), "test.usd"))
            True
        """
        if not Usd.Stage.IsSupportedFile(usd_path):
            raise ValueError(f"The file ({usd_path}) is not USD open-able")
        usd_stage = self._get_stage()
        root_layer = usd_stage.GetRootLayer()
        layer = Sdf.Layer.CreateNew(usd_path)
        layer.TransferContent(root_layer)
        omni.usd.resolve_paths(root_layer.identifier, layer.identifier)
        return layer.Save()

    def close_stage(self) -> bool:
        """Close the stage attached to the USD context.

        Backends: :guilabel:`usd`.

        Returns:
            Whether the stage was closed successfully.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage()
            >>> stage.close_stage()
            True
        """
        cache = UsdUtils.StageCache.Get()
        stage = cache.Find(Usd.StageCache.Id.FromLongInt(self._stage_id)) if self._stage_id >= 0 else None
        result = omni.usd.get_context().close_stage()
        if stage is not None and cache.Contains(stage):
            cache.Erase(stage)
        self._stage_id = -1
        return result

    def add_reference(
        self,
        usd_path: str,
        path: str,
        *,
        prim_type: str = "Xform",
        variants: dict[str, str] | None = None,
    ) -> bool:
        """Add a USD file reference to the stage at the specified prim path.

        Backends: :guilabel:`usd`.

        .. note::

            This function handles stage units verification to ensure compatibility.

        Args:
            usd_path: USD file path to reference.
            path: Prim path where the reference will be attached.
            prim_type: Prim type to create if the given ``path`` doesn't exist.
            variants: Mapping of variant set names to variant selections to author on the prim.

        Returns:
            Whether the reference was added successfully.

        Raises:
            ValueError: If ``path`` is not a valid path string.
            ValueError: If a variant set or selection is invalid.
            Exception: If the USD file cannot be opened.

        Example:

        .. code-block:: python

            >>> from isaacsim.storage.native import get_assets_root_path
            >>>
            >>> stage = Stage().create_stage()
            >>> stage.add_reference(  # doctest: +NO_CHECK
            ...     usd_path=get_assets_root_path() + "/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda",
            ...     path="/panda",
            ...     variants={"Gripper": "alternatefinger", "Mesh": "performance"},
            ... )
        """
        if not Sdf.Path.IsValidPathString(path):
            raise ValueError(f"Prim path ({path}) is not a valid path string")
        sdf_layer = Sdf.Layer.FindOrOpen(usd_path)
        if not sdf_layer:
            raise Exception(
                f"Unable to get Sdf layer. The USD file ({usd_path}) might not exist or is not a valid USD file."
            )
        stage = self._get_stage()
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            prim = stage.DefinePrim(path, prim_type)
        reference = Sdf.Reference(usd_path)
        reference_added = False
        # check for divergent units
        result = get_metrics_assembler_interface().check_layers(
            stage.GetRootLayer().identifier, sdf_layer.identifier, self._stage_id
        )
        if result["ret_val"]:
            try:
                import omni.kit.commands
                import omni.metrics.assembler.ui

                omni.kit.commands.execute("AddReference", stage=stage, prim_path=path, reference=reference)
                reference_added = True
            except Exception:
                carb.log_warn(
                    f"The USD file ({usd_path}) has divergent units. "
                    "Enable the omni.usd.metrics.assembler.ui extension or convert the file into right units."
                )
        # add reference (if not already added during divergent units check)
        if not reference_added:
            if not prim.GetReferences().AddReference(reference):
                raise Exception(f"Unable to add reference to the USD file ({usd_path}).")
        # set variants
        # TODO: use Prim class when it is available
        available_variant_sets = prim.GetVariantSets().GetNames()
        for variant_set, variant_selection in (variants or {}).items():
            if variant_set not in available_variant_sets:
                raise ValueError(f"Invalid variant set: '{variant_set}'. Available sets: {available_variant_sets}")
            available_variant_selections = prim.GetVariantSet(variant_set).GetVariantNames()
            if variant_selection and variant_selection not in available_variant_selections:
                raise ValueError(
                    f"Invalid variant selection (variant set: '{variant_set}'): '{variant_selection}'. "
                    f"Available selections (variant set: '{variant_set}'): {available_variant_selections}"
                )
            prim.GetVariantSet(variant_set).SetVariantSelection(variant_selection)
        return True

    def define_prim(self, path: str, type_name: str = "Xform") -> str:
        """Define a prim of the specified type at the given path.

        Backends: :guilabel:`usd`.

        Common token values for ``type_name`` are:

        * ``"Camera"``, ``"Mesh"``, ``"PhysicsScene"``, ``"Scope"``, ``"Xform"``
        * Shapes (``"Capsule"``, ``"Cone"``, ``"Cube"``, ``"Cylinder"``, ``"Plane"``, ``"Sphere"``)
        * Lights (``"CylinderLight"``, ``"DiskLight"``, ``"DistantLight"``, ``"DomeLight"``, ``"RectLight"``, ``"SphereLight"``)

        Args:
            path: Absolute prim path.
            type_name: Token identifying the prim type.

        Returns:
            The absolute path of the defined prim.

        Raises:
            ValueError: If ``path`` is not a valid or absolute path string.
            RuntimeError: If a prim already exists at ``path`` with a different type.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage()
            >>> stage.define_prim("/World/Sphere", type_name="Sphere")
            '/World/Sphere'
        """
        if not Sdf.Path.IsValidPathString(path) or not Sdf.Path(path).IsAbsolutePath():
            raise ValueError(f"Prim path ({path}) is not a valid or absolute path string")
        stage = self._get_stage()
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            if prim.GetTypeName() != type_name:
                raise RuntimeError(f"A prim already exists at path ({path}) with type ({prim.GetTypeName()})")
            return prim.GetPath().pathString
        return stage.DefinePrim(path, type_name).GetPath().pathString

    def move_prim(self, target: str, destination: str) -> tuple[bool, str]:
        """Move a prim to a different location on the stage hierarchy.

        Backends: :guilabel:`usd`.

        The move operation follows the next rules:

        - If the destination exists, the target prim is moved (as a child) into the destination prim.
          Moved prim keeps its name.
        - If the destination does not exist, the target prim is moved to the specified destination path,
          provided that the parent exists (if not, an error is raised). Moved prim is renamed.

        Args:
            target: Prim path to move.
            destination: Destination path.

        Returns:
            Two-element tuple. Whether the move succeeded and the resulting prim path.

        Raises:
            ValueError: If the target prim is not a valid prim.
            ValueError: If the destination path has unexisting parents.
            ValueError: If the destination path is not a valid path string.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage()
            >>> _ = stage.define_prim("/World/A")
            >>> _ = stage.define_prim("/World/B")
            >>>
            >>> # move prim A to (into) prim B
            >>> stage.move_prim("/World/A", "/World/B")
            (True, '/World/B/A')
            >>> # move prim A next to /World (with name C)
            >>> stage.move_prim("/World/B/A", "/World/C")
            (True, '/World/C')
        """
        if not Sdf.Path.IsValidPathString(destination):
            raise ValueError(f"Destination path '{destination}' is not a valid path string")
        target_path = Sdf.Path(target)
        destination_path = Sdf.Path(destination)
        stage = self._get_stage()
        if not stage.GetPrimAtPath(target_path).IsValid():
            raise ValueError(f"Prim at path '{target}' is not a valid prim")
        if stage.GetPrimAtPath(destination_path).IsValid():
            destination_path = destination_path.AppendChild(target_path.name)
        elif not stage.GetPrimAtPath(destination_path.GetParentPath()).IsValid():
            raise ValueError(
                f"Destination path '{destination_path}' has unexisting parent '{destination_path.GetParentPath()}'"
            )
        result = omni.usd.commands.MovePrimCommand(path_from=target_path, path_to=destination_path).do()
        return result, destination_path.pathString

    def remove_prim(self, path: str) -> bool:
        """Remove/delete a prim from the stage.

        Backends: :guilabel:`usd`.

        Args:
            path: Prim path to delete.

        Returns:
            Whether the prim was deleted successfully.

        Raises:
            ValueError: If the prim at ``path`` is not valid.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage()
            >>> _ = stage.define_prim("/World/Sphere", type_name="Sphere")
            >>> stage.remove_prim("/World/Sphere")
            True
        """
        stage = self._get_stage()
        if not stage.GetPrimAtPath(path).IsValid():
            raise ValueError(f"Prim at path '{path}' is not a valid prim")
        omni.usd.commands.DeletePrimsCommand([path]).do()
        return not stage.GetPrimAtPath(path).IsValid()

    def get_up_axis(self) -> Literal["Y", "Z"]:
        """Get the stage up axis.

        Backends: :guilabel:`usd`.

        Returns:
            The stage up axis (``"Y"`` or ``"Z"``).

        Example:

        .. code-block:: python

            >>> Stage().create_stage().get_up_axis()
            'Z'
        """
        return UsdGeom.GetStageUpAxis(self._get_stage())

    def set_up_axis(self, up_axis: Literal["Y", "Z"]) -> None:
        """Set the stage up axis.

        Backends: :guilabel:`usd`.

        Args:
            up_axis: The stage up axis (``"Y"`` or ``"Z"``).

        Raises:
            ValueError: If ``up_axis`` is not ``"Y"`` or ``"Z"``.

        Example:

        .. code-block:: python

            >>> Stage().create_stage().set_up_axis("Y")
        """
        if up_axis.upper() not in ["Y", "Z"]:
            raise ValueError(f"Invalid up axis: '{up_axis}'")
        UsdGeom.SetStageUpAxis(self._get_stage(), up_axis.upper())

    def get_units(self) -> tuple[float, float]:
        """Get the stage meters per unit and kilograms per unit.

        Backends: :guilabel:`usd`.

        The most common distance units and their values are listed in the following table:

        +------------------+--------+
        | Unit             | Value  |
        +==================+========+
        | kilometer (km)   | 1000.0 |
        +------------------+--------+
        | meters (m)       | 1.0    |
        +------------------+--------+
        | inch (in)        | 0.0254 |
        +------------------+--------+
        | centimeters (cm) | 0.01   |
        +------------------+--------+
        | millimeter (mm)  | 0.001  |
        +------------------+--------+

        The most common mass units and their values are listed in the following table:

        +------------------+--------+
        | Unit             | Value  |
        +==================+========+
        | metric ton (t)   | 1000.0 |
        +------------------+--------+
        | kilogram (kg)    | 1.0    |
        +------------------+--------+
        | gram (g)         | 0.001  |
        +------------------+--------+
        | pound (lb)       | 0.4536 |
        +------------------+--------+
        | ounce (oz)       | 0.0283 |
        +------------------+--------+

        Returns:
            Current stage meters per unit and kilograms per unit.

        Example:

        .. code-block:: python

            >>> Stage().create_stage().get_units()
            (1.0, 1.0)
        """
        stage = self._get_stage()
        return UsdGeom.GetStageMetersPerUnit(stage), UsdPhysics.GetStageKilogramsPerUnit(stage)

    def set_units(self, *, meters_per_unit: float | None = None, kilograms_per_unit: float | None = None) -> None:
        """Set the stage meters per unit and kilograms per unit.

        Backends: :guilabel:`usd`.

        The most common distance units and their values are listed in the following table:

        +------------------+--------+
        | Unit             | Value  |
        +==================+========+
        | kilometer (km)   | 1000.0 |
        +------------------+--------+
        | meters (m)       | 1.0    |
        +------------------+--------+
        | inch (in)        | 0.0254 |
        +------------------+--------+
        | centimeters (cm) | 0.01   |
        +------------------+--------+
        | millimeter (mm)  | 0.001  |
        +------------------+--------+

        The most common mass units and their values are listed in the following table:

        +------------------+--------+
        | Unit             | Value  |
        +==================+========+
        | metric ton (t)   | 1000.0 |
        +------------------+--------+
        | kilogram (kg)    | 1.0    |
        +------------------+--------+
        | gram (g)         | 0.001  |
        +------------------+--------+
        | pound (lb)       | 0.4536 |
        +------------------+--------+
        | ounce (oz)       | 0.0283 |
        +------------------+--------+

        Args:
            meters_per_unit: Meters per unit (e.g., ``1.0`` for meters, ``0.01`` for centimeters).
            kilograms_per_unit: Kilograms per unit (e.g., ``1.0`` for kilograms, ``0.001`` for grams).

        Example:

        .. code-block:: python

            >>> Stage().create_stage().set_units(meters_per_unit=0.01, kilograms_per_unit=0.001)
        """
        stage = self._get_stage()
        if meters_per_unit is not None:
            UsdGeom.SetStageMetersPerUnit(stage, meters_per_unit)
        if kilograms_per_unit is not None:
            UsdPhysics.SetStageKilogramsPerUnit(stage, kilograms_per_unit)

    def get_time_code(self) -> tuple[float, float, float]:
        """Get the stage time code (start, end, and time codes per second).

        Backends: :guilabel:`usd`.

        Returns:
            Three-element tuple: start time code, end time code, and time codes per second.

        Example:

        .. code-block:: python

            >>> Stage().create_stage().get_time_code()
            (0.0, 100.0, 60.0)
        """
        stage = self._get_stage()
        return stage.GetStartTimeCode(), stage.GetEndTimeCode(), stage.GetTimeCodesPerSecond()

    def set_time_code(
        self,
        *,
        start_time_code: float | None = None,
        end_time_code: float | None = None,
        time_codes_per_second: float | None = None,
    ) -> None:
        """Set the stage time code.

        Backends: :guilabel:`usd`.

        Args:
            start_time_code: The start time code.
            end_time_code: The end time code.
            time_codes_per_second: The time codes per second.

        Example:

        .. code-block:: python

            >>> Stage().create_stage().set_time_code(
            ...     start_time_code=0.0,
            ...     end_time_code=200.0,
            ...     time_codes_per_second=30.0,
            ... )
        """
        stage = self._get_stage()
        if start_time_code is not None:
            stage.SetStartTimeCode(start_time_code)
        if end_time_code is not None:
            stage.SetEndTimeCode(end_time_code)
        if time_codes_per_second is not None:
            stage.SetTimeCodesPerSecond(time_codes_per_second)

    def generate_string_representation(self, *, mode: Literal["list", "tree"] = "tree") -> str:
        """Generate a string representation of the stage.

        Backends: :guilabel:`usd`.

        Args:
            mode: ``"list"`` (flat list of all prims) or ``"tree"`` (hierarchical tree).

        Returns:
            String representation of the stage.

        Raises:
            ValueError: If ``mode`` is not ``"list"`` or ``"tree"``.

        Example:

        .. code-block:: python

            >>> stage = Stage().create_stage()
            >>> _ = stage.define_prim("/World/PrimA", type_name="Sphere")
            >>> _ = stage.define_prim("/World/PrimB", type_name="Cube")
            >>> print(stage.generate_string_representation(mode="tree"))
            / ()
            ├─ World ()
            │  ├─ PrimA (Sphere)
            │  ├─ PrimB (Cube)
        """
        usd_stage = self._get_stage()

        def _generate_tree(prim: Usd.Prim, indent: int = 0) -> None:
            prefix = "│  " * (indent - 1) + "├─ " if indent > 0 else ""
            lines.append(f"{prefix}{prim.GetPath().name} ({prim.GetTypeName()})")
            for child in prim.GetChildren():
                _generate_tree(child, indent + 1)

        lines = []
        if mode == "list":
            for prim in usd_stage.Traverse():
                lines.append(f"{prim.GetPath().pathString} ({prim.GetTypeName()})")
        elif mode == "tree":
            _generate_tree(usd_stage.GetPrimAtPath("/"), 0)
        else:
            raise ValueError(f"Invalid mode: {mode}")
        return "\n".join(lines)

    """
    Async methods.
    """

    async def open_stage_async(self, usd_path: str) -> Stage:
        """Open a USD file and attach the resulting stage to this instance.

        Backends: :guilabel:`usd`.

        This method is the asynchronous version of :py:meth:`open_stage`.

        Args:
            usd_path: USD file path to open.

        Returns:
            Self, for method chaining.

        Raises:
            ValueError: If the file is not a valid USD file (shallow check).
        """
        if not Usd.Stage.IsSupportedFile(usd_path):
            raise ValueError(f"The file ({usd_path}) is not USD open-able")
        usd_context = omni.usd.get_context()
        usd_context.disable_save_to_recent_files()
        await usd_context.open_stage_async(usd_path)
        usd_context.enable_save_to_recent_files()
        self._stage_id = self._insert_or_get_id(usd_context.get_stage())
        return self

    async def create_stage_async(self, *, template: str | None = None) -> Stage:
        """Create a new USD stage and attach it to this instance.

        Backends: :guilabel:`usd`.

        This method is the asynchronous version of :py:meth:`create_stage`.

        Args:
            template: The template to use. If ``None``, creates an empty stage.
                See :py:meth:`create_stage` for the list of available templates.

        Returns:
            Self, for method chaining.

        Raises:
            ValueError: When the template is not found.
        """

        # create 'empty' stage
        if template in [None, "gridroom"]:
            await omni.usd.get_context().new_stage_async()
            if template == "gridroom":
                self._stage_id = self._insert_or_get_id(omni.usd.get_context().get_stage())
                self._setup_gridroom()
        # create stage from template
        else:
            templates = [name for item in omni.kit.stage_templates.get_stage_template_list() for name in item]
            if template not in templates:
                raise ValueError(f"Template '{template}' not found. Available templates: {templates}")
            await omni.kit.stage_templates.new_stage_async(template=template)
        await omni.kit.app.get_app().next_update_async()
        self._stage_id = self._insert_or_get_id(omni.usd.get_context().get_stage())
        return self
