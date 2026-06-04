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

"""Functions for working with USD/USDRT stages."""

from __future__ import annotations

import contextlib
import threading
from collections.abc import Generator
from typing import Literal

import carb
import omni.kit.stage_templates
import omni.usd
import omni.usd.commands
import usdrt
from omni.metrics.assembler.core import get_metrics_assembler_interface
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

from . import _templates as stage_templates
from . import backend as backend_utils
from . import prim as prim_utils

_context = threading.local()  # thread-local storage to handle nested contexts and concurrent access


@contextlib.contextmanager
def use_stage(stage: Usd.Stage) -> Generator[None, None, None]:
    """Context manager that sets a thread-local stage instance.

    Args:
        stage: The stage to set in the context.

    Raises:
        AssertionError: If the stage is not a USD stage instance.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_in_memory = Usd.Stage.CreateInMemory()
        >>> with stage_utils.use_stage(stage_in_memory):
        ...    # operate on the specified stage
        ...    pass
        >>> # operate on the default stage attached to the USD context
    """
    # check stage
    assert isinstance(stage, Usd.Stage), f"Expected a USD stage instance, got: {type(stage)}"
    # store previous context value if it exists
    previous_stage = getattr(_context, "stage", None)
    # set new context value
    try:
        _context.stage = stage
        yield
    # remove context value or restore previous one if it exists
    finally:
        if previous_stage is None:
            delattr(_context, "stage")
        else:
            _context.stage = previous_stage


def is_stage_set() -> bool:
    """Check if a stage is set in the context manager.

    Returns:
        Whether a stage is set in the context manager.
    """
    return getattr(_context, "stage", None) is not None


def get_current_stage(*, backend: str | None = None) -> Usd.Stage | usdrt.Usd.Stage:
    """Get the stage set in the context manager or the default stage attached to the USD context.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        backend: Backend to use to get the stage. If not ``None``, it has precedence over the current backend
            set via the :py:func:`~isaacsim.core.experimental.utils.impl.backend.use_backend` context manager.

    Returns:
        The current stage instance or the default stage attached to the USD context if no stage is set.

    Raises:
        ValueError: If the backend is not supported.
        ValueError: If there is no stage (set via context manager or attached to the USD context).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.get_current_stage()
        Usd.Stage.Open(rootLayer=Sdf.Find('anon:...usd'), ...)
    """
    # get backend
    if backend is None:
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
    elif backend not in ["usd", "usdrt", "fabric"]:
        raise ValueError(f"Invalid backend: {backend}")
    # get USD stage
    stage = getattr(_context, "stage", omni.usd.get_context().get_stage())
    if stage is None:
        raise ValueError("No stage found. Create a stage first (see `create_new_stage`).")
    # get USDRT/Fabric stage
    if backend in ["usdrt", "fabric"]:
        stage_cache = UsdUtils.StageCache.Get()
        stage_id = stage_cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            stage_id = stage_cache.Insert(stage).ToLongInt()
        return usdrt.Usd.Stage.Attach(stage_id)
    return stage


def get_stage_id(stage: Usd.Stage) -> int:
    """Get the stage ID of a USD stage.

    Backends: :guilabel:`usd`.

    Args:
        stage: The stage to get the ID of.

    Returns:
        The stage ID.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage = stage_utils.get_current_stage()
        >>> stage_utils.get_stage_id(stage)  # doctest: +NO_CHECK
        9223006
    """
    stage_cache = UsdUtils.StageCache.Get()
    stage_id = stage_cache.GetId(stage).ToLongInt()
    if stage_id < 0:
        stage_id = stage_cache.Insert(stage).ToLongInt()
    return stage_id


def generate_stage_representation(mode: Literal["list", "tree"] = "tree") -> str:
    """Generate a string representation of the stage.

    Args:
        mode: The mode to use to generate the representation. Available modes are:

            - ``"list"``: List all prims in the stage.
            - ``"tree"``: Tree representation of the stage.

    Returns:
        String representation of the stage.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.create_new_stage()  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/PrimA", type_name="Sphere")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/PrimB", type_name="Cube")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/PrimC", type_name="Camera")  # doctest: +NO_CHECK
        >>>
        >>> print(stage_utils.generate_stage_representation(mode="list"))
        /World ()
        /World/PrimA (Sphere)
        /World/PrimB (Cube)
        /PrimC (Camera)
        >>>
        >>> print(stage_utils.generate_stage_representation(mode="tree"))
        / ()
        ├─ World ()
        │  ├─ PrimA (Sphere)
        │  ├─ PrimB (Cube)
        ├─ PrimC (Camera)
    """

    def _generate_tree(prim: Usd.Prim, indent: int = 0) -> None:
        prefix = "│  " * (indent - 1) + "├─ " if indent > 0 else ""
        lines.append(f"{prefix}{prim.GetPath().name} ({prim.GetTypeName()})")
        for child in prim.GetChildren():
            _generate_tree(child, indent + 1)

    lines = []
    if mode == "list":
        for prim in get_current_stage().Traverse():
            lines.append(f"{prim.GetPath().pathString} ({prim.GetTypeName()})")
    elif mode == "tree":
        _generate_tree(get_current_stage().GetPrimAtPath("/"), 0)
    else:
        raise ValueError(f"Invalid mode: {mode}")
    return "\n".join(lines)


def create_new_stage(*, template: str | None = None) -> Usd.Stage:
    """Create a new USD stage attached to the USD context.

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
        template: The template to use to create the stage. If ``None``, a new stage is created with nothing.

    Returns:
        New USD stage instance.

    Raises:
        ValueError: When the template is not found.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # create a new stage from the 'gridroom' template
        >>> stage_utils.create_new_stage(template="gridroom")
        Usd.Stage.Open(rootLayer=Sdf.Find('anon:...usd'), ...)

        >>> # get the list of available Kit templates
        >>> import omni.kit.stage_templates
        >>>
        >>> [name for item in omni.kit.stage_templates.get_stage_template_list() for name in item]
        ['empty', 'sunlight', 'default stage']
    """
    # create 'empty' stage
    if template in [None, "gridroom"]:
        omni.usd.get_context().new_stage()
        if template == "gridroom":
            stage_templates.gridroom()
    # create stage from template
    else:
        templates = [name for item in omni.kit.stage_templates.get_stage_template_list() for name in item]
        if template not in templates:
            raise ValueError(f"Template '{template}' not found. Available templates: {templates}")
        omni.kit.stage_templates.new_stage(template=template)
    return omni.usd.get_context().get_stage()


async def create_new_stage_async(*, template: str | None = None) -> Usd.Stage:
    """Create a new USD stage attached to the USD context.

    Backends: :guilabel:`usd`.

    This function is the asynchronous version of :py:func:`create_new_stage`.

    Args:
        template: The template to use to create the stage. If ``None``, a new stage is created with nothing.

    Returns:
        New USD stage instance.

    Raises:
        ValueError: When the template is not found.
    """
    # create 'empty' stage
    if template in [None, "gridroom"]:
        await omni.usd.get_context().new_stage_async()
        if template == "gridroom":
            stage_templates.gridroom()
    # create stage from template
    else:
        templates = [name for item in omni.kit.stage_templates.get_stage_template_list() for name in item]
        if template not in templates:
            raise ValueError(f"Template '{template}' not found. Available templates: {templates}")
        await omni.kit.stage_templates.new_stage_async(template=template)
    await omni.kit.app.get_app().next_update_async()
    return omni.usd.get_context().get_stage()


def open_stage(usd_path: str) -> tuple[bool, Usd.Stage | None]:
    """Open a USD file attached to the USD context.

    Backends: :guilabel:`usd`.

    Args:
        usd_path: USD file path to open.

    Returns:
        Two-elements tuple. 1) Whether the USD file was opened successfully.
        2) Opened USD stage instance or None if the USD file was not opened.

    Raises:
        ValueError: If the USD file does not exist or is not a valid (shallow check).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.storage.native import get_assets_root_path
        >>>
        >>> # open a USD file
        >>> result, stage = stage_utils.open_stage(
        ...     get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        ... )
        >>> result
        True
        >>> stage
        Usd.Stage.Open(rootLayer=Sdf.Find('...'), ...)
    """
    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError(f"The file ({usd_path}) is not USD open-able")
    usd_context = omni.usd.get_context()
    usd_context.disable_save_to_recent_files()
    result = omni.usd.get_context().open_stage(usd_path)
    usd_context.enable_save_to_recent_files()
    return result, usd_context.get_stage()


async def open_stage_async(usd_path: str) -> tuple[bool, Usd.Stage | None]:
    """Open a USD file attached to the USD context.

    Backends: :guilabel:`usd`.

    This function is the asynchronous version of :py:func:`open_stage`.

    Args:
        usd_path: USD file path to open.

    Returns:
        Two-elements tuple. 1) Whether the USD file was opened successfully.
        2) Opened USD stage instance or None if the USD file was not opened.

    Raises:
        ValueError: If the USD file does not exist or is not a valid (shallow check).
    """
    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError(f"The file ({usd_path}) is not USD open-able")
    usd_context = omni.usd.get_context()
    usd_context.disable_save_to_recent_files()
    result, error = await omni.usd.get_context().open_stage_async(usd_path)
    usd_context.enable_save_to_recent_files()
    return result, usd_context.get_stage()


def save_stage(usd_path: str) -> bool:
    """Save the current stage to a USD file.

    Backends: :guilabel:`usd`.

    Args:
        usd_path: USD file path to save the current stage to.

    Returns:
        Whether the stage was saved successfully.

    Example:

    .. code-block:: python

        >>> import os
        >>> import tempfile
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # save the current stage to a USD file
        >>> usd_path = os.path.join(tempfile.gettempdir(), "test.usd")
        >>> stage_utils.save_stage(usd_path)
        True
    """
    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError(f"The file ({usd_path}) is not USD open-able")
    root_layer = get_current_stage(backend="usd").GetRootLayer()
    layer = Sdf.Layer.CreateNew(usd_path)
    layer.TransferContent(root_layer)
    omni.usd.resolve_paths(root_layer.identifier, layer.identifier)
    return layer.Save()


def close_stage() -> bool:
    """Close the stage attached to the USD context.

    Backends: :guilabel:`usd`.

    Returns:
        Whether the stage was closed successfully.
    """
    return omni.usd.get_context().close_stage()


def add_reference_to_stage(
    usd_path: str, path: str, *, prim_type: str = "Xform", variants: list[tuple[str, str]] | None = None
) -> Usd.Prim:
    """Add a USD file reference to the stage at the specified prim path.

    Backends: :guilabel:`usd`.

    .. note::

        This function handles stage units verification to ensure compatibility.

    Args:
        usd_path: USD file path to reference.
        path: Prim path where the reference will be attached.
        prim_type: Prim type to create if the given ``path`` doesn't exist.
        variants: Variants (variant sets and selections) to author on the USD prim.

    Returns:
        USD prim.

    Raises:
        Exception: The USD file might not exist or might not be a valid USD file.
        ValueError: If a variant set or selection is invalid.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.storage.native import get_assets_root_path
        >>>
        >>> prim = stage_utils.add_reference_to_stage(
        ...     usd_path=get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        ...     path="/panda",
        ...     variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
        ... )
    """
    if not Sdf.Path.IsValidPathString(path):
        raise ValueError(f"Prim path ({path}) is not a valid path string")
    sdf_layer = Sdf.Layer.FindOrOpen(usd_path)
    if not sdf_layer:
        raise Exception(
            f"Unable to get Sdf layer. The USD file ({usd_path}) might not exist or is not a valid USD file."
        )
    stage = get_current_stage(backend="usd")
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        prim = stage.DefinePrim(path, prim_type)
    reference = Sdf.Reference(usd_path)
    reference_added = False
    # check for divergent units
    result = get_metrics_assembler_interface().check_layers(
        stage.GetRootLayer().identifier, sdf_layer.identifier, get_stage_id(stage)
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
        result = prim.GetReferences().AddReference(reference)
        if not result:
            raise Exception(f"Unable to add reference to the USD file ({usd_path}).")
    # set variants
    if variants is None:
        variants = []
    prim_utils.set_prim_variants(prim, variants=variants)
    return prim


def is_stage_loading() -> bool:
    """Check whether the stage is loading.

    Backends: :guilabel:`usd`.

    Returns:
        Whether the stage is loading.
    """
    # check for total files to be loaded
    return omni.usd.get_context().get_stage_loading_status()[2] > 0


def get_stage_units() -> tuple[float, float]:
    """Get the stage meters per unit and kilograms per unit currently set.

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

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.get_stage_units()
        (1.0, 1.0)
    """
    stage = get_current_stage(backend="usd")
    return UsdGeom.GetStageMetersPerUnit(stage), UsdPhysics.GetStageKilogramsPerUnit(stage)


def set_stage_units(*, meters_per_unit: float | None = None, kilograms_per_unit: float | None = None) -> None:
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
        meters_per_unit: Meters per unit.
        kilograms_per_unit: Kilograms per unit.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # set stage units to inch and pound respectively
        >>> stage_utils.set_stage_units(meters_per_unit=0.0254, kilograms_per_unit=0.4536)
    """
    stage = get_current_stage(backend="usd")
    if meters_per_unit is not None:
        UsdGeom.SetStageMetersPerUnit(stage, meters_per_unit)
    if kilograms_per_unit is not None:
        UsdPhysics.SetStageKilogramsPerUnit(stage, kilograms_per_unit)


def get_stage_up_axis() -> Literal["Y", "Z"]:
    """Get the stage up axis.

    Backends: :guilabel:`usd`.

    .. note::

        According to the USD specification, only ``"Y"`` and ``"Z"`` axes are supported.

    Returns:
        The stage up axis.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.get_stage_up_axis()
        'Z'
    """
    return UsdGeom.GetStageUpAxis(get_current_stage(backend="usd"))


def set_stage_up_axis(up_axis: Literal["Y", "Z"]) -> None:
    """Set the stage up axis.

    Backends: :guilabel:`usd`.

    .. note::

        According to the USD specification, only ``"Y"`` and ``"Z"`` axes are supported.

    Args:
        up_axis: The stage up axis.

    Raises:
        ValueError: If the up axis is not a valid token.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.set_stage_up_axis("Y")
    """
    if up_axis.upper() not in ["Y", "Z"]:
        raise ValueError(f"Invalid up axis: '{up_axis}'")
    UsdGeom.SetStageUpAxis(get_current_stage(backend="usd"), up_axis.upper())


def get_stage_time_code() -> tuple[float, float, float]:
    """Get the stage time code (start, end, and time codes per second).

    Backends: :guilabel:`usd`.

    Returns:
        The stage time code (start, end, and time codes per second).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.get_stage_time_code()
        (0.0, 100.0, 60.0)
    """
    stage = get_current_stage(backend="usd")
    return stage.GetStartTimeCode(), stage.GetEndTimeCode(), stage.GetTimeCodesPerSecond()


def set_stage_time_code(
    *,
    start_time_code: float | None = None,
    end_time_code: float | None = None,
    time_codes_per_second: float | None = None,
) -> None:
    """Set the stage time code (start, end, and time codes per second).

    Backends: :guilabel:`usd`.

    Args:
        start_time_code: The start time code.
        end_time_code: The end time code.
        time_codes_per_second: The time codes per second.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.set_stage_time_code(start_time_code=0.0, end_time_code=100.0, time_codes_per_second=10.0)
    """
    stage = get_current_stage(backend="usd")
    if start_time_code is not None:
        stage.SetStartTimeCode(start_time_code)
    if end_time_code is not None:
        stage.SetEndTimeCode(end_time_code)
    if time_codes_per_second is not None:
        stage.SetTimeCodesPerSecond(time_codes_per_second)


def define_prim(path: str, type_name: str = "Xform") -> Usd.Prim | usdrt.Usd.Prim:
    """Attempt to define a prim of the specified type at the given path.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Common token values for ``type_name`` are:

    * ``"Camera"``, ``"Mesh"``, ``"PhysicsScene"``, ``"Scope"``, ``"Xform"``
    * Shapes (``"Capsule"``, ``"Cone"``, ``"Cube"``, ``"Cylinder"``, ``"Plane"``, ``"Sphere"``)
    * Lights (``"CylinderLight"``, ``"DiskLight"``, ``"DistantLight"``, ``"DomeLight"``, ``"RectLight"``, ``"SphereLight"``)

    Args:
        path: Absolute prim path.
        type_name: Token identifying the prim type.

    Raises:
        ValueError: If the path is not a valid or absolute path string.
        RuntimeError: If there is already a prim at the given path with a different type.

    Returns:
        Defined prim.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.define_prim("/World/Sphere", type_name="Sphere")
        Usd.Prim(</World/Sphere>)
    """
    if not Sdf.Path.IsValidPathString(path) or not Sdf.Path(path).IsAbsolutePath():
        raise ValueError(f"Prim path ({path}) is not a valid or absolute path string")
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        if prim.GetTypeName() != type_name:
            raise RuntimeError(f"A prim already exists at path ({path}) with type ({prim.GetTypeName()})")
        return prim
    return stage.DefinePrim(path, type_name)


def delete_prim(prim: str | Usd.Prim) -> bool:
    """Delete a prim from the stage.

    Backends: :guilabel:`usd`.

    Args:
        prim: Prim path or prim instance to delete.

    Returns:
        Whether the prim was deleted successfully.

    Raises:
        ValueError: If the target prim is not a valid prim.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.define_prim("/World/Sphere", type_name="Sphere")  # doctest: +NO_CHECK
        >>> stage_utils.delete_prim("/World/Sphere")
        True
    """
    path = prim if isinstance(prim, str) else prim.GetPath().pathString
    # check if target prim is an existing prim
    stage = get_current_stage(backend="usd")
    if not stage.GetPrimAtPath(path).IsValid():
        raise ValueError(f"Prim at path '{path}' is not a valid prim")
    omni.usd.commands.DeletePrimsCommand([path]).do()
    return not stage.GetPrimAtPath(path).IsValid()


def move_prim(target: str | Usd.Prim, destination: str | Usd.Prim) -> tuple[bool, str]:
    """Move a prim to a different location on the stage hierarchy.

    Backends: :guilabel:`usd`.

    The move operation follows the next rules:

    - If the destination exists, the target prim is moved (as a child) into the destination prim.
      Moved prim keeps its name.
    - If the destination does not exist, the target prim is moved to the specified destination path,
      provided that the parent exists (if not, an error is raised). Moved prim is renamed.

    Args:
        target: Prim path or prim instance to move.
        destination: Destination path or prim instance to move the prim to.

    Returns:
        Whether the prim was moved successfully and the new path.

    Raises:
        ValueError: If the target prim is not a valid prim.
        ValueError: If the destination path has unexisting parents.
        ValueError: If the destination path is not a valid path string.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> prim_A = stage_utils.define_prim("/World/A")
        >>> prim_B = stage_utils.define_prim("/World/B")
        >>>
        >>> # move prim A to (into) prim B
        >>> stage_utils.move_prim(prim_A, prim_B)
        (True, '/World/B/A')
        >>> # move prim A next to /World (with name C)
        >>> stage_utils.move_prim("/World/B/A", "/World/C")
        (True, '/World/C')
    """
    target = Sdf.Path(target) if isinstance(target, str) else target.GetPath()
    destination = destination if isinstance(destination, str) else destination.GetPath().pathString
    if not Sdf.Path.IsValidPathString(destination):
        raise ValueError(f"Destination path '{destination}' is not a valid path string")
    destination = Sdf.Path(destination)
    # check if target prim is an existing prim
    stage = get_current_stage(backend="usd")
    if not stage.GetPrimAtPath(target).IsValid():
        raise ValueError(f"Prim at path '{target}' is not a valid prim")
    # check if the destination is an existing prim
    if stage.GetPrimAtPath(destination).IsValid():
        destination = destination.AppendChild(target.name)
    # check if destination has unexisting parents
    elif not stage.GetPrimAtPath(destination.GetParentPath()).IsValid():
        raise ValueError(f"Destination path '{destination}' has unexisting parent '{destination.GetParentPath()}'")
    # move prim
    return omni.usd.commands.MovePrimCommand(path_from=target, path_to=destination).do(), destination.pathString


def generate_next_free_path(path: str | None = None, *, prepend_default_prim: bool = True) -> str:
    """Generate the next free usd path for the current stage.

    Backends: :guilabel:`usd`.

    Args:
        path: Base path to generate the next free path from.
            If empty, pseudo-root or not defined, ``"Prim"`` will be used as the default name.
        prepend_default_prim: Whether to prepend the default prim path to the base path.

    Returns:
        Next free path.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # given the stage: /World/Cube, /World/Cube_01
        >>> stage_utils.define_prim("/World/Cube", type_name="Cube")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/Cube_01", type_name="Cube")  # doctest: +NO_CHECK
        >>>
        >>> # generate the next available path for /World/Cube
        >>> stage_utils.generate_next_free_path("/World/Cube")
        '/World/Cube_02'
    """
    stage = get_current_stage(backend="usd")
    if path in [None, "", "/"]:
        if prepend_default_prim and stage.HasDefaultPrim():
            path = f"{stage.GetDefaultPrim().GetPath().pathString}/Prim"
        else:
            path = "Prim"
    if not Sdf.Path.IsValidPathString(path):
        raise ValueError(f"Prim path ({path}) is not a valid path string")
    return omni.usd.get_stage_next_free_path(stage, path, prepend_default_prim)
