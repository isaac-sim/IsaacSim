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

"""Functions for working with USD/USDRT prims."""

from __future__ import annotations

import re
from collections.abc import Callable
from typing import Any, Literal

import usdrt
from pxr import Sdf, Usd, UsdPhysics

from . import foundation as foundation_utils
from . import stage as stage_utils


def set_prim_variants(prim: str | Usd.Prim, *, variants: list[tuple[str, str]]) -> None:
    """Set/author variants (variant sets and selections) on a USD prim.

    Backends: :guilabel:`usd`.

    Args:
        prim: Prim path or prim instance.
        variants: Variants (variant sets and selections) to author on the USD prim.

    Raises:
        ValueError: If a variant set or selection is invalid.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.storage.native import get_assets_root_path
        >>>
        >>> stage_utils.open_stage(
        ...     get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        ... )  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.set_prim_variants("/panda", variants=[("Mesh", "Quality"), ("Gripper", "AlternateFinger")])
    """
    prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(prim) if isinstance(prim, str) else prim
    available_variant_sets = prim.GetVariantSets().GetNames()
    for variant_set, variant_selection in variants:
        if variant_set not in available_variant_sets:
            raise ValueError(f"Invalid variant set: '{variant_set}'. Available sets: {available_variant_sets}")
        available_variant_selections = prim.GetVariantSet(variant_set).GetVariantNames()
        if variant_selection and variant_selection not in available_variant_selections:
            raise ValueError(
                f"Invalid variant selection (variant set: '{variant_set}'): '{variant_selection}'. "
                f"Available selections (variant set: '{variant_set}'): {available_variant_selections}"
            )
        prim.GetVariantSet(variant_set).SetVariantSelection(variant_selection)


def get_prim_variants(prim: str | Usd.Prim) -> list[tuple[str, str]]:
    """Get variants (variant sets and selections) authored on a USD prim.

    Backends: :guilabel:`usd`.

    Args:
        prim: Prim path or prim instance.

    Returns:
        Authored variants (variant sets and selections).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.storage.native import get_assets_root_path
        >>>
        >>> stage_utils.open_stage(
        ...     get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        ... )  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.get_prim_variants("/panda")
        [('Gripper', 'Default'), ('Mesh', 'Performance')]
    """
    prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(prim) if isinstance(prim, str) else prim
    return [
        (variant_set, prim.GetVariantSet(variant_set).GetVariantSelection())
        for variant_set in sorted(prim.GetVariantSets().GetNames())
    ]


def get_prim_variant_collection(prim: str | Usd.Prim) -> dict[str, list[str]]:
    """Get variant collection (all variant sets and selections) for a USD prim.

    Backends: :guilabel:`usd`.

    Args:
        prim: Prim path or prim instance.

    Returns:
        Variant collection (variant sets and selections).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.storage.native import get_assets_root_path
        >>>
        >>> stage_utils.open_stage(
        ...     get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        ... )  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.get_prim_variant_collection("/panda")
        {'Mesh': ['Performance', 'Quality'], 'Gripper': ['AlternateFinger', 'Default', 'None', 'Robotiq_2F_85']}
    """
    prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(prim) if isinstance(prim, str) else prim
    return {
        variant_set: prim.GetVariantSet(variant_set).GetVariantNames()
        for variant_set in prim.GetVariantSets().GetNames()
    }


def get_prim_at_path(
    path: str | Sdf.Path | Usd.Prim | usdrt.Usd.Prim | Usd.SchemaBase | usdrt.Usd.SchemaBase,
) -> Usd.Prim | usdrt.Usd.Prim:
    """Get the prim at a given path.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    .. hint::

        To maximize robustness and versatility, this method supports either a USD/USDRT prim or schema
        instance as input. In such a case, the held prim is returned.

    Args:
        path: Prim path. It also accepts a prim/schema instance as input.

    Returns:
        Prim.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.define_prim(f"/World/Cube", "Cube")  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.get_prim_at_path("/World/Cube")
        Usd.Prim(</World/Cube>)
    """
    if isinstance(path, (Usd.Prim, usdrt.Usd.Prim)):
        return path
    elif isinstance(path, (Usd.SchemaBase, usdrt.Usd.SchemaBase)):
        return path.GetPrim()
    return stage_utils.get_current_stage().GetPrimAtPath(path)


def get_prim_path(prim: Usd.Prim | usdrt.Usd.Prim | Usd.SchemaBase | usdrt.Usd.SchemaBase | str | Sdf.Path) -> str:
    """Get the path of a given prim.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    .. hint::

        To maximize robustness and versatility, this method supports either a USD/USDRT schema
        instance or a path as input. In such a case, the held prim path is returned.

    Args:
        prim: Prim instance. It also accepts a schema instance or a path as input.

    Returns:
        Prim path.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> prim = stage_utils.define_prim(f"/World/Cube", "Cube")
        >>> prim_utils.get_prim_path(prim)
        '/World/Cube'
    """
    if isinstance(prim, (str, Sdf.Path)):
        return prim.pathString if isinstance(prim, Sdf.Path) else prim
    elif isinstance(prim, usdrt.Usd.SchemaBase):  # USDRT SchemaBase uses `GetPrimPath` instead of `GetPath`
        return prim.GetPrimPath().pathString
    return prim.GetPath().pathString


def find_matching_prim_paths(path: str, *, traverse: bool = False) -> list[str]:
    """Find all the prim paths in the stage that match the given (regex) path.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        path: Path to match against the stage. It can be a regex expression or a valid prim path.
        traverse: Whether to traverse the stage hierarchy to find all matching prims. If ``True``, the function will
            return all the prim paths in the stage that match the given (regex) path, including its descendants, if any.
            Otherwise, only the paths of the first matching prim (performing a segment-wise search) will be returned.

    Returns:
        List of matching prim paths.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # / (root prim)
        >>> #  |-- World
        >>> #  |    |-- prim_0
        >>> #  |    |    |-- prim_a
        >>> #  |    |    |-- prim_b
        >>> stage_utils.define_prim("/World/prim_0/prim_a")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/prim_0/prim_b")  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.find_matching_prim_paths("/World/prim_.*")
        ['/World/prim_0']
        >>> prim_utils.find_matching_prim_paths("/World/prim_.*", traverse=True)
        ['/World/prim_0', '/World/prim_0/prim_a', '/World/prim_0/prim_b']
        >>>
        >>> prim_utils.find_matching_prim_paths(".*_[ab]")
        []
        >>> prim_utils.find_matching_prim_paths(".*_[ab]", traverse=True)
        ['/World/prim_0/prim_a', '/World/prim_0/prim_b']
    """
    stage = stage_utils.get_current_stage()
    # check for a valid prim path first to avoid unnecessary regex search
    if Sdf.Path.IsValidPathString(path):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            if traverse:
                prim_range = Usd.PrimRange(prim) if isinstance(prim, Usd.Prim) else usdrt.Usd.PrimRange(prim)
                return [prim.GetPath().pathString for prim in prim_range]
            else:
                return [path]
    # regex search
    if traverse:
        pattern = re.compile(path)
        return [res.string for prim in stage.Traverse() if (res := pattern.match(prim.GetPath().pathString))]
    else:
        roots, matches = ["/"], []
        patterns = [re.compile(f"^{token}$") for token in path.strip("/").split("/")]
        for i, pattern in enumerate(patterns):
            for root in roots:
                for child in stage.GetPrimAtPath(root).GetChildren():
                    if pattern.fullmatch(child.GetName()):
                        matches.append(child.GetPath().pathString)
            if i < len(patterns) - 1:
                roots, matches = matches, []
        return matches


def get_all_matching_child_prims(
    prim: str | Usd.Prim | usdrt.Usd.Prim,
    *,
    predicate: Callable[[Usd.Prim | usdrt.Usd.Prim, str], bool],
    include_self: bool = False,
    max_depth: int | None = None,
) -> list[Usd.Prim | usdrt.Usd.Prim]:
    """Get all prim children of the given prim (excluding itself by default) that pass the predicate.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        prim: Prim path or prim instance.
        predicate: Function to test the prims against.
            The function should take two positional arguments: a prim instance and its path.
            The function should return a boolean value indicating whether a prim passes the predicate.
        include_self: Whether to include the given prim in the search.
        max_depth: Maximum depth to search (current prim is at depth 0). If ``None``, search till the end of the tree.

    Returns:
        List of matching prim children.

    Raises:
        ValueError: If ``max_depth`` is defined and is less than 0.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # define some prims
        >>> stage_utils.define_prim("/World/Cube_0", "Cube")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/Cube_0/Cube_1", "Cube")  # doctest: +NO_CHECK
        >>>
        >>> # get all `/World`'s child prims of type Cube
        >>> predicate = lambda prim, path: prim.GetTypeName() == "Cube"
        >>> prim_utils.get_all_matching_child_prims("/World", predicate=predicate)
        [Usd.Prim(</World/Cube_0>), Usd.Prim(</World/Cube_0/Cube_1>)]
        >>>
        >>> # get all `/World`'s child prims of type Cube with max depth 1
        >>> prim_utils.get_all_matching_child_prims("/World", predicate=predicate, max_depth=1)
        [Usd.Prim(</World/Cube_0>)]
    """
    if max_depth is not None and max_depth < 0:
        raise ValueError("If defined, 'max_depth' must be greater or equal to 0")
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    stack = [(prim, 0)] if include_self else [(child, 1) for child in prim.GetChildren()]
    children = []
    while stack:
        prim, current_depth = stack.pop(0)
        if max_depth is not None and current_depth > max_depth:
            break
        if predicate(prim, get_prim_path(prim)):
            children.append(prim)
        stack.extend([(child, current_depth + 1) for child in prim.GetChildren()])
    return children


def get_first_matching_child_prim(
    prim: str | Usd.Prim | usdrt.Usd.Prim,
    *,
    predicate: Callable[[Usd.Prim | usdrt.Usd.Prim, str], bool],
    include_self: bool = False,
) -> Usd.Prim | usdrt.Usd.Prim | None:
    """Get the first prim child of the given prim (excluding itself by default) that passes the predicate.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        prim: Prim path or prim instance.
        predicate: Function to test the prims against.
            The function should take two positional arguments: a prim instance and its path.
            The function should return a boolean value indicating whether a prim passes the predicate.
        include_self: Whether to include the given prim in the search.

    Returns:
        First prim child or ``None`` if no prim child passes the predicate.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # define some prims
        >>> stage_utils.define_prim("/World/Cube", "Cube")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/Cylinder", "Cylinder")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/Sphere", "Sphere")  # doctest: +NO_CHECK
        >>>
        >>> # get the first `/World`'s child prim of type Sphere
        >>> predicate = lambda prim, path: prim.GetTypeName() == "Sphere"
        >>> prim_utils.get_first_matching_child_prim("/World", predicate=predicate)
        Usd.Prim(</World/Sphere>)
    """
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    stack = [prim] if include_self else prim.GetChildren()
    while stack:
        prim = stack.pop(0)
        if predicate(prim, get_prim_path(prim)):
            return prim
        stack.extend(prim.GetChildren())
    return None


def get_first_matching_parent_prim(
    prim: str | Usd.Prim | usdrt.Usd.Prim,
    *,
    predicate: Callable[[Usd.Prim | usdrt.Usd.Prim, str], bool],
    include_self: bool = False,
) -> Usd.Prim | usdrt.Usd.Prim | None:
    """Get the first prim parent of the given prim (excluding itself by default) that passes the predicate.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    .. warning::

        The root prim (``/``) is not considered a valid parent prim but a pseudo-root prim.
        Therefore, it is not taken into account by this function, and any match for this prim will return ``None``.

    Args:
        prim: Prim path or prim instance.
        predicate: Function to test the prims against.
            The function should take two positional arguments: a prim instance and its path.
            The function should return a boolean value indicating whether a prim passes the predicate.
        include_self: Whether to include the given prim in the search.

    Returns:
        First prim parent or ``None`` if no prim parent passes the predicate.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> # define some nested prims
        >>> stage_utils.define_prim("/World/Cube", "Cube")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/Cube/Cylinder", "Cylinder")  # doctest: +NO_CHECK
        >>> stage_utils.define_prim("/World/Cube/Cylinder/Sphere", "Sphere")  # doctest: +NO_CHECK
        >>>
        >>> # get the first `Sphere`'s parent prim of type Cube
        >>> predicate = lambda prim, path: prim.GetTypeName() == "Cube"
        >>> prim_utils.get_first_matching_parent_prim("/World/Cube/Cylinder/Sphere", predicate=predicate)
        Usd.Prim(</World/Cube>)
    """
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    if not include_self:
        prim = prim.GetParent()
    while prim.GetPath().pathString != "/":  # prim.IsPseudoRoot() is not implemented in USDRT
        if predicate(prim, get_prim_path(prim)):
            return prim
        prim = prim.GetParent()
    return None


def has_api(
    prim: str | Usd.Prim, api: str | type | list[str | type], *, test: Literal["all", "any", "none"] = "all"
) -> bool:
    """Check if a prim has or not the given API schema(s) applied.

    Backends: :guilabel:`usd`.

    Args:
        prim: Prim path or prim instance.
        api: API schema name or type, or a list of them.
        test: Checking operation to test for. Supported values are:

            - ``"all"``: All APIs must be present.
            - ``"any"``: Any API must be present.
            - ``"none"``: No APIs must be present.

    Returns:
        Whether the prim has or not (depending on the test) the given API schema applied.

    Raises:
        ValueError: If the test operation is invalid.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from pxr import UsdLux
        >>>
        >>> prim = stage_utils.define_prim("/World/Light", "SphereLight")
        >>> prim_utils.has_api(prim, UsdLux.LightAPI)
        True
    """
    prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(prim) if isinstance(prim, str) else prim
    # get applied status
    status = []
    applied_schemas = prim.GetAppliedSchemas()
    for item in api if isinstance(api, (list, tuple)) else [api]:
        if isinstance(item, str):
            status.append(item in applied_schemas)
        else:
            status.append(prim.HasAPI(item))
    # test condition
    if test == "all":
        return all(status)
    elif test == "any":
        return any(status)
    elif test == "none":
        return not any(status)
    else:
        raise ValueError(f"Invalid test operation: '{test}'")


def ensure_api(prim: str | Usd.Prim, api: type, *args: Any, **kwargs: Any) -> Any:
    """Ensure that a prim has the specified API schema applied.

    Backends: :guilabel:`usd`.

    If a prim doesn't have the API schema, it will be applied.
    If it already has it, the existing API schema will be returned.

    Args:
        prim: Prim path or prim instance.
        api: The API schema type to ensure.
        *args: Additional positional arguments passed to API schema when applying it.
        **kwargs: Additional keyword arguments passed to API schema when applying it.

    Returns:
        API schema object.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from pxr import UsdLux
        >>>
        >>> prim = stage_utils.define_prim("/World/Light", "Xform")
        >>> prim_utils.ensure_api(prim, UsdLux.LightAPI)
        UsdLux.LightAPI(Usd.Prim(</World/Light>))
    """
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    return api(prim, *args, **kwargs) if prim.HasAPI(api) else api.Apply(prim, *args, **kwargs)


def create_prim_attribute(
    prim: str | Usd.Prim | usdrt.Usd.Prim,
    *,
    name: str,
    type_name: Sdf.ValueTypeName | usdrt.Sdf.ValueTypeName,
    exist_ok: bool = True,
) -> Usd.Attribute | usdrt.Usd.Attribute:
    """Create a new attribute on a USD prim.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        prim: Prim path or prim instance.
        name: Name of the attribute to create.
        type_name: Type of the attribute to create.
        exist_ok: Whether to do not raise an error if the attribute already exists.

    Returns:
        Created attribute, or the existing attribute if it already exists (and ``exist_ok`` is ``True``).

    Raises:
        RuntimeError: If the attribute already exists and ``exist_ok`` is ``False``.
        ValueError: If the attribute already exists with a different type name (and ``exist_ok`` is ``True``).
    """
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    type_name = foundation_utils.value_type_name_to_str(type_name)
    # check if attribute already exists
    if prim.HasAttribute(name):
        if not exist_ok:
            raise RuntimeError(f"Attribute '{name}' already exists")
        attribute = prim.GetAttribute(name)
        if foundation_utils.value_type_name_to_str(attribute.GetTypeName()) != type_name:
            raise ValueError(
                f"Attribute '{name}' already exists with type '{attribute.GetTypeName()}', "
                f"but attempting to create it with type '{type_name}'"
            )
    # create attribute
    else:
        type_name = foundation_utils.resolve_value_type_name(
            type_name, backend="usd" if isinstance(prim, Usd.Prim) else "usdrt"
        )
        attribute = prim.CreateAttribute(name, type_name, custom=True)
    return attribute


def get_prim_attribute_value(prim: str | Usd.Prim | usdrt.Usd.Prim, attribute_name: str) -> Any:
    """Get the value of a prim attribute.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    For vector and matrix types (e.g., float3, matrix4d, quatf), the value is returned as a Python list.
    For scalar and other types, the raw attribute value is returned.

    Args:
        prim: Prim path or prim instance.
        attribute_name: Name of the attribute to get.

    Returns:
        The attribute value. Vector and matrix types are returned as lists.

    Raises:
        ValueError: If the prim does not have the specified attribute.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.define_prim("/World/Cube", "Cube")  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.get_prim_attribute_value("/World/Cube", "size")
        2.0
    """
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    if not prim.HasAttribute(attribute_name):
        raise ValueError(f"Prim at path '{prim.GetPath()}' does not have attribute '{attribute_name}'")
    attr = prim.GetAttribute(attribute_name)
    return attr.Get()


def get_prim_attribute_names(prim: str | Usd.Prim | usdrt.Usd.Prim) -> list[str]:
    """Get all valid attribute names for a prim.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        prim: Prim path or prim instance.

    Returns:
        Attribute names authored on the prim.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.define_prim("/World/Cube", "Cube")  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.get_prim_attribute_names("/World/Cube")
        ['doubleSided', 'extent', 'orientation', 'primvars:displayColor', 'primvars:displayOpacity', 'purpose', 'size', 'visibility', 'xformOpOrder']
    """
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    return [attr.GetName() for attr in prim.GetAttributes()]


def is_prim_non_root_articulation_link(prim: str | Usd.Prim | usdrt.Usd.Prim) -> bool:
    """Check whether a prim corresponds to a non-root link in an articulation.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    This function returns ``True`` only if all the following conditions are met:

    - The prim belongs to an articulation.
    - The prim is a link (has the ``RigidBodyAPI`` applied).
    - The prim is related to a joint.

    .. warning::

        While a ``True`` return value guarantees that the prim is a non-root link in an articulation,
        a ``False`` return value does not guarantee that the prim is an articulation root link.

    Args:
        prim: Prim path or prim instance.

    Returns:
        Whether the prim corresponds to a non-root link in an articulation.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.prim as prim_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.storage.native import get_assets_root_path
        >>>
        >>> stage_utils.open_stage(
        ...     get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        ... )  # doctest: +NO_CHECK
        >>>
        >>> prim_utils.is_prim_non_root_articulation_link("/panda")
        False
        >>> prim_utils.is_prim_non_root_articulation_link("/panda/panda_link0")
        True
    """
    prim = stage_utils.get_current_stage().GetPrimAtPath(prim) if isinstance(prim, str) else prim
    backend = "usd" if isinstance(prim, Usd.Prim) else "usdrt"
    # check if the prim belongs to an articulation
    articulation_root_api = (
        UsdPhysics.ArticulationRootAPI if backend == "usd" else usdrt.UsdPhysics.ArticulationRootAPI.GetSchemaTypeName()
    )
    parent = get_first_matching_parent_prim(prim, predicate=lambda prim, _: prim.HasAPI(articulation_root_api))
    if parent is None:
        return False
    # check if the prim is a link (rigid body)
    rigid_body_api = UsdPhysics.RigidBodyAPI if backend == "usd" else usdrt.UsdPhysics.RigidBodyAPI.GetSchemaTypeName()
    if not prim.HasAPI(rigid_body_api):
        return False
    # check if the prim is not a root link
    joint_type = UsdPhysics.Joint if backend == "usd" else usdrt.UsdPhysics.Joint.GetSchemaTypeName()
    joint_class = UsdPhysics.Joint if backend == "usd" else usdrt.UsdPhysics.Joint
    joint_prims = get_all_matching_child_prims(parent, predicate=lambda prim, _: prim.IsA(joint_type))
    for joint_prim in joint_prims:
        joint = joint_class(joint_prim)
        if joint_prim.HasAttribute("physics:excludeFromArticulation") and joint.GetExcludeFromArticulationAttr().Get():
            continue
        body_targets = joint.GetBody0Rel().GetTargets() + joint.GetBody1Rel().GetTargets()
        for target in body_targets:
            if str(target) == get_prim_path(prim):
                return True
    return False
