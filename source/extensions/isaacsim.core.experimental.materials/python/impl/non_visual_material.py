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

"""Non visual material module."""

from __future__ import annotations

import csv
import pathlib
from typing import Any

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import Prim
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Sdf, Usd, UsdShade

# non-visual material attribute names
_NON_VISUAL_MATERIAL_PREFIX_SETTING = "/rtx/materialDb/nonVisualMaterialSemantics/prefix"
_DEFAULT_NON_VISUAL_MATERIAL_PREFIX = "omni:simready:nonvisual"


def _get_non_visual_material_prefix(settings: Any | None = None) -> str:
    """Get the USD attribute prefix used for SimReady non-visual material semantics."""
    if settings is None:
        settings = carb.settings.get_settings()
    prefix = settings.get(_NON_VISUAL_MATERIAL_PREFIX_SETTING)
    if not prefix:
        prefix = _DEFAULT_NON_VISUAL_MATERIAL_PREFIX
        settings.set_default_string(_NON_VISUAL_MATERIAL_PREFIX_SETTING, prefix)
    return prefix


_PREFIX = _get_non_visual_material_prefix()
BASE_ATTR, BASE_SPEC = f"{_PREFIX}:base", {}
COATING_ATTR, COATING_SPEC = f"{_PREFIX}:coating", {}
ATTRIBUTE_ATTR, ATTRIBUTE_SPEC = f"{_PREFIX}:attributes", {}


def _parse_specification(path: str) -> dict[str, int]:
    """Parse the non-visual material specification CSV file at the given path.

    Args:
        path: Path to the CSV file.

    Returns:
        Dictionary of non-visual material specifications.
    """
    spec = {}
    with open(path, encoding="utf-8") as file:
        reader = csv.reader(file)
        next(reader)  # skip header
        for row in reader:
            name, index, _ = row
            if name == "<reserved>":
                continue
            spec[name] = int(index)
    return spec


def _as_attribute_list(value: Any) -> list[str]:
    """Normalize a non-visual material ``attributes`` value into a list of attribute tokens.

    Handles the SimReady ``token[]`` array type as well as the previous custom ``string``
    scalar type (for backwards compatibility) and unset attributes.

    Args:
        value: Attribute value read from a USD attribute (``None``, a scalar string, or an
            iterable of tokens).

    Returns:
        List of attribute tokens. Defaults to ``["none"]`` when no attribute is set.
    """
    if value is None:
        return ["none"]
    if isinstance(value, str):
        return [value]
    attributes = [str(item) for item in value]
    return attributes if attributes else ["none"]


class NonVisualMaterial(Prim):
    """High level wrapper for creating/encapsulating non-visual materials.

    Attributes are authored following the SimReady non-visual materials specification
    (see `Non-Visual Sensor Material Attributes
    <https://nvidia.github.io/simready-foundation/latest/capabilities/nonvisual_sensors/nonvisual_materials/capability-nonvisual_materials.html>`_):
    ``base`` and ``coating`` are single ``token`` values, while ``attributes`` is a ``token[]`` array.
    The set of valid values is defined in the `attributes table
    <https://nvidia.github.io/simready-foundation/latest/capabilities/nonvisual_sensors/nonvisual_materials/nonvisual_attributes_table.html>`_.

    .. note::

        When configuring a non-visual material, the base material is required,
        while the coatings and attributes are optional.

    .. hint::

        A non-visual material carries only the SimReady semantic attributes and defines no surface shader.
        Non-visual materials can be applied to visual materials to create a single material with both visual and
        non-visual properties. In this case, the visual material (``PreviewSurface``, ``OmniPBR``, or ``OmniGlass``)
        must be defined before the non-visual material.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        bases: Bases (shape ``(N,)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        coatings: Coatings (shape ``(N,)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        attributes: Attributes. A single string or list of strings is applied to all materials as a shared set of
            attributes; a list of lists (shape ``(N,)``) assigns a per-prim set of attributes.
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.materials import NonVisualMaterial
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create non-visual material at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = NonVisualMaterial(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        bases: str | list[str] | None = None,
        coatings: str | list[str] | None = None,
        attributes: str | list[str] | list[list[str]] | None = None,
    ) -> None:
        # get or create prims
        self._materials = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = self.resolve_paths(paths)
        # - get prims
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                material = self._get_material(stage, path)
                assert material is not None, f"The wrapped prim at path {path} is not a USD Material"
                self._materials.append(material)
        # - create prims
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                UsdShade.Material.Define(stage, path)
                material = self._get_material(stage, path)
                assert material is not None, f"Unable to create non-visual material at path {path}"
                self._materials.append(material)
        # initialize base class
        super().__init__(paths, resolve_paths=False)
        # apply non-visual material API (create attributes if they don't exist)
        self._apply_non_visual_material_api()
        # initialize instance from arguments
        NonVisualMaterial._parse_specifications()
        if bases is not None:
            self.set_bases(bases)
        if coatings is not None:
            self.set_coatings(coatings)
        if attributes is not None:
            self.set_attributes(attributes)

    """
    Properties.
    """

    @property
    def materials(self) -> list[UsdShade.Material]:
        """USD materials encapsulated by the wrapper.

        Returns:
            List of USD materials.

        Example:

        .. code-block:: python

            >>> prims.materials
            [UsdShade.Material(Usd.Prim(</World/prim_0>)),
             UsdShade.Material(Usd.Prim(</World/prim_1>)),
             UsdShade.Material(Usd.Prim(</World/prim_2>))]
        """
        return self._materials

    """
    Static methods.
    """

    @staticmethod
    def encode_material_ids(
        prims: str | Usd.Prim | UsdShade.Material | list[str | Usd.Prim | UsdShade.Material] | NonVisualMaterial,
    ) -> wp.array:
        """Encode material IDs for the given prims.

        Backends: :guilabel:`usd`.

        Args:
            prims: Prim paths, USD prims, or NonVisualMaterial instances.

        Returns:
            Material IDs (shape ``(N, 1)``).

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import NonVisualMaterial
            >>>
            >>> # given a non-visual material with some values set
            >>> prim = NonVisualMaterial(
            ...     "/World/non_visual_material",
            ...     bases="aluminum",
            ...     coatings="paint",
            ...     attributes="emissive",
            ... )
            >>>
            >>> # encode the material ID for the prim
            >>> NonVisualMaterial.encode_material_ids(prim).numpy().item()
            2305
        """

        def _encode(prim: Usd.Prim) -> int:
            base_value = 0
            coating_value = 0
            attribute_value = 0
            if prim.HasAttribute(BASE_ATTR):
                base_value = BASE_SPEC.get(prim.GetAttribute(BASE_ATTR).Get(), 0)
            if prim.HasAttribute(COATING_ATTR):
                coating_value = COATING_SPEC.get(prim.GetAttribute(COATING_ATTR).Get(), 0)
            if prim.HasAttribute(ATTRIBUTE_ATTR):
                # attributes are a bitfield: OR together the flag of each applied attribute
                for attribute in _as_attribute_list(prim.GetAttribute(ATTRIBUTE_ATTR).Get()):
                    attribute_value |= ATTRIBUTE_SPEC.get(attribute, 0)
            base_value = base_value & 0xFF  # 8 bits (0-255)
            coating_value = coating_value & 0x7  # 3 bits (0-7)
            attribute_value = attribute_value & 0x1F  # 5 bits (0-31)
            return base_value + (coating_value << 8) + (attribute_value << 11)

        NonVisualMaterial._parse_specifications()
        if isinstance(prims, NonVisualMaterial):
            prims = prims.prims
        elif isinstance(prims, (str, Usd.Prim, UsdShade.Material)):
            prims = [prim_utils.get_prim_at_path(prims)]
        elif isinstance(prims, list):
            prims = [prim_utils.get_prim_at_path(prim) for prim in prims]
        else:
            raise ValueError(f"Invalid type: {type(prims)}")
        return ops_utils.place([_encode(prim) for prim in prims], dtype=wp.uint16, device="cpu").reshape((-1, 1))

    @staticmethod
    def decode_material_ids(ids: int | list | np.ndarray | wp.array) -> list[tuple[str, str, list[str]]]:
        """Decode material IDs into base, coating, and attribute values.

        Backends: :guilabel:`usd`.

        Args:
            ids: Material IDs (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).

        Returns:
            List of tuples containing (base, coating, attributes), where ``base`` and ``coating`` are strings
            and ``attributes`` is a list of strings decoded from the attribute bitfield (shape ``(N,)``).

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import NonVisualMaterial
            >>>
            >>> NonVisualMaterial.decode_material_ids(2305)
            [('aluminum', 'paint', ['emissive'])]
        """

        def _get_first_key_by_value(data: dict, value: int) -> str:
            for k, v in data.items():
                if v == value:
                    return k
            return "none"

        def _decode_attributes(bits: int) -> list[str]:
            # attributes are a bitfield: collect every attribute flag set in the value
            attributes = [name for name, flag in ATTRIBUTE_SPEC.items() if flag != 0 and (bits & flag) == flag]
            return attributes if attributes else ["none"]

        def _decode(id: int) -> tuple[str, str, list[str]]:
            if id < 0 or id > 0xFFFF:
                raise ValueError(f"The given material ID ({id}) is outside valid unsigned integer 16-bit range")
            base_value = id & 0xFF  # bits 0-7
            coating_value = (id >> 8) & 0x7  # bits 8-10
            attribute_value = (id >> 11) & 0x1F  # bits 11-15
            return (
                _get_first_key_by_value(BASE_SPEC, base_value),
                _get_first_key_by_value(COATING_SPEC, coating_value),
                _decode_attributes(attribute_value),
            )

        NonVisualMaterial._parse_specifications()
        ids = ops_utils.place(ids, dtype=wp.uint16, device="cpu").numpy().reshape((-1, 1))
        return [_decode(id.item()) for id in ids]

    """
    Methods.
    """

    def set_bases(self, bases: str | list[str], *, indices: int | list | np.ndarray | wp.array | None = None) -> None:
        """Set the base materials for the non-visual materials.

        Backends: :guilabel:`usd`.

        Args:
            bases: Bases (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the bases for all prims to 'aluminum'
            >>> prims.set_bases("aluminum")
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        bases = [bases] if isinstance(bases, str) else bases
        for base in bases:
            if not base in BASE_SPEC:
                raise ValueError(f"Invalid base: '{base}'. Supported bases: {list(BASE_SPEC.keys())}")
        bases = np.broadcast_to(np.array(bases, dtype=object), (indices.shape[0],))
        for i, index in enumerate(indices.numpy()):
            self.prims[index].GetAttribute(BASE_ATTR).Set(bases[i])

    def get_bases(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> list[str]:
        """Get the base materials for the non-visual materials.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of base materials (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the bases of all prims
            >>> prims.get_bases()
            ['none', 'none', 'none']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        bases = np.empty((indices.shape[0],), dtype=object)
        for i, index in enumerate(indices.numpy()):
            bases[i] = self.prims[index].GetAttribute(BASE_ATTR).Get()
        return bases.tolist()

    def set_coatings(
        self, coatings: str | list[str], *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the coatings for the non-visual materials.

        Backends: :guilabel:`usd`.

        Args:
            coatings: Coatings (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the coatings for all prims to 'paint'
            >>> prims.set_coatings("paint")
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        coatings = [coatings] if isinstance(coatings, str) else coatings
        for coating in coatings:
            if not coating in COATING_SPEC:
                raise ValueError(f"Invalid coating: '{coating}'. Supported coatings: {list(COATING_SPEC.keys())}")
        coatings = np.broadcast_to(np.array(coatings, dtype=object), (indices.shape[0],))
        for i, index in enumerate(indices.numpy()):
            self.prims[index].GetAttribute(COATING_ATTR).Set(coatings[i])

    def get_coatings(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> list[str]:
        """Get the coatings for the non-visual materials.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of coatings (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the coatings of all prims
            >>> prims.get_coatings()
            ['none', 'none', 'none']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        coatings = np.empty((indices.shape[0],), dtype=object)
        for i, index in enumerate(indices.numpy()):
            coatings[i] = self.prims[index].GetAttribute(COATING_ATTR).Get()
        return coatings.tolist()

    def set_attributes(
        self,
        attributes: str | list[str] | list[list[str]],
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the attributes for the non-visual materials.

        Attributes are stored as a ``token[]`` array, so each material can have multiple attributes applied
        (following the SimReady non-visual materials spec).

        Backends: :guilabel:`usd`.

        Args:
            attributes: Attributes to apply. A single string or list of strings is broadcast to all processed
                prims as a shared set of attributes; a list of lists (shape ``(N,)``) assigns a per-prim set of
                attributes. If the input shape is smaller than expected, data will be broadcasted (following
                NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the attributes for all prims to 'emissive'
            >>> prims.set_attributes("emissive")
            >>>
            >>> # set multiple attributes for all prims
            >>> prims.set_attributes(["emissive", "retroreflective"])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        # normalize the input into a list of per-prim attribute sets
        if isinstance(attributes, str):
            attribute_sets = [[attributes]]
        elif all(isinstance(item, str) for item in attributes):
            attribute_sets = [list(attributes)]
        else:
            attribute_sets = [list(item) for item in attributes]
        # validate attribute tokens
        for attribute_set in attribute_sets:
            for attribute in attribute_set:
                if not attribute in ATTRIBUTE_SPEC:
                    raise ValueError(
                        f"Invalid attribute: '{attribute}'. Supported attributes: {list(ATTRIBUTE_SPEC.keys())}"
                    )
        # broadcast the attribute sets to the number of processed prims
        source = np.empty((len(attribute_sets),), dtype=object)
        for i, attribute_set in enumerate(attribute_sets):
            source[i] = attribute_set
        attribute_sets = np.broadcast_to(source, (indices.shape[0],))
        for i, index in enumerate(indices.numpy()):
            self.prims[index].GetAttribute(ATTRIBUTE_ATTR).Set(list(attribute_sets[i]))

    def get_attributes(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> list[list[str]]:
        """Get the attributes for the non-visual materials.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of per-prim attribute lists (shape ``(N,)``). Materials authored using the previous custom
            ``string`` attribute type are read as a single-element list for backwards compatibility.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the attributes of all prims
            >>> prims.get_attributes()
            [['none'], ['none'], ['none']]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        attributes = []
        for index in indices.numpy():
            attributes.append(_as_attribute_list(self.prims[index].GetAttribute(ATTRIBUTE_ATTR).Get()))
        return attributes

    """
    Internal methods.
    """

    def _apply_non_visual_material_api(self) -> None:
        """Apply non-visual material API to the wrapped prims.

        Attributes are authored using the SimReady non-visual materials spec types:
        ``base`` and ``coating`` are single ``token`` values, while ``attributes`` is a
        ``token[]`` array. Existing attributes are left untouched to preserve backwards
        compatibility with prims authored using the previous custom ``string`` types.
        """
        for prim in self.prims:
            if not prim.HasAttribute(BASE_ATTR):
                prim.CreateAttribute(BASE_ATTR, Sdf.ValueTypeNames.Token, custom=True).Set("none")
            if not prim.HasAttribute(COATING_ATTR):
                prim.CreateAttribute(COATING_ATTR, Sdf.ValueTypeNames.Token, custom=True).Set("none")
            if not prim.HasAttribute(ATTRIBUTE_ATTR):
                prim.CreateAttribute(ATTRIBUTE_ATTR, Sdf.ValueTypeNames.TokenArray, custom=True).Set(["none"])

    """
    Internal static methods.
    """

    @staticmethod
    def _get_material(stage: Usd.Stage, path: str) -> UsdShade.Material | None:
        """Get the material for a given material path.

        Args:
            stage: USD stage containing the material.
            path: Path to the material prim.

        Returns:
            USD Material, if found, None otherwise.
        """
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.IsA(UsdShade.Material):
            return UsdShade.Material(prim)
        return None

    @staticmethod
    def _parse_specifications() -> None:
        """Parse the non-visual material specifications."""
        ext_path = pathlib.Path(app_utils.get_extension_path("isaacsim.core.experimental.materials"))
        if not BASE_SPEC:
            BASE_SPEC.update(_parse_specification(str(ext_path / "data" / "specifications" / "base.csv")))
        if not COATING_SPEC:
            COATING_SPEC.update(_parse_specification(str(ext_path / "data" / "specifications" / "coating.csv")))
        if not ATTRIBUTE_SPEC:
            ATTRIBUTE_SPEC.update(_parse_specification(str(ext_path / "data" / "specifications" / "attribute.csv")))
