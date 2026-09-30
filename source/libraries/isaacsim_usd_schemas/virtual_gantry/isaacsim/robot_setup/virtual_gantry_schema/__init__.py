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

"""``IsaacVirtualGantry`` USD schema.

Codeless schema for the virtual gantry: an ``Xform``-derived typed prim whose
world transform is the anchor of a one-sided spring-damper rope. Kept out of
``isaacsim.robot.schema`` so that schema stays limited to describing robots.

The owning extension loads at ``order = -100`` (the schema tier) so the prim
type is registered before anything that depends on it starts. USD builds its
concrete prim-type catalog once, at startup: a schema plugin registered later
gets a TfType but no prim definition, and the type then fails ``IsA(Xformable)``
with no error reported.
"""

import json
import logging
import os
from collections.abc import Iterable
from enum import Enum
from typing import Any

from pxr import Plug, Sdf, Usd

logger = logging.getLogger(__name__)

_attr_prefix = "isaac"


def _register_plugin_path(path: str) -> None:
    """Register the USD plugin at ``path`` if it is not already registered.

    Args:
        path: Filesystem path to process.
    """
    pluginfo_path = os.path.join(path, "plugInfo.json")
    if not os.path.exists(pluginfo_path):
        return
    try:
        with open(pluginfo_path) as file_handle:
            json_content = "".join(line for line in file_handle if not line.strip().startswith("#"))
        data = json.loads(json_content)
        names = {p["Name"] for p in data.get("Plugins", []) if "Name" in p}
        if names and names.issubset({p.name for p in Plug.Registry().GetAllPlugins()}):
            return
    except Exception:  # noqa: BLE001 — fall through to an unconditional register.
        pass
    if not Plug.Registry().RegisterPlugins(path):
        logger.error(f"No plugins found at path {path}")


def _register_plugins(ext_path: str) -> None:
    """Register the schema resources from every layout the build may produce.

    Args:
        ext_path: Path to the extension module.
    """
    _register_plugin_path(os.path.join(os.path.dirname(__file__), "virtual_gantry_schema"))
    _register_plugin_path(os.path.join(ext_path, "usd", "schema", "isaac", "virtual_gantry_schema"))
    _register_plugin_path(os.path.join(ext_path, "usd", "plugins", "virtual_gantry_schema", "resources"))
    _register_plugin_path(os.path.join(ext_path, "virtual_gantry_schema"))


_ext_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
_register_plugins(_ext_path)

_SCHEMA_USDA_NAME = "VirtualGantrySchema.usda"


def _schema_layer() -> Any:
    """Open the authored schema layer, searching the same paths as the plugin.

    Returns:
        The resulting value.
    """
    for base in (
        os.path.join(os.path.dirname(__file__), "virtual_gantry_schema"),
        os.path.join(_ext_path, "usd", "schema", "isaac", "virtual_gantry_schema"),
        os.path.join(_ext_path, "usd", "plugins", "virtual_gantry_schema", "resources"),
        os.path.join(_ext_path, "virtual_gantry_schema"),
    ):
        candidate = os.path.join(base, _SCHEMA_USDA_NAME)
        if os.path.exists(candidate):
            return Sdf.Layer.FindOrOpen(candidate)
    return None


class Classes(Enum):
    """Schema classes provided by this extension."""

    VIRTUAL_GANTRY = "IsaacVirtualGantry"
    """Token for the IsaacVirtualGantry schema class."""


class Attributes(Enum):
    """``isaac:gantry:*`` attributes of an ``IsaacVirtualGantry`` prim."""

    GANTRY_ROPE_LENGTH = (f"{_attr_prefix}:gantry:ropeLength", "Rope Length", Sdf.ValueTypeNames.Float)
    """Virtual gantry slack rope length attribute with float value type."""
    GANTRY_STIFFNESS = (f"{_attr_prefix}:gantry:stiffness", "Stiffness", Sdf.ValueTypeNames.Float)
    """Virtual gantry spring stiffness (kp) attribute with float value type."""
    GANTRY_DAMPING = (f"{_attr_prefix}:gantry:damping", "Damping", Sdf.ValueTypeNames.Float)
    """Virtual gantry damping (kd) attribute with float value type."""
    GANTRY_BODY_OFFSET = (f"{_attr_prefix}:gantry:bodyOffset", "Body Offset", Sdf.ValueTypeNames.Float3)
    """Virtual gantry body-local attach offset attribute with float3 value type."""
    GANTRY_ENABLED = (f"{_attr_prefix}:gantry:enabled", "Enabled", Sdf.ValueTypeNames.Bool)
    """Virtual gantry enabled attribute with boolean value type."""
    GANTRY_EMA_ALPHA = (f"{_attr_prefix}:gantry:emaAlpha", "EMA Alpha", Sdf.ValueTypeNames.Float)
    """Virtual gantry EMA smoothing factor attribute with float value type."""
    GANTRY_MIN_ROPE_LENGTH = (f"{_attr_prefix}:gantry:minRopeLength", "Min Rope Length", Sdf.ValueTypeNames.Float)
    """Virtual gantry minimum rope length attribute with float value type."""
    GANTRY_VISUALIZE = (f"{_attr_prefix}:gantry:visualize", "Visualize", Sdf.ValueTypeNames.Bool)
    """Virtual gantry visualize attribute with boolean value type."""
    GANTRY_STATUS = (f"{_attr_prefix}:gantry:status", "Status", Sdf.ValueTypeNames.Token)
    """Virtual gantry runtime status attribute with token value type."""

    @property
    def name(self) -> str:
        """The attribute's USD name."""
        return self.value[0]

    @property
    def display_name(self) -> str:
        """The attribute's display name."""
        return self.value[1]

    @property
    def type(self) -> Any:
        """The attribute's USD value type."""
        return self.value[2]


class Relations(Enum):
    """Relationships of an ``IsaacVirtualGantry`` prim."""

    GANTRY_ATTACH_BODY = (f"{_attr_prefix}:gantry:attachBody", "Attach Body")
    """Relationship token for the articulation link a virtual gantry pulls on."""
    GANTRY_ARTICULATION = (f"{_attr_prefix}:gantry:articulation", "Articulation")
    """Relationship token for the articulation-root prim a virtual gantry suspends."""

    @property
    def name(self) -> str:
        """The relationship's USD name."""
        return self.value[0]

    @property
    def display_name(self) -> str:
        """The relationship's display name."""
        return self.value[1]


def get_allowed_tokens(attribute: "Attributes") -> tuple:
    """Return the ``allowedTokens`` list for a schema attribute.

    Reads the list from the bundled ``VirtualGantrySchema.usda`` so the
    Python-side tuple stays in sync with the authored schema.

    Args:
        attribute: The attribute to query (only Token-typed attributes define
            allowed tokens).

    Returns:
        Tuple of allowed token strings, or an empty tuple when the attribute has
        no ``allowedTokens`` metadata.
    """
    layer = _schema_layer()
    if layer is None:
        logger.warning("Could not open %s", _SCHEMA_USDA_NAME)
        return ()
    attr_spec = layer.GetAttributeAtPath(f"/{Classes.VIRTUAL_GANTRY.value}.{attribute.name}")
    if attr_spec is None:
        return ()
    return tuple(str(token) for token in (attr_spec.GetInfo("allowedTokens") or []))


def _create_attributes(prim: Usd.Prim, attributes: Iterable[Attributes], write_sparsely: bool = True) -> None:
    """Create the given attributes on ``prim``.

    Args:
        prim: USD prim to process.
        attributes: Attribute specifications to create.
        write_sparsely: Whether to author only non-default values.
    """
    for attribute in attributes:
        prim.CreateAttribute(attribute.name, attribute.type, write_sparsely)


def _create_relationships(prim: Usd.Prim, relationships: Iterable[Relations], custom: bool = True) -> None:
    """Create the given relationships on ``prim``.

    Args:
        prim: USD prim to process.
        relationships: Relationship specifications to create.
        custom: Whether to create custom relationships.
    """
    for relationship in relationships:
        prim.CreateRelationship(relationship.name, custom)


def CreateVirtualGantry(stage: Usd.Stage, prim_path: str) -> Usd.Prim:  # noqa: N802 - generated USD API
    """Create a Virtual Gantry prim with its attributes and relationships.

    The prim is ``Xform``-derived: its world transform is the rope anchor.

    Args:
        stage: The USD stage to create the prim in.
        prim_path: The path where to create the prim.

    Returns:
        The created Virtual Gantry prim.

    Example:

    .. code-block:: python

        gantry_prim = CreateVirtualGantry(stage, "/World/VirtualGantry")

    """
    prim = stage.DefinePrim(prim_path, Classes.VIRTUAL_GANTRY.value)
    _create_attributes(prim, list(Attributes), write_sparsely=False)
    _create_relationships(prim, list(Relations), custom=False)
    return prim


__all__ = [
    "Attributes",
    "Classes",
    "CreateVirtualGantry",
    "Relations",
    "get_allowed_tokens",
]
