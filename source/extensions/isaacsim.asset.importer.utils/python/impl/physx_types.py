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

"""Lightweight enum constants for PhysX schema identifiers, attributes, and relationships.

These constants replace direct ``pxr.PhysxSchema`` typed-API usage with
string-based identifiers that work with ``prim.ApplyAPI()``,
``prim.HasAPI()``, ``prim.CreateAttribute()``, and ``prim.GetAttribute()``.

Example:

.. code-block:: python

    from pxr import Sdf
    from isaacsim.asset.importer.utils.impl.physx_types import PhysxSchema, PhysxAttr

    if not prim.HasAPI(PhysxSchema.JOINT_API):
        prim.ApplyAPI(PhysxSchema.JOINT_API)
    prim.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(0.5)
"""

from __future__ import annotations

import logging
import os
from enum import Enum

from pxr import Plug, Sdf

_logger = logging.getLogger(__name__)


def _ensure_physx_schemas_registered() -> None:
    """Register minimal PhysX schema definitions if the full plugin is not available.

    When ``omni.usd.schema.physx`` is loaded (the normal case inside Kit / Isaac Sim),
    this function is a no-op.  Outside that environment the bundled codeless schema
    definitions are registered so that ``prim.ApplyAPI()`` / ``prim.HasAPI()`` still work.
    """
    registry = Plug.Registry()
    if registry.GetPluginWithName("physxSchema"):
        return  # Full plugin already loaded.

    schemas_dir = os.path.join(os.path.dirname(__file__), "schemas")
    if not os.path.isdir(schemas_dir):
        _logger.warning("PhysX schema fallback directory not found: %s", schemas_dir)
        return

    new_plugins = registry.RegisterPlugins(schemas_dir)
    if new_plugins:
        _logger.info("Registered PhysX schema fallback from %s", schemas_dir)
    else:
        _logger.warning("Failed to register PhysX schema fallback from %s", schemas_dir)


_ensure_physx_schemas_registered()

__all__ = [
    "PhysxSchema",
    "PhysxAttr",
    "PhysxMimicAttr",
    "PhysxMimicRel",
]


class PhysxSchema(str, Enum):
    """Schema identifiers for ``prim.ApplyAPI()`` and ``prim.HasAPI()``."""

    JOINT_API = "PhysxJointAPI"
    """PhysX joint API (single-apply)."""
    ARTICULATION_API = "PhysxArticulationAPI"
    """PhysX articulation API (single-apply)."""
    MIMIC_JOINT_API = "PhysxMimicJointAPI"
    """PhysX mimic joint API (multi-apply with axis instance)."""
    JOINT_STATE_API = "PhysicsJointStateAPI"
    """PhysX joint state API (multi-apply with drive instance)."""


class PhysxAttr(Enum):
    """PhysX attribute names and their USD value types.

    Access the USD attribute name via ``.name`` and the type via ``.type``.
    """

    JOINT_ARMATURE = ("physxJoint:armature", Sdf.ValueTypeNames.Float)
    """Joint armature (Float)."""
    JOINT_FRICTION = ("physxJoint:jointFriction", Sdf.ValueTypeNames.Float)
    """Joint friction (Float)."""
    JOINT_MAX_VELOCITY = ("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float)
    """Maximum joint velocity (Float)."""
    ARTICULATION_SELF_COLLISION = ("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool)
    """Articulation self-collision enabled (Bool)."""

    @property
    def name(self) -> str:
        """USD attribute name."""
        return self.value[0]

    @property
    def type(self) -> Sdf.ValueTypeName:
        """USD ``Sdf.ValueTypeNames`` type."""
        return self.value[1]


class PhysxMimicAttr(Enum):
    """Attribute name templates for PhysxMimicJointAPI (multi-apply).

    Use ``attr.format(axis)`` to get the full attribute name for a given axis token.
    Access the USD type via ``.type``.

    Example:

    .. code-block:: python

        attr_name = PhysxMimicAttr.GEARING.format("rotZ")
        # -> "physxMimicJoint:rotZ:gearing"
    """

    GEARING = ("physxMimicJoint:{}:gearing", Sdf.ValueTypeNames.Float)
    """Mimic gearing multiplier (Float)."""
    OFFSET = ("physxMimicJoint:{}:offset", Sdf.ValueTypeNames.Float)
    """Mimic offset (Float)."""
    REFERENCE_JOINT_AXIS = ("physxMimicJoint:{}:referenceJointAxis", Sdf.ValueTypeNames.Token)
    """Reference joint axis token (Token)."""

    @property
    def type(self) -> Sdf.ValueTypeName:
        """USD ``Sdf.ValueTypeNames`` type."""
        return self.value[1]

    def format(self, axis: str) -> str:
        """Return the full attribute name for a given axis token.

        Args:
            axis: Axis instance token (e.g. ``"rotX"``, ``"rotZ"``).

        Returns:
            Formatted PhysxMimicJointAPI attribute name for ``axis``.
        """
        return self.value[0].format(axis)


class PhysxMimicRel(Enum):
    """Relationship name templates for PhysxMimicJointAPI (multi-apply).

    Use ``rel.format(axis)`` to get the full relationship name for a given axis token.

    Example:

    .. code-block:: python

        rel_name = PhysxMimicRel.REFERENCE_JOINT.format("rotZ")
        # -> "physxMimicJoint:rotZ:referenceJoint"
    """

    REFERENCE_JOINT = "physxMimicJoint:{}:referenceJoint"
    """Relationship to the reference (source) joint."""

    def format(self, axis: str) -> str:
        """Return the full relationship name for a given axis token.

        Args:
            axis: Axis instance token (e.g. ``"rotX"``, ``"rotZ"``).

        Returns:
            Formatted PhysxMimicJointAPI relationship name for ``axis``.
        """
        return self.value.format(axis)
