# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# ruff: noqa: N802
# Methods intentionally mirror the PascalCase generated USD schema API
# (N802)
"""Robot schema helpers and applied schema utilities."""

from __future__ import annotations

import logging
import os
from collections.abc import Iterable
from enum import Enum
from functools import lru_cache

import pxr
from pxr import Sdf, Usd  # noqa: F401 — Usd is used through pxr.Usd annotations.

logger = logging.getLogger(__name__)

_SCHEMA_USDA_PATH = os.path.join(os.path.dirname(__file__), "RobotSchema.usda")


@lru_cache(maxsize=None)
def _get_schema_layer() -> Sdf.Layer | None:
    """Open the RobotSchema.usda file as an Sdf layer.

    Returns:
        Schema layer if it can be opened, None otherwise.
    """
    return Sdf.Layer.FindOrOpen(_SCHEMA_USDA_PATH)


def get_allowed_tokens(attribute: "Attributes") -> tuple[str, ...]:
    """Return the ``allowedTokens`` list for a robot schema attribute.

    Reads the list directly from the bundled ``RobotSchema.usda`` so the
    Python-side tuple stays in sync with the authored schema.

    Args:
        attribute: The schema attribute to query (only Token-typed
            attributes define allowed tokens).

    Returns:
        Tuple of allowed token strings, or an empty tuple if the
        attribute has no ``allowedTokens`` metadata.
    """
    layer = _get_schema_layer()
    if layer is None:
        logger.warning("Could not open RobotSchema.usda at %s", _SCHEMA_USDA_PATH)
        return ()

    # IsaacRobotAPI/isaac:robotType -> /IsaacRobotAPI.isaac:robotType
    for class_name in ("IsaacRobotAPI", "IsaacJointAPI", "IsaacSurfaceGripper", "IsaacAttachmentPointAPI"):
        attr_spec = layer.GetAttributeAtPath(f"/{class_name}.{attribute.name}")
        if attr_spec is not None:
            tokens = attr_spec.GetInfo("allowedTokens") or []
            return tuple(str(t) for t in tokens)
    return ()


class Classes(Enum):
    """Enumeration of robot schema class tokens."""

    ROBOT_API = "IsaacRobotAPI"
    """Token for the IsaacRobotAPI schema class."""
    LINK_API = "IsaacLinkAPI"
    """Token for the IsaacLinkAPI schema class."""
    REFERENCE_POINT_API = "IsaacReferencePointAPI"
    """Token for the IsaacReferencePointAPI schema class."""
    SITE_API = "IsaacSiteAPI"
    """Token for the IsaacSiteAPI schema class."""
    JOINT_API = "IsaacJointAPI"
    """Token for the IsaacJointAPI schema class."""
    SURFACE_GRIPPER = "IsaacSurfaceGripper"
    """Token for the IsaacSurfaceGripper schema class."""
    ATTACHMENT_POINT_API = "IsaacAttachmentPointAPI"
    """Token for the IsaacAttachmentPointAPI schema class."""
    NAMED_POSE = "IsaacNamedPose"
    """Token for the IsaacNamedPose schema class."""


_attr_prefix = "isaac"


class DofOffsetOpOrder(Enum):
    """Enumeration of DOF offset operation ordering tokens."""

    TRANS_X = "TransX"
    """X-axis translation operation token."""
    TRANS_Y = "TransY"
    """Y-axis translation operation token."""
    TRANS_Z = "TransZ"
    """Z-axis translation operation token."""
    ROT_X = "RotX"
    """X-axis rotation operation token."""
    ROT_Y = "RotY"
    """Y-axis rotation operation token."""
    ROT_Z = "RotZ"
    """Z-axis rotation operation token."""


class Attributes(Enum):
    """Enumeration of custom robot schema attributes."""

    DESCRIPTION = (f"{_attr_prefix}:description", "Description", pxr.Sdf.ValueTypeNames.String)
    """Description attribute for robots with string value type."""
    NAMESPACE = (f"{_attr_prefix}:namespace", "Namespace", pxr.Sdf.ValueTypeNames.String)
    """Namespace attribute for robots with string value type."""
    ROBOT_TYPE = (f"{_attr_prefix}:robotType", "Robot Type", pxr.Sdf.ValueTypeNames.Token)
    """Robot type attribute with token value type."""
    LICENSE = (f"{_attr_prefix}:license", "License", pxr.Sdf.ValueTypeNames.Token)
    """License attribute with token value type."""
    VERSION = (f"{_attr_prefix}:version", "Version", pxr.Sdf.ValueTypeNames.String)
    """Version attribute with string value type."""
    SOURCE = (f"{_attr_prefix}:source", "Source", pxr.Sdf.ValueTypeNames.String)
    """Source attribute with string value type."""
    CHANGELOG = (f"{_attr_prefix}:changelog", "Changelog", pxr.Sdf.ValueTypeNames.StringArray)
    """Changelog attribute with string array value type."""
    NAME_OVERRIDE = (f"{_attr_prefix}:nameOverride", "Name Override", pxr.Sdf.ValueTypeNames.String)
    """Name override attribute with string value type."""
    REFERENCE_DESCRIPTION = (f"{_attr_prefix}:Description", "Reference Description", pxr.Sdf.ValueTypeNames.String)
    """Reference description attribute with string value type."""
    FORWARD_AXIS = (f"{_attr_prefix}:forwardAxis", "Forward Axis", pxr.Sdf.ValueTypeNames.Token)
    """Forward axis attribute with token value type."""
    JOINT_NAME_OVERRIDE = (f"{_attr_prefix}:NameOverride", "Joint Name Override", pxr.Sdf.ValueTypeNames.String)
    """Joint name override attribute with string value type."""
    DOF_OFFSET_OP_ORDER = (
        f"{_attr_prefix}:physics:DofOffsetOpOrder",
        "Dof Offset Op Order",
        pxr.Sdf.ValueTypeNames.TokenArray,
    )
    """DOF offset operation order attribute with token array value type."""
    ACTUATOR = (f"{_attr_prefix}:actuator", "Actuator", pxr.Sdf.ValueTypeNames.BoolArray)
    """Actuator attribute with boolean array value type."""
    RETRY_INTERVAL = (f"{_attr_prefix}:retryInterval", "Retry Interval", pxr.Sdf.ValueTypeNames.Float)
    """Retry interval attribute with float value type."""
    STATUS = (f"{_attr_prefix}:status", "Status", pxr.Sdf.ValueTypeNames.Token)
    """Status attribute with token value type."""
    SHEAR_FORCE_LIMIT = (f"{_attr_prefix}:shearForceLimit", "Shear Force Limit", pxr.Sdf.ValueTypeNames.Float)
    """Shear force limit attribute with float value type."""
    COAXIAL_FORCE_LIMIT = (f"{_attr_prefix}:coaxialForceLimit", "Coaxial Force Limit", pxr.Sdf.ValueTypeNames.Float)
    """Coaxial force limit attribute with float value type."""
    MAX_GRIP_DISTANCE = (f"{_attr_prefix}:maxGripDistance", "Max Grip Distance", pxr.Sdf.ValueTypeNames.Float)
    """Maximum grip distance attribute with float value type."""
    CLEARANCE_OFFSET = (f"{_attr_prefix}:clearanceOffset", "Clearance Offset", pxr.Sdf.ValueTypeNames.Float)
    """Clearance offset attribute with float value type."""
    POSE_VALID = (f"{_attr_prefix}:robot:pose:valid", "Pose Valid", pxr.Sdf.ValueTypeNames.Bool)
    """Pose valid attribute with boolean value type."""
    POSE_JOINT_VALUES = (
        f"{_attr_prefix}:robot:pose:jointValues",
        "Pose Joint Values",
        pxr.Sdf.ValueTypeNames.FloatArray,
    )
    """Pose joint values attribute with float array value type."""
    POSE_JOINT_FIXED = (
        f"{_attr_prefix}:robot:pose:jointFixed",
        "Pose Joint Fixed",
        pxr.Sdf.ValueTypeNames.BoolArray,
    )
    """Pose joint fixed attribute with boolean array value type."""

    # Custom properties for name and type
    @property
    def name(self) -> str:
        """USD attribute name.

        Returns:
            The USD attribute name string.

        Example:

        .. code-block:: python

            attr_name = Attributes.DESCRIPTION.name

        """
        return self.value[0]

    @property
    def display_name(self) -> str:
        """Display name for the attribute.

        Returns:
            The display name for UI presentation.

        Example:

        .. code-block:: python

            label = Attributes.DESCRIPTION.display_name

        """
        return self.value[1]

    @property
    def type(self) -> pxr.Sdf.ValueTypeName:
        """USD attribute type token.

        Returns:
            The USD attribute type token.

        Example:

        .. code-block:: python

            attr_type = Attributes.DESCRIPTION.type

        """
        return self.value[2]


class Relations(Enum):
    """Enumeration of robot schema relationships."""

    ROBOT_LINKS = (f"{_attr_prefix}:physics:robotLinks", "Robot Links")
    """Relationship token for connecting robot links in physics simulation."""
    ROBOT_JOINTS = (f"{_attr_prefix}:physics:robotJoints", "Robot Joints")
    """Relationship token for connecting robot joints in physics simulation."""
    NAMED_POSES = (f"{_attr_prefix}:robot:namedPoses", "Named Poses")
    """Relationship token for connecting named poses to a robot."""
    ATTACHMENT_POINTS = (f"{_attr_prefix}:attachmentPoints", "Attachment Points")
    """Relationship token for connecting attachment points used by grippers."""
    GRIPPED_OBJECTS = (f"{_attr_prefix}:grippedObjects", "Gripped Objects")
    """Relationship token for connecting objects currently gripped by a gripper."""
    POSE_START_LINK = (f"{_attr_prefix}:robot:pose:startLink", "Pose Start Link")
    """Relationship token for connecting the start link of a named pose."""
    POSE_END_LINK = (f"{_attr_prefix}:robot:pose:endLink", "Pose End Link")
    """Relationship token for connecting the end link of a named pose."""
    POSE_JOINTS = (f"{_attr_prefix}:robot:pose:joints", "Pose Joints")
    """Relationship token for connecting the joints of a named pose."""

    @property
    def name(self) -> str:
        """USD relationship name.

        Returns:
            The USD relationship name string.

        Example:

        .. code-block:: python

            rel_name = Relations.ROBOT_LINKS.name

        """
        return self.value[0]

    @property
    def display_name(self) -> str:
        """Display name for the relationship.

        Returns:
            The display name for UI presentation.

        Example:

        .. code-block:: python

            label = Relations.ROBOT_LINKS.display_name

        """
        return self.value[1]


def _create_attributes(prim: pxr.Usd.Prim, attributes: Iterable[Attributes], write_sparsely: bool = True) -> None:
    """Create attributes on a prim from attribute descriptors.

    Args:
        prim: The prim to receive attributes.
        attributes: Iterable of attribute descriptors to create.
        write_sparsely: Whether to author sparse values.

    """
    for attr in attributes:
        prim.CreateAttribute(attr.name, attr.type, write_sparsely)


def _create_relationships(prim: pxr.Usd.Prim, relationships: Iterable[Relations], custom: bool = True) -> None:
    """Create relationships on a prim from relationship descriptors.

    Args:
        prim: The prim to receive relationships.
        relationships: Iterable of relationship descriptors to create.
        custom: Whether to create custom relationships.

    """
    for rel in relationships:
        prim.CreateRelationship(rel.name, custom=custom)


def _apply_api(
    prim: pxr.Usd.Prim,
    schema: Classes,
    *,
    attributes: Iterable[Attributes] = (),
    relationships: Iterable[Relations] = (),
    write_sparsely: bool = True,
    relationships_custom: bool = True,
) -> None:
    """Apply a schema API and create related attributes and relationships.

    No-op when ``prim`` already has ``schema`` applied anywhere in its layer
    stack. The redundant ``AddAppliedSchema`` write would otherwise dirty
    ``apiSchemas`` metadata on every recalc traversal, forcing USD to
    re-resolve the prim and resetting any session-only or override pose data
    keyed on the previous composition state.

    Args:
        prim: The prim to update.
        schema: The schema class token to apply.
        attributes: Iterable of attribute descriptors to create.
        relationships: Iterable of relationship descriptors to create.
        write_sparsely: Whether to author sparse values.
        relationships_custom: Whether to create custom relationships.

    """
    if prim.HasAPI(schema.value):
        return
    prim.AddAppliedSchema(schema.value)
    _create_attributes(prim, attributes, write_sparsely)
    _create_relationships(prim, relationships, relationships_custom)


def ApplyRobotAPI(prim: pxr.Usd.Prim) -> None:
    """Apply the Robot API schema and populate robot relationships.

    Args:
        prim: The prim to update.

    Example:

    .. code-block:: python

        ApplyRobotAPI(robot_prim)

    """
    _apply_api(
        prim,
        Classes.ROBOT_API,
    )

    stage = prim.GetStage()
    if stage:
        from . import utils as _robot_schema_utils

        _robot_schema_utils.PopulateRobotSchemaFromArticulation(stage, prim, prim)


def ApplyLinkAPI(prim: pxr.Usd.Prim) -> None:
    """Apply the Link API schema to a prim.

    Args:
        prim: The prim to update.

    Example:

    .. code-block:: python

        ApplyLinkAPI(link_prim)

    """
    _apply_api(prim, Classes.LINK_API)


def ApplySiteAPI(prim: pxr.Usd.Prim) -> None:
    """Apply the IsaacSiteAPI schema to a prim.

    Args:
        prim: The USD prim to apply the SiteAPI schema to.

    Example:

    .. code-block:: python

        ApplySiteAPI(site_prim)

    """
    _apply_api(prim, Classes.SITE_API)


def ApplyReferencePointAPI(prim: pxr.Usd.Prim) -> None:
    """Apply the deprecated ReferencePoint API schema to a prim.

    Args:
        prim: The prim to update.

    Example:

    .. code-block:: python

        ApplyReferencePointAPI(reference_prim)

    """
    logger.warning("ApplyReferencePointAPI is deprecated. Use ApplySiteAPI instead.")
    _apply_api(prim, Classes.SITE_API)
    # for attr in [Attributes.REFERENCE_DESCRIPTION, Attributes.FORWARD_AXIS]:
    #     prim.CreateAttribute(attr.name, attr.type, True)


def ApplyJointAPI(prim: pxr.Usd.Prim) -> None:
    """Apply the Joint API schema to a prim.

    Args:
        prim: The prim to update.

    Example:

    .. code-block:: python

        ApplyJointAPI(joint_prim)

    """
    _apply_api(prim, Classes.JOINT_API)


def CreateSurfaceGripper(stage: pxr.Usd.Stage, prim_path: str) -> pxr.Usd.Prim:
    """Create a Surface Gripper prim with attributes and relationships.

    Args:
        stage: The USD stage to create the prim in.
        prim_path: The path where to create the prim.

    Returns:
        The created Surface Gripper prim.

    Example:

    .. code-block:: python

        gripper_prim = CreateSurfaceGripper(stage, "/World/Gripper")

    """
    # Create the prim
    prim = stage.DefinePrim(prim_path, Classes.SURFACE_GRIPPER.value)

    # Create attributes with default values
    _create_attributes(
        prim,
        [
            Attributes.STATUS,
            Attributes.SHEAR_FORCE_LIMIT,
            Attributes.COAXIAL_FORCE_LIMIT,
            Attributes.MAX_GRIP_DISTANCE,
            Attributes.RETRY_INTERVAL,
        ],
        write_sparsely=False,
    )

    # Create relationships
    _create_relationships(
        prim,
        [Relations.ATTACHMENT_POINTS, Relations.GRIPPED_OBJECTS],
        custom=False,
    )

    return prim


def ApplyAttachmentPointAPI(prim: pxr.Usd.Prim) -> None:
    """Apply the Attachment Point API schema to a prim.

    Args:
        prim: The prim to update.

    Example:

    .. code-block:: python

        ApplyAttachmentPointAPI(attachment_prim)

    """
    _apply_api(prim, Classes.ATTACHMENT_POINT_API)
    # for attr in [Attributes.FORWARD_AXIS, Attributes.CLEARANCE_OFFSET]:
    #     prim.CreateAttribute(attr.name, attr.type, False)


def CreateNamedPose(stage: pxr.Usd.Stage, prim_path: str) -> pxr.Usd.Prim:
    """Create a Named Pose prim with attributes and relationships.

    Args:
        stage: The USD stage to create the prim in.
        prim_path: The path where to create the prim.

    Returns:
        The created Named Pose prim.

    Example:

    .. code-block:: python

        named_pose_prim = CreateNamedPose(stage, "/World/Robot/HomePose")

    """
    prim = stage.DefinePrim(prim_path, Classes.NAMED_POSE.value)
    _create_attributes(
        prim,
        [Attributes.POSE_VALID, Attributes.POSE_JOINT_VALUES, Attributes.POSE_JOINT_FIXED],
        write_sparsely=True,
    )
    _create_relationships(
        prim,
        [Relations.POSE_START_LINK, Relations.POSE_END_LINK, Relations.POSE_JOINTS],
        custom=True,
    )
    return prim
