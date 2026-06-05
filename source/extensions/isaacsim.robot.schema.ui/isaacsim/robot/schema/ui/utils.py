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

"""Shared UI utilities for robot schema visualization."""

from enum import Enum, auto
from typing import Any

import carb
import omni.kit.viewport.utility as viewport_utility
import omni.usd
import usd.schema.isaac.robot_schema.utils as robot_schema_utils
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.ui import color as cl
from pxr import Gf, Sdf, Trace, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema

# --------------------------------------------------------------------------
# Shared color constants
# --------------------------------------------------------------------------
CONNECTION_COLOR = cl("#1D5F41")
LINE_COLOR = cl("#383838")
LINE_BACKGROUND_COLOR = cl("#c3c3c3")


# --------------------------------------------------------------------------
# Hierarchy display modes
# --------------------------------------------------------------------------


class HierarchyMode(Enum):
    """Display mode for the robot hierarchy panel.

    Attributes:
        FLAT: Links and joints listed as two flat scopes under the robot,
            ordered as they appear in the robot schema relationships.
        LINKED: Linked-list traversal (parent link → joint → child link),
            which is the default display mode.
        MUJOCO: Tree rooted at the primary link. Each link's child links are
            listed first, followed by the joint connecting that link to its
            own parent as the last child entry.
    """

    FLAT = auto()
    LINKED = auto()
    MUJOCO = auto()


# --------------------------------------------------------------------------
# Stage and Prim Access Utilities
# --------------------------------------------------------------------------


def get_stage_safe(context_name: str = "") -> Usd.Stage | None:
    """Return the current USD stage, or None if unavailable.

    Args:
        context_name: USD context name; empty string uses the default context.

    Returns:
        Current USD stage, or None if unavailable.

    Example:

    .. code-block:: python

        stage = get_stage_safe()
    """
    context = omni.usd.get_context(context_name)
    if not context:
        return None
    return context.get_stage()


def get_prim_safe(path: Sdf.Path | str | None, stage: Usd.Stage | None = None) -> Usd.Prim | None:
    """Return the prim at the given path, or None if missing or invalid.

    Args:
        path: USD path or path string to the prim.
        stage: USD stage; if None, uses the default context.

    Returns:
        Prim if it exists and is valid, None otherwise.

    Example:

    .. code-block:: python

        prim = get_prim_safe("/World/Robot")
    """
    if path is None:
        return None
    if stage is None:
        stage = get_stage_safe()
    if not stage:
        return None
    if not isinstance(path, Sdf.Path):
        path = Sdf.Path(path)
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        return prim
    return None


# --------------------------------------------------------------------------
# Vector Conversion Utilities
# --------------------------------------------------------------------------


def to_vec3d(position: Any) -> Gf.Vec3d | None:
    """Convert a position to a 3D vector.

    Args:
        position: Position value (tuple, list, vector, or indexable object).

    Returns:
        Vector representation, or None if conversion fails.

    Example:

    .. code-block:: python

        position = to_vec3d([0.0, 1.0, 2.0])
    """
    if position is None:
        return None
    if isinstance(position, Gf.Vec3d):
        return position
    try:
        return Gf.Vec3d(position[0], position[1], position[2])
    except (TypeError, IndexError):
        try:
            return Gf.Vec3d(position)
        except Exception:
            return None


def to_float_list(position: Any) -> list[float]:
    """Convert a position to a list of floats.

    Args:
        position: Position value (tuple, list, vector, or indexable object).

    Returns:
        List of three floats [x, y, z].

    Example:

    .. code-block:: python

        floats = to_float_list((1.0, 2.0, 3.0))
    """
    return [float(x) for x in position]


# --------------------------------------------------------------------------
# Camera Utilities
# --------------------------------------------------------------------------


def get_camera_path() -> str | None:
    """Return the current viewport camera's USD path.

    Returns:
        Camera prim path, or None if unavailable.

    Example:

    .. code-block:: python

        camera_path = get_camera_path()
    """
    return viewport_utility.get_viewport_window_camera_path()


def get_camera_position() -> Gf.Vec3d | None:
    """Return the current viewport camera's world position.

    Returns:
        Camera world position, or None if unavailable.

    Example:

    .. code-block:: python

        position = get_camera_position()
    """
    camera_path = get_camera_path()
    if not camera_path:
        return None
    camera_state = ViewportCameraState()
    return camera_state.position_world


def get_camera_pose(offset: float = 0.0) -> tuple[Gf.Vec3d, Gf.Vec3d] | None:
    """Return the current camera position and forward direction.

    Computes the camera's world-space position and forward direction vector.
    USD cameras look down -Z in local space.

    Args:
        offset: Offset along the forward direction to apply to the position.

    Returns:
        Tuple of (position, forward_direction), or None if the camera is unavailable.

    Example:

    .. code-block:: python

        camera_pose = get_camera_pose()
    """
    camera_path = get_camera_path()
    if not camera_path:
        return None
    stage = get_stage_safe()
    if not stage:
        return None
    camera_prim = stage.GetPrimAtPath(camera_path)
    if not camera_prim or not camera_prim.IsValid():
        return None
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world_transform = xform_cache.GetLocalToWorldTransform(camera_prim)
    position = world_transform.ExtractTranslation()
    # USD cameras look down -Z in local space
    forward_direction = -Gf.Vec3d(world_transform[2][0], world_transform[2][1], world_transform[2][2])
    if forward_direction.GetLength() == 0.0:
        return None
    forward_direction.Normalize()
    position = Gf.Vec3d(position[0], position[1], position[2])
    if offset:
        position = position + forward_direction * offset
    return position, forward_direction


def is_in_front_of_camera(
    point: Any, camera_position: Gf.Vec3d, camera_forward: Gf.Vec3d, z_offset: float = 0.001
) -> bool:
    """Return True if the point is in front of the camera.

    Args:
        point: World-space point to check (tuple, list, or vector).
        camera_position: Camera position vector.
        camera_forward: Camera forward direction vector.
        z_offset: Small offset to avoid z-fighting.

    Returns:
        True if the point is in front of the camera, False otherwise.

    Example:

    .. code-block:: python

        visible = is_in_front_of_camera(point, camera_pos, camera_forward)
    """
    point_vector = to_vec3d(point)
    if point_vector is None:
        return False
    direction = point_vector - camera_position
    if direction.GetLength() > 0:
        direction.Normalize()
    return Gf.Dot((point_vector - camera_position) - direction * z_offset, camera_forward) > 0


# --------------------------------------------------------------------------
# Viewport Utilities
# --------------------------------------------------------------------------


def get_active_viewport() -> Any | None:
    """Return the active viewport API.

    Returns:
        Active viewport API object, or None if unavailable.

    Example:

    .. code-block:: python

        viewport_api = get_active_viewport()
    """
    return viewport_utility.get_active_viewport()


def world_to_screen_position(
    position: Any, viewport_api: Any | None = None
) -> tuple[float, float] | tuple[float, float, float] | None:
    """Convert a world position to screen coordinates.

    Args:
        position: World-space position to convert.
        viewport_api: Viewport API; if None, uses the active viewport.

    Returns:
        (x, y) screen coordinates if visible, (x, y, z) world fallback if
        conversion fails, or None if the position is invalid.

    Example:

    .. code-block:: python

        screen_pos = world_to_screen_position((0.0, 0.0, 0.0))
    """
    position_vector = to_vec3d(position)
    if position_vector is None:
        return None
    if viewport_api is None:
        viewport_api = get_active_viewport()
    if viewport_api:
        try:
            ndc_position = viewport_api.world_to_ndc.Transform(position_vector)
            texture_position, viewport = viewport_api.map_ndc_to_texture([ndc_position[0], ndc_position[1]])
            if viewport is not None and texture_position is not None:
                return (texture_position[0], texture_position[1])
        except Exception:
            pass
    return (position_vector[0], position_vector[1], position_vector[2])


def is_position_in_viewport(position: Any, viewport_api: Any | None = None) -> bool:
    """Return True if the world position is within the viewport screen space.

    Args:
        position: World-space position to check.
        viewport_api: Viewport API; if None, uses the active viewport.

    Returns:
        True if the position is visible in the viewport, False otherwise.

    Example:

    .. code-block:: python

        visible = is_position_in_viewport((0.0, 0.0, 0.0))
    """
    position_vector = to_vec3d(position)
    if position_vector is None:
        return False
    if viewport_api is None:
        viewport_api = get_active_viewport()
    if not viewport_api:
        return False
    try:
        ndc_position = viewport_api.world_to_ndc.Transform(position_vector)
        texture_position, viewport = viewport_api.map_ndc_to_texture([ndc_position[0], ndc_position[1]])
        return viewport is not None
    except Exception:
        return False


# --------------------------------------------------------------------------
# Path Mapping for Hierarchy Stage
# --------------------------------------------------------------------------


class PathMap:
    """Bidirectional mapping between original USD paths and hierarchy stage paths.

    Maintains two dictionaries to allow lookup in either direction, enabling
    translation between the original stage's prim paths and the generated
    hierarchy stage's prim paths.
    """

    def __init__(self) -> None:
        self.original_to_hierarchy: dict[Sdf.Path, Sdf.Path] = {}
        self.hierarchy_to_original: dict[Sdf.Path, Sdf.Path] = {}

    def insert(self, original_path: Sdf.Path, hierarchy_path: Sdf.Path) -> None:
        """Add a path mapping.

        Args:
            original_path: The path in the original USD stage.
            hierarchy_path: The corresponding path in the hierarchy stage.

        Example:

        .. code-block:: python

            path_map.insert(original_path, hierarchy_path)
        """
        self.original_to_hierarchy[original_path] = hierarchy_path
        self.hierarchy_to_original[hierarchy_path] = original_path

    def get_hierarchy_path(self, original_path: Sdf.Path) -> Sdf.Path | None:
        """Get the hierarchy stage path for an original path.

        Args:
            original_path: The path in the original USD stage.

        Returns:
            The corresponding hierarchy stage path, or None if not found.

        Example:

        .. code-block:: python

            hierarchy_path = path_map.get_hierarchy_path(original_path)
        """
        return self.original_to_hierarchy.get(original_path)

    def get_original_path(self, hierarchy_path: Sdf.Path) -> Sdf.Path | None:
        """Get the original stage path for a hierarchy path.

        Args:
            hierarchy_path: The path in the hierarchy stage.

        Returns:
            The corresponding original stage path, or None if not found.

        Example:

        .. code-block:: python

            original_path = path_map.get_original_path(hierarchy_path)
        """
        return self.hierarchy_to_original.get(hierarchy_path)

    def clear(self) -> None:
        """Clear all path mappings.

        Example:

        .. code-block:: python

            path_map.clear()
        """
        self.original_to_hierarchy.clear()
        self.hierarchy_to_original.clear()


# --------------------------------------------------------------------------
# Named Pose Utilities
# --------------------------------------------------------------------------


def get_robot_named_poses(robot_prim: Usd.Prim | None) -> list[Usd.Prim]:
    """Return all named pose prims for a robot from the schema relationship.

    Args:
        robot_prim: The robot prim (must have Robot API).

    Returns:
        List of named pose prims in relationship order. Empty if stage or
        robot is invalid or robot has no named poses.

    Example:

    .. code-block:: python

        named_poses = get_robot_named_poses(robot_prim)
    """
    if not robot_prim or not robot_prim.IsValid():
        return []
    stage = get_stage_safe()
    if not stage:
        return []
    return robot_schema_utils.GetAllNamedPoses(stage, robot_prim)


def get_named_pose_joint_values(named_pose_prim: Usd.Prim | None) -> list[float] | None:
    """Get the joint values from a named pose prim.

    Args:
        named_pose_prim: The named pose prim.

    Returns:
        The joint values array, or None if not authored or prim invalid.

    Example:

    .. code-block:: python

        values = get_named_pose_joint_values(named_pose_prim)
    """
    if not named_pose_prim or not named_pose_prim.IsValid():
        return None
    return robot_schema_utils.GetNamedPoseJointValues(named_pose_prim)


# --------------------------------------------------------------------------
# Joint Position Utilities
# --------------------------------------------------------------------------


def joint_has_both_bodies(joint_prim: Usd.Prim | None) -> bool:
    """Return True if the joint has both body0 and body1 relationships defined.

    Args:
        joint_prim: Joint prim to inspect.

    Returns:
        True if both body0 and body1 have valid targets, False otherwise.

    Example:

    .. code-block:: python

        has_bodies = joint_has_both_bodies(joint_prim)
    """
    if joint_prim is None:
        return False
    joint = UsdPhysics.Joint(joint_prim)
    if not joint:
        return False
    body0_rel = joint.GetBody0Rel()
    body1_rel = joint.GetBody1Rel()
    if not body0_rel or not body1_rel:
        return False
    body0_targets = body0_rel.GetTargets()
    body1_targets = body1_rel.GetTargets()
    return bool(body0_targets) and bool(body1_targets)


@Trace.TraceFunction
def get_link_position(link: Usd.Prim | Sdf.Path | str | None) -> Gf.Vec3d | None:
    """Return the world-space position of a link prim origin.

    Args:
        link: Link prim or prim path.

    Returns:
        Link world-space position, or None if computation fails.

    Example:

    .. code-block:: python

        position = get_link_position("/World/Robot/Link1")
    """
    if link is None:
        return None

    prim = None
    if isinstance(link, Usd.Prim):
        prim = link if link.IsValid() else None
    else:
        stage = get_stage_safe()
        if not stage:
            return None
        if not isinstance(link, Sdf.Path):
            link = Sdf.Path(link)
        prim = get_prim_safe(link, stage)

    if not prim:
        return None

    try:
        world_transform = Gf.Matrix4d(omni.usd.get_world_transform_matrix(prim))
        translation = world_transform.ExtractTranslation()
        return Gf.Vec3d(*translation)
    except Exception as error:
        carb.log_warn(f"Failed to compute link position for {prim.GetPath()}: {error}")
        return None


@Trace.TraceFunction
def get_joint_position(robot_root_path: Sdf.Path | str, joint_path: Sdf.Path | str) -> Gf.Vec3d | None:
    """Return the world-space position of a robot joint.

    Retrieves the joint pose from the robot schema and transforms it
    to world coordinates using the robot's world transform.

    Args:
        robot_root_path: Path to the robot root prim.
        joint_path: Path to the joint prim.

    Returns:
        Joint world-space position, or None if computation fails.

    Example:

    .. code-block:: python

        position = get_joint_position("/World/Robot", "/World/Robot/joint")
    """
    stage = get_stage_safe()
    if not stage:
        return None

    if not isinstance(robot_root_path, Sdf.Path):
        robot_root_path = Sdf.Path(robot_root_path)
    if not isinstance(joint_path, Sdf.Path):
        joint_path = Sdf.Path(joint_path)

    robot_prim = get_prim_safe(robot_root_path, stage)
    joint_prim = get_prim_safe(joint_path, stage)

    if not robot_prim or not joint_prim:
        return None

    try:
        pose_matrix = robot_schema_utils.GetJointPose(robot_prim, joint_prim)
        if pose_matrix is None:
            return None
        translation = Gf.Vec3d(*pose_matrix.ExtractTranslation())
        robot_world_transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(robot_prim)
        world_position = robot_world_transform.Transform(translation)
        return world_position
    except Exception as error:
        carb.log_warn(f"Failed to compute joint position for {joint_path}: {error}")
        return None


# --------------------------------------------------------------------------
# Robot Hierarchy Generation
# --------------------------------------------------------------------------


def _copy_applied_schemas(source_prim: Usd.Prim, target_prim: Usd.Prim) -> None:
    """Copy the ``apiSchemas`` metadata from *source_prim* to *target_prim*.

    This transfers all applied API schemas (e.g. ``IsaacJointAPI``,
    ``IsaacLinkAPI``, ``RigidBodyAPI``) so that the hierarchy stage prims
    carry the same schema information as the original stage.

    Args:
        source_prim: Prim whose applied schemas are read.
        target_prim: Prim that receives the copied schemas.
    """
    schemas = source_prim.GetAppliedSchemas()
    if not schemas:
        return
    for schema_name in schemas:
        target_prim.AddAppliedSchema(schema_name)


def _create_hierarchy_node(
    hierarchy_stage: Usd.Stage,
    link_node: Any,
    parent_link_prim: Usd.Prim,
    parent_joint_in_chain: Usd.Prim | None,
    parent_link_source_prim: Usd.Prim | None,
    robot_root_path: Sdf.Path,
    path_map: PathMap,
    joint_connections: list[Any],
) -> None:
    """Create a hierarchy node and its children recursively.

    Internal helper function for generate_robot_hierarchy_stage().

    Args:
        hierarchy_stage: The in-memory hierarchy stage being built.
        link_node: The current link tree node to process.
        parent_link_prim: The parent prim in the hierarchy stage.
        parent_joint_in_chain: The parent joint prim for connection tracking.
        parent_link_source_prim: The parent link prim in the source stage.
        robot_root_path: The path to the robot root prim.
        path_map: The path mapping object for tracking paths.
        joint_connections: List to append connection items to.
    """
    # Import here to avoid circular dependency
    from .model import ConnectionItem

    current_joint_prim = link_node._joint_to_parent
    if current_joint_prim:
        hierarchy_joint_path = parent_link_prim.GetPath().AppendChild(current_joint_prim.GetName())
        hierarchy_joint_prim = hierarchy_stage.DefinePrim(hierarchy_joint_path, current_joint_prim.GetTypeName())
        _copy_applied_schemas(current_joint_prim, hierarchy_joint_prim)
        hierarchy_link_path = hierarchy_joint_prim.GetPath().AppendChild(link_node.name)
        path_map.insert(current_joint_prim.GetPath(), hierarchy_joint_prim.GetPath())

        # Create connection item for this joint
        joint_position = get_joint_position(robot_root_path, current_joint_prim.GetPath())
        if parent_joint_in_chain:
            parent_joint_position = get_joint_position(robot_root_path, parent_joint_in_chain.GetPath())
        else:
            parent_joint_position = get_link_position(parent_link_source_prim)

        connection_item = ConnectionItem(
            joint_prim=current_joint_prim,
            parent_joint_prim=parent_joint_in_chain,
            parent_link_prim=parent_link_source_prim,
            robot_root_path=robot_root_path,
            joint_pos=joint_position,
            parent_joint_pos=parent_joint_position,
        )
        joint_connections.append(connection_item)
        next_parent_joint = current_joint_prim
    else:
        hierarchy_link_path = parent_link_prim.GetPath().AppendChild(link_node.name)
        next_parent_joint = parent_joint_in_chain

    hierarchy_link_prim = hierarchy_stage.DefinePrim(hierarchy_link_path, "Xform")
    _copy_applied_schemas(link_node.prim, hierarchy_link_prim)
    path_map.insert(link_node.prim.GetPath(), hierarchy_link_prim.GetPath())

    for child_node in link_node.children:
        _create_hierarchy_node(
            hierarchy_stage,
            child_node,
            hierarchy_link_prim,
            next_parent_joint,
            link_node.prim,
            robot_root_path,
            path_map,
            joint_connections,
        )


def _generate_flat_hierarchy(
    hierarchy_stage: Usd.Stage,
    source_robot_prim: Usd.Prim,
    hierarchy_robot_prim: Usd.Prim,
    robot_root_path: Sdf.Path,
    path_map: PathMap,
    joint_connections: list[Any],
    stage: Usd.Stage,
) -> None:
    """Build a flat hierarchy under *hierarchy_robot_prim* with Links/Joints scopes.

    Links and joints are listed in the order they appear in the robot schema
    relationships, placed under dedicated ``Links`` and ``Joints`` scope prims.

    Args:
        hierarchy_stage: The in-memory hierarchy stage being built.
        source_robot_prim: The robot root prim in the original stage.
        hierarchy_robot_prim: The robot root prim already created in the hierarchy stage.
        robot_root_path: The path to the robot root prim.
        path_map: The path mapping object for tracking paths.
        joint_connections: List to append connection items to.
        stage: The original USD stage.
    """
    from .model import ConnectionItem

    links_path = hierarchy_robot_prim.GetPath().AppendChild("Links")
    hierarchy_stage.DefinePrim(links_path, "Scope")

    robot_links_rel = source_robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
    if robot_links_rel:
        for link_target in robot_links_rel.GetTargets():
            link_prim = stage.GetPrimAtPath(link_target)
            if not (link_prim and link_prim.IsValid()):
                continue
            hier_link_path = links_path.AppendChild(link_prim.GetName())
            hier_link_prim = hierarchy_stage.DefinePrim(hier_link_path, "Xform")
            _copy_applied_schemas(link_prim, hier_link_prim)
            path_map.insert(link_prim.GetPath(), hier_link_path)

    joints_path = hierarchy_robot_prim.GetPath().AppendChild("Joints")
    hierarchy_stage.DefinePrim(joints_path, "Scope")

    robot_joints_rel = source_robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
    if robot_joints_rel:
        for joint_target in robot_joints_rel.GetTargets():
            joint_prim = stage.GetPrimAtPath(joint_target)
            if not (joint_prim and joint_prim.IsValid()):
                continue
            hier_joint_path = joints_path.AppendChild(joint_prim.GetName())
            hier_joint_prim = hierarchy_stage.DefinePrim(hier_joint_path, joint_prim.GetTypeName())
            _copy_applied_schemas(joint_prim, hier_joint_prim)
            path_map.insert(joint_prim.GetPath(), hier_joint_path)

            joint_position = get_joint_position(robot_root_path, joint_prim.GetPath())
            parent_link_prim = None
            parent_pos = None
            phys_joint = UsdPhysics.Joint(joint_prim)
            if phys_joint:
                body0_rel = phys_joint.GetBody0Rel()
                if body0_rel:
                    body0_targets = body0_rel.GetTargets()
                    if body0_targets:
                        parent_link_prim = stage.GetPrimAtPath(body0_targets[0])
                        parent_pos = get_link_position(parent_link_prim)

            connection_item = ConnectionItem(
                joint_prim=joint_prim,
                parent_joint_prim=None,
                parent_link_prim=parent_link_prim,
                robot_root_path=robot_root_path,
                joint_pos=joint_position,
                parent_joint_pos=parent_pos,
            )
            joint_connections.append(connection_item)


def _create_hierarchy_node_mujoco(
    hierarchy_stage: Usd.Stage,
    link_node: Any,
    parent_prim: Usd.Prim,
    parent_joint_in_chain: Usd.Prim | None,
    parent_link_source_prim: Usd.Prim | None,
    robot_root_path: Sdf.Path,
    path_map: PathMap,
    joint_connections: list[Any],
) -> None:
    """Create a MuJoCo-style hierarchy node and its children recursively.

    Each link is placed under its parent. Child links are listed first
    (in order), and the joint connecting this link to its own parent is
    appended as the last child of this link prim.

    Args:
        hierarchy_stage: The in-memory hierarchy stage being built.
        link_node: The current link tree node to process.
        parent_prim: The parent prim in the hierarchy stage.
        parent_joint_in_chain: The kinematic-chain parent joint for connection lines.
        parent_link_source_prim: The parent link prim in the source stage.
        robot_root_path: The path to the robot root prim.
        path_map: The path mapping object for tracking paths.
        joint_connections: List to append connection items to.
    """
    from .model import ConnectionItem

    hierarchy_link_path = parent_prim.GetPath().AppendChild(link_node.name)
    hierarchy_link_prim = hierarchy_stage.DefinePrim(hierarchy_link_path, "Xform")
    _copy_applied_schemas(link_node.prim, hierarchy_link_prim)
    path_map.insert(link_node.prim.GetPath(), hierarchy_link_prim.GetPath())

    # The joint connecting this link to its parent becomes the "next parent joint"
    # for the children's connection lines (kinematic chain order).
    next_parent_joint = link_node._joint_to_parent if link_node._joint_to_parent else parent_joint_in_chain

    for child_node in link_node.children:
        _create_hierarchy_node_mujoco(
            hierarchy_stage,
            child_node,
            hierarchy_link_prim,
            next_parent_joint,
            link_node.prim,
            robot_root_path,
            path_map,
            joint_connections,
        )

    current_joint_prim = link_node._joint_to_parent
    if current_joint_prim:
        hierarchy_joint_path = hierarchy_link_prim.GetPath().AppendChild(current_joint_prim.GetName())
        hierarchy_joint_prim = hierarchy_stage.DefinePrim(hierarchy_joint_path, current_joint_prim.GetTypeName())
        _copy_applied_schemas(current_joint_prim, hierarchy_joint_prim)
        path_map.insert(current_joint_prim.GetPath(), hierarchy_joint_prim.GetPath())

        joint_position = get_joint_position(robot_root_path, current_joint_prim.GetPath())
        if parent_joint_in_chain:
            parent_joint_position = get_joint_position(robot_root_path, parent_joint_in_chain.GetPath())
        else:
            parent_joint_position = get_link_position(parent_link_source_prim)

        connection_item = ConnectionItem(
            joint_prim=current_joint_prim,
            parent_joint_prim=parent_joint_in_chain,
            parent_link_prim=parent_link_source_prim,
            robot_root_path=robot_root_path,
            joint_pos=joint_position,
            parent_joint_pos=parent_joint_position,
        )
        joint_connections.append(connection_item)


def generate_robot_hierarchy_stage(
    robot_root_path: Sdf.Path | str,
    mode: HierarchyMode = HierarchyMode.LINKED,
    stage: Usd.Stage | None = None,
    masking_layer_id: str | None = None,
) -> tuple[Usd.Stage | None, PathMap, list[Any]]:
    """Generate an in-memory USD stage for a single robot's joint hierarchy.

    Builds a link tree for the robot at ``robot_root_path`` and creates a
    hierarchy stage where joints are represented as parent-child
    relationships rather than as properties. Other robots present on the
    stage are ignored, so the cost of this call is independent of the
    total number of robots in the scene.

    Args:
        robot_root_path: Path of the robot root prim to generate. Required.
        mode: The display mode for the hierarchy.  ``LINKED`` (default) uses
            the parent-link → joint → child-link chain.  ``FLAT`` places all
            links under a ``Links`` scope and all joints under a ``Joints``
            scope in schema order.  ``MUJOCO`` roots the tree at the primary
            link and appends the joint-to-parent as the last child of each link.
        stage: Optional stage to use. If None, the stage from the default
            USD context is used. When provided (e.g. in a background worker),
            masking_layer_id should also be provided.
        masking_layer_id: Optional masking layer to mute while building. If None
            and stage is from context, MaskingState is used.

    Returns:
        A tuple of (hierarchy_stage, path_map, joint_connections).
        ``hierarchy_stage`` is an in-memory stage with the hierarchy structure,
        ``path_map`` translates between original and hierarchy paths, and
        ``joint_connections`` contains connection items for viewport visualization.
        Returns (None, an empty path map, []) if the path does not resolve
        to a prim with the Robot API applied.

    Example:

    .. code-block:: python

        hierarchy_stage, path_map, connections = generate_robot_hierarchy_stage("/World/Robot")
        hierarchy_stage, path_map, connections = generate_robot_hierarchy_stage(
            "/World/Robot", HierarchyMode.FLAT
        )
    """
    from .masking_state import MaskingState

    if stage is None:
        stage = get_stage_safe()
        masking_layer_id = MaskingState.get_instance().get_masking_layer_id()

    path_map = PathMap()
    joint_connections: list[Any] = []

    if not stage:
        return None, path_map, joint_connections

    if not isinstance(robot_root_path, Sdf.Path):
        robot_root_path = Sdf.Path(str(robot_root_path))

    if masking_layer_id:
        stage.MuteLayer(masking_layer_id)

    try:
        return _generate_robot_hierarchy_stage_inner(stage, mode, robot_root_path)
    finally:
        if masking_layer_id:
            stage.UnmuteLayer(masking_layer_id)


def generate_robot_hierarchy_stage_in_background(
    root_layer_identifier: str,
    masking_layer_id: str | None,
    mode: HierarchyMode,
    robot_root_path: str,
) -> dict[str, Any] | None:
    """Run hierarchy generation in a background thread; returns serializable result.

    Opens a stage from root_layer_identifier, mutes the masking layer, runs
    hierarchy generation for the given robot, and returns a dict that can
    be passed to deserialize_hierarchy_result on the main thread to rebuild
    the hierarchy stage and path map and to build ConnectionItems from path
    data.

    Args:
        root_layer_identifier: Identifier of the root layer (e.g. from
            context stage's GetRootLayer().identifier).
        masking_layer_id: Masking layer id to mute, or None.
        mode: Hierarchy display mode.
        robot_root_path: Path string of the robot root to generate. Required.

    Returns:
        Serializable dict with "layer_str", "path_map_list", "connections", or
        None if generation failed or the path does not resolve to a robot.
    """
    try:
        stage = Usd.Stage.Open(root_layer_identifier)
    except Exception:
        return None
    if not stage:
        return None
    hierarchy_stage, path_map, joint_connections = generate_robot_hierarchy_stage(
        robot_root_path, mode=mode, stage=stage, masking_layer_id=masking_layer_id
    )
    if hierarchy_stage is None:
        return None
    layer = hierarchy_stage.GetRootLayer()
    layer_str = layer.ExportToString()

    path_map_list = [(str(orig), str(hier)) for orig, hier in path_map.original_to_hierarchy.items()]
    connections = []
    for c in joint_connections:
        jp = c._joint_prim_path
        pjp = c._parent_joint_path
        plp = c._parent_link_path
        rrp = c._robot_root_path
        jpos = c._joint_position
        ppos = c._parent_joint_position
        connections.append(
            (
                str(jp) if jp else None,
                str(pjp) if pjp else None,
                str(plp) if plp else None,
                str(rrp) if rrp else None,
                (float(jpos[0]), float(jpos[1]), float(jpos[2])) if jpos else None,
                (float(ppos[0]), float(ppos[1]), float(ppos[2])) if ppos else None,
            )
        )
    return {"layer_str": layer_str, "path_map_list": path_map_list, "connections": connections}


def deserialize_hierarchy_result(
    data: dict[str, Any] | None,
) -> tuple[Usd.Stage | None, PathMap, list[tuple[Any, ...]]]:
    """Rebuild hierarchy stage and path map from a background worker result.

    Must be called on the main thread. Build ConnectionItems from the returned
    connection_tuples using ConnectionItem.from_paths.

    Args:
        data: Dict returned by generate_robot_hierarchy_stage_in_background.

    Returns:
        (hierarchy_stage, path_map, connection_tuples). connection_tuples is
        a list of (joint_prim_path, parent_joint_path, parent_link_path,
        robot_root_path, joint_pos, parent_joint_pos) for ConnectionItem.from_paths.
    """
    if not data:
        return None, PathMap(), []
    path_map = PathMap()
    for orig_s, hier_s in data.get("path_map_list", []):
        path_map.insert(Sdf.Path(orig_s), Sdf.Path(hier_s))
    layer_str = data.get("layer_str")
    if not layer_str:
        return None, path_map, []
    layer = Sdf.Layer.CreateAnonymous()
    layer.ImportFromString(layer_str)
    stage = Usd.Stage.Open(layer)
    if not stage:
        return None, path_map, []
    connection_tuples = []
    for t in data.get("connections", []):
        connection_tuples.append(
            (
                Sdf.Path(t[0]) if t[0] else None,
                Sdf.Path(t[1]) if t[1] else None,
                Sdf.Path(t[2]) if t[2] else None,
                Sdf.Path(t[3]) if t[3] else None,
                Gf.Vec3d(*t[4]) if t[4] else None,
                Gf.Vec3d(*t[5]) if t[5] else None,
            )
        )
    return stage, path_map, connection_tuples


def _generate_robot_hierarchy_stage_inner(
    stage: Usd.Stage,
    mode: HierarchyMode,
    robot_root_path: Sdf.Path,
) -> tuple[Usd.Stage | None, PathMap, list[Any]]:
    """Core hierarchy generation executed while the masking layer is muted.

    Args:
        stage: Source USD stage containing the robot prim.
        mode: Hierarchy display mode (FLAT, LINKED, or MUJOCO).
        robot_root_path: Path of the robot root to generate. The stage is
            **not** traversed; only the prim at this path is inspected.

    Returns:
        Tuple of ``(hierarchy_stage, path_map, joint_connections)``; the
        first element is ``None`` if the path does not resolve to a prim
        with the Robot API applied.
    """
    path_map = PathMap()
    joint_connections: list[Any] = []

    prim = stage.GetPrimAtPath(robot_root_path)
    if not prim or not prim.IsValid() or not prim.HasAPI(robot_schema.Classes.ROBOT_API.value):
        return None, path_map, joint_connections
    top_level_robot_prims = [prim]

    # Collect valid robots with their link trees
    robot_data = []
    for prim in top_level_robot_prims:
        joints = robot_schema_utils.GetAllRobotJoints(stage, prim)
        robot_tree = robot_schema_utils.GenerateRobotLinkTree(stage, prim)
        if robot_tree:
            robot_data.append((prim, robot_tree, joints))

    if not robot_data:
        return None, path_map, joint_connections

    hierarchy_stage = Usd.Stage.CreateInMemory()
    if not hierarchy_stage:
        return None, path_map, joint_connections

    for robot_root_prim, robot_tree, joints in robot_data:
        hierarchy_root_prim = hierarchy_stage.DefinePrim(robot_root_prim.GetPath(), "Xform")
        _copy_applied_schemas(robot_root_prim, hierarchy_root_prim)
        path_map.insert(robot_root_prim.GetPath(), hierarchy_root_prim.GetPath())
        current_root_path = robot_root_prim.GetPath()

        if mode == HierarchyMode.FLAT:
            _generate_flat_hierarchy(
                hierarchy_stage,
                robot_root_prim,
                hierarchy_root_prim,
                current_root_path,
                path_map,
                joint_connections,
                stage,
            )
        elif mode == HierarchyMode.MUJOCO:
            _create_hierarchy_node_mujoco(
                hierarchy_stage,
                robot_tree,
                hierarchy_root_prim,
                None,
                None,
                current_root_path,
                path_map,
                joint_connections,
            )
        else:
            _create_hierarchy_node(
                hierarchy_stage,
                robot_tree,
                hierarchy_root_prim,
                None,
                None,
                current_root_path,
                path_map,
                joint_connections,
            )

    return hierarchy_stage, path_map, joint_connections
