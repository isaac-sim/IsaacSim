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

"""Utility helpers for asset importer preprocessing steps."""

from __future__ import annotations

import json
import logging
import os

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .physx_types import PhysxAttr, PhysxMimicAttr, PhysxMimicRel, PhysxSchema

__all__ = [
    "PhysxAttr",
    "PhysxMimicAttr",
    "PhysxMimicRel",
    "PhysxSchema",
    "USD_GEOMETRY_TYPES",
    "MESH_APPROXIMATION_MAP",
    "PHYSICS_AXIS_MAP",
    "ROBOT_TYPE_TOKENS",
    "collision_from_visuals",
    "enable_self_collision",
    "run_asset_transformer_profile",
    "delete_scope",
    "add_joint_schemas",
    "add_rigid_body_schemas",
    "remove_custom_scopes",
    "resolve_unique_path",
    "parse_robot_name",
    "create_robot_schema",
]

_logger = logging.getLogger(__name__)

# USD Geometry types that should be considered as collision shapes.
USD_GEOMETRY_TYPES = {"Mesh", "Cube", "Sphere", "Capsule", "Cylinder", "Cone"}


def resolve_unique_path(path: str, *, is_file: bool | None = None) -> str:
    """Return *path* or a numerically suffixed variant that does not collide on disk.

    A path is considered colliding if it is an existing file or a non-empty
    directory.  For files the counter is inserted before the extension
    (``robot_1.usda``); for directories it is appended (``usdex_robot_1``).

    Args:
        path: File or directory path to resolve.
        is_file: Explicit file/directory hint.  When ``None`` the type is
            inferred from the filesystem or from a conventional extension
            to avoid mangling directories with dots in their name.

    Returns:
        A path that does not cause a conflict on disk.

    Raises:
        RuntimeError: If no unique path could be resolved after 1000 attempts.
    """

    def _collides(candidate: str) -> bool:
        if not os.path.exists(candidate):
            return False
        if os.path.isdir(candidate):
            try:
                return any(os.scandir(candidate))
            except OSError:
                return True
        return True

    if not _collides(path):
        return path

    if is_file is None:
        if os.path.isdir(path):
            is_file = False
        elif os.path.isfile(path):
            is_file = True
        else:
            _, ext_guess = os.path.splitext(path)
            ext_body = ext_guess.lstrip(".")
            is_file = bool(ext_guess) and 1 <= len(ext_body) <= 5 and ext_body.isalnum()

    base, ext = os.path.splitext(path) if is_file else (path, "")

    for counter in range(1, 1001):
        candidate = f"{base}_{counter}{ext}"
        if not _collides(candidate):
            return candidate
    raise RuntimeError(f"Could not find a unique path after 1000 attempts for: {path}")


def parse_robot_name(path: str, *, expected_extension: str) -> str:
    """Derive a robot name from a URDF/MJCF file path and validate its extension.

    The robot name is the file's basename with its trailing extension
    removed.  Hidden-file style names (e.g. ``.franka.urdf``) are handled
    by stripping leading dots so the result is non-empty.  When the stem
    still contains ``.`` characters (e.g. ``franka.v2.urdf``) a warning
    is emitted because the stem is used verbatim as a USD prim / file
    name and dots there can cause downstream issues.

    Args:
        path: Path to the URDF or MJCF source file.
        expected_extension: Required extension including the leading dot
            (e.g. ``".urdf"`` or ``".xml"``).  Matched case-insensitively.

    Returns:
        The derived robot name (never empty).

    Raises:
        ValueError: If the file does not end with ``expected_extension``.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.utils import parse_robot_name
        >>> parse_robot_name("/tmp/franka.urdf", expected_extension=".urdf")
        'franka'
        >>> parse_robot_name("/tmp/.franka.urdf", expected_extension=".urdf")
        'franka'
    """
    if not expected_extension.startswith("."):
        raise ValueError(f"expected_extension must start with '.', got: {expected_extension!r}")

    basename = os.path.basename(path)
    stem, ext = os.path.splitext(basename)

    if ext.lower() != expected_extension.lower():
        raise ValueError(
            f"Expected file with extension '{expected_extension}', got '{ext or '<no extension>'}' for path: {path}"
        )

    # Strip leading dots so hidden-file names (e.g. ".franka.urdf") still yield a usable robot name.
    # The result is always non-empty here: ``os.path.splitext`` only returns a non-empty ``ext`` when
    # the basename has at least one non-dot character before the trailing extension.
    stem = stem.lstrip(".")

    if "." in stem:
        _logger.warning(
            "Robot name '%s' derived from '%s' contains '.' characters; "
            "USD prim and file names with dots may cause downstream issues. "
            "Consider renaming the source file.",
            stem,
            path,
        )

    return stem


# Mapping from UI-friendly labels to mesh collision approximation tokens.
MESH_APPROXIMATION_MAP = {
    "Convex Hull": UsdPhysics.Tokens.convexHull,
    "Convex Decomposition": UsdPhysics.Tokens.convexDecomposition,
    "Bounding Sphere": UsdPhysics.Tokens.boundingSphere,
    "Bounding Cube": UsdPhysics.Tokens.boundingCube,
}

PHYSICS_AXIS_MAP = {
    "X": UsdPhysics.Tokens.rotX,
    "Y": UsdPhysics.Tokens.rotY,
    "Z": UsdPhysics.Tokens.rotZ,
}


def collision_from_visuals(stage: Usd.Stage, collision_type: str) -> int:
    """Apply collisions from visual geometry and remove guide colliders.

    Args:
        stage: USD stage for authoring collision APIs.
        collision_type: Collision approximation label. Defaults to convex hull when unknown.

    Returns:
        Number of visual geometry prims processed.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.asset.importer.utils import collision_from_visuals
        >>>
        >>> stage = Usd.Stage.CreateInMemory()
        >>> stage_utils.use_stage(stage)
        >>> collision_from_visuals(stage, "Convex Hull")  # doctest: +SKIP
    """
    removed_colliders: list[Sdf.Path] = []
    removed_count = 0
    processed_count = 0

    for prim in stage.Traverse():
        prim_type = prim.GetTypeName()
        prim_path = prim.GetPath().pathString
        try:
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                if prim_type in USD_GEOMETRY_TYPES:
                    imageable = UsdGeom.Imageable(prim)
                    purpose = imageable.GetPurposeAttr().Get()
                    if purpose == UsdGeom.Tokens.guide:
                        removed_colliders.append(prim.GetPath())
                        continue

            if prim_type not in USD_GEOMETRY_TYPES:
                continue

            imageable = UsdGeom.Imageable(prim)
            purpose = imageable.GetPurposeAttr().Get()
            if purpose not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
                continue

            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI(prim)
                collision_enabled_attr = collision_api.GetCollisionEnabledAttr()
                if not collision_enabled_attr:
                    collision_enabled_attr = collision_api.CreateCollisionEnabledAttr()
                collision_enabled_attr.Set(True)
            else:
                UsdPhysics.CollisionAPI.Apply(prim)

            if prim_type == "Mesh":
                if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                else:
                    mesh_collision_api = UsdPhysics.MeshCollisionAPI(prim)

                approx_type = MESH_APPROXIMATION_MAP.get(collision_type, UsdPhysics.Tokens.convexHull)
                mesh_collision_api.GetApproximationAttr().Set(approx_type)

            processed_count += 1
        except Exception as exc:
            _logger.error(f"Error processing prim {prim_path}: {exc}")
            continue

    for prim_path in removed_colliders:
        stage.RemovePrim(prim_path)
        removed_count += 1

    _logger.info(f"Removed {removed_count} guide collision geometries")
    _logger.info(f"Processed collision for {processed_count} visual geometries")
    return processed_count


def enable_self_collision(usd_stage: Usd.Stage, enabled: bool = True) -> int:
    """Enable self-collisions on articulation roots.

    Args:
        usd_stage: USD stage for authoring articulation attributes.
        enabled: Whether to enable self collisions on articulation roots.

    Returns:
        Number of articulation roots updated.
    """
    articulation_roots = [
        prim
        for prim in usd_stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        or prim.HasAPI("PhysicsArticulationRootAPI")
        or prim.HasAPI("NewtonArticulationRootAPI")
    ]

    if len(articulation_roots) == 0:
        # If no articulation root found, get default prim and create articulation root schema on it.
        default_prim = usd_stage.GetDefaultPrim()
        if not default_prim:
            return 0
        default_prim.ApplyAPI("PhysicsArticulationRootAPI")

        default_prim.ApplyAPI("NewtonArticulationRootAPI")
        attr = default_prim.GetAttribute("newton:selfCollisionEnabled")
        if not attr:
            attr = default_prim.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool)
        attr.Set(enabled)
        return 1

    for articulation_root in articulation_roots:
        if not articulation_root or not articulation_root.IsValid():
            continue

        if not articulation_root.HasAPI("PhysicsArticulationRootAPI"):
            articulation_root.ApplyAPI("PhysicsArticulationRootAPI")

        if not articulation_root.HasAPI("NewtonArticulationRootAPI"):
            articulation_root.ApplyAPI("NewtonArticulationRootAPI")

        attr = articulation_root.GetAttribute("newton:selfCollisionEnabled")
        if not attr:
            attr = articulation_root.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool)
        attr.Set(enabled)

    return len(articulation_roots)


def run_asset_transformer_profile(
    input_stage_path: str,
    output_package_root: str,
    profile_json_path: str,
    *,
    log_path: str | None = None,
) -> None:
    """Run an asset structure profile against an input stage.

    Args:
        input_stage_path: Path to the input USD stage.
        output_package_root: Destination folder for output assets.
        profile_json_path: Path to the profile JSON file.
        log_path: Optional JSON report output path.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.utils import run_asset_transformer_profile
        >>>
        >>> run_asset_transformer_profile(
        ...     input_stage_path="/tmp/input.usda",
        ...     output_package_root="/tmp/output",
        ...     profile_json_path="/tmp/profile.json",
        ... )  # doctest: +SKIP
    """
    from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile
    from isaacsim.asset.transformer.rules import register_all_rules

    # In Kit, rules are registered by the extension on_startup. Standalone
    # callers need this explicit call (idempotent — safe to call twice).
    register_all_rules()

    with open(profile_json_path, encoding="utf-8") as handle:
        profile = RuleProfile.from_json(handle.read())

    manager = AssetTransformerManager()
    report = manager.run(
        input_stage=input_stage_path,
        profile=profile,
        package_root=output_package_root,
    )

    if log_path:
        log_dir = os.path.dirname(log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        with open(log_path, "w", encoding="utf-8") as log_file:
            json.dump(json.loads(report.to_json()), log_file, indent=2)


def delete_scope(stage: Usd.Stage, prim_path: str) -> None:
    """Delete a scope prim from the stage, reparenting its children to the parent prim.

    Args:
        stage: USD stage containing the prim.
        prim_path: Path to the prim to delete.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid() or prim.IsPseudoRoot():
        _logger.warning(f"Scope does not exist at path: {prim_path}")
        return
    if not prim.IsA(UsdGeom.Scope):
        _logger.warning(f"Prim is not a scope at path: {prim_path}")
        return

    parent_prim = prim.GetParent()
    if not parent_prim:
        _logger.warning(f"Scope has no parent at path: {prim_path}")
        parent_prim = stage.GetPseudoRoot()

    children = list(prim.GetChildren())
    namespace_editor = Usd.NamespaceEditor(stage)
    for child in children:
        namespace_editor.ReparentPrim(child, parent_prim)

    namespace_editor.ApplyEdits()

    stage.RemovePrim(Sdf.Path(prim_path))


def add_joint_schemas(stage: Usd.Stage) -> None:
    """Apply joint-related physics schemas to all joint prims.

    Args:
        stage: USD stage to update with joint schemas.

    """
    for prim in stage.Traverse():
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue

        if not prim.HasAPI(PhysxSchema.JOINT_API):
            prim.ApplyAPI(PhysxSchema.JOINT_API)
        instance_name = "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear"

        if not prim.HasAPI(UsdPhysics.DriveAPI, instance_name):
            UsdPhysics.DriveAPI.Apply(prim, instance_name)

        if not prim.HasAPI(PhysxSchema.JOINT_STATE_API, instance_name):
            prim.ApplyAPI(PhysxSchema.JOINT_STATE_API, instance_name)


def add_rigid_body_schemas(stage: Usd.Stage) -> None:
    """Apply :class:`UsdPhysics.MassAPI` to every rigid body that lacks it.

    This function is deprecated, and will be removed in a future version. Use `asset_utils.apply_link_density()` instead.

    Args:
        stage: USD stage to update with rigid body schemas.
    """
    for prim in stage.Traverse():
        if not (prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI("PhysicsRigidBodyAPI")):
            continue
        if not prim.HasAPI(UsdPhysics.MassAPI):
            UsdPhysics.MassAPI.Apply(prim)


def remove_custom_scopes(stage: Usd.Stage) -> None:
    """Remove custom scopes from the stage.

    Args:
        stage: USD stage to update with custom scopes.
    """
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        return

    default_prim_path = default_prim.GetPath()
    scope_paths = default_prim_path.pathString + "/Geometry/custom"
    scope = stage.GetPrimAtPath(scope_paths)
    if scope and scope.IsA(UsdGeom.Scope):
        stage.RemovePrim(Sdf.Path(scope_paths))
    return


ROBOT_TYPE_TOKENS = [
    "Default",
    "End Effector",
    "Manipulator",
    "Humanoid",
    "Wheeled",
    "Holonomic",
    "Quadruped",
    "Mobile Manipulators",
    "Aerial",
]


def create_robot_schema(
    stage: Usd.Stage,
    robot_type: str = "Default",
    *,
    prim_path: str | None = None,
    add_sites: bool = True,
    sites_last: bool = False,
) -> tuple[Usd.Prim | None, Usd.Prim | None]:
    """Apply the Isaac robot schema to a prim and populate link/joint relationships.

    If the target prim already has the ``IsaacRobotAPI`` applied, the schema is
    recalculated (preserving existing ordering).  Otherwise a fresh
    ``RobotAPI`` is applied and populated from the articulation hierarchy.

    No-op with a warning if ``usd.schema.isaac.robot_schema`` is unavailable.

    Args:
        stage: The USD stage containing the robot.
        robot_type: Robot category token.  Must be one of
            :data:`ROBOT_TYPE_TOKENS` (e.g. ``"Manipulator"``, ``"Humanoid"``).
        prim_path: Prim path to apply the schema to.  Defaults to the stage
            default prim when ``None``.
        add_sites: Detect child Xforms with no children under each link and
            apply ``IsaacSiteAPI`` to them.
        sites_last: When ``True`` all sites are appended at the end of the
            links list; when ``False`` each site follows its parent link.

    Returns:
        A ``(root_link, root_joint)`` tuple of the detected articulation root
        link and root joint.  Either may be ``None``; both are ``None`` when
        the robot schema module is unavailable.

    Raises:
        ValueError: If *robot_type* is not in :data:`ROBOT_TYPE_TOKENS` or
            the resolved prim is invalid.
    """
    if robot_type not in ROBOT_TYPE_TOKENS:
        raise ValueError(f"Invalid robot_type '{robot_type}'. Must be one of: {ROBOT_TYPE_TOKENS}")

    try:
        import usd.schema.isaac.robot_schema as rs
        from usd.schema.isaac.robot_schema import utils as robot_schema_utils
    except ImportError:
        _logger.warning(
            "usd.schema.isaac.robot_schema is not available; skipping IsaacRobotAPI. "
            "Enable the 'isaacsim.robot.schema' extension to populate robot schema attributes."
        )
        return None, None

    if prim_path:
        robot_prim = stage.GetPrimAtPath(prim_path)
    else:
        robot_prim = stage.GetDefaultPrim()

    if not robot_prim or not robot_prim.IsValid():
        raise ValueError(f"Invalid prim at path '{prim_path or '<default prim>'}'")

    has_existing_schema = robot_prim.HasAPI(rs.Classes.ROBOT_API.value)

    if has_existing_schema:
        _logger.info("RobotAPI already applied — recalculating schema while preserving order")
        robot_schema_utils.UpdateDeprecatedSchemas(robot_prim)
        root_link, root_joint = robot_schema_utils.RecalculateRobotSchema(
            stage,
            robot_prim,
            robot_prim,
            detect_sites=add_sites,
            sites_last=sites_last,
        )
    else:
        rs.ApplyRobotAPI(robot_prim)
        _logger.info("Applied RobotAPI to prim %s", robot_prim.GetPath())
        root_link, root_joint = robot_schema_utils.PopulateRobotSchemaFromArticulation(
            stage,
            robot_prim,
            robot_prim,
            detect_sites=add_sites,
            sites_last=sites_last,
        )

    robot_type_attr = robot_prim.GetAttribute("isaac:robotType")
    if robot_type_attr and robot_type_attr.IsValid():
        robot_type_attr.Set(robot_type)
    else:
        robot_type_attr = robot_prim.CreateAttribute("isaac:robotType", Sdf.ValueTypeNames.Token)
        robot_type_attr.Set(robot_type)

    _logger.info("Set isaac:robotType = '%s' on %s", robot_type, robot_prim.GetPath())

    return root_link, root_joint
