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

"""USD authoring for the pre-play policy robot spawn.

:func:`spawn_policy_robot` is the entry point: it authors the robot prim and its env-config
spawn metadata in the ordered, load-bearing pre-``Articulation`` sequence. The property
helpers below it (:func:`ensure_joint_drives_exist`, :func:`apply_rigid_body_props`,
:func:`apply_articulation_root_props`) are its stages. Authoring order and the exact USD API
calls are behavior — verbatim 6.x code, do not reorder.
"""

from __future__ import annotations

import logging
import math
import re
from collections.abc import Mapping
from functools import cache
from typing import TYPE_CHECKING, Any

import carb
import omni.usd
from isaacsim.core.experimental.utils.prim import get_prim_at_path
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

if TYPE_CHECKING:
    from newton.usd import PrimType, SchemaResolver

    from .env_config import PolicyEnvConfig, SpawnProps


_logger = logging.getLogger(__name__)


_STANDARD_MATERIAL_DEFAULT_DESTINATIONS = {
    "static_mu": "physics:staticFriction",
    "dynamic_mu": "physics:dynamicFriction",
    "restitution": "physics:restitution",
}

_NEWTON_CONTACT_DEFAULT_FIELDS = ("ke", "kd", "kf", "ka")
_MATERIAL_DEFAULT_FIELDS = (*_NEWTON_CONTACT_DEFAULT_FIELDS, *_STANDARD_MATERIAL_DEFAULT_DESTINATIONS)


def spawn_policy_robot(
    prim_path: str,
    usd_path: str,
    spawn: SpawnProps,
    newton_shape_defaults: Mapping[str, Any],
) -> None:
    """Author a policy robot with the ordered, load-bearing spawn-time property sequence.

    The order is define/reference prim, Physics variant (+ PhysX articulation API walk),
    rigid-body props, placeholder joint drives, then articulation-root props, all before
    Articulation construction.

    Args:
        prim_path: Stage path to author the robot at; an existing valid prim is reused as-is.
        usd_path: Robot USD referenced onto the prim when it does not already exist.
        spawn: Env-config spawn block carrying the rigid-body/joint-drive/articulation props.
        newton_shape_defaults: Exported ``sim.physics.default_shape_cfg`` values.
    """
    prim = get_prim_at_path(prim_path)
    if not prim.IsValid():
        prim = define_prim(prim_path, "Xform")
        prim.GetReferences().AddReference(usd_path)

    # Variant and spawn props must be authored before Articulation construction.
    set_physics_variant(prim_path)
    stage = omni.usd.get_context().get_stage()
    _apply_newton_import_defaults(stage, prim.GetPath(), newton_shape_defaults)
    apply_rigid_body_props(stage, prim.GetPath(), spawn.rigid_body_props or {})
    joint_drive_props = spawn.joint_drive_props or {}
    if joint_drive_props.get("ensure_drives_exist"):
        ensure_joint_drives_exist(stage, prim.GetPath())
    apply_articulation_root_props(stage, prim.GetPath(), spawn.articulation_props or {})


# -- joint-drive authoring -------------------------------------------------------------------


def ensure_joint_drives_exist(stage: Usd.Stage, prim_path: str | Sdf.Path) -> None:
    """Ensure unambiguous one-DOF USD joint drives exist under an asset subtree.

    Walks the subtree breadth-first, stopping at instanced prims (their internals are not
    editable) and at each joint prim it can handle; a warning is logged when no joint under
    the subtree could be processed.

    Args:
        stage: Stage containing the asset subtree.
        prim_path: Path to the root prim of the asset subtree.
    """
    root_prim = stage.GetPrimAtPath(str(prim_path))
    if not root_prim.IsValid():
        raise ValueError(f"Prim at path '{prim_path}' is not valid.")

    count_success = 0
    instanced_prim_paths = []
    all_prims = [root_prim]
    while all_prims:
        prim = all_prims.pop(0)
        if prim.IsInstance():
            instanced_prim_paths.append(prim.GetPath().pathString)
            continue

        if _ensure_joint_drive_exists_on_prim(prim):
            count_success += 1
        else:
            all_prims += prim.GetChildren()

    if count_success == 0:
        _logger.warning(
            "Could not perform 'ensure_joint_drives_exist' on any prims under: '%s'."
            " This might be because of the following reasons:"
            "\n\t(1) The desired attribute does not exist on any of the prims."
            "\n\t(2) The desired attribute exists on an instanced prim."
            "\n\t\tDiscovered list of instanced prim paths: %s",
            prim_path,
            instanced_prim_paths,
        )


def _ensure_joint_drive_exists_on_prim(prim: Usd.Prim) -> bool:
    """Author a drive on one revolute/prismatic joint prim if it has none. Designed to be an IsaacLab mirror.

    Returns True when the prim is a handled one-DOF joint (the subtree walk stops there),
    False when it is not a joint (the walk descends into its children). Tendon-axis joints
    that are not the tendon root are skipped, and joints already carrying nonzero
    stiffness/damping are left untouched; joints without gains get a near-zero placeholder
    stiffness so the physics parser sees an authored drive without changing dynamics.

    Args:
        prim: Prim to inspect or author.

    Returns:
        True on success.
    """
    if prim.IsA(UsdPhysics.RevoluteJoint):
        drive_api_name = "angular"
        is_linear_drive = False
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        drive_api_name = "linear"
        is_linear_drive = True
    elif _is_multi_axis_joint(prim):
        raise NotImplementedError(
            f"ensure_drives_exist cannot infer missing drives for multi-axis joint {prim.GetPath()}. "
            "Author the required drive axes in the USD asset."
        )
    else:
        return False

    applied_schemas_str = str(prim.GetAppliedSchemas())
    if "PhysxTendonAxisAPI" in applied_schemas_str and "PhysxTendonAxisRootAPI" not in applied_schemas_str:
        return False

    drive_api = UsdPhysics.DriveAPI(prim, drive_api_name)
    if not drive_api:
        drive_api = UsdPhysics.DriveAPI.Apply(prim, drive_api_name)

    stiffness = drive_api.GetStiffnessAttr().Get()
    damping = drive_api.GetDampingAttr().Get()
    if (stiffness is None or stiffness == 0.0) and (damping is None or damping == 0.0):
        placeholder_stiffness = 1.0e-3 if is_linear_drive else 1.0e-3 * math.pi / 180.0
        drive_api.CreateStiffnessAttr().Set(placeholder_stiffness)

    return True


def _is_multi_axis_joint(prim: Usd.Prim) -> bool:
    """Detect a D6 joint or a generic joint with per-axis drive/limit schema instances.

    Args:
        prim: Prim to inspect or author.

    Returns:
        True on success.
    """
    if prim.GetTypeName() == "PhysicsD6Joint":
        return True
    if not prim.IsA(UsdPhysics.Joint):
        return False
    for schema_name in prim.GetAppliedSchemas():
        api_name, _, instance_name = str(schema_name).partition(":")
        # UsdPhysics multi-axis (D6) drives/limits are schema instances namespaced per axis.
        if api_name in ("PhysicsDriveAPI", "PhysicsLimitAPI") and instance_name in (
            "transX",
            "transY",
            "transZ",
            "rotX",
            "rotY",
            "rotZ",
        ):
            return True
    return False


# -- task-scene materials --------------------------------------------------------------------


def _bind_physics_material(
    stage: Usd.Stage,
    prim_path: str | Sdf.Path,
    material_path: str | Sdf.Path,
    static_friction: float,
    dynamic_friction: float,
    restitution: float,
) -> None:
    """Create and strongly bind one physics material to a prim.

    Args:
        stage: Stage containing the target prim and material.
        prim_path: Path of the prim that receives the material binding.
        material_path: Path at which to define the physics material.
        static_friction: Material static friction coefficient.
        dynamic_friction: Material dynamic friction coefficient.
        restitution: Material restitution coefficient.
    """
    prim = stage.GetPrimAtPath(str(prim_path))
    if not prim.IsValid():
        raise ValueError(f"Prim at path {str(prim_path)!r} is not valid.")

    material = UsdShade.Material.Define(stage, str(material_path))
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr().Set(float(static_friction))
    material_api.CreateDynamicFrictionAttr().Set(float(dynamic_friction))
    material_api.CreateRestitutionAttr().Set(float(restitution))
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )


def apply_startup_material_events(
    stage: Usd.Stage,
    env_config: PolicyEnvConfig,
    scene_entities: dict[str, Any],
    material_scope_path: str | Sdf.Path,
    root_paths: dict[str, str | Sdf.Path] | None = None,
) -> set[str]:
    """Apply deterministic exported startup materials to named scene articulations.

    Args:
        stage: Stage containing the scene entities.
        env_config: Parsed exported environment configuration.
        scene_entities: Articulations keyed by exported scene entity name.
        material_scope_path: Scope for generated physics materials.
        root_paths: Optional asset roots for entities whose articulation path is not the
            referenced asset root.

    Returns:
        Names of scene entities that received a material binding.
    """
    root_paths = root_paths or {}
    applied_entities = set()
    for event in env_config.startup_material_events:
        entity = scene_entities.get(event.entity_name)
        if entity is None:
            continue
        if len(entity.paths) != 1:
            raise ValueError(
                f"Startup material event {event.name!r} requires one {event.entity_name!r} instance, "
                f"got {len(entity.paths)}."
            )

        root_path = root_paths.get(event.entity_name, entity.paths[0])
        if not event.body_patterns or any(pattern in ("*", ".*") for pattern in event.body_patterns):
            target_paths = [root_path]
        else:
            target_paths = []
            for pattern in event.body_patterns:
                try:
                    matcher = re.compile(pattern)
                except re.error as error:
                    raise ValueError(
                        f"Invalid body-name pattern {pattern!r} in startup material event {event.name!r}."
                    ) from error
                target_paths.extend(
                    path for name, path in zip(entity.link_names, entity.link_paths[0]) if matcher.fullmatch(name)
                )
            target_paths = list(dict.fromkeys(target_paths))
            if not target_paths:
                raise ValueError(
                    f"Startup material event {event.name!r} did not match any links on {event.entity_name!r}."
                )

        material_path = Sdf.Path(str(material_scope_path)).AppendChild(event.name)
        for target_path in target_paths:
            _bind_physics_material(
                stage,
                target_path,
                material_path,
                event.static_friction,
                event.dynamic_friction,
                event.restitution,
            )
        applied_entities.add(event.entity_name)

    return applied_entities


# -- exported-property authoring (rigid bodies / articulation roots) --------------------------


def _snake_to_camel(name: str) -> str:
    """Convert an exported snake_case field name to the camelCase USD attribute suffix.

    Args:
        name: Identifier to convert.

    Returns:
        The resolved string.
    """
    head, *tail = name.split("_")
    return head + "".join(word.title() for word in tail)


def _set_schema_attr(prim: Usd.Prim, attr_name: str, value: Any) -> None:
    """Set an attribute that must already be provided by an applied schema or preauthored USD.

    Args:
        prim: Prim to inspect or author.
        attr_name: USD attribute name to author.
        value: Raw exported value.
    """
    attr = prim.GetAttribute(attr_name)
    if not attr.IsValid():
        raise RuntimeError(
            f"Cannot set USD attribute {attr_name!r} on {prim.GetPath()}: "
            "attribute is not provided by an applied schema or preauthored USD."
        )
    attr.Set(value)


def _apply_schema(prim: Usd.Prim, schema_name: str | None) -> None:
    """Apply a named API schema once; None and already-applied schemas are no-ops.

    Args:
        prim: Prim to inspect or author.
        schema_name: Schema to apply, or None to skip.
    """
    if not schema_name or schema_name in prim.GetAppliedSchemas():
        return
    if not prim.ApplyAPI(schema_name):
        raise RuntimeError(f"Cannot apply USD API schema {schema_name!r} to {prim.GetPath()}.")


def _should_skip_config_value(value: Any) -> bool:
    """Skip unauthored exported values: None and the inf sentinels the exporter emits.

    Args:
        value: Raw exported value.

    Returns:
        True on success.
    """
    return value is None or (isinstance(value, float) and math.isinf(value))


# -- Newton import-default authoring ----------------------------------------------------------


def _has_authored_attr(prim: Usd.Prim | None, names: tuple[str, ...]) -> bool:
    if prim is None or not prim.IsValid():
        return False
    return any((attr := prim.GetAttribute(name)).IsValid() and attr.HasAuthoredValueOpinion() for name in names)


@cache
def _newton_schema_resolver() -> SchemaResolver:
    """Return the shared canonical Newton schema resolver.

    Returns:
        The canonical Newton schema resolver.
    """
    from newton.usd import SchemaResolverNewton

    return SchemaResolverNewton()


@cache
def _newton_schema_resolvers() -> tuple[SchemaResolver, ...]:
    """Return the shared resolver chain in Newton importer priority order.

    Returns:
        The Newton, MuJoCo, and PhysX schema resolvers in importer priority order.
    """
    from newton.usd import SchemaResolverMjc, SchemaResolverPhysx

    return (_newton_schema_resolver(), SchemaResolverMjc(), SchemaResolverPhysx())


def _resolve_authored_value(prim: Usd.Prim | None, prim_type: PrimType, key: str) -> Any | None:
    """Return the first authored value without falling through to resolver defaults.

    Args:
        prim: USD prim containing the property.
        prim_type: Newton resolver prim type.
        key: Newton resolver property key.

    Returns:
        The authored value, or None when the prim or property is unauthored.
    """
    if prim is None or not prim.IsValid():
        return None
    for resolver in _newton_schema_resolvers():
        value = resolver.get_value(prim, prim_type, key)
        if value is not None:
            return value
    return None


def _newton_attr_name(prim_type: PrimType, key: str) -> str:
    """Return the canonical Newton USD destination for a resolver key.

    Args:
        prim_type: Newton resolver prim type.
        key: Newton resolver property key.

    Returns:
        The canonical Newton USD attribute name.
    """
    return _newton_schema_resolver().mapping[prim_type][key].name


def _has_finite_contact_value(material_prim: Usd.Prim | None, collision_prim: Usd.Prim, key: str) -> bool:
    from newton.usd import PrimType

    values = (
        _resolve_authored_value(material_prim, PrimType.MATERIAL, key),
        _resolve_authored_value(collision_prim, PrimType.SHAPE, key),
    )
    return any(value is not None and math.isfinite(float(value)) for value in values)


def _make_subtree_editable(root_prim: Usd.Prim) -> Usd.Prim:
    pending = [root_prim]
    while pending:
        prim = pending.pop()
        if prim.IsInstance():
            prim.SetInstanceable(False)
        pending.extend(prim.GetChildren())
    return root_prim.GetStage().GetPrimAtPath(root_prim.GetPath())


def _apply_newton_collision_defaults(collision_prim: Usd.Prim, shape_defaults: Mapping[str, float]) -> None:
    from newton.usd import PrimType

    for name in ("margin", "gap"):
        if _resolve_authored_value(collision_prim, PrimType.SHAPE, name) is not None:
            continue
        _apply_schema(collision_prim, "NewtonCollisionAPI")
        _set_schema_attr(collision_prim, _newton_attr_name(PrimType.SHAPE, name), shape_defaults[name])


def _bind_newton_material_defaults(
    stage: Usd.Stage,
    collision_prim: Usd.Prim,
    material_scope_path: Sdf.Path,
    shape_defaults: Mapping[str, float],
    material_cache: dict[tuple[str, tuple[str, ...]], UsdShade.Material],
) -> None:
    from newton.usd import PrimType

    bound_material, _ = UsdShade.MaterialBindingAPI(collision_prim).ComputeBoundMaterial("physics")
    source_prim = bound_material.GetPrim() if bound_material else None
    missing_fields = []
    for name in _MATERIAL_DEFAULT_FIELDS:
        if name in _NEWTON_CONTACT_DEFAULT_FIELDS:
            has_value = _has_finite_contact_value(source_prim, collision_prim, name)
        else:
            has_value = _has_authored_attr(source_prim, (_STANDARD_MATERIAL_DEFAULT_DESTINATIONS[name],))
        if not has_value:
            missing_fields.append(name)
    missing_fields = tuple(missing_fields)
    if not missing_fields:
        return

    source_path = str(source_prim.GetPath()) if source_prim is not None else ""
    cache_key = (source_path, missing_fields)
    local_material = material_cache.get(cache_key)
    if local_material is None:
        UsdGeom.Scope.Define(stage, material_scope_path.GetParentPath())
        UsdGeom.Scope.Define(stage, material_scope_path)
        material_path = material_scope_path.AppendChild(f"Material_{len(material_cache)}")
        local_material = UsdShade.Material.Define(stage, material_path)
        if source_prim is not None:
            local_material.GetPrim().GetReferences().AddInternalReference(source_prim.GetPath())
        UsdPhysics.MaterialAPI.Apply(local_material.GetPrim())
        _apply_schema(local_material.GetPrim(), "NewtonMaterialAPI")

        material_values = {
            "ke": shape_defaults["ke"],
            "kd": shape_defaults["kd"],
            "kf": shape_defaults["kf"],
            "ka": shape_defaults["ka"],
            "static_mu": shape_defaults["mu"],
            "dynamic_mu": shape_defaults["mu"],
            "restitution": shape_defaults["restitution"],
        }
        for name in missing_fields:
            if name in _NEWTON_CONTACT_DEFAULT_FIELDS:
                destination_attr = _newton_attr_name(PrimType.MATERIAL, name)
            else:
                destination_attr = _STANDARD_MATERIAL_DEFAULT_DESTINATIONS[name]
            _set_schema_attr(local_material.GetPrim(), destination_attr, material_values[name])
        material_cache[cache_key] = local_material

    UsdShade.MaterialBindingAPI.Apply(collision_prim).Bind(local_material, materialPurpose="physics")


def _apply_newton_joint_defaults(root_prim: Usd.Prim, joint_defaults: Mapping[str, float]) -> int:
    from newton.usd import PrimType

    joint_count = 0
    for joint_prim in Usd.PrimRange(root_prim):
        if not joint_prim.IsA(UsdPhysics.Joint) or joint_prim.IsA(UsdPhysics.FixedJoint):
            continue

        values = {}
        if _resolve_authored_value(joint_prim, PrimType.JOINT, "armature") is None:
            values[_newton_attr_name(PrimType.JOINT, "armature")] = joint_defaults["armature"]

        if joint_prim.IsA(UsdPhysics.RevoluteJoint):
            limit_namespace = "angular"
            limit_scale = math.pi / 180.0
        elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
            limit_namespace = "linear"
            limit_scale = 1.0
        else:
            limit_namespace = None

        has_authored_mjc_limit = _has_authored_attr(joint_prim, ("mjc:solreflimit",))
        if limit_namespace is not None and not has_authored_mjc_limit:
            for name, suffix in (("limit_ke", "ke"), ("limit_kd", "kd")):
                generic_value = _resolve_authored_value(joint_prim, PrimType.JOINT, name)
                axis_value = _resolve_authored_value(joint_prim, PrimType.JOINT, f"limit_{limit_namespace}_{suffix}")
                if generic_value is None and axis_value is None:
                    values[_newton_attr_name(PrimType.JOINT, name)] = joint_defaults[name] * limit_scale

        if not values:
            continue
        _apply_schema(joint_prim, "NewtonJointAPI")
        for attr_name, value in values.items():
            _set_schema_attr(joint_prim, attr_name, value)
        joint_count += 1

    return joint_count


def apply_newton_robot_defaults(
    stage: Usd.Stage,
    prim_path: str | Sdf.Path,
    shape_defaults: Mapping[str, float],
    joint_defaults: Mapping[str, float],
) -> dict[str, int]:
    """Author fallback Newton import values under a robot subtree.

    Existing Newton, MuJoCo, PhysX, and standard USD physics values take precedence.
    Contact response defaults are authored on robot-local physics materials, leaving
    environment objects and the global Newton configuration unchanged.

    Args:
        stage: Stage containing the asset subtree.
        prim_path: Path to the root prim of the asset subtree.
        shape_defaults: Default collision-shape and material values.
        joint_defaults: Default joint values.

    Returns:
        Number of collision shapes, local materials, and joints updated.
    """
    root_prim = stage.GetPrimAtPath(str(prim_path))
    if not root_prim.IsValid():
        raise ValueError(f"Prim at path '{prim_path}' is not valid.")
    root_prim = _make_subtree_editable(root_prim)

    collision_prims = [
        prim for prim in Usd.PrimRange(root_prim) if prim.IsA(UsdGeom.Gprim) and prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    material_cache: dict[tuple[str, tuple[str, ...]], UsdShade.Material] = {}
    material_scope_path = root_prim.GetPath().AppendChild("PolicyNewtonDefaults").AppendChild("Materials")

    for collision_prim in collision_prims:
        _apply_newton_collision_defaults(collision_prim, shape_defaults)
        _bind_newton_material_defaults(stage, collision_prim, material_scope_path, shape_defaults, material_cache)

    return {
        "collision_shapes": len(collision_prims),
        "materials": len(material_cache),
        "joints": _apply_newton_joint_defaults(root_prim, joint_defaults),
    }


def _apply_newton_import_defaults(
    stage: Usd.Stage, prim_path: str | Sdf.Path, exported_shape_defaults: Mapping[str, Any]
) -> None:
    if SimulationManager.get_active_physics_engine() != "newton":
        return

    import newton

    builder = newton.ModelBuilder()
    shape = builder.default_shape_cfg
    joint = builder.default_joint_cfg
    shape_defaults = {
        "ke": shape.ke,
        "kd": shape.kd,
        "kf": shape.kf,
        "ka": shape.ka,
        "mu": shape.mu,
        "restitution": shape.restitution,
        "margin": shape.margin,
        "gap": shape.gap if shape.gap is not None else builder.rigid_gap,
    }
    for name in shape_defaults:
        value = exported_shape_defaults.get(name)
        if value is not None:
            shape_defaults[name] = value

    apply_newton_robot_defaults(
        stage,
        prim_path,
        shape_defaults,
        {"armature": joint.armature, "limit_ke": joint.limit_ke, "limit_kd": joint.limit_kd},
    )


def apply_rigid_body_props(stage: Usd.Stage, prim_path: str | Sdf.Path, props: dict[str, Any]) -> None:
    """Author rigid-body properties on rigid bodies under a prim.

    ``props`` is one exported env-config property dictionary: plain fields author into the
    solver namespace (``_usd_namespace``, default ``physxRigidBody``), the two ``physics:*``
    common fields author directly, and the ``_usd_*`` metadata keys steer schema application —
    ``_usd_applied_schema`` names an extra schema to apply, and ``_usd_field_exceptions`` maps
    schema -> (namespace, fields) for fields that live on a different schema/namespace than
    the rest. None and inf values are unauthored and skipped.

    Args:
        stage: Stage containing the rigid bodies.
        prim_path: Path to the root prim of the asset subtree.
        props: Exported rigid-body properties to author.
    """
    props = props or {}
    if not props:
        return
    root_prim = stage.GetPrimAtPath(str(prim_path))
    if not root_prim.IsValid():
        return

    common_fields = {
        "rigid_body_enabled": "physics:rigidBodyEnabled",
        "kinematic_enabled": "physics:kinematicEnabled",
    }
    skipped_fields = set(common_fields) | {"_usd_namespace", "_usd_applied_schema", "_usd_field_exceptions"}
    namespace = props.get("_usd_namespace") or "physxRigidBody"
    solver_fields = {k: v for k, v in props.items() if k not in skipped_fields and not _should_skip_config_value(v)}

    for prim in Usd.PrimRange(root_prim):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue

        for field, attr_name in common_fields.items():
            value = props.get(field)
            if not _should_skip_config_value(value):
                _set_schema_attr(prim, attr_name, value)

        fields = dict(solver_fields)
        if namespace == "physxRigidBody" and fields:
            _apply_schema(prim, "PhysxRigidBodyAPI")

        field_exceptions = props.get("_usd_field_exceptions") or {}
        for applied_schema, (exception_namespace, exception_fields) in field_exceptions.items():
            triggered = [
                (field, props.get(field))
                for field in exception_fields
                if not _should_skip_config_value(props.get(field))
            ]
            if not triggered:
                continue
            _apply_schema(prim, applied_schema)
            for field, value in triggered:
                _set_schema_attr(prim, f"{exception_namespace}:{_snake_to_camel(field)}", value)
                fields.pop(field, None)

        _apply_schema(prim, props.get("_usd_applied_schema"))
        for field, value in fields.items():
            _set_schema_attr(prim, f"{namespace}:{_snake_to_camel(field)}", value)


def apply_articulation_root_props(stage: Usd.Stage, prim_path: str | Sdf.Path, props: dict[str, Any]) -> None:
    """Author articulation-root properties on articulation roots under a prim.

    Same exported-dictionary contract as :func:`apply_rigid_body_props` (default namespace
    ``physxArticulation``, ``_usd_*`` metadata keys, None/inf skipped), applied to prims
    carrying a PhysX or Newton articulation-root API. ``fix_root_link`` is excluded — it is
    an Isaac Lab spawn flag, not an authorable USD attribute.

    Args:
        stage: Stage containing the articulation roots.
        prim_path: Path to the root prim of the asset subtree.
        props: Exported articulation-root properties to author.
    """
    props = props or {}
    namespace = props.get("_usd_namespace") or "physxArticulation"
    base_fields = {
        k: v
        for k, v in props.items()
        if not k.startswith("_") and k != "fix_root_link" and not _should_skip_config_value(v)
    }
    if not base_fields:
        return
    root_prim = stage.GetPrimAtPath(str(prim_path))
    if not root_prim.IsValid():
        return

    for prim in Usd.PrimRange(root_prim):
        if not (prim.HasAPI(UsdPhysics.ArticulationRootAPI) or prim.HasAPI("NewtonArticulationRootAPI")):
            continue

        fields = dict(base_fields)
        if namespace == "physxArticulation" and fields:
            _apply_schema(prim, "PhysxArticulationAPI")

        field_exceptions = props.get("_usd_field_exceptions") or {}
        for applied_schema, (exception_namespace, exception_fields) in field_exceptions.items():
            triggered = [
                (field, props.get(field))
                for field in exception_fields
                if not _should_skip_config_value(props.get(field))
            ]
            if not triggered:
                continue
            _apply_schema(prim, applied_schema)
            for field, value in triggered:
                _set_schema_attr(prim, f"{exception_namespace}:{_snake_to_camel(field)}", value)
                fields.pop(field, None)

        _apply_schema(prim, props.get("_usd_applied_schema"))
        for field, value in fields.items():
            _set_schema_attr(prim, f"{namespace}:{_snake_to_camel(field)}", value)


def ensure_physx_articulation_api(prim_path: str) -> None:
    """Apply ``PhysxArticulationAPI`` to every articulation root under ``prim_path`` missing it.

    Args:
        prim_path: Stage path of the target prim.
    """
    root_prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)

    for prim in Usd.PrimRange(root_prim):
        has_root_api = prim.HasAPI(UsdPhysics.ArticulationRootAPI) or prim.HasAPI("NewtonArticulationRootAPI")
        if not has_root_api:
            continue
        # Schema applied by string name: works under any active engine, while the typed
        # PhysxSchema class is only importable with PhysX loaded.
        if prim.HasAPI("PhysxArticulationAPI"):
            continue
        prim.ApplyAPI("PhysxArticulationAPI")
        carb.log_info(f"spawn_policy_robot: applied PhysxArticulationAPI to {prim.GetPath()}.")


def set_physics_variant(prim_path: str) -> None:
    """Select the robot's ``Physics`` variant for the active engine (case-insensitively), if any.

    Args:
        prim_path: Stage path of the target prim.
    """
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    variant_sets = prim.GetVariantSets()
    if "Physics" not in variant_sets.GetNames():
        return

    engine = (SimulationManager.get_active_physics_engine() or "").lower()
    # Newton assets ship their engine-specific data under a "mujoco" variant.
    target_variant = {"physx": "physx", "newton": "mujoco"}.get(engine, engine)

    variant_set = variant_sets.GetVariantSet("Physics")
    available_variants = variant_set.GetVariantNames()
    for available in available_variants:
        if available.lower() == target_variant.lower():
            variant_set.SetVariantSelection(available)
            if engine == "physx":
                ensure_physx_articulation_api(prim_path)
            return

    carb.log_warn(
        f"spawn_policy_robot: requested Physics variant {target_variant!r} not available on "
        f"{prim_path}; available variants: {list(available_variants)}. Variant left unchanged."
    )
