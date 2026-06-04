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

"""Mujoco schema discovery and UI definitions for the Newton physics extension."""


def _get_mujoco_schema_names(pluginName: str) -> tuple[set[str], set[str]]:
    """Get all schema type names from the specified plugin.

    Args:
        pluginName: USD plugin name to query for schema types.

    Returns:
        A tuple of (prim_type_names, api_schema_names) where:
            - prim_type_names: Set of typed schema names (e.g., "PhysicsScene", "PhysicsJoint")
            - api_schema_names: Set of API schema names (e.g., "PhysicsRigidBodyAPI", "PhysicsCollisionAPI")
    """
    from pxr import Plug, Tf, Usd

    prim_type_names: set[str] = set()
    api_schema_names: set[str] = set()

    plug_registry = Plug.Registry()
    schema_registry = Usd.SchemaRegistry()

    plugin = plug_registry.GetPluginWithName(pluginName)
    if plugin is None:
        return prim_type_names, api_schema_names

    # Get all types derived from UsdSchemaBase
    all_types = Tf.Type.Find(Usd.SchemaBase).GetAllDerivedTypes()

    for tf_type in all_types:
        if not plugin.DeclaresType(tf_type):
            continue

        # Get the schema name for this type
        schema_type_name = schema_registry.GetSchemaTypeName(tf_type)
        if not schema_type_name:
            continue

        # Get schema kind to determine if it's a prim type or API schema
        schema_kind = schema_registry.GetSchemaKind(tf_type)

        if schema_kind in (Usd.SchemaKind.AbstractTyped, Usd.SchemaKind.ConcreteTyped):
            # This is a typed schema (prim type)
            prim_type_names.add(schema_type_name)
        elif schema_kind in (Usd.SchemaKind.SingleApplyAPI, Usd.SchemaKind.MultipleApplyAPI):
            # This is an API schema
            api_schema_names.add(schema_type_name)

    return prim_type_names, api_schema_names


def get_mujoco_schema_names() -> tuple[set[str], set[str]]:
    """Get all Mujoco schema type names.

    Returns:
        A tuple of (prim_type_names, api_schema_names) for Mujoco schemas.
    """
    prim_type_names: set[str] = set()
    api_schema_names: set[str] = set()

    prim_types, api_schemas = _get_mujoco_schema_names("mjcPhysics")
    prim_type_names.update(prim_types)
    api_schema_names.update(api_schemas)

    # List of schemas that are not capabilities
    not_capabilities = [
        "MjcCollisionAPI",
        "MjcMaterialAPI",
        "MjcImageableAPI",
        "MjcJointAPI",
    ]

    for schema_name in not_capabilities:
        prim_type_names.discard(schema_name)
        api_schema_names.discard(schema_name)

    return prim_type_names, api_schema_names


# NOTE: omni.physics.physx.ui/omni/physics/physxui/schemas/physxschema.py for reference

from omni.kit.property.physics.builders import HideWidgetBuilder
from pxr import UsdPhysics

from .utils import DisableByCallbackBuilder, PrimType, make_hide_cb

CallbackBuilder = DisableByCallbackBuilder
HIDE_PROPERTY = [HideWidgetBuilder]


class MujocoUiDefinitions:
    """UI definitions (widgets, property builders, ordering) for Mujoco schemas."""

    ignore = {}
    """Dictionary of schema types to ignore in UI rendering."""

    extensions = {UsdPhysics.Joint: ["MjcJointAPI"]}
    """Dictionary mapping USD schema types to their corresponding Mujoco API extensions."""

    extras = {}
    """Dictionary of additional UI configurations for specific schema types."""

    widgets = {}
    """Dictionary mapping schema properties to custom UI widget definitions."""

    property_builders = {
        "mjc:compiler:useThread": HIDE_PROPERTY,
        # Common Mjc/Newton properties - hidden when the Newton resolver provides the value.
        # SCENE
        "mjc:flag:gravity": [CallbackBuilder, make_hide_cb("mjc", PrimType.SCENE, "gravity_enabled", True)],
        "mjc:option:iterations": [CallbackBuilder, make_hide_cb("mjc", PrimType.SCENE, "max_solver_iterations", None)],
        "mjc:option:timestep": [CallbackBuilder, make_hide_cb("mjc", PrimType.SCENE, "time_steps_per_second", 1000)],
        # JOINT
        "mjc:armature": [CallbackBuilder, make_hide_cb("mjc", PrimType.JOINT, "armature", 0.0)],
        "mjc:frictionloss": [CallbackBuilder, make_hide_cb("mjc", PrimType.JOINT, "friction", 0.0)],
        "mjc:solref": [
            CallbackBuilder,
            make_hide_cb(
                "mjc",
                PrimType.JOINT,
                [
                    "limit_transX_ke",
                    "limit_transY_ke",
                    "limit_transZ_ke",
                    "limit_transX_kd",
                    "limit_transY_kd",
                    "limit_transZ_kd",
                    "limit_linear_ke",
                    "limit_angular_ke",
                    "limit_rotX_ke",
                    "limit_rotY_ke",
                    "limit_rotZ_ke",
                    "limit_linear_kd",
                    "limit_angular_kd",
                    "limit_rotX_kd",
                    "limit_rotY_kd",
                    "limit_rotZ_kd",
                ],
            ),
        ],
        # SHAPE
        "mjc:maxhullvert": [CallbackBuilder, make_hide_cb("mjc", PrimType.SHAPE, "max_hull_vertices", -1)],
        "mjc:margin": [CallbackBuilder, make_hide_cb("mjc", PrimType.SHAPE, "margin", 0.0)],
        "mjc:gap": [CallbackBuilder, make_hide_cb("mjc", PrimType.SHAPE, "gap", 0.0)],
        # MATERIAL
        "mjc:torsionalfriction": [CallbackBuilder, make_hide_cb("mjc", PrimType.MATERIAL, "mu_torsional", 0.005)],
        "mjc:rollingfriction": [CallbackBuilder, make_hide_cb("mjc", PrimType.MATERIAL, "mu_rolling", 0.0001)],
    }
    """Dictionary mapping property names to their corresponding UI builder classes."""
    property_order = {
        # Sorted by displayName; properties with a property_builder appear first.
        "MjcActuator": [
            # (no property_builders for MjcActuator)
            # sorted by displayName (property name used when no displayName)
            "mjc:actDim",  # actDim
            "mjc:actEarly",  # actEarly
            "mjc:actLimited",  # actLimited
            "mjc:actRange:max",  # actRange:max
            "mjc:actRange:min",  # actRange:min
            "mjc:biasPrm",  # biasPrm
            "mjc:biasType",  # biasType
            "mjc:crankLength",  # crankLength
            "mjc:ctrlLimited",  # ctrlLimited
            "mjc:ctrlRange:max",  # ctrlRange:max
            "mjc:ctrlRange:min",  # ctrlRange:min
            "mjc:dynPrm",  # dynPrm
            "mjc:dynType",  # dynType
            "mjc:forceLimited",  # forceLimited
            "mjc:forceRange:max",  # forceRange:max
            "mjc:forceRange:min",  # forceRange:min
            "mjc:gainPrm",  # gainPrm
            "mjc:gainType",  # gainType
            "mjc:gear",  # gear
            "mjc:group",  # Group
            "mjc:inheritRange",  # inheritRange
            "mjc:jointInParent",  # jointInParent
            "mjc:lengthRange:max",  # lengthRange:max
            "mjc:lengthRange:min",  # lengthRange:min
            "mjc:refSite",  # refSite
            "mjc:sliderSite",  # sliderSite
            "mjc:target",  # target
        ],
        "MjcCollisionAPI": [
            # property_builders first (sorted by displayName)
            "mjc:gap",  # Gap
            "mjc:margin",  # Margin
            "mjc:solref",  # SolRef
            # rest sorted by displayName
            "mjc:condim",  # ConDim
            "mjc:group",  # Group
            "mjc:priority",  # Priority
            "mjc:shellinertia",  # Shell Inertia
            "mjc:solimp",  # SolImp
            "mjc:solmix",  # SolMix
        ],
        "MjcEqualityAPI": [
            # property_builders first (sorted by displayName)
            "mjc:solref",  # SolRef
            # rest sorted by displayName
            "mjc:solimp",  # SolImp
            "mjc:target",  # target
        ],
        "MjcEqualityJointAPI": [
            # (no property_builders)
            "mjc:coef0",  # Coefficient 0
            "mjc:coef1",  # Coefficient 1
            "mjc:coef2",  # Coefficient 2
            "mjc:coef3",  # Coefficient 3
            "mjc:coef4",  # Coefficient 4
        ],
        "MjcEqualityWeldAPI": [
            # (no property_builders)
            "mjc:torqueScale",  # Torque Scale
        ],
        "MjcImageableAPI": [
            # (no property_builders)
            "mjc:group",  # Group
        ],
        "MjcJointAPI": [
            # property_builders first (sorted by displayName / property name)
            "mjc:armature",  # armature
            "mjc:frictionloss",  # frictionloss
            "mjc:margin",  # margin
            # rest sorted by displayName (property name used when no displayName)
            "mjc:actuatorfrclimited",  # actuatorfrclimited
            "mjc:actuatorfrcrange:max",  # actuatorfrcrange:max
            "mjc:actuatorfrcrange:min",  # actuatorfrcrange:min
            "mjc:actuatorgravcomp",  # actuatorgravcomp
            "mjc:damping",  # damping
            "mjc:group",  # Group
            "mjc:ref",  # ref
            "mjc:solimpfriction",  # solimpfriction
            "mjc:solimplimit",  # solimplimit
            "mjc:solreffriction",  # solreffriction
            "mjc:solreflimit",  # solreflimit
            "mjc:springdamper",  # springdamper
            "mjc:springref",  # springref
            "mjc:stiffness",  # stiffness
        ],
        "MjcKeyframe": [
            # (no property_builders; no displayNames — sorted by property name)
            "mjc:act",
            "mjc:ctrl",
            "mjc:mpos",
            "mjc:mquat",
            "mjc:qpos",
            "mjc:qvel",
        ],
        "MjcMaterialAPI": [
            # property_builders first (sorted by displayName)
            "mjc:rollingfriction",  # Rolling Friction
            "mjc:torsionalfriction",  # Torsional Friction
        ],
        "MjcMeshCollisionAPI": [
            # property_builders first (sorted by displayName)
            "mjc:maxhullvert",  # Maximum Hull Vertices
            # rest sorted by displayName
            "mjc:inertia",  # Inertia
        ],
        "MjcSceneAPI": [
            # property_builders first (sorted by displayName)
            "mjc:flag:gravity",  # Gravity Toggle
            "mjc:option:iterations",  # Solver Iterations
            "mjc:option:timestep",  # Timestep
            "mjc:compiler:useThread",  # Use Thread
            # rest sorted by displayName
            "mjc:flag:actuation",  # Actuation Forces Toggle
            "mjc:option:actuatorgroupdisable",  # Actuator Group Disable
            "mjc:compiler:alignFree",  # Align Free
            "mjc:compiler:angle",  # Angle
            "mjc:compiler:autoLimits",  # Automatic Limits
            "mjc:flag:autoreset",  # Automatic Simulation Reset Toggle
            "mjc:compiler:balanceInertia",  # Balance Inertia
            "mjc:compiler:boundInertia",  # Bound Inertia
            "mjc:compiler:boundMass",  # Bound Mass
            "mjc:option:ccd_iterations",  # CCD Iterations
            "mjc:option:ccd_tolerance",  # CCD Tolerance
            "mjc:flag:island",  # Constraint Island Discovery Toggle
            "mjc:flag:constraint",  # Constraint Solver Toggle
            "mjc:flag:contact",  # Contact Constraints and Collision Detection Toggle
            "mjc:option:o_friction",  # Contact Override Friction
            "mjc:option:o_margin",  # Contact Override Margin
            "mjc:flag:override",  # Contact Override Mechanism Toggle
            "mjc:option:o_solimp",  # Contact Override SolImp
            "mjc:option:o_solref",  # Contact Override SolRef
            "mjc:flag:clampctrl",  # Control Input Clamping Toggle
            "mjc:flag:damper",  # Damper Forces Toggle
            "mjc:option:density",  # Density
            "mjc:flag:invdiscrete",  # Discrete-Time Inverse Dynamics Toggle
            "mjc:flag:energy",  # Energy Computation Toggle
            "mjc:flag:equality",  # Equality Constraints Toggle
            "mjc:flag:eulerdamp",  # Euler Integrator Damping Toggle
            "mjc:compiler:fitAABB",  # Fit AABB
            "mjc:flag:fwdinv",  # Forward/Inverse Dynamics Comparison Toggle
            "mjc:option:cone",  # Friction Cone Type
            "mjc:flag:frictionloss",  # Friction Loss Constraints Toggle
            "mjc:compiler:fuseStatic",  # Fuse Static
            "mjc:option:impratio",  # Impedance Ratio
            "mjc:compiler:inertiaFromGeom",  # Inertia From Geom
            "mjc:compiler:inertiaGroupRange:max",  # Inertia Group Range Max
            "mjc:compiler:inertiaGroupRange:min",  # Inertia Group Range Min
            "mjc:option:integrator",  # Integrator
            "mjc:option:jacobian",  # Jacobian Type
            "mjc:flag:limit",  # Joint and Tendon Limit Constraints Toggle
            "mjc:option:ls_iterations",  # Linesearch Iterations
            "mjc:option:ls_tolerance",  # Linesearch Tolerance
            "mjc:option:magnetic",  # Magnetic Flux
            "mjc:flag:midphase",  # Mid-Phase Collision Filtering Toggle
            "mjc:flag:multiccd",  # Multiple Contact Collision Detection (CCD) Toggle
            "mjc:flag:nativeccd",  # Native Convex Collision Detection Toggle
            "mjc:option:noslip_iterations",  # Noslip Iterations
            "mjc:option:noslip_tolerance",  # Noslip Tolerance
            "mjc:flag:filterparent",  # Parent-Child Contact Filtering Toggle
            "mjc:compiler:saveInertial",  # Save Inertial
            "mjc:option:sdf_initpoints",  # SDF Initial Points
            "mjc:option:sdf_iterations",  # SDF Iterations
            "mjc:flag:sensor",  # Sensor Computations Toggle
            "mjc:compiler:setTotalMass",  # Set Total Mass
            "mjc:option:solver",  # Solver
            "mjc:flag:refsafe",  # Solver Reference Safety Mechanism Toggle
            "mjc:option:tolerance",  # Solver Tolerance
            "mjc:flag:warmstart",  # Solver Warm-Starting Toggle
            "mjc:flag:spring",  # Spring Forces Toggle
            "mjc:option:viscosity",  # Viscosity
            "mjc:option:wind",  # Wind Velocity
        ],
        "MjcSiteAPI": [
            # (no property_builders)
            "mjc:group",  # Group
        ],
        "MjcTendon": [
            # property_builders first (sorted by displayName)
            "mjc:armature",  # Armature
            "mjc:frictionloss",  # Friction Loss
            "mjc:margin",  # Margin
            # rest sorted by displayName
            "mjc:actuatorfrclimited",  # Actuator Force Limited
            "mjc:actuatorfrcrange:max",  # Actuator Force Range Max
            "mjc:actuatorfrcrange:min",  # Actuator Force Range Min
            "mjc:damping",  # Damping
            "mjc:group",  # Group
            "mjc:limited",  # Limited
            "mjc:path",  # path
            "mjc:path:coef",  # path:coef
            "mjc:path:divisors",  # path:divisors
            "mjc:path:indices",  # path:indices
            "mjc:path:segments",  # path:segments
            "mjc:range:max",  # Range Max
            "mjc:range:min",  # Range Min
            "mjc:rgba",  # RGBA
            "mjc:sideSites",  # sideSites
            "mjc:sideSites:indices",  # sideSites:indices
            "mjc:solimpfriction",  # Solver Impedance Friction
            "mjc:solimplimit",  # Solver Impedance Limit
            "mjc:solreffriction",  # Solver Reference Friction
            "mjc:solreflimit",  # Solver Reference Limit
            "mjc:springlength",  # Spring Length
            "mjc:stiffness",  # Stiffness
            "mjc:type",  # Type
            "mjc:width",  # Width
        ],
    }
    """Dictionary defining the display order of properties in the UI."""
