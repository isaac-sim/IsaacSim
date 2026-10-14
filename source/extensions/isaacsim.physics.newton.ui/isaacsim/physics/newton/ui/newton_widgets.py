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

"""Newton related UI menu widgets."""

from collections import namedtuple
from functools import partial
from typing import Any

from pxr import Usd

from .utils import NewtonWidgetBase, apply_codeless_api_with_dependencies

# newton-usd-schema are code-less, so we will have to use strings directly


def get_schema_properties(schema: str) -> list[Any]:
    """Collect the property specs declared by an applied API schema.

    Args:
        schema: Name of the applied API schema to inspect.

    Returns:
        The property spec for every property the schema declares.
    """
    registry = Usd.SchemaRegistry()
    schema_def = registry.FindAppliedAPIPrimDefinition(schema)
    if not schema_def:
        return []
    return [schema_def.GetSchemaPropertySpec(name) for name in schema_def.GetPropertyNames()]


# List of newton schemas from newton-usd-schemas
# "NewtonPhysicsSceneAPI" - ExtensionSchemaWidget (ExtendedNewtonSceneWidget)
# "NewtonPhysicsXpbdSceneAPI" - NewtonWidgetBase below
# "NewtonPhysicsKaminoSceneAPI" - NewtonWidgetBase below
# "NewtonPhysicsMPMSceneAPI" - NewtonWidgetBase below
# "NewtonPhysicsArticulationRootAPI"
# "NewtonPhysicsJointAPI" - NewtonWidgetBase below
# "NewtonPhysicsMassAPI"
# "NewtonPhysicsCollisionAPI"
# "NewtonPhysicsMeshCollisionAPI"
# "NewtonPhysicsSDFCollisionAPI"
# "NewtonPhysicsSiteAPI"
# "NewtonPhysicsPointsDeformableSimAPI"
# "NewtonPhysicsMaterialAPI"
# "NewtonPhysicsMPMMaterialAPI"
# "NewtonPhysicsCurvesDeformableMaterialAPI"
# "NewtonPhysicsMimicAPI"
# "NewtonPhysicsActuator" - Ignored, not a API schema, but a prim type
# "NewtonPhysicsActuatorDelayAPI"
# "NewtonPhysicsActuatorControlBaseAPI" - Ignored, base API schema
# "NewtonPhysicsPDControlAPI"
# "NewtonPhysicsPIDControlAPI"
# "NewtonPhysicsNeuralControlAPI"
# "NewtonPhysicsActuatorClampingBaseAPI" - Ignored, base API schema
# "NewtonPhysicsMaxEffortClampingAPI"
# "NewtonPhysicsDCMotorClampingAPI"
# "NewtonPhysicsPositionBasedClampingAPI"


NewtonSchemaItem = namedtuple(
    "NewtonSchemaItem",
    "name, prefix, title, menu_label, schema, collapsed, attributes, ignored_attributes, apply_fn, show_fn, relationships, exclusive_classes",
)

NewtonSchemaItem.__new__.__defaults__ = (
    False,  # collapsed
    [],  # attributes
    [],  # ignored_attributes
    None,  # apply_fn
    None,  # show_fn
    None,  # relationships
    None,  # exclusive_classes
)

controller_exclusive_classes = [
    "NewtonPDControlAPI",
    "NewtonPIDControlAPI",
    "NewtonNeuralControlAPI",
]

collision_exclusive_classes = [
    "NewtonMeshCollisionAPI",
    "NewtonSDFCollisionAPI",
]

solver_scene_exclusive_classes = [
    "NewtonXpbdSceneAPI",
    "NewtonKaminoSceneAPI",
    "NewtonMPMSceneAPI",
]

# Base APIs are applied transitively with derived control/clamping schemas; suppress their
# automatic parent-schema frames (they have no properties and no remove affordance).
_base_api_schemas = {
    "NewtonActuatorControlBaseAPI",
    "NewtonActuatorClampingBaseAPI",
}

newton_schema_menu_items = [
    NewtonSchemaItem(
        "NewtonPhysicsXpbdSceneAPI",  # name
        "Physics/Newton",  # prefix
        "Newton XPBD Scene",  # title
        "Newton XPBD Scene",  # menu label
        "NewtonXpbdSceneAPI",  # schema
        attributes=get_schema_properties("NewtonXpbdSceneAPI"),
        apply_fn=partial(
            apply_codeless_api_with_dependencies,
            dependencies=("NewtonSceneAPI",),
            api="NewtonXpbdSceneAPI",
        ),
        show_fn=lambda p: not p.HasAPI("NewtonXpbdSceneAPI") and p.IsA("PhysicsScene"),
        exclusive_classes=solver_scene_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsKaminoSceneAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Kamino Scene",  # title
        "Newton Kamino Scene",  # menu label
        "NewtonKaminoSceneAPI",  # schema
        attributes=get_schema_properties("NewtonKaminoSceneAPI"),
        apply_fn=partial(
            apply_codeless_api_with_dependencies,
            dependencies=("NewtonSceneAPI",),
            api="NewtonKaminoSceneAPI",
        ),
        show_fn=lambda p: not p.HasAPI("NewtonKaminoSceneAPI") and p.IsA("PhysicsScene"),
        exclusive_classes=solver_scene_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsMPMSceneAPI",  # name
        "Physics/Newton",  # prefix
        "Newton MPM Scene",  # title
        "Newton MPM Scene",  # menu label
        "NewtonMPMSceneAPI",  # schema
        attributes=get_schema_properties("NewtonMPMSceneAPI"),
        apply_fn=partial(
            apply_codeless_api_with_dependencies,
            dependencies=("NewtonSceneAPI",),
            api="NewtonMPMSceneAPI",
        ),
        show_fn=lambda p: not p.HasAPI("NewtonMPMSceneAPI") and p.IsA("PhysicsScene"),
        exclusive_classes=solver_scene_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsArticulationRootAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Articulation Root",  # title
        "Newton Articulation Root",  # menu label
        "NewtonArticulationRootAPI",  # schema
        attributes=get_schema_properties("NewtonArticulationRootAPI"),
        ignored_attributes=["newton:selfCollisionEnabled"],  # controlled by PhysX
        show_fn=lambda p: not p.HasAPI("NewtonArticulationRootAPI")
        and not p.IsA("NewtonActuator")
        and not p.IsA("PhysicsScene"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsJointAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Joint",  # title
        "Newton Joint",  # menu label
        "NewtonJointAPI",  # schema
        attributes=get_schema_properties("NewtonJointAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonJointAPI") and p.IsA("PhysicsJoint"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsMassAPI",  # name
        "Physics/Newton",  # prefix
        "NewtonMassAPI",  # title
        "Newton Mass",  # menu label
        "NewtonMassAPI",  # schema
        show_fn=lambda p: not p.HasAPI("NewtonMassAPI") and p.IsA("Xformable"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsCollisionAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Collider",  # title
        "Newton Collider",  # menu label
        "NewtonCollisionAPI",  # schema
        attributes=get_schema_properties("NewtonCollisionAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonCollisionAPI") and p.IsA("Gprim"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsMeshCollisionAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Mesh Collider",  # title
        "Newton Mesh Collider",  # menu label
        "NewtonMeshCollisionAPI",  # schema
        attributes=get_schema_properties("NewtonMeshCollisionAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonMeshCollisionAPI") and p.IsA("Mesh"),
        exclusive_classes=collision_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsSDFCollisionAPI",  # name
        "Physics/Newton",  # prefix
        "Newton SDF Collider",  # title
        "Newton SDF Collider",  # menu label
        "NewtonSDFCollisionAPI",  # schema
        attributes=get_schema_properties("NewtonSDFCollisionAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonSDFCollisionAPI") and p.IsA("Gprim"),
        exclusive_classes=collision_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsSiteAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Site",  # title
        "Newton Site",  # menu label
        "NewtonSiteAPI",  # schema
        attributes=get_schema_properties("NewtonSiteAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonSiteAPI") and p.IsA("Gprim"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsPointsDeformableSimAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Points Deformable Simulation",  # title
        "Newton Points Deformable Simulation",  # menu label
        "NewtonPointsDeformableSimAPI",  # schema
        attributes=get_schema_properties("NewtonPointsDeformableSimAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonPointsDeformableSimAPI") and p.IsA("Points"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsMaterialAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Material",  # title
        "Newton Material",  # menu label
        "NewtonMaterialAPI",  # schema
        attributes=get_schema_properties("NewtonMaterialAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonMaterialAPI") and p.IsA("Material"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsMPMMaterialAPI",  # name
        "Physics/Newton",  # prefix
        "Newton MPM Material",  # title
        "Newton MPM Material",  # menu label
        "NewtonMPMMaterialAPI",  # schema
        attributes=get_schema_properties("NewtonMPMMaterialAPI"),
        apply_fn=partial(
            apply_codeless_api_with_dependencies,
            dependencies=("PhysicsMaterialAPI",),
            api="NewtonMPMMaterialAPI",
        ),
        show_fn=lambda p: not p.HasAPI("NewtonMPMMaterialAPI") and p.IsA("Material"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsCurvesDeformableMaterialAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Curves Deformable Material",  # title
        "Newton Curves Deformable Material",  # menu label
        "NewtonCurvesDeformableMaterialAPI",  # schema
        attributes=get_schema_properties("NewtonCurvesDeformableMaterialAPI"),
        apply_fn=partial(
            apply_codeless_api_with_dependencies,
            dependencies=("PhysicsMaterialAPI",),
            api="NewtonCurvesDeformableMaterialAPI",
        ),
        show_fn=lambda p: not p.HasAPI("NewtonCurvesDeformableMaterialAPI") and p.IsA("Material"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsMimicAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Mimic Joint",  # title
        "Newton Mimic Joint",  # menu label
        "NewtonMimicAPI",  # schema
        attributes=get_schema_properties("NewtonMimicAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonMimicAPI") and p.IsA("PhysicsJoint"),
    ),
    # TODO: Newton Actuator are not exclusive to newton simulator, we should move them out of the Newton UI extension
    NewtonSchemaItem(
        "NewtonPhysicsActuatorDelayAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Actuator Delay",  # title
        "Newton Actuator Delay",  # menu label
        "NewtonActuatorDelayAPI",  # schema
        attributes=get_schema_properties("NewtonActuatorDelayAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonActuatorDelayAPI") and p.IsA("NewtonActuator"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsPDControlAPI",  # name
        "Physics/Newton",  # prefix
        "Newton PD Controller",  # title
        "Newton PD Controller",  # menu label
        "NewtonPDControlAPI",  # schema
        attributes=get_schema_properties("NewtonPDControlAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonPDControlAPI") and p.IsA("NewtonActuator"),
        exclusive_classes=controller_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsPIDControlAPI",  # name
        "Physics/Newton",  # prefix
        "Newton PID Controller",  # title
        "Newton PID Controller",  # menu label
        "NewtonPIDControlAPI",  # schema
        attributes=get_schema_properties("NewtonPIDControlAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonPIDControlAPI") and p.IsA("NewtonActuator"),
        exclusive_classes=controller_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsNeuralControlAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Neural Controller",  # title
        "Newton Neural Controller",  # menu label
        "NewtonNeuralControlAPI",  # schema
        attributes=get_schema_properties("NewtonNeuralControlAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonNeuralControlAPI") and p.IsA("NewtonActuator"),
        exclusive_classes=controller_exclusive_classes,
    ),
    NewtonSchemaItem(
        "NewtonPhysicsMaxEffortClampingAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Actuator Max Effort Clamping",  # title
        "Newton Actuator Max Effort Clamping",  # menu label
        "NewtonMaxEffortClampingAPI",  # schema
        attributes=get_schema_properties("NewtonMaxEffortClampingAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonMaxEffortClampingAPI") and p.IsA("NewtonActuator"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsDCMotorClampingAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Actuator DC Motor Clamping",  # title
        "Newton Actuator DC Motor Clamping",  # menu label
        "NewtonDCMotorClampingAPI",  # schema
        attributes=get_schema_properties("NewtonDCMotorClampingAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonDCMotorClampingAPI") and p.IsA("NewtonActuator"),
    ),
    NewtonSchemaItem(
        "NewtonPhysicsPositionBasedClampingAPI",  # name
        "Physics/Newton",  # prefix
        "Newton Actuator Position Based Clamping",  # title
        "Newton Actuator Position Based Clamping",  # menu label
        "NewtonPositionBasedClampingAPI",  # schema
        attributes=get_schema_properties("NewtonPositionBasedClampingAPI"),
        show_fn=lambda p: not p.HasAPI("NewtonPositionBasedClampingAPI") and p.IsA("NewtonActuator"),
    ),
]

# some schemas will need to be taken care of by some other logic involving PhysX and overlapping attributes,
# so we will let newton_schema to still create a widget for them, while we ignore the problematic schemas attributes in their widget (see the ignored_attributes)
not_ignored_schemas = ["NewtonArticulationRootAPI"]
# let newton_schemas know which schema has its own widget so it can skip them
ignored_schemas = {
    item.schema for item in newton_schema_menu_items if item.schema not in not_ignored_schemas
} | _base_api_schemas


class NewtonWidgets:
    """Collection of the property widgets for every Newton API schema."""

    def __init__(self) -> None:
        self._widgets = {}
        for item in newton_schema_menu_items:
            self._widgets[item.name] = NewtonWidgetBase(
                item.prefix,
                item.title,
                item.menu_label,
                item.schema,
                item.collapsed,
                item.attributes,
                item.ignored_attributes,
                item.apply_fn,
                item.show_fn,
                item.relationships,
                item.exclusive_classes,
            )

    def register(self, property_window: Any) -> None:
        """Register every Newton widget with the property window.

        Args:
            property_window: Property window module the widgets are registered with.
        """
        for name, widget in self._widgets.items():
            property_window.register_widget(name, widget)

    # we assume unregister is called before the widget object is destroyed
    def unregister(self, property_window: Any) -> None:
        """Remove every Newton widget from the property window.

        Args:
            property_window: Property window module the widgets were registered with.
        """
        for name in self._widgets:
            property_window.unregister_widget(name)
