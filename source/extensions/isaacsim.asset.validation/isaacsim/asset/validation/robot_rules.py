# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import omni.asset_validator.core as av_core
import omni.client
from omni.asset_validator.core import AuthoringLayers, registerRule
from pxr import Usd
from usd.schema.isaac import robot_schema

from .util import is_relationship_prepended, make_relationship_prepended


@registerRule("IsaacSim.RobotRules")
class RobotNaming(av_core.BaseRuleChecker):
    """Validates that robot assets follow the standard naming convention.

    This rule checks that robot assets follow the naming convention of
    <Manufacturer>/<robot>/<robot.usd> or <Manufacturer>/<robot>/<version>/<robot.usd>.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        """Check if the robot asset follows proper naming conventions.

        Args:
            stage: The USD stage to validate.
        """
        path = stage.GetRootLayer().realPath

        parts = path.replace("\\", "/").split("/")

        if len(parts) < 3:
            self._AddWarning(
                message=f"Robot not nested in enough folders: must be at least <Manufacturer>/<robot>/<robot.usd>: <{path}>",
                at=stage,
            )
            return

        # try Manufacturer/Robot/Robot.usd
        if parts[-2].lower() != parts[-1].split(".")[0].lower():
            # Does not match parts[-2]; try this format Manufacturer/Robot/Version/Robot.usd
            if len(parts) >= 4:
                if parts[-3].lower() == parts[-1].split(".")[0].lower():
                    # nothing wrong; we do match parts[-3]
                    return
            self._AddError(message=f"Folder name does not match Robot name: <{parts[-2]}> != <{parts[-1]}>", at=stage)


@registerRule("IsaacSim.RobotRules")
class CleanFolder(av_core.BaseRuleChecker):
    """Validates that robot asset folders don't contain unexpected files.

    This rule checks that the folder containing a robot asset doesn't contain
    unexpected files that might cause confusion or conflicts.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        folders = stage.GetRootLayer().realPath.replace("\\", "/").split("/")
        folder = "/".join(folders[:-1])
        res, entries = omni.client.list(folder)
        if res != omni.client.Result.OK:
            self._AddError(message=f"Failed to list folder <{folder}>", at=stage)
            return
        for entry in entries:
            if entry.relative_path.lower() == folders[-1].lower():
                continue
            if entry.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
                continue
            self._AddWarning(message=f"Folder <{folder}> contains unexpected file <{entry.relative_path}>")


def get_overridden_attributes(prim):
    """Get list of attribute names that have overridden values.

    Args:
        prim: The USD prim to check for overridden attributes.

    Returns:
        List of attribute names that have overridden values.
    """
    overridden_attrs = []

    for attr in prim.GetAttributes():
        # Check if this attribute has authored value in the current edit target
        if attr.HasAuthoredValue():
            # Get the layer where the value is authored
            layer_stack = prim.GetStage().GetLayerStack()
            for layer in layer_stack:
                attr_spec = layer.GetAttributeAtPath(attr.GetPath())
                if attr_spec and attr_spec.HasDefaultValue():
                    overridden_attrs.append(attr.GetName())
                    break

    return overridden_attrs


@registerRule("IsaacSim.RobotRules")
class NoOverrides(av_core.BaseRuleChecker):
    """Validates that prims don't have overridden attributes.

    This rule checks that prims don't have attributes with the SpecifierOver specifier,
    which can cause unexpected behavior in robot assets. This only applies for the open stage.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:

        if "/Render" in prim.GetPath().pathString:
            return

        attrs = get_overridden_attributes(prim)
        if len(attrs) > 0:
            self._AddError(
                message=f"Prim is overridden: {prim.GetPath()}, {attrs}",
                at=prim,
            )


@registerRule("IsaacSim.RobotRules")
class RobotSchema(av_core.BaseRuleChecker):
    """Validates that robot assets have the required RobotAPI and relationships.

    This rule checks that robot assets have a default prim with the RobotAPI applied
    and the required robotLinks and robotJoints relationships defined.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        prim = stage.GetDefaultPrim()
        if not prim:
            self._AddError(
                message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> is not set",
                at=stage,
            )
            return

        if not prim.HasAPI(robot_schema.Classes.ROBOT_API.value):
            self._AddError(
                message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> does not have a RobotAPI",
                at=prim,
            )

        links_rel = prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
        if links_rel:
            if len(links_rel.GetTargets()) == 0:
                self._AddWarning(
                    message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> has no entries in isaac:physics:robotLinks",
                    at=links_rel,
                )
        else:
            self._AddError(
                message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> does not have a isaac:physics:robotLinks relationship",
                at=prim,
            )

        joints_rel = prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
        if joints_rel:
            if len(joints_rel.GetTargets()) == 0:
                self._AddWarning(
                    message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> has no entries in isaac:physics:robotJoints",
                    at=links_rel,
                )
        else:
            self._AddError(
                message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> does not have a isaac:physics:robotJoints relationship",
                at=prim,
            )


@registerRule("IsaacSim.RobotRules")
class JointsExist(av_core.BaseRuleChecker):
    """Validates that robot assets contain at least one joint.

    This rule checks that robot assets have at least one prim with the JointAPI
    applied, which is typically required for articulated robots.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        for prim in stage.Traverse():
            if prim.HasAPI(robot_schema.Classes.JOINT_API.value):
                return
        self._AddWarning(
            message=f"No joints found in robot asset <{stage.GetRootLayer().realPath}>",
            at=stage,
        )


@registerRule("IsaacSim.RobotRules")
class LinksExist(av_core.BaseRuleChecker):
    """Validates that robot assets contain at least one link.

    This rule checks that robot assets have at least one prim with the LinkAPI
    applied, which is typically required for articulated robots.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        for prim in stage.Traverse():
            if prim.HasAPI(robot_schema.Classes.LINK_API.value):
                return
        self._AddWarning(
            message=f"No links found in robot asset <{stage.GetRootLayer().realPath}>",
            at=stage,
        )


@registerRule("IsaacSim.RobotRules")
class ThumbnailExists(av_core.BaseRuleChecker):
    """Validates that robot assets have a thumbnail image.

    This rule checks that robot assets have a thumbnail image at the expected
    path, which is used for display in asset browsers.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        folders = stage.GetRootLayer().realPath.replace("\\", "/").split("/")
        folder = "/".join(folders[:-1])
        thumbnail_path = f"{folder}/.thumbs/256x256/{folders[-1]}.png"
        if omni.client.stat(thumbnail_path)[0] != omni.client.Result.OK:
            self._AddWarning(
                message=f"No thumbnail found at {thumbnail_path} for robot asset <{stage.GetRootLayer().realPath}>",
                at=stage,
            )


@registerRule("IsaacSim.RobotRules")
class CheckRobotRelationships(av_core.BaseRuleChecker):
    """Validates that robot relationships are properly defined and prepended.

    This rule checks that robot assets have the required robotLinks and robotJoints
    relationships defined and that they are prepended for proper composition.
    """

    @classmethod
    def create_link_relationship(cls, stage, prim):
        """Create the robotLinks relationship on a prim.

        Args:
            stage: The USD stage containing the prim.
            prim: The prim to create the relationship on.
        """
        relationship = prim.CreateRelationship(robot_schema.Relations.ROBOT_LINKS.name)

    @classmethod
    def create_joint_relationship(cls, stage, prim):
        """Create the robotJoints relationship on a prim.

        Args:
            stage: The USD stage containing the prim.
            prim: The prim to create the relationship on.
        """
        relationship = prim.CreateRelationship(robot_schema.Relations.ROBOT_JOINTS.name)

    @classmethod
    def make_joint_relationship_prepended(cls, stage, prim):
        """Make the robotJoints relationship prepended for composition.

        Args:
            stage: The USD stage containing the prim.
            prim: The prim with the relationship to modify.
        """
        relationship = prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
        make_relationship_prepended(relationship)

    @classmethod
    def make_link_relationship_prepended(cls, stage, prim):
        """Make the robotLinks relationship prepended for composition.

        Args:
            stage: The USD stage containing the prim.
            prim: The prim with the relationship to modify.
        """
        relationship = prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
        make_relationship_prepended(relationship)

    def CheckStage(self, stage: Usd.Stage) -> None:
        """Check if robot relationships are properly configured.

        Args:
            stage: The USD stage to validate.
        """
        prim = stage.GetDefaultPrim()
        if not prim:
            self._AddError(
                message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> is not set",
                at=stage,
            )
            return

        if prim.HasAPI(robot_schema.Classes.ROBOT_API.value):
            relationship_name_list = [robot_schema.Relations.ROBOT_LINKS.name, robot_schema.Relations.ROBOT_JOINTS.name]
            fix_methods = [self.create_link_relationship, self.create_joint_relationship]
            make_methods = [self.make_link_relationship_prepended, self.make_joint_relationship_prepended]
            for relationship_name, fix_method, make_method in zip(relationship_name_list, fix_methods, make_methods):
                relationship = prim.GetRelationship(relationship_name)
                if not relationship:
                    self._AddError(
                        message=f"DefaultPrim in robot asset <{stage.GetRootLayer().realPath}> does not have a {relationship_name} relationship",
                        at=prim,
                        suggestion=av_core.Suggestion(
                            message="Create relationship", callable=fix_method, at=AuthoringLayers(prim)
                        ),
                    )
                    continue

                # Check if the relationship is prepended
                is_prepended = is_relationship_prepended(relationship)
                if not is_prepended:
                    self._AddError(
                        message=f"Relationship {relationship_name} is not prepended",
                        at=prim,
                        suggestion=av_core.Suggestion(message="Make relationship prepended", callable=make_method),
                    )


@registerRule("IsaacSim.RobotRules")
class VerifyRobotPhysicsAttributesSourceLayer(av_core.BaseRuleChecker):
    """Validates that physics attributes are authored in the physics layer.

    This rule checks that physics attributes in robot assets are authored in
    the physics layer (_physics.usd), following the recommended layer structure.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        # examine every physics attribute in the stage and ensure that they are authored in the physics layer
        for prim in stage.Traverse():
            for attr in prim.GetAttributes():
                property_stack = attr.GetPropertyStack()
                for stack_item in property_stack:
                    if attr.GetName().startswith("physics:") and not stack_item.layer.identifier.endswith(
                        "_physics.usd"
                    ):
                        self._AddWarning(
                            message=f"Physics Attribute {attr.GetName()} in robot asset <{stage.GetRootLayer().realPath}> has authored value NOT in the physics layer",
                            at=attr,
                        )


@registerRule("IsaacSim.RobotRules")
class VerifyRobotPhysicsSchemaSourceLayer(av_core.BaseRuleChecker):
    """Validates that physics schemas are applied in the physics layer.

    This rule checks that physics schemas in robot assets are applied in
    the physics layer (_physics.usd), following the recommended layer structure.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:
        # examine every prim schema in the stage and ensure that they are authored in the physics layer
        for prim in stage.Traverse():
            for layer in stage.GetLayerStack():
                prim_spec = layer.GetPrimAtPath(prim.GetPath())

                if not prim_spec:
                    continue

                api_schemas = prim_spec.GetInfo("apiSchemas")

                if api_schemas:
                    for applied_api in api_schemas.GetAppliedItems():
                        if (
                            applied_api.startswith("Physx") or applied_api.startswith("Physics")
                        ) and not layer.identifier.endswith("_physics.usd"):
                            self._AddWarning(
                                message=f"Physics Schema [{applied_api}] on {prim.GetPath()} in robot asset <{stage.GetRootLayer().realPath}> has applied schema NOT in the physics layer",
                                at=prim,
                            )
