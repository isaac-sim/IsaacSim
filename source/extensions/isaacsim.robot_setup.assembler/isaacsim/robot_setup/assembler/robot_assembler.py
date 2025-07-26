# SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import asyncio
from enum import IntEnum
from typing import List, Tuple

import carb
import numpy as np
import omni.kit.commands
import omni.timeline
import usd.schema.isaac.robot_schema as robot_schema
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.core.utils.prims import (
    get_articulation_root_api_prim_path,
    get_prim_at_path,
    get_prim_object_type,
    is_prim_path_valid,
)
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.string import find_unique_string_name
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .session_layer_util import start_assembly_session_sublayer, stop_assembly_session_sublayer

# ------------------------------------------
# JW - this code is from omni\extensions\ux\source\omni.physx.ui\python\scripts\utils.py
AXES_INDICES = {"X": 0, "Y": 1, "Z": 2}


def set_opposite_body_transform(stage, cache, prim, body0base, fixpos, fixrot):
    joint = UsdPhysics.Joint(prim)
    rel_tm = get_aligned_body_transform(stage, cache, joint, body0base)

    axis = 255
    if prim.IsA(UsdPhysics.PrismaticJoint):
        axis_attrib = prim.GetAttribute("physics:axis")
        axis = AXES_INDICES[axis_attrib.Get()] if axis_attrib.IsValid() else 255

    omni.kit.undo.begin_group()

    if fixpos:
        pos = rel_tm.GetTranslation()
        if body0base:
            if axis < 3:
                pos[axis] = joint.GetLocalPos1Attr().Get()[axis]
            omni.kit.commands.execute(
                "ChangePropertyCommand", prop_path=joint.GetLocalPos1Attr().GetPath(), value=pos, prev=None
            )
        else:
            if axis < 3:
                pos[axis] = joint.GetLocalPos0Attr().Get()[axis]
            omni.kit.commands.execute(
                "ChangePropertyCommand", prop_path=joint.GetLocalPos0Attr().GetPath(), value=pos, prev=None
            )

    if fixrot:
        rot = Gf.Quatf(rel_tm.GetRotation().GetQuat())
        if body0base:
            omni.kit.commands.execute(
                "ChangePropertyCommand", prop_path=joint.GetLocalRot1Attr().GetPath(), value=rot, prev=None
            )
        else:
            omni.kit.commands.execute(
                "ChangePropertyCommand", prop_path=joint.GetLocalRot0Attr().GetPath(), value=rot, prev=None
            )

    omni.kit.undo.end_group()


def get_aligned_body_transform(stage, cache, joint, body0base):
    # get both bodies if available
    b0paths = joint.GetBody0Rel().GetTargets()
    b1paths = joint.GetBody1Rel().GetTargets()

    b0prim = None
    b1prim = None

    if len(b0paths):
        b0prim = stage.GetPrimAtPath(b0paths[0])
        if not b0prim.IsValid():
            b0prim = None

    if len(b1paths):
        b1prim = stage.GetPrimAtPath(b1paths[0])
        if not b1prim.IsValid():
            b1prim = None

    b0locpos = joint.GetLocalPos0Attr().Get()
    b1locpos = joint.GetLocalPos1Attr().Get()
    b0locrot = joint.GetLocalRot0Attr().Get()
    b1locrot = joint.GetLocalRot1Attr().Get()

    # switch depending on which is the base
    if body0base:
        t0prim = b0prim
        t0locpos = b0locpos
        t0locrot = b0locrot
        t1prim = b1prim
    else:
        t0prim = b1prim
        t0locpos = b1locpos
        t0locrot = b1locrot
        t1prim = b0prim

    if t0prim:
        t0world = cache.GetLocalToWorldTransform(t0prim)
    else:
        t0world = Gf.Matrix4d()
        t0world.SetIdentity()

    if t1prim:
        t1world = cache.GetLocalToWorldTransform(t1prim)
    else:
        t1world = Gf.Matrix4d()
        t1world.SetIdentity()

    t0local = Gf.Transform()
    t0local.SetRotation(Gf.Rotation(Gf.Quatd(t0locrot)))
    t0local.SetTranslation(Gf.Vec3d(t0locpos))
    t0mult = t0local * Gf.Transform(t0world)

    t1world = Gf.Transform(t1world.GetInverse())
    rel_tm = t0mult * t1world
    return rel_tm


# ------------------------------------------


class AssembledBodies:
    """Class representing two assembled rigid bodies connected by a fixed joint.

    This class maintains references to the base and attach bodies, the fixed joint connecting them,
    and provides methods to manipulate their relative transform.
    """

    def __init__(
        self,
        base_path: str,
        attach_path: str,
        fixed_joint: UsdPhysics.FixedJoint,
        root_joints: List[UsdPhysics.Joint],
        attach_body_articulation_root: Usd.Prim,
        collision_mask=None,
    ):
        """Initialize an AssembledBodies instance.

        Args:
            base_path (str): Prim path of the base body
            attach_path (str): Prim path of the attach body
            fixed_joint (UsdPhysics.FixedJoint): Fixed joint connecting the bodies
            root_joints (List[UsdPhysics.Joint]): Root joints of the attach body
            attach_body_articulation_root (Usd.Prim): Articulation root of attach body
            collision_mask (Optional[Usd.Relationship]): Collision mask between bodies
        """
        self._base_path = base_path
        self._attach_path = attach_path
        self._fixed_joint = fixed_joint

        self._root_joints = root_joints

        self._is_assembled = True
        self._collision_mask = collision_mask
        self._articulation_root = attach_body_articulation_root

    @property
    def base_path(self) -> str:
        """Prim path of the base body

        Returns:
            str: Prim path of the base body
        """
        return self._base_path

    @property
    def attach_path(self) -> str:
        """Prim path of the floating (attach) body

        Returns:
            str: Prim path of the floating (attach) body
        """
        return self._attach_path

    @property
    def fixed_joint(self) -> UsdPhysics.FixedJoint:
        """USD fixed joint linking base and floating body together

        Returns:
            UsdPhysics.FixedJoint: USD fixed joint linking base and floating body together
        """
        return self._fixed_joint

    @property
    def attach_body_articulation_root(self) -> Usd.Prim:
        """USD articulation root of the floating body

        Returns:
            Usd.Prim: USD articulation root of the floating body
        """
        return self._articulation_root

    @property
    def root_joints(self) -> List[UsdPhysics.Joint]:
        """Root joints that tie the floating body to the USD stage.  These are disabled in an assembled body,
        and will be re-enabled by the disassemble() function.

        Returns:
            List[UsdPhysics.Joint]: Root joints that tie the floating body to the USD stage.
        """
        return self._root_joints

    @property
    def collision_mask(self) -> Usd.Relationship:
        """A Usd Relationship masking collisions between the two assembled bodies

        Returns:
            Usd.Relationship: A Usd Relationship masking collisions between the two assembled bodies
        """
        return self._collision_mask

    def get_fixed_joint_transform(self) -> Tuple[np.array, np.array]:
        """Get the transform between mount frames in composed robot.

        Returns:
            Tuple[np.array, np.array]: translation with shape (3,) and orientation with shape (4,)
        """
        if not self.is_assembled():
            carb.log_warn("Fixed joint no longer exists in composed robot.  Robots have been disassembled.")
            return None, None
        fixed_joint = self.fixed_joint
        translation = np.array(fixed_joint.GetLocalPos0Attr().Get())
        orientation = np.array(fixed_joint.GetLocalRot0Attr().Get())

        return translation, orientation

    def set_fixed_joint_transform(self, translation: np.array, orientation: np.array):
        """Set the transform between mount frames in the composed body.

        Args:
            translation (np.array): Local translation relative to mount frame on base body.
            orientation (np.array): Local quaternion orientation relative to mount frame on base body.
        """
        if not self.is_assembled():
            carb.log_warn("Fixed joint no longer exists in composed robot.  Rigid Bodies have been disassembled.")
            return
        fixed_joint = self.fixed_joint
        fixed_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*translation.astype(float)))
        fixed_joint.GetLocalRot0Attr().Set(Gf.Quatf(*orientation.astype(float)))

        # Do the physics solvers work for it so it can't mess up and
        # explode.

        fixed_joint_prim = fixed_joint.GetPrim()
        body0 = str(fixed_joint_prim.GetProperty("physics:body0").GetTargets()[0])
        body1 = str(fixed_joint_prim.GetProperty("physics:body1").GetTargets()[0])

        RobotAssembler._move_obj_b_to_local_pos(body0, self.attach_path, body1, translation, orientation)

        self._refresh_asset(self._attach_path)
        self._refresh_asset(self._base_path)

    def _unmask_collisions(self):
        if self._collision_mask is not None:
            [self._collision_mask.RemoveTarget(target) for target in self._collision_mask.GetTargets()]
            self._collision_mask = None

    def _refresh_asset(self, prim_path):
        # Refreshing payloads manually is a way to get the Articulation to update immediately while the timeline is
        # still playing.  Usd Physics should be doing this automatically, but there is currently a bug.  This function
        # will eventually become unnecessary.
        stage = get_current_stage()
        prim = get_prim_at_path(prim_path)

        composed_payloads = omni.usd.get_composed_payloads_from_prim(prim)
        if len(composed_payloads) != 0:
            payload = Sdf.Payload(prim_path)
            omni.kit.commands.execute("RemovePayload", stage=stage, prim_path=prim_path, payload=payload)
            omni.kit.commands.execute("AddPayload", stage=stage, prim_path=prim_path, payload=payload)

        composed_refs = omni.usd.get_composed_references_from_prim(prim)
        if len(composed_refs) != 0:
            reference = Sdf.Reference(prim_path)
            omni.kit.commands.execute(
                "RemoveReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference
            )
            omni.kit.commands.execute("AddReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference)


class AssembledRobot:
    def __init__(self, assembled_robots: AssembledBodies):
        self.assembled_robots = assembled_robots

    @property
    def base_path(self) -> str:
        """Prim path of the base body

        Returns:
            str: Prim path of the base body
        """
        return self.assembled_robots.base_path

    @property
    def attach_path(self) -> str:
        """Prim path of the floating (attach) body

        Returns:
            str: Prim path of the floating (attach) body
        """
        return self.assembled_robots.attach_path

    @property
    def fixed_joint(self) -> UsdPhysics.FixedJoint:
        """USD fixed joint linking base and floating body together

        Returns:
            UsdPhysics.FixedJoint: USD fixed joint linking base and floating body together
        """
        return self.assembled_robots.fixed_joint

    @property
    def root_joints(self) -> List[UsdPhysics.Joint]:
        """Root joints that tie the floating body to the USD stage.  These are disabled in an assembled body,
        and will be re-enabled by the disassemble() function.

        Returns:
            List[UsdPhysics.Joint]: Root joints that tie the floating body to the USD stage.
        """
        return self.assembled_robots.root_joints

    @property
    def collision_mask(self) -> Usd.Relationship:
        """A Usd Relationship masking collisions between the two assembled robots

        Returns:
            Usd.Relationship: A Usd Relationship masking collisions between the two assembled robots
        """
        return self.assembled_robots.collision_mask

    def get_fixed_joint_transform(self):
        """Get the transform between mount frames in composed robot.

        Returns:
            Tuple[np.array, np.array]: translation with shape (3,) and orientation with shape (4,)
        """
        return self.assembled_robots.get_fixed_joint_transform()

    def set_fixed_joint_transform(self, translation: np.array, orientation: np.array):
        """Set the transform between mount frames in the composed robot.

        Args:
            translation (np.array): Local translation relative to mount frame on base robot.
            orientation (np.array): Local quaternion orientation relative to mount frame on base robot.
        """
        self.assembled_robots.set_fixed_joint_transform(translation, orientation)


class AssemblyStatus(IntEnum):
    """Enum representing the current status of the robot assembly process."""

    ASSEMBLING = 0  # Assembly in progress
    DISASSEMBLING = 1  # Disassembly in progress
    IDLE = 2  # No assembly operation active


class RobotAssembler:
    """
    RobotAssembler is a class to assemble robots from a base robot and an attachment robot. It will create a new USD stage with the assembly and configure a variant selection to enable the attachment robot to be selected.
    If the variant set already exists in the source asset, it creates a new entry to it, otherwise it creates a new variant set.
    """

    def __init__(self):
        """
        Initialize the RobotAssembler

        Args:
            None

        Returns:
            None
        """

        self._timeline = omni.timeline.get_timeline_interface()
        self._status = AssemblyStatus.IDLE
        # Session layer to store temporary assembly status
        self.reset()

    def reset(self):
        if self._status == AssemblyStatus.ASSEMBLING:
            self.cancel_assembly()
        self._assembly_session = None

        # Stage where the assembly happens
        self._stage = None

        # Base robot prim path
        self._base_robot_prim = None
        self._base_mount_frame_prim = None

        # Gripper prim path
        self._attachment_robot_prim = None
        self._attachment_mount_frame_prim = None

        # Destination file where the assembly will be saved
        self._configuration_destination = None

    def is_root_joint(self, prim) -> bool:
        """Check if a prim is a root joint (has no body0 or body1 target).

        Args:
            prim (Usd.Prim): Prim to check

        Returns:
            bool: True if prim is a root joint, False otherwise
        """
        return UsdPhysics.Joint(prim) and (
            len(UsdPhysics.Joint(prim).GetBody0Rel().GetTargets()) == 0
            or len(UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()) == 0
        )

    def _set_joint_states_to_zero(self, prim_path: str):
        """Set all joint state values to zero for a prim and its children.

        Args:
            prim_path (str): Path to prim whose joint states should be zeroed
        """
        p = get_prim_at_path(prim_path)
        for prim in Usd.PrimRange(p):
            if not UsdPhysics.Joint(prim):
                continue
            if prim.HasProperty("state:angular:physics:position"):
                prim.GetProperty("state:angular:physics:position").Set(0.0)
            if prim.HasProperty("state:angular:physics:velocity"):
                prim.GetProperty("state:angular:physics:velocity").Set(0.0)

    def mask_collisions(self, prim_path_a: str, prim_path_b: str) -> Usd.Relationship:
        """Mask collisions between two prims.  All nested prims will also be included.

        Args:
            prim_path_a (str): Path to a prim
            prim_path_b (str): Path to a prim

        Returns:
            Usd.Relationship: A relationship filtering collisions between prim_path_a and prim_path_b
        """
        filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(get_prim_at_path(prim_path_a))
        rel = filteringPairsAPI.CreateFilteredPairsRel()
        rel.AddTarget(Sdf.Path(prim_path_b))
        return rel

    def __del__(self):
        self.reset()

    def begin_assembly(
        self,
        stage,
        base_prim_path,
        base_mount_path,
        attachment_prim_path,
        attachment_mount_path,
        variant_set,
        variant_name,
    ):
        """Start the robot assembly process.

        Places the attachment robot relative to the base robot but does not compose them yet.

        Args:
            stage (Usd.Stage): USD stage for assembly
            base_prim_path (str): Path to base robot prim
            base_mount_path (str): Path to mount frame on base robot
            attachment_prim_path (str): Path to attachment robot prim
            attachment_mount_path (str): Path to mount frame on attachment robot
            variant_set (str): Name of variant set to create/modify
            variant_name (str): Name of variant to create
        """
        self._stage = stage
        self._base_robot_prim = base_prim_path
        self._base_mount_frame_prim = base_mount_path
        self._attachment_robot_prim = attachment_prim_path
        self._attachment_mount_frame_prim = attachment_mount_path
        self._variant_set = variant_set
        self._variant_name = variant_name

        stage_identifier = self._stage.GetRootLayer().identifier
        self._direct_edit = False
        self._assembly_identifier = f"{self._variant_set}_{self._variant_name}.usd"
        self._local_assembly_identifier = f"configuration/{self._assembly_identifier}"
        if self._stage.GetDefaultPrim().GetPath().pathString == self._base_robot_prim:
            self._direct_edit = True
            # Editing the robot asset directly
            stage_path = "/".join(stage_identifier.split("/")[:-1])
            stage_name = stage_identifier.split("/")[-1].split(".")[0]
            self._local_assembly_identifier = f"configuration/{stage_name}_{self._assembly_identifier}"
            self._assembly_identifier = stage_path + f"/configuration/{stage_name}_{self._assembly_identifier}"

            base_prim = self._stage.GetPrimAtPath(self._base_robot_prim)
            variant_set = base_prim.GetVariantSet(self._variant_set)
            if variant_set:
                variant_set.SetVariantSelection("None")

        self._assembly_layer = start_assembly_session_sublayer(self._stage, sublayer_filename=self._assembly_identifier)

        self._stage.SetEditTarget(self._assembly_layer)
        self._status = AssemblyStatus.ASSEMBLING

        attachment_prim = self._stage.GetPrimAtPath(self._attachment_robot_prim)
        attachment_mount_frame_prim = self._stage.GetPrimAtPath(self._attachment_mount_frame_prim)

        base_prim = self._stage.GetPrimAtPath(self._base_robot_prim)
        base_mount_frame_prim = self._stage.GetPrimAtPath(self._base_mount_frame_prim)

        mount_pose = omni.usd.get_world_transform_matrix(base_mount_frame_prim)

        attachment_pose = omni.usd.get_local_transform_matrix(attachment_mount_frame_prim)

        base_mount_pose = attachment_pose.GetInverse() * mount_pose

        attachment_parent_pose = omni.usd.get_world_transform_matrix(attachment_prim.GetParent())

        attachment_pose_local = attachment_parent_pose.GetInverse() * base_mount_pose

        omni.kit.commands.execute(
            "TransformPrimCommand", path=attachment_prim.GetPath(), new_transform_matrix=attachment_pose_local
        )

    def cancel_assembly(self):
        """Cancel the current assembly operation and reset state."""
        if self._assembly_identifier:
            stop_assembly_session_sublayer(self._stage, self._assembly_identifier, save=False)
        self._status = AssemblyStatus.IDLE

        async def _end_assembly():
            await omni.kit.app.get_app().next_update_async()
            while omni.usd.get_context().get_stage_loading_status()[2] > 0:
                await omni.kit.app.get_app().next_update_async()
            self.reset()

        asyncio.ensure_future(_end_assembly())

    def assemble(self):
        """
        Composes the attachment robot onto the base robot, so that the attachment robot is a child of the base robot, and ready to simulate
        """
        attachment_prim = self._stage.GetPrimAtPath(self._attachment_robot_prim)
        attachment_mount_frame_prim = self._stage.GetPrimAtPath(self._attachment_mount_frame_prim)

        base_prim = self._stage.GetPrimAtPath(self._base_robot_prim)
        base_mount_frame_prim = self._stage.GetPrimAtPath(self._base_mount_frame_prim)

        self.assemble_rigid_bodies(
            self._base_robot_prim,
            self._attachment_robot_prim,
            self._base_mount_frame_prim,
            self._attachment_mount_frame_prim,
            mask_all_collisions=False,
        )

    def finish_assemble(self):

        if self._direct_edit:

            async def stop_sublayer():
                omni.kit.commands.execute(
                    "MovePrimSpecsToLayer",
                    src_layer_identifier=self._stage.GetRootLayer().identifier,
                    dst_layer_identifier=self._assembly_layer.identifier,
                    prim_spec_path=self._attachment_robot_prim,
                    dst_stronger_than_src=True,
                )
                await omni.kit.app.get_app().next_update_async()
                stop_assembly_session_sublayer(self._stage, self._assembly_identifier, save=True)
                await omni.kit.app.get_app().next_update_async()
                assembly_stage = Usd.Stage.Open(self._assembly_identifier)
                assembly_stage.SetDefaultPrim(assembly_stage.DefinePrim(self._base_robot_prim, "Xform"))
                assembly_stage.Save()

                base_prim = self._stage.GetPrimAtPath(self._base_robot_prim)
                variant_set = base_prim.GetVariantSet(self._variant_set)
                if not variant_set:
                    variant_set = base_prim.CreateVariantSet(self._variant_set)

                variant_set.AddVariant("None")
                variant_set.AddVariant(self._variant_name)
                variant_set.SetVariantSelection(self._variant_name)

                with variant_set.GetVariantEditContext():
                    base_prim.GetPayloads().AddPayload(self._local_assembly_identifier)
                    links_rel = base_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
                    links_rel.AddTarget(self._stage.GetPrimAtPath(self._attachment_robot_prim).GetPath())
                    joints_rel = base_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
                    joints_rel.AddTarget(self._stage.GetPrimAtPath(self._attachment_robot_prim).GetPath())

            asyncio.ensure_future(stop_sublayer())

        else:

            async def stop_sublayer():
                base_prim = self._stage.GetPrimAtPath(self._base_robot_prim)
                links_rel = base_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
                links_rel.AddTarget(self._stage.GetPrimAtPath(self._attachment_robot_prim).GetPath())
                joints_rel = base_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
                joints_rel.AddTarget(self._stage.GetPrimAtPath(self._attachment_robot_prim).GetPath())
                await omni.kit.app.get_app().next_update_async()
                omni.kit.commands.execute(
                    "MovePrimSpecsToLayer",
                    src_layer_identifier=self._assembly_layer.identifier,
                    dst_layer_identifier=self._stage.GetRootLayer().identifier,
                    prim_spec_path=self._attachment_robot_prim,
                    dst_stronger_than_src=False,
                )
                await omni.kit.app.get_app().next_update_async()
                stop_assembly_session_sublayer(self._stage, self._assembly_identifier, save=False)

            asyncio.ensure_future(stop_sublayer())
        self._status = AssemblyStatus.IDLE

    def assemble_rigid_bodies(
        self,
        base_path: str,
        attach_path: str,
        base_mount_frame: str,
        attach_mount_frame: str,
        mask_all_collisions: bool = True,
        refresh_asset_paths: bool = False,
    ) -> AssembledBodies:
        """Assemble two rigid bodies into one physical structure

        Args:
            base_robot_path (str): Path to base robot.
            attach_robot_path (str): Path to attach robot.  The attach robot will be unrooted from the stage and attached only to the base robot
            base_robot_mount_frame (str): Relative path to frame in base robot where there is the desired attach point.
            attach_robot_mount_frame (str): Relative path to frame in the attach robot where there is the desired attach point.
            mask_all_collisions (bool, optional): Mask all collisions between attach robot and base robot.  This is necessary when setting single_robot=False to prevent Physics constraint
                violations from the new fixed joint.  Advanced users may set this flag to False and use the mask_collisions() function separately for more customizable behavior.  Defaults to True.

        Returns:
            AssembledBodies: An object representing the assembled bodies. This object can detach the composed robots and edit the fixed joint transform.
        """

        # Make mount_frames if they are not specified
        if base_mount_frame == "":
            base_mount_path = base_path + "/assembler_mount_frame"
            base_mount_path = find_unique_string_name(base_mount_path, lambda x: not is_prim_path_valid(x))
            # SingleXFormPrim(base_mount_path, translation=np.array([0, 0, 0]))
        else:
            base_mount_path = base_mount_frame

        if attach_mount_frame == "":
            attach_mount_path = attach_path + "/assembler_mount_frame"
            attach_mount_path = find_unique_string_name(attach_mount_path, lambda x: not is_prim_path_valid(x))
            # SingleXFormPrim(attach_mount_path, translation=np.array([0, 0, 0]))
        else:
            attach_mount_path = attach_mount_frame

        articulation_root = get_prim_at_path(get_articulation_root_api_prim_path(attach_path))
        if self.is_root_joint(articulation_root):
            articulation_root.SetActive(False)
        else:
            articulation_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        # Find and remove any fixed joint that tie the robot to world
        root_joints = [
            p for p in Usd.PrimRange(articulation_root.GetStage().GetPrimAtPath(attach_path)) if self.is_root_joint(p)
        ]
        for root_joint in root_joints:
            root_joint.SetActive(False)

        attach_prim = get_prim_at_path(attach_path)

        # Find and Disable Fixed Joints that Tie Object B to the Stage
        root_joints = [p for p in Usd.PrimRange(attach_prim) if self.is_root_joint(p)]

        for root_joint in root_joints:
            root_joint.GetProperty("physics:jointEnabled").Set(False)

        if attach_prim.HasAttribute("physics:kinematicEnabled"):
            attach_prim.GetAttribute("physics:kinematicEnabled").Set(False)

        # Create fixed Joint between attach frames
        fixed_joint = self.create_fixed_joint(attach_mount_path, base_mount_path, attach_mount_path)

        # Make sure that Articulation B is not parsed as a part of Articulation A.
        # fixed_joint.GetExcludeFromArticulationAttr().Set(True)

        collision_mask = None
        if mask_all_collisions:
            base_path_art_root = get_articulation_root_api_prim_path(base_path)
            collision_mask = self.mask_collisions(base_path_art_root, attach_path)

        # Strange values can be written into the JointStateAPIs when nesting robots through the UI
        # in the assemble phase.  These values are supposed to be zero.
        # This can cause physics constraint violations and explosions.
        self._set_joint_states_to_zero(base_path)
        self._set_joint_states_to_zero(attach_path)

        if refresh_asset_paths:
            self._refresh_asset(base_path)
            self._refresh_asset(attach_path)

        return AssembledBodies(base_path, attach_path, fixed_joint, root_joints, articulation_root, collision_mask)

    def assemble_articulations(
        self,
        base_robot_path: str,
        attach_robot_path: str,
        base_robot_mount_frame: str,
        attach_robot_mount_frame: str,
        mask_all_collisions=True,
        single_robot=False,
        refresh_asset_paths: bool = False,
    ) -> AssembledRobot:
        """Compose two robots into one physical structure

        Args:
            base_robot_path (str): Path to base robot.
            attach_robot_path (str): Path to attach robot.  The attach robot will be unrooted from the stage and attached only to the base robot
            base_robot_mount_frame (str): Relative path to frame in base robot where there is the desired attach point.
            attach_robot_mount_frame (str): Relative path to frame in the attach robot where there is the desired attach point.
            mask_all_collisions (bool, optional): Mask all collisions between attach robot and base robot.  This is necessary when setting single_robot=False to prevent Physics constraint
                violations from the new fixed joint.  Advanced users may set this flag to False and use the mask_collisions() function separately for more customizable behavior.  Defaults to True.
            single_robot (bool, optional): If True: control the resulting composed robots as a single robot Articulation at base_robot_path.
                Setting this flag to True may resolve unstable physics behavior when teleporting the robot base.  Defaults to False.

        Returns:
            AssembledRobot: An object representing the assembled robot.  This object can detach the composed robots and edit the fixed joint transform.
        """
        assemblage = self.assemble_rigid_bodies(
            base_robot_path,
            attach_robot_path,
            base_robot_mount_frame,
            attach_robot_mount_frame,
            mask_all_collisions,
        )

        # Disable Articulation Root on Articulation B so that A is always the prim path for the composed robot
        if single_robot:
            art_b_prim = get_prim_at_path(attach_robot_path)
            if art_b_prim.HasProperty("physxArticulation:articulationEnabled"):
                art_b_prim.GetProperty("physxArticulation:articulationEnabled").Set(False)

            assemblage.fixed_joint.GetExcludeFromArticulationAttr().Set(False)

            if refresh_asset_paths:
                self._refresh_asset(base_robot_path)
                self._refresh_asset(attach_robot_path)

        return AssembledRobot(assemblage)

    def create_fixed_joint(
        self,
        prim_path: str,
        target0: str = None,
        target1: str = None,
    ) -> UsdPhysics.FixedJoint:
        """Create a fixed joint between two bodies

        Args:
            prim_path (str): Prim path at which to place new fixed joint.
            target0 (str, optional): Prim path of frame at which to attach fixed joint. Defaults to None.
            target1 (str, optional): Prim path of frame at which to attach fixed joint. Defaults to None.

        Returns:
            UsdPhysics.FixedJoint: A USD fixed joint
        """

        stage = get_current_stage()

        fixed_joint_path = prim_path + "/AssemblerFixedJoint"
        fixed_joint_path = find_unique_string_name(fixed_joint_path, lambda x: not is_prim_path_valid(x))
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)

        fixed_joint_prim = fixed_joint.GetPrim()
        fixed_joint_prim.GetRelationship("physics:body0").SetTargets([Sdf.Path(target0)])
        fixed_joint_prim.GetRelationship("physics:body1").SetTargets([Sdf.Path(target1)])

        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.XformCache()

        set_opposite_body_transform(stage, cache, fixed_joint_prim, body0base=False, fixpos=True, fixrot=True)
        set_opposite_body_transform(stage, cache, fixed_joint_prim, body0base=True, fixpos=True, fixrot=True)

        return fixed_joint

    def convert_prim_to_rigid_body(self, prim_path: str) -> None:
        """Convert a prim to a rigid body by applying the UsdPhysics.RigidBodyAPI
        Also sets physics:kinematicEnabled property to true to prevent falling from gravity without needing a fixed joint.

        Args:
            prim_path (str): Path to prim to convert.
        """
        prim_to_convert = get_prim_at_path(prim_path)
        if get_prim_object_type(prim_path) == "articulation":
            carb.log_warn("Cannot convert Articulation to Rigid Body")
            return False
        if not prim_to_convert.IsValid():
            carb.log_warn(f"No prim can be found at path {prim_path}")
            return False
        else:
            if not prim_to_convert.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(prim_to_convert)

        if prim_to_convert.HasAttribute("physics:kinematicEnabled"):
            prim_to_convert.GetAttribute("physics:kinematicEnabled").Set(True)

        return True

    def _refresh_asset(self, prim_path):
        # Refreshing payloads manually is a way to get the Articulation to update immediately while the timeline is
        # still playing.  Usd Physics should be doing this automatically, but there is currently a bug.  This function
        # will eventually become unnecessary.
        stage = get_current_stage()
        prim = get_prim_at_path(prim_path)

        composed_payloads = omni.usd.get_composed_payloads_from_prim(prim)
        if len(composed_payloads) != 0:
            payload = Sdf.Payload(prim_path)
            omni.kit.commands.execute("RemovePayload", stage=stage, prim_path=prim_path, payload=payload)
            omni.kit.commands.execute("AddPayload", stage=stage, prim_path=prim_path, payload=payload)

        composed_refs = omni.usd.get_composed_references_from_prim(prim)
        if len(composed_refs) != 0:
            reference = Sdf.Reference(prim_path)
            omni.kit.commands.execute(
                "RemoveReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference
            )
            omni.kit.commands.execute("AddReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference)

    @staticmethod
    def _move_obj_b_to_local_pos(base_mount_path, attach_path, attach_mount_path, rel_offset, rel_orient):
        # Get the position of base_mount_path as `a`
        a_trans, a_orient = SingleXFormPrim(base_mount_path).get_world_pose()

        a_rot = quats_to_rot_matrices(a_orient)
        rel_rot = quats_to_rot_matrices(rel_orient)

        # Get the desired position of attach_mount_path as `c`
        c_trans = a_rot @ rel_offset + a_trans
        c_rot = a_rot @ rel_rot
        c_quat = rot_matrices_to_quats(c_rot)

        # The attach_mount_path local xform is a free variable, and setting its world pose doesn't
        # change the world pose of every part of the Articulation.

        # We need to set the position of attach_path such that attach_mount_path ends up in the
        # desired location.  Let the attach path location be `b`.

        # t_bc denotes the translation that brings b to c
        # r_bc rotates from b to c

        t_bc, q_bc = SingleXFormPrim(attach_mount_path).get_local_pose()
        r_bc = quats_to_rot_matrices(q_bc)

        b_rot = c_rot @ r_bc.T
        b_trans = c_trans - b_rot @ t_bc
        b_orient = rot_matrices_to_quats(b_rot)

        SingleXFormPrim(attach_path).set_world_pose(b_trans, b_orient)

        # These should be roughly equal
        # print(c_trans,c_quat, SingleXFormPrim(attach_mount_path).get_world_pose())
