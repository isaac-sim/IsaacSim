# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Robot assembler module that provides functionality for assembling and managing robot components with USD physics joints."""

import asyncio
import os
from enum import IntEnum

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import omni.timeline
import usd.schema.isaac.robot_schema as robot_schema
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .session_layer_util import start_assembly_session_sublayer, stop_assembly_session_sublayer

# ------------------------------------------
# JW - this code is from omni\extensions\ux\source\omni.physx.ui\python\scripts\utils.py
AXES_INDICES = {"X": 0, "Y": 1, "Z": 2}


def set_opposite_body_transform(
    stage: object, cache: object, prim: object, body0base: bool, fixpos: bool, fixrot: bool
) -> None:
    """Set the transform of one joint body to align with the opposite body.

    This function updates the local position and rotation attributes of a joint body to match the
    relative transform calculated from the opposite body connection.

    Args:
        stage: USD stage containing the joint prim.
        cache: UsdGeom.XformCache for efficient transform computations.
        prim: Joint prim whose body transform should be updated.
        body0base: Whether to use body0 as the reference frame.
        fixpos: Whether to update the local position attributes.
        fixrot: Whether to update the local rotation attributes.
    """
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


def get_aligned_body_transform(stage: object, cache: object, joint: object, body0base: bool) -> Gf.Transform:
    """Calculate the relative transform between two bodies connected by a joint.

    This function computes the transform needed to align one body frame with another based on their world
    positions and the joint's local transformations.

    Args:
        stage: USD stage containing the joint and body prims.
        cache: UsdGeom.XformCache for efficient transform computations.
        joint: UsdPhysics.Joint connecting the two bodies.
        body0base: Whether to use body0 as the reference frame for alignment.

    Returns:
        The relative transform between the two bodies.
    """
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

    Args:
        base_path: Prim path of the base body.
        attach_path: Prim path of the attach body.
        fixed_joint: Fixed joint connecting the bodies.
        root_joints: Root joints of the attach body.
        attach_body_articulation_root: Articulation root of attach body.
        collision_mask: Collision mask between bodies.
    """

    def __init__(
        self,
        base_path: str,
        attach_path: str,
        fixed_joint: UsdPhysics.FixedJoint,
        root_joints: list[UsdPhysics.Joint],
        attach_body_articulation_root: Usd.Prim,
        collision_mask: object = None,
    ) -> None:
        self._base_path = base_path
        self._attach_path = attach_path
        self._fixed_joint = fixed_joint

        self._root_joints = root_joints

        self._is_assembled = True
        self._collision_mask = collision_mask
        self._articulation_root = attach_body_articulation_root

    @property
    def base_path(self) -> str:
        """Prim path of the base body.

        Returns:
            Prim path of the base body.
        """
        return self._base_path

    @property
    def attach_path(self) -> str:
        """Prim path of the floating (attach) body.

        Returns:
            Prim path of the floating (attach) body.
        """
        return self._attach_path

    @property
    def fixed_joint(self) -> UsdPhysics.FixedJoint:
        """USD fixed joint linking base and floating body together.

        Returns:
            USD fixed joint linking base and floating body together.
        """
        return self._fixed_joint

    @property
    def attach_body_articulation_root(self) -> Usd.Prim:
        """USD articulation root of the floating body.

        Returns:
            USD articulation root of the floating body.
        """
        return self._articulation_root

    @property
    def root_joints(self) -> list[UsdPhysics.Joint]:
        """Root joints that tie the floating body to the USD stage. These are disabled in an assembled body,.

            and will be re-enabled by the disassemble() function.

        Returns:
            Root joints that tie the floating body to the USD stage.
        """
        return self._root_joints

    @property
    def collision_mask(self) -> Usd.Relationship:
        """A Usd Relationship masking collisions between the two assembled bodies.

        Returns:
            A Usd Relationship masking collisions between the two assembled bodies.
        """
        return self._collision_mask

    def _unmask_collisions(self) -> None:
        """Remove collision masking between the assembled bodies."""
        if self._collision_mask is not None:
            [self._collision_mask.RemoveTarget(target) for target in self._collision_mask.GetTargets()]
            self._collision_mask = None

    def _refresh_asset(self, prim_path: str) -> None:
        """Refresh payloads and references to update articulation immediately during timeline playback.

        Args:
            prim_path: Path to the prim whose payloads and references should be refreshed.
        """
        # Refreshing payloads manually is a way to get the Articulation to update immediately while the timeline is
        # still playing.  Usd Physics should be doing this automatically, but there is currently a bug.  This function
        # will eventually become unnecessary.
        stage = stage_utils.get_current_stage()
        prim = prim_utils.get_prim_at_path(prim_path)

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
    """A wrapper class providing convenient access to assembled robot body information.

    This class serves as a high-level interface for interacting with assembled robots, encapsulating
    the underlying AssembledBodies instance and exposing its key properties through a simplified API.
    It provides direct access to robot paths, joint information, and collision masking relationships
    for assembled robot configurations.

    The class acts as a facade over AssembledBodies, making it easier to work with assembled robot
    data without needing to interact directly with the lower-level assembly implementation.

    Args:
        assembled_robots: The AssembledBodies instance containing the robot assembly data.
    """

    def __init__(self, assembled_robots: AssembledBodies) -> None:
        self.assembled_robots = assembled_robots

    @property
    def base_path(self) -> str:
        """Prim path of the base body.

        Returns:
            Prim path of the base body.
        """
        return self.assembled_robots.base_path

    @property
    def attach_path(self) -> str:
        """Prim path of the floating (attach) body.

        Returns:
            Prim path of the floating (attach) body.
        """
        return self.assembled_robots.attach_path

    @property
    def fixed_joint(self) -> UsdPhysics.FixedJoint:
        """USD fixed joint linking base and floating body together.

        Returns:
            USD fixed joint linking base and floating body together.
        """
        return self.assembled_robots.fixed_joint

    @property
    def root_joints(self) -> list[UsdPhysics.Joint]:
        """Root joints that tie the floating body to the USD stage.  These are disabled in an assembled body,.

        and will be re-enabled by the disassemble() function.

        Returns:
            Root joints that tie the floating body to the USD stage.
        """
        return self.assembled_robots.root_joints

    @property
    def collision_mask(self) -> Usd.Relationship:
        """A Usd Relationship masking collisions between the two assembled robots.

        Returns:
            A Usd Relationship masking collisions between the two assembled robots.
        """
        return self.assembled_robots.collision_mask


class AssemblyStatus(IntEnum):
    """Enum representing the current status of the robot assembly process."""

    ASSEMBLING = 0  # Assembly in progress
    """Assembly operation is currently in progress."""
    DISASSEMBLING = 1  # Disassembly in progress
    """Disassembly operation is currently in progress."""
    IDLE = 2  # No assembly operation active
    """No assembly operation is currently active."""


class RobotAssembler:
    """RobotAssembler is a class to assemble robots from a base robot and an attachment robot. It will create a new USD stage with the assembly and configure a variant selection to enable the attachment robot to be selected.

    If the variant set already exists in the source asset, it creates a new entry to it, otherwise it creates a new variant set.
    """

    def __init__(self) -> None:

        self._timeline = omni.timeline.get_timeline_interface()
        # ``_status`` must be initialized BEFORE ``reset()`` so the
        # cancel-if-assembling guard in ``reset()`` has a defined value to read.
        self._status = AssemblyStatus.IDLE
        self.reset()

    def reset(self) -> None:
        """Reset the assembler to its initial state.

        Performs three responsibilities:

        1. If an assembly is in progress (``self._status == AssemblyStatus.ASSEMBLING``)
           cancels it via :meth:`cancel_assembly`.
        2. Clears the references to the active stage and to the base/attachment
           robot and mount prim paths.
        3. Initializes the assembly-sublayer bookkeeping fields
           (``_assembly_identifier``, ``_local_assembly_identifier``,
           ``_assembly_layer``, ``_direct_edit``, ``_variant_set``,
           ``_variant_name``) to safe defaults so that :meth:`cancel_assembly`
           remains valid to call on a fresh or already-reset instance.
        """
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

        # Assembly sublayer state. Initialized here (in addition to ``begin_assembly``)
        # so that ``cancel_assembly`` is safe to call on an idle/fresh instance and
        # after teardown, instead of raising AttributeError on the guard clause.
        self._assembly_identifier = None
        self._local_assembly_identifier = None
        self._assembly_layer = None
        self._direct_edit = False
        self._variant_set = None
        self._variant_name = None

    def is_root_joint(self, prim: object) -> bool:
        """Check if a prim is a root joint (has no body0 or body1 target).

        Args:
            prim: Prim to check

        Returns:
            True if prim is a root joint, False otherwise
        """
        return UsdPhysics.Joint(prim) and (
            len(UsdPhysics.Joint(prim).GetBody0Rel().GetTargets()) == 0
            or len(UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()) == 0
        )

    def get_articulation_root_api_path(self, prim_path: str) -> str:
        """Get the prim path that has the Articulation Root API applied.

        Args:
            prim_path: Path to a prim

        Returns:
            Path to the prim that has the Articulation Root API applied
        """
        predicate = lambda prim, path: prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        prim = prim_utils.get_first_matching_child_prim(prim_path, predicate=predicate, include_self=True)
        return prim_utils.get_prim_path(prim) if prim is not None else ""

    def _set_joint_states_to_zero(self, prim_path: str) -> None:
        """Set all joint state values to zero for a prim and its children.

        Args:
            prim_path: Path to prim whose joint states should be zeroed
        """
        p = prim_utils.get_prim_at_path(prim_path)
        for prim in Usd.PrimRange(p):
            if not UsdPhysics.Joint(prim):
                continue
            if prim.HasProperty("state:angular:physics:position"):
                prim.GetProperty("state:angular:physics:position").Set(0.0)
            if prim.HasProperty("state:angular:physics:velocity"):
                prim.GetProperty("state:angular:physics:velocity").Set(0.0)

    def mask_collisions(self, prim_path_a: str, prim_path_b: str) -> Usd.Relationship:
        """Mask collisions between two prims. All nested prims will also be included.

        Args:
            prim_path_a: Path to a prim
            prim_path_b: Path to a prim

        Returns:
            A relationship filtering collisions between prim_path_a and prim_path_b
        """
        filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(prim_utils.get_prim_at_path(prim_path_a))
        rel = filteringPairsAPI.CreateFilteredPairsRel()
        rel.AddTarget(Sdf.Path(prim_path_b))
        return rel

    def __del__(self) -> None:
        """Best-effort cleanup when the instance is garbage collected.

        Deliberately does not call :meth:`reset` / :meth:`cancel_assembly` here:
        those paths touch USD (``stop_assembly_session_sublayer``) and schedule
        coroutines on the Kit event loop. During interpreter shutdown the
        ``omni.usd`` context and the asyncio loop may already be torn down, so
        executing them from a finalizer can crash or raise spurious exceptions.
        Callers that need to terminate an in-flight assembly must call
        :meth:`reset` explicitly while the application is still alive.
        """
        try:
            self._status = AssemblyStatus.IDLE
            self._assembly_session = None
            self._stage = None
            self._base_robot_prim = None
            self._base_mount_frame_prim = None
            self._attachment_robot_prim = None
            self._attachment_mount_frame_prim = None
            self._configuration_destination = None
        except Exception:
            # Finalizers must never raise; swallow any teardown-order failure.
            pass

    def begin_assembly(
        self,
        stage: object,
        base_prim_path: str,
        base_mount_path: str,
        attachment_prim_path: str,
        attachment_mount_path: str,
        variant_set: str,
        variant_name: str,
    ) -> None:
        """Start the robot assembly process.

        Places the attachment robot relative to the base robot but does not compose them yet.

        Args:
            stage: USD stage for assembly
            base_prim_path: Path to base robot prim
            base_mount_path: Path to mount frame on base robot
            attachment_prim_path: Path to attachment robot prim
            attachment_mount_path: Path to mount frame on attachment robot
            variant_set: Name of variant set to create/modify
            variant_name: Name of variant to create
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
        self._assembly_identifier = f"{self._variant_name}.usd"
        self._local_assembly_identifier = f"payloads/{self._variant_set}/{self._assembly_identifier}"
        if self._stage.GetDefaultPrim().GetPath().pathString == self._base_robot_prim:
            self._direct_edit = True
            # Editing the robot asset directly. Resolve the payload to an absolute path
            # alongside the robot asset, under ``payloads/<variant_set>/<variant_name>.usd``.
            stage_path = "/".join(stage_identifier.split("/")[:-1])
            self._assembly_identifier = f"{stage_path}/{self._local_assembly_identifier}"
            os.makedirs(os.path.dirname(self._assembly_identifier), exist_ok=True)

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

        attachment_pose_local = base_mount_pose * attachment_parent_pose.GetInverse()

        omni.kit.commands.execute(
            "TransformPrimCommand", path=attachment_prim.GetPath(), new_transform_matrix=attachment_pose_local
        )

    def cancel_assembly(self) -> None:
        """Cancel the current assembly operation and reset state."""
        if self._assembly_identifier:
            stop_assembly_session_sublayer(self._stage, self._assembly_identifier, save=False)
        self._status = AssemblyStatus.IDLE

        async def _end_assembly() -> None:
            await omni.kit.app.get_app().next_update_async()
            while omni.usd.get_context().get_stage_loading_status()[2] > 0:
                await omni.kit.app.get_app().next_update_async()
            self.reset()

        asyncio.ensure_future(_end_assembly())

    def assemble(self) -> None:
        """Composes the attachment robot onto the base robot, so that the attachment robot is a child of the base robot, and ready to simulate."""
        attachment_prim = self._stage.GetPrimAtPath(self._attachment_robot_prim)
        attachment_mount_frame_prim = self._stage.GetPrimAtPath(self._attachment_mount_frame_prim)

        base_prim = self._stage.GetPrimAtPath(self._base_robot_prim)
        base_mount_frame_prim = self._stage.GetPrimAtPath(self._base_mount_frame_prim)

        self._assemble_rigid_bodies(
            self._base_robot_prim,
            self._attachment_robot_prim,
            self._base_mount_frame_prim,
            self._attachment_mount_frame_prim,
            mask_all_collisions=False,
        )

    def finish_assemble(self) -> None:
        """Finalize the assembly process by configuring variant sets and saving the assembly to a USD file."""
        if self._direct_edit:

            async def stop_sublayer() -> None:
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

            async def stop_sublayer() -> None:
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

    def _assemble_rigid_bodies(
        self,
        base_path: str,
        attach_path: str,
        base_mount_frame: str,
        attach_mount_frame: str,
        mask_all_collisions: bool = True,
        refresh_asset_paths: bool = False,
    ) -> AssembledBodies:
        """Assemble two rigid bodies into one physical structure.

        Args:
            base_path: Path to base robot.
            attach_path: Path to attach robot. The attach robot will be unrooted from the stage and attached only
                to the base robot.
            base_mount_frame: Relative path to frame in base robot where there is the desired attach point.
            attach_mount_frame: Relative path to frame in the attach robot where there is the desired attach
                point.
            mask_all_collisions: Mask all collisions between attach robot and base robot. This is necessary
                when setting single_robot=False to prevent Physics constraint violations from the new fixed joint.
                Advanced users may set this flag to False and use the mask_collisions() function separately for
                more customizable behavior.
            refresh_asset_paths: Whether to refresh asset paths after assembly.

        Returns:
            An object representing the assembled bodies. This object can detach the composed robots and edit
            the fixed joint transform.
        """
        # Make mount_frames if they are not specified
        if base_mount_frame == "":
            base_mount_path = base_path + "/assembler_mount_frame"
            base_mount_path = stage_utils.generate_next_free_path(base_mount_path)
            # SingleXFormPrim(base_mount_path, translation=np.array([0, 0, 0]))
        else:
            base_mount_path = base_mount_frame

        if attach_mount_frame == "":
            attach_mount_path = attach_path + "/assembler_mount_frame"
            attach_mount_path = stage_utils.generate_next_free_path(attach_mount_path)
            # SingleXFormPrim(attach_mount_path, translation=np.array([0, 0, 0]))
        else:
            attach_mount_path = attach_mount_frame

        articulation_root = prim_utils.get_prim_at_path(self.get_articulation_root_api_path(attach_path))
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

        attach_prim = prim_utils.get_prim_at_path(attach_path)

        # Find and Disable Fixed Joints that Tie Object B to the Stage
        root_joints = [p for p in Usd.PrimRange(attach_prim) if self.is_root_joint(p)]

        for root_joint in root_joints:
            root_joint.GetProperty("physics:jointEnabled").Set(False)

        if attach_prim.HasAttribute("physics:kinematicEnabled"):
            attach_prim.GetAttribute("physics:kinematicEnabled").Set(False)

        # Create fixed Joint between attach frames
        fixed_joint = self._create_fixed_joint(attach_mount_path, base_mount_path, attach_mount_path)

        # Make sure that Articulation B is not parsed as a part of Articulation A.
        # fixed_joint.GetExcludeFromArticulationAttr().Set(True)

        collision_mask = None
        if mask_all_collisions:
            base_path_art_root = self.get_articulation_root_api_path(base_path)
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

    def _create_fixed_joint(
        self,
        prim_path: str,
        target0: str = None,
        target1: str = None,
    ) -> UsdPhysics.FixedJoint:
        """Create a fixed joint between two bodies.

        Args:
            prim_path: Prim path at which to place new fixed joint.
            target0: Prim path of frame at which to attach fixed joint.
            target1: Prim path of frame at which to attach fixed joint.

        Returns:
            A USD fixed joint
        """
        stage = stage_utils.get_current_stage()

        fixed_joint_path = prim_path + "/AssemblerFixedJoint"
        fixed_joint_path = stage_utils.generate_next_free_path(fixed_joint_path)
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)

        fixed_joint_prim = fixed_joint.GetPrim()
        fixed_joint_prim.GetRelationship("physics:body0").SetTargets([Sdf.Path(target0)])
        fixed_joint_prim.GetRelationship("physics:body1").SetTargets([Sdf.Path(target1)])

        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.XformCache()

        set_opposite_body_transform(stage, cache, fixed_joint_prim, body0base=False, fixpos=True, fixrot=True)
        set_opposite_body_transform(stage, cache, fixed_joint_prim, body0base=True, fixpos=True, fixrot=True)

        return fixed_joint

    def _refresh_asset(self, prim_path: str) -> None:
        """Refresh asset payloads and references to update the articulation immediately.

        This manually refreshes payloads to get the articulation to update while the timeline is still
        playing. This function will eventually become unnecessary when USD Physics fixes the automatic
        update bug.

        Args:
            prim_path: Path to the prim whose asset should be refreshed.
        """
        # Refreshing payloads manually is a way to get the Articulation to update immediately while the timeline is
        # still playing.  Usd Physics should be doing this automatically, but there is currently a bug.  This function
        # will eventually become unnecessary.
        stage = stage_utils.get_current_stage()
        prim = prim_utils.get_prim_at_path(prim_path)

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
