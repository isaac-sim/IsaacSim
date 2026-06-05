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

"""Backend of "applying hierarchy".

The two windows in robot hierarchy page ultimates provides a lookup table of old paths and new paths.
when applying hierarchy, we look up the new path for the old path move that prim and its children to the new path.
This way it keeps all the attributes and relationships and APIs that already defined on the prim (as oppose to copying, where you have to redefine them if anything is broken)

anything that's not moved will stay where they are.

It also applys robot schemas
"""

import os
import shutil

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from ..utils.utils import can_create_dir
from .collider_helper import remove_collider
from .robot_templates import RobotRegistry

MESH_TYPES = ["Mesh", "Cube", "Sphere", "Cylinder", "Cone", "Capsule"]
MATERIAL_TYPES = ["Material", "Shader"]
JOINT_PRIM_TYPES = [
    "PhysicsJoint",
    "PhysicsPrismaticJoint",
    "PhysicsRevoluteJoint",
    "PhysicsFixedJoint",
    "PhysicsSphericalJoint",
    "PhysicsD6Joint",
]


def apply_hierarchy(lookup_table: dict, reference_mesh: dict, delete_prim_paths: list) -> None:
    """Apply new robot hierarchy structure using lookup table mapping.

    Lookup_table is a dictionary of old paths and new paths. This function reorganizes the robot structure by moving prims according to the lookup table, processes meshes, applies transforms, and creates physics variants.

    Args:
        lookup_table: Dictionary mapping old prim paths to new prim paths.
        reference_mesh: Dictionary mapping link paths to reference mesh prim paths.
        delete_prim_paths: List of prim paths to be deleted during cleanup.
    """
    # get it from registry of where to save the original stage, and where they want to save the new file,

    robot = RobotRegistry().get()
    robot_name = robot.name
    config_dir = os.path.join(robot.robot_root_folder, "configurations")
    base_layer_path = robot.base_file_path
    physics_layer_path = robot.physics_file_path
    original_stage_path = robot.original_stage_path

    # a stage has to be opened already, save it to the original stage path if the user indicated so on the previous page, and have write access to
    if robot.save_stage_original and can_create_dir(os.path.dirname(original_stage_path)):
        omni.usd.get_context().save_as_stage(original_stage_path)
        copy_from_path = original_stage_path
    else:
        if can_create_dir(robot.robot_root_folder):
            stage_copy_path = os.path.join(robot.robot_root_folder, "stage_copy.usd")
            omni.usd.get_context().save_as_stage(stage_copy_path)
            copy_from_path = stage_copy_path
            # a copy of the stage is always saved in the robot root folder during the wizard configuration. If the user chose not to save a copy of the stage, this file will be deleted at the end of the wizard
        else:
            raise Exception(f"Cannot write to {robot.robot_root_folder}, please check write access")

    # make sure the directory exists, if not, create it
    os.makedirs(os.path.dirname(base_layer_path), exist_ok=True)

    # copy the file to the new path, open the new stage in memory
    shutil.copy(copy_from_path, base_layer_path)
    omni.usd.get_context().open_stage(base_layer_path)

    robot_stage = omni.usd.get_context().get_stage()

    # create the scaffolding to rearrange the robot
    scaffolding(robot_stage)

    # use the lookup table to move prims into the new structure
    for old_path, new_path in lookup_table.items():
        prim = robot_stage.GetPrimAtPath(old_path)  ## get the prim at the old path to check the attributes
        if prim:
            move_prim_by_type(prim, old_path, new_path)

    # If any prims are left over and wasn't in the lookup table, move them directly under the new robot prim so we don't lose anything
    left_over_prims = robot_stage.GetPrimAtPath(robot.parent_prim_path)
    left_over_robot_child = left_over_prims.GetChildren()
    if left_over_robot_child:
        for prim in left_over_robot_child:
            # new path is the same as old path, except the first part is robot name instead of the old prim name (replace the old prim name with the robot name)
            old_path = prim.GetPath().pathString
            # replace whatever is between the first and second slash with the robot name
            new_path = old_path.replace(old_path.split("/")[1], robot_name)
            if new_path != old_path:
                try:
                    omni.kit.commands.execute(
                        "MovePrimCommand",
                        path_from=old_path,
                        path_to=new_path,
                        destructive=True,
                    )
                except Exception as e:
                    print(f"Error moving prim {old_path} to {new_path}: {e}")

    # second pass to process meshes
    robot_prim = robot_stage.GetPrimAtPath(f"/{robot_name}")
    process_mesh(robot_stage, robot_prim, reference_mesh)

    # # # clean up the transforms, so that scales and relative transforms (to the link) are in the meshes folder, and any link-level translation and rotation happens on the robot link level.
    cleanup_xforms(delete_prim_paths)

    # save the current stage as the base usd
    stage = omni.usd.get_context().get_stage()
    stage.SetDefaultPrim(stage.GetPrimAtPath(f"/{robot_name}"))
    omni.usd.get_context().save_as_stage(os.path.join(config_dir, base_layer_path))

    # # # apply the physics layer
    create_physics_variant()


def scaffolding(robot_stage: object) -> None:
    """Creates the scaffolding structure for the robot reorganization.

    Sets up the necessary scope hierarchies including Looks, Joints, meshes, visuals, colliders folders
    and creates link-specific xform prims under each scope.

    Args:
        robot_stage: The USD stage containing the robot to scaffold.

    """
    robot = RobotRegistry().get()

    if not robot_stage or not robot:
        return

    robot_name = robot.name
    link_names = robot.links
    # new prim for the robot if robot name changed
    robot_prim = robot_stage.GetPrimAtPath(f"/{robot_name}")
    if not robot_prim:
        robot_prim = robot_stage.DefinePrim(f"/{robot_name}", "Xform")

    # create scopes(folders) for visual, collider, meshes, and joints
    if not robot_stage.GetPrimAtPath(f"/{robot_name}/Looks"):
        looks_scope = robot_stage.DefinePrim(f"/{robot_name}/Looks", "Scope")
    if not robot_stage.GetPrimAtPath(f"/{robot_name}/Joints"):
        joints_scope = robot_stage.DefinePrim(f"/{robot_name}/Joints", "Scope")
    if not robot_stage.GetPrimAtPath("/meshes"):
        meshes_scope = robot_stage.DefinePrim("/meshes", "Scope")  # contains the original meshes
    if not robot_stage.GetPrimAtPath("/visuals"):
        visuals_scope = robot_stage.DefinePrim("/visuals", "Scope")  # contains references to the meshes folder
    if not robot_stage.GetPrimAtPath("/colliders"):
        colliders_scope = robot_stage.DefinePrim("/colliders", "Scope")  # contains references to the meshes folder

    ### under meshes, collider, and visual scope, create all prims for all the links
    for link_name in link_names:  # may also need to add a check for if they already exist
        UsdGeom.Xform.Define(robot_stage, f"/meshes/{link_name}")
        UsdGeom.Xform.Define(robot_stage, f"/colliders/{link_name}")
        UsdGeom.Xform.Define(robot_stage, f"/visuals/{link_name}")
        UsdGeom.Xform.Define(robot_stage, f"/{robot_name}/{link_name}")
        UsdGeom.Xform.Define(robot_stage, f"/{robot_name}/{link_name}/visual")
        # no colliders yet, that's added later in the physics layer


def move_prim_by_type(prim: object, old_path: str, new_path: str) -> None:
    """Move prim to appropriate location based on its type.

    Recursively processes prim and its children, moving materials to Looks folder, joints to Joints folder, and other prims to the specified new path.

    Args:
        prim: The USD prim to move.
        old_path: Current path of the prim.
        new_path: Target path for the prim.
    """
    prim_name = new_path.split("/")[3:][0]
    ## everything after the link name gets kept (assuming format of robot_name/link_name/stuff)
    if prim.GetChildren():
        for child in prim.GetChildren():
            move_prim_by_type(child, old_path, new_path)

    prim_type = prim.GetTypeName()
    if prim_type in MATERIAL_TYPES:
        """
        if the prim is a material, move it to the Looks folder
        TODO: need to somehow differentiate physics materials from visual materials
        """
        path_to = f"/Looks/{prim_name}"
        try:
            omni.kit.commands.execute(
                "MovePrimCommand",
                path_from=old_path,
                path_to=path_to,
                destructive=True,
            )
        except Exception as e:
            print(f"Error moving prim {old_path} to {path_to}: {e}")

    elif prim_type in JOINT_PRIM_TYPES:
        """
        if the prim is a joint, move it to the Joints folder
        """
        path_to = f"/Joints/{prim_name}"
        try:
            omni.kit.commands.execute(
                "MovePrimCommand",
                path_from=old_path,
                path_to=path_to,
                destructive=True,
            )
        except Exception as e:
            print(f"Error moving prim {old_path} to {path_to}: {e}")

    else:
        ## move everything else as it is to new path,
        path_to = new_path
        try:
            omni.kit.commands.execute(
                "MovePrimCommand",
                path_from=old_path,
                path_to=new_path,
                destructive=True,
            )
        except Exception as e:
            print(f"Error moving prim {old_path} to {new_path}: {e}")


def process_mesh(stage: object, robot_prim: object, reference_mesh: dict) -> None:
    """Process and reorganize robot meshes into the mesh folder structure.

    Before the meshes are moved to the meshe folder, it first has to transform based on the link it's under. steps are:
    0. move all the meshes to under the robot first so that all the meshes associated with the link are under the same prim
    1. find the reference mesh for each link
    2. apply the transform to the link, and mesh
    3. move the mesh to the meshes folder
    4. go through each link of the meshes, check if the mesh already has rigid body and/or collision API, if yes, strip them and create a copy without the physics, link that to the visual links

    Args:
        stage: The USD stage containing the robot.
        robot_prim: The root robot prim.
        reference_mesh: Dictionary mapping link paths to reference mesh prim paths.

    """
    robot = RobotRegistry().get()

    if not stage or not robot or not robot_prim:
        return

    link_names = robot.links
    robot_name = robot.name

    # Step 1:
    for link_name in link_names:
        link_path = f"/{robot_name}/{link_name}"
        link_prim = stage.GetPrimAtPath(link_path)
        reference_mesh_local_prim = None
        # check reference mesh for reference link.
        if link_path in reference_mesh:
            reference_mesh_local_prim_path = reference_mesh[link_path]
            reference_mesh_local_prim = stage.GetPrimAtPath(reference_mesh_local_prim_path)
        else:
            # if no reference mesh is found, use the first mesh under the link
            all_children = stage.GetPrimAtPath(link_path).GetChildren()
            for child in all_children:
                if child.GetTypeName() in MESH_TYPES:
                    reference_mesh_local_prim = child
                    break
        ## Step 2:
        ## align the link xform to the reference mesh xform, (also relocating all the children of the link relative to the reference mesh origin)
        if reference_mesh_local_prim:
            reference_mesh_local_path = reference_mesh_local_prim.GetPath().pathString
            relocate_parent_to_child_origin(link_path, reference_mesh_local_path)
        else:
            print(f"no reference mesh found for {link_name}, keeping the link at its original position")

    # Step 3:
    # moving the original meshes (currently under robot prim) to the meshes folder
    for link_name in link_names:
        link_path = f"/{robot_name}/{link_name}"
        link_prim = stage.GetPrimAtPath(link_path)
        for child in link_prim.GetChildren():
            if child.GetTypeName() in MESH_TYPES:
                child_path = child.GetPath().pathString
                child_name = child_path.split("/")[3:][0]  ## keep everything after "/robot/link/"
                path_to = f"/meshes/{link_name}/{child_name}"
                try:
                    omni.kit.commands.execute(
                        "MovePrimCommand",
                        path_from=child_path,
                        path_to=path_to,
                        destructive=True,
                        keep_world_transform=False,
                    )  # must keep the local transform, otherwise the transform won't be relative to the new link origin
                except Exception as e:
                    print(f"Error moving prim {child_path} to /meshes/{link_name}/{child_name}: {e}")

                # step 4:
                # if move successful, create a reference to the mesh in the visual and collider xforms
                ref_mesh_path = f"/meshes/{link_name}"
                mesh_prim = stage.GetPrimAtPath(ref_mesh_path)
                # make sure there's no physics properties on the mesh
                recursive_physics_removal(mesh_prim)

                visual_xform = stage.GetPrimAtPath(f"/visuals/{link_name}")
                visual_xform.GetReferences().AddInternalReference(ref_mesh_path)
                # also create the reference to the robot's visual xforms
                robot_visual_xform = stage.GetPrimAtPath(f"/{robot_name}/{link_name}/visual")
                robot_visual_xform.GetReferences().AddInternalReference(visual_xform.GetPath())
                robot_visual_xform.SetInstanceable(True)


def recursive_physics_removal(prim: object) -> None:
    """Remove the api from the prim.

    Recursively removes physics APIs (RigidBodyAPI and CollisionAPI) from the prim and all its children to ensure clean visual mesh references.

    Args:
        prim: The USD prim to process for physics API removal.
    """
    for child in prim.GetChildren():
        if child.HasAPI(UsdPhysics.RigidBodyAPI):
            child.RemoveAPI(UsdPhysics.RigidBodyAPI)
            # child.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
        if child.HasAPI(UsdPhysics.CollisionAPI):
            remove_collider(child)
        recursive_physics_removal(child)


def relocate_parent_to_child_origin(parent_prim_path: str, reference_child_prim_path: str) -> bool | None:
    """Moves the parent's transform to the reference child's origin and updates all children's.

    transforms to remain in their current global positions.

    Args:
        parent_prim_path: Path to the parent prim whose transform will be moved.
        reference_child_prim_path: Path to the child prim that will serve as the new reference frame.

    Returns:
        False if either the parent or reference prim is invalid, None on success.
    """
    stage = omni.usd.get_context().get_stage()
    parent_prim = stage.GetPrimAtPath(parent_prim_path)
    reference_prim = stage.GetPrimAtPath(reference_child_prim_path)
    if not parent_prim.IsValid() or not reference_prim.IsValid():
        return False

    # Get the global transform of the reference child
    ref_xform = UsdGeom.Xformable(reference_prim)
    ref_world_full = ref_xform.ComputeLocalToWorldTransform(0)
    _, _, _, ref_rot_mat, ref_translation, _ = ref_world_full.Factor()
    ref_world_transform_no_scale = Gf.Matrix4d(ref_rot_mat.ExtractRotationMatrix(), ref_translation)

    # go through all the children of this parent and update their transforms relative to the new parent origin
    immediate_children_list = parent_prim.GetAllChildren()
    for child in immediate_children_list:
        if child.IsValid():
            child_xform = UsdGeom.Xformable(child)
            child_global_transform = child_xform.ComputeLocalToWorldTransform(0)
            _, _, child_scale, child_rot_mat, child_translation, _ = child_global_transform.Factor()
            child_global_transform_mat = Gf.Matrix4d(child_rot_mat.ExtractRotationMatrix(), child_translation)

            # Calculate the new child transform relative to the new parent transform
            new_child_rel_xform = child_global_transform_mat * ref_world_transform_no_scale.GetInverse()
            # Update the child's transform
            update_xforms(child_xform, new_child_rel_xform, child_scale)

    # # update the transforms of the parent last
    update_xforms(parent_prim, ref_world_transform_no_scale)


def update_xforms(prim: object, new_xform: object, scale: tuple = (1, 1, 1)) -> None:
    """Updates the transform operations of a prim with new translation, rotation, and scale values.

    Sets the xformOp:translate, xformOp:orient, and xformOp:scale attributes on the prim,
    creating them if they don't exist.

    Args:
        prim: The prim or UsdGeom.Xformable to update.
        new_xform: The new transform matrix containing translation and rotation.
        scale: The scale values to apply to the prim.
    """
    xformable = UsdGeom.Xformable(prim)
    prim = xformable.GetPrim()
    ## add the new one
    _, _, _, rot_mat, translation, _ = new_xform.Factor()
    rot_quat = rot_mat.ExtractRotationQuat()

    # specifically add the translation, orientation, and scale, (as oppose to the matrix) for better property visualization in the property panel
    translate_attr = prim.GetAttribute("xformOp:translate")
    if translate_attr and translate_attr.IsValid():
        translate_attr.Set(translation)
    else:
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(translation)

    orient_attr = prim.GetAttribute("xformOp:orient")
    if orient_attr and orient_attr.IsValid():
        orient_attr.Set(rot_quat)
    else:
        orientOp = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        orientOp.Set(rot_quat)

    scale_attr = prim.GetAttribute("xformOp:scale")
    if scale_attr and scale_attr.IsValid():
        scale_attr.Set(scale)
    else:
        scaleOp = xformable.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble)
        scaleOp.Set(scale)


def cleanup_xforms(delete_prim_paths: list) -> None:
    """Apply transform cleanup rules to robot structure.

    Xform rules:
    - no xforms on the links in the meshes folder, keep the relative transformd within each link for the meshes(i.e. the foundational mesh for each link should be at the origin, not necesarily each piece that makes up that link)
    - default xforms on link/collider references
    - xform translation and rotation on /robot/link stays (these should be already post-reference-aligned)
    - delete the prims in delete_prim_paths (should all be under /robot)

    Args:
        delete_prim_paths: List of prim paths to delete during cleanup.

    Returns:
        None if the robot or stage is invalid.
    """
    robot = RobotRegistry().get()
    stage = omni.usd.get_context().get_stage()
    if not robot or not stage:
        return

    link_names = robot.links
    robot_name = robot.name

    # remove xformOp attributes from link level prims in meshes folder (but should keep object level xforms)
    mesh_scope_prim = stage.GetPrimAtPath("/meshes")
    for mesh_link in mesh_scope_prim.GetChildren():
        for attr in mesh_link.GetAttributes():
            if attr.GetName().startswith("xformOp:"):
                mesh_link.RemoveProperty(attr.GetName())

    # for all the robot/link, reset scale to 1,1,1, keep other xformOp attributes
    for link_name in link_names:
        link_prim = stage.GetPrimAtPath(f"/{robot_name}/{link_name}")
        recursive_reset_transforms(link_prim, exclude_list=["xformOp:translate", "xformOp:orient", "xformOp:pivot"])

    # remove the xformOp attributes from the robot/link/visual
    for link_name in link_names:
        link_visual_prim = stage.GetPrimAtPath(f"/{robot_name}/{link_name}/visual")
        for attr in link_visual_prim.GetAttributes():
            if attr.GetName().startswith("xformOp:"):
                link_visual_prim.RemoveProperty(attr.GetName())

    # delete the prims in delete_prim_paths
    for prim_path in list(set(delete_prim_paths)):
        # need to check if the prims marked to delete is a link in the new stage, don't delete it if it is, cause then you'd be deleting a new link. otherwise it's a leftover, delete it (including all its children)
        if prim_path.split("/")[2] not in link_names:
            stage.RemovePrim(prim_path)


def recursive_reset_transforms(prim: object, exclude_list: list | None = None) -> None:
    """Reset the transforms of the prim and all its children.

    For xform:translate, default is 0,0,0 xform:orient, default is 0,0,0,1 xform:scale, default is 1,1,1 xform:pivot, default is 0,0,0.

    Args:
        prim: The USD prim to reset transforms for.
        exclude_list: List of transform attribute names to exclude from reset.
    """
    if exclude_list is None:
        exclude_list = []
    for child in prim.GetChildren():
        for attr in child.GetAttributes():
            if attr.GetName() not in exclude_list:

                if attr.GetName() == "xformOp:translate":
                    attr.Set(Gf.Vec3d(0, 0, 0))
                elif attr.GetName() == "xformOp:orient":
                    attr.Set(Gf.Quatd(1, 0, 0, 0))
                elif attr.GetName() == "xformOp:scale":
                    attr.Set(Gf.Vec3d(1, 1, 1))
                elif attr.GetName() == "xformOp:pivot":
                    attr.Set(Gf.Vec3d(0, 0, 0))
        recursive_reset_transforms(child, exclude_list)


def recursive_transform_removal(prim: object, exclude_list: list | None = None) -> None:
    """Recursively remove xformOp attributes from all descendants.

    Traverses through all child prims and removes transform operation attributes, with option to exclude specific attributes from removal.

    Args:
        prim: The USD prim to process.
        exclude_list: List of transform attribute names to exclude from removal.
    """
    if exclude_list is None:
        exclude_list = []
    for child_prim in prim.GetAllChildren():
        # Get a list of attributes that start with xformOp
        for attr in child_prim.GetAttributes():
            if attr.GetName().startswith("xformOp:"):
                if attr.GetName() not in exclude_list:
                    child_prim.RemoveProperty(attr.GetName())
        # Recursively remove xformOp attributes from all descendants
        recursive_transform_removal(child_prim, exclude_list)


def create_physics_variant() -> None:
    """Create a physics variant for the robot.

    Creates a new physics layer with collision APIs, rigid body APIs, and references to collider meshes. Sets up the physics structure while preserving the base visual structure.

    Returns:
        None if the robot is not registered.
    """
    robot = RobotRegistry().get()
    if not robot:
        return

    # get the file paths from the robot registry
    robot_name = robot.name
    robot_links = robot.links
    base_layer_path = robot.base_file_path
    physics_layer_path = robot.physics_file_path

    # open a clean new stage call it the physics usd
    omni.usd.get_context().new_stage()
    physics_stage = omni.usd.get_context().get_stage()
    # insert the base usd as a layer
    # base_layer_relative_path = os.path.relpath(base_layer_path, os.path.dirname(physics_layer_path))
    physics_stage.GetRootLayer().subLayerPaths.append(base_layer_path)
    # lock the base layer
    base_layer = Sdf.Layer.Find(base_layer_path)
    base_layer.SetPermissionToEdit(False)

    # set edit target to root layer
    root_layer = physics_stage.GetRootLayer()
    edit_target = physics_stage.GetEditTargetForLocalLayer(root_layer)
    physics_stage.SetEditTarget(edit_target)

    stage = omni.usd.get_context().get_stage()

    # add collider API to the meshes
    for link_name in robot_links:
        link_path = f"/meshes/{link_name}"
        link_prim = stage.GetPrimAtPath(link_path)
        link_children = link_prim.GetChildren()
        if len(link_children) > 0:
            for child in link_children:
                if child.GetTypeName() in MESH_TYPES:
                    child_path = child.GetPath().pathString

    # add collider links under the robot, and reference the collider meshes
    for link_name in robot_links:
        # under the colliders folder
        collider_xform = stage.GetPrimAtPath(f"/colliders/{link_name}")
        collider_xform.GetReferences().AddInternalReference(f"/meshes/{link_name}")
        # under the robot
        link_collider_path = f"/{robot_name}/{link_name}/collider"
        UsdGeom.Xform.Define(stage, link_collider_path)
        # match the xform of the collider link with the visual link
        link_collider_prim = stage.GetPrimAtPath(link_collider_path)
        link_collider_prim.GetReferences().AddInternalReference(f"/colliders/{link_name}")
        link_collider_prim.SetInstanceable(True)

    # move the joint folder to the physics layer
    joint_folder = f"/{robot_name}/Joints"
    joint_prim = stage.GetPrimAtPath(joint_folder)
    omni.kit.commands.execute(
        "MovePrimSpecsToLayer",
        dst_layer_identifier=physics_layer_path,
        src_layer_identifier=base_layer_path,
        prim_spec_path=str(joint_prim.GetPath().pathString),
        dst_stronger_than_src=True,
    )

    # add all the physics relevant APIs (RigidBodyAPI, ColliderAPI, etc) to the robot prim
    for link_name in robot_links:
        link_prim = stage.GetPrimAtPath(f"/{robot_name}/{link_name}")
        # rigid body api
        if "PhysicsRigidBodyAPI" not in link_prim.GetAppliedSchemas():
            UsdPhysics.RigidBodyAPI.Apply(link_prim)
            # remove any rigid body api from its children
            _recursive_api_removal(link_prim, "PhysicsRigidBodyAPI", UsdPhysics.RigidBodyAPI)


def _recursive_api_removal(prim: object, api_name: str, api_handle: object) -> None:
    """Remove the api from the prim.

    Recursively traverses all children of the prim and removes the specified API schema from each child that has it applied.

    Args:
        prim: The USD prim to process.
        api_name: The name of the API schema to remove.
        api_handle: The API class handle used to remove the schema.
    """
    for child in prim.GetChildren():
        if api_name in child.GetAppliedSchemas():
            child.RemoveAPI(api_handle)
        _recursive_api_removal(child, api_name, api_handle)


def clean_up(robot_stage: object) -> None:
    """Delete all the scopes and prims that are not needed.

    Delete all the scopes and prims that are not needed (how can you tell if a prim is "leftover"? or do we need user input: a list of vestigial prims, and ask to keep/delete/label as reference).

    Args:
        robot_stage: The USD stage containing the robot to clean up.
    """
