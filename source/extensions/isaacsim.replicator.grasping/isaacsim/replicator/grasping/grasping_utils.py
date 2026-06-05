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

"""Utilities for physics simulation, grasping operations, and USD manipulation in Isaac Sim replicator grasping workflows."""

import os

import carb
import isaacsim.replicator.grasping.sampler_utils as sampler_utils
import omni.kit.commands
import omni.physx
import omni.timeline
import omni.usd
import yaml
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


def create_or_get_physics_scene(stage: Usd.Stage, path: str) -> UsdPhysics.Scene | None:
    """Get the PhysicsScene prim at the given path, creating it if it doesn't exist.

    Args:
        stage: The USD stage to operate on.
        path: The stage path where the physics scene should be located.

    Returns:
        The UsdPhysics.Scene at the specified path, or None if creation/retrieval fails.
    """
    scene_prim = stage.GetPrimAtPath(path)
    if scene_prim:
        if scene_prim.IsA(UsdPhysics.Scene):
            return UsdPhysics.Scene(scene_prim)
        else:
            carb.log_warn(f"Prim at '{path}' exists but is not a UsdPhysics.Scene.")
            return None
    else:
        scene_prim = UsdPhysics.Scene.Define(stage, path).GetPrim()
        if scene_prim:
            print(f"Created UsdPhysics.Scene at '{path}'.")
            return UsdPhysics.Scene(scene_prim)
        else:
            carb.log_warn(f"Failed to create UsdPhysics.Scene at '{path}'.")
            return None


def get_physics_scene(stage: Usd.Stage, path: str) -> UsdPhysics.Scene | None:
    """Get the PhysicsScene prim at the given path if it exists and is of the correct type.

    Args:
        stage: The USD stage.
        path: The absolute path for the physics scene prim.

    Returns:
        The UsdPhysics.Scene at the path, or None if not found or invalid.
    """
    if not Sdf.Path.IsValidPathString(path):
        carb.log_warn(f"Invalid physics scene path string provided: '{path}'.")
        return None
    scene_prim = stage.GetPrimAtPath(path)
    if not scene_prim:
        carb.log_warn(f"No prim found at path '{path}'.")
        return None
    if not scene_prim.IsA(UsdPhysics.Scene):
        carb.log_warn(f"Prim at path '{path}' is not a UsdPhysics.Scene (Type: {scene_prim.GetTypeName()}).")
        return None
    return UsdPhysics.Scene(scene_prim)


def remove_physics_scene(stage: Usd.Stage, physics_scene_path: str) -> bool:
    """Remove a UsdPhysics.Scene prim at the specified path.

    Args:
        stage: The USD stage.
        physics_scene_path: The absolute path for the physics scene prim.

    Returns:
        True if successful, False otherwise.
    """
    physics_scene_prim = stage.GetPrimAtPath(physics_scene_path)
    if physics_scene_prim.IsValid():
        stage.RemovePrim(physics_scene_path)
        return True
    else:
        carb.log_warn(f"Physics scene at path {physics_scene_path} does not exist.")
        return False


def set_rigid_body_simulation_owner(
    prims: list[Usd.Prim] | Usd.Prim, physics_scene_prim: UsdPhysics.Scene, include_descendants: bool = True
) -> list[Usd.Prim]:
    """Set the simulationOwner for a prim and all its descendants to the given physics scene.

    Args:
        prims: The prim or list of prims to set the simulation owner for.
        physics_scene_prim: The physics scene to set the simulation owner for.
        include_descendants: Whether to include the descendants of the prim in the simulation owner set.

    Returns:
        List of prims that have been added to the physics scene.
    """
    physics_scene_path = physics_scene_prim.GetPath()
    added_prims = []
    prims_to_process = prims if isinstance(prims, list) else [prims]

    for p in prims_to_process:
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI(p)
            sim_owner_rel = rigid_body_api.GetSimulationOwnerRel()
            targets = sim_owner_rel.GetTargets()

            if physics_scene_path not in targets:
                new_targets = list(targets)
                new_targets.append(physics_scene_path)
                if sim_owner_rel.SetTargets(new_targets):
                    added_prims.append(p)

        if include_descendants:
            for child in Usd.PrimRange(p):
                if child != p and child.HasAPI(UsdPhysics.RigidBodyAPI):
                    rigid_body_api = UsdPhysics.RigidBodyAPI(child)
                    sim_owner_rel = rigid_body_api.GetSimulationOwnerRel()
                    targets = sim_owner_rel.GetTargets()

                    if physics_scene_path not in targets:
                        new_targets = list(targets)
                        new_targets.append(physics_scene_path)
                        if sim_owner_rel.SetTargets(new_targets):
                            added_prims.append(child)

    return added_prims


def remove_rigid_body_simulation_owner(
    prims: list[Usd.Prim] | Usd.Prim, physics_scene_prim: UsdPhysics.Scene, include_descendants: bool = True
) -> list[Usd.Prim]:
    """Remove the simulationOwner for a prim and all its descendants from the given physics scene.

    Args:
        prims: The prim or list of prims to remove the simulation owner for.
        physics_scene_prim: The physics scene to remove the simulation owner for.
        include_descendants: Whether to include the descendants of the prim in the simulation owner removal.

    Returns:
        A list of rigid body prims that were successfully removed as owners.
    """
    physics_scene_path = physics_scene_prim.GetPath()
    removed_prims = []
    prims_to_process = prims if isinstance(prims, list) else [prims]

    for p in prims_to_process:
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI(p)
            sim_owner_rel = rigid_body_api.GetSimulationOwnerRel()
            targets = sim_owner_rel.GetTargets()

            if physics_scene_path in targets:
                new_targets = list(targets)
                new_targets.remove(physics_scene_path)
                if sim_owner_rel.SetTargets(new_targets):
                    removed_prims.append(p)

        if include_descendants:
            for child in Usd.PrimRange(p):
                if child != p and child.HasAPI(UsdPhysics.RigidBodyAPI):
                    rigid_body_api = UsdPhysics.RigidBodyAPI(child)
                    sim_owner_rel = rigid_body_api.GetSimulationOwnerRel()
                    targets = sim_owner_rel.GetTargets()

                    if physics_scene_path in targets:
                        new_targets = list(targets)
                        new_targets.remove(physics_scene_path)
                        if sim_owner_rel.SetTargets(new_targets):
                            removed_prims.append(child)

    return removed_prims


def set_joint_drive_parameters(
    joint_prim: Usd.Prim,
    target_value: float,
    target_type: str = "position",
    stiffness: float = None,
    damping: float = None,
    max_force: float = None,
) -> bool:
    """Set drive parameters for a joint.

    Sets the target position, velocity, and optionally stiffness, damping, and max force
    for a joint, handling both revolute and prismatic joints appropriately.

    Args:
        joint_prim: The USD joint prim to set drive parameters for.
        target_value: The target value to set.
        target_type: The type of target to set ("position" or "velocity").
        stiffness: Optional stiffness value for the drive.
        damping: Optional damping value for the drive.
        max_force: Optional maximum force value for the drive.

    Returns:
        True if successful, False otherwise.
    """
    if not joint_prim.IsValid():
        return False

    # Determine the appropriate drive type based on joint type
    drive_type = None
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        drive_type = "angular"
    elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
        drive_type = "linear"
    else:
        carb.log_warn(f"Cannot set joint drive parameters, '{joint_prim.GetPath()}' is not a valid joint type.")
        return False

    # Get or create the drive API with the correct type
    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, drive_type)
    if not drive_api:
        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, drive_type)
        if not drive_api:
            carb.log_warn(
                f"Cannot set joint drive parameters, failed to apply UsdPhysics.DriveAPI to '{joint_prim.GetPath()}'."
            )
            return False

    # Set the target based on type
    if target_type == "position":
        if not drive_api.GetTargetPositionAttr():
            drive_api.CreateTargetPositionAttr(target_value)
        else:
            drive_api.GetTargetPositionAttr().Set(target_value)
    elif target_type == "velocity":
        if not drive_api.GetTargetVelocityAttr():
            drive_api.CreateTargetVelocityAttr(target_value)
        else:
            drive_api.GetTargetVelocityAttr().Set(target_value)
    else:
        carb.log_warn(
            f"Cannot set joint drive parameters, invalid target type '{target_type}' for joint '{joint_prim.GetPath()}'."
        )
        return False

    # Set optional parameters if provided
    if stiffness is not None:
        if not drive_api.GetStiffnessAttr():
            drive_api.CreateStiffnessAttr(stiffness)
        else:
            drive_api.GetStiffnessAttr().Set(stiffness)

    if damping is not None:
        if not drive_api.GetDampingAttr():
            drive_api.CreateDampingAttr(damping)
        else:
            drive_api.GetDampingAttr().Set(damping)

    if max_force is not None:
        if not drive_api.GetMaxForceAttr():
            drive_api.CreateMaxForceAttr(max_force)
        else:
            drive_api.GetMaxForceAttr().Set(max_force)

    return True


def set_joint_state(joint_prim: Usd.Prim, position_value: float = 0.0, velocity_value: float = 0.0) -> bool:
    """Set the joint state parameters for a joint. Applies PhysxSchema.JointStateAPI if missing.

    Args:
        joint_prim: The USD joint prim to set joint state parameters for.
        position_value: The position value to set.
        velocity_value: The velocity value to set.

    Returns:
        True if successful, False otherwise.
    """
    if not joint_prim.IsValid():
        return False

    # Determine the appropriate drive type based on joint type
    drive_type = None
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        drive_type = "angular"
    elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
        drive_type = "linear"
    else:
        carb.log_warn(f"Cannot set joint state, '{joint_prim.GetPath()}' is not a valid joint type.")
        return False

    # Check if the joint state API is already applied
    if not joint_prim.HasAPI(PhysxSchema.JointStateAPI, drive_type):
        joint_state_api = PhysxSchema.JointStateAPI.Apply(joint_prim, drive_type)
        carb.log_warn(
            f"Applied missing PhysxSchema.JointStateAPI to joint '{joint_prim.GetPath()}' to set joint state."
        )
        if not joint_state_api:
            carb.log_warn(
                f"Cannot set joint state, failed to apply PhysxSchema.JointStateAPI to '{joint_prim.GetPath()}'."
            )
            return False
    else:
        joint_state_api = PhysxSchema.JointStateAPI.Get(joint_prim, drive_type)

    # Set the position and velocity values
    if not joint_state_api.GetPositionAttr():
        joint_state_api.CreatePositionAttr(position_value)
    else:
        joint_state_api.GetPositionAttr().Set(position_value)

    if not joint_state_api.GetVelocityAttr():
        joint_state_api.CreateVelocityAttr(velocity_value)
    else:
        joint_state_api.GetVelocityAttr().Set(velocity_value)

    return True


def get_joint_state(joint_prim: Usd.Prim) -> tuple[float, float]:
    """Get the current joint state (position, velocity) from a joint prim.

    Reads from the PhysxSchema.JointStateAPI. If the API or its attributes
    are not found, returns (0.0, 0.0).

    Args:
        joint_prim: The USD joint prim (Revolute or Prismatic).

    Returns:
        A tuple containing the (position, velocity).
    """
    position = 0.0
    velocity = 0.0

    if not joint_prim or not joint_prim.IsValid():
        return position, velocity

    # Determine the appropriate drive type name
    drive_type = None
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        drive_type = "angular"
    elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
        drive_type = "linear"
    else:
        # Not a supported joint type for state reading
        return position, velocity

    # Check if the joint state API exists
    if joint_prim.HasAPI(PhysxSchema.JointStateAPI, drive_type):
        joint_state_api = PhysxSchema.JointStateAPI.Get(joint_prim, drive_type)
        if joint_state_api:
            pos_attr = joint_state_api.GetPositionAttr()
            if pos_attr:
                pos_val = pos_attr.Get()
                if pos_val is not None:
                    position = pos_val

            vel_attr = joint_state_api.GetVelocityAttr()
            if vel_attr:
                vel_val = vel_attr.Get()
                if vel_val is not None:
                    velocity = vel_val

    return position, velocity


def _disable_scene_auto_updates(physics_scene: UsdPhysics.Scene) -> tuple[Usd.Attribute, str | None, bool]:
    """Disable automatic updates for a PhysX scene and return its previous update state."""
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    update_type_attr = physx_scene_api.GetUpdateTypeAttr()
    original_update_type = update_type_attr.Get() if update_type_attr else None
    original_update_type_authored = update_type_attr.HasAuthoredValueOpinion() if update_type_attr else False

    update_type_attr = physx_scene_api.CreateUpdateTypeAttr()
    update_type_attr.Set(PhysxSchema.Tokens.Disabled)
    return update_type_attr, original_update_type, original_update_type_authored


def _restore_scene_auto_updates(
    update_type_attr: Usd.Attribute, original_update_type: str | None, original_update_type_authored: bool
) -> None:
    """Restore a PhysX scene update type captured by `_disable_scene_auto_updates()`."""
    if original_update_type_authored and original_update_type is not None:
        update_type_attr.Set(original_update_type)
    else:
        update_type_attr.Clear()


async def simulate_physics_async(
    num_frames: int, step_dt: float, physics_scene: UsdPhysics.Scene | None = None, render: bool = False
) -> None:
    """Simulate physics for a fixed number of frames asynchronously.

    Args:
        num_frames: The number of simulation steps/frames to run (> 0).
        step_dt: The time step for each simulation step (> 0).
        physics_scene: The specific physics scene to simulate. If None,
                       simulates the default global physics context (optional).
        render: Whether to render the simulation by waiting for omni.kit.app updates.

    Raises:
        ValueError: If num_frames <= 0 or step_dt <= 0.
    """
    if num_frames <= 0:
        raise ValueError("num_frames must be positive.")
    if step_dt <= 0:
        raise ValueError("step_dt must be positive.")

    physx_sim_interface = omni.physx.get_physx_simulation_interface()

    if physics_scene:
        update_type_attr, original_update_type, original_update_type_authored = _disable_scene_auto_updates(
            physics_scene
        )
        try:
            physics_scene_path = physics_scene.GetPath()
            scene_int = PhysicsSchemaTools.sdfPathToInt(physics_scene_path)
            for _ in range(num_frames):
                physx_sim_interface.simulate_scene(scene_int, step_dt, 0)
                physx_sim_interface.fetch_results_scene(scene_int)
                if render:
                    await omni.kit.app.get_app().next_update_async()
            physx_sim_interface.fetch_results_scene(scene_int)
            await omni.kit.app.get_app().next_update_async()
        finally:
            _restore_scene_auto_updates(update_type_attr, original_update_type, original_update_type_authored)
    else:
        for _ in range(num_frames):
            physx_sim_interface.simulate(step_dt, 0)
            physx_sim_interface.fetch_results()
            if render:
                await omni.kit.app.get_app().next_update_async()
        physx_sim_interface.fetch_results()
        await omni.kit.app.get_app().next_update_async()


async def simulate_physics_with_forces_async(
    stage_id: int,
    body_ids: list[int],
    forces: list[tuple[float, float, float]],
    positions: list[tuple[float, float, float]],
    sim_steps: int,
    physx_dt: float,
    physics_scene: UsdPhysics.Scene | None = None,
    render: bool = True,
) -> None:
    """Apply forces to assets at specified positions and simulate physics asynchronously.

    Can simulate either the default physics context or a specific physics scene.

    Args:
        stage_id: The ID of the stage context (used for applying forces).
        body_ids: List of integer body IDs to apply forces to.
        forces: List of force vectors (tuples of 3 floats) corresponding to body_ids.
        positions: List of position vectors (tuples of 3 floats) where forces are applied,
                   corresponding to body_ids.
        sim_steps: The number of simulation steps to run (> 0).
        physx_dt: The time step for each simulation step (> 0).
        physics_scene: The specific physics scene to simulate. If None,
                       simulates the default global physics context (optional).
        render: Whether to render the simulation by waiting for omni.kit.app updates.

    Raises:
        ValueError: If sim_steps <= 0 or physx_dt <= 0.
    """
    if sim_steps <= 0:
        raise ValueError("sim_steps must be positive.")
    if physx_dt <= 0:
        raise ValueError("physx_dt must be positive.")

    # Physx APIs to apply the forces and to advance the simulation
    physx_interface = omni.physx.get_physx_simulation_interface()

    # Apply the forces - Assumes forces are applied globally via stage_id
    # even when simulating a specific scene. Bodies not in the scene
    # should not be affected by the scene's simulation step.
    for body_id, force, position in zip(body_ids, forces, positions):
        physx_interface.apply_force_at_pos(stage_id, body_id, carb.Float3(*force), carb.Float3(*position))

    # Run the simulation for the specified number of steps
    if physics_scene:
        print(
            f"Simulating physics with forces using scene: '{physics_scene.GetPath()}', steps {sim_steps}, dt {physx_dt}"
        )
        # Ensure the scene isn't updating automatically if we drive it manually
        update_type_attr, original_update_type, original_update_type_authored = _disable_scene_auto_updates(
            physics_scene
        )
        try:
            physics_scene_path = physics_scene.GetPath()
            scene_int = PhysicsSchemaTools.sdfPathToInt(physics_scene_path)
            for _ in range(sim_steps):
                physx_interface.simulate_scene(scene_int, physx_dt, 0)
                physx_interface.fetch_results_scene(scene_int)
                if render:
                    await omni.kit.app.get_app().next_update_async()
            # Fetch results one last time after the loop
            physx_interface.fetch_results_scene(scene_int)
            await omni.kit.app.get_app().next_update_async()
        finally:
            _restore_scene_auto_updates(update_type_attr, original_update_type, original_update_type_authored)
    else:
        print(f"Simulating physics with forces using default scene, steps {sim_steps}, dt {physx_dt}")
        for _ in range(sim_steps):
            physx_interface.simulate(physx_dt, 0)
            physx_interface.fetch_results()
            if render:
                await omni.kit.app.get_app().next_update_async()
        # Fetch results one last time after the loop
        physx_interface.fetch_results()
        await omni.kit.app.get_app().next_update_async()


def reset_physics_simulation() -> None:
    """Reset the simulation to its initial state."""
    physx_interface = omni.physx.get_physx_interface()
    physx_interface.reset_simulation()


async def advance_timeline_async(num_frames: int, render: bool = False) -> None:
    """Simulate by advancing the main Omniverse timeline for a fixed number of frames.

    Args:
        num_frames: The number of timeline frames to advance (> 0).
        render: Whether to wait for kit updates between frames.

    Raises:
        ValueError: If num_frames <= 0.
    """
    if num_frames <= 0:
        raise ValueError("num_frames must be positive.")

    timeline = omni.timeline.get_timeline_interface()

    for _ in range(num_frames):
        timeline.forward_one_frame()
        if render:
            await omni.kit.app.get_app().next_update_async()
        else:
            timeline.commit()

    timeline.pause()
    await omni.kit.app.get_app().next_update_async()


def stop_timeline() -> None:
    """Reset the simulation to its initial state and stops the timeline."""
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()


def find_first_mesh_in_hierarchy(prim: Usd.Prim) -> UsdGeom.Mesh | None:
    """Find the first UsdGeom.Mesh schema under the given prim (including itself and descendants).

    Args:
        prim: The root prim to search from.

    Returns:
        The first UsdGeom.Mesh found, or None if no mesh is found.
    """
    if prim.IsA(UsdGeom.Mesh):
        return UsdGeom.Mesh(prim)
    for descendant in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if descendant.IsA(UsdGeom.Mesh):
            print(f"Using mesh '{descendant.GetPath()}' under '{prim.GetPath()}'.")
            return UsdGeom.Mesh(descendant)
    carb.log_warn(f"No UsdGeom.Mesh found for prim '{prim.GetPath()}' or its descendants.")
    return None


def duplicate_prim(stage: Usd.Stage, source_prim_path: str, destination_prim_path: str) -> Usd.Prim | None:
    """Duplicates a prim using the CopyPrimCommand.

    Args:
        stage: The USD stage.
        source_prim_path: The Sdf.Path string of the prim to duplicate.
        destination_prim_path: The desired Sdf.Path string for the duplicated prim.

    Returns:
        The duplicated Usd.Prim at destination_prim_path if successful, None otherwise.
    """
    if not Sdf.Path.IsValidPathString(source_prim_path):
        carb.log_warn(f"Invalid source path for duplication: {source_prim_path}")
        return None
    if not Sdf.Path.IsValidPathString(destination_prim_path):
        carb.log_warn(f"Invalid destination path for duplication: {destination_prim_path}")
        return None

    source_prim = stage.GetPrimAtPath(source_prim_path)
    if not source_prim.IsValid():
        carb.log_warn(f"Source prim not found or invalid at: {source_prim_path}")
        return None

    # Check if destination already exists
    if stage.GetPrimAtPath(destination_prim_path):
        carb.log_warn(f"Prim already exists at destination path: {destination_prim_path}")
        return None

    # Execute CopyPrimCommand
    success, _ = omni.kit.commands.execute("CopyPrimCommand", path_from=source_prim_path, path_to=destination_prim_path)

    if not success:
        carb.log_warn(f"CopyPrimCommand failed for source '{source_prim_path}' to '{destination_prim_path}'")
        # Double-check if it exists despite failure report
        prim = stage.GetPrimAtPath(destination_prim_path)
        if prim.IsValid():
            carb.log_warn(f"CopyPrimCommand reported failure but prim exists at {destination_prim_path}. Proceeding.")
            return prim
        return None

    # Verify prim exists at destination
    prim = stage.GetPrimAtPath(destination_prim_path)
    if not prim.IsValid():
        carb.log_warn(f"CopyPrimCommand reported success, but prim not found at destination: {destination_prim_path}")
        return None

    print(f"Successfully duplicated prim to {destination_prim_path} using CopyPrimCommand.")
    return prim


def get_gripper_joints_info(gripper_prim_path: str) -> list[dict]:
    """Get all the joints from the gripper prim at the given path with their relevant information.

    Args:
        gripper_prim_path: The stage path to the gripper prim.

    Returns:
        List of dictionaries containing joint information including path, mimic status, drive status,
        and validity for grasping operations.
    """
    stage = omni.usd.get_context().get_stage()
    if not stage:
        carb.log_warn("Cannot get gripper joints info: Stage is not set.")
        return []
    if not Sdf.Path.IsValidPathString(gripper_prim_path):
        carb.log_warn(f"Cannot get gripper joints info: Invalid gripper path '{gripper_prim_path}'.")
        return []
    gripper_prim = stage.GetPrimAtPath(gripper_prim_path)
    if not gripper_prim or not gripper_prim.IsValid():
        carb.log_warn(f"Cannot get gripper joints info: Gripper prim at '{gripper_prim_path}' is not valid.")
        return []

    joint_info_list = []
    # Use PrimRange to iterate through descendants
    for prim in Usd.PrimRange(gripper_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if prim.IsA(UsdPhysics.Joint):
            path = str(prim.GetPath())
            is_mimic = prim.HasAPI(PhysxSchema.PhysxMimicJointAPI)
            is_drive = prim.HasAPI(UsdPhysics.DriveAPI)
            # Only drive joints (that are not mimic joints) will be included in the grasp
            can_be_included = is_drive and not is_mimic

            joint_info = {
                "path": path,
                "is_mimic": is_mimic,
                "is_drive": is_drive,
                "is_valid_grasp_joint": can_be_included,
            }
            joint_info_list.append(joint_info)

    return joint_info_list


def apply_joint_pregrasp_state(joint_path: str, position_value: float) -> None:
    """Apply the pregrasp state to the given joint prim.

    Args:
        joint_path: The stage path to the joint prim.
        position_value: The position value to set for the joint.
    """
    stage = omni.usd.get_context().get_stage()
    if not stage:
        carb.log_warn("Cannot apply joint pregrasp state: Stage is not set.")
        return

    if not Sdf.Path.IsValidPathString(joint_path):
        carb.log_warn(f"Skipping invalid joint path string: '{joint_path}'")
        return

    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim or not joint_prim.IsA(UsdPhysics.Joint):
        carb.log_warn(f"Skipping invalid or non-joint prim at path '{joint_path}'.")
        return

    success = set_joint_state(joint_prim, position_value=position_value, velocity_value=0.0)
    if success:
        print(f"Applied pregrasp state to joint {joint_path}.")
    else:
        carb.log_warn(f"Failed to apply pregrasp state to joint {joint_path}.")


def apply_joint_pregrasp_states(joint_pregrasp_states: dict[str, float]) -> None:
    """Apply the pregrasp states to the given joint prims.

    Args:
        joint_pregrasp_states: Dictionary mapping joint paths to their position values.
    """
    for joint_path, position_value in joint_pregrasp_states.items():
        apply_joint_pregrasp_state(joint_path, position_value)


def isolate_prims_to_scene(prims_to_isolate: list[Usd.Prim], physics_scene: UsdPhysics.Scene) -> list[Usd.Prim]:
    """Set the simulation owner for the given prims (and their descendants) to the specified physics scene.

    Args:
        prims_to_isolate: List of prims to set the simulation owner for.
        physics_scene: The physics scene to assign owners to.

    Returns:
        A list of rigid body prims that were successfully added as owners, or an empty list.
    """
    isolated_rigid_body_prims = []
    if not physics_scene:
        carb.log_warn("Cannot isolate simulation: Physics scene is not set.")
        return isolated_rigid_body_prims
    if not prims_to_isolate:
        carb.log_warn("No prims provided to isolate.")
        return isolated_rigid_body_prims

    print(f"Attempting isolated simulation using scene: {physics_scene.GetPath()}")
    isolated_rigid_body_prims = set_rigid_body_simulation_owner(
        prims_to_isolate, physics_scene, include_descendants=True
    )
    if isolated_rigid_body_prims:
        print(
            f"Successfully added {len(isolated_rigid_body_prims)} rigid bodies to custom scene '{physics_scene.GetPath()}'."
        )
    else:
        carb.log_warn(f"Failed to add any rigid bodies to custom scene '{physics_scene.GetPath()}'.")

    return isolated_rigid_body_prims


def clear_isolated_simulation_owners(rigid_body_prims: list[Usd.Prim], physics_scene: UsdPhysics.Scene | None) -> None:
    """Clear the simulation owners for the given rigid body prims from the specified physics scene.

    Args:
        rigid_body_prims: List of rigid body prims to remove from the physics scene.
        physics_scene: The physics scene to remove the simulation owners from.
    """
    if not physics_scene:
        carb.log_warn("Cannot clear isolated simulation owners: Physics scene is not set.")
        return
    if not rigid_body_prims:
        carb.log_warn("No rigid body prims provided to clear from the scene.")
        return

    print(f"Removing {len(rigid_body_prims)} rigid bodies from custom physics scene: {physics_scene.GetPath()}")
    removed_rigid_body_prims = remove_rigid_body_simulation_owner(
        rigid_body_prims, physics_scene, include_descendants=True
    )
    if removed_rigid_body_prims:
        print(
            f"Successfully removed {len(removed_rigid_body_prims)} prims from custom physics scene: {physics_scene.GetPath()}"
        )
    else:
        carb.log_warn(f"Failed to remove prims from custom physics scene: {physics_scene.GetPath()}")


def generate_grasp_poses_from_config(
    object_prim: Usd.Prim,
    config: dict,
) -> list[tuple[Gf.Vec3d, Gf.Quatd]] | None:
    """Generate local grasp poses for the given object prim based on the configuration.

    Args:
        object_prim: The object prim (must contain a UsdGeom.Mesh).
        config: Dictionary containing grasp generation parameters (see GraspingManager.generate_grasp_poses docstring).

    Returns:
        A list of grasp poses as (location, orientation) tuples in the object's local frame,
        or None if generation fails.
    """
    if not object_prim or not object_prim.IsValid():
        carb.log_warn(f"Cannot generate grasp poses: Object prim is not valid.")
        return None

    mesh_prim = find_first_mesh_in_hierarchy(object_prim)
    if not mesh_prim:
        carb.log_warn(f"Cannot generate grasp poses: No mesh found under '{object_prim.GetPath()}'.")
        return None

    # Create a working copy of the config with the mesh prim added
    sampler_config = config.copy()
    sampler_config["mesh_prim"] = mesh_prim

    # Select the appropriate sampler based on the config
    sampler_type = sampler_config.get("sampler_type", "antipodal")

    try:
        # Choose the sampling function based on sampler_type
        if sampler_type == "antipodal":
            locations, quaternions = sampler_utils.generate_grasp_poses(sampler_config)
        # Add other samplers here if needed
        # elif sampler_type == "other_sampler":
        #     locations, quaternions = sampler_utils.generate_other_poses(sampler_config)
        else:
            carb.log_warn(f"Unknown sampler type: {sampler_type}")
            return None

        if locations is not None and quaternions is not None and len(locations) == len(quaternions):
            print(f"Successfully generated {len(locations)} grasp poses for '{object_prim.GetPath()}'.")
            return list(zip(locations, quaternions))
        else:
            carb.log_warn(f"Grasp pose generation returned invalid data for '{object_prim.GetPath()}'.")
            return None

    except Exception as e:
        carb.log_warn(f"Exception during grasp pose generation for '{object_prim.GetPath()}': {e}")
        return None


def get_gripper_joint_states(gripper_base_path: str) -> dict[str, float] | None:
    """Retrieve the current position state of all valid joints under a gripper prim.

    Args:
        gripper_base_path: The stage path to the base prim of the gripper.

    Returns:
        A dictionary mapping relative joint paths (to the gripper base) to their
        current position, or None if the gripper path is invalid or no joints are found.
    """
    if not gripper_base_path or not Sdf.Path.IsValidPathString(gripper_base_path):
        carb.log_warn(f"Invalid gripper base path provided: '{gripper_base_path}'")
        return None

    stage = omni.usd.get_context().get_stage()
    if not stage:
        carb.log_warn("Cannot get joint states: Stage is not available.")
        return None

    gripper_prim = stage.GetPrimAtPath(gripper_base_path)
    if not gripper_prim or not gripper_prim.IsValid():
        carb.log_warn(f"Gripper prim at path '{gripper_base_path}' is not valid.")
        return None

    joint_states = {}
    gripper_joints_info = get_gripper_joints_info(gripper_base_path)

    for joint_info in gripper_joints_info:
        joint_path = joint_info["path"]
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim and joint_prim.IsValid():
            try:
                state = get_joint_state(joint_prim)
                if state is not None:
                    position, _ = state
                    # USD paths are always POSIX (forward-slash). Use a string slice so the
                    # relative key stays OS-independent; os.path.relpath would inject os.sep
                    # on Windows and corrupt nested keys (e.g. 'link1\\joint_left').
                    relative_path = joint_path[len(gripper_base_path) + 1 :]
                    joint_states[relative_path] = position
                else:
                    carb.log_warn(f"Could not get state for joint {joint_path} for results.")
            except Exception as e:
                carb.log_warn(f"Error getting state for joint {joint_path} for results: {e}")

    if not joint_states:
        carb.log_warn(f"No valid joint states found under '{gripper_base_path}'.")
        return {}  # Return empty dict if no states found

    return joint_states


def populate_joint_pregrasp_states_from_current(joint_paths: list[str]) -> dict[str, float]:
    """Populate a dictionary with current joint positions for the given joint paths.

    Retrieves the current position for each valid joint prim found at the provided paths
    and returns a dictionary mapping the absolute joint path to its position.

    Args:
        joint_paths: A list of absolute Sdf.Path strings for the joints.

    Returns:
        A dictionary mapping absolute joint path to current position (float).
        Joints that are invalid or whose state cannot be retrieved will be omitted.
    """
    pregrasp_states = {}
    stage = omni.usd.get_context().get_stage()
    if not stage:
        carb.log_warn("Cannot populate pregrasp states: Stage is not available.")
        return pregrasp_states

    for joint_path in joint_paths:
        if not Sdf.Path.IsValidPathString(joint_path):
            carb.log_warn(f"Skipping invalid joint path string for pregrasp state: '{joint_path}'")
            continue

        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim and joint_prim.IsValid() and joint_prim.IsA(UsdPhysics.Joint):
            try:
                current_state = get_joint_state(joint_prim)
                if current_state is not None:
                    position, _ = current_state
                    pregrasp_states[joint_path] = position
                else:
                    carb.log_warn(
                        f"Could not get current state for joint {joint_path}. Using 0.0 for initial pregrasp."
                    )
                    pregrasp_states[joint_path] = 0.0
            except Exception as e:
                carb.log_warn(f"Error getting state for joint {joint_path}: {e}. Using 0.0 for initial pregrasp.")
                pregrasp_states[joint_path] = 0.0
        else:
            carb.log_warn(
                f"Joint prim at path '{joint_path}' is invalid or not a Joint. Using 0.0 for initial pregrasp."
            )
            pregrasp_states[joint_path] = 0.0

    return pregrasp_states


def write_yaml_config(file_path: str, config: dict, overwrite: bool = False) -> bool:
    """Save the configuration dictionary to a YAML file.

    Args:
        file_path: The path to the YAML file.
        config: The configuration dictionary to save.
        overwrite: Whether to overwrite the file if it exists.

    Returns:
        True if saving was successful, False otherwise.
    """
    if not file_path or not isinstance(file_path, str) or not file_path.strip():
        carb.log_warn(f"Invalid file path provided for writing YAML: '{file_path}'")
        return False

    # Expand user path (~)
    file_path = os.path.expanduser(file_path)

    if os.path.exists(file_path) and not overwrite:
        carb.log_warn(f"File '{file_path}' already exists and overwrite is False. YAML save cancelled.")
        return False

    try:
        dir_path = os.path.dirname(file_path)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)

        if os.path.exists(file_path) and overwrite:
            print(f"Overwriting existing YAML file: '{file_path}'")
        with open(file_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False)

        print(f"Successfully saved configuration to YAML file: {file_path}")
        return True
    except Exception as e:
        carb.log_warn(f"Could not save configuration to YAML path '{file_path}': {str(e)}")
        return False


def read_yaml_config(file_path: str) -> dict | None:
    """Load configuration from a YAML file and return as a dictionary.

    Args:
        file_path: The path to the YAML file.

    Returns:
        The loaded configuration dictionary, or None on error (e.g., file not found, invalid format).
    """
    if not file_path or not isinstance(file_path, str) or not file_path.strip():
        carb.log_warn(f"Invalid file path provided for reading YAML: '{file_path}'")
        return None
    if not file_path.lower().endswith(".yaml"):
        carb.log_warn(f"File path '{file_path}' does not end with .yaml. Attempting to load anyway.")
        # Allow non-.yaml extensions for flexibility, but warn

    # Expand user path (~)
    file_path = os.path.expanduser(file_path)

    if not os.path.exists(file_path):
        carb.log_warn(f"Configuration YAML file not found at: '{file_path}'")
        return None

    try:
        with open(file_path) as f:
            config_data = yaml.safe_load(f)
        if not isinstance(config_data, dict):
            carb.log_warn(f"Error: Invalid configuration file format in '{file_path}'. Expected a dictionary.")
            return None
        print(f"Successfully parsed YAML configuration file: {file_path}")
        return config_data
    except yaml.YAMLError as e:
        carb.log_warn(f"Error parsing YAML file '{file_path}': {str(e)}")
        return None
    except Exception as e:
        carb.log_warn(f"Could not load configuration from YAML file '{file_path}': {str(e)}")
        return None
