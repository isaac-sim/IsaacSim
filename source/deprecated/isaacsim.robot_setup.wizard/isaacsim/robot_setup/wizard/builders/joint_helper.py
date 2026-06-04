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

"""Backend of "Add Joints and Drives"."""

import omni.usd
from pxr import PhysxSchema, UsdPhysics

JOINT_TYPES = ["Prismatic", "Revolute", "Fixed"]  ## TODO: future suppor:  "Spherical", "D6", "Mimic"
DRIVE_TYPES = ["force", "acceleration"]
ACTUATOR_TYPES = ["linear", "angular"]
AXIS_LIST = ["X", "Y", "Z"]


def get_joint_handle(joint_prim: object) -> None:
    """Get the joint handle.

    Args:
        joint_prim: The joint primitive to get the handle for.

    Returns:
        The USD Physics joint handle corresponding to the joint type, or None if the joint type is invalid.
    """
    stage = omni.usd.get_context().get_stage()
    joint_path = joint_prim.GetPath()
    if joint_prim.GetTypeName() == "PhysicsPrismaticJoint":
        joint_handle = UsdPhysics.PrismaticJoint.Get(stage, joint_path)
    elif joint_prim.GetTypeName() == "PhysicsRevoluteJoint":
        joint_handle = UsdPhysics.RevoluteJoint.Get(stage, joint_path)
    elif joint_prim.GetTypeName() == "PhysicsFixedJoint":
        joint_handle = UsdPhysics.FixedJoint.Get(stage, joint_path)
    elif joint_prim.GetTypeName() == "PhysicsSphericalJoint":
        joint_handle = UsdPhysics.SphericalJoint.Get(stage, joint_path)
    elif joint_prim.GetTypeName() == "PhysicsD6Joint":
        joint_handle = UsdPhysics.D6Joint.Get(stage, joint_path)
    else:
        print(f"joint {joint_prim.GetName()} is not a valid joint")
        return None

    return joint_handle


def define_joints(joint_path: str, joint_type: str, axis: str, parent: str, child: str, **kwargs: object) -> None:
    """Define the joints for the robot, or update the existing joint.

    Args:
        joint_path: USD path where the joint will be created or updated.
        joint_type: Type of joint to create (must be one of: Prismatic, Revolute, Fixed).
        axis: Joint axis (must be X, Y, or Z).
        parent: USD path of the parent body for the joint.
        child: USD path of the child body for the joint.
        **kwargs: Additional keyword arguments passed to the function.

    Returns:
        None if the input is invalid or the joint creation fails.
    """
    if joint_type is None or parent is None or child is None or axis is None:
        print("joint_type, parent, child, axis are required")
        return

    if axis is not None:
        if axis not in AXIS_LIST:
            print("axis must be one of the following: ", AXIS_LIST)
            return

    stage = omni.usd.get_context().get_stage()
    if not stage:
        return

    joint_prim = stage.GetPrimAtPath(joint_path)

    # first check if any joint prim of the same name needs to be deleted.
    # if joint already exist, get the joint handle
    if joint_prim and joint_prim.HasAttribute("physics:jointEnabled"):
        # if the joint type is different, delete the old one and create a new one
        prim_type_to_joint_type = f"Physics{joint_type}Joint"
        if prim_type_to_joint_type != joint_prim.GetTypeName():
            stage.RemovePrim(joint_path)
            joint_prim = None
        else:
            joint_type = joint_prim.GetTypeName()
            if joint_type == "PhysicsFixedJoint":
                joint_handle = UsdPhysics.FixedJoint.Get(stage, joint_path)
            elif joint_type == "PhysicsPrismaticJoint":
                joint_handle = UsdPhysics.PrismaticJoint.Get(stage, joint_path)
            elif joint_type == "PhysicsRevoluteJoint":
                joint_handle = UsdPhysics.RevoluteJoint.Get(stage, joint_path)
            elif joint_type == "PhysicsSphericalJoint":
                joint_handle = UsdPhysics.SphericalJoint.Get(stage, joint_path)
            elif joint_type == "PhysicsD6Joint":
                joint_handle = UsdPhysics.D6Joint.Get(stage, joint_path)
            else:
                print("joint type must be one of the following: ", JOINT_TYPES)
                return

    # (not elif, because the joint might get deleted above)
    if not joint_prim:
        # define the joint
        if joint_type == "Prismatic":
            joint_handle = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
        elif joint_type == "Revolute":
            joint_handle = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        elif joint_type == "Fixed":
            joint_handle = UsdPhysics.FixedJoint.Define(stage, joint_path)
        elif joint_type == "Spherical":
            joint_handle = UsdPhysics.SphericalJoint.Define(stage, joint_path)
        elif joint_type == "D6":
            joint_handle = UsdPhysics.D6Joint.Define(stage, joint_path)
        # elif joint_type == "Mimic":
        #
        #     mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, nonsenseDofMap[axis])
        #     mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
        #     mimicJointAPI.GetGearingAttr().Set(gearing)
        #     mimicJointAPI.GetOffsetAttr().Set(offset)
        else:
            print("joint_type must be one of the following: ", JOINT_TYPES)
            return

    # set rest of the joint settings
    joint_prim = stage.GetPrimAtPath(joint_path)
    if joint_prim.GetTypeName() != "PhysicsFixedJoint":
        # fixed joints doesn't have axis attribute
        if joint_prim.HasAttribute("physics:axis"):
            joint_handle.GetAxisAttr().Set(axis)
        else:
            joint_handle.CreateAxisAttr(axis)
        # fixed joints doesn't need a parent body
        if joint_prim.HasAttribute("physics:body0"):
            joint_handle.GetBody0Rel().SetTargets([f"{parent}"])
        else:
            joint_handle.CreateBody0Rel().SetTargets([f"{parent}"])
    if joint_prim.HasAttribute("physics:body1"):
        joint_handle.GetBody1Rel().SetTargets([f"{child}"])
    else:
        joint_handle.CreateBody1Rel().SetTargets([f"{child}"])

    return


def apply_joint_settings(joint_path: str, **kwargs: object) -> None:
    """Apply the joint settings to the joint.

    Args:
        joint_path: USD path of the joint to apply settings to.
        **kwargs: Additional keyword arguments for joint configuration including break_force, break_torque,
            lower_limit, and upper_limit.

    Returns:
        None if the joint prim or stage is invalid, or if the joint is a fixed joint.
    """
    stage = omni.usd.get_context().get_stage()
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim or not stage:
        return

    if joint_prim.GetTypeName() == "PhysicsFixedJoint":
        # fixed joints don't have any settings to apply
        return

    joint_handle = get_joint_handle(joint_prim)

    break_force = kwargs.get("break_force", None)
    break_torque = kwargs.get("break_torque", None)
    lower_limit = kwargs.get("lower_limit", None)
    upper_limit = kwargs.get("upper_limit", None)

    if break_force is not None:
        if joint_prim.HasAttribute("physics:breakForce"):
            joint_handle.GetBreakForceAttr().Set(break_force)
        else:
            joint_handle.CreateBreakForceAttr(break_force)
    if break_torque is not None:
        if joint_prim.HasAttribute("physics:breakTorque"):
            joint_handle.GetBreakTorqueAttr().Set(break_torque)
        else:
            joint_handle.CreateBreakTorqueAttr(break_torque)
    if lower_limit is not None:
        if joint_prim.HasAttribute("physics:lowerLimit"):
            joint_handle.GetLowerLimitAttr().Set(lower_limit)
        else:
            joint_handle.CreateLowerLimitAttr(lower_limit)
    if upper_limit is not None:
        if joint_prim.HasAttribute("physics:upperLimit"):
            joint_handle.GetUpperLimitAttr().Set(upper_limit)
        else:
            joint_handle.CreateUpperLimitAttr(upper_limit)


def apply_drive_settings(joint_path: str, **kwargs: object) -> None:
    """Define the drive for the joint.

    Args:
        joint_path: USD path of the joint to apply drive settings to.
        **kwargs: Additional keyword arguments for drive configuration including drive_type, max_force,
            target_velocity, target_position, damping, and stiffness.

    Returns:
        None if the joint prim or stage is invalid, or if the joint type is unsupported.
    """
    stage = omni.usd.get_context().get_stage()
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim or not stage:
        return

    if joint_prim.GetTypeName() == "PhysicsFixedJoint":
        # fixed joints don't have any settings to apply
        return

    joint_handle = get_joint_handle(joint_prim)

    if not joint_handle:
        return

    drive_type = kwargs.get("drive_type", None)
    max_force = kwargs.get("max_force", None)
    target_velocity = kwargs.get("target_velocity", None)
    target_position = kwargs.get("target_position", None)
    damping = kwargs.get("damping", None)
    stiffness = kwargs.get("stiffness", None)

    # add/modify drives to the joint
    if joint_prim.GetTypeName() == "PhysicsPrismaticJoint":
        actuator_type = "linear"
    elif joint_prim.GetTypeName() == "PhysicsRevoluteJoint":
        actuator_type = "angular"
    else:
        # joint type not supported with drive
        return

    # first check if the drive already exists
    applied = joint_prim.GetAppliedSchemas()
    driveNames = [s.split(":", 1)[1] for s in applied if s.startswith("PhysicsDriveAPI:")]
    for driveName in driveNames:
        if driveName != actuator_type:
            joint_prim.RemoveAPI(UsdPhysics.DriveAPI, driveName)
    # by now either driveName is empty or driveName is the same as actuator_type
    if not driveNames:
        drive_handle = UsdPhysics.DriveAPI.Apply(joint_prim, actuator_type)
    else:
        drive_handle = UsdPhysics.DriveAPI.Get(joint_prim, actuator_type)

    ### now apply rest of the drive settings
    if drive_type and joint_prim.HasAttribute(f"drive:{actuator_type}:physics:type"):
        drive_handle.GetTypeAttr().Set(drive_type)
    else:
        drive_handle.CreateTypeAttr(drive_type)
    if max_force and joint_prim.HasAttribute(f"drive:{actuator_type}:physics:maxForce"):
        drive_handle.GetMaxForceAttr().Set(max_force)
    else:
        drive_handle.CreateMaxForceAttr(max_force)
    if target_velocity and joint_prim.HasAttribute(f"drive:{actuator_type}:physics:targetVelocity"):
        drive_handle.GetTargetVelocityAttr().Set(target_velocity)
    else:
        drive_handle.CreateTargetVelocityAttr(target_velocity)
    if target_position and joint_prim.HasAttribute(f"drive:{actuator_type}:physics:targetPosition"):
        drive_handle.GetTargetPositionAttr().Set(target_position)
    else:
        drive_handle.CreateTargetPositionAttr(target_position)
    if damping and joint_prim.HasAttribute(f"drive:{actuator_type}:physics:damping"):
        drive_handle.GetDampingAttr().Set(damping)
    else:
        drive_handle.CreateDampingAttr(damping)
    if stiffness and joint_prim.HasAttribute(f"drive:{actuator_type}:physics:stiffness"):
        drive_handle.GetStiffnessAttr().Set(stiffness)
    else:
        drive_handle.CreateStiffnessAttr(stiffness)


def get_all_settings(joint_path: str) -> dict:
    """Get all the settings for the joint.

    Args:
        joint_path: USD path of the joint to retrieve settings from.

    Returns:
        Dictionary containing all joint settings including joint name, type, parent, child, axis, limits,
        break forces, and drive parameters.
    """
    stage = omni.usd.get_context().get_stage()
    joint_prim = stage.GetPrimAtPath(joint_path)
    settings_dict = {}
    settings_dict["joint_name"] = joint_prim.GetName()
    settings_dict["joint_type"] = joint_prim.GetTypeName()
    settings_dict["parent"] = joint_prim.GetAttribute("physics:body0")
    settings_dict["child"] = joint_prim.GetAttribute("physics:body1")

    # TODO: joints may or may not have the attributes below
    settings_dict["axis"] = joint_prim.GetAttribute("physics:axis")

    settings_dict["break_force"] = joint_prim.GetAttribute("physics:breakForce")
    settings_dict["break_torque"] = joint_prim.GetAttribute("physics:breakTorque")
    settings_dict["lower_limit"] = joint_prim.GetAttribute("physics:lowerLimit")
    settings_dict["upper_limit"] = joint_prim.GetAttribute("physics:upperLimit")
    # drive settings
    if joint_prim.GetTypeName() == "PhysicsPrismaticJoint":
        actuator_type = "linear"
    elif joint_prim.GetTypeName() == "PhysicsRevoluteJoint":
        actuator_type = "angular"
    else:
        # joint type not supported with drive
        return None
    settings_dict["drive_type"] = joint_prim.GetAttribute(f"drive:{actuator_type}:physics:type")
    settings_dict["max_force"] = joint_prim.GetAttribute(f"drive:{actuator_type}:physics:maxForce")
    settings_dict["target_velocity"] = joint_prim.GetAttribute(f"drive:{actuator_type}:physics:targetVelocity")
    settings_dict["target_position"] = joint_prim.GetAttribute(f"drive:{actuator_type}:physics:targetPosition")
    settings_dict["damping"] = joint_prim.GetAttribute(f"drive:{actuator_type}:physics:damping")
    settings_dict["stiffness"] = joint_prim.GetAttribute(f"drive:{actuator_type}:physics:stiffness")

    return settings_dict


def apply_joint_apis(robot_path: str) -> None:
    """Apply the APIs to the joint (joint state, robot joint, articulation root).

    Args:
        robot_path: USD path of the robot to apply joint APIs to.
    """
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_path)
    robot_name = robot_prim.GetName()
    # joint state api on every joint and
    joint_scope_prim = stage.GetPrimAtPath(f"/{robot_name}/Joints")

    for joint_prim in joint_scope_prim.GetChildren():
        # apply joint state api
        joint_type = joint_prim.GetTypeName()
        actuator_type = None
        if joint_type == "PhysicsPrismaticJoint":
            actuator_type = "linear"
        elif joint_type == "PhysicsRevoluteJoint":
            actuator_type = "angular"
        else:
            # joint type not supported with joint state
            continue

        if actuator_type is not None:
            PhysxSchema.JointStateAPI.Apply(joint_prim, actuator_type)
