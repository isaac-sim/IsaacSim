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

"""Backend of "Save Robot".

- create a file with variants for _base,_physics, and _robot layer
- add background (light and potential floor) and physicsscene outside of the defaultprim in the new usd

"""

import os
from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.usd
import usd.schema.isaac.robot_schema as rs
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from ..utils.utils import apply_standard_stage_settings
from .robot_templates import RobotRegistry


def create_variant_usd(add_ground: bool = False, add_lights: bool = False, add_physics_scene: bool = False) -> None:
    """Creates a variant USD file with multiple physics configurations for the registered robot.

    Generates a master USD file with variant sets containing three physics levels: None, PhysX, and Robot.
    Each variant references separate configuration files (_base.usd, _physics.usd, _robot.usd) and
    optionally adds environment components like ground plane, lighting, and physics scene.

    Args:
        add_ground: Whether to add a ground plane to the environment.
        add_lights: Whether to add a distant light to the environment.
        add_physics_scene: Whether to add a physics scene to the environment.
    """

    def _add_ground(stage: Any) -> None:
        print("Adding ground plane")
        GroundPlane("/Environment/groundPlane")

    def _add_light(stage: Any) -> None:
        print("Adding light")
        light = DistantLight("/Environment/defaultLight")
        light.set_intensities(1000.0)

    def _add_physics_scene(stage: Any) -> None:
        print("Adding physics scene")
        stage_utils.define_prim("/Environment/physicsScene", type_name="PhysicsScene")

    robot = RobotRegistry().get()
    # start a new stage for the variant usd
    stage = Usd.Stage.CreateInMemory()
    apply_standard_stage_settings(stage)
    robot_xform = UsdGeom.Xform.Define(stage, Sdf.Path(f"/{robot.name}"))
    stage.SetDefaultPrim(robot_xform.GetPrim())

    # create a variant set for physics selection
    vs = robot_xform.GetPrim().GetVariantSets().AddVariantSet("Physics")
    for level in ("None", "PhysX", "Robot"):
        vs.AddVariant(level)

    base_filepath = f"{robot.name}_base.usd"
    physics_filepath = f"{robot.name}_physics.usd"
    robot_schema_filepath = f"{robot.name}_robot.usd"

    vs.SetVariantSelection("None")
    with vs.GetVariantEditContext():
        # define a prim to carry the payload
        # addPayload(assetPath, primPath) — primPath optional (defaults to defaultPrim)
        robot_xform.GetPrim().GetPayloads().AddPayload(
            assetPath=f"configurations/{base_filepath}",
        )

    vs.SetVariantSelection("PhysX")
    with vs.GetVariantEditContext():
        # define a prim to carry the payload
        # addPayload(assetPath, primPath) — primPath optional (defaults to defaultPrim)
        robot_xform.GetPrim().GetPayloads().AddPayload(
            assetPath=f"configurations/{physics_filepath}",
        )

    vs.SetVariantSelection("Robot")
    with vs.GetVariantEditContext():
        robot_xform.GetPrim().GetPayloads().AddPayload(
            assetPath=f"configurations/{robot_schema_filepath}",
        )

    # 4) Export master layer
    root_dir = robot.robot_root_folder
    variant_usd_path = os.path.join(root_dir, f"{robot.name}.usd")

    stage.GetRootLayer().Export(variant_usd_path)

    # open the master layer as stage, and add the ground, light and physics scene as needed, save again
    omni.usd.get_context().open_stage(variant_usd_path)
    stage = omni.usd.get_context().get_stage()
    if add_ground:
        _add_ground(stage)
    if add_lights:
        _add_light(stage)
    if add_physics_scene:
        _add_physics_scene(stage)


def apply_articulation_apis(robot_path: str, articulation_root_path: str) -> None:
    """Applies articulation APIs to the specified robot prim for physics simulation.

    Removes any existing articulation root APIs from the robot hierarchy and applies
    UsdPhysics.ArticulationRootAPI and PhysxSchema.PhysxArticulationAPI to the specified
    articulation root prim.

    Args:
        robot_path: Path to the robot prim in the stage.
        articulation_root_path: Path to the prim that should become the articulation root.
            Use "Pick from the Robot" to automatically use the robot prim as root.
    """
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_path)
    robot_name = robot_prim.GetName()
    # if articulation root not given, use the parent robot prim
    if articulation_root_path == "Pick from the Robot":
        articulation_prim = stage.GetPrimAtPath(f"/{robot_name}")
    else:
        articulation_prim = stage.GetPrimAtPath(articulation_root_path)
    # make sure there isn't already an articulation root on stage, if there is, delete it if it's not on the prim desired

    def remove_articulation_root_recursive(prim: Any) -> None:
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        for child in prim.GetChildren():
            remove_articulation_root_recursive(child)

    # delete any previous articulation root prim that might be on the robot
    remove_articulation_root_recursive(robot_prim)
    articulation_prim.ApplyAPI(UsdPhysics.ArticulationRootAPI)
    articulation_prim.ApplyAPI(PhysxSchema.PhysxArticulationAPI)


def apply_robot_schema(robot_path: str) -> None:
    """Applies Isaac Robot Schema to the robot prim and its components.

    Applies RobotAPI to the main robot prim, LinkAPI to all robot links, and JointAPI
    to all joints. Creates relationships between the robot and its links/joints as
    defined by the Isaac Robot Schema.

    Args:
        robot_path: Path to the robot prim in the stage.

    Raises:
        ValueError: If the robot prim does not exist or no robot is found in the registry.
    """
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_path)
    robot = RobotRegistry().get()

    if not robot_prim:
        raise ValueError(f"Robot prim {robot_path} does not exist")
    if not robot:
        raise ValueError("No robot found in Registry")

    # applying robot schema
    robot_name = robot.name
    robot_prim = stage.GetPrimAtPath(robot_path)
    link_prims = [stage.GetPrimAtPath(f"{robot_path}/{link}") for link in robot.links]
    joint_scope_prim = stage.GetPrimAtPath(f"/{robot_name}/Joints")

    # apply the robot api to the robot prim
    rs.ApplyRobotAPI(robot_prim)

    robot_links = robot_prim.GetRelationship(rs.Relations.ROBOT_LINKS.name)
    for link_prim in link_prims:
        rs.ApplyLinkAPI(link_prim)
        robot_links.AddTarget(link_prim.GetPath())

    robot_joints = robot_prim.GetRelationship(rs.Relations.ROBOT_JOINTS.name)
    for joint_prim in joint_scope_prim.GetChildren():
        # apply robot schema joint api and relationship
        rs.ApplyJointAPI(joint_prim)
        robot_joints.AddTarget(joint_prim.GetPath())
