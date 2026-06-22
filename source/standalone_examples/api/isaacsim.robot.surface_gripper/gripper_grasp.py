# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Surface Gripper Object Grasping Example

Scene:
    - Ground plane.
    - A static "carrier" rigid body (kinematic, fixed in space) that anchors the
      attachment joint.
    - A dynamic gripper body (gravity disabled), connected to the carrier by a
      prismatic Z joint with a strong linear drive. We move the gripper by
      setting the prismatic drive's target position -- this is the same
      mechanism the SurfaceGripper gantry example uses.
    - A D6 joint between the gripper body and the carrier carries the
      IsaacAttachmentPointAPI; the SurfaceGripper prim references it as its
      attachment point.
    - A dynamic target cube resting on the ground.

The script does, in order:
    1. Gripper starts Open with no gripped objects.
    2. Lower the gripper, Close it -> status becomes Closed and the cube appears
       in gripped_objects.
    3. Lift the gripper -> the cube follows (its world Z rises clearly above
       its initial resting height).
    4. Open the gripper -> status returns to Open, gripped_objects is empty,
       and the released cube falls back toward the ground.
"""

import argparse
import sys
from typing import Any

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true", help="Run in test mode")
parser.add_argument("--headless", action="store_true", help="Run without a viewport")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless or args.test})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.robot.surface_gripper import GripperView, _surface_gripper
from pxr import Gf, UsdPhysics
from usd.schema.isaac import robot_schema

GripperStatus = _surface_gripper.GripperStatus
Vec3 = tuple[float, float, float]

GRIPPER_PRIM_PATH = "/World/SurfaceGripper"
GRIPPER_JOINTS_SCOPE = "/World/Surface_Gripper_Joints"
ATTACH_JOINT_PATH = f"{GRIPPER_JOINTS_SCOPE}/AttachJoint"
DRIVE_JOINT_PATH = "/World/Joints/PrismaticZ"
CARRIER_PATH = "/World/Carrier"
GRIPPER_BODY_PATH = "/World/GripperBody"
CUBE_PATH = "/World/TargetCube"

CUBE_SIZE = 0.04
CUBE_REST_Z = CUBE_SIZE / 2.0
GRIPPER_SIZE = 0.04
CARRIER_SIZE = 0.04

# Carrier is a kinematic body fixed at the top of the scene; it serves as the
# anchor for the prismatic drive and the D6 attach joint.
CARRIER_Z = 0.40
GRIPPER_INIT_Z = 0.20
# Prismatic-drive target Z is interpreted as the position of the gripper body
# along the joint axis relative to its joint frame; we just set it to the
# desired world Z of the gripper body since the joint's local frame on the
# carrier sits at the carrier's center.
GRIPPER_GRASP_Z = CUBE_SIZE + GRIPPER_SIZE / 2.0 + 0.005  # ~46 mm
GRIPPER_LIFT_Z = 0.30


def _make_cube_geom(path: str, size: float, position: Vec3, color: Vec3) -> Any:
    # Create the cube with the core (experimental) Cube helper and apply collision
    # via GeomPrim. Cube sets the geom's native ``size`` attribute (rather than a
    # scale op), which the surface gripper's scene-query raycast relies on -- it
    # does not reliably hit a scaled box collider.
    Cube(paths=path, sizes=size, positions=[position], colors=[color])
    GeomPrim(paths=path, apply_collision_apis=True)
    return prim_utils.get_prim_at_path(path)


def _make_kinematic_cube(path: str, size: float, position: Vec3, color: Vec3) -> Any:
    prim = _make_cube_geom(path, size, position, color)
    rigid_prim = RigidPrim(path)
    prim_utils.ensure_api(prim, UsdPhysics.RigidBodyAPI).GetKinematicEnabledAttr().Set(True)
    rigid_prim.set_enabled_gravities([False])
    return prim


def _make_dynamic_cube(
    path: str,
    size: float,
    position: Vec3,
    color: Vec3,
    mass: float,
    *,
    disable_gravity: bool = False,
) -> RigidPrim:
    """Create a dynamic rigid cube and wrap it in a RigidPrim for pose readback."""
    _make_cube_geom(path, size, position, color)
    # RigidPrim applies the rigid-body APIs and sets the mass and gravity state.
    rigid_prim = RigidPrim(path, masses=[mass])
    if disable_gravity:
        rigid_prim.set_enabled_gravities([False])
    return rigid_prim


def build_scene(stage: Any) -> tuple[RigidPrim, RigidPrim]:
    stage_utils.set_stage_up_axis("Z")
    stage_utils.define_prim("/physicsScene", "PhysicsScene")
    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)

    # Static carrier (kinematic, never moved). Anchors all joints.
    _make_kinematic_cube(
        CARRIER_PATH,
        size=CARRIER_SIZE,
        position=(0.0, 0.0, CARRIER_Z),
        color=(0.9, 0.6, 0.0),
    )

    # Dynamic gripper body, gravity disabled.
    gripper_body = _make_dynamic_cube(
        GRIPPER_BODY_PATH,
        size=GRIPPER_SIZE,
        position=(0.0, 0.0, GRIPPER_INIT_Z),
        color=(0.6, 0.6, 0.6),
        mass=0.05,
        disable_gravity=True,
    )

    # Target dynamic cube to grasp.
    cube = _make_dynamic_cube(
        CUBE_PATH,
        size=CUBE_SIZE,
        position=(0.0, 0.0, CUBE_REST_Z),
        color=(0.0, 0.4, 1.0),
        mass=0.05,
    )

    # Prismatic joint between carrier (body0) and gripper body (body1) along Z,
    # with a strong linear drive used to move the gripper up/down.
    stage_utils.define_prim("/World/Joints", "Scope")
    pris = UsdPhysics.PrismaticJoint(stage_utils.define_prim(DRIVE_JOINT_PATH, "PhysicsPrismaticJoint"))
    pris.CreateBody0Rel().SetTargets([CARRIER_PATH])
    pris.CreateBody1Rel().SetTargets([GRIPPER_BODY_PATH])
    pris.CreateAxisAttr("Z")
    # Local frames: carrier-side at carrier center; gripper-side at gripper
    # center. Drive target = (gripper_z - carrier_z).
    pris.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    pris.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pris.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    pris.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pris.CreateLowerLimitAttr(-1.0)
    pris.CreateUpperLimitAttr(1.0)
    pris.CreateBreakForceAttr().Set(3.4028235e38)
    pris.CreateBreakTorqueAttr().Set(3.4028235e38)

    # Strong linear drive so the gripper tracks the target precisely.
    drive = UsdPhysics.DriveAPI.Apply(pris.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateMaxForceAttr(1.0e6)
    drive.CreateTargetPositionAttr(GRIPPER_INIT_Z - CARRIER_Z)
    drive.CreateStiffnessAttr(1.0e6)
    drive.CreateDampingAttr(1.0e4)

    # <start-gripper-grasp-setup-snippet>
    # Attachment-point joint: D6 between gripper body (body0) and carrier
    # (body1). This joint's only purpose is to mark the surface-gripper
    # attachment pose -- the prismatic above does the structural work.
    stage_utils.define_prim(GRIPPER_JOINTS_SCOPE, "Scope")
    attach = UsdPhysics.Joint(stage_utils.define_prim(ATTACH_JOINT_PATH, "PhysicsJoint"))
    attach.CreateBody0Rel().SetTargets([GRIPPER_BODY_PATH])
    attach.CreateBody1Rel().SetTargets([CARRIER_PATH])

    # 180-deg flip about X so the joint's local Z points DOWN (toward cube).
    flip_x = Gf.Quatf(0.0, 1.0, 0.0, 0.0)
    attach.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -GRIPPER_SIZE / 2.0))
    attach.CreateLocalRot0Attr().Set(flip_x)
    # body1 (carrier) local pos doesn't really matter for the attachment, just
    # make it sensible relative to where the gripper sits initially.
    attach.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, GRIPPER_INIT_Z - CARRIER_Z - GRIPPER_SIZE / 2.0))
    attach.CreateLocalRot1Attr().Set(flip_x)
    attach.CreateExcludeFromArticulationAttr().Set(True)
    attach_prim = attach.GetPrim()

    # Lock all 6 DOFs (low > high == locked). This is critical: when the
    # surface gripper closes, it re-points body1 of THIS joint at the gripped
    # object, so the joint's DOF locks become the gripper-to-object constraint.
    for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
        limit = UsdPhysics.LimitAPI.Apply(attach_prim, axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)

    robot_schema.ApplyAttachmentPointAPI(attach_prim)
    prim_utils.create_prim_attribute(
        attach_prim,
        name=robot_schema.Attributes.FORWARD_AXIS.name,
        type_name=robot_schema.Attributes.FORWARD_AXIS.type,
    ).Set("Z")
    prim_utils.create_prim_attribute(
        attach_prim,
        name=robot_schema.Attributes.CLEARANCE_OFFSET.name,
        type_name=robot_schema.Attributes.CLEARANCE_OFFSET.type,
    ).Set(0.005)

    surface_gripper_prim = robot_schema.CreateSurfaceGripper(stage, GRIPPER_PRIM_PATH)
    surface_gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets([ATTACH_JOINT_PATH])
    # <end-gripper-grasp-setup-snippet>

    return gripper_body, cube


def step_for(frames: int) -> None:
    # With the timeline playing, each app update advances physics by one dt.
    for _ in range(frames):
        simulation_app.update()


def _world_z(rigid_prim: RigidPrim) -> float:
    positions, _ = rigid_prim.get_world_poses()
    return float(positions.numpy()[0, 2])


def set_gripper_target_z(target_world_z: float) -> None:
    """Drive the gripper body to a world Z by setting the prismatic drive target."""
    target_attr = prim_utils.get_prim_at_path(DRIVE_JOINT_PATH).GetAttribute("drive:linear:physics:targetPosition")
    target_attr.Set(target_world_z - CARRIER_Z)


def main() -> int:
    stage = stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    gripper_body, cube = build_scene(stage)

    # <start-gripper-grasp-view-snippet>
    # Mirror gripper state (status, gripped objects) back to USD, then wrap the
    # SurfaceGripper prim in a GripperView to drive it from Python.
    _surface_gripper.acquire_surface_gripper_interface().set_write_to_usd(True)
    gripper_view = GripperView(
        paths=GRIPPER_PRIM_PATH,
        max_grip_distance=[0.02],
        coaxial_force_limit=[100.0],
        shear_force_limit=[100.0],
        retry_interval=[1.0],
    )
    # Positive action closes/grips, negative opens/releases:
    #   gripper_view.apply_gripper_action([0.5])   # close
    #   gripper_view.apply_gripper_action([-0.5])  # open
    # <end-gripper-grasp-view-snippet>

    app_utils.play()
    simulation_app.update()

    step_for(60)

    failures: list[str] = []

    # 1) Initial state.
    status = GripperStatus(gripper_view.get_surface_gripper_status()[0])
    gripped = gripper_view.get_gripped_objects()[0]
    cube_initial_z = _world_z(cube)
    print(f"[init]  status={status.name}, gripped={gripped}")
    print(f"[init]  cube z = {cube_initial_z:.4f}, gripper z = {_world_z(gripper_body):.4f}")
    if status != GripperStatus.Open:
        failures.append(f"init: expected Open, got {status.name}")
    if len(gripped) != 0:
        failures.append(f"init: expected empty grip, got {gripped}")

    # 2) Lower the gripper toward the cube, then close.
    set_gripper_target_z(GRIPPER_GRASP_Z)
    step_for(90)
    print(f"[lower] gripper z = {_world_z(gripper_body):.4f}")

    gripper_view.apply_gripper_action([0.5])
    step_for(60)
    status = GripperStatus(gripper_view.get_surface_gripper_status()[0])
    gripped = gripper_view.get_gripped_objects()[0]
    print(f"[close] status={status.name}, gripped={gripped}, gripper z={_world_z(gripper_body):.4f}")
    if status != GripperStatus.Closed:
        failures.append(f"close: expected Closed, got {status.name}")
    if CUBE_PATH not in gripped:
        failures.append(f"close: expected {CUBE_PATH} in gripped, got {gripped}")

    # 3) Lift the gripper; the cube should follow.
    for z in np.linspace(GRIPPER_GRASP_Z, GRIPPER_LIFT_Z, 30):
        set_gripper_target_z(float(z))
        simulation_app.update()
    step_for(60)

    cube_lifted_z = _world_z(cube)
    print(f"[lift]  cube z = {cube_lifted_z:.4f}, gripper z = {_world_z(gripper_body):.4f}")
    if cube_lifted_z < cube_initial_z + 0.10:
        failures.append(f"lift: cube did not rise (initial={cube_initial_z:.4f}, lifted={cube_lifted_z:.4f})")
    status = GripperStatus(gripper_view.get_surface_gripper_status()[0])
    gripped = gripper_view.get_gripped_objects()[0]
    if status != GripperStatus.Closed or CUBE_PATH not in gripped:
        failures.append(f"lift: lost grip (status={status.name}, gripped={gripped})")

    # 4) Open -> release.
    gripper_view.apply_gripper_action([-0.5])
    step_for(5)
    status = GripperStatus(gripper_view.get_surface_gripper_status()[0])
    gripped = gripper_view.get_gripped_objects()[0]
    print(f"[open]  status={status.name}, gripped={gripped}")
    if status != GripperStatus.Open:
        failures.append(f"open: expected Open, got {status.name}")
    if len(gripped) != 0:
        failures.append(f"open: expected empty grip, got {gripped}")

    step_for(60)
    cube_dropped_z = _world_z(cube)
    print(f"[drop]  cube z = {cube_dropped_z:.4f}")
    if cube_dropped_z > cube_lifted_z - 0.05:
        failures.append(f"drop: cube did not fall (lifted={cube_lifted_z:.4f}, after_open={cube_dropped_z:.4f})")

    if not (args.headless or args.test):
        step_for(60)

    if failures:
        print("\nFAIL:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nPASS: surface gripper open/close/grasp verified.")
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    except Exception:
        import traceback

        traceback.print_exc()
        rc = 1
    except KeyboardInterrupt:
        print("\nExiting...")
        rc = 1
    finally:
        simulation_app.close()
    sys.exit(rc)
