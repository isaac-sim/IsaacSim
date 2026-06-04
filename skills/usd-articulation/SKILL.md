---
name: usd-articulation
description: >
  Validate and build multi-arm robot articulations in USD for Isaac Sim 6 / Lab.
  Covers programmatic assembly (chassis + arms + `FixedJoint`s),
  `ArticulationRootAPI` placement, the modern Isaac Robot Schema overlay
  (`IsaacRobotAPI`, `IsaacLinkAPI`, `IsaacJointAPI`, `IsaacSiteAPI`,
  `IsaacNamedPose`, `KinematicChain`), `PopulateRobotSchemaFromArticulation`
  for retrofitting existing USDs, flatten-before-deploy, and the validation
  checklist (articulation roots, joint connections, robot-schema integrity,
  floating arms).
  Use when creating multi-limb robots in USD, retrofitting an existing
  articulation with the Robot Schema, debugging "floating arms" or
  disconnected geometry, or flattening articulations for deployment to
  Isaac Lab.
---

# USD Articulation — Multi-Arm Robots

Two layers stack on a robot USD:

1. **Physics**: `UsdPhysics.ArticulationRootAPI`, `UsdPhysics.Joint`, `UsdPhysics.RigidBodyAPI`, `PhysxSchema.PhysxArticulationAPI`.
2. **Robot Schema** (`usd.schema.isaac.robot_schema`): semantic overlay used by `isaacsim.robot.poser`, the importers, manipulators examples, and any tool that walks "this robot's links / joints / named poses".

Rule: visual plausibility is not mechanical correctness. If it doesn't articulate, it doesn't exist.

## Multi-arm assembly (correct pattern)

```python
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

# Spawn chassis
chassis = stage.DefinePrim("/World/Robot", "Xform")
chassis.GetReferences().AddReference(CHASSIS_USD)

# Spawn arms as children
left = stage.DefinePrim("/World/Robot/LeftArm", "Xform")
left.GetReferences().AddReference(ARM_USD)
UsdGeom.Xformable(left).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.6, 0.5))

# REQUIRED: FixedJoint connecting arm to chassis
joint = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/LeftArmAttachment")
joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/Robot/.../Chassis")])
joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Robot/LeftArm/.../BaseMount")])

# REQUIRED: Remove ArticulationRootAPI from arm (only chassis keeps it)
arm_prim = stage.GetPrimAtPath("/World/Robot/LeftArm/...")
arm_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

```

### What does not work

- Referencing the same arm USD twice via sublayer composition.
- Using `over "Geometry"` to add arms in a parts layer.
- Assuming visual presence equals physical attachment.

## Robot Schema overlay (Kit 110)

Apply the modern Isaac Robot Schema on top of the physics layer. The URDF/MJCF importers do this automatically; for hand-authored or retrofitted USDs apply it manually.

```python
from pxr import Usd
from usd.schema.isaac.robot_schema import (
    Classes, Attributes,
    ApplyRobotAPI, ApplyLinkAPI, ApplyJointAPI, ApplySiteAPI,
    CreateNamedPose, CreateSurfaceGripper,
    PopulateRobotSchemaFromArticulation, GenerateRobotLinkTree, GetAllNamedPoses,
)

stage = Usd.Stage.Open("/path/robot.usd")
robot_prim = stage.GetPrimAtPath("/World/Robot")

# IsaacRobotAPI on the root: stores robot_type, ordered link/joint relations.
ApplyRobotAPI(robot_prim)
robot_prim.GetAttribute(Attributes.ROBOT_TYPE).Set("Mobile Manipulators")
# Or pick from get_allowed_tokens(Attributes.ROBOT_TYPE) — tokens include:
#   Default, End Effector, Manipulator, Humanoid,
#   Wheeled, Holonomic, Quadruped, Mobile Manipulators, Aerial.

# IsaacLinkAPI on each rigid link; IsaacJointAPI on each joint.
for link in (link_prims):  # walk articulated links
    ApplyLinkAPI(link)
for joint in (joint_prims):
    ApplyJointAPI(joint)

# IsaacSiteAPI on grasp / mount / reference frames (replaces deprecated
# IsaacReferencePointAPI).
ApplySiteAPI(grasp_frame_prim)

# One-shot retrofit: populate ordered link/joint graph from an existing
# Articulation (uses physics traversal to fill ROBOT_LINKS / ROBOT_JOINTS).
PopulateRobotSchemaFromArticulation(stage, robot_prim)
```

| Schema | Applied to | Role |
|---|---|---|
| `IsaacRobotAPI` (`Classes.ROBOT_API`) | robot root | `robot_type`, ordered link/joint relations, named-pose container |
| `IsaacLinkAPI` (`Classes.LINK_API`) | each rigid link | mass/visual aux for tools that walk the chain |
| `IsaacJointAPI` (`Classes.JOINT_API`) | each joint | semantic joint metadata |
| `IsaacSiteAPI` (`Classes.SITE_API`) | grasp / mount / reference frames | named frames for IK targets, mounts, sensors |
| `IsaacNamedPose` (`Classes.NAMED_POSE`) | pose container prims | stored joint configurations (see `manipulation-ik`) |
| `Classes.SURFACE_GRIPPER` | end-effector site | author surface gripper via `CreateSurfaceGripper` |
| `Classes.ATTACHMENT_POINT_API` | site or link | attachment points used by accessory tooling |

## Validation checklist

Before trusting any multi-arm robot:

1. Exactly 1 `UsdPhysics.ArticulationRootAPI` on the chassis, nowhere else.
2. `FixedJoint`s connect each arm `BaseMount` to the chassis.
3. Joint count matches: `N` arm joints per arm + chassis joints.
4. `IsaacRobotAPI` present on the root; `robot_type` set to a valid token.
5. `ROBOT_LINKS` / `ROBOT_JOINTS` relations populated (call `PopulateRobotSchemaFromArticulation` if not).
6. Each grasp frame carries `IsaacSiteAPI` (not deprecated `IsaacReferencePointAPI`).
7. Render from four angles (front, side, 3/4, top).
8. Send to a vision LLM with a binary question ("Arms attached? PASS/FAIL").

## Validation snippet (self-contained)

Run via `$ISAAC_SIM_DIR/python.sh`. Enforces exactly one `ArticulationRootAPI`, prints joints and connected bodies, flags arms whose `BaseMount` is not bound to the chassis via a `FixedJoint`, and reports the Robot Schema state.

```python
import sys
from pxr import Usd, UsdPhysics, Sdf
from usd.schema.isaac.robot_schema import Classes, Attributes, GetAllNamedPoses

stage = Usd.Stage.Open(sys.argv[1])

art_roots = [p.GetPath() for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"ArticulationRootAPI count: {len(art_roots)} -- {art_roots}")
assert len(art_roots) == 1, "must be exactly 1 articulation root"

joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
chassis_paths = {str(art_roots[0])}
attached_arms = set()
for j in joints:
    j_api = UsdPhysics.Joint(j)
    b0 = j_api.GetBody0Rel().GetTargets()
    b1 = j_api.GetBody1Rel().GetTargets()
    if j.IsA(UsdPhysics.FixedJoint) and any(str(t).startswith(p) for t in b0 for p in chassis_paths):
        attached_arms.update(str(t) for t in b1)
print(f"Joints: {len(joints)} | FixedJoint-attached children: {sorted(attached_arms)}")

# Robot Schema overlay
robots = [p for p in stage.Traverse() if p.HasAPI(Classes.ROBOT_API)]
print(f"IsaacRobotAPI count: {len(robots)}")
for r in robots:
    rt = r.GetAttribute(Attributes.ROBOT_TYPE).Get()
    poses = GetAllNamedPoses(stage, r)
    print(f"  {r.GetPath()}: robot_type={rt!r}  named_poses={list(poses)}")
```

Extend with whatever arm/leg path patterns your asset uses. Binary goal: exactly one root, every limb connected to the chassis via a `FixedJoint` or articulated joint chain, and the Robot Schema overlay present.


## Common failures

| Symptom | Cause | Fix |
|---|---|---|
| Arms render but float | No `FixedJoint` to chassis | add `FixedJoint` |
| Multiple articulation roots | Arm USD has its own root | `RemoveAPI(UsdPhysics.ArticulationRootAPI)` from arms |
| Training works but arms independent | Separate articulation trees | single root + `FixedJoint`s |
| `RobotPoser.solve_ik()` errors out | missing `IsaacRobotAPI` / link relations | `ApplyRobotAPI` + `PopulateRobotSchemaFromArticulation` |
| Importer applied `IsaacReferencePointAPI` | older asset | re-import with current Isaac Sim, or migrate to `IsaacSiteAPI` (deprecation warning) |
| `robot_type` attribute value rejected | typo or stale token | pick from `get_allowed_tokens(Attributes.ROBOT_TYPE)` |
