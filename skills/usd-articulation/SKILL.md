---
name: usd-articulation
description: "Validate and build multi-arm articulations in USD. Use when assembling or fixing robot joint hierarchies."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# USD Articulation — Multi-Arm Robots

## Purpose

Assemble and validate multi-link robot articulations with ArticulationRootAPI, fixed joints, and Isaac robot schema overlays before deployment.

## Prerequisites

- Built Isaac Sim (`$ISAAC_SIM_DIR` or `_build/linux-x86_64/release`).
- NVIDIA GPU with a current driver (`nvidia-smi`).
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR`.

## Limitations

- Targets Isaac Sim 6 / Kit 110 unless a section states otherwise.
- Does not replace official NVIDIA documentation for unsupported edge cases.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Extension or import not found | Wrong `$ISAAC_SIM_DIR` or stale build | Point env vars at `_build/linux-x86_64/release` or rebuild |
| Black or empty frames | Missing lights or non-RTX render mode | Add dome/key light; confirm RTX / PathTracing settings |
| Hang on stage load or first render | MDL compile or oversized stage | Follow isolation steps in `isaac-sim-troubleshooting` |

Two layers stack on a robot USD:

1. **Physics**: `UsdPhysics.ArticulationRootAPI`, `UsdPhysics.Joint`, `UsdPhysics.RigidBodyAPI`, `PhysxSchema.PhysxArticulationAPI`.
2. **Robot Schema** (`usd.schema.isaac.robot_schema`): semantic overlay used by `isaacsim.robot.poser`, the importers, manipulators examples, and any tool that walks "this robot's links / joints / named poses".

Rule: visual plausibility is not mechanical correctness. If it doesn't articulate, it doesn't exist.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/apply_robot_schema.py` | Apply Isaac Robot Schema overlay to an existing USD articulation | see script --help |
| `scripts/multi_arm_assembly.py` | Assemble a multi-arm robot in USD using FixedJoints | see script --help |
| `scripts/validate_articulation.py` | Validate multi-arm robot USD articulation structure | CLI args via sys.argv (see script --help) |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/apply_robot_schema.py", args=["--help"])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`, or execute shell helpers directly when they do not require the simulator.

## Multi-arm assembly (correct pattern)

`assemble_multi_arm_robot(stage, chassis_usd, arm_usd, robot_root, arm_offset, chassis_body_path, arm_base_path)` — spawn chassis + arm, create a FixedJoint, remove ArticulationRootAPI from the arm.

See [`scripts/multi_arm_assembly.py`](scripts/multi_arm_assembly.py).



### What does not work

- Referencing the same arm USD twice via sublayer composition.
- Using `over "Geometry"` to add arms in a parts layer.
- Assuming visual presence equals physical attachment.

## Robot Schema overlay (Kit 110)

Apply the modern Isaac Robot Schema on top of the physics layer. The URDF/MJCF importers do this automatically; for hand-authored or retrofitted USDs apply it manually.

`apply_robot_schema(stage, robot_prim, link_prims, joint_prims, site_prims, robot_type)` — apply `IsaacRobotAPI` on root, `IsaacLinkAPI` on each link, `IsaacJointAPI` on each joint, `IsaacSiteAPI` on grasp/mount frames, then call `PopulateRobotSchemaFromArticulation` to fill `ROBOT_LINKS`/`ROBOT_JOINTS`. Valid `robot_type` tokens: Default, End Effector, Manipulator, Humanoid, Wheeled, Holonomic, Quadruped, Mobile Manipulators, Aerial.

See [`scripts/apply_robot_schema.py`](scripts/apply_robot_schema.py).

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
2. At least one chassis/root-branch `FixedJoint` anchors each top-level attached subsystem before flatten-before-deploy.
3. Every joint body is reachable from the single articulation root through one connected joint hierarchy.
4. `IsaacRobotAPI` present on the root; `robot_type` set to a valid token.
5. `ROBOT_LINKS` / `ROBOT_JOINTS` relations populated (call `PopulateRobotSchemaFromArticulation` if not).
6. Each grasp frame carries `IsaacSiteAPI` (not deprecated `IsaacReferencePointAPI`).
7. Render from four angles (front, side, 3/4, top).
8. Send to a vision LLM with a binary question ("Arms attached? PASS/FAIL").

## Validation snippet (self-contained)

Run via `$ISAAC_SIM_DIR/python.sh`. Enforces exactly one `ArticulationRootAPI`, verifies the entire joint graph is connected to that root, checks for at least one root-branch `FixedJoint` attachment before flatten-before-deploy, and reports the Robot Schema state.

`validate_articulation(usd_path)` — assert exactly 1 ArticulationRootAPI, verify hierarchy connectivity, and print root-attached children plus Robot Schema state.

See [`scripts/validate_articulation.py`](scripts/validate_articulation.py).

Extend with whatever subsystem path patterns your asset uses. Binary goal: exactly one root, one connected articulated hierarchy, at least one root-branch `FixedJoint` anchor before flattening, and the Robot Schema overlay present.


## Common failures

| Symptom | Cause | Fix |
|---|---|---|
| Arms render but float | No `FixedJoint` to chassis | add `FixedJoint` |
| Multiple articulation roots | Arm USD has its own root | `RemoveAPI(UsdPhysics.ArticulationRootAPI)` from arms |
| Training works but arms independent | Separate articulation trees | single root + `FixedJoint`s |
| `RobotPoser.solve_ik()` errors out | missing `IsaacRobotAPI` / link relations | `ApplyRobotAPI` + `PopulateRobotSchemaFromArticulation` |
| Importer applied `IsaacReferencePointAPI` | older asset | re-import with current Isaac Sim, or migrate to `IsaacSiteAPI` (deprecation warning) |
| `robot_type` attribute value rejected | typo or stale token | pick from `get_allowed_tokens(Attributes.ROBOT_TYPE)` |
