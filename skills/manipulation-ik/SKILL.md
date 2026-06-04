---
name: manipulation-ik
description: >
  Robot manipulation in Isaac Sim 6 / Kit 110: differential inverse kinematics
  via `Articulation` + `get_jacobian_matrices`, schema-native IK via
  `isaacsim.robot.poser.RobotPoser` (with named-pose storage / replay), grasp
  frames, `FixedJoint` and `SurfaceGripper` grasping, hybrid IK + joint-space
  control, and obstacle-aware motion via `isaacsim.robot_motion.cumotion`
  (`RmpFlowController`) or `isaacsim.robot_motion.pink` (`PinkIKController`).
  Use when controlling a robot arm to pick / place / follow, implementing
  IK-based end-effector control, storing or applying named poses, choosing
  between native IK / cuMotion / PINK / Lula, or validating pick-and-place.
---

# Manipulation IK

Patterns reference Isaac Sim docs and local example files; embedded code is a pattern sketch, not the canonical source. Always read the linked example; upstream code reflects the installed Isaac Sim version.

## When to use

- Control an articulated arm to reach, grasp, transport, place.
- Set up IK-based end-effector control (vs joint-space).
- Store reusable robot poses as named poses and apply them later.
- Set up grasping (FixedJoint, `SurfaceGripper`, contact-based).
- Validate manipulation success with a feedback loop.

## Pick the right IK stack

| Stack | Module | When |
|---|---|---|
| Differential IK on `Articulation` | `isaacsim.core.experimental.prims.Articulation` + per-robot wrapper | direct end-effector control; matches `isaacsim.robot.experimental.manipulators.examples.*` |
| Schema-native IK + named poses | `isaacsim.robot.poser.RobotPoser` (LM solver via `usd.schema.isaac.robot_schema.IKSolverRegistry`) | offline pose authoring, persisted "pick_position" / "approach" poses |
| Obstacle-aware reactive | `isaacsim.robot_motion.cumotion.RmpFlowController` (+ YAML configs in `robot_configurations/`) | dynamic obstacle avoidance, reactive trajectories |
| Pinocchio / PINK | `isaacsim.robot_motion.pink.PinkIKController` | alternative full IK stack with joint limits / task hierarchies |
| Lula motion generation | `isaacsim.robot_motion.lula` + `isaacsim.robot_motion.motion_generation` | legacy; supported but use one of the above for new work ([rename map](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html)) |

## Local example files (canonical source)

Relative to `$ISAAC_SIM_DIR/source/standalone_examples/api/isaacsim.robot.experimental.manipulators/`:

| Topic | Path |
|---|---|
| Differential IK (UR10 follow-target, `--ik-method`) | `universal_robots/follow_target_with_ik.py` |
| RMP flow (UR10) | `universal_robots/follow_target_with_rmpflow.py` |
| RMP flow (Franka) | `franka/follow_target_with_rmpflow.py` |
| Pick & place (Franka) | `franka/pick_place.py` |
| Stacking (Franka) | `franka/stacking.py` |
| Multi-task | `franka/multiple_tasks.py` |
| UR10 stacking | `universal_robots/stacking.py` |

Robot-side implementations: `source/extensions/isaacsim.robot.experimental.manipulators.examples/isaacsim/robot/experimental/manipulators/examples/{franka,universal_robots}/*.py` (e.g. `UR10`, `Franka` classes — `differential_inverse_kinematics`, `set_end_effector_pose`, `reset_to_default_pose`).

Legacy/deprecated examples under `$ISAAC_SIM_DIR/source/standalone_examples/deprecated/api/isaacsim.robot.manipulators/`.

> **Migration:** for the `omni.isaac.franka` / `omni.isaac.universal_robots` / `omni.isaac.manipulators` → `isaacsim.robot.manipulators*` rename map (including the experimental examples extension), see [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html).

## Docs references

| Topic | URL |
|---|---|
| Pick-and-place tutorial | https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/tutorial_pickplace_example.html |
| Setup a manipulator (import / assemble) | https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/tutorial_import_assemble_manipulator.html |
| Physics fundamentals (joints, schemas) | https://docs.isaacsim.omniverse.nvidia.com/latest/physics/simulation_fundamentals.html |
| Python scripting index | https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/index.html |

## Differential IK pattern (modern `Articulation`)

Pattern source: `UR10.differential_inverse_kinematics` (UR wrapper) + `UR10FollowTarget.move_to_target`. The wrapper does:

1. Get the Jacobian: `self.get_jacobian_matrices()` (shape includes a virtual base for fixed-base robots; always slice past the base DOFs).
2. Compute the 6-DOF pose error from current EE pose to goal.
3. Apply the chosen solver to map error -> joint delta:
   - `damped-least-squares`: `dq = J^T (J J^T + lambda^2 I)^-1 . error` (default).
   - `pseudoinverse`, `transpose`, `singular-value-decomposition` also available.
4. Push as `set_dof_position_targets(current + dq, dof_indices=arm_dofs)`.

```python
# Conceptual sketch — see ur10.py and follow_target_with_ik.py for the live code
from isaacsim.core.experimental.prims import Articulation, RigidPrim

arm = MyArm("/World/robot", create_robot=True)
ee  = arm.end_effector_link
J   = arm.get_jacobian_matrices().numpy()[:, arm.end_effector_link_index - 1, :, :arm_dofs]
cur_pos, cur_q = ee.get_world_poses()
dq = arm.differential_inverse_kinematics(
    jacobian_end_effector=J,
    current_position=cur_pos.numpy(),
    current_orientation=cur_q.numpy(),
    goal_position=target_pos,
    goal_orientation=target_quat,
    method="damped-least-squares",
    method_cfg={"scale": 1.0, "damping": 0.05, "min_singular_value": 1e-5},
)
arm.set_dof_position_targets(cur_dofs[:, :arm_dofs] + dq, dof_indices=list(range(arm_dofs)))
```

Tuning (start conservative, increase after stability):

| Parameter | Conservative | Moderate | Aggressive |
|---|---|---|---|
| `damping` | 0.1 | 0.05 | 0.01 |
| `max_delta` per step | 0.02 rad | 0.05 rad | 0.10 rad |
| Drive `stiffness` | 200 | 400 | 800 |

Aggressive settings cause PhysX divergence under payload.

### Hybrid IK + joint-space (arms with < 6 DOF)

Pure differential IK on under-actuated arms fails on:
- Large lateral transport with payload.
- Configurations near kinematic singularities.
- Sweeping through joint limits.

Pattern: IK for precision (approach, descent, final placement), joint-space interpolation for long transport (lift, traverse, descend). See `franka/pick_place.py` for the sequenced version Isaac Sim ships.

## Schema-native IK + Named Poses (RobotPoser)

For pose authoring, persistence, and replay use `isaacsim.robot.poser`. It wraps a kinematic chain (from `usd.schema.isaac.robot_schema`) and provides IK plus a named-pose library stored as `IsaacNamedPose` prims on the robot.

```python
from isaacsim.robot.poser import (
    RobotPoser, Transform,
    store_named_pose, apply_pose_by_name,
    list_named_poses, delete_named_pose,
    export_poses, import_poses,
    validate_robot_schema,
)

# Robot must carry IsaacRobotAPI (applied during URDF/MJCF import).
validate_robot_schema(stage, robot_prim)

poser = RobotPoser(stage, robot_prim, start_prim, end_prim)
target = Transform(position=[0.5, 0.2, 0.8], orientation=[1, 0, 0, 0])
result = poser.solve_ik(target)
if result.success:
    poser.apply_pose(result.joints)
    store_named_pose(stage, robot_prim, "pick_position", result)

# Later:
apply_pose_by_name(stage, robot_prim, "pick_position")
export_poses(stage, robot_prim, "/path/to/poses.json")
```

Standalone helpers (no `RobotPoser` needed) for FK / DOF target application:

```python
from isaacsim.robot.poser import apply_joint_state, apply_joint_state_anchored
apply_joint_state(stage, robot_prim, joint_values)          # FK off-sim / DOF targets when playing
apply_joint_state_anchored(stage, robot_prim, joint_values, # keep anchor at world pose
                           anchor_prim=base_link_prim)
```

The IK solver is pluggable via `usd.schema.isaac.robot_schema.IKSolverRegistry`; the bundled LM solver (`robot_schema.lm_ik`) is the default.

## Obstacle-aware motion (cuMotion / PINK)

Use the documented loaders for both stacks; do not hand-construct configs. Read the full tutorials before extending:

- [cuMotion RMP flow tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/cumotion/tutorial_rmpflow.html) -> `docs/isaacsim/cumotion/tutorial_rmpflow.rst`
- [cuMotion world interface](https://docs.isaacsim.omniverse.nvidia.com/latest/cumotion/tutorial_world_interface.html)
- [cuMotion robot configuration](https://docs.isaacsim.omniverse.nvidia.com/latest/cumotion/tutorial_robot_configuration.html)
- [PINK IK controller tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/pink/tutorial_ik_controller.html) -> `docs/isaacsim/pink/tutorial_ik_controller.rst`
- [PINK robot configuration](https://docs.isaacsim.omniverse.nvidia.com/latest/pink/tutorial_robot_configuration.html)

### cuMotion (RMP flow)

```python
from isaacsim.robot_motion.cumotion import (
    RmpFlowController, CumotionWorldInterface, load_cumotion_supported_robot,
)
from isaacsim.robot_motion.motion_generation import RobotState

robot = load_cumotion_supported_robot("franka")           # config + chain
world = CumotionWorldInterface(...)                        # populate obstacles
ctrl  = RmpFlowController(
    name="rmp_franka",
    robot=robot,
    world_interface=world,
)

# Required: reset() before the first forward() each episode.
ctrl.reset(RobotState(...))                                # estimated current state
action = ctrl.forward(
    estimated_state=RobotState(...),
    setpoint_state=RobotState(...),                        # target EE pose
)
```

### PINK (Pinocchio-based IK)

```python
from isaacsim.robot_motion.pink import PinkIKController, load_pink_supported_robot

pink_robot = load_pink_supported_robot("franka")           # PinkRobot wrapper
controller = PinkIKController(
    name="pink_franka",
    robot=pink_robot,
    tasks=[...],                                            # frame/joint/posture tasks
)
```

Use `load_pink_robot(...)` when authoring a non-bundled robot. See the `pink/tutorial_robot_configuration.rst` walkthrough.

### Grasps

`isaacsim.replicator.grasping` (`GraspingManager`, `GraspPhase`) drives grasp-dataset workflows and grasp-pose generation; see `source/standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py`.

## Grasp frame discovery (do this first)

Most assets do not ship with a grasp frame. Before any IK:

1. Inspect the gripper USD; find the frame at the closed-finger center.
2. If absent, add a child Xform of the gripper link positioned at the grasp center; mark it with `IsaacSiteAPI` (`ApplySiteAPI` from `robot_schema`) so downstream tools recognize it.
3. Use that site as the IK target. The goal pose is where the *object center* sits when grasped, not where the gripper body is.

## Grasping

### `FixedJoint` (simple, reliable for rigid grasps)

Pattern source: `franka/pick_place.py` plus `UsdPhysics.FixedJoint`. Always compute the gripper -> object relative transform at the moment of contact; never hardcode the offset. Hardcoded offsets + high stiffness produce PhysX snap and explosion.

### `SurfaceGripper` (vacuum / magnetic, used by UR10 example)

Pattern source: `UR10` (`open_gripper`, `close_gripper`).

```python
from isaacsim.robot.surface_gripper import _surface_gripper as surface_gripper

iface = surface_gripper.acquire_surface_gripper_interface()
gripper_path = f"{end_effector_path}/SurfaceGripper"
iface.close_gripper(gripper_path)   # attach
iface.open_gripper(gripper_path)    # release
status = iface.get_gripper_status(gripper_path)  # GripperStatus.{Open,Closed}
```

Authored on the robot via `usd.schema.isaac.robot_schema.CreateSurfaceGripper`.

### Grasp dataset workflow

For generating grasp datasets, see `isaacsim.replicator.grasping` (`GraspingManager`, `GraspPhase`) and `source/standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py`.

## Grasp validation (feedback loop)

After executing a pick-and-place, validate at three checkpoints:

| Checkpoint | Check | Failure action |
|---|---|---|
| Grasp contact | fingers visually around object; object within ~2 cm of grasp frame | adjust grasp frame offset or IK target |
| Lift success | object Z rises with the gripper, not left behind | `FixedJoint` missing or wrong offset; gripper not engaged |
| Place success | object resting on target within ~5 cm | transport trajectory missed target |

Smooth motion is necessary but not sufficient. Do not declare success unless all three pass.

## Rules

1. Read the local example first; this skill describes patterns, not syntax.
2. Always create or identify a grasp frame (`IsaacSiteAPI`) before IK.
3. Start conservative with IK gains; increase only after confirming stability.
4. The URDF importer applies `PhysxArticulationAPI` automatically; if you author articulations manually, apply it on the base link.
5. Run standalone scripts with `$ISAAC_SIM_DIR/python.sh`, not `isaaclab.sh -p`, when using `SimulationApp` directly.
6. Jacobian column layout: `[virtual base DOFs | real DOFs]` for fixed-base robots. Always slice past the virtual base.
7. Store reusable poses with `store_named_pose`; do not re-solve IK from scratch every session.
8. `print()` is unreliable in headless mode; use file writes for debug logging.
9. Validate visually at every phase. Smooth motion is not successful manipulation.
10. Hybrid IK + joint-space is the pragmatic default for arms with < 6 DOF.

## Lessons (2026-04-08)

- SO-101 5-DOF: pure DLS IK converges for local moves (~0.008 m error) but diverges on lateral transport under load. Hybrid is required.
- `FixedJoint` with hardcoded offset + high stiffness causes PhysX snap and explosion. Compute the offset at grasp time.
- Jacobian virtual-base offset: easy to miss; breaks IK silently. Always slice past the base.
