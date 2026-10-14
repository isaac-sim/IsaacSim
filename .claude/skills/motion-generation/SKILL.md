---
name: motion-generation
description: "Build obstacle-aware arm and mobile-base controllers. Primary for UR10 pick-and-place around obstacles with physics/video; use manipulation-ik for obstacle-free grasp tasks."
license: Apache-2.0
metadata:
  author: Aaron Young <info@nvidia.com>
---

# Motion Generation

## Purpose

Build obstacle-aware arm and mobile-base motion with the motion_generation controller substrate; cuMotion with RMPflow is the reference arm implementation.

## Skill selection and invocation

For obstacle-aware arm demos, invoke `motion-generation` as the primary skill
before scene implementation. A requested video does not make the orchestrator
the primary skill. Use `manipulation-ik` for supporting grasp/contact checks and
`isaac-sim-remote` for live execution.

When the agent runtime exposes a native skill invocation tool (for example,
Claude's Skill tool), use it to invoke this skill and the needed supporting
skills. Reading their files with shell commands is supplemental reference
access, not a substitute for that invocation. On runtimes without such a tool,
load the canonical `SKILL.md` through the supported discovery mechanism and
report that native invocation tracing is unavailable; do not fabricate it.

## Prerequisites

- For library implementation and standalone consumers: a completed
  `source/libraries` build or installed matching wheels/SDKs.
- For interactive Kit adapters: built Isaac Sim and the shell environment from
  `isaac-sim-orchestrator`; a GPU is required only by the selected controller or
  rendering workflow.

## Limitations

- Targets Isaac Sim 6 / Kit 110 unless a section states otherwise.
- Does not replace official NVIDIA documentation for unsupported edge cases.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Black or empty frames | Missing lights or non-RTX render mode | Add dome/key light; confirm RTX / PathTracing settings |
| Hang on stage load or first render | MDL compile or oversized stage | Follow isolation steps in `isaac-sim-troubleshooting` |

## Ownership and execution routes

The registered library surfaces for `isaacsim.robot_motion.controllers`,
`isaacsim.robot_motion.cumotion`, and
`isaacsim.robot_motion.experimental.motion_generation` are under
`source/libraries/isaacsim/robot_motion/`. Use their module tests to determine
which behavior has actually migrated and
`source/examples/series/motion_generation/` for installed standalone consumer
behavior. Run that route through the configured library environment; do not
infer solver-specific parity from namespace registration.

Use the corresponding extensions for behavior that module tests have not yet
established as standalone, plus Kit discovery, lifecycle, and interactive
samples.

Owns the generic `isaacsim.robot_motion.experimental.motion_generation` (`mg`)
substrate and the phase-machine workflow for obstacle-aware end-effector
motion. The motion-generation controller is pluggable: cuMotion
(`isaacsim.robot_motion.cumotion`) is the reference implementation here. PINK and
Lula are also motion-generation controllers but are not yet documented in this
skill; see the stack-selection table in `manipulation-ik` to choose.

Shared substrate lives in sibling skills:

| Need | Read |
|---|---|
| Python-server launch, asset-root checks, screenshots, markers, run videos, final object-pose oracle | `isaac-sim-remote` |
| Grasp frames, contact-only gates, physical grasp validation, IK-stack selection | `manipulation-ik` |
| Mobile-base planning, footprints, differential-drive/Ackermann kinematics, chase cameras | `navigation-primitives` |
| Dynamic object collision, rigid bodies, materials, physics readback | `physics-simulation` |
| Headless rendering and video capture | `isaac-sim-rendering` |
| Final output QA | `isaac-sim-validator` |

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/control_loop.py` | Control loop | import helper |
| `scripts/cumotion_setup.py` | Cumotion setup | import helper |
| `scripts/frames_and_grasping.py` | Frames and grasping | import helper |
| `scripts/inspect_scene.py` | Inspect scene | Python-server injected globals |
| `scripts/phase_machine.py` | Phase machine | import helper |
| `scripts/world_binding.py` | World binding | import helper |

## Running scripts

The control-loop, world-binding, frame, controller-setup, and phase-label files
are importable helpers, not CLIs with `--help`. Import or adapt them in a scene
script. For Kit demo iteration, load `isaac-sim-remote` and send scene logic to
the running Python server. Use `./python.sh` or `python.bat` for the delivered
standalone script after validating the scene through that server. Agent runtimes
with `run_script()` may use it to launch the adapted standalone entry point;
do not invoke import-only helpers as scripts.

## Scope Rules

- Use current experimental Isaac Sim APIs. Do not add deprecated manipulator
  APIs or legacy `omni.isaac.*` compatibility unless the user asks for
  migration. `RobotState` and the world-binding substrate live in
  `isaacsim.robot_motion.experimental.motion_generation`; the non-experimental
  `isaacsim.robot_motion.motion_generation` is deprecated.
- For Kit demos, iterate through the Python server before standalone capture,
  unless the user explicitly requests standalone validation. If the server is
  unavailable, report the blocked validation rather than silently substituting
  restart-per-attempt runs. Validate
  migrated library behavior through its module tests and `source/examples`, not
  through the Python server.
- Do not trust shell exit code alone. Inspect stdout/logs for tracebacks,
  `RuntimeError`, explicit pass/fail JSON, and phase completion.
- Keep one-off probes in `/tmp`. Only keep reusable, domain-specific helpers
  under this skill.
- A standalone Kit script is run by a person, so it must open a visible window
  by default. Gate headless mode and any frame capture / video encoding behind
  explicit opt-in flags (for example `--render OUTPUT.MP4` switches to headless
  capture). Never set `--headless` or `SimulationApp(headless=...)` to
  default-on, and never capture frames or encode a video unless asked. Iterating
  on the Python server (above) is the separate, headless path for tuning.

## Workflow

For Kit demo development, establish the Python-server session using
[`references/workflow.md`](references/workflow.md#python-server-iteration).
Record a successful health response, then execute scene inspection, controller
iterations, and validation through that session. Starting a server or describing
this plan alone does not satisfy live iteration. The explicit user-requested
standalone exception in Scope Rules still applies. Keep the invocation and client
response evidence alongside the run metrics; successful physics/video alone
cannot establish that the required workflow was followed.

1. Inspect robot prim path, DOF/link names, controller tool frames, object
   pose/AABB, physics schemas, and collision APIs.
2. Define the task frame before tuning: tool frame, physical grasp/action
   point, object semantic axis, and target axis. Measure and log the tool-local
   grasp offset in the approach orientation before tuning gains, obstacle
   inflation, or timeouts; freeze it through close/lift.
3. Build world binding and controller (`references/world-binding.md`,
   `references/cumotion.md`).
4. Run an explicit phase machine (`references/control-loop.md`) with target,
   reset policy, convergence predicate, timeout, and trace line per phase.
   Advance required motion phases only on measured convergence. A timeout or
   failed grasp/contact gate terminates the attempt as FAIL; never release or
   continue to the next phase on timeout.
5. Validate measured outputs from the actual run. For dynamic manipulation,
   use `manipulation-ik` and `physics-simulation` for grasp/contact gates. Overall
   PASS requires every required phase and gate, plus stable final object pose,
   to pass in the same run (with recording enabled when video is requested).
   An interrupted run is incomplete;
   a final pose alone cannot override a failed phase.

## References And Helpers

- `references/workflow.md`: task architecture and phase-machine shape.
- `references/world-binding.md`: obstacles and robot-root transforms
  (`SceneQuery`, `ObstacleStrategy`, `WorldBinding`, `synchronize_transforms`).
- `references/control-loop.md`: `RobotState`/`JointState`/`SpatialState` builders
  and the `reset`/`forward` step that applies joint targets.
- `references/frames-and-grasping.md`: tool-frame offsets, axis contracts, and
  gripper targeting.
- `references/cumotion.md`: cuMotion-specific controller (`RmpFlowController`,
  supported robots, cspace params, RMPflow failure modes).
- `scripts/inspect_scene.py`: Python-server scene inspection (optional cuMotion
  probe via the `supported_robot` arg).
- `scripts/cumotion/standalone_demo_template.py`: copy-adapt standalone cuMotion
  scaffold.
- `scripts/control_loop.py`: name-addressed `RobotState` builders and joint-target
  application.
- `scripts/world_binding.py`: cuMotion obstacle discovery and per-frame world sync.
- `scripts/cumotion_setup.py`: supported-robot RMPflow controller construction.
- `scripts/frames_and_grasping.py`: tool/contact offset conversion helpers.
- `scripts/phase_machine.py`: reusable manipulation phase labels.

Generic helpers live in `isaac-sim-remote`:

- `isaac-sim-remote/scripts/verify_asset.py`
- `isaac-sim-remote/scripts/set_debug_view.py`
- `isaac-sim-remote/scripts/viewport_video.py`

## Controller wiring (quick rules)

Full code in `references/world-binding.md` + `references/control-loop.md`;
controller construction in `references/cumotion.md`. The non-negotiables:

- Build joint and site state by name, not by index assumptions
  (`robot.dof_names`, the controller's reported tool/site frames).
- Call `controller.reset(estimated, setpoint, t=0.0)` before the first
  `forward()` and after intentional target discontinuities.
- Pass controller clock time into `forward(...)`; do not pass `dt`.
- Apply positions, velocities, and efforts from the desired joint state when
  present.
- Synchronize the world binding every control frame
  (`update_world_to_robot_root_transforms(...)` then `synchronize_transforms()`).
- Keep the grasped object out of the tracked obstacle set, or the controller
  plans around it.
- For simple reactive obstacle avoidance, bind obstacles through
  `CumotionWorldInterface` / `WorldBinding` and keep the transport phase in the
  `RmpFlowController.forward()` loop. Do not switch to graph planning or author
  bypass/clearance targets unless the demo explicitly needs global planning.
- Tune obstacle inflation in small steps; large clearance buffers can improve
  avoidance but break grasp/place convergence.
- Log requested planner clearance and measured runtime clearance separately;
  they are not the same.
- Start with example-scale c-space/posture weights; large bias can hide weak
  task-space motion.
- For rendered demos, validate the recorder-enabled run, not only a non-captured
  metrics run.

## Mobile-Base Controller Pattern

For mobile bases, route planning and wheel geometry to
`navigation-primitives`. Use this skill only for `RobotState` / controller
integration, and apply joint velocities/efforts/positions rather than writing
root transforms as the success path for a physics run.

## Frame Discipline

Most failures are frame errors. Log these separately (see
`references/frames-and-grasping.md`):

- controller tool frame, such as `tool0` or `wrist_3_link`
- desired and measured tool local `+Z` axis
- visible fingertip midpoint or suction tip
- task grasp point on the object
- object origin, center of mass, and AABB center
- object semantic axis and final target axis
- target pose sent to the controller

Command the tool pose that makes the physical grasp point land on the object
grasp point. If the controller drives a flange/tool frame but the gripper
contacts at a fingertip or suction cup, calibrate the tool-local offset in the
approach orientation and freeze it through close/lift.

Before coding a flip or placement task, prove the final pose is geometrically
feasible for the grasp. If a requested final pose requires an upward-facing
gripper or below-surface wrist, change the grasp strategy before tuning
controller gains or timeouts.

## Canonical Sources

Kit tutorial scripts (`source/standalone_examples/tutorials/manipulation/`, all
use the `mg` substrate):

- `tutorial_9_arm_trajectory.py`, `tutorial_9_follow_target.py`
- `tutorial_9_pick_place_cumotion.py` (cuMotion), `tutorial_9_pick_place_pink.py` (PINK)

Docs (general API first, then implementations):

- `docs/isaacsim/robot_motion_experimental/index.rst`: framework overview
- `docs/isaacsim/motion_generation/{index,scene_interaction,trajectory_planning,mobile_robot_control_example}.rst`
- `docs/isaacsim/cumotion/index.rst`, `docs/isaacsim/pink/index.rst`
