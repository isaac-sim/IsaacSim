---
name: isaac-sim-orchestrator
description: "Integrate approved multi-skill Isaac Sim demos and validate delivery. Route focused obstacle-aware arm demos to motion-generation, including video requests; scope gaps with isaac-sim-workflow."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# Isaac Sim Orchestrator

## Purpose

Turn an established goal or contract into a validated library, Kit integration, or runnable simulation by mapping capabilities and executing specialist skills in order. A request such as "the forklift demo is scoped and approved; build it and give me the video" enters the four phases below directly. Use `isaac-sim-workflow` first only when the deliverable or acceptance criteria still need scoping.

## Specialist ownership

A focused obstacle-aware arm demo belongs to `motion-generation`, including
pick-and-place around a pole with physics and video. If this orchestrator is
loaded first, hand off to `motion-generation` before implementation and keep it
as the primary skill. Orchestration may support composition and delivery.

When a native skill invocation tool is available, invoke the specialist with
that tool; shell reads of its instructions or scripts do not replace invocation.
When the runtime only supports file-based skill loading, use that mechanism and
state the trace limitation accurately.

## Prerequisites

- For Kit integration: built Isaac Sim, a supported NVIDIA GPU, and the shell
  environment below.
- For a standalone library foundation: a configured `source/libraries` build or
  installed wheel/SDK; Kit and a GPU are not universal requirements.

## Limitations

- Kit-specific guidance targets Isaac Sim 6 / Kit 110 unless a section states otherwise.
- Does not replace official NVIDIA documentation for unsupported edge cases.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Extension not found | Wrong `$ISAAC_SIM_DIR` or stale Kit build | Point env vars at `_build/linux-x86_64/release` or rebuild |
| Black or empty frames | Missing lights or non-RTX render mode | Add dome/key light; confirm RTX / PathTracing settings |
| Hang on stage load or first render | MDL compile or oversized stage | Follow isolation steps in `isaac-sim-troubleshooting` |

## Environment contract

Kit-routed skills use these variables; standalone library work uses its configured build/install environment instead:

| Variable | Purpose | Example |
|---|---|---|
| `$ISAAC_SIM_DIR` | Isaac Sim install root or built repo path | `$HOME/IsaacSim` (install) or `<this-repo>/_build/linux-x86_64/release` (source build) |
| `$ISAAC_LAB_DIR` | Isaac Lab checkout | `$ISAAC_SIM_DIR/IsaacLab` |
| `$WORKSPACE_DIR` | Per-agent outputs, scratch, caches | unset by default; pick a project-local path or `~/.cache/<repo>` |
| `$CIP_ROOT` (Windows) | Content-pipeline install (CIP/WRAPP) | `C:\_Data` |

Run `nvidia-smi` before GPU-backed simulation or rendering to size `num_envs`
and pick RT2 vs PathTracing. Do not require it for a library-only CPU workflow
or hardcode a GPU class.

## Execution mode

Choose the owning surface before running code:

| Surface | Execution |
|---|---|
| Standalone library | Use the configured module build, installed wheel/SDK, or `source/examples/examples.sh`; do not create `SimulationApp` |
| Kit demo/controller | Iterate and validate through `isaac-sim-remote` before standalone handoff/capture, unless the user explicitly requests standalone validation |
| Other Kit extension or adapter | Prefer live Python-server iteration through `isaac-sim-remote`; use `python.sh` for a standalone Kit handoff test |
| Library staged in Kit | Prove the library in a normal process first, then test its wheel staging and lifecycle adapter in Kit |

### Extension-to-library rewrite sequence

When an established contract moves reusable behavior out of an extension:

1. Inventory and classify the old API, implementation, adapters, packaging, dependencies, tests, imports, and version surfaces.
2. Keep one implementation in `source/libraries`; preserve supported imports and prove its public facade without Kit.
3. Stage the built wheel and SDK in a thin Kit extension that retains only Kit lifecycle or translation behavior.
4. Validate library and Kit routes independently with `isaac-sim-validator`, including their separate version decisions.

Launch a server-enabled sim using the launch procedure in `isaac-sim-remote`; do not duplicate its command here.
Before Kit controller iteration, require a successful health response and then
send scene setup, tuning, and validation probes through the client. Retain their
responses as execution evidence. If the server cannot be made available, report
that workflow validation is blocked; do not silently switch to repeated
`SimulationApp` launches or claim workflow completion from a successful video.
For graceful shutdown after live iteration, invoke the [remote skill's shutdown helper](../isaac-sim-remote/scripts/shutdown_app.py) through its localhost client.

## Task decomposition

For any request, run all four phases. The specific steps inside each phase depend on the goal; identify capabilities first, then verify each in isolation before combining.

### Phase 1 — Verify foundations

**1a. Feature/skill mapping** (before any code):
- If a demo still lacks a deliverable or acceptance criteria, load `isaac-sim-workflow` to define them, then return to these phases. If the request already fixes the subject, behavior, constraints, and output (for example, a collision-free aisle drive delivered as a 20-second video), use that contract directly; do not repeat the scoping pass.
- Cross-check the request against documented Isaac Sim features and APIs.
- For each capability, look up an existing skill:
  - Skill exists -> load it, follow its procedure.
  - Skill missing -> build one inline. Mark its frontmatter `status: draft`, flag it `HIGH PRIORITY` in `skill-distillation`, tell the user upfront, and shorten iteration cycles (share intermediate results, ask targeted questions early).
- Write the feature -> skill mapping into the task `WORKLOG.md` before 1b.

**1b. Foundation verification** (capability by capability):
- List every capability the task needs (assets, physics, robot control, sensors, rendering, ...).
- Verify each in isolation: does it load, does it behave correctly on its own.
- Do not move on until each foundation passes.

### Phase 2 — Incremental integration

Combine verified foundations one at a time. Re-run stability/correctness checks after each addition. Every failure has exactly one new variable.

### Phase 3 — Polish & deliver

- Validate output visually or programmatically. Task success, not just script completion.
- Add output-specific requirements (writers, annotations, video capture, DR).
- Package and hand off with a short summary.

### Phase 4 — Distill (mandatory)

- List iterations, failures, workarounds.
- Record user corrections.
- Classify each lesson: new skill, skill update, procedure fix, or `MEMORY.md` fact.
- Follow `skill-distillation` to decide whether a reusable lesson warrants a skill change. Apply scoped edits when the user has authorized them; otherwise record or propose the patch before writing. If there is no reusable lesson, record that conclusion.

See `skill-distillation` for the full procedure. Phase 4 is not optional, even when no file change is warranted.

## Sub-agent rules

- Each phase can run as a sub-agent with its own `WORKLOG.md`.
- Sub-agents commit at logical checkpoints, never half-done.
- Large script generation: write incrementally to files. Do not try to produce 200+ lines in one turn.
- If a sub-agent times out, `WORKLOG.md` survives for the next pickup.

## Routed skills

| Skill | Use for |
|---|---|
| `isaac-sim-remote` | Live Python-server iteration and validation for Kit development |
| `motion-generation` | Primary owner for obstacle-aware arm controllers and pick-and-place, including physics/video demos |
| `urdf-mjcf-to-usd-conversion` | Import URDF/MJCF robot descriptions to USD with physics APIs and articulation structure |
| `physx-to-newton-conversion` | Convert an existing PhysX-only USD robot to a Newton-capable variant package |
| `usd-pipeline` | Asset insertion, scaling, materials, headless render compatibility |
| `usd-composition-architecture` | Layered USD assets (root + physics + appearance) |
| `usd-articulation` | Multi-link articulations, joint hierarchies, Robot Schema overlay |
| `physics-simulation` | PhysicsScene config, per-prim setup, contact materials, Newton vs PhysX |
| `isaac-sim-sensor` | RTX/physics sensors (camera, LiDAR, IMU, contact), render products, annotators |
| `isaac-sim-rendering` | Headless Kit 110 capture, RT2/PathTracing, ACES |
| `isaac-sim-validator` | Final QA gate before delivery |
For Kit bootstrap and the distinction from standalone library imports, see
[`references/api-cheatsheet.md`](references/api-cheatsheet.md).

## Multi-robot fleet reference

### Sample robots

| Robot | Start Z | Drive | Notes |
|---|---|---|---|
| Nova Carter | 0.0 | differential | wheel radius 0.14 m, track 0.499 m; damping 100K |
| VSVXL | 0.0 | differential | most reliable; wheel radius 0.15 m, track 1.52 m |
| Spot | 0.75 | omni-wheel | bbox min Z = -0.69; needs ground clearance |
| FR3 | 0.0 | fixed-base | end-effector only, not mobile |

### Scene setup

- `sim_warehouse_v4.usda` pattern: `shell` + `lights` + `racks` + `PhysicsScene` + ground collision.
- Shell (`sm_warehouse_mega.usd`) is in cm; robots and equipment in meters.
- Strip physics from environment assets offline. Runtime stripping core-dumps on large stages.

### PhysicsScene

```python
from pxr import UsdPhysics, PhysxSchema

physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
physics_scene.CreateGravityDirectionAttr().Set((0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
physx_scene.CreateEnableCCDAttr().Set(True)
physx_scene.CreateEnableStabilizationAttr().Set(True)
physx_scene.CreateSolverTypeAttr().Set("TGS")
physx_scene.CreateTimeStepsPerSecondAttr().Set(60)
physx_scene.CreateGpuMaxNumPartitionsAttr().Set(8)   # 10+ robots
```

### Grid placement

```python
import math

def place_robots_grid(stage, robot_usd_path, prefix, count, spacing=3.0, start_z=0.0):
    cols = math.ceil(math.sqrt(count))
    robots = []
    for i in range(count):
        row, col = divmod(i, cols)
        x, y = col * spacing, row * spacing
        prim_path = f"/World/Robots/{prefix}_{i}"
        ref = stage.OverridePrim(prim_path)
        ref.GetReferences().AddReference(robot_usd_path)
        from pxr import UsdGeom, Gf
        xform = UsdGeom.Xformable(ref)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, start_z))
        robots.append(prim_path)
    return robots
```

### Separation

- Mobile robots: minimum 2 m between centers.
- Articulated arms: 1.5x reach radius minimum.
- Aerial: stagger altitudes by >= 2 m.

### Collision groups

```python
from pxr import PhysxSchema

def create_collision_group(stage, group_path, robot_paths):
    group = PhysxSchema.PhysxCollisionAPI.Apply(stage.DefinePrim(group_path))
    for path in robot_paths:
        prim = stage.GetPrimAtPath(path)
        collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision_api.GetCollisionGroupsRel().AddTarget(group_path)
```

### Scaling limits (by VRAM)

Limits scale approximately linearly with available VRAM. Beyond these thresholds risks CUDA OOM.

| Metric | 12 GB | 24 GB | 48 GB | 96 GB | Notes |
|---|---|---|---|---|---|
| Total prims | ~12K | ~25K | ~50K | ~100K | scales linearly |
| Robots | <= 2 | <= 5 | <= 10 | <= 20 | depends on complexity |
| Active rigid bodies per robot | ~200 | ~200 | ~200 | ~200 | per-robot constant |
| Articulations (multi-DOF) | <= 2 | <= 5 | <= 10 | <= 20 | |
| Render resolution | 1280x720 | 1600x900 | 1920x1080 | 2560x1440 | single viewport |

Optimization:

- `make_instanceable: true` in URDF config.yaml (shared mesh data).
- LOD switching for distant robots.
- Disable physics on robots outside the active zone.

### Navigation

Differential drive kinematics:

```
vL = (vx - omega * tw/2) / wheel_r
vR = (vx + omega * tw/2) / wheel_r
```

PD steering defaults: `KP=2.5`, `KD=1.2`, `MAX_W=1.5`, waypoint tolerance 4.0 m. Out-of-bounds: `|Z| > 50` or `|X|/|Y| > 500` -> mark dead.

### Camera

Chase camera: 12 m behind, min height 2.5 m, clamped inside warehouse bounds, smooth interpolation `alpha = min(1.0, DT*2.0)`. Dynamically raise camera to avoid rack intrusion.

View modes (cycle every 4 s): chase, overhead (z=50 m), aisle (eye-level), wide (z=20 m, yaw=0).

### Lighting (warehouse default)

| Parameter | Value |
|---|---|
| filmISO | 100-120 (200 overexposes, 80 too dark in aisles) |
| DomeLight | intensity 150, color (0.85, 0.88, 0.95) |
| Fill SphereLights | intensity 1200, color (1.0, 0.95, 0.85), height 8-9 m |
| RectLights | ceiling-mounted, aisle-aligned |

### Rendering

- `RayTracedLighting` (RT2), 1920x1080.
- 320x240 window with `hideUi=1` to save GPU.
- Always set `DISPLAY=:0`; headless viewport init fails for complex regions.
- `maxBounces=7`, `aovs=none`.

### Timing

```
DT = 1/60
Settle: 200-500 frames after timeline.play()
Capture: every 4th step (15 fps)
```

## Workflow: multi-robot sim

1. Receive request (e.g. "6 Novas in a warehouse with 100 racks").
2. Compose scene: load warehouse USD, position robots via `place_robots_grid`.
3. Create collision groups if needed.
4. Use `usd-pipeline` to validate mesh scale and shaders.
5. Run in a persistent session: iterate live via `isaac-sim-remote` (Python server), or `isaac-sim.sh --exec script.py` for a standalone/handoff run.
6. Use a render-pulse loop every 100 steps.
7. Validate the render via `isaac-sim-validator`.
8. Deliver video and final scene.

## Workflow: Physical AI end-to-end pipeline

Route chain for tasks that span asset import, physics, sensors, and validation (e.g. "import a Franka arm, simulate grasping, capture LiDAR + RGB, validate").

### Pipeline stages

```
┌─────────────────────────┐     ┌──────────────────────┐     ┌───────────────────┐     ┌─────────────────────┐
│ 1. Asset Import         │────▶│ 2. Physics Setup     │────▶│ 3. Sensor Attach  │────▶│ 4. Validate & Ship  │
│                         │     │                      │     │                   │     │                     │
│ urdf-mjcf-to-usd-conv  │     │ physics-simulation   │     │ isaac-sim-sensor  │     │ isaac-sim-validator │
│ usd-pipeline            │     │ usd-articulation     │     │ isaac-camera      │     │ isaac-sim-rendering │
│ usd-composition-arch    │     │                      │     │ isaac-sim-remote  │     │                     │
└─────────────────────────┘     └──────────────────────┘     └───────────────────┘     └─────────────────────┘
```

### Stage 1 — Asset import

| Input | Skill | Output contract |
|---|---|---|
| URDF/MJCF file | `urdf-mjcf-to-usd-conversion` | USD with `IsaacRobotAPI`, `IsaacLinkAPI`, `IsaacJointAPI`, collision meshes |
| USD environment assets | `usd-pipeline` | Measured, shader-classified, placed assets with bbox offsets |
| Multi-layer composition | `usd-composition-architecture` | Root + physics + appearance layers |

Handoff to Stage 2: USD file(s) on disk, prim paths known, `make_instanceable: true` for RL workloads.

### Stage 2 — Physics setup

| Input | Skill | Output contract |
|---|---|---|
| Imported USD stage | `physics-simulation` | `PhysicsScene` (gravity, solver, timestep, CCD), per-prim `RigidBodyAPI`/`CollisionAPI`/`MassAPI`, contact materials, joint drives |
| Multi-DOF robot | `usd-articulation` | Articulation root, joint hierarchy, drive stiffness/damping |

Prerequisites from Stage 1:
- Robot USD must have `IsaacRobotAPI` on root (applied by importer).
- Static environment prims need `CollisionAPI` only (no `RigidBodyAPI`).
- PhysicsScene is always created here even if the importer applied per-joint attrs.

Handoff to Stage 3: Sim plays without crashes, robot holds pose under gravity for 200 frames.

### Stage 3 — Sensor attachment

| Input | Skill | Output contract |
|---|---|---|
| Stable sim stage | `isaac-sim-sensor` | Sensor prims parented to robot links, render products with annotators |
| Camera intrinsics | `isaac-camera` | Configured USD cameras with lens model and focal params |
| Runtime verification | `isaac-sim-remote` | Push sensor config live, verify data stream non-zero |

Mount-point convention: sensors attach to Xform prims under robot links. If the imported URDF lacks a mount link, create one:
```python
mount = UsdGeom.Xform.Define(stage, f"{robot_path}/{link_name}/sensor_mount")
```

Handoff to Stage 4: At least one frame of non-zero sensor data (depth > 0, point cloud non-empty, IMU reports gravity).

### Stage 4 — Validate and deliver

| Input | Skill | Checks |
|---|---|---|
| Complete sim script | `isaac-sim-validator` | No deprecated imports, no hardcoded paths, lights present, render not black |
| Rendered frames | `isaac-sim-rendering` | Frame quality, ACES tonemap, resolution |
| Runtime behavior | (manual or scripted) | Physics: robot stays grounded, no NaN. Sensors: data stream matches expected range |

The validator gates delivery but does not cover runtime physics/sensor correctness. For runtime checks, verify:
1. `RigidBodyAPI.GetVelocityAttr()` stays finite across the sim window.
2. Sensor annotator data shape matches configured resolution/channels.
3. Articulation joint positions stay within drive limits.

### End-to-end procedure

1. **Phase 1a** — Map request features to the pipeline stages above.
2. **Phase 1b** — Verify each stage in isolation:
   - Import: USD loads without errors, prim count reasonable for VRAM.
   - Physics: Robot holds pose, `timeline.play()` + 200 frames stable.
   - Sensors: One sensor produces valid data on a simplified scene.
3. **Phase 2** — Integrate incrementally (import → physics → sensors), re-checking stability after each addition.
4. **Phase 3** — Run `isaac-sim-validator`, capture final output, deliver.
5. **Phase 4** — Distill lessons per `skill-distillation`.

## Debug protocol

### Rendering

| Issue | Action |
|---|---|
| Black frame | DomeLight + DistantLight present; force `settings.set("/rtx/rendermode", "RayTracedLighting")`; check `nvidia-smi` |
| Garbled color | ACES tonemap (`/rtx/post/tonemap/op=4`); `filmISO=600` for warehouse; remove `PathTracing` |
| Stuttering | `DT=1/60`; `setTimeStepsPerSecond=60`; update display rate, not physics rate |
| Fractures | Mesh integrity; reduce bump/normal map resolution; low-res collision meshes |
| Articulations move wrong | `SolverType=TGS` (fabric); `maxPositionIterations >= 6`; `make_instanceable: true` |
| OOM crash | `pkill -f "kit/kit"`; clean `/dev/shm/carb-*`; reduce num_envs |

### Asset loading

| Issue | Action |
|---|---|
| Asset renders black | `UsdPreviewSurface` or dual-shader; relative paths (`./meshes/asset.usd`); confirm instanceable when valid |
| Transform wrong | Check `mpu` (default 1.0); apply offset before placement; verify with `measure_asset()` |
| Mesh missing | Case-sensitive paths; use absolute; validate via `stage.GetPrimAtPath()` |

### Training

| Issue | Action |
|---|---|
| NaN loss | Lower LR; clip rewards to `[-5, 5]`; check divergent teleop benchmarks |
| Reward flat at 0 | Add dense shaping; reduce reward scale 50%; add input noise for exploration |
| Value divergence | Increase target-net update frequency; prioritized replay; larger batch |

## Operating rules

- Never delete work folders; reuse and branch.
- Always save the `.usd` file; never assume it lives only in memory.
- Validate every render; never deliver black frames.
- Every long-running process must be killable (`pkill` or pidfile).
- No bare `~/` in produced scripts; expand to `$HOME` or `$ENV_VAR`.
- `make_instanceable: true` for all RL robots.
- Call `simulation_app.update()` 5x after a camera switch.
- Never import torch before `timeline.play()`.
- Log GPU memory every epoch.
- Lazy-load HoD assets (decompress on demand).
- Run `skill-distillation` (step 5 of the request loop) at task end. Capture lessons in the relevant SKILL.md, not in scratch memory.
