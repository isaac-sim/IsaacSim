---
name: isaac-sim-orchestrator
description: >
  Top-level dispatcher: turns a natural-language Isaac Sim request into a runnable
  simulation. Owns the env-var contract (`$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`,
  `$WORKSPACE_DIR`, `$CIP_ROOT`), decomposes the task into capabilities, routes
  to specialist skills (`usd-pipeline`, `isaac-sim-rendering`,
  `isaac-sim-validator`, `physics-simulation`, `usd-composition-architecture`),
  and validates output before delivery.
  Use when (1) creating a sim scene with robots, objects, environments,
  (2) controlling robots in Isaac Sim, (3) generating renders or camera captures,
  (4) collecting Physical AI training data, (5) running headless sims on GPU,
  (6) orchestrating multi-robot fleets in warehouses.
---

# Isaac Sim Orchestrator

## Environment contract

Every routed skill assumes these variables; set them in the agent config or shell:

| Variable | Purpose | Example |
|---|---|---|
| `$ISAAC_SIM_DIR` | Isaac Sim install root or built repo path | `$HOME/IsaacSim` (install) or `<this-repo>/_build/linux-x86_64/release` (source build) |
| `$ISAAC_LAB_DIR` | Isaac Lab checkout | `$ISAAC_SIM_DIR/IsaacLab` |
| `$WORKSPACE_DIR` | Per-agent outputs, scratch, caches | unset by default; pick a project-local path or `~/.cache/<repo>` |
| `$CIP_ROOT` (Windows) | Content-pipeline install (CIP/WRAPP) | `C:\_Data` |

Run `nvidia-smi` at session start to size `num_envs` and pick RT2 vs PathTracing. Do not hardcode GPU class.

## Task decomposition

For any request, run all four phases. The specific steps inside each phase depend on the goal; identify capabilities first, then verify each in isolation before combining.

### Phase 1 — Verify foundations

**1a. Feature/skill mapping** (before any code):
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
- Update the skill files; re-read to confirm a fresh agent can follow them.

See `skill-distillation` for the full procedure. Phase 4 is not optional.

## Sub-agent rules

- Each phase can run as a sub-agent with its own `WORKLOG.md`.
- Sub-agents commit at logical checkpoints, never half-done.
- Large script generation: write incrementally to files. Do not try to produce 200+ lines in one turn.
- If a sub-agent times out, `WORKLOG.md` survives for the next pickup.

## Routed skills

| Skill | Use for |
|---|---|
| `usd-pipeline` | Asset insertion, scaling, materials, headless render compatibility |
| `usd-composition-architecture` | Layered USD assets (root + physics + appearance) |
| `isaac-sim-rendering` | Headless Kit 110 capture, RT2/PathTracing, ACES |
| `physics-simulation` | PhysicsScene config, per-prim setup, contact materials, Newton vs PhysX |
| `isaac-sim-validator` | Final QA gate before delivery |

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
5. Launch `isaac-sim.sh --exec script.py` in a persistent session.
6. Use a render-pulse loop every 100 steps.
7. Validate the render via `isaac-sim-validator`.
8. Deliver video and final scene.

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
