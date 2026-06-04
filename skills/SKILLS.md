# Isaac Sim Skills Index

Platform-neutral inventory of agent skills in this repository. Both
`AGENTS.md` (Cursor / Codex CLI / other AGENTS.md-aware tools) and `CLAUDE.md`
(Claude Code) point here.

Legacy `.claude/skills/` and `.cursor/skills/` are retired; everything lives
under `skills/`.

A skill is a directory with a `SKILL.md` (YAML frontmatter for auto-loading)
plus optional `scripts/` and `references/`. Skills are content-addressable by
the descriptions below.

---

## Library layers

| Layer | Purpose | Source |
|---|---|---|
| Repo-native dev | Build, test, debug, profile, document, operate this Isaac Sim source repo | Authored against this repo |
| Robotics-sim | Build, render, validate Isaac Sim simulations as a downstream user | Imported from the isaac-claw library |

Repo-native skills run inside this repo (`./build.sh`, `tools/ci/`, `./repo.sh
docs`, the `python_server` socket). Robotics-sim skills drive a built Isaac
Sim from a script (`SimulationApp`, `isaacsim.core.experimental.*`, USD
authoring).

---

## Quick index

<!-- AUTOREMOVE: BEGIN -->
- [Repo-native developer skills (dev-only)](#repo-native-developer-skills-dev-only) — build, CI, debug, doc-snippets
<!-- AUTOREMOVE: END -->
- [Repo-native public skills](#repo-native-public-skills) — remote-control, profile, validate
- [Foundations & operating loop](#foundations--operating-loop) — what every robotics-sim session loads
- [Robot asset pipeline](#robot-asset-pipeline) — URDF → USD
- [Physics simulation](#physics-simulation) — PhysX, Newton, contact, joints
- [Mobile robot navigation](#mobile-robot-navigation) — runtime nav, SDG
- [Manipulation](#manipulation) — IK + grasp
- [Sensors & perception](#sensors--perception) — camera, LiDAR, IMU
- [Synthetic data generation](#synthetic-data-generation) — Replicator, MobilityGen
- [Rendering & lighting](#rendering--lighting) — RT2 production rendering
- [USD pipeline](#usd-pipeline) — composition, scaling, material binding
- [ROS 2 integration](#ros-2-integration) — Nav2, multi-robot bridges
- [Agent meta](#agent-meta) — composition patterns, distillation
- [How the library composes](#how-the-library-composes) — pipeline diagrams

---

<!-- AUTOREMOVE: BEGIN -->
## Repo-native developer skills (dev-only)

Operate on this repo's source tree. Run from the repo root. Excluded from published packages.

| Skill | What it does |
|---|---|
| [`build-docs`](_internal/build-docs/SKILL.md) | Build the Isaac Sim user guide / API docs (full or partial), serve them locally for preview, and run the pre-commit formatter. |
| [`cicd`](_internal/cicd/SKILL.md) | Navigate this repo's GitLab pipelines: stages, jobs, variables, common debug flows, and the IsaacLab / IsaacSim dashboard subcommands under `tools/ci/dashboards/`. |
| [`debug-with-local-kit`](_internal/debug-with-local-kit/SKILL.md) | Build Kit from source, link Isaac Sim against the local Kit build, add debug prints / step through Kit code, and investigate Kit-level rendering / multitick / sensor issues. |
| [`doc-snippets`](_internal/doc-snippets/SKILL.md) | Author and runtime-test the Python code samples shown in Isaac Sim user docs (`.py` + `literalinclude`, the snippet test runner, async vs `SimulationApp` styles, common pitfalls). |

---
<!-- AUTOREMOVE: END -->

## Repo-native public skills

Operate on this repo's source tree. Shipped in published packages.

| Skill | What it does |
|---|---|
| [`isaac-sim-remote`](isaac-sim-remote/SKILL.md) | Drive a running Isaac Sim instance over the `python_server` TCP socket (port 8226): run code, open stages, inspect/modify prims, take screenshots, step physics, read console logs. Works headless. |
| [`profile-isaac-sim`](profile-isaac-sim/SKILL.md) | Profile and optimize Isaac Sim with the in-repo benchmark scripts and Tracy. Compare runs, diff frame times, isolate hot zones. |
| [`validation-diff-gifs`](validation-diff-gifs/SKILL.md) | Build pixel-diff GIFs comparing a validation capture run against its golden data — fastest way to triage benchmark image failures. |

---

## Foundations & operating loop

Loaded by default for any robotics-sim session, plus the distillation step.

| Skill | What it does |
|---|---|
| [`isaac-sim-orchestrator`](isaac-sim-orchestrator/SKILL.md) | Top-level dispatcher: turns natural-language requests into runnable sims. Declares the env-var contract every other skill assumes (`$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR`, `$CIP_ROOT`). Routes to `usd-pipeline`, `isaac-sim-rendering`, `isaac-sim-validator`, `physics-simulation`. |
| [`meta-skills`](meta-skills/SKILL.md) | Composition patterns + Meta-Skilling Framework. Read first to learn how to navigate, compose, and author skills. |
| [`skill-distillation`](skill-distillation/SKILL.md) | *Always-on* — step 5 of every request loop: capture what you learned before delivering. |
| [`isaac-sim-validator`](isaac-sim-validator/SKILL.md) | Final QA gate before delivery: rejects black frames, hardcoded user paths, deprecated `omni.isaac.core` imports, missing lights, mounting bugs. |
| [`isaac-sim-troubleshooting`](isaac-sim-troubleshooting/SKILL.md) | Kit 110 hang/freeze/perf reference — startup hangs, MDL freezes, physics stepping hangs, Replicator hangs, Hydra issues. |

---

## Robot asset pipeline

URDF/MJCF/CAD → sim-ready USD asset.

| Skill | What it does |
|---|---|
| [`urdf-mjcf-to-usd-conversion`](urdf-mjcf-to-usd-conversion/SKILL.md) | URDF/MJCF → USD for Isaac Sim & Isaac Lab. `config.yaml` schema, `make_instanceable`, RL vs teleop drives, instanceable meshes. XACRO must be pre-expanded. Every new robot starts here. |
| [`usd-articulation`](usd-articulation/SKILL.md) | Multi-link / multi-arm articulation validation. `ArticulationRootAPI`, `FixedJoint`, flatten-before-deploy. |

---

## Physics simulation

Single source of truth for physics scene config and per-prim setup.

| Skill | What it does |
|---|---|
| [`physics-simulation`](physics-simulation/SKILL.md) | PhysicsScene/Hz/CCD/stabilization, RigidBodyAPI/MassAPI/CollisionAPI/kinematic, contact materials, joint drives, Newton solver selection (Featherstone/MuJoCo/XPBD/SemiImplicit/VBD) vs PhysX, physics sensors (contact/IMU/raycast), `RigidPrim` vs `XformCache` readback. Includes worked examples (impact/crash, vibratory feeder, domino cascade, spinning top, Newton's cradle, pendulum wave, escapement). |

---

## Mobile robot navigation

Shared substrate plus runtime / SDG specializations.

| Skill | What it does |
|---|---|
| [`navigation-primitives`](navigation-primitives/SKILL.md) | Shared substrate: `OccupancyMap`, A* planner, robot footprints (Spot Z=0.69, Carter, VSVXL, Jetbot, Kaya, H1), differential/holonomic kinematics, look-at chase camera math. Read FIRST. |
| [`occupancy-map`](occupancy-map/SKILL.md) | Generate ROS-compatible `map.yaml` from USD warehouses for Nav2 / MobilityGen / A*. |
| [`isaac-sim-robot-navigation`](isaac-sim-robot-navigation/SKILL.md) | Runtime nav in custom scripts: RL policy, physics-vs-baked-vs-per-frame, GPU OOM avoidance, Kit 110 gotchas. |
| [`mobility-gen`](mobility-gen/SKILL.md) | Two-phase SDG: record trajectories (physics) → replay + render (sensors). Custom `MobilityGenRobot` subclassing. |

---

## Manipulation

| Skill | What it does |
|---|---|
| [`manipulation-ik`](manipulation-ik/SKILL.md) | Differential IK (damped least-squares), grasp frames, `FixedJoint` grasping, hybrid IK + joint-space. |

---

## Sensors & perception

| Skill | What it does |
|---|---|
| [`isaac-sim-sensor`](isaac-sim-sensor/SKILL.md) | Replicator sensor suite: RGB/depth/seg/optical-flow, LiDAR, IMU, contact, ultrasonic, DR. Vendor LiDAR/radar/acoustic catalog (`SUPPORTED_LIDAR_CONFIGS`: Ouster, Hesai, Velodyne, Robosense, SICK, Zvision, NVIDIA), USDA mount attachment, custom emitter-state scan-pattern authoring. |
| [`isaac-camera`](isaac-camera/SKILL.md) | `UsdGeomCamera` setup, render products, intrinsics, AOVs, OpenCV/fisheye distortion. |

---

## Synthetic data generation

| Skill | What it does |
|---|---|
| [`data-collection-sim`](data-collection-sim/SKILL.md) | Static-scene Replicator SDG: `BasicWriter`, `PoseWriter`, `KittiWriter`. Sibling of `mobility-gen` (mobile-robot SDG). |
| [`mobility-gen`](mobility-gen/SKILL.md) | (See [Mobile Robot Navigation](#mobile-robot-navigation).) |

---

## Rendering & lighting

| Skill | What it does |
|---|---|
| [`isaac-sim-rendering`](isaac-sim-rendering/SKILL.md) | Headless Kit 110 production rendering: SimulationApp + Replicator capture, RT2 vs PathTracing, ACES tone mapping (`filmIso` 200/400/600), multi-layer warehouse lighting, deep-aisle solutions, validation thresholds. |
| [`isaac-sim-headless-deployment`](isaac-sim-headless-deployment/SKILL.md) | Headless `--no-window` usage: launch modes, CLI flags, `SimulationApp` batch pattern, perf tuning. |

---

## USD pipeline

| Skill | What it does |
|---|---|
| [`spatial-reasoning`](spatial-reasoning/SKILL.md) | Transform math cornerstone: mpu, bbox, T·R·S ordering, look-at, collision-free grids, A*, Dubins, R-tree, 2D bin packing, OSHA/NFPA/RMI standards. |
| [`usd-pipeline`](usd-pipeline/SKILL.md) | Asset discovery, bbox/shader measurement, placeholder-to-asset placement, headless render compatibility (MDL vs UsdPreviewSurface). |
| [`usd-composition-architecture`](usd-composition-architecture/SKILL.md) | NVIDIA-style layered USD (root + physics + appearance payloads). Skip appearance for load-time optimization. |

---

## ROS 2 integration

| Skill | What it does |
|---|---|
| [`isaac-sim-ros2-bridge`](isaac-sim-ros2-bridge/SKILL.md) | OmniGraph ROS 2 nodes (current `isaacsim.ros2.bridge.*` namespace), Nav2 integration, multi-robot namespacing, prerequisites. |

---

## Agent meta

Library navigation and skill authoring.

| Skill | What it does |
|---|---|
| [`meta-skills`](meta-skills/SKILL.md) | Composition patterns, Stackability, MSF five phases (Discovery, Practice, Capture, Validation, Iteration), the SKILL.md template. |
| [`skill-distillation`](skill-distillation/SKILL.md) | Step 5 of every request loop — generalization rule, classification table, draft-skill promotion gates. |

---

## Environment contract

All robotics-sim skills assume these shell variables (set in your agent config
or shell profile). Skills reference them rather than hardcode paths:

| Variable | Purpose | Example |
|---|---|---|
| `$ISAAC_SIM_DIR` | Isaac Sim install root or built repo path | `$HOME/IsaacSim` (install) or `<this-repo>/_build/linux-x86_64/release` (source build) |
| `$ISAAC_LAB_DIR` | Isaac Lab checkout root | `$ISAAC_SIM_DIR/IsaacLab` |
| `$WORKSPACE_DIR` | Per-agent workspace (outputs, scratch, caches) | unset by default — pick a project-local path or `~/.cache/<repo>` |
| `$CIP_ROOT` (Windows) | Content-pipeline install (CIP/WRAPP skills, not currently imported) | `C:\_Data` |

---

## How the library composes

Pipeline diagrams for the major robotics-sim skill chains. Use at the orient
step to find the entry point.

### Core robotics simulation pipeline

```
URDF / MJCF / CAD source
       │
       ▼
urdf-mjcf-to-usd-conversion
       │
       ▼
usd-articulation        ◄──── usd-composition-architecture
(validate joints)              (root + physics + appearance layers)
       │
       ▼
physics-simulation
       │
       ▼
isaac-sim-orchestrator    →    isaac-sim-sensor
       │
       ▼
       data-collection-sim (SDG)
                │
                ▼
        mobility-gen (mobile-robot variant)
```

### Mobile robot navigation

```
                       navigation-primitives
                  (omap, A*, kinematics, robot footprints, camera math)
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
   isaac-sim-robot-navigation         mobility-gen
   (live RL/baked/per-frame nav)      (record → replay SDG)
              │                               │
              └───────────────┬───────────────┘
                              ▼
              occupancy-map (map.yaml)
                              │
                              ▼
              isaac-sim-ros2-bridge (Nav2 integration)
```

### Rendering & validation pipeline

```
isaac-sim-rendering (RT2 + ACES production)
       │
       ▼
isaac-sim-validator (final QA gate)
```

<!-- AUTOREMOVE: BEGIN -->
### Repo-native dev workflows

```
build-docs ─────────► serve docs locally ─────────► doc-snippets
   │                                                    │
   │                                                    ▼
   │                                            (snippet test runner)
   ▼
cicd ────► investigate pipeline / job ──┐
                                        │
debug-with-local-kit ─► local Kit build ┤
                                        ▼
profile-isaac-sim ────► tracy capture ► validation-diff-gifs
                                        │
                                        ▼
isaac-sim-remote ────► drive running Isaac Sim from agent
```

---
<!-- AUTOREMOVE: END -->


## Robotics-sim skills (read order)

Read in this order if you can only load a slice. Item 21 is the always-on pair.

| # | Skill | What it gives you |
|---|---|---|
| 1 | [`isaac-sim-orchestrator`](isaac-sim-orchestrator/SKILL.md) | Top-level dispatcher; declares the env-var contract every other skill assumes |
| 2 | [`urdf-mjcf-to-usd-conversion`](urdf-mjcf-to-usd-conversion/SKILL.md) | Bring any robot from URDF/MJCF into Isaac Sim (pre-expand XACRO) |
| 3 | [`usd-articulation`](usd-articulation/SKILL.md) | Validate and assemble multi-link/multi-arm articulations |
| 4 | [`physics-simulation`](physics-simulation/SKILL.md) | Single source of truth for physics scene + per-prim setup |
| 5 | [`navigation-primitives`](navigation-primitives/SKILL.md) | Shared substrate for all mobile-robot work |
| 6 | [`data-collection-sim`](data-collection-sim/SKILL.md) | Static-scene Replicator SDG |
| 7 | [`mobility-gen`](mobility-gen/SKILL.md) | Mobile-robot SDG (record → replay + render) |
| 8 | [`isaac-sim-robot-navigation`](isaac-sim-robot-navigation/SKILL.md) | Runtime navigation in your own scripts |
| 9 | [`isaac-sim-sensor`](isaac-sim-sensor/SKILL.md) | Sensor primitives for SDG; vendor LiDAR/radar/acoustic catalog + USDA scan patterns |
| 10 | [`isaac-sim-rendering`](isaac-sim-rendering/SKILL.md) | Headless Kit 110 production rendering |
| 11 | [`isaac-sim-headless-deployment`](isaac-sim-headless-deployment/SKILL.md) | `--no-window` launch modes and `SimulationApp` batch pattern |
| 12 | [`isaac-sim-troubleshooting`](isaac-sim-troubleshooting/SKILL.md) | Kit 110 hang/freeze/perf reference |
| 13 | [`isaac-sim-ros2-bridge`](isaac-sim-ros2-bridge/SKILL.md) | OmniGraph ROS 2 nodes, Nav2 integration |
| 14 | [`spatial-reasoning`](spatial-reasoning/SKILL.md) | Transform math cornerstone |
| 15 | [`manipulation-ik`](manipulation-ik/SKILL.md) | Differential IK, grasp frames, `FixedJoint` grasping |
| 16 | [`isaac-camera`](isaac-camera/SKILL.md) | `UsdGeomCamera` setup, intrinsics, AOVs, distortion |
| 17 | [`usd-pipeline`](usd-pipeline/SKILL.md) | USD asset discovery, bbox/shader measurement, headless compatibility |
| 18 | [`usd-composition-architecture`](usd-composition-architecture/SKILL.md) | NVIDIA's layered USD pattern |
| 19 | [`isaac-sim-validator`](isaac-sim-validator/SKILL.md) | Final QA gate before delivery |
| 20 | [`occupancy-map`](occupancy-map/SKILL.md) | Generate ROS-compatible occupancy maps from USD warehouses |
| 21 | [`meta-skills`](meta-skills/SKILL.md) + [`skill-distillation`](skill-distillation/SKILL.md) | Always-on pair: navigation + step-5 capture |

---

## When stuck

- [`isaac-sim-troubleshooting`](isaac-sim-troubleshooting/SKILL.md) — Kit 110 hang / freeze / perf issues
- [`isaac-sim-validator`](isaac-sim-validator/SKILL.md) — refuses bad deliverables before they reach the user

---

## How to navigate

- **New to this repo?** Start with [Repo-native developer skills](#repo-native-developer-skills-dev-only) — `build-docs`, `cicd`, `debug-with-local-kit`.
- **Starting a new sim task?** → `meta-skills` → `isaac-sim-orchestrator` → match capability to the category tables above.
- **Bringing in a new robot?** → [Robot asset pipeline](#robot-asset-pipeline) → [Physics simulation](#physics-simulation).
- **Generating synthetic data?** → [Synthetic data generation](#synthetic-data-generation).
- **Rendering?** → `isaac-sim-rendering` is the source of truth.
- **Stuck?** → [When stuck](#when-stuck).

---

*Library size: 29 skills (7 repo-native + 22 robotics-sim). Last consolidation: 2026-05-21.*
