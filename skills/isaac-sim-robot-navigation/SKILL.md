---
name: isaac-sim-robot-navigation
description: "Runtime mobile-robot navigation in custom scripts (RL, baked, per-frame). Use when driving robots live in simulation."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# Isaac Sim Robot Navigation — Runtime

## Purpose

Drive mobile robots at runtime with RL policies, trajectory followers, and physics/baked/per-frame stage strategies while avoiding common Kit 110 pitfalls.

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

Runtime navigation specialization. Driving a robot through a USD scene live, in your own Isaac Sim script.

## Read These Skills First

- **navigation-primitives** — occupancy maps, A* planning, differential/holonomic kinematics, robot footprints (Spot Z=0.69, etc.), look-at chase camera math, common gotchas. This skill *assumes* you know that substrate.
- **isaac-sim-rendering** — RT2, persistent session, ACES, frame validation
- **isaac-sim-troubleshooting** — Kit 110 hang/freeze reference

## When To Use This Skill (vs siblings)

| Goal | Use |
|---|---|
| Drive a robot through a scene in real time, see it move | **this skill** |
| Record trajectories then re-render with sensors for SDG | `mobility-gen` |
| Publish/subscribe Nav2 topics to ROS 2 | `isaac-sim-ros2-bridge` |

## Runtime APIs

| Capability | API |
|---|---|
| RL policy execution | `isaacsim.robot.policy.examples.RobotPolicyRunner` + root `get_*_spec` factory |
| Robot articulation | `isaacsim.core.experimental.prims.Articulation` |
| Physics lifecycle | `isaacsim.core.simulation_manager.SimulationManager` |

For shared primitives (omap, A*, kinematics, look-at), see `navigation-primitives`.

## Physics Simulation on Kit 110 (CRITICAL)

### Timeline MUST be playing for Physics simulation
```python
import isaacsim.core.experimental.utils.app as app_utils
app_utils.play(commit=True)  # WITHOUT THIS, PHYSICS NEVER STEPS
```

### Strip non-robot physics from heavy stages
On 33K+ prim stages with multiple physic bodies, simulation hangs. Strip ALL physics from non-robot prims:

```python
from pxr import Usd, UsdPhysics

for p in Usd.PrimRange(stage.GetPseudoRoot()):
    if str(p.GetPath()).startswith("/World/Robot"):
        continue
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        p.RemoveAPI(UsdPhysics.RigidBodyAPI)
    if p.HasAPI(UsdPhysics.CollisionAPI):
        p.RemoveAPI(UsdPhysics.CollisionAPI)
```
## Kit 110 Exec Model (CRITICAL — 2026-03-15)

- **Synchronous code executes immediately** in `--exec` scripts. No async needed for scene setup.
- **`asyncio.ensure_future()` needs explicit event loop subscription**:
  ```python
  omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(lambda e: None)
  asyncio.ensure_future(main())
  ```
- **`capture_viewport_to_file()` returns `MultiAOVFileCapture`** — use `.wait_for_result(completion_frames=8)`, NOT `.wait()`.
- Pattern: sync scene build → async render scheduling → event loop keeps it alive
- Scene load + 100 asset placement + 3-camera render in ~40s total

## Camera Chase Cam

The look-at math and standard camera offsets (chase/overhead/POV) live in `navigation-primitives`. Runtime considerations on top of that:

- **Camera collision avoidance** (TODO): chase camera clips into rack geometry when robot enters narrow aisles. Need raycast from chase eye → robot; if occluded, raise camera or shift laterally. Not yet implemented.

## Failure Modes & Recovery

**FAILURE:** `VkResult: ERROR_OUT_OF_DEVICE_MEMORY` during navigation render
- SYMPTOMS: Vulkan crash partway through capture loop on heavy stage
- CAUSE: Approach B (baked timeSamples) + RT2 + 50K+ prims
- FIX: Switch to approach C (per-frame transform)

**FAILURE:** Robot floats above ground or falls through floor
- CAUSE: Wrong Z offset
- FIX: See `navigation-primitives` Robot Footprint table

**FAILURE:** Physics never steps after kill+relaunch
- CAUSE: Kill+relaunch bug, dirty GPU state
- FIX: Wait 30+ seconds, verify zero Kit processes, or reboot

**FAILURE:** A* path looks fine on omap but robot clips a rack at render
- CAUSE: Skipped step 5/6 of the validation pipeline (see `navigation-primitives`)
- FIX: Always validate every smoothed point is in navigable space



## Integration Points

- **RECEIVES from:** `navigation-primitives` — omap, A*, kinematics, robot specs, camera math
- **RECEIVES from:** `occupancy-map` — `map.yaml` or runtime grid
- **RECEIVES from:** `urdf-mjcf-to-usd-conversion` — robot USD
- **PRODUCES for:** `isaac-sim-rendering` — animated scene ready for RT2 capture and frame sequences of nav runs
