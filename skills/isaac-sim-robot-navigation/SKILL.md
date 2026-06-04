---
name: isaac-sim-robot-navigation
description: >
  Runtime robot navigation in NVIDIA Isaac Sim — driving a robot through a scene live in
  a custom script. Covers RL policy loading (PolicyController), trajectory following,
  physics-vs-baked-vs-per-frame approaches for heavy stages, GPU OOM avoidance on
  98K-prim scenes, Kit 110-specific timeline/physics gotchas, and standard Spot route
  catalog. For occupancy maps, A*, kinematics, and camera math, see `navigation-primitives`.
  For sensor-recording SDG, see `mobility-gen`.
  Use when navigating a robot live in your own Isaac Sim script, choosing between physics
  simulation and pre-baked transforms for heavy stages, debugging GPU OOM on large scenes,
  or troubleshooting Kit 110 physics hangs after kill+relaunch.
  Triggers on: PolicyController, RL policy load, trajectory follow, baked vs physics,
  per-frame transform, GPU OOM navigation, Kit 110 timeline.play, robot navigation runtime.
---

# Isaac Sim Robot Navigation — Runtime

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
| RL policy execution | `isaacsim.robot.policy.examples.controllers.PolicyController` |
| Robot articulation | `isaacsim.core.experimental.prims.Articulation` |
| Physics lifecycle | `isaacsim.core.simulation_manager.SimulationManager` |

For shared primitives (omap, A*, kinematics, look-at), see `navigation-primitives`.

## Physics Simulation on Kit 110 (CRITICAL)

### Timeline MUST be playing for Physics simulation
```python
import omni.timeline
timeline = omni.timeline.get_timeline_interface()
timeline.play()  # WITHOUT THIS, PHYSICS NEVER STEPS
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
