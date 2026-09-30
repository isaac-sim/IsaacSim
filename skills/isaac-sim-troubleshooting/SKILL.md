---
name: isaac-sim-troubleshooting
description: "Reference for Isaac Sim hangs, freezes, and performance stalls. Use when startup, MDL, physics, or Replicator blocks progress."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# Isaac Sim 6 Troubleshooting — Large USD Scene Hangs

## Purpose

Diagnose Isaac Sim startup hangs, stage-load stalls, MDL/shader compilation freezes, physics stepping blocks, Replicator/Hydra issues, Nucleus latency, and GPU OOM crashes.

## Prerequisites

- Built Isaac Sim (`$ISAAC_SIM_DIR` or `_build/linux-x86_64/release`).
- NVIDIA GPU with a current driver (`nvidia-smi`).
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR`.

## Limitations

- Symptom lists are heuristic; root causes can overlap across subsystems.
- Some fixes require local Kit builds or NVIDIA internal-only packages.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Extension or import not found | Wrong `$ISAAC_SIM_DIR` or stale build | Point env vars at `_build/linux-x86_64/release` or rebuild |
| Black or empty frames | Missing lights or non-RTX render mode | Add dome/key light; confirm RTX / PathTracing settings |
| Hang on stage load or first render | MDL compile or oversized stage | Follow isolation steps in `isaac-sim-troubleshooting` |
| Hang on shutdown / `SimulationApp.close()` | `carb.tasking` thread pool deadlock during `shutdown_and_release_framework()` | Use SIGALRM watchdog (see Section 1a) — threading.Event cannot fire when GIL is held by the deadlocking C extension |

Diagnosis and resolution for Isaac Sim 6.0 (Kit 110) hangs, freezes, and perf degradation on large USD scenes (10K-150K+ prims, factory/warehouse twins).

Upstream debugging docs: `docs/isaacsim/utilities/debugging/` (profiling, Python debugging tutorials). Pair this skill with `profile-isaac-sim` for Tracy-based diagnosis.

## Quick Diagnosis Flowchart

```
Isaac Sim hangs/freezes
    |
    +-- During startup (before stage load)?
    |       -> Section 1: Startup Hangs
    |
    +-- During shutdown / SimulationApp.close()?
    |       -> Section 1a: Shutdown Hangs (Kit Framework Deadlock)
    |
    +-- During stage open / USD loading?
    |       -> Section 2: Stage Loading Hangs
    |
    +-- During shader/material compilation?
    |       -> Section 3: MDL/Shader Compilation
    |
    +-- During physics stepping / `SimulationManager` setup or timeline play?
    |       -> Section 4: Physics Hangs
    |
    +-- During Replicator / sensor capture?
    |       -> Section 5: Replicator Hangs
    |
    +-- During rendering (viewport frozen)?
    |       -> Section 6: Hydra/Rendering Hangs
    |
    +-- Nucleus/remote asset fetch?
    |       -> Section 7: Nucleus Network Hangs
    |
    +-- OOM crash or memory spike?
    |       -> Section 8: Memory Issues
    |
    +-- Slow but not frozen?
            -> Section 9: Performance Optimization
```

## Section 1: Startup Hangs

### Common Causes and Fixes

**1. Extension conflict (circular import)**
```bash
./isaac-sim.sh --/app/extensions/exclude='["problematic.extension"]'
```

**2. Renderer initialization freeze**
```bash
./isaac-sim.sh --vulkan         # Force Vulkan
./isaac-sim.sh --reset-user     # Reset user settings
```

**3. Thread over-subscription on high-core-count CPUs**
```bash
./isaac-sim.sh \
    --/plugins/carb.tasking.plugin/threadCount=16 \
    --/plugins/omni.tbb.globalcontrol/maxThreadCount=16
```

```python
simulation_app = SimulationApp({"headless": False, "limit_cpu_threads": 16})
```

**4. Nucleus login popup hang** — Force-kill Isaac Sim and restart. Complete login before proceeding.

## Section 1a: Shutdown Hangs (Kit Framework Deadlock)

### Symptom

`SimulationApp.close()` never returns. Process hangs indefinitely (or until CI kills it at 1500s). Stack trace shows threads blocked in `futex_wait` inside `carb.tasking` thread pool teardown during `shutdown_and_release_framework()` or `unload_all_plugins()`.

### Root Cause

`shutdown_and_release_framework()` calls `app->shutdown()` then `framework->unloadAllPlugins()`. These are C extension calls that hold the GIL while internally deadlocking on native thread joins. A Python-side `threading.Event.wait()` watchdog cannot fire because it needs the GIL to check its timeout and call `os.kill()` — it starves.

### Fix: SIGALRM Watchdog

`SIGALRM` is kernel-delivered regardless of GIL state. Set an alarm before the blocking C call; if it deadlocks, the alarm handler SIGKILLs the process unconditionally.

Pattern (generalized — apply to any potentially-deadlocking native teardown):
1. Save the previous `SIGALRM` handler.
2. Install a handler that calls `os.kill(os.getpid(), signal.SIGKILL)`.
3. `signal.alarm(timeout_seconds)` — 120s for full shutdown, 30s for plugin unload.
4. Call the blocking native function.
5. In `finally`: cancel the alarm (`signal.alarm(0)`) and restore the previous handler.

### When to Suspect This

- CI job hangs after all test assertions pass (shutdown phase).
- `strace` shows the main thread stuck in `futex_wait` inside Kit's plugin system.
- Python-based timeout mechanisms (threading, asyncio) silently fail to trigger.

### Key Insight

Any time a Python watchdog must guard a C extension call that can hold the GIL indefinitely, `threading.Event`/`threading.Timer` will be starved. Use `signal.alarm()` (Linux) or `os.alarm()` as the only reliable GIL-independent timeout mechanism in CPython.

### Reference

- File: `source/extensions/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py`

## Section 2: Stage Loading Hangs

### Diagnosis

Check layer count:
```python
for layer in stage.GetUsedLayers():
    print(f"  {layer.GetDisplayName()} ({layer.GetFileFormat().formatId})")
```
If layer count > 1,000, this is the primary bottleneck.

### Common Causes

**1. Excessive layer count (>1,000 layers)**

| Strategy | Cold Load | Layers |
|----------|-----------|--------|
| Per-asset files | 4 min | 11,488 |
| Library packaging | 53s | 8 |

Fix: Package assets into library layers (consolidate per-asset files into a small number of library `.usd`/`.usdc` layers; see `usd-composition-architecture` for the layered-asset pattern).

**2. Missing/broken references** — Cause USD to try every resolver including network fallbacks. Fix: Audit references:
```python
from pxr import Sdf
for layer in stage.GetUsedLayers():
    for ref in layer.GetExternalReferences():
        if not os.path.exists(ref):
            print(f"MISSING: {ref}")
```

## Section 3: Shader/MDL Compilation

- First launch always compiles MDL shaders — can take 5-15 minutes, normal
- Subsequent launches use shader cache
- Cache location: `~/.nvidia-omniverse/data/Kit/Isaac-Sim/shader_cache/`
- Force rebuild: delete cache folder

## Section 4: Physics Hangs

**Physics setup / first-play hangs** (`SimulationManager.setup_simulation`, `timeline.play()`, or the legacy `world.reset()` flow — see [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html) to migrate off `omni.isaac.core.World`): usually caused by:
- `PhysicsScene` not yet defined when physics starts.
- Too many contact pairs on the first step.
- GPU dynamics enabled with too many rigid bodies (>100K).

Fix:
```python
# Ensure PhysicsScene exists before reset
if not stage.GetPrimAtPath("/World/PhysicsScene"):
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    
# Reduce initial contact storm
px_scene.CreateEnableStabilizationAttr().Set(True)
```

## Section 5: Replicator Hangs

- `rep.orchestrator.run()` hangs: ensure the timeline is playing (`omni.timeline.get_timeline_interface().play()` or `isaacsim.core.experimental.utils.app.play(commit=True)`) before invoking.
- `step_async()` never completes: use `await rep.orchestrator.step_async()` correctly inside an async context.
- Frame capture hangs: call `app_utils.update_app()` (or a `simulation_app.update()`) once before capture so the renderer has a fresh frame.

## Section 6: Rendering (Hydra) Hangs

- Viewport black and frozen: Check GPU memory (`nvidia-smi`)
- RTX renderer OOM: Reduce texture streaming budget:
  ```python
  carb.settings.get_settings().set("/rtx/resourcemanager/textureMipCountBudget", 256)
  ```

## Section 7: Nucleus Network Hangs

- Set timeout: `carb.settings.set("/omni/client/timeout_seconds", 10.0)`
- Use local assets when possible: Copy needed USDs to local disk
- Disable Nucleus: Launch with `--/omni/client/enabled=false` for local-only work

## Section 8: Memory Issues

- Prims > 150K: Enable instancing (`UsdGeom.PointInstancer`)
- Textures: Use texture atlases, reduce resolution
- Monitor: `watch -n1 'nvidia-smi --query-gpu=memory.used,memory.free --format=csv'`

## Section 9: Performance Optimization

- Layer count: Package into library layers (see Section 2)
- Mesh instancing: Use `PointInstancer` for repeated assets
- Payload loading: Mark large sub-scenes as unloaded payloads
- Physics: Reduce collision mesh complexity for non-critical objects
