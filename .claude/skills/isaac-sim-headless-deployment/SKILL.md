---
name: isaac-sim-headless-deployment
description: "Run a built Isaac Sim 6 headless with --no-window and the renderer disabled. Use for batch SDG jobs, physics-only CI, or unattended server runs."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# Isaac Sim Headless Usage (`--no-window`)

## Purpose

Launch Isaac Sim without a window for batch simulation, automated data generation, and server workloads using supported CLI flags and `SimulationApp` patterns.

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

This skill covers headless usage of a built or installed Isaac Sim. It does
not cover Docker images, container orchestration, or CI/CD wiring — those
live with the deployment infrastructure that wraps Isaac Sim, not in this
skill.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/batch_simulation.py` | Headless batch simulation loop for Isaac Sim (Kit 110) | see script --help |
| `scripts/ci_physics_nightly.sh` | CI physics-only nightly runner via python.sh or Kit CLI | `<test_script.py> [args...]` |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/batch_simulation.py", args=["--help"])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`, or execute shell helpers directly when they do not require the simulator.

## Launch modes

### 1. Headed (Default)

```bash
./isaac-sim.sh
# Launches isaacsim.exp.full.kit with full UI
```

### 2. Headless with Rendering

For headless GPU rendering (generates images without a window):

```bash
# Native headless — uses EGL/Vulkan offscreen
./isaac-sim.sh --no-window

# With explicit display (required on some systems)
DISPLAY=:0 ./isaac-sim.sh --no-window
```

### 3. Headless No Rendering

For physics-only simulation (no viewport, no rendering):

```bash
./isaac-sim.sh --no-window --/app/renderer/enabled=false
```

### 4. Python Standalone Scripts

Run simulation logic from a standalone Python script:

```bash
./python.sh my_simulation.py
# Uses Isaac Sim's Python environment with all extensions available
```

The script controls headless behavior via the `SimulationApp` config (see
[Batch simulation pattern](#batch-simulation-pattern) below) — `python.sh`
does not need a `--no-window` flag.

### 5. Kit CLI with Script Execution

```bash
# Default — full Isaac Sim experience with rendering enabled
./kit/kit apps/isaacsim.exp.full.kit --exec "my_script.py"

# Headless physics-only batch
./kit/kit apps/isaacsim.exp.full.kit \
    --no-window \
    --/app/renderer/enabled=false \
    --exec "physics_batch.py"
```

## Key CLI Flags

| Flag | Description |
|------|-------------|
| `--no-window` | Disable window creation (headless) |
| `--/app/renderer/enabled=false` | Disable rendering entirely |
| `--/app/window/width=W` | Set viewport width |
| `--/app/window/height=H` | Set viewport height |
| `--/app/settings/fabricDefaultStageFrameHistoryCount=N` | Frame history for Fabric |
| `--enable EXT_ID` | Enable specific extension |
| `--ext-folder PATH` | Add extension search path |
| `--exec SCRIPT` | Execute script on startup |
| `--no-ros-env` | Skip ROS 2 auto-sourcing |

## App Configurations

| Config File | Use Case |
|-------------|----------|
| `isaacsim.exp.full.kit` | Full simulation (default) |
| `isaacsim.exp.full.newton.kit` | Newton physics engine |
| `isaacsim.exp.full.fabric.kit` | Fabric scene delegate |
| `isaacsim.exp.base.kit` | Minimal base (faster startup) |
| `isaacsim.exp.base.python.kit` | Python-only base |
| `isaacsim.exp.base.python.physics.headless.kit` | Physics-only headless (CI nightly) |
| `isaacsim.exp.base.zero_delay.kit` | Zero-delay base |

## CI Physics-Only Nightly Runs

For physics-only nightly CI (no rendering, no window), use the dedicated kit file:

```bash
# Kit CLI mode — self-contained headless physics config
./kit/kit apps/isaacsim.exp.base.python.physics.headless.kit \
    --no-window \
    --/app/renderer/enabled=false \
    --exec "test_physics_batch.py"
```

For standalone Python scripts that bootstrap `SimulationApp` themselves, use
`python.sh` with renderer-disable flags:

```bash
# python.sh mode — flags passed through to Carbonite/Kit settings
./python.sh test_physics_batch.py \
    --/renderer/enabled=false \
    --/app/window/enabled=false \
    --/app/livestream/enabled=false
```

The kit file (`isaacsim.exp.base.python.physics.headless.kit`) bakes in:
- `renderer.enabled = false` — no GPU rendering subsystem
- `app.window.enabled = false` — no window creation
- `app.livestream.enabled = false` — no streaming
- All UI/viewport/manipulator extensions disabled
- All rendering extensions disabled (`omni.hydra.rtx`, `omni.rtx.*`, `omni.syntheticdata`, `omni.replicator.core`)
- Physics-optimal loop settings (`rateLimitEnabled = false`, `manualModeEnabled = true`, `useFixedTimeStepping = true`)

The wrapper script `scripts/ci_physics_nightly.sh` encapsulates both modes.
Set `CI_KIT_MODE=1` for Kit CLI, or leave unset for `python.sh` (default).

## Batch Simulation Pattern

The canonical headless bootstrap pattern lives in:

- `source/standalone_examples/api/isaacsim.simulation_app/hello_world.py` (minimal)
- `source/standalone_examples/api/isaacsim.simulation_app/load_stage.py` (stage open + step loop with `--test` mode)

For an N-episode batch driver, wrap that bootstrap in a Python `for episode in range(N)` loop. Use the **current** APIs:

- `isaacsim.core.experimental.utils.stage` for stage open / waiting on load
- `isaacsim.core.simulation_manager.SimulationManager.setup_simulation(dt=..., device=...)` to configure physics
- `isaacsim.core.experimental.utils.app` (`enable_extension`, `play`, `stop`, `update_app`) to drive the loop

Skeleton (paraphrased from `load_stage.py`, adapted for batch):

`run_batch_simulation(scene_path, num_episodes, steps_per_episode, dt, device)` — headless SimulationApp loop that opens a USD, waits for load, sets up physics, steps, and stops each episode.

See [`scripts/batch_simulation.py`](scripts/batch_simulation.py).

Avoid the legacy `isaacsim.core.api.World` / `isaacsim.core.utils.stage` paths — they are deprecated in Kit 110.

> **Migration:** see [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html) for the `omni.isaac.*` → `isaacsim.*` map covering `omni.isaac.core`, `omni.isaac.utils`, and friends.

## Performance Tuning for Batch/Headless

### Physics-Only (Fastest)

```python
config = {"headless": True, "renderer": None}
# In step loop:
world.step(render=False)
```

**Throughput:** 10-50x real-time depending on scene complexity.

### Memory Management

- **Clear stage between episodes**: `world.clear()` + garbage collect
- **Use instanceable assets**: `make_instanceable: true` in config.yaml
- **Monitor GPU memory**: `nvidia-smi` — Kit leaks if stages aren't properly cleared
- **GB10 (Jetson)**: Max ~35K prims, 16GB shared memory — reduce scene complexity

## Known Issues (Kit 110)

- **DISPLAY=:0 required** for headless viewport on Kit 110 with complex stages on some GPU configurations
- **Viewport never initializes** without DISPLAY on some configurations — use `--no-window` + offscreen rendering
- **CUDA OOM**: Monitor with `nvidia-smi`, keep under 90% GPU VRAM utilization
- **Startup time**: ~30-60s cold start; use persistent processes for interactive batch
- **Newton physics + torch conflict**: Never import torch before physics settle
