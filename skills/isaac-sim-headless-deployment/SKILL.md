---
name: isaac-sim-headless-deployment
description: >
  Running Isaac Sim 6.0 headlessly with `--no-window` for batch simulation,
  automated data generation, and server-side workloads. Covers launch modes
  (native headless, physics-only, standalone Python), key CLI flags, app
  configurations, and the canonical `SimulationApp` bootstrap pattern.
  Use when: launching Isaac Sim without a display, running batch simulations
  on a server, generating synthetic data headlessly, or wrapping a standalone
  Python script in the headless `SimulationApp` lifecycle.
  Triggers on: headless Isaac Sim, --no-window, batch simulation,
  SimulationApp, physics-only, standalone Python script.
---

# Isaac Sim Headless Usage (`--no-window`)

This skill covers headless usage of a built or installed Isaac Sim. It does
not cover Docker images, container orchestration, or CI/CD wiring — those
live with the deployment infrastructure that wraps Isaac Sim, not in this
skill.

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

<!-- CODE: scripts/kit_cli_with_script_execution.sh -->
*[Code: `scripts/kit_cli_with_script_execution.sh`]*

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
| `--allow-root` | Allow running as root |

## App Configurations

| Config File | Use Case |
|-------------|----------|
| `isaacsim.exp.full.kit` | Full simulation (default) |
| `isaacsim.exp.full.newton.kit` | Newton physics engine |
| `isaacsim.exp.full.fabric.kit` | Fabric scene delegate |
| `isaacsim.exp.base.kit` | Minimal base (faster startup) |
| `isaacsim.exp.base.python.kit` | Python-only base |
| `isaacsim.exp.base.zero_delay.kit` | Zero-delay base |

## Batch Simulation Pattern

The canonical headless bootstrap pattern lives in:

- `source/standalone_examples/api/isaacsim.simulation_app/hello_world.py` (minimal)
- `source/standalone_examples/api/isaacsim.simulation_app/load_stage.py` (stage open + step loop with `--test` mode)

For an N-episode batch driver, wrap that bootstrap in a Python `for episode in range(N)` loop. Use the **current** APIs:

- `isaacsim.core.experimental.utils.stage` for stage open / waiting on load
- `isaacsim.core.simulation_manager.SimulationManager.setup_simulation(dt=..., device=...)` to configure physics
- `isaacsim.core.experimental.utils.app` (`enable_extension`, `play`, `stop`, `update_app`) to drive the loop

Skeleton (paraphrased from `load_stage.py`, adapted for batch):

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

NUM_EPISODES, STEPS_PER_EPISODE = 100, 1000
for episode in range(NUM_EPISODES):
    stage_utils.open_stage("/path/to/scene.usd")
    while stage_utils.is_stage_loading():
        simulation_app.update()
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    for _ in range(STEPS_PER_EPISODE):
        simulation_app.update()
    app_utils.stop()

simulation_app.close()
```

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
