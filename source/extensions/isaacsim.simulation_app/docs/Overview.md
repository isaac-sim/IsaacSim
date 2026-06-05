# Overview

The isaacsim.simulation_app extension provides Python classes for launching and controlling Omniverse applications from Python code. This extension enables developers to programmatically start Isaac Sim applications, manage their lifecycle, and interact with the underlying Omniverse framework through Python scripts.

## Key Components

### {class}`SimulationApp <isaacsim.simulation_app.SimulationApp>`

**{class}`SimulationApp <isaacsim.simulation_app.SimulationApp>` is the primary helper class for launching fully-featured Isaac Sim applications.** It handles the complex initialization process of Omniverse Toolkit, which must be running before other Omniverse modules can be imported.

The class provides comprehensive configuration through launch dictionaries and experience file paths. It automatically loads one of several default experience files if none is specified, following a priority order from most specific (omni.isaac.sim.python.kit) to most general (isaacsim.exp.base.kit).

```python
from isaacsim.simulation_app import SimulationApp

config = {
    "width": 1280,
    "height": 720,
    "headless": False,
}
simulation_app = SimulationApp(config)
# Application is now running and ready for use
simulation_app.close()
```

#### Application Control

{class}`SimulationApp <isaacsim.simulation_app.SimulationApp>` provides several methods for controlling application execution:

- `update()` advances the simulation by one frame
- `run_coroutine()` executes asynchronous tasks within Kit's event loop system
- `set_setting()` modifies Carbonite framework settings at runtime
- `reset_render_settings()` reapplies rendering configuration when opening new stages

#### State Management

The class tracks application state through `is_running()` and `is_exiting()` methods, allowing scripts to properly handle application lifecycle. The `close()` method supports both graceful shutdown with cleanup and immediate exit modes.

### {class}`AppFramework <isaacsim.simulation_app.AppFramework>`

**{class}`AppFramework <isaacsim.simulation_app.AppFramework>` provides a minimal Omniverse application launcher without default configuration.** This class is designed for cases where developers need complete control over application setup or want to build custom experiences from scratch.

```python
from isaacsim.simulation_app import AppFramework

app_framework = AppFramework(name="my_app", argv=["--custom-arg"])
app_framework.update()
app_framework.close()
```

Unlike {class}`SimulationApp <isaacsim.simulation_app.SimulationApp>`, {class}`AppFramework <isaacsim.simulation_app.AppFramework>` exposes the underlying Carb framework directly through its `framework` property, enabling low-level system access for advanced use cases.

## Integration

The extension integrates with **omni.usd** to provide USD context access through the `context` property, enabling scripts to interact with USD stages and perform scene operations after application launch.
