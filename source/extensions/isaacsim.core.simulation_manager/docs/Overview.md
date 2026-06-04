# Overview

The isaacsim.core.simulation_manager extension provides APIs to control and query simulation state and physics engine functionality within Isaac Sim. This extension manages physics scenes, simulation time tracking, and event callbacks for physics simulation workflows.

<div align="center">

```{mermaid}
graph TD
    %% Inheritance relationships
    PhysicsScene --> PhysxScene
    
    %% PhysxScene uses PhysxGpuCfg
    PhysxScene -.->|uses| PhysxGpuCfg
    
    %% SimulationManager uses SimulationEvent and IsaacEvents
    SimulationManager -.->|uses| SimulationEvent
    SimulationManager -.->|uses| IsaacEvents
    
    %% SimulationManager uses PhysicsScene (and by extension PhysxScene)
    SimulationManager -.->|uses| PhysicsScene
```

</div>

## Key Components

### {class}`SimulationManager <isaacsim.core.simulation_manager.SimulationManager>`

**The {class}`SimulationManager <isaacsim.core.simulation_manager.SimulationManager>` serves as the central control interface for simulation operations.** It manages physics engine switching, simulation state control, and callback registration for simulation events.

Key functionality includes:
- Physics engine management with support for PhysX and Newton engines
- Simulation stepping and time control with device configuration
- Event callback registration for simulation lifecycle events
- Physics simulation view access for tensor-based operations
- Fabric integration for GPU-accelerated physics workflows

```python
from isaacsim.core.simulation_manager import SimulationManager

# Setup simulation with specific timestep and device
SimulationManager.setup_simulation(dt=1.0/60.0, device="cuda:0")

# Register callback for physics events
def physics_callback(dt, context):
    print(f"Physics step with dt: {dt}")

callback_id = SimulationManager.register_callback(
    physics_callback, 
    event=SimulationEvent.PHYSICS_POST_STEP
)

# Control simulation stepping
SimulationManager.step(steps=10)
```

### Physics Scene Management

**{class}`PhysicsScene <isaacsim.core.simulation_manager.PhysicsScene>` and {class}`PhysxScene <isaacsim.core.simulation_manager.PhysxScene>` classes provide wrappers for USD Physics Scene manipulation.** {class}`PhysicsScene <isaacsim.core.simulation_manager.PhysicsScene>` offers base functionality for physics scene attributes, while {class}`PhysxScene <isaacsim.core.simulation_manager.PhysxScene>` extends this with PhysX-specific features.

The {class}`PhysicsScene <isaacsim.core.simulation_manager.PhysicsScene>` class handles:
- Gravity vector configuration
- Physics timestep (delta time) settings  
- Solver iteration limits
- Physics scene discovery across USD stages

```python
from isaacsim.core.simulation_manager import PhysxScene

# Create or access physics scene
physx_scene = PhysxScene("/World/physicsScene")

# Configure physics parameters
physx_scene.set_gravity((0.0, 0.0, -9.81))
physx_scene.set_dt(0.0166)
physx_scene.set_enabled_gpu_dynamics(True)
```

{class}`PhysxScene <isaacsim.core.simulation_manager.PhysxScene>` adds PhysX-specific capabilities:
- GPU dynamics configuration through {class}`PhysxGpuCfg <isaacsim.core.simulation_manager.PhysxGpuCfg>`
- Solver type selection (TGS/PGS)
- Continuous Collision Detection (CCD) control
- Broadphase algorithm selection
- Stabilization settings

### Simulation Events

**{class}`SimulationEvent <isaacsim.core.simulation_manager.SimulationEvent>` enumeration defines the available simulation lifecycle events** that can trigger registered callbacks. These events include physics stepping, simulation state changes, and USD prim operations.

Available events:
- `PHYSICS_PRE_STEP` / `PHYSICS_POST_STEP`: Before and after physics simulation steps
- `SIMULATION_SETUP` / `SIMULATION_STARTED` / `SIMULATION_PAUSED` / `SIMULATION_RESUMED` / `SIMULATION_STOPPED`: Simulation state transitions
- `PRIM_DELETED`: USD prim deletion notifications

## Integration

The extension integrates with **omni.physics** and **omni.physx.tensors** to provide tensor-based physics simulation views. It uses **omni.physics.stageupdate** for synchronizing USD data with physics simulations and supports both PhysX and Newton physics engines through their respective USD schema extensions.

## Functionality

### Time and State Management

The extension tracks simulation time, physics step counts, and provides both regular and monotonic time queries. It manages simulation state transitions and provides APIs to check if the simulation is running or paused.

### Device Configuration

{class}`SimulationManager <isaacsim.core.simulation_manager.SimulationManager>` supports device selection for physics computations, enabling GPU-accelerated physics when available. This integrates with Warp device management for consistent compute device handling.

### Fabric Integration

The extension provides fabric integration controls to enable GPU-accelerated USD data updates during physics simulation, with APIs to manage fabric-specific USD notice handlers for different stages.

## Simulation Lifecycle

The following diagram illustrates the simulation lifecycle and the events taking place within it.
Refer to the {class}`~isaacsim.core.simulation_manager.SimulationEvent` enum for more details.

These events can be triggered/operated from:

* The application window, via the **Play**, **Pause**, and **Stop** buttons.
* The Core Experimental API {func}`~isaacsim.core.experimental.utils.impl.app.play`,
  {func}`~isaacsim.core.experimental.utils.impl.app.pause`, and
  {func}`~isaacsim.core.experimental.utils.impl.app.stop` utils functions,
  which are a thin and convenient wrapper around the `omni.timeline` API.

```{mermaid}
sequenceDiagram
    participant Timeline as Omniverse<br/>Timeline
    participant SimulationManager
    participant Consumers as Consumers<br/>(Extensions / Users)
    %% play
    Timeline->>+SimulationManager: PLAY
    Note over SimulationManager: Initialize Physics
    SimulationManager->>+Consumers: SIMULATION_SETUP #185;
    Consumers-->-SimulationManager: #160
    SimulationManager->>+Consumers: SIMULATION_STARTED #178;
    Consumers-->-SimulationManager: #160
    SimulationManager-->-Timeline: #160
    %% pause
    opt
    Timeline->>+SimulationManager: PAUSE
    SimulationManager->>+Consumers: SIMULATION_PAUSED
    Consumers-->-SimulationManager: #160
    SimulationManager-->-Timeline: #160
    end
    %% resume
    opt
    Timeline->>+SimulationManager: PLAY
    SimulationManager->>+Consumers: SIMULATION_RESUMED
    Consumers-->-SimulationManager: #160
    SimulationManager-->-Timeline: #160
    end
    %% stop
    opt
    Timeline->>+SimulationManager: STOP
    Note over SimulationManager: Invalidate Physics
    SimulationManager->>+Consumers: SIMULATION_STOPPED
    Consumers-->-SimulationManager: #160
    SimulationManager-->-Timeline: #160
    end
```

**Notes:**

* [1,2] When the application is played for the first time (or after being stopped),
  the {attr}`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_SETUP` and
  the {attr}`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_STARTED` events
  are triggered one after the other. The {attr}`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_SETUP`
  event is used by the Isaac Sim Core extensions, among other extensions, to prepare the physics tensor entities, for example.

  Therefore, from the user perspective and to avoid conflicts, it is recommended that the
  {attr}`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_STARTED` event be used
  to carry out custom preparatory procedures just before the simulation progresses rather than the
  {attr}`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_SETUP`.
