# Overview

The isaacsim.asset.gen.conveyor extension provides utilities for authoring and simulating conveyor belts in Isaac Sim. It includes a C++ plugin for runtime conveyor physics, an OmniGraph node for controlling conveyor behavior during simulation, and Python commands for programmatic conveyor creation.

## Key Components

### Python API — create_conveyor_belt

The `create_conveyor_belt` function creates an Action Graph containing the IsaacConveyor node and wires it to a target rigid body prim. If the selected prim does not already have a rigid body API, the function applies one.

```python
from isaacsim.asset.gen.conveyor import create_conveyor_belt

stage = omni.usd.get_context().get_stage()
conveyor_prim = stage.GetPrimAtPath("/World/ConveyorBeltRigidBody")
conveyor_node = create_conveyor_belt(stage, conveyor_prim, prim_name="ConveyorActionGraph")
```

> **Deprecated:** The `CreateConveyorBelt` Kit command is deprecated. Use `create_conveyor_belt(stage, conveyor_prim)` directly instead.

### C++ Plugin

A native Carbonite plugin (`IOmniIsaacConveyor`) provides the runtime interface used by the OmniGraph node for conveyor physics computations.

## Usage

To add a conveyor belt interactively, use **Create > Isaac Sim > Warehouse Items > Conveyor**. For track-based conveyor systems built from Digital Twin Assets, see the companion extension `isaacsim.asset.gen.conveyor.ui` and its **Tools > Conveyor Track Builder** window.
