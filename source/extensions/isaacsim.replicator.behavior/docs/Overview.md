# Overview

The isaacsim.replicator.behavior extension provides a collection of behavior scripts for Isaac Sim's Synthetic Data Generation (SDG) workflows. These scripts enable procedural randomization and dynamic behaviors that can be attached to USD prims, making simulation scenarios more diverse and realistic for training machine learning models.

## Functionality

The extension offers several categories of behavior scripts that operate during simulation playback:

**Randomization Behaviors** apply procedural variations to scene elements:
- LocationRandomizer moves prims within specified spatial bounds
- RotationRandomizer applies random rotations using various xformOps
- TextureRandomizer creates and randomly applies materials from texture lists
- LightRandomizer varies light properties like intensity and color

**Interactive Behaviors** create dynamic object interactions:
- LookAtBehavior orients prims to track target locations or other prims
- VolumeStackRandomizer drops and stacks assets within designated areas

**Custom Event Behaviors** enable complex scripted sequences through event-driven workflows with configurable state machines.

## Key Components

### BaseBehavior Class

BaseBehavior provides the foundation for creating custom behavior scripts. It extends **omni.kit.scripting.BehaviorScript** with a standardized system for exposing script variables as USD attributes, making them accessible through the Property Panel UI.

The class defines two key properties that subclasses must implement:
- BEHAVIOR_NS: A namespace identifier for organizing related behaviors
- VARIABLES_TO_EXPOSE: Configuration data that determines which script variables become USD attributes

### USD Attribute System

The extension automatically creates USD attributes from behavior script variables, enabling persistent storage and UI editing. When a behavior script is assigned to a prim, the system:

1. Creates corresponding USD attributes for exposed variables
2. Synchronizes attribute values with script properties
3. Makes attributes editable through the Property Panel
4. Maintains attribute persistence across sessions

This system allows users to configure behavior parameters directly in the USD stage without modifying Python code.

### Event-Driven Architecture

Advanced behaviors like VolumeStackRandomizer use an event system for complex operations. These behaviors can:
- Publish custom events during execution
- Subscribe to events from other behaviors
- Coordinate multi-step operations across simulation frames
- Maintain state through BehaviorState enumeration

### Transform and Physics Integration

The extension provides utilities for manipulating USD transforms and physics properties:
- Transform operations handle location, rotation, and scale modifications
- Physics integration supports collider creation, rigid body dynamics, and material assignment
- Simulation control enables programmatic stepping and force application

## Usage Examples

Behavior scripts are applied to prims through the Isaac Sim interface or programmatically:

```python
import isaacsim.replicator.behavior as behavior

# Apply location randomization to a prim
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/MyObject")

# Add behavior script with custom parameters
await behavior.add_behavior_script_with_parameters_async(
    prim,
    "isaacsim.replicator.behavior.LocationRandomizer",
    exposed_variables={
        "min_location": (-5, 0, -5),
        "max_location": (5, 2, 5),
        "seed": 42
    }
)
```

Multiple behavior scripts can be applied to the same prim, and each script's parameters are stored as USD attributes that persist with the stage file.

## Integration

The extension integrates with Isaac Sim's physics simulation through **omni.physics** and **omni.physx** dependencies. Behaviors automatically respond to simulation events:
- on_play() executes when simulation starts
- on_update() runs each simulation frame
- on_stop() handles cleanup when simulation ends

The **omni.kit.scripting** integration provides the behavior script framework and UI integration, while USD attribute exposure enables seamless workflow integration with existing Isaac Sim tools and Property Panel editing capabilities.
