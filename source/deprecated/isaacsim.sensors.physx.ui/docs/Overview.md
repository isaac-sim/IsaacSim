# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.sensors.physics.ui`.
```

The isaacsim.sensors.physx.ui extension provides UI components for Isaac Sim PhysX-raycast-based sensor simulation, specifically menu options for range sensor operations. This extension acts as a UI layer on top of the PhysX sensor system, enabling users to interact with range sensors through menu interfaces.

```{image} ../data/preview.png
---
align: center
---
```


## Key Components

### {class}`RangeSensorMenu <isaacsim.sensors.physx.ui.RangeSensorMenu>`

The {class}`RangeSensorMenu <isaacsim.sensors.physx.ui.RangeSensorMenu>` class provides menu integration for range sensor operations within Isaac Sim. This menu component allows users to access range sensor functionality through the application's menu system, offering a convenient interface for sensor configuration and control.

The menu integrates with Isaac Sim's context menu system to provide sensor-specific options when working with range sensors in the scene. Users can access range sensor operations directly from the interface without needing to write code or use lower-level APIs.

## Relationships

This extension depends on isaacsim.sensors.physx for the underlying PhysX sensor simulation capabilities and isaacsim.gui.components for UI component infrastructure. It uses **omni.kit.actions.core** for action registration and **omni.kit.context_menu** for menu integration, enabling the menu components to appear in appropriate contexts within the Isaac Sim interface.
