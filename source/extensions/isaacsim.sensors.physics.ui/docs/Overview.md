# Overview

The isaacsim.sensors.physics.ui extension provides menu integration for Isaac Sim physics-based sensor simulation. This extension adds menu items that enable users to create and configure physics sensors through the application's menu system, making sensor creation accessible through standard UI interactions.

```{image} ../../../../source/extensions/isaacsim.sensors.physics.ui/data/preview.png
---
align: center
---
```


## Functionality

### Menu Registration

The extension registers menu items specifically for physics sensor creation when it starts up. These menu entries provide users with direct access to physics sensor functionality without requiring programmatic interaction.

### Sensor Creation Interface

**Physics sensor menu options** allow users to create various types of physics-based sensors through the menu system. The extension integrates with the existing Isaac Sim menu structure to provide intuitive access to sensor creation workflows.

## Integration

The extension uses **omni.kit.actions.core** to register sensor-related actions that can be triggered through menu selections. It also integrates with **omni.kit.context_menu** to provide contextual menu options for physics sensor operations.

The extension connects with isaacsim.sensors.experimental.physics to access the underlying physics sensor functionality and uses isaacsim.core.experimental.prims for primitive-based sensor operations. Additionally, it leverages isaacsim.gui.components for consistent UI component integration within the Isaac Sim interface.
