# Overview

This extension provides UI integration for creating RTX sensors in Isaac Sim. It adds menu items and actions that allow users to create RTX Lidar and RTX Radar sensors directly from the Create menu and viewport context menus, streamlining the sensor creation workflow within the Isaac Sim interface.

```{image} ../../../../source/extensions/isaacsim.sensors.rtx.ui/data/preview.png
---
align: center
---
```


## Functionality

### RTX Sensor Creation

The extension enables users to create RTX-based sensors through familiar UI interactions. RTX Lidar sensors are organized by vendor-specific configurations, allowing users to select from multiple manufacturer presets. RTX Radar sensors are also available for creation through the same interface.

When sensors are created, they are automatically placed at the next available path in the scene hierarchy. If prims are selected in the scene, newly created sensors can be automatically parented to those selected objects.

### Menu Integration

**Create Menu Integration**: RTX sensor creation options are integrated into the main Create menu, providing a consistent location for sensor creation alongside other USD primitive creation tools.

**Context Menu Support**: Right-click context menus in the viewport provide quick access to sensor creation capabilities, enabling efficient workflow integration during scene composition.

### Vendor Organization

RTX Lidar sensors are organized into vendor-specific submenus, making it easy for users to locate and select sensors based on manufacturer specifications. This organization helps users quickly identify the appropriate sensor configuration for their simulation requirements.

## Key Components

### Action Registration

The extension registers sensor creation actions with the action registry system, enabling these operations to be executed from multiple UI entry points. Each RTX sensor type has corresponding actions that handle the creation and placement logic.

### Menu Builder

A menu hierarchy system organizes sensor options into logical groupings, with RTX Lidar sensors categorized by vendor and RTX Radar sensors presented as separate options. This structure provides clear navigation paths for users accessing different sensor types.

## Integration

The extension integrates with `isaacsim.sensors.experimental.rtx` to access the underlying RTX sensor creation functionality and uses `**omni.kit.actions.core**` for action registration mechanisms. Context menu integration is provided through `**omni.kit.context_menu**`, enabling right-click sensor creation workflows in the viewport.
