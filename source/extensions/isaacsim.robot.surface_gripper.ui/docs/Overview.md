# Overview

The isaacsim.robot.surface_gripper.ui extension provides user interface components for creating and configuring surface grippers in Isaac Sim. Surface grippers simulate physics-based suction or surface-type grippers that can attach to and manipulate objects in robotic simulations. This extension integrates surface gripper functionality directly into the Isaac Sim interface through menu items and specialized property widgets.

```{image} ../../../../source/extensions/isaacsim.robot.surface_gripper.ui/data/preview.png
---
align: center
---
```


## UI Components

### Menu Integration

The extension adds surface gripper creation capabilities to Isaac Sim's menu system. Users can access surface gripper creation through menu items that execute the CreateSurfaceGripper command to add new surface grippers to the scene. This provides a convenient workflow for setting up gripper components without requiring scripting.

### Surface Gripper Properties Widget

The SurfaceGripperPropertiesWidget provides a specialized property interface that appears in the Property panel when Surface Gripper prims are selected. This widget offers comprehensive control over gripper configuration and operation.

#### Key Features

**Interactive Controls**: The widget includes dedicated open and close gripper buttons that allow users to directly control gripper state from the property panel. These controls operate on all currently selected Surface Gripper prims simultaneously.

**Specialized Property Display**: Properties are filtered and organized to show only Surface Gripper-relevant attributes and relationships. The widget displays gripper-specific properties including status information, grip limits, force constraints, and attachment points in a logical order with custom display names and tooltips.

**Real-time Updates**: The widget automatically monitors USD changes and refreshes the property display when gripper states or configurations change, ensuring users always see current gripper information.

**Dual Property Support**: The interface handles both attributes (such as status, limits, and offsets) and relationships (including attachment points and gripped objects), providing complete gripper configuration access through the UI.

## Functionality

The extension creates a seamless integration between surface gripper physics simulation and Isaac Sim's user interface. Users can create surface grippers through menu actions and then configure their behavior through the specialized property widget. The real-time property updates ensure that changes in simulation state are immediately reflected in the UI, while the interactive controls allow direct gripper manipulation during scene setup and testing.

## Relationships

This extension builds upon isaacsim.robot.surface_gripper for the core surface gripper functionality and physics simulation. It uses isaacsim.gui.components for UI element construction and isaacsim.gui.menu for menu integration, creating the complete user interface layer for surface gripper workflow in Isaac Sim.
