# Overview

The Camera Inspector extension provides a comprehensive tool for examining and managing cameras within Isaac Sim scenes. This extension creates a dedicated window interface that automatically discovers all cameras in the scene and presents detailed information about their properties, positioning, and orientation in real-time.

```{image} ../../../../source/extensions/isaacsim.util.camera_inspector/data/preview.png
---
align: center
---
```


## UI Components

### Camera Inspector Window

The main interface is accessible through the Tools > Sensors menu and provides a dockable window that integrates seamlessly with the Omniverse interface. The window automatically refreshes to display current camera information as the scene changes.

### Camera Discovery and Selection

The extension automatically scans the scene to identify all available cameras and presents them in an organized list. Users can select individual cameras to view detailed information and perform management operations.

### Camera Pose Display

**Real-time pose information is displayed for selected cameras in multiple coordinate systems:**
- World coordinates showing absolute position and orientation
- Local coordinates relative to parent objects
- Support for different axis conventions including world, USD, and ROS coordinate systems

The pose data is presented in both numeric format for precise measurements and as copyable text output suitable for use in scripts or external applications.

### Viewport Management

The interface includes controls for creating new viewports associated with selected cameras, allowing users to visualize scenes from specific camera perspectives. Users can also assign cameras to existing viewports, providing flexible viewport configuration options.

## Functionality

### Live Camera Statistics

Camera properties and statistics update continuously as the scene evolves, ensuring users always have current information about camera states, positions, and orientations.

### Multi-Axis Coordinate Support

The extension accommodates different coordinate system conventions, making it valuable for users working across different robotics and simulation frameworks that may use varying axis orientations.

### Scene Integration

The tool maintains awareness of scene hierarchy and relationships, displaying camera information within the context of the broader scene structure and parent-child relationships.

## Relationships

This extension integrates with Isaac Sim's camera system through isaacsim.sensors.camera for accessing camera properties and managing camera-related operations. It utilizes isaacsim.gui.components for consistent UI presentation within the Isaac Sim interface.
