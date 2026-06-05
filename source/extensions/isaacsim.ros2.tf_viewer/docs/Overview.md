# Overview

The TF Viewer extension provides real-time visualization of ROS 2 transform (TF) data directly within the Isaac Sim viewport. It displays the coordinate frame hierarchy broadcast through the ROS 2 tf2 system, showing relationships between frames, their spatial positions, and orientations as they update dynamically.

## Functionality

The extension connects to a ROS 2 system to monitor transform broadcasts and renders them as visual elements in the viewport. Users can observe how coordinate frames relate to each other through parent-child relationships, track frame positions and orientations in 3D space, and understand the complete transform tree structure emanating from a chosen root frame.

### Transform Listening

The extension uses a dedicated transform listener that operates independently from the main application thread. For ROS 2, it initializes a node with a transform buffer and listener that subscribes to `/tf` and `/tf_static` topics. The listener continuously processes incoming transform messages and maintains an up-to-date view of all available frames and their relationships.

Transform queries retrieve both individual frame-to-frame transformations and complete transform trees relative to a specified root frame. The system returns frame names, their transforms as translation and rotation tuples, and the parent-child relationships that define the frame hierarchy.

### Visual Representation

The viewport manipulator renders multiple visual components to represent the transform data:

- **Frame markers**: Visual indicators at each frame's origin
- **Frame names**: Text labels identifying each coordinate frame
- **Coordinate axes**: RGB-colored axes showing frame orientation (X-red, Y-green, Z-blue)
- **Relationship arrows**: Lines connecting parent and child frames

Each visual element can be independently configured for visibility, color, size, and other display properties. The visualization updates automatically when new transform data arrives or when configuration settings change.

### Configuration Options

Users can customize the visualization through several parameters:

- **Root frame selection**: Choose which frame serves as the reference for the transform tree
- **Component visibility**: Toggle display of frames, names, axes, and arrows independently
- **Visual styling**: Adjust colors (RGBA), sizes, lengths, and thickness values for each component
- **Implementation mode**: Switch between C++ and Python transform listener implementations via settings

The `exts."isaacsim.ros2.tf_viewer".cpp` setting controls whether to use the C++ implementation for transform listening. When set to `true`, it provides better performance for processing high-frequency transform updates. Setting `exts."isaacsim.ros2.tf_viewer".include_root_frame` to `true` ensures the root frame appears in the visualization even when it's not explicitly published in the transform tree.

## UI Components

### TF Viewer Window

The window presents controls organized into collapsible sections for managing different aspects of the visualization. An update frequency indicator shows the rate at which transform data refreshes. Controls for the root frame allow users to specify which coordinate frame serves as the reference point for displaying the entire transform tree.

Separate sections provide toggles and sliders for each visual component type. Frame marker controls adjust visibility, color channels, and size. Name label controls manage text display properties. Axis controls configure the length and thickness of coordinate axes. Arrow controls modify the appearance of lines connecting related frames.

A reset button clears the transform buffer to remove stale data and resolve timing-related warnings. The window integrates with the application's menu system, appearing under a configurable menu path for easy access.

## Integration

The extension depends on `isaacsim.ros2.bridge` to establish ROS 2 connectivity and handle the underlying communication with the ROS 2 system. It uses `**omni.ui.scene**` to create the viewport manipulator that renders transform visualizations as scene overlays. The `**omni.kit.viewport.utility**` dependency provides viewport context and window management capabilities needed to properly attach the visualization to the active viewport.
