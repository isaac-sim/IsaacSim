# Overview

The Isaac Sim UI Example extension demonstrates Isaac Sim's robotics-centric UI components and patterns for extension development. This extension provides comprehensive examples of UI elements commonly used in robotics applications, serving as a reference implementation for developers creating their own Isaac Sim extensions with graphical user interfaces.

```{image} ../../../../source/extensions/isaacsim.examples.ui/data/preview.png
---
align: center
---
```


## UI Components

### Interactive GUI Grid
The extension provides an interactive grid layout containing various input controls essential for robotics applications:
- Buttons for triggering actions and state changes
- Checkboxes for boolean configuration options
- Dropdown menus for selecting from predefined options
- Sliders for continuous value adjustment
- Toggle controls for application stepping modes

### Plotting Capabilities
Real-time plotting functionality includes both static and time series visualization:
- Time series plots for monitoring sensor data and system performance over time
- Multi-axis plotting support for displaying related data streams
- Dynamic data updates suitable for real-time robotics monitoring

### Search Interface
A searchable list widget provides filtering capabilities:
- Interactive search functionality with real-time filtering
- List-based data presentation with selectable items
- Pattern matching for quickly finding specific entries

### File System Navigation
Directory selection capabilities through a folder picker widget:
- Browse and select folders from the file system
- Integration with Isaac Sim's file handling patterns
- Support for robotics project organization workflows

### Communication Controls
Network communication interface elements include:
- Hostname and port configuration fields
- Connection status indicators showing current state
- Connect/disconnect controls for external system integration
- Progress indicators for communication operations

### Progress Indicators
Visual feedback components for long-running operations:
- Progress bars showing completion percentage
- Trigger buttons for initiating background processes
- Status indicators for operation monitoring

### Directional Control Pads
Custom D-pad controllers designed for robot control applications:
- Multi-directional input capture
- Suitable for teleoperation and manual robot guidance
- Integrated with Isaac Sim's control input systems

## Functionality

The extension creates a dockable window accessible through "Window > Examples > GUI Templates" menu, demonstrating best practices for UI layout, styling, and event handling. Each component group showcases specific UI patterns commonly needed in robotics applications, from basic input controls to specialized visualization tools.

The modular design allows developers to reference specific UI implementations and adapt them for their own extensions. The examples cover both simple interactive elements and more complex components like real-time plotting and communication interfaces.

## Integration

The extension integrates with Isaac Sim's GUI component system through the `isaacsim.gui.components` dependency, which provides the underlying UI utilities and controls. This integration ensures consistency with Isaac Sim's visual design language and interaction patterns while demonstrating proper extension architecture for robotics applications.
