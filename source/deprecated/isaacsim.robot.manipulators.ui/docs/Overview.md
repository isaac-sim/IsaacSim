# Overview

```{deprecated} 6.0.0
This extension is deprecated. Refer to `isaacsim.robot.experimental.manipulators.examples` for recommended alternatives.
```

The isaacsim.robot.manipulators.ui extension provides user interface components for creating OmniGraph-based robot controllers in Isaac Sim. This extension adds menu-driven tools for setting up joint position control, joint velocity control, and gripper control systems by automatically generating the necessary OmniGraph networks and node configurations.

## UI Components

### Menu System

The extension integrates with Isaac Sim's menu system through the Tools/Robotics/OmniGraph Controllers path, providing three specialized controller creation tools:

- **Articulation Position Controller**: Creates position-based joint control systems
- **Articulation Velocity Controller**: Creates velocity-based joint control systems  
- **Gripper Controller**: Creates open-loop gripper control systems

### Articulation Position Controller Window

**Key capabilities**: Automatically generates OmniGraph networks for position-based control of robotic joints. The window discovers joints from USD physics prims and handles proper unit conversion between USD degrees and PhysX radians for revolute joints.

**User workflow**: Users specify a robot prim, configure graph settings, and choose to either create a new graph or add to an existing one. The generated graph includes joint command arrays, joint name arrays, and an IsaacArticulationController node, enabling real-time joint position control through the Property Manager interface during simulation.

### Articulation Velocity Controller Window

**Key capabilities**: Provides interface for generating OmniGraph nodes that control joint velocities of robotic articulations. The system automatically detects joints in the specified robot prim and creates necessary graph nodes with proper connections.

**User workflow**: Users can specify graph creation options, select robot prims containing articulations, and set graph paths. The controller handles unit conversion using radians for revolute joints while USD properties display in degrees, with default velocity values extracted from joint drive APIs.

### Gripper Controller Window

**Key capabilities**: Enables setup of gripper control systems through OmniGraph node creation and connection management. The interface supports configuration of gripper parameters including joint names, position limits, speed settings, and optional keyboard controls.

**User workflow**: Users configure parent robot prims, gripper root prims, and graph paths while setting gripper-specific parameters like open/close positions and movement speeds. Optional keyboard control (O-open, C-close, N-stop) can be enabled, with support for specifying which joints should be controlled when not all articulated joints are part of the gripper mechanism.

## Integration

The extension uses **omni.kit.menu.utils** to integrate controller creation tools into Isaac Sim's menu system. It depends on isaacsim.robot.manipulators for the underlying controller implementations and isaacsim.gui.components for UI framework support. The generated OmniGraph networks integrate with Isaac Sim's simulation timeline and Property Manager interface for runtime control.
