# Overview

The isaacsim.robot_motion.cumotion.examples extension provides interactive UI demonstrations for cuMotion robot motion planning capabilities within Isaac Sim. This extension showcases various motion planning algorithms and techniques through five distinct example modules, each demonstrating different aspects of the cuMotion library for robotic motion generation and control.

## Functionality

The extension offers five specialized motion planning examples:

**RMPflow Motion Planning** - Demonstrates Riemannian Motion Policy flow for reactive motion planning with collision avoidance using the Franka robot arm.

**Trajectory Generation** - Shows trajectory generation capabilities for the UR10 robot with configurable motion parameters and path planning.

**Graph-based Planning** - Provides configuration-space and task-space motion planning using graph-based algorithms, featuring interactive joint sliders for target specification.

**Trajectory Optimization** - Implements trajectory optimization techniques with real-time visualization and both C-space and task-space target planning.

**World Interface** - Offers comprehensive world management controls with different synchronization modes for motion updates including synchronize, synchronize_transforms, and synchronize_properties.

Each example includes world controls for loading scenes and resetting scenarios, along with run controls for executing motion planning demonstrations. The interfaces support both manual configuration through joint sliders and automated planning to target positions or objects.

## Key Components

### {class}`UIBuilder <isaacsim.robot_motion.cumotion.examples.rmp_flow.UIBuilder>` Classes

Each module provides a {class}`UIBuilder <isaacsim.robot_motion.cumotion.examples.rmp_flow.UIBuilder>` class that constructs the specific user interface for its motion planning example. These builders handle:

- Scene loading and initialization with robot assets
- Joint configuration controls through interactive sliders
- Motion planning execution with start/stop functionality
- Real-time physics integration and timeline event management
- Stage event handling for proper resource cleanup

### Extension Framework

All five modules share a common extension structure that:

- Registers menu items under the "cuMotion Examples" menu hierarchy
- Creates scrollable window interfaces docked to the left viewport
- Manages timeline events (play/pause/stop) for synchronized motion execution
- Subscribes to physics step events for real-time motion updates
- Handles stage opening/closing events for proper initialization

## Integration

The extension integrates with several Isaac Sim systems through its dependencies. It uses **omni.timeline** for synchronized motion execution with the simulation timeline, and **omni.ui** for creating interactive user interfaces. The extension connects with isaacsim.robot_motion.cumotion for core motion planning functionality and isaacsim.robot_motion.experimental.motion_generation for advanced motion algorithms. Integration with isaacsim.core.simulation_manager enables physics-based motion simulation and control.
