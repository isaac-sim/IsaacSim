# Overview

isaacsim.ros2.examples provides pre-built demonstration scenes showcasing ROS 2 integration capabilities within Isaac Sim. The extension includes three main example categories: MoveIt manipulation workflows, navigation scenarios with the Nova Carter robot, and waypoint-following demonstrations. Each example comes with its own UI panel for configuration and scene management.

## UI Components

### ROS Samples Window

The extension registers multiple example demonstrations that can be accessed through the Isaac Sim example browser. These examples are organized into three categories:

**ROS2 Navigation Examples** - Demonstrates autonomous navigation using the Nova Carter robot in various environments including a sample scene, hospital, and office settings. These examples showcase ROS 2 navigation stack integration with Isaac Sim.

**ROS2 Isaac ROS Examples** - Features perception-focused demonstrations using Isaac ROS components with the Nova Carter platform and IW Hub configurations.

**ROS2 Multiple Robots Examples** - Illustrates multi-robot coordination scenarios with joint state publishing and control.

### MoveIt Sample Window

Provides a demonstration of MoveIt integration with the Franka robotic arm. The UI enables users to:

- Load and configure a Franka robot in the scene
- Create ROS action graphs for motion planning
- Set up the necessary ROS 2 communication infrastructure for MoveIt control

### Waypoint Follower Sample Window

Features a UI for configuring waypoint-based navigation workflows. The interface includes:

- Radio button controls for selecting between different operation modes (waypoint following, patrolling, or gathering waypoints)
- Input fields for specifying the OmniGraph path and frame ID parameters
- A waypoint count display showing the number of configured waypoints
- Controls for managing waypoint data and navigation behavior

## Key Components

### Example Registration System

The extension uses a registration mechanism to make examples discoverable through the Isaac Sim example browser. Each example category (Navigation, Isaac ROS, Multiple Robots) is registered with associated scene files that users can load and explore.

### Action Graph Creation

For the MoveIt example, the extension programmatically creates ROS action graphs using `**omni.graph**` functionality. These graphs establish the communication bridge between Isaac Sim and the ROS 2 MoveIt framework, enabling motion planning and execution for the Franka robot.

### Scene Templates

The extension provides access to multiple pre-configured scene templates:
- Nova Carter in various environments (sample, hospital, office)
- Perceptor-based perception demonstrations
- IW Hub configurations for Isaac ROS workflows
- Franka arm setup for manipulation examples

## Integration

The extension integrates with isaacsim.examples.browser to register its demonstrations, making them discoverable and launchable from the standard Isaac Sim examples interface. It uses isaacsim.ros2.nodes to provide ROS 2-specific OmniGraph nodes that enable communication between Isaac Sim and external ROS 2 systems. Scene setup uses **isaacsim.core.experimental.utils** for stage operations, **isaacsim.core.rendering_manager** for viewport camera control, and **isaacsim.core.simulation_manager** for physics scene configuration. The extension also leverages **omni.graph.tools** for programmatic action graph construction and management.
