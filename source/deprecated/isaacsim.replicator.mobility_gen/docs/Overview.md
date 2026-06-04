# Overview

```{deprecated} 6.0.0
Extension deprecated since Isaac Sim 6.0.0 in favor of the experimental extension: isaacsim.replicator.experimental.mobility_gen
```

The isaacsim.replicator.mobility_gen extension was a specialized extension for generating mobility training data for robotics applications. It provided a complete framework for simulating robot navigation scenarios, capturing multi-modal sensor data, and recording comprehensive datasets that include RGB images, depth maps, segmentation masks, occupancy maps, and robot state information.

## Functionality

The extension creates structured mobility scenarios where robots navigate through environments defined by occupancy maps. During simulation, it captures synchronized data streams from multiple sensor modalities including cameras, robot pose information, and user input controls. This data can be saved in organized formats suitable for training navigation models or replaying scenarios for analysis.

**Key capabilities include:**

- Multi-modal sensor data capture (RGB, depth, segmentation, normals)
- Robot state tracking (position, orientation, joint states, velocities)
- Human-in-the-loop control through keyboard and gamepad input
- Occupancy map-based navigation scenarios
- Structured data recording and replay functionality
- ROS-compatible occupancy map export

## Key Components

### Robot System

The robot system provides abstract base classes for creating mobility-capable robots with front-facing cameras and chase cameras. Robots track their pose, joint states, and velocities while supporting various control modes including keyboard, gamepad, and autonomous path following. The `MobilityGenRobot` class defines standardized parameters for robot physics, camera positioning, and control gains.

### Input Handling

Two input driver systems capture human control data: `KeyboardDriver` monitors WASD keys for navigation commands, while `GamepadDriver` handles analog stick inputs with configurable deadzone filtering. Both systems implement singleton patterns and provide real-time button and axis state information through numpy arrays.

### Camera System

The `MobilityGenCamera` module wraps USD camera prims and integrates with **omni.replicator.core** to capture various rendering outputs. It supports independent enabling of RGB, semantic segmentation, instance segmentation, depth, and surface normals rendering, with each output type stored in separate buffers for efficient data management.

### Scenario Management

The scenario system organizes robot navigation tasks within occupancy map environments. The `MobilityGenScenario` base class manages robot instances, occupancy maps, and provides buffered occupancy maps for collision detection. Scenarios implement reset and step methods to define specific navigation behaviors and objectives.

### Occupancy Maps

The `OccupancyMap` class represents navigation environments as grid-based maps with three cell states: unknown, freespace, and occupied. It provides coordinate conversion between pixel and world coordinates, supports ROS-format import/export, and offers buffering operations for robot collision avoidance. Path generation utilities create navigation waypoints using pathfinding algorithms.

### C++ Path Planner

The extension includes a C++ path planning backend exposed to Python through pybind11 bindings (`_path_planner` module). It implements a priority-queue-based graph search over the freespace grid, computing distance-to-start maps and parent maps for efficient path unrolling. The two entry points are `generate_paths`, which runs a Dijkstra-like expansion from a start point across the occupancy map, and `unroll_path`, which traces back through the parent maps to reconstruct a waypoint sequence from any reachable endpoint.

### Data Recording

The recording system organizes captured data into hierarchical directory structures. `MobilityGenWriter` handles timestamped output of sensor data, robot states, and configuration files, while `MobilityGenReader` provides access to recorded datasets with support for individual data type retrieval or complete state dictionary reconstruction.

## Integration

The extension integrates with isaacsim.core.api for world management and robot articulation, **omni.replicator.core** for multi-modal rendering, and isaacsim.core.prims for articulation control. The modular design allows easy extension with custom robot types, input devices, and scenario implementations through registry-based component discovery.
