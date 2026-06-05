# Overview

The ROS 2 Bridge extension enables communication between Isaac Sim and ROS 2 systems. It provides a complete integration layer that allows Isaac Sim to publish and subscribe to ROS 2 topics and services through OmniGraph nodes and Action graphs. This bridge makes it possible to connect Isaac Sim simulations with the ROS 2 ecosystem, enabling robotic applications to interact with simulated environments in real-time.

**Important**: ROS 2 publishers, subscribers, and services are only active during simulation playback (when play is pressed). Before launching Isaac Sim, ensure ROS 2 libraries are sourced in the terminal, or use the lightweight ROS 2 libraries included with Isaac Sim.

## Functionality

The extension enables bidirectional data exchange between Isaac Sim and ROS 2 systems by providing the infrastructure for ROS 2 message handling within the simulation environment. Communication flows through OmniGraph nodes, which can be assembled into Action graphs to define complex behaviors and data pipelines. This allows robotic simulations to send sensor data, receive control commands, and interact with ROS 2-based perception and planning systems.

### OmniGraph Integration

The bridge operates through OmniGraph nodes that handle ROS 2 communication. Each node can act as either a publisher or subscriber, converting data between Isaac Sim's internal representation and ROS 2 message formats. These nodes can be connected in Action graphs to create complete robotic workflows, from sensor simulation to actuator control.

### Activation Behavior

ROS 2 communication nodes remain dormant until simulation playback begins. When play is pressed, all configured publishers and subscribers activate and begin their communication loops. This ensures that ROS 2 traffic only occurs during active simulation, preventing spurious messages during scene setup or editing. When simulation is paused or stopped, ROS 2 communication ceases until playback resumes.

## Integration

The extension consolidates multiple ROS 2-related extensions into a unified bridge:

- **isaacsim.ros2.core** provides the fundamental ROS 2 communication layer and message handling infrastructure, serving as the foundation for all ROS 2 operations
- **isaacsim.ros2.nodes** supplies the OmniGraph node implementations that perform actual ROS 2 publishing, subscribing, and service calls
- **isaacsim.ros2.examples** includes example scenes and Action graphs demonstrating common ROS 2 integration patterns
- **isaacsim.ros2.ui** offers UI components for configuring and monitoring ROS 2 connections within the Isaac Sim interface

All ROS 2 Bridge settings are centrally defined in the isaacsim.ros2.core extension, providing a single configuration point for the entire bridge system.

## Considerations

### ROS 2 Environment Setup

The bridge requires a properly configured ROS 2 environment before Isaac Sim launches. Users must either source their system's ROS 2 installation in the terminal prior to starting Isaac Sim, or utilize the lightweight ROS 2 libraries bundled with Isaac Sim. Without this setup, the bridge will not be able to establish communication with external ROS 2 nodes.
