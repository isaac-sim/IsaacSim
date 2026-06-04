# Overview

The `isaacsim.ros2.nodes` extension provides OmniGraph nodes that enable ROS 2 communication within Isaac Sim. This extension bridges robotics simulation with ROS 2 systems by offering nodes for publishing, subscribing, and handling services that can be connected in Action Graphs. It relies on `isaacsim.ros2.core` for the underlying ROS 2 functionality and integrates sensor data from camera and physics sensors into the ROS 2 ecosystem.

## Functionality

The extension registers OmniGraph nodes that handle ROS 2 message passing and service calls. These nodes can be placed in Action Graphs to stream sensor data, receive commands, and interact with ROS 2 topics and services. The nodes inherit ROS distribution and core settings from `isaacsim.ros2.core`, which centralizes all ROS 2 Bridge configuration under `/exts/isaacsim.ros2.bridge/*`. This allows the nodes to automatically align with the configured ROS 2 environment at runtime without requiring separate configuration.

## Integration

The extension depends on `isaacsim.ros2.core` for the ROS 2 backend, `isaacsim.sensors.camera` and `isaacsim.sensors.experimental.physics` for sensor data access, `isaacsim.sensors.physics.nodes` for physics sensor OmniGraph nodes, and `omni.graph` for the OmniGraph framework. It uses `isaacsim.core.experimental.utils` for stage and prim utilities, `isaacsim.core.experimental.objects` and `isaacsim.core.experimental.prims` for scene object creation in tests, and `isaacsim.core.rendering_manager` and `isaacsim.core.simulation_manager` for viewport and physics management. C++ publisher implementations for images and point clouds provide optimized data transfer for high-bandwidth sensor streams.
