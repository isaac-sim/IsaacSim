# Overview

The isaacsim.ros2.ui extension provides pre-configured OmniGraph templates for publishing ROS 2 messages from Isaac Sim. These templates streamline the process of setting up common ROS 2 publishing workflows by providing ready-to-use graph configurations for sensors, robot data, and simulation state.

## Functionality

The extension offers seven graph template classes that create OmniGraph networks for different ROS 2 publishing scenarios:

**Sensor Publishers**
- {class}`Ros2CameraGraph <isaacsim.ros2.ui.Ros2CameraGraph>`: Creates a graph for publishing camera image data to ROS 2 topics
- {class}`Ros2RtxLidarGraph <isaacsim.ros2.ui.Ros2RtxLidarGraph>`: Creates a graph for publishing RTX lidar point cloud data to ROS 2 topics

**Robot State Publishers**
- {class}`Ros2JointStatesGraph <isaacsim.ros2.ui.Ros2JointStatesGraph>`: Creates a graph for publishing robot joint states
- {class}`Ros2OdometryGraph <isaacsim.ros2.ui.Ros2OdometryGraph>`: Creates a graph for publishing odometry data including position, orientation, and velocities
- {class}`Ros2TfPubGraph <isaacsim.ros2.ui.Ros2TfPubGraph>`: Creates a graph for publishing transform tree information

**Simulation Publishers**
- {class}`Ros2ClockGraph <isaacsim.ros2.ui.Ros2ClockGraph>`: Creates a graph for publishing simulation clock time to synchronize ROS 2 nodes with the simulator
- {class}`Ros2GenericPubGraph <isaacsim.ros2.ui.Ros2GenericPubGraph>`: Creates a graph for publishing custom message types to ROS 2 topics

Each graph class encapsulates the OmniGraph node configuration and connections required for its specific publishing task, eliminating the need to manually construct these graphs through the OmniGraph editor.

## Integration

The extension adds a shortcuts menu to Isaac Sim that provides quick access to ROS 2 graph creation functionality. This menu integration uses **omni.kit.menu.utils** to register menu items that instantiate the graph templates, allowing users to add ROS 2 publishing capabilities to their scenes without writing code.

The graph templates depend on isaacsim.ros2.nodes, which provides the underlying OmniGraph nodes used within each template. The templates act as a convenience layer that assembles these nodes into functional publishing pipelines. The extension uses **isaacsim.core.experimental.utils** for stage path generation and **isaacsim.core.rendering_manager** for viewport camera control.
