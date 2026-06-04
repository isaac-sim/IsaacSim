---
name: isaac-sim-ros2-bridge
description: >
  Setting up and using the ROS 2 Bridge in Isaac Sim 6.0 (Kit 110) for publishing
  and subscribing to ROS 2 topics from simulation. Covers bridge architecture, OmniGraph
  node setup for sensors/commands, Nav2 integration, and multi-robot namespacing.
  Use when: connecting Isaac Sim to ROS 2, publishing sensor data (camera, LiDAR, odometry)
  from simulation to ROS 2, subscribing to robot commands from Nav2 or other ROS 2 nodes,
  or setting up multi-robot namespaced ROS 2 communication in simulation.
  Triggers on: ROS 2 bridge, OmniGraph ROS2, Nav2 Isaac Sim, isaacsim.ros2.bridge,
  ROS2PublishLaserScan, cmd_vel subscribe, multi-robot namespace ROS2.
---

# Isaac Sim ROS 2 Bridge

## Architecture

The ROS 2 Bridge in Isaac Sim 6.0 is extension-based (`isaacsim.ros2.bridge`) and built on OmniGraph action graphs. Publishers, subscribers, and services are **only active when play is pressed**.

### Extension Stack

| Extension | Purpose |
|-----------|---------|
| `isaacsim.ros2.core` | Backend libraries, settings, lightweight ROS 2 libs |
| `isaacsim.ros2.nodes` | OmniGraph nodes for topics/services |
| `isaacsim.ros2.bridge` | Top-level bridge extension (enables all) |
| `isaacsim.ros2.tf_viewer` | TF tree visualization |
| `isaacsim.ros2.urdf` | URDF import with ROS 2 conventions |
| `isaacsim.ros2.examples` | Sample scenes (Nova Carter, iw_hub, hospital, office) |
| `isaacsim.ros2.sim_control` | Simulation control via ROS 2 services |

### Prerequisites

No manual setup is required for the default workflow. When `isaacsim.ros2.core` starts, it reads `ROS_DISTRO`; if unset, it falls back to the lightweight ROS 2 libs bundled inside the extension and sets `internal_lib_fallback=True`. The `isaac-sim.sh` / `python.sh` launchers auto-source `setup_ros_env.sh`, which exports `ROS_DISTRO`, `RMW_IMPLEMENTATION`, and `LD_LIBRARY_PATH` for the bundled libs before the process starts.

Source a system ROS 2 install (`/opt/ros/<distro>/setup.bash`) in the parent shell only when you need message types or RMW implementations not provided by the bundled libs. On Linux, `LD_LIBRARY_PATH` must be set in the parent shell (the dynamic linker reads it at process start), which is why this step is shell-sourced and not handled by the extension at runtime.

Pass `--no-ros-env` to bypass the auto-sourcing; in that case the parent shell must provide a working ROS 2 environment.

<!-- CODE: scripts/prerequisites.sh -->
*[Code: `scripts/prerequisites.sh`]*

## OmniGraph ROS 2 Nodes

All ROS 2 communication is configured through OmniGraph action graphs. Key node types:

### Publishers

| Node Type | Topic Type | Use Case |
|-----------|-----------|----------|
| `ROS2PublishClock` | `rosgraph_msgs/Clock` | Sim time sync |
| `ROS2PublishJointState` | `sensor_msgs/JointState` | Joint positions/velocities |
| `ROS2PublishOdometry` | `nav_msgs/Odometry` | Robot odometry |
| `ROS2PublishLaserScan` | `sensor_msgs/LaserScan` | Lidar data |
| `ROS2PublishImage` | `sensor_msgs/Image` | Camera RGB/depth |
| `ROS2PublishCameraInfo` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `ROS2PublishTransformTree` | `tf2_msgs/TFMessage` | TF frames |
| `ROS2PublishSemanticLabels` | `std_msgs/String` | Semantic segmentation labels |
| `ROS2PublishPointCloud` | `sensor_msgs/PointCloud2` | Lidar/depth point clouds |
| `ROS2PublishImu` | `sensor_msgs/Imu` | IMU data |

### Subscribers

| Node Type | Topic Type | Use Case |
|-----------|-----------|----------|
| `ROS2SubscribeJointState` | `sensor_msgs/JointState` | Joint commands |
| `ROS2SubscribeTwist` | `geometry_msgs/Twist` | Velocity commands (cmd_vel) |
| `ROS2SubscribeAckermannDrive` | `ackermann_msgs/AckermannDriveStamped` | Ackermann steering commands |

### Generic / extra

| Node Type | Use case |
|---|---|
| `ROS2Publisher` / `ROS2Subscriber` | generic publish/subscribe for arbitrary message types |
| `ROS2PublishBoundingBox2D` / `ROS2PublishBoundingBox3D` | annotator output to ROS 2 |
| `ROS2PublishRgbd` | combined RGB+D publishing |
| `ROS2PublishSemanticSegmentation` / `ROS2PublishInstanceSegmentation` | segmentation outputs |

## Setting Up a ROS 2 Bridge Graph (Python)

The repo ships a canonical reference for building a ROS 2 publishing action graph with `og.Controller.edit(...)`, including connecting `OnPlaybackTick` to publisher nodes, wiring `IsaacReadSimulationTime` into a `ROS2PublishClock`, and creating a matching `rclpy` subscriber inside the same script.

- Reference example: `source/standalone_examples/api/isaacsim.ros2.bridge/clock.py`

When extending this pattern, use the current `isaacsim.ros2.bridge.ROS2Publish*` / `ROS2Subscribe*` node names (the legacy `omni.isaac.ros2_bridge.*` namespace is deprecated and will not load on Kit 110).

> **Migration:** see [Migrating ROS 2 OmniGraph nodes](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ros2_omnigraph_migration.html) for the node-by-node rename map, and [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html) for the broader `omni.isaac.*` → `isaacsim.*` mapping.

## Multi-Robot Namespacing

For multi-robot scenes, each robot needs its own namespace for ROS 2 topics. The repo's multi-robot scenario (Carter on hospital/office) demonstrates the manual loading + per-robot setup pattern; `scripts/multi_robot_namespacing.py` in this skill provides the per-robot graph factory with namespaced odom / TF / cmd_vel.

- Scenario reference: `source/standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py`
- Per-robot graph factory: `scripts/multi_robot_namespacing.py`

## Nav2 Integration

To use Nav2 with Isaac Sim ROS 2 Bridge:

1. Publish required topics: `/odom`, `/tf`, `/scan` (or `/pointcloud`), `/joint_states`
2. Subscribe to: `/cmd_vel`
3. Publish sim clock on `/clock` and set `use_sim_time: true` in Nav2 params

## Known Issues (Isaac Sim 6.0 rc.22)

- ROS 2 bridge only activates on `timeline.play()` — topics are not published in stopped state
- Camera publishers require camera prims to have render products attached
- TF tree can become inconsistent if robot prim paths change at runtime
- Bundled ROS 2 libs may lack message types needed for custom topics — source a system ROS 2 install in the parent shell before launching Isaac Sim

## Sample Scenes

Pre-built example USD files (from `isaacsim.ros2.examples`):

| Scene | Path | Description |
|-------|------|-------------|
| Nova Carter Navigation | `/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd` | Single robot Nav2 |
| Nova Carter Joint States | `/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation_joint_states.usd` | With joint state publishing |
| iw_hub Navigation | `/Isaac/Samples/ROS2/Scenario/iw_hub_warehouse_navigation.usd` | iw_hub AMR |
| Multi-Robot Navigation | ROS2/Navigation/Multiple Robots category | Fleet Nav2 |
| Hospital Scene | `/Isaac/Samples/ROS2/Scenario/hospital_scene.usd` | Perceptor + hospital |
| Office Scene | `/Isaac/Samples/ROS2/Scenario/office_scene.usd` | Office navigation |
