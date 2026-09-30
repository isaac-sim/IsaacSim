---
name: isaac-sim-ros2-bridge
description: "ROS 2 OmniGraph bridge, Nav2, and multi-robot namespacing in Isaac Sim 6. Use for ROS 2 integration and fleet topics."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# Isaac Sim ROS 2 Bridge

## Purpose

Publish and subscribe to ROS 2 topics from Isaac Sim using OmniGraph bridge nodes, including Nav2 integration and multi-robot namespacing.

## Limitations

- Targets Isaac Sim 6 / Kit 110 unless a section states otherwise.
- Does not replace official NVIDIA documentation for unsupported edge cases.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| No ROS topics | Bridge graph not playing or wrong domain | Press Play; set `ROS_DOMAIN_ID` consistently |
| Missing humble libs | `LD_LIBRARY_PATH` not set before process start | Fix the parent shell (source ROS env or use default launcher), then relaunch — do not hot-patch a running process |
| Namespaced topic mismatch | Frame or namespace drift | Align OmniGraph namespace with Nav2 config |

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/multi_robot_namespacing.py` | Fleet demo: play-gated per-robot bridge graphs (odom, TF, scan, joint states, cmd_vel) + shared clock | `--robots NAME:PRIM_PATH [...]` `--usd_path` `--lidar_suffix` `--headless` `--test` |
| `scripts/prerequisites.sh` | Prerequisites | positional args per script header (see script) |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/multi_robot_namespacing.py", args=["--help"])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`, or execute shell helpers directly when they do not require the simulator.

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

No manual setup is required for the default workflow. When `isaacsim.ros2.core` starts, it reads `ROS_DISTRO`; if unset, it falls back to the lightweight ROS 2 libs bundled inside the extension and sets `internal_lib_fallback=True`. The `isaac-sim.sh` / `python.sh` launchers auto-source `setup_ros_env.sh`, which exports `ROS_DISTRO`, `RMW_IMPLEMENTATION`, and the bundled ROS library path before the process starts.

Source a system ROS 2 install (`/opt/ros/<distro>/setup.bash`) in the parent shell only when you need message types or RMW implementations not provided by the bundled libs. On Linux, any `LD_LIBRARY_PATH` changes must exist before launching Isaac Sim because the dynamic linker reads them at process start; changing `LD_LIBRARY_PATH` after the process is already running does not retroactively fix ROS library resolution.

Pass `--no-ros-env` to bypass the auto-sourcing; in that case the parent shell must provide a working ROS 2 environment, including any required `LD_LIBRARY_PATH` entries.

### ROS Environment Decision Rule

Use this precedence order when a user reports ROS 2 bridge startup or symbol-resolution issues:

1. **Default path:** launch via `isaac-sim.sh` / `python.sh` and let `setup_ros_env.sh` provide the bundled ROS environment.
2. **System ROS path:** if custom message packages or alternate RMW implementations are required, source `/opt/ros/<distro>/setup.bash` (and any overlay workspace) in the parent shell *before* launch.
3. **Do not hot-patch `LD_LIBRARY_PATH`:** if Isaac Sim is already running with the wrong loader environment, restart from a correctly sourced shell instead of trying to repair the process in-place.

Treat "export `LD_LIBRARY_PATH` and retry inside the current Isaac Sim process" as an anti-pattern; the corrective action is always "fix the parent shell, then relaunch."

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

For multi-robot scenes, each robot needs its own namespace for ROS 2 topics. The repo's multi-robot scenario (Carter on hospital/office) demonstrates the manual loading + per-robot setup pattern; `scripts/multi_robot_namespacing.py` in this skill provides the full fleet bridge factory.

- Scenario reference: `source/standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py`
- Fleet graph factory: `scripts/multi_robot_namespacing.py`

### Play-Gated Activation

All bridge graphs use `OnPlaybackTick` as their execution trigger. This means:

1. Graphs are created while the stage is in a stopped state (no topics published).
2. Once `app_utils.play()` is called (or Play is pressed in the GUI), `OnPlaybackTick` fires each frame and drives all connected publishers/subscribers.
3. Stopping playback immediately silences all topics.

This is the correct pattern for fleet demos — Nav2 nodes can be launched and waiting before topics appear, and the fleet begins publishing atomically when Play starts.

### API Usage

```python
from multi_robot_namespacing import setup_fleet, create_ros2_bridge_for_robot

# Full fleet setup (clock + per-robot graphs)
setup_fleet([
    ("carter1", "/World/Carter1"),
    ("carter2", "/World/Carter2"),
    ("carter3", "/World/Carter3"),
])

# Or create individual robot graphs
create_ros2_bridge_for_robot(
    robot_name="carter1",
    robot_prim_path="/World/Carter1",
    lidar_prim_path="/World/Carter1/Lidar",
    publish_joint_states=True,
)
```

### Per-Robot Published Topics (after Play)

| Topic | Message Type | Frame |
|-------|-------------|-------|
| `/<name>/odom` | `nav_msgs/Odometry` | `<name>/odom` → `<name>/base_link` |
| `/<name>/scan` | `sensor_msgs/LaserScan` | `<name>/lidar_link` |
| `/<name>/joint_states` | `sensor_msgs/JointState` | — |
| `/tf` | `tf2_msgs/TFMessage` | robot subtree |
| `/clock` | `rosgraph_msgs/Clock` | shared, one graph |

### Subscribed Topics

| Topic | Message Type | Use |
|-------|-------------|-----|
| `/<name>/cmd_vel` | `geometry_msgs/Twist` | Velocity commands from Nav2 |

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
