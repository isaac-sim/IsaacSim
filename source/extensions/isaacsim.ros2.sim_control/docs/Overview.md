# Overview

The `isaacsim.ros2.sim_control` extension bridges Isaac Sim with ROS 2 by implementing the standard `simulation_interfaces` ROS 2 package. It enables external ROS 2 nodes to control Isaac Sim's timeline, spawn and delete entities, query simulation state, and manage worlds through service-based and action-based communication.

## Key Components

The extension exposes two main classes:

- {class}`ROS2ServiceManager <isaacsim.ros2.sim_control.ROS2ServiceManager>` — Singleton that owns the ROS 2 node, `MultiThreadedExecutor`, and `ReentrantCallbackGroup`. Provides `register_service()` and `register_action_server()` methods for attaching handlers at startup.
- {class}`SimulationControl <isaacsim.ros2.sim_control.SimulationControl>` — Registers all 19 services and the `SimulateSteps` action server against the `ROS2ServiceManager`. Implements each callback using Isaac Sim's experimental prims API and stage utilities.

Helper utilities in {mod}`isaacsim.ros2.sim_control` include:

- {func}`get_filtered_entities <isaacsim.ros2.sim_control.get_filtered_entities>` — Traverses the USDRT stage and applies optional regex filtering to return matching prim paths.
- {func}`get_entity_state <isaacsim.ros2.sim_control.get_entity_state>` — Async function that retrieves pose, linear velocity, and angular velocity for a given entity, using {class}`RigidPrim` for rigid bodies or {class}`XformPrim` for other prims.
- {func}`create_empty_entity_state <isaacsim.ros2.sim_control.create_empty_entity_state>` — Returns an `EntityState` message with default zero values.

## Functionality

### Timeline control

The extension registers ROS 2 services to control Isaac Sim's simulation timeline:

- **Get/Set simulation state**: Query or set the simulation state (`STOPPED`, `PLAYING`, `PAUSED`).
- **Step simulation**: Advance the simulation by a specified number of physics frames while paused.
- **Reset simulation**: Remove all entities spawned via `SpawnEntity` or `SpawnEntities` and restore the stage to its original state.

The `SimulateSteps` action server provides step-by-step execution with per-frame feedback, allowing clients to monitor progress and cancel mid-run.

### Entity management

The extension supports dynamic entity operations through ROS 2 services:

- **Spawn entity**: Create a new entity from a USD asset URI or as an empty Xform prim. Duplicate names are auto-renamed. Spawned entities are tagged with `simulationInterfacesSpawned` so `ResetSimulation` can remove them.
- **Spawn entities**: Batch-spawn multiple entities in a single request, with per-entity success/error results.
- **Delete entity**: Remove an entity from the stage by prim path.
- **Get entities**: List all prim paths in the stage, with optional regex filtering.
- **Get entity info**: Return the category of an entity (currently always `OBJECT`).
- **Get entity bounds**: Return the world-space axis-aligned bounding box (AABB) of an entity as a `TYPE_BOX` geometry.

### Entity state

- **Get entity state**: Retrieve pose, linear velocity, and angular velocity for a prim. Rigid bodies return full velocity data; non-rigid bodies return pose only.
- **Get entities states**: Batch-retrieve states for multiple entities, with optional regex filtering.
- **Set entity state**: Set the pose and optionally the linear and angular velocity of an entity at runtime.

### World management

- **Load world**: Open a USD world file onto the stage, with optional asset root path prefix expansion.
- **Unload world**: Clear the current stage and create a new empty anonymous stage.
- **Get current world**: Return information about the currently loaded world, including its resource URI.
- **Get available worlds**: Discover USD world files in configured search paths, with tag-based filtering using `ANY` or `ALL` matching modes.

### Asset and feature discovery

- **Get spawnables**: Discover available USD assets from `/Isaac/Samples/ROS2/Robots` in the Isaac assets root path, plus any additional caller-supplied source paths.
- **Get simulator features**: Return the list of `simulation_interfaces` features supported by this extension.

## Integration

The extension depends on:

- `isaacsim.ros2.bridge` — Provides the ROS 2 context and `rclpy` bindings used by `ROS2ServiceManager`.
- `isaacsim.core.experimental.prims` — Supplies `RigidPrim` and `XformPrim` for reading and writing entity poses and velocities.
- `isaacsim.core.experimental.utils` — Supplies stage utilities (`open_stage_async`, `create_new_stage_async`) and prim utilities used by world and entity operations.
- `isaacsim.storage.native` — Supplies `get_assets_root_path_async`, `resolve_asset_path_async`, `find_filtered_files_async`, and `is_local_path` for asset URI resolution and discovery.

Service callbacks execute on a `MultiThreadedExecutor` with a `ReentrantCallbackGroup`, allowing multiple services to run concurrently. Async Isaac Sim operations are bridged to the synchronous ROS 2 callback thread using `nest_asyncio`.
