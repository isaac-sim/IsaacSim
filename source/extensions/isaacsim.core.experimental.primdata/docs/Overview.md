# Overview

The isaacsim.core.experimental.primdata extension provides the default C++ provider implementation for the `IPrimDataReader` and `IPrimDataReaderManager` Carbonite interfaces defined in `isaacsim.core.experimental.prims`. It delivers high-performance, read-only access to prim transform, rigid body, and articulation data from both PhysX and Newton physics backends, with support for CPU and GPU (CUDA) buffer storage.

The interface contracts remain in `isaacsim.core.experimental.prims` so consumer extensions can depend on stable shared Carbonite interfaces while selecting the concrete provider through extension enablement.

## Key Components

### PrimDataReader

The `PrimDataReader` implements the `IPrimDataReader` interface, managing the lifecycle of data views for different prim types. It routes data access by physics engine:

- **PhysX**: Direct C++ TensorApi calls for zero-copy GPU buffer access without Python involvement
- **Newton**: Python callbacks fill C++-owned buffers via registered field callbacks
- **Transforms**: Engine-agnostic Fabric/USDRT hierarchy queries for world and local poses

The reader supports three view types:

- **XformDataView**: Read-only access to world positions, world orientations, local translations, local orientations, and local scales for transform prims
- **RigidBodyDataView**: Extends transform data with linear and angular velocities for rigid body prims
- **ArticulationDataView**: Extends transform data with DOF positions, velocities, efforts, root transforms, root velocities, DOF types, DOF names, link hierarchy, Jacobians, and mass matrices for articulated robots

### PrimDataReaderManager

The `PrimDataReaderManager` implements the `IPrimDataReaderManager` interface, centralizing lifecycle management for the shared `IPrimDataReader` instance. It subscribes to physics simulation events (stop/resume) to trigger automatic reinitialization, ensuring that sensor plugins and nodes do not need to call `initialize()` independently.

### Buffer Management

The `BufferRegistry` module provides per-field buffer state management through `FieldEntry` structures. Each named field in a view owns a device buffer (CPU or GPU), an optional CPU staging buffer for host-side access, and a fill callback with dirty tracking based on physics step counts. This lazy evaluation ensures buffers are only updated when accessed and stale.

## Functionality

### Lazy Evaluation and Dirty Tracking

Field data is fetched on demand. Each field tracks the last physics step at which it was filled. When a consumer requests data, the reader compares the current step count against the field's last update and runs the fill callback only if the data is stale, minimizing redundant physics queries.

### Host and Device Access

All view types provide both device-pointer and host-pointer access methods. For GPU-resident data, host variants automatically copy from the device staging buffer to a CPU staging buffer, caching the result until the next physics step invalidates it.

### Physics Engine Integration

For PhysX views, the reader creates native `IArticulationView` and `IRigidBodyView` handles through the TensorApi, enabling direct C++ buffer fills without crossing the Python boundary. For Newton views, the buffer infrastructure supports Python-registered callbacks that write into the same C++-managed buffers.
