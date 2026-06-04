```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Overview

The `isaacsim.physics.newton.tensors` extension provides a C++ tensor backend for the Newton physics engine,
implementing the `omni.physics.tensors` interface. It enables batched get/set operations on articulations,
rigid bodies, and contact sensors with native CUDA acceleration and zero-copy Warp array interop.

The backend registers as a first-class `omni.physics.tensors` plugin, allowing Newton simulations to be
controlled through the same Python API used by the PhysX backend.

```text
omni.physics.tensors (api.py)
        │
        ▼  C++ plugin
SimulationBackend
        │
        ▼
BaseSimulationView
   ├── CpuSimulationView          ├── GpuSimulationView
   │     creates:                 │     creates:
   │     ├─ CpuArticulationView   │     ├─ GpuArticulationView
   │     ├─ CpuRigidBodyView      │     ├─ GpuRigidBodyView
   │     └─ CpuRigidContactView   │     └─ GpuRigidContactView
   │                              │
   │   Each Cpu/Gpu view inherits from a shared Base class:
   │
   ├── BaseArticulationView  ◄── CpuArticulationView, GpuArticulationView
   ├── BaseRigidBodyView     ◄── CpuRigidBodyView,    GpuRigidBodyView
   └── BaseRigidContactView  ◄── CpuRigidContactView, GpuRigidContactView
```

## Architecture

The backend separates device-independent logic from device-specific data transfer through a three-layer
class hierarchy for each view type:

- **Base classes** (`BaseSimulationView`, `BaseArticulationView`, `BaseRigidBodyView`, `BaseRigidContactView`)
  hold Newton/Python state, build index mappings from the Newton model, cache raw Warp array pointers,
  and implement the `omni.physics.tensors` interface methods that are shared across devices.

- **CPU classes** (`CpuSimulationView`, `CpuArticulationView`, `CpuRigidBodyView`, `CpuRigidContactView`)
  implement getter/setter methods using CPU gather/scatter loops that read from and write to
  host-side Warp arrays directly.

- **GPU classes** (`GpuSimulationView`, `GpuArticulationView`, `GpuRigidBodyView`, `GpuRigidContactView`)
  implement getter/setter methods using CUDA kernels, pre-allocated device index buffers, and
  a staging buffer for D2H transfers when the caller's output tensor is on CPU.

The simulation backend automatically selects CPU or GPU view classes based on the Newton model's
device. GPU views also support the GC (GPU simulation, CPU view tensors) configuration by staging
CPU index and source data into pre-allocated device buffers before kernel execution.

### Memory Management

All GPU memory is allocated during construction and reused across API calls:

- **Device index buffers** store DOF, link, body, and root joint index mappings uploaded once from host.
- **Staging buffers** are pre-allocated to the maximum output size needed by any getter or setter.
- **Scratch buffers** in CPU views and contact views are pre-allocated members reused across calls,
  avoiding per-call heap allocation.
- No `cudaMalloc`, `cudaFree`, or `std::vector` heap allocation occurs in runtime API methods.

### Warp Interop

The backend accesses Newton simulation data (stored as Warp arrays in Python) directly from C++
via `pybind11`. Raw data pointers are extracted once and cached in base classes, avoiding repeated
Python attribute lookups in hot paths. GPU views operate on these cached pointers directly when
launching CUDA kernels.

## Key Components

### SimulationView

The simulation view is the entry point for creating child views. It detects the Newton model's
simulation device and selects the appropriate CPU or GPU implementation. Factory methods
(`createArticulationView`, `createRigidBodyView`, `createRigidContactView`) perform USD pattern
matching, resolve paths against the Newton model, and delegate to the device-specific subclass.

```python
import omni.physics.tensors as tensors

sim_view = tensors.create_simulation_view(backend="newton")
```

### ArticulationView

Provides batched access to articulation state: DOF positions, velocities, limits, stiffnesses,
dampings, armatures, actuation forces, targets, root transforms/velocities, link transforms/velocities,
masses, COMs, and inertias. Supports both indexed and full-view operations.

The view builds index mappings at construction from the Newton model's `joint_q_start`,
`joint_qd_start`, `joint_type`, and `body_label` arrays. These mappings handle non-uniform
DOF counts across articulations by padding with sentinel values (`-1`).

```python
arti_view = sim_view.create_articulation_view("/World/Robot")

positions = arti_view.get_dof_positions()
arti_view.set_dof_positions(new_positions)

transforms = arti_view.get_root_transforms()
arti_view.set_root_transforms(new_transforms)
```

### RigidBodyView

Provides batched access to rigid body state: transforms (as 7-float `[x,y,z,qx,qy,qz,qw]`),
velocities (as 6-float `[vx,vy,vz,wx,wy,wz]`), masses, inverse masses, COMs, inertias,
and inverse inertias. Supports force and torque application at arbitrary positions.

```python
rb_view = sim_view.create_rigid_body_view("/World/Objects/.*")

transforms = rb_view.get_transforms()
rb_view.set_velocities(new_velocities)
rb_view.apply_forces_and_torques_at_position(forces, torques, positions)
```

### RigidContactView

Provides contact force queries between sensor bodies and filter bodies:

- **Net contact forces**: Aggregated force per sensor body.
- **Contact force matrix**: Force per sensor-filter pair.
- **Contact data**: Per-contact point forces, positions, normals, and separation distances.
- **Raw contact data**: Per-contact data with other-actor identification.

Contact pointers are refreshed lazily using the Newton stage's `simulation_timestamp`,
avoiding redundant Python queries when the simulation state has not changed.

```python
contact_view = sim_view.create_rigid_contact_view(
    "/World/Robot/.*",
    ["/World/Ground"]
)

net_forces = contact_view.get_net_contact_forces(dt)
force_matrix = contact_view.get_contact_force_matrix(dt)
```

### ArticulationMetatype

Provides structural metadata for each articulation: link/joint/DOF counts and names,
joint types, DOF types, joint DOF offsets, parent link relationships, and fixed-base detection.
Built from Newton model metadata at construction time.

## Device Configurations

The backend supports three device configurations:

| Configuration | Simulation | View Tensors | Implementation |
|---|---|---|---|
| **CC** | CPU | CPU | `CpuSimulationView` + `Cpu*View` classes |
| **GC** | GPU | CPU | `GpuSimulationView` + `Gpu*View` with H2D staging |
| **GG** | GPU | GPU | `GpuSimulationView` + `Gpu*View` with direct device pointers |

In the GC configuration, GPU views transparently stage CPU index and source tensors to
pre-allocated device buffers before kernel execution, and stage GPU results back to
host-side output tensors after kernel completion.

## Testing

Tests are located in `python/tests/` and use the standard extension test framework. Each test class
runs in three device configurations (CC, GC, GG) via parameterized base classes.

### Running Tests

Run all tests:
```bash
./repo.sh test --ext-folder source/extensions/isaacsim.physics.newton.tensors
```

Run a specific test class:
```bash
./repo.sh test --ext-folder source/extensions/isaacsim.physics.newton.tensors --filter TestArticulationViewBasic
```

### Test Coverage

- **`test_articulation.py`**: Articulation view creation, DOF position/velocity get/set, root transform/velocity, link transforms, metatype properties, simulation view gravity
- **`test_rigid_body.py`**: Rigid body transforms, velocities, masses, COMs, inertias, force application
- **`test_rigid_contact.py`**: Net contact forces, contact force matrix, contact data, raw contact data
- **`test_object_type.py`**: `getObjectType` correctness for articulation roots, links, free bodies

## Dependencies

- **omni.physics.tensors**: C++ interface headers (`ISimulationView.h`, `IArticulationView.h`, `IRigidBodyView.h`, `IRigidContactView.h`, `TensorDesc.h`)
- **isaacsim.physics.newton**: Newton physics engine and stage management
- **pybind11**: Accessing Python Newton objects from C++
- **pxr (USD)**: `SdfPath`, `UsdStage`, `UsdPrim` for pattern matching
- **CUDA runtime**: Kernel launch, `cudaMemcpy`, device memory management
- **Warp** (runtime): Python arrays accessed via `WarpInterop`; `WarpCompat.h` for compile-time type definitions
