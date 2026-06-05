# Overview

The isaacsim.core.experimental.utils extension provides a comprehensive set of utility functions for Isaac Sim applications. These utilities streamline common operations across application lifecycle management, USD stage manipulation, prim operations, geometric transformations, and backend switching, offering developers essential tools for building Isaac Sim applications efficiently.

## Functionality

### Application Management

The extension provides comprehensive application lifecycle control through timeline operations, extension management, and update stepping. Users can programmatically control the timeline state with `play()`, `pause()`, and `stop()` functions, while also managing extension loading and querying extension information.

```python
import isaacsim.core.experimental.utils.app as app_utils

# Control timeline playback
app_utils.play()
app_utils.pause() 
app_utils.stop()

# Step through application updates
app_utils.update_app(steps=10)

# Manage extensions
app_utils.enable_extension("omni.pip.cloud")
app_utils.is_extension_enabled("omni.pip.cloud")
```

### Stage Operations

Stage management utilities handle USD file operations, prim creation and manipulation, and stage configuration. The extension supports creating new stages from templates, opening and saving USD files, and managing stage properties like units and coordinate systems.

```python
import isaacsim.core.experimental.utils.stage as stage_utils

# Create and manage stages
stage_utils.create_new_stage(template="sunlight")
stage_utils.open_stage("/path/to/file.usd")
stage_utils.save_stage("/path/to/output.usd")

# Define and manipulate prims
stage_utils.define_prim("/World/Cube", "Cube")
stage_utils.add_reference_to_stage("/path/to/asset.usd", "/World/Asset")
```

### Prim Utilities

Comprehensive prim manipulation tools enable attribute management, API schema operations, and hierarchical searches. These utilities simplify working with USD prims by providing convenient functions for common operations.

```python
import isaacsim.core.experimental.utils.prim as prim_utils

# Work with prim attributes
value = prim_utils.get_prim_attribute_value("/World/Cube", "size")
attributes = prim_utils.get_prim_attribute_names("/World/Cube")

# Find prims with predicates
matching_prims = prim_utils.get_all_matching_child_prims(
    "/World", 
    predicate=lambda prim, path: prim.GetTypeName() == "Cube"
)
```

### Transform Operations

Mathematical transformation utilities support conversion between rotation representations, quaternion operations, and coordinate system transformations. These functions work with various input formats including lists, NumPy arrays, and Warp arrays.

```python
import isaacsim.core.experimental.utils.transform as transform_utils

# Convert between rotation representations
rotation_matrix = transform_utils.euler_angles_to_rotation_matrix([0, np.pi/2, 0])
quaternion = transform_utils.euler_angles_to_quaternion([0, np.pi/2, 0])

# Perform quaternion operations
result = transform_utils.quaternion_multiplication(quat1, quat2)
conjugate = transform_utils.quaternion_conjugate(quaternion)

# Compute relative transform between two world matrices
relative = transform_utils.compute_relative_transform(source_to_world, target_to_world)

# Compute camera look-at transform
matrix = transform_utils.look_at_matrix(eye=[5.0, 5.0, 5.0], target=[0.0, 0.0, 0.0])
```

The {func}`look_at_matrix <isaacsim.core.experimental.utils.transform.look_at_matrix>` function computes the ``Gf.Matrix4d`` camera transform (position + orientation) that places a camera at a given eye position oriented toward a target. It accepts lists, NumPy arrays, or ``Gf.Vec3d`` and automatically selects a fallback up vector when the forward direction is collinear with the specified up axis. The batched {func}`look_at_quaternion <isaacsim.core.experimental.utils.transform.look_at_quaternion>` variant returns a Warp array quaternion and supports batched inputs for multi-camera workflows.

### Pose Manipulation

Spatial transformation utilities provide local and world pose operations for prims. These functions support getting and setting both local and world coordinates with automatic coordinate system handling.

```python
import isaacsim.core.experimental.utils.xform as xform_utils

# Get and set poses
translation, orientation = xform_utils.get_local_pose("/World/Cube")
world_translation, world_orientation = xform_utils.get_world_pose("/World/Cube")

# Set poses (USDRT/Fabric backends)
xform_utils.set_local_pose("/World/Cube", translation=[1, 2, 3])
xform_utils.set_world_pose("/World/Cube", position=[5, 6, 7])

# Compute relative transform between two prims
relative_tf = xform_utils.get_relative_transform("/World/Source", "/World/Target")
```

## Key Components

### Backend System

The backend system enables switching between USD, USDRT, and Fabric processing backends through context managers. This allows applications to choose the optimal backend for specific operations.

```python
import isaacsim.core.experimental.utils.backend as backend_utils

# Switch processing backends
with backend_utils.use_backend("usdrt"):
    # Operations use USDRT backend
    pass
```

### Data Operations

Array and data manipulation utilities provide consistent interfaces for working with Python primitives, NumPy arrays, and Warp arrays. These utilities handle device placement, type conversion, and broadcasting operations.

```python
import isaacsim.core.experimental.utils.ops as ops_utils

# Convert and place data
array = ops_utils.place([1, 2, 3], device="cuda", dtype=wp.float32)
indices = ops_utils.resolve_indices([0, 1, 2], device="cpu")
broadcasted = ops_utils.broadcast_to(5.0, shape=(3, 3))
```

### Semantic Labeling

Semantic utilities manage taxonomies and labels on USD prims, supporting multiple classification schemes for organizing and querying scene content.

```python
import isaacsim.core.experimental.utils.semantics as semantics_utils

# Manage semantic labels
semantics_utils.add_labels("/World/Cube", labels=["furniture", "wooden"])
labels = semantics_utils.get_labels("/World/Cube")
semantics_utils.remove_labels("/World/Cube", labels=["wooden"])
```

## Integration

The extension integrates with the broader Isaac Sim ecosystem through its dependencies. It uses **omni.usd** for USD operations, **omni.warp.core** for array processing, and **omni.kit.stage_templates** for stage creation templates. The utilities serve as building blocks for higher-level Isaac Sim functionality while maintaining compatibility across different processing backends.
