# Overview

The isaacsim.robot.surface_gripper extension provides functionality for creating and controlling surface-based grippers in Isaac Sim, including suction grippers and distance-based grippers. Surface grippers can attach to and manipulate objects through contact detection rather than traditional mechanical clamping, making them suitable for handling a wide variety of objects with different shapes and materials.

```{image} ../../../../source/extensions/isaacsim.robot.surface_gripper/data/preview.png
---
align: center
---
```


## Key Components

### {func}`create_surface_gripper <isaacsim.robot.surface_gripper.create_surface_gripper>`

The {func}`create_surface_gripper <isaacsim.robot.surface_gripper.create_surface_gripper>` function creates a Surface Gripper prim under a given parent path. It automatically picks a unique prim name (e.g. ``SurfaceGripper``, ``SurfaceGripper_01``).

```python
import omni.usd
from isaacsim.robot.surface_gripper import create_surface_gripper

stage = omni.usd.get_context().get_stage()
gripper_prim = create_surface_gripper(stage, "/World/ee_link")
```

```{deprecated} 3.6.0
The ``CreateSurfaceGripper`` Kit command is deprecated. Use ``create_surface_gripper(stage, prim_path)`` directly instead.
```

### {class}`GripperView <isaacsim.robot.surface_gripper.GripperView>`

The {class}`GripperView <isaacsim.robot.surface_gripper.GripperView>` class provides batch operations for managing multiple surface grippers simultaneously. It inherits from XformPrim and supports regex-based path matching to control collections of grippers efficiently.

```python
gripper_view = GripperView(
    paths="/World/Env[1-5]/Gripper",
    max_grip_distance=np.array([0.1, 0.1, 0.1, 0.1, 0.1]),
    coaxial_force_limit=np.array([50.0, 50.0, 50.0, 50.0, 50.0])
)
```

The view supports configurable parameters including maximum grip distance, coaxial and shear force limits, and retry intervals for gripping attempts.

### SurfaceGripperInterface

The SurfaceGripperInterface provides low-level control operations for individual grippers and batch operations. It manages gripper states through three primary states: Open, Closing, and Closed.

Action values control gripper behavior where values less than -0.3 open the gripper, values greater than 0.3 close the gripper, and intermediate values maintain the current state.

## Functionality

### Grip Control

Surface grippers operate through contact detection and force-based attachment. When commanded to close, the gripper attempts to grip objects in contact with its surface. The gripping mechanism considers both coaxial forces (perpendicular to the surface) and shear forces (parallel to the surface) to determine grip stability.

### Batch Operations

The extension provides efficient batch operations for controlling multiple grippers simultaneously. Both {class}`GripperView <isaacsim.robot.surface_gripper.GripperView>` and SurfaceGripperInterface support batch methods that process multiple gripper paths in parallel, improving performance in multi-robot scenarios.

### State Management

Grippers maintain state information including current status (Open, Closing, Closed), gripped object paths, and configurable properties. The interface can optionally write state changes to USD for persistence or maintain them in memory for improved performance.

### Force Limits

Each gripper can be configured with coaxial and shear force limits that determine when gripped objects will be released. This prevents damage to both the gripper and handled objects while providing realistic gripping behavior.
