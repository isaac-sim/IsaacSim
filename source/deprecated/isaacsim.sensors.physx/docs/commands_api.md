# Commands
Public command API for module **isaacsim.sensors.physx**:

- [IsaacSensorCreateLightBeamSensor](#isaacsensorcreatelightbeamsensor)
- [RangeSensorCreateGeneric](#rangesensorcreategeneric)
- [RangeSensorCreateLidar](#rangesensorcreatelidar)
- [RangeSensorCreatePrim](#rangesensorcreateprim)


## IsaacSensorCreateLightBeamSensor
Command class to create a light beam sensor.

### Arguments
- path: Path for the new prim.
- parent: Parent prim path.
- translation: Translation vector for the prim.
- orientation: Orientation quaternion for the prim.
- num_rays: Number of rays for the light beam sensor.
- curtain_length: Length of the curtain for multi-ray sensors.
- forward_axis: Forward direction axis.
- curtain_axis: Curtain direction axis.
- min_range: Minimum range of the sensor.
- max_range: Maximum range of the sensor.
- draw_points: Whether to draw points for visualization.
- draw_lines: Whether to draw lines for visualization.

### Usage

```python
import omni.kit.commands
from pxr import Gf

# Create a light beam sensor with default settings
result, prim = omni.kit.commands.execute(
    "IsaacSensorCreateLightBeamSensor",
    path="/LightBeam_Sensor",
    parent=None,
    translation=Gf.Vec3d(0, 0, 1),
    orientation=Gf.Quatd(1, 0, 0, 0),
    num_rays=5,
    curtain_length=2.0,
    forward_axis=Gf.Vec3d(1, 0, 0),
    curtain_axis=Gf.Vec3d(0, 0, 1),
    min_range=0.4,
    max_range=100.0,
    draw_points=True,
    draw_lines=True
)
```

## RangeSensorCreateGeneric
Command class to create a generic range sensor.

Typical usage example:

.. code-block:: python

result, prim = omni.kit.commands.execute(
"RangeSensorCreateGeneric",
path="/GenericSensor",
parent=None,
translation=Gf.Vec3d(0, 0, 0),
orientation=Gf.Quatd(1, 0, 0, 0),
min_range=0.4,
max_range=100.0,
draw_points=False,
draw_lines=False,
sampling_rate=60,
)

### Arguments
- path
- parent
- translation
- orientation
- min_range
- max_range
- draw_points
- draw_lines
- sampling_rate

### Usage

```python
import omni.kit.commands
from pxr import Gf

# Create a generic range sensor with default parameters
result, prim = omni.kit.commands.execute(
    "RangeSensorCreateGeneric",
    path="/GenericSensor",
    parent=None,
    translation=Gf.Vec3d(0, 0, 1),
    orientation=Gf.Quatd(1, 0, 0, 0),
    min_range=0.4,
    max_range=100.0,
    draw_points=True,
    draw_lines=True,
    sampling_rate=60
)
```

## RangeSensorCreateLidar
Command class to create a lidar sensor.

Typical usage example:

.. code-block:: python

result, prim = omni.kit.commands.execute(
"RangeSensorCreateLidar",
path="/Lidar",
parent=None,
translation=Gf.Vec3d(0, 0, 0),
orientation=Gf.Quatd(1, 0, 0, 0),
min_range=0.4,
max_range=100.0,
draw_points=False,
draw_lines=False,
horizontal_fov=360.0,
vertical_fov=30.0,
horizontal_resolution=0.4,
vertical_resolution=4.0,
rotation_rate=20.0,
high_lod=False,
yaw_offset=0.0,
enable_semantics=False,
)

### Arguments
- path
- parent
- translation
- orientation
- min_range
- max_range
- draw_points
- draw_lines
- horizontal_fov
- vertical_fov
- horizontal_resolution
- vertical_resolution
- rotation_rate
- high_lod
- yaw_offset
- enable_semantics

### Usage

```python
import omni.kit.commands
from pxr import Gf

# Create a lidar sensor with default parameters
omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path="/World/Lidar",
    parent="/World",
    translation=Gf.Vec3d(0, 0, 2),
    orientation=Gf.Quatd(1, 0, 0, 0),
    min_range=0.4,
    max_range=100.0,
    draw_points=True,
    draw_lines=False,
    horizontal_fov=360.0,
    vertical_fov=30.0,
    horizontal_resolution=0.4,
    vertical_resolution=4.0,
    rotation_rate=20.0,
    high_lod=False,
    yaw_offset=0.0,
    enable_semantics=False
)
```

## RangeSensorCreatePrim
Base command for creating range sensor prims.

This command is used to create each range sensor prim and handles undo operations
so that individual prim commands don't have to implement their own undo logic.

### Arguments
- path: Path for the new prim.
- parent: Parent prim path.
- schema_type: Schema type to use for the prim.
- translation: Translation vector for the prim.
- orientation: Orientation quaternion for the prim.
- visibility: Whether the prim is visible.
- min_range: Minimum range of the sensor.
- max_range: Maximum range of the sensor.
- draw_points: Whether to draw points for visualization.
- draw_lines: Whether to draw lines for visualization.

### Usage

```python
import omni.kit.commands
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from pxr import Gf

# Create a basic range sensor prim using RangeSensorCreatePrim
result, prim = omni.kit.commands.execute(
    "RangeSensorCreatePrim",
    path="/World/RangeSensor",
    parent="",
    schema_type=RangeSensorSchema.Lidar,
    translation=Gf.Vec3d(0, 0, 1),
    orientation=Gf.Quatd(1, 0, 0, 0),
    visibility=False,
    min_range=0.4,
    max_range=100.0,
    draw_points=True,
    draw_lines=True
)
```

