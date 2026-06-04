# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.sensors.experimental.physics`.
```

The isaacsim.sensors.physx extension provides PhysX-based sensor implementations for robotics simulation in Isaac Sim. It offers range sensors, proximity detection, and rotating lidar functionality that use PhysX raycasting for physics-accurate sensor data collection.

## Key Components

### Range Sensors

The extension provides several range sensor types through command-based creation patterns. **{class}`RangeSensorCreateLidar <isaacsim.sensors.physx.RangeSensorCreateLidar>`** creates rotating lidar sensors with configurable field of view, resolution, and rotation rates. **{class}`RangeSensorCreateGeneric <isaacsim.sensors.physx.RangeSensorCreateGeneric>`** creates basic range sensors for general distance measurement applications. **{class}`IsaacSensorCreateLightBeamSensor <isaacsim.sensors.physx.IsaacSensorCreateLightBeamSensor>`** creates light beam sensors that can form curtain patterns for safety zone detection.

These commands handle USD prim creation, configuration, and provide undo functionality through the Kit command system. They set up the underlying PhysX schemas and attributes needed for sensor operation.

### Rotating Lidar

**{class}`RotatingLidarPhysX <isaacsim.sensors.physx.RotatingLidarPhysX>`** provides a complete rotating lidar sensor implementation that extends the base sensor framework. It supports configurable rotation frequency, field of view, resolution settings, and multiple data output types including depth, intensity, point clouds, and semantic data.

The sensor operates through PhysX simulation callbacks and provides frame-based data collection with pause/resume functionality. It can generate various data types simultaneously:

```python
lidar = RotatingLidarPhysX(prim_path="/World/Lidar", rotation_frequency=20.0)
lidar.add_depth_data_to_frame()
lidar.add_point_cloud_data_to_frame()
lidar.add_intensity_data_to_frame()
```

### Proximity Sensor

**{class}`ProximitySensor <isaacsim.sensors.physx.ProximitySensor>`** uses PhysX box overlap queries to detect when objects enter, remain within, or exit defined detection zones. It tracks overlap duration and distances while providing callback-based event handling for zone transitions.

The sensor uses the parent prim's scale property to define detection box dimensions and maintains internal state for tracking entry and exit events:

```python
def on_enter(sensor):
    print("Object entered detection zone")

def on_exit(sensor):
    print("Object left detection zone")

proximity_sensor = ProximitySensor(
    parent=parent_prim,
    callback_fns=[on_enter, None, on_exit]
)
```

## Functionality

### Data Collection

All sensors provide frame-based data collection with configurable output types. The rotating lidar supports depth, linear depth, intensity, point cloud, zenith/azimuth angles, and semantic segmentation data. Range sensors output distance measurements, hit positions, and beam status information.

### Physics Integration

The sensors integrate directly with PhysX simulation through scene queries and collision detection. They operate on physics step callbacks to ensure data collection synchronizes with simulation timing. The proximity sensor uses continuous overlap detection while range sensors perform raycast queries.

### Visualization

Sensors support built-in visualization with configurable detail levels. Users can enable point cloud rendering, line visualization, and adjust level-of-detail settings for performance optimization during development and debugging.
