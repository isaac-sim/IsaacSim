# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.sensors.experimental.rtx`.
```

The isaacsim.sensors.rtx extension provides APIs for creating and managing RTX-based sensors in Isaac Sim, including RTX Lidar, RTX Radar, and RTX Idealized Depth Sensors (IDS). These sensors leverage RTX ray tracing technology for high-fidelity sensor simulation in robotics applications.

## Migration

New code should use the `isaacsim.sensors.experimental.rtx` extension. The mappings below cover the most common call sites; see the **Multitick Rendering** section of the Isaac Sim sensors documentation for the full migration guide, including the multi-tick rendering changes that ship with Isaac Sim 6.0.

| 5.x (this extension)                                  | 6.0 replacement (`isaacsim.sensors.experimental.rtx`)                                                                  |
| ----------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `IsaacSensorCreateRtxLidar` Kit command               | {class}`Lidar.create(path, config=...) <isaacsim.sensors.experimental.rtx.Lidar>`                                      |
| `IsaacSensorCreateRtxRadar` Kit command               | {class}`Radar(path, ...) <isaacsim.sensors.experimental.rtx.Radar>`                                                    |
| `IsaacSensorCreateRtxIDS` Kit command                 | No equivalent in `isaacsim.sensors.experimental.rtx` today; may be supported in a future release. In the meantime, continue to use this deprecated command or author the IDS occupancy prim directly in USD. |
| `LidarRtx` runtime class                              | {class}`LidarSensor <isaacsim.sensors.experimental.rtx.LidarSensor>` wrapping a {class}`Lidar <isaacsim.sensors.experimental.rtx.Lidar>` authoring object |
| `omni:sensor:Core:auxOutputType` USD attribute (Lidar) | `_replicator:rendervar:GenericModelOutput:channels = ["FULL"]` on the `OmniLidar` prim, or `aux_output_level="FULL"` on the constructor |
| `omni:sensor:WpmDmat:auxOutputType` USD attribute (Radar) | `_replicator:rendervar:GenericModelOutput:channels = ["BASIC"]` on the `OmniRadar` prim, or `aux_output_level="BASIC"` on the constructor |
| `IsaacExtractRTXSensorPointCloudNoAccumulator` annotator | `IsaacCreateRTXLidarScanBuffer` with `omni:sensor:Core:accumulateOutputs = false` on the prim, or the `IsaacExtractRTXSensorPointCloud` annotator from `isaacsim.sensors.rtx.nodes` |
| `RtxLidarDebugDrawPointCloudBuffer` writer            | Same writer; still registered alongside the experimental extension                                                     |
| Implicit "render every frame" sensor scheduling       | Set `omni:sensor:tickRate` on the prim. For `OmniLidar` it must equal `omni:sensor:Core:scanRateBaseHz`, otherwise the lidar emits partial scans every frame (see the **Multitick Rendering** documentation, which notes that the `OmniLidar` tick rate must match its scan rate) |

The deprecated extension still ships and continues to publish RTX sensor data to ROS 2/UCX/HSB pipelines for backward compatibility, but new features are added only to `isaacsim.sensors.experimental.rtx`.

## Key Components

### Sensor Creation Commands

The extension provides specialized command classes that inherit from `IsaacSensorCreateRtxSensor` for creating different types of RTX sensors:

- **{class}`IsaacSensorCreateRtxLidar <isaacsim.sensors.rtx.IsaacSensorCreateRtxLidar>`**: Creates RTX Lidar sensors with configurable parameters for point cloud generation
- **{class}`IsaacSensorCreateRtxRadar <isaacsim.sensors.rtx.IsaacSensorCreateRtxRadar>`**: Creates RTX Radar sensors with motion compensation capabilities (requires Motion BVH)
- **{class}`IsaacSensorCreateRtxIDS <isaacsim.sensors.rtx.IsaacSensorCreateRtxIDS>`**: Creates RTX Idealized Depth Sensors with occupancy detection features

These commands support multiple creation methods including USD references, Replicator API integration, or direct camera prim creation based on the configuration and available APIs.

### {class}`LidarRtx <isaacsim.sensors.rtx.LidarRtx>` Sensor Interface

The {class}`LidarRtx <isaacsim.sensors.rtx.LidarRtx>` class provides a comprehensive interface for RTX-based Lidar sensors, extending the base sensor API with RTX-specific functionality:

```python
from isaacsim.sensors.rtx import LidarRtx

# Create and configure a Lidar sensor
lidar = LidarRtx(prim_path="/World/Lidar", name="my_lidar")
lidar.initialize()

# Attach annotators for data collection
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer")

# Get sensor data
frame_data = lidar.get_current_frame()
```

### Annotators and Writers

The extension supports various annotators for different data collection modes:

- **IsaacComputeRTXLidarFlatScan**: Generates flat scan data with point clouds and metadata
- **IsaacExtractRTXSensorPointCloudNoAccumulator**: Extracts point cloud data without accumulation
- **IsaacCreateRTXLidarScanBuffer**: Creates structured scan buffers for processing
- **StableIdMap**: Provides stable object identification mapping
- **{class}`GenericModelOutput <isaacsim.sensors.rtx.generic_model_output.GenericModelOutput>`**: Outputs standardized sensor data format

Writers enable visualization and debug capabilities, such as `RtxLidarDebugDrawPointCloud` for real-time point cloud visualization.

## Relationships

### Generic Model Output

The `isaacsim.sensors.rtx.generic_model_output` module defines a standardized data format for sensor outputs. It provides the {class}`GenericModelOutput <isaacsim.sensors.rtx.generic_model_output.GenericModelOutput>` class and associated enums for sensor modalities (LIDAR, RADAR, USS, IDS), coordinate types (CARTESIAN, SPHERICAL), and frame reference systems. This module enables consistent data exchange between different sensor types and external processing systems.

### Nonvisual Materials

The `isaacsim.sensors.rtx.nonvisual_materials` module provides functionality for applying and retrieving nonvisual material properties used by RTX sensors like LiDAR and radar. It includes {func}`apply_nonvisual_material <isaacsim.sensors.rtx.apply_nonvisual_material>` for setting material properties on USD prims, {func}`get_material_id <isaacsim.sensors.rtx.get_material_id>` for reading the current material assignment, and {func}`decode_material_id <isaacsim.sensors.rtx.decode_material_id>` for decomposing a material ID into its base, surface, and retroreflective categories. The module contains predefined dictionaries covering metals, non-metals, vegetation, and other material categories.

### Sensor Checker

The `isaacsim.sensors.rtx.sensor_checker` module provides validation utilities through the {class}`SensorCheckerUtil <isaacsim.sensors.rtx.sensor_checker.SensorCheckerUtil>` class. It validates sensor configurations, parameters, and AOV (Arbitrary Output Variable) data against predefined schemas, ensuring sensor models conform to expected specifications before deployment.

## Functionality

### Configuration Management

The extension includes predefined sensor configurations accessible through `SUPPORTED_LIDAR_CONFIGS`, covering various real-world sensor models from manufacturers like Velodyne, Ouster, HESAI, and others. These configurations provide realistic sensor parameters for accurate simulation.

### Data Processing Pipeline

RTX sensors support a flexible data processing pipeline through the annotator system. Users can attach multiple annotators to collect different types of sensor data simultaneously, such as point clouds, depth maps, intensity values, and object identification information.

### Motion Compensation

RTX Radar sensors include motion compensation capabilities when Motion BVH is enabled, providing accurate velocity measurements for moving objects in the simulation environment.
