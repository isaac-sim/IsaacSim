# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.sensors.physics.examples`.
```

The isaacsim.sensors.physx.examples extension provides interactive demonstrations of PhysX-based sensor functionality in Isaac Sim. This extension includes three comprehensive examples showcasing different types of physics-based sensors: Generic Range Sensors, LIDAR sensors, and LightBeam sensors, each demonstrating real-time data collection through PhysX line tracing for collision detection and distance measurements.

```{image} ../data/preview.png
---
align: center
---
```


## Key Components

### Generic Range Sensor Example

**Interactive demonstration showcasing custom scanning patterns and sensor configuration.** The example demonstrates how to create PhysX-based range sensors with customizable scanning patterns, configure sensor properties like range limits and sampling rates, and process sensor data in both streaming and batch modes.

Key workflows include:
- Creating sensors with custom scanning patterns
- Setting up physics scenes with collision objects  
- Streaming continuous data flow or batch processing predefined patterns
- Visualizing scanning patterns through point cloud data
- Saving pattern images for analysis
- Processing point cloud data to extract specific surface hits

### LIDAR Sensor Example  

**Real-time LIDAR sensor simulation with customizable parameters and live data visualization.** This example provides an interactive interface for configuring LIDAR sensors and viewing their output in real-time, including depth, azimuth, and zenith measurements displayed in formatted tables.

Features include:
- Configurable LIDAR parameters (FOV, resolution, range, rotation rate)
- Real-time data streams showing sensor measurements
- Obstacle environment creation for testing
- Viewport visualization of LIDAR rays and detection points
- Automatic stage configuration with Z-up axis orientation and physics scene setup

### LightBeam Sensor Example

**Physics-based ray casting demonstration with interactive object manipulation.** The LightBeam sensor example creates a scene with a moveable cube target and demonstrates real-time sensor response to object positioning and movement.

The demonstration includes:
- Real-time visualization of 5 light beam rays with color-coded display
- Live updates of beam hit detection, linear depth, and hit position data
- Interactive cube manipulation for testing sensor behavior
- Automatic scene setup with proper lighting and camera positioning
- Action graph configuration for sensor data processing and visualization

## Functionality

### Data Processing and Visualization

Each example provides specialized data processing capabilities tailored to the specific sensor type. The Generic Range Sensor example includes point cloud filtering to extract wall surface hits, while the LIDAR example formats measurement data into readable tables, and the LightBeam example provides real-time hit detection status updates.

### Scene Configuration

All examples automatically configure the USD stage with appropriate physics settings, including PhysicsScene setup required for PhysX line tracing operations. The extensions handle stage initialization, object creation with collision properties, and camera positioning for optimal demonstration viewing.

### Interactive Controls

The extensions provide UI panels with buttons and controls for loading sensors, spawning test environments, configuring sensor parameters, and displaying live data streams. Users can interact with demonstration scenes by manipulating objects and observing real-time sensor responses.

## Integration

The extension integrates with isaacsim.examples.browser to register demonstrations in Isaac Sim's examples interface, allowing users to easily discover and launch sensor examples. It uses isaacsim.sensors.physx for the underlying sensor implementations and isaacsim.gui.components for consistent UI elements across demonstrations.

The examples work within Isaac Sim's physics simulation system, requiring active PhysicsScene components and supporting real-time timeline updates for continuous sensor data collection and visualization.
