# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.sensors.experimental.physics`.
```

The isaacsim.sensors.physics extension provides physics-based sensor simulation capabilities for Isaac Sim. This extension offers contact sensors for detecting physical interactions, IMU sensors for measuring inertial data, and effort sensors for monitoring joint forces and torques in robotic simulations. The sensors integrate with the physics simulation to provide real-time measurements with configurable sampling rates, filtering, and interpolation capabilities.

## Key Components

### {class}`ContactSensor <isaacsim.sensors.physics.ContactSensor>`

**{class}`ContactSensor <isaacsim.sensors.physics.ContactSensor>` detects physical contact and measures contact forces in real-time.** The sensor creates a collision detection area that monitors when objects come into contact and reports the magnitude of contact forces. It requires the parent prim to have UsdPhysics.CollisionAPI enabled and supports configurable force thresholds to filter contacts based on minimum and maximum force values.

The sensor provides both basic contact information (boolean contact state and force magnitude) and detailed contact data including contact positions, normals, and impulses. Detection radius can be configured to limit the sensing area, and the sensor operates at specified frequencies or time steps.

```python
import isaacsim.sensors.physics as sensors_physics

# Create a contact sensor with force thresholds
contact_sensor = sensors_physics.ContactSensor(
    prim_path="/World/ContactSensor",
    min_threshold=1.0,
    max_threshold=1000.0,
    radius=0.5,
    frequency=60
)

# Get current contact data
frame_data = contact_sensor.get_current_frame()
```

### {class}`IMUSensor <isaacsim.sensors.physics.IMUSensor>`

**{class}`IMUSensor <isaacsim.sensors.physics.IMUSensor>` measures linear acceleration, angular velocity, and orientation through simulated inertial measurement.** The sensor provides three-axis linear acceleration, three-axis angular velocity, and quaternion orientation data from physics simulation. It supports configurable filtering for each measurement type using moving average filters to smooth sensor readings.

The sensor can be positioned and oriented relative to its parent body through local translation and rotation offsets. It operates at configurable sampling frequencies and can include or exclude gravity from acceleration measurements.

```python
# Create an IMU sensor with filtering
imu_sensor = sensors_physics.IMUSensor(
    prim_path="/World/Robot/IMU",
    frequency=100,
    linear_acceleration_filter_size=5,
    angular_velocity_filter_size=3,
    orientation_filter_size=2
)

# Get current inertial measurements
imu_data = imu_sensor.get_current_frame(read_gravity=False)
```

### {class}`EffortSensor <isaacsim.sensors.physics.EffortSensor>`

**{class}`EffortSensor <isaacsim.sensors.physics.EffortSensor>` monitors joint effort (force/torque) in articulated bodies during physics simulation.** The sensor measures the effort applied to specific joints in robotic systems and provides configurable sampling rates with data interpolation capabilities. It maintains internal buffers for temporal data processing and automatically manages data acquisition through physics simulation callbacks.

The sensor can operate at different frequencies than the physics step rate and provides interpolation between samples when needed. It supports both latest data retrieval and time-based interpolated values.

```python
# Create an effort sensor for a robot joint
effort_sensor = sensors_physics.EffortSensor(
    prim_path="/World/Robot/joint_1",
    sensor_period=0.01,  # 100 Hz sampling
    use_latest_data=False
)

# Get interpolated effort reading
reading = effort_sensor.get_sensor_reading()
print(f"Joint effort: {reading.value} at time {reading.time}")
```

## Functionality

### Sensor Creation Commands

The extension provides USD commands for programmatically creating sensor prims in the stage. {class}`IsaacSensorCreateContactSensor <isaacsim.sensors.physics.IsaacSensorCreateContactSensor>` creates contact sensors with collision detection capabilities, while {class}`IsaacSensorCreateImuSensor <isaacsim.sensors.physics.IsaacSensorCreateImuSensor>` creates IMU sensors with inertial measurement functionality. These commands handle prim creation, schema application, and initial configuration.

### Data Structures

{class}`EsSensorReading <isaacsim.sensors.physics.EsSensorReading>` encapsulates effort sensor measurement data including the measured value, timestamp, and validity status. The sensor classes use these data containers to manage buffered sensor readings and provide consistent interfaces for accessing measurement data across different sensor types.

## Integration

The extension integrates with **omni.physics** for physics simulation callbacks and **omni.timeline** for simulation timing control. It uses isaacsim.core.api for base sensor functionality and connects with the broader Isaac Sim ecosystem for robotic simulation workflows. The sensors automatically register with physics simulation events to collect data during simulation steps.
