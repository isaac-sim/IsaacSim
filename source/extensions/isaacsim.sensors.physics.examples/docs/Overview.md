# Overview

The isaacsim.sensors.physics.examples extension provides interactive example demonstrations for Isaac Sim physics-based sensors, including contact sensors and IMU sensors. The extension creates dedicated UI windows that allow users to explore sensor functionality through hands-on scenarios and real-time visualization of sensor data.

```{image} ../../../../source/extensions/isaacsim.sensors.physics.examples/data/preview.png
---
align: center
---
```


## UI Components

### Contact Sensor Example Window

The contact sensor example provides a complete demonstration environment for understanding contact detection in physics simulations. Users can create scenarios that showcase how contact sensors work within Isaac Sim's physics system.

**Key features include:**
- Interactive UI controls for configuring contact sensor parameters
- Real-time visualization of sensor geometry and contact detection
- Dynamic scenario creation with sensor placement and offset configuration
- Color-coded visual feedback to highlight sensor activation states

### IMU Sensor Example Window

The IMU sensor example demonstrates inertial measurement unit functionality within Isaac Sim's physics environment. This example helps users understand how IMU sensors capture motion and orientation data from simulated objects.

**Key features include:**
- UI controls for IMU sensor configuration and testing
- Scenario generation for IMU sensor placement and calibration
- Real-time display of sensor readings and data interpretation

## Functionality

### Scenario Creation

Both sensor examples provide automated scenario creation capabilities that establish complete testing environments. The scenarios include appropriate physics objects, sensor placement, and environmental conditions needed to demonstrate sensor functionality effectively.

### Interactive Controls

Each example window includes UI controls that allow users to:
- Configure sensor parameters and properties
- Create and modify test scenarios
- Monitor sensor output and performance
- Visualize sensor data in real-time

### Visual Feedback

The extension incorporates visual elements to enhance the learning experience:
- Sensor geometry visualization for contact sensors
- Color-coded indicators for sensor states
- Real-time data display for sensor readings

## Integration

The extension integrates with the Isaac Sim examples browser system through isaacsim.examples.browser, making the sensor examples discoverable and accessible from the main examples interface. It utilizes isaacsim.sensors.experimental.physics for the underlying sensor implementations and physics interactions.
