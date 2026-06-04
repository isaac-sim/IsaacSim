# Overview

The isaacsim.robot.schema extension provides comprehensive USD schemas for robot modeling and sensor simulation in Isaac Sim. This extension defines formal USD schemas for range sensors, Isaac-specific sensors, and robot structural components, enabling standardized representation of robotic systems in USD stages. The extension is loaded early in the Kit startup sequence to ensure all robot-related schemas are available to other extensions.

<div align="center">

```{mermaid}
graph TD
    %% Inheritance relationships (deprecated)
    Generic["Generic (deprecated)"] --> RangeSensor["RangeSensor (deprecated)"]
    Lidar["Lidar (deprecated)"] --> RangeSensor
```

</div>

## Key Components

### Range Sensor Schemas (Deprecated)

> **Deprecated since 6.2.0**: The range sensor schemas are deprecated. Use `IsaacRaycastSensor` with `isaacsim.sensors.experimental.physics` or `isaacsim.sensors.experimental.rtx` instead.

The `**omni.isaac.RangeSensorSchema**` module provides schemas for distance-measuring sensors. The base {class}`RangeSensor <omni.isaac.RangeSensorSchema.RangeSensor>` class defines common attributes for range detection, visualization controls, and detection thresholds.

**{class}`Lidar <omni.isaac.RangeSensorSchema.Lidar>`** extends the range sensor with sophisticated scanning capabilities including field-of-view configuration, rotation rates, and semantic data collection. It supports both continuous scanning and single-shot operation modes.

**{class}`Generic <omni.isaac.RangeSensorSchema.Generic>`** offers a simplified range sensor implementation with basic sampling rate and streaming controls for custom sensor implementations.

### Isaac Sensor Schemas

The `**omni.isaac.IsaacSensorSchema**` module provides specialized sensors for robotics simulation. {class}`IsaacBaseSensor <omni.isaac.IsaacSensorSchema.IsaacBaseSensor>` serves as the foundation class with basic enable/disable functionality.

**{class}`IsaacContactSensor <omni.isaac.IsaacSensorSchema.IsaacContactSensor>`** detects contact forces with configurable thresholds and measurement periods, providing visual feedback through colored sphere representation.

**{class}`IsaacImuSensor <omni.isaac.IsaacSensorSchema.IsaacImuSensor>`** simulates inertial measurement units with configurable filter widths for linear acceleration, angular velocity, and orientation measurements.

**{class}`IsaacLightBeamSensor <omni.isaac.IsaacSensorSchema.IsaacLightBeamSensor>`** *(deprecated since 6.2.0)* creates light curtain sensors with multiple rays, configurable beam direction, and range detection capabilities for industrial automation applications. Use `IsaacRaycastSensor` with `isaacsim.sensors.experimental.physics` instead.

**{class}`IsaacRtxLidarSensorAPI <omni.isaac.IsaacSensorSchema.IsaacRtxLidarSensorAPI>`** and **{class}`IsaacRtxRadarSensorAPI <omni.isaac.IsaacSensorSchema.IsaacRtxRadarSensorAPI>`** provide API schema extensions for RTX-accelerated sensor implementations.

### Robot Structural Schemas

The `usd.schema.isaac` module defines the robot modeling framework through several interconnected schemas:

**IsaacRobotAPI** serves as the primary robot schema, managing metadata like description, version, and license information. It maintains ordered relationships for robot links and joints that define the kinematic structure.

**IsaacLinkAPI** and **IsaacJointAPI** provide specialized schemas for robot components with name overrides and configuration options. The joint API includes DOF offset ordering and actuator specifications.

**IsaacSiteAPI** defines reference points within the robot structure with directional information for attachment and measurement purposes.

**IsaacSurfaceGripper** implements specialized gripping mechanisms with force limits, attachment point management, and status tracking for object manipulation.

**IsaacAttachmentPointAPI** configures individual contact points within surface grippers with clearance offsets and directional constraints.

## Functionality

The extension provides utility functions for robot structure management and schema migration. `PopulateRobotSchemaFromArticulation` and `RecalculateRobotSchema` automatically discover and organize robot components from physics articulations. These functions traverse joint connections to build ordered link and joint relationships while applying appropriate API schemas.

The schema migration system handles deprecated schema updates, replacing obsolete APIs like `IsaacReferencePointAPI` with current equivalents. Joint migration specifically handles deprecated per-axis DOF offset attributes, converting them to the modern token array format.

Robot tree generation creates hierarchical representations of robot structures through `RobotLinkNode` objects, enabling kinematic analysis and visualization. The tree structure supports parent-child relationships and joint connectivity information.

## Integration

The extension loads early in the Kit startup sequence to ensure schema availability for other robot-related extensions. It integrates with USD's schema system through proper inheritance hierarchies, with range sensors inheriting from `UsdGeom.Xformable` and robot APIs extending `UsdAPISchemaBase`.

The schemas work together to create complete robot descriptions - robot APIs organize structural components, sensor schemas define sensing capabilities, and utility functions maintain consistency across the robot definition. This integrated approach enables comprehensive robot modeling from kinematic structure to sensor simulation within a unified USD framework.
