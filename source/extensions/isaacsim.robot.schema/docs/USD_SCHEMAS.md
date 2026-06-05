# USD Schemas

## rangeSensorSchema

> **Deprecated since 6.2.0**: The rangeSensorSchema (RangeSensor, Lidar, Generic) is deprecated.
> Used only by the deprecated `isaacsim.sensors.physx` extension.
> Use `IsaacRaycastSensor` from the `isaacSensorSchema` plugin with
> `isaacsim.sensors.experimental.physics` or `isaacsim.sensors.experimental.rtx` instead.

### RangeSensor

#### enabled
  Flag used to enable or disable this sensor

#### drawPoints
  Set to true to draw debug points on sensor hit locations

#### drawLines
  Set to true to draw debug lines representing sensor ray casts

#### minRange
  Minimum range for sensor to detect a hit

#### maxRange
  Maximum range for sensor to detect a hit


### Lidar

#### yawOffset
  Offset along yaw axis to account for sensor having a different forward direction

#### rotationRate
  Rotation rate of sensor in Hz, set to zero to make sensor fire all rays at once

#### highLod
  Enable High Lod for 3D Lidar sensor

#### horizontalFov
  Horizontal Field of View in degrees

#### verticalFov
  Vertical Field of View in degrees

#### horizontalResolution
  Degrees in between rays for horizontal axis

#### verticalResolution
  Degrees in between rays for vertical axis

#### enableSemantics
  Set to true to get semantic information of sensor hit locations


### Generic

#### samplingRate
  sampling rate of the custom sensor data in Hz

#### streaming
  Streaming lidar point data. Default to true


## isaacSensorSchema

### IsaacBaseSensor

#### enabled
  Set to True to enable this sensor, False to Disable


### IsaacRtxLidarSensorAPI


### IsaacRtxRadarSensorAPI


### IsaacContactSensor

#### threshold
  Min, Max force that is detected by this sensor, units in (kg) * (stage length unit) / (second)^2

#### radius
  Radius of the contact sensor, unit in stage length unit

#### color
  Color of the contact sensor sphere, R, G, B, A

#### sensorPeriod
  Time between each sensor measurement, unit in simulator seconds


### IsaacImuSensor

#### sensorPeriod
  Time between each sensor measurement, unit in simulator seconds

#### linearAccelerationFilterWidth
  Number of linear acceleration measurements used in the rolling average

#### angularVelocityFilterWidth
  Number of angular velocity measurements used in the rolling average

#### orientationFilterWidth
  Number of orientation measurements used in the rolling average


### IsaacLightBeamSensor

> **Deprecated since 6.2.0**: IsaacLightBeamSensor is deprecated.
> Used only by the deprecated `isaacsim.sensors.physx` extension.
> Use `IsaacRaycastSensor` with `isaacsim.sensors.experimental.physics` instead.

#### numRays
  Number of rays for the light curtain, default 1

#### curtainLength
  Total length of the light curtain

#### forwardAxis
  Direction to shoot the light beam in [AxisX, AxisY, AxisZ]

#### curtainAxis
  Direction to expand the light curtain in [AxisX, AxisY, AxisZ]

#### minRange
  Minimum range for sensor to detect a hit

#### maxRange
  Maximum range for sensor to detect a hit


### IsaacRaycastSensor

#### numRays
  Number of rays cast by this sensor (unsigned int, authoritative count)

#### minRange
  Minimum range for sensor to detect a hit, in stage length units

#### maxRange
  Maximum range for sensor to detect a hit, in stage length units

#### rayOrigins
  Per-ray origin translations in the sensor's local coordinate frame

#### rayDirections
  Per-ray cast direction vectors in the sensor's local coordinate frame

#### rayTimeOffsets
  Per-ray time offsets in seconds, relative to the current simulation time

#### outputFrameOfReference
  Coordinate frame for hit positions and hit normals ('SENSOR' or 'WORLD')

#### reportHitPrimPaths
  When true, the sensor reading includes the USD prim path of each hit surface


## robot_schema

### IsaacRobotAPI

#### isaac:changelog

#### isaac:description

#### isaac:license

#### isaac:namespace

#### isaac:robotType

#### isaac:source

#### isaac:version


### IsaacLinkAPI

#### isaac:nameOverride


### IsaacSiteAPI

#### isaac:Description

#### isaac:forwardAxis


### IsaacJointAPI

#### isaac:actuator

#### isaac:NameOverride

#### isaac:physics:DofOffsetOpOrder


### IsaacSurfaceGripper

#### isaac:coaxialForceLimit

#### isaac:maxGripDistance

#### isaac:retryInterval

#### isaac:shearForceLimit

#### isaac:status


### IsaacAttachmentPointAPI

#### isaac:clearanceOffset

#### isaac:forwardAxis

