// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <pxr/base/tf/token.h>

namespace isaacsim
{
namespace robot
{
namespace schema
{
namespace sensors
{

/** @brief Type name for the active Isaac contact sensor schema. */
inline const pxr::TfToken kIsaacContactSensorType{ "IsaacContactSensor" };
/** @brief Type name for the active Isaac IMU sensor schema. */
inline const pxr::TfToken kIsaacImuSensorType{ "IsaacImuSensor" };
/** @brief Type name for the active Isaac raycast sensor schema. */
inline const pxr::TfToken kIsaacRaycastSensorType{ "IsaacRaycastSensor" };

/** @brief Applied-schema name for the active RTX lidar sensor API. */
inline const pxr::TfToken kIsaacRtxLidarSensorAPI{ "IsaacRtxLidarSensorAPI" };
/** @brief Applied-schema name for the active RTX radar sensor API. */
inline const pxr::TfToken kIsaacRtxRadarSensorAPI{ "IsaacRtxRadarSensorAPI" };

/** @brief Shared sensor enabled attribute. */
inline const pxr::TfToken kEnabledAttr{ "enabled" };
/** @brief Shared sensor minimum-range attribute. */
inline const pxr::TfToken kMinRangeAttr{ "minRange" };
/** @brief Shared sensor maximum-range attribute. */
inline const pxr::TfToken kMaxRangeAttr{ "maxRange" };
/** @brief Ray-count attribute shared by raycast and legacy light-beam sensors. */
inline const pxr::TfToken kNumRaysAttr{ "numRays" };

/** @brief Contact sensor threshold attribute. */
inline const pxr::TfToken kThresholdAttr{ "threshold" };
/** @brief Contact sensor radius attribute. */
inline const pxr::TfToken kRadiusAttr{ "radius" };
/** @brief Contact sensor display-color attribute. */
inline const pxr::TfToken kColorAttr{ "color" };

/**
 * @brief Legacy PhysX sensor period attribute.
 * @deprecated Retained only for the deprecated @c isaacsim.sensors.physx extension.
 */
inline const pxr::TfToken kSensorPeriodAttr{ "sensorPeriod" };

/** @brief IMU linear-acceleration filter-width attribute. */
inline const pxr::TfToken kLinearAccelerationFilterWidthAttr{ "linearAccelerationFilterWidth" };
/** @brief IMU angular-velocity filter-width attribute. */
inline const pxr::TfToken kAngularVelocityFilterWidthAttr{ "angularVelocityFilterWidth" };
/** @brief IMU orientation filter-width attribute. */
inline const pxr::TfToken kOrientationFilterWidthAttr{ "orientationFilterWidth" };

/** @brief Raycast sensor ray-origins attribute. */
inline const pxr::TfToken kRayOriginsAttr{ "rayOrigins" };
/** @brief Raycast sensor ray-directions attribute. */
inline const pxr::TfToken kRayDirectionsAttr{ "rayDirections" };
/** @brief Raycast sensor per-ray time-offsets attribute. */
inline const pxr::TfToken kRayTimeOffsetsAttr{ "rayTimeOffsets" };
/** @brief Raycast sensor output-frame attribute. */
inline const pxr::TfToken kOutputFrameOfReferenceAttr{ "outputFrameOfReference" };
/** @brief Raycast sensor hit-prim-path reporting attribute. */
inline const pxr::TfToken kReportHitPrimPathsAttr{ "reportHitPrimPaths" };
/** @brief Sensor-relative output-frame token. */
inline const pxr::TfToken kOutputFrameSensor{ "SENSOR" };
/** @brief World-relative output-frame token. */
inline const pxr::TfToken kOutputFrameWorld{ "WORLD" };

/**
 * @brief Legacy lidar type name.
 * @deprecated Deprecated since 6.2.0 and retained only for @c isaacsim.sensors.physx.
 */
inline const pxr::TfToken kLidarType{ "Lidar" };
/**
 * @brief Legacy generic range-sensor type name.
 * @deprecated Deprecated since 6.2.0 and retained only for @c isaacsim.sensors.physx.
 */
inline const pxr::TfToken kGenericType{ "Generic" };
/**
 * @brief Legacy light-beam sensor type name.
 * @deprecated Deprecated since 6.2.0 and retained only for @c isaacsim.sensors.physx.
 */
inline const pxr::TfToken kIsaacLightBeamSensorType{ "IsaacLightBeamSensor" };

/** @brief Legacy range-sensor point-visualization attribute. @deprecated Use active sensor schemas. */
inline const pxr::TfToken kDrawPointsAttr{ "drawPoints" };
/** @brief Legacy range-sensor line-visualization attribute. @deprecated Use active sensor schemas. */
inline const pxr::TfToken kDrawLinesAttr{ "drawLines" };

/** @brief Legacy lidar horizontal field-of-view attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kHorizontalFovAttr{ "horizontalFov" };
/** @brief Legacy lidar vertical field-of-view attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kVerticalFovAttr{ "verticalFov" };
/** @brief Legacy lidar horizontal-resolution attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kHorizontalResolutionAttr{ "horizontalResolution" };
/** @brief Legacy lidar vertical-resolution attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kVerticalResolutionAttr{ "verticalResolution" };
/** @brief Legacy lidar rotation-rate attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kRotationRateAttr{ "rotationRate" };
/** @brief Legacy lidar high-level-of-detail attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kHighLodAttr{ "highLod" };
/** @brief Legacy lidar yaw-offset attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kYawOffsetAttr{ "yawOffset" };
/** @brief Legacy lidar semantics attribute. @deprecated Use an active lidar schema. */
inline const pxr::TfToken kEnableSemanticsAttr{ "enableSemantics" };

/** @brief Legacy generic sensor sampling-rate attribute. @deprecated Use an active sensor schema. */
inline const pxr::TfToken kSamplingRateAttr{ "samplingRate" };
/** @brief Legacy generic sensor streaming attribute. @deprecated Use an active sensor schema. */
inline const pxr::TfToken kStreamingAttr{ "streaming" };

/** @brief Legacy light-beam curtain-length attribute. @deprecated Use an active sensor schema. */
inline const pxr::TfToken kCurtainLengthAttr{ "curtainLength" };
/** @brief Legacy light-beam forward-axis attribute. @deprecated Use an active sensor schema. */
inline const pxr::TfToken kForwardAxisAttr{ "forwardAxis" };
/** @brief Legacy light-beam curtain-axis attribute. @deprecated Use an active sensor schema. */
inline const pxr::TfToken kCurtainAxisAttr{ "curtainAxis" };

} // namespace sensors
} // namespace schema
} // namespace robot
} // namespace isaacsim
