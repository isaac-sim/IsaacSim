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

namespace isaacsim::robot::schema::sensors
{

// ── Active type names ──────────────────────────────────────────────────────────
inline const pxr::TfToken kIsaacContactSensorType{ "IsaacContactSensor" };
inline const pxr::TfToken kIsaacImuSensorType{ "IsaacImuSensor" };
inline const pxr::TfToken kIsaacRaycastSensorType{ "IsaacRaycastSensor" };

// Active API schema identifiers (for HasAPI / AddAppliedSchema)
inline const pxr::TfToken kIsaacRtxLidarSensorAPI{ "IsaacRtxLidarSensorAPI" };
inline const pxr::TfToken kIsaacRtxRadarSensorAPI{ "IsaacRtxRadarSensorAPI" };

// ── Active shared attributes ───────────────────────────────────────────────────
inline const pxr::TfToken kEnabledAttr{ "enabled" };
inline const pxr::TfToken kMinRangeAttr{ "minRange" };
inline const pxr::TfToken kMaxRangeAttr{ "maxRange" };
inline const pxr::TfToken kNumRaysAttr{ "numRays" }; // shared: IsaacRaycastSensor (uint) and IsaacLightBeamSensor (int)

// IsaacContactSensor attributes
inline const pxr::TfToken kThresholdAttr{ "threshold" };
inline const pxr::TfToken kRadiusAttr{ "radius" };
inline const pxr::TfToken kColorAttr{ "color" };

// DEPRECATED sensorPeriod attribute (only used by deprecated isaacsim.sensors.physx)
inline const pxr::TfToken kSensorPeriodAttr{ "sensorPeriod" };

// IsaacImuSensor attributes
inline const pxr::TfToken kLinearAccelerationFilterWidthAttr{ "linearAccelerationFilterWidth" };
inline const pxr::TfToken kAngularVelocityFilterWidthAttr{ "angularVelocityFilterWidth" };
inline const pxr::TfToken kOrientationFilterWidthAttr{ "orientationFilterWidth" };

// IsaacRaycastSensor attributes
inline const pxr::TfToken kRayOriginsAttr{ "rayOrigins" };
inline const pxr::TfToken kRayDirectionsAttr{ "rayDirections" };
inline const pxr::TfToken kRayTimeOffsetsAttr{ "rayTimeOffsets" };
inline const pxr::TfToken kOutputFrameOfReferenceAttr{ "outputFrameOfReference" };
inline const pxr::TfToken kReportHitPrimPathsAttr{ "reportHitPrimPaths" };
inline const pxr::TfToken kOutputFrameSensor{ "SENSOR" };
inline const pxr::TfToken kOutputFrameWorld{ "WORLD" };

// ── DEPRECATED type names (deprecated since 6.2.0) ────────────────────────────
// These types are only used by the deprecated isaacsim.sensors.physx extension.
// They will be removed in a future major release.
inline const pxr::TfToken kLidarType{ "Lidar" };
inline const pxr::TfToken kGenericType{ "Generic" };
inline const pxr::TfToken kIsaacLightBeamSensorType{ "IsaacLightBeamSensor" };

// DEPRECATED RangeSensor base attributes
inline const pxr::TfToken kDrawPointsAttr{ "drawPoints" };
inline const pxr::TfToken kDrawLinesAttr{ "drawLines" };

// DEPRECATED Lidar attributes
inline const pxr::TfToken kHorizontalFovAttr{ "horizontalFov" };
inline const pxr::TfToken kVerticalFovAttr{ "verticalFov" };
inline const pxr::TfToken kHorizontalResolutionAttr{ "horizontalResolution" };
inline const pxr::TfToken kVerticalResolutionAttr{ "verticalResolution" };
inline const pxr::TfToken kRotationRateAttr{ "rotationRate" };
inline const pxr::TfToken kHighLodAttr{ "highLod" };
inline const pxr::TfToken kYawOffsetAttr{ "yawOffset" };
inline const pxr::TfToken kEnableSemanticsAttr{ "enableSemantics" };

// DEPRECATED Generic attributes
inline const pxr::TfToken kSamplingRateAttr{ "samplingRate" };
inline const pxr::TfToken kStreamingAttr{ "streaming" };

// DEPRECATED IsaacLightBeamSensor attributes
inline const pxr::TfToken kCurtainLengthAttr{ "curtainLength" };
inline const pxr::TfToken kForwardAxisAttr{ "forwardAxis" };
inline const pxr::TfToken kCurtainAxisAttr{ "curtainAxis" };

} // namespace isaacsim::robot::schema::sensors
