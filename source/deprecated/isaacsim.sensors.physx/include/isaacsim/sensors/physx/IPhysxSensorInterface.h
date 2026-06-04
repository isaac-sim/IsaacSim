// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/Defines.h>
#include <carb/Types.h>

#include <usdrt/gf/matrix.h>

namespace isaacsim
{
namespace sensors
{
namespace physx
{

using RangeSensorHandle = uint64_t;
constexpr RangeSensorHandle g_kInvalidHandle = static_cast<RangeSensorHandle>(0);

/**
 * @class LidarSensorInterface
 * @brief Interface for accessing LIDAR sensor data and properties.
 * @details
 * Provides methods to access various aspects of LIDAR sensor data including
 * depth measurements, beam timing, point clouds, and sensor configuration.
 */
struct LidarSensorInterface
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::physx::LidarSensorInterface", 0, 1);

    /**
     * @brief Gets the number of columns in the LIDAR scan pattern.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Number of columns in the scan pattern.
     */
    int(CARB_ABI* getNumCols)(const char* sensorPath);

    /**
     * @brief Gets the number of rows in the LIDAR scan pattern.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Number of rows in the scan pattern.
     */
    int(CARB_ABI* getNumRows)(const char* sensorPath);

    /**
     * @brief Gets the number of columns processed in the current tick.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Number of columns processed.
     */
    int(CARB_ABI* getNumColsTicked)(const char* sensorPath);

    /**
     * @brief Gets the raw depth data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the depth data buffer.
     */
    uint16_t*(CARB_ABI* getDepthData)(const char* sensorPath);

    /**
     * @brief Gets the beam timing data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the beam time data buffer.
     */
    float*(CARB_ABI* getBeamTimeData)(const char* sensorPath);

    /**
     * @brief Gets the linear depth data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the linear depth data buffer.
     */
    float*(CARB_ABI* getLinearDepthData)(const char* sensorPath);

    /**
     * @brief Gets the intensity data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the intensity data buffer.
     */
    uint8_t*(CARB_ABI* getIntensityData)(const char* sensorPath);

    /**
     * @brief Gets the zenith angle data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the zenith angle data buffer.
     */
    float*(CARB_ABI* getZenithData)(const char* sensorPath);

    /**
     * @brief Gets the azimuth angle data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the azimuth angle data buffer.
     */
    float*(CARB_ABI* getAzimuthData)(const char* sensorPath);

    /**
     * @brief Gets the point cloud data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the point cloud data buffer.
     */
    carb::Float3*(CARB_ABI* getPointCloud)(const char* sensorPath);

    /**
     * @brief Gets the primitive data for hit objects.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Vector of primitive paths that were hit.
     */
    std::vector<std::string>(CARB_ABI* getPrimData)(const char* sensorPath);

    /**
     * @brief Checks if a given path refers to a LIDAR sensor.
     * @param[in] sensorPath Path to check.
     * @return True if path refers to a LIDAR sensor, false otherwise.
     */
    bool(CARB_ABI* isLidarSensor)(const char* sensorPath);

    /**
     * @brief Gets the sequence number of the current scan.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Current sequence number.
     */
    uint64_t(CARB_ABI* getSequenceNumber)(const char* sensorPath);

    /**
     * @brief Gets the azimuth angle range of the sensor.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Azimuth range as (min, max) angles.
     */
    carb::Float2(CARB_ABI* getAzimuthRange)(const char* sensorPath);

    /**
     * @brief Gets the zenith angle range of the sensor.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Zenith range as (min, max) angles.
     */
    carb::Float2(CARB_ABI* getZenithRange)(const char* sensorPath);
};

/**
 * @class RadarSensorInterface
 * @brief Interface for accessing radar sensor functionality.
 * @details
 * Provides methods to interact with radar sensors in the simulation.
 */
struct RadarSensorInterface
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::physx::RadarSensorInterface", 0, 1);

    /**
     * @brief Checks if a given path refers to a radar sensor.
     * @param[in] sensorPath Path to check.
     * @return True if path refers to a radar sensor, false otherwise.
     */
    bool(CARB_ABI* isRadarSensor)(const char* sensorPath);
};

/**
 * @class GenericSensorInterface
 * @brief Interface for accessing generic range sensor functionality.
 * @details
 * Provides methods to interact with configurable range sensors that can be
 * customized for different scanning patterns and behaviors.
 */
struct GenericSensorInterface
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::physx::GenericSensorInterface", 0, 1);

    /**
     * @brief Checks if a given path refers to a generic sensor.
     * @param[in] sensorPath Path to check.
     * @return True if path refers to a generic sensor, false otherwise.
     */
    bool(CARB_ABI* isGenericSensor)(const char* sensorPath);

    /**
     * @brief Gets the number of samples processed in the current tick.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Number of samples processed.
     */
    int(CARB_ABI* getNumSamplesTicked)(const char* sensorPath);

    /**
     * @brief Gets the depth data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the depth data buffer.
     */
    uint16_t*(CARB_ABI* getDepthData)(const char* sensorPath);

    /**
     * @brief Gets the linear depth data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the linear depth data buffer.
     */
    float*(CARB_ABI* getLinearDepthData)(const char* sensorPath);

    /**
     * @brief Gets the intensity data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the intensity data buffer.
     */
    uint8_t*(CARB_ABI* getIntensityData)(const char* sensorPath);

    /**
     * @brief Gets the zenith angle data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the zenith angle data buffer.
     */
    float*(CARB_ABI* getZenithData)(const char* sensorPath);

    /**
     * @brief Gets the azimuth angle data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the azimuth angle data buffer.
     */
    float*(CARB_ABI* getAzimuthData)(const char* sensorPath);

    /**
     * @brief Gets the point cloud data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the point cloud data buffer.
     */
    carb::Float3*(CARB_ABI* getPointCloud)(const char* sensorPath);

    /**
     * @brief Gets the offset data buffer.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return Pointer to the offset data buffer.
     */
    carb::Float3*(CARB_ABI* getOffsetData)(const char* sensorPath);

    /**
     * @brief Sends the next batch of ray configurations to be processed.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @return True if batch was sent successfully, false otherwise.
     */
    bool(CARB_ABI* sendNextBatch)(const char* sensorPath);

    /**
     * @brief Sets the ray configuration for the next batch.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @param[in] azimuth_angles Array of azimuth angles for each ray.
     * @param[in] zenith_angles Array of zenith angles for each ray.
     * @param[in] sample_length Number of rays in the batch.
     */
    void(CARB_ABI* setNextBatchRays)(const char* sensorPath,
                                     const float* azimuthAngles,
                                     const float* zenithAngles,
                                     const int sampleLength);

    /**
     * @brief Sets the origin offsets for the next batch of rays.
     * @param[in] sensorPath Path to the sensor in the scene.
     * @param[in] origin_offsets Array of origin offsets for each ray.
     * @param[in] sample_length Number of rays in the batch.
     */
    void(CARB_ABI* setNextBatchOffsets)(const char* sensorPath, const float* originOffsets, const int sampleLength);
};

/**
 * @class LightBeamSensorInterface
 * @brief Interface for accessing light beam sensor functionality.
 * @details
 * Provides methods to interact with light beam sensors that can detect
 * intersections with objects in the scene.
 */
struct LightBeamSensorInterface
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::physx::LightBeamSensorInterface", 0, 1);

    /**
     * @brief Checks if a given path refers to a light beam sensor.
     * @param[in] usdPath Path to the sensor in the scene.
     * @return True if path refers to a light beam sensor, false otherwise.
     */
    bool(CARB_ABI* isLightBeamSensor)(const char* usdPath);
    /**
     * @brief Gets the linear depth data buffer.
     * @param[in] usdPath Path to the sensor in the scene.
     * @return Pointer to the linear depth data buffer.
     */
    float*(CARB_ABI* getLinearDepthData)(const char* usdPath);

    /**
     * @brief Gets the number of rays in the beam configuration.
     * @param[in] usdPath Path to the sensor in the scene.
     * @return Number of rays.
     */
    int(CARB_ABI* getNumRays)(const char* usdPath);

    /**
     * @brief Gets the beam hit data buffer.
     * @param[in] usdPath Path to the sensor in the scene.
     * @return Pointer to the beam hit data buffer.
     */
    uint8_t*(CARB_ABI* getBeamHitData)(const char* usdPath);

    /**
     * @brief Gets the hit position data buffer.
     * @param[in] usdPath Path to the sensor in the scene.
     * @return Pointer to the hit position data buffer.
     */
    carb::Float3*(CARB_ABI* getHitPosData)(const char* usdPath);

    /**
     * @brief Gets the transform data of the sensor.
     * @param[in] usdPath Path to the sensor in the scene.
     * @param[out] matrixOutput Transform matrix output.
     */
    void(CARB_ABI* getTransformData)(const char* usdPath, omni::math::linalg::matrix4d& matrixOutput);

    /**
     * @brief Gets the beam origin positions buffer.
     * @param[in] usdPath Path to the sensor in the scene.
     * @return Pointer to the beam origins buffer.
     */
    carb::Float3*(CARB_ABI* getBeamOrigins)(const char* usdPath);

    /**
     * @brief Gets the beam end points buffer.
     * @param[in] usdPath Path to the sensor in the scene.
     * @return Pointer to the beam end points buffer.
     */
    carb::Float3*(CARB_ABI* getBeamEndPoints)(const char* usdPath);
};

}
}
}
