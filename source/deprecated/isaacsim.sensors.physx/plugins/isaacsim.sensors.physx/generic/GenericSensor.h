// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


#include "../core/RangeSensorComponent.h"

#include <extensions/PxSceneQueryExt.h>
#include <isaacsim/core/includes/Color.h>
#include <isaacsim/core/includes/Conversions.h>
#include <isaacsim/robot/schema/sensor_tokens.h>
#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <pxr/base/gf/vec3f.h>

#include <vector>

namespace isaacsim
{
namespace sensors
{
namespace physx
{

/**
 * @class GenericSensor
 * @brief A generic range sensor implementation that supports customizable scanning patterns
 * @details This sensor class provides a flexible range sensing capability with configurable
 *          azimuth and zenith angles, allowing for custom scanning patterns. It supports
 *          batch processing of sensor data and visualization options. The sensor can be
 *          configured to operate in streaming mode with adjustable sampling rates and
 *          batch sizes for efficient data processing.
 */
class GenericSensor : public RangeSensorComponent
{

public:
    /**
     * @brief Constructs a new Generic Sensor instance
     * @param[in] physxPtr Pointer to the PhysX interface for physics simulation
     */
    explicit GenericSensor(omni::physx::IPhysx* physxPtr);

    /**
     * @brief Virtual destructor for proper cleanup
     */
    ~GenericSensor();

    /**
     * @brief Initializes the generic sensor when the component starts
     * @details Sets up initial parameters, allocates memory for scan data, and prepares the sensor for operation
     */
    virtual void onStart();

    /**
     * @brief Performs pre-tick operations before the main sensor update
     * @details Updates sensor parameters and prepares for the next scan cycle
     */
    virtual void preTick();

    /**
     * @brief Performs the main sensor update during each tick
     * @details Executes the scanning operation and processes sensor data based on current configuration
     */
    virtual void tick();

    /**
     * @brief Handles component property changes
     * @details Updates sensor configuration when properties are modified through the interface
     */
    virtual void onComponentChange();

    /**
     * @brief Gets the number of samples processed in the last tick
     * @return Number of samples processed
     */
    int getNumSamplesTicked() const
    {
        return m_samplesPerTick;
    }

    /**
     * @brief Gets the latest depth data from the sensor
     * @return Reference to vector containing normalized depth values (0-65535)
     */
    std::vector<uint16_t>& getDepthData()
    {
        return m_lastDepth;
    }

    /**
     * @brief Gets the latest linear depth data in meters
     * @return Reference to vector containing depth values in meters
     */
    std::vector<float>& getLinearDepthData()
    {
        return m_lastLinearDepth;
    }

    /**
     * @brief Gets the latest intensity data from the sensor
     * @return Reference to vector containing intensity values (0-255)
     */
    std::vector<uint8_t>& getIntensityData()
    {
        return m_lastIntensity;
    }

    /**
     * @brief Gets the zenith angles for each sensor ray
     * @return Reference to vector containing zenith angles in radians
     */
    std::vector<float>& getZenithData()
    {
        return m_zenith;
    }

    /**
     * @brief Gets the azimuth angles for each sensor ray
     * @return Reference to vector containing azimuth angles in radians
     */
    std::vector<float>& getAzimuthData()
    {
        return m_lastAzimuth;
    }

    /**
     * @brief Gets the offset positions for each sensor ray
     * @return Reference to vector containing 3D offset positions
     */
    std::vector<carb::Float3>& getOffsetData()
    {
        return m_lastOffset;
    }

    /**
     * @brief Checks if the next batch of sensor pattern vectors should be sent
     * @return True if ready for next batch, false otherwise
     */
    bool sendNextBatch();

    /**
     * @brief Sets the next batch of sensor pattern angles
     * @param[in] azimuthAngles Array of azimuth angles in radians
     * @param[in] zenithAngles Array of zenith angles in radians
     * @param[in] sampleLength Number of samples in the batch
     */
    void setNextBatchRays(const float* azimuthAngles, const float* zenithAngles, const int sampleLength);

    /**
     * @brief Sets the origin offsets for each ray in the next batch
     * @param[in] originOffsets Array of 3D offset positions for each ray
     * @param[in] sampleLength Number of samples in the batch
     */
    void setNextBatchOffsets(const float* originOffsets, const int sampleLength);

private:
    /**
     * @brief Wraps the sensor data arrays at the specified starting index
     * @param[in] start Starting index for the wrap operation
     */
    void wrapData(int start);

    /**
     * @brief Dumps the current sensor data (for debugging purposes)
     */
    void dumpData();

    /**
     * @brief Performs a raycast with additional proximity checks
     * @param[in] pos Starting position of the ray
     * @param[in] dir Direction of the ray
     * @param[in] distance Maximum distance to check
     * @param[out] hit Hit result information
     * @param[in] physxScene PhysX scene to perform the raycast in
     * @return True if the ray hit something, false otherwise
     */
    bool raycastClose(const ::physx::PxVec3& pos,
                      const ::physx::PxVec3& dir,
                      float distance,
                      ::physx::PxRaycastHit& hit,
                      ::physx::PxScene* physxScene)
    {


        const bool ret = ::physx::PxSceneQueryExt::raycastSingle(*physxScene, pos, dir, distance, m_hitFlags, hit);
        return ret;
    }

    template <bool drawPoints, bool drawLines>
    void scan(const ::physx::PxVec3& sensorOrigin,
              const ::physx::PxQuat& worldRotation,
              ::physx::PxScene* physxScenePtr,
              std::vector<uint16_t>& depth,
              std::vector<carb::Float3>& hitPosition,
              std::vector<float>& linearDepth,
              std::vector<uint8_t>& intensity,
              std::vector<float>& zenith,
              std::vector<float>& azimuth,
              std::vector<carb::Float3>& originOffset,
              float maxDepth,
              float minDepth,
              float metersPerUnit,
              bool zUp)
    {
        if (!physxScenePtr)
        {
            return;
        }
        float invMaxDepth = 1.0f / maxDepth;
        // This isn't correct because the same prim (like carter) would have a different lidar axis if it was in a Y up
        // vs Z up stage. So commented this out and using the pure Z up rotation version
        // ::physx::PxVec3 azimuthDir = zUp ? ::physx::PxVec3(0.0f, 0.0f, 1.0f) : ::physx::PxVec3(0.0f, 1.0f, 0.0f);
        // ::physx::PxVec3 zenithDir = zUp ? ::physx::PxVec3(0.0f, 1.0f, 0.0f) : ::physx::PxVec3(0.0f, 0.0f, 1.0f);

        ::physx::PxVec3 azimuthDir = ::physx::PxVec3(0.0f, 0.0f, 1.0f);
        ::physx::PxVec3 zenithDir = ::physx::PxVec3(0.0f, 1.0f, 0.0f);

        size_t nScan = azimuth.size();
        for (size_t i = 0; i < nScan; i++)
        {
            // Pitch then yaw
            ::physx::PxQuat mainrot = worldRotation * ::physx::PxQuat(azimuth[i], azimuthDir);
            ::physx::PxQuat rot = mainrot * ::physx::PxQuat(zenith[i], zenithDir);
            ::physx::PxVec3 unitDir = rot.rotate(::physx::PxVec3(1.0f, 0.0f, 0.0f)).getNormalized();
            ::physx::PxRaycastHit raycastHit;
            // Project the start point out to prevent collisions from origin
            ::physx::PxVec3 origin = sensorOrigin + isaacsim::core::includes::conversions::asPxVec3(originOffset[i]);
            bool hit = raycastClose(origin + unitDir * minDepth, unitDir, maxDepth, raycastHit, physxScenePtr);

            if (hit)
            {
                // the distance of the ray should be from center of lidar
                depth[i] = static_cast<uint16_t>((raycastHit.distance + minDepth) * invMaxDepth * 65535.0f);
                linearDepth[i] = (raycastHit.distance + minDepth) * metersPerUnit; // in meters
                intensity[i] = 255;

                // if (linearDepth[i] < minDepth * metersPerUnit)
                // {
                //     depth[i] = 0;
                //     linearDepth[i] = minDepth * metersPerUnit; // in meters
                //     intensity[i] = 0;
                //     continue;
                // }
                carb::Float3 hitPos = { raycastHit.position.x, raycastHit.position.y, raycastHit.position.z };
                // ::physx::PxVec3 hitPosRelRay = worldRotation.rotateInv(raycastHit.position - origin);
                // hitPosRay[i] = { hitPosRelRay.x, hitPosRelRay.y, hitPosRelRay.z }; // relative to the ray's origin
                ::physx::PxVec3 hitPosRel = worldRotation.rotateInv(raycastHit.position - sensorOrigin);
                hitPosition[i] = { hitPosRel.x, hitPosRel.y, hitPosRel.z }; // relative to the sensor's origin, not
                                                                            // accounting for individual ray origin
                                                                            // offset
                if (drawPoints)
                {
                    // set ratio for color.  should be zero at minDepth and unity at maxDepth
                    auto ratio = (linearDepth[i] - minDepth * metersPerUnit) / ((maxDepth - minDepth) * metersPerUnit);
                    m_pointDrawing->addVertex(hitPos, isaacsim::core::includes::color::distToRgba(ratio), 5.0f);
                }

                // else
                if (drawLines)
                {
                    ::physx::PxVec3 diff = raycastHit.position - origin;
                    auto temp = origin + diff.getNormalized() * minDepth;
                    // set ratio for color.  should be zero at minDepth and unity at maxDepth
                    auto ratio = (linearDepth[i] - minDepth * metersPerUnit) / ((maxDepth - minDepth) * metersPerUnit);

                    m_lineDrawing->addVertex(
                        { temp.x, temp.y, temp.z }, isaacsim::core::includes::color::distToRgba(ratio), 1.0);
                    m_lineDrawing->addVertex(hitPos, isaacsim::core::includes::color::distToRgba(ratio), 1.0);
                }
            }
            else
            {
                depth[i] = 65535;
                linearDepth[i] = maxDepth * metersPerUnit; // in meters
                intensity[i] = 0;
                ::physx::PxVec3 hitPos = origin + unitDir * (maxDepth + minDepth);
                // ::physx::PxVec3 hitPosRelRay = worldRotation.rotateInv(hitPos - origin);
                // hitPosRay[i] = { hitPosRelRay.x, hitPosRelRay.y, hitPosRelRay.z }; // relative to the ray's origin
                ::physx::PxVec3 hitPosRel = worldRotation.rotateInv(hitPos - sensorOrigin);
                hitPosition[i] = { hitPosRel.x, hitPosRel.y, hitPosRel.z }; // relative to the sensor's origin, not
                                                                            // accounting for individual ray origin
                                                                            // offset
                if (drawLines)
                {

                    auto temp = origin + unitDir * minDepth;

                    m_lineDrawing->addVertex({ temp.x, temp.y, temp.z }, { 1, 1, 1, 50.0f / 255.0f }, 1.0);
                    m_lineDrawing->addVertex({ hitPos.x, hitPos.y, hitPos.z }, { 1, 1, 1, 50.0f / 255.0f }, 1.0);
                }
            }
        }
    }

    int m_samplingRate; // number of samples per second
    bool m_streaming;

    /**
     * @brief Total number of samples for each batch of data
     */
    int m_batchSize = 0;

    /**
     * @brief Minimum allowed batch size
     */
    int m_minBatchSize = 0;

    /**
     * @brief Length of the A buffer for double buffering
     */
    int m_lengthA = 0;

    /**
     * @brief Length of the B buffer for double buffering
     */
    int m_lengthB = 0;

    /**
     * @brief Index of the last processed sample
     */
    int m_lastSample = 0;

    /**
     * @brief Number of samples processed per tick
     */
    int m_samplesPerTick = 60;

    /**
     * @brief Maximum allowed samples per tick
     */
    int m_maxSamplesPerTick = 1000000;

    /**
     * @brief Minimum depth range in world units
     */
    float m_minDepth = 0;

    /**
     * @brief Maximum depth range in world units
     */
    float m_maxDepth = 1e8;

    /**
     * @brief Double-buffered azimuth angle arrays
     */
    std::vector<float> m_azimuthA{}, m_azimuthB{};

    /**
     * @brief Double-buffered zenith angle arrays
     */
    std::vector<float> m_zenithA{}, m_zenithB{};

    /**
     * @brief Double-buffered offset position arrays
     */
    std::vector<carb::Float3> m_offsetA{}, m_offsetB{};

    /**
     * @brief Pointers to active azimuth angle buffer
     */
    float *m_activeAzimuthPtr, *m_activeZenithPtr;

    /**
     * @brief Pointer to active offset position buffer
     */
    carb::Float3* m_activeOffsetPtr;

    /**
     * @brief Current and last zenith angles
     */
    std::vector<float> m_zenith, m_lastZenith;

    /**
     * @brief Current and last azimuth angles
     */
    std::vector<float> m_azimuth, m_lastAzimuth;

    /**
     * @brief Current and last offset positions
     */
    std::vector<carb::Float3> m_offset, m_lastOffset;

    /**
     * @brief Current and last linear depth measurements in meters
     */
    std::vector<float> m_linearDepth, m_lastLinearDepth;

    /**
     * @brief Current and last intensity measurements
     */
    std::vector<uint8_t> m_intensity, m_lastIntensity;

    /**
     * @brief Current and last normalized depth measurements
     */
    std::vector<uint16_t> m_depth, m_lastDepth;

    /**
     * @brief Hit positions in sensor local space
     */
    std::vector<carb::Float3> m_hitPos;

    /**
     * @brief PhysX ray cast hit flags configuration
     */
    const ::physx::PxHitFlags m_hitFlags = ::physx::PxHitFlag::eDEFAULT | ::physx::PxHitFlag::eMESH_BOTH_SIDES;

    /**
     * @brief Final translation of the sensor in world space
     */
    ::physx::PxVec3 m_finalTranslation;

    /**
     * @brief Final rotation of the sensor in world space
     */
    ::physx::PxQuat m_finalRotation;
};


}
}
}
