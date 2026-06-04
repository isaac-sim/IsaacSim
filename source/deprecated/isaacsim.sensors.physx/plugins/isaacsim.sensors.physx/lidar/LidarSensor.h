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

#include "../core/RangeSensorComponent.h"

#include <extensions/PxSceneQueryExt.h>
#include <isaacsim/core/includes/Color.h>
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
 * @class LidarSensor
 * @brief A LiDAR (Light Detection and Ranging) sensor implementation
 * @details This class simulates a LiDAR sensor with configurable parameters such as
 *          rotation rate, field of view, and resolution. It provides both depth and
 *          intensity data, along with optional semantic information about detected objects.
 *          The sensor performs ray casting in the physics scene to detect objects and
 *          measure distances.
 */
class LidarSensor : public RangeSensorComponent
{

public:
    /**
     * @brief Constructs a new LiDAR sensor instance
     * @param[in] physxPtr Pointer to the PhysX interface for physics simulation
     */
    explicit LidarSensor(omni::physx::IPhysx* physxPtr);

    /**
     * @brief Virtual destructor for proper cleanup
     */
    ~LidarSensor();

    /**
     * @brief Initializes the LiDAR sensor when the component starts
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
     * @details Executes the LiDAR scanning operation, updates sensor data, and processes results
     */
    virtual void tick();

    /**
     * @brief Updates LiDAR data during physics-only simulation steps
     */
    virtual void onPhysicsStep() override;

    /**
     * @brief Returns whether the LiDAR already simulated during the latest physics step
     * @return True when the stage-update path should draw only
     */
    virtual bool hasTickedOnPhysicsStep() const override
    {
        return m_tickedOnPhysicsStep;
    }

    /**
     * @brief Clears the marker used to prevent double simulation after physics-step scans
     */
    virtual void clearTickedOnPhysicsStep() override
    {
        m_tickedOnPhysicsStep = false;
    }

    /**
     * @brief Handles component property changes
     * @details Updates sensor configuration when properties are modified through the interface
     */
    virtual void onComponentChange();

    /**
     * @brief Gets the number of columns (horizontal samples) in the scan pattern
     * @return Number of columns in the scan grid
     */
    int getNumCols() const
    {
        return m_cols;
    }

    /**
     * @brief Gets the number of rows (vertical samples) in the scan pattern
     * @return Number of rows in the scan grid
     */
    int getNumRows() const
    {
        return m_rows;
    }

    /**
     * @brief Gets the number of columns processed in the last tick
     * @return Number of columns processed in the most recent update
     */
    int getNumColsTicked() const
    {
        return m_lastNumColsTicked;
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
     * @brief Gets the timestamp for each beam in the scan
     * @return Reference to vector containing beam timestamps in seconds
     */
    std::vector<float>& getBeamTimeData()
    {
        return m_lastBeamTime;
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
     * @brief Gets the primitive data for each hit point
     * @return Reference to vector containing primitive names/identifiers
     */
    std::vector<std::string>& getPrimData()
    {
        return m_lastPrimData;
    }

    /**
     * @brief Gets the azimuth angle range of the sensor
     * @return Pair of minimum and maximum azimuth angles in radians
     */
    carb::Float2 getAzimuthRange()
    {
        return m_azimuthRange;
    }

    /**
     * @brief Gets the zenith angle range of the sensor
     * @return Pair of minimum and maximum zenith angles in radians
     */
    carb::Float2 getZenithRange()
    {
        return m_zenithRange;
    }

private:
    /**
     * @brief Dumps sensor data for debugging purposes
     * @param[in] start Starting index in the scan pattern
     * @param[in] stop Ending index in the scan pattern
     * @param[in] elapsedTime Time elapsed during the scan in seconds
     */
    void dumpData(int start, int stop, double elapsedTime);

    /**
     * @brief Performs the LiDAR scanning operation
     * @tparam drawPoints Enable/disable point visualization
     * @tparam drawLines Enable/disable line visualization
     * @tparam enableSemantics Enable/disable semantic data collection
     * @param[in] start Starting column index for the scan
     * @param[in] stop Ending column index for the scan
     * @param[in] rows Number of vertical samples
     * @param[in] cols Number of horizontal samples
     * @param[in] origin Origin point of the sensor in world space
     * @param[in] worldRotation Rotation of the sensor in world space
     * @param[in] zUp Whether the world coordinate system is Z-up
     * @details Performs ray casting for each point in the scan pattern and processes hit results
     */
    template <bool drawPoints, bool drawLines, bool enableSemantics>
    void scan(int start,
              int stop,
              int rows,
              int cols,
              const ::physx::PxVec3& origin,
              const ::physx::PxQuat& worldRotation,
              bool zUp)
    {
        if (!m_pxScene)
        {
            return;
        }
        float invMaxDepth = 1.0f / m_maxDepth;
        // This isn't correct because the same prim (like carter) would have a different lidar axis if it was in a Y up
        // vs Z up stage. So commented this out and using the pure Z up rotation version
        // ::physx::PxVec3 azimuthDir = zUp ? ::physx::PxVec3(0.0f, 0.0f, 1.0f) : ::physx::PxVec3(0.0f, 1.0f, 0.0f);
        // ::physx::PxVec3 zenithDir = zUp ? ::physx::PxVec3(0.0f, 1.0f, 0.0f) : ::physx::PxVec3(0.0f, 0.0f, 1.0f);

        ::physx::PxVec3 azimuthDir = ::physx::PxVec3(0.0f, 0.0f, 1.0f);
        ::physx::PxVec3 zenithDir = ::physx::PxVec3(0.0f, 1.0f, 0.0f);

        auto lidarLambda = [&](int colPreMod)
        {
            int col = colPreMod % cols;
            ::physx::PxQuat mainrot = worldRotation * ::physx::PxQuat(m_azimuth[col], azimuthDir);

            for (int row = 0; row < rows; row++)
            {
                int i = row + colPreMod * rows % (rows * cols);

                // Time will be the same for all beams in this bucket - note beams are not interpolated over frame.
                m_beamTime[i] = static_cast<float>(m_timeSeconds);
                // Pitch then yaw
                ::physx::PxQuat rot = mainrot * ::physx::PxQuat(m_zenith[row], zenithDir);
                ::physx::PxVec3 unitDir = rot.rotate(::physx::PxVec3(1.0f, 0.0f, 0.0f)).getNormalized();
                ::physx::PxRaycastHit raycastHit;
                // Project the start point out to prevent collisions from origin

                const bool hit = ::physx::PxSceneQueryExt::raycastSingle(
                    *m_pxScene, origin + unitDir * m_minDepth, unitDir, m_maxDepth, m_hitFlags, raycastHit);

                if (hit)
                {
                    // the distance of the ray should be from center of lidar
                    m_depth[i] = static_cast<uint16_t>((raycastHit.distance + m_minDepth) * invMaxDepth * 65535.0f);
                    m_linearDepth[i] = (raycastHit.distance + m_minDepth) * m_metersPerUnit; // in meters
                    m_intensity[i] = 255;

                    carb::Float3 hitPos = { raycastHit.position.x, raycastHit.position.y, raycastHit.position.z };
                    ::physx::PxVec3 hitPosRel = worldRotation.rotateInv(raycastHit.position - origin);
                    m_hitPos[i] = { hitPosRel.x, hitPosRel.y, hitPosRel.z }; // relative to the sensor location
                    if (enableSemantics)
                    {
                        const char* hitActorName = raycastHit.actor->getName();
                        m_primData[i] = hitActorName;
                    }
                    if (drawPoints)
                    {
                        // set ratio for color.  should be zero at m_minDepth and unity at m_maxDepth
                        auto ratio = (m_linearDepth[i] - m_minDepth * m_metersPerUnit) /
                                     ((m_maxDepth - m_minDepth) * m_metersPerUnit);

                        m_pointDrawing->addVertex(hitPos, isaacsim::core::includes::color::distToRgba(ratio), 5.0);
                    }

                    if (drawLines)
                    {

                        ::physx::PxVec3 diff = raycastHit.position - origin;
                        auto temp = origin + diff.getNormalized() * m_minDepth;
                        // set ratio for color.  should be zero at m_minDepth and unity at m_maxDepth
                        auto ratio = (m_linearDepth[i] - m_minDepth * m_metersPerUnit) /
                                     ((m_maxDepth - m_minDepth) * m_metersPerUnit);
                        m_lineDrawing->addVertex(
                            { temp.x, temp.y, temp.z }, isaacsim::core::includes::color::distToRgba(ratio), 1.0);
                        m_lineDrawing->addVertex(hitPos, isaacsim::core::includes::color::distToRgba(ratio), 1.0);
                    }
                }
                else
                {
                    m_depth[i] = 65535;
                    m_linearDepth[i] = m_maxDepth * m_metersPerUnit; // in meters
                    m_intensity[i] = 0;
                    ::physx::PxVec3 hitPos = origin + unitDir * (m_maxDepth + m_minDepth);
                    ::physx::PxVec3 hitPosRel = worldRotation.rotateInv(hitPos - origin);
                    m_hitPos[i] = { hitPosRel.x, hitPosRel.y, hitPosRel.z };
                    if (drawLines)
                    {
                        auto temp = origin + unitDir * m_minDepth;

                        m_lineDrawing->addVertex({ temp.x, temp.y, temp.z }, { 1, 1, 1, 50.0f / 255.0f }, 1.0);
                        m_lineDrawing->addVertex({ hitPos.x, hitPos.y, hitPos.z }, { 1, 1, 1, 50.0f / 255.0f }, 1.0);
                    }
                }
            }
        };
        if (drawLines || drawPoints || enableSemantics)
        {
            for (int colPreMod = start; colPreMod < stop; colPreMod++)
            {
                lidarLambda(colPreMod);
            }
        }
        else
        {
            m_tasking->parallelFor(start, stop, lidarLambda);
        }
    }

    // From the prim
    float m_rotationRate = 20.0f;

    /**
     * @brief High level of detail flag for sensor operation
     */
    bool m_highLod = true;

    /**
     * @brief Horizontal field of view in degrees
     */
    float m_horizontalFov = 360.0f;

    /**
     * @brief Vertical field of view in degrees
     */
    float m_verticalFov = 30.0f;

    /**
     * @brief Horizontal angular resolution in degrees
     */
    float m_horizontalResolution = 0.4f;

    /**
     * @brief Vertical angular resolution in degrees
     */
    float m_verticalResolution = 4.0f;

    /**
     * @brief Yaw offset angle in degrees
     */
    float m_yawOffset = 0.0f;

    /**
     * @brief Minimum depth range in world units
     */
    float m_minDepth = 0;

    /**
     * @brief Maximum depth range in world units
     */
    float m_maxDepth = 1e8;

    /**
     * @brief Maximum step size for scanning
     */
    float m_maxStepSize = 0;

    /**
     * @brief Maximum number of columns to process per tick
     */
    int m_maxColsPerTick = 0;

    /**
     * @brief Last processed column index
     */
    int m_lastCol = 0;

    /**
     * @brief Scanning speed in columns per second
     */
    float m_colScanSpeed = 0;

    /**
     * @brief Remaining time for the current scan cycle
     */
    double m_remainingTime = 0;

    /**
     * @brief Number of vertical samples (rows)
     */
    int m_rows = 0;

    /**
     * @brief Number of horizontal samples (columns)
     */
    int m_cols = 0;

    /**
     * @brief Number of columns processed in the last tick
     */
    int m_lastNumColsTicked = 0;

    /**
     * @brief Vector of zenith angles for each row
     */
    std::vector<float> m_zenith;

    /**
     * @brief Current and last azimuth angles for each column
     */
    std::vector<float> m_azimuth, m_lastAzimuth;

    /**
     * @brief Range of azimuth angles (min, max) in radians
     */
    carb::Float2 m_azimuthRange;

    /**
     * @brief Range of zenith angles (min, max) in radians
     */
    carb::Float2 m_zenithRange;

    /**
     * @brief Current and last beam timestamps
     */
    std::vector<float> m_beamTime, m_lastBeamTime;

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

    /**
     * @brief Flag to enable/disable semantic data collection
     */
    bool m_enableSemantics;

    /**
     * @brief Current and last primitive data for semantic information
     */
    std::vector<std::string> m_primData, m_lastPrimData;

    /**
     * @brief Tracks whether the latest physics callback already simulated this LiDAR
     */
    bool m_tickedOnPhysicsStep = false;
};


}
}
}
