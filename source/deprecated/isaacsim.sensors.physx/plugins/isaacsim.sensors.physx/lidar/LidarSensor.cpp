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

#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "LidarSensor.h"

#include <isaacsim/core/includes/Conversions.h>
#include <isaacsim/core/includes/Pose.h>
#include <omni/physx/IPhysx.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>

using namespace ::physx;
using namespace pxr;

namespace isaacsim
{
namespace sensors
{
namespace physx
{


LidarSensor::LidarSensor(omni::physx::IPhysx* physxPtr) : RangeSensorComponent(physxPtr), m_enableSemantics(false)
{
}

LidarSensor::~LidarSensor() = default;

void LidarSensor::onStart()
{
    m_tickedOnPhysicsStep = false;
    RangeSensorComponent::onStart();
}

void LidarSensor::onComponentChange()
{

    RangeSensorComponent::onComponentChange();

    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kHorizontalFovAttr), m_horizontalFov);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kVerticalFovAttr), m_verticalFov);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kHorizontalResolutionAttr), m_horizontalResolution);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kVerticalResolutionAttr), m_verticalResolution);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kRotationRateAttr), m_rotationRate);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kHighLodAttr), m_highLod);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kYawOffsetAttr), m_yawOffset);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kEnableSemanticsAttr), m_enableSemantics);

    // we have to have atleast one beam so the FOV can never be smaller than resolution
    m_horizontalResolution = pxr::GfClamp(m_horizontalResolution, 0.005f, 1024);
    m_horizontalFov = pxr::GfClamp(m_horizontalFov, m_horizontalResolution, 360);

    m_verticalResolution = pxr::GfClamp(m_verticalResolution, 0.005f, 1024);
    m_verticalFov = pxr::GfClamp(m_verticalFov, m_verticalResolution, 360);
    m_rotationRate = pxr::GfClamp(m_rotationRate, 0, 1024);
    m_minRange = pxr::GfClamp(m_minRange, 0, 1e9f);
    m_maxRange = pxr::GfClamp(m_maxRange, m_minRange, 1e9f);

    m_minDepth = m_minRange / m_metersPerUnit;
    m_maxDepth = m_maxRange / m_metersPerUnit;

    m_maxStepSize = static_cast<float>(1.0 / 30.0);

    m_cols = static_cast<int>(m_horizontalFov / m_horizontalResolution);

    // Add one so that we have symmetry
    // Otherwise we are missing one angle for the Velodyne 16 case as 30/2 = 15
    m_rows = m_highLod ? static_cast<int>(m_verticalFov / m_verticalResolution) + 1 : 1;

    if (m_rotationRate != 0.0f && m_rotationRate > 1.0 / m_maxStepSize)
    {
        m_rotationRate = static_cast<float>(1.0 / m_maxStepSize);
    }


    m_colScanSpeed = m_cols * m_rotationRate;
    m_maxColsPerTick = static_cast<int>(m_colScanSpeed * m_maxStepSize);

    m_depth.assign(m_rows * m_cols, 0);
    m_hitPos.assign(m_rows * m_cols, { 0, 0, 0 });
    m_linearDepth.assign(m_rows * m_cols, 0);
    m_beamTime.assign(m_rows * m_cols, 0);
    m_intensity.assign(m_rows * m_cols, 0);
    m_zenith.assign(m_rows, 0.0f);
    m_azimuth.assign(m_cols, 0.0f);

    float startAzimuth = -0.5f * m_horizontalFov + m_yawOffset;
    float startZenith = -0.5f * m_verticalFov;

    for (int col = 0; col < m_cols; col++)
    {
        m_azimuth[col] = static_cast<float>((startAzimuth + col * m_horizontalResolution) * M_PI / 180.0f);
    }

    for (int row = 0; row < m_rows; row++)
    {
        m_zenith[row] = static_cast<float>((startZenith + row * m_verticalResolution) * M_PI / 180.0f);
    }

    m_azimuthRange = { m_azimuth[0], m_azimuth[m_cols - 1] };
    m_zenithRange = { m_zenith[0], m_zenith[m_rows - 1] };

    // High LOD false means 2D lidar
    if (!m_highLod)
    {
        m_zenith[0] = 0.0f;
        m_zenithRange = { 0.0f, 0.0f };
    }

    m_lastAzimuth.assign(m_maxColsPerTick, 0.0f);
    m_lastDepth.assign(m_rows * m_maxColsPerTick, 0);
    m_lastLinearDepth.assign(m_rows * m_maxColsPerTick, 0);
    m_lastBeamTime.assign(m_rows * m_maxColsPerTick, 0);
    m_lastHitPos.assign(m_rows * m_cols, { 0, 0, 0 });
    m_lastPrimData.assign(m_rows * m_cols, std::string());
    m_lastCol = 0;
    m_lastNumColsTicked = 0;
    m_remainingTime = 0.0f;
    m_primData.assign(m_rows * m_cols, std::string());
}


void LidarSensor::dumpData(int start, int stop, double dt)
{

    // Size of m_lastDepth and m_lastIntensity == m_rows * m_lastNumColsTicked
    // Size of m_depth, and m_intensity == m_rows * m_cols
    // Size of m_azimuth == m_cols
    // Size of m_lastAzimuth == m_lastNumColsTicked

    int colsToTick = stop - start;
    int unwrappedSize = std::min(stop, m_cols) - start;
    int wrappedSize = std::max(0, stop - m_cols);

    m_lastDepth.resize(m_rows * colsToTick);
    m_lastHitPos.resize(m_rows * colsToTick);
    m_lastLinearDepth.resize(m_rows * colsToTick);
    m_lastBeamTime.resize(m_rows * colsToTick);
    m_lastIntensity.resize(m_rows * colsToTick);
    m_lastAzimuth.resize(colsToTick);
    m_lastPrimData.resize(m_rows * colsToTick);

    std::copy(m_azimuth.begin() + start, m_azimuth.begin() + (start + unwrappedSize), m_lastAzimuth.begin());
    std::copy(m_depth.begin() + start * m_rows, m_depth.begin() + (start + unwrappedSize) * m_rows, m_lastDepth.begin());
    std::copy(
        m_hitPos.begin() + start * m_rows, m_hitPos.begin() + (start + unwrappedSize) * m_rows, m_lastHitPos.begin());

    std::copy(m_beamTime.begin() + start * m_rows, m_beamTime.begin() + (start + unwrappedSize) * m_rows,
              m_lastBeamTime.begin());

    std::copy(m_linearDepth.begin() + start * m_rows, m_linearDepth.begin() + (start + unwrappedSize) * m_rows,
              m_lastLinearDepth.begin());

    std::copy(m_intensity.begin() + start * m_rows, m_intensity.begin() + (start + unwrappedSize) * m_rows,
              m_lastIntensity.begin());

    std::copy(m_primData.begin() + start * m_rows, m_primData.begin() + (start + unwrappedSize) * m_rows,
              m_lastPrimData.begin());

    // We wrapped around
    if (wrappedSize > 0)
    {
        std::copy(m_azimuth.begin(), m_azimuth.begin() + wrappedSize, m_lastAzimuth.begin() + unwrappedSize);
        std::copy(m_depth.begin(), m_depth.begin() + wrappedSize * m_rows, m_lastDepth.begin() + unwrappedSize * m_rows);
        std::copy(
            m_hitPos.begin(), m_hitPos.begin() + wrappedSize * m_rows, m_lastHitPos.begin() + unwrappedSize * m_rows);
        std::copy(m_beamTime.begin(), m_beamTime.begin() + wrappedSize * m_rows,
                  m_lastBeamTime.begin() + unwrappedSize * m_rows);
        std::copy(m_linearDepth.begin(), m_linearDepth.begin() + wrappedSize * m_rows,
                  m_lastLinearDepth.begin() + unwrappedSize * m_rows);
        std::copy(m_intensity.begin(), m_intensity.begin() + wrappedSize * m_rows,
                  m_lastIntensity.begin() + unwrappedSize * m_rows);
        std::copy(m_primData.begin(), m_primData.begin() + wrappedSize * m_rows,
                  m_lastPrimData.begin() + unwrappedSize * m_rows);
    }
}

void LidarSensor::preTick()
{
    auto worldMat = isaacsim::core::includes::pose::computeWorldXformNoCache(
        m_stage, m_usdrtStage, m_prim.GetPath(), m_parentPrimTimeCode);

    m_finalTranslation = isaacsim::core::includes::conversions::asPxVec3(worldMat.ExtractTranslation());
    m_finalRotation = isaacsim::core::includes::conversions::asPxQuat(worldMat.ExtractRotation());
}

void LidarSensor::onPhysicsStep()
{
    preTick();
    tick();
    m_tickedOnPhysicsStep = true;
}

void LidarSensor::tick()
{
    // Clear active semantic IDs each frame
    m_primData.assign(m_rows * m_cols, std::string());

    m_lineDrawing->clear();
    m_pointDrawing->clear();

    if (!m_pxScene)
    {
        CARB_LOG_ERROR("Physics Scene does not exist");
        return;
    }


    double elapsedTime = m_timeDelta;
    bool zUp = pxr::UsdGeomGetStageUpAxis(m_stage) == pxr::UsdGeomTokens->z;

    // Every tick does a full scan
    if (m_rotationRate == 0.0f)
    {
        m_lastNumColsTicked = m_cols;


        if (m_enableSemantics)
        {
            if (m_drawPoints && m_drawLines)
            {
                scan<true, true, true>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
            else if (m_drawPoints)
            {
                scan<true, false, true>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
            else if (m_drawLines)
            {
                scan<false, true, true>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
            else
            {
                scan<false, false, true>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
        }
        else
        {
            if (m_drawPoints && m_drawLines)
            {
                scan<true, true, false>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
            else if (m_drawPoints)
            {
                scan<true, false, false>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
            else if (m_drawLines)
            {
                scan<false, true, false>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
            else
            {
                scan<false, false, false>(0, m_cols, m_rows, m_cols, m_finalTranslation, m_finalRotation, zUp);
            }
        }

        if (m_firstFrame)
        {
            m_firstFrame = false;
        }
        else
        {
            dumpData(0, m_cols, elapsedTime);

            m_lastCol = 0;
        }
    }
    else
    {
        m_remainingTime += elapsedTime;
        m_lastNumColsTicked = static_cast<int>(m_colScanSpeed * m_remainingTime);

        // If too much time is remaining, cap the number of columns
        if (m_lastNumColsTicked > m_maxColsPerTick)
        {
            m_lastNumColsTicked = m_maxColsPerTick;
        }

        float simulateTime = m_lastNumColsTicked / m_colScanSpeed;
        m_remainingTime -= simulateTime;


        // In the case where we capped the number of columns, we drop from m_remainingTime
        // a multiple of m_maxStepSize
        m_remainingTime = std::fmod(m_remainingTime, m_maxStepSize);

        // Now scan the columns and dump the data
        if (m_enableSemantics)
        {
            if (m_drawPoints && m_drawLines)
            {
                scan<true, true, true>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols, m_finalTranslation,
                                       m_finalRotation, zUp);
            }
            else if (m_drawPoints)
            {
                scan<true, false, true>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols, m_finalTranslation,
                                        m_finalRotation, zUp);
            }
            else if (m_drawLines)
            {
                scan<false, true, true>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols, m_finalTranslation,
                                        m_finalRotation, zUp);
            }
            else
            {
                scan<false, false, true>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols, m_finalTranslation,
                                         m_finalRotation, zUp);
            }
        }
        else
        {
            if (m_drawPoints && m_drawLines)
            {
                scan<true, true, false>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols, m_finalTranslation,
                                        m_finalRotation, zUp);
            }
            else if (m_drawPoints)
            {
                scan<true, false, false>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols, m_finalTranslation,
                                         m_finalRotation, zUp);
            }
            else if (m_drawLines)
            {
                scan<false, true, false>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols, m_finalTranslation,
                                         m_finalRotation, zUp);
            }
            else
            {
                scan<false, false, false>(m_lastCol, m_lastCol + m_lastNumColsTicked, m_rows, m_cols,
                                          m_finalTranslation, m_finalRotation, zUp);
            }
        }

        if (m_firstFrame)
        {
            m_firstFrame = false;
        }
        else
        {
            dumpData(m_lastCol, m_lastCol + m_lastNumColsTicked, simulateTime);

            m_lastCol = (m_lastCol + m_lastNumColsTicked) % m_cols;
        }
    }
    m_sequenceNumber++;
}


}
}
}
