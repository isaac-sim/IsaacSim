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

#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "GenericSensor.h"

#include <isaacsim/core/includes/Pose.h>
#include <omni/physx/IPhysx.h>

#include <chrono>
#include <iostream>
#include <string.h>

using namespace ::physx;
using namespace pxr;

namespace isaacsim
{
namespace sensors
{
namespace physx
{


GenericSensor::GenericSensor(omni::physx::IPhysx* physxPtr)
    : RangeSensorComponent(physxPtr),
      m_samplingRate(0),
      m_streaming(false),
      m_activeAzimuthPtr(nullptr),
      m_activeZenithPtr(nullptr),
      m_activeOffsetPtr(nullptr)
{
}

GenericSensor::~GenericSensor() = default;

void GenericSensor::onStart()
{
    RangeSensorComponent::onStart();
}

void GenericSensor::onComponentChange()
{

    RangeSensorComponent::onComponentChange();

    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kSamplingRateAttr), m_samplingRate);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kStreamingAttr), m_streaming);

    m_minRange = pxr::GfClamp(m_minRange, 0, 1e9f);
    m_maxRange = pxr::GfClamp(m_maxRange, m_minRange, 1e9f);
    m_minDepth = m_minRange / m_metersPerUnit;
    m_maxDepth = m_maxRange / m_metersPerUnit;


    // all of these will be resized to m_samplesPerTick;
    m_azimuth.assign(m_samplingRate, 0);
    m_zenith.assign(m_samplingRate, 0);
    m_offset.assign(m_samplingRate, { 0, 0, 0 });
    m_linearDepth.assign(m_samplingRate, 0);
    m_intensity.assign(m_samplingRate, 0);
    m_depth.assign(m_samplingRate, 0);
    m_hitPos.assign(m_samplingRate, { 0, 0, 0 });
}

bool GenericSensor::sendNextBatch()
{
    if (m_azimuthA.empty() || m_azimuthB.empty())
    {
        return true;
    }
    else
    {
        return false;
    }
}

void GenericSensor::setNextBatchRays(const float* azimuthAngles, const float* zenithAngles, const int sampleLength)
{
    // if not streaming
    // copy the first half of the data to batch_A, second half to batch_B
    // now neither batch are empty, and no more sentNextBatch should be set to True

    // if streaming
    // if both batches are empty, set A first, then B on the next tick.
    // the last batch that's set is always the backup batch, the pActiveBatch should point to the earlier set batch.
    // if one of the batches become empty during data wrapping, the next tick should fill the batch before it checks in
    // line 475 (so it shouldn't skip any ticks for scanning)

    if (!m_streaming)
    {
        m_lengthA = (sampleLength / 2);
        m_lengthB = sampleLength - m_lengthA;
        m_azimuthA.assign(m_lengthA, 0);
        m_zenithA.assign(m_lengthA, 0);
        m_azimuthB.assign(m_lengthB, 0);
        m_zenithB.assign(m_lengthB, 0);
        memcpy(&m_azimuthA.front(), azimuthAngles, m_lengthA * sizeof(float));
        memcpy(&m_zenithA.front(), zenithAngles, m_lengthA * sizeof(float));
        memcpy(&m_azimuthB.front(), azimuthAngles + m_lengthA, m_lengthB * sizeof(float));
        memcpy(&m_zenithB.front(), zenithAngles + m_lengthA, m_lengthB * sizeof(float));

        // set pointer to start with A
        m_activeAzimuthPtr = m_azimuthA.data();
        m_activeZenithPtr = m_zenithA.data();
        m_batchSize = m_lengthA;
    }
    else
    {
        if (m_azimuthA.empty())
        {
            m_azimuthA.assign(sampleLength, 0);
            m_zenithA.assign(sampleLength, 0);
            memcpy(&m_azimuthA.front(), azimuthAngles, sampleLength * sizeof(float));
            memcpy(&m_zenithA.front(), zenithAngles, sampleLength * sizeof(float));

            // the last batch that's set is always the backup batch, the pActiveBatch should point to the earlier set
            // batch.
            if (!m_azimuthB.empty())
            {
                m_activeAzimuthPtr = m_azimuthB.data();
                m_activeZenithPtr = m_zenithB.data();
            }
            else
            {
                CARB_LOG_WARN("need more data");
            }
        }
        else if (m_azimuthB.empty())
        {

            m_azimuthB.assign(sampleLength, 0);
            m_zenithB.assign(sampleLength, 0);
            memcpy(&m_azimuthB.front(), azimuthAngles, sampleLength * sizeof(float));
            memcpy(&m_zenithB.front(), zenithAngles, sampleLength * sizeof(float));

            // the last batch that's set is always the backup batch, the pActiveBatch should point to the earlier set
            // batch.
            if (!m_azimuthA.empty())
            {
                m_activeAzimuthPtr = m_azimuthA.data();
                m_activeZenithPtr = m_zenithA.data();
            }
            else
            {
                CARB_LOG_WARN("need more data ");
            }
        }
        else
        {
            CARB_LOG_WARN("new sensor pattern data not set. Only send new data when send_next_batch() returns true");
        }
        m_batchSize = sampleLength;
    }
}


void GenericSensor::setNextBatchOffsets(const float* originOffsets, const int sampleLength)
{
    if (!m_streaming)
    {
        int offsetMLengthA = (sampleLength / 2);
        int offsetMLengthB = sampleLength - offsetMLengthA;
        if ((offsetMLengthA != m_lengthA) || (offsetMLengthB != m_lengthB))
        {
            CARB_LOG_WARN("offset data size mismatch");
        }
        m_offsetA.assign(m_lengthA, { 0, 0, 0 });
        m_offsetB.assign(m_lengthB, { 0, 0, 0 });
        memcpy(&m_offsetA.front(), originOffsets, m_lengthA * sizeof(carb::Float3));
        memcpy(&m_offsetB.front(), originOffsets + m_lengthA, m_lengthB * sizeof(carb::Float3));
        m_activeOffsetPtr = m_offsetA.data();
    }
    else
    {
        if (sampleLength != m_batchSize)
        {
            CARB_LOG_WARN("offset data size mismatch");
        }

        if (m_offsetA.empty())
        {
            m_offsetA.assign(sampleLength, { 0, 0, 0 });
            memcpy(&m_offsetA.front(), originOffsets, sampleLength * sizeof(carb::Float3));
            if (!m_offsetB.empty())
            {
                m_activeOffsetPtr = m_offsetB.data();
            }
        }
        else if (m_offsetB.empty())
        {
            m_offsetB.assign(sampleLength, { 0, 0, 0 });
            memcpy(&m_offsetB.front(), originOffsets, sampleLength * sizeof(carb::Float3));
            if (!m_offsetA.empty())
            {
                m_activeOffsetPtr = m_offsetA.data();
            }
        }
        else
        {
            CARB_LOG_WARN("offset data not set. Only send new data when send_next_batch() returns true");
        }
    }
}


void GenericSensor::wrapData(int start)
{

    double currentFps = 1.0 / m_timeDelta;
    m_samplesPerTick = std::max(1, static_cast<int>(m_samplingRate / currentFps)); // scan at least once per tick
    if (m_batchSize < m_samplesPerTick)
    {
        CARB_LOG_ERROR(
            "Not enough data per sample batch to match sampling rate. Scanning faster than intended and could have missing data. Lower the sample rate or send more data per batch.");
    }
    else if (m_samplesPerTick > m_maxSamplesPerTick)
    {
        CARB_LOG_WARN("Sampling rate exceed specs. Scanning slower than intended.");
        m_samplesPerTick = m_maxSamplesPerTick;
    }

    m_azimuth.resize(m_samplesPerTick);
    m_zenith.resize(m_samplesPerTick);
    m_offset.resize(m_samplesPerTick);
    m_depth.resize(m_samplesPerTick);
    m_intensity.resize(m_samplesPerTick);
    m_hitPos.resize(m_samplesPerTick);
    m_linearDepth.resize(m_samplesPerTick);


    int stop = start + m_samplesPerTick;
    int ptCounter = 0;

    if (stop <= m_batchSize)
    {
        // get all the data from currentBatch
        for (int i = start; i < stop; i++)
        {
            m_azimuth[ptCounter] = m_activeAzimuthPtr[i];
            m_zenith[ptCounter] = m_activeZenithPtr[i];
            m_offset[ptCounter] = m_activeOffsetPtr[i];
            ptCounter++;
        }
        m_lastSample = stop;
    }
    else
    {
        for (int j = start; j < m_batchSize; j++)
        {
            m_azimuth[ptCounter] = m_activeAzimuthPtr[j];
            m_zenith[ptCounter] = m_activeZenithPtr[j];
            m_offset[ptCounter] = m_activeOffsetPtr[j];

            ptCounter++;
        }
        // switch batches
        if (m_activeAzimuthPtr == m_azimuthA.data())
        {
            m_activeAzimuthPtr = m_azimuthB.data();
            m_activeZenithPtr = m_zenithB.data();
            m_activeOffsetPtr = m_offsetB.data();

            // if streaming, empty out the batch that has just been read
            if (m_streaming)
            {
                m_azimuthA = {};
                m_zenithA = {};
                m_offsetA = {};
            }
            else
            {
                m_batchSize = m_lengthB;
            }
        }
        else if (m_activeAzimuthPtr == m_azimuthB.data())
        {
            m_activeAzimuthPtr = m_azimuthA.data();
            m_activeZenithPtr = m_zenithA.data();
            m_activeOffsetPtr = m_offsetA.data();

            // if streaming, empty out the batch that has just been read
            if (m_streaming)
            {
                m_azimuthB = {};
                m_zenithB = {};
                m_offsetB = {};
            }
            else
            {
                m_batchSize = m_lengthA;
            }
        }
        else
        {
            CARB_LOG_ERROR("something's wrong with batch switching");
            return;
        }

        for (int k = 0; k <= (stop % m_batchSize); k++)
        {
            m_azimuth[ptCounter] = m_activeAzimuthPtr[k];
            m_zenith[ptCounter] = m_activeZenithPtr[k];
            m_offset[ptCounter] = m_activeOffsetPtr[k];
            ptCounter++;
        }
        m_lastSample = stop % m_batchSize;
    }
}


void GenericSensor::dumpData()
{
    m_lastAzimuth = m_azimuth;
    m_lastZenith = m_zenith;
    m_lastOffset = m_offset;
    m_lastDepth = m_depth;
    m_lastLinearDepth = m_linearDepth;
    m_lastIntensity = m_intensity;
    m_lastHitPos = m_hitPos;
}


void GenericSensor::preTick()
{

    auto worldMat = isaacsim::core::includes::pose::computeWorldXformNoCache(
        m_stage, m_usdrtStage, m_prim.GetPath(), m_parentPrimTimeCode);

    m_finalTranslation = isaacsim::core::includes::conversions::asPxVec3(worldMat.ExtractTranslation());
    m_finalRotation = isaacsim::core::includes::conversions::asPxQuat(worldMat.ExtractRotation());
}

void GenericSensor::tick()
{
    m_lineDrawing->clear();
    m_pointDrawing->clear();

    if (!m_pxScene)
    {
        CARB_LOG_ERROR("Physics Scene does not exist");
        return;
    }

    bool zUp = pxr::UsdGeomGetStageUpAxis(m_stage) == pxr::UsdGeomTokens->z;

    // scanning
    // make sure both batches have data
    if (m_azimuthA.empty() || m_azimuthB.empty())
    {
        return; // and fetch more data one batch at a time until both are full
    }

    if (m_offsetA.empty() && m_offsetB.empty())
    {
        // if offset is not set, default to (0,0,0)
        m_offsetA.assign(m_batchSize, { 0, 0, 0 });
        m_offsetB.assign(m_batchSize, { 0, 0, 0 });
        m_activeOffsetPtr = m_offsetA.data();
    }

    if (m_activeAzimuthPtr != nullptr)
    {
        wrapData(m_lastSample);
        if (m_drawLines && m_drawPoints)
        {
            scan<true, true>(m_finalTranslation, m_finalRotation, m_pxScene, m_depth, m_hitPos, m_linearDepth,
                             m_intensity, m_zenith, m_azimuth, m_offset, m_maxDepth, m_minDepth, m_metersPerUnit, zUp);
        }
        else if (m_drawLines)
        {
            scan<false, true>(m_finalTranslation, m_finalRotation, m_pxScene, m_depth, m_hitPos, m_linearDepth,
                              m_intensity, m_zenith, m_azimuth, m_offset, m_maxDepth, m_minDepth, m_metersPerUnit, zUp);
        }
        else if (m_drawPoints)
        {
            scan<true, false>(m_finalTranslation, m_finalRotation, m_pxScene, m_depth, m_hitPos, m_linearDepth,
                              m_intensity, m_zenith, m_azimuth, m_offset, m_maxDepth, m_minDepth, m_metersPerUnit, zUp);
        }
        else
        {
            scan<false, false>(m_finalTranslation, m_finalRotation, m_pxScene, m_depth, m_hitPos, m_linearDepth,
                               m_intensity, m_zenith, m_azimuth, m_offset, m_maxDepth, m_minDepth, m_metersPerUnit, zUp);
        }
        dumpData();
    }
}


}
}
}
