// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "GenericModelOutput.h"
#include "LidarConfigHelper.h"
#include "isaacsim/core/includes/BaseResetNode.h"
#include "isaacsim/core/includes/Buffer.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <OgnIsaacCreateRTXLidarScanBufferDatabase.h>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

class OgnIsaacCreateRTXLidarScanBuffer : public isaacsim::core::includes::BaseResetNode
{
private:
    bool m_firstFrame{ true };
    bool m_isInitialized{ false };
    bool m_deprecationWarned{ false };

    size_t m_maxPoints{ 0 };

    bool m_outputAzimuth{ false };
    bool m_outputElevation{ false };
    bool m_outputDistance{ false };
    bool m_outputIntensity{ false };
    bool m_outputTimestamp{ false };
    bool m_outputEmitterId{ false };
    bool m_outputChannelId{ false };
    bool m_outputMaterialId{ false };
    bool m_outputTickId{ false };
    bool m_outputHitNormal{ false };
    bool m_outputVelocity{ false };
    bool m_outputObjectId{ false };
    bool m_outputEchoId{ false };
    bool m_outputTickState{ false };
    bool m_outputRadialVelocityMS{ false };

    isaacsim::core::includes::HostBufferBase<float3> h_pcBuffer;
    isaacsim::core::includes::HostBufferBase<uint64_t> h_timestampBuffer;

    omni::sensors::GenericModelOutput* hostGMO{ nullptr };
    omni::sensors::LidarAuxiliaryData* hostLidarAuxPoints{ nullptr };
    omni::sensors::RadarAuxiliaryData* hostRadarAuxPoints{ nullptr };

    static constexpr size_t MAX_POINTS_RADAR = 2500;

public:
    void reset()
    {
        m_firstFrame = true;
        m_isInitialized = false;
        m_maxPoints = 0;
        hostGMO = nullptr;
        hostLidarAuxPoints = nullptr;
        hostRadarAuxPoints = nullptr;
    }

    bool initialize(OgnIsaacCreateRTXLidarScanBufferDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] IsaacCreateRTXLidarScanBuffer initialize");

        hostGMO = omni::sensors::getModelOutputPtrFromBuffer(reinterpret_cast<void*>(db.inputs.dataPtr()));
        if (!hostGMO)
        {
            CARB_LOG_ERROR("IsaacCreateRTXLidarScanBuffer: Failed to parse GenericModelOutput from buffer.");
            return false;
        }
        if (hostGMO->numElements == 0)
        {
            CARB_LOG_INFO(
                "IsaacCreateRTXLidarScanBuffer: No returns in the input buffer for timestamp %lu. Skipping execution.",
                hostGMO->timestampNs);
            return false;
        }

        const auto auxType = hostGMO->auxType;
        const auto modality = hostGMO->modality;
        const std::string renderProductPath = std::string(db.tokenToString(db.inputs.renderProductPath()));
        const pxr::UsdPrim sensorPrim = isaacsim::core::includes::getCameraPrimFromRenderProduct(renderProductPath);
        const std::string sensorPrimPath = sensorPrim.GetPath().GetString();

        if (modality == omni::sensors::Modality::LIDAR)
        {
            if (auxType > omni::sensors::AuxType::NONE)
            {
                hostLidarAuxPoints = reinterpret_cast<omni::sensors::LidarAuxiliaryData*>(hostGMO->auxiliaryData);
            }

            if (renderProductPath.length() == 0)
            {
                CARB_LOG_ERROR("IsaacCreateRTXLidarScanBuffer: renderProductPath input is empty. Skipping execution.");
                return false;
            }

            if (sensorPrim.IsA<pxr::UsdGeomCamera>())
            {
                CARB_LOG_WARN(
                    "RTX sensors as camera prims are deprecated as of Isaac Sim 5.0, and support will be removed in a future release. Please use an OmniLidar prim with the new OmniSensorGenericLidarCoreAPI schema.");
                LidarConfigHelper configHelper;
                configHelper.updateLidarConfig(renderProductPath.c_str());
                if (configHelper.scanType == LidarScanType::kUnknown)
                {
                    CARB_LOG_ERROR(
                        "IsaacCreateRTXLidarScanBuffer: Lidar prim scanType is Unknown. Stop the simulation, correct the issue, and restart.");
                    return false;
                }
                m_maxPoints = configHelper.numChannels * configHelper.maxReturns *
                              static_cast<size_t>(std::ceil(static_cast<float>(configHelper.reportRateBaseHz) /
                                                            static_cast<float>(configHelper.scanRateBaseHz)));
            }
            else
            {
                uint32_t maxReturns = 0;
                uint32_t numChannels = 0;
                uint32_t patternFiringRateHz = 0;
                uint32_t scanRateBaseHz = 1;
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:maxReturns")).Get(&maxReturns);
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:numberOfChannels")).Get(&numChannels);
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:patternFiringRateHz")).Get(&patternFiringRateHz);
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:scanRateBaseHz")).Get(&scanRateBaseHz);
                m_maxPoints = numChannels * maxReturns *
                              static_cast<size_t>(std::ceil(static_cast<float>(patternFiringRateHz) /
                                                            static_cast<float>(scanRateBaseHz)));
            }
        }
        else if (modality == omni::sensors::Modality::RADAR)
        {
            if (auxType > omni::sensors::AuxType::NONE)
            {
                hostRadarAuxPoints = reinterpret_cast<omni::sensors::RadarAuxiliaryData*>(hostGMO->auxiliaryData);
            }
            m_maxPoints = MAX_POINTS_RADAR;
        }
        else
        {
            CARB_LOG_ERROR("IsaacCreateRTXLidarScanBuffer: Invalid modality: %d, expected LIDAR or RADAR",
                           static_cast<int>(modality));
            return false;
        }

        m_outputAzimuth = db.inputs.outputAzimuth();
        m_outputElevation = db.inputs.outputElevation();
        m_outputDistance = db.inputs.outputDistance();

        auto validateOutput = [&](bool inputEnabled, omni::sensors::AuxType requiredAuxType,
                                  omni::sensors::Modality requiredModality, omni::sensors::LidarAuxHas auxMember,
                                  const char* outputName) -> bool
        {
            if (!inputEnabled)
                return false;

            bool auxTypeValid = auxType >= requiredAuxType;
            bool modalityValid = modality == requiredModality;
            if (!auxTypeValid)
            {
                CARB_LOG_ERROR(
                    "IsaacCreateRTXLidarScanBuffer: %s requested for sensor '%s' but auxType (%d) is insufficient (requires %d). "
                    "Set the 'omni:sensor:Core:auxOutputType' attribute on the OmniLidar prim to 'BASIC' (for emitter/channel/echo/tick* IDs), "
                    "'EXTRA' (also for materialId/objectId) or 'FULL' (also for hit normals/velocity). The corresponding output buffer will be empty.",
                    outputName, sensorPrimPath.c_str(), static_cast<int>(auxType), static_cast<int>(requiredAuxType));
                return false;
            }

            if (!modalityValid)
            {
                CARB_LOG_INFO(
                    "IsaacCreateRTXLidarScanBuffer: %s requested for sensor '%s' but modality (%d) is incorrect (requires %d)",
                    outputName, sensorPrimPath.c_str(), static_cast<int>(modality), static_cast<int>(requiredModality));
                return false;
            }

            if (requiredModality == omni::sensors::Modality::LIDAR && auxType > omni::sensors::AuxType::NONE)
            {
                bool auxMemberFilled = (hostLidarAuxPoints->filledAuxMembers & auxMember) == auxMember;
                if (!auxMemberFilled)
                {
                    CARB_LOG_INFO(
                        "IsaacCreateRTXLidarScanBuffer: %s requested for sensor '%s' but auxMember (%d) is not filled.",
                        outputName, sensorPrimPath.c_str(), static_cast<int>(auxMember));
                    return false;
                }
            }

            return true;
        };

        m_outputIntensity = db.inputs.outputIntensity();
        m_outputTimestamp = db.inputs.outputTimestamp();
        m_outputEmitterId =
            validateOutput(db.inputs.outputEmitterId(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::EMITTER_ID, "outputEmitterId");
        m_outputChannelId =
            validateOutput(db.inputs.outputChannelId(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::CHANNEL_ID, "outputChannelId");
        m_outputMaterialId =
            validateOutput(db.inputs.outputMaterialId(), omni::sensors::AuxType::EXTRA, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::MAT_ID, "outputMaterialId");
        m_outputTickId =
            validateOutput(db.inputs.outputTickId(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::TICK_ID, "outputTickId");
        m_outputHitNormal =
            validateOutput(db.inputs.outputHitNormal(), omni::sensors::AuxType::FULL, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::HIT_NORMALS, "outputHitNormal");
        m_outputVelocity =
            validateOutput(db.inputs.outputVelocity(), omni::sensors::AuxType::FULL, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::VELOCITIES, "outputVelocity");
        m_outputObjectId =
            validateOutput(db.inputs.outputObjectId(), omni::sensors::AuxType::EXTRA, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::OBJ_ID, "outputObjectId");
        m_outputEchoId =
            validateOutput(db.inputs.outputEchoId(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::ECHO_ID, "outputEchoId");
        m_outputTickState =
            validateOutput(db.inputs.outputTickState(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
                           omni::sensors::LidarAuxHas::TICK_STATES, "outputTickState");
        m_outputRadialVelocityMS =
            validateOutput(db.inputs.outputRadialVelocityMS(), omni::sensors::AuxType::BASIC,
                           omni::sensors::Modality::RADAR, omni::sensors::LidarAuxHas::NONE, "outputRadialVelocityMS");

        h_pcBuffer.resize(m_maxPoints);
        if (m_outputTimestamp)
            h_timestampBuffer.resize(m_maxPoints);

        return true;
    }

    static void resetOutputs(OgnIsaacCreateRTXLidarScanBufferDatabase& db)
    {
        auto& matrixOutput = *reinterpret_cast<omni::math::linalg::matrix4d*>(&db.outputs.transform());
        matrixOutput.SetIdentity();
        db.outputs.dataPtr() = 0;
        db.outputs.bufferSize() = 0;
        db.outputs.width() = 0;
        db.outputs.height() = 1;
        db.outputs.azimuthPtr() = 0;
        db.outputs.azimuthBufferSize() = 0;
        db.outputs.elevationPtr() = 0;
        db.outputs.elevationBufferSize() = 0;
        db.outputs.distancePtr() = 0;
        db.outputs.distanceBufferSize() = 0;
        db.outputs.intensityPtr() = 0;
        db.outputs.intensityBufferSize() = 0;
        db.outputs.timestampPtr() = 0;
        db.outputs.timestampBufferSize() = 0;
        db.outputs.emitterIdPtr() = 0;
        db.outputs.emitterIdBufferSize() = 0;
        db.outputs.channelIdPtr() = 0;
        db.outputs.channelIdBufferSize() = 0;
        db.outputs.materialIdPtr() = 0;
        db.outputs.materialIdBufferSize() = 0;
        db.outputs.tickIdPtr() = 0;
        db.outputs.tickIdBufferSize() = 0;
        db.outputs.hitNormalPtr() = 0;
        db.outputs.hitNormalBufferSize() = 0;
        db.outputs.velocityPtr() = 0;
        db.outputs.velocityBufferSize() = 0;
        db.outputs.objectIdPtr() = 0;
        db.outputs.objectIdBufferSize() = 0;
        db.outputs.echoIdPtr() = 0;
        db.outputs.echoIdBufferSize() = 0;
        db.outputs.tickStatePtr() = 0;
        db.outputs.tickStateBufferSize() = 0;
        db.outputs.radialVelocityMSPtr() = 0;
        db.outputs.radialVelocityMSBufferSize() = 0;
    }

    static bool compute(OgnIsaacCreateRTXLidarScanBufferDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] Create RTX Lidar Scan Buffer");
        db.outputs.exec() = kExecutionAttributeStateEnabled;
        auto& state = db.perInstanceState<OgnIsaacCreateRTXLidarScanBuffer>();

        if (!state.m_deprecationWarned)
        {
            CARB_LOG_WARN(
                "IsaacCreateRTXLidarScanBuffer is deprecated and will be removed in a future release. "
                "Use the GenericModelOutput annotator directly to access full-scan RTX Lidar data.");
            state.m_deprecationWarned = true;
        }

        db.outputs.cudaDeviceIndex() = -1;

        if (state.m_firstFrame)
        {
            resetOutputs(db);
        }

        if (db.inputs.cudaDeviceIndex() != -1)
        {
            CARB_LOG_ERROR(
                "IsaacCreateRTXLidarScanBuffer: GPU data path is no longer supported. "
                "Set cudaDeviceIndex to -1 (host) or use GenericModelOutput directly.");
            return false;
        }

        if (!db.inputs.dataPtr())
        {
            CARB_LOG_WARN("IsaacCreateRTXLidarScanBuffer: dataPtr input is empty. Skipping execution.");
            return false;
        }

        if (state.m_firstFrame)
        {
            state.m_isInitialized = state.initialize(db);
            if (!state.m_isInitialized)
            {
                CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: Failed to initialize correctly. Skipping execution.");
                return false;
            }
            state.m_firstFrame = false;
        }

        // Parse GMO from host buffer
        state.hostGMO = omni::sensors::getModelOutputPtrFromBuffer(reinterpret_cast<void*>(db.inputs.dataPtr()));
        if (!state.hostGMO)
        {
            CARB_LOG_ERROR("IsaacCreateRTXLidarScanBuffer: Failed to parse GenericModelOutput from buffer.");
            return false;
        }

        const size_t numElements = static_cast<size_t>(state.hostGMO->numElements);
        if (numElements == 0)
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: No returns in the input buffer. Skipping execution.");
            return false;
        }

        const auto modality = state.hostGMO->modality;
        const auto auxType = state.hostGMO->auxType;

        // Rebase auxiliary data pointers
        if (auxType > omni::sensors::AuxType::NONE)
        {
            if (modality == omni::sensors::Modality::LIDAR)
                state.hostLidarAuxPoints =
                    reinterpret_cast<omni::sensors::LidarAuxiliaryData*>(state.hostGMO->auxiliaryData);
            else if (modality == omni::sensors::Modality::RADAR)
                state.hostRadarAuxPoints =
                    reinterpret_cast<omni::sensors::RadarAuxiliaryData*>(state.hostGMO->auxiliaryData);
        }

        // Ensure buffers are large enough
        if (numElements > state.h_pcBuffer.size())
        {
            state.h_pcBuffer.resize(numElements);
            if (state.m_outputTimestamp)
                state.h_timestampBuffer.resize(numElements);
        }

        // Compute Cartesian point cloud and timestamps in parallel
        {
            CARB_PROFILE_ZONE(0, "[IsaacSim] Host Path - Cartesian Conversion");
            auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

            const float* azData = state.hostGMO->elements.x;
            const float* elData = state.hostGMO->elements.y;
            const float* distData = state.hostGMO->elements.z;
            float3* pcOut = state.h_pcBuffer.data();
            const float degToRad = static_cast<float>(M_PI) / 180.0f;

            uint64_t* tsDst = state.m_outputTimestamp ? state.h_timestampBuffer.data() : nullptr;
            const int32_t* tsOffsets = state.m_outputTimestamp ? state.hostGMO->elements.timeOffsetNs : nullptr;
            const uint64_t tsBaseNs = state.hostGMO->timestampNs;

            constexpr float kMinDistance = 1e-6f;
            tasking->parallelFor(size_t(0), numElements,
                                 [=](size_t idx)
                                 {
                                     const float r = distData[idx];
                                     if (r < kMinDistance)
                                     {
                                         pcOut[idx] = make_float3(0.f, 0.f, 0.f);
                                     }
                                     else
                                     {
                                         const float az = azData[idx] * degToRad;
                                         const float el = elData[idx] * degToRad;
                                         const float rxy = r * cosf(el);
                                         pcOut[idx] = make_float3(rxy * cosf(az), rxy * sinf(az), r * sinf(el));
                                     }

                                     if (tsDst)
                                         tsDst[idx] = tsBaseNs + static_cast<uint64_t>(tsOffsets[idx]);
                                 });
        }

        // Set output point cloud
        auto& matrixOutput = *reinterpret_cast<omni::math::linalg::matrix4d*>(&db.outputs.transform());
        db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.h_pcBuffer.data());
        db.outputs.bufferSize() = numElements * sizeof(float3);
        db.outputs.width() = static_cast<uint32_t>(numElements);
        db.outputs.height() = 1;

        // Point output pointers directly to GMO data (no copy needed)
        if (state.m_outputAzimuth)
        {
            db.outputs.azimuthPtr() = reinterpret_cast<uint64_t>(state.hostGMO->elements.x);
            db.outputs.azimuthBufferSize() = numElements * sizeof(float);
        }
        if (state.m_outputElevation)
        {
            db.outputs.elevationPtr() = reinterpret_cast<uint64_t>(state.hostGMO->elements.y);
            db.outputs.elevationBufferSize() = numElements * sizeof(float);
        }
        if (state.m_outputDistance)
        {
            db.outputs.distancePtr() = reinterpret_cast<uint64_t>(state.hostGMO->elements.z);
            db.outputs.distanceBufferSize() = numElements * sizeof(float);
        }
        if (state.m_outputIntensity)
        {
            db.outputs.intensityPtr() = reinterpret_cast<uint64_t>(state.hostGMO->elements.scalar);
            db.outputs.intensityBufferSize() = numElements * sizeof(float);
        }
        if (state.m_outputTimestamp)
        {
            db.outputs.timestampPtr() = reinterpret_cast<uint64_t>(state.h_timestampBuffer.data());
            db.outputs.timestampBufferSize() = numElements * sizeof(uint64_t);
        }

        // Point auxiliary output pointers directly to GMO auxiliary data
        if (state.hostLidarAuxPoints)
        {
            if (state.m_outputEmitterId)
            {
                db.outputs.emitterIdPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->emitterId);
                db.outputs.emitterIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (state.m_outputChannelId)
            {
                db.outputs.channelIdPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->channelId);
                db.outputs.channelIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (state.m_outputMaterialId)
            {
                db.outputs.materialIdPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->matId);
                db.outputs.materialIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (state.m_outputTickId)
            {
                db.outputs.tickIdPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->tickId);
                db.outputs.tickIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (state.m_outputHitNormal)
            {
                db.outputs.hitNormalPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->hitNormals);
                db.outputs.hitNormalBufferSize() = numElements * sizeof(float3);
            }
            if (state.m_outputVelocity)
            {
                db.outputs.velocityPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->velocities);
                db.outputs.velocityBufferSize() = numElements * sizeof(float3);
            }
            if (state.m_outputObjectId)
            {
                db.outputs.objectIdPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->objId);
                db.outputs.objectIdBufferSize() = numElements * sizeof(uint8_t) * 16;
            }
            if (state.m_outputEchoId)
            {
                db.outputs.echoIdPtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->echoId);
                db.outputs.echoIdBufferSize() = numElements * sizeof(uint8_t);
            }
            if (state.m_outputTickState)
            {
                db.outputs.tickStatePtr() = reinterpret_cast<uint64_t>(state.hostLidarAuxPoints->tickStates);
                db.outputs.tickStateBufferSize() = numElements * sizeof(uint8_t);
            }
        }

        if (state.hostRadarAuxPoints)
        {
            if (state.m_outputRadialVelocityMS)
            {
                db.outputs.radialVelocityMSPtr() = reinterpret_cast<uint64_t>(state.hostRadarAuxPoints->rv_ms);
                db.outputs.radialVelocityMSBufferSize() = numElements * sizeof(float);
            }
        }

        auto frameEnd = state.hostGMO->frameEnd;
        getTransformFromSensorPose(frameEnd, matrixOutput);

        return true;
    }
};

REGISTER_OGN_NODE()
} // namespace rtx
} // namespace sensors
} // namespace isaacsim
