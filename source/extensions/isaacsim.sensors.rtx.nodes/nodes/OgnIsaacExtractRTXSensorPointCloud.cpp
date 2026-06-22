// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "isaacsim/core/includes/BaseResetNode.h"
#include "isaacsim/core/includes/Buffer.h"

#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <omni/math/linalg/matrix.h>
#include <omni/math/linalg/quat.h>
#include <omni/math/linalg/vec.h>

#include <OgnIsaacExtractRTXSensorPointCloudDatabase.h>
#include <cmath>
#include <cstdint>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{
namespace nodes
{

static void getTransformFromSensorPose(const omni::sensors::FrameAtTime& inPose,
                                       omni::math::linalg::matrix4d& matrixOutput)
{
    omni::math::linalg::vec3d posM{ inPose.posM[0], inPose.posM[1], inPose.posM[2] };
    omni::math::linalg::quatd quat{ inPose.orientation[3], inPose.orientation[0], inPose.orientation[1],
                                    inPose.orientation[2] };
    matrixOutput.SetIdentity();
    matrixOutput.SetRotateOnly(quat);
    matrixOutput.SetTranslateOnly(posM);
}

class OgnIsaacExtractRTXSensorPointCloud : public isaacsim::core::includes::BaseResetNode
{
private:
    isaacsim::core::includes::HostBufferBase<float3> m_pcBuffer;
    isaacsim::core::includes::HostBufferBase<uint64_t> m_timestampBuffer;

public:
    void reset()
    {
        m_pcBuffer.resize(0);
        m_timestampBuffer.resize(0);
    }

    static void resetOutputs(OgnIsaacExtractRTXSensorPointCloudDatabase& db)
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

    static bool compute(OgnIsaacExtractRTXSensorPointCloudDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] Extract RTX Sensor Point Cloud");
        db.outputs.exec() = kExecutionAttributeStateEnabled;
        db.outputs.cudaDeviceIndex() = -1;
        auto& state = db.perInstanceState<OgnIsaacExtractRTXSensorPointCloud>();

        if (!db.inputs.dataPtr())
        {
            resetOutputs(db);
            return false;
        }

        if (db.inputs.cudaDeviceIndex() != -1)
        {
            CARB_LOG_ERROR(
                "OgnIsaacExtractRTXSensorPointCloud: GPU data path is not supported. "
                "Set cudaDeviceIndex to -1 (host).");
            return false;
        }

        // Parse GMO from host buffer
        auto* gmo = omni::sensors::getModelOutputPtrFromBuffer(reinterpret_cast<void*>(db.inputs.dataPtr()));
        if (!gmo)
        {
            CARB_LOG_ERROR("OgnIsaacExtractRTXSensorPointCloud: Failed to parse GenericModelOutput from buffer.");
            return false;
        }

        const size_t numElements = static_cast<size_t>(gmo->numElements);
        if (numElements == 0)
        {
            resetOutputs(db);
            return false;
        }

        // Verify modality — only lidar and radar produce point clouds
        const auto modality = gmo->modality;
        if (modality != omni::sensors::Modality::LIDAR && modality != omni::sensors::Modality::RADAR)
        {
            CARB_LOG_ERROR(
                "OgnIsaacExtractRTXSensorPointCloud: Unsupported modality %d. "
                "Only LIDAR and RADAR sensors produce point clouds.",
                static_cast<int>(modality));
            return false;
        }

        const auto auxType = gmo->auxType;

        // Rebase auxiliary data pointers
        omni::sensors::LidarAuxiliaryData* lidarAux = nullptr;
        omni::sensors::RadarAuxiliaryData* radarAux = nullptr;
        if (auxType > omni::sensors::AuxType::NONE)
        {
            if (modality == omni::sensors::Modality::LIDAR)
                lidarAux = reinterpret_cast<omni::sensors::LidarAuxiliaryData*>(gmo->auxiliaryData);
            else if (modality == omni::sensors::Modality::RADAR)
                radarAux = reinterpret_cast<omni::sensors::RadarAuxiliaryData*>(gmo->auxiliaryData);
        }

        // Ensure buffers are large enough
        if (numElements > state.m_pcBuffer.size())
        {
            state.m_pcBuffer.resize(numElements);
            state.m_timestampBuffer.resize(numElements);
        }

        // ---------------------------------------------------------------
        // Point cloud extraction + timestamp computation.
        // SPHERICAL GMOs are converted to Cartesian; CARTESIAN GMOs are
        // copied directly to avoid a second coordinate transformation.
        // ---------------------------------------------------------------
        {
            CARB_PROFILE_ZONE(0, "[IsaacSim] Extract RTX Sensor Point Cloud");
            auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

            const float* xData = gmo->elements.x;
            const float* yData = gmo->elements.y;
            const float* zData = gmo->elements.z;
            float3* pcOut = state.m_pcBuffer.data();
            constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
            constexpr float kMinDistance = 1e-6f;

            uint64_t* tsDst = state.m_timestampBuffer.data();
            const int32_t* tsOffsets = gmo->elements.timeOffsetNs;
            const uint64_t tsBaseNs = gmo->timestampNs;

            const bool isSpherical = (gmo->elementsCoordsType == omni::sensors::CoordsType::SPHERICAL);

            tasking->parallelFor(size_t(0), numElements,
                                 [=](size_t idx)
                                 {
                                     if (isSpherical)
                                     {
                                         const float r = zData[idx];
                                         if (r < kMinDistance)
                                         {
                                             pcOut[idx] = make_float3(0.f, 0.f, 0.f);
                                         }
                                         else
                                         {
                                             const float az = xData[idx] * kDegToRad;
                                             const float el = yData[idx] * kDegToRad;
                                             const float rxy = r * cosf(el);
                                             pcOut[idx] = make_float3(rxy * cosf(az), rxy * sinf(az), r * sinf(el));
                                         }
                                     }
                                     else
                                     {
                                         pcOut[idx] = make_float3(xData[idx], yData[idx], zData[idx]);
                                     }

                                     tsDst[idx] = tsBaseNs + static_cast<uint64_t>(tsOffsets[idx]);
                                 });
        }

        // ---------------------------------------------------------------
        // Always-available outputs: point cloud, az, el, dist, intensity, timestamp
        // ---------------------------------------------------------------
        db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.m_pcBuffer.data());
        db.outputs.bufferSize() = numElements * sizeof(float3);
        db.outputs.width() = static_cast<uint32_t>(numElements);
        db.outputs.height() = 1;

        db.outputs.azimuthPtr() = reinterpret_cast<uint64_t>(gmo->elements.x);
        db.outputs.azimuthBufferSize() = numElements * sizeof(float);
        db.outputs.elevationPtr() = reinterpret_cast<uint64_t>(gmo->elements.y);
        db.outputs.elevationBufferSize() = numElements * sizeof(float);
        db.outputs.distancePtr() = reinterpret_cast<uint64_t>(gmo->elements.z);
        db.outputs.distanceBufferSize() = numElements * sizeof(float);
        db.outputs.intensityPtr() = reinterpret_cast<uint64_t>(gmo->elements.scalar);
        db.outputs.intensityBufferSize() = numElements * sizeof(float);
        db.outputs.timestampPtr() = reinterpret_cast<uint64_t>(state.m_timestampBuffer.data());
        db.outputs.timestampBufferSize() = numElements * sizeof(uint64_t);

        // ---------------------------------------------------------------
        // Lidar auxiliary outputs — wired based on what the GMO buffer provides
        // ---------------------------------------------------------------
        if (lidarAux)
        {
            auto has = [&](omni::sensors::LidarAuxHas member) -> bool
            { return (lidarAux->filledAuxMembers & member) == member; };

            if (has(omni::sensors::LidarAuxHas::EMITTER_ID))
            {
                db.outputs.emitterIdPtr() = reinterpret_cast<uint64_t>(lidarAux->emitterId);
                db.outputs.emitterIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (has(omni::sensors::LidarAuxHas::CHANNEL_ID))
            {
                db.outputs.channelIdPtr() = reinterpret_cast<uint64_t>(lidarAux->channelId);
                db.outputs.channelIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (has(omni::sensors::LidarAuxHas::MAT_ID))
            {
                db.outputs.materialIdPtr() = reinterpret_cast<uint64_t>(lidarAux->matId);
                db.outputs.materialIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (has(omni::sensors::LidarAuxHas::TICK_ID))
            {
                db.outputs.tickIdPtr() = reinterpret_cast<uint64_t>(lidarAux->tickId);
                db.outputs.tickIdBufferSize() = numElements * sizeof(uint32_t);
            }
            if (has(omni::sensors::LidarAuxHas::HIT_NORMALS))
            {
                db.outputs.hitNormalPtr() = reinterpret_cast<uint64_t>(lidarAux->hitNormals);
                db.outputs.hitNormalBufferSize() = numElements * sizeof(float3);
            }
            if (has(omni::sensors::LidarAuxHas::VELOCITIES))
            {
                db.outputs.velocityPtr() = reinterpret_cast<uint64_t>(lidarAux->velocities);
                db.outputs.velocityBufferSize() = numElements * sizeof(float3);
            }
            if (has(omni::sensors::LidarAuxHas::OBJ_ID))
            {
                db.outputs.objectIdPtr() = reinterpret_cast<uint64_t>(lidarAux->objId);
                db.outputs.objectIdBufferSize() = numElements * sizeof(uint8_t) * 16;
            }
            if (has(omni::sensors::LidarAuxHas::ECHO_ID))
            {
                db.outputs.echoIdPtr() = reinterpret_cast<uint64_t>(lidarAux->echoId);
                db.outputs.echoIdBufferSize() = numElements * sizeof(uint8_t);
            }
            if (has(omni::sensors::LidarAuxHas::TICK_STATES))
            {
                db.outputs.tickStatePtr() = reinterpret_cast<uint64_t>(lidarAux->tickStates);
                db.outputs.tickStateBufferSize() = numElements * sizeof(uint8_t);
            }
        }

        // ---------------------------------------------------------------
        // Radar auxiliary outputs
        // ---------------------------------------------------------------
        if (radarAux)
        {
            db.outputs.radialVelocityMSPtr() = reinterpret_cast<uint64_t>(radarAux->rv_ms);
            db.outputs.radialVelocityMSBufferSize() = numElements * sizeof(float);
        }

        // ---------------------------------------------------------------
        // Sensor-to-world transform
        // ---------------------------------------------------------------
        auto& matrixOutput = *reinterpret_cast<omni::math::linalg::matrix4d*>(&db.outputs.transform());
        getTransformFromSensorPose(gmo->frameEnd, matrixOutput);

        return true;
    }
};

REGISTER_OGN_NODE()
} // namespace nodes
} // namespace rtx
} // namespace sensors
} // namespace isaacsim
