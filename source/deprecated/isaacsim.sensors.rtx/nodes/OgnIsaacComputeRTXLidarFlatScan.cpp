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

#include <pch/UsdPCH.h>
// clang-format on

#include "GenericModelOutput.h"
#include "LidarConfigHelper.h"
#include "OgnIsaacComputeRTXLidarFlatScanDatabase.h"
#include "isaacsim/core/includes/BaseResetNode.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <cstddef>
#include <math.h>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

class OgnIsaacComputeRTXLidarFlatScan : public isaacsim::core::includes::BaseResetNode
{
private:
    bool m_firstFrame{ true };
    bool m_isInitialized{ false };
    bool m_deprecationWarned{ false };

    float m_rotationRate{ 0.0f };
    float m_horizontalFov{ 0.0f };
    float m_horizontalResolution{ 0.0f };
    float m_azimuthRangeStart{ 0.0f };
    float m_azimuthRangeEnd{ 0.0f };
    float m_nearRangeM{ 0.0f };
    float m_farRangeM{ 0.0f };

public:
    void reset() override
    {
        m_firstFrame = true;
        m_isInitialized = false;
    }

    bool initialize(OgnIsaacComputeRTXLidarFlatScanDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] IsaacComputeRTXLidarFlatScan initialize");
        auto& state = db.perInstanceState<OgnIsaacComputeRTXLidarFlatScan>();

        const std::string renderProductPath = std::string(db.tokenToString(db.inputs.renderProductPath()));
        if (renderProductPath.length() == 0)
        {
            CARB_LOG_ERROR("IsaacComputeRTXLidarFlatScan: renderProductPath input is empty. Skipping execution.");
            return false;
        }
        pxr::UsdPrim lidarPrim = isaacsim::core::includes::getCameraPrimFromRenderProduct(renderProductPath);
        if (lidarPrim.IsA<pxr::UsdGeomCamera>())
        {
            CARB_LOG_WARN(
                "RTX sensors as camera prims are deprecated as of Isaac Sim 5.0, and support will be removed in a future release. Please use an OmniLidar prim with the new OmniSensorGenericLidarCoreAPI schema.");
            LidarConfigHelper configHelper;
            configHelper.updateLidarConfig(renderProductPath.c_str());
            if (configHelper.scanType == LidarScanType::kUnknown)
            {
                CARB_LOG_ERROR(
                    "IsaacComputeRTXLidarFlatScan: Lidar prim scanType is Unknown. Stop the simulation, correct the issue, and restart.");
                return false;
            }
            if (!configHelper.is2D)
            {
                CARB_LOG_ERROR(
                    "IsaacComputeRTXLidarFlatScan: Lidar prim is not a 2D Lidar. Stop the simulation, correct the issue, and restart.");
                return false;
            }
            if (configHelper.scanType == LidarScanType::kSolidState)
            {
                state.m_azimuthRangeStart = configHelper.azimuthStartDeg;
                state.m_azimuthRangeEnd = configHelper.azimuthEndDeg;
                state.m_horizontalFov = state.m_azimuthRangeStart - state.m_azimuthRangeEnd;
                state.m_horizontalResolution = state.m_horizontalFov / static_cast<float>(configHelper.numberOfEmitters);
                if (state.m_azimuthRangeEnd > 180.0f)
                {
                    state.m_azimuthRangeStart -= 180.0f;
                    state.m_azimuthRangeEnd -= 180.0f;
                }
            }
            else
            {
                state.m_horizontalResolution = 360.0f * configHelper.scanRateBaseHz / configHelper.reportRateBaseHz;
                state.m_azimuthRangeStart = -180.0f;
                state.m_azimuthRangeEnd = 180.0f;
                state.m_horizontalFov = 360.0f;
            }
            state.m_nearRangeM = configHelper.nearRangeM;
            state.m_farRangeM = configHelper.farRangeM;
            state.m_rotationRate = static_cast<float>(configHelper.scanRateBaseHz);
        }
        else
        {
            pxr::TfToken elementsCoordsType;
            if (lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:elementsCoordsType")).Get(&elementsCoordsType) &&
                elementsCoordsType != pxr::TfToken("SPHERICAL"))
            {
                CARB_LOG_ERROR(
                    "IsaacComputeRTXLidarFlatScan: Lidar prim elementsCoordsType is not set to SPHERICAL. Stop the simulation, correct the issue, and restart.");
                return false;
            }
            pxr::VtFloatArray elevationDeg;
            if (lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:emitterState:s001:elevationDeg")).Get(&elevationDeg))
            {
                const float epsilon = 1e-3f;
                for (const float elev : elevationDeg)
                {
                    if (::fabs(elev) > epsilon)
                    {
                        CARB_LOG_ERROR(
                            "IsaacComputeRTXLidarFlatScan: Lidar prim elevationDeg contains nonzero value %f.", elev);
                        CARB_LOG_ERROR(
                            "IsaacComputeRTXLidarFlatScan: Lidar prim is not a 2D Lidar. Stop the simulation, correct the issue, and restart.");
                        return false;
                    }
                }
            }
            uint32_t rotationRateAsInt;
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:scanRateBaseHz")).Get(&rotationRateAsInt);
            state.m_rotationRate = static_cast<float>(rotationRateAsInt);
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:nearRangeM")).Get(&state.m_nearRangeM);
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:farRangeM")).Get(&state.m_farRangeM);

            pxr::TfToken outputType;
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:scanType")).Get(&outputType);
            if (outputType == pxr::TfToken("SOLID_STATE"))
            {
                pxr::VtFloatArray azimuthDeg;
                lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:emitterState:s001:azimuthDeg")).Get(&azimuthDeg);

                state.m_azimuthRangeStart = *std::min_element(azimuthDeg.begin(), azimuthDeg.end());
                state.m_azimuthRangeEnd = *std::max_element(azimuthDeg.begin(), azimuthDeg.end());
                state.m_horizontalFov = state.m_azimuthRangeEnd - state.m_azimuthRangeStart;
                state.m_horizontalResolution = state.m_horizontalFov / static_cast<float>(azimuthDeg.size());
                if (state.m_azimuthRangeEnd > 180.0f)
                {
                    state.m_azimuthRangeStart -= 180.0f;
                    state.m_azimuthRangeEnd -= 180.0f;
                }
            }
            else
            {
                uint32_t reportRateBaseHzAsInt;
                lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:patternFiringRateHz")).Get(&reportRateBaseHzAsInt);
                float reportRateBaseHz = static_cast<float>(reportRateBaseHzAsInt);

                state.m_horizontalResolution = 360.0f * state.m_rotationRate / reportRateBaseHz;
                state.m_azimuthRangeStart = -180.0f;
                state.m_azimuthRangeEnd = 180.0f;
                state.m_horizontalFov = 360.0;
            }
        }
        return true;
    }

    static bool compute(OgnIsaacComputeRTXLidarFlatScanDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] IsaacComputeRTXLidarFlatScan compute");
        auto& state = db.perInstanceState<OgnIsaacComputeRTXLidarFlatScan>();
        db.outputs.exec() = kExecutionAttributeStateEnabled;

        if (!state.m_deprecationWarned)
        {
            CARB_LOG_WARN(
                "IsaacComputeRTXLidarFlatScan is deprecated and will be removed in a future release. "
                "Use the GenericModelOutput annotator directly to access full-scan RTX Lidar data.");
            state.m_deprecationWarned = true;
        }

        if (state.m_firstFrame)
        {
            state.m_isInitialized = state.initialize(db);
            state.m_firstFrame = false;
        }

        if (!state.m_isInitialized)
        {
            CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: Failed to initialize correctly. Skipping execution.");
            return false;
        }

        db.outputs.horizontalFov() = state.m_horizontalFov;
        db.outputs.horizontalResolution() = state.m_horizontalResolution;
        db.outputs.azimuthRange() = { state.m_azimuthRangeStart, state.m_azimuthRangeEnd - state.m_horizontalResolution };
        db.outputs.rotationRate() = state.m_rotationRate;
        db.outputs.depthRange() = { state.m_nearRangeM, state.m_farRangeM };
        db.outputs.numCols() = 0;
        db.outputs.numRows() = 1;

        const float* intensityData = nullptr;
        const float* distanceData = nullptr;
        const float* azimuthData = nullptr;
        size_t numInputElements = 0;

        // Primary path: read directly from GenericModelOutput via dataPtr
        if (db.inputs.dataPtr() != 0)
        {
            auto* gmo = omni::sensors::getModelOutputPtrFromBuffer(reinterpret_cast<void*>(db.inputs.dataPtr()));
            if (!gmo || gmo->numElements == 0)
            {
                CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: GMO buffer is empty or invalid. Skipping execution.");
                return false;
            }
            azimuthData = gmo->elements.x;
            distanceData = gmo->elements.z;
            intensityData = gmo->elements.scalar;
            numInputElements = static_cast<size_t>(gmo->numElements);
        }
        // Deprecated fallback: use individual pointer inputs (host data only)
        else if (db.inputs.intensityPtr() != 0 && db.inputs.distancePtr() != 0 && db.inputs.azimuthPtr() != 0)
        {
            if (db.inputs.intensityBufferSize() == 0 || db.inputs.distanceBufferSize() == 0 ||
                db.inputs.azimuthBufferSize() == 0)
            {
                CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: Buffer sizes are 0. Skipping execution.");
                return false;
            }

            if (db.inputs.intensityBufferSize() != db.inputs.distanceBufferSize() ||
                db.inputs.intensityBufferSize() != db.inputs.azimuthBufferSize())
            {
                CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: Buffer sizes are not equal. Skipping execution.");
                return false;
            }

            intensityData = reinterpret_cast<const float*>(db.inputs.intensityPtr());
            distanceData = reinterpret_cast<const float*>(db.inputs.distancePtr());
            azimuthData = reinterpret_cast<const float*>(db.inputs.azimuthPtr());
            numInputElements = db.inputs.intensityBufferSize() / sizeof(float);
        }
        else
        {
            CARB_LOG_INFO(
                "IsaacComputeRTXLidarFlatScan: No valid data input (dataPtr or individual pointers). Skipping execution.");
            return false;
        }

        size_t numOutputElements = static_cast<size_t>(state.m_horizontalFov / state.m_horizontalResolution);
        db.outputs.numCols() = static_cast<int>(numOutputElements);
        db.outputs.intensitiesData().resize(numOutputElements);
        db.outputs.linearDepthData().resize(numOutputElements);
        for (size_t i = 0; i < numOutputElements; i++)
        {
            db.outputs.linearDepthData()[i] = -1.0f;
            db.outputs.intensitiesData()[i] = 0;
        }

        for (size_t inIdx = 0; inIdx < numInputElements; inIdx++)
        {
            float azimuth = azimuthData[inIdx];
            float distance = distanceData[inIdx];
            uint8_t intensity = static_cast<uint8_t>(intensityData[inIdx] * 255.0f);
            size_t outIdx = static_cast<size_t>((azimuth - state.m_azimuthRangeStart) / state.m_horizontalResolution);
            if (outIdx >= numOutputElements)
            {
                CARB_LOG_INFO(
                    "IsaacComputeRTXLidarFlatScan: outIdx %zu is greater than numOutputElements %zu, clamping. azimuth: %f, distance: %f, intensity: %d, m_azimuthRangeStart: %f, m_horizontalResolution: %f",
                    outIdx, numOutputElements, azimuth, distance, intensity, state.m_azimuthRangeStart,
                    state.m_horizontalResolution);
                outIdx = numOutputElements - 1;
            }
            db.outputs.linearDepthData().at(outIdx) = distance;
            db.outputs.intensitiesData().at(outIdx) = intensity;
        }

        return true;
    }
};

REGISTER_OGN_NODE()
} // namespace rtx
} // namespace sensors
} // namespace isaacsim
