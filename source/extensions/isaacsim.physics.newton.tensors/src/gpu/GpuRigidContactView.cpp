// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "GpuRigidContactView.h"

#include "CudaCommon.h"
#include "CudaKernels.h"
#include "utils/TensorOps.h"

#include <carb/logging/Log.h>

#include <cstring>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

GpuRigidContactView::GpuRigidContactView(py::object newtonStage,
                                         const std::vector<std::string>& sensorPaths,
                                         const std::vector<std::vector<std::string>>& filterPaths,
                                         uint32_t maxContactDataCount,
                                         int deviceOrdinal)
    : BaseRigidContactView(newtonStage, sensorPaths, filterPaths, maxContactDataCount), m_deviceOrdinal(deviceOrdinal)
{
    _uploadMappingsToGpu();
    _allocateFallbackBuffers();
}

GpuRigidContactView::~GpuRigidContactView()
{
    safeCudaFree(m_deviceBodySensorMap);
    safeCudaFree(m_deviceBodyFilterMap);
    safeCudaFree(m_deviceExtractedForce);
    safeCudaFree(m_deviceZeroVec3Buffer);
    safeCudaFree(m_deviceZeroFloatBuffer);
}

void GpuRigidContactView::_uploadMappingsToGpu()
{
    if (!validateCudaContext(m_deviceOrdinal))
        return;

    {
        size_t bytes = m_hostBodySensorMap.size() * sizeof(int);
        (void)cudaGetLastError();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceBodySensorMap), bytes) != cudaSuccess)
        {
            (void)cudaGetLastError();
            m_deviceBodySensorMap = nullptr;
            return;
        }
        if (cudaMemcpy(m_deviceBodySensorMap, m_hostBodySensorMap.data(), bytes, cudaMemcpyHostToDevice) != cudaSuccess)
        {
            (void)cudaGetLastError();
            safeCudaFree(m_deviceBodySensorMap);
            m_deviceBodySensorMap = nullptr;
            CARB_LOG_ERROR("GpuRigidContactView: failed to upload body-sensor map to device");
            return;
        }
    }
    {
        size_t bytes = m_hostBodyFilterMap.size() * sizeof(int);
        (void)cudaGetLastError();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceBodyFilterMap), bytes) != cudaSuccess)
        {
            (void)cudaGetLastError();
            m_deviceBodyFilterMap = nullptr;
            return;
        }
        if (cudaMemcpy(m_deviceBodyFilterMap, m_hostBodyFilterMap.data(), bytes, cudaMemcpyHostToDevice) != cudaSuccess)
        {
            (void)cudaGetLastError();
            safeCudaFree(m_deviceBodyFilterMap);
            m_deviceBodyFilterMap = nullptr;
            CARB_LOG_ERROR("GpuRigidContactView: failed to upload body-filter map to device");
            return;
        }
    }

    if (m_rigidContactMax > 0)
    {
        size_t bytes = m_rigidContactMax * 3 * sizeof(float);
        (void)cudaGetLastError();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceExtractedForce), bytes) != cudaSuccess)
        {
            (void)cudaGetLastError();
            m_deviceExtractedForce = nullptr;
        }
    }
}

void GpuRigidContactView::_allocateFallbackBuffers()
{
    if (m_rigidContactMax <= 0)
        return;
    if (!validateCudaContext(m_deviceOrdinal))
        return;

    size_t vec3Bytes = m_rigidContactMax * 3 * sizeof(float);
    size_t floatBytes = m_rigidContactMax * sizeof(float);
    (void)cudaGetLastError();
    if (cudaMalloc(reinterpret_cast<void**>(&m_deviceZeroVec3Buffer), vec3Bytes) == cudaSuccess)
    {
        cudaMemset(m_deviceZeroVec3Buffer, 0, vec3Bytes);
    }
    else
    {
        (void)cudaGetLastError();
        m_deviceZeroVec3Buffer = nullptr;
    }
    if (cudaMalloc(reinterpret_cast<void**>(&m_deviceZeroFloatBuffer), floatBytes) == cudaSuccess)
    {
        cudaMemset(m_deviceZeroFloatBuffer, 0, floatBytes);
    }
    else
    {
        (void)cudaGetLastError();
        m_deviceZeroFloatBuffer = nullptr;
    }
}

const float* GpuRigidContactView::_resolveContactForce() const
{
    if (!m_hasSpatialForce)
        return m_cachedContactForce;
    if (!m_cachedSpatialForce)
        return nullptr;
    if (m_deviceExtractedForce)
        launchExtractVec3FromSpatial(m_cachedSpatialForce, m_deviceExtractedForce, m_rigidContactMax);
    return m_deviceExtractedForce;
}

const float* GpuRigidContactView::_getContactPoint0() const
{
    return m_cachedContactPoint0 ? m_cachedContactPoint0 : m_deviceZeroVec3Buffer;
}
const float* GpuRigidContactView::_getContactPoint1() const
{
    return m_cachedContactPoint1 ? m_cachedContactPoint1 : m_deviceZeroVec3Buffer;
}
const float* GpuRigidContactView::_getThickness0() const
{
    return m_cachedThickness0 ? m_cachedThickness0 : m_deviceZeroFloatBuffer;
}
const float* GpuRigidContactView::_getThickness1() const
{
    return m_cachedThickness1 ? m_cachedThickness1 : m_deviceZeroFloatBuffer;
}

bool GpuRigidContactView::getNetContactForces(const TensorDesc* dstTensor, float dt) const
{
    if (!dstTensor || !dstTensor->data || m_sensorCount == 0)
        return false;

    if (!checkTensorDevice(*dstTensor, m_deviceOrdinal, "net contact forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "net contact forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, m_sensorCount * 3u, "net contact forces", __FUNCTION__))
    {
        return false;
    }

    _refreshContactPointers();
    size_t outBytes = m_sensorCount * 3 * sizeof(float);

    const float* force = _resolveContactForce();
    if (!force || m_rigidContactMax <= 0)
    {
        cudaMemset(dstTensor->data, 0, outBytes);
        return true;
    }

    float dtScale = _getPhysicsDtScale(dt);
    cudaMemset(dstTensor->data, 0, outBytes);
    bool ok = launchNetContactForces(m_cachedContactCount, m_cachedShape0, m_cachedShape1, force, m_cachedShapeBody,
                                     m_deviceBodySensorMap, m_bodyCount, m_worldBodyIndex, dtScale,
                                     static_cast<float*>(dstTensor->data), m_rigidContactMax);
    cudaDeviceSynchronize();
    return ok;
}

bool GpuRigidContactView::getContactForceMatrix(const TensorDesc* dstTensor, float dt) const
{
    if (!dstTensor || !dstTensor->data || m_sensorCount == 0 || m_filterCount == 0)
        return false;

    if (!checkTensorDevice(*dstTensor, m_deviceOrdinal, "contact force matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "contact force matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, m_sensorCount * m_filterCount * 3u, "contact force matrix", __FUNCTION__))
    {
        return false;
    }

    _refreshContactPointers();
    size_t outBytes = m_sensorCount * m_filterCount * 3 * sizeof(float);

    const float* force = _resolveContactForce();
    if (!force || m_rigidContactMax <= 0)
    {
        cudaMemset(dstTensor->data, 0, outBytes);
        return true;
    }

    float dtScale = _getPhysicsDtScale(dt);
    cudaMemset(dstTensor->data, 0, outBytes);
    return launchContactForceMatrix(m_cachedContactCount, m_cachedShape0, m_cachedShape1, force, m_cachedShapeBody,
                                    m_deviceBodySensorMap, m_bodyCount, m_deviceBodyFilterMap, m_bodyCount,
                                    m_worldBodyIndex, dtScale, m_filterCount, static_cast<float*>(dstTensor->data),
                                    m_rigidContactMax);
}

bool GpuRigidContactView::getContactData(const TensorDesc* contactForceTensor,
                                         const TensorDesc* contactPointTensor,
                                         const TensorDesc* contactNormalTensor,
                                         const TensorDesc* contactSeparationTensor,
                                         const TensorDesc* contactCountTensor,
                                         const TensorDesc* contactStartIndicesTensor,
                                         float dt) const
{
    if (!contactForceTensor || !contactPointTensor || !contactNormalTensor || !contactSeparationTensor ||
        !contactCountTensor || !contactStartIndicesTensor)
        return false;
    if (!contactForceTensor->data || !contactPointTensor->data || !contactNormalTensor->data ||
        !contactSeparationTensor->data || !contactCountTensor->data || !contactStartIndicesTensor->data)
        return false;
    if (m_sensorCount == 0 || m_filterCount == 0 || m_rigidContactMax <= 0)
        return false;

    const uint32_t maxCount = m_maxContactDataCount;
    const uint32_t pairCount = m_sensorCount * m_filterCount;

    if (!checkTensorDevice(*contactForceTensor, m_deviceOrdinal, "contact force", __FUNCTION__) ||
        !checkTensorFloat32(*contactForceTensor, "contact force", __FUNCTION__) ||
        !checkTensorSizeExact(*contactForceTensor, maxCount, "contact force", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactPointTensor, m_deviceOrdinal, "contact point", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "contact point", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, maxCount * 3u, "contact point", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactNormalTensor, m_deviceOrdinal, "contact normal", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "contact normal", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor, maxCount * 3u, "contact normal", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactSeparationTensor, m_deviceOrdinal, "contact separation", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "contact separation", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, maxCount, "contact separation", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactCountTensor, m_deviceOrdinal, "contact count", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "contact count", __FUNCTION__) ||
        !checkTensorSizeExact(*contactCountTensor, pairCount, "contact count", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactStartIndicesTensor, m_deviceOrdinal, "contact start indices", __FUNCTION__) ||
        !checkTensorInt32(*contactStartIndicesTensor, "contact start indices", __FUNCTION__) ||
        !checkTensorSizeExact(*contactStartIndicesTensor, pairCount, "contact start indices", __FUNCTION__))
    {
        return false;
    }

    _refreshContactPointers();
    const float* force = _resolveContactForce();
    if (!force)
        return false;
    float dtScale = _getPhysicsDtScale(dt);
    size_t countBytes = pairCount * sizeof(uint32_t);
    auto* outCounts = static_cast<uint32_t*>(contactCountTensor->data);
    auto* outStartIdx = static_cast<uint32_t*>(contactStartIndicesTensor->data);

    cudaMemset(outCounts, 0, countBytes);
    launchCountContactsPerPair(m_cachedContactCount, m_cachedShape0, m_cachedShape1, m_cachedShapeBody,
                               m_deviceBodySensorMap, m_bodyCount, m_deviceBodyFilterMap, m_bodyCount,
                               static_cast<int>(m_filterCount), m_worldBodyIndex, outCounts, m_rigidContactMax);
    cudaDeviceSynchronize();

    cudaMemcpy(m_scratchCounts.data(), outCounts, countBytes, cudaMemcpyDeviceToHost);

    m_scratchStartIndices[0] = 0;
    for (uint32_t i = 1; i < pairCount; ++i)
        m_scratchStartIndices[i] = m_scratchStartIndices[i - 1] + m_scratchCounts[i - 1];

    cudaMemcpy(outStartIdx, m_scratchStartIndices.data(), countBytes, cudaMemcpyHostToDevice);

    cudaMemset(contactForceTensor->data, 0, maxCount * sizeof(float));
    cudaMemset(contactPointTensor->data, 0, maxCount * 3 * sizeof(float));
    cudaMemset(contactNormalTensor->data, 0, maxCount * 3 * sizeof(float));
    cudaMemset(contactSeparationTensor->data, 0, maxCount * sizeof(float));
    cudaMemset(outCounts, 0, countBytes);

    launchContactData(m_cachedContactCount, m_cachedShape0, m_cachedShape1, _getContactPoint0(), _getContactPoint1(),
                      m_cachedContactNormal, force, _getThickness0(), _getThickness1(), m_cachedShapeBody,
                      m_cachedBodyQ, m_deviceBodySensorMap, m_bodyCount, m_deviceBodyFilterMap, m_bodyCount,
                      static_cast<int>(m_filterCount), m_worldBodyIndex, dtScale, maxCount,
                      static_cast<float*>(contactForceTensor->data), static_cast<float*>(contactPointTensor->data),
                      static_cast<float*>(contactNormalTensor->data), static_cast<float*>(contactSeparationTensor->data),
                      outCounts, outStartIdx, m_rigidContactMax, m_contactPointsInWorldSpace);
    cudaDeviceSynchronize();

    return true;
}

bool GpuRigidContactView::getRawContactData(const TensorDesc* contactForceTensor,
                                            const TensorDesc* contactPointTensor,
                                            const TensorDesc* contactNormalTensor,
                                            const TensorDesc* contactSeparationTensor,
                                            const TensorDesc* contactCountTensor,
                                            const TensorDesc* contactStartIndicesTensor,
                                            const TensorDesc* otherActorIdsTensor,
                                            float dt) const
{
    if (!contactForceTensor || !contactPointTensor || !contactNormalTensor || !contactSeparationTensor ||
        !contactCountTensor || !contactStartIndicesTensor || !otherActorIdsTensor)
        return false;
    if (!contactForceTensor->data || !contactPointTensor->data || !contactNormalTensor->data ||
        !contactSeparationTensor->data || !contactCountTensor->data || !contactStartIndicesTensor->data ||
        !otherActorIdsTensor->data)
        return false;
    if (m_sensorCount == 0 || m_rigidContactMax <= 0)
        return false;

    const uint32_t maxCount = m_maxContactDataCount;

    if (!checkTensorDevice(*contactForceTensor, m_deviceOrdinal, "raw contact force", __FUNCTION__) ||
        !checkTensorFloat32(*contactForceTensor, "raw contact force", __FUNCTION__) ||
        !checkTensorSizeExact(*contactForceTensor, maxCount, "raw contact force", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactPointTensor, m_deviceOrdinal, "raw contact point", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "raw contact point", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, maxCount * 3u, "raw contact point", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactNormalTensor, m_deviceOrdinal, "raw contact normal", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "raw contact normal", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor, maxCount * 3u, "raw contact normal", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactSeparationTensor, m_deviceOrdinal, "raw contact separation", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "raw contact separation", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, maxCount, "raw contact separation", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactCountTensor, m_deviceOrdinal, "raw contact count", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "raw contact count", __FUNCTION__) ||
        !checkTensorSizeExact(*contactCountTensor, m_sensorCount, "raw contact count", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactStartIndicesTensor, m_deviceOrdinal, "raw contact start indices", __FUNCTION__) ||
        !checkTensorInt32(*contactStartIndicesTensor, "raw contact start indices", __FUNCTION__) ||
        !checkTensorSizeExact(*contactStartIndicesTensor, m_sensorCount, "raw contact start indices", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*otherActorIdsTensor, m_deviceOrdinal, "raw other actor ids", __FUNCTION__) ||
        !checkTensorInt64(*otherActorIdsTensor, "raw other actor ids", __FUNCTION__) ||
        !checkTensorSizeExact(*otherActorIdsTensor, maxCount, "raw other actor ids", __FUNCTION__))
    {
        return false;
    }

    _refreshContactPointers();
    const float* force = _resolveContactForce();
    if (!force)
        return false;

    float dtScale = _getPhysicsDtScale(dt);
    size_t sensorBytes = m_sensorCount * sizeof(uint32_t);
    auto* outCounts = static_cast<uint32_t*>(contactCountTensor->data);
    auto* outStartIdx = static_cast<uint32_t*>(contactStartIndicesTensor->data);

    cudaMemset(outCounts, 0, sensorBytes);
    launchCountRawContactsPerSensor(m_cachedContactCount, m_cachedShape0, m_cachedShape1, m_cachedShapeBody,
                                    m_deviceBodySensorMap, m_bodyCount, m_worldBodyIndex, outCounts, m_rigidContactMax);
    cudaDeviceSynchronize();

    cudaMemcpy(m_scratchCounts.data(), outCounts, sensorBytes, cudaMemcpyDeviceToHost);

    m_scratchStartIndices[0] = 0;
    for (uint32_t i = 1; i < m_sensorCount; ++i)
        m_scratchStartIndices[i] = m_scratchStartIndices[i - 1] + m_scratchCounts[i - 1];

    cudaMemcpy(outStartIdx, m_scratchStartIndices.data(), sensorBytes, cudaMemcpyHostToDevice);

    cudaMemset(contactForceTensor->data, 0, maxCount * sizeof(float));
    cudaMemset(contactPointTensor->data, 0, maxCount * 3 * sizeof(float));
    cudaMemset(contactNormalTensor->data, 0, maxCount * 3 * sizeof(float));
    cudaMemset(contactSeparationTensor->data, 0, maxCount * sizeof(float));
    cudaMemset(otherActorIdsTensor->data, 0, maxCount * sizeof(uint64_t));
    cudaMemset(outCounts, 0, sensorBytes);

    launchRawContactData(
        m_cachedContactCount, m_cachedShape0, m_cachedShape1, _getContactPoint0(), _getContactPoint1(),
        m_cachedContactNormal, force, _getThickness0(), _getThickness1(), m_cachedShapeBody, m_cachedBodyQ,
        m_deviceBodySensorMap, m_bodyCount, m_worldBodyIndex, dtScale, maxCount,
        static_cast<float*>(contactForceTensor->data), static_cast<float*>(contactPointTensor->data),
        static_cast<float*>(contactNormalTensor->data), static_cast<float*>(contactSeparationTensor->data), outCounts,
        outStartIdx, static_cast<uint64_t*>(otherActorIdsTensor->data), m_rigidContactMax, m_contactPointsInWorldSpace);
    cudaDeviceSynchronize();

    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
