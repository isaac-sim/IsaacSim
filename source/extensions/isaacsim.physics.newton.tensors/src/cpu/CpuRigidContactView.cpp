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

#include "CpuRigidContactView.h"

#include "utils/TensorOps.h"

#include <carb/logging/Log.h>

#include <algorithm>
#include <cstring>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

CpuRigidContactView::CpuRigidContactView(py::object newtonStage,
                                         const std::vector<std::string>& sensorPaths,
                                         const std::vector<std::vector<std::string>>& filterPaths,
                                         uint32_t maxContactDataCount)
    : BaseRigidContactView(newtonStage, sensorPaths, filterPaths, maxContactDataCount)
{
    if (m_rigidContactMax > 0)
    {
        m_hostZeroVec3Buffer.assign(m_rigidContactMax * 3, 0.0f);
        m_hostZeroFloatBuffer.assign(m_rigidContactMax, 0.0f);
    }
}

const float* CpuRigidContactView::_resolveContactForce() const
{
    if (!m_hasSpatialForce)
        return m_cachedContactForce;
    if (!m_cachedSpatialForce)
        return nullptr;

    static thread_local std::vector<float> cpuExtractedForce;
    cpuExtractedForce.resize(m_rigidContactMax * 3);
    cpuExtractVec3FromSpatial(m_cachedSpatialForce, cpuExtractedForce.data(), m_rigidContactMax);
    return cpuExtractedForce.data();
}

const float* CpuRigidContactView::_getContactPoint0() const
{
    return m_cachedContactPoint0 ? m_cachedContactPoint0 : m_hostZeroVec3Buffer.data();
}
const float* CpuRigidContactView::_getContactPoint1() const
{
    return m_cachedContactPoint1 ? m_cachedContactPoint1 : m_hostZeroVec3Buffer.data();
}
const float* CpuRigidContactView::_getThickness0() const
{
    return m_cachedThickness0 ? m_cachedThickness0 : m_hostZeroFloatBuffer.data();
}
const float* CpuRigidContactView::_getThickness1() const
{
    return m_cachedThickness1 ? m_cachedThickness1 : m_hostZeroFloatBuffer.data();
}

bool CpuRigidContactView::getNetContactForces(const TensorDesc* dstTensor, float dt) const
{
    if (!dstTensor || !dstTensor->data || m_sensorCount == 0)
        return false;

    if (!checkTensorDevice(*dstTensor, -1, "net contact forces", __FUNCTION__) ||
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
        std::memset(dstTensor->data, 0, outBytes);
        return true;
    }

    float dtScale = _getPhysicsDtScale(dt);
    std::memset(dstTensor->data, 0, outBytes);
    cpuNetContactForces(m_cachedContactCount, m_cachedShape0, m_cachedShape1, force, m_cachedShapeBody,
                        m_hostBodySensorMap.data(), m_bodyCount, m_worldBodyIndex, dtScale,
                        static_cast<float*>(dstTensor->data), m_rigidContactMax);
    return true;
}

bool CpuRigidContactView::getContactForceMatrix(const TensorDesc* dstTensor, float dt) const
{
    if (!dstTensor || !dstTensor->data || m_sensorCount == 0 || m_filterCount == 0)
        return false;

    if (!checkTensorDevice(*dstTensor, -1, "contact force matrix", __FUNCTION__) ||
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
        std::memset(dstTensor->data, 0, outBytes);
        return true;
    }

    float dtScale = _getPhysicsDtScale(dt);
    std::memset(dstTensor->data, 0, outBytes);
    cpuContactForceMatrix(m_cachedContactCount, m_cachedShape0, m_cachedShape1, force, m_cachedShapeBody,
                          m_hostBodySensorMap.data(), m_bodyCount, m_hostBodyFilterMap.data(), m_bodyCount,
                          m_worldBodyIndex, dtScale, m_filterCount, static_cast<float*>(dstTensor->data),
                          m_rigidContactMax);
    return true;
}

bool CpuRigidContactView::getContactData(const TensorDesc* contactForceTensor,
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

    if (!checkTensorDevice(*contactForceTensor, -1, "contact force", __FUNCTION__) ||
        !checkTensorFloat32(*contactForceTensor, "contact force", __FUNCTION__) ||
        !checkTensorSizeExact(*contactForceTensor, maxCount, "contact force", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactPointTensor, -1, "contact point", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "contact point", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, maxCount * 3u, "contact point", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactNormalTensor, -1, "contact normal", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "contact normal", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor, maxCount * 3u, "contact normal", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactSeparationTensor, -1, "contact separation", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "contact separation", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, maxCount, "contact separation", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactCountTensor, -1, "contact count", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "contact count", __FUNCTION__) ||
        !checkTensorSizeExact(*contactCountTensor, pairCount, "contact count", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactStartIndicesTensor, -1, "contact start indices", __FUNCTION__) ||
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

    std::fill_n(m_scratchCounts.data(), pairCount, 0u);
    cpuCountContactsPerPair(m_cachedContactCount, m_cachedShape0, m_cachedShape1, m_cachedShapeBody,
                            m_hostBodySensorMap.data(), m_bodyCount, m_hostBodyFilterMap.data(), m_bodyCount,
                            static_cast<int>(m_filterCount), m_worldBodyIndex, m_scratchCounts.data(), m_rigidContactMax);

    m_scratchStartIndices[0] = 0;
    for (uint32_t i = 1; i < pairCount; ++i)
        m_scratchStartIndices[i] = m_scratchStartIndices[i - 1] + m_scratchCounts[i - 1];

    std::memcpy(contactCountTensor->data, m_scratchCounts.data(), pairCount * sizeof(uint32_t));
    std::memcpy(contactStartIndicesTensor->data, m_scratchStartIndices.data(), pairCount * sizeof(uint32_t));

    std::memset(contactForceTensor->data, 0, maxCount * sizeof(float));
    std::memset(contactPointTensor->data, 0, maxCount * 3 * sizeof(float));
    std::memset(contactNormalTensor->data, 0, maxCount * 3 * sizeof(float));
    std::memset(contactSeparationTensor->data, 0, maxCount * sizeof(float));

    std::fill_n(m_scratchFillCounts.data(), pairCount, 0u);
    cpuContactData(m_cachedContactCount, m_cachedShape0, m_cachedShape1, _getContactPoint0(), _getContactPoint1(),
                   m_cachedContactNormal, force, _getThickness0(), _getThickness1(), m_cachedShapeBody, m_cachedBodyQ,
                   m_hostBodySensorMap.data(), m_bodyCount, m_hostBodyFilterMap.data(), m_bodyCount,
                   static_cast<int>(m_filterCount), m_worldBodyIndex, dtScale, maxCount,
                   static_cast<float*>(contactForceTensor->data), static_cast<float*>(contactPointTensor->data),
                   static_cast<float*>(contactNormalTensor->data), static_cast<float*>(contactSeparationTensor->data),
                   m_scratchFillCounts.data(), m_scratchStartIndices.data(), m_rigidContactMax,
                   m_contactPointsInWorldSpace);

    std::memcpy(contactCountTensor->data, m_scratchFillCounts.data(), pairCount * sizeof(uint32_t));
    return true;
}

bool CpuRigidContactView::getRawContactData(const TensorDesc* contactForceTensor,
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

    if (!checkTensorDevice(*contactForceTensor, -1, "raw contact force", __FUNCTION__) ||
        !checkTensorFloat32(*contactForceTensor, "raw contact force", __FUNCTION__) ||
        !checkTensorSizeExact(*contactForceTensor, maxCount, "raw contact force", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactPointTensor, -1, "raw contact point", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "raw contact point", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, maxCount * 3u, "raw contact point", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactNormalTensor, -1, "raw contact normal", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "raw contact normal", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor, maxCount * 3u, "raw contact normal", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactSeparationTensor, -1, "raw contact separation", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "raw contact separation", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, maxCount, "raw contact separation", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactCountTensor, -1, "raw contact count", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "raw contact count", __FUNCTION__) ||
        !checkTensorSizeExact(*contactCountTensor, m_sensorCount, "raw contact count", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactStartIndicesTensor, -1, "raw contact start indices", __FUNCTION__) ||
        !checkTensorInt32(*contactStartIndicesTensor, "raw contact start indices", __FUNCTION__) ||
        !checkTensorSizeExact(*contactStartIndicesTensor, m_sensorCount, "raw contact start indices", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*otherActorIdsTensor, -1, "raw other actor ids", __FUNCTION__) ||
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

    std::fill_n(m_scratchCounts.data(), m_sensorCount, 0u);
    cpuCountRawContactsPerSensor(m_cachedContactCount, m_cachedShape0, m_cachedShape1, m_cachedShapeBody,
                                 m_hostBodySensorMap.data(), m_bodyCount, m_worldBodyIndex, m_scratchCounts.data(),
                                 m_rigidContactMax);

    m_scratchStartIndices[0] = 0;
    for (uint32_t i = 1; i < m_sensorCount; ++i)
        m_scratchStartIndices[i] = m_scratchStartIndices[i - 1] + m_scratchCounts[i - 1];

    std::memcpy(contactCountTensor->data, m_scratchCounts.data(), m_sensorCount * sizeof(uint32_t));
    std::memcpy(contactStartIndicesTensor->data, m_scratchStartIndices.data(), m_sensorCount * sizeof(uint32_t));

    std::memset(contactForceTensor->data, 0, maxCount * sizeof(float));
    std::memset(contactPointTensor->data, 0, maxCount * 3 * sizeof(float));
    std::memset(contactNormalTensor->data, 0, maxCount * 3 * sizeof(float));
    std::memset(contactSeparationTensor->data, 0, maxCount * sizeof(float));
    std::memset(otherActorIdsTensor->data, 0, maxCount * sizeof(uint64_t));

    std::fill_n(m_scratchFillCounts.data(), m_sensorCount, 0u);
    cpuRawContactData(m_cachedContactCount, m_cachedShape0, m_cachedShape1, _getContactPoint0(), _getContactPoint1(),
                      m_cachedContactNormal, force, _getThickness0(), _getThickness1(), m_cachedShapeBody,
                      m_cachedBodyQ, m_hostBodySensorMap.data(), m_bodyCount, m_worldBodyIndex, dtScale, maxCount,
                      static_cast<float*>(contactForceTensor->data), static_cast<float*>(contactPointTensor->data),
                      static_cast<float*>(contactNormalTensor->data), static_cast<float*>(contactSeparationTensor->data),
                      m_scratchFillCounts.data(), m_scratchStartIndices.data(),
                      static_cast<uint64_t*>(otherActorIdsTensor->data), m_rigidContactMax, m_contactPointsInWorldSpace);

    std::memcpy(contactCountTensor->data, m_scratchFillCounts.data(), m_sensorCount * sizeof(uint32_t));
    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
