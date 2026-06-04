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

#include "CpuRigidBodyView.h"

#include "CpuGatherHelper.h"

#include <carb/logging/Log.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

CpuRigidBodyView::CpuRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& bodyPaths)
    : BaseRigidBodyView(newtonStage, bodyPaths)
{
}

bool CpuRigidBodyView::getTransforms(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 7u, "transform", __FUNCTION__))
        return false;
    gatherTransform(reinterpret_cast<const wp::transform*>(m_cachedBodyQ), static_cast<float*>(dstTensor->data),
                    m_bodyIndices.data(), m_bodyIndices.size());
    return true;
}

bool CpuRigidBodyView::getVelocities(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 6u, "velocity", __FUNCTION__))
        return false;
    gatherSpatialVector(reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQd),
                        static_cast<float*>(dstTensor->data), m_bodyIndices.data(), m_bodyIndices.size());
    return true;
}

bool CpuRigidBodyView::getAccelerations(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid || !m_cachedBodyQdd)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 6u, "acceleration", __FUNCTION__))
        return false;
    gatherSpatialVector(reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQdd),
                        static_cast<float*>(dstTensor->data), m_bodyIndices.data(), m_bodyIndices.size());
    return true;
}

bool CpuRigidBodyView::getMasses(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, m_count, "masses", __FUNCTION__))
        return false;
    gatherFloat(m_cachedBodyMass, static_cast<float*>(dstTensor->data), m_bodyIndices.data(), m_bodyIndices.size());
    return true;
}

bool CpuRigidBodyView::getInvMasses(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, m_count, "inv masses", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedBodyInverseMass, static_cast<float*>(dstTensor->data), m_bodyIndices.data(), m_bodyIndices.size());
    return true;
}

bool CpuRigidBodyView::getCOMs(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 7u, "com", __FUNCTION__))
        return false;
    gatherCenterOfMass(reinterpret_cast<const wp::vec3*>(m_cachedBodyCenterOfMass), static_cast<float*>(dstTensor->data),
                       m_bodyIndices.data(), m_bodyIndices.size(), m_cachedComOrientation.data());
    return true;
}

bool CpuRigidBodyView::getInertias(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 9u, "inertia", __FUNCTION__))
        return false;
    gatherMat33(reinterpret_cast<const wp::mat33*>(m_cachedBodyInertia), static_cast<float*>(dstTensor->data),
                m_bodyIndices.data(), m_bodyIndices.size());
    return true;
}

bool CpuRigidBodyView::getInvInertias(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 9u, "inv inertia", __FUNCTION__))
        return false;
    gatherMat33(reinterpret_cast<const wp::mat33*>(m_cachedBodyInverseInertia), static_cast<float*>(dstTensor->data),
                m_bodyIndices.data(), m_bodyIndices.size());
    return true;
}

bool CpuRigidBodyView::setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * 7u, "transform", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& viewIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    for (uint32_t idx : viewIndices)
    {
        if (idx >= m_count)
            continue;
        int qStart = m_freeJointQStartIndices[idx];
        if (qStart < 0)
            continue;
        for (int e = 0; e < 7; ++e)
        {
            m_scratchSourceOffset.push_back(static_cast<int>(idx * 7 + e));
            m_scratchDestinationIndex.push_back(qStart + e);
        }
    }
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointQ, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    _evalForwardKinematics();
    return true;
}

bool CpuRigidBodyView::setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * 6u, "velocity", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& viewIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildScatterMappings(viewIndices, 6, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
    {
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedBodyQd, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    }
    _evalInverseKinematics();
    return true;
}

bool CpuRigidBodyView::setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32Tensor(srcTensor, -1, m_count, "mass", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& viewIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildScatterMappings(viewIndices, 1, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedBodyMass, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    _updateInverseMasses();
    return true;
}

bool CpuRigidBodyView::setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * 7u, "com", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const float* srcData = static_cast<const float*>(srcTensor->data);
    const auto& viewIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    for (uint32_t idx : viewIndices)
    {
        if (idx >= m_count)
            continue;
        int bodyIdx = m_bodyIndices[idx];
        for (int c = 0; c < 3; ++c)
        {
            m_scratchSourceOffset.push_back(static_cast<int>(idx * 7 + c));
            m_scratchDestinationIndex.push_back(bodyIdx * 3 + c);
        }
        for (int c = 0; c < 4; ++c)
            m_cachedComOrientation[idx * 4 + c] = srcData[idx * 7 + 3 + c];
    }
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(srcData, m_cachedBodyCenterOfMass, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    return true;
}

bool CpuRigidBodyView::setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * 9u, "inertia", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& viewIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildScatterMappings(viewIndices, 9, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), reinterpret_cast<float*>(m_cachedBodyInertia),
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    _updateInverseInertias();
    return true;
}

bool CpuRigidBodyView::applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return applyForcesAndTorquesAtPosition(srcTensor, nullptr, nullptr, indexTensor, true);
}

bool CpuRigidBodyView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                       const TensorDesc* srcTorqueTensor,
                                                       const TensorDesc* srcPositionTensor,
                                                       const TensorDesc* indexTensor,
                                                       bool isGlobal)
{
    const bool hasForce = srcForceTensor && srcForceTensor->data;
    const bool hasTorque = srcTorqueTensor && srcTorqueTensor->data;
    if (!hasForce && !hasTorque)
        return false;

    if (hasForce && !validateFloat32Tensor(srcForceTensor, -1, size_t(m_count) * 3u, "force", __FUNCTION__))
        return false;
    if (hasTorque && !validateFloat32Tensor(srcTorqueTensor, -1, size_t(m_count) * 3u, "torque", __FUNCTION__))
        return false;
    if (srcPositionTensor && srcPositionTensor->data &&
        !validateFloat32Tensor(srcPositionTensor, -1, size_t(m_count) * 3u, "position", __FUNCTION__))
        return false;
    if (!validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;

    if (hasForce)
    {
        const auto& viewIndices = _resolveIndices(indexTensor);
        m_scratchSourceOffset.clear();
        m_scratchDestinationIndex.clear();
        for (uint32_t idx : viewIndices)
        {
            if (idx >= m_count)
                continue;
            int bodyIdx = m_bodyIndices[idx];
            for (int e = 0; e < 3; ++e)
            {
                m_scratchSourceOffset.push_back(static_cast<int>(idx * 3 + e));
                m_scratchDestinationIndex.push_back(bodyIdx * 6 + e);
            }
        }
        if (!m_scratchSourceOffset.empty())
            indirectAddFloat(static_cast<const float*>(srcForceTensor->data), m_cachedBodyF, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    }

    if (hasTorque)
    {
        const auto& viewIndices = _resolveIndices(indexTensor);
        m_scratchSourceOffset.clear();
        m_scratchDestinationIndex.clear();
        for (uint32_t idx : viewIndices)
        {
            if (idx >= m_count)
                continue;
            int bodyIdx = m_bodyIndices[idx];
            for (int e = 0; e < 3; ++e)
            {
                m_scratchSourceOffset.push_back(static_cast<int>(idx * 3 + e));
                m_scratchDestinationIndex.push_back(bodyIdx * 6 + 3 + e);
            }
        }
        if (!m_scratchSourceOffset.empty())
            indirectAddFloat(static_cast<const float*>(srcTorqueTensor->data), m_cachedBodyF,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    }

    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
