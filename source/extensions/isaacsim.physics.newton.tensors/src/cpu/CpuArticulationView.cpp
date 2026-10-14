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

#include "CpuArticulationView.hpp"

#include "CpuGatherHelper.hpp"
#include "utils/WrenchOps.hpp"

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

CpuArticulationView::CpuArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& articulationPaths)
    : BaseArticulationView(newtonStage, articulationPaths)
{
}

// ---- DOF State ----

bool CpuArticulationView::getDofPositions(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof positions", __FUNCTION__))
        return false;
    gatherFloat(m_cachedJointQ, static_cast<float*>(dstTensor->data), m_dofPosIndices.data(), m_dofPosIndices.size());
    return true;
}

bool CpuArticulationView::getDofVelocities(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof velocities", __FUNCTION__))
        return false;
    gatherFloat(m_cachedJointQd, static_cast<float*>(dstTensor->data), m_dofVelIndices.data(), m_dofVelIndices.size());
    return true;
}

bool CpuArticulationView::setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof positions", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofPosIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointQ, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    return true;
}

bool CpuArticulationView::setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof velocities", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofVelIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointQd, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    return true;
}

// ---- DOF Property Getters ----

bool CpuArticulationView::getDofLimits(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs * 2u, "dof limits", __FUNCTION__))
        return false;
    gatherPairedFloat(m_cachedJointLimitLower, m_cachedJointLimitUpper, static_cast<float*>(dstTensor->data),
                      m_dofAxisIndices.data(), m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofStiffnesses(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof stiffnesses", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedJointTargetKe, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(), m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofDampings(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof dampings", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedJointTargetKd, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(), m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofArmatures(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof armatures", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedJointArmature, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(), m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofMaxForces(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof max forces", __FUNCTION__))
        return false;
    gatherFloat(m_cachedJointEffortLimit, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(),
                m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofMaxVelocities(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof max velocities", __FUNCTION__))
        return false;
    gatherFloat(m_cachedJointVelocityLimit, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(),
                m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofPositionTargets(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof position targets", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedCtrlTargetPos, static_cast<float*>(dstTensor->data), m_dofPosIndices.data(), m_dofPosIndices.size());
    return true;
}

bool CpuArticulationView::getDofVelocityTargets(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof velocity targets", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedCtrlTargetVel, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(), m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofActuationForces(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "dof actuation forces", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedJointTorques, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(), m_dofAxisIndices.size());
    return true;
}

bool CpuArticulationView::getDofTypes(const TensorDesc* dstTensor) const
{
    if (m_hostDofTypes.empty())
        return false;
    if (!validateUint8Tensor(dstTensor, -1, m_hostDofTypes.size(), "dof types", __FUNCTION__))
        return false;
    uint32_t n = static_cast<uint32_t>(m_hostDofTypes.size());
    std::memcpy(dstTensor->data, m_hostDofTypes.data(), n);
    return true;
}

// ---- DOF Property Setters ----

bool CpuArticulationView::setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs * 2u, "dof limits", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    const float* src = static_cast<const float*>(srcTensor->data);

    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    for (uint32_t idx : artiIndices)
    {
        if (idx >= m_count)
            continue;
        for (uint32_t d = 0; d < m_maxDofs; ++d)
        {
            int dst = m_dofAxisIndices[idx * m_maxDofs + d];
            if (dst < 0)
                continue;
            m_scratchSourceOffset.push_back(static_cast<int>((idx * m_maxDofs + d) * 2));
            m_scratchDestinationIndex.push_back(dst);
        }
    }
    if (!m_scratchSourceOffset.empty())
    {
        indirectScatterFloat(src, m_cachedJointLimitLower, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
        for (size_t i = 0; i < m_scratchSourceOffset.size(); ++i)
            m_scratchSourceOffset[i] += 1;
        indirectScatterFloat(src, m_cachedJointLimitUpper, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    }
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return true;
}

bool CpuArticulationView::setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof stiffnesses", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointTargetKe,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
        _syncCtrlDirectActuatorGains();
    }
    return true;
}

bool CpuArticulationView::setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof dampings", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointTargetKd,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
        _syncCtrlDirectActuatorGains();
    }
    return true;
}

bool CpuArticulationView::setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof max forces", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointEffortLimit,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return true;
}

bool CpuArticulationView::setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof max velocities", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointVelocityLimit,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return true;
}

bool CpuArticulationView::setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof armatures", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointArmature,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return true;
}

bool CpuArticulationView::setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof actuation forces", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointTorques,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    return true;
}

bool CpuArticulationView::setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof position targets", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofPosIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedCtrlTargetPos,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    {
        py::gil_scoped_acquire gil;
        _syncCtrlDirectPositionTargets();
    }
    return true;
}

bool CpuArticulationView::setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs, "dof velocity targets", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedCtrlTargetVel,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    return true;
}

// ---- Root Transforms / Velocities ----

bool CpuArticulationView::getRootTransforms(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 7u, "root transform", __FUNCTION__))
        return false;
    gatherTransform(reinterpret_cast<const wp::transform*>(m_cachedBodyQ), static_cast<float*>(dstTensor->data),
                    m_rootBodyIndices.data(), m_rootBodyIndices.size());
    return true;
}

bool CpuArticulationView::getRootVelocities(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * 6u, "root velocity", __FUNCTION__))
        return false;
    gatherSpatialVector(reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQd),
                        static_cast<float*>(dstTensor->data), m_rootBodyIndices.data(), m_rootBodyIndices.size());
    return true;
}

bool CpuArticulationView::setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * 7u, "root transform", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const float* srcData = static_cast<const float*>(srcTensor->data);
    const auto& artiIndices = _resolveIndices(indexTensor);

    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();

    std::vector<int> jointXpSrcOffsets;
    std::vector<int> jointXpDstIndices;

    for (uint32_t idx : artiIndices)
    {
        if (idx >= m_count)
            continue;
        int qStart = m_rootJointQStartIndices[idx];
        if (qStart >= 0)
        {
            for (int e = 0; e < 7; ++e)
            {
                m_scratchSourceOffset.push_back(static_cast<int>(idx * 7 + e));
                m_scratchDestinationIndex.push_back(qStart + e);
            }
        }
        else
        {
            int rootJoint = m_rootJointIndices[idx];
            for (int e = 0; e < 7; ++e)
            {
                jointXpSrcOffsets.push_back(static_cast<int>(idx * 7 + e));
                jointXpDstIndices.push_back(rootJoint * 7 + e);
            }
        }
    }
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(srcData, m_cachedJointQ, m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    if (!jointXpSrcOffsets.empty())
        indirectScatterFloat(
            srcData, m_cachedJointXp, jointXpSrcOffsets.data(), jointXpDstIndices.data(), jointXpSrcOffsets.size());
    _evalForwardKinematics();
    return true;
}

bool CpuArticulationView::setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * 6u, "root velocity", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildRootScatterMappings(artiIndices, 6, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
    {
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedBodyQd, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    }
    _evalInverseKinematics();
    return true;
}

// ---- Link Transforms / Velocities ----

bool CpuArticulationView::getLinkTransforms(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxLinks * 7u, "link transform", __FUNCTION__))
        return false;
    gatherTransform(reinterpret_cast<const wp::transform*>(m_cachedBodyQ), static_cast<float*>(dstTensor->data),
                    m_linkFlatIndices.data(), m_linkFlatIndices.size());
    return true;
}

bool CpuArticulationView::getLinkVelocities(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxLinks * 6u, "link velocity", __FUNCTION__))
        return false;
    gatherSpatialVector(reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQd),
                        static_cast<float*>(dstTensor->data), m_linkFlatIndices.data(), m_linkFlatIndices.size());
    return true;
}

bool CpuArticulationView::getLinkIncomingJointForce(const TensorDesc* dstTensor) const
{
    if (!_checkBinding() || !_hasJointForceOutput())
        return false;
    if (!validateFloat32Tensor(
            dstTensor, -1, size_t(m_count) * m_maxLinks * 6u, "link incoming joint force", __FUNCTION__))
        return false;
    gatherIncomingJointForce(reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyParentForce),
                             reinterpret_cast<const wp::transform*>(m_cachedBodyQ),
                             reinterpret_cast<const wp::vec3*>(m_cachedBodyCenterOfMass),
                             reinterpret_cast<const wp::transform*>(m_cachedJointXc),
                             static_cast<float*>(dstTensor->data), m_linkFlatIndices.data(),
                             m_linkJointFlatIndices.data(), m_linkFlatIndices.size());
    return true;
}

bool CpuArticulationView::getDofProjectedJointForces(const TensorDesc* dstTensor) const
{
    if (!_checkBinding() || !_hasJointForceOutput())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs, "DOF projected joint forces", __FUNCTION__))
        return false;
    if (!_evaluateMotionSubspaces())
        return false;
    gatherProjectedJointForce(reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyParentForce),
                              reinterpret_cast<const wp::transform*>(m_cachedBodyQ),
                              reinterpret_cast<const wp::vec3*>(m_cachedBodyCenterOfMass),
                              reinterpret_cast<const wp::spatial_vector*>(m_cachedMotionSubspaces),
                              static_cast<float*>(dstTensor->data), m_dofBodyIndices.data(), m_dofAxisIndices.data(),
                              m_dofAxisIndices.size());
    return true;
}

// ---- Body Properties ----

bool CpuArticulationView::getMasses(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxLinks, "masses", __FUNCTION__))
        return false;
    gatherFloat(
        m_cachedBodyMass, static_cast<float*>(dstTensor->data), m_linkFlatIndices.data(), m_linkFlatIndices.size());
    return true;
}

bool CpuArticulationView::getInvMasses(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxLinks, "inv masses", __FUNCTION__))
        return false;
    gatherFloat(m_cachedBodyInverseMass, static_cast<float*>(dstTensor->data), m_linkFlatIndices.data(),
                m_linkFlatIndices.size());
    return true;
}

bool CpuArticulationView::setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxLinks, "masses", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildLinkScatterMappings(artiIndices, 1, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedBodyMass, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    _updateInverseMasses();
    return true;
}

bool CpuArticulationView::getCOMs(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxLinks * 7u, "com", __FUNCTION__))
        return false;
    gatherCenterOfMass(reinterpret_cast<const wp::vec3*>(m_cachedBodyCenterOfMass), static_cast<float*>(dstTensor->data),
                       m_linkFlatIndices.data(), m_linkFlatIndices.size(), m_cachedComOrientation.data());
    return true;
}

bool CpuArticulationView::setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxLinks * 7u, "com", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const float* srcData = static_cast<const float*>(srcTensor->data);
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    for (uint32_t idx : artiIndices)
    {
        if (idx >= m_count)
            continue;
        const auto& links = m_linkIndicesPerArticulation[idx];
        for (uint32_t j = 0; j < m_maxLinks && j < links.size(); ++j)
        {
            int srcBase = (idx * m_maxLinks + j) * 7;
            int linkIdx = links[j];
            for (int c = 0; c < 3; ++c)
            {
                m_scratchSourceOffset.push_back(srcBase + c);
                m_scratchDestinationIndex.push_back(linkIdx * 3 + c);
            }
            uint32_t flatLink = idx * m_maxLinks + j;
            for (int c = 0; c < 4; ++c)
                m_cachedComOrientation[flatLink * 4 + c] = srcData[srcBase + 3 + c];
        }
    }
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(srcData, m_cachedBodyCenterOfMass, m_scratchSourceOffset.data(),
                             m_scratchDestinationIndex.data(), m_scratchSourceOffset.size());
    return true;
}

bool CpuArticulationView::getInertias(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxLinks * 9u, "inertia", __FUNCTION__))
        return false;
    gatherMat33(reinterpret_cast<const wp::mat33*>(m_cachedBodyInertia), static_cast<float*>(dstTensor->data),
                m_linkFlatIndices.data(), m_linkFlatIndices.size());
    return true;
}

bool CpuArticulationView::getInvInertias(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxLinks * 9u, "inv inertia", __FUNCTION__))
        return false;
    gatherMat33(reinterpret_cast<const wp::mat33*>(m_cachedBodyInverseInertia), static_cast<float*>(dstTensor->data),
                m_linkFlatIndices.data(), m_linkFlatIndices.size());
    return true;
}

bool CpuArticulationView::setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxLinks * 9u, "inertia", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildLinkScatterMappings(artiIndices, 9, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedBodyInertia,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size());
    _updateInverseInertias();
    return true;
}

// ---- Applied Forces ----

bool CpuArticulationView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                          const TensorDesc* srcTorqueTensor,
                                                          const TensorDesc* srcPositionTensor,
                                                          const TensorDesc* indexTensor,
                                                          bool isGlobal)
{
    if (!_checkBinding())
        return false;
    const bool hasForce = srcForceTensor && srcForceTensor->data;
    const bool hasTorque = srcTorqueTensor && srcTorqueTensor->data;
    if (!hasForce && !hasTorque)
        return false;

    if (hasForce && !validateFloat32Tensor(srcForceTensor, -1, size_t(m_count) * m_maxLinks * 3u, "force", __FUNCTION__))
        return false;
    if (hasTorque &&
        !validateFloat32Tensor(srcTorqueTensor, -1, size_t(m_count) * m_maxLinks * 3u, "torque", __FUNCTION__))
        return false;
    if (srcPositionTensor && srcPositionTensor->data &&
        !validateFloat32Tensor(srcPositionTensor, -1, size_t(m_count) * m_maxLinks * 3u, "position", __FUNCTION__))
        return false;
    if (!validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;

    const bool hasPosition = srcPositionTensor && srcPositionTensor->data;
    const float* forceData = hasForce ? static_cast<const float*>(srcForceTensor->data) : nullptr;
    const float* torqueData = hasTorque ? static_cast<const float*>(srcTorqueTensor->data) : nullptr;
    const float* positionData = hasPosition ? static_cast<const float*>(srcPositionTensor->data) : nullptr;
    const auto* bodyTransforms = reinterpret_cast<const wp::transform*>(m_cachedBodyQ);
    const auto* bodyCentersOfMass = reinterpret_cast<const wp::vec3*>(m_cachedBodyCenterOfMass);

    const auto& articulationIndices = _resolveIndices(indexTensor);
    for (uint32_t idx : articulationIndices)
    {
        if (idx >= m_count)
            continue;

        const auto& links = m_linkIndicesPerArticulation[idx];
        for (uint32_t linkSlot = 0; linkSlot < m_maxLinks && linkSlot < links.size(); ++linkSlot)
        {
            const int bodyIdx = links[linkSlot];
            const size_t sourceOffset = (size_t(idx) * m_maxLinks + linkSlot) * 3;
            wp::vec3 forceWorld;
            wp::vec3 torqueWorld;
            details::computeWorldWrench(
                hasForce ? forceData + sourceOffset : nullptr, hasTorque ? torqueData + sourceOffset : nullptr,
                hasPosition ? positionData + sourceOffset : nullptr, bodyTransforms[bodyIdx],
                bodyCentersOfMass[bodyIdx], isGlobal, hasForce, hasTorque, hasPosition, forceWorld, torqueWorld);
            for (int component = 0; component < 3; ++component)
            {
                m_cachedBodyF[bodyIdx * 6 + component] += forceWorld[component];
                m_cachedBodyF[bodyIdx * 6 + 3 + component] += torqueWorld[component];
            }
        }
    }

    return true;
}

// ---- Joint Frictions ----//

bool CpuArticulationView::setDofFrictionProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!_checkBinding())
        return false;
    // We expect 3 properties, but we will only use the first one (static friction)
    if (!validateFloat32Tensor(srcTensor, -1, size_t(m_count) * m_maxDofs * 3u, "friction properties", __FUNCTION__) ||
        !validateOptionalIndexTensor(indexTensor, -1, __FUNCTION__))
        return false;
    const auto& artiIndices = _resolveIndices(indexTensor);
    m_scratchSourceOffset.clear();
    m_scratchDestinationIndex.clear();
    _buildDofScatterMappings(artiIndices, m_dofAxisIndices, m_scratchSourceOffset, m_scratchDestinationIndex);
    if (!m_scratchSourceOffset.empty())
        indirectScatterFloat(static_cast<const float*>(srcTensor->data), m_cachedJointFriction,
                             m_scratchSourceOffset.data(), m_scratchDestinationIndex.data(),
                             m_scratchSourceOffset.size(), 3u);
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return true;
}
bool CpuArticulationView::getDofFrictionProperties(const TensorDesc* dstTensor) const
{
    if (!_checkBinding())
        return false;
    if (!validateFloat32Tensor(dstTensor, -1, size_t(m_count) * m_maxDofs * 3u, "friction properties", __FUNCTION__))
        return false;
    // Static friction
    gatherFloat(m_cachedJointFriction, static_cast<float*>(dstTensor->data), m_dofAxisIndices.data(),
                m_dofAxisIndices.size(), 3u);
    // Dynamic friction
    gatherFloat(m_cachedJointFriction, static_cast<float*>(dstTensor->data) + 1, m_dofAxisIndices.data(),
                m_dofAxisIndices.size(), 3u);
    // Viscous friction
    zeroFloat(static_cast<float*>(dstTensor->data) + 2, m_dofAxisIndices.size(), 3u);
    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
