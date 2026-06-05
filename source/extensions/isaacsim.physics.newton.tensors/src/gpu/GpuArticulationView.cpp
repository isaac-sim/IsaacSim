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

#include "GpuArticulationView.h"

#include "CudaCommon.h"
#include "CudaKernels.h"
#include "GpuGatherHelper.h"

#include <carb/logging/Log.h>

#include <algorithm>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

GpuArticulationView::GpuArticulationView(py::object newtonStage,
                                         const std::vector<pxr::SdfPath>& articulationPaths,
                                         int deviceOrdinal)
    : BaseArticulationView(newtonStage, articulationPaths), m_deviceOrdinal(deviceOrdinal)
{

    if (m_count > 0)
    {
        _uploadMappingsToGpu();
        _allocateStagingBuffer();
    }
}

GpuArticulationView::~GpuArticulationView()
{
    safeCudaFree(m_deviceDofPositionIndices);
    safeCudaFree(m_deviceDofVelocityIndices);
    safeCudaFree(m_deviceDofAxisIndices);
    safeCudaFree(m_deviceLinkFlatIndices);
    safeCudaFree(m_deviceRootIndices);
    safeCudaFree(m_deviceRootJointQStartIndices);
    safeCudaFree(m_deviceFixedRootJointMapping);
    safeCudaFree(m_deviceIndexScratch);
    safeCudaFree(m_deviceDofTypes);
    safeCudaFree(m_stagingBuffer);
    safeCudaFree(m_deviceComOrientation);
}

void GpuArticulationView::_uploadMappingsToGpu()
{
    if (!validateCudaContext(m_deviceOrdinal))
        return;

    auto upload = [](const std::vector<int>& host, int*& dev)
    {
        if (host.empty())
            return;
        size_t bytes = host.size() * sizeof(int);
        cudaError_t err = cudaMalloc(&dev, bytes);
        if (err != cudaSuccess)
        {
            (void)cudaGetLastError();
            return;
        }
        err = cudaMemcpy(dev, host.data(), bytes, cudaMemcpyHostToDevice);
        if (err != cudaSuccess)
        {
            (void)cudaGetLastError();
            cudaFree(dev);
            (void)cudaGetLastError();
            dev = nullptr;
        }
    };
    upload(m_dofPosIndices, m_deviceDofPositionIndices);
    upload(m_dofVelIndices, m_deviceDofVelocityIndices);
    upload(m_dofAxisIndices, m_deviceDofAxisIndices);
    upload(m_linkFlatIndices, m_deviceLinkFlatIndices);
    upload(m_rootBodyIndices, m_deviceRootIndices);
    upload(m_rootJointQStartIndices, m_deviceRootJointQStartIndices);
    upload(m_fixedRootJointMapping, m_deviceFixedRootJointMapping);

    if (!m_hostDofTypes.empty())
    {
        size_t bytes = m_hostDofTypes.size() * sizeof(uint8_t);
        cudaError_t err = cudaMalloc(&m_deviceDofTypes, bytes);
        if (err != cudaSuccess)
        {
            (void)cudaGetLastError();
            return;
        }
        err = cudaMemcpy(m_deviceDofTypes, m_hostDofTypes.data(), bytes, cudaMemcpyHostToDevice);
        if (err != cudaSuccess)
        {
            (void)cudaGetLastError();
            cudaFree(m_deviceDofTypes);
            (void)cudaGetLastError();
            m_deviceDofTypes = nullptr;
        }
    }
}

void GpuArticulationView::_allocateStagingBuffer()
{
    if (!validateCudaContext(m_deviceOrdinal))
        return;
    m_stagingMaxFloats = std::max({
        m_linkFlatIndices.size() * size_t(9), // mat33
        m_linkFlatIndices.size() * size_t(7), // transform / COM
        m_rootBodyIndices.size() * size_t(7), // root transform
        m_dofAxisIndices.size() * size_t(2), // paired (limits)
        size_t(m_count) * m_maxDofs, // DOF setter input
    });
    if (m_stagingMaxFloats == 0)
        return;
    cudaError_t err = cudaMalloc(&m_stagingBuffer, m_stagingMaxFloats * sizeof(float));
    if (err != cudaSuccess)
    {
        (void)cudaGetLastError();
        CARB_LOG_ERROR("_allocateStagingBuffer cudaMalloc failed: %s", cudaGetErrorString(err));
        m_stagingBuffer = nullptr;
        m_stagingMaxFloats = 0;
    }
    err = cudaMalloc(&m_deviceIndexScratch, m_count * sizeof(int));
    if (err != cudaSuccess)
    {
        (void)cudaGetLastError();
        CARB_LOG_ERROR("_allocateStagingBuffer cudaMalloc for index scratch failed: %s", cudaGetErrorString(err));
        m_deviceIndexScratch = nullptr;
    }
    size_t orientBytes = m_cachedComOrientation.size() * sizeof(float);
    if (orientBytes > 0)
    {
        err = cudaMalloc(&m_deviceComOrientation, orientBytes);
        if (err != cudaSuccess)
        {
            (void)cudaGetLastError();
            m_deviceComOrientation = nullptr;
        }
        else
        {
            cudaMemcpy(m_deviceComOrientation, m_cachedComOrientation.data(), orientBytes, cudaMemcpyHostToDevice);
        }
    }
}

// ---- DOF State ----

bool GpuArticulationView::getDofPositions(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof positions", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointQ, dst, m_deviceDofPositionIndices, n); },
                     dstTensor, static_cast<int>(m_dofPosIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofVelocities(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof velocities", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointQd, dst, m_deviceDofVelocityIndices, n); },
                     dstTensor, static_cast<int>(m_dofVelIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof positions", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    return launchFusedDofScatter(src, m_cachedJointQ, idx.ptr, m_deviceDofPositionIndices, idx.count, m_maxDofs);
}

bool GpuArticulationView::setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof velocities", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    return launchFusedDofScatter(src, m_cachedJointQd, idx.ptr, m_deviceDofVelocityIndices, idx.count, m_maxDofs);
}

// ---- DOF Property Getters ----

bool GpuArticulationView::getDofLimits(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs * 2u, "dof limits", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n) {
            return launchGatherPairedFloat(
                m_cachedJointLimitLower, m_cachedJointLimitUpper, dst, m_deviceDofAxisIndices, n);
        },
        dstTensor, static_cast<int>(m_dofAxisIndices.size()), 2, m_stagingBuffer);
}

bool GpuArticulationView::getDofStiffnesses(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof stiffnesses", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointTargetKe, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofDampings(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof dampings", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointTargetKd, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofArmatures(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof armatures", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointArmature, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofMaxForces(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof max forces", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointEffortLimit, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofMaxVelocities(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof max velocities", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointVelocityLimit, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofPositionTargets(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof position targets", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedCtrlTargetPos, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofVelocityTargets(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof velocity targets", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedCtrlTargetVel, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofActuationForces(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxDofs, "dof actuation forces", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedJointTorques, dst, m_deviceDofAxisIndices, n); },
                     dstTensor, static_cast<int>(m_dofAxisIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getDofTypes(const TensorDesc* dstTensor) const
{
    if (m_hostDofTypes.empty())
        return false;
    if (!dstTensor || !dstTensor->data)
    {
        CARB_LOG_ERROR("dof types tensor is null or has no data in %s", __FUNCTION__);
        return false;
    }
    if (dstTensor->dtype != TensorDataType::eUint8 && dstTensor->dtype != TensorDataType::eInt8)
    {
        CARB_LOG_ERROR("Incompatible data type of dof types tensor in function %s: expected 8-bit integer, received %s",
                       __FUNCTION__, getTensorDtypeCstr(*dstTensor));
        return false;
    }
    if (!checkTensorSizeExact(*dstTensor, m_hostDofTypes.size(), "dof types", __FUNCTION__))
        return false;
    uint32_t n = static_cast<uint32_t>(m_hostDofTypes.size());
    if (dstTensor->device < 0)
    {
        std::memcpy(dstTensor->data, m_hostDofTypes.data(), n);
    }
    else
    {
        if (!m_deviceDofTypes)
            return false;
        return launchCopyUint8(m_deviceDofTypes, static_cast<uint8_t*>(dstTensor->data), n);
    }
    return true;
}

// ---- DOF Property Setters ----

bool GpuArticulationView::setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs * 2u, "dof limits", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedPairedDofScatter(
        src, m_cachedJointLimitLower, m_cachedJointLimitUpper, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    if (ok)
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return ok;
}

bool GpuArticulationView::setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof stiffnesses", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedDofScatter(src, m_cachedJointTargetKe, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    if (ok)
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
        _syncCtrlDirectActuatorGains();
    }
    return ok;
}

bool GpuArticulationView::setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof dampings", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedDofScatter(src, m_cachedJointTargetKd, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    if (ok)
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
        _syncCtrlDirectActuatorGains();
    }
    return ok;
}

bool GpuArticulationView::setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof max forces", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedDofScatter(src, m_cachedJointEffortLimit, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    if (ok)
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return ok;
}

bool GpuArticulationView::setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof max velocities", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok =
        launchFusedDofScatter(src, m_cachedJointVelocityLimit, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    if (ok)
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return ok;
}

bool GpuArticulationView::setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof armatures", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedDofScatter(src, m_cachedJointArmature, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    if (ok)
    {
        py::gil_scoped_acquire gil;
        _notifyJointDofPropertiesChanged();
    }
    return ok;
}

bool GpuArticulationView::setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof actuation forces", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    return launchFusedDofScatter(src, m_cachedJointTorques, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
}

bool GpuArticulationView::setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof position targets", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedDofScatter(src, m_cachedCtrlTargetPos, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    if (ok)
    {
        py::gil_scoped_acquire gil;
        _syncCtrlDirectPositionTargets();
    }
    return ok;
}

bool GpuArticulationView::setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxDofs, "dof velocity targets", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedDofScatter(src, m_cachedCtrlTargetVel, idx.ptr, m_deviceDofAxisIndices, idx.count, m_maxDofs);
    return ok;
}

// ---- Root Transforms / Velocities ----

bool GpuArticulationView::getRootTransforms(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 7u, "root transform", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n) {
            return launchGatherTransform(
                reinterpret_cast<const wp::transform*>(m_cachedBodyQ), dst, m_deviceRootIndices, n);
        },
        dstTensor, static_cast<int>(m_rootBodyIndices.size()), 7, m_stagingBuffer);
}

bool GpuArticulationView::getRootVelocities(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 6u, "root velocity", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherSpatialVector(
                reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQd), dst, m_deviceRootIndices, n);
        },
        dstTensor, static_cast<int>(m_rootBodyIndices.size()), 6, m_stagingBuffer);
}

bool GpuArticulationView::setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * 7u, "root transform", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedRootFlatScatter(src, m_cachedJointQ, idx.ptr, m_deviceRootJointQStartIndices, idx.count, 7);
    if (ok && m_deviceFixedRootJointMapping)
        ok = launchFusedRootScatter(src, m_cachedJointXp, idx.ptr, m_deviceFixedRootJointMapping, idx.count, 7);
    if (ok)
        _evalForwardKinematics();
    return ok;
}

bool GpuArticulationView::setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * 6u, "root velocity", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedRootScatter(src, m_cachedBodyQd, idx.ptr, m_deviceRootIndices, idx.count, 6);
    if (ok)
        _evalInverseKinematics();
    return ok;
}

// ---- Link Transforms / Velocities ----

bool GpuArticulationView::getLinkTransforms(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxLinks * 7u, "link transform", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherTransform(
                reinterpret_cast<const wp::transform*>(m_cachedBodyQ), dst, m_deviceLinkFlatIndices, n);
        },
        dstTensor, static_cast<int>(m_linkFlatIndices.size()), 7, m_stagingBuffer);
}

bool GpuArticulationView::getLinkVelocities(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxLinks * 6u, "link velocity", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherSpatialVector(
                reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQd), dst, m_deviceLinkFlatIndices, n);
        },
        dstTensor, static_cast<int>(m_linkFlatIndices.size()), 6, m_stagingBuffer);
}

// ---- Body Properties ----

bool GpuArticulationView::getMasses(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxLinks, "masses", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedBodyMass, dst, m_deviceLinkFlatIndices, n); },
                     dstTensor, static_cast<int>(m_linkFlatIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::getInvMasses(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxLinks, "inv masses", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedBodyInverseMass, dst, m_deviceLinkFlatIndices, n); },
                     dstTensor, static_cast<int>(m_linkFlatIndices.size()), 1, m_stagingBuffer);
}

bool GpuArticulationView::setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxLinks, "masses", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedLinkScatter(
        src, m_cachedBodyMass, idx.ptr, m_deviceLinkFlatIndices, idx.count, m_maxLinks, 1, 1, 0, 1);
    if (ok)
        _updateInverseMasses();
    return ok;
}

bool GpuArticulationView::getCOMs(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxLinks * 7u, "com", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherCenterOfMass(reinterpret_cast<const wp::vec3*>(m_cachedBodyCenterOfMass), dst,
                                            m_deviceLinkFlatIndices, n, m_deviceComOrientation);
        },
        dstTensor, static_cast<int>(m_linkFlatIndices.size()), 7, m_stagingBuffer);
}

bool GpuArticulationView::setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxLinks * 7u, "com", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedLinkScatter(
        src, m_cachedBodyCenterOfMass, idx.ptr, m_deviceLinkFlatIndices, idx.count, m_maxLinks, 7, 3, 0, 3);
    if (ok && m_deviceComOrientation)
    {
        ok = launchScatterComOrientation(src, m_deviceComOrientation, idx.ptr, idx.count, m_maxLinks, 7);
    }
    return ok;
}

bool GpuArticulationView::getInertias(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxLinks * 9u, "inertia", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n) {
            return launchGatherMat33(
                reinterpret_cast<const wp::mat33*>(m_cachedBodyInertia), dst, m_deviceLinkFlatIndices, n);
        },
        dstTensor, static_cast<int>(m_linkFlatIndices.size()), 9, m_stagingBuffer);
}

bool GpuArticulationView::getInvInertias(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * m_maxLinks * 9u, "inv inertia", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherMat33(
                reinterpret_cast<const wp::mat33*>(m_cachedBodyInverseInertia), dst, m_deviceLinkFlatIndices, n);
        },
        dstTensor, static_cast<int>(m_linkFlatIndices.size()), 9, m_stagingBuffer);
}

bool GpuArticulationView::setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * m_maxLinks * 9u, "inertia", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedLinkScatter(
        src, m_cachedBodyInertia, idx.ptr, m_deviceLinkFlatIndices, idx.count, m_maxLinks, 9, 9, 0, 9);
    if (ok)
        _updateInverseInertias();
    return ok;
}

// ---- Applied Forces ----

bool GpuArticulationView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                          const TensorDesc* srcTorqueTensor,
                                                          const TensorDesc* srcPositionTensor,
                                                          const TensorDesc* indexTensor,
                                                          bool isGlobal)
{
    const bool hasForce = srcForceTensor && srcForceTensor->data;
    const bool hasTorque = srcTorqueTensor && srcTorqueTensor->data;
    if (!hasForce && !hasTorque)
        return false;

    if (hasForce &&
        !validateFloat32TensorAnyDevice(srcForceTensor, size_t(m_count) * m_maxLinks * 3u, "force", __FUNCTION__))
        return false;
    if (hasTorque &&
        !validateFloat32TensorAnyDevice(srcTorqueTensor, size_t(m_count) * m_maxLinks * 3u, "torque", __FUNCTION__))
        return false;
    if (!validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;

    if (hasForce)
    {
        auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
        const float* src = ensureGpuSrc(srcForceTensor, m_stagingBuffer, m_stagingMaxFloats);
        if (!src)
            return false;
        if (!launchFusedLinkAdd(src, m_cachedBodyF, idx.ptr, m_deviceLinkFlatIndices, idx.count, m_maxLinks, 3, 6, 0, 3))
            return false;
    }

    if (hasTorque)
    {
        auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
        const float* src = ensureGpuSrc(srcTorqueTensor, m_stagingBuffer, m_stagingMaxFloats);
        if (!src)
            return false;
        if (!launchFusedLinkAdd(src, m_cachedBodyF, idx.ptr, m_deviceLinkFlatIndices, idx.count, m_maxLinks, 3, 6, 3, 3))
            return false;
    }

    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
