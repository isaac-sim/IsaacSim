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

#include "GpuRigidBodyView.h"

#include "CudaCommon.h"
#include "CudaKernels.h"
#include "GpuGatherHelper.h"

#include <carb/logging/Log.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

GpuRigidBodyView::GpuRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& bodyPaths, int deviceOrdinal)
    : BaseRigidBodyView(newtonStage, bodyPaths), m_deviceOrdinal(deviceOrdinal)
{

    if (m_count > 0)
    {
        _uploadMappingsToGpu();
        _allocateStagingBuffer();
    }
}

GpuRigidBodyView::~GpuRigidBodyView()
{
    safeCudaFree(m_deviceBodyIndices);
    safeCudaFree(m_deviceFreeJointQStartIndices);
    safeCudaFree(m_deviceIndexScratch);
    safeCudaFree(m_stagingBuffer);
    safeCudaFree(m_deviceComOrientation);
}

void GpuRigidBodyView::_uploadMappingsToGpu()
{
    if (!validateCudaContext(m_deviceOrdinal))
        return;
    if (m_bodyIndices.empty())
        return;

    size_t bytes = m_bodyIndices.size() * sizeof(int);
    cudaError_t err = cudaMalloc(&m_deviceBodyIndices, bytes);
    if (err != cudaSuccess)
    {
        (void)cudaGetLastError();
        CARB_LOG_ERROR("_uploadMappingsToGpu cudaMalloc failed: %s", cudaGetErrorString(err));
        return;
    }
    err = cudaMemcpy(m_deviceBodyIndices, m_bodyIndices.data(), bytes, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
        (void)cudaGetLastError();
        cudaFree(m_deviceBodyIndices);
        (void)cudaGetLastError();
        m_deviceBodyIndices = nullptr;
    }

    if (!m_freeJointQStartIndices.empty())
    {
        bytes = m_freeJointQStartIndices.size() * sizeof(int);
        err = cudaMalloc(&m_deviceFreeJointQStartIndices, bytes);
        if (err != cudaSuccess)
        {
            (void)cudaGetLastError();
            return;
        }
        err = cudaMemcpy(m_deviceFreeJointQStartIndices, m_freeJointQStartIndices.data(), bytes, cudaMemcpyHostToDevice);
        if (err != cudaSuccess)
        {
            (void)cudaGetLastError();
            cudaFree(m_deviceFreeJointQStartIndices);
            (void)cudaGetLastError();
            m_deviceFreeJointQStartIndices = nullptr;
        }
    }
}

void GpuRigidBodyView::_allocateStagingBuffer()
{
    if (!validateCudaContext(m_deviceOrdinal))
        return;
    m_stagingMaxFloats = m_count * size_t(9);
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
    size_t orientBytes = size_t(m_count) * 4 * sizeof(float);
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

// ---- Getters ----

bool GpuRigidBodyView::getTransforms(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 7u, "transform", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n) {
            return launchGatherTransform(
                reinterpret_cast<const wp::transform*>(m_cachedBodyQ), dst, m_deviceBodyIndices, n);
        },
        dstTensor, static_cast<int>(m_bodyIndices.size()), 7, m_stagingBuffer);
}

bool GpuRigidBodyView::getVelocities(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 6u, "velocity", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherSpatialVector(
                reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQd), dst, m_deviceBodyIndices, n);
        },
        dstTensor, static_cast<int>(m_bodyIndices.size()), 6, m_stagingBuffer);
}

bool GpuRigidBodyView::getAccelerations(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid || !m_cachedBodyQdd)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 6u, "acceleration", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherSpatialVector(
                reinterpret_cast<const wp::spatial_vector*>(m_cachedBodyQdd), dst, m_deviceBodyIndices, n);
        },
        dstTensor, static_cast<int>(m_bodyIndices.size()), 6, m_stagingBuffer);
}

bool GpuRigidBodyView::getMasses(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, m_count, "masses", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedBodyMass, dst, m_deviceBodyIndices, n); },
                     dstTensor, static_cast<int>(m_bodyIndices.size()), 1, m_stagingBuffer);
}

bool GpuRigidBodyView::getInvMasses(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, m_count, "inv masses", __FUNCTION__))
        return false;
    return gpuGather([this](float* dst, int n)
                     { return launchGatherFloat(m_cachedBodyInverseMass, dst, m_deviceBodyIndices, n); },
                     dstTensor, static_cast<int>(m_bodyIndices.size()), 1, m_stagingBuffer);
}

bool GpuRigidBodyView::getCOMs(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 7u, "com", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherCenterOfMass(reinterpret_cast<const wp::vec3*>(m_cachedBodyCenterOfMass), dst,
                                            m_deviceBodyIndices, n, m_deviceComOrientation);
        },
        dstTensor, static_cast<int>(m_bodyIndices.size()), 7, m_stagingBuffer);
}

bool GpuRigidBodyView::getInertias(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 9u, "inertia", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n) {
            return launchGatherMat33(
                reinterpret_cast<const wp::mat33*>(m_cachedBodyInertia), dst, m_deviceBodyIndices, n);
        },
        dstTensor, static_cast<int>(m_bodyIndices.size()), 9, m_stagingBuffer);
}

bool GpuRigidBodyView::getInvInertias(const TensorDesc* dstTensor) const
{
    if (!m_cacheValid)
        return false;
    if (!validateFloat32TensorAnyDevice(dstTensor, size_t(m_count) * 9u, "inv inertia", __FUNCTION__))
        return false;
    return gpuGather(
        [this](float* dst, int n)
        {
            return launchGatherMat33(
                reinterpret_cast<const wp::mat33*>(m_cachedBodyInverseInertia), dst, m_deviceBodyIndices, n);
        },
        dstTensor, static_cast<int>(m_bodyIndices.size()), 9, m_stagingBuffer);
}

// ---- Setters ----

bool GpuRigidBodyView::setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * 7u, "transform", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedRootFlatScatter(src, m_cachedJointQ, idx.ptr, m_deviceFreeJointQStartIndices, idx.count, 7);
    if (ok)
        _evalForwardKinematics();
    return ok;
}

bool GpuRigidBodyView::setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * 6u, "velocity", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedRootScatter(src, m_cachedBodyQd, idx.ptr, m_deviceBodyIndices, idx.count, 6);
    if (ok)
        _evalInverseKinematics();
    return ok;
}

bool GpuRigidBodyView::setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, m_count, "mass", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedRootScatter(src, m_cachedBodyMass, idx.ptr, m_deviceBodyIndices, idx.count, 1);
    if (ok)
        _updateInverseMasses();
    return ok;
}

bool GpuRigidBodyView::setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * 7u, "com", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok =
        launchFusedLinkScatter(src, m_cachedBodyCenterOfMass, idx.ptr, m_deviceBodyIndices, idx.count, 1, 7, 3, 0, 3);
    if (ok && m_deviceComOrientation)
    {
        ok = launchScatterComOrientation(src, m_deviceComOrientation, idx.ptr, idx.count, 1, 7);
    }
    return ok;
}

bool GpuRigidBodyView::setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!validateFloat32TensorAnyDevice(srcTensor, size_t(m_count) * 9u, "inertia", __FUNCTION__) ||
        !validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;
    auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
    const float* src = ensureGpuSrc(srcTensor, m_stagingBuffer, m_stagingMaxFloats);
    if (!src)
        return false;
    bool ok = launchFusedLinkScatter(src, m_cachedBodyInertia, idx.ptr, m_deviceBodyIndices, idx.count, 1, 9, 9, 0, 9);
    if (ok)
        _updateInverseInertias();
    return ok;
}

bool GpuRigidBodyView::applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return applyForcesAndTorquesAtPosition(srcTensor, nullptr, nullptr, indexTensor, true);
}

bool GpuRigidBodyView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                       const TensorDesc* srcTorqueTensor,
                                                       const TensorDesc* srcPositionTensor,
                                                       const TensorDesc* indexTensor,
                                                       bool isGlobal)
{
    const bool hasForce = srcForceTensor && srcForceTensor->data;
    const bool hasTorque = srcTorqueTensor && srcTorqueTensor->data;
    if (!hasForce && !hasTorque)
        return false;

    if (hasForce && !validateFloat32TensorAnyDevice(srcForceTensor, size_t(m_count) * 3u, "force", __FUNCTION__))
        return false;
    if (hasTorque && !validateFloat32TensorAnyDevice(srcTorqueTensor, size_t(m_count) * 3u, "torque", __FUNCTION__))
        return false;
    if (srcPositionTensor && srcPositionTensor->data &&
        !validateFloat32TensorAnyDevice(srcPositionTensor, size_t(m_count) * 3u, "position", __FUNCTION__))
        return false;
    if (!validateOptionalIndexTensorAnyDevice(indexTensor, __FUNCTION__))
        return false;

    if (hasForce)
    {
        auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
        const float* src = ensureGpuSrc(srcForceTensor, m_stagingBuffer, m_stagingMaxFloats);
        if (!src)
            return false;
        if (!launchFusedLinkAdd(src, m_cachedBodyF, idx.ptr, m_deviceBodyIndices, idx.count, 1, 3, 6, 0, 3))
            return false;
    }

    if (hasTorque)
    {
        auto idx = _resolveGpuIndices(indexTensor, m_deviceIndexScratch);
        const float* src = ensureGpuSrc(srcTorqueTensor, m_stagingBuffer, m_stagingMaxFloats);
        if (!src)
            return false;
        if (!launchFusedLinkAdd(src, m_cachedBodyF, idx.ptr, m_deviceBodyIndices, idx.count, 1, 3, 6, 3, 3))
            return false;
    }

    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
