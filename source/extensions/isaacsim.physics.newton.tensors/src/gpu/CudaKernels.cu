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

#include "CudaKernels.h"
#include "CudaCommon.h"

namespace isaacsim {
namespace physics {
namespace newton {
namespace tensors {

constexpr int BLOCK_SIZE = 256;

// ---- Scalar float gather ----

template<typename T>
__global__ void gatherKernel(const T* src, T* dst, const int* indices, int numIndices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices) {
        int idx = indices[i];
        dst[i] = (idx >= 0) ? src[idx] : T(0);
    }
}

// ---- In-place inverse mass ----

__global__ void updateInverseMassKernel(const float* mass, float* inverseMass, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        float m = mass[i];
        inverseMass[i] = (m > 1e-8f) ? (1.0f / m) : 0.0f;
    }
}

// ---- In-place inverse inertia (3x3 matrix) ----

__global__ void updateInverseInertiaKernel(const wp::mat33* inertia, wp::mat33* inverseInertia, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        wp::mat33 I = inertia[i];
        float a = I.data[0][0], b = I.data[0][1], c = I.data[0][2];
        float d = I.data[1][0], e = I.data[1][1], f = I.data[1][2];
        float g = I.data[2][0], h = I.data[2][1], k = I.data[2][2];
        float det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
        if (fabsf(det) > 1e-8f) {
            float invDet = 1.0f / det;
            wp::mat33 inv;
            inv.data[0][0] = (e * k - f * h) * invDet;
            inv.data[0][1] = (c * h - b * k) * invDet;
            inv.data[0][2] = (b * f - c * e) * invDet;
            inv.data[1][0] = (f * g - d * k) * invDet;
            inv.data[1][1] = (a * k - c * g) * invDet;
            inv.data[1][2] = (c * d - a * f) * invDet;
            inv.data[2][0] = (d * h - e * g) * invDet;
            inv.data[2][1] = (b * g - a * h) * invDet;
            inv.data[2][2] = (a * e - b * d) * invDet;
            inverseInertia[i] = inv;
        } else {
            wp::mat33 zero;
            for (int r = 0; r < 3; ++r)
                for (int col = 0; col < 3; ++col)
                    zero.data[r][col] = 0.0f;
            inverseInertia[i] = zero;
        }
    }
}

// ---- Transform (7 floats: px,py,pz, qx,qy,qz,qw) ----

__global__ void gatherTransformKernel(const wp::transform* src, float* dst, const int* indices, int numIndices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices) {
        int idx = indices[i];
        int offset = i * 7;
        if (idx >= 0) {
            wp::transform t = src[idx];
            dst[offset + 0] = t.p[0]; dst[offset + 1] = t.p[1]; dst[offset + 2] = t.p[2];
            dst[offset + 3] = t.q[0]; dst[offset + 4] = t.q[1]; dst[offset + 5] = t.q[2]; dst[offset + 6] = t.q[3];
        } else {
            for (int k = 0; k < 7; ++k) dst[offset + k] = 0.0f;
        }
    }
}

// ---- SpatialVector (6 floats) ----
// Newton stores body_qd as [linear(3), angular(3)] in wp::spatial_vector memory,
// which already matches the PhysX tensor API convention. No reordering needed.

__global__ void gatherSpatialVectorKernel(const wp::spatial_vector* src, float* dst, const int* indices, int numIndices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices) {
        int idx = indices[i];
        int offset = i * 6;
        if (idx >= 0) {
            wp::spatial_vector sv = src[idx];
            dst[offset + 0] = sv.w[0]; dst[offset + 1] = sv.w[1]; dst[offset + 2] = sv.w[2];
            dst[offset + 3] = sv.v[0]; dst[offset + 4] = sv.v[1]; dst[offset + 5] = sv.v[2];
        } else {
            for (int k = 0; k < 6; ++k) dst[offset + k] = 0.0f;
        }
    }
}

// ---- Mat33 (9 floats, row-major) ----

__global__ void gatherMat33Kernel(const wp::mat33* src, float* dst, const int* indices, int numIndices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices) {
        int idx = indices[i];
        int offset = i * 9;
        if (idx >= 0) {
            wp::mat33 m = src[idx];
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col)
                    dst[offset + row * 3 + col] = m.data[row][col];
        } else {
            for (int k = 0; k < 9; ++k) dst[offset + k] = 0.0f;
        }
    }
}

// ---- Paired gather for [lo, hi] interleaved layout ----

__global__ void gatherPairedFloatKernel(const float* srcA, const float* srcB, float* dst,
                                        const int* indices, int numIndices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices) {
        int idx = indices[i];
        int off = i * 2;
        if (idx >= 0) {
            dst[off]     = srcA[idx];
            dst[off + 1] = srcB[idx];
        } else {
            dst[off] = dst[off + 1] = 0.0f;
        }
    }
}

// ---- Center-of-mass gather: vec3 → 7-float (px,py,pz, 0,0,0,1) ----

__global__ void gatherCenterOfMassKernel(const wp::vec3* src, float* dst, const int* indices, int numIndices,
                                         const float* cachedOrientation) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices) {
        int idx = indices[i];
        float* out = dst + i * 7;
        if (idx >= 0) {
            wp::vec3 v = src[idx];
            out[0] = v.c[0]; out[1] = v.c[1]; out[2] = v.c[2];
        } else {
            out[0] = out[1] = out[2] = 0.0f;
        }
        const float* q = cachedOrientation + i * 4;
        out[3] = q[0]; out[4] = q[1]; out[5] = q[2]; out[6] = q[3];
    }
}

// ---- COM orientation scatter: extract quat from 7-float COM tensor ----

__global__ void scatterComOrientationKernel(const float* src, float* dst,
                                            const int* artiIndices, int count,
                                            int elemPerSlot, int srcStride) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int total = count * elemPerSlot;
    if (i >= total) return;
    int slot = i / elemPerSlot;
    int elem = i % elemPerSlot;
    int artiIdx = artiIndices ? artiIndices[slot] : slot;
    int flatIdx = artiIdx * elemPerSlot + elem;
    const float* in = src + flatIdx * srcStride + 3;
    float* out = dst + flatIdx * 4;
    out[0] = in[0]; out[1] = in[1]; out[2] = in[2]; out[3] = in[3];
}

// ---- Fused indexed scatter/add kernels ----

__global__ void fusedDofScatterKernel(const float* src, float* dst,
                                      const int* artiIndices, const int* dofMapping,
                                      int numArti, int maxDofs) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numArti * maxDofs) return;
    int slot = i / maxDofs;
    int localDof = i % maxDofs;
    int artiIdx = artiIndices ? artiIndices[slot] : slot;
    int flatIdx = artiIdx * maxDofs + localDof;
    int dstIdx = dofMapping[flatIdx];
    if (dstIdx >= 0) {
        dst[dstIdx] = src[flatIdx];
    }
}

__global__ void fusedPairedDofScatterKernel(const float* src, float* dstA, float* dstB,
                                            const int* artiIndices, const int* dofMapping,
                                            int numArti, int maxDofs) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numArti * maxDofs) return;
    int slot = i / maxDofs;
    int localDof = i % maxDofs;
    int artiIdx = artiIndices ? artiIndices[slot] : slot;
    int flatIdx = artiIdx * maxDofs + localDof;
    int dstIdx = dofMapping[flatIdx];
    if (dstIdx >= 0) {
        int srcBase = flatIdx * 2;
        dstA[dstIdx] = src[srcBase];
        dstB[dstIdx] = src[srcBase + 1];
    }
}

__global__ void fusedLinkScatterKernel(const float* src, float* dst,
                                       const int* artiIndices, const int* linkMapping,
                                       int numArti, int maxLinks,
                                       int srcElemSize, int dstElemSize,
                                       int dstElemOffset, int numComponents) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int totalWork = numArti * maxLinks * numComponents;
    if (i >= totalWork) return;
    int slot = i / (maxLinks * numComponents);
    int remainder = i % (maxLinks * numComponents);
    int linkSlot = remainder / numComponents;
    int comp = remainder % numComponents;
    int artiIdx = artiIndices ? artiIndices[slot] : slot;
    int bodyIdx = linkMapping[artiIdx * maxLinks + linkSlot];
    if (bodyIdx >= 0) {
        int srcIdx = (artiIdx * maxLinks + linkSlot) * srcElemSize + comp;
        int dstOff = bodyIdx * dstElemSize + dstElemOffset + comp;
        dst[dstOff] = src[srcIdx];
    }
}

__global__ void fusedLinkAddKernel(const float* src, float* dst,
                                   const int* artiIndices, const int* linkMapping,
                                   int numArti, int maxLinks,
                                   int srcElemSize, int dstElemSize,
                                   int dstElemOffset, int numComponents) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int totalWork = numArti * maxLinks * numComponents;
    if (i >= totalWork) return;
    int slot = i / (maxLinks * numComponents);
    int remainder = i % (maxLinks * numComponents);
    int linkSlot = remainder / numComponents;
    int comp = remainder % numComponents;
    int artiIdx = artiIndices ? artiIndices[slot] : slot;
    int bodyIdx = linkMapping[artiIdx * maxLinks + linkSlot];
    if (bodyIdx >= 0) {
        int srcIdx = (artiIdx * maxLinks + linkSlot) * srcElemSize + comp;
        int dstOff = bodyIdx * dstElemSize + dstElemOffset + comp;
        atomicAdd(&dst[dstOff], src[srcIdx]);
    }
}

__global__ void fusedRootScatterKernel(const float* src, float* dst,
                                       const int* artiIndices, const int* rootMapping,
                                       int numArti, int elemSize) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numArti * elemSize) return;
    int slot = i / elemSize;
    int elem = i % elemSize;
    int artiIdx = artiIndices ? artiIndices[slot] : slot;
    if (artiIdx < 0) return;
    int bodyIdx = rootMapping[artiIdx];
    if (bodyIdx < 0) return;
    dst[bodyIdx * elemSize + elem] = src[artiIdx * elemSize + elem];
}

__global__ void fusedRootFlatScatterKernel(const float* src, float* dst,
                                           const int* artiIndices, const int* rootFlatMapping,
                                           int numArti, int elemSize) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numArti * elemSize) return;
    int slot = i / elemSize;
    int elem = i % elemSize;
    int artiIdx = artiIndices ? artiIndices[slot] : slot;
    if (artiIdx < 0) return;
    int flatBase = rootFlatMapping[artiIdx];
    if (flatBase < 0) return;
    dst[flatBase + elem] = src[artiIdx * elemSize + elem];
}

// ---- Launch wrappers ----

#define LAUNCH(kernel, ...) \
    do { \
        int numBlocks = (numIndices + BLOCK_SIZE - 1) / BLOCK_SIZE; \
        (void)cudaGetLastError(); \
        kernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(__VA_ARGS__); \
        CHECK_CUDA_LAUNCH(); \
        return true; \
    } while(0)

bool launchGatherFloat(const float* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream) {
    LAUNCH(gatherKernel<float>, src, dst, devIndices, numIndices);
}

bool launchGatherTransform(const wp::transform* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream) {
    LAUNCH(gatherTransformKernel, src, dst, devIndices, numIndices);
}

bool launchGatherSpatialVector(const wp::spatial_vector* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream) {
    LAUNCH(gatherSpatialVectorKernel, src, dst, devIndices, numIndices);
}

bool launchGatherMat33(const wp::mat33* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream) {
    LAUNCH(gatherMat33Kernel, src, dst, devIndices, numIndices);
}

bool launchGatherCenterOfMass(const wp::vec3* src, float* dst, const int* devIndices, int numIndices,
                              const float* cachedOrientation, cudaStream_t stream) {
    LAUNCH(gatherCenterOfMassKernel, src, dst, devIndices, numIndices, cachedOrientation);
}

bool launchGatherPairedFloat(const float* srcA, const float* srcB, float* dst,
                             const int* devIndices, int numIndices, cudaStream_t stream) {
    LAUNCH(gatherPairedFloatKernel, srcA, srcB, dst, devIndices, numIndices);
}

#undef LAUNCH

bool launchScatterComOrientation(const float* src, float* dst, const int* devArtiIndices,
                                 int count, int elemPerSlot, int srcStride, cudaStream_t stream) {
    int total = count * elemPerSlot;
    if (total <= 0) return true;
    int numBlocks = (total + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    scatterComOrientationKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        src, dst, devArtiIndices, count, elemPerSlot, srcStride);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchUpdateInverseMass(const float* mass, float* inverseMass, int n, cudaStream_t stream) {
    int numBlocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    updateInverseMassKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(mass, inverseMass, n);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchUpdateInverseInertia(const wp::mat33* inertia, wp::mat33* inverseInertia, int n, cudaStream_t stream) {
    int numBlocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    updateInverseInertiaKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(inertia, inverseInertia, n);
    CHECK_CUDA_LAUNCH();
    return true;
}

// ---- Fused launch wrappers ----

bool launchFusedDofScatter(const float* src, float* dst,
                           const int* devArtiIndices, const int* devDofMapping,
                           int numArti, int maxDofs, cudaStream_t stream) {
    int totalWork = numArti * maxDofs;
    int numBlocks = (totalWork + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    fusedDofScatterKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        src, dst, devArtiIndices, devDofMapping, numArti, maxDofs);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchFusedPairedDofScatter(const float* src, float* dstA, float* dstB,
                                 const int* devArtiIndices, const int* devDofMapping,
                                 int numArti, int maxDofs, cudaStream_t stream) {
    int totalWork = numArti * maxDofs;
    int numBlocks = (totalWork + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    fusedPairedDofScatterKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        src, dstA, dstB, devArtiIndices, devDofMapping, numArti, maxDofs);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchFusedLinkScatter(const float* src, float* dst,
                            const int* devArtiIndices, const int* devLinkMapping,
                            int numArti, int maxLinks,
                            int srcElemSize, int dstElemSize,
                            int dstElemOffset, int numComponents,
                            cudaStream_t stream) {
    int totalWork = numArti * maxLinks * numComponents;
    int numBlocks = (totalWork + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    fusedLinkScatterKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        src, dst, devArtiIndices, devLinkMapping, numArti, maxLinks,
        srcElemSize, dstElemSize, dstElemOffset, numComponents);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchFusedLinkAdd(const float* src, float* dst,
                        const int* devArtiIndices, const int* devLinkMapping,
                        int numArti, int maxLinks,
                        int srcElemSize, int dstElemSize,
                        int dstElemOffset, int numComponents,
                        cudaStream_t stream) {
    int totalWork = numArti * maxLinks * numComponents;
    int numBlocks = (totalWork + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    fusedLinkAddKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        src, dst, devArtiIndices, devLinkMapping, numArti, maxLinks,
        srcElemSize, dstElemSize, dstElemOffset, numComponents);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchFusedRootScatter(const float* src, float* dst,
                            const int* devArtiIndices, const int* devRootMapping,
                            int numArti, int elemSize, cudaStream_t stream) {
    int totalWork = numArti * elemSize;
    int numBlocks = (totalWork + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    fusedRootScatterKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        src, dst, devArtiIndices, devRootMapping, numArti, elemSize);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchFusedRootFlatScatter(const float* src, float* dst,
                                const int* devArtiIndices, const int* devRootFlatMapping,
                                int numArti, int elemSize, cudaStream_t stream) {
    int totalWork = numArti * elemSize;
    int numBlocks = (totalWork + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    fusedRootFlatScatterKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        src, dst, devArtiIndices, devRootFlatMapping, numArti, elemSize);
    CHECK_CUDA_LAUNCH();
    return true;
}

// ---- Flat uint8 copy (device-to-device) ----

__global__ void copyUint8Kernel(const uint8_t* src, uint8_t* dst, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) dst[i] = src[i];
}

bool launchCopyUint8(const uint8_t* src, uint8_t* dst, int n, cudaStream_t stream) {
    if (n <= 0) return true;
    int numBlocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    copyUint8Kernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(src, dst, n);
    CHECK_CUDA_LAUNCH();
    return true;
}

// ---- Contact kernels ----

// Helper device functions for contact sensor resolution
__device__ inline void resolveContact(int shapeA, int shapeB,
                                      const int* shapeBody, const int* bodySensorMap,
                                      int bodySensorMapSize, int worldBodyIdx,
                                      int& bodyA, int& bodyB, int& sensorA, int& sensorB) {
    bodyA = shapeBody[shapeA];
    bodyB = shapeBody[shapeB];
    int mappedA = (bodyA < 0) ? worldBodyIdx : bodyA;
    int mappedB = (bodyB < 0) ? worldBodyIdx : bodyB;

    sensorA = (mappedA >= 0 && mappedA < bodySensorMapSize) ? bodySensorMap[mappedA] : -1;
    sensorB = (mappedB >= 0 && mappedB < bodySensorMapSize) ? bodySensorMap[mappedB] : -1;
}

__global__ void extractVec3FromSpatialKernel(const float* src, float* dst, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        dst[i * 3 + 0] = src[i * 6 + 0];
        dst[i * 3 + 1] = src[i * 6 + 1];
        dst[i * 3 + 2] = src[i * 6 + 2];
    }
}

__global__ void netContactForcesKernel(const int* contactCount, const int* shape0, const int* shape1,
                                       const float* contactForce, const int* shapeBody,
                                       const int* bodySensorMap, int bodySensorMapSize,
                                       int worldBodyIdx, float dtScale,
                                       float* netForces, int rigidContactMax) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= rigidContactMax) return;
    int count = contactCount[0];
    if (tid >= count) return;

    int shapeA = shape0[tid];
    int shapeB = shape1[tid];
    if (shapeA == shapeB || shapeA < 0 || shapeB < 0) return;

    int bodyA, bodyB, sensorA, sensorB;
    resolveContact(shapeA, shapeB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx,
                   bodyA, bodyB, sensorA, sensorB);
    if (sensorA < 0 && sensorB < 0) return;

    float fx = contactForce[tid * 3 + 0] * dtScale;
    float fy = contactForce[tid * 3 + 1] * dtScale;
    float fz = contactForce[tid * 3 + 2] * dtScale;

    if (sensorA >= 0) {
        atomicAdd(&netForces[sensorA * 3 + 0], fx);
        atomicAdd(&netForces[sensorA * 3 + 1], fy);
        atomicAdd(&netForces[sensorA * 3 + 2], fz);
    }
    if (sensorB >= 0) {
        atomicAdd(&netForces[sensorB * 3 + 0], -fx);
        atomicAdd(&netForces[sensorB * 3 + 1], -fy);
        atomicAdd(&netForces[sensorB * 3 + 2], -fz);
    }
}

__global__ void contactForceMatrixKernel(const int* contactCount, const int* shape0, const int* shape1,
                                         const float* contactForce, const int* shapeBody,
                                         const int* bodySensorMap, int bodySensorMapSize,
                                         const int* bodyFilterMap, int numBodies,
                                         int worldBodyIdx, float dtScale, int filterCount,
                                         float* forceMatrix, int rigidContactMax) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= rigidContactMax) return;
    int count = contactCount[0];
    if (tid >= count) return;

    int shapeA = shape0[tid];
    int shapeB = shape1[tid];
    if (shapeA == shapeB || shapeA < 0 || shapeB < 0) return;

    int bodyA, bodyB, sensorA, sensorB;
    resolveContact(shapeA, shapeB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx,
                   bodyA, bodyB, sensorA, sensorB);
    if (sensorA < 0 && sensorB < 0) return;

    float fx = contactForce[tid * 3 + 0] * dtScale;
    float fy = contactForce[tid * 3 + 1] * dtScale;
    float fz = contactForce[tid * 3 + 2] * dtScale;

    int mappedB = (bodyB < 0) ? worldBodyIdx : bodyB;
    int mappedA = (bodyA < 0) ? worldBodyIdx : bodyA;

    if (sensorA >= 0 && mappedB >= 0 && mappedB < numBodies) {
        int filterIdx = bodyFilterMap[sensorA * numBodies + mappedB];
        if (filterIdx >= 0) {
            int base = (sensorA * filterCount + filterIdx) * 3;
            atomicAdd(&forceMatrix[base + 0], fx);
            atomicAdd(&forceMatrix[base + 1], fy);
            atomicAdd(&forceMatrix[base + 2], fz);
        }
    }
    if (sensorB >= 0 && mappedA >= 0 && mappedA < numBodies) {
        int filterIdx = bodyFilterMap[sensorB * numBodies + mappedA];
        if (filterIdx >= 0) {
            int base = (sensorB * filterCount + filterIdx) * 3;
            atomicAdd(&forceMatrix[base + 0], -fx);
            atomicAdd(&forceMatrix[base + 1], -fy);
            atomicAdd(&forceMatrix[base + 2], -fz);
        }
    }
}

__global__ void countContactsPerPairKernel(const int* contactCount, const int* shape0, const int* shape1,
                                           const int* shapeBody,
                                           const int* bodySensorMap, int bodySensorMapSize,
                                           const int* bodyFilterMap, int numBodies, int filterCount,
                                           int worldBodyIdx,
                                           uint32_t* counts, int rigidContactMax) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= rigidContactMax) return;
    int count = contactCount[0];
    if (tid >= count) return;

    int shapeA = shape0[tid];
    int shapeB = shape1[tid];
    if (shapeA == shapeB || shapeA < 0 || shapeB < 0) return;

    int bodyA, bodyB, sensorA, sensorB;
    resolveContact(shapeA, shapeB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx,
                   bodyA, bodyB, sensorA, sensorB);
    if (sensorA < 0 && sensorB < 0) return;

    int mappedB = (bodyB < 0) ? worldBodyIdx : bodyB;
    int mappedA = (bodyA < 0) ? worldBodyIdx : bodyA;

    if (sensorA >= 0 && mappedB >= 0 && mappedB < numBodies) {
        int filterIdx = bodyFilterMap[sensorA * numBodies + mappedB];
        if (filterIdx >= 0 && filterIdx < filterCount)
            atomicAdd(&counts[sensorA * filterCount + filterIdx], 1u);
    }
    if (sensorB >= 0 && mappedA >= 0 && mappedA < numBodies) {
        int filterIdx = bodyFilterMap[sensorB * numBodies + mappedA];
        if (filterIdx >= 0 && filterIdx < filterCount)
            atomicAdd(&counts[sensorB * filterCount + filterIdx], 1u);
    }
}

// Transform body-local point to world space, applying thickness offset
__device__ inline void transformContactPoint(const float* point, const float* bodyQ, int rawBody,
                                             float thickness, const float* normal, float sign,
                                             float& wx, float& wy, float& wz) {
    float px = point[0], py = point[1], pz = point[2];
    if (rawBody >= 0) {
        const float* t = bodyQ + rawBody * 7;
        float tx = t[0], ty = t[1], tz = t[2];
        float qx = t[3], qy = t[4], qz = t[5], qw = t[6];
        // quaternion rotate: q * p * q^-1
        float ax = qw * px + qy * pz - qz * py;
        float ay = qw * py + qz * px - qx * pz;
        float az = qw * pz + qx * py - qy * px;
        float aw = -(qx * px + qy * py + qz * pz);
        wx = tx + ax * qw - aw * qx + (ay * qz - az * qy) + sign * thickness * normal[0];
        wy = ty + ay * qw - aw * qy + (az * qx - ax * qz) + sign * thickness * normal[1];
        wz = tz + az * qw - aw * qz + (ax * qy - ay * qx) + sign * thickness * normal[2];
    } else {
        wx = px + sign * thickness * normal[0];
        wy = py + sign * thickness * normal[1];
        wz = pz + sign * thickness * normal[2];
    }
}

__global__ void contactDataKernel(const int* contactCount, const int* shape0, const int* shape1,
                                  const float* point0, const float* point1,
                                  const float* normal, const float* contactForce,
                                  const float* thickness0, const float* thickness1,
                                  const int* shapeBody, const float* bodyQ,
                                  const int* bodySensorMap, int bodySensorMapSize,
                                  const int* bodyFilterMap, int numBodies, int filterCount,
                                  int worldBodyIdx, float dtScale, int maxContactDataCount,
                                  float* outForces, float* outPoints, float* outNormals,
                                  float* outSeparations, uint32_t* outCounts, const uint32_t* startIndices,
                                  int rigidContactMax, bool pointsInWorldSpace) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= rigidContactMax) return;
    int count = contactCount[0];
    if (tid >= count) return;

    int shapeA = shape0[tid];
    int shapeB = shape1[tid];
    if (shapeA == shapeB || shapeA < 0 || shapeB < 0) return;

    int rawBodyA = shapeBody[shapeA];
    int rawBodyB = shapeBody[shapeB];
    int mappedA = (rawBodyA < 0) ? worldBodyIdx : rawBodyA;
    int mappedB = (rawBodyB < 0) ? worldBodyIdx : rawBodyB;

    int sensorA = (mappedA >= 0 && mappedA < bodySensorMapSize) ? bodySensorMap[mappedA] : -1;
    int sensorB = (mappedB >= 0 && mappedB < bodySensorMapSize) ? bodySensorMap[mappedB] : -1;
    if (sensorA < 0 && sensorB < 0) return;

    float nx = normal[tid * 3 + 0], ny = normal[tid * 3 + 1], nz = normal[tid * 3 + 2];
    float fx = contactForce[tid * 3 + 0], fy = contactForce[tid * 3 + 1], fz = contactForce[tid * 3 + 2];
    float forceMag = sqrtf(fx * fx + fy * fy + fz * fz) * dtScale;

    float thk0 = thickness0[tid], thk1 = thickness1[tid];
    float nArr[3] = {nx, ny, nz};
    float p0[3] = {point0[tid * 3], point0[tid * 3 + 1], point0[tid * 3 + 2]};
    float p1[3] = {point1[tid * 3], point1[tid * 3 + 1], point1[tid * 3 + 2]};
    int bodyA = pointsInWorldSpace ? -1 : rawBodyA;
    int bodyB = pointsInWorldSpace ? -1 : rawBodyB;

    float wax, way, waz, wbx, wby, wbz;
    transformContactPoint(p0, bodyQ, bodyA, thk0, nArr, -1.0f, wax, way, waz);
    transformContactPoint(p1, bodyQ, bodyB, thk1, nArr, 1.0f, wbx, wby, wbz);

    float d = nx * (wax - wbx) + ny * (way - wby) + nz * (waz - wbz);
    float cpx = (wax + wbx) * 0.5f, cpy = (way + wby) * 0.5f, cpz = (waz + wbz) * 0.5f;

    if (sensorA >= 0 && mappedB >= 0 && mappedB < numBodies) {
        int filterIdx = bodyFilterMap[sensorA * numBodies + mappedB];
        if (filterIdx >= 0 && filterIdx < filterCount) {
            uint32_t localIdx = atomicAdd(&outCounts[sensorA * filterCount + filterIdx], 1u);
            int writeIdx = (int)startIndices[sensorA * filterCount + filterIdx] + (int)localIdx;
            if (writeIdx < maxContactDataCount) {
                outForces[writeIdx] = -forceMag;
                outPoints[writeIdx * 3 + 0] = cpx; outPoints[writeIdx * 3 + 1] = cpy; outPoints[writeIdx * 3 + 2] = cpz;
                outNormals[writeIdx * 3 + 0] = -nx; outNormals[writeIdx * 3 + 1] = -ny; outNormals[writeIdx * 3 + 2] = -nz;
                outSeparations[writeIdx] = -d;
            }
        }
    }
    if (sensorB >= 0 && mappedA >= 0 && mappedA < numBodies) {
        int filterIdx = bodyFilterMap[sensorB * numBodies + mappedA];
        if (filterIdx >= 0 && filterIdx < filterCount) {
            uint32_t localIdx = atomicAdd(&outCounts[sensorB * filterCount + filterIdx], 1u);
            int writeIdx = (int)startIndices[sensorB * filterCount + filterIdx] + (int)localIdx;
            if (writeIdx < maxContactDataCount) {
                outForces[writeIdx] = forceMag;
                outPoints[writeIdx * 3 + 0] = cpx; outPoints[writeIdx * 3 + 1] = cpy; outPoints[writeIdx * 3 + 2] = cpz;
                outNormals[writeIdx * 3 + 0] = nx; outNormals[writeIdx * 3 + 1] = ny; outNormals[writeIdx * 3 + 2] = nz;
                outSeparations[writeIdx] = d;
            }
        }
    }
}

__global__ void countRawContactsPerSensorKernel(const int* contactCount, const int* shape0, const int* shape1,
                                                const int* shapeBody,
                                                const int* bodySensorMap, int bodySensorMapSize,
                                                int worldBodyIdx,
                                                uint32_t* counts, int rigidContactMax) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= rigidContactMax) return;
    int count = contactCount[0];
    if (tid >= count) return;

    int shapeA = shape0[tid];
    int shapeB = shape1[tid];
    if (shapeA == shapeB || shapeA < 0 || shapeB < 0) return;

    int bodyA, bodyB, sensorA, sensorB;
    resolveContact(shapeA, shapeB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx,
                   bodyA, bodyB, sensorA, sensorB);

    if (sensorA >= 0) atomicAdd(&counts[sensorA], 1u);
    if (sensorB >= 0) atomicAdd(&counts[sensorB], 1u);
}

__global__ void rawContactDataKernel(const int* contactCount, const int* shape0, const int* shape1,
                                     const float* point0In, const float* point1In,
                                     const float* normal, const float* contactForce,
                                     const float* thickness0, const float* thickness1,
                                     const int* shapeBody, const float* bodyQ,
                                     const int* bodySensorMap, int bodySensorMapSize,
                                     int worldBodyIdx, float dtScale, int maxContactDataCount,
                                     float* outForces, float* outPoints, float* outNormals,
                                     float* outSeparations, uint32_t* outCounts,
                                     const uint32_t* startIndices, uint64_t* otherActorIds,
                                     int rigidContactMax, bool pointsInWorldSpace) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= rigidContactMax) return;
    int count = contactCount[0];
    if (tid >= count) return;

    int shapeA = shape0[tid];
    int shapeB = shape1[tid];
    if (shapeA == shapeB || shapeA < 0 || shapeB < 0) return;

    int rawBodyA = shapeBody[shapeA];
    int rawBodyB = shapeBody[shapeB];
    int mappedA = (rawBodyA < 0) ? worldBodyIdx : rawBodyA;
    int mappedB = (rawBodyB < 0) ? worldBodyIdx : rawBodyB;

    int sensorA = (mappedA >= 0 && mappedA < bodySensorMapSize) ? bodySensorMap[mappedA] : -1;
    int sensorB = (mappedB >= 0 && mappedB < bodySensorMapSize) ? bodySensorMap[mappedB] : -1;
    if (sensorA < 0 && sensorB < 0) return;

    float nx = normal[tid * 3 + 0], ny = normal[tid * 3 + 1], nz = normal[tid * 3 + 2];
    float fx = contactForce[tid * 3 + 0], fy = contactForce[tid * 3 + 1], fz = contactForce[tid * 3 + 2];
    float forceMag = sqrtf(fx * fx + fy * fy + fz * fz) * dtScale;

    float thk0 = thickness0[tid], thk1 = thickness1[tid];
    float nArr[3] = {nx, ny, nz};
    float p0[3] = {point0In[tid * 3], point0In[tid * 3 + 1], point0In[tid * 3 + 2]};
    float p1[3] = {point1In[tid * 3], point1In[tid * 3 + 1], point1In[tid * 3 + 2]};
    int bodyA = pointsInWorldSpace ? -1 : rawBodyA;
    int bodyB = pointsInWorldSpace ? -1 : rawBodyB;

    float wax, way, waz, wbx, wby, wbz;
    transformContactPoint(p0, bodyQ, bodyA, thk0, nArr, -1.0f, wax, way, waz);
    transformContactPoint(p1, bodyQ, bodyB, thk1, nArr, 1.0f, wbx, wby, wbz);

    float d = nx * (wax - wbx) + ny * (way - wby) + nz * (waz - wbz);
    float cpx = (wax + wbx) * 0.5f, cpy = (way + wby) * 0.5f, cpz = (waz + wbz) * 0.5f;

    if (sensorA >= 0) {
        uint32_t localIdx = atomicAdd(&outCounts[sensorA], 1u);
        int writeIdx = (int)startIndices[sensorA] + (int)localIdx;
        if (writeIdx < maxContactDataCount) {
            outForces[writeIdx] = -forceMag;
            outPoints[writeIdx * 3 + 0] = cpx; outPoints[writeIdx * 3 + 1] = cpy; outPoints[writeIdx * 3 + 2] = cpz;
            outNormals[writeIdx * 3 + 0] = -nx; outNormals[writeIdx * 3 + 1] = -ny; outNormals[writeIdx * 3 + 2] = -nz;
            outSeparations[writeIdx] = -d;
            otherActorIds[writeIdx] = (uint64_t)mappedB;
        }
    }
    if (sensorB >= 0) {
        uint32_t localIdx = atomicAdd(&outCounts[sensorB], 1u);
        int writeIdx = (int)startIndices[sensorB] + (int)localIdx;
        if (writeIdx < maxContactDataCount) {
            outForces[writeIdx] = forceMag;
            outPoints[writeIdx * 3 + 0] = cpx; outPoints[writeIdx * 3 + 1] = cpy; outPoints[writeIdx * 3 + 2] = cpz;
            outNormals[writeIdx * 3 + 0] = nx; outNormals[writeIdx * 3 + 1] = ny; outNormals[writeIdx * 3 + 2] = nz;
            outSeparations[writeIdx] = d;
            otherActorIds[writeIdx] = (uint64_t)mappedA;
        }
    }
}

// ---- Contact launch wrappers ----

bool launchExtractVec3FromSpatial(const float* spatialSrc, float* vec3Dst, int n, cudaStream_t stream) {
    if (n <= 0) return true;
    int numBlocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    extractVec3FromSpatialKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(spatialSrc, vec3Dst, n);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchNetContactForces(const int* contactCount, const int* shape0, const int* shape1,
                            const float* contactForce, const int* shapeBody,
                            const int* bodySensorMap, int bodySensorMapSize,
                            int worldBodyIdx, float dtScale,
                            float* netForces, int rigidContactMax, cudaStream_t stream) {
    if (rigidContactMax <= 0) return true;
    int numBlocks = (rigidContactMax + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    netContactForcesKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        contactCount, shape0, shape1, contactForce, shapeBody,
        bodySensorMap, bodySensorMapSize, worldBodyIdx, dtScale,
        netForces, rigidContactMax);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchContactForceMatrix(const int* contactCount, const int* shape0, const int* shape1,
                              const float* contactForce, const int* shapeBody,
                              const int* bodySensorMap, int bodySensorMapSize,
                              const int* bodyFilterMap, int numBodies,
                              int worldBodyIdx, float dtScale, int filterCount,
                              float* forceMatrix, int rigidContactMax, cudaStream_t stream) {
    if (rigidContactMax <= 0) return true;
    int numBlocks = (rigidContactMax + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    contactForceMatrixKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        contactCount, shape0, shape1, contactForce, shapeBody,
        bodySensorMap, bodySensorMapSize, bodyFilterMap, numBodies,
        worldBodyIdx, dtScale, filterCount, forceMatrix, rigidContactMax);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchCountContactsPerPair(const int* contactCount, const int* shape0, const int* shape1,
                                const int* shapeBody,
                                const int* bodySensorMap, int bodySensorMapSize,
                                const int* bodyFilterMap, int numBodies, int filterCount,
                                int worldBodyIdx,
                                uint32_t* counts, int rigidContactMax, cudaStream_t stream) {
    if (rigidContactMax <= 0) return true;
    int numBlocks = (rigidContactMax + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    countContactsPerPairKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        contactCount, shape0, shape1, shapeBody,
        bodySensorMap, bodySensorMapSize, bodyFilterMap, numBodies, filterCount,
        worldBodyIdx, counts, rigidContactMax);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchContactData(const int* contactCount, const int* shape0, const int* shape1,
                       const float* point0, const float* point1,
                       const float* normal, const float* contactForce,
                       const float* thickness0, const float* thickness1,
                       const int* shapeBody, const float* bodyQ,
                       const int* bodySensorMap, int bodySensorMapSize,
                       const int* bodyFilterMap, int numBodies, int filterCount,
                       int worldBodyIdx, float dtScale, int maxContactDataCount,
                       float* outForces, float* outPoints, float* outNormals,
                       float* outSeparations, uint32_t* outCounts, const uint32_t* startIndices,
                       int rigidContactMax, bool pointsInWorldSpace, cudaStream_t stream) {
    if (rigidContactMax <= 0) return true;
    int numBlocks = (rigidContactMax + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    contactDataKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        contactCount, shape0, shape1, point0, point1, normal, contactForce,
        thickness0, thickness1, shapeBody, bodyQ,
        bodySensorMap, bodySensorMapSize, bodyFilterMap, numBodies, filterCount,
        worldBodyIdx, dtScale, maxContactDataCount,
        outForces, outPoints, outNormals, outSeparations, outCounts, startIndices,
        rigidContactMax, pointsInWorldSpace);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchCountRawContactsPerSensor(const int* contactCount, const int* shape0, const int* shape1,
                                     const int* shapeBody,
                                     const int* bodySensorMap, int bodySensorMapSize,
                                     int worldBodyIdx,
                                     uint32_t* counts, int rigidContactMax, cudaStream_t stream) {
    if (rigidContactMax <= 0) return true;
    int numBlocks = (rigidContactMax + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    countRawContactsPerSensorKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        contactCount, shape0, shape1, shapeBody,
        bodySensorMap, bodySensorMapSize, worldBodyIdx,
        counts, rigidContactMax);
    CHECK_CUDA_LAUNCH();
    return true;
}

bool launchRawContactData(const int* contactCount, const int* shape0, const int* shape1,
                          const float* point0, const float* point1,
                          const float* normal, const float* contactForce,
                          const float* thickness0, const float* thickness1,
                          const int* shapeBody, const float* bodyQ,
                          const int* bodySensorMap, int bodySensorMapSize,
                          int worldBodyIdx, float dtScale, int maxContactDataCount,
                          float* outForces, float* outPoints, float* outNormals,
                          float* outSeparations, uint32_t* outCounts,
                          const uint32_t* startIndices, uint64_t* otherActorIds,
                          int rigidContactMax, bool pointsInWorldSpace, cudaStream_t stream) {
    if (rigidContactMax <= 0) return true;
    int numBlocks = (rigidContactMax + BLOCK_SIZE - 1) / BLOCK_SIZE;
    (void)cudaGetLastError();
    rawContactDataKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        contactCount, shape0, shape1, point0, point1, normal, contactForce,
        thickness0, thickness1, shapeBody, bodyQ,
        bodySensorMap, bodySensorMapSize,
        worldBodyIdx, dtScale, maxContactDataCount,
        outForces, outPoints, outNormals, outSeparations, outCounts,
        startIndices, otherActorIds, rigidContactMax, pointsInWorldSpace);
    CHECK_CUDA_LAUNCH();
    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
