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

#include "TensorOps.h"

#include "WarpCompat.h"
#include "gpu/CudaKernels.h"

#include <carb/logging/Log.h>

#include <cmath>
#include <cstring>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

// ---- Inverse mass/inertia recomputation ----

bool updateInverseMass(float* mass, float* inverseMass, int deviceOrdinal, size_t count)
{
    if (!mass || !inverseMass || count == 0)
        return false;

    if (deviceOrdinal < 0)
    {
        for (size_t i = 0; i < count; ++i)
        {
            float m = mass[i];
            inverseMass[i] = (m > 1e-8f) ? (1.0f / m) : 0.0f;
        }
        return true;
    }
    return launchUpdateInverseMass(mass, inverseMass, static_cast<int>(count));
}

bool updateInverseInertia(float* inertia, float* inverseInertia, int deviceOrdinal, size_t count)
{
    if (!inertia || !inverseInertia || count == 0)
        return false;

    auto* src = reinterpret_cast<wp::mat33*>(inertia);
    auto* dst = reinterpret_cast<wp::mat33*>(inverseInertia);

    if (deviceOrdinal < 0)
    {
        for (size_t i = 0; i < count; ++i)
        {
            wp::mat33& I = src[i];
            float a = I.data[0][0], b = I.data[0][1], c = I.data[0][2];
            float d = I.data[1][0], e = I.data[1][1], f = I.data[1][2];
            float g = I.data[2][0], h = I.data[2][1], k = I.data[2][2];
            float det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
            if (fabsf(det) > 1e-8f)
            {
                float inv = 1.0f / det;
                wp::mat33& out = dst[i];
                out.data[0][0] = (e * k - f * h) * inv;
                out.data[0][1] = (c * h - b * k) * inv;
                out.data[0][2] = (b * f - c * e) * inv;
                out.data[1][0] = (f * g - d * k) * inv;
                out.data[1][1] = (a * k - c * g) * inv;
                out.data[1][2] = (c * d - a * f) * inv;
                out.data[2][0] = (d * h - e * g) * inv;
                out.data[2][1] = (b * g - a * h) * inv;
                out.data[2][2] = (a * e - b * d) * inv;
            }
            else
            {
                wp::mat33& out = dst[i];
                for (int r = 0; r < 3; ++r)
                    for (int col = 0; col < 3; ++col)
                        out.data[r][col] = 0.0f;
            }
        }
        return true;
    }
    return launchUpdateInverseInertia(src, dst, static_cast<int>(count));
}

// ---- CPU contact operations ----

static inline void cpuResolveContact(int shapeA,
                                     int shapeB,
                                     const int* shapeBody,
                                     const int* bodySensorMap,
                                     int bodySensorMapSize,
                                     int worldBodyIdx,
                                     int& rawBodyA,
                                     int& rawBodyB,
                                     int& mappedA,
                                     int& mappedB,
                                     int& sensorA,
                                     int& sensorB)
{
    rawBodyA = shapeBody[shapeA];
    rawBodyB = shapeBody[shapeB];
    mappedA = (rawBodyA < 0) ? worldBodyIdx : rawBodyA;
    mappedB = (rawBodyB < 0) ? worldBodyIdx : rawBodyB;
    sensorA = (mappedA >= 0 && mappedA < bodySensorMapSize) ? bodySensorMap[mappedA] : -1;
    sensorB = (mappedB >= 0 && mappedB < bodySensorMapSize) ? bodySensorMap[mappedB] : -1;
}

static inline void cpuTransformPoint(const float* point,
                                     const float* bodyQ,
                                     int rawBody,
                                     float thickness,
                                     const float* normal,
                                     float sign,
                                     float& wx,
                                     float& wy,
                                     float& wz)
{
    float px = point[0], py = point[1], pz = point[2];
    if (rawBody >= 0)
    {
        const float* t = bodyQ + rawBody * 7;
        float tx = t[0], ty = t[1], tz = t[2];
        float qx = t[3], qy = t[4], qz = t[5], qw = t[6];
        float ax = qw * px + qy * pz - qz * py;
        float ay = qw * py + qz * px - qx * pz;
        float az = qw * pz + qx * py - qy * px;
        float aw = -(qx * px + qy * py + qz * pz);
        wx = tx + ax * qw - aw * qx + (ay * qz - az * qy) + sign * thickness * normal[0];
        wy = ty + ay * qw - aw * qy + (az * qx - ax * qz) + sign * thickness * normal[1];
        wz = tz + az * qw - aw * qz + (ax * qy - ay * qx) + sign * thickness * normal[2];
    }
    else
    {
        wx = px + sign * thickness * normal[0];
        wy = py + sign * thickness * normal[1];
        wz = pz + sign * thickness * normal[2];
    }
}

void cpuExtractVec3FromSpatial(const float* spatialSrc, float* vec3Dst, int n)
{
    for (int i = 0; i < n; ++i)
    {
        vec3Dst[i * 3 + 0] = spatialSrc[i * 6 + 0];
        vec3Dst[i * 3 + 1] = spatialSrc[i * 6 + 1];
        vec3Dst[i * 3 + 2] = spatialSrc[i * 6 + 2];
    }
}

void cpuNetContactForces(const int* contactCount,
                         const int* shape0,
                         const int* shape1,
                         const float* contactForce,
                         const int* shapeBody,
                         const int* bodySensorMap,
                         int bodySensorMapSize,
                         int worldBodyIdx,
                         float dtScale,
                         float* netForces,
                         int rigidContactMax)
{
    int count = contactCount[0];
    int limit = (count < rigidContactMax) ? count : rigidContactMax;
    for (int tid = 0; tid < limit; ++tid)
    {
        int sA = shape0[tid], sB = shape1[tid];
        if (sA == sB || sA < 0 || sB < 0)
            continue;
        int rawA, rawB, mA, mB, senA, senB;
        cpuResolveContact(
            sA, sB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx, rawA, rawB, mA, mB, senA, senB);
        if (senA < 0 && senB < 0)
            continue;
        float fx = contactForce[tid * 3 + 0] * dtScale;
        float fy = contactForce[tid * 3 + 1] * dtScale;
        float fz = contactForce[tid * 3 + 2] * dtScale;
        if (senA >= 0)
        {
            netForces[senA * 3 + 0] += fx;
            netForces[senA * 3 + 1] += fy;
            netForces[senA * 3 + 2] += fz;
        }
        if (senB >= 0)
        {
            netForces[senB * 3 + 0] -= fx;
            netForces[senB * 3 + 1] -= fy;
            netForces[senB * 3 + 2] -= fz;
        }
    }
}

void cpuContactForceMatrix(const int* contactCount,
                           const int* shape0,
                           const int* shape1,
                           const float* contactForce,
                           const int* shapeBody,
                           const int* bodySensorMap,
                           int bodySensorMapSize,
                           const int* bodyFilterMap,
                           int numBodies,
                           int worldBodyIdx,
                           float dtScale,
                           int filterCount,
                           float* forceMatrix,
                           int rigidContactMax)
{
    int count = contactCount[0];
    int limit = (count < rigidContactMax) ? count : rigidContactMax;
    for (int tid = 0; tid < limit; ++tid)
    {
        int sA = shape0[tid], sB = shape1[tid];
        if (sA == sB || sA < 0 || sB < 0)
            continue;
        int rawA, rawB, mA, mB, senA, senB;
        cpuResolveContact(
            sA, sB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx, rawA, rawB, mA, mB, senA, senB);
        if (senA < 0 && senB < 0)
            continue;
        float fx = contactForce[tid * 3 + 0] * dtScale;
        float fy = contactForce[tid * 3 + 1] * dtScale;
        float fz = contactForce[tid * 3 + 2] * dtScale;
        if (senA >= 0 && mB >= 0 && mB < numBodies)
        {
            int fi = bodyFilterMap[senA * numBodies + mB];
            if (fi >= 0)
            {
                int base = (senA * filterCount + fi) * 3;
                forceMatrix[base + 0] += fx;
                forceMatrix[base + 1] += fy;
                forceMatrix[base + 2] += fz;
            }
        }
        if (senB >= 0 && mA >= 0 && mA < numBodies)
        {
            int fi = bodyFilterMap[senB * numBodies + mA];
            if (fi >= 0)
            {
                int base = (senB * filterCount + fi) * 3;
                forceMatrix[base + 0] -= fx;
                forceMatrix[base + 1] -= fy;
                forceMatrix[base + 2] -= fz;
            }
        }
    }
}

void cpuCountContactsPerPair(const int* contactCount,
                             const int* shape0,
                             const int* shape1,
                             const int* shapeBody,
                             const int* bodySensorMap,
                             int bodySensorMapSize,
                             const int* bodyFilterMap,
                             int numBodies,
                             int filterCount,
                             int worldBodyIdx,
                             uint32_t* counts,
                             int rigidContactMax)
{
    int count = contactCount[0];
    int limit = (count < rigidContactMax) ? count : rigidContactMax;
    for (int tid = 0; tid < limit; ++tid)
    {
        int sA = shape0[tid], sB = shape1[tid];
        if (sA == sB || sA < 0 || sB < 0)
            continue;
        int rawA, rawB, mA, mB, senA, senB;
        cpuResolveContact(
            sA, sB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx, rawA, rawB, mA, mB, senA, senB);
        if (senA < 0 && senB < 0)
            continue;
        if (senA >= 0 && mB >= 0 && mB < numBodies)
        {
            int fi = bodyFilterMap[senA * numBodies + mB];
            if (fi >= 0 && fi < filterCount)
                counts[senA * filterCount + fi]++;
        }
        if (senB >= 0 && mA >= 0 && mA < numBodies)
        {
            int fi = bodyFilterMap[senB * numBodies + mA];
            if (fi >= 0 && fi < filterCount)
                counts[senB * filterCount + fi]++;
        }
    }
}

void cpuContactData(const int* contactCount,
                    const int* shape0,
                    const int* shape1,
                    const float* point0,
                    const float* point1,
                    const float* normal,
                    const float* contactForce,
                    const float* thickness0,
                    const float* thickness1,
                    const int* shapeBody,
                    const float* bodyQ,
                    const int* bodySensorMap,
                    int bodySensorMapSize,
                    const int* bodyFilterMap,
                    int numBodies,
                    int filterCount,
                    int worldBodyIdx,
                    float dtScale,
                    int maxContactDataCount,
                    float* outForces,
                    float* outPoints,
                    float* outNormals,
                    float* outSeparations,
                    uint32_t* outCounts,
                    const uint32_t* startIndices,
                    int rigidContactMax,
                    bool pointsInWorldSpace)
{
    int count = contactCount[0];
    int limit = (count < rigidContactMax) ? count : rigidContactMax;
    for (int tid = 0; tid < limit; ++tid)
    {
        int sA = shape0[tid], sB = shape1[tid];
        if (sA == sB || sA < 0 || sB < 0)
            continue;
        int rawA, rawB, mA, mB, senA, senB;
        cpuResolveContact(
            sA, sB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx, rawA, rawB, mA, mB, senA, senB);
        if (senA < 0 && senB < 0)
            continue;

        float nx = normal[tid * 3], ny = normal[tid * 3 + 1], nz = normal[tid * 3 + 2];
        float fx = contactForce[tid * 3], fy = contactForce[tid * 3 + 1], fz = contactForce[tid * 3 + 2];
        float forceMag = sqrtf(fx * fx + fy * fy + fz * fz) * dtScale;
        float nArr[3] = { nx, ny, nz };
        float p0[3] = { point0[tid * 3], point0[tid * 3 + 1], point0[tid * 3 + 2] };
        float p1[3] = { point1[tid * 3], point1[tid * 3 + 1], point1[tid * 3 + 2] };
        int bodyA = pointsInWorldSpace ? -1 : rawA;
        int bodyB = pointsInWorldSpace ? -1 : rawB;
        float wax, way, waz, wbx, wby, wbz;
        cpuTransformPoint(p0, bodyQ, bodyA, thickness0[tid], nArr, -1.0f, wax, way, waz);
        cpuTransformPoint(p1, bodyQ, bodyB, thickness1[tid], nArr, 1.0f, wbx, wby, wbz);
        float d = nx * (wax - wbx) + ny * (way - wby) + nz * (waz - wbz);
        float cpx = (wax + wbx) * 0.5f, cpy = (way + wby) * 0.5f, cpz = (waz + wbz) * 0.5f;

        if (senA >= 0 && mB >= 0 && mB < numBodies)
        {
            int fi = bodyFilterMap[senA * numBodies + mB];
            if (fi >= 0 && fi < filterCount)
            {
                uint32_t localIdx = outCounts[senA * filterCount + fi]++;
                int wi = (int)startIndices[senA * filterCount + fi] + (int)localIdx;
                if (wi < maxContactDataCount)
                {
                    outForces[wi] = -forceMag;
                    outPoints[wi * 3] = cpx;
                    outPoints[wi * 3 + 1] = cpy;
                    outPoints[wi * 3 + 2] = cpz;
                    outNormals[wi * 3] = -nx;
                    outNormals[wi * 3 + 1] = -ny;
                    outNormals[wi * 3 + 2] = -nz;
                    outSeparations[wi] = -d;
                }
            }
        }
        if (senB >= 0 && mA >= 0 && mA < numBodies)
        {
            int fi = bodyFilterMap[senB * numBodies + mA];
            if (fi >= 0 && fi < filterCount)
            {
                uint32_t localIdx = outCounts[senB * filterCount + fi]++;
                int wi = (int)startIndices[senB * filterCount + fi] + (int)localIdx;
                if (wi < maxContactDataCount)
                {
                    outForces[wi] = forceMag;
                    outPoints[wi * 3] = cpx;
                    outPoints[wi * 3 + 1] = cpy;
                    outPoints[wi * 3 + 2] = cpz;
                    outNormals[wi * 3] = nx;
                    outNormals[wi * 3 + 1] = ny;
                    outNormals[wi * 3 + 2] = nz;
                    outSeparations[wi] = d;
                }
            }
        }
    }
}

void cpuCountRawContactsPerSensor(const int* contactCount,
                                  const int* shape0,
                                  const int* shape1,
                                  const int* shapeBody,
                                  const int* bodySensorMap,
                                  int bodySensorMapSize,
                                  int worldBodyIdx,
                                  uint32_t* counts,
                                  int rigidContactMax)
{
    int count = contactCount[0];
    int limit = (count < rigidContactMax) ? count : rigidContactMax;
    for (int tid = 0; tid < limit; ++tid)
    {
        int sA = shape0[tid], sB = shape1[tid];
        if (sA == sB || sA < 0 || sB < 0)
            continue;
        int rawA, rawB, mA, mB, senA, senB;
        cpuResolveContact(
            sA, sB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx, rawA, rawB, mA, mB, senA, senB);
        if (senA >= 0)
            counts[senA]++;
        if (senB >= 0)
            counts[senB]++;
    }
}

void cpuRawContactData(const int* contactCount,
                       const int* shape0,
                       const int* shape1,
                       const float* point0,
                       const float* point1,
                       const float* normal,
                       const float* contactForce,
                       const float* thickness0,
                       const float* thickness1,
                       const int* shapeBody,
                       const float* bodyQ,
                       const int* bodySensorMap,
                       int bodySensorMapSize,
                       int worldBodyIdx,
                       float dtScale,
                       int maxContactDataCount,
                       float* outForces,
                       float* outPoints,
                       float* outNormals,
                       float* outSeparations,
                       uint32_t* outCounts,
                       const uint32_t* startIndices,
                       uint64_t* otherActorIds,
                       int rigidContactMax,
                       bool pointsInWorldSpace)
{
    int count = contactCount[0];
    int limit = (count < rigidContactMax) ? count : rigidContactMax;
    for (int tid = 0; tid < limit; ++tid)
    {
        int sA = shape0[tid], sB = shape1[tid];
        if (sA == sB || sA < 0 || sB < 0)
            continue;
        int rawA, rawB, mA, mB, senA, senB;
        cpuResolveContact(
            sA, sB, shapeBody, bodySensorMap, bodySensorMapSize, worldBodyIdx, rawA, rawB, mA, mB, senA, senB);
        if (senA < 0 && senB < 0)
            continue;

        float nx = normal[tid * 3], ny = normal[tid * 3 + 1], nz = normal[tid * 3 + 2];
        float fx = contactForce[tid * 3], fy = contactForce[tid * 3 + 1], fz = contactForce[tid * 3 + 2];
        float forceMag = sqrtf(fx * fx + fy * fy + fz * fz) * dtScale;
        float nArr[3] = { nx, ny, nz };
        float p0[3] = { point0[tid * 3], point0[tid * 3 + 1], point0[tid * 3 + 2] };
        float p1[3] = { point1[tid * 3], point1[tid * 3 + 1], point1[tid * 3 + 2] };
        int bodyA = pointsInWorldSpace ? -1 : rawA;
        int bodyB = pointsInWorldSpace ? -1 : rawB;
        float wax, way, waz, wbx, wby, wbz;
        cpuTransformPoint(p0, bodyQ, bodyA, thickness0[tid], nArr, -1.0f, wax, way, waz);
        cpuTransformPoint(p1, bodyQ, bodyB, thickness1[tid], nArr, 1.0f, wbx, wby, wbz);
        float d = nx * (wax - wbx) + ny * (way - wby) + nz * (waz - wbz);
        float cpx = (wax + wbx) * 0.5f, cpy = (way + wby) * 0.5f, cpz = (waz + wbz) * 0.5f;

        if (senA >= 0)
        {
            uint32_t localIdx = outCounts[senA]++;
            int wi = (int)startIndices[senA] + (int)localIdx;
            if (wi < maxContactDataCount)
            {
                outForces[wi] = -forceMag;
                outPoints[wi * 3] = cpx;
                outPoints[wi * 3 + 1] = cpy;
                outPoints[wi * 3 + 2] = cpz;
                outNormals[wi * 3] = -nx;
                outNormals[wi * 3 + 1] = -ny;
                outNormals[wi * 3 + 2] = -nz;
                outSeparations[wi] = -d;
                otherActorIds[wi] = (uint64_t)mB;
            }
        }
        if (senB >= 0)
        {
            uint32_t localIdx = outCounts[senB]++;
            int wi = (int)startIndices[senB] + (int)localIdx;
            if (wi < maxContactDataCount)
            {
                outForces[wi] = forceMag;
                outPoints[wi * 3] = cpx;
                outPoints[wi * 3 + 1] = cpy;
                outPoints[wi * 3 + 2] = cpz;
                outNormals[wi * 3] = nx;
                outNormals[wi * 3 + 1] = ny;
                outNormals[wi * 3 + 2] = nz;
                outSeparations[wi] = d;
                otherActorIds[wi] = (uint64_t)mA;
            }
        }
    }
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
