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

#pragma once

// Device-agnostic tensor operations for the Newton tensor backend.
//
// Functions in this header are pure computation: they take raw pointers, device ordinals,
// and counts, and operate without any Python or GIL interaction. All GPU buffer allocation
// is the caller's responsibility; these functions never call cudaMalloc/cudaFree.

#include <cstddef>
#include <cstdint>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

/// Recompute inverse mass/inertia arrays from their forward counterparts.
/// Dispatches to CPU or GPU based on deviceOrdinal (-1 = CPU, >= 0 = GPU).
bool updateInverseMass(float* mass, float* inverseMass, int deviceOrdinal, size_t count);
bool updateInverseInertia(float* inertia, float* inverseInertia, int deviceOrdinal, size_t count);

// ---- CPU contact operations ----

void cpuExtractVec3FromSpatial(const float* spatialSrc, float* vec3Dst, int n);

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
                         int rigidContactMax);

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
                           int rigidContactMax);

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
                             int rigidContactMax);

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
                    bool pointsInWorldSpace = false);

void cpuCountRawContactsPerSensor(const int* contactCount,
                                  const int* shape0,
                                  const int* shape1,
                                  const int* shapeBody,
                                  const int* bodySensorMap,
                                  int bodySensorMapSize,
                                  int worldBodyIdx,
                                  uint32_t* counts,
                                  int rigidContactMax);

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
                       bool pointsInWorldSpace = false);

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
