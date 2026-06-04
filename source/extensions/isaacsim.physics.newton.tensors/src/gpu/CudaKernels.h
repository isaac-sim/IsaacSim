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

/// CUDA kernel launch declarations for Newton tensor operations.
///
/// Every function in this header launches one or more CUDA kernels on the
/// given ``stream`` and returns ``true`` on a successful launch (``false`` on
/// an API error). No device memory is allocated here: callers must provide all
/// device-resident buffers with the required sizes. Indexing buffers use ``-1``
/// as a sentinel to mark "skip": the output for that slot is zero-filled.

#include "utils/WarpCompat.h"

#include <cstdint>
#include <cuda_runtime.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

/// Gather scalar floats by index, writing zero for ``-1`` entries.
///
/// ``dst[i] = devIndices[i] >= 0 ? src[devIndices[i]] : 0.0f`` for
/// ``i`` in ``[0, numIndices)``.
///
/// @param src Device pointer to the source scalar array.
/// @param dst Device pointer to the destination array, length ``numIndices``.
/// @param devIndices Device index array, length ``numIndices``.
/// @param numIndices Number of gather slots.
/// @param stream CUDA stream to launch on.
/// @return ``true`` if the launch succeeded.
bool launchGatherFloat(const float* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream = nullptr);

/// Gather :class:`wp::transform` entries (7 floats each) into a packed output.
///
/// @param src Device pointer to an array of :class:`wp::transform`.
/// @param dst Device pointer to ``[numIndices, 7]`` floats.
/// @param devIndices Device index array, length ``numIndices``.
/// @param numIndices Number of gather slots.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchGatherTransform(
    const wp::transform* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream = nullptr);

/// Gather :class:`wp::spatial_vector` entries (6 floats each).
///
/// @param src Device pointer to an array of :class:`wp::spatial_vector`.
/// @param dst Device pointer to ``[numIndices, 6]`` floats.
/// @param devIndices Device index array, length ``numIndices``.
/// @param numIndices Number of gather slots.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchGatherSpatialVector(
    const wp::spatial_vector* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream = nullptr);

/// Gather :class:`wp::mat33` entries (9 floats each, row-major).
///
/// @param src Device pointer to an array of :class:`wp::mat33`.
/// @param dst Device pointer to ``[numIndices, 9]`` floats.
/// @param devIndices Device index array, length ``numIndices``.
/// @param numIndices Number of gather slots.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchGatherMat33(
    const wp::mat33* src, float* dst, const int* devIndices, int numIndices, cudaStream_t stream = nullptr);

/// Gather center-of-mass vec3 entries and pair with cached orientation quaternions.
///
/// Writes ``{px, py, pz, qx, qy, qz, qw}`` per selected entry where the
/// orientation is read from ``cachedOrientation`` (4 floats per slot).
///
/// @param src Device pointer to source vec3 array.
/// @param dst Device pointer to ``[numIndices, 7]`` floats.
/// @param devIndices Device index array, length ``numIndices``.
/// @param numIndices Number of gather slots.
/// @param cachedOrientation Device pointer to ``[numIndices, 4]`` cached quaternion floats.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchGatherCenterOfMass(const wp::vec3* src,
                              float* dst,
                              const int* devIndices,
                              int numIndices,
                              const float* cachedOrientation,
                              cudaStream_t stream = nullptr);

/// Extract orientation quaternions from a COM tensor and scatter to a flat cache.
///
/// For each element, reads 4 floats starting at offset 3 in the source stride
/// and writes them contiguously to ``dst``.
///
/// @param src Device source buffer with stride ``srcStride`` per element.
/// @param dst Device orientation cache, 4 floats per element.
/// @param devArtiIndices Optional index array; ``nullptr`` for identity.
/// @param count Number of selected articulations/bodies.
/// @param elemPerSlot Elements per slot (1 for rigid body, maxLinks for articulation).
/// @param srcStride Float stride of each source element (7 for COM).
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchScatterComOrientation(const float* src,
                                 float* dst,
                                 const int* devArtiIndices,
                                 int count,
                                 int elemPerSlot,
                                 int srcStride,
                                 cudaStream_t stream = nullptr);

/// Interleaved gather from two parallel source arrays.
///
/// ``dst[2*i]   = srcA[devIndices[i]]``
/// ``dst[2*i+1] = srcB[devIndices[i]]``
///
/// @param srcA Device pointer to the first source array.
/// @param srcB Device pointer to the second source array.
/// @param dst Device pointer to ``[numIndices, 2]`` floats.
/// @param devIndices Device index array, length ``numIndices``.
/// @param numIndices Number of gather slots.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchGatherPairedFloat(const float* srcA,
                             const float* srcB,
                             float* dst,
                             const int* devIndices,
                             int numIndices,
                             cudaStream_t stream = nullptr);

/// Compute per-element inverse mass on device.
///
/// ``inverseMass[i] = mass[i] > eps ? 1.0f / mass[i] : 0.0f``.
///
/// @param mass Device pointer to the mass array.
/// @param inverseMass Device pointer to the output array; may alias ``mass``.
/// @param n Element count.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchUpdateInverseMass(const float* mass, float* inverseMass, int n, cudaStream_t stream = nullptr);

/// Compute per-element inverse inertia matrix on device.
///
/// Inverts each 3x3 inertia tensor in place; writes zero for singular inputs.
///
/// @param inertia Device pointer to an array of :class:`wp::mat33`.
/// @param inverseInertia Device pointer to the output array; may alias ``inertia``.
/// @param n Element count.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchUpdateInverseInertia(const wp::mat33* inertia, wp::mat33* inverseInertia, int n, cudaStream_t stream = nullptr);

/// Fused indexed DOF scatter: writes view-scoped DOF values into a flat model DOF array.
///
/// For each view articulation ``a`` and DOF slot ``d``, writes
/// ``dst[ devDofMapping[a * maxDofs + d] ] = src[a * maxDofs + d]`` when the
/// mapping entry is non-negative. ``devArtiIndices`` may be ``nullptr`` to use
/// the identity ``0..numArti-1``.
///
/// @param src Device source buffer, shape ``[numArti, maxDofs]``.
/// @param dst Device destination buffer (flat model DOF array).
/// @param devArtiIndices Optional device articulation index array.
/// @param devDofMapping Device DOF mapping, shape ``[numArti * maxDofs]``.
/// @param numArti Number of selected articulations.
/// @param maxDofs Maximum DOF count per articulation.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchFusedDofScatter(const float* src,
                           float* dst,
                           const int* devArtiIndices,
                           const int* devDofMapping,
                           int numArti,
                           int maxDofs,
                           cudaStream_t stream = nullptr);

/// Fused indexed DOF scatter that writes the same value into two destinations.
///
/// Useful for paired ``[lower, upper]`` limit arrays.
///
/// @param src Device source buffer, shape ``[numArti, maxDofs]``.
/// @param dstA Device destination A (same size as the model DOF array).
/// @param dstB Device destination B.
/// @param devArtiIndices Optional device articulation index array.
/// @param devDofMapping Device DOF mapping, shape ``[numArti * maxDofs]``.
/// @param numArti Number of selected articulations.
/// @param maxDofs Maximum DOF count per articulation.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchFusedPairedDofScatter(const float* src,
                                 float* dstA,
                                 float* dstB,
                                 const int* devArtiIndices,
                                 const int* devDofMapping,
                                 int numArti,
                                 int maxDofs,
                                 cudaStream_t stream = nullptr);

/// Fused link scatter for articulation link properties.
///
/// Writes ``numComponents`` components from ``src`` to ``dst``, indexing
/// selected articulations via ``devArtiIndices`` (or identity if null) and
/// individual links via ``devLinkMapping``.
///
/// @param src Device source buffer, stride ``srcElemSize``.
/// @param dst Device destination, stride ``dstElemSize``, offset ``dstElemOffset``.
/// @param devArtiIndices Optional device articulation index array.
/// @param devLinkMapping Device link mapping, shape ``[numArti * maxLinks]``.
/// @param numArti Number of selected articulations.
/// @param maxLinks Maximum link count per articulation.
/// @param srcElemSize Stride (float count) of source elements.
/// @param dstElemSize Stride (float count) of destination elements.
/// @param dstElemOffset Starting float offset inside each destination element.
/// @param numComponents Number of contiguous floats copied per link.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchFusedLinkScatter(const float* src,
                            float* dst,
                            const int* devArtiIndices,
                            const int* devLinkMapping,
                            int numArti,
                            int maxLinks,
                            int srcElemSize,
                            int dstElemSize,
                            int dstElemOffset,
                            int numComponents,
                            cudaStream_t stream = nullptr);

/// Same as :func:`launchFusedLinkScatter` but performs an atomic add into ``dst``.
///
/// Used for force/torque accumulation so that concurrent writes to the same
/// link do not race.
///
/// @param src Device source buffer.
/// @param dst Device destination buffer, accumulated via atomic add.
/// @param devArtiIndices Optional device articulation index array.
/// @param devLinkMapping Device link mapping, shape ``[numArti * maxLinks]``.
/// @param numArti Number of selected articulations.
/// @param maxLinks Maximum link count per articulation.
/// @param srcElemSize Stride (float count) of source elements.
/// @param dstElemSize Stride (float count) of destination elements.
/// @param dstElemOffset Starting float offset inside each destination element.
/// @param numComponents Number of contiguous floats added per link.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchFusedLinkAdd(const float* src,
                        float* dst,
                        const int* devArtiIndices,
                        const int* devLinkMapping,
                        int numArti,
                        int maxLinks,
                        int srcElemSize,
                        int dstElemSize,
                        int dstElemOffset,
                        int numComponents,
                        cudaStream_t stream = nullptr);

/// Fused scatter for articulation root properties (transforms, velocities).
///
/// @param src Device source buffer, shape ``[numArti, elemSize]``.
/// @param dst Device destination buffer.
/// @param devArtiIndices Optional device articulation index array.
/// @param devRootMapping Device mapping from view articulation index to model root slot.
/// @param numArti Number of selected articulations.
/// @param elemSize Number of floats per root element.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchFusedRootScatter(const float* src,
                            float* dst,
                            const int* devArtiIndices,
                            const int* devRootMapping,
                            int numArti,
                            int elemSize,
                            cudaStream_t stream = nullptr);

/// Fused scatter for articulation root into a flat joint-qd-style layout.
///
/// Differs from :func:`launchFusedRootScatter` in that ``devRootFlatMapping``
/// references flat float offsets inside ``dst`` rather than element slots.
///
/// @param src Device source buffer.
/// @param dst Device destination buffer (flat model float array).
/// @param devArtiIndices Optional device articulation index array.
/// @param devRootFlatMapping Device mapping, shape ``[numArti * elemSize]``.
/// @param numArti Number of selected articulations.
/// @param elemSize Number of floats per root element.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchFusedRootFlatScatter(const float* src,
                                float* dst,
                                const int* devArtiIndices,
                                const int* devRootFlatMapping,
                                int numArti,
                                int elemSize,
                                cudaStream_t stream = nullptr);

/// Device-to-device byte copy of a flat ``uint8`` buffer.
///
/// @param src Device source pointer.
/// @param dst Device destination pointer.
/// @param n Element count.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchCopyUint8(const uint8_t* src, uint8_t* dst, int n, cudaStream_t stream = nullptr);

// ---- Contact kernels ----

/// Extract the linear component of each :class:`wp::spatial_vector` into a vec3.
///
/// @param spatialSrc Device source, shape ``[n, 6]``.
/// @param vec3Dst Device output, shape ``[n, 3]``.
/// @param n Number of spatial vectors.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchExtractVec3FromSpatial(const float* spatialSrc, float* vec3Dst, int n, cudaStream_t stream = nullptr);

/// Accumulate per-sensor net contact forces.
///
/// Sums contact forces whose shape bodies are mapped to a sensor by
/// ``bodySensorMap`` (``-1`` = ignore). Output is scaled by ``dtScale``.
///
/// @param contactCount Device pointer to the physics contact count (single int).
/// @param shape0 Device array, contact pair shape A indices.
/// @param shape1 Device array, contact pair shape B indices.
/// @param contactForce Device array, shape ``[rigidContactMax, 3]``.
/// @param shapeBody Device array mapping shape index to body index.
/// @param bodySensorMap Device array mapping body index to sensor index.
/// @param bodySensorMapSize Length of ``bodySensorMap``.
/// @param worldBodyIdx Sentinel body index representing the static world; pairs
///        touching this body are still counted as sensor contacts.
/// @param dtScale Multiplicative scale applied to each force.
/// @param netForces Device output, shape ``[numSensors, 3]``.
/// @param rigidContactMax Capacity of the contact arrays.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchNetContactForces(const int* contactCount,
                            const int* shape0,
                            const int* shape1,
                            const float* contactForce,
                            const int* shapeBody,
                            const int* bodySensorMap,
                            int bodySensorMapSize,
                            int worldBodyIdx,
                            float dtScale,
                            float* netForces,
                            int rigidContactMax,
                            cudaStream_t stream = nullptr);

/// Accumulate per-(sensor, filter) contact forces into a dense matrix.
///
/// Output ``forceMatrix`` has shape ``[numSensors, filterCount, 3]``.
///
/// @param contactCount Device pointer to the physics contact count.
/// @param shape0 Device array, contact pair shape A indices.
/// @param shape1 Device array, contact pair shape B indices.
/// @param contactForce Device array, shape ``[rigidContactMax, 3]``.
/// @param shapeBody Device array mapping shape index to body index.
/// @param bodySensorMap Device array mapping body index to sensor index.
/// @param bodySensorMapSize Length of ``bodySensorMap``.
/// @param bodyFilterMap Device array mapping ``(sensor, body)`` to filter index.
/// @param numBodies Total body count in the model.
/// @param worldBodyIdx Sentinel body index representing the static world.
/// @param dtScale Multiplicative scale applied to each force.
/// @param filterCount Number of filters per sensor.
/// @param forceMatrix Device output, shape ``[numSensors, filterCount, 3]``.
/// @param rigidContactMax Capacity of the contact arrays.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchContactForceMatrix(const int* contactCount,
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
                              int rigidContactMax,
                              cudaStream_t stream = nullptr);

/// Count the number of contacts per ``(sensor, filter)`` pair.
///
/// Populates ``counts`` with shape ``[numSensors * filterCount]``; each cell
/// holds the number of contacts that would be written by
/// :func:`launchContactData` for that pair.
///
/// @param contactCount Device pointer to the physics contact count.
/// @param shape0 Device array, contact pair shape A indices.
/// @param shape1 Device array, contact pair shape B indices.
/// @param shapeBody Device array mapping shape index to body index.
/// @param bodySensorMap Device array mapping body index to sensor index.
/// @param bodySensorMapSize Length of ``bodySensorMap``.
/// @param bodyFilterMap Device array mapping ``(sensor, body)`` to filter index.
/// @param numBodies Total body count in the model.
/// @param filterCount Number of filters per sensor.
/// @param worldBodyIdx Sentinel body index representing the static world.
/// @param counts Device output counts, shape ``[numSensors * filterCount]``.
/// @param rigidContactMax Capacity of the contact arrays.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchCountContactsPerPair(const int* contactCount,
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
                                int rigidContactMax,
                                cudaStream_t stream = nullptr);

/// Emit per-contact data (force, point, normal, separation) for each sensor/filter pair.
///
/// Uses ``startIndices`` (exclusive prefix sum over the counts returned by
/// :func:`launchCountContactsPerPair`) to know where each pair's slice begins
/// inside the output arrays. ``outCounts`` is written with the actual written
/// count per pair, clipped to ``maxContactDataCount``.
///
/// @param contactCount Device pointer to the physics contact count.
/// @param shape0 Device array, contact pair shape A indices.
/// @param shape1 Device array, contact pair shape B indices.
/// @param point0 Device array, contact points on shape A (``[rigidContactMax, 3]``).
/// @param point1 Device array, contact points on shape B.
/// @param normal Device array, contact normals.
/// @param contactForce Device array, contact forces.
/// @param thickness0 Device array, thicknesses of shape A.
/// @param thickness1 Device array, thicknesses of shape B.
/// @param shapeBody Device array mapping shape index to body index.
/// @param bodyQ Device array of body transforms (7 floats each).
/// @param bodySensorMap Device array mapping body index to sensor index.
/// @param bodySensorMapSize Length of ``bodySensorMap``.
/// @param bodyFilterMap Device array mapping ``(sensor, body)`` to filter index.
/// @param numBodies Total body count in the model.
/// @param filterCount Number of filters per sensor.
/// @param worldBodyIdx Sentinel body index representing the static world.
/// @param dtScale Multiplicative scale applied to each force.
/// @param maxContactDataCount Maximum contacts stored per pair.
/// @param outForces Device output, shape ``[numSensors, filterCount, maxContactDataCount, 3]``.
/// @param outPoints Device output of the same shape (world-frame points).
/// @param outNormals Device output of the same shape.
/// @param outSeparations Device output, shape ``[numSensors, filterCount, maxContactDataCount]``.
/// @param outCounts Device output count per pair, shape ``[numSensors * filterCount]``.
/// @param startIndices Device exclusive prefix-sum of counts.
/// @param rigidContactMax Capacity of the contact arrays.
/// @param pointsInWorldSpace If ``true``, output points are expressed in world space; otherwise in the local frame of
/// the corresponding body.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchContactData(const int* contactCount,
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
                       bool pointsInWorldSpace = false,
                       cudaStream_t stream = nullptr);

/// Count raw contacts per sensor (ignoring filters).
///
/// @param contactCount Device pointer to the physics contact count.
/// @param shape0 Device array, contact pair shape A indices.
/// @param shape1 Device array, contact pair shape B indices.
/// @param shapeBody Device array mapping shape index to body index.
/// @param bodySensorMap Device array mapping body index to sensor index.
/// @param bodySensorMapSize Length of ``bodySensorMap``.
/// @param worldBodyIdx Sentinel body index representing the static world.
/// @param counts Device output counts, length ``numSensors``.
/// @param rigidContactMax Capacity of the contact arrays.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchCountRawContactsPerSensor(const int* contactCount,
                                     const int* shape0,
                                     const int* shape1,
                                     const int* shapeBody,
                                     const int* bodySensorMap,
                                     int bodySensorMapSize,
                                     int worldBodyIdx,
                                     uint32_t* counts,
                                     int rigidContactMax,
                                     cudaStream_t stream = nullptr);

/// Emit raw per-sensor contact data with the "other actor" identifier.
///
/// Same as :func:`launchContactData` but tags each contact with the body index
/// of the shape on the non-sensor side, letting Python code resolve the actor
/// it was colliding with. Filter information is not used.
///
/// @param contactCount Device pointer to the physics contact count.
/// @param shape0 Device array, contact pair shape A indices.
/// @param shape1 Device array, contact pair shape B indices.
/// @param point0 Device array, contact points on shape A.
/// @param point1 Device array, contact points on shape B.
/// @param normal Device array, contact normals.
/// @param contactForce Device array, contact forces.
/// @param thickness0 Device array, thicknesses of shape A.
/// @param thickness1 Device array, thicknesses of shape B.
/// @param shapeBody Device array mapping shape index to body index.
/// @param bodyQ Device array of body transforms.
/// @param bodySensorMap Device array mapping body index to sensor index.
/// @param bodySensorMapSize Length of ``bodySensorMap``.
/// @param worldBodyIdx Sentinel body index representing the static world.
/// @param dtScale Multiplicative scale applied to each force.
/// @param maxContactDataCount Maximum contacts stored per sensor.
/// @param outForces Device output forces per sensor slot.
/// @param outPoints Device output points (world frame).
/// @param outNormals Device output normals.
/// @param outSeparations Device output separations.
/// @param outCounts Device output count per sensor.
/// @param startIndices Device exclusive prefix-sum of per-sensor counts.
/// @param otherActorIds Device output, other-side body indices per contact.
/// @param rigidContactMax Capacity of the contact arrays.
/// @param pointsInWorldSpace If ``true``, output points are expressed in world space; otherwise in the local frame of
/// the corresponding body.
/// @param stream CUDA stream.
/// @return ``true`` if the launch succeeded.
bool launchRawContactData(const int* contactCount,
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
                          bool pointsInWorldSpace = false,
                          cudaStream_t stream = nullptr);

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
