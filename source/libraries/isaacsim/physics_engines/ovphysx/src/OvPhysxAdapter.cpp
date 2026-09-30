// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "OvPhysxAdapter.hpp"

#include <isaacsim/common/logging/Logging.hpp>
#include <isaacsim/common/ovstage/TransformJournal.hpp>
#include <isaacsim/physics/registration/Physics.hpp>
#include <isaacsim/physics/registration/simulator/Benchmark.hpp>
#include <isaacsim/physics/registration/simulator/Interaction.hpp>
#include <isaacsim/physics/registration/simulator/Simulator.hpp>
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovx/path_dictionary/path_dictionary.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

namespace
{

const isaacsim::common::logging::Logger g_kLogger("isaacsim.physics_engines.ovphysx");

// Process-global lifecycle guard so ovphysx_initialize() / ovphysx_shutdown()
// are called exactly once per process even with multiple OvPhysxAdapter instances.
std::mutex g_lifecycleMutex;
int g_instanceCount{ 0 };
// ovphysx's lifecycle is a single latch (initialize() errors if already set), not a
// refcount. Track whether OUR initialize() set it so we never shut down a lifecycle
// another consumer in this process owns.
bool g_weInitialized{ false };

void initializeGlobalState()
{
    std::lock_guard<std::mutex> lock(g_lifecycleMutex);
    if (g_instanceCount == 0)
    {
#if defined(_WIN32)
        const ovstage_api_status_t status = ovstage_initialize(nullptr);
        if (status != OVSTAGE_OK)
        {
            throw std::runtime_error("ovstage_initialize failed with status " + std::to_string(static_cast<int>(status)));
        }
#endif
        // A non-success status means the latch was already set -- the only failure
        // ovphysx_initialize() documents -- i.e. another consumer owns the lifecycle.
        g_weInitialized = (ovphysx_initialize().status == OVPHYSX_API_SUCCESS);
    }
    ++g_instanceCount;
}

void releaseGlobalState()
{
    std::lock_guard<std::mutex> lock(g_lifecycleMutex);
    --g_instanceCount;
    if (g_instanceCount == 0)
    {
        // Only release the latch if we set it; otherwise the consumer that
        // initialized ovphysx still owns it.
        if (g_weInitialized)
        {
            ovphysx_shutdown();
        }
        g_weInitialized = false;
#if defined(_WIN32)
        const ovstage_api_status_t status = ovstage_shutdown();
        if (status != OVSTAGE_OK)
        {
            ISAACSIM_LOG_ERROR(g_kLogger, "ovstage_shutdown failed with status {}", static_cast<int>(status));
        }
#endif
    }
}

void checkResult(ovphysx_result_t result, const char* context)
{
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t errorString = ovphysx_get_last_error();
        std::string message(errorString.ptr ? errorString.ptr : "", errorString.ptr ? errorString.length : 0);
        throw std::runtime_error(std::string(context) + ": ovphysx error: " + message);
    }
}

// The manager parses each scene once at this ordinal and steps it; it never
// authors control edits into the Stage (state is written via the tensor API),
// so no ovphysx_update_from_ovstage / higher ordinals are needed. ovstage ordinals
// are application-owned (see ovstage_population.h): the caller-owned Stage must be
// populated at this ordinal, or the attach reads an empty scene. Part of the
// dual-input contract documented on PhysicsSimulation::initialize.
constexpr ovstage_ordinal_t g_kAttachOrdinal = 1;

constexpr char g_kWorldMatrixAttribute[] = "omni:fabric:worldMatrix";

using TransformJournalEntry = isaacsim::common::ovstage::TransformJournalEntry;
using TransformJournalHeader = isaacsim::common::ovstage::TransformJournalHeader;

struct TransformJournalBatch
{
    std::vector<TransformJournalEntry> entries;
    bool valid{ true };
};

std::atomic<uint64_t> g_nextTransformJournalGeneration{ 1 };

ovx_string_or_token_t createAttributeName(const char* name)
{
    ovx_string_or_token_t result{};
    result.string = { name, std::strlen(name) };
    return result;
}

bool completeOvstageOperation(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue, const char* context)
{
    if (enqueue.status != OVSTAGE_OK)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "{}: {}", context, ovstage_get_error_string(stage, enqueue.status));
        return false;
    }
    if (enqueue.op_index == OVSTAGE_INVALID_OP_ID)
    {
        return true;
    }

    ovstage_op_wait_result_t waitResult{};
    const ovstage_api_status_t status = ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &waitResult);
    if (status != OVSTAGE_OK)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "{}: {}", context, ovstage_get_error_string(stage, status));
    }
    for (size_t index = 0; index < waitResult.error_op_id_count; ++index)
    {
        const ovx_string_t error = ovstage_get_last_op_error(stage, waitResult.error_op_ids[index]);
        ISAACSIM_LOG_ERROR(
            g_kLogger, "{}: {}", context, std::string_view(error.ptr ? error.ptr : "", error.ptr ? error.length : 0));
    }
    const ovstage_api_status_t releaseStatus = ovstage_release_op(stage, enqueue.op_index);
    if (releaseStatus != OVSTAGE_OK)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "Release {}: {}", context, ovstage_get_error_string(stage, releaseStatus));
    }
    return status == OVSTAGE_OK && waitResult.error_op_id_count == 0 && releaseStatus == OVSTAGE_OK;
}

bool getNextOrdinal(ovstage_instance_t* stage, ovstage_ordinal_t& ordinal)
{
    ovstage_ordinal_query_handle_t query = OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE;
    const ovstage_enqueue_result_t enqueue = ovstage_get_attribute_write_floor(stage, {}, &query);
    if (enqueue.status != OVSTAGE_OK || query == OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "Unable to query the attached OVStage write floor");
        return false;
    }

    ovstage_ordinal_t floor = 0;
    const ovstage_api_status_t fetchStatus = ovstage_fetch_ordinal(stage, query, OVSTAGE_TIMEOUT_INFINITE, &floor);
    const bool queryCompleted = completeOvstageOperation(stage, enqueue, "Query OVStage write floor");
    const bool queryReleased = completeOvstageOperation(
        stage, ovstage_release_ordinal_query(stage, query), "Release OVStage write-floor query");
    if (fetchStatus != OVSTAGE_OK || !queryCompleted || !queryReleased ||
        floor == std::numeric_limits<ovstage_ordinal_t>::max())
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "Unable to fetch a usable attached OVStage write floor");
        return false;
    }
    ordinal = floor + 1;
    return true;
}

bool sealOrdinal(ovstage_instance_t* stage, ovstage_ordinal_t ordinal)
{
    ovstage_write_floor_desc_t description{};
    description.ordinal = ordinal;
    description.scope = OVSTAGE_SCOPE_ALL;
    return completeOvstageOperation(
        stage, ovstage_advance_write_floor(stage, &description), "Advance OVStage write floor");
}

bool isCpuFloatVector(const DLTensor& tensor, uint8_t lanes)
{
    return tensor.data != nullptr && tensor.device.device_type == kDLCPU && tensor.ndim == 1 && tensor.shape != nullptr &&
           tensor.dtype.code == kDLFloat && tensor.dtype.bits == 32 && tensor.dtype.lanes == lanes;
}

const float* getFloatVector(const DLTensor& tensor, size_t index)
{
    const size_t elementBytes = sizeof(float) * tensor.dtype.lanes;
    const int64_t stride = tensor.strides ? tensor.strides[0] : 1;
    const auto* bytes = static_cast<const unsigned char*>(tensor.data) + tensor.byte_offset;
    return reinterpret_cast<const float*>(bytes + index * static_cast<size_t>(stride) * elementBytes);
}

void buildWorldMatrix(const float* position, const float* orientation, const double* scale, double* matrix)
{
    const double x = orientation[0];
    const double y = orientation[1];
    const double z = orientation[2];
    const double w = orientation[3];
    const double length = std::sqrt(x * x + y * y + z * z + w * w);
    const double inverseLength = length > std::numeric_limits<double>::epsilon() ? 1.0 / length : 1.0;
    const double qx = x * inverseLength;
    const double qy = y * inverseLength;
    const double qz = z * inverseLength;
    const double qw = length > std::numeric_limits<double>::epsilon() ? w * inverseLength : 1.0;

    // OVStage/USD matrices use row vectors, so this is the transpose of the usual
    // column-vector quaternion matrix. Preserve the authored world scale because
    // scale is not a simulated OVPhysX output.
    matrix[0] = scale[0] * (1.0 - 2.0 * (qy * qy + qz * qz));
    matrix[1] = scale[0] * (2.0 * (qx * qy + qw * qz));
    matrix[2] = scale[0] * (2.0 * (qx * qz - qw * qy));
    matrix[3] = 0.0;
    matrix[4] = scale[1] * (2.0 * (qx * qy - qw * qz));
    matrix[5] = scale[1] * (1.0 - 2.0 * (qx * qx + qz * qz));
    matrix[6] = scale[1] * (2.0 * (qy * qz + qw * qx));
    matrix[7] = 0.0;
    matrix[8] = scale[2] * (2.0 * (qx * qz + qw * qy));
    matrix[9] = scale[2] * (2.0 * (qy * qz - qw * qx));
    matrix[10] = scale[2] * (1.0 - 2.0 * (qx * qx + qy * qy));
    matrix[11] = 0.0;
    matrix[12] = position[0];
    matrix[13] = position[1];
    matrix[14] = position[2];
    matrix[15] = 1.0;
}

std::vector<double> readWorldScales(ovstage_instance_t* stage,
                                    ovstage_query_handle_t query,
                                    size_t primCount,
                                    ovstage_ordinal_t ordinal)
{
    std::vector<double> scales(primCount * 3, 1.0);
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
    {
        return scales;
    }

    const char* names[] = { g_kWorldMatrixAttribute, "worldMatrix" };
    ovx_string_t strings[] = { { names[0], std::strlen(names[0]) }, { names[1], std::strlen(names[1]) } };
    ovx_token_t attributes[2]{};
    if (path_dictionary_create_tokens_from_strings(dictionary, strings, 2, attributes).status != OVX_API_SUCCESS)
    {
        return scales;
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = ordinal;
    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t enqueue = ovstage_read_attributes(stage, query, attributes, 2, range, &read);
    if (enqueue.status != OVSTAGE_OK || read == OVSTAGE_INVALID_READ_HANDLE)
    {
        return scales;
    }
    if (!completeOvstageOperation(stage, enqueue, "Read authored world scales"))
    {
        completeOvstageOperation(stage, ovstage_release_read(stage, read), "Release failed world-scale read");
        return scales;
    }

    for (;;)
    {
        ovstage_read_group_t group{};
        const ovstage_api_status_t status = ovstage_fetch_read_next(stage, read, OVSTAGE_TIMEOUT_INFINITE, &group);
        if (status == OVSTAGE_ERROR_END_OF_ITERATION)
        {
            break;
        }
        if (status != OVSTAGE_OK)
        {
            ISAACSIM_LOG_WARN(g_kLogger, "Unable to read an authored world-matrix group: {}",
                              ovstage_get_error_string(stage, status));
            break;
        }

        if (!group.is_delete && !group.is_array && !group.data.mask && group.data.tensors && group.data.tensor_count == 1)
        {
            const DLTensor& tensor = group.data.tensors[0];
            if (tensor.data && tensor.device.device_type == kDLCPU && tensor.ndim == 1 && tensor.shape &&
                tensor.dtype.code == kDLFloat && tensor.dtype.bits == 64 && tensor.dtype.lanes == 16)
            {
                const auto* bytes = static_cast<const unsigned char*>(tensor.data) + tensor.byte_offset;
                const size_t elementBytes = 16 * sizeof(double);
                const int64_t stride = tensor.strides ? tensor.strides[0] : 1;
                for (size_t index = 0; index < group.prims.count; ++index)
                {
                    const size_t outputIndex =
                        group.prims.index_map ? group.prims.index_map[index] : group.prims.offset + index;
                    const size_t dataIndex = group.data.index_map ? group.data.index_map[index] : index;
                    if (outputIndex >= primCount || dataIndex >= static_cast<size_t>(tensor.shape[0]))
                    {
                        continue;
                    }
                    const auto* matrix =
                        reinterpret_cast<const double*>(bytes + dataIndex * static_cast<size_t>(stride) * elementBytes);
                    for (size_t axis = 0; axis < 3; ++axis)
                    {
                        const size_t row = axis * 4;
                        scales[outputIndex * 3 + axis] =
                            std::sqrt(matrix[row] * matrix[row] + matrix[row + 1] * matrix[row + 1] +
                                      matrix[row + 2] * matrix[row + 2]);
                    }
                }
            }
        }
        ovstage_release_group(stage, &group);
    }
    completeOvstageOperation(stage, ovstage_release_read(stage, read), "Release authored world-scale read");
    return scales;
}

struct PoseOutputGroups
{
    const ovstage_read_group_t* positions{ nullptr };
    const ovstage_read_group_t* orientations{ nullptr };
};

bool writeWorldMatrices(ovstage_instance_t* stage,
                        const ovstage_read_group_t& positions,
                        const ovstage_read_group_t& orientations,
                        ovstage_ordinal_t readOrdinal,
                        ovstage_ordinal_t writeOrdinal,
                        TransformJournalBatch& journal);

bool publishObjectTransforms(ovphysx_handle_t handle,
                             ovstage_instance_t* stage,
                             ovphysx_sim_object_type_t objectType,
                             ovstage_ordinal_t readOrdinal,
                             ovstage_ordinal_t writeOrdinal,
                             bool& wroteOutput,
                             TransformJournalBatch& journal)
{
    ovphysx_query_handle_t query = 0;
    // The API promises the latest complete stage state, not merely objects moved
    // during the last solver substep. A caller may publish after several steps,
    // by which time a body can already be sleeping even though its authored pose
    // is stale. Query all objects so that settled poses are not omitted.
    const ovphysx_result_t queryResult = ovphysx_query(handle, objectType, OVPHYSX_SCOPE_ALL, &query);
    if (queryResult.status != OVPHYSX_API_SUCCESS || query == 0)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "Unable to query OVPhysX transform output");
        return false;
    }
    const ovx_string_or_token_t attributes[] = { createAttributeName(OVPHYSX_ATTR_POSITION),
                                                 createAttributeName(OVPHYSX_ATTR_ORIENTATION) };
    ovphysx_read_handle_t read = 0;
    const ovphysx_result_t readResult = ovphysx_read(handle, query, attributes, 2, &read);
    if (readResult.status != OVPHYSX_API_SUCCESS || read == 0)
    {
        ovphysx_release_query(handle, query);
        ISAACSIM_LOG_ERROR(g_kLogger, "Unable to read OVPhysX transform output");
        return false;
    }

    bool success = true;
    PoseOutputGroups outputs;
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t fetchResult = ovphysx_fetch_read_next(handle, read, &group);
        if (fetchResult.status == OVPHYSX_API_END_OF_ITERATION)
        {
            break;
        }
        if (fetchResult.status != OVPHYSX_API_SUCCESS || group == nullptr)
        {
            ISAACSIM_LOG_ERROR(g_kLogger, "Unable to fetch OVPhysX transform output");
            success = false;
            break;
        }
        if (group->is_delete || group->data.tensor_count == 0 || group->data.tensors == nullptr ||
            group->prims.count == 0 || group->prims.list == OVX_INVALID_PRIMPATH_LIST)
        {
            continue;
        }
        if (group->is_array)
        {
            // Point-instancer transforms are outside the initial publication scope.
            continue;
        }

        const uint8_t lanes = group->data.tensors[0].dtype.lanes;
        if (lanes == 3)
        {
            outputs.positions = group;
        }
        else if (lanes == 4)
        {
            outputs.orientations = group;
        }
        else
        {
            ISAACSIM_LOG_ERROR(g_kLogger, "OVPhysX returned an unexpected transform output width ({})", lanes);
            success = false;
        }
    }

    if (outputs.positions || outputs.orientations)
    {
        if (!outputs.positions || !outputs.orientations)
        {
            ISAACSIM_LOG_ERROR(g_kLogger, "OVPhysX returned an incomplete position/orientation output pair");
            success = false;
        }
        else
        {
            success = writeWorldMatrices(
                          stage, *outputs.positions, *outputs.orientations, readOrdinal, writeOrdinal, journal) &&
                      success;
            wroteOutput = true;
        }
    }
    if (ovphysx_release_read(handle, read).status != OVPHYSX_API_SUCCESS)
    {
        success = false;
    }
    if (ovphysx_release_query(handle, query).status != OVPHYSX_API_SUCCESS)
    {
        success = false;
    }
    return success;
}

bool writeWorldMatrices(ovstage_instance_t* stage,
                        const ovstage_read_group_t& positions,
                        const ovstage_read_group_t& orientations,
                        ovstage_ordinal_t readOrdinal,
                        ovstage_ordinal_t writeOrdinal,
                        TransformJournalBatch& journal)
{
    if (positions.prims.offset != 0 || orientations.prims.offset != 0 || positions.prims.index_map ||
        orientations.prims.index_map || positions.data.index_map || orientations.data.index_map ||
        positions.data.mask || orientations.data.mask || positions.prims.count != orientations.prims.count ||
        positions.data.tensor_count != 1 || orientations.data.tensor_count != 1)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "OVPhysX returned incompatible position/orientation output groups");
        return false;
    }

    const DLTensor& positionTensor = positions.data.tensors[0];
    const DLTensor& orientationTensor = orientations.data.tensors[0];
    const size_t count = positions.prims.count;
    if (!isCpuFloatVector(positionTensor, 3) || !isCpuFloatVector(orientationTensor, 4) ||
        positionTensor.shape[0] != static_cast<int64_t>(count) ||
        orientationTensor.shape[0] != static_cast<int64_t>(count))
    {
        ISAACSIM_LOG_ERROR(
            g_kLogger, "OVPhysX transform publication currently requires dense CPU float position/orientation output");
        return false;
    }

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(stage, positions.prims.list, &query) != OVSTAGE_OK ||
        query == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ISAACSIM_LOG_ERROR(g_kLogger, "Unable to target OVPhysX poses in OVStage");
        return false;
    }

    const std::vector<double> scales = readWorldScales(stage, query, count, readOrdinal);
    std::vector<double> matrices(count * 16);
    for (size_t index = 0; index < count; ++index)
    {
        buildWorldMatrix(getFloatVector(positionTensor, index), getFloatVector(orientationTensor, index),
                         scales.data() + index * 3, matrices.data() + index * 16);
    }

    int64_t shape = static_cast<int64_t>(count);
    DLTensor tensor{};
    tensor.data = matrices.data();
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 1;
    tensor.dtype = { kDLFloat, 64, 16 };
    tensor.shape = &shape;
    ovstage_write_data_t write{};
    write.tensors = &tensor;
    write.tensor_count = 1;
    write.semantic = OVSTAGE_SEMANTIC_MATRIX;
    write.is_array = false;
    const bool written =
        completeOvstageOperation(stage,
                                 ovstage_write_attribute(stage, query, createAttributeName(g_kWorldMatrixAttribute),
                                                         writeOrdinal, write, OVSTAGE_PRIM_MODE_UPSERT),
                                 "Publish OVPhysX world matrices");
    const bool released =
        completeOvstageOperation(stage, ovstage_release_query(stage, query), "Release OVStage pose query");
    if (!written || !released)
    {
        return false;
    }

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    std::vector<ovx_primpath_t> paths(count);
    size_t pathCount = 0;
    if (!dictionary ||
        path_dictionary_get_paths_from_path_list(dictionary, positions.prims.list, 0, count, paths.data(), &pathCount).status !=
            OVX_API_SUCCESS ||
        pathCount != count)
    {
        ISAACSIM_LOG_WARN(g_kLogger, "Unable to capture OVPhysX transform paths for the optional change journal");
        journal.entries.clear();
        journal.valid = false;
        return true;
    }
    if (!journal.valid)
    {
        return true;
    }
    journal.entries.reserve(journal.entries.size() + count);
    for (size_t index = 0; index < count; ++index)
    {
        TransformJournalEntry entry{};
        entry.primPath = static_cast<uint64_t>(paths[index]);
        std::memcpy(entry.worldMatrix, matrices.data() + index * 16, sizeof(entry.worldMatrix));
        journal.entries.push_back(entry);
    }
    return true;
}

bool publishTransformJournal(ovstage_instance_t* stage,
                             const TransformJournalBatch& journal,
                             ovstage_ordinal_t baseOrdinal,
                             ovstage_ordinal_t ordinal,
                             uint64_t generation)
{
    using namespace isaacsim::common::ovstage;
    if (!journal.valid || journal.entries.empty())
    {
        return false;
    }

    TransformJournalHeader header{};
    header.generation = generation;
    header.baseOrdinal = baseOrdinal;
    header.ordinal = ordinal;
    header.entryCount = journal.entries.size();
    std::vector<uint8_t> payload(sizeof(header) + journal.entries.size() * sizeof(TransformJournalEntry));
    std::memcpy(payload.data(), &header, sizeof(header));
    std::memcpy(payload.data() + sizeof(header), journal.entries.data(),
                journal.entries.size() * sizeof(TransformJournalEntry));

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
    {
        return false;
    }
    const ovx_string_t journalPath{ g_kTransformJournalPrimPath, std::strlen(g_kTransformJournalPrimPath) };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &journalPath, 1, &pathList).status != OVX_API_SUCCESS ||
        pathList == OVX_INVALID_PRIMPATH_LIST)
    {
        return false;
    }
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(stage, pathList, &query) != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return false;
    }

    int64_t shape[2] = { 1, static_cast<int64_t>(payload.size()) };
    DLTensor tensor{};
    tensor.data = payload.data();
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 2;
    tensor.dtype = { kDLUInt, 8, 1 };
    tensor.shape = shape;
    ovstage_write_data_t write{};
    write.tensors = &tensor;
    write.tensor_count = 1;
    write.semantic = OVSTAGE_SEMANTIC_NONE;
    write.is_array = true;
    const bool written =
        completeOvstageOperation(stage,
                                 ovstage_write_attribute(stage, query, createAttributeName(g_kTransformJournalAttribute),
                                                         ordinal, write, OVSTAGE_PRIM_MODE_UPSERT),
                                 "Publish OVPhysX transform journal");
    const bool queryReleased =
        completeOvstageOperation(stage, ovstage_release_query(stage, query), "Release transform-journal query");
    const bool pathsReleased =
        path_dictionary_release_path_list_reference(dictionary, pathList).status == OVX_API_SUCCESS;
    return written && queryReleased && pathsReleased;
}

// ovphysx reports the contact-pair event kind as 0 = found, 1 = lost, 2 = persist. Map the three
// known values explicitly rather than casting, and report anything else instead of passing it off
// as a persist -- a renumbering on either side would otherwise mislabel every event silently. The
// warning fires once per process: this runs per contact pair per step.
isaacsim::physics::registration::ContactEventType convertToContactEventType(int32_t eventType)
{
    using isaacsim::physics::registration::ContactEventType;
    switch (eventType)
    {
    case 0:
        return ContactEventType::eContactFound;
    case 1:
        return ContactEventType::eContactLost;
    case 2:
        return ContactEventType::eContactPersist;
    default:
        break;
    }

    static std::atomic<bool> s_reportedUnknownEventType{ false };
    if (!s_reportedUnknownEventType.exchange(true))
    {
        ISAACSIM_LOG_WARN(g_kLogger,
                          "OvPhysxAdapter: unknown ovphysx contact event type {}; reporting it as persist. "
                          "Further occurrences are not logged.",
                          eventType);
    }
    return ContactEventType::eContactPersist;
}

// SweepHit and RaycastHit share the same set of fields; fill them from one
// templated helper so the two converters can't drift out of sync.
template <class HitType>
HitType convertToHit(const ovphysx_scene_query_hit_t& sourceHit)
{
    HitType outputHit{};
    outputHit.collision = static_cast<isaacsim::physics::registration::PathToken>(sourceHit.collision);
    outputHit.rigidBody = static_cast<isaacsim::physics::registration::PathToken>(sourceHit.rigid_body);
    outputHit.prototypeIndex = sourceHit.proto_index;
    outputHit.normal = { sourceHit.normal[0], sourceHit.normal[1], sourceHit.normal[2] };
    outputHit.position = { sourceHit.position[0], sourceHit.position[1], sourceHit.position[2] };
    outputHit.distance = sourceHit.distance;
    outputHit.faceIndex = sourceHit.face_index;
    outputHit.material = static_cast<isaacsim::physics::registration::PathToken>(sourceHit.material);
    return outputHit;
}

isaacsim::physics::registration::SweepHit convertToSweepHit(const ovphysx_scene_query_hit_t& sourceHit)
{
    return convertToHit<isaacsim::physics::registration::SweepHit>(sourceHit);
}

isaacsim::physics::registration::RaycastHit convertToRaycastHit(const ovphysx_scene_query_hit_t& sourceHit)
{
    return convertToHit<isaacsim::physics::registration::RaycastHit>(sourceHit);
}

isaacsim::physics::registration::OverlapHit convertToOverlapHit(const ovphysx_scene_query_hit_t& sourceHit)
{
    isaacsim::physics::registration::OverlapHit outputHit{};
    outputHit.collision = static_cast<isaacsim::physics::registration::PathToken>(sourceHit.collision);
    outputHit.rigidBody = static_cast<isaacsim::physics::registration::PathToken>(sourceHit.rigid_body);
    outputHit.prototypeIndex = sourceHit.proto_index;
    return outputHit;
}

ovphysx_scene_query_geometry_desc_t makeSphereGeometry(float radius,
                                                       const isaacsim::physics::registration::Float3& position)
{
    ovphysx_scene_query_geometry_desc_t geometry{};
    geometry.type = OVPHYSX_SCENE_QUERY_GEOMETRY_SPHERE;
    geometry.sphere.radius = radius;
    geometry.sphere.position[0] = position.x;
    geometry.sphere.position[1] = position.y;
    geometry.sphere.position[2] = position.z;
    return geometry;
}

ovphysx_scene_query_geometry_desc_t makeBoxGeometry(const isaacsim::physics::registration::Float3& halfExtent,
                                                    const isaacsim::physics::registration::Float3& position,
                                                    const isaacsim::physics::registration::Float4& rotation)
{
    ovphysx_scene_query_geometry_desc_t geometry{};
    geometry.type = OVPHYSX_SCENE_QUERY_GEOMETRY_BOX;
    geometry.box.half_extent[0] = halfExtent.x;
    geometry.box.half_extent[1] = halfExtent.y;
    geometry.box.half_extent[2] = halfExtent.z;
    geometry.box.position[0] = position.x;
    geometry.box.position[1] = position.y;
    geometry.box.position[2] = position.z;
    geometry.box.rotation[0] = rotation.x;
    geometry.box.rotation[1] = rotation.y;
    geometry.box.rotation[2] = rotation.z;
    geometry.box.rotation[3] = rotation.w;
    return geometry;
}

} // namespace

// ----------------------------------------------------------------------------
// Construction / destruction
// ----------------------------------------------------------------------------

std::shared_ptr<OvPhysxAdapter> OvPhysxAdapter::create(const Configuration& configuration)
{
    return std::shared_ptr<OvPhysxAdapter>(new OvPhysxAdapter(configuration));
}

OvPhysxAdapter::GlobalLifecycleGuard::GlobalLifecycleGuard()
{
    initializeGlobalState();
}

OvPhysxAdapter::GlobalLifecycleGuard::~GlobalLifecycleGuard()
{
    releaseGlobalState();
}

OvPhysxAdapter::OvPhysxAdapter(const Configuration& configuration)
{
    ovphysx_register_schema_paths();

    ovphysx_create_args arguments = OVPHYSX_CREATE_ARGS_DEFAULT;
    // Non-static (per-call, so concurrent constructions don't share it) but declared here so it
    // outlives ovphysx_create_instance below -- active_cuda_gpus borrows the pointer, not a copy.
    char gpuOrdinalBuffer[32];
    if (configuration.gpuOrdinal > 0)
    {
        snprintf(gpuOrdinalBuffer, sizeof(gpuOrdinalBuffer), "%d", configuration.gpuOrdinal);
        arguments.active_cuda_gpus = ovphysx_cstr(gpuOrdinalBuffer);
    }

    if (configuration.useGpuPipeline)
    {
        // DirectGPU: suppress readback before instance creation.
        ovphysx_config_entry_t suppressReadbackEntry{};
        suppressReadbackEntry.key_type = OVPHYSX_CONFIG_KEY_TYPE_CARBONITE;
        suppressReadbackEntry.key.carbonite_key = OVPHYSX_LITERAL("/physics/suppressReadback");
        suppressReadbackEntry.value.string_value = OVPHYSX_LITERAL("true");
        ovphysx_set_global_config(suppressReadbackEntry);

        ovphysx_config_entry_t suppressFabricUpdateEntry{};
        suppressFabricUpdateEntry.key_type = OVPHYSX_CONFIG_KEY_TYPE_CARBONITE;
        suppressFabricUpdateEntry.key.carbonite_key = OVPHYSX_LITERAL("/physics/suppressFabricUpdate");
        suppressFabricUpdateEntry.value.string_value = OVPHYSX_LITERAL("true");
        ovphysx_set_global_config(suppressFabricUpdateEntry);
    }

    checkResult(ovphysx_create_instance(&arguments, &m_handle), "OvPhysxAdapter::create");
}

OvPhysxAdapter::~OvPhysxAdapter()
{
    unregisterFromManager();
    // Defensive: detach any borrowed Stage still attached if _doClose wasn't called.
    // The Stage is caller-owned, so it is only detached here, never destroyed.
    _detachOvstage();
    if (m_handle != OVPHYSX_INVALID_HANDLE)
    {
        ovphysx_destroy_instance(m_handle);
        m_handle = OVPHYSX_INVALID_HANDLE;
    }
}

// ----------------------------------------------------------------------------
// Registration
// ----------------------------------------------------------------------------

size_t OvPhysxAdapter::registerWithManager(const std::string& name)
{
    isaacsim::physics::registration::Simulation simulation;
    _buildSimulation(simulation);

    const isaacsim::physics::registration::SimulationId simulationId =
        isaacsim::physics::registration::registerSimulation(simulation, name);
    if (simulationId == isaacsim::physics::registration::g_kInvalidSimulationId)
    {
        throw std::runtime_error("OvPhysxAdapter: registerSimulation returned g_kInvalidSimulationId");
    }

    m_simulationId = simulationId.id;
    return m_simulationId;
}

void OvPhysxAdapter::unregisterFromManager()
{
    if (m_simulationId == isaacsim::physics::registration::g_kInvalidSimulationId.id)
    {
        return;
    }
    isaacsim::physics::registration::unregisterSimulation(isaacsim::physics::registration::SimulationId(m_simulationId));
    m_simulationId = isaacsim::physics::registration::g_kInvalidSimulationId.id;
}

// ----------------------------------------------------------------------------
// Simulation struct construction
// ----------------------------------------------------------------------------

void OvPhysxAdapter::_buildSimulation(isaacsim::physics::registration::Simulation& simulation)
{
    auto& simulationFunctions = simulation.simulationFunctions;

    simulationFunctions.initialize = [this](void* ovstage, const char* usdIdentifier)
    { return _doInitialize(ovstage, usdIdentifier); };
    simulationFunctions.close = [this]() { return _doClose(); };
    simulationFunctions.getAttachedStage = [this]() { return _doGetAttachedStage(); };
    simulationFunctions.hasAttachedStage = [this]() { return _doHasAttachedStage(); };
    simulationFunctions.simulateAsynchronously = [this](float elapsed, float current)
    { _doSimulateAsynchronously(elapsed, current); };
    simulationFunctions.simulate = [this](float elapsed, float current) { _doSimulate(elapsed, current); };
    simulationFunctions.fetchResults = [this]() { _doFetchResults(); };
    simulationFunctions.checkResults = [this]() { return _doCheckResults(); };
    simulationFunctions.publishTransformsToStage = [this]() { return _doPublishTransformsToStage(); };
    simulationFunctions.flushChanges = [this]() { _doFlushChanges(); };
    simulationFunctions.pauseChangeTracking = [this](bool pause) { _doPauseChangeTracking(pause); };
    simulationFunctions.isChangeTrackingPaused = [this]() { return _doIsChangeTrackingPaused(); };
    simulationFunctions.subscribePhysicsContactReportEvents =
        [this](isaacsim::physics::registration::OnContactReportEventFunction callback)
    { return _doSubscribeContactReport(std::move(callback)); };
    simulationFunctions.unsubscribePhysicsContactReportEvents =
        [this](isaacsim::physics::registration::SubscriptionId subscriptionId)
    { _doUnsubscribeContactReport(subscriptionId); };
    simulationFunctions.getSimulationTimeStepsPerSecond =
        [this](long stageId, isaacsim::physics::registration::PathToken scenePath)
    { return _doGetTimeStepsPerSecond(stageId, scenePath); };
    simulationFunctions.getSimulationTimestamp = [this]() { return _doGetTimestamp(); };
    simulationFunctions.getSimulationStepCount = [this]() { return _doGetStepCount(); };
    simulationFunctions.subscribePhysicsOnStepEvents =
        [this](bool preStep, int order, isaacsim::physics::registration::OnPhysicsStepEventFunction callback)
    { return _doSubscribeStepEvents(preStep, order, std::move(callback)); };
    simulationFunctions.unsubscribePhysicsOnStepEvents =
        [this](isaacsim::physics::registration::SubscriptionId subscriptionId)
    { _doUnsubscribeStepEvents(subscriptionId); };
    simulationFunctions.isCapableOfSimulating = [this](const char** schemaNames, size_t count, bool* isCapable)
    { return _doIsCapableOfSimulating(schemaNames, count, isCapable); };

    _fillSceneQueryFunctions(simulation.sceneQueryFunctions);

    // InteractionFunctions: wire no-op stubs so callers don't crash on a null function pointer.
    simulation.interactionFunctions.handleRaycast = [](const isaacsim::physics::registration::Float3&,
                                                       const isaacsim::physics::registration::Float3&, bool) {};
    simulation.interactionFunctions.getPrimDebugData =
        [](const std::string&) -> isaacsim::physics::registration::DebugDataDictionary { return {}; };

    // BenchmarkFunctions: no-op stubs -- profile stats not yet plumbed to ovphysx.
    simulation.benchmarkFunctions.subscribeProfileStatisticsEvents =
        [](isaacsim::physics::registration::ProfileStatisticsNotificationFunction) -> isaacsim::physics::registration::SubscriptionId
    { return isaacsim::physics::registration::SubscriptionId(0); };
    simulation.benchmarkFunctions.unsubscribeProfileStatisticsEvents =
        [](isaacsim::physics::registration::SubscriptionId) {};
}

// ----------------------------------------------------------------------------
// ovstage attach/detach (the Stage is caller-owned; the adapter borrows it)
// ----------------------------------------------------------------------------

bool OvPhysxAdapter::_detachOvstage()
{
    if (m_ovstage == nullptr)
    {
        return true;
    }
    // The Stage is caller-owned, so only detach it -- never destroy it. On a failed
    // detach ovphysx still references the Stage, so keep m_ovstage set and report the
    // failure so the caller retries rather than proceeds with inconsistent state.
    ovphysx_result_t result = ovphysx_detach_ovstage(m_handle);
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t errorString = ovphysx_get_last_error();
        const std::string_view error(errorString.ptr ? errorString.ptr : "", errorString.ptr ? errorString.length : 0);
        ISAACSIM_LOG_ERROR(g_kLogger, "OvPhysxAdapter::_detachOvstage: ovphysx_detach_ovstage failed: {}", error);
        return false;
    }
    m_ovstage = nullptr;
    return true;
}

// ----------------------------------------------------------------------------
// SimulationFunctions implementations
// ----------------------------------------------------------------------------

bool OvPhysxAdapter::_doInitialize(void* callerOvstage, const char* usdIdentifier)
{
    // The adapter is process-global and may be re-initialized for a new stage (each
    // test/scene calls initialize on the same instance). Detach any borrowed Stage
    // from a previous initialize first; if detach fails the old Stage is still
    // attached, so abort rather than attach a new one over it. m_attachedStageId is
    // left untouched on that abort, so getAttachedStage() keeps reporting the Stage
    // that is still attached.
    if (!_detachOvstage())
    {
        return false;
    }

    // The detach succeeded (or nothing was attached): nothing is attached now, so
    // getAttachedStage() must read 0 until a new attach succeeds below. The id is
    // committed only on a success path, so a failed attach cannot leave a stale id
    // (getAttachedStage() == 0 after a failed initialize).
    m_attachedStageId = 0;
    m_initialized = false;
    m_stepCount = 0;
    m_timestamp = 0;
    // Drop any step dispatched against the previous stage: its op index does not carry over,
    // and its post-step event must not fire against the new one.
    m_hasPendingStep = false;
    m_postStepPending = false;
    m_transformJournalGeneration = g_nextTransformJournalGeneration.fetch_add(1);
    m_transformJournalBaseOrdinal = 0;
    m_lastTransformJournalOrdinal = 0;

    // ovphysx parses physics from the caller-owned ovstage. The USD identifier (a
    // StageCache id as a string, or "0") is only republished via getAttachedStage();
    // the tensor views no longer read USD metadata through it.
    const long stageId = usdIdentifier ? std::strtol(usdIdentifier, nullptr, 10) : 0L;

    // Dual-input contract: ovphysx consumes the caller-owned ovstage directly. A
    // null Stage is an explicit empty scene; a non-null Stage is attached and
    // borrowed (never owned/destroyed here).
    ovstage_instance_t* stage = static_cast<ovstage_instance_t*>(callerOvstage);
    if (stage == nullptr)
    {
        m_ovstage = nullptr;
        m_attachedStageId = stageId;
        m_initialized = true;
        return true;
    }

    if (ovphysx_attach_ovstage(m_handle, stage, g_kAttachOrdinal).status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t errorString = ovphysx_get_last_error();
        const std::string_view error(errorString.ptr ? errorString.ptr : "", errorString.ptr ? errorString.length : 0);
        ISAACSIM_LOG_ERROR(g_kLogger, "OvPhysxAdapter::_doInitialize: ovphysx_attach_ovstage failed: {}", error);
        return false;
    }
    m_ovstage = stage;
    m_attachedStageId = stageId;
    m_initialized = true;
    return true;
}

bool OvPhysxAdapter::_doClose()
{
    if (!m_initialized)
    {
        return true;
    }
    // Keep the instance's state intact if detach fails so the Stage is not lost
    // while ovphysx still owns it; a later _doClose/destructor can retry. Report
    // the failure so the caller keeps the Stage alive rather than freeing it.
    if (!_detachOvstage())
    {
        return false;
    }
    m_initialized = false;
    m_attachedStageId = 0;
    return true;
}

long OvPhysxAdapter::_doGetAttachedStage()
{
    return m_attachedStageId;
}

void OvPhysxAdapter::_doSimulateAsynchronously(float elapsed, float /*current*/)
{
    if (!m_initialized)
    {
        return;
    }

    m_lastStepElapsed = elapsed;
    _fireStepSubscribers(/*preStep=*/true, elapsed);

    // A dispatched step owes its post-step notification even when the engine enqueues
    // nothing: an instance initialized without a Stage still steps an empty scene, and the
    // manager's contract is one post-step event per step call. Only a fetch with nothing
    // dispatched stays silent.
    m_postStepPending = true;

    if (m_handle != OVPHYSX_INVALID_HANDLE)
    {
        ovphysx_enqueue_result_t result = ovphysx_step(m_handle, elapsed);
        if (result.status == OVPHYSX_API_SUCCESS)
        {
            m_pendingStepOperation = result.op_index;
            m_hasPendingStep = true;
        }
    }

    ++m_stepCount;
    ++m_timestamp;
    // Post-step subscribers fire in _doFetchResults after ovphysx_wait_op completes,
    // so they observe settled physics state rather than the pre-wait snapshot.
}

void OvPhysxAdapter::_doSimulate(float elapsed, float current)
{
    static_cast<void>(current);
    if (!m_initialized)
    {
        return;
    }

    _fireStepSubscribers(/*preStep=*/true, elapsed);

    if (m_handle != OVPHYSX_INVALID_HANDLE)
    {
        checkResult(ovphysx_step_sync(m_handle, elapsed), "OvPhysxAdapter::_doSimulate");
    }

    ++m_stepCount;
    ++m_timestamp;

    _fireStepSubscribers(/*preStep=*/false, elapsed);
    _fireContactReportSubscribers();
}

void OvPhysxAdapter::_doFetchResults()
{
    if (!m_initialized)
    {
        return;
    }

    if (m_hasPendingStep && m_handle != OVPHYSX_INVALID_HANDLE)
    {
        ovphysx_op_wait_result_t waitResult{};
        ovphysx_wait_op(m_handle, m_pendingStepOperation, UINT64_MAX, &waitResult);
        ovphysx_destroy_wait_result(&waitResult);
        m_hasPendingStep = false;
    }

    // Post-step subscribers fire here -- once the step is complete -- so they observe settled
    // physics state. Gated on the owed notification, not on m_hasPendingStep: a _doCheckResults
    // poll may already have consumed the op index. fetchResults() is public and may also run
    // with nothing dispatched, or after a synchronous simulate() that fired its post-step
    // inline, and neither owes an event here.
    if (m_postStepPending)
    {
        m_postStepPending = false;
        _fireStepSubscribers(/*preStep=*/false, m_lastStepElapsed);
        _fireContactReportSubscribers();
    }
}

bool OvPhysxAdapter::_doCheckResults()
{
    // Non-blocking readiness poll. The synchronous path completes inside _doSimulate, so
    // only an in-flight asynchronous step can report not-ready.
    if (!m_hasPendingStep || m_handle == OVPHYSX_INVALID_HANDLE)
    {
        return true;
    }

    // A zero timeout makes this a poll rather than a wait.
    ovphysx_op_wait_result_t waitResult{};
    const ovphysx_result_t result = ovphysx_wait_op(m_handle, m_pendingStepOperation, 0, &waitResult);
    ovphysx_destroy_wait_result(&waitResult);

    if (result.status == OVPHYSX_API_TIMEOUT)
    {
        return false;
    }
    if (result.status == OVPHYSX_API_SUCCESS || result.status == OVPHYSX_API_NOT_FOUND)
    {
        // The step finished and the poll consumed the op index (single-use), so
        // _doFetchResults must not wait on it again. It still owes the post-step
        // event, which m_postStepPending carries.
        m_hasPendingStep = false;
    }
    // On an internal error keep the pending op so _doFetchResults' blocking wait resolves it.
    return true;
}

bool OvPhysxAdapter::_doPublishTransformsToStage()
{
    if (!m_initialized || m_ovstage == nullptr || m_handle == OVPHYSX_INVALID_HANDLE)
    {
        return false;
    }

    // Publishing is a read of the latest completed simulation state. Settle an
    // asynchronous step through the adapter first so its operation bookkeeping and
    // post-step notifications remain consistent with an explicit fetchResults().
    _doFetchResults();

    ovstage_ordinal_t writeOrdinal = 0;
    if (!getNextOrdinal(m_ovstage, writeOrdinal))
    {
        return false;
    }

    bool wroteOutput = false;
    bool success = true;
    TransformJournalBatch journal;
    const bool continuesJournalChain = m_lastTransformJournalOrdinal != 0 &&
                                       m_lastTransformJournalOrdinal != std::numeric_limits<ovstage_ordinal_t>::max() &&
                                       writeOrdinal == m_lastTransformJournalOrdinal + 1;
    const ovstage_ordinal_t journalBaseOrdinal = continuesJournalChain ? m_transformJournalBaseOrdinal : writeOrdinal - 1;
    // The initial API supports the two transform categories exercised by the examples.
    constexpr ovphysx_sim_object_type_t kTransformObjectTypes[] = { OVPHYSX_OBJECT_RIGID_BODY,
                                                                    OVPHYSX_OBJECT_ARTICULATION_LINK };
    for (const ovphysx_sim_object_type_t objectType : kTransformObjectTypes)
    {
        success = publishObjectTransforms(
                      m_handle, m_ovstage, objectType, writeOrdinal - 1, writeOrdinal, wroteOutput, journal) &&
                  success;
    }

    // An empty transform set is a successful no-op. Only seal an ordinal that
    // actually received output, since sealing an empty frame needlessly advances
    // every stage consumer.
    if (success && wroteOutput &&
        !publishTransformJournal(m_ovstage, journal, journalBaseOrdinal, writeOrdinal, m_transformJournalGeneration))
    {
        ISAACSIM_LOG_WARN(g_kLogger, "Unable to publish the optional OVPhysX transform journal; consumers will query");
    }
    const bool sealed = success && (!wroteOutput || sealOrdinal(m_ovstage, writeOrdinal));
    if (sealed && wroteOutput)
    {
        m_transformJournalBaseOrdinal = journalBaseOrdinal;
        m_lastTransformJournalOrdinal = writeOrdinal;
    }
    return sealed;
}

void OvPhysxAdapter::_doFlushChanges()
{
    // No-op: ovphysx processes changes internally on step.
}

void OvPhysxAdapter::_doPauseChangeTracking(bool pause)
{
    m_changeTrackingPaused = pause;
}

bool OvPhysxAdapter::_doIsChangeTrackingPaused()
{
    return m_changeTrackingPaused;
}

isaacsim::physics::registration::SubscriptionId OvPhysxAdapter::_doSubscribeContactReport(
    isaacsim::physics::registration::OnContactReportEventFunction callback)
{
    std::lock_guard<std::mutex> lock(m_subscriberMutex);
    size_t subscriptionId = m_nextSubscriptionId++;
    m_contactSubscribers[subscriptionId] = std::move(callback);
    return isaacsim::physics::registration::SubscriptionId(subscriptionId);
}

void OvPhysxAdapter::_doUnsubscribeContactReport(isaacsim::physics::registration::SubscriptionId subscriptionId)
{
    std::lock_guard<std::mutex> lock(m_subscriberMutex);
    m_contactSubscribers.erase(subscriptionId.id);
}

uint32_t OvPhysxAdapter::_doGetTimeStepsPerSecond(long /*stageId*/,
                                                  isaacsim::physics::registration::PathToken /*scenePath*/)
{
    return 60u;
}

uint64_t OvPhysxAdapter::_doGetTimestamp()
{
    return m_timestamp.load();
}

uint64_t OvPhysxAdapter::_doGetStepCount()
{
    return m_stepCount.load();
}

isaacsim::physics::registration::SubscriptionId OvPhysxAdapter::_doSubscribeStepEvents(
    bool preStep, int order, isaacsim::physics::registration::OnPhysicsStepEventFunction callback)
{
    std::lock_guard<std::mutex> lock(m_subscriberMutex);
    size_t subscriptionId = m_nextSubscriptionId++;
    m_stepSubscribers[subscriptionId] = { preStep, order, std::move(callback) };
    return isaacsim::physics::registration::SubscriptionId(subscriptionId);
}

void OvPhysxAdapter::_doUnsubscribeStepEvents(isaacsim::physics::registration::SubscriptionId subscriptionId)
{
    std::lock_guard<std::mutex> lock(m_subscriberMutex);
    m_stepSubscribers.erase(subscriptionId.id);
}

bool OvPhysxAdapter::_doIsCapableOfSimulating(const char** schemaNames, size_t count, bool* isCapable)
{
    static const char* s_kSupportedSchemas[] = {
        "PhysicsRigidBodyAPI",  "PhysicsCollisionAPI",   "PhysicsArticulationRootAPI",
        "PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsSphericalJoint",
        "PhysicsFixedJoint",    "PhysicsDistanceJoint",  "PhysicsD6Joint",
    };
    for (size_t i = 0; i < count; ++i)
    {
        isCapable[i] = false;
        for (auto* supportedSchema : s_kSupportedSchemas)
        {
            if (schemaNames[i] && std::string(schemaNames[i]) == supportedSchema)
            {
                isCapable[i] = true;
                break;
            }
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// Step subscriber dispatch
// ----------------------------------------------------------------------------

void OvPhysxAdapter::_fireStepSubscribers(bool preStep, float elapsed)
{
    // Collect matching subscribers under the lock, then fire outside it.
    std::vector<std::pair<int, isaacsim::physics::registration::OnPhysicsStepEventFunction>> subscribers;
    {
        std::lock_guard<std::mutex> lock(m_subscriberMutex);
        for (auto& subscriberEntry : m_stepSubscribers)
        {
            if (subscriberEntry.second.preStep == preStep)
            {
                subscribers.emplace_back(subscriberEntry.second.order, subscriberEntry.second.callback);
            }
        }
    }
    std::sort(subscribers.begin(), subscribers.end(),
              [](const auto& left, const auto& right) { return left.first < right.first; });

    isaacsim::physics::registration::PhysicsStepContext context{};
    context.simulationId = isaacsim::physics::registration::SimulationId(m_simulationId);
    context.scenePath = 0;

    for (auto& subscriber : subscribers)
    {
        try
        {
            subscriber.second(elapsed, context);
        }
        catch (...)
        {
        }
    }
}

void OvPhysxAdapter::_fireContactReportSubscribers()
{
    // Collect matching subscribers under the lock, then fire outside it (mirrors
    // _fireStepSubscribers). Skip the engine query entirely when nobody is listening.
    std::vector<isaacsim::physics::registration::OnContactReportEventFunction> subscribers;
    {
        std::lock_guard<std::mutex> lock(m_subscriberMutex);
        subscribers.reserve(m_contactSubscribers.size());
        for (auto& subscriberEntry : m_contactSubscribers)
        {
            subscribers.push_back(subscriberEntry.second);
        }
    }
    if (subscribers.empty() || m_handle == OVPHYSX_INVALID_HANDLE)
    {
        return;
    }

    const ovphysx_contact_event_header_t* eventHeaders = nullptr;
    uint32_t eventHeaderCount = 0;
    const ovphysx_contact_point_t* contactPoints = nullptr;
    uint32_t contactPointCount = 0;
    const ovphysx_friction_anchor_t* frictionAnchors = nullptr;
    uint32_t frictionAnchorCount = 0;
    const ovphysx_result_t result =
        ovphysx_get_contact_report(m_handle, &eventHeaders, &eventHeaderCount, &contactPoints, &contactPointCount,
                                   &frictionAnchors, &frictionAnchorCount);
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t errorString = ovphysx_get_last_error();
        const std::string_view error(errorString.ptr ? errorString.ptr : "", errorString.ptr ? errorString.length : 0);
        ISAACSIM_LOG_ERROR(
            g_kLogger, "OvPhysxAdapter: ovphysx_get_contact_report failed, contact report dropped: {}", error);
        return;
    }

    // Copy out of the engine's buffers: they are read-only views invalidated by the next step,
    // and a subscriber may outlive this call's scope in its own bookkeeping.
    isaacsim::physics::registration::ContactEventHeaderVector headers;
    headers.reserve(eventHeaderCount);
    for (uint32_t i = 0; i < eventHeaderCount; ++i)
    {
        const ovphysx_contact_event_header_t& source = eventHeaders[i];
        isaacsim::physics::registration::ContactEventHeader header{};
        header.type = convertToContactEventType(source.type);
        header.stageId = source.stageId;
        header.actor0 = static_cast<isaacsim::physics::registration::PathToken>(source.actor0);
        header.actor1 = static_cast<isaacsim::physics::registration::PathToken>(source.actor1);
        header.collider0 = static_cast<isaacsim::physics::registration::PathToken>(source.collider0);
        header.collider1 = static_cast<isaacsim::physics::registration::PathToken>(source.collider1);
        header.contactDataOffset = source.contactDataOffset;
        header.contactDataCount = source.numContactData;
        header.frictionAnchorsDataOffset = source.frictionAnchorsDataOffset;
        header.frictionAnchorDataCount = source.numfrictionAnchorsData;
        header.prototypeIndex0 = source.protoIndex0;
        header.prototypeIndex1 = source.protoIndex1;
        headers.push_back(header);
    }

    isaacsim::physics::registration::ContactDataVector contactData;
    contactData.reserve(contactPointCount);
    for (uint32_t i = 0; i < contactPointCount; ++i)
    {
        const ovphysx_contact_point_t& source = contactPoints[i];
        isaacsim::physics::registration::ContactData point{};
        point.position = { source.position[0], source.position[1], source.position[2] };
        point.normal = { source.normal[0], source.normal[1], source.normal[2] };
        point.impulse = { source.impulse[0], source.impulse[1], source.impulse[2] };
        point.separation = source.separation;
        point.faceIndex0 = source.faceIndex0;
        point.faceIndex1 = source.faceIndex1;
        point.material0 = static_cast<isaacsim::physics::registration::PathToken>(source.material0);
        point.material1 = static_cast<isaacsim::physics::registration::PathToken>(source.material1);
        contactData.push_back(point);
    }

    isaacsim::physics::registration::FrictionAnchorsDataVector anchors;
    anchors.reserve(frictionAnchorCount);
    for (uint32_t i = 0; i < frictionAnchorCount; ++i)
    {
        const ovphysx_friction_anchor_t& source = frictionAnchors[i];
        isaacsim::physics::registration::FrictionAnchor anchor{};
        anchor.position = { source.position[0], source.position[1], source.position[2] };
        anchor.impulse = { source.impulse[0], source.impulse[1], source.impulse[2] };
        anchors.push_back(anchor);
    }

    for (auto& callback : subscribers)
    {
        try
        {
            callback(headers, contactData, anchors);
        }
        catch (...)
        {
            ISAACSIM_LOG_ERROR(g_kLogger, "OvPhysxAdapter: a contact-report subscriber threw; continuing");
        }
    }
}

// ----------------------------------------------------------------------------
// SceneQueryFunctions
// ----------------------------------------------------------------------------

void OvPhysxAdapter::_fillSceneQueryFunctions(isaacsim::physics::registration::SceneQueryFunctions& sceneQueryFunctions)
{
    sceneQueryFunctions.raycastClosest = [this](const isaacsim::physics::registration::Float3& origin,
                                                const isaacsim::physics::registration::Float3& direction, float distance,
                                                isaacsim::physics::registration::RaycastHit& hit, bool both) -> bool
    {
        const float originCoordinates[3]{ origin.x, origin.y, origin.z };
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_raycast(m_handle, originCoordinates, directionCoordinates, distance, both,
                        OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
        if (count == 0)
        {
            return false;
        }
        hit = convertToRaycastHit(hits[0]);
        return true;
    };

    sceneQueryFunctions.raycastAny = [this](const isaacsim::physics::registration::Float3& origin,
                                            const isaacsim::physics::registration::Float3& direction, float distance,
                                            bool both) -> bool
    {
        const float originCoordinates[3]{ origin.x, origin.y, origin.z };
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_raycast(m_handle, originCoordinates, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_ANY,
                        &hits, &count);
        return count > 0;
    };

    sceneQueryFunctions.raycastAll = [this](const isaacsim::physics::registration::Float3& origin,
                                            const isaacsim::physics::registration::Float3& direction, float distance,
                                            isaacsim::physics::registration::RaycastHitReportFunction callback, bool both)
    {
        const float originCoordinates[3]{ origin.x, origin.y, origin.z };
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_raycast(m_handle, originCoordinates, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_ALL,
                        &hits, &count);
        for (uint32_t i = 0; i < count; ++i)
        {
            if (!callback(convertToRaycastHit(hits[i])))
            {
                break;
            }
        }
    };

    sceneQueryFunctions.sweepSphereClosest = [this](float radius, const isaacsim::physics::registration::Float3& origin,
                                                    const isaacsim::physics::registration::Float3& direction,
                                                    float distance, isaacsim::physics::registration::SweepHit& hit,
                                                    bool both) -> bool
    {
        auto geometry = makeSphereGeometry(radius, origin);
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_sweep(
            m_handle, &geometry, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
        if (count == 0)
        {
            return false;
        }
        hit = convertToSweepHit(hits[0]);
        return true;
    };

    sceneQueryFunctions.sweepSphereAny = [this](float radius, const isaacsim::physics::registration::Float3& origin,
                                                const isaacsim::physics::registration::Float3& direction,
                                                float distance, bool both) -> bool
    {
        auto geometry = makeSphereGeometry(radius, origin);
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_sweep(
            m_handle, &geometry, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_ANY, &hits, &count);
        return count > 0;
    };

    sceneQueryFunctions.sweepSphereAll = [this](float radius, const isaacsim::physics::registration::Float3& origin,
                                                const isaacsim::physics::registration::Float3& direction, float distance,
                                                isaacsim::physics::registration::SweepHitReportFunction callback,
                                                bool both)
    {
        auto geometry = makeSphereGeometry(radius, origin);
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_sweep(
            m_handle, &geometry, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_ALL, &hits, &count);
        for (uint32_t i = 0; i < count; ++i)
        {
            if (!callback(convertToSweepHit(hits[i])))
            {
                break;
            }
        }
    };

    sceneQueryFunctions.sweepBoxClosest = [this](const isaacsim::physics::registration::Float3& halfExtent,
                                                 const isaacsim::physics::registration::Float3& position,
                                                 const isaacsim::physics::registration::Float4& rotation,
                                                 const isaacsim::physics::registration::Float3& direction, float distance,
                                                 isaacsim::physics::registration::SweepHit& hit, bool both) -> bool
    {
        auto geometry = makeBoxGeometry(halfExtent, position, rotation);
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_sweep(
            m_handle, &geometry, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
        if (count == 0)
        {
            return false;
        }
        hit = convertToSweepHit(hits[0]);
        return true;
    };

    sceneQueryFunctions.sweepBoxAny = [this](const isaacsim::physics::registration::Float3& halfExtent,
                                             const isaacsim::physics::registration::Float3& position,
                                             const isaacsim::physics::registration::Float4& rotation,
                                             const isaacsim::physics::registration::Float3& direction, float distance,
                                             bool both) -> bool
    {
        auto geometry = makeBoxGeometry(halfExtent, position, rotation);
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_sweep(
            m_handle, &geometry, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_ANY, &hits, &count);
        return count > 0;
    };

    sceneQueryFunctions.sweepBoxAll = [this](const isaacsim::physics::registration::Float3& halfExtent,
                                             const isaacsim::physics::registration::Float3& position,
                                             const isaacsim::physics::registration::Float4& rotation,
                                             const isaacsim::physics::registration::Float3& direction, float distance,
                                             isaacsim::physics::registration::SweepHitReportFunction callback, bool both)
    {
        auto geometry = makeBoxGeometry(halfExtent, position, rotation);
        const float directionCoordinates[3]{ direction.x, direction.y, direction.z };
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_sweep(
            m_handle, &geometry, directionCoordinates, distance, both, OVPHYSX_SCENE_QUERY_MODE_ALL, &hits, &count);
        for (uint32_t i = 0; i < count; ++i)
        {
            if (!callback(convertToSweepHit(hits[i])))
            {
                break;
            }
        }
    };

    // Shape-based sweep/overlap: the C API takes a prim path string. The manager
    // passes an encoded PathToken (uint64 SdfPath). Decode to string is not available
    // without USD; leave these as no-ops for now -- they match the Python behavior
    // which also required a USD prim path but was never exercised by the manager tests.
    sceneQueryFunctions.sweepShapeClosest = nullptr;
    sceneQueryFunctions.sweepShapeAny = nullptr;
    sceneQueryFunctions.sweepShapeAll = nullptr;

    sceneQueryFunctions.overlapSphere = [this](float radius, const isaacsim::physics::registration::Float3& position,
                                               isaacsim::physics::registration::OverlapHitReportFunction callback) -> uint32_t
    {
        auto geometry = makeSphereGeometry(radius, position);
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_overlap(m_handle, &geometry, OVPHYSX_SCENE_QUERY_MODE_ALL, &hits, &count);
        for (uint32_t i = 0; i < count; ++i)
        {
            if (!callback(convertToOverlapHit(hits[i])))
            {
                break;
            }
        }
        return count;
    };

    sceneQueryFunctions.overlapSphereAny = [this](float radius,
                                                  const isaacsim::physics::registration::Float3& position) -> bool
    {
        auto geometry = makeSphereGeometry(radius, position);
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_overlap(m_handle, &geometry, OVPHYSX_SCENE_QUERY_MODE_ANY, &hits, &count);
        return count > 0;
    };

    sceneQueryFunctions.overlapBox = [this](const isaacsim::physics::registration::Float3& halfExtent,
                                            const isaacsim::physics::registration::Float3& position,
                                            const isaacsim::physics::registration::Float4& rotation,
                                            isaacsim::physics::registration::OverlapHitReportFunction callback) -> uint32_t
    {
        auto geometry = makeBoxGeometry(halfExtent, position, rotation);
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_overlap(m_handle, &geometry, OVPHYSX_SCENE_QUERY_MODE_ALL, &hits, &count);
        for (uint32_t i = 0; i < count; ++i)
        {
            if (!callback(convertToOverlapHit(hits[i])))
            {
                break;
            }
        }
        return count;
    };

    sceneQueryFunctions.overlapBoxAny = [this](const isaacsim::physics::registration::Float3& halfExtent,
                                               const isaacsim::physics::registration::Float3& position,
                                               const isaacsim::physics::registration::Float4& rotation) -> bool
    {
        auto geometry = makeBoxGeometry(halfExtent, position, rotation);
        const ovphysx_scene_query_hit_t* hits = nullptr;
        uint32_t count = 0;
        ovphysx_overlap(m_handle, &geometry, OVPHYSX_SCENE_QUERY_MODE_ANY, &hits, &count);
        return count > 0;
    };

    sceneQueryFunctions.overlapShape = nullptr;
    sceneQueryFunctions.overlapShapeAny = nullptr;
}

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim
