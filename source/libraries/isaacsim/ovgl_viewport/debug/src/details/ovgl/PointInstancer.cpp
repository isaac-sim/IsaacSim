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

/* PointInstancer expansion for the portable ovstage -> OVGL bridge.
 *
 * This deliberately reuses the already-built SceneMesh prototype payloads.
 * It keeps material/UV/topology handling in Scene.cpp authoritative while
 * adding the missing USD PointInstancer placements as deep-owned mesh draws.
 * The CPU expansion is bounded; the Scene compact-instance fields remain the
 * future large-instance path.
 */

#include "PointInstancer.h"

#include "details/OvstageHelpers.hpp"

#include <ovx/path_dictionary/path_dictionary.h>
#include <ovx/types.h>

#if defined(OVGL_HAS_CUDA)
#    include <cuda_runtime.h>
#endif

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{

thread_local std::string g_error;

bool finishEnqueue(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue)
{
    if (!stage || enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_INVALID_OP_ID)
        return false;
    ovstage_op_wait_result_t wait{};
    const ovstage_api_status_t waited = ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait);
    const ovstage_api_status_t released = ovstage_release_op(stage, enqueue.op_index);
    return waited == OVSTAGE_OK && wait.error_op_id_count == 0 && released == OVSTAGE_OK;
}

struct ScopedPathListReference
{
    path_dictionary_instance_t* dictionary = nullptr;
    ovx_primpath_list_t handle = OVX_INVALID_PRIMPATH_LIST;
    ~ScopedPathListReference()
    {
        if (dictionary && handle != OVX_INVALID_PRIMPATH_LIST)
            (void)path_dictionary_release_path_list_reference(dictionary, handle);
    }
};

struct ScopedQueryHandle
{
    ovstage_instance_t* stage = nullptr;
    ovstage_query_handle_t handle = OVSTAGE_INVALID_QUERY_HANDLE;
    ~ScopedQueryHandle()
    {
        if (stage && handle != OVSTAGE_INVALID_QUERY_HANDLE)
            (void)finishEnqueue(stage, ovstage_release_query(stage, handle));
    }
};

struct ScopedReadHandle
{
    ovstage_instance_t* stage = nullptr;
    ovstage_read_handle_t handle = OVSTAGE_INVALID_READ_HANDLE;
    ~ScopedReadHandle()
    {
        if (stage && handle != OVSTAGE_INVALID_READ_HANDLE)
            (void)finishEnqueue(stage, ovstage_release_read(stage, handle));
    }
};

enum class ReadStatus
{
    Missing,
    Valid,
    Invalid
};

struct HostAttribute
{
    std::vector<uint8_t> bytes;
    DLDataType dtype{ kDLUInt, 8, 1 };
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
    size_t element_count = 0;
    bool is_array = false;
};

struct Orientation
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct PointInstancer
{
    std::string path;
    std::vector<std::string> prototypes;
    std::vector<int32_t> proto_indices;
    std::vector<float> positions;
    std::vector<float> scales;
    std::vector<Orientation> orientations;
    std::vector<int64_t> ids;
    std::unordered_set<int64_t> masked_ids;
    double world[16]{};
    bool included = true;
};

ovx_string_t toOvx(const std::string& value)
{
    ovx_string_t output{};
    output.ptr = value.c_str();
    output.length = value.size();
    return output;
}

ovx_token_t internAttribute(ovstage_instance_t* stage, const std::string& name)
{
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
        return OVX_INVALID_TOKEN;
    const ovx_string_t value = toOvx(name);
    ovx_token_t token = OVX_INVALID_TOKEN;
    if (path_dictionary_create_tokens_from_strings(dictionary, &value, 1, &token).status != OVX_API_SUCCESS)
    {
        return OVX_INVALID_TOKEN;
    }
    return token;
}

ReadStatus readAttribute(ovstage_instance_t* stage,
                         ovstage_ordinal_t ordinal,
                         const std::string& path,
                         const std::string& name,
                         HostAttribute& output,
                         size_t maximum_bytes = std::numeric_limits<size_t>::max())
{
    output = HostAttribute{};
    const ovx_token_t token = internAttribute(stage, name);
    if (token == OVX_INVALID_TOKEN)
    {
        g_error = "could not intern " + name;
        return ReadStatus::Invalid;
    }
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
    {
        g_error = "stage has no path dictionary";
        return ReadStatus::Invalid;
    }
    const ovx_string_t path_string = toOvx(path);
    ovx_primpath_list_t path_list = OVX_INVALID_PRIMPATH_LIST;
    ScopedPathListReference path_ref{ dictionary, path_list };
    if (path_dictionary_create_path_list_from_strings(dictionary, &path_string, 1, &path_list).status != OVX_API_SUCCESS ||
        path_list == OVX_INVALID_PRIMPATH_LIST)
    {
        g_error = "could not intern path " + path;
        return ReadStatus::Invalid;
    }
    path_ref.handle = path_list;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ScopedQueryHandle query_ref{ stage, query };
    if (ovstage_query_from_path_list(stage, path_list, &query) != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        g_error = "could not query " + path;
        return ReadStatus::Invalid;
    }
    query_ref.handle = query;

    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    ScopedReadHandle read_ref{ stage, read };
    const ovstage_ordinal_range_t range{ 0, ordinal, false };
    const auto begin = ovstage_read_attributes(stage, query, &token, 1, range, &read);
    read_ref.handle = read;
    if (begin.status != OVSTAGE_OK || read == OVSTAGE_INVALID_READ_HANDLE)
    {
        return ReadStatus::Missing;
    }

    ReadStatus result = ReadStatus::Missing;
    for (;;)
    {
        ovstage_read_group_t group{};
        const ovstage_api_status_t next = ovstage_fetch_read_next(stage, read, OVSTAGE_TIMEOUT_INFINITE, &group);
        if (next == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        if (next != OVSTAGE_OK)
        {
            g_error = "could not read " + path + "." + name;
            result = ReadStatus::Invalid;
            break;
        }
        if (result == ReadStatus::Missing && !group.is_delete)
        {
            if (group.data.tensor_count < 1 || !group.data.tensors || group.prims.count == 0)
            {
                g_error = "invalid read group for " + path + "." + name;
                result = ReadStatus::Invalid;
            }
            else
            {
                const DLTensor& value = group.data.tensors[0];
                const size_t element_bytes = (static_cast<size_t>(value.dtype.bits) * value.dtype.lanes + 7) / 8;
                bool valid = element_bytes != 0 && value.ndim == 1 && value.shape && value.shape[0] >= 0;
                size_t total_bytes = 0;
                if (valid && static_cast<uint64_t>(value.shape[0]) <= std::numeric_limits<size_t>::max() / element_bytes)
                {
                    total_bytes = static_cast<size_t>(value.shape[0]) * element_bytes;
                }
                else
                {
                    valid = false;
                }

                size_t first = 0;
                size_t last = total_bytes;
                /* Legacy CSR offsets tolerance — FOREIGN producers only. The
                 * in-repo engine emits official transports since the
                 * read-layout port, and every read here queries ONE prim, so
                 * an engine group is tensor_count == 1 (a 1-row ragged group's
                 * single row IS tensors[0]) and this branch stays dead. */
                if (valid && group.data.tensor_count >= 2)
                {
                    const DLTensor& offsets = group.data.tensors[1];
                    valid = offsets.data && offsets.device.device_type == kDLCPU && offsets.dtype.code == kDLUInt &&
                            offsets.dtype.bits == 64 && offsets.dtype.lanes == 1 && offsets.ndim == 1 &&
                            offsets.shape && offsets.shape[0] >= 2;
                    if (valid)
                    {
                        const uint8_t* bytes = static_cast<const uint8_t*>(offsets.data) + offsets.byte_offset;
                        uint64_t begin_offset = 0;
                        uint64_t end_offset = 0;
                        std::memcpy(&begin_offset, bytes, sizeof(begin_offset));
                        std::memcpy(&end_offset, bytes + sizeof(begin_offset), sizeof(end_offset));
                        valid = begin_offset <= std::numeric_limits<size_t>::max() &&
                                end_offset <= std::numeric_limits<size_t>::max();
                        if (valid)
                        {
                            first = static_cast<size_t>(begin_offset);
                            last = static_cast<size_t>(end_offset);
                        }
                    }
                }
                valid = valid && last >= first && last <= total_bytes && first % element_bytes == 0 &&
                        last % element_bytes == 0 && ordinal <= group.meta.attribute_write_floor_ordinal &&
                        (last == first || value.data);
                if (!valid)
                {
                    g_error = "invalid value storage for " + path + "." + name;
                    result = ReadStatus::Invalid;
                }
                else if (last - first > maximum_bytes)
                {
                    /* The CPU expansion cap is also an ingestion cap. Reject
                     * from tensor metadata before allocating HostAttribute
                     * storage or copying a potentially hostile payload. */
                    g_error = "PointInstancer " + path + "." + name + " exceeds OVGL CPU expansion limit";
                    result = ReadStatus::Invalid;
                }
                else
                {
                    try
                    {
                        output.bytes.resize(last - first);
                    }
                    catch (const std::bad_alloc&)
                    {
                        g_error = "allocating " + path + "." + name + " failed";
                        result = ReadStatus::Invalid;
                    }
                    if (result != ReadStatus::Invalid && last != first)
                    {
                        const uint8_t* source = static_cast<const uint8_t*>(value.data) + value.byte_offset + first;
                        if (value.device.device_type == kDLCPU)
                        {
                            std::memcpy(output.bytes.data(), source, last - first);
                        }
                        else if (value.device.device_type == kDLCUDA)
                        {
#if defined(OVGL_HAS_CUDA)
                            const cudaError_t copy =
                                cudaMemcpy(output.bytes.data(), source, last - first, cudaMemcpyDeviceToHost);
                            if (copy != cudaSuccess)
                            {
                                g_error = "copying " + path + "." + name + " from CUDA failed";
                                result = ReadStatus::Invalid;
                            }
#else
                            g_error = "CUDA-resident " + path + "." + name + " is unsupported by this OVGL build";
                            result = ReadStatus::Invalid;
#endif
                        }
                        else
                        {
                            g_error = "unsupported tensor device for " + path + "." + name;
                            result = ReadStatus::Invalid;
                        }
                    }
                    if (result != ReadStatus::Invalid)
                    {
                        output.dtype = value.dtype;
                        output.semantic = group.semantic;
                        output.element_count = (last - first) / element_bytes;
                        output.is_array = group.is_array;
                        result = ReadStatus::Valid;
                    }
                }
            }
        }
        if (ovstage_release_group(stage, &group) != OVSTAGE_OK)
        {
            g_error = "could not release read group for " + path + "." + name;
            result = ReadStatus::Invalid;
        }
        if (result == ReadStatus::Invalid)
            break;
    }
    if (!finishEnqueue(stage, begin))
    {
        g_error = "could not complete read for " + path + "." + name;
        result = ReadStatus::Invalid;
    }
    return result;
}

template <typename T>
ReadStatus readArray(ovstage_instance_t* stage,
                     ovstage_ordinal_t ordinal,
                     const std::string& path,
                     const char* name,
                     uint8_t code,
                     uint8_t bits,
                     uint16_t lanes,
                     std::vector<T>& output,
                     size_t maximum_elements = std::numeric_limits<size_t>::max())
{
    HostAttribute value;
    output.clear();
    size_t maximum_bytes = std::numeric_limits<size_t>::max();
    if (maximum_elements != std::numeric_limits<size_t>::max())
    {
        if (maximum_elements > std::numeric_limits<size_t>::max() / sizeof(T))
        {
            g_error = "PointInstancer " + path + "." + name + " input bound overflow";
            return ReadStatus::Invalid;
        }
        maximum_bytes = maximum_elements * sizeof(T);
    }
    const ReadStatus status = readAttribute(stage, ordinal, path, name, value, maximum_bytes);
    if (status != ReadStatus::Valid)
        return status;
    if (!value.is_array || value.dtype.code != code || value.dtype.bits != bits || value.dtype.lanes != lanes ||
        value.bytes.size() % sizeof(T) != 0)
    {
        g_error = "invalid PointInstancer attribute type at " + path + "." + name;
        return ReadStatus::Invalid;
    }
    output.resize(value.bytes.size() / sizeof(T));
    if (!output.empty())
    {
        std::memcpy(output.data(), value.bytes.data(), value.bytes.size());
    }
    return ReadStatus::Valid;
}

std::vector<std::string> resolvePathIds(ovstage_instance_t* stage, const std::vector<uint8_t>& bytes)
{
    if (bytes.empty() || bytes.size() % sizeof(ovx_primpath_t) != 0)
        return {};
    std::vector<ovx_primpath_t> paths(bytes.size() / sizeof(ovx_primpath_t));
    std::memcpy(paths.data(), bytes.data(), bytes.size());
    if (std::find(paths.begin(), paths.end(), OVX_INVALID_PRIMPATH) != paths.end())
    {
        return {};
    }
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
        return {};
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_paths(dictionary, paths.data(), paths.size(), &list).status !=
            OVX_API_SUCCESS ||
        list == OVX_INVALID_PRIMPATH_LIST)
    {
        return {};
    }
    std::vector<std::string> output = isaacsim::ovgl_viewport::debug::details::resolvePrimPaths(dictionary, list);
    path_dictionary_release_path_list_reference(dictionary, list);
    return output.size() == paths.size() ? output : std::vector<std::string>{};
}

ReadStatus readPrototypes(ovstage_instance_t* stage,
                          ovstage_ordinal_t ordinal,
                          const std::string& path,
                          std::vector<std::string>& output)
{
    HostAttribute value;
    output.clear();
    const ReadStatus status = readAttribute(stage, ordinal, path, "prototypes", value);
    if (status != ReadStatus::Valid)
        return status;
    if (!value.is_array)
    {
        g_error = "PointInstancer prototypes is not an array at " + path;
        return ReadStatus::Invalid;
    }
    if (value.dtype.code == kDLUInt && value.dtype.bits == 8 && value.dtype.lanes == 1)
    {
        size_t begin = 0;
        while (begin < value.bytes.size())
        {
            size_t end = begin;
            while (end < value.bytes.size() && value.bytes[end] != 0)
                ++end;
            if (end == begin || end == value.bytes.size())
            {
                g_error = "malformed PointInstancer prototype path at " + path;
                output.clear();
                return ReadStatus::Invalid;
            }
            output.emplace_back(reinterpret_cast<const char*>(value.bytes.data() + begin), end - begin);
            begin = end + 1;
        }
    }
    else if (value.dtype.code == kDLUInt && value.dtype.bits == 64 && value.dtype.lanes == 1)
    {
        output = resolvePathIds(stage, value.bytes);
        if (output.empty() && !value.bytes.empty())
        {
            g_error = "unresolvable PointInstancer prototype paths at " + path;
            return ReadStatus::Invalid;
        }
    }
    else
    {
        g_error = "unsupported PointInstancer prototype storage at " + path;
        return ReadStatus::Invalid;
    }
    for (const std::string& target : output)
    {
        /* Relationship targets may legally be property paths, but prototypes
         * must target prims. Accepting "/Prim.attr" makes expansion find no
         * source mesh and can turn a malformed instancer into a blank success. */
        if (target.size() <= 1 || target.front() != '/' || target.back() == '/' ||
            target.find("//") != std::string::npos || target.find_first_of(".[]{}") != std::string::npos)
        {
            g_error = "invalid PointInstancer prototype target at " + path;
            output.clear();
            return ReadStatus::Invalid;
        }
    }
    return ReadStatus::Valid;
}

void identity(double output[16])
{
    static const double s_kValue[16] = {
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    };
    std::memcpy(output, s_kValue, sizeof(s_kValue));
}

void multiply(const double left[16], const double right[16], double output[16])
{
    double value[16]{};
    for (int row = 0; row < 4; ++row)
    {
        for (int column = 0; column < 4; ++column)
        {
            for (int k = 0; k < 4; ++k)
            {
                value[row * 4 + column] += left[row * 4 + k] * right[k * 4 + column];
            }
        }
    }
    std::memcpy(output, value, sizeof(value));
}

bool readMatrix(
    ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& path, const char* name, double output[16])
{
    identity(output);
    HostAttribute value;
    const ReadStatus status = readAttribute(stage, ordinal, path, name, value, 16 * sizeof(double));
    if (status == ReadStatus::Missing)
        return true;
    if (status == ReadStatus::Invalid)
        return false;
    if (status != ReadStatus::Valid || value.is_array || value.dtype.code != kDLFloat || value.dtype.bits != 64 ||
        value.dtype.lanes != 16 || value.element_count != 1 || value.bytes.size() != 16 * sizeof(double))
    {
        g_error = "invalid matrix type or shape at " + path + "." + name;
        return false;
    }
    std::memcpy(output, value.bytes.data(), value.bytes.size());
    if (!std::all_of(output, output + 16, [](double component) { return std::isfinite(component); }))
    {
        g_error = "non-finite matrix at " + path + "." + name;
        return false;
    }
    return true;
}

std::string parentPath(const std::string& path)
{
    const size_t slash = path.rfind('/');
    return slash == std::string::npos || slash == 0 ? "/" : path.substr(0, slash);
}

bool sameOrDescendant(const std::string& path, const std::string& root)
{
    return path == root ||
           (path.size() > root.size() && path.compare(0, root.size(), root) == 0 && path[root.size()] == '/');
}

std::string readText(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& path, const char* name)
{
    HostAttribute value;
    /* purpose/visibility are tokens. Keep even malformed dormant metadata from
     * causing an unbounded allocation during the structural-prune pass. */
    constexpr size_t kMaximumTokenBytes = 4096;
    if (readAttribute(stage, ordinal, path, name, value, kMaximumTokenBytes) != ReadStatus::Valid ||
        value.dtype.code != kDLUInt || value.bytes.empty())
    {
        return {};
    }
    /* Official 0.1 (P1.3): TOKEN_ID columns carry interned u64 token ids. */
    if (value.dtype.bits == 64 && value.dtype.lanes == 1 && value.bytes.size() >= 8)
    {
        uint64_t tok = 0;
        std::memcpy(&tok, value.bytes.data(), 8);
        path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
        ovx_string_t sv{};
        if (dict && tok && path_dictionary_get_strings_from_tokens(dict, &tok, 1, &sv).status == OVX_API_SUCCESS &&
            sv.length)
            return std::string(sv.ptr, sv.length);
        return {};
    }
    if (value.dtype.bits != 8 || value.dtype.lanes != 1)
        return {};
    std::string output(reinterpret_cast<const char*>(value.bytes.data()), value.bytes.size());
    const size_t nul = output.find('\0');
    if (nul != std::string::npos)
        output.resize(nul);
    return output;
}

bool includedPurpose(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& prim)
{
    for (std::string path = prim; path.size() > 1; path = parentPath(path))
    {
        const std::string purpose = readText(stage, ordinal, path, "purpose");
        if (purpose == "default" || purpose == "render")
            return true;
        if (purpose == "guide" || purpose == "proxy")
            return false;
    }
    return true;
}

bool includedVisibility(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& prim)
{
    for (std::string path = prim; path.size() > 1; path = parentPath(path))
    {
        if (readText(stage, ordinal, path, "visibility") == "invisible")
            return false;
    }
    return true;
}

ReadStatus readBool(
    ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& path, const char* name, bool& output)
{
    output = false;
    HostAttribute value;
    const ReadStatus status = readAttribute(stage, ordinal, path, name, value, sizeof(uint64_t));
    if (status != ReadStatus::Valid)
        return status;
    if (value.is_array || value.dtype.code != kDLUInt || value.dtype.bits != 8 || value.dtype.lanes != 1 ||
        value.element_count != 1 || value.bytes.size() != 1 || value.bytes[0] > 1)
    {
        g_error = "invalid boolean type, shape, or value at " + path + "." + name;
        return ReadStatus::Invalid;
    }
    output = value.bytes[0] != 0;
    return ReadStatus::Valid;
}

float halfToFloat(uint16_t half)
{
    const uint32_t sign = static_cast<uint32_t>(half & 0x8000u) << 16;
    uint32_t exponent = (half >> 10) & 0x1fu;
    uint32_t mantissa = half & 0x03ffu;
    uint32_t bits = 0;
    if (exponent == 0)
    {
        if (mantissa == 0)
        {
            bits = sign;
        }
        else
        {
            exponent = 127u - 15u + 1u;
            while ((mantissa & 0x0400u) == 0)
            {
                mantissa <<= 1;
                --exponent;
            }
            bits = sign | (exponent << 23) | ((mantissa & 0x03ffu) << 13);
        }
    }
    else if (exponent == 0x1fu)
    {
        bits = sign | 0x7f800000u | (mantissa << 13);
    }
    else
    {
        bits = sign | ((exponent + 112u) << 23) | (mantissa << 13);
    }
    float output = 0.0f;
    std::memcpy(&output, &bits, sizeof(output));
    return output;
}

ReadStatus readOrientations(ovstage_instance_t* stage,
                            ovstage_ordinal_t ordinal,
                            const std::string& path,
                            const char* name,
                            size_t count,
                            size_t maximum_count,
                            std::vector<Orientation>& output)
{
    HostAttribute value;
    output.clear();
    if (maximum_count > std::numeric_limits<size_t>::max() / (4 * sizeof(float)))
    {
        g_error = "PointInstancer " + path + "." + name + " input bound overflow";
        return ReadStatus::Invalid;
    }
    const ReadStatus status = readAttribute(stage, ordinal, path, name, value, maximum_count * 4 * sizeof(float));
    if (status != ReadStatus::Valid)
        return status;
    if (!value.is_array || value.dtype.code != kDLFloat || value.dtype.lanes != 4 ||
        (value.dtype.bits != 16 && value.dtype.bits != 32))
    {
        g_error = "invalid PointInstancer orientation storage at " + path;
        return ReadStatus::Invalid;
    }
    const size_t component_bytes = value.dtype.bits / 8;
    if (value.bytes.size() != count * 4 * component_bytes)
    {
        g_error = "PointInstancer orientation count mismatch at " + path;
        return ReadStatus::Invalid;
    }
    output.resize(count);
    for (size_t index = 0; index < count; ++index)
    {
        if (value.dtype.bits == 16)
        {
            uint16_t q[4];
            std::memcpy(q, value.bytes.data() + index * sizeof(q), sizeof(q));
            output[index] = { halfToFloat(q[0]), halfToFloat(q[1]), halfToFloat(q[2]), halfToFloat(q[3]) };
        }
        else
        {
            float q[4];
            std::memcpy(q, value.bytes.data() + index * sizeof(q), sizeof(q));
            output[index] = { q[0], q[1], q[2], q[3] };
        }
    }
    return ReadStatus::Valid;
}

bool instanceLimit(size_t& output)
{
    const char* text = std::getenv("OVGL_MAX_CPU_POINT_INSTANCES");
    if (!text)
    {
        output = OVGL_POINT_INSTANCER_DEFAULT_CPU_LIMIT;
        return true;
    }
    const auto invalid = [&]()
    {
        g_error =
            "invalid OVGL_MAX_CPU_POINT_INSTANCES; expected a decimal "
            "integer in [1, " +
            std::to_string(OVGL_POINT_INSTANCER_ABSOLUTE_CPU_LIMIT) + "]";
        return false;
    };
    if (!*text)
        return invalid();
    for (const unsigned char* cursor = reinterpret_cast<const unsigned char*>(text); *cursor; ++cursor)
    {
        /* strtoull accepts signs and leading whitespace.  Neither belongs in
         * this security boundary: accept only the canonical decimal grammar. */
        if (*cursor < '0' || *cursor > '9')
            return invalid();
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(text, &end, 10);
    if (errno == ERANGE || !end || *end != '\0' || parsed == 0 || parsed > OVGL_POINT_INSTANCER_ABSOLUTE_CPU_LIMIT)
    {
        return invalid();
    }
    output = static_cast<size_t>(parsed);
    return true;
}

void initializeInstancer(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& path, PointInstancer& output)
{
    output = PointInstancer{};
    output.path = path;
    identity(output.world);
    output.included = includedPurpose(stage, ordinal, path) && includedVisibility(stage, ordinal, path);
}

bool readInstancerPayload(ovstage_instance_t* stage,
                          ovstage_ordinal_t ordinal,
                          PointInstancer& output,
                          size_t& remaining_instances)
{
    const std::string& path = output.path;

    const ReadStatus indices = readArray<int32_t>(
        stage, ordinal, path, "protoIndices", kDLInt, 32, 1, output.proto_indices, remaining_instances);
    if (indices == ReadStatus::Invalid)
        return false;
    const size_t count = output.proto_indices.size();
    if (count > remaining_instances)
    {
        g_error = "PointInstancer " + path + " exceeds OVGL CPU expansion limit";
        return false;
    }
    remaining_instances -= count;
    if (count > std::numeric_limits<size_t>::max() / 3)
    {
        g_error = "PointInstancer " + path + " instance component count overflow";
        return false;
    }
    const size_t maximum_vector_components = count * 3;

    /* Read the bounded cardinality column first. An over-limit instancer must
     * fail before relationship resolution, bulk value copies, or matrix
     * decoding can consume additional memory. */
    const ReadStatus targets = readPrototypes(stage, ordinal, path, output.prototypes);
    const ReadStatus positions = readArray<float>(
        stage, ordinal, path, "positions", kDLFloat, 32, 3, output.positions, maximum_vector_components);
    const ReadStatus scales =
        readArray<float>(stage, ordinal, path, "scales", kDLFloat, 32, 3, output.scales, maximum_vector_components);

    ReadStatus orientations =
        readOrientations(stage, ordinal, path, "orientationsf", output.proto_indices.size(), count, output.orientations);
    if (orientations == ReadStatus::Missing || (orientations == ReadStatus::Valid && output.orientations.empty()))
    {
        orientations = readOrientations(
            stage, ordinal, path, "orientations", output.proto_indices.size(), count, output.orientations);
    }

    const ReadStatus ids = readArray<int64_t>(stage, ordinal, path, "ids", kDLInt, 64, 1, output.ids, count);
    std::vector<int64_t> invisible;
    const ReadStatus invisible_status =
        readArray<int64_t>(stage, ordinal, path, "invisibleIds", kDLInt, 64, 1, invisible, count);
    std::vector<int64_t> inactive;
    const ReadStatus inactive_status =
        readArray<int64_t>(stage, ordinal, path, "inactiveIds", kDLInt, 64, 1, inactive, count);
    output.masked_ids.insert(invisible.begin(), invisible.end());
    output.masked_ids.insert(inactive.begin(), inactive.end());

    if (!readMatrix(stage, ordinal, path, "worldMatrix", output.world))
        return false;

    const bool empty = count == 0;
    if (empty && (positions == ReadStatus::Invalid || scales == ReadStatus::Invalid ||
                  orientations == ReadStatus::Invalid || ids == ReadStatus::Invalid ||
                  invisible_status == ReadStatus::Invalid || inactive_status == ReadStatus::Invalid))
    {
        g_error = "malformed PointInstancer topology or attribute contract at " + path;
        return false;
    }
    const bool empty_payload_is_consistent = !empty || (output.positions.empty() && output.scales.empty() &&
                                                        output.orientations.empty() && output.ids.empty());
    const bool valid =
        indices != ReadStatus::Invalid && positions != ReadStatus::Invalid && targets != ReadStatus::Invalid &&
        scales != ReadStatus::Invalid && orientations != ReadStatus::Invalid && ids != ReadStatus::Invalid &&
        invisible_status != ReadStatus::Invalid && inactive_status != ReadStatus::Invalid &&
        empty_payload_is_consistent &&
        (empty || (indices == ReadStatus::Valid && targets == ReadStatus::Valid && positions == ReadStatus::Valid &&
                   !output.prototypes.empty())) &&
        (empty || output.positions.size() == count * 3) && (output.scales.empty() || output.scales.size() == count * 3) &&
        (output.orientations.empty() || output.orientations.size() == count) &&
        (output.ids.empty() || output.ids.size() == count) &&
        std::all_of(output.proto_indices.begin(), output.proto_indices.end(),
                    [&](int32_t index) { return index >= 0 && static_cast<size_t>(index) < output.prototypes.size(); }) &&
        std::all_of(output.positions.begin(), output.positions.end(), [](float value) { return std::isfinite(value); }) &&
        std::all_of(output.scales.begin(), output.scales.end(), [](float value) { return std::isfinite(value); });
    if (!valid)
    {
        if (g_error.empty())
        {
            g_error = "malformed PointInstancer topology or attribute contract at " + path;
        }
        return false;
    }
    return true;
}

bool collectInstancerPaths(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, std::vector<std::string>& output)
{
    const char* schema = "PointInstancer";
    ovx_string_t schema_value{};
    schema_value.ptr = schema;
    schema_value.length = std::strlen(schema);
    ovstage_predicate_t predicate{};
    predicate.attribute.string.ptr = "usd-prim-type";
    predicate.attribute.string.length = std::strlen("usd-prim-type");
    predicate.op = OVSTAGE_FILTER_OP_IN;
    predicate.values = &schema_value;
    predicate.value_count = 1;
    ovstage_filter_t filter{};
    filter.predicates = &predicate;
    filter.count = 1;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ScopedQueryHandle query_ref{ stage, query };
    const ovstage_enqueue_result_t query_operation = ovstage_query(stage, &filter, nullptr, 0, &query);
    query_ref.handle = query;
    if (query_operation.status != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        g_error = "query(PointInstancer) failed";
        return false;
    }
    ovstage_query_result_t result{};
    const ovstage_api_status_t fetch = ovstage_fetch_query_result(stage, query, OVSTAGE_TIMEOUT_INFINITE, &result);
    if (fetch != OVSTAGE_OK)
    {
        (void)finishEnqueue(stage, query_operation);
        g_error = "fetch_query_result(PointInstancer) failed";
        return false;
    }
    const auto paths = isaacsim::ovgl_viewport::debug::details::resolveQueryPrimPaths(
        stage, query, result.attributes, result.attribute_count, ordinal);
    const bool result_released = ovstage_release_query_result(stage, &result) == OVSTAGE_OK;
    const bool query_finished = finishEnqueue(stage, query_operation);
    if (!paths || !result_released || !query_finished)
    {
        g_error = "query lifecycle(PointInstancer) failed";
        return false;
    }
    output = *paths;
    std::sort(output.begin(), output.end());
    return true;
}

bool prototypeRelativeTransform(ovstage_instance_t* stage,
                                ovstage_ordinal_t ordinal,
                                const std::string& root,
                                const std::string& source,
                                double output[16])
{
    if (!sameOrDescendant(source, root))
        return false;
    identity(output);
    for (std::string path = source;; path = parentPath(path))
    {
        double local[16];
        double composed[16];
        if (!readMatrix(stage, ordinal, path, "localMatrix", local))
            return false;
        multiply(output, local, composed);
        std::memcpy(output, composed, sizeof(composed));
        bool reset_xform_stack = false;
        const ReadStatus reset = readBool(stage, ordinal, path, "resetXformStack", reset_xform_stack);
        if (reset == ReadStatus::Invalid)
            return false;
        if ((reset == ReadStatus::Valid && reset_xform_stack) || path == root)
        {
            return true;
        }
        const std::string parent = parentPath(path);
        if (parent == path || !sameOrDescendant(parent, root))
            return false;
    }
}

bool instanceWorldTransform(ovstage_instance_t* stage,
                            ovstage_ordinal_t ordinal,
                            const PointInstancer& instancer,
                            const std::string& prototype_root,
                            const std::string& source_path,
                            size_t index,
                            double output[16])
{
    double relative[16];
    if (!prototypeRelativeTransform(stage, ordinal, prototype_root, source_path, relative))
    {
        return false;
    }
    double scale[16];
    identity(scale);
    if (!instancer.scales.empty())
    {
        scale[0] = instancer.scales[index * 3];
        scale[5] = instancer.scales[index * 3 + 1];
        scale[10] = instancer.scales[index * 3 + 2];
    }
    double rotation[16];
    identity(rotation);
    if (!instancer.orientations.empty())
    {
        Orientation q = instancer.orientations[index];
        const double length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (length > 1e-15)
        {
            q.x /= length;
            q.y /= length;
            q.z /= length;
            q.w /= length;
        }
        const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        rotation[0] = 1.0 - 2.0 * (yy + zz);
        rotation[1] = 2.0 * (xy + wz);
        rotation[2] = 2.0 * (xz - wy);
        rotation[4] = 2.0 * (xy - wz);
        rotation[5] = 1.0 - 2.0 * (xx + zz);
        rotation[6] = 2.0 * (yz + wx);
        rotation[8] = 2.0 * (xz + wy);
        rotation[9] = 2.0 * (yz - wx);
        rotation[10] = 1.0 - 2.0 * (xx + yy);
    }
    double translation[16];
    identity(translation);
    translation[12] = instancer.positions[index * 3];
    translation[13] = instancer.positions[index * 3 + 1];
    translation[14] = instancer.positions[index * 3 + 2];
    double first[16], second[16], third[16];
    multiply(relative, scale, first);
    multiply(first, rotation, second);
    multiply(second, translation, third);
    multiply(third, instancer.world, output);
    return std::all_of(output, output + 16, [](double value) { return std::isfinite(value); });
}

void releaseMesh(SceneMesh& mesh)
{
    std::free(mesh.positions);
    std::free(mesh.normals);
    std::free(mesh.colors);
    std::free(mesh.texcoords);
    std::free(mesh.indices);
    std::free(mesh.ptex_tri_colors);
    std::free(mesh.path);
    std::memset(&mesh, 0, sizeof(mesh));
}

class OwnedMesh
{
public:
    OwnedMesh() = default;
    ~OwnedMesh()
    {
        releaseMesh(value);
    }

    OwnedMesh(const OwnedMesh&) = delete;
    OwnedMesh& operator=(const OwnedMesh&) = delete;

    SceneMesh value{};

    void relinquish()
    {
        std::memset(&value, 0, sizeof(value));
    }
};

class MeshAdditions
{
public:
    ~MeshAdditions()
    {
        if (m_ownsMeshes)
        {
            for (SceneMesh& mesh : values)
                releaseMesh(mesh);
        }
    }

    std::vector<SceneMesh> values;

    void relinquish()
    {
        m_ownsMeshes = false;
    }

private:
    bool m_ownsMeshes = true;
};

template <typename T>
bool cloneArray(const T* source, size_t count, T*& output)
{
    output = nullptr;
    if (!source || count == 0)
        return true;
    if (count > std::numeric_limits<size_t>::max() / sizeof(T))
        return false;
    output = static_cast<T*>(std::malloc(count * sizeof(T)));
    if (!output)
        return false;
    std::memcpy(output, source, count * sizeof(T));
    return true;
}

void updateMeshBounds(SceneMesh& mesh)
{
    float local_min[3] = { 1e30f, 1e30f, 1e30f };
    float local_max[3] = { -1e30f, -1e30f, -1e30f };
    float world_min[3] = { 1e30f, 1e30f, 1e30f };
    float world_max[3] = { -1e30f, -1e30f, -1e30f };
    for (int vertex = 0; vertex < mesh.nvertices; ++vertex)
    {
        const float* point = mesh.positions + static_cast<size_t>(vertex) * 3;
        for (int component = 0; component < 3; ++component)
        {
            local_min[component] = std::min(local_min[component], point[component]);
            local_max[component] = std::max(local_max[component], point[component]);
            const float transformed =
                static_cast<float>(point[0] * mesh.world_xform[component] + point[1] * mesh.world_xform[4 + component] +
                                   point[2] * mesh.world_xform[8 + component] + mesh.world_xform[12 + component]);
            world_min[component] = std::min(world_min[component], transformed);
            world_max[component] = std::max(world_max[component], transformed);
        }
    }
    std::memcpy(mesh.local_bounds_min, local_min, sizeof(local_min));
    std::memcpy(mesh.local_bounds_max, local_max, sizeof(local_max));
    std::memcpy(mesh.bounds_min, world_min, sizeof(world_min));
    std::memcpy(mesh.bounds_max, world_max, sizeof(world_max));
}

bool cloneMesh(const SceneMesh& source, const double world[16], bool visible, SceneMesh& output)
{
    output = source;
    output.positions = nullptr;
    output.normals = nullptr;
    output.colors = nullptr;
    output.texcoords = nullptr;
    output.indices = nullptr;
    output.ptex_tri_colors = nullptr;
    output.path = nullptr;
    const bool copied = cloneArray(source.positions, static_cast<size_t>(source.nvertices) * 3, output.positions) &&
                        cloneArray(source.normals, static_cast<size_t>(source.nvertices) * 3, output.normals) &&
                        cloneArray(source.colors, static_cast<size_t>(source.nvertices) * 3, output.colors) &&
                        cloneArray(source.texcoords, static_cast<size_t>(source.nvertices) * 2, output.texcoords) &&
                        cloneArray(source.indices, static_cast<size_t>(source.nindices), output.indices) &&
                        cloneArray(source.ptex_tri_colors, static_cast<size_t>(std::max(source.ptex_tri_color_count, 0)),
                                   output.ptex_tri_colors);
    if (!copied)
    {
        releaseMesh(output);
        return false;
    }
    if (source.path)
    {
        const size_t length = std::strlen(source.path) + 1;
        output.path = static_cast<char*>(std::malloc(length));
        if (!output.path)
        {
            releaseMesh(output);
            return false;
        }
        std::memcpy(output.path, source.path, length);
    }
    std::memcpy(output.world_xform, world, sizeof(output.world_xform));
    output.is_proto_only = 0;
    output.visible = visible ? 1 : 0;
    updateMeshBounds(output);
    return true;
}

bool instanceIsVisible(const PointInstancer& instancer, size_t index)
{
    const int64_t id = index < instancer.ids.size() ? instancer.ids[index] : static_cast<int64_t>(index);
    return instancer.masked_ids.find(id) == instancer.masked_ids.end();
}

void updateSceneBounds(Scene& scene)
{
    float lower[3] = { 1e30f, 1e30f, 1e30f };
    float upper[3] = { -1e30f, -1e30f, -1e30f };
    bool found = false;
    for (int index = 0; index < scene.nmeshes; ++index)
    {
        const SceneMesh& mesh = scene.meshes[index];
        if (!mesh.visible || mesh.is_proto_only || mesh.nvertices <= 0)
            continue;
        found = true;
        for (int component = 0; component < 3; ++component)
        {
            lower[component] = std::min(lower[component], mesh.bounds_min[component]);
            upper[component] = std::max(upper[component], mesh.bounds_max[component]);
        }
    }
    if (found)
    {
        std::memcpy(scene.bounds_min, lower, sizeof(lower));
        std::memcpy(scene.bounds_max, upper, sizeof(upper));
    }
}

int expandPointInstancers(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* scene)
{
    if (!stage || !scene || (scene->nmeshes > 0 && !scene->meshes))
    {
        g_error = "invalid PointInstancer expansion argument";
        return 0;
    }
    std::vector<std::string> paths;
    if (!collectInstancerPaths(stage, ordinal, paths))
        return 0;
    if (paths.empty())
        return 1;

    std::vector<PointInstancer> instancers;
    instancers.reserve(paths.size());
    for (const std::string& path : paths)
    {
        PointInstancer instancer;
        initializeInstancer(stage, ordinal, path, instancer);
        instancers.push_back(std::move(instancer));
    }

    /* A PointInstancer is a structural traversal boundary. Even an explicitly
     * render-purpose descendant cannot restart traversal below an excluded
     * ancestor instancer: USD imaging would already have pruned that subtree.
     * Fold this structural exclusion into the descendant before applying the
     * nested-instancer checks or independently expanding each record. */
    for (size_t inner = 0; inner < paths.size(); ++inner)
    {
        for (size_t outer = 0; outer < paths.size(); ++outer)
        {
            if (outer != inner && !instancers[outer].included && sameOrDescendant(paths[inner], paths[outer]))
            {
                instancers[inner].included = false;
                break;
            }
        }
    }

    /* CPU expansion clones ordinary prototype meshes. Treating an included
     * nested PointInstancer as an ordinary prototype would silently discard
     * its placements while producing plausible geometry. Excluded hierarchies
     * are pruned instead: visibility/purpose must be able to hide unsupported
     * content without tripping its expansion limits or structure checks. */
    for (size_t outer = 0; outer < paths.size(); ++outer)
    {
        for (size_t inner = 0; inner < paths.size(); ++inner)
        {
            if (outer != inner && instancers[outer].included && instancers[inner].included &&
                sameOrDescendant(paths[inner], paths[outer]))
            {
                g_error = "nested PointInstancer is unsupported: " + paths[inner] + " below " + paths[outer];
                return 0;
            }
        }
    }

    /* Visibility/purpose exclusion is a structural prune, not merely a draw
     * mask. Read no relationship/array/matrix payload until ancestor exclusion
     * has propagated: hidden instancers may contain malformed, non-finite, or
     * intentionally huge dormant data without consuming the CPU expansion
     * budget or failing an otherwise valid frame. */
    const bool has_included_instancer = std::any_of(
        instancers.begin(), instancers.end(), [](const PointInstancer& instancer) { return instancer.included; });
    size_t maximum = OVGL_POINT_INSTANCER_DEFAULT_CPU_LIMIT;
    if (has_included_instancer && !instanceLimit(maximum))
        return 0;
    size_t remaining_instances = maximum;

    for (PointInstancer& instancer : instancers)
    {
        if (instancer.included && !readInstancerPayload(stage, ordinal, instancer, remaining_instances))
        {
            return 0;
        }
    }

    for (const PointInstancer& instancer : instancers)
    {
        if (!instancer.included)
            continue;
        for (const std::string& root : instancer.prototypes)
        {
            for (size_t candidate = 0; candidate < paths.size(); ++candidate)
            {
                if (instancers[candidate].included && sameOrDescendant(paths[candidate], root))
                {
                    g_error =
                        "PointInstancer prototype hierarchy contains a "
                        "PointInstancer: " +
                        root;
                    return 0;
                }
            }
        }
    }

    const int base_count = scene->nmeshes;
    MeshAdditions additions;
    std::unordered_set<int> internal_prototypes;

    /* USD imaging traversals prune every descendant of a PointInstancer, not
     * only the relationship targets that happen to receive an instance. Mark
     * those source meshes before filtering visibility or instance count so an
     * invisible/empty instancer cannot leak its prototype hierarchy. */
    for (const PointInstancer& instancer : instancers)
    {
        for (int mesh_index = 0; mesh_index < base_count; ++mesh_index)
        {
            const SceneMesh& source = scene->meshes[mesh_index];
            if (source.path && sameOrDescendant(source.path, instancer.path))
            {
                internal_prototypes.insert(mesh_index);
            }
        }
    }

    for (const PointInstancer& instancer : instancers)
    {
        if (!instancer.included)
            continue;
        for (size_t target_index = 0; target_index < instancer.prototypes.size(); ++target_index)
        {
            const std::string& root = instancer.prototypes[target_index];
            for (int mesh_index = 0; mesh_index < base_count; ++mesh_index)
            {
                const SceneMesh& source = scene->meshes[mesh_index];
                if (!source.path || !sameOrDescendant(source.path, root))
                    continue;
                for (size_t instance_index = 0; instance_index < instancer.proto_indices.size(); ++instance_index)
                {
                    if (instancer.proto_indices[instance_index] != static_cast<int32_t>(target_index))
                    {
                        continue;
                    }
                    if (additions.values.size() >= maximum)
                    {
                        g_error = "PointInstancer expansion exceeds OVGL draw limit";
                        return 0;
                    }
                    double world[16];
                    if (!instanceWorldTransform(stage, ordinal, instancer, root, source.path, instance_index, world))
                    {
                        if (g_error.empty())
                        {
                            g_error =
                                "failed to compose PointInstancer transform for " + instancer.path + " using " + root;
                        }
                        return 0;
                    }
                    OwnedMesh clone;
                    if (!cloneMesh(
                            source, world, source.visible && instanceIsVisible(instancer, instance_index), clone.value))
                    {
                        g_error = "allocating PointInstancer mesh payload failed";
                        return 0;
                    }
                    /* Preserve the source index as provenance. Besides matching
                     * SceneMesh's instance contract, this lets the transform
                     * fast path distinguish expanded draws whose matrix cannot
                     * be refreshed by reading the shared prototype path alone. */
                    clone.value.prototype_idx = mesh_index;
                    additions.values.push_back(clone.value);
                    clone.relinquish();
                }
            }
        }
    }
    if (additions.values.empty())
    {
        for (int index : internal_prototypes)
        {
            scene->meshes[index].is_proto_only = 1;
            scene->meshes[index].visible = 0;
        }
        updateSceneBounds(*scene);
        return 1;
    }
    if (static_cast<size_t>(base_count) + additions.values.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
    {
        g_error = "expanded PointInstancer mesh count exceeds signed range";
        return 0;
    }
    const size_t total = static_cast<size_t>(base_count) + additions.values.size();
    SceneMesh* combined = static_cast<SceneMesh*>(std::malloc(total * sizeof(SceneMesh)));
    if (!combined)
    {
        g_error = "allocating expanded PointInstancer scene failed";
        return 0;
    }
    if (base_count > 0)
    {
        std::memcpy(combined, scene->meshes, static_cast<size_t>(base_count) * sizeof(SceneMesh));
    }
    std::memcpy(combined + base_count, additions.values.data(), additions.values.size() * sizeof(SceneMesh));
    additions.relinquish();
    std::free(scene->meshes);
    scene->meshes = combined;
    scene->nmeshes = static_cast<int>(total);
    for (int index : internal_prototypes)
    {
        scene->meshes[index].is_proto_only = 1;
        scene->meshes[index].visible = 0;
    }
    updateSceneBounds(*scene);
    return 1;
}

} // namespace

extern "C" int ovgl_expand_point_instancers(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* scene)
{
    g_error.clear();
    try
    {
        return expandPointInstancers(stage, ordinal, scene);
    }
    catch (const std::bad_alloc&)
    {
        g_error = "allocating PointInstancer state failed";
        return 0;
    }
    catch (...)
    {
        g_error = "unexpected PointInstancer expansion failure";
        return 0;
    }
}

extern "C" const char* ovgl_point_instancer_last_error(void)
{
    return g_error.c_str();
}
