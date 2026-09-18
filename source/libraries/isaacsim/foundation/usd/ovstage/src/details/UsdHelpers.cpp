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

#include "UsdHelpers.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <ovx/path_dictionary/path_dictionary.h>
#include <ovx/path_dictionary/path_dictionary_utils.h>

#include <algorithm>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace usd
{
namespace ovstage
{
namespace details
{

namespace
{
int64_t g_stageId = 1;
std::mutex g_stageMutex;
std::unordered_map<int64_t, StageEntry> g_stageCache;
} // namespace

int64_t registerInstance(ovstage_instance_t* instance, ovstage_ordinal_t nextOrdinal)
{
    std::lock_guard<std::mutex> lock(g_stageMutex);
    const int64_t stageId = g_stageId++;
    g_stageCache[stageId] = { instance, nextOrdinal };
    return stageId;
}

ovstage_instance_t* unregisterInstance(int64_t stageId)
{
    std::lock_guard<std::mutex> lock(g_stageMutex);
    if (auto it = g_stageCache.find(stageId); it != g_stageCache.end())
    {
        ovstage_instance_t* instance = it->second.instance;
        g_stageCache.erase(it);
        return instance;
    }
    return nullptr;
}

ovstage_instance_t* getInstance(int64_t stageId, bool throwIfInvalid)
{
    std::lock_guard<std::mutex> lock(g_stageMutex);
    if (auto it = g_stageCache.find(stageId); it != g_stageCache.end())
    {
        return it->second.instance;
    }
    if (throwIfInvalid)
    {
        throw std::invalid_argument("Invalid stage ID (" + std::to_string(stageId) + ")");
    }
    return nullptr;
}

bool isValidPathString(const std::string& path)
{
    if (path.empty())
        return false;
    for (unsigned char c : path)
    {
        if (c < 0x20 || c == '?' || c == '#' || c == '<' || c == '>' || c == '"' || c == '|' || c == '*')
            return false;
    }
    if (path.size() > 1 && path.back() == '/')
        return false;
    if (path.find("//") != std::string::npos)
        return false;
    return true;
}

bool validatePrimAtPath(int64_t stageId, const std::string& path, bool throwIfInvalid)
{
    ovstage_instance_t* instance = getInstance(stageId, throwIfInvalid);
    if (!instance)
        return false;
    return validatePrimAtPath(instance, path, throwIfInvalid);
}

bool validatePrimAtPath(ovstage_instance_t* instance, const std::string& path, bool throwIfInvalid)
{
    if (!instance)
    {
        if (throwIfInvalid)
            throw std::invalid_argument("Invalid stage");
        return false;
    }
    if (!isValidPathString(path))
    {
        if (throwIfInvalid)
            throw isaacsim::common::exceptions::PrimPathStringError(path);
        return false;
    }
    if (path[0] != '/')
    {
        if (throwIfInvalid)
            throw isaacsim::common::exceptions::PrimPathError(path);
        return false;
    }
    if (path == "/")
    {
        return true;
    }

    static constexpr char s_kUsdPathAttr[] = "usd-path";
    const ovx_string_t pathValue{ path.c_str(), path.size() };
    ovstage_predicate_t predicate{};
    predicate.attribute = { OVX_INVALID_TOKEN, ovx_string_t{ s_kUsdPathAttr, sizeof(s_kUsdPathAttr) - 1 } };
    predicate.op = OVSTAGE_FILTER_OP_IN;
    predicate.values = &pathValue;
    predicate.value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = &predicate;
    filter.count = 1;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_enqueue_result_t enqueueResult = ovstage_query(instance, &filter, nullptr, 0, &query);
    if (enqueueResult.status != OVSTAGE_OK)
    {
        return false;
    }
    waitAndRelease(instance, enqueueResult);

    size_t count = 0;
    ovstage_query_result_t result{};
    if (ovstage_fetch_query_result(instance, query, OVSTAGE_TIMEOUT_INFINITE, &result) == OVSTAGE_OK)
    {
        count = result.total_prim_count;
        ovstage_release_query_result(instance, &result);
    }
    waitAndRelease(instance, ovstage_release_query(instance, query));

    if (throwIfInvalid && !count)
    {
        throw isaacsim::common::exceptions::PrimPathError(path);
    }
    return count > 0;
}

std::string primPathToString(path_dictionary_instance_t* dict, ovx_primpath_t primPath)
{
    std::vector<ovx_token_t> tokenBuffer(64);
    ovx_token_t* tokensForPath = nullptr;
    size_t numTokensForPath = 0;
    size_t numProcessed = 0;
    if (path_dictionary_get_tokens_from_paths(dict, &primPath, 1, tokenBuffer.data(), tokenBuffer.size(),
                                              &tokensForPath, &numTokensForPath, &numProcessed)
                .status != OVX_API_SUCCESS ||
        numProcessed != 1)
    {
        return {};
    }

    std::string result;
    for (size_t j = 0; j < numTokensForPath; ++j)
    {
        ovx_string_t name{};
        if (path_dictionary_get_strings_from_tokens(dict, &tokensForPath[j], 1, &name).status == OVX_API_SUCCESS &&
            name.ptr)
        {
            result += '/';
            result.append(name.ptr, name.length);
        }
    }
    return result;
}

ovstage_ordinal_t consumeOrdinal(int64_t stageId)
{
    std::lock_guard<std::mutex> lock(g_stageMutex);
    auto it = g_stageCache.find(stageId);
    if (it == g_stageCache.end())
        return 0;

    // OVStage can have serialized writers outside Foundation (for example a
    // physics backend publishing simulation output). Reconcile our cached next
    // ordinal with the stage's global floor before every Foundation write so we
    // never reuse an ordinal another writer has already sealed.
    ovstage_ordinal_query_handle_t query = OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE;
    const ovstage_enqueue_result_t enqueue = ovstage_get_attribute_write_floor(it->second.instance, {}, &query);
    if (enqueue.status != OVSTAGE_OK || query == OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE)
        throw std::runtime_error("Unable to query the OVStage write floor");

    ovstage_ordinal_t floor = 0;
    const ovstage_api_status_t fetchStatus =
        ovstage_fetch_ordinal(it->second.instance, query, OVSTAGE_TIMEOUT_INFINITE, &floor);
    waitAndRelease(it->second.instance, enqueue);
    waitAndRelease(it->second.instance, ovstage_release_ordinal_query(it->second.instance, query));
    if (fetchStatus != OVSTAGE_OK || floor == std::numeric_limits<ovstage_ordinal_t>::max())
        throw std::runtime_error("Unable to fetch a usable OVStage write floor");

    it->second.nextOrdinal = std::max(it->second.nextOrdinal, floor + 1);
    return it->second.nextOrdinal++;
}

void waitAndRelease(ovstage_instance_t* instance, ovstage_enqueue_result_t enqueueResult)
{
    if (enqueueResult.status == OVSTAGE_OK)
    {
        ovstage_wait_op(instance, enqueueResult.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
        ovstage_release_op(instance, enqueueResult.op_index);
    }
}

} // namespace details
} // namespace ovstage
} // namespace usd
} // namespace foundation
} // namespace isaacsim
