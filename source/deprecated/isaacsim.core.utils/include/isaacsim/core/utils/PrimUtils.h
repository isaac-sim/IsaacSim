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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <string>

#ifdef _MSC_VER
#    if ISAACSIM_CORE_UTILS_EXPORT
#        define PRIMUTILS_DLL_EXPORT __declspec(dllexport)
#    else
#        define PRIMUTILS_DLL_EXPORT __declspec(dllimport)
#    endif
#else
#    define PRIMUTILS_DLL_EXPORT
#endif

namespace isaacsim
{
namespace core
{
namespace utils
{

/**
 * @brief Finds USD prims matching a specified path pattern.
 * @details
 * Searches for prims in a USD stage that match the provided path pattern.
 * The pattern supports wildcard matching for path components.
 * Optionally filters results by specified API schemas.
 *
 * @param[in] pattern Path pattern to match against prim paths.
 * @param[in] stageId ID of the USD stage to search.
 * @param[in] api Optional API schema filter. When specified, only returns prims with the given API.
 *                Current supported values are "articulation" and "rigid_body".
 *
 * @return Vector of strings containing paths of matching prims.
 *
 * @throws std::invalid_argument If the stage ID is invalid or the API type is not supported.
 *
 * @note The pattern supports standard glob-style wildcards for matching.
 */
PRIMUTILS_DLL_EXPORT std::vector<std::string> findMatchingPrimPaths(const std::string& pattern,
                                                                    long int stageId,
                                                                    const std::string& api = std::string(""));

/**
 * @brief Finds child prims matching a specified name pattern.
 * @details
 * Searches immediate children of a root prim for names matching the specified pattern.
 * Uses TfPatternMatcher for name comparison.
 *
 * @param[in] root The parent prim whose children will be searched.
 * @param[in] pattern The name pattern to match against child prim names.
 * @param[out] primsRet Vector that will be populated with matching child prims.
 *
 * @note If the root prim is invalid, the function returns without modifying primsRet.
 * @note The pattern uses TfPatternMatcher syntax which supports glob-style wildcards.
 */
PRIMUTILS_DLL_EXPORT void findMatchingChildren(pxr::UsdPrim root,
                                               const std::string& pattern,
                                               std::vector<pxr::UsdPrim>& primsRet);
}
}
}
