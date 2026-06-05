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

// clang-format off
#include <pch/UsdPCH.h>
#include <pxr/base/tf/patternMatcher.h>
// clang-format on

#include <isaacsim/core/utils/PrimUtils.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#include <algorithm>

/**
 * @brief Implementation of findMatchingChildren function.
 * @details
 * Finds all immediate children of a root prim whose names match the provided pattern.
 * Uses TfPatternMatcher for pattern matching with case-insensitive and glob-enabled options.
 *
 * @param[in] root The parent prim whose children will be searched.
 * @param[in] pattern The name pattern to match against child prim names.
 * @param[out] primsRet Vector that will be populated with matching child prims.
 *
 * @note If the root prim is invalid, the function returns immediately.
 * @note This function only searches immediate children, not descendants further down the hierarchy.
 */
void isaacsim::core::utils::findMatchingChildren(pxr::UsdPrim root,
                                                 const std::string& pattern,
                                                 std::vector<pxr::UsdPrim>& primsRet)
{
    if (!root)
    {
        return;
    }

    pxr::TfPatternMatcher matcher(pattern, true, true);
    pxr::UsdPrimSiblingRange range = root.GetAllChildren();
    for (auto child : range)
    {
        if (matcher.Match(child.GetName()))
        {
            primsRet.push_back(child);
        }
    }
}

/**
 * @brief Implementation of findMatchingPrimPaths function.
 * @details
 * Performs a hierarchical search through a USD stage to find prims matching a path pattern.
 * The pattern is split into path components, and matching is performed level by level.
 * Optionally filters results by specified API schemas.
 *
 * The algorithm works as follows:
 * 1. Splits the pattern into path components.
 * 2. Prepares each component for pattern matching.
 * 3. Searches the hierarchy level by level, matching each component.
 * 4. Optionally filters by API schema type.
 * 5. Returns the paths of matching prims.
 *
 * @param[in] pattern Path pattern to match against prim paths.
 * @param[in] stageId ID of the USD stage to search.
 * @param[in] api Optional API schema filter. When specified, only returns prims with the given API.
 *
 * @return Vector of strings containing paths of matching prims.
 *
 * @throws std::invalid_argument If the stage ID is invalid or the API type is not supported.
 *
 * @note The pattern supports standard glob-style wildcards for matching path components.
 */
std::vector<std::string> isaacsim::core::utils::findMatchingPrimPaths(const std::string& pattern,
                                                                      long int stageId,
                                                                      const std::string& api)
{
    pxr::UsdStageRefPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    if (!stage)
    {
        throw std::invalid_argument("stage id doesn't correspond to an existing stage");
    }

    std::string trimmedPattern = pxr::TfStringTrim(pattern, "/");
    std::vector<std::string> tokens = pxr::TfStringSplit(trimmedPattern, "/");

    // need to wrap the token patterns in '^' and '$' to prevent matching anywhere in the string
    std::transform(tokens.begin(), tokens.end(), tokens.begin(), [](const std::string& tok) { return '^' + tok + '$'; });

    std::vector<pxr::UsdPrim> roots;
    std::vector<pxr::UsdPrim> matches;
    std::vector<std::string> pathsRet;

    roots.push_back(stage->GetPseudoRoot());

    int numTokens = static_cast<int>(tokens.size());

    for (int i = 0; i < numTokens; i++)
    {
        for (auto& prim : roots)
        {
            findMatchingChildren(prim, tokens[i], matches);
        }

        if (i < numTokens - 1)
        {
            std::swap(roots, matches);
            matches.clear();
        }
    }

    if (!api.empty())
    {
        pxr::TfType type;
        if (api == "articulation")
        {
            type = pxr::TfType::Find<pxr::UsdPhysicsArticulationRootAPI>();
        }
        else if (api == "rigid_body")
        {
            type = pxr::TfType::Find<pxr::UsdPhysicsRigidBodyAPI>();
        }
        else
        {
            throw std::invalid_argument("apis supported: articulation and rigid_body.");
        }
        for (auto& prim : matches)
        {
            if (prim.HasAPI(type))
            {
                pathsRet.push_back(prim.GetPrimPath().GetString());
            }
        }
    }
    else
    {
        for (auto& prim : matches)
        {
            pathsRet.push_back(prim.GetPrimPath().GetString());
        }
    }

    return pathsRet;
}
