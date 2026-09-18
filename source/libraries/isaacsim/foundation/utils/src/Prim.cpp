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

#include <isaacsim/foundation/usd/openusd/Usd.hpp>
#include <isaacsim/foundation/utils/Prim.hpp>
#include <isaacsim/foundation/utils/Stage.hpp>

#include <deque>

namespace isaacsim
{
namespace foundation
{
namespace utils
{

namespace openusd = isaacsim::foundation::usd::openusd;

std::vector<std::string> findMatchingPrimPaths(const std::string& path, bool traverse)
{
    return openusd::findMatchingPrimPaths(getActiveStage().getStageId(), path, traverse);
}

std::vector<std::string> getAllMatchingChildPrims(const std::string& path,
                                                  std::function<bool(const std::string&)> predicate,
                                                  bool includeSelf,
                                                  std::optional<std::size_t> maxDepth)
{
    int64_t stageId = getActiveStage().getStageId();
    std::vector<std::string> result;
    std::deque<std::pair<std::string, std::size_t>> queue;

    if (includeSelf)
    {
        queue.push_back({ path, 0 });
    }
    else
    {
        for (const auto& child : openusd::getChildren(stageId, path))
        {
            queue.push_back({ child, 1 });
        }
    }

    while (!queue.empty())
    {
        auto [currentPath, currentDepth] = queue.front();
        queue.pop_front();

        if (maxDepth.has_value() && currentDepth > maxDepth.value())
        {
            break; // all remaining items are at this depth or deeper
        }

        if (predicate(currentPath))
        {
            result.push_back(currentPath);
        }

        for (const auto& child : openusd::getChildren(stageId, currentPath))
        {
            queue.push_back({ child, currentDepth + 1 });
        }
    }

    return result;
}

std::optional<std::string> getFirstMatchingChildPrim(const std::string& path,
                                                     std::function<bool(const std::string&)> predicate,
                                                     bool includeSelf,
                                                     std::optional<std::size_t> maxDepth)
{
    int64_t stageId = getActiveStage().getStageId();
    std::deque<std::pair<std::string, std::size_t>> queue;

    if (includeSelf)
    {
        queue.push_back({ path, 0 });
    }
    else
    {
        for (const auto& child : openusd::getChildren(stageId, path))
        {
            queue.push_back({ child, 1 });
        }
    }

    while (!queue.empty())
    {
        auto [currentPath, currentDepth] = queue.front();
        queue.pop_front();

        if (maxDepth.has_value() && currentDepth > maxDepth.value())
        {
            break; // all remaining items are at this depth or deeper
        }

        if (predicate(currentPath))
        {
            return currentPath;
        }

        for (const auto& child : openusd::getChildren(stageId, currentPath))
        {
            queue.push_back({ child, currentDepth + 1 });
        }
    }

    return std::nullopt;
}

std::optional<std::string> getFirstMatchingParentPrim(const std::string& path,
                                                      std::function<bool(const std::string&)> predicate,
                                                      bool includeSelf)
{
    int64_t stageId = getActiveStage().getStageId();
    std::string currentPath = includeSelf ? path : openusd::getParent(stageId, path);

    while (!currentPath.empty() && currentPath != "/")
    {
        if (predicate(currentPath))
        {
            return currentPath;
        }
        currentPath = openusd::getParent(stageId, currentPath);
    }
    return std::nullopt;
}

} // namespace utils
} // namespace foundation
} // namespace isaacsim
