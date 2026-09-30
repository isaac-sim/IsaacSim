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
#include <isaacsim/foundation/utils/Semantics.hpp>
#include <isaacsim/foundation/utils/Stage.hpp>

#include <algorithm>
#include <functional>
#include <string_view>
#include <unordered_set>

namespace isaacsim
{
namespace foundation
{
namespace utils
{

namespace openusd = isaacsim::foundation::usd::openusd;

namespace
{

constexpr std::string_view g_kLabelsApiPrefix = "SemanticsLabelsAPI:";

std::vector<std::string> toVectorString(const std::variant<std::string, std::vector<std::string>>& labels)
{
    return std::holds_alternative<std::string>(labels) ? std::vector<std::string>{ std::get<std::string>(labels) } :
                                                         std::get<std::vector<std::string>>(labels);
}

std::vector<std::string> getLabelsAttribute(int64_t stageId, const std::string& path, const std::string& taxonomy)
{
    auto value = openusd::getPrimAttributeValues(stageId, { path }, "semantics:labels:" + taxonomy);
    if (std::holds_alternative<std::vector<std::vector<std::string>>>(value))
    {
        return std::get<std::vector<std::vector<std::string>>>(value).at(0);
    }
    return {};
}

void setLabelsAttribute(int64_t stageId,
                        const std::string& path,
                        const std::string& taxonomy,
                        const std::vector<std::string>& labels)
{
    openusd::setPrimAttributeValues(
        stageId, { path }, "semantics:labels:" + taxonomy, std::vector<std::vector<std::string>>{ labels });
}

void visitPrims(int64_t stageId,
                const std::string& path,
                bool includeDescendants,
                const std::function<void(const std::string&)>& visitor)
{
    if (includeDescendants)
    {
        openusd::traversePrim(stageId, path,
                              [&](const std::string& _path)
                              {
                                  visitor(_path);
                                  return true;
                              });
    }
    else
    {
        visitor(path);
    }
}

} // namespace

void addLabels(const std::string& path,
               const std::variant<std::string, std::vector<std::string>>& labels,
               const std::string& taxonomy)
{
    const int64_t stageId = getActiveStage().getStageId();
    openusd::applyApi(stageId, path, "SemanticsLabelsAPI", taxonomy);
    auto existingLabels = getLabelsAttribute(stageId, path, taxonomy);
    for (const auto& label : toVectorString(labels))
    {
        if (std::find(existingLabels.begin(), existingLabels.end(), label) == existingLabels.end())
        {
            existingLabels.push_back(label);
        }
    }
    setLabelsAttribute(stageId, path, taxonomy, existingLabels);
}

std::unordered_map<std::string, std::vector<std::string>> getLabels(const std::string& path, bool includeDescendants)
{
    std::unordered_map<std::string, std::vector<std::string>> result;

    const int64_t stageId = getActiveStage().getStageId();
    visitPrims(stageId, path, includeDescendants,
               [&](const std::string& _path)
               {
                   for (const auto& schema : openusd::getAppliedSchemas(stageId, _path))
                   {
                       if (!schema.rfind(g_kLabelsApiPrefix, 0))
                       {
                           const std::string taxonomy = schema.substr(g_kLabelsApiPrefix.size());
                           auto& destination = result[taxonomy];
                           auto currentLabels = getLabelsAttribute(stageId, _path, taxonomy);
                           destination.insert(destination.end(), currentLabels.begin(), currentLabels.end());
                       }
                   }
               });

    return result;
}

void removeLabels(const std::string& path,
                  const std::variant<std::string, std::vector<std::string>>& labels,
                  const std::optional<std::string>& taxonomy,
                  bool includeDescendants)
{
    const auto _labels = toVectorString(labels);
    const std::unordered_set<std::string> labelsToRemove(_labels.begin(), _labels.end());

    const int64_t stageId = getActiveStage().getStageId();
    visitPrims(stageId, path, includeDescendants,
               [&](const std::string& primPath)
               {
                   for (const auto& schema : openusd::getAppliedSchemas(stageId, primPath))
                   {
                       if (!schema.rfind(g_kLabelsApiPrefix, 0))
                       {
                           const std::string currentTaxonomy = schema.substr(g_kLabelsApiPrefix.size());
                           if (!taxonomy.has_value() || currentTaxonomy == *taxonomy)
                           {
                               auto currentLabels = getLabelsAttribute(stageId, primPath, currentTaxonomy);
                               currentLabels.erase(std::remove_if(currentLabels.begin(), currentLabels.end(),
                                                                  [&](const std::string& label)
                                                                  { return labelsToRemove.count(label) > 0; }),
                                                   currentLabels.end());
                               setLabelsAttribute(stageId, primPath, currentTaxonomy, currentLabels);
                           }
                       }
                   }
               });
}

void removeAllLabels(const std::string& path, bool removeTaxonomies, bool includeDescendants)
{
    const int64_t stageId = getActiveStage().getStageId();
    visitPrims(stageId, path, includeDescendants,
               [&](const std::string& primPath)
               {
                   // Snapshot schemas before any modification to avoid iterator invalidation during schema removal.
                   const auto schemas = openusd::getAppliedSchemas(stageId, primPath);
                   for (const auto& schema : schemas)
                   {
                       if (!schema.rfind(g_kLabelsApiPrefix, 0))
                       {
                           const std::string currentTaxonomy = schema.substr(g_kLabelsApiPrefix.size());
                           if (removeTaxonomies)
                           {
                               openusd::removeApi(stageId, primPath, "SemanticsLabelsAPI", currentTaxonomy);
                           }
                           else
                           {
                               setLabelsAttribute(stageId, primPath, currentTaxonomy, {});
                           }
                       }
                   }
               });
}

} // namespace utils
} // namespace foundation
} // namespace isaacsim
