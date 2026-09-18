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

#pragma once

#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/foundation/objects/Export.h>
#include <isaacsim/foundation/objects/Stage.hpp>

#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

namespace array = isaacsim::common::array;

using InputValueType =
    std::variant<std::string, std::vector<std::string>, std::vector<std::vector<std::string>>, array::Array>;
using OutputValueType = std::variant<std::vector<std::string>, std::vector<std::vector<std::string>>, array::Array>;

/**
 * @class Prim
 * @brief High-level wrapper over one or more USD prims for batch attribute and schema operations.
 * @details
 * Wraps a list of USD prim paths and exposes a unified interface for querying and modifying prim
 * metadata, schema APIs, variant selections, and arbitrary attributes. All methods that accept an
 * @p indices parameter operate only on the prims at those positions; when @p indices is omitted,
 * all wrapped prims are processed.
 *
 * Paths may contain regular expressions; the constructor resolves them against the active stage
 * unless @p resolvePaths is @c false.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Prim
{
public:
    /**
     * @brief Construct a Prim wrapper for one or more USD prim paths.
     * @param[in] paths  Single path string or list of path strings. May include regular
     *                   expressions that are expanded against the active stage.
     * @param[in] resolvePaths Whether to resolve and expand the given paths (@c true)
     *                         or use them as-is (@c false).
     * @throws std::runtime_error if no active or default stage has been set.
     */
    Prim(const std::variant<std::string, std::vector<std::string>>& paths, bool resolvePaths = true);
    ~Prim() = default;

    /**
     * @brief Return the number of wrapped prims.
     * @return Count of prims managed by this wrapper.
     */
    size_t size() const;

    /**
     * @brief Return the resolved stage paths of all wrapped prims.
     * @return Ordered list of absolute USD stage paths.
     */
    const std::vector<std::string>& paths() const;

    /**
     * @brief Return the Stage associated with the wrapped prims.
     * @return Reference to the Stage wrapper.
     */
    const Stage& getStage() const;

    /**
     * @brief Resolve a path expression against the active stage.
     * @param[in] paths             Single path or list of paths, may include regular expressions.
     * @param[in] raiseOnMixedPaths If @c true, throws when existing and non-existing paths are mixed.
     * @return A pair of (existing paths, non-existing paths).
     */
    static std::tuple<std::vector<std::string>, std::vector<std::string>> resolvePaths(
        const std::variant<std::string, std::vector<std::string>>& paths, bool raiseOnMixedPaths = true);

    /**
     * @brief Get the prim names (leaf tokens of each path) for the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of name tokens, one per selected prim.
     */
    std::vector<std::string> getName(const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Get the USD schema type names of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of type name strings, one per selected prim.
     */
    std::vector<std::string> getTypeName(const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Get the parent prim paths of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of parent path strings, one per selected prim.
     */
    std::vector<std::string> getParent(const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Get the direct child prim paths of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of child path lists, one list per selected prim.
     */
    std::vector<std::vector<std::string>> getChildren(const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Get the variant sets and their available variants for the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of maps from variant-set name to list of available variant names,
     *         one map per selected prim.
     */
    std::vector<std::unordered_map<std::string, std::vector<std::string>>> getVariantSets(
        const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Get the active variant selection for each selected prim.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of maps from variant-set name to selected variant name,
     *         one map per selected prim.
     */
    std::vector<std::unordered_map<std::string, std::string>> getVariantSelection(
        const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Set the active variant selection on the selected prims.
     * @param[in] variants Map of variant-set name to variant name to activate.
     * @param[in] indices  Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setVariantSelection(const std::unordered_map<std::string, std::string>& variants,
                             const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Check whether each selected prim is typed as the given USD schema type.
     * @param[in] schemaType USD schema type name to test (e.g. @c "UsdGeomMesh").
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per selected prim.
     */
    array::Array isA(const std::string& schemaType, const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Check whether each selected prim has the given USD API schema applied.
     * @param[in] schemaType   USD API schema type name.
     * @param[in] instanceName For multi-apply schemas, the instance name to check. Single-apply schemas leave this
     * empty.
     * @param[in] indices      Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per selected prim.
     */
    array::Array hasApi(const std::string& schemaType,
                        const std::optional<std::string>& instanceName = std::nullopt,
                        const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Apply a USD API schema to each selected prim.
     * @param[in] schemaType   USD API schema type name.
     * @param[in] instanceName For multi-apply schemas, the instance name to use.
     * @param[in] indices      Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating success (dtype bool, shape @c (N,1)), one per selected prim.
     */
    array::Array applyApi(const std::string& schemaType,
                          const std::optional<std::string>& instanceName = std::nullopt,
                          const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Remove a USD API schema from each selected prim.
     * @param[in] schemaType   USD API schema type name.
     * @param[in] instanceName For multi-apply schemas, the instance name to remove.
     * @param[in] indices      Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating success (dtype bool, shape @c (N,1)), one per selected prim.
     */
    array::Array removeApi(const std::string& schemaType,
                           const std::optional<std::string>& instanceName = std::nullopt,
                           const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Get the names of all applied API schemas for each selected prim.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Ordered list of schema name lists, one list per selected prim.
     */
    std::vector<std::vector<std::string>> getAppliedSchemas(const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Create a custom attribute on each selected prim.
     * @param[in] attributeName Name of the attribute to create.
     * @param[in] typeName      USD value type name (e.g. @c "float", @c "double3").
     * @param[in] indices       Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating whether the attribute was created (dtype bool, shape @c (N,1)), one per selected
     * prim.
     */
    array::Array createAttribute(const std::string& attributeName,
                                 const std::string& typeName,
                                 const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Remove a custom attribute from each selected prim.
     * @param[in] attributeName Name of the attribute to remove.
     * @param[in] indices       Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags indicating whether the attribute was removed (dtype bool, shape @c (N,1)), one per selected
     * prim.
     */
    array::Array removeAttribute(const std::string& attributeName,
                                 const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Get the values of a named attribute for each selected prim.
     * @param[in] attributeName Name of the attribute to query.
     * @param[in] indices       Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Attribute values as an @c OutputValueType variant (string list, nested string list, or numeric array).
     */
    OutputValueType getAttributeValues(const std::string& attributeName,
                                       const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Set the values of a named attribute on each selected prim.
     * @param[in] attributeName Name of the attribute to set.
     * @param[in] values        New attribute values as an @c InputValueType variant.
     * @param[in] indices       Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setAttributeValues(const std::string& attributeName,
                            const InputValueType& values,
                            const std::optional<array::Array>& indices = std::nullopt) const;

protected:
    Prim();

    /** @brief Converts optional selection indices into signed index values. */
    std::vector<int64_t> _resolveIndexValues(const std::optional<array::Array>& indices) const;
    /** @brief Returns the number of selected prims. */
    int64_t _resolveIndexedSize(const std::optional<array::Array>& indices) const;
    /** @brief Returns paths for the selected prims. */
    std::vector<std::string> _resolveIndexedPaths(const std::optional<array::Array>& indices) const;
    /** @brief Expands a scalar or list value to match the selected prim count. */
    std::vector<std::string> _resolveStringList(const std::variant<std::string, std::vector<std::string>>& values,
                                                const std::optional<array::Array>& indices) const;

    /** @brief Resolved USD paths wrapped by this object. */
    std::vector<std::string> m_paths;

private:
    struct PrimOps
    {
        std::vector<std::string> (*findMatchingPrimPaths)(int64_t, const std::string&, bool);
        bool (*isValidPathString)(const std::string&);
        std::string (*getName)(int64_t, const std::string&);
        std::string (*getTypeName)(int64_t, const std::string&);
        std::string (*getParent)(int64_t, const std::string&);
        std::vector<std::string> (*getChildren)(int64_t, const std::string&);
        bool (*isA)(int64_t, const std::string&, const std::string&);
        bool (*hasApi)(int64_t, const std::string&, const std::string&, const std::optional<std::string>&);
        bool (*applyApi)(int64_t, const std::string&, const std::string&, const std::optional<std::string>&);
        bool (*removeApi)(int64_t, const std::string&, const std::string&, const std::optional<std::string>&);
        std::vector<std::string> (*getAppliedSchemas)(int64_t, const std::string&);
        std::unordered_map<std::string, std::vector<std::string>> (*getVariants)(int64_t, const std::string&, bool);
        void (*setVariants)(int64_t, const std::string&, const std::unordered_map<std::string, std::vector<std::string>>&);
        bool (*createPrimAttribute)(int64_t, const std::string&, const std::string&, const std::string&);
        bool (*removePrimAttribute)(int64_t, const std::string&, const std::string&);
        std::string (*getPrimAttributeTypeName)(int64_t, const std::string&, const std::string&);
        OutputValueType (*getPrimAttributeValues)(int64_t, const std::vector<std::string>&, const std::string&);
        std::vector<bool> (*setPrimAttributeValues)(int64_t,
                                                    const std::vector<std::string>&,
                                                    const std::string&,
                                                    const OutputValueType&);
    };

    static PrimOps _populatePrimOps(const std::string& backend);

    Stage m_stage;
    PrimOps m_primOps;
};

} // namespace objects
} // namespace foundation
} // namespace isaacsim
