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

#include <isaacsim/foundation/utils/Export.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace utils
{

/**
 * @brief Returns all prim paths on the default stage that match the given path expression.
 *
 * @param[in] path Literal USD path or regex pattern to match against prims on the default stage.
 * @param[in] traverse For a literal path: when @c true, also returns all descendants of the matched
 *            prim; when @c false, returns only that prim. For a pattern: when @c false, each
 *            @c '/'-separated segment is matched as a regex level by level; when @c true, the whole
 *            pattern is tested as a single regex against every prim path on the stage.
 *
 * @return Ordered list of matching prim path strings.
 */
ISAACSIM_FOUNDATION_UTILS_API std::vector<std::string> findMatchingPrimPaths(const std::string& path,
                                                                             bool traverse = false);

/**
 * @brief Returns all prims in the subtree rooted at @p path for which @p predicate returns @c true.
 * @details
 * Performs a breadth-first traversal of the default stage starting at @p path. Each visited prim
 * is tested against @p predicate; those that pass are collected into the result.
 *
 * When @p includeSelf is @c false the root prim itself is skipped and traversal begins with its
 * direct children (depth 1). When @p includeSelf is @c true the root is enqueued at depth 0.
 *
 * When @p maxDepth is set, traversal stops as soon as the current prim's depth exceeds
 * @p maxDepth. Because the deque is ordered by depth in BFS order, all remaining items at that
 * point are at the same or greater depth, so the entire search halts immediately.
 *
 * @param[in] path USD path of the subtree root on the default stage.
 * @param[in] predicate Callable invoked with the string path of each visited prim; return
 *            @c true to include the prim in the result.
 * @param[in] includeSelf Whether to test and potentially include @p path itself.
 * @param[in] maxDepth Maximum traversal depth relative to @p path, where @p path itself is at
 *            depth 0 and its direct children at depth 1. @c std::nullopt means unlimited depth.
 *            Note that depth is always counted from @p path regardless of @p includeSelf, so
 *            @c maxDepth=0 combined with @c includeSelf=false visits no prim at all: the direct
 *            children the traversal starts from are already at depth 1. Use @c maxDepth=1 to
 *            visit the direct children.
 *
 * @return All prim paths in BFS order for which @p predicate returned @c true.
 *
 * @see getFirstMatchingChildPrim()
 */
ISAACSIM_FOUNDATION_UTILS_API std::vector<std::string> getAllMatchingChildPrims(
    const std::string& path,
    std::function<bool(const std::string&)> predicate,
    bool includeSelf = false,
    std::optional<std::size_t> maxDepth = std::nullopt);

/**
 * @brief Returns the first prim in the subtree rooted at @p path for which @p predicate returns @c true.
 * @details
 * Performs a breadth-first traversal of the default stage. Prims are visited in BFS order; the
 * first one for which @p predicate returns @c true is returned immediately and traversal stops.
 *
 * When @p includeSelf is @c false the root prim is skipped and traversal begins with its direct
 * children. When @p includeSelf is @c true the root is the first candidate tested.
 *
 * When @p maxDepth is set, traversal stops as soon as the current prim's depth exceeds
 * @p maxDepth. Because the deque is ordered by depth in BFS order, all remaining items at that
 * point are at the same or greater depth, so the entire search halts immediately.
 *
 * @param[in] path USD path of the subtree root on the default stage.
 * @param[in] predicate Callable invoked with the string path of each visited prim; return
 *            @c true to select that prim.
 * @param[in] includeSelf Whether to test @p path itself before its descendants.
 * @param[in] maxDepth Maximum traversal depth relative to @p path, where @p path itself is at
 *            depth 0 and its direct children at depth 1. @c std::nullopt means unlimited depth.
 *            Note that depth is always counted from @p path regardless of @p includeSelf, so
 *            @c maxDepth=0 combined with @c includeSelf=false visits no prim at all: the direct
 *            children the traversal starts from are already at depth 1. Use @c maxDepth=1 to
 *            visit the direct children.
 *
 * @return The path of the first matching prim, or @c std::nullopt if no match is found.
 *
 * @see getAllMatchingChildPrims()
 */
ISAACSIM_FOUNDATION_UTILS_API std::optional<std::string> getFirstMatchingChildPrim(
    const std::string& path,
    std::function<bool(const std::string&)> predicate,
    bool includeSelf = false,
    std::optional<std::size_t> maxDepth = std::nullopt);

/**
 * @brief Returns the first ancestor of @p path for which @p predicate returns @c true.
 * @details
 * Walks up the parent chain on the default stage, testing each prim against @p predicate,
 * and returns the first match. Traversal stops before reaching the stage pseudoroot (@c "/");
 * the pseudoroot is never tested or returned.
 *
 * When @p includeSelf is @c false the walk starts from the direct parent of @p path. When
 * @p includeSelf is @c true the walk starts from @p path itself.
 *
 * @param[in] path USD path whose ancestors are searched on the default stage.
 * @param[in] predicate Callable invoked with the string path of each ancestor; return
 *            @c true to select that ancestor.
 * @param[in] includeSelf Whether to test @p path itself before walking up to its parents.
 *
 * @return The path of the first matching ancestor, or @c std::nullopt if no match is found
 *         before reaching the pseudoroot.
 *
 * @see getFirstMatchingChildPrim()
 */
ISAACSIM_FOUNDATION_UTILS_API std::optional<std::string> getFirstMatchingParentPrim(
    const std::string& path, std::function<bool(const std::string&)> predicate, bool includeSelf = false);

} // namespace utils
} // namespace foundation
} // namespace isaacsim
