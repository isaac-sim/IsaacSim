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

#include <cstddef>
#include <cstdint>

namespace isaacsim
{
namespace physics
{
namespace registration
{

/**
 * @brief Three-component vector of single-precision floating-point values.
 */
struct Float3
{
    /** @brief The x component. */
    float x{ 0.0f };

    /** @brief The y component. */
    float y{ 0.0f };

    /** @brief The z component. */
    float z{ 0.0f };
};

/**
 * @brief Four-component vector of single-precision floating-point values.
 */
struct Float4
{
    /** @brief The x component. */
    float x;

    /** @brief The y component. */
    float y;

    /** @brief The z component. */
    float z;

    /** @brief The w component. */
    float w;
};

/**
 * @brief Three-component vector of double-precision floating-point values.
 */
struct Double3
{
    /** @brief The x component. */
    double x;

    /** @brief The y component. */
    double y;

    /** @brief The z component. */
    double z;
};

/**
 * @brief Four-component vector of double-precision floating-point values.
 */
struct Double4
{
    /** @brief The x component. */
    double x;

    /** @brief The y component. */
    double y;

    /** @brief The z component. */
    double z;

    /** @brief The w component. */
    double w;
};

/**
 * @brief Strong type for a USD path encoded as an unsigned 64-bit value.
 *
 * Physics APIs use path tokens when a USD prim path identifier crosses the ABI boundary, such as in contact events,
 * scene queries, and step contexts.
 */
class PathToken
{
public:
    /**
     * @brief Constructs an empty path token.
     */
    PathToken() : path(0)
    {
    }

    /**
     * @brief Constructs a path token from an encoded path value.
     *
     * @param[in] value The encoded USD path value.
     */
    PathToken(uint64_t value) : path(value)
    {
    }

    /**
     * @brief Returns a hash value for this path token.
     *
     * @return The encoded path value converted to @c size_t.
     */
    size_t computeHash() const
    {
        return path;
    }

    /**
     * @brief Compares two path tokens for equality.
     *
     * @param[in] other The path token to compare with.
     * @return @c true if both tokens contain the same encoded path; otherwise, @c false.
     */
    bool operator==(const PathToken& other) const
    {
        return (path == other.path);
    }

    /**
     * @brief Compares two path tokens for inequality.
     *
     * @param[in] other The path token to compare with.
     * @return @c true if the tokens contain different encoded paths; otherwise, @c false.
     */
    bool operator!=(const PathToken& other) const
    {
        return (path != other.path);
    }

    /** @brief The encoded USD path value. A value of zero represents an empty path. */
    uint64_t path;
};

/**
 * @brief Hash function object for @ref PathToken.
 */
class PathTokenHash
{
public:
    /**
     * @brief Computes a hash value for a path token.
     *
     * @param[in] pathToken The path token to hash.
     * @return The path token's hash value.
     */
    size_t operator()(const PathToken& pathToken) const
    {
        return pathToken.computeHash();
    }
};

} // namespace registration
} // namespace physics
} // namespace isaacsim
