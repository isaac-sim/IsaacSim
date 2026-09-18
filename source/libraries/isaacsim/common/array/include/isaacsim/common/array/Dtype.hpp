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

#include "isaacsim/common/array/Export.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace isaacsim
{
namespace common
{
namespace array
{

/**
 * @class Dtype
 * @brief Represents the element data type of an array.
 * @details
 * A Dtype identifies the scalar kind (boolean, signed/unsigned integer, or
 * floating-point) together with its storage width. Instances are value types
 * and can be compared for equality.
 *
 * Instances can be constructed from a @ref Kind enumerator, a string name
 * (e.g. `"float32"`, `"int8"`), or via the named factory methods such as
 * @ref Float32() and @ref Int32(). The template factory @ref fromType() maps a
 * C++ type directly to its corresponding Dtype at compile time.
 */
class ISAACSIM_COMMON_ARRAY_API Dtype
{
public:
    /**
     * @brief Enumerates all scalar kinds supported as array element types.
     */
    enum class Kind
    {
        /**
         * @brief Boolean (1 byte).
         */
        eBool,

        /**
         * @brief Signed 8-bit integer.
         */
        eInt8,

        /**
         * @brief Signed 16-bit integer.
         */
        eInt16,

        /**
         * @brief Signed 32-bit integer.
         */
        eInt32,

        /**
         * @brief Signed 64-bit integer.
         */
        eInt64,

        /**
         * @brief Unsigned 8-bit integer.
         */
        eUInt8,

        /**
         * @brief Unsigned 16-bit integer.
         */
        eUInt16,

        /**
         * @brief Unsigned 32-bit integer.
         */
        eUInt32,

        /**
         * @brief Unsigned 64-bit integer.
         */
        eUInt64,

        /**
         * @brief IEEE 754 single-precision (32-bit) floating-point.
         */
        eFloat32,

        /**
         * @brief IEEE 754 double-precision (64-bit) floating-point.
         */
        eFloat64,
    };

    /**
     * @brief Constructs a Dtype from the given kind enumerator.
     * @param[in] kind The scalar kind to represent.
     */
    constexpr Dtype(Kind kind) : m_kind(kind)
    {
        // fromType() needs Dtype to be a literal type, which MSVC enforces strictly.
        // Therefore, it has to be defined in .hpp file.
    }

    /**
     * @brief Constructs a Dtype from a string name.
     * @details
     * Accepted names match the string produced by @ref toString(), e.g.
     * `"bool"`, `"int8"`, `"float32"`.
     *
     * @param[in] name String name of the data type.
     * @throws std::invalid_argument if @p name is not a recognized type name.
     */
    Dtype(const std::string& name);

    Dtype() = delete;
    /** @brief Copy-constructs a data type descriptor. */
    Dtype(const Dtype& other) = default;
    /** @brief Move-constructs a data type descriptor. */
    Dtype(Dtype&& other) = default;
    ~Dtype() = default;

    /** @brief Copy-assigns a data type descriptor. */
    Dtype& operator=(const Dtype& other) = default;
    /** @brief Move-assigns a data type descriptor. */
    Dtype& operator=(Dtype&& other) = default;

    /**
     * @brief Checks whether two Dtypes represent the same scalar kind.
     * @param[in] other The Dtype to compare against.
     * @return `true` if both Dtypes have equal kinds.
     */
    bool operator==(const Dtype& other) const;

    /**
     * @brief Checks whether two Dtypes represent different scalar kinds.
     * @param[in] other The Dtype to compare against.
     * @return `true` if the Dtypes have different kinds.
     */
    bool operator!=(const Dtype& other) const;

    /**
     * @brief Returns the scalar kind enumerator for this Dtype.
     * @return The @ref Kind value identifying the element type.
     */
    Kind kind() const;

    /**
     * @brief Returns the storage size of a single element.
     * @return Size in bytes.
     */
    size_t size() const;

    /**
     * @brief Returns whether this Dtype is a floating-point type.
     * @return `true` for @ref Kind::eFloat32 and @ref Kind::eFloat64.
     */
    bool isFloating() const;

    /**
     * @brief Returns whether this Dtype is an integral type.
     * @return `true` for all integer kinds.
     */
    bool isIntegral() const;

    /**
     * @brief Returns whether this Dtype is a signed type.
     * @return `true` for signed integers and floating-point types.
     */
    bool isSigned() const;

    /**
     * @brief Returns whether this Dtype is an unsigned type.
     * @return `true` for unsigned integer kinds.
     */
    bool isUnsigned() const;

    /**
     * @brief Returns the canonical string name for this Dtype.
     * @return A lowercase string such as `"bool"`, `"int8"`, or `"float32"`.
     */
    std::string toString() const;

    /**
     * @brief Constructs a Dtype from a string name.
     * @details
     * Equivalent to the string constructor. Accepted names match the output of
     * @ref toString().
     *
     * @param[in] name String name of the data type.
     * @return The corresponding Dtype.
     * @throws std::invalid_argument if @p name is not a recognized type name.
     */
    static Dtype fromString(const std::string& name);

    /**
     * @brief Constructs the Dtype corresponding to the C++ type @p T at compile time.
     * @tparam T A C++ scalar type that maps to a supported @ref Kind.
     * @return The Dtype for @p T.
     */
    template <typename T>
    static constexpr Dtype fromType()
    {
        if constexpr (std::is_same_v<T, bool>)
            return Dtype(Kind::eBool);
        else if constexpr (std::is_same_v<T, int8_t>)
            return Dtype(Kind::eInt8);
        else if constexpr (std::is_same_v<T, int16_t>)
            return Dtype(Kind::eInt16);
        else if constexpr (std::is_same_v<T, int32_t>)
            return Dtype(Kind::eInt32);
        else if constexpr (std::is_same_v<T, int64_t>)
            return Dtype(Kind::eInt64);
        else if constexpr (std::is_same_v<T, uint8_t>)
            return Dtype(Kind::eUInt8);
        else if constexpr (std::is_same_v<T, uint16_t>)
            return Dtype(Kind::eUInt16);
        else if constexpr (std::is_same_v<T, uint32_t>)
            return Dtype(Kind::eUInt32);
        else if constexpr (std::is_same_v<T, uint64_t>)
            return Dtype(Kind::eUInt64);
        else if constexpr (std::is_same_v<T, float>)
            return Dtype(Kind::eFloat32);
        else if constexpr (std::is_same_v<T, double>)
            return Dtype(Kind::eFloat64);
        else
            static_assert(sizeof(T) == 0, "Dtype::fromType(): unsupported scalar type");
    }

    /**
     * @brief Returns the common type two Dtypes can both be safely cast to, following NumPy rules.
     * @details
     * Implements NumPy's array-to-array type promotion (`numpy.promote_types`): boolean yields to
     * the other operand; two types of the same kind promote to the wider one; a signed and an
     * unsigned integer promote to the smallest signed type holding both ranges, or to `float64`
     * when no signed type is wide enough (as with `int64` and `uint64`); and an integer and a
     * floating-point type promote to the smallest float representing every integer value exactly,
     * so `float32` is ruled out for integers wider than its 24-bit mantissa.
     *
     * @param[in] a First data type.
     * @param[in] b Second data type.
     * @return The promoted Dtype.
     */
    static Dtype promoteDtypes(Dtype a, Dtype b);

    /**
     * @brief Maps a runtime @ref Kind to its C++ scalar type and invokes @p function for it.
     * @details
     * Calls `function(T{})` with a value-initialized sample of the scalar type corresponding to
     * @p kind, so that a generic lambda can recover the type with `decltype` and instantiate
     * type-specific code:
     *
     * @code
     * Dtype::dispatchByKind(dtype.kind(), [&](auto sample) { using T = decltype(sample); ... });
     * @endcode
     *
     * Whatever @p function returns is returned unchanged, so the same dispatcher serves callers
     * returning `void`, `cudaError_t`, or any other type.
     *
     * @tparam Function Callable accepting a value-initialized sample of every supported scalar type.
     * @param[in] kind Element kind to dispatch on.
     * @param[in] function Callable to invoke.
     * @return The value returned by @p function.
     * @throws std::logic_error if @p kind is not a valid enumerator, which can only result from an
     *         out-of-range cast.
     */
    template <typename Function>
    static decltype(auto) dispatchByKind(Kind kind, Function&& function)
    {
        switch (kind)
        {
        case Kind::eBool:
            return function(bool{});
        case Kind::eInt8:
            return function(int8_t{});
        case Kind::eInt16:
            return function(int16_t{});
        case Kind::eInt32:
            return function(int32_t{});
        case Kind::eInt64:
            return function(int64_t{});
        case Kind::eUInt8:
            return function(uint8_t{});
        case Kind::eUInt16:
            return function(uint16_t{});
        case Kind::eUInt32:
            return function(uint32_t{});
        case Kind::eUInt64:
            return function(uint64_t{});
        case Kind::eFloat32:
            return function(float{});
        case Kind::eFloat64:
            return function(double{});
        }
        throw std::logic_error("Dtype::dispatchByKind(): unknown data type kind");
    }

    /**
     * @brief Returns the Dtype for boolean values.
     * @return A Dtype with kind @ref Kind::eBool.
     */
    static Dtype Bool();

    /**
     * @brief Returns the Dtype for signed 8-bit integers.
     * @return A Dtype with kind @ref Kind::eInt8.
     */
    static Dtype Int8();

    /**
     * @brief Returns the Dtype for signed 16-bit integers.
     * @return A Dtype with kind @ref Kind::eInt16.
     */
    static Dtype Int16();

    /**
     * @brief Returns the Dtype for signed 32-bit integers.
     * @return A Dtype with kind @ref Kind::eInt32.
     */
    static Dtype Int32();

    /**
     * @brief Returns the Dtype for signed 64-bit integers.
     * @return A Dtype with kind @ref Kind::eInt64.
     */
    static Dtype Int64();

    /**
     * @brief Returns the Dtype for unsigned 8-bit integers.
     * @return A Dtype with kind @ref Kind::eUInt8.
     */
    static Dtype UInt8();

    /**
     * @brief Returns the Dtype for unsigned 16-bit integers.
     * @return A Dtype with kind @ref Kind::eUInt16.
     */
    static Dtype UInt16();

    /**
     * @brief Returns the Dtype for unsigned 32-bit integers.
     * @return A Dtype with kind @ref Kind::eUInt32.
     */
    static Dtype UInt32();

    /**
     * @brief Returns the Dtype for unsigned 64-bit integers.
     * @return A Dtype with kind @ref Kind::eUInt64.
     */
    static Dtype UInt64();

    /**
     * @brief Returns the Dtype for single-precision floating-point values.
     * @return A Dtype with kind @ref Kind::eFloat32.
     */
    static Dtype Float32();

    /**
     * @brief Returns the Dtype for double-precision floating-point values.
     * @return A Dtype with kind @ref Kind::eFloat64.
     */
    static Dtype Float64();

private:
    /**
     * @brief The scalar kind this Dtype represents.
     */
    Kind m_kind;
};

} // namespace array
} // namespace common
} // namespace isaacsim
