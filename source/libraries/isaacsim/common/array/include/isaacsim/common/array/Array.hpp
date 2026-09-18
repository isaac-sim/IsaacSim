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

#include "isaacsim/common/array/Device.hpp"
#include "isaacsim/common/array/Dtype.hpp"
#include "isaacsim/common/array/Export.h"
#include "isaacsim/common/array/Shape.hpp"

#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace details
{

/**
 * @brief CPU backing storage holding a reference-counted byte buffer.
 */
struct CpuStorage
{
    /**
     * @brief Shared ownership of the raw byte allocation.
     */
    std::shared_ptr<std::byte[]> data;
};

/**
 * @brief CUDA device backing storage holding a reference-counted device pointer.
 */
struct CudaStorage
{
    /**
     * @brief Shared ownership of the CUDA device allocation.
     */
    std::shared_ptr<void> data;
};

#define ISAACSIM_COMMON_ARRAY_DIM0(T) T
#define ISAACSIM_COMMON_ARRAY_DIM1(T) std::vector<T>
#define ISAACSIM_COMMON_ARRAY_DIM2(T) std::vector<std::vector<T>>
#define ISAACSIM_COMMON_ARRAY_SCALARS(D)                                                                               \
    D(bool), D(int8_t), D(int16_t), D(int32_t), D(int64_t), D(uint8_t), D(uint16_t), D(uint32_t), D(uint64_t),         \
        D(float), D(double)

/**
 * @brief Variant type covering all scalar, 1-D, and 2-D inputs accepted by the Array constructors and setters.
 * @details
 * Scalar alternatives produce a 0-D array. `std::vector<T>` alternatives produce a 1-D array.
 * `std::vector<std::vector<T>>` alternatives produce a 2-D array. Supported element types are
 * `bool`, the fixed-width integer types from `<cstdint>`, `float`, and `double`.
 */
using SupportedInputSpecification = std::variant<ISAACSIM_COMMON_ARRAY_SCALARS(ISAACSIM_COMMON_ARRAY_DIM0),
                                                 ISAACSIM_COMMON_ARRAY_SCALARS(ISAACSIM_COMMON_ARRAY_DIM1),
                                                 ISAACSIM_COMMON_ARRAY_SCALARS(ISAACSIM_COMMON_ARRAY_DIM2)>;

#undef ISAACSIM_COMMON_ARRAY_SCALARS
#undef ISAACSIM_COMMON_ARRAY_DIM2
#undef ISAACSIM_COMMON_ARRAY_DIM1
#undef ISAACSIM_COMMON_ARRAY_DIM0

} // namespace details

/**
 * @class Array
 * @brief Multi-dimensional array with support for CPU and CUDA devices.
 * @details
 * An Array combines a contiguous element buffer with a @ref Shape, a @ref Dtype, and a
 * @ref Device. Multiple Array instances may share the same underlying buffer; operations
 * such as @ref reshape() and @ref at() return views over the original allocation without
 * copying data. Copy-construction and copy-assignment are likewise cheap and share the
 * buffer; use @ref copy() to allocate an independent buffer, or @ref clone() when you also
 * need to preserve the full buffer layout including any pre-element offset from a prior
 * @ref at() call.
 *
 * Arrays can be constructed from scalar values, `std::vector`, or `std::vector<std::vector>`
 * via @c SupportedInputSpecification, or from an existing buffer via @ref fromBuffer(). Device and
 * dtype conversion are available through @ref toDevice() and @ref toDtype().
 *
 * @note The element buffer is reference-counted; @ref reshape(), @ref at(), copy-construction,
 *       and copy-assignment all share it, while @ref copy() and @ref clone() each allocate a
 *       fresh buffer.
 */
class ISAACSIM_COMMON_ARRAY_API Array
{
public:
    /**
     * @brief Constructs an Array from a scalar, 1-D, or 2-D value.
     * @details
     * The element type is inferred from the active alternative of @p value unless
     * @p dtype is specified, in which case elements are cast to that type. The array
     * is allocated on @p device.
     *
     * @param[in] value  Scalar or nested vector providing the initial element data.
     * @param[in] dtype  Element type for the resulting array. Inferred from @p value if omitted.
     * @param[in] device Target compute device. Defaults to the CPU.
     */
    Array(const details::SupportedInputSpecification& value,
          const std::optional<Dtype>& dtype = std::nullopt,
          const Device& device = Device::Cpu());

    /**
     * @brief Copy-constructs a shallow view of @p other sharing its storage.
     * @details No data is copied: the new array has the same shape, dtype, device, and offset
     *          as @p other and shares ownership of the same reference-counted buffer, so writes
     *          through either array are visible through the other.
     * @param[in] other The Array to copy.
     * @see copy() for an independent, compacted deep copy.
     * @see clone() for a deep copy that preserves the full buffer layout including any pre-element offset.
     */
    Array(const Array& other) = default;
    /** @brief Move-constructs an Array and transfers ownership of its storage. */
    Array(Array&& other) = default;
    ~Array() = default;

    /**
     * @brief Copy-assigns this Array as a shallow view of @p other sharing its storage.
     * @details No data is copied: after assignment this array has the same shape, dtype, device,
     *          and offset as @p other and shares ownership of the same reference-counted buffer,
     *          so writes through either array are visible through the other.
     * @param[in] other The Array to copy-assign from.
     * @return Reference to this Array.
     * @see copy() for an independent, compacted deep copy.
     * @see clone() for a deep copy that preserves the full buffer layout including any pre-element offset.
     */
    Array& operator=(const Array& other) = default;
    /** @brief Move-assigns an Array and transfers ownership of its storage. */
    Array& operator=(Array&& other) = default;

    /**
     * @brief Returns the shape of this array.
     * @return A const reference to the @ref Shape describing the dimensions.
     */
    const Shape& shape() const;

    /**
     * @brief Returns the compute device on which this array resides.
     * @return The @ref Device of this array.
     */
    Device device() const;

    /**
     * @brief Returns the element data type of this array.
     * @return The @ref Dtype of this array.
     */
    Dtype dtype() const;

    /**
     * @brief Returns the number of dimensions (rank) of this array.
     * @return Number of axes.
     */
    size_t ndim() const;

    /**
     * @brief Returns the total number of elements in this array.
     * @return Product of all dimension sizes, or 1 for a 0-D array.
     */
    size_t size() const;

    /**
     * @brief Returns the total size of the element data in bytes.
     * @return `size() * dtype().size()` bytes.
     */
    size_t nbytes() const;

    /**
     * @brief Returns a mutable raw pointer to the first element of this array.
     * @details
     * The pointer is offset-adjusted and points directly to element 0 within the
     * underlying buffer, accounting for any slice offset from @ref at().
     *
     * For a CUDA array this is a device pointer into the device allocation: it is valid only for
     * CUDA APIs and kernels on @ref device(), and must not be dereferenced by host code.
     *
     * @return Mutable pointer to the element data.
     */
    void* data();

    /**
     * @brief Returns a read-only raw pointer to the first element of this array.
     * @details
     * The pointer is offset-adjusted and points directly to element 0 within the
     * underlying buffer, accounting for any slice offset from @ref at().
     *
     * For a CUDA array this is a device pointer into the device allocation: it is valid only for
     * CUDA APIs and kernels on @ref device(), and must not be dereferenced by host code.
     *
     * @return Const pointer to the element data.
     */
    const void* data() const;

    /**
     * @brief Returns shared ownership of the underlying byte buffer.
     * @details
     * The returned pointer refers to the start of the full allocation, which may
     * precede element 0 when this array is a slice obtained via @ref at(). Use
     * @ref data() to obtain a pointer directly to element 0.
     *
     * For a CUDA array this shares ownership of the device allocation and the stored pointer is a
     * device pointer: it is valid only for CUDA APIs and kernels on @ref device(), and must not be
     * dereferenced by host code.
     *
     * @return Shared pointer to the raw byte buffer.
     */
    std::shared_ptr<std::byte[]> buffer() const;

    /**
     * @brief Returns a view of this array with a different shape over the same buffer.
     * @details
     * Uses @ref Shape::resolve() to interpret the target shape, so @p shape may contain
     * at most one -1 dimension whose size is inferred from the total element count.
     * No data is copied; the returned array shares the underlying buffer.
     *
     * @param[in] shape Target shape, optionally containing one -1 dimension to infer.
     * @return A new Array with the resolved shape sharing this array's buffer.
     * @throws std::invalid_argument if @p shape is incompatible with the total element count.
     * @see Shape::resolve()
     */
    Array reshape(const Shape& shape) const;

    /**
     * @brief Returns a 1-D view of this array over the same buffer.
     * @details Shorthand for reshaping to a single dimension holding all @ref size() elements.
     *          No data is copied; the returned array shares the underlying buffer.
     * @return A new 1-D Array sharing this array's buffer.
     * @see reshape()
     */
    Array flatten() const;

    /**
     * @brief Returns an independent, compacted deep copy of this array.
     * @details Allocates a new buffer of exactly @ref nbytes() and copies all visible
     *          elements. The returned array has offset 0, so a slice obtained via @ref at()
     *          is materialized into a standalone, densely-packed array.
     * @return A new Array with identical metadata and an independent buffer.
     * @see clone() to preserve the full buffer layout including any pre-element offset.
     */
    Array copy() const;

    /**
     * @brief Returns an independent deep copy of this array with its own buffer.
     * @details
     * Duplicates the whole allocation, including any bytes preceding element 0, so the
     * offset of a slice obtained via @ref at() is preserved (unlike @ref copy(), which
     * compacts the visible elements to offset 0). The returned array does not share
     * storage with this array.
     *
     * @return A new Array with the same shape, dtype, device, offset, and copied element data.
     */
    Array clone() const;

    /**
     * @brief Constructs an Array that takes shared ownership of an existing byte buffer.
     * @details
     * No data is copied. The resulting array shares ownership of @p data with the caller.
     *
     * When @p device is a CUDA device, @p data must hold a pointer to memory resident on it (as
     * returned by @ref buffer() for a CUDA array); its deleter is preserved, so whatever owns the
     * device allocation keeps owning it.
     *
     * @param[in] data   Shared pointer to the byte buffer.
     * @param[in] shape  Shape of the array.
     * @param[in] dtype  Element type. Required: raw bytes carry no type of their own.
     * @param[in] device Device on which the buffer resides. Defaults to the CPU.
     * @param[in] offset Byte offset within @p data at which element 0 begins.
     * @return A new Array backed by @p data.
     */
    static Array fromBuffer(std::shared_ptr<std::byte[]> data,
                            const Shape& shape,
                            Dtype dtype,
                            const Device& device = Device::Cpu(),
                            size_t offset = 0);

    /**
     * @brief Returns this array on the specified device, or a new array if a transfer or copy is required.
     * @details
     * If @p device matches this array's device and @p copy is `false`, the returned
     * array shares the underlying buffer. If @p copy is `true`, a fresh copy is always
     * returned. When the target device differs, a transfer is performed.
     *
     * @param[in] device Target compute device.
     * @param[in] copy   If `true`, always return an independent copy.
     * @return An Array residing on @p device.
     */
    Array toDevice(const Device& device, bool copy = false) const;

    /**
     * @brief Returns this array cast to the specified element type, or a new array if a transfer or copy is required.
     * @details
     * If @p dtype matches this array's dtype and @p copy is `false`, the returned
     * array shares the underlying buffer. If @p copy is `true`, a fresh copy is always
     * returned. When the dtype differs, a new buffer is allocated and all elements are
     * cast to @p dtype.
     *
     * @param[in] dtype Target element type.
     * @param[in] copy  If `true`, always return an independent copy.
     * @return An Array with element type @p dtype.
     */
    Array toDtype(Dtype dtype, bool copy = false) const;

    /**
     * @brief Returns this array broadcast to the specified shape, or a new array if a transfer or copy is required.
     * @details
     * If the shape already matches and @p copy is `false`, the returned array shares
     * the underlying buffer. If @p copy is `true`, a fresh copy is always returned.
     * When broadcasting is required, a new buffer is allocated and elements are
     * replicated along broadcast axes following NumPy rules.
     *
     * @param[in] shape Target shape. Must be broadcast-compatible with this array's shape.
     * @param[in] copy  If `true`, always return an independent copy.
     * @return An Array with shape @p shape.
     * @throws std::invalid_argument if this array's shape cannot be broadcast to @p shape.
     * @see Shape::canBroadcastTo()
     */
    Array broadcastTo(const Shape& shape, bool copy = false) const;

    /**
     * @brief Returns a mutable view of the slice at the given index along axis 0.
     * @details
     * Negative indices are supported and count from the last element along axis 0.
     * The returned array shares the underlying buffer; no data is copied.
     *
     * @param[in] index Index along axis 0, in the range `[-shape()[0], shape()[0])`.
     * @return A mutable Array view with one fewer dimension than this array.
     * @throws std::out_of_range if this array is 0-D or if @p index is out of range.
     */
    Array at(int64_t index);


    /**
     * @brief Overwrites the contents of this array with the given value.
     * @details
     * Constructs a temporary Array from @p value and delegates to @ref set(const Array&).
     * The value is broadcast and cast to match this array's shape and dtype.
     *
     * @param[in] value Scalar or nested vector to write into this array.
     */
    void set(const details::SupportedInputSpecification& value);

    /**
     * @brief Overwrites the contents of this array with the elements of @p other.
     * @details
     * @p other is cast to this array's dtype, transferred to this array's device,
     * and broadcast to this array's shape before the data is copied.
     *
     * @param[in] other Source array. Must be broadcast-compatible with this array's shape.
     * @throws std::invalid_argument if @p other cannot be broadcast to this array's shape.
     */
    void set(const Array& other);

    /**
     * @brief Returns the array contents as a C++ scalar or nested vector, with dtype conversion.
     * @details
     * The requested type `T` determines the output format:
     * - An arithmetic type returns a scalar; the array must contain exactly one element.
     * - `std::vector<U>` returns a 1-D vector; the array must have `ndim() <= 1`.
     * - `std::vector<std::vector<V>>` returns a 2-D vector; the array must have `ndim() <= 2`.
     *
     * Elements are cast from the array's dtype to `T` (or its element type).
     *
     * @tparam T Target type: an arithmetic scalar, `std::vector<U>`, or `std::vector<std::vector<V>>`.
     * @return Array contents as `T`.
     * @throws std::runtime_error if the array rank or element count is incompatible with `T`.
     */
    template <typename T>
    T get() const;

    /**
     * @brief Returns the single element of this array as the requested arithmetic type.
     * @details
     * Stricter than @ref get(): requires the array to contain exactly one element regardless
     * of shape. The element is cast from the array's dtype to `T`.
     *
     * @tparam T Target arithmetic type.
     * @return The sole element cast to `T`.
     * @throws std::runtime_error if the array does not contain exactly one element.
     */
    template <typename T>
    T item() const;

    /**
     * @brief Returns a human-readable string representation of this array.
     * @return A string of the form `Array(data, shape=shape, dtype='dtype', device='device')`.
     */
    std::string toString() const;

private:
    std::shared_ptr<std::byte[]> _copyToCpuBuffer() const;
    std::shared_ptr<std::byte[]> _buildCpuBuffer(const details::SupportedInputSpecification& value);

    Array(std::variant<details::CpuStorage, details::CudaStorage> storage,
          size_t offset,
          Shape shape,
          Dtype dtype,
          Device device);

    std::string _formatItem() const;
    std::string _formatData() const;

    template <typename T>
    void _setItem(std::byte* data, const T& value);

    template <typename T>
    T _getItem(const std::byte* data) const;

protected:
    /**
     * @brief Shape of this array.
     */
    Shape m_shape;

    /**
     * @brief Element data type of this array.
     */
    Dtype m_dtype;

    /**
     * @brief Compute device on which this array resides.
     */
    Device m_device;

    /**
     * @brief Byte offset of element 0 within the underlying buffer.
     */
    size_t m_offset;

    /**
     * @brief Reference-counted backing storage, either CPU or CUDA.
     */
    std::variant<details::CpuStorage, details::CudaStorage> m_storage;
};

} // namespace array
} // namespace common
} // namespace isaacsim

#include "isaacsim/common/array/Array.tpp"
#include "isaacsim/common/array/DLPack.hpp"
#include "isaacsim/common/array/Functions.hpp"
