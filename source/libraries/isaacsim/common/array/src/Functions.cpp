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

#include "isaacsim/common/array/Functions.hpp"

#include "details/CudaKernel.hpp"
#include "details/CudaRuntime.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace
{

// Materializes a new array by gathering elements of `source` through a stride map: for every
// element of the destination, in row-major order, reads the source element at the dot product of
// the destination multi-index with `sourceStrides`. A permutation of the source's contiguous
// strides is how transpose() expresses an axis permutation.
//
// This mirrors the gather in Array::broadcastTo(), which expresses a broadcast axis as a stride of
// 0. The two are deliberately kept as separate copies rather than factored into a shared helper.
Array gather(const Array& source, const std::vector<int64_t>& sourceStrides, const Shape& destinationShape)
{
    const std::vector<int64_t> shape = destinationShape.shape();
    const size_t destinationNdim = shape.size();
    const size_t elementSize = source.dtype().size();
    const size_t totalElements = destinationShape.size();
    const Device device = source.device();

    Array result = empty(destinationShape, source.dtype(), device);
    if (totalElements == 0)
    {
        return result;
    }

    // CUDA
    if (device.isCuda())
    {
        DeviceGuard deviceGuard(device);
        details::CudaKernel::getInstance().broadcastTo(
            source.data(), result.data(), sourceStrides, shape, elementSize, totalElements);
        return result;
    }
    // CPU
    else
    {
        const std::byte* sourceData = static_cast<const std::byte*>(source.data());
        std::byte* destinationData = static_cast<std::byte*>(result.data());

        std::vector<size_t> index(destinationNdim, 0);
        for (size_t linear = 0; linear < totalElements; ++linear)
        {
            size_t sourceOffset = 0;
            for (size_t axis = 0; axis < destinationNdim; ++axis)
            {
                sourceOffset += index[axis] * static_cast<size_t>(sourceStrides[axis]);
            }
            std::memcpy(destinationData + linear * elementSize, sourceData + sourceOffset * elementSize, elementSize);
            // Increment multi-dimensional index (row-major)
            for (size_t axis = destinationNdim; axis-- > 0;)
            {
                if (++index[axis] < static_cast<size_t>(shape[axis]))
                {
                    break;
                }
                index[axis] = 0;
            }
        }
        return result;
    }
}

// Materializes a new array by gathering slices of `source` along a single axis. Views `source` as
// the three-dimensional shape (outer, axisSize, inner) and the destination as
// (outer, indices.size(), inner), so that destination[o][j][i] == source[o][indices[j]][i] -- the
// layout-independent form of NumPy's take() along one axis. Because `inner` elements are
// contiguous in both operands, the copy proceeds in `inner`-sized blocks rather than element by
// element. `indices` must already be resolved to the range [0, axisSize).
Array gatherAxis(const Array& source,
                 const std::vector<int64_t>& indices,
                 size_t outer,
                 size_t axisSize,
                 size_t inner,
                 const Shape& destinationShape)
{
    const size_t elementSize = source.dtype().size();
    const size_t indexCount = indices.size();
    const size_t totalElements = destinationShape.size();
    const Device device = source.device();

    Array result = empty(destinationShape, source.dtype(), device);
    if (totalElements == 0)
    {
        return result;
    }

    // CUDA
    if (device.isCuda())
    {
        DeviceGuard deviceGuard(device);
        details::CudaKernel::getInstance().take(
            source.data(), result.data(), indices, axisSize, inner, elementSize, totalElements);
        return result;
    }
    // CPU
    else
    {
        const std::byte* sourceData = static_cast<const std::byte*>(source.data());
        std::byte* destinationData = static_cast<std::byte*>(result.data());
        const size_t blockSize = inner * elementSize;

        for (size_t outerIndex = 0; outerIndex < outer; ++outerIndex)
        {
            for (size_t position = 0; position < indexCount; ++position)
            {
                const size_t sourceOffset = (outerIndex * axisSize + static_cast<size_t>(indices[position])) * inner;
                const size_t destinationOffset = (outerIndex * indexCount + position) * inner;
                std::memcpy(destinationData + destinationOffset * elementSize, sourceData + sourceOffset * elementSize,
                            blockSize);
            }
        }
        return result;
    }
}

// NumPy's axis normalization: a negative axis counts from the last one, and the result is always a
// valid index into an array of `ndim` dimensions.
int64_t resolveAxis(int64_t axis, size_t ndim, std::string_view callerName)
{
    const int64_t resolved = axis < 0 ? axis + static_cast<int64_t>(ndim) : axis;
    if (resolved < 0 || resolved >= static_cast<int64_t>(ndim))
    {
        throw std::invalid_argument(std::string(callerName) + "(): axis " + std::to_string(axis) +
                                    " is out of range for an array of " + std::to_string(ndim) + " dimensions");
    }
    return resolved;
}

// NumPy's broadcasting of two shapes against each other.
Shape broadcastShapes(const Shape& a, const Shape& b, std::string_view callerName)
{
    const size_t ndim = std::max(a.ndim(), b.ndim());
    std::vector<int64_t> result(ndim, 1);
    for (size_t i = 0; i < ndim; ++i)
    {
        const int64_t axis = -1 - static_cast<int64_t>(i);
        const int64_t dimensionA = i < a.ndim() ? a[axis] : 1;
        const int64_t dimensionB = i < b.ndim() ? b[axis] : 1;
        if (dimensionA != dimensionB && dimensionA != 1 && dimensionB != 1)
        {
            throw std::invalid_argument(std::string(callerName) + "(): cannot broadcast arrays of shapes " +
                                        a.toString() + " and " + b.toString());
        }
        result[ndim - 1 - i] = dimensionA == 1 ? dimensionB : dimensionA;
    }
    return Shape(result);
}

template <typename T, typename Operation>
void applyElementWise(const T* source, T* destination, size_t count, Operation operation)
{
    for (size_t i = 0; i < count; ++i)
    {
        destination[i] = operation(source[i]);
    }
}

template <typename T, typename Operation>
void applyElementWise(const T* source1, const T* source2, T* destination, size_t count, Operation operation)
{
    for (size_t i = 0; i < count; ++i)
    {
        destination[i] = operation(source1[i], source2[i]);
    }
}

// Applies `operation` to each element of the source. The operation is resolved once, outside the loop, so each arm
// compiles down to a tight, vectorizable loop.
template <typename T>
void applyUnaryOp(const T* source, T* destination, size_t count, details::UnaryOp operation)
{
    // Negation of a boolean is rejected by the caller; absolute value of a boolean is the identity.
    // eLogicalNot never reaches here: its destination is boolean rather than T.
    if constexpr (std::is_same_v<T, bool>)
    {
        static_cast<void>(operation);
        applyElementWise(source, destination, count, [](bool value) { return value; });
    }
    else
    {
        switch (operation)
        {
        case details::UnaryOp::eNegative:
            // Unsigned negation is written as `0 - value` so that it wraps modulo the type's range
            // without tripping compiler warnings about applying unary minus to an unsigned type.
            return applyElementWise(source, destination, count, [](T value) { return static_cast<T>(T{} - value); });
        case details::UnaryOp::eAbsolute:
            if constexpr (std::is_unsigned_v<T>)
            {
                return applyElementWise(source, destination, count, [](T value) { return value; });
            }
            else if constexpr (std::is_floating_point_v<T>)
            {
                // std::fabs() rather than a comparison, so that the absolute value of -0.0 is +0.0.
                return applyElementWise(
                    source, destination, count, [](T value) { return static_cast<T>(std::fabs(value)); });
            }
            else
            {
                return applyElementWise(
                    source, destination, count, [](T value) { return static_cast<T>(value < T{} ? -value : value); });
            }
        default:
            break;
        }
        throw std::logic_error("applyUnaryOp(): unknown operation");
    }
}

// Applies `operation` to every element pair. The operation is resolved once, outside the loop, so
// each arm compiles down to a tight, vectorizable loop.
template <typename T>
void applyBinaryOp(const T* source1, const T* source2, T* destination, size_t count, details::BinaryOp operation)
{
    // Boolean addition is a logical OR and boolean multiplication a logical AND.
    // Booleans never reach the other two operations: subtraction is rejected and division promotes
    // to a floating-point dtype.
    if constexpr (std::is_same_v<T, bool>)
    {
        switch (operation)
        {
        case details::BinaryOp::eAdd:
            return applyElementWise(source1, source2, destination, count, [](bool a, bool b) { return a || b; });
        case details::BinaryOp::eMultiply:
            return applyElementWise(source1, source2, destination, count, [](bool a, bool b) { return a && b; });
        default:
            throw std::logic_error("applyBinaryOp(): unsupported boolean operation");
        }
    }
    else
    {
        switch (operation)
        {
        case details::BinaryOp::eAdd:
            return applyElementWise(source1, source2, destination, count, [](T a, T b) { return static_cast<T>(a + b); });
        case details::BinaryOp::eSubtract:
            return applyElementWise(source1, source2, destination, count, [](T a, T b) { return static_cast<T>(a - b); });
        case details::BinaryOp::eMultiply:
            return applyElementWise(source1, source2, destination, count, [](T a, T b) { return static_cast<T>(a * b); });
        case details::BinaryOp::eDivide:
            return applyElementWise(source1, source2, destination, count, [](T a, T b) { return static_cast<T>(a / b); });
        }
        throw std::logic_error("applyBinaryOp(): unknown operation");
    }
}

Array applyUnaryOp(const Array& a, details::UnaryOp operation, std::string_view callerName)
{
    // Negation of a boolean is not supported.
    if (operation == details::UnaryOp::eNegative && a.dtype() == Dtype::Bool())
    {
        throw std::invalid_argument(std::string(callerName) +
                                    "(): boolean negative is not supported; use logicalNot() instead");
    }
    // Logical negation narrows every input kind to a boolean result; the arithmetic operations keep
    // the operand's dtype.
    const bool isLogicalNot = operation == details::UnaryOp::eLogicalNot;
    const Dtype dtype = isLogicalNot ? Dtype::Bool() : a.dtype();
    const Device device = a.device();
    const Shape& shape = a.shape();
    const size_t count = shape.size();
    Array result = empty(shape, dtype, device);

    // CUDA
    if (device.isCuda())
    {
        DeviceGuard deviceGuard(device);
        details::CudaKernel::getInstance().unaryOp(a.data(), result.data(), count, a.dtype().kind(), operation);
        return result;
    }
    // CPU
    else
    {
        Dtype::dispatchByKind(a.dtype().kind(),
                              [&](auto sample)
                              {
                                  using T = decltype(sample);
                                  const auto* source = static_cast<const T*>(a.data());
                                  if (isLogicalNot)
                                  {
                                      auto* destination = static_cast<bool*>(result.data());
                                      for (size_t i = 0; i < count; ++i)
                                      {
                                          destination[i] = !static_cast<bool>(source[i]);
                                      }
                                  }
                                  else
                                  {
                                      applyUnaryOp(source, static_cast<T*>(result.data()), count, operation);
                                  }
                              });
        return result;
    }
}

Dtype reductionDtype(details::ReduceOp operation, Dtype dtype)
{
    switch (operation)
    {
    case details::ReduceOp::eAll:
    case details::ReduceOp::eAny:
        return Dtype::Bool();
    case details::ReduceOp::eMin:
    case details::ReduceOp::eMax:
        return dtype;
    case details::ReduceOp::eSum:
    case details::ReduceOp::eProd:
        if (dtype.isFloating())
        {
            return dtype;
        }
        // Boolean is neither signed nor unsigned here, so it accumulates in int64.
        return dtype.isUnsigned() ? Dtype::UInt64() : Dtype::Int64();
    }
    throw std::logic_error("reductionDtype(): unknown operation");
}

// eAll and eAny: reduces to a boolean for every operand kind. Both algorithms stop at the first
// element that decides the result, and their empty-range answers are the identities NumPy returns
// for an empty array. The CUDA path folds every element instead, so it agrees on the value but not
// on how much it reads.
template <typename T, bool IsAll>
bool allAnyReduce(const T* source, size_t count)
{
    auto isTrue = [](T value) { return static_cast<bool>(value); };
    if constexpr (IsAll)
    {
        return std::all_of(source, source + count, isTrue);
    }
    else
    {
        return std::any_of(source, source + count, isTrue);
    }
}

// eMin and eMax: reduces to an element of T, non-empty since the caller rejects an empty operand
// for the extremum reductions. This is the only pair booleans reach: sum and product promote a
// boolean operand to an integer accumulator.
template <typename T, bool IsMin>
T minMaxReduce(const T* source, size_t count)
{
    if constexpr (std::is_same_v<T, bool>)
    {
        if constexpr (IsMin)
        {
            return std::all_of(source, source + count, [](bool value) { return value; });
        }
        else
        {
            return std::any_of(source, source + count, [](bool value) { return value; });
        }
    }
    else
    {
        // NumPy propagates NaN through the extremum reductions. Every comparison against NaN is
        // false, so min_element()/max_element() would silently skip it: test for it up front.
        if constexpr (std::is_floating_point_v<T>)
        {
            if (std::any_of(source, source + count, [](T value) { return std::isnan(value); }))
            {
                return std::numeric_limits<T>::quiet_NaN();
            }
        }
        if constexpr (IsMin)
        {
            return *std::min_element(source, source + count);
        }
        else
        {
            return *std::max_element(source, source + count);
        }
    }
}

// eSum and eProd: reduces to an element of T, which the caller has already promoted to the
// accumulator kind. Booleans never reach this pair -- sum and product promote a boolean operand to
// an integer accumulator before calling -- so the arithmetic below is guarded out of
// the T=bool instantiation, which would otherwise warn on multiplying two booleans.
template <typename T, bool IsSum>
T sumProdReduce(const T* source, size_t count)
{
    if constexpr (std::is_same_v<T, bool>)
    {
        return T{ !IsSum };
    }
    else if constexpr (IsSum)
    {
        return std::accumulate(
            source, source + count, T{}, [](T total, T value) { return static_cast<T>(total + value); });
    }
    else
    {
        return std::accumulate(
            source, source + count, static_cast<T>(1), [](T total, T value) { return static_cast<T>(total * value); });
    }
}

Array applyReduceOp(const Array& a, details::ReduceOp operation, std::string_view callerName)
{
    const size_t count = a.shape().size();
    const bool isExtremum = operation == details::ReduceOp::eMin || operation == details::ReduceOp::eMax;
    // Every other reduction has an identity to fall back on; the extremum of nothing is undefined,
    // so an empty operand is an error.
    if (isExtremum && count == 0)
    {
        throw std::invalid_argument(std::string(callerName) + "(): reduction of an empty array is not supported");
    }

    const Device device = a.device();
    const Dtype dtype = reductionDtype(operation, a.dtype());
    const bool isBooleanResult = operation == details::ReduceOp::eAll || operation == details::ReduceOp::eAny;
    // all() and any() read the operand in its own dtype and narrow it to a boolean while reducing;
    // every other reduction accumulates in the result dtype, so the operand is cast up front.
    const Array source = isBooleanResult ? a : a.toDtype(dtype);
    Array result = empty(Shape(), dtype, device);

    // CUDA
    if (device.isCuda())
    {
        DeviceGuard deviceGuard(device);
        details::CudaKernel::getInstance().reduceOp(
            source.data(), result.data(), count, source.dtype().kind(), operation);
        return result;
    }
    // CPU
    else
    {
        Dtype::dispatchByKind(source.dtype().kind(),
                              [&](auto sample)
                              {
                                  using T = decltype(sample);
                                  const auto* values = static_cast<const T*>(source.data());
                                  switch (operation)
                                  {
                                  case details::ReduceOp::eAll:
                                      *static_cast<bool*>(result.data()) = allAnyReduce<T, true>(values, count);
                                      break;
                                  case details::ReduceOp::eAny:
                                      *static_cast<bool*>(result.data()) = allAnyReduce<T, false>(values, count);
                                      break;
                                  case details::ReduceOp::eMin:
                                      *static_cast<T*>(result.data()) = minMaxReduce<T, true>(values, count);
                                      break;
                                  case details::ReduceOp::eMax:
                                      *static_cast<T*>(result.data()) = minMaxReduce<T, false>(values, count);
                                      break;
                                  case details::ReduceOp::eSum:
                                      *static_cast<T*>(result.data()) = sumProdReduce<T, true>(values, count);
                                      break;
                                  case details::ReduceOp::eProd:
                                      *static_cast<T*>(result.data()) = sumProdReduce<T, false>(values, count);
                                      break;
                                  }
                              });
        return result;
    }
}

Array applyBinaryOp(const Array& a, const Array& b, details::BinaryOp operation, std::string_view callerName)
{
    // Device validation
    const Device device = a.device();
    if (a.device() != b.device())
    {
        throw std::invalid_argument(std::string(callerName) + "(): arrays reside on different devices: " +
                                    a.device().toString() + " and " + b.device().toString());
    }

    // Dtype promotion
    Dtype dtype = Dtype::promoteDtypes(a.dtype(), b.dtype());
    // - Boolean subtract is not supported
    if (operation == details::BinaryOp::eSubtract && dtype == Dtype::Bool())
    {
        throw std::invalid_argument(std::string(callerName) + "(): boolean subtract is not supported");
    }
    // - True division: boolean and integer operands are computed in floating-point.
    if (operation == details::BinaryOp::eDivide && !dtype.isFloating())
    {
        dtype = Dtype::Float64();
    }

    const Shape shape = broadcastShapes(a.shape(), b.shape(), callerName);
    const size_t count = shape.size();
    const Array source1 = a.toDtype(dtype).broadcastTo(shape);
    const Array source2 = b.toDtype(dtype).broadcastTo(shape);
    Array result = empty(shape, dtype, device);

    // CUDA
    if (device.isCuda())
    {
        DeviceGuard deviceGuard(device);
        details::CudaKernel::getInstance().binaryOp(
            source1.data(), source2.data(), result.data(), count, dtype.kind(), operation);
        return result;
    }
    // CPU
    else
    {
        Dtype::dispatchByKind(dtype.kind(),
                              [&](auto sample)
                              {
                                  using T = decltype(sample);
                                  applyBinaryOp(static_cast<const T*>(source1.data()),
                                                static_cast<const T*>(source2.data()), static_cast<T*>(result.data()),
                                                count, operation);
                              });
        return result;
    }
}

} // namespace

Array empty(const Shape& shape, Dtype dtype, const Device& device)
{
    const size_t byteCount = shape.size() * dtype.size();

    // CUDA
    if (device.isCuda())
    {
        // A zero-element array still allocates one byte: cudaMalloc(0) yields a null pointer, which
        // data() would then hand out as if it addressed elements. `new std::byte[0]` already returns
        // a valid distinct pointer, so the CPU branch needs no such guard.
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(device);
            CudaRuntime::getInstance().cudaMalloc(&devicePointer, std::max<size_t>(byteCount, 1));
        }
        // Runs from a shared_ptr destructor, which is a noexcept context: anything escaping here
        // calls std::terminate. cudaFree() is asked not to throw, and the catch-all covers
        // CudaRuntime::getInstance() and DeviceGuard's constructor, both of which can fail during
        // process teardown or after a cudaDeviceReset(). Leaking an allocation that is already
        // being torn down beats aborting the process.
        //
        // It does not cover ~DeviceGuard(), which restores the previous device through a throwing
        // cudaSetDevice() from its own noexcept destructor; that would terminate before unwinding
        // reaches this handler. Fixing it belongs in DeviceGuard rather than here.
        auto deleter = [device](std::byte* pointer) noexcept
        {
            try
            {
                DeviceGuard deleterDeviceGuard(device);
                CudaRuntime::getInstance().cudaFree(pointer, false);
            }
            catch (...)
            {
            }
        };
        return Array::fromBuffer(std::shared_ptr<std::byte[]>(static_cast<std::byte*>(devicePointer), std::move(deleter)),
                                 shape, dtype, device);
    }
    // CPU
    else
    {
        // Deliberately not value-initialized: the elements are unspecified, as in numpy.empty().
        return Array::fromBuffer(std::shared_ptr<std::byte[]>(new std::byte[byteCount]), shape, dtype, device);
    }
}

Array zeros(const Shape& shape, Dtype dtype, const Device& device)
{
    Array result = empty(shape, dtype, device);
    const size_t byteCount = result.nbytes();
    if (byteCount == 0)
    {
        return result;
    }

    // A zeroed byte pattern is the representation of 0 for every supported dtype -- false for bool,
    // 0 for the integers, and +0.0 for the IEEE floats -- so a plain memset covers all of them.
    // CUDA
    if (device.isCuda())
    {
        DeviceGuard deviceGuard(device);
        CudaRuntime::getInstance().cudaMemset(result.data(), 0, byteCount);
    }
    // CPU
    else
    {
        std::memset(result.data(), 0, byteCount);
    }
    return result;
}

Array ones(const Shape& shape, Dtype dtype, const Device& device)
{
    if (shape.size() == 0)
    {
        return empty(shape, dtype, device);
    }
    // Unlike zeros(), one has no dtype-independent byte pattern (float 1.0f is 0x3F800000), so this
    // casts a scalar to `dtype` and broadcasts it out, reusing the existing gather rather than
    // adding a fill kernel. broadcastTo() from a 0-D operand materializes a dense result, so no
    // further copy is needed.
    return Array(int32_t{ 1 }, dtype, device).broadcastTo(shape);
}

Array add(const Array& a, const Array& b)
{
    return applyBinaryOp(a, b, details::BinaryOp::eAdd, "add");
}

Array add(const Array& a, const details::SupportedInputSpecification& b)
{
    return add(a, Array(b, std::nullopt, a.device()));
}

Array add(const details::SupportedInputSpecification& a, const Array& b)
{
    return add(Array(a, std::nullopt, b.device()), b);
}

Array subtract(const Array& a, const Array& b)
{
    return applyBinaryOp(a, b, details::BinaryOp::eSubtract, "subtract");
}

Array subtract(const Array& a, const details::SupportedInputSpecification& b)
{
    return subtract(a, Array(b, std::nullopt, a.device()));
}

Array subtract(const details::SupportedInputSpecification& a, const Array& b)
{
    return subtract(Array(a, std::nullopt, b.device()), b);
}

Array multiply(const Array& a, const Array& b)
{
    return applyBinaryOp(a, b, details::BinaryOp::eMultiply, "multiply");
}

Array multiply(const Array& a, const details::SupportedInputSpecification& b)
{
    return multiply(a, Array(b, std::nullopt, a.device()));
}

Array multiply(const details::SupportedInputSpecification& a, const Array& b)
{
    return multiply(Array(a, std::nullopt, b.device()), b);
}

Array divide(const Array& a, const Array& b)
{
    return applyBinaryOp(a, b, details::BinaryOp::eDivide, "divide");
}

Array divide(const Array& a, const details::SupportedInputSpecification& b)
{
    return divide(a, Array(b, std::nullopt, a.device()));
}

Array divide(const details::SupportedInputSpecification& a, const Array& b)
{
    return divide(Array(a, std::nullopt, b.device()), b);
}

Array logicalNot(const Array& a)
{
    return applyUnaryOp(a, details::UnaryOp::eLogicalNot, "logicalNot");
}

Array negative(const Array& a)
{
    return applyUnaryOp(a, details::UnaryOp::eNegative, "negative");
}

Array absolute(const Array& a)
{
    return applyUnaryOp(a, details::UnaryOp::eAbsolute, "absolute");
}

Array all(const Array& a)
{
    return applyReduceOp(a, details::ReduceOp::eAll, "all");
}

Array any(const Array& a)
{
    return applyReduceOp(a, details::ReduceOp::eAny, "any");
}

Array sum(const Array& a)
{
    return applyReduceOp(a, details::ReduceOp::eSum, "sum");
}

Array prod(const Array& a)
{
    return applyReduceOp(a, details::ReduceOp::eProd, "prod");
}

Array amin(const Array& a)
{
    return applyReduceOp(a, details::ReduceOp::eMin, "amin");
}

Array amax(const Array& a)
{
    return applyReduceOp(a, details::ReduceOp::eMax, "amax");
}

Array operator+(const Array& a, const Array& b)
{
    return add(a, b);
}

Array operator+(const Array& a, const details::SupportedInputSpecification& b)
{
    return add(a, b);
}

Array operator+(const details::SupportedInputSpecification& a, const Array& b)
{
    return add(a, b);
}

Array operator-(const Array& a, const Array& b)
{
    return subtract(a, b);
}

Array operator-(const Array& a, const details::SupportedInputSpecification& b)
{
    return subtract(a, b);
}

Array operator-(const details::SupportedInputSpecification& a, const Array& b)
{
    return subtract(a, b);
}

Array operator*(const Array& a, const Array& b)
{
    return multiply(a, b);
}

Array operator*(const Array& a, const details::SupportedInputSpecification& b)
{
    return multiply(a, b);
}

Array operator*(const details::SupportedInputSpecification& a, const Array& b)
{
    return multiply(a, b);
}

Array operator/(const Array& a, const Array& b)
{
    return divide(a, b);
}

Array operator/(const Array& a, const details::SupportedInputSpecification& b)
{
    return divide(a, b);
}

Array operator/(const details::SupportedInputSpecification& a, const Array& b)
{
    return divide(a, b);
}

Array transpose(const Array& a, const std::optional<std::vector<int64_t>>& axes)
{
    const size_t ndim = a.ndim();

    std::vector<int64_t> reversedAxes;
    if (!axes.has_value())
    {
        reversedAxes.resize(ndim);
        std::iota(reversedAxes.rbegin(), reversedAxes.rend(), int64_t{ 0 });
    }

    const std::vector<int64_t>& requestedAxes = axes.has_value() ? *axes : reversedAxes;
    if (requestedAxes.size() != ndim)
    {
        throw std::invalid_argument("transpose(): expected one axis per dimension, got " +
                                    std::to_string(requestedAxes.size()) + " for an array of shape " +
                                    a.shape().toString());
    }

    std::vector<int64_t> resolvedAxes(ndim);
    std::vector<bool> visited(ndim, false);
    for (size_t i = 0; i < ndim; ++i)
    {
        const int64_t axis = resolveAxis(requestedAxes[i], ndim, "transpose");
        if (visited[static_cast<size_t>(axis)])
        {
            throw std::invalid_argument("transpose(): axis " + std::to_string(axis) + " appears more than once");
        }
        visited[static_cast<size_t>(axis)] = true;
        resolvedAxes[i] = axis;
    }

    // Contiguous source strides (in elements), then reordered onto the destination axes so that
    // axis i of the result walks the source along its axis `resolvedAxes[i]`.
    std::vector<int64_t> contiguousStrides(ndim);
    int64_t stride = 1;
    for (size_t i = ndim; i-- > 0;)
    {
        contiguousStrides[i] = stride;
        stride *= a.shape()[static_cast<int64_t>(i)];
    }

    std::vector<int64_t> sourceStrides(ndim);
    std::vector<int64_t> destinationShape(ndim);
    for (size_t i = 0; i < ndim; ++i)
    {
        sourceStrides[i] = contiguousStrides[static_cast<size_t>(resolvedAxes[i])];
        destinationShape[i] = a.shape()[resolvedAxes[i]];
    }
    return gather(a, sourceStrides, Shape(destinationShape));
}

Array take(const Array& a, const std::vector<int64_t>& indices, const std::optional<int64_t>& axis)
{
    // NumPy's default axis=None gathers from the flattened operand and yields a 1-D result. A 0-D
    // operand is also flattened even when an axis is given, because NumPy treats it as 1-D of size
    // 1 there rather than rejecting it: np.take(np.array(1.0), [0], axis=0) is array([1.0]), and
    // its axis=1 error reports "an array of dimension 1".
    const bool gathersAlongAxis = axis.has_value() && a.ndim() > 0;
    const Array source = gathersAlongAxis ? a : a.flatten();
    const int64_t resolvedAxis = resolveAxis(axis.value_or(0), source.ndim(), "take");
    const size_t ndim = source.ndim();
    const int64_t axisSize = source.shape()[resolvedAxis];

    std::vector<int64_t> resolvedIndices(indices.size());
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const int64_t index = indices[i] < 0 ? indices[i] + axisSize : indices[i];
        if (index < 0 || index >= axisSize)
        {
            // NumPy separates the two failures the same way: an out-of-range axis is an AxisError,
            // which is a ValueError, while an out-of-range index is an IndexError. Array::at()
            // already reports an out-of-range index as std::out_of_range.
            throw std::out_of_range("take(): index " + std::to_string(indices[i]) + " is out of range for axis " +
                                    std::to_string(resolvedAxis) + " of size " + std::to_string(axisSize));
        }
        resolvedIndices[i] = index;
    }

    // Collapse the operand to (outer, axisSize, inner): every axis other than the indexed one only
    // contributes to the size of a contiguous block that moves as a unit.
    size_t outer = 1;
    size_t inner = 1;
    for (size_t i = 0; i < ndim; ++i)
    {
        const int64_t dimension = source.shape()[static_cast<int64_t>(i)];
        if (static_cast<int64_t>(i) < resolvedAxis)
        {
            outer *= static_cast<size_t>(dimension);
        }
        else if (static_cast<int64_t>(i) > resolvedAxis)
        {
            inner *= static_cast<size_t>(dimension);
        }
    }

    std::vector<int64_t> destinationShape = source.shape().shape();
    destinationShape[static_cast<size_t>(resolvedAxis)] = static_cast<int64_t>(resolvedIndices.size());
    return gatherAxis(source, resolvedIndices, outer, static_cast<size_t>(axisSize), inner, Shape(destinationShape));
}

Array operator-(const Array& a)
{
    return negative(a);
}

Array operator!(const Array& a)
{
    return logicalNot(a);
}

} // namespace array
} // namespace common
} // namespace isaacsim
