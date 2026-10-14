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

#include "isaacsim/common/array/Array.hpp"

#include "details/ArrayUtils.hpp"
#include "details/CudaKernel.hpp"
#include "details/CudaRuntime.hpp"
#include "details/Storage.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>

#include <algorithm>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace details
{

static Dtype inferDtype(const SupportedInputSpecification& value)
{
    return std::visit(
        [](const auto& v) -> Dtype
        {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_arithmetic_v<T>)
                return Dtype::fromType<T>();
            else if constexpr (std::is_arithmetic_v<typename T::value_type>)
                return Dtype::fromType<typename T::value_type>();
            else
                return Dtype::fromType<typename T::value_type::value_type>();
        },
        value);
}

std::vector<int64_t> computeContiguousStrides(const Shape& shape)
{
    const bool empty = shape.size() == 0;
    std::vector<int64_t> strides(shape.ndim(), 1);
    int64_t stride = 1;
    for (size_t axis = shape.ndim(); axis-- > 0;)
    {
        strides[axis] = stride;
        const int64_t dimension = shape[static_cast<int64_t>(axis)];
        if (dimension < 0)
        {
            throw std::invalid_argument("Array: negative dimensions are not supported");
        }
        if (dimension != 0 && stride > std::numeric_limits<int64_t>::max() / dimension)
        {
            if (empty)
            {
                return std::vector<int64_t>(shape.ndim(), 0);
            }
            throw std::overflow_error("Array: contiguous strides overflow int64_t");
        }
        stride *= dimension;
    }
    return strides;
}

std::uintptr_t operationStream(const Device& device)
{
    if (!device.isCuda())
    {
        return 0;
    }

    // Created with the default (blocking) flag rather than CU_STREAM_NON_BLOCKING so it stays
    // implicitly ordered against the legacy default stream. Callers still holding a raw data()
    // pointer enqueue their work there, and that implicit ordering is the only thing keeping them
    // correct; a non-blocking stream would silently race them.
    //
    // Deliberately never destroyed. Teardown order against the CUDA runtime is not guaranteed and
    // destroying a stream after the driver has shut down is undefined, the same reasoning that
    // governs the retirement service's quarantine path.
    static std::mutex mutex;
    static std::unordered_map<int, CUstream> streams;

    const int ordinal = device.ordinal();
    std::lock_guard<std::mutex> lock(mutex);
    const auto existing = streams.find(ordinal);
    if (existing != streams.end())
    {
        return reinterpret_cast<std::uintptr_t>(existing->second);
    }

    DeviceGuard deviceGuard(device);
    CUstream stream = nullptr;
    CudaRuntime::getInstance().cudaStreamCreateWithFlags(&stream, CU_STREAM_DEFAULT);
    try
    {
        streams.emplace(ordinal, stream);
    }
    catch (...)
    {
        CudaRuntime::getInstance(false).cudaStreamDestroy(stream, false);
        throw;
    }
    return reinterpret_cast<std::uintptr_t>(stream);
}

size_t computeStorageSpanNbytes(const Shape& shape, const std::vector<int64_t>& strides, const Dtype& dtype)
{
    if (strides.size() != shape.ndim())
    {
        throw std::invalid_argument("Array: strides must have the same rank as the shape");
    }
    if (std::any_of(strides.begin(), strides.end(), [](int64_t stride) { return stride < 0; }))
    {
        throw std::invalid_argument("Array: negative strides are not supported");
    }
    if (shape.size() == 0)
    {
        return 0;
    }

    size_t maximumElementOffset = 0;
    for (size_t axis = 0; axis < shape.ndim(); ++axis)
    {
        const size_t dimension = static_cast<size_t>(shape[static_cast<int64_t>(axis)]);
        const size_t stride = static_cast<size_t>(strides[axis]);
        if (dimension > 1 && stride > (std::numeric_limits<size_t>::max() - maximumElementOffset) / (dimension - 1))
        {
            throw std::overflow_error("Array: strided buffer span overflows size_t");
        }
        maximumElementOffset += (dimension - 1) * stride;
    }
    if (maximumElementOffset > std::numeric_limits<size_t>::max() / dtype.size() - 1)
    {
        throw std::overflow_error("Array: strided buffer span overflows size_t");
    }
    return (maximumElementOffset + 1) * dtype.size();
}

} // namespace details

namespace
{

struct StorageOwner
{
    void operator()(std::byte*) noexcept
    {
    }

    std::shared_ptr<details::Storage> storage;
};

// Whether the layout maps more than one logical position onto the same buffer location, which is
// what makes a broadcast view read-only.
bool hasReplicatedElements(const Shape& shape, const std::vector<int64_t>& strides)
{
    // An empty array addresses no element, so no write through it can alias. Checked first because
    // a contiguous layout puts a 0 stride on every axis preceding a zero-length one -- shape {4, 0}
    // has strides {0, 1} -- which the test below would misread.
    if (shape.size() == 0)
    {
        return false;
    }
    for (size_t axis = 0; axis < strides.size(); ++axis)
    {
        if (strides[axis] == 0 && shape[static_cast<int64_t>(axis)] > 1)
        {
            return true;
        }
    }
    return false;
}

// Element-wise copy between two host-resident strided layouts sharing `shape`, walking the logical
// elements in row-major order. A source stride of 0 replicates that axis; contiguous
// `destinationStrides` make this a compacting gather.
void copyStridedCpu(const std::byte* source,
                    const std::vector<int64_t>& sourceStrides,
                    std::byte* destination,
                    const std::vector<int64_t>& destinationStrides,
                    const Shape& shape,
                    size_t elementSize,
                    size_t totalElements)
{
    const size_t rank = shape.ndim();
    std::vector<size_t> index(rank, 0);
    for (size_t linear = 0; linear < totalElements; ++linear)
    {
        size_t sourceOffset = 0;
        size_t destinationOffset = 0;
        for (size_t axis = 0; axis < rank; ++axis)
        {
            sourceOffset += index[axis] * static_cast<size_t>(sourceStrides[axis]);
            destinationOffset += index[axis] * static_cast<size_t>(destinationStrides[axis]);
        }
        std::memcpy(destination + destinationOffset * elementSize, source + sourceOffset * elementSize, elementSize);
        // Increment the multi-dimensional index (row-major).
        for (size_t axis = rank; axis-- > 0;)
        {
            if (++index[axis] < static_cast<size_t>(shape[static_cast<int64_t>(axis)]))
            {
                break;
            }
            index[axis] = 0;
        }
    }
}

// Allocates `byteCount` bytes on `device` and hands the pointer to `fill` under a DeviceGuard,
// freeing the allocation if `fill` throws. The returned deleter re-establishes the guard, so
// whichever device is current when the last reference drops, the memory goes back to the right one.
template <typename FillFunction>
std::shared_ptr<std::byte[]> allocateCudaBuffer(const Device& device, size_t byteCount, FillFunction&& fill)
{
    auto& cudaRuntime = CudaRuntime::getInstance();
    void* devicePointer = nullptr;
    {
        DeviceGuard deviceGuard(device);
        // cuMemAlloc() rejects a zero size; one byte keeps data() addressable for an empty array.
        cudaRuntime.cudaMalloc(&devicePointer, std::max<size_t>(byteCount, 1));
        try
        {
            fill(devicePointer);
        }
        catch (...)
        {
            cudaRuntime.cudaFree(devicePointer, false);
            throw;
        }
    }
    return std::shared_ptr<std::byte[]>(static_cast<std::byte*>(devicePointer),
                                        [device](std::byte* pointer) noexcept
                                        {
                                            try
                                            {
                                                DeviceGuard deleterDeviceGuard(device);
                                                CudaRuntime::getInstance(false).cudaFree(pointer, false);
                                            }
                                            catch (...)
                                            {
                                            }
                                        });
}

std::shared_ptr<details::Storage> makeStorage(std::shared_ptr<std::byte[]> data, size_t capacity, const Device& device)
{
    if (StorageOwner* owner = std::get_deleter<StorageOwner>(data))
    {
        if (owner->storage && owner->storage->pointer() == data.get() && owner->storage->device() == device)
        {
            if (capacity > owner->storage->capacity())
            {
                throw std::invalid_argument("Array::fromBuffer(): declared capacity exceeds the retained allocation");
            }
            return owner->storage;
        }
    }
    return std::make_shared<details::Storage>(std::move(data), capacity, device);
}

void validateCudaPointer(const std::shared_ptr<std::byte[]>& data, const Device& device)
{
    if (!device.isCuda() || !data)
    {
        return;
    }

    DeviceGuard deviceGuard(device);
    // Queried without throwing: the driver reports a plain host allocation as an outright error
    // rather than describing it, and "not device memory" is exactly the case this rejects below.
    CudaRuntime::PointerAttributes attributes{};
    CudaRuntime::getInstance().cudaPointerGetAttributes(&attributes, data.get(), /*throwIfInvalid=*/false);
    if (!attributes.isDevice || attributes.device != device.ordinal())
    {
        throw std::invalid_argument("Array::fromBuffer(): buffer does not reside on " + device.toString());
    }
}

// Whether two arrays may address the same allocation. owner_before() in both directions is the
// standard "same control block" test; the raw pointer comparison additionally catches a borrowed
// alias (a DLPack view with a no-op deleter) pointing into an allocation it does not own.
bool haveSharedStorage(const Array& first, const Array& second)
{
    const std::shared_ptr<std::byte[]> firstBuffer = first.buffer();
    const std::shared_ptr<std::byte[]> secondBuffer = second.buffer();
    return firstBuffer.get() == secondBuffer.get() ||
           (!firstBuffer.owner_before(secondBuffer) && !secondBuffer.owner_before(firstBuffer));
}

} // namespace

Array::Array(const details::SupportedInputSpecification& value, const std::optional<Dtype>& dtype, const Device& device)
    // Storage allocated below is owned by this array alone, so nothing aliases it.
    : m_writeable(true), m_dtype(dtype.has_value() ? *dtype : details::inferDtype(value)), m_device(device), m_offset(0)
{
    // CUDA
    if (device.isCuda())
    {
        if (!device.isAvailable())
        {
            throw std::runtime_error("Array: CUDA device not available: " + device.toString());
        }
        // _buildCpuBuffer() sets m_shape, so nbytes() is only meaningful after it runs.
        const auto cpuBuffer = _buildCpuBuffer(value);
        const size_t byteCount = this->nbytes();
        m_storage = makeStorage(
            allocateCudaBuffer(device, byteCount,
                               [&](void* devicePointer)
                               { CudaRuntime::getInstance().cudaMemcpy(devicePointer, cpuBuffer.get(), byteCount); }),
            byteCount, device);
        m_storage->recordExternalWrite(0);
    }
    // CPU
    else
    {
        auto buffer = _buildCpuBuffer(value);
        m_storage = makeStorage(std::move(buffer), this->nbytes(), device);
    }
    m_strides = details::computeContiguousStrides(m_shape);
}

const Shape& Array::shape() const
{
    return m_shape;
}

const std::vector<int64_t>& Array::strides() const
{
    return m_strides;
}

bool Array::isContiguous() const
{
    if (this->size() == 0)
    {
        return true;
    }

    // expectedStride is bounded by size(), which the constructor already proved representable, so
    // the running product cannot overflow here.
    int64_t expectedStride = 1;
    for (size_t axis = this->ndim(); axis-- > 0;)
    {
        const int64_t dimension = m_shape[static_cast<int64_t>(axis)];
        if (dimension > 1 && m_strides[axis] != expectedStride)
        {
            return false;
        }
        expectedStride *= dimension;
    }
    return true;
}

bool Array::isWriteable() const
{
    return m_writeable;
}

Array Array::_viewWithLayout(Shape shape, std::vector<int64_t> strides, std::optional<size_t> offset) const
{
    return Array(
        m_storage, offset.value_or(m_offset), std::move(shape), std::move(strides), m_dtype, m_device, m_writeable);
}

void Array::_requireWriteable(const std::string& callerName) const
{
    if (!this->isWriteable())
    {
        // Deliberately not contiguous(): it decides on layout alone, so it hands a contiguous read-only
        // array straight back and would land the caller right back here.
        throw std::runtime_error(callerName +
                                 ": the array is read-only because it replicates elements or was derived from an "
                                 "array that does; call copy() for an independent, writeable array");
    }
}

Device Array::device() const
{
    return m_device;
}

Dtype Array::dtype() const
{
    return m_dtype;
}

size_t Array::ndim() const
{
    return m_shape.ndim();
}

size_t Array::size() const
{
    return m_shape.size();
}

size_t Array::nbytes() const
{
    return this->size() * m_dtype.size();
}

size_t Array::capacity() const
{
    return m_storage->capacity();
}

size_t Array::offset() const
{
    return m_offset;
}

void* Array::data()
{
    this->_requireWriteable("Array::data()");
    return const_cast<void*>(std::as_const(*this).data());
}

const void* Array::data() const
{
    const auto* data = m_storage->pointer();
    return data ? data + m_offset : nullptr;
}

std::shared_ptr<std::byte[]> Array::buffer() const
{
    return std::shared_ptr<std::byte[]>(m_storage->pointer(), StorageOwner{ m_storage });
}

Array Array::reshape(const Shape& shape) const
{
    const Shape resolvedShape = m_shape.resolve(shape);
    // The contiguity check comes first: a strided array must be compacted even when the requested
    // shape already matches, otherwise the result would keep strides that no longer describe a
    // row-major layout.
    if (!this->isContiguous())
    {
        return this->copy().reshape(resolvedShape);
    }
    if (resolvedShape == m_shape)
    {
        return *this;
    }
    return _viewWithLayout(resolvedShape, details::computeContiguousStrides(resolvedShape));
}

Array Array::flatten() const
{
    return this->reshape(Shape({ static_cast<int64_t>(this->size()) }));
}

Array Array::contiguous() const
{
    return this->isContiguous() ? *this : this->copy();
}

Array Array::copy() const
{
    const size_t byteCount = this->nbytes();
    const size_t totalElements = this->size();
    const std::vector<int64_t> denseStrides = details::computeContiguousStrides(m_shape);

    std::shared_ptr<std::byte[]> buffer;
    if (m_device.isCuda())
    {
        buffer = allocateCudaBuffer(m_device, byteCount, [](void*) {});
    }
    else
    {
        buffer = std::shared_ptr<std::byte[]>(new std::byte[byteCount]);
    }
    const size_t capacity = m_device.isCuda() ? std::max<size_t>(byteCount, 1) : byteCount;
    Array result(makeStorage(std::move(buffer), capacity, m_device), 0, m_shape, denseStrides, m_dtype, m_device, true);
    if (byteCount != 0)
    {
        // Both operands live on this device, so the operation only has to be ordered on the stream:
        // releasing the accesses records the producer event, and any later consumer waits on it.
        // Nothing here is host-visible, so the host never blocks on the copy.
        const std::uintptr_t stream = details::operationStream(m_device);
        ArrayReadAccess sourceAccess = acquireRead(*this, stream);
        ArrayWriteAccess resultAccess = acquireWrite(result, stream);
        if (this->isContiguous())
        {
            if (m_device.isCuda())
            {
                DeviceGuard deviceGuard(m_device);
                CudaRuntime::getInstance().cudaMemcpyAsync(
                    resultAccess.data(), sourceAccess.data(), byteCount, reinterpret_cast<CUstream>(stream));
            }
            else
            {
                std::memcpy(resultAccess.data(), sourceAccess.data(), byteCount);
            }
        }
        else if (m_device.isCuda())
        {
            DeviceGuard deviceGuard(m_device);
            details::CudaKernel::getInstance().copyStrided(sourceAccess.data(), resultAccess.data(), m_strides,
                                                           denseStrides, m_shape.shape(), m_dtype.size(), totalElements,
                                                           reinterpret_cast<CUstream>(stream));
        }
        else
        {
            copyStridedCpu(static_cast<const std::byte*>(sourceAccess.data()), m_strides,
                           static_cast<std::byte*>(resultAccess.data()), denseStrides, m_shape, m_dtype.size(),
                           totalElements);
        }
    }
    return result;
}

Array Array::clone() const
{
    // The constructor proved that m_offset plus the reachable span fits size_t, so this sum is safe.
    const size_t totalSize = m_offset + _storageSpanNbytes();

    std::shared_ptr<std::byte[]> newData;
    if (m_device.isCuda())
    {
        newData = allocateCudaBuffer(m_device, totalSize, [](void*) {});
    }
    else
    {
        newData = std::shared_ptr<std::byte[]>(new std::byte[totalSize]());
    }
    const size_t capacity = m_device.isCuda() ? std::max<size_t>(totalSize, 1) : totalSize;
    Array result(
        makeStorage(std::move(newData), capacity, m_device), m_offset, m_shape, m_strides, m_dtype, m_device, true);
    if (totalSize != 0)
    {
        ArrayReadAccess sourceAccess = acquireRead(*this);
        // clone() preserves read-only layout semantics, but initialization owns this fresh storage
        // exclusively and must be able to populate it before restoring that public property.
        result.m_writeable = true;
        ArrayWriteAccess resultAccess = acquireWrite(result);
        const auto* sourceBase = static_cast<const std::byte*>(sourceAccess.data()) - m_offset;
        auto* destinationBase = static_cast<std::byte*>(resultAccess.data()) - m_offset;
        if (m_device.isCuda())
        {
            DeviceGuard deviceGuard(m_device);
            CudaRuntime::getInstance().cudaMemcpy(destinationBase, sourceBase, totalSize);
        }
        else
        {
            std::memcpy(destinationBase, sourceBase, totalSize);
        }
    }
    result.m_writeable = m_writeable;
    return result;
}

Array Array::fromBuffer(std::shared_ptr<std::byte[]> data, const Shape& shape, Dtype dtype, const Device& device, size_t offset)
{
    const auto strides = details::computeContiguousStrides(shape);
    const size_t span = details::computeStorageSpanNbytes(shape, strides, dtype);
    if (offset > std::numeric_limits<size_t>::max() - span)
    {
        throw std::overflow_error("Array::fromBuffer(): byte offset and contiguous span overflow size_t");
    }
    return fromBuffer(std::move(data), offset + span, shape, dtype, device, offset, strides);
}

Array Array::fromBuffer(std::shared_ptr<std::byte[]> data,
                        size_t capacity,
                        const Shape& shape,
                        Dtype dtype,
                        const Device& device,
                        size_t offset,
                        const std::vector<int64_t>& strides)
{
    // The caller owns the buffer's layout, so the array starts writeable; the constructor validates
    // the stride rank, rejects negative strides and an unrepresentable span, and downgrades
    // writeability if the strides replicate an axis.
    std::vector<int64_t> resolvedStrides = strides.empty() ? details::computeContiguousStrides(shape) : strides;
    return details::ArrayStorageFactory::fromBuffer(
        std::move(data), capacity, shape, dtype, device, offset, resolvedStrides, true);
}

Array details::ArrayStorageFactory::fromBuffer(std::shared_ptr<std::byte[]> data,
                                               size_t capacity,
                                               const Shape& shape,
                                               Dtype dtype,
                                               const Device& device,
                                               size_t offset,
                                               const std::vector<int64_t>& strides,
                                               bool writeable)
{
    const size_t span = computeStorageSpanNbytes(shape, strides, dtype);
    if (offset > std::numeric_limits<size_t>::max() - span)
    {
        throw std::overflow_error("Array::fromBuffer(): byte offset and strided span overflow size_t");
    }
    if (offset > capacity || span > capacity - offset)
    {
        throw std::invalid_argument("Array::fromBuffer(): view extends beyond the declared allocation");
    }
    if (span != 0 && !data)
    {
        throw std::invalid_argument("Array::fromBuffer(): a non-empty view requires a non-null buffer");
    }
    validateCudaPointer(data, device);
    return Array(makeStorage(std::move(data), capacity, device), offset, shape, strides, dtype, device, writeable);
}

const std::shared_ptr<details::Storage>& details::ArrayStorageFactory::storageOf(const Array& array)
{
    return array.m_storage;
}

void details::ArrayStorageFactory::requireWriteable(const Array& array, const std::string& callerName)
{
    array._requireWriteable(callerName);
}

Array Array::toDevice(const Device& device, bool copy) const
{
    // Same device
    if (device == m_device)
    {
        if (copy)
        {
            return this->copy();
        }
        return *this;
    }

    // Different device
    if (device.isCuda() && !device.isAvailable())
    {
        throw isaacsim::common::exceptions::CudaRuntimeError("Array::toDevice()",
                                                             "device not available: " + device.toString(),
                                                             static_cast<int>(CUDA_ERROR_INVALID_DEVICE));
    }
    const Array source = this->contiguous();
    const size_t byteCount = this->nbytes();
    const std::vector<int64_t> denseStrides = details::computeContiguousStrides(m_shape);

    // CUDA -> CPU
    if (device.isCpu())
    {
        auto hostBuffer = std::shared_ptr<std::byte[]>(new std::byte[byteCount]());
        Array result(
            makeStorage(std::move(hostBuffer), byteCount, device), 0, m_shape, denseStrides, m_dtype, device, true);
        // The source is acquired on the operation stream so the copy is ordered after whatever
        // produced it. The destination is host memory and carries no CUDA event, so the transfer is
        // queued and then waited on here: the caller reads the result through a plain pointer.
        const std::uintptr_t stream = details::operationStream(m_device);
        ArrayReadAccess sourceAccess = acquireRead(source, stream);
        ArrayWriteAccess resultAccess = acquireWrite(result);
        {
            DeviceGuard sourceDeviceGuard(m_device);
            if (byteCount != 0)
            {
                auto& cudaRuntime = CudaRuntime::getInstance();
                cudaRuntime.cudaMemcpyAsync(
                    resultAccess.data(), sourceAccess.data(), byteCount, reinterpret_cast<CUstream>(stream));
                cudaRuntime.cudaStreamSynchronize(reinterpret_cast<CUstream>(stream));
            }
        }
        return result;
    }
    // CPU -> CUDA and CUDA -> CUDA (different ordinals) differ only in the transfer direction, which
    // the driver resolves from the pointers themselves.
    if (m_device.isCuda())
    {
        // A null stream is device-local. Resolve source readiness before switching devices so the
        // target-device copy cannot race a producer on the source device.
        source.m_storage->synchronizeReady();
    }
    auto storage = allocateCudaBuffer(device, byteCount, [](void*) {});
    Array result(makeStorage(std::move(storage), std::max<size_t>(byteCount, 1), device), 0, m_shape, denseStrides,
                 m_dtype, device, true);
    // The transfer is queued on the destination device's operation stream, which is also the stream
    // the result's producer event is recorded on.
    const std::uintptr_t stream = details::operationStream(device);
    ArrayReadAccess sourceAccess = acquireRead(source, m_device.isCuda() ? details::operationStream(m_device) : 0);
    ArrayWriteAccess resultAccess = acquireWrite(result, stream);
    if (byteCount != 0)
    {
        DeviceGuard destinationDeviceGuard(device);
        auto& cudaRuntime = CudaRuntime::getInstance();
        cudaRuntime.cudaMemcpyAsync(
            resultAccess.data(), sourceAccess.data(), byteCount, reinterpret_cast<CUstream>(stream));
        // Waited on rather than left in flight for two reasons: a host source is pageable, so its
        // buffer has to stay valid until the copy drains, and a peer copy is not ordered by an event
        // on the source device, so the retained source access cannot be released before it lands.
        cudaRuntime.cudaStreamSynchronize(reinterpret_cast<CUstream>(stream));
    }
    return result;
}

Array Array::toDtype(Dtype dtype, bool copy) const
{
    // Same dtype
    if (dtype == m_dtype)
    {
        // Copy data
        if (copy)
        {
            return this->copy();
        }
        return *this;
    }

    // Different dtype
    const Array source = this->contiguous();
    const size_t count = this->size();
    const std::vector<int64_t> denseStrides = details::computeContiguousStrides(m_shape);

    std::shared_ptr<std::byte[]> newData;
    if (m_device.isCuda())
    {
        newData = allocateCudaBuffer(m_device, count * dtype.size(), [](void*) {});
    }
    else
    {
        newData = std::shared_ptr<std::byte[]>(new std::byte[count * dtype.size()]());
    }
    const size_t resultNbytes = count * dtype.size();
    const size_t capacity = m_device.isCuda() ? std::max<size_t>(resultNbytes, 1) : resultNbytes;
    Array result(makeStorage(std::move(newData), capacity, m_device), 0, m_shape, denseStrides, dtype, m_device, true);
    // Device-resident result: queue the cast on the operation stream and let the write access
    // publish its producer event rather than blocking the host on completion.
    const std::uintptr_t stream = details::operationStream(m_device);
    ArrayReadAccess sourceAccess = acquireRead(source, stream);
    ArrayWriteAccess resultAccess = acquireWrite(result, stream);
    if (m_device.isCuda())
    {
        if (count != 0)
        {
            DeviceGuard deviceGuard(m_device);
            details::CudaKernel::getInstance().cast(sourceAccess.data(), resultAccess.data(), count, m_dtype.kind(),
                                                    dtype.kind(), reinterpret_cast<CUstream>(stream));
        }
    }
    else
    {
        const std::byte* sourceData = static_cast<const std::byte*>(sourceAccess.data());
        std::byte* destinationData = static_cast<std::byte*>(resultAccess.data());
        const size_t sourceElementSize = m_dtype.size();
        const size_t destinationElementSize = dtype.size();
        for (size_t i = 0; i < count; ++i)
        {
            const std::byte* sourceElement = sourceData + i * sourceElementSize;
            std::byte* destinationElement = destinationData + i * destinationElementSize;
            if (m_dtype.isFloating())
            {
                result._setItem(destinationElement, _getItem<double>(sourceElement));
            }
            else if (m_dtype.isSigned())
            {
                result._setItem(destinationElement, _getItem<int64_t>(sourceElement));
            }
            else
            {
                result._setItem(destinationElement, _getItem<uint64_t>(sourceElement));
            }
        }
    }
    return result;
}

Array Array::broadcastTo(const Shape& shape, bool copy) const
{
    return broadcastView(shape, copy || shape != m_shape);
}

Array Array::broadcastView(const Shape& shape, bool copy) const
{
    if (!m_shape.canBroadcastTo(shape))
    {
        throw std::invalid_argument("Array::broadcastTo(): cannot broadcast array of shape " + m_shape.toString() +
                                    " to shape " + shape.toString());
    }

    const size_t sourceNdim = this->ndim();
    const size_t destinationNdim = shape.ndim();
    const size_t sizeOffset = destinationNdim - sourceNdim;

    std::vector<int64_t> destinationStrides(destinationNdim, 0);
    for (size_t axis = 0; axis < sourceNdim; ++axis)
    {
        const size_t destinationAxis = sizeOffset + axis;
        if (m_shape[static_cast<int64_t>(axis)] == shape[static_cast<int64_t>(destinationAxis)])
        {
            destinationStrides[destinationAxis] = m_strides[axis];
        }
    }

    // A replicated axis carries a zero stride, which makes the view read-only.
    Array view = this->_viewWithLayout(shape, std::move(destinationStrides));
    if (copy)
    {
        return view.copy();
    }
    return view;
}

Array Array::at(int64_t index)
{
    if (this->ndim() == 0)
    {
        throw std::out_of_range("Array::at() called on a 0-D array");
    }
    if ((index < 0 && index < -m_shape[0]) || index >= m_shape[0])
    {
        throw std::out_of_range("Array::at(): index " + std::to_string(index) + " out of range for axis 0 with size " +
                                std::to_string(m_shape[0]));
    }
    if (index < 0)
    {
        index += m_shape[0];
    }
    // For a non-empty array the constructor's span invariant already bounds this arithmetic, but an
    // empty array reaches no element, so its span is 0 and its strides go unvalidated -- shape
    // {3, 0} with strides {SIZE_MAX / 4, 1} is constructible. Indexing axis 0 of such an array still
    // scales by that stride, so the products are checked explicitly.
    const size_t resolvedIndex = static_cast<size_t>(index);
    const size_t elementStride = static_cast<size_t>(m_strides[0]);
    if (resolvedIndex != 0 && elementStride > std::numeric_limits<size_t>::max() / resolvedIndex)
    {
        throw std::overflow_error("Array::at(): element offset overflows size_t");
    }
    const size_t elementOffset = resolvedIndex * elementStride;
    if (elementOffset > (std::numeric_limits<size_t>::max() - m_offset) / m_dtype.size())
    {
        throw std::overflow_error("Array::at(): byte offset overflows size_t");
    }
    const size_t newOffset = m_offset + elementOffset * m_dtype.size();
    const std::vector<int64_t> dimensions = m_shape.shape();
    return _viewWithLayout(Shape(std::vector<int64_t>(dimensions.begin() + 1, dimensions.end())),
                           std::vector<int64_t>(m_strides.begin() + 1, m_strides.end()), newOffset);
}

void Array::set(const details::SupportedInputSpecification& value)
{
    this->set(Array(value, m_dtype, m_device));
}

void Array::set(const Array& other)
{
    this->_requireWriteable("Array::set()");

    // broadcastTo() runs before the empty-destination shortcut so that an incompatible source is
    // still rejected when there is nothing to write.
    Array prepared = other.toDtype(m_dtype).toDevice(m_device).broadcastView(m_shape);
    const size_t totalElements = this->size();
    if (totalElements == 0)
    {
        return;
    }

    // `prepared` may be a read-only broadcast view, so every read of it goes through the const
    // data() overload; only the destination is written.
    if (std::as_const(prepared).data() == std::as_const(*this).data() && prepared.strides() == m_strides)
    {
        // Self-assignment of the identical view: nothing to do, and no copy to pay for.
        return;
    }
    if (haveSharedStorage(prepared, *this))
    {
        // Any other overlap would read bytes this call has already overwritten.
        prepared = prepared.copy();
    }

    const bool dense = this->isContiguous() && prepared.isContiguous();
    if (m_device.isCuda())
    {
        DeviceGuard deviceGuard(m_device);
        // Both operands are already on this device (toDevice() above moved the source), so the write
        // only needs stream ordering; releasing the access publishes the new producer event.
        const std::uintptr_t stream = details::operationStream(m_device);
        ArrayReadAccess sourceAccess = acquireRead(prepared, stream);
        ArrayWriteAccess destinationAccess = acquireWrite(*this, stream);
        if (dense)
        {
            CudaRuntime::getInstance().cudaMemcpyAsync(
                destinationAccess.data(), sourceAccess.data(), this->nbytes(), reinterpret_cast<CUstream>(stream));
        }
        else
        {
            details::CudaKernel::getInstance().copyStrided(
                sourceAccess.data(), destinationAccess.data(), prepared.strides(), m_strides, m_shape.shape(),
                m_dtype.size(), totalElements, reinterpret_cast<CUstream>(stream));
        }
        return;
    }
    ArrayReadAccess sourceAccess = acquireRead(prepared);
    ArrayWriteAccess destinationAccess = acquireWrite(*this);
    if (dense)
    {
        std::memcpy(destinationAccess.data(), sourceAccess.data(), this->nbytes());
    }
    else
    {
        copyStridedCpu(static_cast<const std::byte*>(sourceAccess.data()), prepared.strides(),
                       static_cast<std::byte*>(destinationAccess.data()), m_strides, m_shape, m_dtype.size(),
                       totalElements);
    }
}

std::string Array::toString() const
{
    const std::string data = m_device.isCuda() ? this->toDevice(Device::Cpu())._formatData() : _formatData();
    return "Array(" + data + ", shape=" + m_shape.toString() + ", dtype='" + m_dtype.toString() + "', device='" +
           m_device.toString() + "')";
}

std::shared_ptr<std::byte[]> Array::_buildCpuBuffer(const details::SupportedInputSpecification& value)
{
    return std::visit(
        [this](const auto& v) -> std::shared_ptr<std::byte[]>
        {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_arithmetic_v<T>)
            {
                m_shape = Shape();
                auto buffer = std::shared_ptr<std::byte[]>(new std::byte[m_dtype.size()]());
                _setItem(buffer.get(), v);
                return buffer;
            }
            else if constexpr (std::is_arithmetic_v<typename T::value_type>)
            {
                m_shape = Shape(v.size());
                auto buffer = std::shared_ptr<std::byte[]>(new std::byte[v.size() * m_dtype.size()]());
                for (size_t i = 0; i < v.size(); ++i)
                {
                    if constexpr (std::is_same_v<typename T::value_type, bool>)
                    {
                        bool b = v[i];
                        _setItem(buffer.get() + i * m_dtype.size(), b);
                    }
                    else
                    {
                        _setItem(buffer.get() + i * m_dtype.size(), v[i]);
                    }
                }
                return buffer;
            }
            else
            {
                using U = typename T::value_type::value_type;
                size_t rows = v.size();
                size_t cols = rows ? v[0].size() : 0;
                for (size_t i = 1; i < rows; ++i)
                {
                    if (v[i].size() != cols)
                        throw std::invalid_argument("The input has an inhomogeneous shape");
                }
                m_shape = Shape(std::vector<uint64_t>{ rows, cols });
                auto buffer = std::shared_ptr<std::byte[]>(new std::byte[this->size() * m_dtype.size()]());
                for (size_t i = 0; i < rows; ++i)
                {
                    for (size_t j = 0; j < cols; ++j)
                    {
                        size_t offset = (i * cols + j) * m_dtype.size();
                        if constexpr (std::is_same_v<U, bool>)
                        {
                            bool b = v[i][j];
                            _setItem(buffer.get() + offset, b);
                        }
                        else
                        {
                            _setItem(buffer.get() + offset, v[i][j]);
                        }
                    }
                }
                return buffer;
            }
        },
        value);
}

Array::Array(std::shared_ptr<details::Storage> storage,
             size_t offset,
             Shape shape,
             std::vector<int64_t> strides,
             Dtype dtype,
             Device device,
             bool writeable)
    : m_shape(std::move(shape)),
      m_strides(std::move(strides)),
      m_writeable(writeable),
      m_dtype(dtype),
      m_device(device),
      m_offset(offset),
      m_storage(std::move(storage))
{
    if (!m_storage)
    {
        throw std::invalid_argument("Array: storage must not be null");
    }
    if (m_storage->device() != m_device)
    {
        throw std::invalid_argument("Array: storage and view devices must match");
    }
    if (m_strides.size() != this->ndim())
    {
        throw std::invalid_argument("Array: strides must have the same rank as the shape");
    }
    if (std::any_of(m_strides.begin(), m_strides.end(), [](int64_t stride) { return stride < 0; }))
    {
        throw std::invalid_argument("Array: negative strides are not supported");
    }
    // A layout that replicates elements is read-only whatever the caller asked for; the rank check
    // above has to pass first so that the shape and strides can be indexed together.
    if (hasReplicatedElements(m_shape, m_strides))
    {
        m_writeable = false;
    }
    // Establishes the invariant that offset + span is representable, which at(), clone() and
    // fromBuffer() all rely on instead of re-deriving it.
    const size_t span = details::computeStorageSpanNbytes(m_shape, m_strides, m_dtype);
    if (m_offset > std::numeric_limits<size_t>::max() - span)
    {
        throw std::overflow_error("Array: byte offset and strided span overflow size_t");
    }
    if (m_offset > m_storage->capacity() || span > m_storage->capacity() - m_offset)
    {
        throw std::invalid_argument("Array: view extends beyond the retained buffer");
    }
    if (span != 0 && !m_storage->pointer())
    {
        throw std::invalid_argument("Array: a non-empty view requires a non-null buffer");
    }
}

size_t Array::_storageSpanNbytes() const
{
    return details::computeStorageSpanNbytes(m_shape, m_strides, m_dtype);
}

std::string Array::_formatItem() const
{
    // TODO: use std::format once C++20 is supported
    switch (m_dtype.kind())
    {
    case Dtype::Kind::eBool:
        return item<bool>() ? "True" : "False";
    case Dtype::Kind::eInt8:
        return std::to_string(static_cast<int>(item<int8_t>()));
    case Dtype::Kind::eInt16:
        return std::to_string(item<int16_t>());
    case Dtype::Kind::eInt32:
        return std::to_string(item<int32_t>());
    case Dtype::Kind::eInt64:
        return std::to_string(item<int64_t>());
    case Dtype::Kind::eUInt8:
        return std::to_string(static_cast<unsigned>(item<uint8_t>()));
    case Dtype::Kind::eUInt16:
        return std::to_string(item<uint16_t>());
    case Dtype::Kind::eUInt32:
        return std::to_string(item<uint32_t>());
    case Dtype::Kind::eUInt64:
        return std::to_string(item<uint64_t>());
    case Dtype::Kind::eFloat32:
        return std::to_string(item<float>());
    case Dtype::Kind::eFloat64:
        return std::to_string(item<double>());
    default:
        return "?";
    }
}

std::string Array::_formatData() const
{
    if (this->ndim() == 0)
    {
        return _formatItem();
    }
    if (this->ndim() == 1)
    {
        std::string s = "[";
        size_t count = static_cast<size_t>(m_shape[0]);
        for (size_t i = 0; i < count; ++i)
        {
            if (i)
                s += ", ";
            s += const_cast<Array*>(this)->at(static_cast<int64_t>(i))._formatItem();
        }
        return s + "]";
    }
    if (this->ndim() == 2)
    {
        size_t rows = static_cast<size_t>(m_shape[0]);
        std::string s = "[";
        for (size_t i = 0; i < rows; ++i)
        {
            if (i)
                s += ", ";
            s += const_cast<Array*>(this)->at(static_cast<int64_t>(i))._formatData();
        }
        return s + "]";
    }
    return "...";
}

} // namespace array
} // namespace common
} // namespace isaacsim
