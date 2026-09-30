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

#include "details/CudaKernel.hpp"
#include "details/CudaRuntime.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>

#include <algorithm>
#include <stdexcept>
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

} // namespace details

Array::Array(const details::SupportedInputSpecification& value, const std::optional<Dtype>& dtype, const Device& device)
    : m_dtype(dtype.has_value() ? *dtype : details::inferDtype(value)), m_device(device), m_offset(0)
{
    // CUDA
    if (device.isCuda())
    {
        // Validate CUDA runtime and device
        auto& cudaRuntime = CudaRuntime::getInstance();
        if (!device.isAvailable())
        {
            throw std::runtime_error("Array: CUDA device not available: " + device.toString());
        }
        // Build CPU buffer
        auto cpuBuffer = _buildCpuBuffer(value);
        // Allocate device memory and copy data from CPU to device
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(device);
            size_t byteCount = this->nbytes();
            cudaRuntime.cudaMalloc(&devicePointer, byteCount);
            try
            {
                cudaRuntime.cudaMemcpy(devicePointer, cpuBuffer.get(), byteCount, cudaMemcpyHostToDevice);
            }
            catch (...)
            {
                cudaRuntime.cudaFree(devicePointer, false);
                throw;
            }
        }
        // Create CUDA storage
        auto deleter = [device](void* pointer)
        {
            DeviceGuard deleterDeviceGuard(device);
            CudaRuntime::getInstance().cudaFree(pointer);
        };
        m_storage = details::CudaStorage{ std::shared_ptr<void>(devicePointer, deleter) };
    }
    // CPU
    else
    {
        m_storage = details::CpuStorage{ _buildCpuBuffer(value) };
    }
}

const Shape& Array::shape() const
{
    return m_shape;
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

void* Array::data()
{
    return const_cast<void*>(std::as_const(*this).data());
}

const void* Array::data() const
{
    // CUDA
    if (m_device.isCuda())
    {
        const auto* data = static_cast<const std::byte*>(std::get<details::CudaStorage>(m_storage).data.get());
        return data ? data + m_offset : nullptr;
    }
    // CPU
    else
    {
        const auto* data = _copyToCpuBuffer().get();
        return data ? data + m_offset : nullptr;
    }
}

std::shared_ptr<std::byte[]> Array::buffer() const
{
    // CUDA: array shares ownership of the device allocation (aliasing constructor): the
    // deleter stays the one that frees it on the owning device, and the stored pointer is a
    // device pointer to the start of the allocation.
    if (m_device.isCuda())
    {
        const auto& deviceData = std::get<details::CudaStorage>(m_storage).data;
        return std::shared_ptr<std::byte[]>(deviceData, static_cast<std::byte*>(deviceData.get()));
    }
    // CPU
    else
    {
        return _copyToCpuBuffer();
    }
}

Array Array::reshape(const Shape& shape) const
{
    // CUDA
    if (m_device.isCuda())
    {
        return Array(details::CudaStorage{ std::get<details::CudaStorage>(m_storage).data }, m_offset,
                     m_shape.resolve(shape), m_dtype, m_device);
    }
    // CPU
    else
    {
        return Array(details::CpuStorage{ _copyToCpuBuffer() }, m_offset, m_shape.resolve(shape), m_dtype, m_device);
    }
}

Array Array::flatten() const
{
    return this->reshape(Shape({ static_cast<int64_t>(this->size()) }));
}

Array Array::copy() const
{
    // Only the visible elements are duplicated, so any bytes preceding element 0 are dropped and
    // the offset is reset to 0 (unlike clone(), which preserves the whole allocation).
    const size_t byteCount = this->nbytes();
    // CUDA
    if (m_device.isCuda())
    {
        auto& cudaRuntime = CudaRuntime::getInstance();
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(m_device);
            cudaRuntime.cudaMalloc(&devicePointer, byteCount);
            try
            {
                cudaRuntime.cudaMemcpy(devicePointer, this->data(), byteCount, cudaMemcpyDeviceToDevice);
            }
            catch (...)
            {
                cudaRuntime.cudaFree(devicePointer, false);
                throw;
            }
        }
        auto deleter = [device = m_device](void* pointer)
        {
            DeviceGuard deleterDeviceGuard(device);
            CudaRuntime::getInstance().cudaFree(pointer);
        };
        return Array(
            details::CudaStorage{ std::shared_ptr<void>(devicePointer, deleter) }, 0, m_shape, m_dtype, m_device);
    }
    // CPU
    else
    {
        auto buffer = std::shared_ptr<std::byte[]>(new std::byte[byteCount]);
        std::memcpy(buffer.get(), this->data(), byteCount);
        return Array(details::CpuStorage{ std::move(buffer) }, 0, m_shape, m_dtype, m_device);
    }
}

Array Array::clone() const
{
    // The whole allocation is duplicated, including any bytes preceding element 0, so the offset
    // is preserved (unlike copy(), which compacts the visible elements to offset 0).
    size_t totalSize = m_offset + this->nbytes();
    // CUDA
    if (m_device.isCuda())
    {
        auto& cudaRuntime = CudaRuntime::getInstance();
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(m_device);
            cudaRuntime.cudaMalloc(&devicePointer, totalSize);
            try
            {
                const void* sourceDevicePointer = std::get<details::CudaStorage>(m_storage).data.get();
                cudaRuntime.cudaMemcpy(devicePointer, sourceDevicePointer, totalSize, cudaMemcpyDeviceToDevice);
            }
            catch (...)
            {
                cudaRuntime.cudaFree(devicePointer, false);
                throw;
            }
        }
        auto deleter = [device = m_device](void* pointer)
        {
            DeviceGuard deleterDeviceGuard(device);
            CudaRuntime::getInstance().cudaFree(pointer);
        };
        return Array(details::CudaStorage{ std::shared_ptr<void>(devicePointer, deleter) }, m_offset, m_shape, m_dtype,
                     m_device);
    }
    // CPU
    else
    {
        auto newData = std::shared_ptr<std::byte[]>(new std::byte[totalSize]());
        std::memcpy(newData.get(), _copyToCpuBuffer().get(), totalSize);
        return Array(details::CpuStorage{ std::move(newData) }, m_offset, m_shape, m_dtype, m_device);
    }
}

Array Array::fromBuffer(std::shared_ptr<std::byte[]> data, const Shape& shape, Dtype dtype, const Device& device, size_t offset)
{
    // CUDA: `data` holds a device pointer (as returned by buffer() for a CUDA array):
    // adopt it as device storage via the aliasing constructor, which keeps the caller's deleter and
    // so preserves whatever owns the allocation.
    if (device.isCuda())
    {
        std::shared_ptr<void> deviceData(data, data.get());
        return Array(details::CudaStorage{ std::move(deviceData) }, offset, shape, dtype, device);
    }
    // CPU
    else
    {
        return Array(details::CpuStorage{ std::move(data) }, offset, shape, dtype, device);
    }
}

Array Array::toDevice(const Device& device, bool copy) const
{
    // Same device
    if (device == m_device)
    {
        // Copy data
        if (copy)
        {
            return this->copy();
        }
        // Share data
        if (m_device.isCuda())
        {
            return Array(details::CudaStorage{ std::get<details::CudaStorage>(m_storage).data }, m_offset, m_shape,
                         m_dtype, m_device);
        }
        return Array(details::CpuStorage{ _copyToCpuBuffer() }, m_offset, m_shape, m_dtype, m_device);
    }

    // Different device
    const size_t byteCount = this->nbytes();
    auto& cudaRuntime = CudaRuntime::getInstance();
    if (device.isCuda() && !device.isAvailable())
    {
        throw isaacsim::common::exceptions::CudaRuntimeError(
            "Array::toDevice()", "device not available: " + device.toString(), static_cast<int>(cudaErrorInvalidDevice));
    }
    // - CUDA -> CPU
    if (m_device.isCuda() && device.isCpu())
    {
        auto hostBuffer = std::shared_ptr<std::byte[]>(new std::byte[byteCount]());
        {
            DeviceGuard sourceDeviceGuard(m_device);
            cudaRuntime.cudaMemcpy(hostBuffer.get(), this->data(), byteCount, cudaMemcpyDeviceToHost);
        }
        return Array(details::CpuStorage{ std::move(hostBuffer) }, 0, m_shape, m_dtype, device);
    }
    // - CPU -> CUDA
    else if (m_device.isCpu() && device.isCuda())
    {
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(device);
            cudaRuntime.cudaMalloc(&devicePointer, byteCount);
            try
            {
                cudaRuntime.cudaMemcpy(devicePointer, this->data(), byteCount, cudaMemcpyHostToDevice);
            }
            catch (...)
            {
                cudaRuntime.cudaFree(devicePointer, false);
                throw;
            }
        }
        auto deleter = [device](void* pointer)
        {
            DeviceGuard deleterDeviceGuard(device);
            CudaRuntime::getInstance().cudaFree(pointer);
        };
        return Array(details::CudaStorage{ std::shared_ptr<void>(devicePointer, deleter) }, 0, m_shape, m_dtype, device);
    }
    // - CUDA -> CUDA (different ordinals)
    else
    {
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(device);
            cudaRuntime.cudaMalloc(&devicePointer, byteCount);
            try
            {
                cudaRuntime.cudaMemcpy(devicePointer, this->data(), byteCount, cudaMemcpyDeviceToDevice);
            }
            catch (...)
            {
                cudaRuntime.cudaFree(devicePointer, false);
                throw;
            }
        }
        auto deleter = [device](void* pointer)
        {
            DeviceGuard deleterDeviceGuard(device);
            CudaRuntime::getInstance().cudaFree(pointer);
        };
        return Array(details::CudaStorage{ std::shared_ptr<void>(devicePointer, deleter) }, 0, m_shape, m_dtype, device);
    }
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
        // Share data
        if (m_device.isCuda())
        {
            return Array(details::CudaStorage{ std::get<details::CudaStorage>(m_storage).data }, m_offset, m_shape,
                         m_dtype, m_device);
        }
        return Array(details::CpuStorage{ _copyToCpuBuffer() }, m_offset, m_shape, m_dtype, m_device);
    }

    // Different dtype
    const size_t count = this->size();
    // - CUDA
    if (m_device.isCuda())
    {
        auto& cudaRuntime = CudaRuntime::getInstance();
        auto& cudaKernel = details::CudaKernel::getInstance();
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(m_device);
            cudaRuntime.cudaMalloc(&devicePointer, count * dtype.size());
            try
            {
                cudaKernel.cast(this->data(), devicePointer, count, m_dtype.kind(), dtype.kind());
            }
            catch (...)
            {
                cudaRuntime.cudaFree(devicePointer, false);
                throw;
            }
        }
        auto deleter = [device = m_device](void* pointer)
        {
            DeviceGuard deleterDeviceGuard(device);
            CudaRuntime::getInstance().cudaFree(pointer);
        };
        return Array(details::CudaStorage{ std::shared_ptr<void>(devicePointer, deleter) }, 0, m_shape, dtype, m_device);
    }
    // - CPU
    else
    {
        auto newData = std::shared_ptr<std::byte[]>(new std::byte[count * dtype.size()]());
        Array result(details::CpuStorage{ std::move(newData) }, 0, m_shape, dtype, m_device);

        const std::byte* sourceData = static_cast<const std::byte*>(this->data());
        std::byte* destinationData = static_cast<std::byte*>(result.data());

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
        return result;
    }
}

Array Array::broadcastTo(const Shape& shape, bool copy) const
{
    if (!m_shape.canBroadcastTo(shape))
    {
        throw std::invalid_argument("Array::broadcastTo(): cannot broadcast array of shape " + m_shape.toString() +
                                    " to shape " + shape.toString());
    }

    // Same shape
    if (m_shape == shape)
    {
        // Copy data
        if (copy)
        {
            return this->copy();
        }
        // Share data
        if (m_device.isCuda())
        {
            return Array(details::CudaStorage{ std::get<details::CudaStorage>(m_storage).data }, m_offset, m_shape,
                         m_dtype, m_device);
        }
        return Array(details::CpuStorage{ _copyToCpuBuffer() }, m_offset, m_shape, m_dtype, m_device);
    }

    const size_t sourceNdim = this->ndim();
    const size_t destinationNdim = shape.ndim();
    const size_t sizeOffset = destinationNdim - sourceNdim;

    // Source strides (in elements) aligned to the destination axes; 0 marks a broadcast axis
    std::vector<int64_t> sourceStrides(destinationNdim, 0);
    int64_t stride = 1;
    for (size_t i = 0; i < sourceNdim; ++i)
    {
        size_t axis = sourceNdim - 1 - i;
        if (m_shape[static_cast<int64_t>(axis)] != 1)
        {
            sourceStrides[sizeOffset + axis] = stride;
        }
        stride *= m_shape[static_cast<int64_t>(axis)];
    }

    const std::vector<int64_t> destinationShape = shape.shape();
    const size_t elementSize = m_dtype.size();
    const size_t totalElements = shape.size();

    // CUDA
    if (m_device.isCuda())
    {
        auto& cudaRuntime = CudaRuntime::getInstance();
        auto& cudaKernel = details::CudaKernel::getInstance();
        void* devicePointer = nullptr;
        {
            DeviceGuard deviceGuard(m_device);
            cudaRuntime.cudaMalloc(&devicePointer, totalElements * elementSize);
            try
            {
                cudaKernel.broadcastTo(
                    this->data(), devicePointer, sourceStrides, destinationShape, elementSize, totalElements);
            }
            catch (...)
            {
                cudaRuntime.cudaFree(devicePointer, false);
                throw;
            }
        }
        auto deleter = [device = m_device](void* pointer)
        {
            DeviceGuard deleterDeviceGuard(device);
            CudaRuntime::getInstance().cudaFree(pointer);
        };
        return Array(details::CudaStorage{ std::shared_ptr<void>(devicePointer, deleter) }, 0, shape, m_dtype, m_device);
    }
    // CPU
    else
    {
        auto newData = std::shared_ptr<std::byte[]>(new std::byte[totalElements * elementSize]());
        Array result(details::CpuStorage{ std::move(newData) }, 0, shape, m_dtype, m_device);

        const std::byte* sourceData = static_cast<const std::byte*>(this->data());
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
                if (++index[axis] < static_cast<size_t>(destinationShape[axis]))
                {
                    break;
                }
                index[axis] = 0;
            }
        }
        return result;
    }
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
    std::vector<int64_t> newShape;
    newShape.reserve(this->ndim() - 1);
    for (size_t i = 1; i < this->ndim(); ++i)
    {
        newShape.push_back(m_shape[i]);
    }
    size_t sliceSize = 1;
    for (size_t i = 1; i < this->ndim(); ++i)
    {
        sliceSize *= static_cast<size_t>(m_shape[i]);
    }
    size_t newOffset = m_offset + static_cast<size_t>(index) * sliceSize * m_dtype.size();

    // CUDA
    if (m_device.isCuda())
    {
        return Array(details::CudaStorage{ std::get<details::CudaStorage>(m_storage).data }, newOffset, Shape(newShape),
                     m_dtype, m_device);
    }
    // CPU
    else
    {
        return Array(details::CpuStorage{ _copyToCpuBuffer() }, newOffset, Shape(newShape), m_dtype, m_device);
    }
}

void Array::set(const details::SupportedInputSpecification& value)
{
    this->set(Array(value, m_dtype, m_device));
}

void Array::set(const Array& other)
{
    Array prepared = other.toDtype(m_dtype).toDevice(m_device).broadcastTo(m_shape);
    const size_t n = this->nbytes();

    // CUDA
    if (m_device.isCuda())
    {
        DeviceGuard deviceGuard(m_device);
        CudaRuntime::getInstance().cudaMemcpy(this->data(), prepared.data(), n, cudaMemcpyDeviceToDevice);
        return;
    }
    // CPU
    else
    {
        if (prepared._copyToCpuBuffer() == _copyToCpuBuffer())
        {
            std::memmove(this->data(), prepared.data(), n);
        }
        else
        {
            std::memcpy(this->data(), prepared.data(), n);
        }
    }
}

std::string Array::toString() const
{
    const std::string data = m_device.isCuda() ? this->toDevice(Device::Cpu())._formatData() : _formatData();
    return "Array(" + data + ", shape=" + m_shape.toString() + ", dtype='" + m_dtype.toString() + "', device='" +
           m_device.toString() + "')";
}

std::shared_ptr<std::byte[]> Array::_copyToCpuBuffer() const
{
    return std::get<details::CpuStorage>(m_storage).data;
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

Array::Array(std::variant<details::CpuStorage, details::CudaStorage> storage,
             size_t offset,
             Shape shape,
             Dtype dtype,
             Device device)
    : m_shape(std::move(shape)), m_dtype(dtype), m_device(device), m_offset(offset), m_storage(std::move(storage))
{
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
