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

#include "isaacsim/common/array/ManagedAccess.hpp"

#include "details/ArrayUtils.hpp"
#include "details/CudaRuntime.hpp"
#include "details/Storage.hpp"
#include "isaacsim/common/array/Array.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace
{

// Rejects a stream that belongs to a different device than the array, and a non-zero stream on a
// CPU array. A CUDA stream is only meaningful together with the device that created it, so mixing
// the two would order the handshake against work that never runs.
//
// `movableScope` additionally rejects `CU_STREAM_PER_THREAD`, which resolves to a different physical
// stream on every calling thread. An access scope may be moved and released on a thread other than
// the one that acquired it, and the completion event would then be recorded on the releasing
// thread's stream: it completes without ever covering the acquiring thread's work, and the
// allocation retires underneath a pending read or write. Two threads reading through the handle
// also collide in the per-stream completion map, which is keyed by the stream value, so one
// reader's event silently replaces the other's and the next writer never waits for it. The handle
// cannot be resolved to a concrete stream at acquisition -- CUDA exposes no such API -- and release
// runs from a destructor where throwing is not an option, so admission is the only place to refuse
// it. Immediate republication (recordCudaWrite()) records on the calling thread and is unaffected.
void validateCudaStream(const Device& device, std::uintptr_t stream, const char* callerName, bool movableScope)
{
    if (!device.isCuda())
    {
        if (stream != 0)
        {
            throw std::invalid_argument(std::string(callerName) + ": CPU access requires stream 0");
        }
        return;
    }

    if (movableScope && stream == reinterpret_cast<std::uintptr_t>(CU_STREAM_PER_THREAD))
    {
        throw std::invalid_argument(std::string(callerName) +
                                    ": CU_STREAM_PER_THREAD is not supported for scoped access, because the scope "
                                    "may be released on a thread other than the one that acquired it; pass an "
                                    "explicit stream, or 0 for the legacy default stream");
    }

    DeviceGuard deviceGuard(device);
    int streamDevice = -1;
    CudaRuntime::getInstance().cudaStreamGetDevice(reinterpret_cast<CUstream>(stream), &streamDevice);
    if (streamDevice != device.ordinal())
    {
        throw std::invalid_argument(std::string(callerName) + ": CUDA stream belongs to device " +
                                    std::to_string(streamDevice) + ", not " + device.toString());
    }
}

} // namespace

ArrayReadAccess::ArrayReadAccess(
    std::shared_ptr<details::Storage> storage, const void* data, size_t nbytes, Device device, std::uintptr_t cudaStream)
    : m_storage(std::move(storage)),
      m_data(data),
      m_nbytes(nbytes),
      m_device(device),
      m_cudaStream(cudaStream),
      m_owner(std::this_thread::get_id())
{
}

ArrayReadAccess::~ArrayReadAccess()
{
    _release(false);
}

ArrayReadAccess::ArrayReadAccess(ArrayReadAccess&& other) noexcept
    : m_storage(std::move(other.m_storage)),
      m_data(std::exchange(other.m_data, nullptr)),
      m_nbytes(std::exchange(other.m_nbytes, 0)),
      m_device(other.m_device),
      m_cudaStream(std::exchange(other.m_cudaStream, 0)),
      m_owner(other.m_owner)
{
}

ArrayReadAccess& ArrayReadAccess::operator=(ArrayReadAccess&& other) noexcept
{
    if (this != &other)
    {
        _release(false);
        m_storage = std::move(other.m_storage);
        m_data = std::exchange(other.m_data, nullptr);
        m_nbytes = std::exchange(other.m_nbytes, 0);
        m_device = other.m_device;
        m_cudaStream = std::exchange(other.m_cudaStream, 0);
        m_owner = other.m_owner;
    }
    return *this;
}

ArrayReadAccess::operator bool() const noexcept
{
    return static_cast<bool>(m_storage);
}

const void* ArrayReadAccess::data() const noexcept
{
    return m_data;
}

size_t ArrayReadAccess::nbytes() const noexcept
{
    return m_nbytes;
}

void ArrayReadAccess::release()
{
    _release(true);
}

void ArrayReadAccess::_release(bool throwOnError)
{
    if (!m_storage)
    {
        return;
    }

    std::exception_ptr error;
    try
    {
        m_storage->endRead(m_cudaStream, throwOnError, m_owner);
    }
    catch (...)
    {
        error = std::current_exception();
    }

    m_storage.reset();
    m_data = nullptr;
    m_nbytes = 0;
    m_cudaStream = 0;
    if (error)
    {
        std::rethrow_exception(error);
    }
}

ArrayWriteAccess::ArrayWriteAccess(
    std::shared_ptr<details::Storage> storage, void* data, size_t nbytes, Device device, std::uintptr_t cudaStream)
    : m_storage(std::move(storage)),
      m_data(data),
      m_nbytes(nbytes),
      m_device(device),
      m_cudaStream(cudaStream),
      m_owner(std::this_thread::get_id())
{
}

ArrayWriteAccess::~ArrayWriteAccess()
{
    _release(false);
}

ArrayWriteAccess::ArrayWriteAccess(ArrayWriteAccess&& other) noexcept
    : m_storage(std::move(other.m_storage)),
      m_data(std::exchange(other.m_data, nullptr)),
      m_nbytes(std::exchange(other.m_nbytes, 0)),
      m_device(other.m_device),
      m_cudaStream(std::exchange(other.m_cudaStream, 0)),
      m_owner(other.m_owner)
{
}

ArrayWriteAccess& ArrayWriteAccess::operator=(ArrayWriteAccess&& other) noexcept
{
    if (this != &other)
    {
        _release(false);
        m_storage = std::move(other.m_storage);
        m_data = std::exchange(other.m_data, nullptr);
        m_nbytes = std::exchange(other.m_nbytes, 0);
        m_device = other.m_device;
        m_cudaStream = std::exchange(other.m_cudaStream, 0);
        m_owner = other.m_owner;
    }
    return *this;
}

ArrayWriteAccess::operator bool() const noexcept
{
    return static_cast<bool>(m_storage);
}

void* ArrayWriteAccess::data() const noexcept
{
    return m_data;
}

size_t ArrayWriteAccess::nbytes() const noexcept
{
    return m_nbytes;
}

void ArrayWriteAccess::release()
{
    _release(true);
}

void ArrayWriteAccess::_release(bool throwOnError)
{
    if (!m_storage)
    {
        return;
    }

    std::exception_ptr error;
    try
    {
        m_storage->endWrite(m_cudaStream, throwOnError, m_owner);
    }
    catch (...)
    {
        error = std::current_exception();
    }

    m_storage.reset();
    m_data = nullptr;
    m_nbytes = 0;
    m_cudaStream = 0;
    if (error)
    {
        std::rethrow_exception(error);
    }
}

ArrayReadAccess acquireRead(const Array& array, std::uintptr_t cudaStream)
{
    const Device device = array.device();
    validateCudaStream(device, cudaStream, "acquireRead()", /*movableScope=*/true);
    const std::shared_ptr<details::Storage>& storage = details::ArrayStorageFactory::storageOf(array);
    storage->beginRead(cudaStream);
    return ArrayReadAccess(storage, array.data(), array.nbytes(), device, cudaStream);
}

ArrayWriteAccess acquireWrite(Array& array, std::uintptr_t cudaStream)
{
    details::ArrayStorageFactory::requireWriteable(array, "acquireWrite()");
    const Device device = array.device();
    validateCudaStream(device, cudaStream, "acquireWrite()", /*movableScope=*/true);
    const std::shared_ptr<details::Storage>& storage = details::ArrayStorageFactory::storageOf(array);
    storage->beginWrite(cudaStream);
    return ArrayWriteAccess(storage, array.data(), array.nbytes(), device, cudaStream);
}

std::optional<ArrayWriteAccess> tryAcquireWrite(Array& array, std::uintptr_t cudaStream)
{
    details::ArrayStorageFactory::requireWriteable(array, "tryAcquireWrite()");
    const Device device = array.device();
    validateCudaStream(device, cudaStream, "tryAcquireWrite()", /*movableScope=*/true);
    const std::shared_ptr<details::Storage>& storage = details::ArrayStorageFactory::storageOf(array);
    if (!storage->tryBeginWrite(cudaStream))
    {
        return std::nullopt;
    }
    return ArrayWriteAccess(storage, array.data(), array.nbytes(), device, cudaStream);
}

void recordCudaWrite(Array& array, std::uintptr_t cudaStream)
{
    const Device device = array.device();
    if (!device.isCuda())
    {
        throw std::invalid_argument("recordCudaWrite(): array must reside on a CUDA device");
    }
    validateCudaStream(device, cudaStream, "recordCudaWrite()", /*movableScope=*/false);
    details::ArrayStorageFactory::storageOf(array)->recordExternalWrite(cudaStream);
}

} // namespace array
} // namespace common
} // namespace isaacsim
