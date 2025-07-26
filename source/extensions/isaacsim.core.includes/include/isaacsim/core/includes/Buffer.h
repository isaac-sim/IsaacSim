// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "ScopedCudaDevice.h"

#include <cuda.h>
#include <cudaTypedefs.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @brief Enumeration specifying the type of memory allocation.
 * @details Defines whether the buffer resides in host (CPU) or device (GPU) memory.
 */
enum class MemoryType
{
    /** @brief Memory allocated in host (CPU) RAM */
    eHost = 0,
    /** @brief Memory allocated in device (GPU) VRAM */
    eDevice = 1,
};

/**
 * @class Buffer
 * @brief Abstract base class for memory buffer management.
 * @details
 * Provides a common interface for managing memory buffers, whether they reside in
 * host (CPU) or device (GPU) memory. This class defines the basic operations that
 * all buffer implementations must support.
 *
 * @tparam T The data type stored in the buffer
 *
 * @note All derived classes must implement resize(), data(), and size() functions
 */
template <typename T>
class Buffer
{
public:
    /** @brief Virtual destructor for proper cleanup of derived classes */
    virtual ~Buffer() = default;

    /**
     * @brief Resizes the buffer to hold the specified number of elements.
     * @param[in] size The new size of the buffer in number of elements
     */
    virtual void resize(size_t size) = 0;

    /**
     * @brief Gets a pointer to the buffer's data.
     * @return Pointer to the buffer's data
     */
    virtual T* data() const = 0;

    /**
     * @brief Gets the number of elements in the buffer.
     * @return Number of elements in the buffer
     */
    virtual size_t size() const = 0;

    /**
     * @brief Gets the size of a single element in bytes.
     * @return Size of type T in bytes
     */
    size_t sizeofType() const
    {
        return sizeof(T);
    }

    /**
     * @brief Gets the total size of the buffer in bytes.
     * @return Total size of the buffer in bytes
     */
    size_t sizeInBytes() const
    {
        return size() * sizeofType();
    }

    /**
     * @brief Gets the memory type of the buffer.
     * @return Memory type (Host or Device)
     */
    MemoryType type() const
    {
        return m_memoryType;
    }

protected:
    /** @brief Type of memory where the buffer resides */
    MemoryType m_memoryType;
};

/**
 * @class DeviceBufferBase
 * @brief CUDA device (GPU) memory buffer implementation.
 * @details
 * Manages a buffer of memory allocated on a CUDA device. Provides functionality for:
 * - Memory allocation and deallocation
 * - Device selection and switching
 * - Memory copying between host and device
 * - Debug printing of buffer contents
 *
 * @tparam T The data type stored in the buffer
 *
 * @note Uses RAII principles for automatic resource management
 * @warning Requires proper CUDA environment setup
 */
template <typename T>
class DeviceBufferBase : public Buffer<T>
{
    using Buffer<T>::m_memoryType;

public:
    /**
     * @brief Constructs a new device buffer.
     * @param[in] size Initial size of the buffer in elements (default: 0)
     * @param[in] device CUDA device ID to allocate on (default: -1 for CPU)
     */
    DeviceBufferBase(const size_t& size = 0, const int device = -1)
    {
        m_memoryType = MemoryType::eDevice;
        m_device = device;
        resize(size);
    }

    /**
     * @brief Destructor that ensures proper cleanup of device memory.
     */
    virtual ~DeviceBufferBase()
    {
        ScopedDevice scopedDevice(m_device);
        CUDA_CHECK(cudaFree(m_buffer));
        m_buffer = nullptr;
    }

    /**
     * @brief Changes the CUDA device for this buffer.
     * @details
     * If the device changes, existing memory is freed on the old device
     * and reallocated on the new device.
     *
     * @param[in] device New CUDA device ID (-1 for CPU)
     */
    virtual void setDevice(const int device = -1)
    {
        if (device != m_device)
        {
            // if the device doesn't match and we had a buffer allocated, release it on the old device and switch
            if (m_buffer)
            {
                ScopedDevice scopedDevice(m_device);
                CUDA_CHECK(cudaFree(m_buffer));
                m_buffer = nullptr;
            }
            m_device = device;
            resize(m_size);
        }
    }

    /**
     * @brief Resizes the device buffer.
     * @details
     * Reallocates memory if the new size is different from the current size.
     * Handles deallocation of existing memory if necessary.
     *
     * @param[in] size New size in number of elements
     */
    virtual void resize(size_t size)
    {
        if ((size != m_size && size > 0) || m_buffer == nullptr)
        {
            ScopedDevice scopedDevice(m_device);
            if (m_buffer)
            {
                CUDA_CHECK(cudaFree(m_buffer));
                m_buffer = nullptr;
            }
            if (size > 0)
            {
                CUDA_CHECK(cudaMalloc(&m_buffer, size * sizeof(T)));
            }
            m_size = size;
        }
    }

    /**
     * @brief Asynchronously resizes the device buffer.
     * @details
     * Reallocates memory if the new size is different from the current size.
     * Handles deallocation of existing memory if necessary.
     *
     * @param[in] size New size in number of elements
     * @param[in] cudaStream CUDA stream to use for asynchronous operation
     */
    virtual void resizeAsync(size_t size, cudaStream_t cudaStream = 0)
    {
        if ((size != m_size && size > 0) || m_buffer == nullptr)
        {
            ScopedDevice scopedDevice(m_device);
            if (m_buffer)
            {
                CUDA_CHECK(cudaFreeAsync(m_buffer, cudaStream));
                m_buffer = nullptr;
            }
            if (size > 0)
            {
                CUDA_CHECK(cudaMallocAsync(&m_buffer, size * sizeof(T), cudaStream));
            }
            m_size = size;
        }
    }

    /**
     * @brief Gets a pointer to the device memory.
     * @return Raw pointer to the device memory
     */
    virtual T* data() const
    {
        return m_buffer;
    }

    /**
     * @brief Gets the current size of the buffer.
     * @return Number of elements in the buffer
     */
    virtual size_t size() const
    {
        return m_size;
    }

    /**
     * @brief Synchronously copies data to the device buffer.
     * @param[in] src Source pointer to copy from
     * @param[in] size Number of elements to copy
     * @param[in] kind Type of memory copy operation
     */
    virtual void copy(const void* src, size_t size, enum cudaMemcpyKind kind = cudaMemcpyDeviceToHost)
    {
        ScopedDevice scopedDevice(m_device);
        CUDA_CHECK(cudaMemcpy(m_buffer, src, size * sizeof(T), kind));
    }

    /**
     * @brief Asynchronously copies data to the device buffer.
     * @param[in] src Source pointer to copy from
     * @param[in] size Number of elements to copy
     * @param[in] kind Type of memory copy operation
     * @param[in] cudaStream CUDA stream to use for the copy operation
     */
    virtual void copyAsync(const void* src,
                           size_t size,
                           enum cudaMemcpyKind kind = cudaMemcpyDeviceToHost,
                           cudaStream_t cudaStream = 0)
    {
        ScopedDevice scopedDevice(m_device);
        CUDA_CHECK(cudaMemcpyAsync(m_buffer, src, size * sizeof(T), kind, cudaStream));
    }

    /**
     * @brief Prints the buffer contents for debugging.
     * @param[in] start String to print before the buffer contents
     * @param[in] end String to print after the buffer contents
     */
    void debugPrint(const std::string& start, const std::string& end)
    {
        ScopedDevice scopedDevice(m_device);
        printf("%s", start.c_str());
        std::vector<T> hostBuffer(m_size);
        CUDA_CHECK(cudaMemcpyAsync(hostBuffer.data(), m_buffer, m_size * sizeof(T), cudaMemcpyDeviceToHost));
        for (size_t i = 0; i < m_size; ++i)
        {
            std::cout << hostBuffer[i];
            if (i != m_size - 1)
            {
                std::cout << ", ";
            }
        }
        printf("%s", end.c_str());
    }

private:
    /** @brief Pointer to device memory */
    T* m_buffer = nullptr;
    /** @brief Current size of the buffer in elements */
    size_t m_size = 0;
    /** @brief CUDA device ID where memory is allocated */
    int m_device = 0;
};

/**
 * @class HostBufferBase
 * @brief Host (CPU) memory buffer implementation.
 * @details
 * Manages a buffer of memory allocated in host RAM using std::vector.
 * Provides a simple wrapper around std::vector with the Buffer interface.
 *
 * @tparam T The data type stored in the buffer
 */
template <typename T>
class HostBufferBase : public Buffer<T>
{
    using Buffer<T>::m_memoryType;

public:
    /**
     * @brief Constructs a new host buffer.
     * @param[in] size Initial size of the buffer in elements (default: 0)
     */
    HostBufferBase(size_t size = 0)
    {
        m_memoryType = MemoryType::eHost;
        resize(size);
    }

    /**
     * @brief Resizes the host buffer.
     * @param[in] size New size in number of elements
     */
    virtual void resize(size_t size)
    {
        m_buffer.resize(size);
    }

    /**
     * @brief Resizes the host buffer and initializes new elements.
     * @param[in] size New size in number of elements
     * @param[in] val Value to initialize new elements with
     */
    virtual void resize(size_t size, const T& val)
    {
        m_buffer.resize(size, val);
    }

    /**
     * @brief Gets a pointer to the host memory.
     * @return Raw pointer to the host memory
     */
    virtual T* data() const
    {
        return (T*)m_buffer.data();
    }

    /**
     * @brief Gets the current size of the buffer.
     * @return Number of elements in the buffer
     */
    virtual size_t size() const
    {
        return m_buffer.size();
    }

    /** @brief Underlying vector storing the data */
    std::vector<T> m_buffer;
};


/**
 * @class GenericBufferBase
 * @brief Device-generic (CPU or CUDA device) memory buffer implementation.
 * @details
 * Manages a buffer of memory allocated in host RAM (using std::vector) or on a CUDA device.
 * Provides functionality for:
 * - Memory allocation and deallocation.
 * - Device selection and switching (with or without data transfer).
 * - Memory copying between host and device.
 * - Fill the buffer with a constant value.
 * - Generate a string representation of the buffer.
 *
 * @tparam T The data type stored in the buffer.
 *
 * @note Uses RAII principles for automatic resource management.
 * @warning Requires proper CUDA environment setup and Unified Virtual Address (UVA) space.
 */
template <typename T>
class GenericBufferBase : public Buffer<T>
{
    using Buffer<T>::m_memoryType;

public:
    /**
     * @brief Construct a new device-generic (CPU or CUDA device) buffer.
     * @param[in] size Initial size of the buffer in elements.
     * @param[in] device CUDA device ID (higher or equal to 0) to allocate on or -1 for CPU.
     */
    GenericBufferBase(const size_t& size = 0, const int device = -1)
    {
        m_memoryType = device < 0 ? MemoryType::eHost : MemoryType::eDevice;
        m_device = device;
        resize(size);
    }

    /**
     * @brief Destructor that ensures proper cleanup of allocated memory.
     */
    virtual ~GenericBufferBase()
    {
        clear();
    }

    /**
     * @brief Gets a pointer to the buffer's data.
     * @return Pointer to the buffer's data.
     */
    virtual T* data() const
    {
        return m_buffer;
    }

    /**
     * @brief Gets the current size of the buffer.
     * @return Number of elements in the buffer.
     */
    virtual size_t size() const
    {
        return m_size;
    }

    /**
     * @brief Clear the buffer.
     * @details
     * Clear the buffer by freeing the memory and resetting the size to 0.
     */
    virtual void clear()
    {
        {
            ScopedDevice scopedDevice(m_device);
            CUDA_CHECK(m_memoryType == MemoryType::eDevice ? cudaFree(m_buffer) : cudaFreeHost(m_buffer));
            m_buffer = nullptr;
        }
        m_size = 0;
    }

    /**
     * @brief Changes the device for this buffer.
     * @details
     * If the device changes, existing memory is freed on the old device
     * and reallocated on the new device.
     *
     * @param[in] device New CUDA device ID (higher or equal to 0) or -1 for CPU.
     * @param[in] keepData If true, the data from the old device is kept on the new device.
     * @return True if the device was changed, false otherwise.
     */
    virtual bool setDevice(const int device = -1, bool keepData = false)
    {
        if (device != m_device)
        {
            // Define variables to transfer the data in case we want to keep it
            MemoryType tmpMemoryType;
            T* tmpBuffer;
            // Save data temporarily
            if (keepData)
            {
                tmpMemoryType = m_memoryType;
                tmpBuffer = m_buffer;
                m_buffer = nullptr;
            }
            // Release old allocated memory if it exists and we don't want to keep the data
            else if (m_buffer)
            {
                ScopedDevice scopedDevice(m_device);
                CUDA_CHECK(m_memoryType == MemoryType::eDevice ? cudaFree(m_buffer) : cudaFreeHost(m_buffer));
                m_buffer = nullptr;
            }
            // Switch to new device
            m_memoryType = device < 0 ? MemoryType::eHost : MemoryType::eDevice;
            m_device = device;
            resize(m_size); // Recreate the buffer on the new device
            // Transfer/restore data from temporary buffer
            if (keepData)
            {
                ScopedDevice scopedDevice(m_device);
                CUDA_CHECK(cudaMemcpy(m_buffer, tmpBuffer, m_size * sizeof(T), cudaMemcpyDefault));
                CUDA_CHECK(tmpMemoryType == MemoryType::eDevice ? cudaFree(tmpBuffer) : cudaFreeHost(tmpBuffer));
                tmpBuffer = nullptr;
            }
            return true;
        }
        return false;
    }

    /**
     * @brief Resizes the device buffer (synchronous version of \ref resizeAsync).
     * @details
     * Reallocates memory if the new size is different from the current size.
     * Handles deallocation of existing memory if necessary.
     *
     * @param[in] size New size in number of elements.
     * @return True if the size was changed, false otherwise.
     */
    virtual void resize(size_t size)
    {
        if (size != m_size || m_buffer == nullptr)
        {
            ScopedDevice scopedDevice(m_device);
            if (m_buffer)
            {
                CUDA_CHECK(m_memoryType == MemoryType::eDevice ? cudaFree(m_buffer) : cudaFreeHost(m_buffer));
                m_buffer = nullptr;
            }
            if (size > 0)
            {
                CUDA_CHECK(m_memoryType == MemoryType::eDevice ?
                               cudaMalloc(&m_buffer, size * sizeof(T)) :
                               cudaHostAlloc(&m_buffer, size * sizeof(T), cudaHostAllocDefault));
                m_size = size;
            }
            else
            {
                clear();
            }
        }
    }

    /**
     * @brief Resizes the device buffer asynchronously (asynchronous version of \ref resize).
     * @details
     * Reallocates memory if the new size is different from the current size.
     * Handles deallocation of existing memory if necessary.
     *
     * @param[in] size New size in number of elements.
     * @param[in] cudaStream CUDA stream to use for the operation.
     * @return True if the size was changed, false otherwise.
     */
    virtual void resizeAsync(size_t size, cudaStream_t cudaStream = 0)
    {
        if (size != m_size || m_buffer == nullptr)
        {
            ScopedDevice scopedDevice(m_device);
            if (m_buffer)
            {
                CUDA_CHECK(m_memoryType == MemoryType::eDevice ? cudaFreeAsync(m_buffer, cudaStream) :
                                                                 cudaFreeHost(m_buffer));
                m_buffer = nullptr;
            }
            if (size > 0)
            {
                CUDA_CHECK(m_memoryType == MemoryType::eDevice ?
                               cudaMallocAsync(&m_buffer, size * sizeof(T), cudaStream) :
                               cudaHostAlloc(&m_buffer, size * sizeof(T), cudaHostAllocDefault));
                m_size = size;
            }
            else
            {
                clear();
            }
        }
    }

    /**
     * @brief Copies the buffer's data to a destination memory (synchronous version of \ref copyToAsync).
     * @note If the specified size is greater than the buffer size, only the first `.size()` elements are copied.
     * A warning message is logged in this case.
     *
     * @param[out] dst Destination pointer to copy to.
     * @param[in] size Number of elements to copy.
     */
    virtual void copyTo(void* dst, size_t size)
    {
        if (size > m_size)
        {
            CARB_LOG_WARN(
                "Buffer::copyTo: Specified size (%zu) is greater than the buffer size (%zu). Copying only %zu elements.",
                size, m_size, m_size);
            size = m_size;
        }
        {
            ScopedDevice scopedDevice(m_device);
            CUDA_CHECK(cudaMemcpy(dst, m_buffer, size * sizeof(T), cudaMemcpyDefault));
        }
    }

    /**
     * @brief Copies the buffer's data to a destination memory asynchronously (asynchronous version of \ref copyTo).
     * @note If the specified size is greater than the buffer size, only the first `.size()` elements are copied.
     * A warning message is logged in this case.
     *
     * @param[out] dst Destination pointer to copy to.
     * @param[in] size Number of elements to copy.
     * @param[in] cudaStream CUDA stream to use for the operation.
     */
    virtual void copyToAsync(void* dst, size_t size, cudaStream_t cudaStream = 0)
    {
        if (size > m_size)
        {
            CARB_LOG_WARN(
                "Buffer::copyToAsync: Specified size (%zu) is greater than the buffer size (%zu). Copying only %zu elements.",
                size, m_size, m_size);
            size = m_size;
        }
        {
            ScopedDevice scopedDevice(m_device);
            CUDA_CHECK(cudaMemcpyAsync(dst, m_buffer, size * sizeof(T), cudaMemcpyDefault, cudaStream));
        }
    }

    /**
     * @brief Copies the buffer's data from a source memory (synchronous version of \ref copyFromAsync).
     * @note If the specified size is greater than the buffer size, only the first `.size()` elements are copied.
     * A warning message is logged in this case.
     *
     * @param[in] src Source pointer to copy from.
     * @param[in] size Number of elements to copy.
     */
    virtual void copyFrom(const void* src, size_t size)
    {
        if (size > m_size)
        {
            CARB_LOG_WARN(
                "Buffer::copyFrom: Specified size (%zu) is greater than the buffer size (%zu). Copying only %zu elements.",
                size, m_size, m_size);
            size = m_size;
        }
        {
            ScopedDevice scopedDevice(m_device);
            CUDA_CHECK(cudaMemcpy(m_buffer, src, size * sizeof(T), cudaMemcpyDefault));
        }
    }

    /**
     * @brief Copies the buffer's data from a source memory asynchronously (asynchronous version of \ref copyFrom).
     * @note If the specified size is greater than the buffer size, only the first `.size()` elements are copied.
     * A warning message is logged in this case.
     *
     * @param[in] src Source pointer to copy from.
     * @param[in] size Number of elements to copy.
     * @param[in] cudaStream CUDA stream to use for the operation.
     */
    virtual void copyFromAsync(const void* src, size_t size, cudaStream_t cudaStream = 0)
    {
        if (size > m_size)
        {
            CARB_LOG_WARN(
                "Buffer::copyFromAsync: Specified size (%zu) is greater than the buffer size (%zu). Copying only %zu elements.",
                size, m_size, m_size);
            size = m_size;
        }
        {
            ScopedDevice scopedDevice(m_device);
            CUDA_CHECK(cudaMemcpyAsync(m_buffer, src, size * sizeof(T), cudaMemcpyDefault, cudaStream));
        }
    }

    /**
     * @brief Fills the buffer with a constant value (synchronous version of \ref fillAsync).
     * @warning Only data types of size 1, 2, and 4 bytes are supported. Using other data type sizes will log a warning.
     *
     * @param[in] value Value to fill the buffer with.
     * @return True if the fill was successful, false otherwise.
     */
    virtual bool fill(const T& value)
    {
        if (m_buffer != nullptr)
        {
            ScopedDevice scopedDevice(m_device);
            switch (sizeof(T))
            {
            case 1:
                // cuMemsetD8: unsigned char
                volatile union
                {
                    T value;
                    unsigned char bytes;
                } data8;
                data8.value = value;
                CU_CHECK(cuMemsetD8((CUdeviceptr)m_buffer, data8.bytes, m_size));
                return true;
                break;
            case 2:
                // cuMemsetD16: unsigned short
                volatile union
                {
                    T value;
                    unsigned short bytes;
                } data16;
                data16.value = value;
                CU_CHECK(cuMemsetD16((CUdeviceptr)m_buffer, data16.bytes, m_size));
                return true;
                break;
            case 4:
                // cuMemsetD32: unsigned int
                volatile union
                {
                    T value;
                    unsigned int bytes;
                } data32;
                data32.value = value;
                CU_CHECK(cuMemsetD32((CUdeviceptr)m_buffer, data32.bytes, m_size));
                return true;
                break;
            default:
                CARB_LOG_WARN("Buffer::fill: Unsupported type size: %zu", sizeof(T));
                break;
            }
        }
        return false;
    }

    /**
     * @brief Fills the buffer with a constant value asynchronously (asynchronous version of \ref fill).
     * @warning Only data types of size 1, 2, and 4 bytes are supported. Using other data type sizes will log a warning.
     *
     * @param[in] value Value to fill the buffer with.
     * @param[in] cudaStream CUDA stream to use for the operation.
     * @return True if the fill was successful, false otherwise.
     */
    virtual bool fillAsync(const T& value, cudaStream_t cudaStream = 0)
    {
        if (m_buffer != nullptr)
        {
            ScopedDevice scopedDevice(m_device);
            switch (sizeof(T))
            {
            case 1:
                // cuMemsetD8: unsigned char
                volatile union
                {
                    T value;
                    unsigned char bytes;
                } data8;
                data8.value = value;
                CU_CHECK(cuMemsetD8Async((CUdeviceptr)m_buffer, data8.bytes, m_size, (CUstream)cudaStream));
                return true;
                break;
            case 2:
                // cuMemsetD16: unsigned short
                volatile union
                {
                    T value;
                    unsigned short bytes;
                } data16;
                data16.value = value;
                CU_CHECK(cuMemsetD16Async((CUdeviceptr)m_buffer, data16.bytes, m_size, (CUstream)cudaStream));
                return true;
                break;
            case 4:
                // cuMemsetD32: unsigned int
                volatile union
                {
                    T value;
                    unsigned int bytes;
                } data32;
                data32.value = value;
                CU_CHECK(cuMemsetD32Async((CUdeviceptr)m_buffer, data32.bytes, m_size, (CUstream)cudaStream));
                return true;
                break;
            default:
                CARB_LOG_WARN("Buffer::fill: Unsupported type size: %zu", sizeof(T));
                break;
            }
        }
        return false;
    }

    /**
     * @brief Generates a string representation of the buffer.
     * @return String representation of the buffer.
     */
    std::string toString() const
    {
        ScopedDevice scopedDevice(m_device);
        // Get device string representation
        std::string deviceString;
        cudaPointerAttributes pointerAttributes;
        cudaPointerGetAttributes(&pointerAttributes, m_buffer);
        switch (pointerAttributes.type)
        {
        case cudaMemoryTypeHost:
            deviceString = "'cpu'";
            break;
        case cudaMemoryTypeDevice:
            deviceString = "'cuda:" + std::to_string(pointerAttributes.device) + "'";
            break;
        case cudaMemoryTypeUnregistered:
            deviceString = "<unregistered>";
            break;
        case cudaMemoryTypeManaged:
            deviceString = "<managed>";
            break;
        default:
            deviceString = "<unknown>";
            break;
        }
        // Generate string representation of the buffer
        T* array = new T[m_size]; // do not use std::vector since for bool, it is a specialization that has no ::data
        CUDA_CHECK(cudaDeviceSynchronize());
        CUDA_CHECK(cudaMemcpy(array, m_buffer, m_size * sizeof(T), cudaMemcpyDefault));
        std::stringstream stringRepresentation;
        stringRepresentation << "Buffer([";
        for (size_t i = 0; i < m_size; ++i)
        {
            stringRepresentation << array[i];
            if (i != m_size - 1)
            {
                stringRepresentation << ", ";
            }
        }
        stringRepresentation << "], device=" << deviceString << ")";
        delete[] array;
        return stringRepresentation.str();
    }

private:
    /** @brief Pointer to device memory (CUDA or CPU). */
    T* m_buffer = nullptr;
    /** @brief Current size of the buffer in elements. */
    size_t m_size = 0;
    /** @brief Device ID where memory is allocated (higher or equal to 0 for CUDA device, -1 for CPU). */
    int m_device = -1;
};


/** @brief Type alias for a device buffer of bytes */
typedef DeviceBufferBase<uint8_t> DeviceBuffer;
/** @brief Type alias for a host buffer of bytes */
typedef HostBufferBase<uint8_t> HostBuffer;
/** @brief Type alias for a generic buffer of bytes */
typedef GenericBufferBase<uint8_t> GenericBuffer;

}
}
}
