// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/** @file
 * @brief Inter-process communication buffer management for CUDA memory
 * @details
 * This file provides a class for managing CUDA device memory buffers that can be
 * shared between processes. It implements a circular buffer pool with POSIX
 * file descriptor-based IPC to facilitate high-performance data exchange
 * between different processes in the ROS 2 core.
 */
#ifndef IPC_BUFFER_MANAGER_HPP
#define IPC_BUFFER_MANAGER_HPP

#include <sys/syscall.h>
#include <sys/types.h>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <stdexcept>
#include <stdio.h>
#include <unistd.h>
#include <vector>

/**
 * @class IPCBufferManager
 * @brief Manages CUDA device memory buffers for inter-process communication (IPC).
 * @details
 * This class creates and manages a pool of CUDA device memory buffers that can be
 * shared between processes using POSIX file descriptors. It handles the allocation,
 * mapping, and access control for these buffers, as well as cycling through them
 * for use in communication scenarios.
 */
class IPCBufferManager
{
public:
    IPCBufferManager() = default;

    /**
     * @brief Constructor that creates device memory buffers and exports them to file descriptors.
     * @details
     * Allocates a specified number of CUDA device memory buffers and exports them as
     * POSIX file descriptors for inter-process communication. The buffers are set up
     * with read/write access permissions.
     *
     * @param[in] size Number of buffers to create in the pool.
     * @param[in] bufferStep Size of each buffer in bytes.
     * @throws std::runtime_error If CUDA memory allocation granularity cannot be determined or is zero.
     */
    IPCBufferManager(size_t size, size_t bufferStep)
    {
        m_bufferSize = size;
        m_bufferStep = bufferStep;

        CUmemAllocationProp prop = {};
        prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
        prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
        prop.location.id = 0;
        prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
        size_t granularity = 0;
        auto cudaErr = cuMemGetAllocationGranularity(&granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
        if (CUDA_SUCCESS != cudaErr)
        {
            const char* errorStr = NULL;
            cuGetErrorString(cudaErr, &errorStr);
            throw std::runtime_error(std::string("IPCBufferManager: Failed to get CUDA memory allocation granularity: ") +
                                     (errorStr ? errorStr : "Unknown CUDA error"));
        }
        m_allocSize = bufferStep - (bufferStep % granularity) + granularity;

        for (size_t i = 0; i < m_bufferSize; i++)
        {
            CUmemGenericAllocationHandle genericAllocationHandle;
            auto cudaErr = cuMemCreate(&genericAllocationHandle, m_allocSize, &prop, 0);
            if (CUDA_SUCCESS != cudaErr)
            {
                const char* errorStr = NULL;
                cuGetErrorString(cudaErr, &errorStr);
                fprintf(stderr, "[Error] IPCBufferManager: Failed to call cuMemCreate %s\n", errorStr);
            }

            int fd = -1;
            cudaErr = cuMemExportToShareableHandle(
                reinterpret_cast<void*>(&fd), genericAllocationHandle, CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0);
            if (CUDA_SUCCESS != cudaErr)
            {
                const char* errorStr = NULL;
                cuGetErrorString(cudaErr, &errorStr);
                fprintf(stderr, "[Error] IPCBufferManager: Failed to cuMemExportToShareableHandle %s\n", errorStr);
            }

            CUdeviceptr dPtr = 0ULL;
            cudaErr = cuMemAddressReserve(&dPtr, m_allocSize, 0, 0, 0);
            if (CUDA_SUCCESS != cudaErr)
            {
                const char* errorStr = NULL;
                cuGetErrorString(cudaErr, &errorStr);
                fprintf(stderr, "[Error] IPCBufferManager: Failed to call cuMemAddressReserve %s\n", errorStr);
            }

            cudaErr = cuMemMap(dPtr, m_allocSize, 0, genericAllocationHandle, 0);
            if (CUDA_SUCCESS != cudaErr)
            {
                const char* errorStr = NULL;
                cuGetErrorString(cudaErr, &errorStr);
                fprintf(stderr, "[Error] IPCBufferManager: Failed to call cuMemMap %s\n", errorStr);
            }

            CUmemAccessDesc accessDesc = {};
            accessDesc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
            accessDesc.location.id = 0;
            accessDesc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
            cudaErr = cuMemSetAccess(dPtr, m_allocSize, &accessDesc, 1);
            if (CUDA_SUCCESS != cudaErr)
            {
                const char* errorStr = NULL;
                cuGetErrorString(cudaErr, &errorStr);
                fprintf(stderr, "[Error] IPCBufferManager: Failed to call cuMemSetAccess %s\n", errorStr);
            }

            m_bufferPtrs.push_back(dPtr);
            m_shareableHandles.push_back({ getpid(), fd });
            m_genericHandles.push_back(genericAllocationHandle);
        }
    }

    /**
     * @brief Destructor
     * @details
     * Frees all allocated device memory and resources when the buffer manager
     * is destroyed. This includes releasing all CUDA memory allocations and unmapping
     * virtual address ranges.
     */
    ~IPCBufferManager()
    {
        // Iterate over the actual number of allocated buffers rather than m_bufferSize so a
        // default-constructed manager (empty vectors) tears down cleanly.
        for (size_t i = 0; i < m_bufferPtrs.size(); i++)
        {
            // Tear down in the reverse order of construction: unmap the virtual address range,
            // release the backing allocation, then free the reserved address range. Calling
            // cuMemRelease before cuMemUnmap (the previous order) is contrary to the documented
            // CUDA VMM teardown sequence.
            cuMemUnmap(m_bufferPtrs[i], m_allocSize);
            cuMemRelease(m_genericHandles[i]);
            // cuMemAddressReserve must be balanced by cuMemAddressFree or the virtual address
            // range is leaked for the lifetime of the process.
            cuMemAddressFree(m_bufferPtrs[i], m_allocSize);
        }

        // Close the exported POSIX file descriptors. cuMemExportToShareableHandle hands back an
        // fd per buffer that is otherwise leaked on every manager teardown (risking fd exhaustion
        // when nodes are recreated).
        for (const auto& handle : m_shareableHandles)
        {
            if (handle.size() > 1 && handle[1] >= 0)
            {
                close(handle[1]);
            }
        }
    }

    /**
     * @brief Advances to the next available device memory buffer in the pool.
     * @details
     * Increments the current buffer index, wrapping around to the beginning
     * when the end of the buffer pool is reached. This implements a circular
     * buffer pattern for cycling through available memory blocks.
     */
    void next()
    {
        m_currentHandleIndex += 1;
        if (m_currentHandleIndex == m_bufferSize)
        {
            m_currentHandleIndex = 0;
        }
    }

    /**
     * @brief Retrieves the device pointer to the current memory buffer.
     * @details
     * Returns the CUDA device pointer for the currently selected buffer
     * in the pool, which can be used for CUDA operations.
     *
     * @return CUdeviceptr Device pointer to the current buffer.
     */
    CUdeviceptr getCurBufferPtr()
    {
        return m_bufferPtrs[m_currentHandleIndex];
    }

    /**
     * @brief Retrieves the IPC handle for the current memory buffer.
     * @details
     * Returns a reference to the vector containing the process ID and file descriptor
     * for the currently selected buffer, which can be used for inter-process communication.
     *
     * @return std::vector<int>& Reference to the vector containing the process ID and file descriptor.
     */
    std::vector<int>& getCurIpcMemHandle()
    {
        return m_shareableHandles[m_currentHandleIndex];
    }

private:
    /** @brief Number of buffers in the pool */
    size_t m_bufferSize;

    /** @brief Requested size of each buffer in bytes */
    size_t m_bufferStep;

    /** @brief Current buffer index in the circular buffer pool */
    size_t m_currentHandleIndex = 0;

    /** @brief Actual allocation size in bytes (aligned to CUDA memory granularity) */
    size_t m_allocSize;

    /** @brief Vector of shareable handles (process ID + file descriptor pairs) for each buffer */
    std::vector<std::vector<int>> m_shareableHandles;

    /** @brief Vector of CUDA memory allocation handles */
    std::vector<CUmemGenericAllocationHandle> m_genericHandles;

    /** @brief Vector of CUDA device pointers to mapped memory */
    std::vector<CUdeviceptr> m_bufferPtrs;
};

#endif // IPC_BUFFER_MANAGER_HPP
