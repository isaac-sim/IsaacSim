// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <carb/logging/Log.h> // CudaRintime.h does not have CARB_LOG_ERROR

#include <cuda.h>
#include <cuda_runtime.h>

namespace isaacsim
{
namespace core
{
namespace includes
{

#define CUDA_SUCCEEDED(result) ((result) == cudaSuccess)
#define CUDA_FAILED(result) ((result) != cudaSuccess)

#define CUDA_CHECK(result)                                                                                             \
    {                                                                                                                  \
        cudaError_t _result = (result);                                                                                \
        if (CUDA_FAILED(_result))                                                                                      \
        {                                                                                                              \
            CARB_LOG_ERROR("CUDA error %d: %s - %s at %s:%d", (_result), cudaGetErrorName(_result),                    \
                           cudaGetErrorString(_result), __FILE__, __LINE__);                                           \
        }                                                                                                              \
    }

#define CU_SUCCEEDED(result) ((result) == CUresult::CUDA_SUCCESS)
#define CU_FAILED(result) ((result) != CUresult::CUDA_SUCCESS)

#define CU_CHECK(result)                                                                                                 \
    {                                                                                                                    \
        CUresult _cu_result = (result);                                                                                  \
        if (CU_FAILED(_cu_result))                                                                                       \
        {                                                                                                                \
            const char* _errorName = nullptr;                                                                            \
            const char* _errorString = nullptr;                                                                          \
            cuGetErrorName(_cu_result, &_errorName);                                                                     \
            cuGetErrorString(_cu_result, &_errorString);                                                                 \
            CARB_LOG_ERROR("CU error %d: %s - %s at %s:%d", (_cu_result), _errorName, _errorString, __FILE__, __LINE__); \
        }                                                                                                                \
    }

/**
 * @class ScopedDevice
 * @brief RAII wrapper for CUDA device context management.
 * @details
 * Provides automatic CUDA device context switching and restoration.
 * When constructed, switches to the specified device (if different from current).
 * When destroyed, restores the previous device context.
 *
 * Key features:
 * - Automatic device context management
 * - Exception-safe device restoration
 * - Support for CPU-only mode (-1 device)
 * - Thread-safe device switching
 *
 * @note Uses RAII pattern to ensure device context is always properly restored
 * @warning CUDA API calls must be error-checked for proper device management
 */
class ScopedDevice
{
public:
    /**
     * @brief Constructs a scoped device context manager.
     * @details
     * Saves the current device and switches to the specified device if different.
     * If device is -1 or CUDA is unavailable, operates in CPU-only mode.
     *
     * @param[in] device CUDA device ID to switch to (-1 for CPU mode)
     *
     * @note Device -1 indicates CPU-only mode
     * @warning Ensure CUDA runtime is initialized before using this class
     */
    ScopedDevice(const int device = -1) : m_device(device)
    {
        // if we want cpu or can't get a cuda device, then do nothing.
        if (device == -1 || cudaGetDevice(&m_oldDevice) != cudaError::cudaSuccess)
        {
            m_oldDevice = m_device = -1;
            return;
        }

        // if we want a device, and its not the current threads host device, then set it.
        if (m_device != m_oldDevice)
        {
            CUDA_CHECK(cudaSetDevice(m_device));
            // NOTE: what do you want to do with an error here?  set m_oldDevice to m_device so you do nothing in dtor?
        }
    }

    /**
     * @brief Destructor that restores the previous device context.
     * @details
     * Automatically switches back to the original device if a switch was performed.
     * Ensures device context is restored even if exceptions occur.
     */
    ~ScopedDevice()
    {
        // return device back to what we had before if we set earlier
        if (m_device != m_oldDevice)
        {
            CUDA_CHECK(cudaSetDevice(m_oldDevice));
        }
    }

private:
    /** @brief Target CUDA device ID (or -1 for CPU mode) */
    int m_device = 0;
    /** @brief Original CUDA device ID before context switch */
    int m_oldDevice = 0;
};

/**
 * @class ScopedCudaTextureObject
 * @brief RAII wrapper for CUDA texture object management.
 * @details
 * Manages the lifecycle of a CUDA texture object, including creation and cleanup.
 * Automatically handles:
 * - Texture object creation from mipmapped arrays
 * - Resource and texture descriptor setup
 * - Automatic cleanup on destruction
 *
 * @note Uses RAII pattern to ensure texture resources are properly freed
 * @warning Requires valid CUDA context when constructed
 */
class ScopedCudaTextureObject final
{
    /** @brief Handle to the CUDA texture object */
    cudaTextureObject_t m_texObj = 0;

public:
    /**
     * @brief Constructs a texture object from a mipmapped array.
     * @details
     * Creates a texture object with specified properties:
     * - Clamp address mode
     * - Point filtering
     * - Element-type read mode
     * - Normalized coordinates
     *
     * @param[in] mmarr CUDA mipmapped array handle
     * @param[in] mipLevel Mipmap level to use (default: 0)
     *
     * @note Fails gracefully if input array is invalid
     * @warning Ensure mipmapped array remains valid during object lifetime
     */
    ScopedCudaTextureObject(cudaMipmappedArray_t mmarr, int mipLevel = 0)
    {
        if (!mmarr)
        {
            return;
        }
        cudaArray_t levelArray = 0;
        CUDA_CHECK(cudaGetMipmappedArrayLevel(&levelArray, mmarr, mipLevel));
        if (!levelArray)
        {
            return;
        }
        struct cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeArray;
        resDesc.res.array.array = levelArray;
        struct cudaTextureDesc texDesc;
        memset(&texDesc, 0, sizeof(texDesc));
        texDesc.addressMode[0] = cudaAddressModeClamp;
        texDesc.addressMode[1] = cudaAddressModeClamp;
        texDesc.filterMode = cudaFilterModePoint;
        texDesc.readMode = cudaReadModeElementType;
        texDesc.normalizedCoords = 1;
        CUDA_CHECK(cudaCreateTextureObject(&m_texObj, &resDesc, &texDesc, nullptr));
    }

    /**
     * @brief Destructor that cleans up the texture object.
     * @details Automatically destroys the texture object if it was successfully created.
     */
    ~ScopedCudaTextureObject()
    {
        if (m_texObj)
        {
            CUDA_CHECK(cudaDestroyTextureObject(m_texObj));
        }
    }

    /**
     * @brief Implicit conversion operator to texture object handle.
     * @return Reference to the underlying CUDA texture object
     */
    operator cudaTextureObject_t&()
    {
        return m_texObj;
    }
};

}
}
}
