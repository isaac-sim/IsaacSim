// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cub/device/device_select.cuh>

#include "GenericModelOutput.h"
#include "isaacsim/core/includes/ScopedCudaDevice.h"
#include "isaacsim/core/includes/Buffer.h"
#include "IsaacSimSensorsRTXCuda.cuh"

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

__global__ void getModelOutputFromBufferKernel(void* dataPtr, omni::sensors::GenericModelOutput* gmoPtrDevice) {
    *gmoPtrDevice = omni::sensors::getModelOutputFromBuffer(dataPtr);
}

__global__ void fillIndicesKernel(size_t* indices, const int length) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= length)
        return;
    indices[idx] = idx;
}

__global__ void fillPointsKernel(omni::sensors::GenericModelOutput* gmoPtr, float3* cartesianPoints, size_t* validIndices, const int numValidPoints) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numValidPoints)
        return;
    if (gmoPtr->elementsCoordsType == omni::sensors::CoordsType::CARTESIAN) {
        cartesianPoints[idx].x = gmoPtr->elements.x[validIndices[idx]];
        cartesianPoints[idx].y = gmoPtr->elements.y[validIndices[idx]];
        cartesianPoints[idx].z = gmoPtr->elements.z[validIndices[idx]];
    } else if (gmoPtr->elementsCoordsType == omni::sensors::CoordsType::SPHERICAL) {
        float azimuthDeg = gmoPtr->elements.x[validIndices[idx]];
        float elevationDeg = gmoPtr->elements.y[validIndices[idx]];
        float cosAzimuth, sinAzimuth, cosElevation, sinElevation;
        sincospif(azimuthDeg/180.0f, &sinAzimuth, &cosAzimuth);
        sincospif(elevationDeg/180.0f, &sinElevation, &cosElevation);
        cartesianPoints[idx].x = gmoPtr->elements.z[validIndices[idx]] * cosElevation * cosAzimuth;
        cartesianPoints[idx].y = gmoPtr->elements.z[validIndices[idx]] * cosElevation * sinAzimuth;
        cartesianPoints[idx].z = gmoPtr->elements.z[validIndices[idx]] * sinElevation;
    }
}

__global__ void fillPointsKernel(float* x, float* y, float* z, float3* cartesianPoints, size_t* validIndices, const int numValidPoints, omni::sensors::CoordsType* elementsCoordsType) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numValidPoints)
        return;
    if (*elementsCoordsType == omni::sensors::CoordsType::CARTESIAN) {
        cartesianPoints[idx].x = x[validIndices[idx]];
        cartesianPoints[idx].y = y[validIndices[idx]];
        cartesianPoints[idx].z = z[validIndices[idx]];
    } else if (*elementsCoordsType == omni::sensors::CoordsType::SPHERICAL) {
        float azimuthDeg = x[validIndices[idx]];
        float elevationDeg = y[validIndices[idx]];
        float range = z[validIndices[idx]];
        float cosAzimuth, sinAzimuth, cosElevation, sinElevation;
        sincospif(azimuthDeg/180.0f, &sinAzimuth, &cosAzimuth);
        sincospif(elevationDeg/180.0f, &sinElevation, &cosElevation);
        cartesianPoints[idx].x = range * cosElevation * cosAzimuth;
        cartesianPoints[idx].y = range * cosElevation * sinAzimuth;
        cartesianPoints[idx].z = range * sinElevation;
    }
}

struct IsValid
{
    uint8_t* flags{ nullptr }; // sensor specific flags

    __host__ __device__ __forceinline__
    IsValid(uint8_t* flags) : flags(flags) {}

    __host__ __device__ __forceinline__
    bool operator()(const int &i) const {
        const uint8_t fl = flags[i];
        return (fl & omni::sensors::ElementFlags::VALID) == omni::sensors::ElementFlags::VALID;
    }
};

void IsaacExtractRTXSensorPointCloudDeviceBuffers::initialize(void* dataPtr, cudaStream_t cudaStream) {

    cudaPointerAttributes attributes;
    CUDA_CHECK(cudaPointerGetAttributes(&attributes, dataPtr));

    this->cudaStream = cudaStream;
    this->gmoOnDevice = attributes.type == cudaMemoryTypeDevice;
    this->cudaDevice = attributes.device;

    isaacsim::core::includes::ScopedDevice scopedDevice(this->cudaDevice);
    if (!this->gmoOnDevice) {
        // We can get the GMO from the host pointer directly, then use that to size the buffers
        omni::sensors::GenericModelOutput gmoHost = omni::sensors::getModelOutputFromBuffer(dataPtr);
        // Allocate twice the size of the current number of elements to account for any variation in return count
        this->bufferSize = std::min(gmoHost.numElements * 2, IsaacExtractRTXSensorPointCloudDeviceBuffers::MAX_ELEMENTS);
        // Copy the elementsCoordsType from the GMO struct onto the device
        this->elementsCoordsType.setDevice(this->cudaDevice);
        this->elementsCoordsType.resize(1);
        CUDA_CHECK(cudaMemcpyAsync(this->elementsCoordsType.data(), &gmoHost.elementsCoordsType, sizeof(omni::sensors::CoordsType), cudaMemcpyHostToDevice, this->cudaStream));
    } else {
        // Pre-allocate device memory for the GenericModelOutput struct
        this->gmo.setDevice(this->cudaDevice);
        this->gmo.resize(1);
        getModelOutputFromBufferKernel<<<1, 1, 0, this->cudaStream>>>(dataPtr, this->gmo.data());
        uint32_t numElements = 0;
        CUDA_CHECK(cudaMemcpyAsync(&numElements, &this->gmo.data()->numElements, sizeof(uint32_t), cudaMemcpyDeviceToHost, this->cudaStream));
        CUDA_CHECK(cudaStreamSynchronize(this->cudaStream));
        this->bufferSize = std::min(numElements * 2, IsaacExtractRTXSensorPointCloudDeviceBuffers::MAX_ELEMENTS);
    }

    // Get the maximum number of threads per block
    CUDA_CHECK(cudaDeviceGetAttribute(&this->maxThreadsPerBlock, cudaDevAttrMaxThreadsPerBlock, this->cudaDevice));

    this->numValidPoints.setDevice(this->cudaDevice);
    this->numValidPoints.resize(1);

    // Resize buffers
    this->resizeBuffers();
}

void IsaacExtractRTXSensorPointCloudDeviceBuffers::resizeBuffers() {
    isaacsim::core::includes::ScopedDevice scopedDevice(this->cudaDevice);

    if (!this->gmoOnDevice) {
        this->x.setDevice(this->cudaDevice);
        this->x.resize(this->bufferSize);
        this->y.setDevice(this->cudaDevice);
        this->y.resize(this->bufferSize);
        this->z.setDevice(this->cudaDevice);
        this->z.resize(this->bufferSize);
        this->flags.setDevice(this->cudaDevice);
        this->flags.resize(this->bufferSize);
    }
    // Pre-allocate device memory for the output buffer
    this->pointCloudBuffer.setDevice(this->cudaDevice);
    this->pointCloudBuffer.resize(this->bufferSize);

    // Pre-allocate device memory for the valid indices buffers and temporary storage
    this->validIndicesIn.setDevice(this->cudaDevice);
    this->validIndicesIn.resize(this->bufferSize);
    const int nb_fillIndices = (this->bufferSize + this->maxThreadsPerBlock - 1) / this->maxThreadsPerBlock;
    fillIndicesKernel<<<nb_fillIndices, this->maxThreadsPerBlock, 0, cudaStream>>>(this->validIndicesIn.data(), this->bufferSize);

    this->validIndicesOut.setDevice(this->cudaDevice);
    this->validIndicesOut.resize(this->bufferSize);

    this->tempStorage.setDevice(this->cudaDevice);
    this->tempStorage.resize(this->bufferSize);
}

void IsaacExtractRTXSensorPointCloudDeviceBuffers::fillPointCloudBuffer(void* dataPtr, size_t& numValidPointsHost, omni::sensors::FrameAtTime& frameAtEndHost) {
    isaacsim::core::includes::ScopedDevice scopedDevice(this->cudaDevice);

    uint32_t numReturns = 0;
    uint8_t* flagsDevicePtr = nullptr;

    omni::sensors::GenericModelOutput gmoHost;

    // Update number of returns and device pointer to flags
    if (this->gmoOnDevice) {
        // Incoming dataPtr is a device address
        getModelOutputFromBufferKernel<<<1, 1, 0, this->cudaStream>>>(dataPtr, this->gmo.data());
        CUDA_CHECK(cudaMemcpyAsync(&numReturns, &this->gmo.data()->numElements, sizeof(uint32_t), cudaMemcpyDeviceToHost, this->cudaStream));
        CUDA_CHECK(cudaMemcpyAsync(&flagsDevicePtr, &this->gmo.data()->elements.flags, sizeof(uint8_t*), cudaMemcpyDeviceToHost, this->cudaStream));
        CUDA_CHECK(cudaMemcpyAsync(&frameAtEndHost, &this->gmo.data()->frameEnd, sizeof(omni::sensors::FrameAtTime), cudaMemcpyDeviceToHost, this->cudaStream));
        CUDA_CHECK(cudaStreamSynchronize(this->cudaStream));
    } else {
        // Incoming dataPtr is a host address
        gmoHost = omni::sensors::getModelOutputFromBuffer(dataPtr);
        numReturns = gmoHost.numElements;
        frameAtEndHost = gmoHost.frameEnd;
    }

    if (numReturns > this->bufferSize) {
        this->bufferSize = numReturns;
        this->resizeBuffers();
    }

    if (!this->gmoOnDevice) {
        CUDA_CHECK(cudaMemcpyAsync(this->x.data(), gmoHost.elements.x, gmoHost.numElements * sizeof(float), cudaMemcpyHostToDevice, this->cudaStream));
        CUDA_CHECK(cudaMemcpyAsync(this->y.data(), gmoHost.elements.y, gmoHost.numElements * sizeof(float), cudaMemcpyHostToDevice, this->cudaStream));
        CUDA_CHECK(cudaMemcpyAsync(this->z.data(), gmoHost.elements.z, gmoHost.numElements * sizeof(float), cudaMemcpyHostToDevice, this->cudaStream));
        CUDA_CHECK(cudaMemcpyAsync(this->flags.data(), gmoHost.elements.flags, gmoHost.numElements * sizeof(uint8_t), cudaMemcpyHostToDevice, this->cudaStream));
        flagsDevicePtr = this->flags.data();
        CUDA_CHECK(cudaStreamSynchronize(this->cudaStream));
    }

    // Store indices of valid returns in the GMO buffer in the validIndices buffer
    // Call cub::DeviceSelect::If with a nullptr to get the size of the temporary storage buffer
    // Then allocate the temporary storage buffer and call cub::DeviceSelect::If again to fill validIndices.
    // Note the second operation automatically resizes validIndices to the number of valid points.
    void* d_temp_storage = nullptr;
    size_t tmpCubStorageBuffBytes = 0;
    cub::DeviceSelect::If(d_temp_storage, tmpCubStorageBuffBytes, this->validIndicesIn.data(), this->validIndicesOut.data(), this->numValidPoints.data(), numReturns, IsValid(flagsDevicePtr), this->cudaStream);
    if (tmpCubStorageBuffBytes > this->tempStorage.size()) {
        this->tempStorage.resizeAsync(tmpCubStorageBuffBytes, this->cudaStream);
    }
    cub::DeviceSelect::If(this->tempStorage.data(), tmpCubStorageBuffBytes, this->validIndicesIn.data(), this->validIndicesOut.data(), this->numValidPoints.data(), numReturns, IsValid(flagsDevicePtr), this->cudaStream);

    // Copy the number of valid points from the device to the host, then synchronize
    CUDA_CHECK(cudaMemcpyAsync(&numValidPointsHost, this->numValidPoints.data(), sizeof(size_t), cudaMemcpyDeviceToHost, this->cudaStream));
    CUDA_CHECK(cudaStreamSynchronize(this->cudaStream));

    // Fill the point cloud buffer with the valid points
    const int nb_fillPoints = (numValidPointsHost + this->maxThreadsPerBlock - 1) / this->maxThreadsPerBlock;
    if (this->gmoOnDevice) {
        fillPointsKernel<<<nb_fillPoints, this->maxThreadsPerBlock, 0, this->cudaStream>>>(this->gmo.data(), this->pointCloudBuffer.data(), this->validIndicesOut.data(), numValidPointsHost);
    } else {
        float* xDevicePtr = this->x.data();
        float* yDevicePtr = this->y.data();
        float* zDevicePtr = this->z.data();
        omni::sensors::CoordsType* elementsCoordsTypeDevicePtr = this->elementsCoordsType.data();
        fillPointsKernel<<<nb_fillPoints, this->maxThreadsPerBlock, 0, this->cudaStream>>>(xDevicePtr, yDevicePtr, zDevicePtr, this->pointCloudBuffer.data(), this->validIndicesOut.data(), numValidPointsHost, elementsCoordsTypeDevicePtr);
    }
    CUDA_CHECK(cudaStreamSynchronize(this->cudaStream));
}

}   // namespace isaacsim
}   // namespace sensors
}   // namespace rtx
