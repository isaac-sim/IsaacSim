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
/*
Test is implemented using the doctest C++ testing framework:
  https://github.com/doctest/doctest/blob/master/doc/markdown/readme.md
*/

#include <carb/BindingsUtils.h>

#include <doctest/doctest.h>
#include <isaacsim/core/includes/Buffer.h>

CARB_BINDINGS("isaacsim.core.includes.tests")

TEST_SUITE("isaacsim.core.includes.tests")
{

    /**
     * Roundtrip sync/async copy test helper: (1) from host -> (2) to device -> (3) from device -> (4) to host
     */
    void testCopy(isaacsim::core::includes::GenericBufferBase<int> & buffer)
    {
        // Synchronous
        int* deviceData = nullptr;
        std::vector<int> hostData(5);
        // - (1) from host
        buffer.resize(5);
        buffer.copyFrom(std::vector<int>({ 1, 2, 3, 4, 5 }).data(), 5);
        // - (2) to device
        cudaMalloc(&deviceData, 5 * sizeof(int));
        buffer.copyTo(deviceData, 5);
        // - (3) from device
        buffer.clear();
        CHECK_UNARY(buffer.data() == nullptr); // ensure buffer is cleared
        buffer.resize(5);
        buffer.copyFrom(deviceData, 5);
        cudaFree(deviceData);
        // - (4) to host
        buffer.copyTo(hostData.data(), 5);
        CHECK_EQ(hostData, std::vector<int>({ 1, 2, 3, 4, 5 }));

        // Asynchronous
        // - (1) from host
        buffer.resizeAsync(5);
        buffer.copyFromAsync(std::vector<int>({ 6, 7, 8, 9, 10 }).data(), 5);
        // - (2) to device
        cudaMallocAsync(&deviceData, 5 * sizeof(int), 0);
        buffer.copyToAsync(deviceData, 5);
        // - (3) from device
        buffer.clear();
        CHECK_UNARY(buffer.data() == nullptr); // ensure buffer is cleared
        buffer.resizeAsync(5);
        buffer.copyFromAsync(deviceData, 5);
        cudaFreeAsync(deviceData, 0);
        // - (4) to host
        buffer.copyToAsync(hostData.data(), 5);
        CHECK_EQ(hostData, std::vector<int>({ 6, 7, 8, 9, 10 }));
    }

    /**
     * Resize test helper
     */
    void testResize(isaacsim::core::includes::GenericBufferBase<int> & buffer)
    {
        // Synchronous
        buffer.resize(3);
        CHECK_EQ(buffer.size(), 3);
        CHECK_UNARY(buffer.data() != nullptr);
        buffer.resize(0);
        CHECK_EQ(buffer.size(), 0);
        CHECK_UNARY(buffer.data() == nullptr);

        // Asynchronous
        buffer.resizeAsync(4);
        CHECK_EQ(buffer.size(), 4);
        CHECK_UNARY(buffer.data() != nullptr);
        buffer.resizeAsync(0);
        CHECK_EQ(buffer.size(), 0);
        CHECK_UNARY(buffer.data() == nullptr);
    }

    // --------------------------------------------------------------------

    TEST_CASE("GenericBufferBase")
    {
        // CPU
        isaacsim::core::includes::GenericBufferBase<int> cpuBuffer(0, -1);
        CHECK_EQ(cpuBuffer.type(), isaacsim::core::includes::MemoryType::eHost);
        CHECK_EQ(cpuBuffer.toString(), "Buffer([], device=<unregistered>)");
        // GPU
        isaacsim::core::includes::GenericBufferBase<int> gpuBuffer(0, 0);
        CHECK_EQ(gpuBuffer.type(), isaacsim::core::includes::MemoryType::eDevice);
        CHECK_EQ(gpuBuffer.toString(), "Buffer([], device=<unregistered>)");
    }

    TEST_CASE("GenericBufferBase::copy")
    {
        isaacsim::core::includes::GenericBufferBase<int> buffer(0);
        // CPU
        buffer.setDevice(-1);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eHost);
        testCopy(buffer);
        // GPU
        buffer.setDevice(0);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eDevice);
        testCopy(buffer);
    }

    TEST_CASE("GenericBufferBase::resize")
    {
        isaacsim::core::includes::GenericBufferBase<int> buffer(0);
        // CPU
        buffer.setDevice(-1);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eHost);
        testResize(buffer);
        // GPU
        buffer.setDevice(0);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eDevice);
        testResize(buffer);
    }

    TEST_CASE("GenericBufferBase::setDevice")
    {
        isaacsim::core::includes::GenericBufferBase<float> buffer(3);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eHost);

        // Change device (keep data)
        // - Initialize buffer on CPU
        buffer.copyFrom(std::vector<float>({ 0.1f, 0.2f, 0.3f }).data(), 3);
        CHECK_EQ(buffer.toString(), "Buffer([0.1, 0.2, 0.3], device='cpu')");
        // - CPU -> GPU
        buffer.setDevice(0, true);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eDevice);
        CHECK_EQ(buffer.toString(), "Buffer([0.1, 0.2, 0.3], device='cuda:0')");
        // - GPU -> CPU
        buffer.setDevice(-1, true);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eHost);
        CHECK_EQ(buffer.toString(), "Buffer([0.1, 0.2, 0.3], device='cpu')");

        // Change device (don't keep data)
        // - Initialize buffer on GPU
        buffer.copyFrom(std::vector<float>({ 0.4f, 0.5f, 0.6f }).data(), 3);
        CHECK_EQ(buffer.toString(), "Buffer([0.4, 0.5, 0.6], device='cpu')");
        // - CPU -> GPU
        buffer.setDevice(0, false);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eDevice);
        CHECK_EQ(buffer.toString(), "Buffer([0, 0, 0], device='cuda:0')");
        // - Initialize buffer on GPU
        buffer.copyFrom(std::vector<float>({ 0.6f, 0.7f, 0.8f }).data(), 3);
        CHECK_EQ(buffer.toString(), "Buffer([0.6, 0.7, 0.8], device='cuda:0')");
        // - GPU -> CPU
        buffer.setDevice(-1, false);
        CHECK_EQ(buffer.type(), isaacsim::core::includes::MemoryType::eHost);
        CHECK_EQ(buffer.toString(), "Buffer([0, 0, 0], device='cpu')");
    }

    TEST_CASE("GenericBufferBase::fill")
    {
        // Type: bool
        isaacsim::core::includes::GenericBufferBase<bool> bufferBool(2);
        // - Synchronous
        bufferBool.setDevice(-1);
        bufferBool.fill(true);
        CHECK_EQ(bufferBool.toString(), "Buffer([1, 1], device='cpu')");
        bufferBool.setDevice(0);
        bufferBool.fill(false);
        CHECK_EQ(bufferBool.toString(), "Buffer([0, 0], device='cuda:0')");
        // - Asynchronous
        bufferBool.setDevice(-1);
        bufferBool.fillAsync(true);
        CHECK_EQ(bufferBool.toString(), "Buffer([1, 1], device='cpu')");
        bufferBool.setDevice(0);
        bufferBool.fillAsync(false);
        CHECK_EQ(bufferBool.toString(), "Buffer([0, 0], device='cuda:0')");

        // Type: int
        isaacsim::core::includes::GenericBufferBase<int> bufferInt(3);
        // - Synchronous
        bufferInt.setDevice(-1);
        bufferInt.fill(1);
        CHECK_EQ(bufferInt.toString(), "Buffer([1, 1, 1], device='cpu')");
        bufferInt.setDevice(0);
        bufferInt.fill(-2);
        CHECK_EQ(bufferInt.toString(), "Buffer([-2, -2, -2], device='cuda:0')");
        // - Asynchronous
        bufferInt.setDevice(-1);
        bufferInt.fillAsync(1);
        CHECK_EQ(bufferInt.toString(), "Buffer([1, 1, 1], device='cpu')");
        bufferInt.setDevice(0);
        bufferInt.fillAsync(-2);
        CHECK_EQ(bufferInt.toString(), "Buffer([-2, -2, -2], device='cuda:0')");

        // Type: float
        isaacsim::core::includes::GenericBufferBase<float> bufferFloat(4);
        // - Synchronous
        bufferFloat.setDevice(-1);
        bufferFloat.fill(0.25f);
        CHECK_EQ(bufferFloat.toString(), "Buffer([0.25, 0.25, 0.25, 0.25], device='cpu')");
        bufferFloat.setDevice(0);
        bufferFloat.fill(-3.14f);
        CHECK_EQ(bufferFloat.toString(), "Buffer([-3.14, -3.14, -3.14, -3.14], device='cuda:0')");
        // - Asynchronous
        bufferFloat.setDevice(-1);
        bufferFloat.fillAsync(0.25f);
        CHECK_EQ(bufferFloat.toString(), "Buffer([0.25, 0.25, 0.25, 0.25], device='cpu')");
        bufferFloat.setDevice(0);
        bufferFloat.fillAsync(-3.14f);
        CHECK_EQ(bufferFloat.toString(), "Buffer([-3.14, -3.14, -3.14, -3.14], device='cuda:0')");
    }
}
