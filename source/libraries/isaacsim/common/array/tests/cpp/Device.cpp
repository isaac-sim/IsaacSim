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

#include "details/CudaRuntime.hpp"

#include <doctest/doctest.h>
#include <isaacsim/common/array/Device.hpp>
#include <isaacsim/common/exceptions/Exceptions.hpp>

#include <stdexcept>

using namespace isaacsim::common::array;

TEST_SUITE("Device")
{

#if defined(_WIN32)
    TEST_CASE("CudaRuntime loads the pinned Windows runtime")
    {
        CHECK_UNARY(CudaRuntime::getInstance().isLoaded());
    }
#endif

    TEST_CASE("Device::Device()")
    {
        SUBCASE("cpu")
        {
            Device device("cpu");
            CHECK_EQ(device.ordinal(), -1);
            CHECK_EQ(device.toString(), "cpu");
            CHECK_UNARY_FALSE(device.isCuda());
            CHECK_UNARY(device.isCpu());
        }

        SUBCASE("cuda")
        {
            Device device("cuda");
            CHECK_EQ(device.ordinal(), 0);
            CHECK_EQ(device.toString(), "cuda:0");
            CHECK_UNARY(device.isCuda());
            CHECK_UNARY_FALSE(device.isCpu());
        }

        SUBCASE("cuda:X")
        {
            Device device("cuda:12345");
            CHECK_EQ(device.ordinal(), 12345);
            CHECK_EQ(device.toString(), "cuda:12345");
            CHECK_UNARY(device.isCuda());
            CHECK_UNARY_FALSE(device.isCpu());
        }

        SUBCASE("Equality")
        {
            CHECK_EQ(Device("cpu"), Device("cpu"));
            CHECK_EQ(Device("cuda:0"), Device("cuda"));
            CHECK_NE(Device("cpu"), Device("cuda:0"));
            CHECK_NE(Device("cuda:0"), Device("cuda:1"));
        }

        SUBCASE("Factory methods")
        {
            Device cpu = Device::Cpu();
            CHECK_EQ(cpu.ordinal(), -1);
            CHECK_UNARY(cpu.isCpu());
            CHECK_EQ(cpu, Device("cpu"));

            Device cuda0 = Device::Cuda();
            CHECK_EQ(cuda0.ordinal(), 0);
            CHECK_UNARY(cuda0.isCuda());
            CHECK_EQ(cuda0, Device("cuda:0"));

            Device cuda2 = Device::Cuda(2);
            CHECK_EQ(cuda2.ordinal(), 2);
            CHECK_UNARY(cuda2.isCuda());

            CHECK_THROWS_AS(Device::Cuda(-1), std::invalid_argument);
        }

        SUBCASE("Exceptions")
        {
            CHECK_THROWS_AS(Device("cpu:0"), std::invalid_argument);
            CHECK_THROWS_AS(Device("cuda:-1"), std::invalid_argument);
            CHECK_THROWS_AS(Device("cuda:abc"), std::invalid_argument);
            CHECK_THROWS_AS(Device("unknown"), std::invalid_argument);
        }
    }

    TEST_CASE("Device::isAvailable()")
    {
        SUBCASE("CPU is always available")
        {
            CHECK_UNARY(Device::Cpu().isAvailable());
        }

        SUBCASE("CUDA(0) availability matches runtime")
        {
            bool hasGpu = CudaRuntime::getInstance(/*throwIfInvalid=*/false).isLoaded() &&
                          CudaRuntime::getInstance(/*throwIfInvalid=*/false).cudaGetDeviceCount() > 0;
            CHECK_EQ(Device::Cuda(0).isAvailable(), hasGpu);
        }

        SUBCASE("CUDA ordinal at device count is unavailable")
        {
            int count = CudaRuntime::getInstance(/*throwIfInvalid=*/false).cudaGetDeviceCount();
            CHECK_UNARY_FALSE(Device::Cuda(count).isAvailable()); // count is always one past the last device
            for (int i = 0; i < count; ++i)
            {
                CHECK_UNARY(Device::Cuda(i).isAvailable());
            }
        }
    }

    TEST_CASE("DeviceGuard::DeviceGuard()")
    {
        auto& runtime = CudaRuntime::getInstance(/*throwIfInvalid=*/false);
        const size_t deviceCount = runtime.cudaGetDeviceCount();
        const bool hasGpu = runtime.isLoaded() && deviceCount > 0;

        SUBCASE("Throws when the runtime is unavailable")
        {
            if (!hasGpu)
            {
                CHECK_THROWS_AS(DeviceGuard(0), isaacsim::common::exceptions::CudaRuntimeError);
            }
        }

        SUBCASE("Leaves the active device unchanged after the scope")
        {
            if (hasGpu)
            {
                CHECK_UNARY(runtime.cudaSetDevice(0));
                {
                    DeviceGuard guard(deviceCount - 1);
                    int current = -1;
                    CHECK_UNARY(runtime.cudaGetDevice(&current));
                    CHECK_EQ(current, deviceCount - 1);
                }
                int restored = -1;
                CHECK_UNARY(runtime.cudaGetDevice(&restored));
                CHECK_EQ(restored, 0);
            }
        }
    }
}
