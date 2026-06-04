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

/*
Test is implemented using the doctest C++ testing framework:
  https://github.com/doctest/doctest/blob/master/doc/markdown/readme.md
*/

#include "dlpack/dlpack.h"
#include "hololink/emulation/emulator_utils.hpp"
#include "hololink/emulation/hsb_config.hpp"
#include "hololink/emulation/linux_data_plane.hpp"

#include <doctest/doctest.h>

#include <memory>
#include <string>

using namespace hololink::emulation;

TEST_SUITE("HSBEmulatorIntegration")
{
    TEST_CASE("HSBEmulator - Headers and libraries link successfully")
    {
        // Test that we can include all the necessary headers
        // This verifies that headers are found and libraries can be linked

        // Create a simple DLTensor on the stack with default values
        int64_t shape[1] = { 1024 }; // Simple frame size
        DLTensor tensor = { .device = { .device_type = DLDeviceType::kDLCPU, .device_id = 0 },
                            .ndim = 1,
                            .dtype = DLDataType{ .code = DLDataTypeCode::kDLUInt, .bits = 8, .lanes = 1 },
                            .shape = &shape[0] };

        // Verify tensor configuration
        CHECK(tensor.device.device_type == DLDeviceType::kDLCPU);
        CHECK(tensor.shape[0] == 1024);
        CHECK(tensor.ndim == 1);
        CHECK(tensor.dtype.bits == 8);

        // If we get here, all headers were found and libraries linked successfully
        CHECK(true);
    }

    TEST_CASE("HSBEmulator - DLTensor configuration")
    {
        // Test various DLTensor configurations

        // CPU tensor
        int64_t cpu_shape[1] = { 1024 };
        DLTensor cpu_tensor = { .device = { .device_type = DLDeviceType::kDLCPU, .device_id = 0 },
                                .ndim = 1,
                                .dtype = DLDataType{ .code = DLDataTypeCode::kDLUInt, .bits = 8, .lanes = 1 },
                                .shape = &cpu_shape[0] };

        CHECK(cpu_tensor.device.device_type == DLDeviceType::kDLCPU);
        CHECK(cpu_tensor.shape[0] == 1024);

        // GPU tensor
        int64_t gpu_shape[1] = { 2048 };
        DLTensor gpu_tensor = { .device = { .device_type = DLDeviceType::kDLCUDA, .device_id = 0 },
                                .ndim = 1,
                                .dtype = DLDataType{ .code = DLDataTypeCode::kDLUInt, .bits = 8, .lanes = 1 },
                                .shape = &gpu_shape[0] };

        CHECK(gpu_tensor.device.device_type == DLDeviceType::kDLCUDA);
        CHECK(gpu_tensor.shape[0] == 2048);
    }

    TEST_CASE("HSBEmulator - Type definitions exist")
    {
        // Verify that key HSB emulator types can be used
        // This confirms headers are properly included

        // Test that we can reference the HSBEmulator type
        using EmulatorType = HSBEmulator;

        // Test that we can reference IPAddress_from_string function
        using IPAddressFunc = decltype(&IPAddress_from_string);

        // Simple compile-time checks
        CHECK(sizeof(EmulatorType) > 0);
        CHECK(sizeof(IPAddressFunc) > 0);
    }
}
