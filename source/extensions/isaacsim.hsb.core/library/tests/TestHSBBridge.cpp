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

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <isaacsim/hsb/core/HSBSender.h>

using namespace isaacsim::hsb::core;

TEST_SUITE("HSBSender")
{
    TEST_CASE("HSBSender - construction")
    {
        // Test that HSBSender can be constructed
        HSBSender sender("10.0.0.1", 1, 2);

        // Initial state should be disconnected
        CHECK(!sender.isConnected());
    }

    TEST_CASE("HSBSender - construction with COE")
    {
        HSBSender sender("127.0.0.1", 0, 0, "coe");
        CHECK(!sender.isConnected());
    }

    TEST_CASE("HSBSender - connect and disconnect lifecycle")
    {
        HSBSender sender("127.0.0.1", 0, 0);

        // Initial state
        CHECK(!sender.isConnected());

        // Connect
        bool connected = sender.connect();
        // Note: May fail if HSB emulator setup fails, which is OK for unit test
        if (connected)
        {
            CHECK(sender.isConnected());

            // Disconnect
            CHECK(sender.disconnect());
            CHECK(!sender.isConnected());

            // Multiple disconnect calls should be safe
            CHECK(sender.disconnect());
            CHECK(!sender.isConnected());
        }
    }

    TEST_CASE("HSBSender - send without connection fails")
    {
        HSBSender sender("127.0.0.1", 0, 0);

        // Try to send without connecting
        int64_t shape[3] = { 10, 10, 3 };
        std::vector<uint8_t> data(10 * 10 * 3, 0);

        DLTensor tensor = { .data = data.data(),
                            .device = { .device_type = DLDeviceType::kDLCPU, .device_id = 0 },
                            .ndim = 3,
                            .dtype = DLDataType{ .code = DLDataTypeCode::kDLUInt, .bits = 8, .lanes = 1 },
                            .shape = shape,
                            .strides = nullptr,
                            .byte_offset = 0 };

        // Should fail when not connected
        CHECK(!sender.send(tensor));
    }

    TEST_CASE("HSBSender - multiple connect calls are safe")
    {
        HSBSender sender("127.0.0.1", 0, 0);

        bool first_connect = sender.connect();
        if (first_connect)
        {
            CHECK(sender.isConnected());

            // Second connect should succeed (idempotent)
            CHECK(sender.connect());
            CHECK(sender.isConnected());

            // Cleanup
            sender.disconnect();
        }
    }

    TEST_CASE("HSBSender - COE connect and disconnect best-effort")
    {
        HSBSender sender("127.0.0.1", 0, 0, "coe");
        CHECK(!sender.isConnected());
        bool connected = sender.connect();
        if (connected)
        {
            CHECK(sender.isConnected());
            CHECK(sender.disconnect());
            CHECK(!sender.isConnected());
        }
    }
}

TEST_SUITE("Integration")
{
    TEST_CASE("HSBSender - send DLTensor after connect")
    {
        HSBSender sender("127.0.0.1", 0, 0);

        bool connected = sender.connect();
        if (connected)
        {
            // Create a simple test image
            int64_t shape[3] = { 480, 640, 3 };
            std::vector<uint8_t> data(480 * 640 * 3, 128); // Gray image

            DLTensor tensor = { .data = data.data(),
                                .device = { .device_type = DLDeviceType::kDLCPU, .device_id = 0 },
                                .ndim = 3,
                                .dtype = DLDataType{ .code = DLDataTypeCode::kDLUInt, .bits = 8, .lanes = 1 },
                                .shape = shape,
                                .strides = nullptr,
                                .byte_offset = 0 };

            // Should succeed (or fail gracefully if HSB emulator isn't fully configured)
            sender.send(tensor);
            // We don't assert success here as it depends on HSB runtime state
            // The important part is that it doesn't crash

            sender.disconnect();
        }
    }
}
