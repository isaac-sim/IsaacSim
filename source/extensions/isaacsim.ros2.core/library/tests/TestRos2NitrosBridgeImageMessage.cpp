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

#include "TestBase.h"

#include <doctest/doctest.h>
#include <isaacsim/ros2/core/Ros2Factory.h>

#include <vector>

#if !defined(_WIN32)
TEST_SUITE("isaacsim.ros2.core.nitros_bridge_image_message_tests")
{
    TEST_CASE("Ros2NitrosBridgeImageMessage: creation and basic functionality")
    {
        ROS2_TEST_SETUP();

        // Create a NitrosBridgeImage message
        auto message = testBase.getFactory()->createNitrosBridgeImageMessage();
        REQUIRE(message);

        // Test type support
        auto typeSupport = message->getTypeSupportHandle();
        REQUIRE(typeSupport != nullptr);

        // Test message data pointer
        auto msgPtr = message->getPtr();
        REQUIRE(msgPtr != nullptr);
    }

    TEST_CASE("Ros2NitrosBridgeImageMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createNitrosBridgeImageMessage();
        REQUIRE(message);

        // Write header
        double timeStamp = 123.456;
        std::string frameId = "camera_frame";
        REQUIRE_NOTHROW(message->writeHeader(timeStamp, frameId));
    }

    TEST_CASE("Ros2NitrosBridgeImageMessage: generate buffer")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createNitrosBridgeImageMessage();
        REQUIRE(message);

        // Generate buffer (this only computes fields, no memory allocation)
        uint32_t height = 480;
        uint32_t width = 640;
        std::string encoding = "rgb8";
        REQUIRE_NOTHROW(message->generateBuffer(height, width, encoding));
    }

    TEST_CASE("Ros2NitrosBridgeImageMessage: write data")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createNitrosBridgeImageMessage();
        REQUIRE(message);

        // Write IPC data (process ID and CUDA memory file descriptor)
        std::vector<int32_t> ipcData = { 12345, 67890 }; // Mock process ID and file descriptor
        REQUIRE_NOTHROW(message->writeData(ipcData));
    }

    TEST_CASE("Ros2NitrosBridgeImageMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context);
        REQUIRE_NOTHROW(context->init(0, nullptr));

        auto node = testBase.getFactory()->createNodeHandle("test_nitros_image_node", "test", context.get());
        REQUIRE(node);

        // Create message
        auto message = testBase.getFactory()->createNitrosBridgeImageMessage();
        REQUIRE(message);

        // Set up QoS profile
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create publisher
        auto publisher = testBase.getFactory()->createPublisher(
            node.get(), "/test_nitros_image", message->getTypeSupportHandle(), qos);
        REQUIRE(publisher);

        // Write message data
        message->writeHeader(123.456, "camera_frame");
        message->generateBuffer(480, 640, "rgb8");
        std::vector<int32_t> ipcData = { 12345, 67890 };
        message->writeData(ipcData);

        // Publish the message
        REQUIRE_NOTHROW(publisher->publish(message->getPtr()));

        // Clean up
        REQUIRE_NOTHROW(context->shutdown());
    }
}
#endif // !defined(_WIN32)
