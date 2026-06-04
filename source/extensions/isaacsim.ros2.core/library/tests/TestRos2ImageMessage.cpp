// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "TestBase.h"

#include <carb/BindingsUtils.h>

#include <doctest/doctest.h>

#include <memory>
#include <string>

// CARB_BINDINGS moved to first test file

TEST_SUITE("isaacsim.ros2.core.image_message_tests")
{
    TEST_CASE("Ros2ImageMessage: creation and validation")
    {
        ROS2_TEST_SETUP();

        // Create image message
        auto imageMsg = testBase.getFactory()->createImageMessage();
        CHECK(imageMsg != nullptr);

        // Check that message data and type support are valid
        CHECK(imageMsg != nullptr);
        CHECK(imageMsg->getPtr() != nullptr);
        CHECK(imageMsg->getTypeSupportHandle() != nullptr);
    }

    TEST_CASE("Ros2ImageMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto imageMsg = testBase.getFactory()->createImageMessage();
        CHECK(imageMsg != nullptr);

        SUBCASE("Normal header")
        {
            double timestamp = 123.456;
            std::string frameId = "camera_optical_frame";
            imageMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }

        SUBCASE("Empty frame ID")
        {
            double timestamp = 456.789;
            std::string frameId = "";
            imageMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }

        SUBCASE("Nested frame ID")
        {
            double timestamp = 789.012;
            std::string frameId = "robot/camera/optical_frame";
            imageMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2ImageMessage: generate buffer for different encodings")
    {
        ROS2_TEST_SETUP();

        auto imageMsg = testBase.getFactory()->createImageMessage();
        CHECK(imageMsg != nullptr);

        SUBCASE("RGB8 encoding")
        {
            uint32_t height = 480;
            uint32_t width = 640;
            std::string encoding = "rgb8";
            imageMsg->generateBuffer(height, width, encoding);

            // Check expected buffer size
            size_t expectedSize = height * width * 3; // 3 bytes per pixel
            CHECK(imageMsg->getTotalBytes() == expectedSize);
        }

        SUBCASE("RGBA8 encoding")
        {
            uint32_t height = 480;
            uint32_t width = 640;
            std::string encoding = "rgba8";
            imageMsg->generateBuffer(height, width, encoding);

            // Check expected buffer size
            size_t expectedSize = height * width * 4; // 4 bytes per pixel
            CHECK(imageMsg->getTotalBytes() == expectedSize);
        }

        SUBCASE("BGR8 encoding")
        {
            uint32_t height = 480;
            uint32_t width = 640;
            std::string encoding = "bgr8";
            imageMsg->generateBuffer(height, width, encoding);

            size_t expectedSize = height * width * 3;
            CHECK(imageMsg->getTotalBytes() == expectedSize);
        }

        SUBCASE("BGRA8 encoding")
        {
            uint32_t height = 480;
            uint32_t width = 640;
            std::string encoding = "bgra8";
            imageMsg->generateBuffer(height, width, encoding);

            size_t expectedSize = height * width * 4;
            CHECK(imageMsg->getTotalBytes() == expectedSize);
        }

        SUBCASE("32FC1 encoding (depth)")
        {
            uint32_t height = 480;
            uint32_t width = 640;
            std::string encoding = "32FC1";
            imageMsg->generateBuffer(height, width, encoding);

            size_t expectedSize = height * width * 4; // 4 bytes per float
            CHECK(imageMsg->getTotalBytes() == expectedSize);
        }

        SUBCASE("16UC1 encoding")
        {
            uint32_t height = 480;
            uint32_t width = 640;
            std::string encoding = "16UC1";
            imageMsg->generateBuffer(height, width, encoding);

            size_t expectedSize = height * width * 2; // 2 bytes per pixel
            CHECK(imageMsg->getTotalBytes() == expectedSize);
        }

        SUBCASE("Mono8 encoding")
        {
            uint32_t height = 480;
            uint32_t width = 640;
            std::string encoding = "mono8";
            imageMsg->generateBuffer(height, width, encoding);

            size_t expectedSize = height * width * 1; // 1 byte per pixel
            CHECK(imageMsg->getTotalBytes() == expectedSize);
        }
    }

    TEST_CASE("Ros2ImageMessage: different resolutions")
    {
        ROS2_TEST_SETUP();

        auto imageMsg = testBase.getFactory()->createImageMessage();
        CHECK(imageMsg != nullptr);

        std::string encoding = "rgb8";

        SUBCASE("Small resolution")
        {
            imageMsg->generateBuffer(240, 320, encoding);
            CHECK(imageMsg->getTotalBytes() == 240 * 320 * 3);
        }

        SUBCASE("HD resolution")
        {
            imageMsg->generateBuffer(720, 1280, encoding);
            CHECK(imageMsg->getTotalBytes() == 720 * 1280 * 3);
        }

        SUBCASE("Full HD resolution")
        {
            imageMsg->generateBuffer(1080, 1920, encoding);
            CHECK(imageMsg->getTotalBytes() == 1080 * 1920 * 3);
        }

        SUBCASE("Square resolution")
        {
            imageMsg->generateBuffer(512, 512, encoding);
            CHECK(imageMsg->getTotalBytes() == 512 * 512 * 3);
        }

        SUBCASE("Very small resolution")
        {
            imageMsg->generateBuffer(10, 10, encoding);
            CHECK(imageMsg->getTotalBytes() == 10 * 10 * 3);
        }
    }

    TEST_CASE("Ros2ImageMessage: buffer pointers")
    {
        ROS2_TEST_SETUP();

        auto imageMsg = testBase.getFactory()->createImageMessage();
        CHECK(imageMsg != nullptr);

        // Generate buffer
        uint32_t height = 100;
        uint32_t width = 200;
        std::string encoding = "rgb8";
        imageMsg->generateBuffer(height, width, encoding);

        // Get buffer pointer
        auto bufferPtr = imageMsg->getBufferPtr();
        CHECK(bufferPtr != nullptr);

        // Check buffer size
        CHECK(imageMsg->getTotalBytes() == height * width * 3);
    }

    TEST_CASE("Ros2ImageMessage: complete image message")
    {
        ROS2_TEST_SETUP();

        auto imageMsg = testBase.getFactory()->createImageMessage();
        CHECK(imageMsg != nullptr);

        // Write complete image message
        double timestamp = 100.5;
        std::string frameId = "camera_color_optical_frame";
        imageMsg->writeHeader(timestamp, frameId);

        uint32_t height = 480;
        uint32_t width = 640;
        std::string encoding = "rgb8";
        imageMsg->generateBuffer(height, width, encoding);

        CHECK(imageMsg != nullptr);
        CHECK(imageMsg->getTotalBytes() == height * width * 3);
    }

    TEST_CASE("Ros2ImageMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("image_pub_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Create image message
        auto imageMsg = testBase.getFactory()->createImageMessage();
        CHECK(imageMsg != nullptr);

        // Create publisher for image messages
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eBestEffort;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 5;
        auto pub = testBase.getFactory()->createPublisher(
            node.get(), "/camera/image_raw", imageMsg->getTypeSupportHandle(), qos);
        CHECK(pub != nullptr);
        CHECK(pub->isValid());

        // Publish image data
        double timestamp = 123.456;
        std::string frameId = "camera_frame";
        imageMsg->writeHeader(timestamp, frameId);

        uint32_t height = 240;
        uint32_t width = 320;
        std::string encoding = "rgb8";
        imageMsg->generateBuffer(height, width, encoding);

        pub->publish(imageMsg->getPtr());

        CHECK(ctx->shutdown("test-image-publisher"));
    }
}
