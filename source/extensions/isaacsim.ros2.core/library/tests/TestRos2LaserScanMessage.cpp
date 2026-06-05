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

#include "TestBase.h"

#include <pxr/base/gf/vec2f.h>

TEST_SUITE("isaacsim.ros2.core.laser_scan_message_tests")
{
    TEST_CASE("Ros2LaserScanMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2LaserScanMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2LaserScanMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2LaserScanMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "laser_frame";

        // Should not crash when writing header
        CHECK_NOTHROW(msg->writeHeader(timestamp, frameId));
    }

    TEST_CASE("Ros2LaserScanMessage: generate buffers")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        size_t buffSize = 360; // 360 range measurements (1 degree resolution)

        // Should not crash when generating buffers
        CHECK_NOTHROW(msg->generateBuffers(buffSize));
    }

    TEST_CASE("Ros2LaserScanMessage: write data")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        // Typical 2D lidar parameters
        pxr::GfVec2f azimuthRange(-180.0f, 180.0f); // Full 360 degree scan
        float rotationRate = 10.0f; // 10 Hz
        pxr::GfVec2f depthRange(0.1f, 30.0f); // 10cm to 30m range
        float horizontalResolution = 1.0f; // 1 degree resolution
        float horizontalFov = 360.0f; // Full 360 degree FOV

        // Should not crash when writing data
        CHECK_NOTHROW(msg->writeData(azimuthRange, rotationRate, depthRange, horizontalResolution, horizontalFov));
    }

    TEST_CASE("Ros2LaserScanMessage: write data with limited FOV")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        // Limited FOV lidar parameters (like Sick LMS)
        pxr::GfVec2f azimuthRange(-90.0f, 90.0f); // 180 degree scan
        float rotationRate = 20.0f; // 20 Hz
        pxr::GfVec2f depthRange(0.05f, 50.0f); // 5cm to 50m range
        float horizontalResolution = 0.5f; // 0.5 degree resolution
        float horizontalFov = 180.0f; // 180 degree FOV

        // Should not crash with limited FOV parameters
        CHECK_NOTHROW(msg->writeData(azimuthRange, rotationRate, depthRange, horizontalResolution, horizontalFov));
    }

    TEST_CASE("Ros2LaserScanMessage: complete workflow")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        // Full workflow: header -> buffers -> data
        double timestamp = 1.0;
        std::string frameId = "laser_frame";
        size_t buffSize = 180; // 180 measurements
        pxr::GfVec2f azimuthRange(-90.0f, 90.0f);
        float rotationRate = 10.0f;
        pxr::GfVec2f depthRange(0.1f, 10.0f);
        float horizontalResolution = 1.0f;
        float horizontalFov = 180.0f;

        CHECK_NOTHROW(msg->writeHeader(timestamp, frameId));
        CHECK_NOTHROW(msg->generateBuffers(buffSize));
        CHECK_NOTHROW(msg->writeData(azimuthRange, rotationRate, depthRange, horizontalResolution, horizontalFov));
    }

    TEST_CASE("Ros2LaserScanMessage: use with publisher")
    {
        ROS2_TEST_SETUP_WITH_NODE("test_laser_scan_node");

        auto msg = testBase.getFactory()->createLaserScanMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile appropriate for laser scans using helper
        auto qos = testBase.createDefaultQoS();

        // Create publisher - should not crash
        auto publisher =
            testBase.getFactory()->createPublisher(testBase.getNode().get(), "scan", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Setup message and publish - should not crash
        msg->writeHeader(1.0, "laser_frame");
        msg->generateBuffers(360);
        pxr::GfVec2f azimuthRange(-180.0f, 180.0f);
        float rotationRate = 10.0f;
        pxr::GfVec2f depthRange(0.1f, 30.0f);
        msg->writeData(azimuthRange, rotationRate, depthRange, 1.0f, 360.0f);

        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }
}
