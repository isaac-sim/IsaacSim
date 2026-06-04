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

TEST_SUITE("isaacsim.ros2.core.bounding_box_2d_message_tests")
{
    TEST_CASE("Ros2BoundingBox2DMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox2DMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2BoundingBox2DMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox2DMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2BoundingBox2DMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox2DMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2BoundingBox2DMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox2DMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "camera_frame";

        // Should not crash when writing header
        CHECK_NOTHROW(msg->writeHeader(timestamp, frameId));
    }

    TEST_CASE("Ros2BoundingBox2DMessage: write bbox data")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox2DMessage();
        REQUIRE(msg != nullptr);

        // Create sample bounding box data (matching Bbox2DData structure)
        struct TestBbox2DData
        {
            uint32_t semanticId;
            int32_t xMin;
            int32_t yMin;
            int32_t xMax;
            int32_t yMax;
            float occlusionRatio;
        };

        TestBbox2DData bboxData[] = { { 1, 10, 20, 100, 80, 0.1f }, { 2, 150, 30, 200, 90, 0.05f } };

        // Should not crash when writing bbox data
        CHECK_NOTHROW(msg->writeBboxData(bboxData, 2));
    }

    TEST_CASE("Ros2BoundingBox2DMessage: write empty bbox data")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox2DMessage();
        REQUIRE(msg != nullptr);

        // Should not crash when writing empty bbox data
        CHECK_NOTHROW(msg->writeBboxData(nullptr, 0));
    }

    TEST_CASE("Ros2BoundingBox2DMessage: use with publisher")
    {
        ROS2_TEST_SETUP_WITH_NODE("test_bbox2d_node");

        auto msg = testBase.getFactory()->createBoundingBox2DMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile using helper
        auto qos = testBase.createDefaultQoS();

        // Create publisher - should not crash
        auto publisher = testBase.getFactory()->createPublisher(
            testBase.getNode().get(), "test_detections", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Write some data and publish - should not crash
        msg->writeHeader(1.0, "camera_frame");
        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }
}
