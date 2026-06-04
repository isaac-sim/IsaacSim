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

#include <doctest/doctest.h>

TEST_SUITE("isaacsim.ros2.core.bounding_box_3d_message_tests")
{
    TEST_CASE("Ros2BoundingBox3DMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox3DMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2BoundingBox3DMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox3DMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2BoundingBox3DMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox3DMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2BoundingBox3DMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox3DMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "camera_frame";

        // Should not crash when writing header
        CHECK_NOTHROW(msg->writeHeader(timestamp, frameId));
    }

    TEST_CASE("Ros2BoundingBox3DMessage: write bbox data")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox3DMessage();
        REQUIRE(msg != nullptr);

        // Create sample bounding box data (matching Bbox3DData structure)
        struct TestBbox3DData
        {
            uint32_t semanticId;
            float xMin;
            float yMin;
            float zMin;
            float xMax;
            float yMax;
            float zMax;
            float transform[16]; // GfMatrix4f as flat array
            float occlusionRatio;
        };

        TestBbox3DData bboxData[] = {
            { 1,
              -1.0f,
              -1.0f,
              -1.0f,
              1.0f,
              1.0f,
              1.0f,
              { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f },
              0.1f },
            { 2,
              0.0f,
              0.0f,
              0.0f,
              2.0f,
              2.0f,
              2.0f,
              { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f },
              0.05f }
        };

        // Should not crash when writing bbox data
        CHECK_NOTHROW(msg->writeBboxData(bboxData, 2));
    }

    TEST_CASE("Ros2BoundingBox3DMessage: write empty bbox data")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createBoundingBox3DMessage();
        REQUIRE(msg != nullptr);

        // Should not crash when writing empty bbox data
        CHECK_NOTHROW(msg->writeBboxData(nullptr, 0));
    }

    TEST_CASE("Ros2BoundingBox3DMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context != nullptr);
        context->init(0, nullptr);

        auto node = testBase.getFactory()->createNodeHandle("test_bbox3d_node", "test", context.get());
        REQUIRE(node != nullptr);

        auto msg = testBase.getFactory()->createBoundingBox3DMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create publisher - should not crash
        auto publisher =
            testBase.getFactory()->createPublisher(node.get(), "test_detections_3d", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Write some data and publish - should not crash
        msg->writeHeader(1.0, "camera_frame");
        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }
}
