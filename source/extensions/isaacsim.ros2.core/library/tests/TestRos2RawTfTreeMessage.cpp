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
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/vec3d.h>

TEST_SUITE("isaacsim.ros2.core.raw_tf_tree_message_tests")
{
    TEST_CASE("Ros2RawTfTreeMessage: creation and basic functionality")
    {
        ROS2_TEST_SETUP();

        // Create a RawTfTree message
        auto message = testBase.getFactory()->createRawTfTreeMessage();
        REQUIRE(message);

        // Test type support
        auto typeSupport = message->getTypeSupportHandle();
        REQUIRE(typeSupport != nullptr);

        // Test message data pointer
        auto msgPtr = message->getPtr();
        REQUIRE(msgPtr != nullptr);
    }

    TEST_CASE("Ros2RawTfTreeMessage: write transform data")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createRawTfTreeMessage();
        REQUIRE(message);

        // Write transform data
        double timeStamp = 123.456;
        std::string frameId = "base_link";
        std::string childFrame = "camera_link";
        pxr::GfVec3d translation(1.0, 2.0, 3.0);
        pxr::GfQuatd rotation(1.0, 0.0, 0.0, 0.0); // Identity quaternion (w, x, y, z)

        REQUIRE_NOTHROW(message->writeData(timeStamp, frameId, childFrame, translation, rotation));
    }

    TEST_CASE("Ros2RawTfTreeMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context);
        REQUIRE_NOTHROW(context->init(0, nullptr));

        auto node = testBase.getFactory()->createNodeHandle("test_raw_tf_tree_node", "test", context.get());
        REQUIRE(node);

        // Create message
        auto message = testBase.getFactory()->createRawTfTreeMessage();
        REQUIRE(message);

        // Set up QoS profile
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create publisher
        auto publisher = testBase.getFactory()->createPublisher(node.get(), "/tf", message->getTypeSupportHandle(), qos);
        REQUIRE(publisher);

        // Write message data
        double timeStamp = 123.456;
        std::string frameId = "world";
        std::string childFrame = "base_link";
        pxr::GfVec3d translation(1.5, 2.5, 0.0);
        pxr::GfQuatd rotation(0.7071, 0.0, 0.0, 0.7071); // 90 degree rotation around Z-axis

        message->writeData(timeStamp, frameId, childFrame, translation, rotation);

        // Publish the message
        REQUIRE_NOTHROW(publisher->publish(message->getPtr()));

        // Clean up
        REQUIRE_NOTHROW(context->shutdown());
    }

    TEST_CASE("Ros2RawTfTreeMessage: multiple transforms")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createRawTfTreeMessage();
        REQUIRE(message);

        // Test writing multiple transforms (should overwrite the single transform)
        double timeStamp1 = 100.0;
        pxr::GfVec3d translation1(1.0, 0.0, 0.0);
        pxr::GfQuatd rotation1(1.0, 0.0, 0.0, 0.0);

        double timeStamp2 = 200.0;
        pxr::GfVec3d translation2(0.0, 1.0, 0.0);
        pxr::GfQuatd rotation2(0.7071, 0.7071, 0.0, 0.0);

        REQUIRE_NOTHROW(message->writeData(timeStamp1, "frame1", "child1", translation1, rotation1));
        REQUIRE_NOTHROW(message->writeData(timeStamp2, "frame2", "child2", translation2, rotation2));
    }
}
