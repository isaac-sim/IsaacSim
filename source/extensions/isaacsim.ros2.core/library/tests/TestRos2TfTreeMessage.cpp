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

TEST_SUITE("isaacsim.ros2.core.tf_tree_message_tests")
{
    TEST_CASE("Ros2TfTreeMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTfTreeMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2TfTreeMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTfTreeMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2TfTreeMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTfTreeMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2TfTreeMessage: write empty transforms")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTfTreeMessage();
        REQUIRE(msg != nullptr);

        std::vector<isaacsim::ros2::core::TfTransformStamped> transforms;
        double timestamp = 1.0;

        // Should not crash with empty transforms
        CHECK_NOTHROW(msg->writeData(timestamp, transforms));
    }

    TEST_CASE("Ros2TfTreeMessage: write single transform")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTfTreeMessage();
        REQUIRE(msg != nullptr);

        std::vector<isaacsim::ros2::core::TfTransformStamped> transforms(1);
        transforms[0].timeStamp = 1.0;
        transforms[0].parentFrame = "map";
        transforms[0].childFrame = "base_link";
        transforms[0].translationX = 1.0;
        transforms[0].translationY = 2.0;
        transforms[0].translationZ = 0.0;
        transforms[0].rotationX = 0.0;
        transforms[0].rotationY = 0.0;
        transforms[0].rotationZ = 0.0;
        transforms[0].rotationW = 1.0;

        double timestamp = 1.0;

        // Should not crash with single transform
        CHECK_NOTHROW(msg->writeData(timestamp, transforms));
    }

    TEST_CASE("Ros2TfTreeMessage: write multiple transforms")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTfTreeMessage();
        REQUIRE(msg != nullptr);

        std::vector<isaacsim::ros2::core::TfTransformStamped> transforms(3);

        // First transform: map -> odom
        transforms[0].timeStamp = 1.0;
        transforms[0].parentFrame = "map";
        transforms[0].childFrame = "odom";
        transforms[0].translationX = 0.0;
        transforms[0].translationY = 0.0;
        transforms[0].translationZ = 0.0;
        transforms[0].rotationX = 0.0;
        transforms[0].rotationY = 0.0;
        transforms[0].rotationZ = 0.0;
        transforms[0].rotationW = 1.0;

        // Second transform: odom -> base_link
        transforms[1].timeStamp = 1.0;
        transforms[1].parentFrame = "odom";
        transforms[1].childFrame = "base_link";
        transforms[1].translationX = 1.0;
        transforms[1].translationY = 0.5;
        transforms[1].translationZ = 0.0;
        transforms[1].rotationX = 0.0;
        transforms[1].rotationY = 0.0;
        transforms[1].rotationZ = 0.707;
        transforms[1].rotationW = 0.707; // 90 degree rotation

        // Third transform: base_link -> laser_frame
        transforms[2].timeStamp = 1.0;
        transforms[2].parentFrame = "base_link";
        transforms[2].childFrame = "laser_frame";
        transforms[2].translationX = 0.2;
        transforms[2].translationY = 0.0;
        transforms[2].translationZ = 0.1;
        transforms[2].rotationX = 0.0;
        transforms[2].rotationY = 0.0;
        transforms[2].rotationZ = 0.0;
        transforms[2].rotationW = 1.0;

        double timestamp = 1.0;

        // Should not crash with multiple transforms
        CHECK_NOTHROW(msg->writeData(timestamp, transforms));
    }

    TEST_CASE("Ros2TfTreeMessage: read data from empty message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTfTreeMessage();
        REQUIRE(msg != nullptr);

        std::vector<isaacsim::ros2::core::TfTransformStamped> transforms;

        // Should not crash reading from empty message
        CHECK_NOTHROW(msg->readData(transforms));

        // Should return empty vector
        CHECK(transforms.empty());
    }

    TEST_CASE("Ros2TfTreeMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context != nullptr);
        context->init(0, nullptr);

        auto node = testBase.getFactory()->createNodeHandle("test_tf_tree_node", "test", context.get());
        REQUIRE(node != nullptr);

        auto msg = testBase.getFactory()->createTfTreeMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile appropriate for TF
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 100; // TF typically needs larger buffer

        // Create publisher - should not crash
        auto publisher = testBase.getFactory()->createPublisher(node.get(), "/tf", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Setup message and publish - should not crash
        std::vector<isaacsim::ros2::core::TfTransformStamped> transforms(1);
        transforms[0].timeStamp = 1.0;
        transforms[0].parentFrame = "map";
        transforms[0].childFrame = "base_link";
        transforms[0].translationX = 0.0;
        transforms[0].translationY = 0.0;
        transforms[0].translationZ = 0.0;
        transforms[0].rotationX = 0.0;
        transforms[0].rotationY = 0.0;
        transforms[0].rotationZ = 0.0;
        transforms[0].rotationW = 1.0;

        CHECK_NOTHROW(msg->writeData(1.0, transforms));

        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }
}
