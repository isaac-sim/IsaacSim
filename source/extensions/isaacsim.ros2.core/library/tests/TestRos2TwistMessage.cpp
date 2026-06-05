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
#include <pxr/base/gf/vec3d.h>

TEST_SUITE("isaacsim.ros2.core.twist_message_tests")
{
    TEST_CASE("Ros2TwistMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTwistMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2TwistMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTwistMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2TwistMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTwistMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2TwistMessage: read data from fresh message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createTwistMessage();
        REQUIRE(msg != nullptr);

        pxr::GfVec3d linearVelocity;
        pxr::GfVec3d angularVelocity;

        // Should not crash reading from fresh message
        CHECK_NOTHROW(msg->readData(linearVelocity, angularVelocity));

        // Fresh message should have zero velocities
        CHECK(linearVelocity[0] == 0.0);
        CHECK(linearVelocity[1] == 0.0);
        CHECK(linearVelocity[2] == 0.0);
        CHECK(angularVelocity[0] == 0.0);
        CHECK(angularVelocity[1] == 0.0);
        CHECK(angularVelocity[2] == 0.0);
    }

    TEST_CASE("Ros2TwistMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context != nullptr);
        context->init(0, nullptr);

        auto node = testBase.getFactory()->createNodeHandle("test_twist_node", "test", context.get());
        REQUIRE(node != nullptr);

        auto msg = testBase.getFactory()->createTwistMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile appropriate for velocity commands
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create publisher - should not crash
        auto publisher = testBase.getFactory()->createPublisher(node.get(), "cmd_vel", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Read current data and publish - should not crash
        pxr::GfVec3d linearVelocity;
        pxr::GfVec3d angularVelocity;
        CHECK_NOTHROW(msg->readData(linearVelocity, angularVelocity));

        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }

    TEST_CASE("Ros2TwistMessage: use with subscriber")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context != nullptr);
        context->init(0, nullptr);

        auto node = testBase.getFactory()->createNodeHandle("test_twist_sub_node", "test", context.get());
        REQUIRE(node != nullptr);

        auto msg = testBase.getFactory()->createTwistMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create subscriber - should not crash
        auto subscriber =
            testBase.getFactory()->createSubscriber(node.get(), "cmd_vel", msg->getTypeSupportHandle(), qos);
        CHECK(subscriber != nullptr);

        // Try to spin (likely no messages available, but should not crash)
        if (subscriber)
        {
            CHECK_NOTHROW(subscriber->spin(msg->getPtr()));
        }
    }
}
