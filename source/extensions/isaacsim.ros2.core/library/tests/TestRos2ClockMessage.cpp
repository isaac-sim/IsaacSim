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

TEST_SUITE("isaacsim.ros2.core.clock_message_tests")
{
    TEST_CASE("Ros2ClockMessage: creation and validation")
    {
        ROS2_TEST_SETUP();

        // Create clock message
        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Check that message data and type support are valid
        CHECK(clockMsg != nullptr);
        CHECK(clockMsg->getPtr() != nullptr);
        CHECK(clockMsg->getTypeSupportHandle() != nullptr);
    }

    TEST_CASE("Ros2ClockMessage: write timestamp data")
    {
        ROS2_TEST_SETUP();

        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Test writing various timestamps
        SUBCASE("Positive timestamp")
        {
            double timestamp = 123.456789;
            clockMsg->writeData(timestamp);
            // No crash means success
            CHECK(true);
        }

        SUBCASE("Zero timestamp")
        {
            double timestamp = 0.0;
            clockMsg->writeData(timestamp);
            CHECK(true);
        }

        SUBCASE("Large timestamp")
        {
            double timestamp = 1e9; // 1 billion seconds
            clockMsg->writeData(timestamp);
            CHECK(true);
        }

        SUBCASE("Fractional timestamp")
        {
            double timestamp = 0.123456789;
            clockMsg->writeData(timestamp);
            CHECK(true);
        }

        SUBCASE("Negative timestamp")
        {
            double timestamp = -123.456;
            clockMsg->writeData(timestamp);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2ClockMessage: message type support")
    {
        ROS2_TEST_SETUP();

        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Get type support
        auto typeSupport = clockMsg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2ClockMessage: multiple instances")
    {
        ROS2_TEST_SETUP();

        // Create multiple clock messages
        auto clockMsg1 = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg1 != nullptr);

        auto clockMsg2 = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg2 != nullptr);

        auto clockMsg3 = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg3 != nullptr);

        // Each should have its own data pointer
        CHECK(clockMsg1->getPtr() != clockMsg2->getPtr());
        CHECK(clockMsg2->getPtr() != clockMsg3->getPtr());
        CHECK(clockMsg1->getPtr() != clockMsg3->getPtr());

        // Write different data to each
        clockMsg1->writeData(1.0);
        clockMsg2->writeData(2.0);
        clockMsg3->writeData(3.0);

        // All should still be valid
        CHECK(clockMsg1 != nullptr);
        CHECK(clockMsg2 != nullptr);
        CHECK(clockMsg3 != nullptr);
    }

    TEST_CASE("Ros2ClockMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("clock_pub_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Create clock message
        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Create publisher for clock messages
        isaacsim::ros2::core::Ros2QoSProfile qos; // Default constructor gives default values
        auto pub = testBase.getFactory()->createPublisher(node.get(), "/clock", clockMsg->getTypeSupportHandle(), qos);
        CHECK(pub != nullptr);
        CHECK(pub->isValid());

        // Write and publish multiple clock messages
        for (int i = 0; i < 5; ++i)
        {
            double timestamp = i * 0.1;
            clockMsg->writeData(timestamp);
            pub->publish(clockMsg->getPtr());
        }

        CHECK(ctx->shutdown("test-clock-publisher"));
    }
}
