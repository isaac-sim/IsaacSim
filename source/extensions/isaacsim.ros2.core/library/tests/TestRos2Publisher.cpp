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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

// CARB_BINDINGS moved to first test file

TEST_SUITE("isaacsim.ros2.core.publisher_tests")
{
    TEST_CASE("Ros2Publisher: creation and validation")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_publisher_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Create a clock message for testing
        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Create publisher with default QoS
        isaacsim::ros2::core::Ros2QoSProfile qos;
        auto pub =
            testBase.getFactory()->createPublisher(node.get(), "/test_clock", clockMsg->getTypeSupportHandle(), qos);
        CHECK(pub != nullptr);
        CHECK(pub->isValid());

        // Check initial subscription count
        CHECK(pub->getSubscriptionCount() == 0);

        CHECK(ctx->shutdown("test-publisher-creation"));
    }

    TEST_CASE("Ros2Publisher: publishing messages")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_publisher_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Create a clock message
        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Create publisher
        isaacsim::ros2::core::Ros2QoSProfile qos;
        auto pub =
            testBase.getFactory()->createPublisher(node.get(), "/test_clock_pub", clockMsg->getTypeSupportHandle(), qos);
        CHECK(pub != nullptr);

        // Write data to message
        double timestamp = 123.456;
        clockMsg->writeData(timestamp);

        // Publish the message (should not crash)
        pub->publish(clockMsg->getPtr());

        CHECK(ctx->shutdown("test-publisher-publish"));
    }

    TEST_CASE("Ros2Publisher: topic name validation")
    {
        ROS2_TEST_SETUP();

        // Test topic name validation
        CHECK(testBase.getFactory()->validateTopicName("/valid_topic"));
        CHECK(testBase.getFactory()->validateTopicName("/test/nested/topic"));


        CHECK_FALSE(testBase.getFactory()->validateTopicName(""));
        CHECK_FALSE(testBase.getFactory()->validateTopicName("topic_without_slash"));
        CHECK_FALSE(testBase.getFactory()->validateTopicName("/topic with space"));
        CHECK_FALSE(testBase.getFactory()->validateTopicName("/topic-with-dash"));
        CHECK_FALSE(testBase.getFactory()->validateTopicName("/123topic")); // component starts with number
        CHECK_FALSE(testBase.getFactory()->validateTopicName("//double_slash"));
    }

    TEST_CASE("Ros2Publisher: different QoS profiles")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_qos_node", "test", ctx.get());
        CHECK(node != nullptr);

        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Test with sensor data QoS
        isaacsim::ros2::core::Ros2QoSProfile sensorQos;
        sensorQos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eBestEffort;
        sensorQos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        sensorQos.depth = 5;
        auto sensorPub = testBase.getFactory()->createPublisher(
            node.get(), "/test_sensor", clockMsg->getTypeSupportHandle(), sensorQos);
        CHECK(sensorPub != nullptr);
        CHECK(sensorPub->isValid());

        // Test with system default QoS
        isaacsim::ros2::core::Ros2QoSProfile systemQos;
        auto systemPub = testBase.getFactory()->createPublisher(
            node.get(), "/test_system", clockMsg->getTypeSupportHandle(), systemQos);
        CHECK(systemPub != nullptr);
        CHECK(systemPub->isValid());

        // Test with services default QoS
        isaacsim::ros2::core::Ros2QoSProfile servicesQos;
        servicesQos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        servicesQos.history = isaacsim::ros2::core::Ros2QoSHistoryPolicy::eKeepLast;
        servicesQos.depth = 10;
        auto servicesPub = testBase.getFactory()->createPublisher(
            node.get(), "/test_services", clockMsg->getTypeSupportHandle(), servicesQos);
        CHECK(servicesPub != nullptr);
        CHECK(servicesPub->isValid());

        CHECK(ctx->shutdown("test-qos-profiles"));
    }

    TEST_CASE("Ros2Publisher: multiple publishers on same node")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("multi_pub_node", "test", ctx.get());
        CHECK(node != nullptr);

        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        isaacsim::ros2::core::Ros2QoSProfile qos;

        // Create multiple publishers
        auto pub1 = testBase.getFactory()->createPublisher(node.get(), "/clock1", clockMsg->getTypeSupportHandle(), qos);
        CHECK(pub1 != nullptr);

        auto pub2 = testBase.getFactory()->createPublisher(node.get(), "/clock2", clockMsg->getTypeSupportHandle(), qos);
        CHECK(pub2 != nullptr);

        auto pub3 =
            testBase.getFactory()->createPublisher(node.get(), "/test/clock3", clockMsg->getTypeSupportHandle(), qos);
        CHECK(pub3 != nullptr);

        // All should be valid
        CHECK(pub1->isValid());
        CHECK(pub2->isValid());
        CHECK(pub3->isValid());

        CHECK(ctx->shutdown("test-multiple-publishers"));
    }
}
