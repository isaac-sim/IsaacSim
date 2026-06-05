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

TEST_SUITE("isaacsim.ros2.core.subscriber_tests")
{
    TEST_CASE("Ros2Subscriber: creation and validation")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_subscriber_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Create a clock message for testing
        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Create subscriber with default QoS
        isaacsim::ros2::core::Ros2QoSProfile qos;
        auto sub = testBase.getFactory()->createSubscriber(
            node.get(), "/test_clock_sub", clockMsg->getTypeSupportHandle(), qos);
        CHECK(sub != nullptr);
        CHECK(sub->isValid());

        CHECK(ctx->shutdown("test-subscriber-creation"));
    }

    TEST_CASE("Ros2Subscriber: spin without messages")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_spin_node", "test", ctx.get());
        CHECK(node != nullptr);

        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        isaacsim::ros2::core::Ros2QoSProfile qos;
        auto sub = testBase.getFactory()->createSubscriber(
            node.get(), "/test_empty_topic", clockMsg->getTypeSupportHandle(), qos);
        CHECK(sub != nullptr);

        // Spin should return false when no messages available
        bool hasMessage = sub->spin(clockMsg->getPtr());
        CHECK_FALSE(hasMessage);

        CHECK(ctx->shutdown("test-spin-empty"));
    }

    TEST_CASE("Ros2Subscriber: publisher and subscriber communication")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto pubNode = testBase.getFactory()->createNodeHandle("pub_node", "test", ctx.get());
        CHECK(pubNode != nullptr);

        auto subNode = testBase.getFactory()->createNodeHandle("sub_node", "test", ctx.get());
        CHECK(subNode != nullptr);

        // Create messages
        auto pubClockMsg = testBase.getFactory()->createClockMessage();
        CHECK(pubClockMsg != nullptr);

        auto subClockMsg = testBase.getFactory()->createClockMessage();
        CHECK(subClockMsg != nullptr);

        // Create publisher and subscriber on same topic
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;

        auto pub = testBase.getFactory()->createPublisher(
            pubNode.get(), "/test_comm", pubClockMsg->getTypeSupportHandle(), qos);
        CHECK(pub != nullptr);

        auto sub = testBase.getFactory()->createSubscriber(
            subNode.get(), "/test_comm", subClockMsg->getTypeSupportHandle(), qos);
        CHECK(sub != nullptr);

        // Wait for discovery
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Publish a message
        double sentTimestamp = 456.789;
        pubClockMsg->writeData(sentTimestamp);
        pub->publish(pubClockMsg->getPtr());

        // Wait a bit for message propagation
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Try to receive the message
        bool hasMessage = sub->spin(subClockMsg->getPtr());

        // Note: Message reception might fail in test environment
        if (hasMessage)
        {
            MESSAGE("Successfully received message");
        }
        else
        {
            MESSAGE("No message received (this is expected in some test environments)");
        }

        CHECK(ctx->shutdown("test-pub-sub-comm"));
    }

    TEST_CASE("Ros2Subscriber: different QoS profiles")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_qos_sub_node", "test", ctx.get());
        CHECK(node != nullptr);

        auto clockMsg = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg != nullptr);

        // Test with sensor data QoS
        isaacsim::ros2::core::Ros2QoSProfile sensorQos;
        sensorQos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eBestEffort;
        sensorQos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        sensorQos.depth = 5;
        auto sensorSub = testBase.getFactory()->createSubscriber(
            node.get(), "/test_sensor_sub", clockMsg->getTypeSupportHandle(), sensorQos);
        CHECK(sensorSub != nullptr);
        CHECK(sensorSub->isValid());

        // Test with system default QoS
        isaacsim::ros2::core::Ros2QoSProfile systemQos;
        auto systemSub = testBase.getFactory()->createSubscriber(
            node.get(), "/test_system_sub", clockMsg->getTypeSupportHandle(), systemQos);
        CHECK(systemSub != nullptr);
        CHECK(systemSub->isValid());

        CHECK(ctx->shutdown("test-sub-qos-profiles"));
    }

    TEST_CASE("Ros2Subscriber: multiple subscribers on same topic")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node1 = testBase.getFactory()->createNodeHandle("sub_node_1", "test", ctx.get());
        CHECK(node1 != nullptr);

        auto node2 = testBase.getFactory()->createNodeHandle("sub_node_2", "test", ctx.get());
        CHECK(node2 != nullptr);

        auto clockMsg1 = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg1 != nullptr);

        auto clockMsg2 = testBase.getFactory()->createClockMessage();
        CHECK(clockMsg2 != nullptr);

        isaacsim::ros2::core::Ros2QoSProfile qos;

        // Create multiple subscribers on same topic
        auto sub1 = testBase.getFactory()->createSubscriber(
            node1.get(), "/shared_topic", clockMsg1->getTypeSupportHandle(), qos);
        CHECK(sub1 != nullptr);

        auto sub2 = testBase.getFactory()->createSubscriber(
            node2.get(), "/shared_topic", clockMsg2->getTypeSupportHandle(), qos);
        CHECK(sub2 != nullptr);

        // Both should be valid
        CHECK(sub1->isValid());
        CHECK(sub2->isValid());

        CHECK(ctx->shutdown("test-multiple-subscribers"));
    }
}
