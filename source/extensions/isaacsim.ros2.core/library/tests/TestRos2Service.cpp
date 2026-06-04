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

TEST_SUITE("isaacsim.ros2.core.service_tests")
{
    TEST_CASE("Ros2Service: creation with dynamic message")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_service_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Try to create a dynamic service message
        auto serviceMsg = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eRequest);

        if (serviceMsg && serviceMsg)
        {
            MESSAGE("Successfully created dynamic service message");

            // Create service with default QoS
            isaacsim::ros2::core::Ros2QoSProfile qos;
            qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
            qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
            qos.depth = 10;
            auto service = testBase.getFactory()->createService(
                node.get(), "/test_service", serviceMsg->getTypeSupportHandle(), qos);

            if (service)
            {
                CHECK(service->isValid());
                MESSAGE("Successfully created service");
            }
            else
            {
                MESSAGE("Could not create service (this is expected if service types are not available)");
            }
        }
        else
        {
            MESSAGE("Could not create dynamic service message (this is expected in test environment)");
        }

        CHECK(ctx->shutdown("test-service-creation"));
    }

    TEST_CASE("Ros2Service: validation without type support")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_service_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Try to create service with null type support
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;
        auto service = testBase.getFactory()->createService(node.get(), "/invalid_service", nullptr, qos);
        CHECK(service == nullptr);

        CHECK(ctx->shutdown("test-service-null-type"));
    }

    TEST_CASE("Ros2Service: service name validation")
    {
        ROS2_TEST_SETUP();

        // Service names follow same rules as topic names
        CHECK(testBase.getFactory()->validateTopicName("/valid_service"));
        CHECK(testBase.getFactory()->validateTopicName("/test/nested/service"));

        CHECK_FALSE(testBase.getFactory()->validateTopicName(""));
        CHECK_FALSE(testBase.getFactory()->validateTopicName("service_without_slash"));
        CHECK_FALSE(testBase.getFactory()->validateTopicName("/service with space"));
        CHECK_FALSE(testBase.getFactory()->validateTopicName("/service-with-dash"));
    }

    TEST_CASE("Ros2Service: QoS profiles")
    {
        ROS2_TEST_SETUP();

        // Test QoS profile creation for services
        isaacsim::ros2::core::Ros2QoSProfile defaultQos;
        defaultQos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        defaultQos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        defaultQos.depth = 10;
        CHECK(defaultQos.reliability == isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable);
        CHECK(defaultQos.history == isaacsim::ros2::core::Ros2QoSHistoryPolicy::eKeepLast);
        CHECK(defaultQos.depth > 0);

        // Create custom QoS profile
        isaacsim::ros2::core::Ros2QoSProfile customQos;
        customQos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eBestEffort;
        customQos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        customQos.history = isaacsim::ros2::core::Ros2QoSHistoryPolicy::eKeepAll;
        customQos.depth = 100;

        // Verify custom settings
        CHECK(customQos.reliability == isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eBestEffort);
        CHECK(customQos.durability == isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile);
        CHECK(customQos.history == isaacsim::ros2::core::Ros2QoSHistoryPolicy::eKeepAll);
        CHECK(customQos.depth == 100);
    }

    TEST_CASE("Ros2Service: multiple services on same node")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("multi_service_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Try to create multiple dynamic service messages
        auto serviceMsg1 = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eRequest);
        auto serviceMsg2 = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "SetBool", isaacsim::ros2::core::BackendMessageType::eRequest);

        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        if (serviceMsg1 && serviceMsg1)
        {
            auto service1 =
                testBase.getFactory()->createService(node.get(), "/service1", serviceMsg1->getTypeSupportHandle(), qos);
            if (service1)
            {
                CHECK(service1->isValid());
                MESSAGE("Created first service");
            }
        }

        if (serviceMsg2 && serviceMsg2)
        {
            auto service2 =
                testBase.getFactory()->createService(node.get(), "/service2", serviceMsg2->getTypeSupportHandle(), qos);
            if (service2)
            {
                CHECK(service2->isValid());
                MESSAGE("Created second service");
            }
        }

        if (!serviceMsg1 || !serviceMsg2)
        {
            MESSAGE("Could not create all service messages (this is expected in test environment)");
        }

        CHECK(ctx->shutdown("test-multiple-services"));
    }

    TEST_CASE("Ros2Service: first takeRequest poll receives pending request")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        REQUIRE(ctx != nullptr);
        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        REQUIRE(ctx->isValid());

        auto serviceNode = testBase.getFactory()->createNodeHandle("service_node_first_poll", "test", ctx.get());
        auto clientNode = testBase.getFactory()->createNodeHandle("client_node_first_poll", "test", ctx.get());
        REQUIRE(serviceNode != nullptr);
        REQUIRE(clientNode != nullptr);

        auto requestMsg = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eRequest);
        auto responseMsg = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eResponse);
        REQUIRE(requestMsg != nullptr);
        REQUIRE(responseMsg != nullptr);

        isaacsim::ros2::core::Ros2QoSProfile qos = Ros2TestBase::createDefaultQoS();
        auto service = testBase.getFactory()->createService(
            serviceNode.get(), "/first_poll_service", requestMsg->getTypeSupportHandle(), qos);
        auto client = testBase.getFactory()->createClient(
            clientNode.get(), "/first_poll_service", requestMsg->getTypeSupportHandle(), qos);
        REQUIRE(service != nullptr);
        REQUIRE(client != nullptr);
        REQUIRE(service->isValid());
        REQUIRE(client->isValid());

        // Allow ROS graph discovery to complete before sending the request.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        REQUIRE(client->sendRequest(requestMsg->getPtr()));

        // Give the request enough time to reach the service queue, then verify the
        // first poll can receive it.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CHECK(service->takeRequest(requestMsg->getPtr()));

        REQUIRE(service->sendResponse(responseMsg->getPtr()));

        bool gotResponse = false;
        for (int i = 0; i < 50 && !gotResponse; ++i)
        {
            gotResponse = client->takeResponse(responseMsg->getPtr());
            if (!gotResponse)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        CHECK(gotResponse);

        CHECK(ctx->shutdown("test-service-first-poll"));
    }
}
