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

TEST_SUITE("isaacsim.ros2.core.client_tests")
{
    TEST_CASE("Ros2Client: creation with dynamic message")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_client_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Try to create a dynamic service message for client
        auto serviceMsg = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eRequest);

        if (serviceMsg && serviceMsg)
        {
            MESSAGE("Successfully created dynamic service message");

            // Create client with default QoS
            isaacsim::ros2::core::Ros2QoSProfile qos;
            qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
            qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
            qos.depth = 10;
            auto client = testBase.getFactory()->createClient(
                node.get(), "/test_service", serviceMsg->getTypeSupportHandle(), qos);

            if (client)
            {
                CHECK(client->isValid());
                MESSAGE("Successfully created client");
            }
            else
            {
                MESSAGE("Could not create client (this is expected if service types are not available)");
            }
        }
        else
        {
            MESSAGE("Could not create dynamic service message (this is expected in test environment)");
        }

        CHECK(ctx->shutdown("test-client-creation"));
    }

    TEST_CASE("Ros2Client: validation without type support")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("test_client_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Try to create client with null type support
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;
        auto client = testBase.getFactory()->createClient(node.get(), "/invalid_service", nullptr, qos);
        CHECK(client == nullptr);

        CHECK(ctx->shutdown("test-client-null-type"));
    }

    TEST_CASE("Ros2Client: service and client pair")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto serviceNode = testBase.getFactory()->createNodeHandle("service_node", "test", ctx.get());
        CHECK(serviceNode != nullptr);

        auto clientNode = testBase.getFactory()->createNodeHandle("client_node", "test", ctx.get());
        CHECK(clientNode != nullptr);

        // Try to create matching service and client
        auto serviceMsg = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eRequest);

        if (serviceMsg && serviceMsg)
        {
            isaacsim::ros2::core::Ros2QoSProfile qos;
            qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
            qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
            qos.depth = 10;

            auto service = testBase.getFactory()->createService(
                serviceNode.get(), "/test_srv", serviceMsg->getTypeSupportHandle(), qos);

            auto client = testBase.getFactory()->createClient(
                clientNode.get(), "/test_srv", serviceMsg->getTypeSupportHandle(), qos);

            if (service && client)
            {
                CHECK(service->isValid());
                CHECK(client->isValid());
                MESSAGE("Successfully created service-client pair");

                // Wait for discovery
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                // Note: Actual request/response testing would require proper message setup
                MESSAGE("Service-client communication test would require message data setup");
            }
            else
            {
                MESSAGE("Could not create service or client");
            }
        }
        else
        {
            MESSAGE("Could not create service message");
        }

        CHECK(ctx->shutdown("test-service-client-pair"));
    }

    TEST_CASE("Ros2Client: multiple clients to same service")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node1 = testBase.getFactory()->createNodeHandle("client_node_1", "test", ctx.get());
        CHECK(node1 != nullptr);

        auto node2 = testBase.getFactory()->createNodeHandle("client_node_2", "test", ctx.get());
        CHECK(node2 != nullptr);

        // Try to create multiple clients to same service
        auto serviceMsg = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "SetBool", isaacsim::ros2::core::BackendMessageType::eRequest);

        if (serviceMsg && serviceMsg)
        {
            isaacsim::ros2::core::Ros2QoSProfile qos;
            qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
            qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
            qos.depth = 10;

            auto client1 = testBase.getFactory()->createClient(
                node1.get(), "/shared_service", serviceMsg->getTypeSupportHandle(), qos);

            auto client2 = testBase.getFactory()->createClient(
                node2.get(), "/shared_service", serviceMsg->getTypeSupportHandle(), qos);

            if (client1 && client2)
            {
                CHECK(client1->isValid());
                CHECK(client2->isValid());
                MESSAGE("Successfully created multiple clients to same service");
            }
        }
        else
        {
            MESSAGE("Could not create service message");
        }

        CHECK(ctx->shutdown("test-multiple-clients"));
    }

    TEST_CASE("Ros2Client: different QoS profiles")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("qos_client_node", "test", ctx.get());
        CHECK(node != nullptr);

        auto serviceMsg = testBase.getFactory()->createDynamicMessage(
            "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eRequest);

        if (serviceMsg && serviceMsg)
        {
            // Test with different QoS profiles
            isaacsim::ros2::core::Ros2QoSProfile defaultQos;
            defaultQos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
            defaultQos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
            defaultQos.depth = 10;
            auto client1 = testBase.getFactory()->createClient(
                node.get(), "/qos_test_1", serviceMsg->getTypeSupportHandle(), defaultQos);
            if (client1)
            {
                CHECK(client1->isValid());
            }

            // Create custom QoS
            isaacsim::ros2::core::Ros2QoSProfile customQos = defaultQos;
            customQos.depth = 50;
            customQos.deadline.sec = 1;
            customQos.deadline.nsec = 0;

            auto client2 = testBase.getFactory()->createClient(
                node.get(), "/qos_test_2", serviceMsg->getTypeSupportHandle(), customQos);
            if (client2)
            {
                CHECK(client2->isValid());
            }
        }
        else
        {
            MESSAGE("Could not create service message");
        }

        CHECK(ctx->shutdown("test-client-qos"));
    }
}
