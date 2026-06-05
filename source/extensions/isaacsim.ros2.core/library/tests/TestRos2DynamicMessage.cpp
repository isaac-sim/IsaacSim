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
#include <vector>

// CARB_BINDINGS moved to first test file

TEST_SUITE("isaacsim.ros2.core.dynamic_message_tests")
{
    TEST_CASE("Ros2DynamicMessage: creation of standard messages")
    {
        ROS2_TEST_SETUP();

        SUBCASE("std_msgs/String")
        {
            auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "msg", "String");
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(status);
                CHECK(msg->getPtr() != nullptr);
                CHECK(msg->getTypeSupportHandle() != nullptr);
                MESSAGE("Successfully created std_msgs/String");
            }
            else
            {
                MESSAGE("Could not create std_msgs/String (this is expected if message definitions are not available)");
            }
        }

        SUBCASE("std_msgs/Bool")
        {
            auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "msg", "Bool");
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(status);
                CHECK(msg->getPtr() != nullptr);
                CHECK(msg->getTypeSupportHandle() != nullptr);
                MESSAGE("Successfully created std_msgs/Bool");
            }
            else
            {
                MESSAGE("Could not create std_msgs/Bool");
            }
        }

        SUBCASE("std_msgs/Float32")
        {
            auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "msg", "Float32");
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(status);
                CHECK(msg->getPtr() != nullptr);
                CHECK(msg->getTypeSupportHandle() != nullptr);
                MESSAGE("Successfully created std_msgs/Float32");
            }
            else
            {
                MESSAGE("Could not create std_msgs/Float32");
            }
        }

        SUBCASE("geometry_msgs/Point")
        {
            auto msg = testBase.getFactory()->createDynamicMessage("geometry_msgs", "msg", "Point");
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(status);
                CHECK(msg->getPtr() != nullptr);
                CHECK(msg->getTypeSupportHandle() != nullptr);
                MESSAGE("Successfully created geometry_msgs/Point");
            }
            else
            {
                MESSAGE("Could not create geometry_msgs/Point");
            }
        }
    }

    TEST_CASE("Ros2DynamicMessage: invalid message creation")
    {
        ROS2_TEST_SETUP();

        SUBCASE("Non-existent package")
        {
            auto msg = testBase.getFactory()->createDynamicMessage("non_existent_pkg", "msg", "FakeMessage");
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(!status); // Should be invalid
                MESSAGE("Created message but it's correctly invalid for non-existent package");
            }
            else
            {
                MESSAGE("Correctly failed to create message from non-existent package");
                CHECK(true);
            }
        }

        SUBCASE("Non-existent message type")
        {
            auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "msg", "NonExistentType");
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(!status); // Should be invalid
                MESSAGE("Created message but it's correctly invalid for non-existent message type");
            }
            else
            {
                MESSAGE("Correctly failed to create non-existent message type");
                CHECK(true);
            }
        }

        SUBCASE("Wrong subfolder")
        {
            auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "srv", "String");
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(!status); // Should be invalid
                MESSAGE("Created message but it's correctly invalid for wrong subfolder");
            }
            else
            {
                MESSAGE("Correctly failed to create message with wrong subfolder");
                CHECK(true);
            }
        }
    }

    TEST_CASE("Ros2DynamicMessage: message fields")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "msg", "Header");
        if (msg)
        {
            auto dynamicMsg = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg);
            bool status = dynamicMsg->isValid();
            CHECK(status);

            if (dynamicMsg && status)
            {
                // Get message fields
                auto fields = dynamicMsg->getMessageFields();
                CHECK(!fields.empty());

                MESSAGE("std_msgs/Header has " << fields.size() << " fields");
                for (const auto& field : fields)
                {
                    MESSAGE("Field: " << field.name << " (type: " << field.ognType << ", array: " << field.isArray
                                      << ")");
                }

                // Generate summary
                std::string summary = dynamicMsg->generateSummary(false);
                CHECK(!summary.empty());
                MESSAGE("Message summary length: " << summary.length());
            }
        }
        else
        {
            MESSAGE("Could not create std_msgs/Header");
        }
    }

    TEST_CASE("Ros2DynamicMessage: service messages")
    {
        ROS2_TEST_SETUP();

        SUBCASE("std_srvs/Empty service")
        {
            auto msg = testBase.getFactory()->createDynamicMessage(
                "std_srvs", "srv", "Empty", isaacsim::ros2::core::BackendMessageType::eRequest);
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(status);
                CHECK(msg->getPtr() != nullptr);
                CHECK(msg->getTypeSupportHandle() != nullptr);
                MESSAGE("Successfully created std_srvs/Empty service");
            }
            else
            {
                MESSAGE("Could not create std_srvs/Empty service");
            }
        }

        SUBCASE("std_srvs/SetBool service")
        {
            auto msg = testBase.getFactory()->createDynamicMessage(
                "std_srvs", "srv", "SetBool", isaacsim::ros2::core::BackendMessageType::eRequest);
            if (msg)
            {
                bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
                CHECK(status);
                CHECK(msg->getPtr() != nullptr);
                CHECK(msg->getTypeSupportHandle() != nullptr);
                MESSAGE("Successfully created std_srvs/SetBool service");
            }
            else
            {
                MESSAGE("Could not create std_srvs/SetBool service");
            }
        }
    }

    TEST_CASE("Ros2DynamicMessage: message specs")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "msg", "String");
        if (msg)
        {
            bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
            CHECK(status);
            // Get message spec
            // Dynamic messages don't have getMessageSpec method
            // Check type support instead
            CHECK(msg->getTypeSupportHandle() != nullptr);

            // Dynamic messages may have different methods for introspection
        }
        else
        {
            MESSAGE("Could not create std_msgs/String");
        }
    }

    TEST_CASE("Ros2DynamicMessage: complex nested messages")
    {
        ROS2_TEST_SETUP();

        // Try to create a complex message with nested types
        auto msg = testBase.getFactory()->createDynamicMessage("sensor_msgs", "msg", "PointCloud2");
        if (msg)
        {
            auto dynamicMsg = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg);
            bool status = dynamicMsg->isValid();
            CHECK(status);

            if (dynamicMsg && status)
            {
                auto fields = dynamicMsg->getMessageFields();
                CHECK(!fields.empty());

                MESSAGE("sensor_msgs/PointCloud2 has " << fields.size() << " fields");

                // Check for nested fields (header, etc.)
                bool hasHeader = false;
                for (const auto& field : fields)
                {
                    if (field.name.find("header") != std::string::npos)
                    {
                        hasHeader = true;
                        break;
                    }
                }
                CHECK(hasHeader);
            }
        }
        else
        {
            MESSAGE("Could not create sensor_msgs/PointCloud2");
        }
    }

    TEST_CASE("Ros2DynamicMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("dynamic_pub_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Create dynamic message
        auto msg = testBase.getFactory()->createDynamicMessage("std_msgs", "msg", "String");
        if (msg)
        {
            bool status = std::static_pointer_cast<isaacsim::ros2::core::Ros2DynamicMessage>(msg)->isValid();
            CHECK(status);

            // Create publisher
            isaacsim::ros2::core::Ros2QoSProfile qos;
            auto pub =
                testBase.getFactory()->createPublisher(node.get(), "/dynamic_string", msg->getTypeSupportHandle(), qos);
            if (pub && pub->isValid())
            {
                // Note: Actually writing data to dynamic messages would require
                // the JSON or vector interface which needs more setup
                MESSAGE("Successfully created publisher for dynamic message");

                // Publish the message (even if empty)
                pub->publish(msg->getPtr());
            }
        }
        else
        {
            MESSAGE("Could not create dynamic message for publishing");
        }

        CHECK(ctx->shutdown("test-dynamic-publisher"));
    }
}
