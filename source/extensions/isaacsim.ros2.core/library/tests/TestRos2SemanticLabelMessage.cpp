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

TEST_SUITE("isaacsim.ros2.core.semantic_label_message_tests")
{
    TEST_CASE("Ros2SemanticLabelMessage: creation and basic functionality")
    {
        ROS2_TEST_SETUP();

        // Create a SemanticLabel message
        auto message = testBase.getFactory()->createSemanticLabelMessage();
        REQUIRE(message);

        // Test type support
        auto typeSupport = message->getTypeSupportHandle();
        REQUIRE(typeSupport != nullptr);

        // Test message data pointer
        auto msgPtr = message->getPtr();
        REQUIRE(msgPtr != nullptr);
    }

    TEST_CASE("Ros2SemanticLabelMessage: write string data")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createSemanticLabelMessage();
        REQUIRE(message);

        // Write semantic label data
        std::string labelData = "person";
        REQUIRE_NOTHROW(message->writeData(labelData));

        // Test with different label types
        REQUIRE_NOTHROW(message->writeData("car"));
        REQUIRE_NOTHROW(message->writeData("building"));
        REQUIRE_NOTHROW(message->writeData("road"));
    }

    TEST_CASE("Ros2SemanticLabelMessage: write empty and special strings")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createSemanticLabelMessage();
        REQUIRE(message);

        // Test with empty string
        REQUIRE_NOTHROW(message->writeData(""));

        // Test with strings containing special characters
        REQUIRE_NOTHROW(message->writeData("label_with_underscores"));
        REQUIRE_NOTHROW(message->writeData("label-with-dashes"));
        REQUIRE_NOTHROW(message->writeData("label with spaces"));
        REQUIRE_NOTHROW(message->writeData("label123"));
    }

    TEST_CASE("Ros2SemanticLabelMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context);
        REQUIRE_NOTHROW(context->init(0, nullptr));

        auto node = testBase.getFactory()->createNodeHandle("test_semantic_label_node", "test", context.get());
        REQUIRE(node);

        // Create message
        auto message = testBase.getFactory()->createSemanticLabelMessage();
        REQUIRE(message);

        // Set up QoS profile
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create publisher
        auto publisher =
            testBase.getFactory()->createPublisher(node.get(), "/semantic_labels", message->getTypeSupportHandle(), qos);
        REQUIRE(publisher);

        // Write message data
        std::string semanticLabel = "vehicle";
        message->writeData(semanticLabel);

        // Publish the message
        REQUIRE_NOTHROW(publisher->publish(message->getPtr()));

        // Clean up
        REQUIRE_NOTHROW(context->shutdown());
    }

    TEST_CASE("Ros2SemanticLabelMessage: multiple labels")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createSemanticLabelMessage();
        REQUIRE(message);

        // Test writing multiple labels sequentially (should overwrite)
        std::vector<std::string> labels = { "background", "person",   "bicycle", "car",
                                            "motorcycle", "airplane", "bus",     "train" };

        for (const auto& label : labels)
        {
            REQUIRE_NOTHROW(message->writeData(label));
        }
    }

    TEST_CASE("Ros2SemanticLabelMessage: long strings")
    {
        ROS2_TEST_SETUP();

        auto message = testBase.getFactory()->createSemanticLabelMessage();
        REQUIRE(message);

        // Test with a very long label
        std::string longLabel = "very_long_semantic_label_name_that_might_exceed_normal_expectations_for_label_length";
        REQUIRE_NOTHROW(message->writeData(longLabel));

        // Test with repeated pattern
        std::string repeatedLabel(1000, 'a');
        REQUIRE_NOTHROW(message->writeData(repeatedLabel));
    }
}
