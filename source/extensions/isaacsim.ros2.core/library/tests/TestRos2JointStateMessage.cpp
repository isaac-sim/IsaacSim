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

TEST_SUITE("isaacsim.ros2.core.joint_state_message_tests")
{
    TEST_CASE("Ros2JointStateMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createJointStateMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2JointStateMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createJointStateMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2JointStateMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createJointStateMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2JointStateMessage: initial state")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createJointStateMessage();
        REQUIRE(msg != nullptr);

        // Initially should have no joints
        CHECK(msg->getNumJoints() == 0);
    }

    TEST_CASE("Ros2JointStateMessage: validation with empty message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createJointStateMessage();
        REQUIRE(msg != nullptr);

        // Empty message should be valid (all arrays have same size: 0)
        CHECK(msg->checkValid() == true);
    }

    TEST_CASE("Ros2JointStateMessage: read data from empty message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createJointStateMessage();
        REQUIRE(msg != nullptr);

        std::vector<char*> jointNames;
        double* jointPositions = nullptr;
        double* jointVelocities = nullptr;
        double* jointEfforts = nullptr;
        double timeStamp = 0.0;

        // Should not crash reading from empty message
        CHECK_NOTHROW(msg->readData(jointNames, jointPositions, jointVelocities, jointEfforts, timeStamp));

        // Joint names should be empty
        CHECK(jointNames.empty());
    }

    TEST_CASE("Ros2JointStateMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context != nullptr);
        context->init(0, nullptr);

        auto node = testBase.getFactory()->createNodeHandle("test_joint_state_node", "test", context.get());
        REQUIRE(node != nullptr);

        auto msg = testBase.getFactory()->createJointStateMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create publisher - should not crash
        auto publisher =
            testBase.getFactory()->createPublisher(node.get(), "joint_states", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Publish empty joint state - should not crash
        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }

    // Note: The writeData method requires physics articulation objects which are complex to create in tests.
    // For now, we test the basic functionality. Full integration tests would require a physics simulation setup.
}
