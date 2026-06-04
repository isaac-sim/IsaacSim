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
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/vec3d.h>

TEST_SUITE("isaacsim.ros2.core.odometry_message_tests")
{
    TEST_CASE("Ros2OdometryMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createOdometryMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2OdometryMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createOdometryMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2OdometryMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createOdometryMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2OdometryMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createOdometryMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "odom";

        // Should not crash when writing header
        CHECK_NOTHROW(msg->writeHeader(timestamp, frameId));
    }

    TEST_CASE("Ros2OdometryMessage: write data with raw velocities")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createOdometryMessage();
        REQUIRE(msg != nullptr);

        std::string childFrame = "base_link";
        pxr::GfVec3d linearVelocity(1.0, 0.5, 0.0);
        pxr::GfVec3d angularVelocity(0.0, 0.0, 0.1);
        pxr::GfVec3d robotFront(1.0, 0.0, 0.0);
        pxr::GfVec3d robotSide(0.0, 1.0, 0.0);
        pxr::GfVec3d robotUp(0.0, 0.0, 1.0);
        double unitScale = 1.0;
        pxr::GfVec3d position(10.0, 5.0, 0.0);
        pxr::GfQuatd orientation(1.0, 0.0, 0.0, 0.0);
        bool publishRawVelocities = true;

        // Should not crash when writing data
        CHECK_NOTHROW(msg->writeData(childFrame, linearVelocity, angularVelocity, robotFront, robotSide, robotUp,
                                     unitScale, position, orientation, publishRawVelocities));
    }

    TEST_CASE("Ros2OdometryMessage: write data with projected velocities")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createOdometryMessage();
        REQUIRE(msg != nullptr);

        std::string childFrame = "base_link";
        pxr::GfVec3d linearVelocity(1.0, 0.5, 0.0);
        pxr::GfVec3d angularVelocity(0.0, 0.0, 0.1);
        pxr::GfVec3d robotFront(1.0, 0.0, 0.0);
        pxr::GfVec3d robotSide(0.0, 1.0, 0.0);
        pxr::GfVec3d robotUp(0.0, 0.0, 1.0);
        double unitScale = 1.0;
        pxr::GfVec3d position(10.0, 5.0, 0.0);
        pxr::GfQuatd orientation(1.0, 0.0, 0.0, 0.0);
        bool publishRawVelocities = false;

        // Should not crash when writing data with projected velocities
        CHECK_NOTHROW(msg->writeData(childFrame, linearVelocity, angularVelocity, robotFront, robotSide, robotUp,
                                     unitScale, position, orientation, publishRawVelocities));
    }

    TEST_CASE("Ros2OdometryMessage: write data with scale factor")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createOdometryMessage();
        REQUIRE(msg != nullptr);

        std::string childFrame = "base_link";
        pxr::GfVec3d linearVelocity(2.0, 1.0, 0.0);
        pxr::GfVec3d angularVelocity(0.0, 0.0, 0.2);
        pxr::GfVec3d robotFront(1.0, 0.0, 0.0);
        pxr::GfVec3d robotSide(0.0, 1.0, 0.0);
        pxr::GfVec3d robotUp(0.0, 0.0, 1.0);
        double unitScale = 0.01; // Centimeters to meters
        pxr::GfVec3d position(1000.0, 500.0, 0.0); // In centimeters
        pxr::GfQuatd orientation(0.707, 0.0, 0.0, 0.707); // 90 degree rotation
        bool publishRawVelocities = true;

        // Should not crash when writing data with scale factor
        CHECK_NOTHROW(msg->writeData(childFrame, linearVelocity, angularVelocity, robotFront, robotSide, robotUp,
                                     unitScale, position, orientation, publishRawVelocities));
    }

    TEST_CASE("Ros2OdometryMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context != nullptr);
        context->init(0, nullptr);

        auto node = testBase.getFactory()->createNodeHandle("test_odom_node", "test", context.get());
        REQUIRE(node != nullptr);

        auto msg = testBase.getFactory()->createOdometryMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;

        // Create publisher - should not crash
        auto publisher =
            testBase.getFactory()->createPublisher(node.get(), "test_odom", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Write some data and publish - should not crash
        msg->writeHeader(1.0, "odom");
        std::string childFrame = "base_link";
        pxr::GfVec3d linearVelocity(1.0, 0.0, 0.0);
        pxr::GfVec3d angularVelocity(0.0, 0.0, 0.0);
        pxr::GfVec3d robotFront(1.0, 0.0, 0.0);
        pxr::GfVec3d robotSide(0.0, 1.0, 0.0);
        pxr::GfVec3d robotUp(0.0, 0.0, 1.0);
        double unitScale = 1.0;
        pxr::GfVec3d position(0.0, 0.0, 0.0);
        pxr::GfQuatd orientation(1.0, 0.0, 0.0, 0.0);

        CHECK_NOTHROW(msg->writeData(childFrame, linearVelocity, angularVelocity, robotFront, robotSide, robotUp,
                                     unitScale, position, orientation, true));

        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }
}
