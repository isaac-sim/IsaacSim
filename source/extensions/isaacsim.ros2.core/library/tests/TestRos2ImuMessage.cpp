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

TEST_SUITE("isaacsim.ros2.core.imu_message_tests")
{
    TEST_CASE("Ros2ImuMessage: creation and validation")
    {
        ROS2_TEST_SETUP();

        // Create IMU message
        auto imuMsg = testBase.getFactory()->createImuMessage();
        CHECK(imuMsg != nullptr);

        // Check that message data and type support are valid
        CHECK(imuMsg != nullptr);
        CHECK(imuMsg->getPtr() != nullptr);
        CHECK(imuMsg->getTypeSupportHandle() != nullptr);
    }

    TEST_CASE("Ros2ImuMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto imuMsg = testBase.getFactory()->createImuMessage();
        CHECK(imuMsg != nullptr);

        // Test writing header with various timestamps and frame IDs
        SUBCASE("Normal header")
        {
            double timestamp = 123.456;
            std::string frameId = "imu_link";
            imuMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }

        SUBCASE("Empty frame ID")
        {
            double timestamp = 456.789;
            std::string frameId = "";
            imuMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }

        SUBCASE("Long frame ID")
        {
            double timestamp = 789.012;
            std::string frameId = "robot/sensors/imu/base_link";
            imuMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2ImuMessage: write acceleration")
    {
        ROS2_TEST_SETUP();

        auto imuMsg = testBase.getFactory()->createImuMessage();
        CHECK(imuMsg != nullptr);

        SUBCASE("Acceleration without covariance")
        {
            std::vector<double> acceleration = { 1.0, 2.0, 9.81 };
            imuMsg->writeAcceleration(false, acceleration);
            CHECK(true);
        }

        SUBCASE("Acceleration with covariance")
        {
            std::vector<double> acceleration = { 0.5, -0.5, 9.81 };
            imuMsg->writeAcceleration(true, acceleration);
            CHECK(true);
        }

        SUBCASE("Empty acceleration (use defaults)")
        {
            std::vector<double> acceleration;
            imuMsg->writeAcceleration(false, acceleration);
            CHECK(true);
        }

        SUBCASE("Zero acceleration")
        {
            std::vector<double> acceleration = { 0.0, 0.0, 0.0 };
            imuMsg->writeAcceleration(false, acceleration);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2ImuMessage: write velocity")
    {
        ROS2_TEST_SETUP();

        auto imuMsg = testBase.getFactory()->createImuMessage();
        CHECK(imuMsg != nullptr);

        SUBCASE("Angular velocity without covariance")
        {
            std::vector<double> velocity = { 0.1, 0.2, 0.3 };
            imuMsg->writeVelocity(false, velocity);
            CHECK(true);
        }

        SUBCASE("Angular velocity with covariance")
        {
            std::vector<double> velocity = { -0.1, -0.2, -0.3 };
            imuMsg->writeVelocity(true, velocity);
            CHECK(true);
        }

        SUBCASE("High angular velocity")
        {
            std::vector<double> velocity = { 10.0, 20.0, 30.0 };
            imuMsg->writeVelocity(false, velocity);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2ImuMessage: write orientation")
    {
        ROS2_TEST_SETUP();

        auto imuMsg = testBase.getFactory()->createImuMessage();
        CHECK(imuMsg != nullptr);

        SUBCASE("Orientation quaternion without covariance")
        {
            std::vector<double> orientation = { 0.0, 0.0, 0.0, 1.0 }; // Identity quaternion
            imuMsg->writeOrientation(false, orientation);
            CHECK(true);
        }

        SUBCASE("Orientation quaternion with covariance")
        {
            std::vector<double> orientation = { 0.7071, 0.0, 0.0, 0.7071 }; // 90 degree rotation
            imuMsg->writeOrientation(true, orientation);
            CHECK(true);
        }

        SUBCASE("Non-normalized quaternion")
        {
            std::vector<double> orientation = { 1.0, 2.0, 3.0, 4.0 };
            imuMsg->writeOrientation(false, orientation);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2ImuMessage: complete message")
    {
        ROS2_TEST_SETUP();

        auto imuMsg = testBase.getFactory()->createImuMessage();
        CHECK(imuMsg != nullptr);

        // Write complete IMU message
        double timestamp = 100.5;
        std::string frameId = "base_imu";
        imuMsg->writeHeader(timestamp, frameId);

        std::vector<double> acceleration = { 0.1, 0.2, 9.81 };
        imuMsg->writeAcceleration(true, acceleration);

        std::vector<double> angularVelocity = { 0.01, 0.02, 0.03 };
        imuMsg->writeVelocity(true, angularVelocity);

        std::vector<double> orientation = { 0.0, 0.0, 0.0, 1.0 };
        imuMsg->writeOrientation(true, orientation);

        CHECK(imuMsg != nullptr);
    }

    TEST_CASE("Ros2ImuMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        auto node = testBase.getFactory()->createNodeHandle("imu_pub_node", "test", ctx.get());
        CHECK(node != nullptr);

        // Create IMU message
        auto imuMsg = testBase.getFactory()->createImuMessage();
        CHECK(imuMsg != nullptr);

        // Create publisher for IMU messages
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eBestEffort;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 5;
        auto pub = testBase.getFactory()->createPublisher(node.get(), "/imu/data", imuMsg->getTypeSupportHandle(), qos);
        CHECK(pub != nullptr);
        CHECK(pub->isValid());

        // Publish IMU data
        for (int i = 0; i < 3; ++i)
        {
            double timestamp = i * 0.1;
            std::string frameId = "imu_frame";
            imuMsg->writeHeader(timestamp, frameId);

            std::vector<double> accel = { 0.0, 0.0, 9.81 + i * 0.1 };
            imuMsg->writeAcceleration(false, accel);

            std::vector<double> gyro = { i * 0.01, i * 0.02, i * 0.03 };
            imuMsg->writeVelocity(false, gyro);

            std::vector<double> quat = { 0.0, 0.0, 0.0, 1.0 };
            imuMsg->writeOrientation(false, quat);

            pub->publish(imuMsg->getPtr());
        }

        CHECK(ctx->shutdown("test-imu-publisher"));
    }
}
