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

TEST_SUITE("isaacsim.ros2.core.ackermann_drive_stamped_message_tests")
{
    TEST_CASE("Ros2AckermannDriveStampedMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: write header")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "base_link";

        // Should not crash when writing header
        CHECK_NOTHROW(msg->writeHeader(timestamp, frameId));
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: write data")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        // Typical Ackermann drive parameters
        double steeringAngle = 0.5; // radians
        double steeringAngleVelocity = 1.0; // rad/s
        double speed = 2.0; // m/s
        double acceleration = 1.0; // m/s^2
        double jerk = 0.5; // m/s^3

        // Should not crash when writing data
        CHECK_NOTHROW(msg->writeData(steeringAngle, steeringAngleVelocity, speed, acceleration, jerk));
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: write data with extreme values")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        // Extreme values to test robustness
        double steeringAngle = -1.57; // -90 degrees
        double steeringAngleVelocity = 10.0; // High angular velocity
        double speed = -5.0; // Reverse speed
        double acceleration = -2.0; // Deceleration
        double jerk = 0.0; // No jerk

        // Should not crash with extreme values
        CHECK_NOTHROW(msg->writeData(steeringAngle, steeringAngleVelocity, speed, acceleration, jerk));
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: read data from fresh message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        double timeStamp;
        std::string frameId;
        double steeringAngle;
        double steeringAngleVelocity;
        double speed;
        double acceleration;
        double jerk;

        // Should not crash reading from fresh message
        CHECK_NOTHROW(msg->readData(timeStamp, frameId, steeringAngle, steeringAngleVelocity, speed, acceleration, jerk));

        // Fresh message should have zero/default values
        CHECK(steeringAngle == 0.0);
        CHECK(steeringAngleVelocity == 0.0);
        CHECK(speed == 0.0);
        CHECK(acceleration == 0.0);
        CHECK(jerk == 0.0);
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: complete workflow")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        // Write header and data
        double timestamp = 1.0;
        std::string frameId = "base_link";
        double steeringAngle = 0.3;
        double steeringAngleVelocity = 0.5;
        double speed = 1.5;
        double acceleration = 0.8;
        double jerk = 0.2;

        CHECK_NOTHROW(msg->writeHeader(timestamp, frameId));
        CHECK_NOTHROW(msg->writeData(steeringAngle, steeringAngleVelocity, speed, acceleration, jerk));

        // Read back the data
        double readTimeStamp;
        std::string readFrameId;
        double readSteeringAngle;
        double readSteeringAngleVelocity;
        double readSpeed;
        double readAcceleration;
        double readJerk;

        CHECK_NOTHROW(msg->readData(readTimeStamp, readFrameId, readSteeringAngle, readSteeringAngleVelocity, readSpeed,
                                    readAcceleration, readJerk));

        // Verify the data (allowing for floating point precision)
        CHECK(readFrameId == frameId);
        CHECK(readSteeringAngle == doctest::Approx(steeringAngle).epsilon(0.001));
        CHECK(readSteeringAngleVelocity == doctest::Approx(steeringAngleVelocity).epsilon(0.001));
        CHECK(readSpeed == doctest::Approx(speed).epsilon(0.001));
        CHECK(readAcceleration == doctest::Approx(acceleration).epsilon(0.001));
        CHECK(readJerk == doctest::Approx(jerk).epsilon(0.001));
    }

    TEST_CASE("Ros2AckermannDriveStampedMessage: use with publisher")
    {
        ROS2_TEST_SETUP_WITH_NODE("test_ackermann_node");

        auto msg = testBase.getFactory()->createAckermannDriveStampedMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile appropriate for drive commands
        auto qos = testBase.createDefaultQoS();

        // Create publisher - should not crash
        auto publisher = testBase.getFactory()->createPublisher(
            testBase.getNode().get(), "ackermann_cmd", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        // Setup message and publish - should not crash
        msg->writeHeader(1.0, "base_link");
        msg->writeData(0.2, 0.1, 1.0, 0.5, 0.1);

        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }
}
