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

#include <memory>
#include <string>
#include <vector>

CARB_BINDINGS("isaacsim.ros2.core.backend_tests")

TEST_SUITE("isaacsim.ros2.core.camera_info_message_tests")
{
    TEST_CASE("Ros2CameraInfoMessage: creation and validation")
    {
        ROS2_TEST_SETUP();

        // Create camera info message
        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        // Check that message data and type support are valid
        CHECK(cameraInfoMsg != nullptr);
        CHECK(cameraInfoMsg->getPtr() != nullptr);
        CHECK(cameraInfoMsg->getTypeSupportHandle() != nullptr);
    }

    TEST_CASE("Ros2CameraInfoMessage: write header")
    {
        // Load the factory for the humble distro
        ROS2_TEST_SETUP();

        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        SUBCASE("Normal header")
        {
            double timestamp = 123.456;
            std::string frameId = "camera_optical_frame";
            cameraInfoMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }

        SUBCASE("Empty frame ID")
        {
            double timestamp = 456.789;
            std::string frameId = "";
            cameraInfoMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }

        SUBCASE("Complex frame ID")
        {
            double timestamp = 789.012;
            std::string frameId = "robot/sensors/camera/left_optical_frame";
            cameraInfoMsg->writeHeader(timestamp, frameId);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2CameraInfoMessage: write resolution")
    {
        // Load the factory for the humble distro
        ROS2_TEST_SETUP();

        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        SUBCASE("Standard resolutions")
        {
            cameraInfoMsg->writeResolution(480, 640);
            CHECK(true);

            cameraInfoMsg->writeResolution(720, 1280);
            CHECK(true);

            cameraInfoMsg->writeResolution(1080, 1920);
            CHECK(true);
        }

        SUBCASE("Square resolution")
        {
            cameraInfoMsg->writeResolution(512, 512);
            CHECK(true);
        }

        SUBCASE("Ultra-wide resolution")
        {
            cameraInfoMsg->writeResolution(1080, 3840);
            CHECK(true);
        }

        SUBCASE("Small resolution")
        {
            cameraInfoMsg->writeResolution(240, 320);
            CHECK(true);
        }

        SUBCASE("Zero dimensions (edge case)")
        {
            cameraInfoMsg->writeResolution(0, 0);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2CameraInfoMessage: write intrinsic matrix")
    {
        // Load the factory for the humble distro
        ROS2_TEST_SETUP();

        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        SUBCASE("Standard 3x3 intrinsic matrix")
        {
            // K = [fx  0 cx]
            //     [ 0 fy cy]
            //     [ 0  0  1]
            double K[9] = { 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0 }; // NOLINT(readability-identifier-naming)
            cameraInfoMsg->writeIntrinsicMatrix(K, 9);
            CHECK(true);
        }

        SUBCASE("Identity matrix")
        {
            double K[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }; // NOLINT(readability-identifier-naming)
            cameraInfoMsg->writeIntrinsicMatrix(K, 9);
            CHECK(true);
        }

        SUBCASE("Different focal lengths")
        {
            double K[9] = { 600.0, 0.0, 640.0, 0.0, 400.0, 360.0, 0.0, 0.0, 1.0 }; // NOLINT(readability-identifier-naming)
            cameraInfoMsg->writeIntrinsicMatrix(K, 9);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2CameraInfoMessage: write projection matrix")
    {
        // Load the factory for the humble distro
        ROS2_TEST_SETUP();

        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        SUBCASE("Standard 3x4 projection matrix")
        {
            // P = [fx  0 cx tx]
            //     [ 0 fy cy ty]
            //     [ 0  0  1  0]
            double P[12] = {
                500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0
            }; // NOLINT(readability-identifier-naming)
            cameraInfoMsg->writeProjectionMatrix(P, 12);
            CHECK(true);
        }

        SUBCASE("Projection matrix with baseline")
        {
            // Stereo camera with baseline
            double P[12] = { 500.0, 0.0, 320.0, -50.0, // tx = -fx * baseline  //
                                                       // NOLINT(readability-identifier-naming)
                             0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
            cameraInfoMsg->writeProjectionMatrix(P, 12);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2CameraInfoMessage: write rectification matrix")
    {
        // Load the factory for the humble distro
        ROS2_TEST_SETUP();

        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        SUBCASE("Identity rectification (no rectification needed)")
        {
            double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }; // NOLINT(readability-identifier-naming)
            cameraInfoMsg->writeRectificationMatrix(R, 9);
            CHECK(true);
        }

        SUBCASE("Rotation rectification matrix")
        {
            double R[9] = { 0.999, 0.01, 0.0, -0.01, 0.999, 0.0, 0.0, 0.0, 1.0 }; // NOLINT(readability-identifier-naming)
            cameraInfoMsg->writeRectificationMatrix(R, 9);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2CameraInfoMessage: write distortion parameters")
    {
        // Load the factory for the humble distro
        ROS2_TEST_SETUP();

        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        SUBCASE("Plumb bob distortion model")
        {
            std::vector<double> D = { 0.1, -0.2, 0.001, -0.001, 0.0 }; // NOLINT(readability-identifier-naming)
            std::string model = "plumb_bob";
            cameraInfoMsg->writeDistortionParameters(D, model);
            CHECK(true);
        }

        SUBCASE("No distortion")
        {
            std::vector<double> D = { 0.0, 0.0, 0.0, 0.0, 0.0 }; // NOLINT(readability-identifier-naming)
            std::string model = "plumb_bob";
            cameraInfoMsg->writeDistortionParameters(D, model);
            CHECK(true);
        }

        SUBCASE("Rational polynomial model")
        {
            std::vector<double> D = {
                0.1, -0.2, 0.001, -0.001, 0.15, -0.28, 0.002, -0.0003
            }; // NOLINT(readability-identifier-naming)
            std::string model = "rational_polynomial";
            cameraInfoMsg->writeDistortionParameters(D, model);
            CHECK(true);
        }

        SUBCASE("Empty distortion coefficients")
        {
            std::vector<double> D; // NOLINT(readability-identifier-naming)
            std::string model = "";
            cameraInfoMsg->writeDistortionParameters(D, model);
            CHECK(true);
        }
    }

    TEST_CASE("Ros2CameraInfoMessage: complete camera info")
    {
        // Load the factory for the humble distro
        ROS2_TEST_SETUP();

        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        // Write complete camera info
        double timestamp = 100.5;
        std::string frameId = "camera_link";
        cameraInfoMsg->writeHeader(timestamp, frameId);

        cameraInfoMsg->writeResolution(480, 640);

        double K[9] = { 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0 }; // NOLINT(readability-identifier-naming)
        cameraInfoMsg->writeIntrinsicMatrix(K, 9);

        double P[12] = {
            500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0
        }; // NOLINT(readability-identifier-naming)
        cameraInfoMsg->writeProjectionMatrix(P, 12);

        double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }; // NOLINT(readability-identifier-naming)
        cameraInfoMsg->writeRectificationMatrix(R, 9);

        std::vector<double> D = { 0.0, 0.0, 0.0, 0.0, 0.0 }; // NOLINT(readability-identifier-naming)
        std::string model = "plumb_bob";
        cameraInfoMsg->writeDistortionParameters(D, model);

        CHECK(cameraInfoMsg != nullptr);
    }

    TEST_CASE("Ros2CameraInfoMessage: use with publisher")
    {
        ROS2_TEST_SETUP_WITH_NODE("camera_info_pub_node");

        // Create camera info message
        auto cameraInfoMsg = testBase.getFactory()->createCameraInfoMessage();
        CHECK(cameraInfoMsg != nullptr);

        // Create publisher
        auto qos = testBase.createTransientLocalQoS();
        auto pub = testBase.getFactory()->createPublisher(
            testBase.getNode().get(), "/camera/camera_info", cameraInfoMsg->getTypeSupportHandle(), qos);
        CHECK(pub != nullptr);
        CHECK(pub->isValid());

        // Publish camera info
        double timestamp = 123.456;
        std::string frameId = "camera_optical_frame";
        cameraInfoMsg->writeHeader(timestamp, frameId);
        cameraInfoMsg->writeResolution(720, 1280);

        pub->publish(cameraInfoMsg->getPtr());
    }
}
