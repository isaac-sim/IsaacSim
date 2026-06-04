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

TEST_SUITE("isaacsim.ros2.core.point_cloud_message_tests")
{
    TEST_CASE("Ros2PointCloudMessage: factory creates non-null message")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        CHECK(msg != nullptr);
    }

    TEST_CASE("Ros2PointCloudMessage: type support handle is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        const void* typeSupport = msg->getTypeSupportHandle();
        CHECK(typeSupport != nullptr);
    }

    TEST_CASE("Ros2PointCloudMessage: message pointer is valid")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        const void* ptr = msg->getPtr();
        CHECK(ptr != nullptr);
    }

    TEST_CASE("Ros2PointCloudMessage: message parameters are null before buffer generation")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        CHECK(msg->getBufferPtr() == nullptr);
        CHECK(msg->getTotalBytes() == 0);
        CHECK(msg->getNumPoints() == 0);
        CHECK(msg->getPointStep() == 0);
        CHECK(msg->getOrderedFields().empty());
    }

    TEST_CASE("Ros2PointCloudMessage: generate buffer for unorganized cloud with no metadata")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "lidar_frame";
        const size_t numPoints = 1000;
        const size_t bufferSize = numPoints * sizeof(pxr::GfVec3f);
        float* intensityPtr = nullptr;
        uint64_t* timestampPtr = nullptr;
        uint32_t* emitterIdPtr = nullptr;
        uint32_t* channelIdPtr = nullptr;
        uint32_t* materialIdPtr = nullptr;
        uint32_t* tickIdPtr = nullptr;
        pxr::GfVec3f* hitNormalPtr = nullptr;
        pxr::GfVec3f* velocityPtr = nullptr;
        uint32_t* objectIdPtr = nullptr;
        uint8_t* echoIdPtr = nullptr;
        uint8_t* tickStatePtr = nullptr;
        float* radialVelocityMSPtr = nullptr;

        // Should not crash when generating buffer
        REQUIRE_NOTHROW(msg->generateBuffer(timestamp, frameId, bufferSize, intensityPtr, timestampPtr, emitterIdPtr,
                                            channelIdPtr, materialIdPtr, tickIdPtr, hitNormalPtr, velocityPtr,
                                            objectIdPtr, echoIdPtr, tickStatePtr, radialVelocityMSPtr));

        const size_t expectedPointStep = sizeof(pxr::GfVec3f);
        CHECK(msg->getTotalBytes() == bufferSize);
        CHECK(msg->getNumPoints() == numPoints);
        CHECK(msg->getPointStep() == expectedPointStep);
        CHECK(msg->getOrderedFields().empty());
    }

    TEST_CASE("Ros2PointCloudMessage: generate buffer for unorganized cloud with partial metadata")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "lidar_frame";
        const size_t numPoints = 10;
        const size_t bufferSize = numPoints * sizeof(pxr::GfVec3f);
        std::array<float, numPoints> intensityData;
        std::array<uint64_t, numPoints> timestampData;
        std::array<pxr::GfVec3f, numPoints> hitNormalData;
        std::array<uint32_t, numPoints> objectIdData;

        REQUIRE_NOTHROW(msg->generateBuffer(timestamp, frameId, bufferSize, intensityData.data(), timestampData.data(),
                                            nullptr, nullptr, nullptr, nullptr, hitNormalData.data(), nullptr,
                                            objectIdData.data(), nullptr, nullptr, nullptr));

        const size_t expectedPointStep =
            sizeof(pxr::GfVec3f) + sizeof(float) + sizeof(uint64_t) + sizeof(pxr::GfVec3f) + sizeof(uint32_t) * 4;
        const size_t expectedBufferSize = numPoints * expectedPointStep;
        CHECK(msg->getTotalBytes() == expectedBufferSize);
        CHECK(msg->getNumPoints() == numPoints);
        CHECK(msg->getPointStep() == expectedPointStep);
        CHECK(msg->getOrderedFields().size() == 4);
        CHECK(msg->getOrderedFields()[0] == std::make_tuple(intensityData.data(), sizeof(float), sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[1] ==
              std::make_tuple(timestampData.data(), sizeof(uint64_t),
                              std::get<1>(msg->getOrderedFields()[0]) + std::get<2>(msg->getOrderedFields()[0])));
        CHECK(msg->getOrderedFields()[2] ==
              std::make_tuple(hitNormalData.data(), sizeof(pxr::GfVec3f),
                              std::get<1>(msg->getOrderedFields()[1]) + std::get<2>(msg->getOrderedFields()[1])));
        CHECK(msg->getOrderedFields()[3] ==
              std::make_tuple(objectIdData.data(), sizeof(uint32_t) * 4,
                              std::get<1>(msg->getOrderedFields()[2]) + std::get<2>(msg->getOrderedFields()[2])));
    }

    TEST_CASE("Ros2PointCloudMessage: generate buffer for unorganized cloud with full metadata")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "lidar_frame";
        const size_t numPoints = 10;
        const size_t bufferSize = numPoints * sizeof(pxr::GfVec3f);
        std::array<float, numPoints> intensityData;
        std::array<uint64_t, numPoints> timestampData;
        std::array<uint32_t, numPoints> emitterIdData;
        std::array<uint32_t, numPoints> channelIdData;
        std::array<uint32_t, numPoints> materialIdData;
        std::array<uint32_t, numPoints> tickIdData;
        std::array<pxr::GfVec3f, numPoints> hitNormalData;
        std::array<pxr::GfVec3f, numPoints> velocityData;
        std::array<uint32_t, numPoints> objectIdData;
        std::array<uint8_t, numPoints> echoIdData;
        std::array<uint8_t, numPoints> tickStateData;
        std::array<float, numPoints> radialVelocityMSData;

        REQUIRE_NOTHROW(msg->generateBuffer(
            timestamp, frameId, bufferSize, intensityData.data(), timestampData.data(), emitterIdData.data(),
            channelIdData.data(), materialIdData.data(), tickIdData.data(), hitNormalData.data(), velocityData.data(),
            objectIdData.data(), echoIdData.data(), tickStateData.data(), radialVelocityMSData.data()));

        const size_t expectedPointStep = sizeof(pxr::GfVec3f) + sizeof(float) + sizeof(uint64_t) + sizeof(uint32_t) +
                                         sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(pxr::GfVec3f) +
                                         sizeof(pxr::GfVec3f) + sizeof(uint32_t) * 4 + sizeof(uint8_t) +
                                         sizeof(uint8_t) + sizeof(float);
        const size_t expectedBufferSize = numPoints * expectedPointStep;
        CHECK(msg->getTotalBytes() == expectedBufferSize);
        CHECK(msg->getNumPoints() == numPoints);
        CHECK(msg->getPointStep() == expectedPointStep);
        CHECK(msg->getOrderedFields().size() == 12);
        CHECK(msg->getOrderedFields()[0] == std::make_tuple(intensityData.data(), sizeof(float), sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[1] ==
              std::make_tuple(timestampData.data(), sizeof(uint32_t) * 2, sizeof(float) + sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[2] == std::make_tuple(emitterIdData.data(), sizeof(uint32_t),
                                                            sizeof(uint32_t) * 2 + sizeof(float) + sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[3] == std::make_tuple(channelIdData.data(), sizeof(uint32_t),
                                                            sizeof(uint32_t) * 3 + sizeof(float) + sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[4] == std::make_tuple(materialIdData.data(), sizeof(uint32_t),
                                                            sizeof(uint32_t) * 4 + sizeof(float) + sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[5] == std::make_tuple(tickIdData.data(), sizeof(uint32_t),
                                                            sizeof(uint32_t) * 5 + sizeof(float) + sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[6] == std::make_tuple(hitNormalData.data(), sizeof(pxr::GfVec3f),
                                                            sizeof(uint32_t) * 6 + sizeof(float) + sizeof(pxr::GfVec3f)));
        CHECK(msg->getOrderedFields()[7] ==
              std::make_tuple(velocityData.data(), sizeof(pxr::GfVec3f),
                              sizeof(uint32_t) * 6 + sizeof(float) + sizeof(pxr::GfVec3f) * 2));
        CHECK(msg->getOrderedFields()[8] ==
              std::make_tuple(objectIdData.data(), sizeof(uint32_t) * 4,
                              sizeof(uint32_t) * 6 + sizeof(float) + sizeof(pxr::GfVec3f) * 3));
        CHECK(msg->getOrderedFields()[9] ==
              std::make_tuple(echoIdData.data(), sizeof(uint8_t),
                              sizeof(uint32_t) * 10 + sizeof(float) + sizeof(pxr::GfVec3f) * 3));
        CHECK(msg->getOrderedFields()[10] ==
              std::make_tuple(tickStateData.data(), sizeof(uint8_t),
                              sizeof(uint8_t) + sizeof(uint32_t) * 10 + sizeof(float) + sizeof(pxr::GfVec3f) * 3));
        CHECK(msg->getOrderedFields()[11] ==
              std::make_tuple(radialVelocityMSData.data(), sizeof(float),
                              sizeof(uint8_t) * 2 + sizeof(uint32_t) * 10 + sizeof(float) + sizeof(pxr::GfVec3f) * 3));
    }

    TEST_CASE("Ros2PointCloudMessage: generate buffer for organized cloud")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        double timestamp = 123.456;
        std::string frameId = "camera_frame";
        size_t width = 640; // Image width
        size_t height = 480; // Image height
        uint32_t pointStep = 16; // x, y, z, intensity * 4 bytes each
        size_t bufferSize = width * height * pointStep;
        float* intensityPtr = nullptr;
        uint64_t* timestampPtr = nullptr;
        uint32_t* emitterIdPtr = nullptr;
        uint32_t* channelIdPtr = nullptr;
        uint32_t* materialIdPtr = nullptr;
        uint32_t* tickIdPtr = nullptr;
        pxr::GfVec3f* hitNormalPtr = nullptr;
        pxr::GfVec3f* velocityPtr = nullptr;
        uint32_t* objectIdPtr = nullptr;
        uint8_t* echoIdPtr = nullptr;
        uint8_t* tickStatePtr = nullptr;
        float* radialVelocityMSPtr = nullptr;

        // Should not crash when generating organized cloud buffer
        CHECK_NOTHROW(msg->generateBuffer(timestamp, frameId, bufferSize, intensityPtr, timestampPtr, emitterIdPtr,
                                          channelIdPtr, materialIdPtr, tickIdPtr, hitNormalPtr, velocityPtr,
                                          objectIdPtr, echoIdPtr, tickStatePtr, radialVelocityMSPtr));
    }

    TEST_CASE("Ros2PointCloudMessage: generate buffer with minimal data")
    {
        ROS2_TEST_SETUP();

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        size_t bufferSize = sizeof(pxr::GfVec3f);
        double timestamp = 0.0;
        std::string frameId = "base_link";
        float* intensityPtr = nullptr;
        uint64_t* timestampPtr = nullptr;
        uint32_t* emitterIdPtr = nullptr;
        uint32_t* channelIdPtr = nullptr;
        uint32_t* materialIdPtr = nullptr;
        uint32_t* tickIdPtr = nullptr;
        pxr::GfVec3f* hitNormalPtr = nullptr;
        pxr::GfVec3f* velocityPtr = nullptr;
        uint32_t* objectIdPtr = nullptr;
        uint8_t* echoIdPtr = nullptr;
        uint8_t* tickStatePtr = nullptr;
        float* radialVelocityMSPtr = nullptr;

        // Should not crash with minimal buffer
        CHECK_NOTHROW(msg->generateBuffer(timestamp, frameId, bufferSize, intensityPtr, timestampPtr, emitterIdPtr,
                                          channelIdPtr, materialIdPtr, tickIdPtr, hitNormalPtr, velocityPtr,
                                          objectIdPtr, echoIdPtr, tickStatePtr, radialVelocityMSPtr));
    }

    TEST_CASE("Ros2PointCloudMessage: use with publisher")
    {
        ROS2_TEST_SETUP();

        // Create context and node
        auto context = testBase.getFactory()->createContextHandle();
        REQUIRE(context != nullptr);
        context->init(0, nullptr);

        auto node = testBase.getFactory()->createNodeHandle("test_pointcloud_node", "test", context.get());
        REQUIRE(node != nullptr);

        auto msg = testBase.getFactory()->createPointCloudMessage();
        REQUIRE(msg != nullptr);

        // Create QoS profile appropriate for point clouds
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 5; // Lower depth for large point cloud messages

        // Create publisher - should not crash
        auto publisher =
            testBase.getFactory()->createPublisher(node.get(), "test_pointcloud", msg->getTypeSupportHandle(), qos);
        CHECK(publisher != nullptr);

        size_t bufferSize = 100 * sizeof(pxr::GfVec3f);
        float* intensityPtr = nullptr;
        uint64_t* timestampPtr = nullptr;
        uint32_t* emitterIdPtr = nullptr;
        uint32_t* channelIdPtr = nullptr;
        uint32_t* materialIdPtr = nullptr;
        uint32_t* tickIdPtr = nullptr;
        pxr::GfVec3f* hitNormalPtr = nullptr;
        pxr::GfVec3f* velocityPtr = nullptr;
        uint32_t* objectIdPtr = nullptr;
        uint8_t* echoIdPtr = nullptr;
        uint8_t* tickStatePtr = nullptr;
        float* radialVelocityMSPtr = nullptr;

        // Generate buffer and publish - should not crash
        msg->generateBuffer(1.0, "lidar_frame", bufferSize, intensityPtr, timestampPtr, emitterIdPtr, channelIdPtr,
                            materialIdPtr, tickIdPtr, hitNormalPtr, velocityPtr, objectIdPtr, echoIdPtr, tickStatePtr,
                            radialVelocityMSPtr);
        if (publisher)
        {
            CHECK_NOTHROW(publisher->publish(msg->getPtr()));
        }
    }
}
