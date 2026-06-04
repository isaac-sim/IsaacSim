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

#include <carb/BindingsUtils.h>

#include <doctest/doctest.h>
#include <isaacsim/sensors/experimental/physics/IImuSensor.h>

#include <cmath>
#include <vector>

CARB_BINDINGS("isaacsim.sensors.experimental.physics.tests")

using namespace isaacsim::sensors::experimental::physics;

TEST_SUITE("isaacsim.sensors.experimental.physics.tests")
{
    TEST_CASE("ImuSensorReading default construction")
    {
        ImuSensorReading reading;
        CHECK_EQ(reading.linearAccelerationX, 0.0f);
        CHECK_EQ(reading.linearAccelerationY, 0.0f);
        CHECK_EQ(reading.linearAccelerationZ, 0.0f);
        CHECK_EQ(reading.angularVelocityX, 0.0f);
        CHECK_EQ(reading.angularVelocityY, 0.0f);
        CHECK_EQ(reading.angularVelocityZ, 0.0f);
        CHECK_EQ(reading.orientationW, 1.0f);
        CHECK_EQ(reading.orientationX, 0.0f);
        CHECK_EQ(reading.orientationY, 0.0f);
        CHECK_EQ(reading.orientationZ, 0.0f);
        CHECK_EQ(reading.time, 0.0f);
        CHECK_FALSE(reading.isValid);
    }

    TEST_CASE("ImuRawData default construction")
    {
        ImuRawData raw;
        CHECK_EQ(raw.time, 0.0f);
        CHECK_EQ(raw.dt, 0.0f);
        CHECK_EQ(raw.linearVelocityX, 0.0f);
        CHECK_EQ(raw.linearVelocityY, 0.0f);
        CHECK_EQ(raw.linearVelocityZ, 0.0f);
        CHECK_EQ(raw.angularVelocityX, 0.0f);
        CHECK_EQ(raw.angularVelocityY, 0.0f);
        CHECK_EQ(raw.angularVelocityZ, 0.0f);
        CHECK_EQ(raw.orientationW, 1.0f);
        CHECK_EQ(raw.orientationX, 0.0f);
        CHECK_EQ(raw.orientationY, 0.0f);
        CHECK_EQ(raw.orientationZ, 0.0f);
    }

    TEST_CASE("Circular buffer push and pop pattern")
    {
        const int bufferSize = 5;
        std::vector<ImuRawData> buffer(bufferSize, ImuRawData());

        for (int step = 0; step < 10; ++step)
        {
            buffer.pop_back();
            buffer.insert(buffer.begin(), ImuRawData());
            buffer[0].time = static_cast<float>(step);
            buffer[0].linearVelocityX = static_cast<float>(step) * 0.1f;
        }

        CHECK_EQ(buffer.size(), bufferSize);
        CHECK_EQ(buffer[0].time, 9.0f);
        CHECK_EQ(buffer[1].time, 8.0f);
        CHECK_EQ(buffer[4].time, 5.0f);
    }

    TEST_CASE("Angular velocity rolling average filter")
    {
        const int bufferSize = 10;
        std::vector<ImuRawData> buffer(bufferSize, ImuRawData());

        for (int i = 0; i < bufferSize; ++i)
        {
            buffer[i].angularVelocityZ = static_cast<float>(i + 1);
        }

        int filterSize = 3;
        float sum = 0.0f;
        for (int i = 0; i < filterSize; i++)
        {
            sum += buffer[i].angularVelocityZ;
        }
        float average = sum / filterSize;

        CHECK(std::abs(average - 2.0f) < 1e-6f);
    }

    TEST_CASE("Linear acceleration finite difference")
    {
        const int bufferSize = 20;
        std::vector<ImuRawData> buffer(bufferSize, ImuRawData());

        float dt = 1.0f / 60.0f;
        float acceleration = 5.0f;
        for (int i = 0; i < bufferSize; ++i)
        {
            buffer[i].time = static_cast<float>(bufferSize - 1 - i) * dt;
            buffer[i].linearVelocityX = acceleration * buffer[i].time;
        }

        int filterSize = 1;
        float accelerationSum = 0.0f;
        for (int i = 0; i < filterSize; i++)
        {
            float timeDiff = buffer[i].time - buffer[i + filterSize].time;
            if (timeDiff > 1e-10f)
            {
                accelerationSum += (buffer[i].linearVelocityX - buffer[i + filterSize].linearVelocityX) / timeDiff;
            }
        }
        accelerationSum /= filterSize;

        CHECK(std::abs(accelerationSum - acceleration) < 0.5f);
    }

    TEST_CASE("Orientation quaternion normalization")
    {
        float w = 2.0f, x = 0.0f, y = 0.0f, z = 0.0f;
        float norm = std::sqrt(w * w + x * x + y * y + z * z);
        CHECK(norm > 0.0f);
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;

        float normalized = std::sqrt(w * w + x * x + y * y + z * z);
        CHECK(std::abs(normalized - 1.0f) < 1e-6f);
        CHECK(std::abs(w - 1.0f) < 1e-6f);
    }

    TEST_CASE("Buffer resize on filter width change")
    {
        int rawBufferSize = 20;
        std::vector<ImuRawData> rawBuffer(rawBufferSize, ImuRawData());
        std::vector<ImuSensorReading> readings(rawBufferSize, ImuSensorReading());

        int linearFilter = 15;
        int angularFilter = 5;
        int orientationFilter = 5;

        int maxRolling = std::max({ linearFilter, angularFilter, orientationFilter });
        int desiredSize = std::max(2 * maxRolling, 20);

        CHECK_EQ(desiredSize, 30);

        if (desiredSize != rawBufferSize)
        {
            rawBufferSize = desiredSize;
            rawBuffer.assign(rawBufferSize, ImuRawData());
            readings.assign(rawBufferSize, ImuSensorReading());
        }

        CHECK_EQ(rawBuffer.size(), 30);
        CHECK_EQ(readings.size(), 30);
    }
}
