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

#include <doctest/doctest.h>
#include <isaacsim/sensors/experimental/physics/IRaycastSensor.h>

#include <cmath>
#include <vector>

using namespace isaacsim::sensors::experimental::physics;

TEST_SUITE("isaacsim.sensors.experimental.physics.tests.raycast")
{
    TEST_CASE("RaycastSensorReading default construction")
    {
        RaycastSensorReading reading;
        CHECK_EQ(reading.rayCount, 0);
        CHECK_EQ(reading.depths, nullptr);
        CHECK_EQ(reading.hitPositions, nullptr);
        CHECK_EQ(reading.hitNormals, nullptr);
        CHECK_EQ(reading.hitPrimPaths, nullptr);
        CHECK_EQ(reading.rayOriginsWorld, nullptr);
        CHECK_EQ(reading.rayEndPointsWorld, nullptr);
        CHECK_EQ(reading.time, 0.0f);
        CHECK_FALSE(reading.isValid);
    }

    TEST_CASE("RaycastSensorReading with populated arrays")
    {
        std::vector<float> depths = { 5.0f, 10.0f, 100.0f };
        std::vector<float> hitPos = { 5.0f, 0.0f, 1.0f, 10.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
        std::vector<float> hitNorm = { -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        std::vector<float> origins = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f };
        std::vector<float> endpoints = { 5.0f, 0.0f, 1.0f, 10.0f, 0.0f, 1.0f, 100.0f, 0.0f, 1.0f };

        RaycastSensorReading reading;
        reading.rayCount = 3;
        reading.depths = depths.data();
        reading.hitPositions = hitPos.data();
        reading.hitNormals = hitNorm.data();
        reading.rayOriginsWorld = origins.data();
        reading.rayEndPointsWorld = endpoints.data();
        reading.time = 1.5f;
        reading.isValid = true;

        CHECK_EQ(reading.rayCount, 3);
        CHECK(reading.isValid);
        CHECK_EQ(reading.time, 1.5f);

        CHECK_EQ(reading.depths[0], 5.0f);
        CHECK_EQ(reading.depths[1], 10.0f);
        CHECK_EQ(reading.depths[2], 100.0f);

        for (uint32_t i = 0; i < reading.rayCount; ++i)
        {
            CHECK_EQ(reading.rayOriginsWorld[i * 3 + 2], 1.0f);
        }

        CHECK_EQ(reading.rayEndPointsWorld[0 * 3 + 0], 5.0f);
        CHECK_EQ(reading.rayEndPointsWorld[2 * 3 + 0], 100.0f);
    }

    TEST_CASE("Sweep period computation from time offsets")
    {
        std::vector<float> timeOffsets = { 0.0f, 0.1f, 0.2f, 0.3f, 0.5f, 0.7f, 0.9f };

        float sweepPeriod = 0.0f;
        for (float t : timeOffsets)
        {
            if (t > sweepPeriod)
            {
                sweepPeriod = t;
            }
        }
        CHECK(std::abs(sweepPeriod - 0.9f) < 1e-6f);
    }

    TEST_CASE("Time-windowed ray selection")
    {
        // Use 37 rays (prime, coprime with 60) so no ray offset lands exactly
        // on a window boundary, avoiding FP precision edge cases.
        const int numRays = 37;
        const float sweepPeriod = 1.0f;
        std::vector<float> timeOffsets(numRays);
        for (int i = 0; i < numRays; ++i)
        {
            timeOffsets[i] = sweepPeriod * static_cast<float>(i) / static_cast<float>(numRays);
        }

        float dt = 1.0f / 60.0f;
        const uint64_t stepsPerPeriod = 60;

        int totalActive = 0;
        std::vector<bool> everSelected(numRays, false);

        for (uint64_t stepCount = 1; stepCount <= stepsPerPeriod; ++stepCount)
        {
            float sweepTime = std::fmod(static_cast<float>(stepCount) * dt, sweepPeriod);
            float windowEnd = sweepTime;
            float windowStart = windowEnd - dt;
            bool windowWrapped = (windowStart < 0.0f);
            if (windowWrapped)
            {
                windowStart += sweepPeriod;
            }

            int activeCount = 0;
            for (int i = 0; i < numRays; ++i)
            {
                bool active = windowWrapped ? (timeOffsets[i] >= windowStart || timeOffsets[i] < windowEnd) :
                                              (timeOffsets[i] >= windowStart && timeOffsets[i] < windowEnd);
                if (active)
                {
                    ++activeCount;
                    everSelected[i] = true;
                }
            }

            CHECK(activeCount < numRays);
            totalActive += activeCount;
        }

        CHECK(totalActive >= numRays);
        for (int i = 0; i < numRays; ++i)
        {
            CHECK(everSelected[i]);
        }
    }

    TEST_CASE("Sweep time wraps around period")
    {
        float sweepPeriod = 1.0f;
        float dt = 1.0f / 60.0f;

        for (uint64_t step = 1; step <= 120; ++step)
        {
            float sweepTime = std::fmod(static_cast<float>(step) * dt, sweepPeriod);
            CHECK(sweepTime >= 0.0f);
            CHECK(sweepTime < sweepPeriod);
        }
    }

    TEST_CASE("Hit depth within min/max range")
    {
        float minRange = 0.4f;
        float maxRange = 100.0f;

        std::vector<float> rawDepths = { 0.1f, 0.5f, 50.0f, 150.0f };
        std::vector<float> clampedDepths(rawDepths.size());

        for (size_t i = 0; i < rawDepths.size(); ++i)
        {
            if (rawDepths[i] < minRange || rawDepths[i] > maxRange)
            {
                clampedDepths[i] = maxRange;
            }
            else
            {
                clampedDepths[i] = rawDepths[i];
            }
        }

        CHECK_EQ(clampedDepths[0], maxRange);
        CHECK_EQ(clampedDepths[1], 0.5f);
        CHECK_EQ(clampedDepths[2], 50.0f);
        CHECK_EQ(clampedDepths[3], maxRange);
    }

    TEST_CASE("Ray direction normalization")
    {
        float dx = 1.0f, dy = 1.0f, dz = 0.0f;
        float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        CHECK(len > 0.0f);
        dx /= len;
        dy /= len;
        dz /= len;

        float normalized = std::sqrt(dx * dx + dy * dy + dz * dz);
        CHECK(std::abs(normalized - 1.0f) < 1e-6f);
        CHECK(std::abs(dx - 0.70710678f) < 1e-5f);
        CHECK(std::abs(dy - 0.70710678f) < 1e-5f);
    }

    TEST_CASE("World-space endpoint computation")
    {
        float originX = 1.0f, originY = 2.0f, originZ = 3.0f;
        float dirX = 1.0f, dirY = 0.0f, dirZ = 0.0f;
        float depth = 5.0f;

        float endX = originX + dirX * depth;
        float endY = originY + dirY * depth;
        float endZ = originZ + dirZ * depth;

        CHECK_EQ(endX, 6.0f);
        CHECK_EQ(endY, 2.0f);
        CHECK_EQ(endZ, 3.0f);
    }
}
