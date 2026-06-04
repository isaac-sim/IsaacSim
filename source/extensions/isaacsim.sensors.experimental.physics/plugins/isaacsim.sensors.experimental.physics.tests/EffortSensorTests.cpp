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
#include <isaacsim/sensors/experimental/physics/IEffortSensor.h>

using namespace isaacsim::sensors::experimental::physics;

TEST_SUITE("isaacsim.sensors.experimental.physics.effort.tests")
{
    TEST_CASE("EffortSensorReading default construction")
    {
        EffortSensorReading reading;
        CHECK_EQ(reading.value, 0.0f);
        CHECK_EQ(reading.time, 0.0f);
        CHECK_FALSE(reading.isValid);
    }

    TEST_CASE("EffortSensorReading value assignment")
    {
        EffortSensorReading reading;
        reading.value = 42.5f;
        reading.time = 1.0f;
        reading.isValid = true;

        CHECK_EQ(reading.value, 42.5f);
        CHECK_EQ(reading.time, 1.0f);
        CHECK(reading.isValid);
    }
}
