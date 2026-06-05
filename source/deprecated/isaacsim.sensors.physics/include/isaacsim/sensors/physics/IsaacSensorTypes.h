// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <usdrt/gf/matrix.h>
// #include <PxActor.h>
#include <limits>

namespace isaacsim
{
namespace sensors
{
namespace physics
{
/**
 * @struct CsProperties
 * @brief Properties configuration for a contact sensor.
 */
struct CsProperties
{
    /** @brief Radius from the sensor position. Negative values indicate it's a full body sensor. */
    float radius{ 0.0f };

    /** @brief Minimum force that the sensor can read. Forces below this value will not trigger a reading. */
    float minThreshold{ 0.0f };

    /** @brief Maximum force that the sensor can register. Forces above this value will be clamped. */
    float maxThreshold{ 0.0f };

    /** @brief Sensor reading speed, in seconds. Zero means sync with simulation timestep. */
    float sensorPeriod{ 0.0f };
};

/**
 * @struct CsReading
 * @brief Contact sensor reading data structure.
 */
struct CsReading
{
    /** @brief Simulation timestamp for contact sensor reading. */
    float time{ 0.0f };

    /** @brief Reading value, in Newtons. */
    float value{ 0.0f };

    /** @brief Flag that checks if the sensor is in contact with something or not. */
    bool inContact{ false };

    /** @brief Validity of the data. False for when the sensor is disabled, true for enabled. */
    bool isValid{ false };
};

/**
 * @struct CsRawData
 * @brief Raw contact data from the physics simulation.
 */
struct CsRawData
{
    /** @brief Simulation timestamp. */
    float time{ 0.0f };

    /** @brief Simulation time step for the impulse. */
    float dt{ 0.0f };

    /** @brief First body involved in the contact. */
    uint64_t body0;

    /** @brief Second body involved in the contact. */
    uint64_t body1;

    /** @brief Contact position in world coordinates. */
    carb::Float3 position{ 0.0f, 0.0f, 0.0f };

    /** @brief Contact normal in world coordinates. */
    carb::Float3 normal{ 0.0f, 0.0f, 0.0f };

    /** @brief Contact impulse in world coordinates. */
    carb::Float3 impulse{ 0.0f, 0.0f, 0.0f };
};

/**
 * @struct IsProperties
 * @brief Properties configuration for an IMU sensor.
 */
struct IsProperties
{
    /** @brief Orientation matrix relative to the parent body where the sensor is placed. */
    usdrt::GfMatrix3d orientation;

    /** @brief Sensor reading speed, in seconds. Zero means sync with simulation timestep. */
    float sensorPeriod{ 0.0f };
};

/**
 * @struct IsReading
 * @brief IMU sensor reading data structure.
 */
struct IsReading
{
    /** @brief Simulation timestamp for IMU sensor reading. */
    float time{ 0.0f };

    /** @brief Accelerometer reading value x axis, in m/s^2. */
    float linAccX{ 0.0f };

    /** @brief Accelerometer reading value y axis, in m/s^2. */
    float linAccY{ 0.0f };

    /** @brief Accelerometer reading value z axis, in m/s^2. */
    float linAccZ{ 0.0f };

    /** @brief Gyroscope reading value x axis, in rad/s. */
    float angVelX{ 0.0f };

    /** @brief Gyroscope reading value y axis, in rad/s. */
    float angVelY{ 0.0f };

    /** @brief Gyroscope reading value z axis, in rad/s. */
    float angVelZ{ 0.0f };

    /** @brief Quaternion orientation of parent body (x, y, z, w). */
    carb::Float4 orientation{ 0.0f, 0.0f, 0.0f, 0.0f };

    /** @brief Validity of the data. False for when the sensor is disabled, true for enabled. */
    bool isValid{ false };
};

/**
 * @struct IsRawData
 * @brief Raw IMU data from the physics simulation.
 */
struct IsRawData
{
    /** @brief Simulation timestamp. */
    float time{ 0.0f };

    /** @brief Simulation time step for the impulse. */
    float dt{ 0.0f };

    /** @brief Linear velocity x raw reading value, in m/s. */
    float linVelX{ 0.0f };

    /** @brief Linear velocity y raw reading value, in m/s. */
    float linVelY{ 0.0f };

    /** @brief Linear velocity z raw reading value, in m/s. */
    float linVelZ{ 0.0f };

    /** @brief Angular velocity x raw reading value, in rad/s. */
    float angVelX{ 0.0f };

    /** @brief Angular velocity y raw reading value, in rad/s. */
    float angVelY{ 0.0f };

    /** @brief Angular velocity z raw reading value, in rad/s. */
    float angVelZ{ 0.0f };

    /** @brief Quaternion orientation of parent body (w, x, y, z). */
    carb::Float4 orientation{ 0.0f, 0.0f, 0.0f, 0.0f };
};

}
}
}
