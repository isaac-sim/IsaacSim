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

#pragma once

#include <carb/Interface.h>

#include <cstdint>

namespace isaacsim
{
namespace sensors
{
namespace experimental
{
namespace physics
{

/**
 * @struct ContactSensorReading
 * @brief Contact sensor reading with force value and contact state.
 */
struct ContactSensorReading
{
    float time{ 0.0f }; ///< Simulation time of this reading in seconds.
    float value{ 0.0f }; ///< Contact force magnitude in Newtons.
    bool inContact{ false }; ///< Whether the sensor is currently in contact.
    bool isValid{ false }; ///< Whether this reading contains valid data.
};

/**
 * @struct ContactRawData
 * @brief Raw contact data from a single contact point.
 */
struct ContactRawData
{
    uint64_t body0{ 0 }; ///< First body involved in the contact.
    uint64_t body1{ 0 }; ///< Second body involved in the contact.
    float positionX{ 0.0f }; ///< Contact position X in world coordinates.
    float positionY{ 0.0f }; ///< Contact position Y in world coordinates.
    float positionZ{ 0.0f }; ///< Contact position Z in world coordinates.
    float normalX{ 0.0f }; ///< Contact normal X in world coordinates.
    float normalY{ 0.0f }; ///< Contact normal Y in world coordinates.
    float normalZ{ 0.0f }; ///< Contact normal Z in world coordinates.
    float impulseX{ 0.0f }; ///< Contact impulse X in world coordinates.
    float impulseY{ 0.0f }; ///< Contact impulse Y in world coordinates.
    float impulseZ{ 0.0f }; ///< Contact impulse Z in world coordinates.
    float time{ 0.0f }; ///< Simulation time of this contact in seconds.
    float dt{ 0.0f }; ///< Physics timestep for this contact.
};

/**
 * @struct IContactSensor
 * @brief Carbonite interface for managing C++ contact sensors.
 * @details Processes raw PhysX contact reports to produce filtered sensor
 * readings. Supports configurable radius filtering and force thresholds.
 *
 * The plugin is self-driving: it subscribes to PhysX simulation events
 * (eResumed / eStopped) and physics step events internally, so it
 * initializes, discovers sensors, and processes each substep without
 * any Python relay.
 *
 * Lifecycle:
 * 1. acquire via carb::getCachedInterface or pybind11 acquire function
 * 2. createSensor() for each IsaacContactSensor prim (plugin self-initializes on eResumed)
 * 3. getSensorReading() to read data (sensors update automatically each substep)
 * 4. getRawContacts() to read raw contact data for a sensor
 * 5. shutdown() on extension unload
 */
struct IContactSensor
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::experimental::physics::IContactSensor", 2, 0);

    /**
     * @brief Shut down the manager, destroying all sensors and freeing resources.
     */
    virtual void shutdown() = 0;

    /**
     * @brief Create a contact sensor for the given IsaacContactSensor prim.
     * @details Finds the parent rigid body by walking up the prim hierarchy.
     * If a sensor already exists for this prim path the call succeeds without
     * creating a duplicate.
     *
     * @note Side effect: This method modifies the USD stage by applying
     * PhysxSchemaPhysxContactReportAPI on the parent rigid body (with
     * threshold=0 and sleepThreshold=0). This is required for PhysX
     * getFullContactReport() to return contact data for this body.
     *
     * @param primPath USD path to the IsaacContactSensor prim.
     * @return true on success (sensor created or already exists), false on failure.
     */
    virtual bool createSensor(const char* primPath) = 0;

    /**
     * @brief Remove a sensor and free its resources.
     * @param primPath USD path used when the sensor was created.
     */
    virtual void removeSensor(const char* primPath) = 0;

    /**
     * @brief Get the latest reading for a sensor.
     * @param primPath USD path used when the sensor was created.
     * @return Sensor reading. isValid is false if sensor is disabled or not found.
     */
    virtual ContactSensorReading getSensorReading(const char* primPath) = 0;

    /**
     * @brief Get raw contact data for a sensor's parent body.
     * @param primPath USD path used when the sensor was created.
     * @param outData Pointer to receive the raw contact data array.
     * @param outCount Pointer to receive the number of raw contact entries.
     */
    virtual void getRawContacts(const char* primPath, const ContactRawData** outData, int32_t* outCount) = 0;
};

} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim
