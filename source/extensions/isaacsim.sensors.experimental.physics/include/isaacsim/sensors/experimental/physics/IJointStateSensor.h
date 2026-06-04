// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

namespace isaacsim::sensors::experimental::physics
{

/**
 * @struct JointStateSensorReading
 * @brief Complete joint state reading for all DOFs of an articulation.
 * @details All array pointers (dofNames, positions, velocities, efforts, dofTypes) point
 * into sensor-internal storage and are valid until the next physics step or
 * sensor destruction. Copy the data before the next simulation step if longer
 * lifetime is needed. DOF indices are 0..dofCount-1; name slots for articulation
 * indices with no discovered joint may be empty. When the backend provides fewer
 * than dofCount velocities or efforts, the remaining elements are reported as zero.
 * dofTypes: 0 = rotation (revolute), 1 = translation (prismatic).
 */
struct JointStateSensorReading
{
    float time{ 0.0f }; ///< Simulation time of this reading in seconds.
    bool isValid{ false }; ///< Whether this reading contains valid data.
    int32_t dofCount{ 0 }; ///< Number of DOFs in the articulation.
    const char* const* dofNames{ nullptr }; ///< Array of DOF name strings, length dofCount.
    const float* positions{ nullptr }; ///< DOF positions (rad or m), length dofCount.
    const float* velocities{ nullptr }; ///< DOF velocities (rad/s or m/s), length dofCount.
    const float* efforts{ nullptr }; ///< DOF efforts (Nm or N), length dofCount.
    const uint8_t* dofTypes{ nullptr }; ///< Per-DOF type: 0 = rotation, 1 = translation; length dofCount.
    float stageMetersPerUnit{ 0.0f }; ///< Stage meters per USD unit for SI conversion (e.g. 1.0 for cm stage).
};

/**
 * @struct IJointStateSensor
 * @brief Carbonite interface for managing C++ joint state sensors.
 * @details Reads all DOF positions, velocities, and efforts from an articulation
 * in a single sensor. Uses IArticulationDataView for engine-agnostic access to
 * articulation joint state data.
 *
 * Unlike EffortSensor (which targets a single joint), this sensor attaches to the
 * articulation root prim and reports all DOFs in articulation order.
 *
 * The plugin is self-driving: it subscribes to PhysX simulation events
 * (eResumed / eStopped) and physics step events internally.
 *
 * Lifecycle:
 * 1. acquire via carb::getCachedInterface or pybind11 acquire function
 * 2. createSensor() with the articulation root prim path
 * 3. getSensorReading() — returns the complete reading (names + positions + velocities + efforts)
 * 4. shutdown() on extension unload
 *
 * When simulation stops (eStopped), all sensors are destroyed and existing sensor IDs
 * become invalid; call createSensor() again after resume if needed.
 *
 * Not thread-safe: call createSensor(), removeSensor(), and getSensorReading() from a
 * single thread, or synchronize externally.
 */
struct IJointStateSensor
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::experimental::physics::IJointStateSensor", 2, 0);

    /**
     * @brief Shut down the manager, destroying all sensors and freeing resources.
     */
    virtual void shutdown() = 0;

    /**
     * @brief Create a joint state sensor for the given articulation root prim.
     * @details If a sensor already exists for this prim path the call succeeds
     * without creating a duplicate.
     * @param articulationRootPath USD path to the articulation root prim.
     * @return true on success (sensor created or already exists), false on failure.
     */
    virtual bool createSensor(const char* articulationRootPath) = 0;

    /**
     * @brief Remove a sensor and free its resources.
     * @param articulationRootPath USD path used when the sensor was created.
     */
    virtual void removeSensor(const char* articulationRootPath) = 0;

    /**
     * @brief Get the complete joint state reading for a sensor.
     * @details On a valid reading, all array pointers in the returned struct point into
     * sensor-internal storage and remain valid until the next physics step or sensor destruction.
     * @param articulationRootPath USD path used when the sensor was created.
     * @return Complete reading with dofCount, dofNames, positions, velocities, and efforts.
     *         isValid is false before the first physics step or if the sensor is disabled.
     */
    virtual JointStateSensorReading getSensorReading(const char* articulationRootPath) = 0;
};

} // namespace isaacsim::sensors::experimental::physics
