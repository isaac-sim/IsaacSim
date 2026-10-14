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
 * @struct RaycastSensorReading
 * @brief Raycast sensor reading with per-ray depths, hit positions, normals, and prim paths.
 *
 * Arrays are owned by the C++ plugin and valid until the next physics step or
 * sensor removal. The rayCount field indicates the length of each array.
 */
struct RaycastSensorReading
{
    uint32_t rayCount{ 0 }; ///< Number of rays (length of each per-ray array).
    const float* depths{ nullptr }; ///< Per-ray depth in stage length units (maxRange on miss).
    const float* hitPositions{ nullptr }; ///< Per-ray hit positions as flat [x,y,z, x,y,z, ...] (3 * rayCount).
    const float* hitNormals{ nullptr }; ///< Per-ray surface normals as flat [x,y,z, ...] (3 * rayCount).
    const char* const* hitPrimPaths{ nullptr }; ///< Per-ray hit prim USD paths (nullptr unless reportHitPrimPaths).
    const float* rayOriginsWorld{ nullptr }; ///< Per-ray world-space origins as flat [x,y,z, ...] (3 * rayCount).
    const float* rayEndPointsWorld{ nullptr }; ///< Per-ray world-space endpoints as flat [x,y,z, ...] (3 * rayCount).
    float time{ 0.0f }; ///< Simulation time of this reading in seconds.
    bool isValid{ false }; ///< Whether this reading contains valid data.
};

/**
 * @class RaycastSensorReadingView
 * @brief Read-only live view of raycast sensor output buffers.
 * @details The view retains its backing buffers until release(). Numeric pointers remain stable for the view lifetime,
 * while their values and getTime() update on each physics step. isValid() becomes false when the sensor is removed,
 * reset, stopped, or otherwise loses a valid reading. Previously returned buffers remain readable and contain their
 * last values after invalidation.
 *
 * The view performs no synchronization with the physics step that writes these buffers. Reading it while a step is in
 * flight yields torn values that mix the current and the previous reading, so read it only from the thread that drives
 * physics and only between steps. Use getSensorReading() instead when a reader cannot be ordered against the step.
 */
class RaycastSensorReadingView
{
public:
    /** @brief Get the number of rays represented by each output buffer. */
    virtual uint32_t getRayCount() const = 0;

    /** @brief Get per-ray depths in stage length units. */
    virtual const float* getDepths() const = 0;

    /** @brief Get flat per-ray hit positions with three values per ray. */
    virtual const float* getHitPositions() const = 0;

    /** @brief Get flat per-ray hit normals with three values per ray. */
    virtual const float* getHitNormals() const = 0;

    /**
     * @brief Get per-ray hit prim USD paths, or nullptr when path reporting is disabled.
     * @details Each path pointer stays dereferenceable for the view lifetime even once a later step replaces the path,
     * so a concurrent step can tear the characters a reader observes but cannot dangle the pointer. Load each element
     * once and copy the string before dereferencing it again.
     */
    virtual const char* const* getHitPrimPaths() const = 0;

    /** @brief Get flat per-ray world-space origins with three values per ray. */
    virtual const float* getRayOriginsWorld() const = 0;

    /** @brief Get flat per-ray world-space endpoints with three values per ray. */
    virtual const float* getRayEndPointsWorld() const = 0;

    /** @brief Get the simulation time of the currently exposed reading in seconds. */
    virtual float getTime() const = 0;

    /** @brief Check whether the sensor and its current reading remain valid. */
    virtual bool isValid() const = 0;

    /** @brief Release the view and its retained backing buffers. */
    virtual void release() = 0;

protected:
    virtual ~RaycastSensorReadingView() = default;
};

/**
 * @struct IRaycastSensor
 * @brief Carbonite interface for managing C++ raycast sensors.
 * @details Casts rays per physics step using the physics umbrella API
 * (PhysX C++ or Newton Python fallback) and stores per-ray results.
 *
 * The plugin is self-driving: it subscribes to PhysX simulation events
 * (eResumed / eStopped) and physics step events internally, so it
 * initializes, discovers sensors, and processes each substep without
 * any Python relay.
 *
 * Lifecycle:
 * 1. acquire via carb::getCachedInterface or pybind11 acquire function
 * 2. createSensor() for each IsaacRaycastSensor prim (plugin self-initializes on eResumed)
 * 3. getSensorReading() to read data (sensors update automatically each substep)
 * 4. shutdown() on extension unload
 */
struct IRaycastSensor
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::experimental::physics::IRaycastSensor", 2, 1);

    /**
     * @brief Shut down the manager, destroying all sensors and freeing resources.
     */
    virtual void shutdown() = 0;

    /**
     * @brief Create a raycast sensor for the given IsaacRaycastSensor prim.
     * @param primPath USD path to the IsaacRaycastSensor prim.
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
     * @return Sensor reading. isValid is false if sensor is disabled, errored, or not found.
     */
    virtual RaycastSensorReading getSensorReading(const char* primPath) = 0;

    /**
     * @brief Acquire a retained read-only live view of the latest sensor buffers.
     * @details The returned view owns a reference to the sensor buffers and must be released with
     * RaycastSensorReadingView::release(). The view is invalid when no valid reading is available.
     * @param primPath USD path used when the sensor was created.
     * @return Retained view object, or nullptr if allocation fails.
     */
    virtual RaycastSensorReadingView* getSensorReadingView(const char* primPath) = 0;
};

} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim
