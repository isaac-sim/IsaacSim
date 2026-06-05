// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaacsim/sensors/physics/ContactManager.h"
#include "isaacsim/sensors/physics/IsaacSensorComponent.h"

#include <isaacsim/robot/schema/sensor_tokens.h>
#include <isaacsim/sensors/physics/IPhysicsSensor.h>
#include <pxr/base/gf/vec4d.h>
#include <usdrt/gf/matrix.h>
#include <usdrt/gf/vec.h>

#include <cmath>
#include <vector>

#define PI 3.14159265

namespace isaacsim
{
namespace sensors
{
namespace physics
{
/**
 * @class ContactSensor
 * @brief Component for simulating contact sensors in the physics environment.
 * @details
 * Manages a contact sensor that can detect and measure forces between physical bodies.
 * Inherits from IsaacBaseSensorComponent to integrate with the sensor framework.
 */
class ContactSensor : public IsaacBaseSensorComponent
{
public:
    /**
     * @brief Constructor for ContactSensor.
     * @param[in] physXInterface Pointer to the PhysX interface.
     * @param[in] contactManager Pointer to the contact manager instance.
     */
    ContactSensor(omni::physx::IPhysx* physXInterface, ContactManager* contactManager) : IsaacBaseSensorComponent()
    {
        m_physXInterfacePtr = physXInterface;
        m_contactManagerPtr = contactManager;
    }

    /**
     * @brief Virtual destructor.
     */
    virtual ~ContactSensor();

    /**
     * @brief Resets the sensor to its initial state.
     */
    void reset();

    /**
     * @brief Gets the raw contact data from the sensor.
     * @param[out] size Number of contact data points.
     * @return Pointer to array of raw contact data.
     */
    CsRawData* getRawData(size_t& size);

    /**
     * @brief Gets the processed sensor reading.
     * @param[in] getLatestValue If true, returns the latest simulation value instead of the last sensor reading.
     * @return Processed contact sensor reading.
     */
    CsReading getSensorReading(const bool& getLatestValue = false);

    /**
     * @brief Processes raw contact data into sensor readings.
     * @param[in] rawContact Array of raw contact data.
     * @param[in] size Number of contact data points.
     * @param[in] index Index indicating data recency (0 for old, 1 for new).
     * @param[in] time Current simulation time.
     */
    void processRawContacts(CsRawData* rawContact, const size_t& size, const size_t& index, const double& time);

    /**
     * @brief Called each physics step to update sensor state.
     */
    virtual void onPhysicsStep();

    /**
     * @brief Called each tick to update sensor state.
     * @note onPhysicsStep is used to update the sensor state.
     */
    virtual void tick()
    {
    }

    /**
     * @brief Sets up the contact report API for the sensor.
     */
    void setContactReportApi();

    /**
     * @brief Finds a valid parent body for the sensor.
     * @return True if a valid parent was found, false otherwise.
     */
    bool findValidParent();

    /**
     * @brief Handles component property changes.
     */
    void onComponentChange();

    /**
     * @brief Called when the simulation stops.
     * @details Redraws the sensor after stopping, unlike the base class implementation.
     */
    virtual void onStop();

    /**
     * @brief Debug function to print raw contact data.
     * @param[in] data Pointer to raw contact data to print.
     */
    void printRawData(CsRawData* data);

    /**
     * @brief Debug function to print the current reading pair.
     */
    void printReadingPair();

private:
    /** @brief Color for visualization (RGBA). */
    pxr::GfVec4f m_color = pxr::GfVec4f(1.0f, 1.0f, 1.0f, 1.0f);

    /** @brief Sensor properties configuration. */
    CsProperties m_props;

    /** @brief Number of contact data points. */
    size_t m_size;

    /** @brief Array of two readings for interpolation (current and previous). */
    CsReading m_readingPair[2]; // Data obtained on simulation timestamp

    /** @brief Latest processed sensor reading. */
    CsReading m_sensorReading;

    /** @brief Raw contact data from physics simulation. */
    CsRawData* m_contactsRawData = nullptr;

    /** @brief Pointer to the contact manager instance. */
    ContactManager* m_contactManagerPtr = nullptr;

    /** @brief Pointer to the PhysX interface. */
    omni::physx::IPhysx* m_physXInterfacePtr = nullptr;

    /** @brief Flag indicating which reading in the pair is current. */
    bool m_current{ 0 };

    /** @brief Previous enabled state of the sensor. */
    bool m_previousEnabled{ true };

    /** @brief Current simulation time. */
    float m_currentTime{ 0 };

    /** @brief Last sensor reading time. */
    float m_sensorTime{ 0 };
};
}
}
}
