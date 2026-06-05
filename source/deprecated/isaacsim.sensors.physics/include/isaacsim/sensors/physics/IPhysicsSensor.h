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

#include "IsaacSensorTypes.h"


namespace isaacsim
{
namespace sensors
{
namespace physics
{

/**
 * @class ContactSensorInterface
 * @brief Interface for contact sensor functionality.
 * @details
 * Provides methods for accessing contact sensor data and raw physics engine data
 * related to contact events between rigid bodies.
 */
struct ContactSensorInterface
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::physics::ContactSensorInterface", 0, 1);


    //! Gets contact raw data from physics engine
    /*! Gets Contact raw data, for validation purposes and ground truth
     * \param primPath path of the sensor prim
     * \param numContacts size of contacts
     * \return Raw Data
     */
    CsRawData*(CARB_ABI* getSensorRawData)(const char* primPath, size_t& numContacts);

    //! Gets Sensor latest simulation
    /*! Check is the prim path contact sensor
     * \param primPath path of the sensor prim
     * \param getLatestValue boolean flag for getting the latest sim value or the last sensor measured value
     * \return time-stamped sensor values
     */
    CsReading(CARB_ABI* getSensorReading)(const char* primPath, const bool& getLatestValue);

    /**
     * @brief Checks if a prim is a contact sensor.
     * @param[in] primPath Path of the prim to check.
     * @return True if the prim is a contact sensor, false otherwise.
     */
    bool(CARB_ABI* isContactSensor)(const char* primPath);

    /**
     * @brief Decodes a rigid body identifier into a human-readable name.
     * @param[in] body Unique identifier of the rigid body.
     * @return Human-readable name of the rigid body.
     */
    const char*(CARB_ABI* decodeBodyName)(uint64_t body);

    //! Gets contact raw data of a rigid body with contact report API from physics engine
    /*! Gets Contact raw data, for validation purposes and ground truth
     * \param primPath path of the rigid body prim
     * \param numContacts size of contacts
     * \return Raw Data
     */
    CsRawData*(CARB_ABI* getBodyRawData)(const char* primPath, size_t& numContacts);
};


/**
 * @class ImuSensorInterface
 * @brief Interface for IMU (Inertial Measurement Unit) sensor functionality.
 * @details
 * Provides methods for accessing IMU sensor data including acceleration,
 * angular velocity, and orientation measurements.
 */
struct ImuSensorInterface
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::physics::ImuSensorInterface", 0, 1);

    //! Gets Sensor last reading
    /*! Gets the sensor last reading on its latest sensor period
     * \param usdPath sensor prim path
     * \param interpolationFunction interpolation functional pointer
     * \param getLatestValue flag for getting the latest sim value or the last sensor measured value
     * \param readGravity flag indicating whether to include gravity in the measurements
     * \return time-stamped sensor values.
     */
    IsReading(CARB_ABI* getSensorReading)(
        const char* usdPath,
        const std::function<isaacsim::sensors::physics::IsReading(std::vector<isaacsim::sensors::physics::IsReading>,
                                                                  float)>& interpolateFunction,
        const bool& getLatestValue,
        const bool& readGravity);

    //! Check is Prim an ImuSensorSchema
    /*! Return True for is, False for is not an ImuSensorSchema
     * \param usdPath sensor prim path
     * \return true for is, false for is not an ImuSensorSchema
     */
    bool(CARB_ABI* isImuSensor)(const char* usdPath);
};

}
}
}
