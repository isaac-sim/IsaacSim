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

#include <isaacsim/robot/schema/sensor_tokens.h>
#include <isaacsim/sensors/physics/IPhysicsSensor.h>
#include <isaacsim/sensors/physics/IsaacSensorComponent.h>
#include <omni/physx/IPhysx.h>
#include <usdrt/gf/matrix.h>
#include <usdrt/gf/vec.h>

#include <map>
#include <memory>
#include <vector>

namespace isaacsim
{
namespace sensors
{
namespace physics
{

/**
 * @class ImuSensor
 * @brief Implementation of an Inertial Measurement Unit (IMU) sensor component
 * @details This class provides functionality for simulating an IMU sensor in the Isaac environment.
 *          It handles acceleration, angular velocity, and orientation measurements, with support
 *          for raw data buffering and interpolation. The sensor supports configurable filtering
 *          for various measurements and maintains a history of readings for signal processing.
 */
class ImuSensor : public IsaacBaseSensorComponent
{
public:
    /**
     * @brief Default constructor for the IMU sensor
     * @details Initializes the raw data buffer and resets the sensor to its initial state
     */
    ImuSensor() : IsaacBaseSensorComponent()
    {
        m_rawBuffer.resize(m_rawBufferSize, IsRawData());
        reset();
    }

    /**
     * @brief Initializes the IMU sensor with a data buffer
     * @param[in] rigidBodyDataBuffer Pointer to the buffer storing rigid body data
     * @param[in,out] m_dataBufferIndex Reference to the current buffer index
     */
    void initialize(float* rigidBodyDataBuffer, size_t& m_dataBufferIndex);

    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~ImuSensor();

    /**
     * @brief Retrieves the current sensor reading with optional interpolation
     * @param[in] interpolateFunction Optional function for interpolating between readings
     * @param[in] getLatestValue Whether to return the most recent value without interpolation
     * @param[in] readGravity Whether to include gravity in the acceleration readings
     * @return The current IMU sensor reading
     */
    IsReading getSensorReading(const std::function<IsReading(std::vector<IsReading>, float)>& interpolateFunction = nullptr,
                               const bool& getLatestValue = false,
                               const bool& readGravity = true);

    /**
     * @brief Resets the sensor to its initial state
     * @details Clears all buffers and resets internal state variables to their default values
     */
    void reset();

    /**
     * @brief Called by component manager tick to update sensor data
     * @details Processes finite difference data in mRawReadingList and saves in m_readingPair
     */
    virtual void onPhysicsStep();

    /**
     * @brief Empty tick implementation as processing is done in onPhysicsStep
     */
    virtual void tick()
    {
    }

    /**
     * @brief Searches for and validates the parent rigid body for the IMU sensor
     * @return True if a valid parent is found, false otherwise
     */
    bool findValidParent();

    /**
     * @brief Handles component property changes
     * @details Updates sensor configuration when properties are modified through the interface
     */
    void onComponentChange();

    /**
     * @brief Called when simulation is stopped
     * @details The virtual onStop will clear everything on stop
     */
    virtual void onStop();

    /**
     * @brief Utility function to print IMU sensor reading data for debugging
     * @param[in] reading The sensor reading to print
     */
    void printIsReading(const IsReading& reading);

private:
    /** @brief Properties structure for the IMU sensor */
    IsProperties m_props;

    /** @brief Size of raw data buffer, must be larger than 2*m_linearAccelerationFilterSize */
    int m_rawBufferSize = 20;

    /** @brief Current index in the data buffer */
    size_t m_dataBufferIndex = 0;

    /** @brief Pointer to the rigid body data buffer */
    float* m_rigidBodyDataBuffer = nullptr;

    /** @brief Initial buffer state */
    IsRawData m_initBuffer;

    /** @brief Raw velocities data array */
    std::vector<IsRawData> m_rawBuffer;

    /** @brief Moving average window size for angular velocities */
    int m_angularVelocityFilterSize = 1;

    /** @brief Moving average window size for finite difference acceleration */
    int m_linearAccelerationFilterSize = 1;

    /** @brief Moving average window size for orientation values */
    int m_orientationFilterSize = 1;

    /** @brief IsReading at the measurement sensor period */
    std::vector<IsReading> m_sensorReadingsSensorFrame;

    /** @brief Sensor readings in sensor timestamps */
    std::vector<IsReading> m_sensorReadings;

    /** @brief Unit scale factor for measurements */
    double m_unitScale{ 1.0 };

    /** @brief Current sensor time */
    float m_sensorTime{ 0 };

    /** @brief Previous enabled state */
    bool m_previousEnabled{ true };

    /** @brief Gravity vector in sensor frame */
    omni::math::linalg::vec3d m_gravitySensorFrame;

    /** @brief Gravity vector in world frame */
    omni::math::linalg::vec3d m_gravity;
};


}
}
}
