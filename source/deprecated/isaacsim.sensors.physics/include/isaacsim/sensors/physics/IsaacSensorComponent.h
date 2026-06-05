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

#include "isaacsim/core/includes/Component.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <isaacsim/robot/schema/sensor_tokens.h>

#include <string>
#include <vector>

/**
 * @namespace isaacsim
 * @brief Root namespace for Isaac Sim functionality.
 */
namespace isaacsim
{
/**
 * @namespace sensors
 * @brief Namespace containing sensor-related functionality.
 */
namespace sensors
{
/**
 * @namespace physics
 * @brief Namespace containing physics-based sensor implementations.
 */
namespace physics
{
/**
 * @brief Linear interpolation between two values.
 * @details Performs linear interpolation between start and end values based on the interpolation factor t.
 * The interpolation is calculated as: start + ((end - start) * t).
 *
 * @param[in] start Starting value for interpolation.
 * @param[in] end Ending value for interpolation.
 * @param[in] t Interpolation factor between 0.0 and 1.0.
 * @return The interpolated value.
 *
 * @note The interpolation factor t should be between 0.0 and 1.0 for expected results.
 */
inline float lerp(const float& start, const float& end, const float t)
{
    return start + ((end - start) * t);
}

/**
 * @class IsaacSensorComponentBase
 * @brief Base class template for non-RTX Isaac sensors.
 * @details
 * This class serves as the foundation for implementing non-RTX sensors in Isaac Sim.
 * It provides the basic structure and lifecycle methods that all sensors should implement.
 *
 * @tparam PrimType The USD prim type representing the sensor. This type parameter allows
 *                  different sensor implementations to use their specific USD prim types.
 *
 * @note This class inherits from ComponentBase to integrate with the Isaac Sim component system.
 */
template <class PrimType>
class IsaacSensorComponentBase : public isaacsim::core::includes::ComponentBase<PrimType>
{
public:
    /**
     * @brief Default constructor.
     * @details Constructs a new instance of the sensor component.
     */
    IsaacSensorComponentBase() = default;

    /**
     * @brief Virtual destructor.
     * @details Ensures proper cleanup of derived sensor classes.
     */
    ~IsaacSensorComponentBase() = default;

    /**
     * @brief Initializes the sensor component.
     * @details Sets up the sensor component with its USD prim and stage references.
     *
     * @param[in] prim USD prim representing the sensor.
     * @param[in] stage USD stage containing the prim.
     */
    virtual void initialize(const PrimType& prim, const pxr::UsdStageWeakPtr stage)
    {
        isaacsim::core::includes::ComponentBase<PrimType>::initialize(prim, stage);
    }

    /**
     * @brief Called when the sensor starts.
     * @details Handles sensor initialization when the component is started.
     *          Triggers onComponentChange to ensure proper initial state.
     */
    virtual void onStart()
    {
        onComponentChange();
    }

    /**
     * @brief Called when sensor component properties change.
     * @details Updates the sensor's enabled state when component properties are modified.
     */
    virtual void onComponentChange()
    {
        using namespace isaacsim::robot::schema::sensors;
        // base sensor on component change
        isaacsim::core::includes::safeGetAttribute(this->m_prim.GetAttribute(kEnabledAttr), this->m_enabled);
    }

    /**
     * @brief Called before each tick to prepare sensor state.
     * @details Provides an opportunity to prepare the sensor state before the main tick update.
     *          Default implementation does nothing.
     */
    virtual void preTick()
    {
        return;
    }

    /**
     * @brief Pure virtual function called each tick to update sensor state.
     * @details This method must be implemented by derived classes to define the
     *          sensor's behavior during each simulation tick.
     */
    virtual void tick() = 0;

    /**
     * @brief Called each physics step to update sensor state.
     * @details Allows sensors to update their state based on physics simulation steps.
     *          Default implementation does nothing.
     */
    virtual void onPhysicsStep(){};

    /**
     * @brief Called when the sensor stops.
     * @details Handles cleanup when the sensor component is stopped.
     *          Default implementation does nothing.
     */
    virtual void onStop()
    {
    }

    /**
     * @brief Gets the parent prim of the sensor.
     * @details Retrieves the USD prim that is the parent of this sensor.
     *
     * @return USD prim that is the parent of this sensor.
     */
    pxr::UsdPrim getParentPrim()
    {
        return m_parentPrim;
    }

protected:
    /**
     * @brief USD prim that is the parent of this sensor.
     * @details Stores a reference to the parent USD prim that contains this sensor.
     */
    pxr::UsdPrim m_parentPrim;
};

/**
 * @typedef IsaacBaseSensorComponent
 * @brief Convenience typedef for IsaacSensorComponentBase with IsaacBaseSensor prim type.
 * @details Defines a commonly used specialization of IsaacSensorComponentBase using
 *          the IsaacBaseSensor prim type for basic sensor functionality.
 */
using IsaacBaseSensorComponent = IsaacSensorComponentBase<pxr::UsdPrim>;

}
}
}
