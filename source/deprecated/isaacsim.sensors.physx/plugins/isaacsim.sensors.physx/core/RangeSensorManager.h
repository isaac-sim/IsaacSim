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

#include "../generic/GenericSensor.h"
#include "../lidar/LidarSensor.h"
#include "../lightbeam_sensor/LightBeamSensor.h"
#include "RangeSensorComponent.h"
#include "isaacsim/core/includes/PrimManager.h"
#include "isaacsim/core/includes/ScopedTimer.h"

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/dictionary/DictionaryUtils.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/robot/schema/sensor_tokens.h>
#include <isaacsim/util/debug_draw/PrimitiveDrawingHelper.h>
#include <omni/physx/IPhysx.h>
#include <omni/usd/UsdContext.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace sensors
{
namespace physx
{

/**
 * @class RangeSensorManager
 * @brief Manager class for handling multiple range sensor components in the simulation
 * @details This class manages the lifecycle and updates of various range sensor types including
 *          Lidar sensors, generic range sensors, and light beam sensors. It handles sensor
 *          initialization, updates, and cleanup while providing thread-safe access to sensor data.
 *          The manager supports parallel processing of multiple sensors and ensures proper
 *          synchronization with the physics simulation timeline.
 */
class RangeSensorManager : public isaacsim::core::includes::PrimManagerBase<RangeSensorComponent>
{
public:
    /**
     * @brief Constructs a new Sensor Manager object
     * @param[in] physxPtr Pointer to the PhysX interface for physics simulation
     * @param[in] taskingPtr Pointer to the tasking interface for parallel processing
     */
    RangeSensorManager(omni::physx::IPhysx* physxPtr, carb::tasking::ITasking* taskingPtr)
    {
        m_physxPtr = physxPtr;
        m_tasking = taskingPtr;
    }

    /**
     * @brief Virtual destructor for proper cleanup
     */
    ~RangeSensorManager() = default;

    /**
     * @brief Updates all sensor components after each physics simulation step
     * @details Processes each enabled sensor component, updating their timestamps and
     *          triggering their physics step handlers. Also manages the initialization
     *          of components that haven't started yet.
     * @param[in] dt The time step duration in seconds
     */
    void onPhysicsStep(const double& dt)
    {
        for (auto& component : m_components)
        {
            if (component.second->mDoStart == true)
            {
                // if the component has not started yet, check to see if its enabled
                // if not enabled, do not start
                if (component.second->getEnabled())
                {
                    component.second->onStart();
                    component.second->mDoStart = false;
                }
            }
            if (component.second->getEnabled())
            {
                component.second.get()->updateTimestamp(this->m_timeSeconds, dt, this->m_timeNanoSeconds);
                component.second->onPhysicsStep();
            }
        }
        this->m_timeSeconds += dt;
        this->m_timeNanoSeconds = static_cast<int64_t>(m_timeSeconds * 1e9);

        // update timestep
    }

    /**
     * @brief Updates all sensor components during the simulation tick
     * @details Processes each enabled sensor component in parallel if multiple sensors exist.
     *          Handles component initialization, pre-tick operations, main tick updates,
     *          and visualization drawing.
     * @param[in] dt The time step duration in seconds
     */
    void tick(double dt)
    {
        std::unique_lock<std::mutex> lck(m_componentMtx);
        CARB_PROFILE_ZONE(0, "[IsaacSim] Isaac Range Sensor Tick");
        if (m_components.empty())
        {
            return;
        }

        bool tickedOnPhysicsStep = false;
        for (auto& component : m_components)
        {
            if (component.second->mDoStart == true)
            {
                // if the component has not started yet, check to see if its enabled
                // if not enabled, do not start
                if (component.second->getEnabled())
                {
                    component.second->onStart();
                    component.second->mDoStart = false;
                }
            }
            if (component.second->getEnabled())
            {
                if (component.second->hasTickedOnPhysicsStep())
                {
                    tickedOnPhysicsStep = true;
                }
                else
                {
                    component.second->preTick();
                }
            }
        }
        // No need to make threads if there is only one sensor.
        if (m_components.size() > 1)
        {
            m_tasking->applyRange(m_components.size(),
                                  [&](size_t index)
                                  {
                                      auto it = m_components.begin();
                                      std::advance(it, index);

                                      if (it->second.get()->getEnabled() && !it->second.get()->hasTickedOnPhysicsStep())
                                      {
                                          it->second.get()->updateTimestamp(
                                              this->m_timeSeconds, dt, this->m_timeNanoSeconds);
                                          it->second.get()->tick();
                                      }
                                  });
        }
        else
        {
            for (auto& component : m_components)
            {
                if (component.second->getEnabled() && !component.second->hasTickedOnPhysicsStep())
                {
                    component.second.get()->updateTimestamp(this->m_timeSeconds, dt, this->m_timeNanoSeconds);
                    component.second->tick();
                }
            }
        }


        for (auto& component : m_components)
        {
            if (component.second->getEnabled())
            {
                component.second.get()->draw();
            }
            component.second->clearTickedOnPhysicsStep();
        }

        if (!tickedOnPhysicsStep)
        {
            this->m_timeSeconds += dt;
            this->m_timeNanoSeconds = static_cast<int64_t>(m_timeSeconds * 1e9);
        }
    }

    /**
     * @brief Handles cleanup when the simulation scene is stopped
     * @details Resets all components to ensure proper reinitialization on next start
     */
    void onStop()
    {
        // PxScene can change after stop is pressed so reset mDoStart bool to force OnStart to run
        for (auto& component : m_components)
        {
            component.second->clearTickedOnPhysicsStep();
            component.second->mDoStart = true;
            component.second->onStop();
        }
    }

    /**
     * @brief Creates a new sensor component based on the USD prim type
     * @param[in] prim The USD prim representing the sensor to create
     * @details Instantiates the appropriate sensor type (Lidar, Generic, or LightBeam)
     *          based on the prim type and initializes it with the provided parameters
     */
    void onComponentAdd(const pxr::UsdPrim& prim)
    {
        std::unique_ptr<RangeSensorComponent> component;

        if (prim.GetTypeName() == kLidarType)
        {
            component = std::make_unique<LidarSensor>(m_physxPtr);
        }
        else if (prim.GetTypeName() == kGenericType)
        {
            component = std::make_unique<GenericSensor>(m_physxPtr);
        }
        else if (prim.GetTypeName() == kIsaacLightBeamSensorType)
        {
            component = std::make_unique<LightBeamSensor>(m_physxPtr);
        }

        if (component)
        {
            component->initialize(prim, m_stage);
            CARB_LOG_INFO("Create: Range Sensor %s with type: %s", prim.GetPath().GetString().c_str(),
                          component->getPrim().GetTypeName().GetString().c_str());
            m_components[prim.GetPath().GetString()] = std::move(component);
        }
    }

    /**
     * @brief Gets the list of supported sensor component types
     * @return Vector of strings containing the supported sensor type names
     */
    virtual std::vector<std::string> getComponentIsAVector() const
    {
        return { kLidarType.GetString(), kGenericType.GetString(), kIsaacLightBeamSensorType.GetString() };
    }

    /**
     * @brief Handles property changes for a sensor component
     * @param[in] prim The USD prim whose properties have changed
     * @details Updates the corresponding sensor component with the new property values
     */
    virtual void onComponentChange(const pxr::UsdPrim& prim)
    {
        isaacsim::core::includes::PrimManagerBase<RangeSensorComponent>::onComponentChange(prim);
        // update properties of this prim (onComponentChange)
        if (m_components.find(prim.GetPath().GetString()) != m_components.end())
        {
            m_components[prim.GetPath().GetString()]->onComponentChange();
        }
    }

    /**
     * @brief Retrieves a Lidar sensor component associated with the given USD prim
     * @param[in] prim The USD prim associated with the desired Lidar sensor
     * @return Pointer to the LidarSensor component if found, nullptr otherwise
     */
    LidarSensor* getLidarSensor(const pxr::UsdPrim& prim)
    {
        if (prim)
        {
            if (m_components.find(prim.GetPath().GetString()) != m_components.end())
            {
                return dynamic_cast<LidarSensor*>(m_components[prim.GetPath().GetString()].get());
            }
        }
        return nullptr;
    }

    /**
     * @brief Retrieves a generic range sensor component associated with the given USD prim
     * @param[in] prim The USD prim associated with the desired generic sensor
     * @return Pointer to the GenericSensor component if found, nullptr otherwise
     */
    GenericSensor* getGenericSensor(const pxr::UsdPrim& prim)
    {
        if (prim)
        {
            if (m_components.find(prim.GetPath().GetString()) != m_components.end())
            {
                return dynamic_cast<GenericSensor*>(m_components[prim.GetPath().GetString()].get());
            }
        }
        return nullptr;
    }

    /**
     * @brief Retrieves a light beam sensor component associated with the given USD prim
     * @param[in] prim The USD prim associated with the desired light beam sensor
     * @return Pointer to the LightBeamSensor component if found, nullptr otherwise
     */
    LightBeamSensor* getLightBeamSensor(const pxr::UsdPrim& prim)
    {
        if (prim)
        {
            if (m_components.find(prim.GetPath().GetString()) != m_components.end())
            {
                return dynamic_cast<LightBeamSensor*>(m_components[prim.GetPath().GetString()].get());
            }
        }
        return nullptr;
    }

private:
    /**
     * @brief Pointer to the PhysX interface for physics simulation
     */
    omni::physx::IPhysx* m_physxPtr = nullptr;

    /**
     * @brief Pointer to the tasking interface for parallel processing
     */
    carb::tasking::ITasking* m_tasking = nullptr;
};
}
}
}
