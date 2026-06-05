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

#include <carb/Framework.h>
#include <carb/logging/Log.h>

#include <isaacsim/core/includes/PrimManager.h>
#include <isaacsim/sensors/physics/ContactManager.h>
#include <isaacsim/sensors/physics/ContactSensor.h>
#include <isaacsim/sensors/physics/ImuSensor.h>
#include <isaacsim/sensors/physics/IsaacSensorComponent.h>
#include <isaacsim/sensors/physics/IsaacSensorTypes.h>
#include <omni/physx/IPhysx.h>
#include <pxr/usd/usdPhysics/scene.h>

#include <PxActor.h>
#include <PxArticulationLink.h>
#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
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
 * @class IsaacSensorManager
 * @brief Manager class for handling Isaac physics-based sensors.
 * @details
 * This class manages various physics-based sensors in the Isaac environment, including
 * contact sensors and IMU sensors. It handles sensor lifecycle, updates, and provides
 * access to individual sensor components.
 *
 * The manager inherits from PrimManagerBase and provides functionality for
 * adding, updating, and managing sensor components in the physics simulation.
 */
class IsaacSensorManager : public isaacsim::core::includes::PrimManagerBase<IsaacBaseSensorComponent>
{
public:
    /**
     * @brief Constructs a new IsaacSensorManager instance.
     * @details Initializes the manager with a PhysX interface and creates a contact manager instance.
     * @param[in] physXInterface Pointer to the PhysX interface used for physics simulation.
     */
    IsaacSensorManager(omni::physx::IPhysx* physXInterface)
    {
        m_physXInterface = physXInterface;
        m_contactManager = std::make_unique<ContactManager>();
    }

    /**
     * @brief Destructor for IsaacSensorManager.
     * @details Cleans up all components and releases the contact manager.
     */
    ~IsaacSensorManager()
    {
        m_components.clear();
        m_contactManager.reset();
    }

    /**
     * @brief Handles the stop event for all managed sensors.
     * @details
     * Resets all components to their initial state, stops all sensors,
     * resets the contact manager, and resets internal timers.
     */
    void onStop()
    {
        // PxScene can change after stop is pressed so reset mDoStart bool to force OnStart to run
        for (auto& component : m_components)
        {
            component.second->mDoStart = true;
            component.second->onStop();
        }
        m_contactManager->resetSensors();

        // reset timers whenever stopped
        this->m_timeSeconds = 0;
        this->m_timeNanoSeconds = 0;
    }

    /**
     * @brief Handles the addition of a new sensor component.
     * @details
     * Creates and initializes appropriate sensor components based on the USD prim type.
     * Supports creation of contact sensors and IMU sensors.
     *
     * @param[in] prim The USD primitive representing the sensor to be added.
     */
    void onComponentAdd(const pxr::UsdPrim& prim)
    {
        using namespace isaacsim::robot::schema::sensors;
        std::unique_ptr<IsaacBaseSensorComponent> component;
        if (prim.GetTypeName() == kIsaacContactSensorType)
        {
            component = std::make_unique<ContactSensor>(m_physXInterface, m_contactManager.get());
            component->initialize(prim, m_stage);

            ContactSensor* contactSensor = dynamic_cast<ContactSensor*>(component.get());
            bool validParentFound = contactSensor->findValidParent();

            if (!validParentFound)
            {
                CARB_LOG_WARN("Failed to create contact sensor, parent prim is not found or invalid");
                return;
            }
        }
        else if (prim.GetTypeName() == kIsaacImuSensorType)
        {
            component = std::make_unique<ImuSensor>();
            component->initialize(prim, m_stage);

            ImuSensor* imuSensor = dynamic_cast<ImuSensor*>(component.get());
            bool validParentFound = imuSensor->findValidParent();

            if (!validParentFound)
            {
                CARB_LOG_WARN("Failed to create imu sensor, parent prim is not found or invalid");
                return;
            }
        }

        if (component)
        {
            // CARB_LOG_INFO("Create: Isaac Sensor %s with type: %s", prim.GetPath().GetString().c_str(),
            //                 component->getPrim().GetPrim().GetTypeName().GetString().c_str());
            m_components[prim.GetPath().GetString()] = std::move(component);
        }
    }

    /**
     * @brief Returns a vector of supported sensor component types.
     * @details Provides a list of USD prim types that this manager can handle.
     * @return Vector of strings representing supported sensor types.
     */
    virtual std::vector<std::string> getComponentIsAVector() const
    {
        return { "IsaacContactSensor", "IsaacImuSensor" };
    }

    /**
     * @brief Handles changes to a sensor component's properties.
     * @details
     * Updates the component's state when its properties change in the USD stage.
     *
     * @param[in] prim The USD primitive whose properties have changed.
     */
    virtual void onComponentChange(const pxr::UsdPrim& prim)
    {
        isaacsim::core::includes::PrimManagerBase<IsaacBaseSensorComponent>::onComponentChange(prim);
        // update properties of this prim (onComponentChange)
        if (m_components.find(prim.GetPath().GetString()) != m_components.end())
        {
            m_components[prim.GetPath().GetString()]->onComponentChange();
        }
    }

    /**
     * @brief Retrieves a sensor component by its path.
     * @details Looks up a sensor component using its USD path string.
     *
     * @param[in] path String representation of the sensor's USD path.
     * @return Pointer to the sensor component if found, nullptr otherwise.
     */
    IsaacBaseSensorComponent* getComponent(std::string path)
    {
        if (m_components.find(path) != m_components.end())
        {
            return m_components[path].get();
        }
        return nullptr;
    }

    /**
     * @brief Updates all sensor components during the physics simulation step.
     * @details
     * Processes physics updates for all sensor components, including contact and IMU sensors.
     * Handles component initialization, timestep updates, and sensor-specific physics calculations.
     *
     * @param[in] dt Time step delta in seconds.
     */
    void onPhysicsStep(const double& dt)
    {

        m_contactManager->onPhysicsStep(static_cast<float>(m_timeSeconds), static_cast<float>(dt));

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
     * @brief Performs a tick update for all sensor components.
     * @details
     * Updates all enabled sensor components during the simulation tick.
     * Handles component initialization and tick-based updates.
     *
     * @param[in] dt Time step delta in seconds.
     */
    virtual void tick(double dt)
    {
        if (m_components.empty())
        {
            return;
        }
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
                component.second->preTick();
                component.second->tick();
            }
        }
    }

    /**
     * @brief Retrieves a contact sensor component for a given USD primitive.
     * @details Attempts to find and cast a component to a ContactSensor type.
     *
     * @param[in] prim USD primitive associated with the contact sensor.
     * @return Pointer to the ContactSensor if found, nullptr otherwise.
     */
    ContactSensor* getContactSensor(const pxr::UsdPrim& prim)
    {
        if (prim)
        {
            if (m_components.find(prim.GetPath().GetString()) != m_components.end())
            {
                return dynamic_cast<ContactSensor*>(m_components[prim.GetPath().GetString()].get());
            }
        }
        return nullptr;
    }

    /**
     * @brief Gets the contact manager instance.
     * @details Provides access to the manager's contact handling system.
     *
     * @return Pointer to the ContactManager instance.
     */
    ContactManager* getContactManager()
    {
        return m_contactManager.get();
    }

    /**
     * @brief Retrieves an IMU sensor component for a given USD primitive.
     * @details Attempts to find and cast a component to an ImuSensor type.
     *
     * @param[in] prim USD primitive associated with the IMU sensor.
     * @return Pointer to the ImuSensor if found, nullptr otherwise.
     */
    ImuSensor* getImuSensor(const pxr::UsdPrim& prim)
    {
        if (prim)
        {
            if (m_components.find(prim.GetPath().GetString()) != m_components.end())
            {
                return dynamic_cast<ImuSensor*>(m_components[prim.GetPath().GetString()].get());
            }
        }
        return nullptr;
    }

private:
    /**
     * @brief Pointer to the PhysX interface used for physics simulation.
     * @details Provides access to the PhysX physics engine functionality.
     */
    omni::physx::IPhysx* m_physXInterface = nullptr;

    /**
     * @brief Unique pointer to the contact manager instance.
     * @details Manages contact-related functionality for all contact sensors.
     */
    std::unique_ptr<ContactManager> m_contactManager = nullptr;

    /**
     * @brief Map of rigid body identifiers to data buffer indices.
     * @details Used to track and access rigid body data in the data buffer.
     */
    std::unordered_map<std::string, size_t> m_rigidBodyToDataBufferMap;

    /**
     * @brief Buffer containing rigid body data.
     * @details Stores physics-related data for rigid bodies in the simulation.
     */
    std::vector<float> m_rigidBodyDataBuffer;
};
}
}
}
