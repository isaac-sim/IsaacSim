// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "details/Registry.hpp"

#include <isaacsim/physics/manager/PhysicsManager.hpp>
#include <isaacsim/physics/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/physics/ovsim/control/simulation/Simulation.hpp>

#include <stdexcept>

namespace isaacsim
{
namespace physics
{
namespace ovsim
{
namespace control
{

namespace manager = isaacsim::physics::manager;

namespace
{

int64_t g_usdStageId = 0;
void* g_ovstageInstancePointer = nullptr;

} // namespace

namespace authoring
{

bool createStage()
{
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::createStage is not implemented");
}

bool openStage(const std::string& usdPath)
{
    (void)usdPath;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::openStage is not implemented");
}

bool saveStage(const std::string& usdPath)
{
    (void)usdPath;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::saveStage is not implemented");
}

bool importStageFromString(const std::string& usdString)
{
    (void)usdString;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::importStageFromString is not implemented");
}

std::string exportStageToString()
{
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::exportStageToString is not implemented");
}

bool closeStage()
{
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::closeStage is not implemented");
}

bool addReferenceToStage(const std::string& usdPath, const std::string& path, const std::string& typeName)
{
    (void)usdPath;
    (void)path;
    (void)typeName;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::addReferenceToStage is not implemented");
}

bool definePrim(const std::string& path, const std::string& typeName)
{
    (void)path;
    (void)typeName;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::definePrim is not implemented");
}

bool movePrim(const std::string& targetPath, const std::string& destinationPath)
{
    (void)targetPath;
    (void)destinationPath;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::movePrim is not implemented");
}

bool removePrim(const std::string& path)
{
    (void)path;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::removePrim is not implemented");
}

bool createPrimAttribute(const std::string& path, const std::string& attributeName, const std::string& typeName)
{
    (void)path;
    (void)attributeName;
    (void)typeName;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::createPrimAttribute is not implemented");
}

bool removePrimAttribute(const std::string& path, const std::string& attributeName)
{
    (void)path;
    (void)attributeName;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::removePrimAttribute is not implemented");
}

void setParameter(const std::string& provider, const std::string& parameterName, const InputParameterType& value)
{
    (void)provider;
    (void)parameterName;
    (void)value;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::setParameter is not implemented");
}

OutputParameterType getParameter(const std::string& provider, const std::string& parameterName)
{
    (void)provider;
    (void)parameterName;
    throw std::logic_error("isaacsim::physics::ovsim::control::authoring::getParameter is not implemented");
}

} // namespace authoring

namespace simulation
{

void play()
{
}

void pause()
{
}

void stop()
{
}

void initialize()
{
    if (!g_ovstageInstancePointer)
    {
        throw std::runtime_error(
            "Cannot initialize simulation: ovstage instance pointer is not valid. "
            "Set the 'ovstage-stage-ptr' parameter before calling initialize.");
    }
    details::clearRegistry();
    if (!manager::PhysicsManager::getInstance().initialize(g_ovstageInstancePointer, g_usdStageId))
    {
        throw std::runtime_error("Physics manager initialization failed.");
    }
}

void invalidate()
{
    details::clearRegistry();
    if (!manager::PhysicsManager::getInstance().invalidate())
    {
        throw std::runtime_error("Physics manager invalidation failed.");
    }
}

void step()
{
    manager::PhysicsManager::getInstance().step();
}

void setParameter(const std::string& provider, const std::string& parameterName, const InputParameterType& value)
{
    if (provider != "physics")
    {
        throw std::invalid_argument("Invalid provider. Expected 'physics'.");
    }

    if (parameterName == "ovstage-stage-ptr")
    {
        if (std::holds_alternative<uintptr_t>(value))
        {
            g_ovstageInstancePointer = reinterpret_cast<void*>(std::get<uintptr_t>(value));
        }
        else
        {
            throw std::invalid_argument("Invalid value type for parameter 'ovstage-stage-ptr'. Expected uintptr_t.");
        }
    }
    else if (parameterName == "physics-engine")
    {
        if (std::holds_alternative<std::string>(value))
        {
            std::string engineName = std::get<std::string>(value);
            if (!manager::PhysicsManager::getInstance().switchPhysicsEngine(engineName))
            {
                throw std::invalid_argument("Failed to switch physics engine. Engine name may be invalid.");
            }
        }
        else
        {
            throw std::invalid_argument("Invalid value type for parameter 'physics-engine'. Expected std::string.");
        }
    }
    else
    {
        throw std::invalid_argument("Invalid parameter name. Expected 'ovstage-stage-ptr' or 'physics-engine'.");
    }
}

OutputParameterType getParameter(const std::string& provider, const std::string& parameterName)
{
    if (provider != "physics")
    {
        throw std::invalid_argument("Invalid provider. Expected 'physics'.");
    }

    if (parameterName == "ovstage-stage-ptr")
    {
        return reinterpret_cast<uintptr_t>(g_ovstageInstancePointer);
    }
    else
    {
        throw std::invalid_argument("Invalid parameter name. Expected 'ovstage-stage-ptr'.");
    }
}

} // namespace simulation

} // namespace control
} // namespace ovsim
} // namespace physics
} // namespace isaacsim
