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

#include <isaacsim/foundation/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/foundation/ovsim/control/simulation/Simulation.hpp>
#include <isaacsim/ovsim/clients/local/control/authoring/Authoring.hpp>
#include <isaacsim/ovsim/clients/local/control/simulation/Simulation.hpp>
#include <isaacsim/physics/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/physics/ovsim/control/simulation/Simulation.hpp>

#include <stdexcept>

namespace isaacsim
{
namespace ovsim
{
namespace clients
{
namespace local
{
namespace control
{

namespace foundation = isaacsim::foundation::ovsim::control;
namespace physics = isaacsim::physics::ovsim::control;

namespace authoring
{

bool createStage()
{
    return foundation::authoring::createStage();
}

bool openStage(const std::string& usdPath)
{
    return foundation::authoring::openStage(usdPath);
}

bool saveStage(const std::string& usdPath)
{
    return foundation::authoring::saveStage(usdPath);
}

bool importStageFromString(const std::string& usdString)
{
    return foundation::authoring::importStageFromString(usdString);
}

std::string exportStageToString()
{
    return foundation::authoring::exportStageToString();
}

bool closeStage()
{
    return foundation::authoring::closeStage();
}

bool addReferenceToStage(const std::string& usdPath, const std::string& path, const std::string& typeName)
{
    return foundation::authoring::addReferenceToStage(usdPath, path, typeName);
}

bool definePrim(const std::string& path, const std::string& typeName)
{
    return foundation::authoring::definePrim(path, typeName);
}

bool movePrim(const std::string& targetPath, const std::string& destinationPath)
{
    return foundation::authoring::movePrim(targetPath, destinationPath);
}

bool removePrim(const std::string& path)
{
    return foundation::authoring::removePrim(path);
}

bool createPrimAttribute(const std::string& path, const std::string& attributeName, const std::string& typeName)
{
    return foundation::authoring::createPrimAttribute(path, attributeName, typeName);
}

bool removePrimAttribute(const std::string& path, const std::string& attributeName)
{
    return foundation::authoring::removePrimAttribute(path, attributeName);
}

void setParameter(const std::string& provider, const std::string& parameterName, const InputParameterType& value)
{
    if (provider == "stage")
    {
        foundation::authoring::setParameter(provider, parameterName, value);
    }
    else
    {
        throw std::invalid_argument("Invalid provider. Expected 'stage'.");
    }
}

OutputParameterType getParameter(const std::string& provider, const std::string& parameterName)
{
    if (provider == "stage")
    {
        return foundation::authoring::getParameter(provider, parameterName);
    }
    else
    {
        throw std::invalid_argument("Invalid provider. Expected 'stage'.");
    }
}

} // namespace authoring

namespace simulation
{

void play()
{
    return physics::simulation::play();
}

void pause()
{
    return physics::simulation::pause();
}

void stop()
{
    return physics::simulation::stop();
}

void initialize()
{
    foundation::simulation::initialize();
    auto ovstageStagePointer = foundation::authoring::getParameter("stage", "ovstage-stage-ptr");
    physics::simulation::setParameter("physics", "ovstage-stage-ptr", ovstageStagePointer);
    physics::simulation::initialize();
}

void invalidate()
{
    // Physics retains the Foundation-owned OVStage pointer until invalidation completes.
    physics::simulation::invalidate();
    foundation::simulation::invalidate();
}

void step()
{
    return physics::simulation::step();
}

void setParameter(const std::string& provider, const std::string& parameterName, const InputParameterType& value)
{
    if (provider == "physics")
    {
        physics::simulation::setParameter(provider, parameterName, value);
    }
    else
    {
        throw std::invalid_argument("Invalid provider. Expected 'physics'.");
    }
}

OutputParameterType getParameter(const std::string& provider, const std::string& parameterName)
{
    if (provider == "physics")
    {
        return physics::simulation::getParameter(provider, parameterName);
    }
    else
    {
        throw std::invalid_argument("Invalid provider. Expected 'physics'.");
    }
}

} // namespace simulation

} // namespace control
} // namespace local
} // namespace clients
} // namespace ovsim
} // namespace isaacsim
