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

#include <isaacsim/common/string/String.hpp>
#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/objects/Stage.hpp>
#include <isaacsim/foundation/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/foundation/ovsim/control/simulation/Simulation.hpp>
#include <isaacsim/foundation/prims/physics/Articulation.hpp>
#include <isaacsim/foundation/prims/physics/ColliderBody.hpp>
#include <isaacsim/foundation/prims/physics/GroundPlane.hpp>
#include <isaacsim/foundation/prims/physics/RigidBody.hpp>
#include <isaacsim/foundation/utils/Stage.hpp>

#include <algorithm>

namespace isaacsim
{
namespace foundation
{
namespace ovsim
{
namespace control
{

namespace objects = isaacsim::foundation::objects;
namespace prims = isaacsim::foundation::prims;
namespace utils = isaacsim::foundation::utils;

namespace
{

std::optional<objects::Stage> g_usdStage = std::nullopt;
std::optional<objects::Stage> g_ovstageStage = std::nullopt;

} // namespace

namespace authoring
{

bool createStage()
{
    details::clearRegistry();
    objects::Stage stage = objects::Stage("openusd").createStage();
    g_usdStage = stage;
    return stage.isValid();
}

bool openStage(const std::string& usdPath)
{
    details::clearRegistry();
    objects::Stage stage = objects::Stage("openusd").openStage(usdPath);
    g_usdStage = stage;
    return stage.isValid();
}

bool saveStage(const std::string& usdPath)
{
    return utils::getActiveStage().saveStage(usdPath);
}

bool importStageFromString(const std::string& usdString)
{
    details::clearRegistry();
    objects::Stage stage = objects::Stage("openusd").importStageFromString(usdString);
    g_usdStage = stage;
    return stage.isValid();
}

std::string exportStageToString()
{
    return utils::getActiveStage().exportStageToString();
}

bool closeStage()
{
    g_usdStage = std::nullopt;
    g_ovstageStage = std::nullopt;
    details::clearRegistry();
    return utils::getActiveStage().closeStage();
}

bool addReferenceToStage(const std::string& usdPath, const std::string& path, const std::string& typeName)
{
    details::clearRegistry();
    return utils::getActiveStage().addReference(usdPath, path, typeName);
}

bool definePrim(const std::string& path, const std::string& typeName)
{
    details::clearRegistry();
    // Isaac Sim prims
    if (typeName == "Articulation")
    {
        prims::physics::Articulation{ path };
        return true;
    }
    else if (typeName == "ColliderBody")
    {
        prims::physics::ColliderBody{ path };
        return true;
    }
    else if (typeName == "GroundPlane")
    {
        prims::physics::GroundPlane{ path };
        return true;
    }
    else if (typeName == "RigidBody")
    {
        prims::physics::RigidBody{ path };
        return true;
    }
    // USD prims
    return !utils::getActiveStage().definePrim(path, typeName).empty();
}

bool movePrim(const std::string& targetPath, const std::string& destinationPath)
{
    details::clearRegistry();
    return std::get<0>(utils::getActiveStage().movePrim(targetPath, destinationPath));
}

bool removePrim(const std::string& path)
{
    details::clearRegistry();
    return utils::getActiveStage().removePrim(path);
}

bool createPrimAttribute(const std::string& path, const std::string& attributeName, const std::string& typeName)
{
    return objects::Prim(path).createAttribute(attributeName, typeName).at(0).item<bool>();
}

bool removePrimAttribute(const std::string& path, const std::string& attributeName)
{
    return objects::Prim(path).removeAttribute(attributeName).at(0).item<bool>();
}

void setParameter(const std::string& provider, const std::string& parameterName, const InputParameterType& value)
{
    (void)provider;
    (void)parameterName;
    (void)value;
}

OutputParameterType getParameter(const std::string& provider, const std::string& parameterName)
{
    if (provider != "stage")
    {
        throw std::invalid_argument("Invalid provider. Expected 'stage'.");
    }

    static const std::vector<std::string> s_kSupportedParameterNames = { "openusd-stage-id", "openusd-stage-ptr",
                                                                         "ovstage-stage-id", "ovstage-stage-ptr" };

    if (std::find(s_kSupportedParameterNames.begin(), s_kSupportedParameterNames.end(), parameterName) ==
        s_kSupportedParameterNames.end())
    {
        throw std::invalid_argument("Invalid parameter name. Supported values: " +
                                    common::string::join(s_kSupportedParameterNames, ", "));
    }

    if (parameterName == "openusd-stage-id")
    {
        return g_usdStage.has_value() ? g_usdStage->getStageId() : -1;
    }
    else if (parameterName == "openusd-stage-ptr")
    {
        return reinterpret_cast<uintptr_t>(g_usdStage.has_value() ? g_usdStage->getStagePtr() : nullptr);
    }
    else if (parameterName == "ovstage-stage-id")
    {
        return g_ovstageStage.has_value() ? g_ovstageStage->getStageId() : -1;
    }
    else if (parameterName == "ovstage-stage-ptr")
    {
        return reinterpret_cast<uintptr_t>(g_ovstageStage.has_value() ? g_ovstageStage->getStagePtr() : nullptr);
    }
    else
    {
        throw std::invalid_argument("Not implemented parameter name: " + parameterName);
    }
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
    std::string usdString = utils::getActiveStage().exportStageToString();
    g_ovstageStage = objects::Stage("ovstage").importStageFromString(usdString, /*makeDefault=*/false);
}

void invalidate()
{
    if (g_ovstageStage.has_value())
    {
        g_ovstageStage->closeStage();
        g_ovstageStage = std::nullopt;
    }
}

void step()
{
}

void setParameter(const std::string& provider, const std::string& parameterName, const InputParameterType& value)
{
    (void)provider;
    (void)parameterName;
    (void)value;
    throw std::logic_error("isaacsim::foundation::ovsim::control::simulation::setParameter is not implemented");
}

OutputParameterType getParameter(const std::string& provider, const std::string& parameterName)
{
    (void)provider;
    (void)parameterName;
    throw std::logic_error("isaacsim::foundation::ovsim::control::simulation::getParameter is not implemented");
}

} // namespace simulation

} // namespace control
} // namespace ovsim
} // namespace foundation
} // namespace isaacsim
