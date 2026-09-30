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

#include <isaacsim/ovsim/api/Factory.hpp>
#include <isaacsim/ovsim/clients/grpc/GrpcSession.hpp>
#include <isaacsim/ovsim/clients/local/control/authoring/Authoring.hpp>
#include <isaacsim/ovsim/clients/local/control/simulation/Simulation.hpp>
#include <isaacsim/ovsim/clients/local/data/Data.hpp>

#include <memory>
#include <stdexcept>

namespace isaacsim
{
namespace ovsim
{
namespace api
{

namespace
{

namespace client_grpc = isaacsim::ovsim::clients::grpc;
namespace client_local = isaacsim::ovsim::clients::local;
namespace iface_authoring = ::ovsim::interfaces::control::authoring;
namespace iface_simulation = ::ovsim::interfaces::control::simulation;

types::Implementation makeLocalClient(
    const std::optional<std::unordered_map<std::string, std::string>>& configuration = std::nullopt)
{
    (void)configuration;

    types::Authoring authoring{
        &client_local::control::authoring::createStage,
        &client_local::control::authoring::openStage,
        &client_local::control::authoring::saveStage,
        &client_local::control::authoring::importStageFromString,
        &client_local::control::authoring::exportStageToString,
        &client_local::control::authoring::closeStage,
        &client_local::control::authoring::addReferenceToStage,
        &client_local::control::authoring::definePrim,
        &client_local::control::authoring::movePrim,
        &client_local::control::authoring::removePrim,
        &client_local::control::authoring::createPrimAttribute,
        &client_local::control::authoring::removePrimAttribute,
        &client_local::control::authoring::setParameter,
        &client_local::control::authoring::getParameter,
    };

    types::Simulation simulation{
        &client_local::control::simulation::play,         &client_local::control::simulation::pause,
        &client_local::control::simulation::stop,         &client_local::control::simulation::initialize,
        &client_local::control::simulation::invalidate,   &client_local::control::simulation::step,
        &client_local::control::simulation::setParameter, &client_local::control::simulation::getParameter,
    };

    types::Data data{
        &client_local::data::read,
        &client_local::data::write,
    };

    return types::Implementation{ types::Control{ authoring, simulation }, data };
}

types::Implementation makeGrpcClient(const std::optional<std::unordered_map<std::string, std::string>>& configuration)
{
    auto session = std::make_shared<client_grpc::GrpcSession>(configuration);

    types::Authoring authoring{
        [session]() { return session->createStage(); },
        [session](const std::string& usdPath) { return session->openStage(usdPath); },
        [session](const std::string& usdPath) { return session->saveStage(usdPath); },
        [session](const std::string& usdString) { return session->importStageFromString(usdString); },
        [session]() { return session->exportStageToString(); },
        [session]() { return session->closeStage(); },
        [session](const std::string& usdPath, const std::string& path, const std::string& typeName)
        { return session->addReferenceToStage(usdPath, path, typeName); },
        [session](const std::string& path, const std::string& typeName) { return session->definePrim(path, typeName); },
        [session](const std::string& targetPath, const std::string& destinationPath)
        { return session->movePrim(targetPath, destinationPath); },
        [session](const std::string& path) { return session->removePrim(path); },
        [session](const std::string& path, const std::string& attributeName, const std::string& typeName)
        { return session->createPrimAttribute(path, attributeName, typeName); },
        [session](const std::string& path, const std::string& attributeName)
        { return session->removePrimAttribute(path, attributeName); },
        [session](const std::string& provider, const std::string& parameterName,
                  const iface_authoring::InputParameterType& value)
        { session->setAuthoringParameter(provider, parameterName, value); },
        [session](const std::string& provider, const std::string& parameterName) -> iface_authoring::OutputParameterType
        { return session->getAuthoringParameter(provider, parameterName); },
    };

    types::Simulation simulation{
        [session]() { session->play(); },
        [session]() { session->pause(); },
        [session]() { session->stop(); },
        [session]() { session->initialize(); },
        [session]() { session->invalidate(); },
        [session]() { session->step(); },
        [session](const std::string& provider, const std::string& parameterName,
                  const iface_simulation::InputParameterType& value)
        { session->setSimulationParameter(provider, parameterName, value); },
        [session](const std::string& provider, const std::string& parameterName) -> iface_simulation::OutputParameterType
        { return session->getSimulationParameter(provider, parameterName); },
    };

    types::Data data{
        [session](const client_grpc::PathType& paths, const std::string& attributeName, std::optional<double> timestamp)
        { return session->read(paths, attributeName, timestamp); },
        [session](const client_grpc::PathType& paths, const std::string& attributeName,
                  const client_grpc::InputValueType& values, std::optional<double> timestamp)
        { session->write(paths, attributeName, values, timestamp); },
    };

    return types::Implementation{ types::Control{ authoring, simulation }, data };
}

} // namespace


types::Implementation makeClient(const std::string& name,
                                 const std::optional<std::unordered_map<std::string, std::string>>& configuration)
{
    if (name == "in-process" || name == "local")
    {
        return makeLocalClient(configuration);
    }
    if (name == "grpc")
    {
        return makeGrpcClient(configuration);
    }
    throw std::runtime_error("Unknown client name: '" + name + "'. Valid names are: 'in-process' or 'local', 'grpc'.");
}

} // namespace api
} // namespace ovsim
} // namespace isaacsim
