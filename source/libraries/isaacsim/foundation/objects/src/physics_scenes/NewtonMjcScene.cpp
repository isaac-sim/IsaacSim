// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "PhysicsSceneUtils.hpp"

#include <isaacsim/foundation/objects/physics_scenes/NewtonMjcScene.hpp>

#include <cmath>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace physics_scenes
{

NewtonMjcScene::NewtonMjcScene(const std::variant<std::string, std::vector<std::string>>& paths) : PhysicsScene(paths)
{
    this->applyApi("MjcSceneAPI");
}

void NewtonMjcScene::setDeltaTimes(const array::Array& deltaTimes, const std::optional<array::Array>& indices)
{
    // Keep the solver-agnostic delta time in sync. It also validates the range for both attributes.
    PhysicsScene::setDeltaTimes(deltaTimes, indices);
    this->setAttributeValues("mjc:option:timestep", deltaTimes, indices);
}

array::Array NewtonMjcScene::getDeltaTimes(const std::optional<array::Array>& indices)
{
    // The constructor applies the MuJoCo schema to every wrapped prim, so this only falls back if it
    // was removed afterwards. Reading the attribute of a prim that lacks the schema would throw.
    if (!array::all(this->hasApi("MjcSceneAPI", std::nullopt, indices)).item<bool>())
    {
        return PhysicsScene::getDeltaTimes(indices);
    }
    return std::get<array::Array>(this->getAttributeValues("mjc:option:timestep", indices));
}

void NewtonMjcScene::setTimeStepsPerSecond(const array::Array& timeStepsPerSecond,
                                           const std::optional<array::Array>& indices)
{
    const std::vector<double> values = timeStepsPerSecond.flatten().get<std::vector<double>>();

    // The MuJoCo delta time is a duration, so it holds the reciprocal of the frequency. The
    // conversion is element-wise, so setAttributeValues broadcasts the result over the selection.
    std::vector<std::vector<double>> deltaTimes;
    deltaTimes.reserve(values.size());
    for (double timeStepsPerSecond : values)
    {
        if (!(timeStepsPerSecond > 0.0))
        {
            throw std::invalid_argument("The step frequency must be greater than 0, got " +
                                        std::to_string(timeStepsPerSecond));
        }
        deltaTimes.push_back({ 1.0 / timeStepsPerSecond });
    }

    // Keep the solver-agnostic step frequency in sync.
    PhysicsScene::setTimeStepsPerSecond(timeStepsPerSecond, indices);
    this->setAttributeValues("mjc:option:timestep", array::Array(deltaTimes), indices);
}

array::Array NewtonMjcScene::getTimeStepsPerSecond(const std::optional<array::Array>& indices)
{
    // The constructor applies the MuJoCo schema to every wrapped prim, so this only falls back if it
    // was removed afterwards. Reading the attribute of a prim that lacks the schema would throw.
    if (!array::all(this->hasApi("MjcSceneAPI", std::nullopt, indices)).item<bool>())
    {
        return PhysicsScene::getTimeStepsPerSecond(indices);
    }

    const std::vector<double> values = std::get<array::Array>(this->getAttributeValues("mjc:option:timestep", indices))
                                           .flatten()
                                           .get<std::vector<double>>();
    std::vector<std::vector<int32_t>> timeStepsPerSecond;
    timeStepsPerSecond.reserve(values.size());
    for (double dt : values)
    {
        timeStepsPerSecond.push_back({ dt > 0.0 ? static_cast<int32_t>(std::lround(1.0 / dt)) : 0 });
    }
    return array::Array(timeStepsPerSecond);
}

void NewtonMjcScene::setIntegrators(const std::variant<std::string, std::vector<std::string>>& integrators,
                                    const std::optional<array::Array>& indices)
{
    const std::vector<std::string> tokens = _resolveStringList(integrators, indices);
    details::validateTokens(tokens, { "euler", "rk4", "implicit", "implicitfast" }, "integrator");
    this->setAttributeValues("mjc:option:integrator", tokens, indices);
}

std::vector<std::string> NewtonMjcScene::getIntegrators(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("mjc:option:integrator", indices));
}

void NewtonMjcScene::setSolvers(const std::variant<std::string, std::vector<std::string>>& solvers,
                                const std::optional<array::Array>& indices)
{
    const std::vector<std::string> tokens = _resolveStringList(solvers, indices);
    details::validateTokens(tokens, { "pgs", "cg", "newton" }, "solver");
    this->setAttributeValues("mjc:option:solver", tokens, indices);
}

std::vector<std::string> NewtonMjcScene::getSolvers(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("mjc:option:solver", indices));
}

void NewtonMjcScene::setCones(const std::variant<std::string, std::vector<std::string>>& cones,
                              const std::optional<array::Array>& indices)
{
    const std::vector<std::string> tokens = _resolveStringList(cones, indices);
    details::validateTokens(tokens, { "pyramidal", "elliptic" }, "friction cone type");
    this->setAttributeValues("mjc:option:cone", tokens, indices);
}

std::vector<std::string> NewtonMjcScene::getCones(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("mjc:option:cone", indices));
}

void NewtonMjcScene::setJacobians(const std::variant<std::string, std::vector<std::string>>& jacobians,
                                  const std::optional<array::Array>& indices)
{
    const std::vector<std::string> tokens = _resolveStringList(jacobians, indices);
    details::validateTokens(tokens, { "auto", "dense", "sparse" }, "Jacobian type");
    this->setAttributeValues("mjc:option:jacobian", tokens, indices);
}

std::vector<std::string> NewtonMjcScene::getJacobians(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("mjc:option:jacobian", indices));
}

void NewtonMjcScene::setIterations(const array::Array& iterations, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("mjc:option:iterations", iterations, indices);
}

array::Array NewtonMjcScene::getIterations(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("mjc:option:iterations", indices));
}

void NewtonMjcScene::setTolerances(const array::Array& tolerances, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("mjc:option:tolerance", tolerances, indices);
}

array::Array NewtonMjcScene::getTolerances(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("mjc:option:tolerance", indices));
}

void NewtonMjcScene::setImpedanceRatios(const array::Array& impedanceRatios, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("mjc:option:impratio", impedanceRatios, indices);
}

array::Array NewtonMjcScene::getImpedanceRatios(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("mjc:option:impratio", indices));
}

void NewtonMjcScene::setWinds(const array::Array& winds, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("mjc:option:wind", winds, indices);
}

array::Array NewtonMjcScene::getWinds(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("mjc:option:wind", indices));
}

void NewtonMjcScene::setDensities(const array::Array& densities, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("mjc:option:density", densities, indices);
}

array::Array NewtonMjcScene::getDensities(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("mjc:option:density", indices));
}

void NewtonMjcScene::setViscosities(const array::Array& viscosities, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("mjc:option:viscosity", viscosities, indices);
}

array::Array NewtonMjcScene::getViscosities(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("mjc:option:viscosity", indices));
}

} // namespace physics_scenes
} // namespace objects
} // namespace foundation
} // namespace isaacsim
