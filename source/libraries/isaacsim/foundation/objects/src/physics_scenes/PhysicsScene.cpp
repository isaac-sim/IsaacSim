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

#include <isaacsim/foundation/objects/physics_scenes/PhysicsScene.hpp>
#include <isaacsim/foundation/usd/openusd/Usd.hpp>
#include <isaacsim/foundation/usd/ovstage/Usd.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace physics_scenes
{

namespace openusd = isaacsim::foundation::usd::openusd;
namespace ovstage = isaacsim::foundation::usd::ovstage;

namespace details
{

array::Array deltaTimesToTimeStepsPerSecond(const array::Array& deltaTimes)
{
    const std::vector<double> values = deltaTimes.flatten().get<std::vector<double>>();

    std::vector<std::vector<int32_t>> timeStepsPerSecond;
    timeStepsPerSecond.reserve(values.size());
    for (double dt : values)
    {
        if (!(dt > 0.0) || dt > 1.0)
        {
            throw std::invalid_argument("The delta time (DT) must be in the range (0.0, 1.0], got " + std::to_string(dt));
        }
        // Round rather than truncate: a delta time held as a 32-bit float turns 1/120 into
        // 119.99997 Hz, which truncation would degrade to 119 Hz.
        const double frequency = std::round(1.0 / dt);
        if (frequency > static_cast<double>(std::numeric_limits<int32_t>::max()))
        {
            throw std::invalid_argument(
                "The delta time (DT) is too small to be stored as an integer "
                "step frequency, got " +
                std::to_string(dt));
        }
        timeStepsPerSecond.push_back({ static_cast<int32_t>(frequency) });
    }
    return array::Array(timeStepsPerSecond);
}

array::Array timeStepsPerSecondToDeltaTimes(const array::Array& timeStepsPerSecond)
{
    const std::vector<int64_t> values = timeStepsPerSecond.flatten().get<std::vector<int64_t>>();

    std::vector<std::vector<float>> deltaTimes;
    deltaTimes.reserve(values.size());
    for (int64_t steps : values)
    {
        deltaTimes.push_back({ steps > 0 ? static_cast<float>(1.0 / static_cast<double>(steps)) : 0.0f });
    }
    return array::Array(deltaTimes);
}

void validateTokens(const std::vector<std::string>& tokens,
                    const std::vector<std::string>& allowedTokens,
                    const std::string& description)
{
    for (const auto& token : tokens)
    {
        if (std::find(allowedTokens.begin(), allowedTokens.end(), token) == allowedTokens.end())
        {
            std::string joinedTokens;
            for (const auto& allowedToken : allowedTokens)
            {
                joinedTokens += (joinedTokens.empty() ? "'" : ", '") + allowedToken + "'";
            }
            throw std::invalid_argument("The " + description + " must be one of " + joinedTokens + ", got '" + token +
                                        "'");
        }
    }
}

} // namespace details

namespace
{

/** @brief Standard gravity, in meters per second squared. */
constexpr float g_kStandardGravity = 9.81f;

/** @brief Returns the unit vector pointing down for a stage with the given up axis. */
std::array<float, 3> downDirection(const std::string& upAxis)
{
    return upAxis == "Y" ? std::array<float, 3>{ 0.0f, -1.0f, 0.0f } : std::array<float, 3>{ 0.0f, 0.0f, -1.0f };
}

} // namespace

PhysicsScene::PhysicsScene(const std::variant<std::string, std::vector<std::string>>& paths) : Prim()
{
    // Get or create prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    // Get prims.
    if (!existentPaths.empty())
    {
        m_paths = std::move(existentPaths);
        const std::vector<bool> isPhysicsScene = this->isA("PhysicsScene").flatten().get<std::vector<bool>>();
        for (std::size_t i = 0; i < m_paths.size(); ++i)
        {
            if (!isPhysicsScene[i])
            {
                throw std::runtime_error("The wrapped prim at path '" + m_paths[i] + "' is not a USD PhysicsScene");
            }
        }
    }
    // Create prims.
    else
    {
        m_paths = std::move(nonexistentPaths);
        for (const auto& path : m_paths)
        {
            this->getStage().definePrim(path, "PhysicsScene");
        }
    }
    // The solver-agnostic settings exposed by this class are declared by the Newton scene schema.
    this->applyApi("NewtonSceneAPI");
}

array::Array PhysicsScene::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("PhysicsScene");
}

std::vector<std::string> PhysicsScene::getPhysicsScenePaths()
{
    const Stage& stage = objects::details::getActiveStage();
    // TODO: optimize this
    // Traversing from the pseudo-root reports physics scenes nested at any depth.
    std::vector<std::string> paths = stage.getBackend() == "ovstage" ?
                                         ovstage::findMatchingPrimPaths(stage.getStageId(), "/", true) :
                                         openusd::findMatchingPrimPaths(stage.getStageId(), "/", true);
    paths.erase(std::remove(paths.begin(), paths.end(), "/"), paths.end());
    if (paths.empty())
    {
        return paths;
    }

    const std::vector<bool> isPhysicsScene =
        Prim(paths, /*resolvePaths=*/false).isA("PhysicsScene").flatten().get<std::vector<bool>>();
    std::vector<std::string> physicsScenePaths;
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        if (isPhysicsScene[i])
        {
            physicsScenePaths.push_back(std::move(paths[i]));
        }
    }
    return physicsScenePaths;
}

void PhysicsScene::setGravities(const array::Array& gravities, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    const std::vector<float> values =
        gravities.broadcastTo(array::Shape({ batchSize, int64_t{ 3 } })).flatten().get<std::vector<float>>();
    const std::array<float, 3> down = downDirection(this->getStage().getUpAxis());

    std::vector<std::vector<float>> magnitudes;
    std::vector<std::vector<float>> directions;
    magnitudes.reserve(batchSize);
    directions.reserve(batchSize);
    for (int64_t i = 0; i < batchSize; ++i)
    {
        const float x = values[3 * i];
        const float y = values[3 * i + 1];
        const float z = values[3 * i + 2];
        const float length = std::sqrt(x * x + y * y + z * z);
        // The magnitude attribute is expressed in the stage's own distance unit, so the vector needs
        // no conversion. A negative magnitude would read back as a request for earth gravity, which
        // the non-negative length can never produce.
        magnitudes.push_back({ length });
        // A zero-length vector carries no direction; keep the stage's down direction so that a later
        // magnitude change resolves to a sensible gravity.
        directions.push_back(length > 0.0f ? std::vector<float>{ x / length, y / length, z / length } :
                                             std::vector<float>{ down[0], down[1], down[2] });
    }

    this->setAttributeValues("physics:gravityMagnitude", array::Array(magnitudes), indices);
    this->setAttributeValues("physics:gravityDirection", array::Array(directions), indices);
}

array::Array PhysicsScene::getGravities(const std::optional<array::Array>& indices)
{
    const std::vector<float> magnitudes =
        std::get<array::Array>(this->getAttributeValues("physics:gravityMagnitude", indices))
            .flatten()
            .get<std::vector<float>>();
    const std::vector<float> directions =
        std::get<array::Array>(this->getAttributeValues("physics:gravityDirection", indices))
            .flatten()
            .get<std::vector<float>>();

    const float metersPerUnit = std::get<0>(this->getStage().getUnits());
    const std::array<float, 3> down = downDirection(this->getStage().getUpAxis());

    std::vector<std::vector<float>> gravities;
    gravities.reserve(magnitudes.size());
    for (std::size_t i = 0; i < magnitudes.size(); ++i)
    {
        const float x = directions[3 * i];
        const float y = directions[3 * i + 1];
        const float z = directions[3 * i + 2];
        const float length = std::sqrt(x * x + y * y + z * z);
        // USD defines the two sentinels independently: a zero direction is a request for the stage's
        // down direction, and a negative magnitude (the -inf default included) is a request for earth
        // gravity, expressed in stage units regardless of how the stage is scaled.
        const std::array<float, 3> direction =
            length > 0.0f ? std::array<float, 3>{ x / length, y / length, z / length } : down;
        const float magnitude = magnitudes[i] < 0.0f ? g_kStandardGravity / metersPerUnit : magnitudes[i];
        gravities.push_back({ direction[0] * magnitude, direction[1] * magnitude, direction[2] * magnitude });
    }
    return array::Array(gravities);
}

void PhysicsScene::setDeltaTimes(const array::Array& deltaTimes, const std::optional<array::Array>& indices)
{
    this->setTimeStepsPerSecond(details::deltaTimesToTimeStepsPerSecond(deltaTimes), indices);
}

array::Array PhysicsScene::getDeltaTimes(const std::optional<array::Array>& indices)
{
    return details::timeStepsPerSecondToDeltaTimes(this->getTimeStepsPerSecond(indices));
}

void PhysicsScene::setTimeStepsPerSecond(const array::Array& timeStepsPerSecond,
                                         const std::optional<array::Array>& indices)
{
    this->setAttributeValues("newton:timeStepsPerSecond", timeStepsPerSecond, indices);
}

array::Array PhysicsScene::getTimeStepsPerSecond(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("newton:timeStepsPerSecond", indices));
}

void PhysicsScene::setEnabledGravities(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("newton:gravityEnabled", enabled, indices);
}

array::Array PhysicsScene::getEnabledGravities(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("newton:gravityEnabled", indices));
}

void PhysicsScene::setMaxSolverIterations(const array::Array& iterations, const std::optional<array::Array>& indices)
{
    this->setAttributeValues("newton:maxSolverIterations", iterations, indices);
}

array::Array PhysicsScene::getMaxSolverIterations(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("newton:maxSolverIterations", indices));
}


} // namespace physics_scenes
} // namespace objects
} // namespace foundation
} // namespace isaacsim
