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

#include <doctest/doctest.h>
#include <isaacsim/physics/manager/PhysicsSimulation.hpp>
#include <isaacsim/physics/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/physics/ovsim/control/simulation/Simulation.hpp>
#include <isaacsim/physics/registration/Physics.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>

using namespace isaacsim::physics::ovsim::control;

namespace registration = isaacsim::physics::registration;

namespace
{

struct MockBackend
{
    bool initializeResult{ true };
    bool closeResult{ true };
};

registration::Simulation makeSimulation(MockBackend& backend)
{
    registration::Simulation simulation;
    simulation.simulationFunctions.initialize = [&backend](void*, const char*) { return backend.initializeResult; };
    simulation.simulationFunctions.close = [&backend]() { return backend.closeResult; };
    simulation.simulationFunctions.hasAttachedStage = []() { return false; };
    return simulation;
}

} // namespace

TEST_SUITE("Authoring")
{
}

TEST_SUITE("Simulation")
{
    TEST_CASE("surfaces a false PhysicsManager initialization result")
    {
        MockBackend backend;
        backend.initializeResult = false;
        const auto simulationId = registration::registerSimulation(makeSimulation(backend), "ovsim-init-failure");
        simulation::setParameter("physics", "ovstage-stage-ptr", static_cast<uintptr_t>(1));

        CHECK_THROWS_WITH_AS(simulation::initialize(), "Physics manager initialization failed.", std::runtime_error);

        registration::unregisterSimulation(simulationId);
        simulation::setParameter("physics", "ovstage-stage-ptr", static_cast<uintptr_t>(0));
    }

    TEST_CASE("surfaces a false PhysicsManager invalidation result and permits retry")
    {
        MockBackend backend;
        backend.closeResult = false;
        const auto simulationId = registration::registerSimulation(makeSimulation(backend), "ovsim-close-failure");
        simulation::setParameter("physics", "ovstage-stage-ptr", static_cast<uintptr_t>(1));
        REQUIRE_NOTHROW(simulation::initialize());

        CHECK_THROWS_WITH_AS(simulation::invalidate(), "Physics manager invalidation failed.", std::runtime_error);
        backend.closeResult = true;
        CHECK_NOTHROW(simulation::invalidate());

        registration::unregisterSimulation(simulationId);
        simulation::setParameter("physics", "ovstage-stage-ptr", static_cast<uintptr_t>(0));
    }
}
