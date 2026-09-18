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

#include <doctest/doctest.h>
#include <isaacsim/physics/registration/Physics.hpp>

#include <cstring>
#include <string>
#include <vector>


using namespace isaacsim::physics::registration;

//-----------------------------------------------------------------------------
// Simulation Registry Tests
TEST_CASE("Simulation Registry Tests")
{

    SUBCASE("Create a new simulation")
    {
        Simulation simulation;
        const std::string simulationName = "TestSimulation";

        SimulationId simulationId = registerSimulation(simulation, simulationName);
        REQUIRE(simulationId != g_kInvalidSimulationId);

        // Test getting simulation name
        REQUIRE(getSimulationName(simulationId) == simulationName);

        unregisterSimulation(simulationId);
    }

    SUBCASE("Register and unregister simulation")
    {
        Simulation simulation;
        const std::string simulationName = "TestSimulation";

        // Test registration
        SimulationId simulationId = registerSimulation(simulation, simulationName);
        REQUIRE(simulationId != g_kInvalidSimulationId);

        // Test getting simulation by ID
        const Simulation* retrievedSimulation = getSimulation(simulationId);
        REQUIRE(retrievedSimulation != nullptr);

        // Test getting simulation name
        REQUIRE(getSimulationName(simulationId) == simulationName);

        // Test unregistration
        unregisterSimulation(simulationId);
        const Simulation* afterUnregister = getSimulation(simulationId);
        REQUIRE(afterUnregister == nullptr);
    }

    SUBCASE("Get number of simulations")
    {
        size_t initialCount = getSimulationCount();

        Simulation simulation;
        SimulationId simulationId = registerSimulation(simulation, "CountTest");

        size_t newCount = getSimulationCount();
        REQUIRE(newCount == initialCount + 1);

        unregisterSimulation(simulationId);

        size_t finalCount = getSimulationCount();
        REQUIRE(finalCount == initialCount);
    }

    SUBCASE("Activate and deactivate simulation")
    {
        Simulation simulation;
        SimulationId simulationId = registerSimulation(simulation, "ActivationTest");

        // Simulation should start active
        bool isActive = isSimulationActive(simulationId);
        REQUIRE(isActive == true);

        // Deactivate
        deactivateSimulation(simulationId);
        isActive = isSimulationActive(simulationId);
        REQUIRE(isActive == false);

        // Reactivate
        activateSimulation(simulationId);
        isActive = isSimulationActive(simulationId);
        REQUIRE(isActive == true);

        unregisterSimulation(simulationId);
    }

    SUBCASE("Get simulation IDs")
    {
        // Register multiple simulations
        SimulationId ids[3];
        for (int i = 0; i < 3; i++)
        {
            Simulation simulation;
            ids[i] = registerSimulation(simulation, "Sim" + std::to_string(i));
        }

        // Get simulation IDs via the public buffer API
        std::vector<SimulationId> allIds(getSimulationCount());
        const size_t idCount = getSimulationIds(allIds.data(), allIds.size());
        REQUIRE(idCount >= 3);

        // Clean up
        for (int i = 0; i < 3; i++)
        {
            unregisterSimulation(ids[i]);
        }
    }

    SUBCASE("Empty simulation ID buffer")
    {
        Simulation simulation;
        const SimulationId simulationId = registerSimulation(simulation, "ZeroBufferTest");

        REQUIRE(getSimulationIds(nullptr, 0) == 0);
        REQUIRE(getSimulationIds(nullptr, 1) == 0);

        unregisterSimulation(simulationId);
    }

    SUBCASE("Invalid simulation operations")
    {
        // Test with invalid ID
        const Simulation* invalidSimulation = getSimulation(g_kInvalidSimulationId);
        REQUIRE(invalidSimulation == nullptr);

        const std::string invalidName = getSimulationName(g_kInvalidSimulationId);
        REQUIRE(invalidName.empty());

        bool invalidActive = isSimulationActive(g_kInvalidSimulationId);
        REQUIRE(invalidActive == false);
    }
}
