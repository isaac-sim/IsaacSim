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

#include <doctest/doctest.h>
#include <isaacsim/physics/manager/PhysicsManager.hpp>
#include <isaacsim/physics_engines/ovphysx/Backend.hpp>

#include <stdexcept>

namespace
{

class BackendGuard
{
public:
    ~BackendGuard()
    {
        auto& manager = isaacsim::physics::manager::PhysicsManager::getInstance();
        if (manager.isInitialized())
        {
            static_cast<void>(manager.invalidate());
        }
        isaacsim::physics_engines::ovphysx::shutdown();
    }
};

} // namespace

TEST_CASE("ovphysx synchronous step preserves manager accounting on failure")
{
    namespace ovphysx = isaacsim::physics_engines::ovphysx;
    using isaacsim::physics::manager::PhysicsManager;

    REQUIRE(ovphysx::activate());
    BackendGuard guard;

    auto& manager = PhysicsManager::getInstance();
    manager.configure(1.0f / 60.0f);
    // A null Stage initializes an empty simulation, but OvPhysX rejects
    // stepping it because there is no attached Stage.
    REQUIRE(manager.initialize(nullptr, 0));

    CHECK_THROWS_AS(manager.step(), std::runtime_error);
    CHECK(manager.getSimulatedPhysicsStepCount() == 0);
    CHECK(manager.getSimulatedTime() == doctest::Approx(0.0f));
}
