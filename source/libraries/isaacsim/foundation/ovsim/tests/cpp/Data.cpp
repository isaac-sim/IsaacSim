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
#include <isaacsim/foundation/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/foundation/ovsim/data/Data.hpp>

#include <IsaacSimTest.hpp>
#include <filesystem>
#include <stdexcept>

namespace array = isaacsim::common::array;
using namespace isaacsim::foundation::ovsim::data;
using namespace isaacsim::foundation::ovsim::control;

TEST_SUITE("read")
{
    TEST_CASE("read")
    {
        REQUIRE_UNARY(authoring::createStage());

        REQUIRE_UNARY(authoring::definePrim("/World/A", "Cube"));
        REQUIRE_UNARY(authoring::definePrim("/World/A", "RigidBody"));

        CHECK_EQ(std::get<array::Array>(read("/World/A", "size")).item<float>(), 2.0f);
        CHECK_EQ(std::get<array::Array>(read("/World/A", "physics:mass")).item<float>(), 0.0f);

        CHECK_EQ(std::get<array::Array>(read("/World/A", "mass")).item<float>(), 0.0f);
        CHECK_EQ(std::get<array::Array>(read("/World/A", "position")).at(0).get<std::vector<float>>(),
                 std::vector<float>{ 0.0f, 0.0f, 0.0f });

        REQUIRE_UNARY(authoring::closeStage());
    }
}
