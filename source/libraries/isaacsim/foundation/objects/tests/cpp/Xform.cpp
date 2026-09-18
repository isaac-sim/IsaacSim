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
#include <isaacsim/foundation/objects/Stage.hpp>
#include <isaacsim/foundation/objects/Xform.hpp>

#include <IsaacSimTest.hpp>
#include <stdexcept>

using namespace isaacsim::foundation::objects;

TEST_SUITE("Xform")
{

    TEST_CASE("Xform::Xform (create)")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        // Non-existing paths are created as Xform prims
        Xform prim(std::vector<std::string>{ "/World/A", "/World/B" });
        CHECK_EQ(prim.size(), 2u);
        CHECK_EQ(prim.paths(), (std::vector<std::string>{ "/World/A", "/World/B" }));
        CHECK_EQ(prim.getTypeName(), (std::vector<std::string>{ "Xform", "Xform" }));
        CHECK_EQ(prim.isA("Xform").flatten().get<std::vector<bool>>(), (std::vector<bool>{ true, true }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Xform::Xform (wrap existing)")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/X0", "Xform");
        stage.definePrim("/World/X1", "Xform");

        SUBCASE("explicit paths")
        {
            Xform prim(std::vector<std::string>{ "/World/X0", "/World/X1" });
            CHECK_EQ(prim.size(), 2u);
            CHECK_EQ(prim.paths(), (std::vector<std::string>{ "/World/X0", "/World/X1" }));
            CHECK_EQ(prim.isA("Xform").flatten().get<std::vector<bool>>(), (std::vector<bool>{ true, true }));
        }

        SUBCASE("regex")
        {
            Xform prim("/World/X.*");
            CHECK_EQ(prim.size(), 2u);
            CHECK_EQ(prim.getTypeName(), (std::vector<std::string>{ "Xform", "Xform" }));
        }

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Xform::Xform (non-Xform throws)")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/Scope", "Scope");

        CHECK_THROWS_AS(Xform("/World/Scope"), std::runtime_error);
        // A mix that includes a non-Xformable existing prim also throws
        stage.definePrim("/World/Xform", "Xform");
        CHECK_THROWS_AS(Xform(std::vector<std::string>{ "/World/Xform", "/World/Scope" }), std::runtime_error);

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Xform pose methods - rotation format")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        // A unit quaternion whose four components are distinct and non-zero, so that a wrong
        // component permutation cannot coincide with the expected result.
        const std::vector<double> expectedXyzw{ 0.2, -0.4, 0.8, 0.4 };
        const std::vector<double> expectedWxyz{ 0.4, 0.2, -0.4, 0.8 };
        const array::Array orientationsXyzw(std::vector<std::vector<double>>{ expectedXyzw });
        const array::Array orientationsWxyz(std::vector<std::vector<double>>{ expectedWxyz });

        auto checkRow = [](const array::Array& orientations, const std::vector<double>& expected)
        {
            const auto rows = orientations.get<std::vector<std::vector<double>>>();
            REQUIRE_EQ(rows.size(), 1u);
            REQUIRE_EQ(rows[0].size(), 4u);
            for (size_t i = 0; i < expected.size(); ++i)
            {
                CHECK_UNARY(rows[0][i] == doctest::Approx(expected[i]));
            }
        };

        SUBCASE("world poses")
        {
            Xform prim("/World/A");
            // The default is xyzw, so an omitted argument matches an explicit one.
            prim.setWorldPoses(std::nullopt, orientationsXyzw);
            checkRow(std::get<1>(prim.getWorldPoses()), expectedXyzw);
            checkRow(std::get<1>(prim.getWorldPoses(std::nullopt, "xyzw")), expectedXyzw);
            // Reading the same pose as wxyz reorders the components.
            checkRow(std::get<1>(prim.getWorldPoses(std::nullopt, "wxyz")), expectedWxyz);
            // Writing as wxyz round-trips, and agrees with an xyzw read of the same pose.
            prim.setWorldPoses(std::nullopt, orientationsWxyz, std::nullopt, "wxyz");
            checkRow(std::get<1>(prim.getWorldPoses(std::nullopt, "wxyz")), expectedWxyz);
            checkRow(std::get<1>(prim.getWorldPoses()), expectedXyzw);
        }

        SUBCASE("local poses")
        {
            Xform prim("/World/B");
            prim.setLocalPoses(std::nullopt, orientationsXyzw);
            checkRow(std::get<1>(prim.getLocalPoses()), expectedXyzw);
            checkRow(std::get<1>(prim.getLocalPoses(std::nullopt, "xyzw")), expectedXyzw);
            checkRow(std::get<1>(prim.getLocalPoses(std::nullopt, "wxyz")), expectedWxyz);
            prim.setLocalPoses(std::nullopt, orientationsWxyz, std::nullopt, "wxyz");
            checkRow(std::get<1>(prim.getLocalPoses(std::nullopt, "wxyz")), expectedWxyz);
            checkRow(std::get<1>(prim.getLocalPoses()), expectedXyzw);
        }

        SUBCASE("unsupported formats are rejected")
        {
            Xform prim("/World/C");
            for (const char* rotationFormat : { "XYZW", "wxzy", "" })
            {
                CHECK_THROWS_AS(prim.getWorldPoses(std::nullopt, rotationFormat), std::invalid_argument);
                CHECK_THROWS_AS(prim.getLocalPoses(std::nullopt, rotationFormat), std::invalid_argument);
                CHECK_THROWS_AS(prim.setWorldPoses(std::nullopt, orientationsXyzw, std::nullopt, rotationFormat),
                                std::invalid_argument);
                CHECK_THROWS_AS(prim.setLocalPoses(std::nullopt, orientationsXyzw, std::nullopt, rotationFormat),
                                std::invalid_argument);
            }
        }

        REQUIRE_UNARY(stage.closeStage());
    }
}
