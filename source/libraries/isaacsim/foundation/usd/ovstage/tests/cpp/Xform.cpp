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
#include <isaacsim/foundation/usd/ovstage/Usd.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace isaacsim::foundation::usd::ovstage;
namespace array = isaacsim::common::array;

namespace
{

bool hasCanonicalTransformOpsOrder(int64_t stageId, const std::string& path)
{
    auto values = getPrimAttributeValues(stageId, { path }, "xformOpOrder");
    return std::get<std::vector<std::vector<std::string>>>(values)[0] ==
           std::vector<std::string>{ "xformOp:translate", "xformOp:orient", "xformOp:scale" };
}

} // namespace

TEST_SUITE("Xform")
{

    TEST_CASE("getXformLocalScales | setXformLocalScales")
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/A", "Xform");
        definePrim(stageId, "/World/B", "Xform");
        definePrim(stageId, "/World/Scope", "Scope");

        auto scales = getXformLocalScales(stageId, { "/World/A", "/World/B" });
        auto scalesVector = scales.get<std::vector<std::vector<double>>>();
        REQUIRE_EQ(scalesVector.size(), 2);
        for (size_t i = 0; i < scalesVector.size(); ++i)
        {
            CHECK_UNARY(scalesVector[i][0] == doctest::Approx(1.0));
            CHECK_UNARY(scalesVector[i][1] == doctest::Approx(1.0));
            CHECK_UNARY(scalesVector[i][2] == doctest::Approx(1.0));
        }

        // Set different scales for each prim
        // resetXformOpProperties(stageId, "/World/A");
        // resetXformOpProperties(stageId, "/World/B");
        setXformLocalScales(stageId, { "/World/A", "/World/B" },
                            array::Array(std::vector<std::vector<double>>{ { 2.0, 3.0, 4.0 }, { 5.0, 6.0, 7.0 } }));
        scales = getXformLocalScales(stageId, { "/World/A", "/World/B" });
        scalesVector = scales.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(scalesVector[0][0] == doctest::Approx(2.0));
        CHECK_UNARY(scalesVector[0][1] == doctest::Approx(3.0));
        CHECK_UNARY(scalesVector[0][2] == doctest::Approx(4.0));
        CHECK_UNARY(scalesVector[1][0] == doctest::Approx(5.0));
        CHECK_UNARY(scalesVector[1][1] == doctest::Approx(6.0));
        CHECK_UNARY(scalesVector[1][2] == doctest::Approx(7.0));

        closeStage(stageId);
    }

    TEST_CASE("getXformLocalPoses | setXformLocalPoses")
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/A", "Xform");
        definePrim(stageId, "/World/B", "Xform");

        // Default: identity poses
        auto [translations, orientations] = getXformLocalPoses(stageId, { "/World/A", "/World/B" });
        auto translationsVector = translations.get<std::vector<std::vector<double>>>();
        auto orientationsVector = orientations.get<std::vector<std::vector<double>>>();
        REQUIRE_EQ(translationsVector.size(), 2);
        REQUIRE_EQ(orientationsVector.size(), 2);
        for (size_t i = 0; i < translationsVector.size(); ++i)
        {
            CHECK_UNARY(translationsVector[i][0] == doctest::Approx(0.0));
            CHECK_UNARY(translationsVector[i][1] == doctest::Approx(0.0));
            CHECK_UNARY(translationsVector[i][2] == doctest::Approx(0.0));
            CHECK_UNARY(orientationsVector[i][0] == doctest::Approx(0.0));
            CHECK_UNARY(orientationsVector[i][1] == doctest::Approx(0.0));
            CHECK_UNARY(orientationsVector[i][2] == doctest::Approx(0.0));
            CHECK_UNARY(orientationsVector[i][3] == doctest::Approx(1.0));
        }

        // Set translations for both prims
        // resetXformOpProperties(stageId, "/World/A");
        // resetXformOpProperties(stageId, "/World/B");
        setXformLocalPoses(stageId, { "/World/A", "/World/B" },
                           array::Array(std::vector<std::vector<double>>{ { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 } }));
        std::tie(translations, orientations) = getXformLocalPoses(stageId, { "/World/A", "/World/B" });
        translationsVector = translations.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(translationsVector[0][0] == doctest::Approx(1.0));
        CHECK_UNARY(translationsVector[0][1] == doctest::Approx(2.0));
        CHECK_UNARY(translationsVector[0][2] == doctest::Approx(3.0));
        CHECK_UNARY(translationsVector[1][0] == doctest::Approx(4.0));
        CHECK_UNARY(translationsVector[1][1] == doctest::Approx(5.0));
        CHECK_UNARY(translationsVector[1][2] == doctest::Approx(6.0));

        // Set orientations only
        setXformLocalPoses(
            stageId, { "/World/A" }, std::nullopt,
            array::Array(std::vector<std::vector<double>>{ { 0.6180884, 0.0165899, 0.6776496, 0.3980986 } }));
        std::tie(translations, orientations) = getXformLocalPoses(stageId, { "/World/A" });
        orientationsVector = orientations.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(std::abs(orientationsVector[0][0]) == doctest::Approx(0.6180884));
        CHECK_UNARY(std::abs(orientationsVector[0][1]) == doctest::Approx(0.0165899));
        CHECK_UNARY(std::abs(orientationsVector[0][2]) == doctest::Approx(0.6776496));
        CHECK_UNARY(std::abs(orientationsVector[0][3]) == doctest::Approx(0.3980986));

        closeStage(stageId);
    }

    TEST_CASE("getXformWorldPoses | setXformWorldPoses - hierarchy and multi-prim")
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World", "Xform");
        definePrim(stageId, "/World/A", "Xform");
        definePrim(stageId, "/World/A/B", "Xform");
        definePrim(stageId, "/World/C", "Xform");

        // Place parents
        // resetXformOpProperties(stageId, "/World/A");
        // resetXformOpProperties(stageId, "/World/A/B");
        // resetXformOpProperties(stageId, "/World/C");
        setXformLocalPoses(stageId, { "/World/A" }, array::Array(std::vector<std::vector<double>>{ { 10.0, 0.0, 0.0 } }));
        setXformLocalPoses(
            stageId, { "/World/A/B" }, array::Array(std::vector<std::vector<double>>{ { 5.0, 0.0, 0.0 } }));
        setXformLocalPoses(stageId, { "/World/C" }, array::Array(std::vector<std::vector<double>>{ { 0.0, 7.0, 0.0 } }));

        // World positions: /World/A/B -> (15, 0, 0), /World/C -> (0, 7, 0)
        auto [positions, orientations] = getXformWorldPoses(stageId, { "/World/A/B", "/World/C" });
        auto positionsVector = positions.get<std::vector<std::vector<double>>>();
        REQUIRE_EQ(positionsVector.size(), 2);
        CHECK_UNARY(positionsVector[0][0] == doctest::Approx(15.0));
        CHECK_UNARY(positionsVector[0][1] == doctest::Approx(0.0));
        CHECK_UNARY(positionsVector[1][0] == doctest::Approx(0.0));
        CHECK_UNARY(positionsVector[1][1] == doctest::Approx(7.0));

        // setXformWorldPoses: move /World/A/B to world (20, 0, 0); local should become (10, 0, 0)
        setXformWorldPoses(
            stageId, { "/World/A/B" }, array::Array(std::vector<std::vector<double>>{ { 20.0, 0.0, 0.0 } }));
        std::tie(positions, orientations) = getXformWorldPoses(stageId, { "/World/A/B" });
        positionsVector = positions.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(positionsVector[0][0] == doctest::Approx(20.0));

        auto [localTranslations, localOrientations] = getXformLocalPoses(stageId, { "/World/A/B" });
        auto localTranslationsVector = localTranslations.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(localTranslationsVector[0][0] == doctest::Approx(10.0));

        // setXformWorldPoses orientation: 90-degree rotation around Z
        const double s = std::sqrt(2.0) / 2.0;
        setXformWorldPoses(stageId, { "/World/C" }, std::nullopt,
                           array::Array(std::vector<std::vector<double>>{ { 0.0, 0.0, s, s } }));
        std::tie(positions, orientations) = getXformWorldPoses(stageId, { "/World/C" });
        auto orientationsVector = orientations.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(std::abs(orientationsVector[0][0]) == doctest::Approx(0.0));
        CHECK_UNARY(std::abs(orientationsVector[0][1]) == doctest::Approx(0.0));
        CHECK_UNARY(std::abs(orientationsVector[0][2]) == doctest::Approx(s));
        CHECK_UNARY(std::abs(orientationsVector[0][3]) == doctest::Approx(s));

        // Multi-prim set: move both prims at once
        setXformWorldPoses(stageId, { "/World/A/B", "/World/C" },
                           array::Array(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 }, { 2.0, 0.0, 0.0 } }));
        std::tie(positions, orientations) = getXformWorldPoses(stageId, { "/World/A/B", "/World/C" });
        positionsVector = positions.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(positionsVector[0][0] == doctest::Approx(1.0));
        CHECK_UNARY(positionsVector[1][0] == doctest::Approx(2.0));

        // Row count mismatch
        CHECK_THROWS_AS(
            setXformWorldPoses(stageId, { "/World/A/B" },
                               array::Array(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 }, { 2.0, 0.0, 0.0 } })),
            std::invalid_argument);

        closeStage(stageId);
    }

    TEST_CASE("setXformLocalPoses preserves scale across multiple prims")
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/A", "Xform");
        definePrim(stageId, "/World/B", "Xform");

        // resetXformOpProperties(stageId, "/World/A");
        // resetXformOpProperties(stageId, "/World/B");
        setXformLocalScales(stageId, { "/World/A", "/World/B" },
                            array::Array(std::vector<std::vector<double>>{ { 2.0, 3.0, 4.0 }, { 5.0, 6.0, 7.0 } }));
        setXformLocalPoses(stageId, { "/World/A", "/World/B" },
                           array::Array(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }));

        auto scales = getXformLocalScales(stageId, { "/World/A", "/World/B" });
        auto scalesVector = scales.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(scalesVector[0][0] == doctest::Approx(2.0));
        CHECK_UNARY(scalesVector[0][1] == doctest::Approx(3.0));
        CHECK_UNARY(scalesVector[0][2] == doctest::Approx(4.0));
        CHECK_UNARY(scalesVector[1][0] == doctest::Approx(5.0));
        CHECK_UNARY(scalesVector[1][1] == doctest::Approx(6.0));
        CHECK_UNARY(scalesVector[1][2] == doctest::Approx(7.0));

        auto [translations, orientations] = getXformLocalPoses(stageId, { "/World/A", "/World/B" });
        auto translationsVector = translations.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(translationsVector[0][0] == doctest::Approx(1.0));
        CHECK_UNARY(translationsVector[1][1] == doctest::Approx(1.0));

        closeStage(stageId);
    }

    TEST_CASE("setXformWorldPoses - Scope ancestor does not corrupt local matrix")
    {
        // Hierarchy: /World (Xform) -> /World/X (Xform, T=(10,0,0)) -> /World/X/Scope (Scope) -> /World/X/Scope/Child
        // (Xform)
        int64_t stageId = createStage();
        definePrim(stageId, "/World", "Xform");
        definePrim(stageId, "/World/X", "Xform");
        definePrim(stageId, "/World/X/Scope", "Scope");
        definePrim(stageId, "/World/X/Scope/Child", "Xform");

        // Give /World/X a known world translation
        // resetXformOpProperties(stageId, "/World/X");
        setXformLocalPoses(stageId, { "/World/X" }, array::Array(std::vector<std::vector<double>>{ { 10.0, 0.0, 0.0 } }));

        // Child starts at its parent's world position (inherits X's transform through Scope)
        auto [positions, orientations] = getXformWorldPoses(stageId, { "/World/X/Scope/Child" });
        auto positionsVector = positions.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(positionsVector[0][0] == doctest::Approx(10.0));
        CHECK_UNARY(positionsVector[0][1] == doctest::Approx(0.0));
        CHECK_UNARY(positionsVector[0][2] == doctest::Approx(0.0));

        // Move Child to world (20, 5, 0): local offset from X should be (10, 5, 0)
        // resetXformOpProperties(stageId, "/World/X/Scope/Child");
        setXformWorldPoses(
            stageId, { "/World/X/Scope/Child" }, array::Array(std::vector<std::vector<double>>{ { 20.0, 5.0, 0.0 } }));

        std::tie(positions, orientations) = getXformWorldPoses(stageId, { "/World/X/Scope/Child" });
        positionsVector = positions.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(positionsVector[0][0] == doctest::Approx(20.0));
        CHECK_UNARY(positionsVector[0][1] == doctest::Approx(5.0));
        CHECK_UNARY(positionsVector[0][2] == doctest::Approx(0.0));

        auto [localTranslations, localOrientations] = getXformLocalPoses(stageId, { "/World/X/Scope/Child" });
        auto localTranslationsVector = localTranslations.get<std::vector<std::vector<double>>>();
        CHECK_UNARY(localTranslationsVector[0][0] == doctest::Approx(10.0));
        CHECK_UNARY(localTranslationsVector[0][1] == doctest::Approx(5.0));
        CHECK_UNARY(localTranslationsVector[0][2] == doctest::Approx(0.0));

        closeStage(stageId);
    }

    TEST_CASE("resetXformOpProperties" * doctest::skip())
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World", "Xform");

        CHECK_UNARY_FALSE(hasCanonicalTransformOpsOrder(stageId, "/World"));
        resetXformOpProperties(stageId, "/World");
        CHECK_UNARY(hasCanonicalTransformOpsOrder(stageId, "/World"));
        resetXformOpProperties(stageId, "/World");
        CHECK_UNARY(hasCanonicalTransformOpsOrder(stageId, "/World"));

        closeStage(stageId);
    }

    TEST_CASE("Non-Xformable prim throws")
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/Scope", "Scope");

        // CHECK_THROWS_AS(resetXformOpProperties(stageId, "/World/Scope"), std::invalid_argument);
        CHECK_THROWS_AS(getXformLocalScales(stageId, { "/World/Scope" }), std::invalid_argument);
        CHECK_THROWS_AS(getXformLocalPoses(stageId, { "/World/Scope" }), std::invalid_argument);
        CHECK_THROWS_AS(getXformWorldPoses(stageId, { "/World/Scope" }), std::invalid_argument);
        CHECK_THROWS_AS(setXformLocalScales(stageId, { "/World/Scope" },
                                            array::Array(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 } })),
                        std::invalid_argument);
        CHECK_THROWS_AS(setXformLocalPoses(stageId, { "/World/Scope" },
                                           array::Array(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 } })),
                        std::invalid_argument);
        CHECK_THROWS_AS(setXformWorldPoses(stageId, { "/World/Scope" },
                                           array::Array(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 } })),
                        std::invalid_argument);

        closeStage(stageId);
    }

} // TEST_SUITE("Xform")
