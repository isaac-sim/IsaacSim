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
#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <isaacsim/foundation/usd/ovstage/Usd.hpp>
#include <ovstage/ovstage.h>

#include <IsaacSimTest.hpp>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace isaacsim::foundation::usd::ovstage;

TEST_SUITE("Stage")
{

    TEST_CASE("createStage | closeStage | isStageValid")
    {
        // Create and close a stage
        int64_t stageId = createStage();
        REQUIRE_NE(stageId, -1);
        REQUIRE_UNARY(isStageValid(stageId));
        REQUIRE_UNARY(closeStage(stageId));
        // Check that the stage is not valid after closing it
        REQUIRE_UNARY_FALSE(isStageValid(stageId));
        REQUIRE_UNARY_FALSE(closeStage(stageId));
    }

    TEST_CASE("openStage | closeStage")
    {
        std::string usdPath =
            isaacsim::common::test::resolveResourcePath("isaacsim.foundation.usd.ovstage", "scene.usda");
        // Open the stage
        int64_t stageId = openStage(usdPath);
        REQUIRE_NE(stageId, -1);
        REQUIRE_UNARY(isStageValid(stageId));
        REQUIRE_UNARY(closeStage(stageId));
        // Check that the stage is not valid after closing it
        REQUIRE_UNARY_FALSE(isStageValid(stageId));
        REQUIRE_UNARY_FALSE(closeStage(stageId));
    }

    TEST_CASE("saveStage" * doctest::skip())
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/Cube", "Cube");
        const std::string savePath = "isaacsim_foundation_usd_ovstage_test.usda";

        SUBCASE("Saved stage content to a file and can be reopened")
        {
            CHECK_UNARY(saveStage(stageId, savePath));

            int64_t reopenedStageId = openStage(savePath);
            REQUIRE_NE(reopenedStageId, -1);
            CHECK_UNARY(isPrimValid(reopenedStageId, "/World/Cube"));
            closeStage(reopenedStageId);
            std::remove(savePath.c_str());
        }

        SUBCASE("Exceptions")
        {
            CHECK_THROWS_AS(saveStage(-1, savePath), std::invalid_argument);
            CHECK_THROWS_AS(saveStage(stageId, "test.json"), std::invalid_argument);
        }

        closeStage(stageId);
    }

    TEST_CASE("exportStageToString | importStageFromString")
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/Cube", "Cube");

        SUBCASE("Non-empty USDA content")
        {
            // std::string usdaString = exportStageToString(stageId);
            // CHECK_UNARY_FALSE(usdaString.empty());
            // CHECK_UNARY(usdaString.find("def Cube \"Cube\"") != std::string::npos);
        }

        SUBCASE("Round-trip stage content")
        {
            // std::string usdaString = exportStageToString(stageId);
            std::string usdaString =
                "#usda 1.0\n"
                "(\n"
                "upAxis = \"Z\"\n"
                ")\n\n"
                "def \"World\"\n"
                "{\n"
                "def Cube \"Cube\"\n"
                "{\n"
                "}\n"
                "}\n";
            int64_t newStageId = importStageFromString(usdaString);
            REQUIRE_NE(newStageId, -1);
            CHECK_UNARY(isStageValid(newStageId));
            CHECK_UNARY(isPrimValid(newStageId, "/World/Cube"));
            closeStage(newStageId);
        }

        SUBCASE("Invalid USDA")
        {
            CHECK_EQ(importStageFromString("This is not valid USDA"), -1);
        }

        SUBCASE("Exceptions")
        {
            // CHECK_THROWS_AS(exportStageToString(-1), std::invalid_argument);
        }

        closeStage(stageId);
    }

    TEST_CASE("getStagePtr")
    {
        int64_t stageId = createStage();
        CHECK_NE(getStagePtr(stageId), nullptr);
        closeStage(stageId);
        CHECK_EQ(getStagePtr(stageId), nullptr);
        CHECK_EQ(getStagePtr(-1), nullptr);
    }

    TEST_CASE("definePrim")
    {
        int64_t stageId = createStage();

        // Define prims of various types and verify path and type name
        const std::vector<std::string> typeNames = {
            "Camera",    "Capsule",      "Cone",      "Cube",      "Cylinder",    "Mesh",
            "Plane",     "Points",       "Scope",     "Sphere",    "Xform",       "CylinderLight",
            "DiskLight", "DistantLight", "DomeLight", "RectLight", "SphereLight", "PhysicsScene",
        };
        for (const auto& typeName : typeNames)
        {
            const std::string path = "/" + typeName;
            REQUIRE_NOTHROW(definePrim(stageId, path, typeName));
            CHECK_UNARY(isPrimValid(stageId, path));
            CHECK_EQ(getTypeName(stageId, path), typeName);
        }

        // Idempotent: re-defining with the same type does not throw
        REQUIRE_NOTHROW(definePrim(stageId, "/Sphere", "Sphere"));

        // Exceptions
        // - Non-absolute path
        CHECK_THROWS_AS(definePrim(stageId, "World", "Xform"), isaacsim::common::exceptions::PrimPathStringError);
        // - Non-valid path (trailing slash makes it invalid)
        CHECK_THROWS_AS(definePrim(stageId, "/World/", "Xform"), isaacsim::common::exceptions::PrimPathStringError);
        // - Prim already exists with a different type
        CHECK_THROWS_AS(definePrim(stageId, "/Sphere", "Cube"), std::runtime_error);

        closeStage(stageId);
    }

    TEST_CASE("Foundation writes continue after an external writer advances the OVStage floor")
    {
        const int64_t stageId = createStage();
        definePrim(stageId, "/Before", "Xform");

        auto* stage = static_cast<ovstage_instance_t*>(getStagePtr(stageId));
        REQUIRE(stage != nullptr);
        ovstage_write_floor_desc_t description{};
        description.ordinal = 50;
        description.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t enqueue = ovstage_advance_write_floor(stage, &description);
        REQUIRE(enqueue.status == OVSTAGE_OK);
        REQUIRE(ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr) == OVSTAGE_OK);
        ovstage_release_op(stage, enqueue.op_index);

        definePrim(stageId, "/After", "Xform");
        CHECK_UNARY(isPrimValid(stageId, "/After"));
        closeStage(stageId);
    }

    TEST_CASE("traversePrim")
    {
        int64_t stageId = createStage();

        std::vector<std::string> visited;
        definePrim(stageId, "/World/A", "");
        definePrim(stageId, "/World/A/B", "");
        definePrim(stageId, "/World/A/B/C", "");

        SUBCASE("traversePrim (from root prim)")
        {
            traversePrim(stageId, "/",
                         [&](const std::string& path)
                         {
                             visited.push_back(path);
                             return true;
                         });
            CHECK_EQ(visited, (std::vector<std::string>{ "/", "/World", "/World/A", "/World/A/B", "/World/A/B/C" }));
        }

        SUBCASE("traversePrim")
        {
            traversePrim(stageId, "/World/A",
                         [&](const std::string& path)
                         {
                             visited.push_back(path);
                             return true;
                         });
            CHECK_EQ(visited, (std::vector<std::string>{ "/World/A", "/World/A/B", "/World/A/B/C" }));
        }

        SUBCASE("traversePrim: early exit")
        {
            traversePrim(stageId, "/World/A",
                         [&](const std::string& path)
                         {
                             visited.push_back(path);
                             return false;
                         });
            CHECK_EQ(visited, std::vector<std::string>{ "/World/A" });
        }

        closeStage(stageId);
    }

    TEST_CASE("getStageUnits | setStageUnits" * doctest::skip())
    {
        int64_t stageId = createStage();

        SUBCASE("getStageUnits")
        {
            auto [metersPerUnit, kilogramsPerUnit] = getStageUnits(stageId);
            CHECK_EQ(metersPerUnit, 1.0f);
            CHECK_EQ(kilogramsPerUnit, 1.0f);
        }

        SUBCASE("setStageUnits(metersPerUnit)")
        {
            setStageUnits(stageId, 0.01f, std::nullopt);
            auto [metersPerUnit, kilogramsPerUnit] = getStageUnits(stageId);
            CHECK_EQ(doctest::Approx(metersPerUnit), 0.01f);
            CHECK_EQ(doctest::Approx(kilogramsPerUnit), 1.0f);
        }

        SUBCASE("setStageUnits(kilogramsPerUnit)")
        {
            setStageUnits(stageId, std::nullopt, 0.001f);
            auto [metersPerUnit, kilogramsPerUnit] = getStageUnits(stageId);
            CHECK_EQ(doctest::Approx(metersPerUnit), 1.0f);
            CHECK_EQ(doctest::Approx(kilogramsPerUnit), 0.001f);
        }

        SUBCASE("setStageUnits(metersPerUnit, kilogramsPerUnit)")
        {
            setStageUnits(stageId, 0.0254f, 0.4536f);
            auto [metersPerUnit, kilogramsPerUnit] = getStageUnits(stageId);
            CHECK_EQ(doctest::Approx(metersPerUnit), 0.0254f);
            CHECK_EQ(doctest::Approx(kilogramsPerUnit), 0.4536f);
        }

        closeStage(stageId);
    }

    TEST_CASE("getStageUpAxis | setStageUpAxis" * doctest::skip())
    {
        int64_t stageId = createStage();

        SUBCASE("getStageUpAxis")
        {
            CHECK_EQ(getStageUpAxis(stageId), "Z");
        }

        SUBCASE("setStageUpAxis")
        {
            for (const auto& upAxis : std::vector<std::string>{ "Y", "Z", "y", "z" })
            {
                REQUIRE_NOTHROW(setStageUpAxis(stageId, upAxis));
                std::string result = getStageUpAxis(stageId);
                std::string expectedUpAxis = upAxis;
                std::transform(expectedUpAxis.begin(), expectedUpAxis.end(), expectedUpAxis.begin(),
                               [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
                CHECK_EQ(result, expectedUpAxis);
            }
        }

        SUBCASE("setStageUpAxis: invalid up axis")
        {
            CHECK_THROWS_AS(setStageUpAxis(stageId, "X"), std::invalid_argument);
            CHECK_THROWS_AS(setStageUpAxis(stageId, ""), std::invalid_argument);
        }

        closeStage(stageId);
    }

    TEST_CASE("getStageTimeCode | setStageTimeCode" * doctest::skip())
    {
        int64_t stageId = createStage();

        SUBCASE("getStageTimeCode")
        {
            auto [startTimeCode, endTimeCode, timeCodesPerSecond] = getStageTimeCode(stageId);
            CHECK_EQ(startTimeCode, 0.0f);
            CHECK_EQ(endTimeCode, 1000000.0f);
            CHECK_EQ(timeCodesPerSecond, 60.0f);
        }

        SUBCASE("setStageTimeCode(startTimeCode)")
        {
            setStageTimeCode(stageId, 10.0f, std::nullopt, std::nullopt);
            auto [startTimeCode, endTimeCode, timeCodesPerSecond] = getStageTimeCode(stageId);
            CHECK_EQ(startTimeCode, 10.0f);
            CHECK_EQ(endTimeCode, 1000000.0f);
            CHECK_EQ(timeCodesPerSecond, 60.0f);
        }

        SUBCASE("setStageTimeCode(endTimeCode)")
        {
            setStageTimeCode(stageId, std::nullopt, 200.0f, std::nullopt);
            auto [startTimeCode, endTimeCode, timeCodesPerSecond] = getStageTimeCode(stageId);
            CHECK_EQ(startTimeCode, 0.0f);
            CHECK_EQ(endTimeCode, 200.0f);
            CHECK_EQ(timeCodesPerSecond, 60.0f);
        }

        SUBCASE("setStageTimeCode(timeCodesPerSecond)")
        {
            setStageTimeCode(stageId, std::nullopt, std::nullopt, 30.0f);
            auto [startTimeCode, endTimeCode, timeCodesPerSecond] = getStageTimeCode(stageId);
            CHECK_EQ(startTimeCode, 0.0f);
            CHECK_EQ(endTimeCode, 1000000.0f);
            CHECK_EQ(timeCodesPerSecond, 30.0f);
        }

        SUBCASE("setStageTimeCode(startTimeCode, endTimeCode, timeCodesPerSecond)")
        {
            setStageTimeCode(stageId, 20.0f, 300.0f, 40.0f);
            auto [startTimeCode, endTimeCode, timeCodesPerSecond] = getStageTimeCode(stageId);
            CHECK_EQ(startTimeCode, 20.0f);
            CHECK_EQ(endTimeCode, 300.0f);
            CHECK_EQ(timeCodesPerSecond, 40.0f);
        }

        closeStage(stageId);
    }

    TEST_CASE("removePrim")
    {
        int64_t stageId = createStage();

        SUBCASE("Delete a locally defined prim")
        {
            definePrim(stageId, "/World/A", "Xform");
            CHECK_UNARY(removePrim(stageId, "/World/A"));
            CHECK_UNARY_FALSE(isPrimValid(stageId, "/World/A"));
        }

        SUBCASE("Delete an ancestral prim (defined inside the reference)")
        {
            // std::string usdPath = isaacsim::common::test::resolveResourcePath("isaacsim.foundation.usd.ovstage",
            // "variant.usda"); addReferenceToStage(stageId, "/World/Reference", usdPath);
            // CHECK_UNARY_FALSE(removePrim(stageId, "/World/Reference/Cube"));
            // CHECK_UNARY(removePrim(stageId, "/World/Reference"));
            // CHECK_UNARY_FALSE(isPrimValid(stageId, "/World/Reference"));
        }

        SUBCASE("Delete non-existent prim")
        {
            CHECK_THROWS_AS(removePrim(stageId, "/NonExistent"), isaacsim::common::exceptions::PrimPathError);
        }

        closeStage(stageId);
    }

    TEST_CASE("movePrim" * doctest::skip())
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/A", "Xform");
        definePrim(stageId, "/World/B", "Xform");

        SUBCASE("movePrim")
        {
            std::tuple<bool, std::string> result;

            // Move A into B
            result = movePrim(stageId, "/World/A", "/World/B");
            CHECK_UNARY(std::get<0>(result));
            CHECK_EQ(std::get<1>(result), "/World/B/A");
            CHECK_UNARY(isPrimValid(stageId, std::get<1>(result)));
            CHECK_UNARY_FALSE(isPrimValid(stageId, "/World/A"));

            // Move A next to B
            result = movePrim(stageId, "/World/B/A", "/World");
            CHECK_UNARY(std::get<0>(result));
            CHECK_EQ(std::get<1>(result), "/World/A");
            CHECK_UNARY(isPrimValid(stageId, std::get<1>(result)));
            CHECK_UNARY_FALSE(isPrimValid(stageId, "/World/B/A"));

            // Move A to root
            result = movePrim(stageId, "/World/A", "/");
            CHECK_UNARY(std::get<0>(result));
            CHECK_EQ(std::get<1>(result), "/A");
            CHECK_UNARY(isPrimValid(stageId, std::get<1>(result)));
            CHECK_UNARY_FALSE(isPrimValid(stageId, "/World/A"));

            // Move A to an unexisting path
            result = movePrim(stageId, "/A", "/World/C");
            CHECK_UNARY(std::get<0>(result));
            CHECK_EQ(std::get<1>(result), "/World/C");
            CHECK_UNARY(isPrimValid(stageId, std::get<1>(result)));
            CHECK_UNARY_FALSE(isPrimValid(stageId, "/A"));
        }

        SUBCASE("Exceptions")
        {
            CHECK_THROWS_AS(movePrim(stageId, "/NonExistent", "/"), isaacsim::common::exceptions::PrimPathError);
            CHECK_THROWS_AS(movePrim(stageId, "/World/B", "?"), isaacsim::common::exceptions::PrimPathStringError);
            CHECK_THROWS_AS(movePrim(stageId, "/World/B", "/World/X/Y"), std::invalid_argument);
        }

        closeStage(stageId);
    }

    TEST_CASE("addReferenceToStage" * doctest::skip())
    {
        std::string usdPath =
            isaacsim::common::test::resolveResourcePath("isaacsim.foundation.usd.ovstage", "variant.usda");

        SUBCASE("Add reference")
        {
            int64_t stageId = createStage();
            REQUIRE_NOTHROW(addReferenceToStage(stageId, "/World/Reference", usdPath));
            CHECK_UNARY(isPrimValid(stageId, "/World/Reference"));
            closeStage(stageId);
        }

        SUBCASE("Invalid prim path")
        {
            int64_t stageId = createStage();
            CHECK_THROWS_AS(
                addReferenceToStage(stageId, "/World/", usdPath), isaacsim::common::exceptions::PrimPathStringError);
            closeStage(stageId);
        }

        SUBCASE("Non-existent USD file")
        {
            int64_t stageId = createStage();
            CHECK_THROWS_AS(addReferenceToStage(stageId, "/World/Reference", "/unknown/file.usda"), std::runtime_error);
            closeStage(stageId);
        }
    }

    TEST_CASE("findMatchingPrimPaths")
    {
        int64_t stageId = createStage();

        // Build the prim hierarchy for the test case:
        //   / ()
        //   ├─ World ()
        //   │  ├─ A (Xform)
        //   │  ├─ A0 (Xform)
        //   │  │  ├─ B (Xform)
        //   │  │  ├─ B0 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B1 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B2 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  ├─ A1 (Xform)
        //   │  │  ├─ B (Xform)
        //   │  │  ├─ B0 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B1 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B2 (Xform)
        //   │  │  │  ├─ C (Xform)
        definePrim(stageId, "/World/A", "");
        for (int i = 0; i < 2; ++i)
        {
            definePrim(stageId, "/World/A" + std::to_string(i), "");
            definePrim(stageId, "/World/A" + std::to_string(i) + "/B", "");
            for (int j = 0; j < 3; ++j)
            {
                definePrim(stageId, "/World/A" + std::to_string(i) + "/B" + std::to_string(j), "");
                definePrim(stageId, "/World/A" + std::to_string(i) + "/B" + std::to_string(j) + "/C", "");
            }
        }

        // Valid prim path
        CHECK_EQ(findMatchingPrimPaths(stageId, "/World/A0/B0"), std::vector<std::string>{ "/World/A0/B0" });
        CHECK_EQ(findMatchingPrimPaths(stageId, "/World/A0/B0", true),
                 (std::vector<std::string>{ "/World/A0/B0", "/World/A0/B0/C" }));
        // Regex
        // --
        CHECK_EQ(findMatchingPrimPaths(stageId, "/World/A0/B[0-9]"),
                 (std::vector<std::string>{ "/World/A0/B0", "/World/A0/B1", "/World/A0/B2" }));
        CHECK_EQ(findMatchingPrimPaths(stageId, "/World/A0/B[0-9]", true), (std::vector<std::string>{
                                                                               "/World/A0/B0",
                                                                               "/World/A0/B0/C",
                                                                               "/World/A0/B1",
                                                                               "/World/A0/B1/C",
                                                                               "/World/A0/B2",
                                                                               "/World/A0/B2/C",
                                                                           }));
        // --
        CHECK_EQ(findMatchingPrimPaths(stageId, "/World/.*/.*[1,2]"),
                 (std::vector<std::string>{ "/World/A0/B1", "/World/A0/B2", "/World/A1/B1", "/World/A1/B2" }));
        CHECK_EQ(findMatchingPrimPaths(stageId, "/World/.*/.*[1,2]", true), (std::vector<std::string>{
                                                                                "/World/A0/B1",
                                                                                "/World/A0/B1/C",
                                                                                "/World/A0/B2",
                                                                                "/World/A0/B2/C",
                                                                                "/World/A1/B1",
                                                                                "/World/A1/B1/C",
                                                                                "/World/A1/B2",
                                                                                "/World/A1/B2/C",
                                                                            }));
        // --
        CHECK_EQ(findMatchingPrimPaths(stageId, ".*C.*"), std::vector<std::string>{});
        CHECK_EQ(findMatchingPrimPaths(stageId, ".*C.*", true), (std::vector<std::string>{
                                                                    "/World/A0/B0/C",
                                                                    "/World/A0/B1/C",
                                                                    "/World/A0/B2/C",
                                                                    "/World/A1/B0/C",
                                                                    "/World/A1/B1/C",
                                                                    "/World/A1/B2/C",
                                                                }));

        closeStage(stageId);
    }
}
