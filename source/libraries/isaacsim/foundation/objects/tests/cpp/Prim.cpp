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
#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/objects/Stage.hpp>

#include <IsaacSimTest.hpp>
#include <algorithm>
#include <stdexcept>

using namespace isaacsim::foundation::objects;

TEST_SUITE("Prim")
{

    TEST_CASE("Prim::getStage")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        CHECK_EQ(Prim({}, false).getStage().getStageId(), stage.getStageId());

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::resolvePaths")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        Prim prim({}, false);
        for (int i = 0; i < 3; ++i)
        {
            stage.definePrim("/World/A_" + std::to_string(i), "Xform");
            stage.definePrim("/World/A_" + std::to_string(i) + "/B", "Cube");
        }

        SUBCASE("resolvePaths")
        {
            std::tuple<std::vector<std::string>, std::vector<std::string>> result;
            // Single path
            result = prim.resolvePaths("/World/A_0");
            CHECK_EQ(std::get<0>(result).size(), 1u);
            CHECK_EQ(std::get<1>(result).size(), 0u);
            // Single path (non-existing)
            result = prim.resolvePaths("/World/C");
            CHECK_EQ(std::get<0>(result).size(), 0u);
            CHECK_EQ(std::get<1>(result).size(), 1u);
            // Regex
            result = prim.resolvePaths("/World/A_.*/B");
            CHECK_EQ(std::get<0>(result).size(), 3u);
            CHECK_EQ(std::get<1>(result).size(), 0u);
        }

        SUBCASE("Exceptions")
        {
            // Mixed paths
            CHECK_THROWS_AS(prim.resolvePaths(std::vector<std::string>{ "/World/A_.*", "/World/C" }), std::runtime_error);
            // Non-existing or non-existing paths exist
            CHECK_THROWS_AS(prim.resolvePaths("/World/A_.*/C"), std::runtime_error);
            // Incomplete existing paths
            CHECK_THROWS_AS(
                prim.resolvePaths(std::vector<std::string>{ "/World/A_.*/B", "/World/A_.*/C" }), std::runtime_error);
            // Incomplete non-existing paths
            CHECK_THROWS_AS(
                prim.resolvePaths(std::vector<std::string>{ "/World/C", "/World/A_.*/C" }), std::runtime_error);
        }

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::getName")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/A/B");

        CHECK_EQ(Prim("/World/A/B").getName(), (std::vector<std::string>{ "B" }));
        CHECK_EQ(Prim(std::vector<std::string>{ "/World/A/B", "/World/A" }).getName(),
                 (std::vector<std::string>{ "B", "A" }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::getTypeName")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/Cube", "Cube");
        stage.definePrim("/World/Xform", "Xform");

        CHECK_EQ(Prim("/World/Cube").getTypeName(), (std::vector<std::string>{ "Cube" }));
        CHECK_EQ(Prim(std::vector<std::string>{ "/World/Cube", "/World/Xform" }).getTypeName(),
                 (std::vector<std::string>{ "Cube", "Xform" }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::isA")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/Xform", "Xform");
        stage.definePrim("/World/Cube", "Cube");

        // Exact type
        CHECK_EQ(Prim("/World/Xform").isA("Xform").flatten().get<std::vector<bool>>(), (std::vector<bool>{ true }));
        CHECK_EQ(Prim("/World/Cube").isA("Cube").flatten().get<std::vector<bool>>(), (std::vector<bool>{ true }));
        // A Cube is not an Xform (both derive from Xformable, but Cube does not derive from Xform)
        CHECK_EQ(Prim("/World/Cube").isA("Xform").flatten().get<std::vector<bool>>(), (std::vector<bool>{ false }));
        // Base schema: both are Xformable (demonstrates IsA inheritance vs exact type name)
        CHECK_EQ(Prim(std::vector<std::string>{ "/World/Xform", "/World/Cube" })
                     .isA("Xformable")
                     .flatten()
                     .get<std::vector<bool>>(),
                 (std::vector<bool>{ true, true }));
        // Multiple prims
        CHECK_EQ(
            Prim(std::vector<std::string>{ "/World/Xform", "/World/Cube" }).isA("Xform").flatten().get<std::vector<bool>>(),
            (std::vector<bool>{ true, false }));
        // Unknown schema type
        CHECK_THROWS_AS(Prim("/World/Xform").isA("NonExistentType"), std::invalid_argument);
        // Flags are reported one row per prim
        CHECK_EQ(Prim(std::vector<std::string>{ "/World/Xform", "/World/Cube" }).isA("Xform").shape(),
                 array::Shape({ 2, 1 }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::createAttribute/Prim::removeAttribute")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/Xform", "Xform");
        stage.definePrim("/World/Cube", "Cube");
        Prim prims(std::vector<std::string>{ "/World/Xform", "/World/Cube" });

        // Create
        const array::Array created = prims.createAttribute("custom:value", "float");
        CHECK_EQ(created.shape(), array::Shape({ 2, 1 }));
        CHECK_EQ(created.flatten().get<std::vector<bool>>(), (std::vector<bool>{ true, true }));
        // Remove
        const array::Array removed = prims.removeAttribute("custom:value");
        CHECK_EQ(removed.shape(), array::Shape({ 2, 1 }));
        CHECK_EQ(removed.flatten().get<std::vector<bool>>(), (std::vector<bool>{ true, true }));
        // Removing an attribute that is no longer authored is an error
        CHECK_THROWS(prims.removeAttribute("custom:value"));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::getParent")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/A/B/C");

        CHECK_EQ(Prim("/World/A/B/C").getParent(), (std::vector<std::string>{ "/World/A/B" }));
        CHECK_EQ(Prim(std::vector<std::string>{ "/World/A/B/C", "/World/A", "/" }).getParent(),
                 (std::vector<std::string>{ "/World/A/B", "/World", "" }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::getChildren")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/A/X");
        stage.definePrim("/World/A/Y");
        stage.definePrim("/World/B");

        CHECK_EQ(
            Prim("/World/A").getChildren(), (std::vector<std::vector<std::string>>{ { "/World/A/X", "/World/A/Y" } }));
        CHECK_EQ(Prim(std::vector<std::string>{ "/World", "/World/B" }).getChildren(),
                 (std::vector<std::vector<std::string>>{ { "/World/A", "/World/B" }, {} }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::getVariantSets|getVariantSelection|setVariantSelection")
    {
        std::string usdPath = isaacsim::common::test::resolveResourcePath("isaacsim.foundation.objects", "variant.usda");
        Stage stage = Stage("openusd").openStage(usdPath);
        REQUIRE_UNARY(stage.isValid());

        SUBCASE("getVariantSets")
        {
            std::vector<std::unordered_map<std::string, std::vector<std::string>>> result;

            result = Prim("/World/Sphere").getVariantSets();
            REQUIRE_EQ(result.size(), 1u);
            REQUIRE_UNARY(result.at(0).count("color"));
            REQUIRE_UNARY(result.at(0).count("radius"));
            CHECK_EQ(result.at(0).at("color"), (std::vector<std::string>{ "blue", "red" }));
            CHECK_EQ(result.at(0).at("radius"), (std::vector<std::string>{ "large", "small" }));

            result = Prim(std::vector<std::string>{ "/World/Sphere", "/World/Sphere" }).getVariantSets();
            REQUIRE_EQ(result.size(), 2u);
            for (const auto& entry : result)
            {
                REQUIRE_UNARY(entry.count("color"));
                REQUIRE_UNARY(entry.count("radius"));
                CHECK_EQ(entry.at("color"), (std::vector<std::string>{ "blue", "red" }));
                CHECK_EQ(entry.at("radius"), (std::vector<std::string>{ "large", "small" }));
            }
        }

        SUBCASE("getVariantSelection")
        {
            std::vector<std::unordered_map<std::string, std::string>> result;

            result = Prim("/World/Sphere").getVariantSelection();
            REQUIRE_EQ(result.size(), 1u);
            REQUIRE_UNARY(result[0].count("color"));
            REQUIRE_UNARY(result[0].count("radius"));
            CHECK_EQ(result[0].at("color"), "red");
            CHECK_EQ(result[0].at("radius"), "small");

            result = Prim(std::vector<std::string>{ "/World/Sphere", "/World/Sphere" }).getVariantSelection();
            REQUIRE_EQ(result.size(), 2u);
            for (const auto& entry : result)
            {
                CHECK_EQ(entry.at("color"), "red");
                CHECK_EQ(entry.at("radius"), "small");
            }
        }

        SUBCASE("setVariantSelection")
        {
            std::vector<std::unordered_map<std::string, std::string>> result;

            Prim prim("/World/Sphere");
            REQUIRE_NOTHROW(prim.setVariantSelection({ { "color", "blue" }, { "radius", "large" } }));
            result = prim.getVariantSelection();
            CHECK_EQ(result[0].at("color"), "blue");
            CHECK_EQ(result[0].at("radius"), "large");
        }

        SUBCASE("setVariantSelection: invalid variant set/selection")
        {
            Prim prim("/World/Sphere");
            CHECK_THROWS_AS(prim.setVariantSelection({ { "NonExistent", "red" } }), std::invalid_argument);
            CHECK_THROWS_AS(prim.setVariantSelection({ { "radius", "Invalid" } }), std::invalid_argument);
        }

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::getAppliedSchemas")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/Cube", "Cube");
        stage.definePrim("/World/Sphere", "Sphere");

        // No applied schemas
        CHECK_EQ(Prim("/World/Cube").getAppliedSchemas(), (std::vector<std::vector<std::string>>{ {} }));

        // Apply schemas and get them
        Prim prim("/World/Cube");
        prim.applyApi("PhysicsRigidBodyAPI");
        prim.applyApi("PhysicsDriveAPI", "linear");
        prim.applyApi("PhysicsDriveAPI", "angular");

        std::vector<std::vector<std::string>> result;
        result = prim.getAppliedSchemas();
        REQUIRE_EQ(result.size(), 1u);
        CHECK_EQ(result[0].size(), 3u);
        CHECK_UNARY(std::find(result[0].begin(), result[0].end(), "PhysicsRigidBodyAPI") != result[0].end());
        CHECK_UNARY(std::find(result[0].begin(), result[0].end(), "PhysicsDriveAPI:linear") != result[0].end());
        CHECK_UNARY(std::find(result[0].begin(), result[0].end(), "PhysicsDriveAPI:angular") != result[0].end());

        // Multiple prims
        result = Prim(std::vector<std::string>{ "/World/Cube", "/World/Sphere" }).getAppliedSchemas();
        REQUIRE_EQ(result.size(), 2u);
        CHECK_EQ(result[0].size(), 3u);
        CHECK_EQ(result[1].size(), 0u);

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::getAttributeValues|setAttributeValues")
    {
        namespace array = isaacsim::common::array;

        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/A", "Xform");
        stage.definePrim("/World/B", "Xform");
        stage.definePrim("/World/C", "Xform");

        // Author distinct token values so index-to-path mapping is observable
        array::Array indices(0);
        Prim prims(std::vector<std::string>{ "/World/A", "/World/B", "/World/C" });
        prims.setAttributeValues("purpose", std::vector<std::string>{ "render", "proxy", "guide" });

        auto asStrings = [](const OutputValueType& value) { return std::get<std::vector<std::string>>(value); };

        SUBCASE("getAttributeValues")
        {
            // No indices
            CHECK_EQ(asStrings(prims.getAttributeValues("purpose")),
                     (std::vector<std::string>{ "render", "proxy", "guide" }));
            // Indices: subset in requested order
            indices = array::Array(std::vector<int64_t>{ 2, 0, 1 });
            CHECK_EQ(asStrings(prims.getAttributeValues("purpose", indices)),
                     (std::vector<std::string>{ "guide", "render", "proxy" }));
            // Negative indices: Python-style from the end
            indices = array::Array(std::vector<int64_t>{ -1, -2, -3 });
            CHECK_EQ(asStrings(prims.getAttributeValues("purpose", indices)),
                     (std::vector<std::string>{ "guide", "proxy", "render" }));
        }

        SUBCASE("out-of-range index throws")
        {
            CHECK_THROWS_AS(
                prims.getAttributeValues("purpose", array::Array(std::vector<int64_t>{ 3 })), std::out_of_range);
            CHECK_THROWS_AS(
                prims.getAttributeValues("purpose", array::Array(std::vector<int64_t>{ -4 })), std::out_of_range);
        }

        SUBCASE("setAttributeValues honors indices")
        {
            array::Array indices(std::vector<int64_t>{ -1 });
            prims.setAttributeValues("purpose", std::vector<std::string>{ "default" }, indices);
            CHECK_EQ(asStrings(prims.getAttributeValues("purpose")),
                     (std::vector<std::string>{ "render", "proxy", "default" }));

            CHECK_THROWS_AS(prims.setAttributeValues("purpose", std::vector<std::string>{ "default" },
                                                     array::Array(std::vector<int64_t>{ 3 })),
                            std::out_of_range);
        }

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("Prim::hasApi|applyApi|removeApi")
    {
        Stage stage = Stage("openusd").createStage();
        REQUIRE_UNARY(stage.isValid());

        stage.definePrim("/World/Cube", "Cube");
        stage.definePrim("/World/Sphere", "Sphere");

        SUBCASE("Single-apply API: PhysicsRigidBodyAPI")
        {
            Prim prim("/World/Cube");
            CHECK_EQ(prim.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(), (std::vector<bool>{ false }));
            CHECK_EQ(
                prim.applyApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(), (std::vector<bool>{ true }));
            CHECK_EQ(prim.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(), (std::vector<bool>{ true }));
            CHECK_EQ(
                prim.removeApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(), (std::vector<bool>{ true }));
            CHECK_EQ(prim.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(), (std::vector<bool>{ false }));

            Prim prims(std::vector<std::string>{ "/World/Cube", "/World/Sphere" });
            CHECK_EQ(prims.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false, false }));
            CHECK_EQ(prims.applyApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true, true }));
            CHECK_EQ(prims.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true, true }));
            CHECK_EQ(prims.removeApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true, true }));
            CHECK_EQ(prims.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false, false }));
        }

        SUBCASE("Multi-apply API: PhysicsDriveAPI")
        {
            Prim prim("/World/Cube");
            CHECK_EQ(prim.hasApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false }));
            CHECK_EQ(prim.applyApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true }));
            CHECK_EQ(prim.hasApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true }));
            CHECK_EQ(prim.hasApi("PhysicsDriveAPI", "angular").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false }));
            CHECK_EQ(prim.removeApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true }));
            CHECK_EQ(prim.hasApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false }));

            Prim prims(std::vector<std::string>{ "/World/Cube", "/World/Sphere" });
            CHECK_EQ(prims.hasApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false, false }));
            CHECK_EQ(prims.applyApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true, true }));
            CHECK_EQ(prims.hasApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true, true }));
            CHECK_EQ(prims.hasApi("PhysicsDriveAPI", "angular").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false, false }));
            CHECK_EQ(prims.removeApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ true, true }));
            CHECK_EQ(prims.hasApi("PhysicsDriveAPI", "linear").flatten().get<std::vector<bool>>(),
                     (std::vector<bool>{ false, false }));
        }

        SUBCASE("Unknown schema type")
        {
            Prim prim("/World/Cube");
            CHECK_THROWS_AS(prim.hasApi("NonExistentAPI"), std::invalid_argument);
            CHECK_THROWS_AS(prim.applyApi("NonExistentAPI"), std::invalid_argument);
            CHECK_THROWS_AS(prim.removeApi("NonExistentAPI"), std::invalid_argument);
        }

        REQUIRE_UNARY(stage.closeStage());
    }
}
