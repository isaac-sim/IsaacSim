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
#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <isaacsim/foundation/usd/ovstage/Usd.hpp>

#include <IsaacSimTest.hpp>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

using namespace isaacsim::foundation::usd::ovstage;
namespace array = isaacsim::common::array;

TEST_SUITE("Attribute")
{

    TEST_CASE("createPrimAttribute | removePrimAttribute")
    {
        int64_t stageId = createStage();
        definePrim(stageId, "/World/A", "Xform");

        SUBCASE("createPrimAttribute")
        {
            CHECK_UNARY(createPrimAttribute(stageId, "/World/A", "attr1", "float"));
            // Create attribute again, should be idempotent
            CHECK_UNARY(createPrimAttribute(stageId, "/World/A", "attr1", "float"));
            // Type conflict
            CHECK_THROWS_AS(createPrimAttribute(stageId, "/World/A", "attr1", "double"), std::invalid_argument);
            // Unknown type name
            CHECK_THROWS_AS(createPrimAttribute(stageId, "/World/A", "attr1", "nonExistentType"), std::invalid_argument);
        }

        SUBCASE("removePrimAttribute")
        {
            createPrimAttribute(stageId, "/World/A", "attr2", "float");
            CHECK_UNARY(removePrimAttribute(stageId, "/World/A", "attr2"));
            // Attribute is gone after removal (non-existent attribute)
            CHECK_THROWS_AS(
                removePrimAttribute(stageId, "/World/A", "attr2"), isaacsim::common::exceptions::AttributeNameError);
        }

        SUBCASE("Invalid prim path")
        {
            CHECK_THROWS_AS(createPrimAttribute(stageId, "/NonExistent", "attr3", "float"),
                            isaacsim::common::exceptions::PrimPathError);
            CHECK_THROWS_AS(
                removePrimAttribute(stageId, "/NonExistent", "attr3"), isaacsim::common::exceptions::PrimPathError);
        }

        closeStage(stageId);
    }

    TEST_CASE("getPrimAttributeTypeName" * doctest::skip())
    {
        std::string usdPath =
            isaacsim::common::test::resolveResourcePath("isaacsim.foundation.usd.ovstage", "attributes.usda");
        int64_t stageId = openStage(usdPath);

        // Non-array types
        CHECK_EQ(getPrimAttributeTypeName(stageId, "/Prim", "attribute:Bool"), "bool");
        CHECK_EQ(getPrimAttributeTypeName(stageId, "/Prim", "attribute:Token"), "token");
        CHECK_EQ(getPrimAttributeTypeName(stageId, "/Prim", "attribute:String"), "string");
        CHECK_EQ(getPrimAttributeTypeName(stageId, "/Prim", "attribute:Double3"), "double3");
        CHECK_EQ(getPrimAttributeTypeName(stageId, "/Prim", "attribute:Matrix4d"), "matrix4d");

        // Array types
        CHECK_EQ(getPrimAttributeTypeName(stageId, "/Prim", "attribute:FloatArray"), "float[]");
        CHECK_EQ(getPrimAttributeTypeName(stageId, "/Prim", "attribute:StringArray"), "string[]");

        // Non-existent attribute
        CHECK_THROWS_AS(getPrimAttributeTypeName(stageId, "/Prim", "nonExistent"),
                        isaacsim::common::exceptions::AttributeNameError);

        // Non-existent prim
        CHECK_THROWS_AS(getPrimAttributeTypeName(stageId, "/NonExistent", "attribute:Float"),
                        isaacsim::common::exceptions::PrimPathError);

        closeStage(stageId);
    }

    TEST_CASE("getPrimAttributeNames" * doctest::skip())
    {
        std::string usdPath =
            isaacsim::common::test::resolveResourcePath("isaacsim.foundation.usd.ovstage", "attributes.usda");
        int64_t stageId = openStage(usdPath);

        const auto names = getPrimAttributeNames(stageId, "/Prim");

        // attributes.usda defines 106 attributes on /Prim
        CHECK_EQ(names.size(), 106);

        // Spot-check a representative sample of types across all attribute types
        const auto contains = [&](const std::string& name)
        { return std::find(names.begin(), names.end(), name) != names.end(); };
        CHECK_UNARY(contains("attribute:Bool"));
        CHECK_UNARY(contains("attribute:Token"));
        CHECK_UNARY(contains("attribute:String"));
        CHECK_UNARY(contains("attribute:Double3"));
        CHECK_UNARY(contains("attribute:Matrix4d"));
        CHECK_UNARY(contains("attribute:FloatArray"));
        CHECK_UNARY(contains("attribute:StringArray"));

        // Non-existent prim
        CHECK_THROWS_AS(getPrimAttributeNames(stageId, "/NonExistent"), isaacsim::common::exceptions::PrimPathError);

        closeStage(stageId);
    }

    TEST_CASE("getPrimAttributeValues | setPrimAttributeValues")
    {
        int64_t stageId = createStage();

        for (const auto& primPath : { "/World/A", "/World/B" })
        {
            definePrim(stageId, primPath, "Xform");
            createPrimAttribute(stageId, primPath, "attribute:String", "string");
            createPrimAttribute(stageId, primPath, "attribute:Float", "float");
            createPrimAttribute(stageId, primPath, "attribute:Normal3dArray", "normal3d[]");
        }

        SUBCASE("String attribute")
        {
            // Single prim
            setPrimAttributeValues(stageId, { "/World/A" }, "attribute:String", std::vector<std::string>{ "hello" });
            auto result1 = getPrimAttributeValues(stageId, { "/World/A" }, "attribute:String");
            CHECK_EQ(std::get<std::vector<std::string>>(result1), std::vector<std::string>{ "hello" });
            // Multiple prims
            setPrimAttributeValues(
                stageId, { "/World/A", "/World/B" }, "attribute:String", std::vector<std::string>{ "foo", "bar" });
            auto result2 = getPrimAttributeValues(stageId, { "/World/A", "/World/B" }, "attribute:String");
            CHECK_EQ(std::get<std::vector<std::string>>(result2), std::vector<std::string>{ "foo", "bar" });
        }

        SUBCASE("Float attribute")
        {
            // Single prim: shape {1} - one scalar per prim; at(0) yields 0-dim scalar for writeNumericScalar
            setPrimAttributeValues(stageId, { "/World/A" }, "attribute:Float", array::Array(std::vector<float>{ 1.0f }));
            auto result1 = getPrimAttributeValues(stageId, { "/World/A" }, "attribute:Float");
            // Returned Array has shape {1, 1}: 1 prim × 1 scalar
            CHECK_EQ(std::get<array::Array>(result1).get<std::vector<std::vector<float>>>(),
                     (std::vector<std::vector<float>>{ { 1.0f } }));
            // Multiple prims: shape {2} - one scalar per prim
            setPrimAttributeValues(
                stageId, { "/World/A", "/World/B" }, "attribute:Float", array::Array(std::vector<float>{ 3.0f, 7.0f }));
            auto result2 = getPrimAttributeValues(stageId, { "/World/A", "/World/B" }, "attribute:Float");
            // Returned Array has shape {2, 1}: 2 prims × 1 scalar
            CHECK_EQ(std::get<array::Array>(result2).get<std::vector<std::vector<float>>>(),
                     (std::vector<std::vector<float>>{ { 3.0f }, { 7.0f } }));
        }

        SUBCASE("Normal3dArray attribute")
        {
            // 2 prims × 1 normal each: shape {2, 3}; at(i) yields shape {3} treated as 1 row of 3 doubles
            array::Array setVals = array::Array(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 } });
            auto setResult =
                setPrimAttributeValues(stageId, { "/World/A", "/World/B" }, "attribute:Normal3dArray", setVals);
            CHECK_EQ(setResult, (std::vector<bool>{ true, true }));

            auto result = getPrimAttributeValues(stageId, { "/World/A", "/World/B" }, "attribute:Normal3dArray");
            // Returned Array has shape {2, 1, 3}: 2 prims × 1 normal × 3 components; slice per prim to get 2D
            auto arr = std::get<array::Array>(result);
            CHECK_EQ(arr.at(0).get<std::vector<std::vector<double>>>(),
                     (std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 } }));
            CHECK_EQ(arr.at(1).get<std::vector<std::vector<double>>>(),
                     (std::vector<std::vector<double>>{ { 0.0, 0.0, 1.0 } }));
        }

        closeStage(stageId);
    }

    TEST_CASE("getPrimAttributeValues decodes OVStage 0.2 asset IDs")
    {
        const std::string usdPath =
            isaacsim::common::test::resolveResourcePath("isaacsim.foundation.usd.ovstage", "attributes.usda");
        const int64_t stageId = openStage(usdPath);

        CHECK_EQ(std::get<std::vector<std::string>>(getPrimAttributeValues(stageId, { "/Prim" }, "attribute:Asset")),
                 std::vector<std::string>{ "./variant.usda" });
        CHECK_EQ(std::get<std::vector<std::vector<std::string>>>(
                     getPrimAttributeValues(stageId, { "/Prim" }, "attribute:AssetArray")),
                 (std::vector<std::vector<std::string>>{ { "./variant.usda" } }));

        closeStage(stageId);
    }
}
