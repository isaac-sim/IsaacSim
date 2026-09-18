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

using namespace isaacsim::common::exceptions;

TEST_SUITE("Exceptions")
{

    TEST_CASE("PrimPathError")
    {
        PrimPathError error("/World/Prim");
        CHECK_EQ(error.getPrimPath(), "/World/Prim");
        CHECK_EQ(std::string(error.what()), "Invalid prim at path: '/World/Prim'");
    }

    TEST_CASE("PrimPathStringError")
    {
        PrimPathStringError error("not a path");
        CHECK_EQ(error.getPrimPath(), "not a path");
        CHECK_EQ(std::string(error.what()), "Invalid prim path string: 'not a path'");
    }

    TEST_CASE("AttributeNameError")
    {
        SUBCASE("Without valid attribute names")
        {
            AttributeNameError error("attribute");
            CHECK_EQ(error.getAttributeName(), "attribute");
            CHECK(error.getValidAttributeNames().empty());
            CHECK_EQ(std::string(error.what()), "Invalid attribute name: 'attribute'");
        }

        SUBCASE("With valid attribute names")
        {
            AttributeNameError error("attribute", std::vector<std::string>{ "scale", "translate" });
            CHECK_EQ(error.getAttributeName(), "attribute");
            CHECK_EQ(error.getValidAttributeNames(), std::vector<std::string>{ "scale", "translate" });
            CHECK_EQ(std::string(error.what()),
                     "Invalid attribute name: 'attribute'. Valid attribute names: 'scale', 'translate'");
        }
    }

    TEST_CASE("ValueTypeError")
    {
        ValueTypeError error("scale", "float", "str");
        CHECK_EQ(error.getAttributeName(), "scale");
        CHECK_EQ(error.getExpectedType(), "float");
        CHECK_EQ(error.getActualType(), "str");
        CHECK_EQ(std::string(error.what()), "Invalid value type for attribute 'scale': expected float, got str");
    }

    TEST_CASE("CudaRuntimeError")
    {
        SUBCASE("For a failed CUDA call")
        {
            CudaRuntimeError error("cudaMalloc", "out of memory", 2);
            CHECK_EQ(error.getCallerName(), "cudaMalloc");
            CHECK_EQ(error.getErrorCode(), std::optional<int>(2));
            CHECK_EQ(std::string(error.what()), "cudaMalloc: out of memory (error code: 2)");
        }

        SUBCASE("For a CUDA runtime call attempted while the CUDA runtime library is unavailable")
        {
            CudaRuntimeError error("cudaMalloc", "the CUDA runtime was not loaded");
            CHECK_EQ(error.getCallerName(), "cudaMalloc");
            CHECK_FALSE(error.getErrorCode().has_value());
            CHECK_EQ(std::string(error.what()), "cudaMalloc: the CUDA runtime was not loaded");
        }
    }

    TEST_CASE("Every exception is catchable through the common IsaacSimException base")
    {
        CHECK_THROWS_AS(throw PrimPathError("/World/Prim"), const IsaacSimException&);
        CHECK_THROWS_AS(throw PrimPathStringError("not a path"), const IsaacSimException&);
        CHECK_THROWS_AS(throw AttributeNameError("attribute"), const IsaacSimException&);
        CHECK_THROWS_AS(throw ValueTypeError("argument", "int", "str"), const IsaacSimException&);
        CHECK_THROWS_AS(throw CudaRuntimeError("cudaMalloc", "out of memory", 2), const IsaacSimException&);
    }

    TEST_CASE("Every exception is catchable through std::exception")
    {
        CHECK_THROWS_AS(throw PrimPathError("/World/Prim"), const std::exception&);
        CHECK_THROWS_AS(throw PrimPathStringError("not a path"), const std::exception&);
        CHECK_THROWS_AS(throw AttributeNameError("attribute"), const std::exception&);
        CHECK_THROWS_AS(throw ValueTypeError("argument", "int", "str"), const std::exception&);
        CHECK_THROWS_AS(throw CudaRuntimeError("cudaMalloc", "out of memory", 2), const std::exception&);
    }
}
