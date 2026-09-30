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
#include <isaacsim/common/array/Dtype.hpp>

#include <stdexcept>
#include <vector>

using namespace isaacsim::common::array;

TEST_SUITE("Dtype")
{

    TEST_CASE("Dtype::promoteDtypes()")
    {
        SUBCASE("Identical types promote to themselves")
        {
            auto dtype =
                GENERATE(Dtype::Bool(), Dtype::Int8(), Dtype::Int16(), Dtype::Int32(), Dtype::Int64(), Dtype::UInt8(),
                         Dtype::UInt16(), Dtype::UInt32(), Dtype::UInt64(), Dtype::Float32(), Dtype::Float64());
            CHECK_EQ(Dtype::promoteDtypes(dtype, dtype), dtype);
        }

        SUBCASE("Boolean yields to the other type")
        {
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Bool(), Dtype::Int8()), Dtype::Int8());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Bool(), Dtype::UInt64()), Dtype::UInt64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Bool(), Dtype::Float32()), Dtype::Float32());
        }

        SUBCASE("Same signedness widens to the wider type")
        {
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int8(), Dtype::Int32()), Dtype::Int32());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int16(), Dtype::Int64()), Dtype::Int64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::UInt8(), Dtype::UInt64()), Dtype::UInt64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Float32(), Dtype::Float64()), Dtype::Float64());
        }

        SUBCASE("Signed and unsigned widen to a signed type holding both ranges")
        {
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int8(), Dtype::UInt8()), Dtype::Int16());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int8(), Dtype::UInt16()), Dtype::Int32());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int16(), Dtype::UInt8()), Dtype::Int16());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int16(), Dtype::UInt16()), Dtype::Int32());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int32(), Dtype::UInt32()), Dtype::Int64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int64(), Dtype::UInt8()), Dtype::Int64());
        }

        SUBCASE("No signed type holding a 64-bit unsigned range: float64")
        {
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int8(), Dtype::UInt64()), Dtype::Float64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int16(), Dtype::UInt64()), Dtype::Float64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int32(), Dtype::UInt64()), Dtype::Float64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Int64(), Dtype::UInt64()), Dtype::Float64());
        }

        SUBCASE("Integer and float widen to the smallest exactly-representing float")
        {
            // float32 holds every 8- and 16-bit integer exactly, but not every 32-bit one.
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Float32(), Dtype::Int8()), Dtype::Float32());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Float32(), Dtype::UInt16()), Dtype::Float32());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Float32(), Dtype::Int32()), Dtype::Float64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Float32(), Dtype::UInt64()), Dtype::Float64());
            CHECK_EQ(Dtype::promoteDtypes(Dtype::Float64(), Dtype::Int8()), Dtype::Float64());
        }

        SUBCASE("Promotion is commutative")
        {
            const std::vector<Dtype> dtypes{ Dtype::Bool(),   Dtype::Int8(),    Dtype::Int16(),  Dtype::Int32(),
                                             Dtype::Int64(),  Dtype::UInt8(),   Dtype::UInt16(), Dtype::UInt32(),
                                             Dtype::UInt64(), Dtype::Float32(), Dtype::Float64() };
            for (const auto& a : dtypes)
            {
                for (const auto& b : dtypes)
                {
                    CHECK_EQ(Dtype::promoteDtypes(a, b), Dtype::promoteDtypes(b, a));
                }
            }
        }
    }

    TEST_CASE("Dtype::dispatchByKind()")
    {
        SUBCASE("Dispatches to the C++ type matching the kind")
        {
            auto dtype =
                GENERATE(Dtype::Bool(), Dtype::Int8(), Dtype::Int16(), Dtype::Int32(), Dtype::Int64(), Dtype::UInt8(),
                         Dtype::UInt16(), Dtype::UInt32(), Dtype::UInt64(), Dtype::Float32(), Dtype::Float64());

            const size_t size = Dtype::dispatchByKind(dtype.kind(),
                                                      [](auto sample) -> size_t
                                                      {
                                                          using T = decltype(sample);
                                                          return sizeof(T);
                                                      });
            CHECK_EQ(size, dtype.size());

            const Dtype roundTripDtype = Dtype::dispatchByKind(dtype.kind(),
                                                               [](auto sample) -> Dtype
                                                               {
                                                                   using T = decltype(sample);
                                                                   return Dtype::fromType<T>();
                                                               });
            CHECK_EQ(roundTripDtype, dtype);
        }

        SUBCASE("Returns whatever the callable returns, including void")
        {
            bool invoked = false;
            Dtype::dispatchByKind(Dtype::Kind::eFloat32, [&](auto) { invoked = true; });
            CHECK_UNARY(invoked);
        }

        SUBCASE("An out-of-range kind throws")
        {
            const auto invalidKind = static_cast<Dtype::Kind>(static_cast<int32_t>(Dtype::Kind::eFloat64) + 1);
            CHECK_THROWS_AS(Dtype::dispatchByKind(invalidKind, [](auto) {}), std::logic_error);
        }
    }
}
