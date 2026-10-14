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
#include <isaacsim/common/array/Shape.hpp>

#include <limits>
#include <numeric>
#include <stdexcept>

using namespace isaacsim::common::array;

template <typename T>
void ScalarConstructorTest(int64_t dimension)
{
    Shape shape;
    shape = Shape(static_cast<T>(dimension));
    CHECK_EQ(shape.ndim(), 1);
    CHECK_EQ(shape.size(), dimension);
    CHECK_EQ(shape[0], dimension);
    CHECK_EQ(shape.shape(), std::vector<int64_t>{ dimension });
}

template <typename T>
void VectorConstructorTest(std::vector<T> dimensions)
{
    Shape shape;
    shape = Shape(dimensions);
    CHECK_EQ(shape.ndim(), dimensions.size());
    CHECK_EQ(shape.size(), std::accumulate(dimensions.begin(), dimensions.end(), size_t{ 1 }, std::multiplies<size_t>()));
    CHECK_EQ(shape.shape(), std::vector<int64_t>(dimensions.begin(), dimensions.end()));
}

template <typename T>
void ListInitializerConstructorTest(std::initializer_list<T> dimensions)
{
    Shape shape;
    shape = Shape(dimensions);
    CHECK_EQ(shape.ndim(), dimensions.size());
    CHECK_EQ(shape.size(), std::accumulate(dimensions.begin(), dimensions.end(), size_t{ 1 }, std::multiplies<size_t>()));
    CHECK_EQ(shape.shape(), std::vector<int64_t>(dimensions.begin(), dimensions.end()));
}

TEST_SUITE("Shape")
{

    TEST_CASE("Shape::Shape()")
    {
        Shape shape;

        SUBCASE("Default constructor yields 0-dim shape")
        {
            shape = Shape();
            CHECK_EQ(shape.ndim(), 0);
            CHECK_EQ(shape.size(), 1);
            CHECK_EQ(shape.shape(), std::vector<int64_t>{});
        }

        SUBCASE("Scalar constructor")
        {
            ScalarConstructorTest<int8_t>(1);
            ScalarConstructorTest<int16_t>(2);
            ScalarConstructorTest<int32_t>(3);
            ScalarConstructorTest<int64_t>(4);
            ScalarConstructorTest<uint8_t>(5);
            ScalarConstructorTest<uint16_t>(6);
            ScalarConstructorTest<uint32_t>(7);
            ScalarConstructorTest<uint64_t>(8);
        }

        SUBCASE("Vector constructor")
        {
            VectorConstructorTest<int8_t>(std::vector<int8_t>{ 1 });
            VectorConstructorTest<int16_t>(std::vector<int16_t>{ 1, 2 });
            VectorConstructorTest<int32_t>(std::vector<int32_t>{ 1, 2, 3 });
            VectorConstructorTest<int64_t>(std::vector<int64_t>{ 1, 2, 3, 4 });
            VectorConstructorTest<uint8_t>(std::vector<uint8_t>{ 1, 2, 3, 4, 5 });
            VectorConstructorTest<uint16_t>(std::vector<uint16_t>{ 1, 2, 3, 4, 5, 6 });
            VectorConstructorTest<uint32_t>(std::vector<uint32_t>{ 1, 2, 3, 4, 5, 6, 7 });
            VectorConstructorTest<uint64_t>(std::vector<uint64_t>{ 1, 2, 3, 4, 5, 6, 7, 8 });
        }

        SUBCASE("List initializer constructor")
        {
            ListInitializerConstructorTest<int8_t>({ 1 });
            ListInitializerConstructorTest<int16_t>({ 1, 2 });
            ListInitializerConstructorTest<int32_t>({ 1, 2, 3 });
            ListInitializerConstructorTest<int64_t>({ 1, 2, 3, 4 });
            ListInitializerConstructorTest<uint8_t>({ 1, 2, 3, 4, 5 });
            ListInitializerConstructorTest<uint16_t>({ 1, 2, 3, 4, 5, 6 });
            ListInitializerConstructorTest<uint32_t>({ 1, 2, 3, 4, 5, 6, 7 });
            ListInitializerConstructorTest<uint64_t>({ 1, 2, 3, 4, 5, 6, 7, 8 });
        }
    }

    TEST_CASE("Shape::size()")
    {
        SUBCASE("Dimensions multiply to the element count")
        {
            CHECK_EQ(Shape({ 2, 3, 4 }).size(), 24);
        }

        SUBCASE("A zero dimension yields an empty shape")
        {
            CHECK_EQ(Shape({ 8, 0, 4 }).size(), 0);
        }

        SUBCASE("A zero after an overflowing prefix is empty")
        {
            CHECK_EQ(Shape({ std::numeric_limits<int64_t>::max(), int64_t{ 3 }, int64_t{ 0 } }).size(), 0);
            CHECK_THROWS_AS(Shape({ int64_t{ 0 }, int64_t{ -1 } }).size(), std::invalid_argument);
        }

        SUBCASE("A negative dimension is rejected")
        {
            CHECK_THROWS_AS(Shape({ 2, -1, 4 }).size(), std::invalid_argument);
        }

        SUBCASE("A product that exceeds size_t is rejected")
        {
            CHECK_THROWS_AS(Shape({ std::numeric_limits<int64_t>::max(), int64_t{ 3 } }).size(), std::overflow_error);
        }

        SUBCASE("A product that exactly fits size_t is accepted")
        {
            const int64_t half = static_cast<int64_t>(std::numeric_limits<size_t>::max() / 2);
            CHECK_EQ(Shape({ half, int64_t{ 2 } }).size(), static_cast<size_t>(half) * 2);
        }
    }

    TEST_CASE("Shape::operator[]")
    {
        Shape shape({ 2, 3, 4 });
        // Positive index
        CHECK_EQ(shape[0], 2);
        CHECK_EQ(shape[1], 3);
        CHECK_EQ(shape[2], 4);
        // Negative index
        CHECK_EQ(shape[-1], 4);
        CHECK_EQ(shape[-2], 3);
        CHECK_EQ(shape[-3], 2);
    }

    TEST_CASE("Shape::operator==() | Shape::operator!=()")
    {
        SUBCASE("Equal shapes")
        {
            CHECK_EQ(Shape(), Shape());
            CHECK_EQ(Shape({ 5 }), Shape(uint16_t{ 5 }));
            CHECK_EQ(Shape({ 2, 3 }), Shape({ 2, 3 }));
        }

        SUBCASE("Unequal shapes")
        {
            CHECK_NE(Shape(), Shape({ 1 }));
            CHECK_NE(Shape({ 2, 3 }), Shape({ 3, 2 }));
            CHECK_NE(Shape({ 2, 3 }), Shape({ 2, 3, 1 }));
        }
    }

    TEST_CASE("Shape::canBroadcastTo()")
    {
        SUBCASE("Compatible broadcasts")
        {
            // Scalar (0-dim) broadcasts to anything
            CHECK_UNARY(Shape().canBroadcastTo(Shape()));
            CHECK_UNARY(Shape().canBroadcastTo(Shape({ 3 })));
            CHECK_UNARY(Shape().canBroadcastTo(Shape({ 2, 3 })));
            // Exact match
            CHECK_UNARY(Shape({ 3 }).canBroadcastTo(Shape({ 3 })));
            CHECK_UNARY(Shape({ 2, 3 }).canBroadcastTo(Shape({ 2, 3 })));
            // Dim-1 broadcasts along that axis
            CHECK_UNARY(Shape({ 1 }).canBroadcastTo(Shape({ 3 })));
            CHECK_UNARY(Shape({ 1, 1 }).canBroadcastTo(Shape({ 2, 3 })));
            CHECK_UNARY(Shape({ 1, 3 }).canBroadcastTo(Shape({ 2, 3 })));
            CHECK_UNARY(Shape({ 2, 1 }).canBroadcastTo(Shape({ 2, 3 })));
            // Fewer dims: missing leading dims are treated as 1
            CHECK_UNARY(Shape({ 1 }).canBroadcastTo(Shape({ 2, 3 })));
            CHECK_UNARY(Shape({ 3 }).canBroadcastTo(Shape({ 2, 3 })));
            CHECK_UNARY(Shape({ 3 }).canBroadcastTo(Shape({ 4, 2, 3 })));
            CHECK_UNARY(Shape({ 1, 3 }).canBroadcastTo(Shape({ 4, 2, 3 })));
            CHECK_UNARY(Shape({ 4, 1, 1 }).canBroadcastTo(Shape({ 4, 2, 3 })));
        }

        SUBCASE("Incompatible broadcasts")
        {
            // More dims than destination
            CHECK_UNARY_FALSE(Shape({ 2, 3 }).canBroadcastTo(Shape({ 3 })));
            CHECK_UNARY_FALSE(Shape({ 4, 2, 3 }).canBroadcastTo(Shape({ 2, 3 })));
            // Mismatched non-1 dimensions
            CHECK_UNARY_FALSE(Shape({ 2 }).canBroadcastTo(Shape({ 3 })));
            CHECK_UNARY_FALSE(Shape({ 2, 3 }).canBroadcastTo(Shape({ 2, 4 })));
            CHECK_UNARY_FALSE(Shape({ 3, 1 }).canBroadcastTo(Shape({ 2, 3 })));
        }
    }

    TEST_CASE("Shape::resolve()")
    {
        // Exact match: no -1, compatible size
        CHECK_EQ(Shape(6).resolve(Shape(6)), Shape(6));
        CHECK_EQ(Shape(6).resolve(Shape({ 2, 3 })), Shape({ 2, 3 }));
        CHECK_EQ(Shape({ 2, 3 }).resolve(Shape(6)), Shape(6));
        CHECK_EQ(Shape({ 2, 3 }).resolve(Shape({ 2, 3 })), Shape({ 2, 3 }));

        // Inferred dimension
        CHECK_EQ(Shape(6).resolve(Shape({ -1, 3 })), Shape({ 2, 3 }));
        CHECK_EQ(Shape(6).resolve(Shape({ 2, -1 })), Shape({ 2, 3 }));
        // - Sole -1 flattens
        CHECK_EQ(Shape({ 2, 3 }).resolve(Shape(-1)), Shape(6));
        // - Zero-size: -1 inferred as 0 when total is 0
        CHECK_EQ(Shape(std::vector<int64_t>{ 0 }).resolve(Shape({ 0, -1 })), Shape({ 0, 0 }));
        // - Scalar source: size-1 shape resolves to 0-dim
        CHECK_EQ(Shape(1).resolve(Shape()), Shape());

        // Exceptions
        // - Undefined source shape
        CHECK_THROWS_AS(Shape(std::vector<int64_t>{ -1 }).resolve(Shape(1)), std::invalid_argument);
        // - Incompatible size
        CHECK_THROWS_AS(Shape(6).resolve(Shape({ 2, 4 })), std::invalid_argument);
        CHECK_THROWS_AS(Shape(6).resolve(Shape(7)), std::invalid_argument);
        // - Multiple -1
        CHECK_THROWS_AS(Shape(6).resolve(Shape({ -1, -1 })), std::invalid_argument);
        // - Non-divisible -1 inference
        CHECK_THROWS_AS(Shape(6).resolve(Shape({ -1, 4 })), std::invalid_argument);
        // - Negative dim other than -1
        CHECK_THROWS_AS(Shape(6).resolve(Shape({ -2, 3 })), std::invalid_argument);
        // - Zero-size known dim with non-zero total
        CHECK_THROWS_AS(Shape(3).resolve(Shape({ 0, -1 })), std::invalid_argument);
    }

    TEST_CASE("Shape::toString()")
    {
        CHECK_EQ(Shape().toString(), "()");
        CHECK_EQ(Shape({ 1 }).toString(), "(1)");
        CHECK_EQ(Shape({ -2, -3 }).toString(), "(-2, -3)");
        CHECK_EQ(Shape({ 4, 5, 6 }).toString(), "(4, 5, 6)");
    }
}
