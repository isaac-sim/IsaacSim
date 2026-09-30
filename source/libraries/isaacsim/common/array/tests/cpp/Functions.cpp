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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

using namespace isaacsim::common::array;

#define SKIP_IF_CUDA_UNAVAILABLE(device)                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        if ((device) == Device::Cuda() && !(device).isAvailable())                                                     \
        {                                                                                                              \
            MESSAGE("Skipping: no CUDA device available (" << (device).toString() << ")");                             \
            return;                                                                                                    \
        }                                                                                                              \
    } while (false)

TEST_SUITE("Functions")
{

    TEST_CASE("add()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Same shape and dtype")
        {
            Array a(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array b(std::vector<float>{ 10.f, 20.f, 30.f }, Dtype::Float32(), device);
            Array result = add(a, b);
            CHECK_EQ(result.shape(), Shape(3));
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 11.f, 22.f, 33.f });
        }

        SUBCASE("Operands are left unchanged")
        {
            Array a(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device);
            Array b(std::vector<float>{ 3.f, 4.f }, Dtype::Float32(), device);
            add(a, b);
            CHECK_EQ(a.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f });
            CHECK_EQ(b.get<std::vector<float>>(), std::vector<float>{ 3.f, 4.f });
        }

        SUBCASE("0-D operands")
        {
            Array a(2.f, Dtype::Float32(), device);
            Array b(3.f, Dtype::Float32(), device);
            Array result = add(a, b);
            CHECK_EQ(result.ndim(), 0);
            CHECK_EQ(result.item<float>(), 5.f);
        }

        SUBCASE("Booleans add as a logical OR")
        {
            Array a(std::vector<bool>{ true, true, false, false }, Dtype::Bool(), device);
            Array b(std::vector<bool>{ true, false, true, false }, Dtype::Bool(), device);
            Array result = add(a, b);
            CHECK_EQ(result.dtype(), Dtype::Bool());
            CHECK_EQ(result.get<std::vector<bool>>(), std::vector<bool>{ true, true, true, false });
        }
    }

    TEST_CASE("subtract()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Same shape and dtype")
        {
            Array a(std::vector<int32_t>{ 10, 20, 30 }, Dtype::Int32(), device);
            Array b(std::vector<int32_t>{ 1, 2, 3 }, Dtype::Int32(), device);
            Array result = subtract(a, b);
            CHECK_EQ(result.dtype(), Dtype::Int32());
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 9, 18, 27 });
        }

        SUBCASE("Boolean operands are rejected, as in NumPy")
        {
            Array a(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            Array b(std::vector<bool>{ false, true }, Dtype::Bool(), device);
            CHECK_THROWS_AS(subtract(a, b), std::invalid_argument);
        }

        SUBCASE("A single boolean operand is allowed: only the promoted dtype matters")
        {
            Array a(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            Array b(std::vector<int32_t>{ 5, 5 }, Dtype::Int32(), device);
            Array result = subtract(a, b);
            CHECK_EQ(result.dtype(), Dtype::Int32());
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ -4, -5 });
        }
    }

    TEST_CASE("multiply()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Same shape and dtype")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            Array b(std::vector<std::vector<float>>{ { 5.f, 6.f }, { 7.f, 8.f } }, Dtype::Float32(), device);
            Array result = multiply(a, b);
            CHECK_EQ(result.shape(), Shape({ 2, 2 }));
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 5.f, 12.f }, { 21.f, 32.f } });
        }

        SUBCASE("Booleans multiply as a logical AND")
        {
            Array a(std::vector<bool>{ true, true, false, false }, Dtype::Bool(), device);
            Array b(std::vector<bool>{ true, false, true, false }, Dtype::Bool(), device);
            Array result = multiply(a, b);
            CHECK_EQ(result.dtype(), Dtype::Bool());
            CHECK_EQ(result.get<std::vector<bool>>(), std::vector<bool>{ true, false, false, false });
        }
    }

    TEST_CASE("divide()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Integer operands divide as true division into float64")
        {
            Array a(std::vector<int32_t>{ 1, 7, 9 }, Dtype::Int32(), device);
            Array b(std::vector<int32_t>{ 2, 2, 3 }, Dtype::Int32(), device);
            Array result = divide(a, b);
            CHECK_EQ(result.dtype(), Dtype::Float64());
            CHECK_EQ(result.get<std::vector<double>>(), std::vector<double>{ 0.5, 3.5, 3.0 });
        }

        SUBCASE("Boolean operands divide into float64")
        {
            Array a(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            Array b(std::vector<bool>{ true, true }, Dtype::Bool(), device);
            Array result = divide(a, b);
            CHECK_EQ(result.dtype(), Dtype::Float64());
            CHECK_EQ(result.get<std::vector<double>>(), std::vector<double>{ 1.0, 0.0 });
        }

        SUBCASE("Float32 operands stay float32")
        {
            Array a(std::vector<float>{ 1.f, 3.f }, Dtype::Float32(), device);
            Array b(std::vector<float>{ 2.f, 2.f }, Dtype::Float32(), device);
            Array result = divide(a, b);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 0.5f, 1.5f });
        }

        SUBCASE("Division by zero yields infinity rather than being undefined")
        {
            Array a(std::vector<int32_t>{ 1, -1 }, Dtype::Int32(), device);
            Array b(std::vector<int32_t>{ 0, 0 }, Dtype::Int32(), device);
            auto result = divide(a, b).get<std::vector<double>>();
            CHECK(std::isinf(result[0]));
            CHECK_GT(result[0], 0.0);
            CHECK(std::isinf(result[1]));
            CHECK_LT(result[1], 0.0);
        }
    }

    TEST_CASE("Broadcasting")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Row vector against a matrix")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 4.f, 5.f, 6.f } }, Dtype::Float32(), device);
            Array b(std::vector<float>{ 10.f, 20.f, 30.f }, Dtype::Float32(), device);
            Array result = add(a, b);
            CHECK_EQ(result.shape(), Shape({ 2, 3 }));
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 11.f, 22.f, 33.f }, { 14.f, 25.f, 36.f } });
        }

        SUBCASE("Column against row: both operands are broadcast")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f }, { 2.f }, { 3.f } }, Dtype::Float32(),
                    device); // (3, 1)
            Array b(std::vector<float>{ 10.f, 20.f }, Dtype::Float32(), device); // (2,)
            Array result = multiply(a, b);
            CHECK_EQ(result.shape(), Shape({ 3, 2 }));
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 10.f, 20.f }, { 20.f, 40.f }, { 30.f, 60.f } });
        }

        SUBCASE("0-D operand against an array")
        {
            Array a(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array b(10.f, Dtype::Float32(), device);
            Array result = subtract(a, b);
            CHECK_EQ(result.shape(), Shape(3));
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ -9.f, -8.f, -7.f });
        }

        SUBCASE("Zero-sized dimension")
        {
            Array a(std::vector<float>{}, Dtype::Float32(), device); // (0,)
            Array b(std::vector<float>{ 10.f }, Dtype::Float32(), device); // (1,)
            Array result = add(a, b);
            CHECK_EQ(result.shape(), Shape(0));
            CHECK_EQ(result.size(), 0);
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{});
        }

        SUBCASE("Incompatible shapes")
        {
            Array a(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array b(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device);
            CHECK_THROWS_AS(add(a, b), std::invalid_argument);
        }
    }

    TEST_CASE("Element counts spanning several CUDA blocks")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        // 1000 elements exceeds the kernel's 256-thread block, exercising a multi-block launch.
        constexpr size_t count = 1000;
        std::vector<float> values(count);
        for (size_t i = 0; i < count; ++i)
        {
            values[i] = static_cast<float>(i);
        }
        Array a(values, Dtype::Float32(), device);
        Array b(2.f, Dtype::Float32(), device);

        auto result = multiply(a, b).get<std::vector<float>>();
        REQUIRE_EQ(result.size(), count);
        CHECK_EQ(result.front(), 0.f);
        CHECK_EQ(result[count / 2], static_cast<float>(count / 2) * 2.f);
        CHECK_EQ(result.back(), static_cast<float>(count - 1) * 2.f);
    }

    TEST_CASE("Type promotion")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Boolean yields to the other operand")
        {
            Array a(std::vector<bool>{ true, true }, Dtype::Bool(), device);
            Array b(std::vector<int32_t>{ 5, 7 }, Dtype::Int32(), device);
            Array result = add(a, b);
            CHECK_EQ(result.dtype(), Dtype::Int32());
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 6, 8 });
        }

        SUBCASE("Integer and float widen to the smallest exact float")
        {
            Array f32(std::vector<float>{ 1.5f }, Dtype::Float32(), device);
            // float32 holds every int16 value exactly, but not every int32 one.
            CHECK_EQ(add(f32, Array(std::vector<int16_t>{ 2 }, Dtype::Int16(), device)).dtype(), Dtype::Float32());
            CHECK_EQ(add(f32, Array(std::vector<int32_t>{ 2 }, Dtype::Int32(), device)).dtype(), Dtype::Float64());
            CHECK_EQ(add(f32, Array(std::vector<double>{ 2.0 }, Dtype::Float64(), device)).dtype(), Dtype::Float64());
        }

        SUBCASE("Same signedness widens to the wider type")
        {
            Array a(std::vector<int16_t>{ 300 }, Dtype::Int16(), device);
            Array b(std::vector<int64_t>{ 4 }, Dtype::Int64(), device);
            Array result = multiply(a, b);
            CHECK_EQ(result.dtype(), Dtype::Int64());
            CHECK_EQ(result.item<int64_t>(), 1200);
        }

        SUBCASE("Signed and unsigned widen to a signed type holding both ranges")
        {
            Array i8(std::vector<int8_t>{ -1 }, Dtype::Int8(), device);
            Array u8(std::vector<uint8_t>{ 200 }, Dtype::UInt8(), device);
            Array result = add(i8, u8);
            CHECK_EQ(result.dtype(), Dtype::Int16());
            CHECK_EQ(result.item<int16_t>(), 199);

            CHECK_EQ(add(Array(std::vector<int16_t>{ 1 }, Dtype::Int16(), device),
                         Array(std::vector<uint16_t>{ 2 }, Dtype::UInt16(), device))
                         .dtype(),
                     Dtype::Int32());
            CHECK_EQ(add(Array(std::vector<int32_t>{ 1 }, Dtype::Int32(), device),
                         Array(std::vector<uint32_t>{ 2 }, Dtype::UInt32(), device))
                         .dtype(),
                     Dtype::Int64());
            // No signed type holds the full uint64 range.
            CHECK_EQ(add(Array(std::vector<int8_t>{ 1 }, Dtype::Int8(), device),
                         Array(std::vector<uint64_t>{ 2 }, Dtype::UInt64(), device))
                         .dtype(),
                     Dtype::Float64());
        }

        SUBCASE("Int64 and uint64 fall back to float64")
        {
            Array a(std::vector<int64_t>{ 1 }, Dtype::Int64(), device);
            Array b(std::vector<uint64_t>{ 2 }, Dtype::UInt64(), device);
            Array result = add(a, b);
            CHECK_EQ(result.dtype(), Dtype::Float64());
            CHECK_EQ(result.item<double>(), 3.0);
        }
    }

    TEST_CASE("Operators")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array a(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
        Array b(std::vector<float>{ 10.f, 20.f, 30.f }, Dtype::Float32(), device);

        CHECK_EQ((a + b).get<std::vector<float>>(), std::vector<float>{ 11.f, 22.f, 33.f });
        CHECK_EQ((a - b).get<std::vector<float>>(), std::vector<float>{ -9.f, -18.f, -27.f });
        CHECK_EQ((a * b).get<std::vector<float>>(), std::vector<float>{ 10.f, 40.f, 90.f });
        CHECK_EQ((a / b).get<std::vector<float>>(), std::vector<float>{ 0.1f, 0.1f, 0.1f });
    }

    TEST_CASE("SupportedInputSpecification operands")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array a(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);

        SUBCASE("Functions")
        {
            CHECK_EQ(add(a, 10.f).get<std::vector<float>>(), std::vector<float>{ 11.f, 12.f, 13.f });
            CHECK_EQ(add(10.f, a).get<std::vector<float>>(), std::vector<float>{ 11.f, 12.f, 13.f });
            CHECK_EQ(subtract(a, 1.f).get<std::vector<float>>(), std::vector<float>{ 0.f, 1.f, 2.f });
            CHECK_EQ(subtract(1.f, a).get<std::vector<float>>(), std::vector<float>{ 0.f, -1.f, -2.f });
            CHECK_EQ(multiply(a, 2.f).get<std::vector<float>>(), std::vector<float>{ 2.f, 4.f, 6.f });
            CHECK_EQ(multiply(2.f, a).get<std::vector<float>>(), std::vector<float>{ 2.f, 4.f, 6.f });
            CHECK_EQ(divide(a, 2.f).get<std::vector<float>>(), std::vector<float>{ 0.5f, 1.f, 1.5f });
            CHECK_EQ(divide(6.f, a).get<std::vector<float>>(), std::vector<float>{ 6.f, 3.f, 2.f });
        }

        SUBCASE("Operators")
        {
            CHECK_EQ((a + 10.f).get<std::vector<float>>(), std::vector<float>{ 11.f, 12.f, 13.f });
            CHECK_EQ((10.f + a).get<std::vector<float>>(), std::vector<float>{ 11.f, 12.f, 13.f });
            CHECK_EQ((a - 1.f).get<std::vector<float>>(), std::vector<float>{ 0.f, 1.f, 2.f });
            CHECK_EQ((1.f - a).get<std::vector<float>>(), std::vector<float>{ 0.f, -1.f, -2.f });
            CHECK_EQ((a * 2.f).get<std::vector<float>>(), std::vector<float>{ 2.f, 4.f, 6.f });
            CHECK_EQ((2.f * a).get<std::vector<float>>(), std::vector<float>{ 2.f, 4.f, 6.f });
            CHECK_EQ((a / 2.f).get<std::vector<float>>(), std::vector<float>{ 0.5f, 1.f, 1.5f });
            CHECK_EQ((6.f / a).get<std::vector<float>>(), std::vector<float>{ 6.f, 3.f, 2.f });
        }

        SUBCASE("Vector spec")
        {
            CHECK_EQ(add(a, std::vector<float>{ 10.f, 20.f, 30.f }).get<std::vector<float>>(),
                     std::vector<float>{ 11.f, 22.f, 33.f });
        }
    }

    TEST_CASE("logicalNot()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Boolean operand")
        {
            Array a(std::vector<bool>{ true, false, true }, Dtype::Bool(), device);
            Array result = logicalNot(a);
            CHECK_EQ(result.shape(), Shape(3));
            CHECK_EQ(result.dtype(), Dtype::Bool());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.get<std::vector<bool>>(), std::vector<bool>{ false, true, false });
        }

        SUBCASE("Non-zero is true for any dtype, as in NumPy")
        {
            Array integers(std::vector<int32_t>{ 0, 1, -2 }, Dtype::Int32(), device);
            CHECK_EQ(logicalNot(integers).dtype(), Dtype::Bool());
            CHECK_EQ(logicalNot(integers).get<std::vector<bool>>(), std::vector<bool>{ true, false, false });

            Array floats(std::vector<float>{ 0.f, -0.f, 0.5f }, Dtype::Float32(), device);
            CHECK_EQ(logicalNot(floats).get<std::vector<bool>>(), std::vector<bool>{ true, true, false });
        }

        SUBCASE("NaN is true, so its negation is false")
        {
            Array a(std::vector<float>{ std::nanf("") }, Dtype::Float32(), device);
            CHECK_EQ(logicalNot(a).get<std::vector<bool>>(), std::vector<bool>{ false });
        }

        SUBCASE("Operand is left unchanged")
        {
            Array a(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            logicalNot(a);
            CHECK_EQ(a.get<std::vector<bool>>(), std::vector<bool>{ true, false });
        }

        SUBCASE("Shape is preserved")
        {
            Array a(std::vector<std::vector<int32_t>>{ { 0, 1 }, { 2, 0 } }, Dtype::Int32(), device);
            CHECK_EQ(logicalNot(a).shape(), Shape({ 2, 2 }));
        }

        SUBCASE("0-D operand")
        {
            Array a(0.f, Dtype::Float32(), device);
            Array result = logicalNot(a);
            CHECK_EQ(result.ndim(), 0);
            CHECK_EQ(result.item<bool>(), true);
        }

        SUBCASE("Zero-sized dimension")
        {
            Array a(std::vector<float>{}, Dtype::Float32(), device);
            CHECK_EQ(logicalNot(a).size(), 0);
        }
    }

    TEST_CASE("negative()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Float operand keeps its dtype")
        {
            Array a(std::vector<float>{ 1.f, -2.f, 0.f }, Dtype::Float32(), device);
            Array result = negative(a);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ -1.f, 2.f, -0.f });
        }

        SUBCASE("Signed integer operand")
        {
            Array a(std::vector<int32_t>{ 3, -4 }, Dtype::Int32(), device);
            CHECK_EQ(negative(a).get<std::vector<int32_t>>(), std::vector<int32_t>{ -3, 4 });
        }

        SUBCASE("Unsigned negation wraps around, as in NumPy")
        {
            Array a(std::vector<uint8_t>{ 0, 1, 200 }, Dtype::UInt8(), device);
            CHECK_EQ(negative(a).dtype(), Dtype::UInt8());
            CHECK_EQ(negative(a).get<std::vector<uint8_t>>(), std::vector<uint8_t>{ 0, 255, 56 });
        }

        SUBCASE("Signed overflow reproduces the minimum value, as in NumPy")
        {
            Array a(std::vector<int8_t>{ -128 }, Dtype::Int8(), device);
            CHECK_EQ(negative(a).get<std::vector<int8_t>>(), std::vector<int8_t>{ -128 });
        }

        SUBCASE("Boolean operands are rejected, as in NumPy")
        {
            Array a(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            CHECK_THROWS_AS(negative(a), std::invalid_argument);
        }

        SUBCASE("Operand is left unchanged")
        {
            Array a(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device);
            negative(a);
            CHECK_EQ(a.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f });
        }
    }

    TEST_CASE("absolute()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Float operand keeps its dtype")
        {
            Array a(std::vector<float>{ 1.f, -2.5f, 0.f }, Dtype::Float32(), device);
            Array result = absolute(a);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.5f, 0.f });
        }

        SUBCASE("Negative zero becomes positive zero")
        {
            Array a(std::vector<double>{ -0.0 }, Dtype::Float64(), device);
            // Distinguishes +0.0 from -0.0, which compare equal.
            CHECK_UNARY_FALSE(std::signbit(absolute(a).get<std::vector<double>>().front()));
        }

        SUBCASE("Signed integer operand")
        {
            Array a(std::vector<int32_t>{ 3, -4, 0 }, Dtype::Int32(), device);
            CHECK_EQ(absolute(a).get<std::vector<int32_t>>(), std::vector<int32_t>{ 3, 4, 0 });
        }

        SUBCASE("Unsigned and boolean operands are the identity")
        {
            Array unsignedArray(std::vector<uint8_t>{ 0, 200 }, Dtype::UInt8(), device);
            CHECK_EQ(absolute(unsignedArray).get<std::vector<uint8_t>>(), std::vector<uint8_t>{ 0, 200 });

            Array booleans(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            CHECK_EQ(absolute(booleans).dtype(), Dtype::Bool());
            CHECK_EQ(absolute(booleans).get<std::vector<bool>>(), std::vector<bool>{ true, false });
        }

        SUBCASE("Signed overflow reproduces the minimum value, as in NumPy")
        {
            Array a(std::vector<int8_t>{ -128 }, Dtype::Int8(), device);
            CHECK_EQ(absolute(a).get<std::vector<int8_t>>(), std::vector<int8_t>{ -128 });
        }
    }

    TEST_CASE("Unary operators")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array a(std::vector<float>{ 1.f, -2.f, 0.f }, Dtype::Float32(), device);
        CHECK_EQ((-a).get<std::vector<float>>(), std::vector<float>{ -1.f, 2.f, -0.f });
        CHECK_EQ((!a).get<std::vector<bool>>(), std::vector<bool>{ false, false, true });
    }

    TEST_CASE("Unary element counts spanning several CUDA blocks")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        // 1000 elements exceeds the kernel's 256-thread block, exercising a multi-block launch.
        constexpr size_t count = 1000;
        std::vector<float> values(count);
        for (size_t i = 0; i < count; ++i)
        {
            values[i] = -static_cast<float>(i);
        }
        Array a(values, Dtype::Float32(), device);

        auto result = absolute(a).get<std::vector<float>>();
        REQUIRE_EQ(result.size(), count);
        CHECK_EQ(result.front(), 0.f);
        CHECK_EQ(result[count / 2], static_cast<float>(count / 2));
        CHECK_EQ(result.back(), static_cast<float>(count - 1));

        // Only element 0 is zero, so exactly one element negates to true.
        auto mask = logicalNot(a).get<std::vector<bool>>();
        REQUIRE_EQ(mask.size(), count);
        CHECK_EQ(std::count(mask.begin(), mask.end(), true), 1);
    }

    TEST_CASE("all()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Boolean operand")
        {
            Array allTrue(std::vector<bool>{ true, true, true }, Dtype::Bool(), device);
            Array result = all(allTrue);
            CHECK_EQ(result.ndim(), 0);
            CHECK_EQ(result.dtype(), Dtype::Bool());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.item<bool>(), true);

            Array oneFalse(std::vector<bool>{ true, false, true }, Dtype::Bool(), device);
            CHECK_EQ(all(oneFalse).item<bool>(), false);
        }

        SUBCASE("Non-zero is true for any dtype, as in NumPy")
        {
            Array integers(std::vector<int32_t>{ 1, -2, 3 }, Dtype::Int32(), device);
            CHECK_EQ(all(integers).item<bool>(), true);
            CHECK_EQ(all(Array(std::vector<int32_t>{ 1, 0, 3 }, Dtype::Int32(), device)).item<bool>(), false);

            // -0.0 is zero, hence false.
            Array floats(std::vector<float>{ 0.5f, -0.f }, Dtype::Float32(), device);
            CHECK_EQ(all(floats).item<bool>(), false);
        }

        SUBCASE("NaN is true, as in NumPy")
        {
            Array a(std::vector<float>{ std::nanf(""), 1.f }, Dtype::Float32(), device);
            CHECK_EQ(all(a).item<bool>(), true);
        }

        SUBCASE("The whole array is reduced, regardless of rank")
        {
            Array a(std::vector<std::vector<int32_t>>{ { 1, 2 }, { 3, 0 } }, Dtype::Int32(), device);
            Array result = all(a);
            CHECK_EQ(result.shape(), Shape());
            CHECK_EQ(result.item<bool>(), false);
        }

        SUBCASE("0-D operand")
        {
            CHECK_EQ(all(Array(0.f, Dtype::Float32(), device)).item<bool>(), false);
            CHECK_EQ(all(Array(2.f, Dtype::Float32(), device)).item<bool>(), true);
        }

        SUBCASE("An empty array is true, as in NumPy")
        {
            Array a(std::vector<float>{}, Dtype::Float32(), device);
            CHECK_EQ(all(a).item<bool>(), true);
        }

        SUBCASE("Operand is left unchanged")
        {
            Array a(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            all(a);
            CHECK_EQ(a.get<std::vector<bool>>(), std::vector<bool>{ true, false });
        }
    }

    TEST_CASE("any()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Boolean operand")
        {
            Array oneTrue(std::vector<bool>{ false, true, false }, Dtype::Bool(), device);
            Array result = any(oneTrue);
            CHECK_EQ(result.ndim(), 0);
            CHECK_EQ(result.dtype(), Dtype::Bool());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.item<bool>(), true);

            Array allFalse(std::vector<bool>{ false, false }, Dtype::Bool(), device);
            CHECK_EQ(any(allFalse).item<bool>(), false);
        }

        SUBCASE("Non-zero is true for any dtype, as in NumPy")
        {
            Array integers(std::vector<int32_t>{ 0, 0, -3 }, Dtype::Int32(), device);
            CHECK_EQ(any(integers).item<bool>(), true);
            CHECK_EQ(any(Array(std::vector<int32_t>{ 0, 0 }, Dtype::Int32(), device)).item<bool>(), false);

            // -0.0 is zero, hence false.
            Array floats(std::vector<float>{ 0.f, -0.f }, Dtype::Float32(), device);
            CHECK_EQ(any(floats).item<bool>(), false);
        }

        SUBCASE("NaN is true, as in NumPy")
        {
            Array a(std::vector<float>{ 0.f, std::nanf("") }, Dtype::Float32(), device);
            CHECK_EQ(any(a).item<bool>(), true);
        }

        SUBCASE("The whole array is reduced, regardless of rank")
        {
            Array a(std::vector<std::vector<int32_t>>{ { 0, 0 }, { 0, 4 } }, Dtype::Int32(), device);
            Array result = any(a);
            CHECK_EQ(result.shape(), Shape());
            CHECK_EQ(result.item<bool>(), true);
        }

        SUBCASE("0-D operand")
        {
            CHECK_EQ(any(Array(0.f, Dtype::Float32(), device)).item<bool>(), false);
            CHECK_EQ(any(Array(2.f, Dtype::Float32(), device)).item<bool>(), true);
        }

        SUBCASE("An empty array is false, as in NumPy")
        {
            Array a(std::vector<float>{}, Dtype::Float32(), device);
            CHECK_EQ(any(a).item<bool>(), false);
        }

        SUBCASE("Operand is left unchanged")
        {
            Array a(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            any(a);
            CHECK_EQ(a.get<std::vector<bool>>(), std::vector<bool>{ true, false });
        }
    }

    TEST_CASE("sum()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Float operand keeps its dtype")
        {
            Array a(std::vector<float>{ 1.5f, 2.f, -0.5f }, Dtype::Float32(), device);
            Array result = sum(a);
            CHECK_EQ(result.ndim(), 0);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.item<float>(), 3.f);

            CHECK_EQ(sum(Array(std::vector<double>{ 0.25, 0.75 }, Dtype::Float64(), device)).dtype(), Dtype::Float64());
        }

        SUBCASE("Narrow integers accumulate in int64, as in NumPy")
        {
            // 100 + 100 overflows int8, so the widened accumulator is what keeps the sum exact.
            Array a(std::vector<int8_t>{ 100, 100 }, Dtype::Int8(), device);
            Array result = sum(a);
            CHECK_EQ(result.dtype(), Dtype::Int64());
            CHECK_EQ(result.item<int64_t>(), 200);
        }

        SUBCASE("Unsigned integers accumulate in uint64, as in NumPy")
        {
            Array a(std::vector<uint8_t>{ 200, 200 }, Dtype::UInt8(), device);
            Array result = sum(a);
            CHECK_EQ(result.dtype(), Dtype::UInt64());
            CHECK_EQ(result.item<uint64_t>(), 400);
        }

        SUBCASE("Booleans accumulate in int64, counting the true elements, as in NumPy")
        {
            Array a(std::vector<bool>{ true, false, true }, Dtype::Bool(), device);
            Array result = sum(a);
            CHECK_EQ(result.dtype(), Dtype::Int64());
            CHECK_EQ(result.item<int64_t>(), 2);
        }

        SUBCASE("The whole array is reduced, regardless of rank")
        {
            Array a(std::vector<std::vector<int32_t>>{ { 1, 2 }, { 3, 4 } }, Dtype::Int32(), device);
            Array result = sum(a);
            CHECK_EQ(result.shape(), Shape());
            CHECK_EQ(result.item<int64_t>(), 10);
        }

        SUBCASE("An empty array sums to 0, as in NumPy")
        {
            Array a(std::vector<float>{}, Dtype::Float32(), device);
            CHECK_EQ(sum(a).item<float>(), 0.f);
        }

        SUBCASE("0-D operand")
        {
            CHECK_EQ(sum(Array(2.5f, Dtype::Float32(), device)).item<float>(), 2.5f);
        }

        SUBCASE("Operand is left unchanged")
        {
            Array a(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device);
            sum(a);
            CHECK_EQ(a.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f });
        }
    }

    TEST_CASE("prod()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Float operand keeps its dtype")
        {
            Array a(std::vector<float>{ 2.f, 2.5f, -2.f }, Dtype::Float32(), device);
            Array result = prod(a);
            CHECK_EQ(result.ndim(), 0);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.item<float>(), -10.f);
        }

        SUBCASE("Narrow integers accumulate in int64, as in NumPy")
        {
            // 20 * 20 overflows int8, so the widened accumulator is what keeps the product exact.
            Array a(std::vector<int8_t>{ 20, 20 }, Dtype::Int8(), device);
            Array result = prod(a);
            CHECK_EQ(result.dtype(), Dtype::Int64());
            CHECK_EQ(result.item<int64_t>(), 400);
        }

        SUBCASE("Unsigned integers accumulate in uint64, as in NumPy")
        {
            Array a(std::vector<uint8_t>{ 20, 20 }, Dtype::UInt8(), device);
            Array result = prod(a);
            CHECK_EQ(result.dtype(), Dtype::UInt64());
            CHECK_EQ(result.item<uint64_t>(), 400);
        }

        SUBCASE("Booleans accumulate in int64, as in NumPy")
        {
            CHECK_EQ(prod(Array(std::vector<bool>{ true, true }, Dtype::Bool(), device)).item<int64_t>(), 1);
            CHECK_EQ(prod(Array(std::vector<bool>{ true, false }, Dtype::Bool(), device)).item<int64_t>(), 0);
        }

        SUBCASE("Float64 operand keeps its dtype")
        {
            Array a(std::vector<double>{ 0.5, 4.0 }, Dtype::Float64(), device);
            CHECK_EQ(prod(a).dtype(), Dtype::Float64());
            CHECK_EQ(prod(a).item<double>(), 2.0);
        }

        SUBCASE("The whole array is reduced, regardless of rank")
        {
            Array a(std::vector<std::vector<int32_t>>{ { 1, 2 }, { 3, 4 } }, Dtype::Int32(), device);
            CHECK_EQ(prod(a).item<int64_t>(), 24);
        }

        SUBCASE("An empty array has a product of 1, as in NumPy")
        {
            Array a(std::vector<float>{}, Dtype::Float32(), device);
            CHECK_EQ(prod(a).item<float>(), 1.f);
        }

        SUBCASE("0-D operand")
        {
            CHECK_EQ(prod(Array(2.5f, Dtype::Float32(), device)).item<float>(), 2.5f);
        }
    }

    TEST_CASE("amin() and amax()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Operand dtype is preserved")
        {
            Array a(std::vector<float>{ 2.f, -1.5f, 3.f }, Dtype::Float32(), device);
            Array minimum = amin(a);
            CHECK_EQ(minimum.ndim(), 0);
            CHECK_EQ(minimum.dtype(), Dtype::Float32());
            CHECK_EQ(minimum.device(), device);
            CHECK_EQ(minimum.item<float>(), -1.5f);

            Array maximum = amax(a);
            CHECK_EQ(maximum.dtype(), Dtype::Float32());
            CHECK_EQ(maximum.item<float>(), 3.f);
        }

        SUBCASE("Narrow integers keep their dtype, unlike sum()")
        {
            Array a(std::vector<int8_t>{ 100, -5, 7 }, Dtype::Int8(), device);
            CHECK_EQ(amin(a).dtype(), Dtype::Int8());
            CHECK_EQ(amin(a).item<int8_t>(), -5);
            CHECK_EQ(amax(a).item<int8_t>(), 100);
        }

        SUBCASE("Boolean operand")
        {
            Array mixed(std::vector<bool>{ true, false }, Dtype::Bool(), device);
            CHECK_EQ(amin(mixed).dtype(), Dtype::Bool());
            CHECK_EQ(amin(mixed).item<bool>(), false);
            CHECK_EQ(amax(mixed).item<bool>(), true);

            Array allTrue(std::vector<bool>{ true, true }, Dtype::Bool(), device);
            CHECK_EQ(amin(allTrue).item<bool>(), true);
        }

        SUBCASE("The whole array is reduced, regardless of rank")
        {
            Array a(std::vector<std::vector<int32_t>>{ { 4, 9 }, { -2, 7 } }, Dtype::Int32(), device);
            CHECK_EQ(amin(a).shape(), Shape());
            CHECK_EQ(amin(a).item<int32_t>(), -2);
            CHECK_EQ(amax(a).item<int32_t>(), 9);
        }

        SUBCASE("0-D operand")
        {
            Array a(2.5f, Dtype::Float32(), device);
            CHECK_EQ(amin(a).item<float>(), 2.5f);
            CHECK_EQ(amax(a).item<float>(), 2.5f);
        }

        SUBCASE("Float64 operand keeps its dtype")
        {
            Array a(std::vector<double>{ 2.5, -1.25 }, Dtype::Float64(), device);
            CHECK_EQ(amin(a).dtype(), Dtype::Float64());
            CHECK_EQ(amin(a).item<double>(), -1.25);
            CHECK_EQ(amax(a).item<double>(), 2.5);
        }

        SUBCASE("NaN propagates, as in NumPy")
        {
            // Every comparison against NaN is false, so a plain comparison fold would skip it
            // unless it happened to seed the accumulator. Place it last to catch exactly that.
            Array trailing(std::vector<float>{ 1.f, 2.f, std::nanf("") }, Dtype::Float32(), device);
            CHECK_UNARY(std::isnan(amin(trailing).item<float>()));
            CHECK_UNARY(std::isnan(amax(trailing).item<float>()));

            Array leading(std::vector<float>{ std::nanf(""), 1.f, 2.f }, Dtype::Float32(), device);
            CHECK_UNARY(std::isnan(amin(leading).item<float>()));
            CHECK_UNARY(std::isnan(amax(leading).item<float>()));

            Array doubles(std::vector<double>{ 1.0, std::nan("") }, Dtype::Float64(), device);
            CHECK_UNARY(std::isnan(amin(doubles).item<double>()));
            CHECK_UNARY(std::isnan(amax(doubles).item<double>()));
        }

        SUBCASE("An empty array is rejected, as in NumPy")
        {
            Array a(std::vector<float>{}, Dtype::Float32(), device);
            CHECK_THROWS_AS(amin(a), std::invalid_argument);
            CHECK_THROWS_AS(amax(a), std::invalid_argument);
        }

        SUBCASE("Operand is left unchanged")
        {
            Array a(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device);
            amin(a);
            amax(a);
            CHECK_EQ(a.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f });
        }
    }

    TEST_CASE("Reductions over element counts exceeding the CUDA block width")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        // The reduction kernel always launches a single block, so 1000 elements against its 256
        // threads makes each thread fold several elements before the block-wide reduction.
        constexpr size_t count = 1000;
        std::vector<int32_t> values(count, 1);
        CHECK_EQ(all(Array(values, Dtype::Int32(), device)).item<bool>(), true);

        // A single false element anywhere flips all() and is the only true one for any().
        for (size_t index : { size_t(0), count / 2, count - 1 })
        {
            std::vector<int32_t> zeroValues(count, 0);
            zeroValues[index] = 1;
            Array a(zeroValues, Dtype::Int32(), device);
            CHECK_EQ(all(a).item<bool>(), false);
            CHECK_EQ(any(a).item<bool>(), true);
        }

        CHECK_EQ(any(Array(std::vector<int32_t>(count, 0), Dtype::Int32(), device)).item<bool>(), false);

        // 0 + 1 + ... + 999, which the widened accumulator holds exactly.
        std::vector<int32_t> range(count);
        for (size_t i = 0; i < count; ++i)
        {
            range[i] = static_cast<int32_t>(i);
        }
        Array a(range, Dtype::Int32(), device);
        CHECK_EQ(sum(a).item<int64_t>(), static_cast<int64_t>(count) * (static_cast<int64_t>(count) - 1) / 2);
        CHECK_EQ(amin(a).item<int32_t>(), 0);
        CHECK_EQ(amax(a).item<int32_t>(), static_cast<int32_t>(count) - 1);

        // An extremum at the very end is the case a partial block-wide fold would miss.
        std::vector<int32_t> oneValues(count, 1);
        oneValues.back() = 42;
        CHECK_EQ(amax(Array(oneValues, Dtype::Int32(), device)).item<int32_t>(), 42);
        CHECK_EQ(prod(Array(oneValues, Dtype::Int32(), device)).item<int64_t>(), 42);

        // A NaN past the kernel's 256-thread block width only propagates if the per-thread fold and
        // the block-wide tree reduction both carry it through, rather than a thread whose share
        // happens to avoid it masking the result.
        std::vector<float> floats(count, 1.f);
        floats[count - 1] = std::nanf("");
        Array withTrailingNaN(floats, Dtype::Float32(), device);
        CHECK_UNARY(std::isnan(amin(withTrailingNaN).item<float>()));
        CHECK_UNARY(std::isnan(amax(withTrailingNaN).item<float>()));
    }

    TEST_CASE("Operands on different devices")
    {
        SKIP_IF_CUDA_UNAVAILABLE(Device::Cuda());

        Array cpuArray(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), Device::Cpu());
        Array cudaArray(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), Device::Cuda());
        CHECK_THROWS_AS(add(cpuArray, cudaArray), std::invalid_argument);
        CHECK_THROWS_AS(divide(cudaArray, cpuArray), std::invalid_argument);
    }

    TEST_CASE("empty()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Shape, dtype, and device are as requested")
        {
            // The contents are deliberately unspecified, so only the metadata is asserted.
            Array a = empty(Shape({ 2, 3 }), Dtype::Int32(), device);
            CHECK_EQ(a.shape(), Shape({ 2, 3 }));
            CHECK_EQ(a.ndim(), 2);
            CHECK_EQ(a.size(), 6);
            CHECK_EQ(a.nbytes(), 24);
            CHECK_EQ(a.dtype(), Dtype::Int32());
            CHECK_EQ(a.device(), device);
        }

        SUBCASE("The dtype defaults to float64, as in NumPy")
        {
            CHECK_EQ(empty(Shape(3)).dtype(), Dtype::Float64());
            CHECK_EQ(empty(Shape(3)).device(), Device::Cpu());
        }

        SUBCASE("A 0-D shape holds exactly one element")
        {
            Array a = empty(Shape(), Dtype::Float32(), device);
            CHECK_EQ(a.ndim(), 0);
            CHECK_EQ(a.size(), 1);
        }

        SUBCASE("A zero-sized shape allocates nothing but still has a usable pointer")
        {
            Array a = empty(Shape({ 0, 4 }), Dtype::Float32(), device);
            CHECK_EQ(a.shape(), Shape({ 0, 4 }));
            CHECK_EQ(a.size(), 0);
            CHECK_EQ(a.nbytes(), 0);
            // cudaMalloc(0) would return null; empty() allocates a byte so data() stays valid.
            CHECK_NE(a.data(), nullptr);
        }

        SUBCASE("Each call returns an independent buffer")
        {
            Array a = empty(Shape(4), Dtype::Float32(), device);
            Array b = empty(Shape(4), Dtype::Float32(), device);
            CHECK_NE(a.data(), b.data());
        }

        SUBCASE("The result is writable through set()")
        {
            Array a = empty(Shape(3), Dtype::Float32(), device);
            a.set(std::vector<float>{ 1.f, 2.f, 3.f });
            CHECK_EQ(a.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }

        SUBCASE("A negative dimension is rejected")
        {
            CHECK_THROWS_AS(empty(Shape({ -1, 2 }), Dtype::Float32(), device), std::invalid_argument);
            CHECK_THROWS_AS(zeros(Shape({ -1, 2 }), Dtype::Float32(), device), std::invalid_argument);
            CHECK_THROWS_AS(ones(Shape({ -1, 2 }), Dtype::Float32(), device), std::invalid_argument);
        }
    }

    TEST_CASE("zeros()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Every element is zero")
        {
            Array a = zeros(Shape({ 2, 3 }), Dtype::Float32(), device);
            CHECK_EQ(a.shape(), Shape({ 2, 3 }));
            CHECK_EQ(a.dtype(), Dtype::Float32());
            CHECK_EQ(a.device(), device);
            CHECK_EQ(a.flatten().get<std::vector<float>>(), std::vector<float>(6, 0.f));
        }

        SUBCASE("The dtype defaults to float64, as in NumPy")
        {
            CHECK_EQ(zeros(Shape(3)).dtype(), Dtype::Float64());
        }

        SUBCASE("Every dtype zeroes correctly")
        {
            CHECK_EQ(zeros(Shape(4), Dtype::Bool(), device).get<std::vector<bool>>(), std::vector<bool>(4, false));
            CHECK_EQ(zeros(Shape(4), Dtype::Int8(), device).get<std::vector<int8_t>>(), std::vector<int8_t>(4, 0));
            CHECK_EQ(zeros(Shape(4), Dtype::Int64(), device).get<std::vector<int64_t>>(), std::vector<int64_t>(4, 0));
            CHECK_EQ(zeros(Shape(4), Dtype::UInt32(), device).get<std::vector<uint32_t>>(), std::vector<uint32_t>(4, 0));
            CHECK_EQ(zeros(Shape(4), Dtype::Float64(), device).get<std::vector<double>>(), std::vector<double>(4, 0.0));
        }

        SUBCASE("A 0-D shape holds a single zero")
        {
            Array a = zeros(Shape(), Dtype::Float32(), device);
            CHECK_EQ(a.ndim(), 0);
            CHECK_EQ(a.item<float>(), 0.f);
        }

        SUBCASE("A zero-sized shape is empty but well formed")
        {
            Array a = zeros(Shape({ 0, 4 }), Dtype::Float32(), device);
            CHECK_EQ(a.shape(), Shape({ 0, 4 }));
            CHECK_EQ(a.size(), 0);
        }

        SUBCASE("Element counts spanning several CUDA blocks")
        {
            constexpr size_t count = 1000;
            Array a = zeros(Shape(static_cast<int64_t>(count)), Dtype::Int32(), device);
            CHECK_EQ(a.get<std::vector<int32_t>>(), std::vector<int32_t>(count, 0));
        }
    }

    TEST_CASE("ones()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Every element is one")
        {
            Array a = ones(Shape({ 2, 3 }), Dtype::Float32(), device);
            CHECK_EQ(a.shape(), Shape({ 2, 3 }));
            CHECK_EQ(a.dtype(), Dtype::Float32());
            CHECK_EQ(a.device(), device);
            CHECK_EQ(a.flatten().get<std::vector<float>>(), std::vector<float>(6, 1.f));
        }

        SUBCASE("The dtype defaults to float64, as in NumPy")
        {
            CHECK_EQ(ones(Shape(3)).dtype(), Dtype::Float64());
        }

        SUBCASE("Every dtype yields its own representation of one")
        {
            // A memset cannot produce these: float 1.0f is not a repeated byte pattern.
            CHECK_EQ(ones(Shape(4), Dtype::Bool(), device).get<std::vector<bool>>(), std::vector<bool>(4, true));
            CHECK_EQ(ones(Shape(4), Dtype::Int8(), device).get<std::vector<int8_t>>(), std::vector<int8_t>(4, 1));
            CHECK_EQ(ones(Shape(4), Dtype::Int64(), device).get<std::vector<int64_t>>(), std::vector<int64_t>(4, 1));
            CHECK_EQ(ones(Shape(4), Dtype::UInt32(), device).get<std::vector<uint32_t>>(), std::vector<uint32_t>(4, 1));
            CHECK_EQ(ones(Shape(4), Dtype::Float64(), device).get<std::vector<double>>(), std::vector<double>(4, 1.0));
        }

        SUBCASE("A 0-D shape holds a single one")
        {
            Array a = ones(Shape(), Dtype::Float32(), device);
            CHECK_EQ(a.ndim(), 0);
            CHECK_EQ(a.item<float>(), 1.f);
        }

        SUBCASE("A zero-sized shape is empty but well formed")
        {
            Array a = ones(Shape({ 0, 4 }), Dtype::Float32(), device);
            CHECK_EQ(a.shape(), Shape({ 0, 4 }));
            CHECK_EQ(a.size(), 0);
        }

        SUBCASE("Element counts spanning several CUDA blocks")
        {
            constexpr size_t count = 1000;
            Array a = ones(Shape(static_cast<int64_t>(count)), Dtype::Int32(), device);
            CHECK_EQ(a.get<std::vector<int32_t>>(), std::vector<int32_t>(count, 1));
        }

        SUBCASE("Results are independent of each other")
        {
            Array a = ones(Shape(3), Dtype::Float32(), device);
            Array b = ones(Shape(3), Dtype::Float32(), device);
            a.set(0.f);
            CHECK_EQ(b.get<std::vector<float>>(), std::vector<float>{ 1.f, 1.f, 1.f });
        }
    }

    TEST_CASE("transpose()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("2-D reverses the axes")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 4.f, 5.f, 6.f } }, Dtype::Float32(), device);
            Array result = transpose(a);
            CHECK_EQ(result.shape(), Shape({ 3, 2 }));
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 4.f }, { 2.f, 5.f }, { 3.f, 6.f } });
        }

        SUBCASE("3-D with an explicit permutation")
        {
            // Shape (2, 3, 4) holding 0..23 in row-major order.
            std::vector<float> values(24);
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = static_cast<float>(i);
            }
            Array a = Array(values, Dtype::Float32(), device).reshape(Shape({ 2, 3, 4 }));

            // (2, 3, 4) -> (4, 2, 3): destination[i][j][k] == source[j][k][i].
            Array result = transpose(a, std::vector<int64_t>{ 2, 0, 1 });
            CHECK_EQ(result.shape(), Shape({ 4, 2, 3 }));

            std::vector<float> expected(24);
            for (size_t i = 0; i < 4; ++i)
            {
                for (size_t j = 0; j < 2; ++j)
                {
                    for (size_t k = 0; k < 3; ++k)
                    {
                        expected[(i * 2 + j) * 3 + k] = static_cast<float>((j * 3 + k) * 4 + i);
                    }
                }
            }
            CHECK_EQ(result.flatten().get<std::vector<float>>(), expected);
        }

        SUBCASE("Omitting the axes reverses them, matching the explicit permutation")
        {
            std::vector<float> values(24);
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = static_cast<float>(i);
            }
            Array a = Array(values, Dtype::Float32(), device).reshape(Shape({ 2, 3, 4 }));

            Array implicitAxes = transpose(a);
            Array explicitAxes = transpose(a, std::vector<int64_t>{ 2, 1, 0 });
            CHECK_EQ(implicitAxes.shape(), Shape({ 4, 3, 2 }));
            CHECK_EQ(implicitAxes.shape(), explicitAxes.shape());
            CHECK_EQ(implicitAxes.flatten().get<std::vector<float>>(), explicitAxes.flatten().get<std::vector<float>>());

            // destination[i][j][k] == source[k][j][i], element-wise rather than by shape alone.
            std::vector<float> expected(24);
            for (size_t i = 0; i < 4; ++i)
            {
                for (size_t j = 0; j < 3; ++j)
                {
                    for (size_t k = 0; k < 2; ++k)
                    {
                        expected[(i * 3 + j) * 2 + k] = static_cast<float>((k * 3 + j) * 4 + i);
                    }
                }
            }
            CHECK_EQ(implicitAxes.flatten().get<std::vector<float>>(), expected);
        }

        SUBCASE("Negative axes count from the last dimension")
        {
            Array a(std::vector<std::vector<int32_t>>{ { 1, 2, 3 }, { 4, 5, 6 } }, Dtype::Int32(), device);
            CHECK_EQ(transpose(a, std::vector<int64_t>{ -1, -2 }).get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 1, 4 }, { 2, 5 }, { 3, 6 } });
        }

        SUBCASE("Transposing twice restores the original")
        {
            Array a =
                Array(std::vector<float>{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f }, Dtype::Float32(), device).reshape(Shape({ 2, 3 }));
            CHECK_EQ(transpose(transpose(a)).flatten().get<std::vector<float>>(),
                     std::vector<float>{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f });
        }

        SUBCASE("0-D and 1-D operands are unchanged")
        {
            Array scalar(42.f, Dtype::Float32(), device);
            Array transposedScalar = transpose(scalar);
            CHECK_EQ(transposedScalar.ndim(), 0);
            CHECK_EQ(transposedScalar.item<float>(), 42.f);

            Array vector(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array transposedVector = transpose(vector);
            CHECK_EQ(transposedVector.shape(), Shape(3));
            CHECK_EQ(transposedVector.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }

        SUBCASE("dtype and device are preserved")
        {
            Array a(std::vector<std::vector<int64_t>>{ { 1, 2 }, { 3, 4 } }, Dtype::Int64(), device);
            Array result = transpose(a);
            CHECK_EQ(result.dtype(), Dtype::Int64());
            CHECK_EQ(result.device(), device);
        }

        SUBCASE("The result is an independent copy and the operand is unchanged")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            Array result = transpose(a);
            result.set(0.f);
            CHECK_EQ(a.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } });
        }

        SUBCASE("Gathers from a sliced operand, honoring its buffer offset")
        {
            Array a = Array(std::vector<float>{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f }, Dtype::Float32(), device)
                          .reshape(Shape({ 2, 2, 2 }));
            // The second (2, 2) slice: {{5, 6}, {7, 8}}.
            Array slice = a.at(1);
            CHECK_EQ(transpose(slice).get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 5.f, 7.f }, { 6.f, 8.f } });
        }

        SUBCASE("An axis of size zero is preserved")
        {
            Array a = Array(std::vector<float>{}, Dtype::Float32(), device).reshape(Shape({ 0, 3 }));
            Array result = transpose(a);
            CHECK_EQ(result.shape(), Shape({ 3, 0 }));
            CHECK_EQ(result.size(), 0);
        }

        SUBCASE("Invalid permutations throw")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            CHECK_THROWS_AS(transpose(a, std::vector<int64_t>{ 0 }), std::invalid_argument);
            CHECK_THROWS_AS(transpose(a, std::vector<int64_t>{ 0, 1, 2 }), std::invalid_argument);
            CHECK_THROWS_AS(transpose(a, std::vector<int64_t>{ 0, 2 }), std::invalid_argument);
            CHECK_THROWS_AS(transpose(a, std::vector<int64_t>{ 0, -3 }), std::invalid_argument);
            CHECK_THROWS_AS(transpose(a, std::vector<int64_t>{ 1, 1 }), std::invalid_argument);
            CHECK_THROWS_AS(transpose(a, std::vector<int64_t>{ -1, 1 }), std::invalid_argument);
        }

        SUBCASE("Element counts spanning several CUDA blocks")
        {
            constexpr size_t rows = 37;
            constexpr size_t columns = 29; // 1073 elements: several 256-thread blocks, with a partial one.
            std::vector<int32_t> values(rows * columns);
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = static_cast<int32_t>(i);
            }
            Array a = Array(values, Dtype::Int32(), device)
                          .reshape(Shape({ static_cast<int64_t>(rows), static_cast<int64_t>(columns) }));
            Array result = transpose(a);
            CHECK_EQ(result.shape(), Shape({ static_cast<int64_t>(columns), static_cast<int64_t>(rows) }));

            std::vector<int32_t> expected(rows * columns);
            for (size_t column = 0; column < columns; ++column)
            {
                for (size_t row = 0; row < rows; ++row)
                {
                    expected[column * rows + row] = static_cast<int32_t>(row * columns + column);
                }
            }
            CHECK_EQ(result.flatten().get<std::vector<int32_t>>(), expected);
        }
    }

    TEST_CASE("take()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Along axis 0")
        {
            Array a(
                std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f }, { 5.f, 6.f } }, Dtype::Float32(), device);
            Array result = take(a, { 2, 0 }, 0);
            CHECK_EQ(result.shape(), Shape({ 2, 2 }));
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 5.f, 6.f }, { 1.f, 2.f } });
        }

        SUBCASE("Along the last axis")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 4.f, 5.f, 6.f } }, Dtype::Float32(), device);
            CHECK_EQ(take(a, { 2, 1 }, 1).get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 3.f, 2.f }, { 6.f, 5.f } });
        }

        SUBCASE("Along a middle axis")
        {
            // Shape (2, 3, 2) holding 0..11 in row-major order.
            std::vector<float> values(12);
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = static_cast<float>(i);
            }
            Array a = Array(values, Dtype::Float32(), device).reshape(Shape({ 2, 3, 2 }));
            Array result = take(a, { 2, 0 }, 1);
            CHECK_EQ(result.shape(), Shape({ 2, 2, 2 }));
            // Rows 2 and 0 of each (3, 2) block: {4, 5, 0, 1} then {10, 11, 6, 7}.
            CHECK_EQ(result.flatten().get<std::vector<float>>(),
                     std::vector<float>{ 4.f, 5.f, 0.f, 1.f, 10.f, 11.f, 6.f, 7.f });
        }

        SUBCASE("Negative axis counts from the last dimension")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 4.f, 5.f, 6.f } }, Dtype::Float32(), device);
            CHECK_EQ(take(a, { 0, 2 }, -1).get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 3.f }, { 4.f, 6.f } });
        }

        SUBCASE("Negative indices count from the end of the axis")
        {
            Array a(std::vector<float>{ 1.f, 2.f, 3.f, 4.f }, Dtype::Float32(), device);
            CHECK_EQ(take(a, { -1, -4 }, 0).get<std::vector<float>>(), std::vector<float>{ 4.f, 1.f });
        }

        SUBCASE("Omitting the axis operates on the flattened array")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 4.f, 5.f, 6.f } }, Dtype::Float32(), device);
            Array result = take(a, { 5, 0, -2 });
            CHECK_EQ(result.shape(), Shape(3));
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 6.f, 1.f, 5.f });

            // A 0-D operand flattens to a single element, as in NumPy.
            Array scalar(7.f, Dtype::Float32(), device);
            CHECK_EQ(take(scalar, { 0, 0 }).get<std::vector<float>>(), std::vector<float>{ 7.f, 7.f });
        }

        SUBCASE("Repeated indices duplicate elements")
        {
            Array a(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device);
            CHECK_EQ(take(a, { 1, 1, 0, 1 }, 0).get<std::vector<float>>(), std::vector<float>{ 2.f, 2.f, 1.f, 2.f });
        }

        SUBCASE("Empty indices produce a zero-sized axis")
        {
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            Array result = take(a, {}, 0);
            CHECK_EQ(result.shape(), Shape({ 0, 2 }));
            CHECK_EQ(result.size(), 0);
        }

        SUBCASE("dtype and device are preserved")
        {
            Array a(std::vector<std::vector<int64_t>>{ { 1, 2 }, { 3, 4 } }, Dtype::Int64(), device);
            Array result = take(a, { 1, 0 }, 0);
            CHECK_EQ(result.dtype(), Dtype::Int64());
            CHECK_EQ(result.device(), device);
        }

        SUBCASE("The result is an independent copy and the operand is unchanged")
        {
            Array a(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result = take(a, { 0, 1, 2 }, 0);
            result.set(0.f);
            CHECK_EQ(a.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }

        SUBCASE("Gathers from a sliced operand, honoring its buffer offset")
        {
            Array a =
                Array(std::vector<float>{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f }, Dtype::Float32(), device).reshape(Shape({ 3, 2 }));
            Array slice = a.at(2); // {5, 6}
            CHECK_EQ(take(slice, { 1, 0 }, 0).get<std::vector<float>>(), std::vector<float>{ 6.f, 5.f });
        }

        SUBCASE("An out-of-range axis throws invalid_argument, an out-of-range index out_of_range")
        {
            // NumPy draws the same distinction: an AxisError is a ValueError, while an
            // out-of-range index is an IndexError.
            Array a(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            CHECK_THROWS_AS(take(a, { 0 }, 2), std::invalid_argument);
            CHECK_THROWS_AS(take(a, { 0 }, -3), std::invalid_argument);
            CHECK_THROWS_AS(take(a, { 2 }, 0), std::out_of_range);
            CHECK_THROWS_AS(take(a, { -3 }, 0), std::out_of_range);

            // An axis of size zero has no index in range at all.
            Array empty = Array(std::vector<float>{}, Dtype::Float32(), device).reshape(Shape({ 0, 2 }));
            CHECK_THROWS_AS(take(empty, { 0 }, 0), std::out_of_range);
        }

        SUBCASE("A 0-D operand behaves as 1-D of size one, as in NumPy")
        {
            Array scalar(7.f, Dtype::Float32(), device);

            // np.take(np.array(7.0), [0], axis=0) is array([7.0]) rather than an error.
            CHECK_EQ(take(scalar, { 0 }, 0).get<std::vector<float>>(), std::vector<float>{ 7.f });
            CHECK_EQ(take(scalar, { 0, 0 }, -1).get<std::vector<float>>(), std::vector<float>{ 7.f, 7.f });

            // Only an axis beyond that single dimension is out of range.
            CHECK_THROWS_AS(take(scalar, { 0 }, 1), std::invalid_argument);
            CHECK_THROWS_AS(take(scalar, { 0 }, -2), std::invalid_argument);
            CHECK_THROWS_AS(take(scalar, { 1 }, 0), std::out_of_range);
        }

        SUBCASE("Quaternion component reordering")
        {
            // (N, 4) quaternions laid out as w, x, y, z.
            Array wxyz(std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f, 4.f }, { 5.f, 6.f, 7.f, 8.f } },
                       Dtype::Float32(), device);

            Array xyzw = take(wxyz, { 1, 2, 3, 0 }, 1);
            CHECK_EQ(xyzw.shape(), Shape({ 2, 4 }));
            CHECK_EQ(xyzw.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 2.f, 3.f, 4.f, 1.f }, { 6.f, 7.f, 8.f, 5.f } });

            // Round trip back to w, x, y, z.
            CHECK_EQ(take(xyzw, { 3, 0, 1, 2 }, 1).get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f, 4.f }, { 5.f, 6.f, 7.f, 8.f } });
        }

        SUBCASE("Index lists on both sides of the CUDA inline-argument threshold")
        {
            // takeFunction() passes short index lists as a by-value kernel argument and falls back to a
            // device scratch buffer past kMaxInlineIndices (64), so both paths need exercising.
            constexpr int64_t axisSize = 70;
            std::vector<float> values(axisSize);
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = static_cast<float>(i);
            }
            Array a(values, Dtype::Float32(), device);

            // Reversed indices, so a path that silently ignored them would not match.
            for (size_t indexCount : { size_t{ 63 }, size_t{ 64 }, size_t{ 65 }, size_t{ 70 } })
            {
                std::vector<int64_t> indices(indexCount);
                std::vector<float> expected(indexCount);
                for (size_t i = 0; i < indexCount; ++i)
                {
                    indices[i] = static_cast<int64_t>(indexCount - 1 - i);
                    expected[i] = static_cast<float>(indexCount - 1 - i);
                }
                CAPTURE(indexCount);
                CHECK_EQ(take(a, indices, 0).get<std::vector<float>>(), expected);
            }
        }

        SUBCASE("A middle axis spanning several CUDA blocks")
        {
            // The last-axis case below leaves `inner` at 1, which hides the kernel's
            // `linear % inner` / `remaining / indexCount` split. Shape (7, 5, 31) gathers 3 of the
            // 5 middle rows with inner = 31, for 651 destination elements across several blocks.
            constexpr size_t outer = 7;
            constexpr size_t axisSize = 5;
            constexpr size_t inner = 31;
            std::vector<int32_t> values(outer * axisSize * inner);
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = static_cast<int32_t>(i);
            }
            Array a = Array(values, Dtype::Int32(), device)
                          .reshape(Shape({ static_cast<int64_t>(outer), static_cast<int64_t>(axisSize),
                                           static_cast<int64_t>(inner) }));

            const std::vector<int64_t> order{ 4, 0, 2 };
            Array result = take(a, order, 1);
            CHECK_EQ(result.shape(), Shape({ static_cast<int64_t>(outer), static_cast<int64_t>(order.size()),
                                             static_cast<int64_t>(inner) }));

            std::vector<int32_t> expected(outer * order.size() * inner);
            for (size_t outerIndex = 0; outerIndex < outer; ++outerIndex)
            {
                for (size_t position = 0; position < order.size(); ++position)
                {
                    for (size_t innerIndex = 0; innerIndex < inner; ++innerIndex)
                    {
                        expected[(outerIndex * order.size() + position) * inner + innerIndex] = static_cast<int32_t>(
                            (outerIndex * axisSize + static_cast<size_t>(order[position])) * inner + innerIndex);
                    }
                }
            }
            CHECK_EQ(result.flatten().get<std::vector<int32_t>>(), expected);
        }

        SUBCASE("Element counts spanning several CUDA blocks")
        {
            constexpr size_t rows = 401; // 1604 elements: several 256-thread blocks, with a partial one.
            std::vector<float> values(rows * 4);
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = static_cast<float>(i);
            }
            const Shape quaternions({ static_cast<int64_t>(rows), int64_t{ 4 } });
            Array wxyz = Array(values, Dtype::Float32(), device).reshape(quaternions);
            Array xyzw = take(wxyz, { 1, 2, 3, 0 }, 1);
            CHECK_EQ(xyzw.shape(), quaternions);

            std::vector<float> expected(rows * 4);
            constexpr size_t order[4] = { 1, 2, 3, 0 };
            for (size_t row = 0; row < rows; ++row)
            {
                for (size_t component = 0; component < 4; ++component)
                {
                    expected[row * 4 + component] = static_cast<float>(row * 4 + order[component]);
                }
            }
            CHECK_EQ(xyzw.flatten().get<std::vector<float>>(), expected);
        }
    }
}
