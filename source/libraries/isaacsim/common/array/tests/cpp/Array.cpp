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

#include "details/CudaRuntime.hpp"

#include <doctest/doctest.h>
#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/common/array/Functions.hpp>

#include <chrono>
#include <cstring>
#include <future>
#include <limits>
#include <optional>
#include <stdexcept>

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

class ProtectedArray : public Array
{
public:
    using Array::Array;

    ProtectedArray(Array&& other) : Array(std::move(other))
    {
    }

    size_t getProtectedOffset() const
    {
        return m_offset;
    }

    std::shared_ptr<std::byte[]> getProtectedData() const
    {
        return buffer();
    }
};

TEST_SUITE("Array")
{

    TEST_CASE("Array::Array()")
    {
        auto dtype =
            GENERATE(Dtype::Bool(), Dtype::Int8(), Dtype::Int16(), Dtype::Int32(), Dtype::Int64(), Dtype::UInt8(),
                     Dtype::UInt16(), Dtype::UInt32(), Dtype::UInt64(), Dtype::Float32(), Dtype::Float64());
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array array(0);

        SUBCASE("Scalar: 0-dim shape")
        {
            array = Array(1, dtype, device);
            // Check attributes
            CHECK_EQ(array.ndim(), 0);
            CHECK_EQ(array.size(), 1);
            CHECK_EQ(array.nbytes(), dtype.size());
            CHECK_EQ(array.dtype(), dtype);
            CHECK_EQ(array.device(), device);
            CHECK_EQ(array.shape(), Shape());
            CHECK_EQ(array.strides(), std::vector<int64_t>{});
            CHECK_UNARY(array.isContiguous());
        }

        SUBCASE("1D Vector: 1-dim shape")
        {
            array = Array(std::vector<int>{ 0, 1, 2 }, dtype, device);
            // Check attributes
            CHECK_EQ(array.ndim(), 1);
            CHECK_EQ(array.size(), 3);
            CHECK_EQ(array.nbytes(), 3 * dtype.size());
            CHECK_EQ(array.dtype(), dtype);
            CHECK_EQ(array.device(), device);
            CHECK_EQ(array.shape(), Shape(3));
            CHECK_EQ(array.strides(), std::vector<int64_t>{ 1 });
            CHECK_UNARY(array.isContiguous());
        }

        SUBCASE("2D Vector: 2-dim shape")
        {
            array = Array(std::vector<std::vector<float>>{ { 0.f, 1.f, 2.f }, { 3.f, 4.f, 5.f } }, dtype, device);
            // Check attributes
            CHECK_EQ(array.ndim(), 2);
            CHECK_EQ(array.size(), 6);
            CHECK_EQ(array.nbytes(), 6 * dtype.size());
            CHECK_EQ(array.dtype(), dtype);
            CHECK_EQ(array.device(), device);
            CHECK_EQ(array.shape(), Shape({ 2, 3 }));
            CHECK_EQ(array.strides(), std::vector<int64_t>{ 3, 1 });
            CHECK_UNARY(array.isContiguous());
        }
    }

    TEST_CASE("Array::Array(const Array& other) / Array::operator=(const Array& other)")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Copy-construction shares storage: mutation propagates both ways")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result(array);
            CHECK_EQ(result.shape(), array.shape());
            CHECK_EQ(result.dtype(), array.dtype());
            CHECK_EQ(result.device(), array.device());
            CHECK_EQ(result.buffer().get(), array.buffer().get());
            CHECK_EQ(result.data(), array.data());

            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });

            array.set(std::vector<float>{ 100.f, 200.f, 300.f });
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 100.f, 200.f, 300.f });
        }

        SUBCASE("Copy-assignment shares storage: mutation propagates both ways")
        {
            ProtectedArray source(
                std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            ProtectedArray array = source.at(1); // Shares data with a non-zero offset
            ProtectedArray result(std::vector<double>{ 0.0 }, Dtype::Float64(), device);
            result = array;
            CHECK_EQ(result.shape(), array.shape());
            CHECK_EQ(result.dtype(), array.dtype());
            CHECK_EQ(result.device(), array.device());
            CHECK_EQ(result.getProtectedOffset(), array.getProtectedOffset());
            CHECK_EQ(result.buffer().get(), array.buffer().get());

            result.set(std::vector<float>{ 30.f, 40.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 30.f, 40.f });
        }

        SUBCASE("Keeps the buffer alive after the source is destroyed")
        {
            std::optional<Array> result;
            {
                Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
                result = array;
            }
            CHECK_EQ(result->get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }

        SUBCASE("Preserves the offset of a view instead of compacting it")
        {
            ProtectedArray array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            ProtectedArray subarray = array.at(1); // Shares data with a non-zero offset
            CHECK_EQ(subarray.getProtectedOffset(), 8);
            ProtectedArray result(subarray);
            CHECK_EQ(result.getProtectedOffset(), 8);
            CHECK_EQ(result.buffer().get(), array.buffer().get());

            CHECK_EQ(result.shape(), subarray.shape());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 3.f, 4.f });
            result.set(std::vector<float>{ 30.f, 40.f });
            CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                     (std::vector<std::vector<float>>{ { 1.f, 2.f }, { 30.f, 40.f } }));
        }

        SUBCASE("Self-assignment leaves the array unchanged")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            const void* data = array.data();
            Array& reference = array;
            array = reference;
            CHECK_EQ(array.data(), data);
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }
    }

    TEST_CASE("Array::at()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("1D: positive/negative index returns scalar")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            // Check attributes
            Array subarray = array.at(0);
            CHECK_EQ(subarray.ndim(), 0);
            CHECK_EQ(subarray.size(), 1);
            CHECK_EQ(subarray.nbytes(), 4);
            CHECK_EQ(subarray.dtype(), Dtype::Float32());
            CHECK_EQ(subarray.device(), device);
            CHECK_EQ(subarray.shape(), Shape());
            // Check values
            // - Positive index
            CHECK_EQ(array.at(0).get<float>(), doctest::Approx(1.f));
            CHECK_EQ(array.at(1).get<float>(), doctest::Approx(2.f));
            CHECK_EQ(array.at(2).get<float>(), doctest::Approx(3.f));
            // - Negative index
            CHECK_EQ(array.at(-1).get<int32_t>(), 3);
            CHECK_EQ(array.at(-2).get<int32_t>(), 2);
            CHECK_EQ(array.at(-3).get<int32_t>(), 1);
            // Consecutive calls to at()
            CHECK_EQ(array.at(0).item<uint64_t>(), 1);
            CHECK_THROWS_AS(array.at(0).at(0), std::out_of_range);
        }

        SUBCASE("2D: positive/negative index along axis 0 returns 1D array")
        {
            Array array(std::vector<std::vector<int32_t>>{ { 1, 2, 3 }, { 4, 5, 6 } }, Dtype::UInt8(), device);
            // Check attributes
            Array subarray = array.at(0);
            CHECK_EQ(subarray.ndim(), 1);
            CHECK_EQ(subarray.size(), 3);
            CHECK_EQ(subarray.nbytes(), 3);
            CHECK_EQ(subarray.dtype(), Dtype::UInt8());
            CHECK_EQ(subarray.device(), device);
            CHECK_EQ(subarray.shape(), Shape(3));
            // Check values
            // - Positive index
            CHECK_EQ(array.at(0).shape(), Shape(3));
            CHECK_EQ(array.at(0).get<std::vector<double>>(), std::vector<double>{ 1, 2, 3 });
            CHECK_EQ(array.at(1).shape(), Shape(3));
            CHECK_EQ(array.at(1).get<std::vector<double>>(), std::vector<double>{ 4, 5, 6 });
            // - Negative index
            CHECK_EQ(array.at(-1).shape(), Shape(3));
            CHECK_EQ(array.at(-1).get<std::vector<uint16_t>>(), std::vector<uint16_t>{ 4, 5, 6 });
            CHECK_EQ(array.at(-2).shape(), Shape(3));
            CHECK_EQ(array.at(-2).get<std::vector<uint16_t>>(), std::vector<uint16_t>{ 1, 2, 3 });
            // Consecutive calls to at()
            CHECK_EQ(array.at(0).get<std::vector<double>>(), std::vector<double>{ 1, 2, 3 });
            CHECK_EQ(array.at(0).at(0).get<int8_t>(), 1);
            CHECK_EQ(array.at(0).at(0).item<uint64_t>(), 1);
            CHECK_THROWS_AS(array.at(0).at(0).at(0), std::out_of_range);
        }

        SUBCASE("Exceptions: out-of-bounds index throws std::out_of_range")
        {
            Array array0d(1.f, Dtype::Float32(), device);
            CHECK_THROWS_AS(array0d.at(0), std::out_of_range);

            Array array1d(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device);
            CHECK_THROWS_AS(array1d.at(2), std::out_of_range);
            CHECK_THROWS_AS(array1d.at(-3), std::out_of_range);

            Array array2d(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            CHECK_THROWS_AS(array2d.at(2), std::out_of_range);
            CHECK_THROWS_AS(array2d.at(-3), std::out_of_range);
        }

        SUBCASE("Mutation through view propagates to original")
        {
            Array array(std::vector<float>{ 5.f, 6.f, 7.f }, Dtype::Float32(), device);
            array.at(2).set(99.f);
            CHECK_EQ(array.at(2).get<float>(), doctest::Approx(99.f));
        }
    }

    TEST_CASE("Array::reshape()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Data and attributes are preserved after reshape")
        {
            Array array(std::vector<float>{ 0.f, 1.f, 2.f, 3.f, 4.f, 5.f }, Dtype::Float32(), device);
            Array result = array.reshape(Shape({ 2, 3 }));
            CHECK_EQ(result.shape(), Shape({ 2, 3 }));
            CHECK_EQ(result.ndim(), 2);
            CHECK_EQ(result.size(), 6);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 0.f, 1.f, 2.f }, { 3.f, 4.f, 5.f } });
        }

        SUBCASE("Shared data: mutation in reshaped array propagates to original")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f, 4.f }, Dtype::Float32(), device);
            Array result = array.reshape(Shape({ 2, 2 }));
            result.at(0).set(std::vector<float>{ 10.f, 20.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 3.f, 4.f });
        }

        SUBCASE("A non-contiguous array is compacted even when the shape is unchanged")
        {
            Array source(std::vector<std::vector<int32_t>>{ { 1, 2 }, { 3, 4 } }, Dtype::Int32(), device);
            Array transposed = transposeView(source);
            REQUIRE_FALSE(transposed.isContiguous());

            Array result = transposed.reshape(Shape({ 2, 2 }));
            CHECK_UNARY(result.isContiguous());
            CHECK_EQ(result.strides(), std::vector<int64_t>{ 2, 1 });
            CHECK_EQ(result.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 1, 3 }, { 2, 4 } });
        }
    }

    TEST_CASE("Array::flatten()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Data and attributes are preserved after flatten")
        {
            Array array(
                std::vector<std::vector<float>>{ { 0.f, 1.f, 2.f }, { 3.f, 4.f, 5.f } }, Dtype::Float32(), device);
            Array result = array.flatten();
            CHECK_EQ(result.shape(), Shape({ 6 }));
            CHECK_EQ(result.ndim(), 1);
            CHECK_EQ(result.size(), 6);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 0.f, 1.f, 2.f, 3.f, 4.f, 5.f });
        }

        SUBCASE("Already 1-D arrays are unchanged")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            CHECK_EQ(array.flatten().shape(), Shape({ 3 }));
        }

        SUBCASE("A strided 1-D array is compacted even though its shape already matches")
        {
            auto data = std::shared_ptr<std::byte[]>(new std::byte[6 * sizeof(int32_t)]());
            const int32_t values[6] = { 1, 2, 3, 4, 5, 6 };
            std::memcpy(data.get(), values, sizeof(values));
            Array array = Array::fromBuffer(
                data, sizeof(values), Shape({ 3 }), Dtype::Int32(), Device::Cpu(), 0, std::vector<int64_t>{ 2 });

            Array result = array.flatten();
            CHECK_UNARY(result.isContiguous());
            CHECK_EQ(result.strides(), std::vector<int64_t>{ 1 });
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 3, 5 });
        }

        SUBCASE("0-D arrays become single-element 1-D arrays")
        {
            Array array(1.f, Dtype::Float32(), device);
            CHECK_EQ(array.flatten().shape(), Shape({ 1 }));
        }

        SUBCASE("Shared data: mutation in flattened array propagates to original")
        {
            Array array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            Array result = array.flatten();
            result.at(0).set(10.f);
            CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 10.f, 2.f }, { 3.f, 4.f } });
        }
    }

    TEST_CASE("Array::toDevice()")
    {

        SUBCASE("Same device")
        {
            auto devices =
                GENERATE(std::pair{ Device::Cpu(), Device::Cpu() }, std::pair{ Device::Cuda(), Device::Cuda() });
            SKIP_IF_CUDA_UNAVAILABLE(devices.first);
            SKIP_IF_CUDA_UNAVAILABLE(devices.second);

            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), devices.first);
            Array result(0);

            // Argument: copy=true (no mutation of array)
            result = array.toDevice(devices.second, true);
            CHECK_EQ(result.device(), devices.second);
            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            // Argument: copy=false (mutation of result propagates to array)
            result = array.toDevice(devices.second, false);
            CHECK_EQ(result.device(), devices.second);
            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });
        }

        SUBCASE("Different device")
        {
            auto devices =
                GENERATE(std::pair{ Device::Cpu(), Device::Cuda() }, std::pair{ Device::Cuda(), Device::Cpu() });
            SKIP_IF_CUDA_UNAVAILABLE(devices.first);
            SKIP_IF_CUDA_UNAVAILABLE(devices.second);

            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), devices.first);
            Array result(0);

            // Argument: copy=false (no mutation of array since devices are different)
            result = array.toDevice(devices.second, false);
            CHECK_EQ(result.device(), devices.second);
            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }
    }

    TEST_CASE("Array::toDtype()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Same dtype")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result(0);

            // Argument: copy=true (no mutation of array)
            result = array.toDtype(Dtype::Float32(), true);
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });

            // Argument: copy=false (mutation of result propagates to array)
            result = array.toDtype(Dtype::Float32());
            CHECK_EQ(result.dtype(), Dtype::Float32());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });
        }

        SUBCASE("Different dtype")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result(0);

            // Argument: copy=true (no mutation of array)
            result = array.toDtype(Dtype::Int32(), true);
            CHECK_EQ(result.dtype(), Dtype::Int32());
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            result.set(std::vector<int32_t>{ 10, 20, 30 });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 10, 20, 30 });

            // Argument: copy=false (no mutation of array)
            result = array.toDtype(Dtype::Int32());
            CHECK_EQ(result.dtype(), Dtype::Int32());
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            result.set(std::vector<int32_t>{ 10, 20, 30 });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 10, 20, 30 });
        }

        SUBCASE("Intermediate type: double (source is floating-point)")
        {
            SUBCASE("float32 -> int32 truncates toward zero")
            {
                Array array(std::vector<float>{ 1.7f, -2.9f, 3.0f }, Dtype::Float32(), device);
                Array result = array.toDtype(Dtype::Int32());
                CHECK_EQ(result.dtype(), Dtype::Int32());
                CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, -2, 3 });
            }

            SUBCASE("float32 -> float64")
            {
                Array array(3.5f, Dtype::Float32(), device);
                Array result = array.toDtype(Dtype::Float64());
                CHECK_EQ(result.dtype(), Dtype::Float64());
                CHECK_EQ(result.item<double>(), doctest::Approx(3.5));
            }
        }

        SUBCASE("Intermediate type: int64_t (source is signed integer)")
        {
            SUBCASE("int32 -> float64")
            {
                Array array(std::vector<int32_t>{ 0, 1, -1 }, Dtype::Int32(), device);
                Array result = array.toDtype(Dtype::Float64());
                CHECK_EQ(result.dtype(), Dtype::Float64());
                CHECK_EQ(result.get<std::vector<double>>(), std::vector<double>{ 0.0, 1.0, -1.0 });
            }

            SUBCASE("int32 -> bool")
            {
                Array array(std::vector<int32_t>{ 0, 1, 42 }, Dtype::Int32(), device);
                Array result = array.toDtype(Dtype::Bool());
                CHECK_EQ(result.dtype(), Dtype::Bool());
                CHECK_EQ(result.get<std::vector<bool>>(), std::vector<bool>{ false, true, true });
            }

            SUBCASE("int64 -> uint64 is exact above double precision")
            {
                int64_t large = ((int64_t)1 << 53) + 1;
                Array array(large, Dtype::Int64(), device);
                Array result = array.toDtype(Dtype::UInt64());
                CHECK_EQ(result.item<uint64_t>(), static_cast<uint64_t>(large));
            }
        }

        SUBCASE("Intermediate type: uint64_t (source is unsigned integer or bool)")
        {
            SUBCASE("bool -> int32")
            {
                Array array(std::vector<bool>{ true, false, true }, Dtype::Bool(), device);
                Array result = array.toDtype(Dtype::Int32());
                CHECK_EQ(result.dtype(), Dtype::Int32());
                CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 0, 1 });
            }

            SUBCASE("uint64 -> int64 is exact above double precision")
            {
                uint64_t large = ((uint64_t)1 << 53) + 1;
                Array array(std::vector<uint64_t>{ large }, Dtype::UInt64(), device);
                Array result = array.toDtype(Dtype::Int64());
                CHECK_EQ(result.item<int64_t>(), static_cast<int64_t>(large));
            }
        }

        SUBCASE("Attributes are preserved after conversion")
        {
            Array array(std::vector<std::vector<int32_t>>{ { 1, 2, 3 }, { 4, 5, 6 } }, Dtype::Int32(), device);
            Array result(0);

            for (auto dtype :
                 { Dtype::Bool(), Dtype::Int8(), Dtype::Int16(), Dtype::Int32(), Dtype::Int64(), Dtype::UInt8(),
                   Dtype::UInt16(), Dtype::UInt32(), Dtype::UInt64(), Dtype::Float32(), Dtype::Float64() })
            {
                result = array.toDtype(dtype);
                CHECK_EQ(result.dtype(), dtype);
                CHECK_EQ(result.device(), device);
                CHECK_EQ(result.ndim(), 2);
                CHECK_EQ(result.size(), 6);
                CHECK_EQ(result.shape(), Shape({ 2, 3 }));
                CHECK_EQ(result.nbytes(), 6 * dtype.size());
            }
        }
    }

    TEST_CASE("Array::copy()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Independent of source: mutation does not propagate either way")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result = array.copy();
            CHECK_EQ(result.shape(), array.shape());
            CHECK_EQ(result.dtype(), array.dtype());
            CHECK_EQ(result.device(), array.device());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            array.set(std::vector<float>{ 100.f, 200.f, 300.f });
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });
        }

        SUBCASE("Compacts a view: offset is reset to 0")
        {
            ProtectedArray array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            ProtectedArray subarray = array.at(1); // Shares data with a non-zero offset
            CHECK_EQ(subarray.getProtectedOffset(), 8);
            CHECK_EQ(subarray.buffer().get(), array.buffer().get());
            ProtectedArray result = subarray.copy();
            CHECK_EQ(result.getProtectedOffset(), 0);
            CHECK_NE(result.buffer().get(), array.buffer().get());

            CHECK_EQ(result.shape(), subarray.shape());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 3.f, 4.f });
            result.set(std::vector<float>{ 30.f, 40.f });
            CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                     (std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }));
        }
    }

    TEST_CASE("Array::isWriteable()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Freshly allocated arrays are writeable")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            CHECK_UNARY(array.isWriteable());
            array.set(std::vector<float>{ 4.f, 5.f, 6.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 4.f, 5.f, 6.f });
        }

        SUBCASE("An empty array stays writeable whichever axis has length zero")
        {
            // A contiguous layout puts a 0 stride on every axis preceding a zero-length one, which
            // must not be mistaken for a replicated axis.
            for (const Shape& shape : { Shape({ 4, 0 }), Shape({ 0, 3 }), Shape({ 0 }) })
            {
                Array array = empty(shape, Dtype::Float32(), device);
                CHECK_UNARY(array.isWriteable());
                CHECK_NOTHROW(array.data());
                CHECK_NOTHROW(array.set(0.f));
                CHECK_UNARY(array.copy().isWriteable());
                // Arithmetic writes into a freshly allocated result of the same shape.
                CHECK_NOTHROW(negative(array));
                CHECK_NOTHROW(add(array, array));
            }
        }

        SUBCASE("A replicating broadcast view is read-only")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array view = array.broadcastView(Shape({ 2, 3 }));

            CHECK_FALSE(view.isWriteable());
            CHECK_THROWS_AS(view.set(0.f), std::runtime_error);
            CHECK_THROWS_AS(view.data(), std::runtime_error);
            // The operand must survive the rejected writes intact.
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            // Reading through the view stays available.
            CHECK_EQ(view.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 1.f, 2.f, 3.f } });
        }

        SUBCASE("A non-replicating broadcast stays writeable")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array view = array.broadcastView(Shape({ 3 }));
            CHECK_UNARY(view.isWriteable());
        }

        SUBCASE("Materializing a broadcast yields a writeable array")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array materialized = array.broadcastView(Shape({ 2, 3 }), true);
            CHECK_UNARY(materialized.isWriteable());
            materialized.set(0.f);
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }

        SUBCASE("copy() of a read-only view is writeable, and so is contiguous() when it compacts")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array view = array.broadcastView(Shape({ 2, 3 }));
            CHECK_UNARY(view.copy().isWriteable());
            // Only because the replicating strides force an allocation -- contiguous() decides on
            // layout alone, so it does not lift the restriction from an already-contiguous view.
            REQUIRE_FALSE(view.isContiguous());
            CHECK_UNARY(view.contiguous().isWriteable());
        }

        SUBCASE("Read-only-ness follows the strides through derived views")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array view = array.broadcastView(Shape({ 2, 3 }));

            // transposeView() and clone() keep the aliasing strides, so they stay read-only.
            CHECK_FALSE(transposeView(view).isWriteable());
            CHECK_FALSE(view.clone().isWriteable());
        }

        SUBCASE("Views created from a read-only array inherit the lock")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array view = array.broadcastView(Shape({ 2, 3 }));

            // at() drops the replicated axis, so the slice no longer aliases itself -- but it still
            // aliases the operand, and the lock follows it.
            Array row = view.at(1);
            CHECK_FALSE(row.isWriteable());
            CHECK_THROWS_AS(row.set(0.f), std::runtime_error);
            CHECK_FALSE(row.reshape(Shape({ 3 })).isWriteable());
            CHECK_FALSE(transposeView(row).isWriteable());
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            // copy() breaks the chain by allocating an independent buffer.
            Array independent = row.copy();
            CHECK_UNARY(independent.isWriteable());
            independent.set(0.f);
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }

        SUBCASE("Moving a read-only array to another device or dtype yields writeable storage")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array view = array.broadcastView(Shape({ 2, 3 }));
            CHECK_UNARY(view.toDtype(Dtype::Int32()).isWriteable());
        }

        SUBCASE("A transposed view of a plain array stays writeable")
        {
            Array array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            Array view = transposeView(array);
            REQUIRE_UNARY(view.isWriteable());
            view.set(0.f);
            CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 0.f, 0.f }, { 0.f, 0.f } });
        }
    }

    TEST_CASE("Array::contiguous()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Already contiguous arrays share storage")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result = array.contiguous();
            CHECK_UNARY(result.isContiguous());
            CHECK_EQ(result.strides(), std::vector<int64_t>{ 1 });
            CHECK_EQ(result.buffer().get(), array.buffer().get());
        }

        SUBCASE("Strided arrays are compacted in logical order")
        {
            Array source(std::vector<std::vector<int32_t>>{ { 1, 2, 3 }, { 4, 5, 6 } }, Dtype::Int32(), device);
            Array array = transposeView(source);

            Array result = array.contiguous();
            CHECK_UNARY(result.isContiguous());
            CHECK_EQ(result.strides(), std::vector<int64_t>{ 2, 1 });
            CHECK_NE(result.buffer().get(), array.buffer().get());
            CHECK_EQ(result.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 1, 4 }, { 2, 5 }, { 3, 6 } });
        }

        SUBCASE("A contiguous read-only array is returned unchanged and stays read-only")
        {
            // Layout is the only thing contiguous() considers, so it has no reason to allocate here
            // and cannot lift the inherited restriction. Slicing the replicating axis off a broadcast
            // is what produces a view that is contiguous and read-only at the same time.
            Array source(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array view = source.broadcastView(Shape({ 2, 3 }));
            Array slice = view.at(0);
            REQUIRE_UNARY(slice.isContiguous());
            REQUIRE_FALSE(slice.isWriteable());

            Array result = slice.contiguous();
            CHECK_EQ(result.buffer().get(), slice.buffer().get());
            CHECK_FALSE(result.isWriteable());
            CHECK_THROWS_AS(result.set(0.f), std::runtime_error);
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            // copy() is the escape hatch the read-only error message points at.
            Array writeable = slice.copy();
            CHECK_UNARY(writeable.isWriteable());
            CHECK_NE(writeable.buffer().get(), slice.buffer().get());
        }
    }

    TEST_CASE("Array::clone()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Independent of source: mutation does not propagate either way")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result = array.clone();
            CHECK_EQ(result.shape(), array.shape());
            CHECK_EQ(result.dtype(), array.dtype());
            CHECK_EQ(result.device(), array.device());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            array.set(std::vector<float>{ 100.f, 200.f, 300.f });
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });
        }

        SUBCASE("Preserves a view's offset: further indexing still works")
        {
            ProtectedArray array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            ProtectedArray subarray = array.at(1); // Shares data with a non-zero offset
            CHECK_EQ(subarray.getProtectedOffset(), 8);
            CHECK_EQ(subarray.buffer().get(), array.buffer().get());
            ProtectedArray result = subarray.clone();
            CHECK_EQ(result.getProtectedOffset(), 8);
            CHECK_NE(result.buffer().get(), array.buffer().get());

            CHECK_EQ(result.shape(), subarray.shape());
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 3.f, 4.f });
            result.set(std::vector<float>{ 30.f, 40.f });
            CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                     (std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }));
        }
    }

    TEST_CASE("Array::broadcastView()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Same shape")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            Array result(0);

            // Argument: copy=true (no mutation of array)
            result = array.broadcastView(Shape({ 3 }), true);
            CHECK_EQ(result.shape(), Shape({ 3 }));
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });

            // Argument: copy=false (mutation of result propagates to array)
            result = array.broadcastView(Shape({ 3 }), false);
            CHECK_EQ(result.shape(), Shape({ 3 }));
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
            result.set(std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });
            CHECK_EQ(result.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 30.f });
        }

        SUBCASE("Broadcasting rules")
        {
            Array array(0);
            Array result(0);

            // Scalar (0-dim) broadcast to N-dim
            array = Array(7.f, Dtype::Float32(), device);
            result = array.broadcastView(Shape({ 2, 3 }));
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.shape(), Shape({ 2, 3 }));
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 7.f, 7.f, 7.f }, { 7.f, 7.f, 7.f } });

            // 1-D array broadcast to 2-D (trailing dimension matches)
            array = Array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            result = array.broadcastView(Shape({ 2, 3 }));
            CHECK_EQ(result.shape(), Shape({ 2, 3 }));
            CHECK_EQ(result.strides(), std::vector<int64_t>{ 0, 1 });
            CHECK_EQ(result.buffer().get(), array.buffer().get());
            CHECK_FALSE(result.isContiguous());
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 1.f, 2.f, 3.f } });

            // 2-D array with a size-1 axis broadcast to a larger 2-D shape
            array = Array(std::vector<std::vector<float>>{ { 1.f }, { 2.f } }, Dtype::Float32(), device);
            result = array.broadcastView(Shape({ 2, 3 }));
            CHECK_EQ(result.shape(), Shape({ 2, 3 }));
            CHECK_EQ(result.strides(), std::vector<int64_t>{ 1, 0 });
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 1.f, 1.f }, { 2.f, 2.f, 2.f } });

            // 2-D array with a leading size-1 axis broadcast to more rows
            array = Array(std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f } }, Dtype::Float32(), device);
            result = array.broadcastView(Shape({ 4, 3 }));
            CHECK_EQ(result.shape(), Shape({ 4, 3 }));
            CHECK_EQ(result.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{
                         { 1.f, 2.f, 3.f }, { 1.f, 2.f, 3.f }, { 1.f, 2.f, 3.f }, { 1.f, 2.f, 3.f } });

            // 1-D array broadcast to 3-D adds leading broadcast axes
            array = Array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            result = array.broadcastView(Shape({ 2, 2, 3 }));
            CHECK_EQ(result.shape(), Shape({ 2, 2, 3 }));
            for (int64_t i = 0; i < 2; ++i)
            {
                for (int64_t j = 0; j < 2; ++j)
                {
                    CHECK_EQ(result.at(i).at(j).get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
                }
            }

            // Non-contiguous broadcast of a wider dtype (element size > 4 bytes)
            array = Array(std::vector<std::vector<double>>{ { 1.0 }, { 2.0 } }, Dtype::Float64(), device);
            result = array.broadcastView(Shape({ 2, 3 }));
            CHECK_EQ(result.dtype(), Dtype::Float64());
            CHECK_EQ(result.get<std::vector<std::vector<double>>>(),
                     std::vector<std::vector<double>>{ { 1.0, 1.0, 1.0 }, { 2.0, 2.0, 2.0 } });
        }

        SUBCASE("Wrong shapes raise")
        {
            Array array(
                std::vector<std::vector<float>>{ { 1.f, 2.f, 3.f }, { 4.f, 5.f, 6.f } }, Dtype::Float32(), device);
            CHECK_THROWS_AS(array.broadcastView(Shape({ 2, 4 })), std::invalid_argument);
            CHECK_THROWS_AS(array.broadcastView(Shape({ 2, 2 })), std::invalid_argument);
            CHECK_THROWS_AS(array.broadcastView(Shape({ 3, 3 })), std::invalid_argument);
        }
    }

    TEST_CASE("Array::set()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("A source that aliases the destination is read before it is overwritten")
        {
            Array array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            array.set(transposeView(array));
            CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 3.f }, { 2.f, 4.f } });
        }

        SUBCASE("Assigning an array to itself leaves it unchanged")
        {
            Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
            array.set(array);
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });

            array.set(array.broadcastView(Shape({ 3 })));
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 1.f, 2.f, 3.f });
        }

        SUBCASE("A strided destination is written through its strides")
        {
            Array array(std::vector<std::vector<int32_t>>{ { 1, 2, 3 }, { 4, 5, 6 } }, Dtype::Int32(), device);
            Array view = transposeView(array);
            view.set(
                Array(std::vector<std::vector<int32_t>>{ { 10, 40 }, { 20, 50 }, { 30, 60 } }, Dtype::Int32(), device));
            CHECK_EQ(array.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 10, 20, 30 }, { 40, 50, 60 } });
        }

        SUBCASE("A shape-incompatible source is rejected even when the destination is empty")
        {
            Array array = empty(Shape({ 0, 3 }), Dtype::Float32(), device);
            CHECK_THROWS_AS(
                array.set(Array(std::vector<float>{ 1.f, 2.f }, Dtype::Float32(), device)), std::invalid_argument);
        }
    }

    TEST_CASE("Array::fromBuffer()")
    {
        SUBCASE("Aliases a contiguous 2-D buffer without copying")
        {
            auto data = std::shared_ptr<std::byte[]>(new std::byte[6 * sizeof(int32_t)]());
            const int32_t values[6] = { 1, 2, 3, 4, 5, 6 };
            std::memcpy(data.get(), values, sizeof(values));

            Array array = Array::fromBuffer(data, sizeof(values), Shape({ 2, 3 }), Dtype::Int32());
            CHECK_EQ(array.shape(), Shape({ 2, 3 }));
            CHECK_EQ(array.dtype(), Dtype::Int32());
            CHECK_EQ(array.device(), Device::Cpu());
            CHECK_EQ(array.capacity(), sizeof(values));
            CHECK_EQ(array.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 1, 2, 3 }, { 4, 5, 6 } });

            // The Array shares ownership of the buffer and aliases it (no copy was made).
            CHECK_GE(data.use_count(), 2);
            CHECK_EQ(static_cast<const void*>(data.get()), array.data());

            // Mutations to the shared buffer are visible through the Array.
            const int32_t updated = 42;
            std::memcpy(data.get(), &updated, sizeof(updated));
            CHECK_EQ(array.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 42, 2, 3 }, { 4, 5, 6 } });
        }

        SUBCASE("Honors a non-zero offset into the shared buffer")
        {
            auto data = std::shared_ptr<std::byte[]>(new std::byte[4 * sizeof(float)]());
            const float values[4] = { 10.f, 20.f, 30.f, 40.f };
            std::memcpy(data.get(), values, sizeof(values));

            // Skip the first element via the byte offset.
            Array array =
                Array::fromBuffer(data, sizeof(values), Shape({ 3 }), Dtype::Float32(), Device::Cpu(), sizeof(float));
            CHECK_EQ(array.get<std::vector<float>>(), std::vector<float>{ 20.f, 30.f, 40.f });
            CHECK_EQ(array.capacity(), sizeof(values));
        }

        SUBCASE("Preserves explicit element strides")
        {
            auto data = std::shared_ptr<std::byte[]>(new std::byte[6 * sizeof(int32_t)]());
            const int32_t values[6] = { 1, 2, 3, 4, 5, 6 };
            std::memcpy(data.get(), values, sizeof(values));

            Array array = Array::fromBuffer(
                data, sizeof(values), Shape({ 3, 2 }), Dtype::Int32(), Device::Cpu(), 0, std::vector<int64_t>{ 1, 3 });
            CHECK_EQ(array.strides(), std::vector<int64_t>{ 1, 3 });
            CHECK_FALSE(array.isContiguous());
            CHECK_EQ(array.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 1, 4 }, { 2, 5 }, { 3, 6 } });

            array.set(std::vector<std::vector<int32_t>>{ { 10, 40 }, { 20, 50 }, { 30, 60 } });
            CHECK_EQ(*reinterpret_cast<int32_t*>(data.get()), 10);
            CHECK_EQ(array.contiguous().get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 10, 40 }, { 20, 50 }, { 30, 60 } });
        }

        SUBCASE("Rejects invalid strides")
        {
            auto data = std::shared_ptr<std::byte[]>(new std::byte[4 * sizeof(float)]());
            CHECK_THROWS_AS(Array::fromBuffer(data, 4 * sizeof(float), Shape({ 2, 2 }), Dtype::Float32(), Device::Cpu(),
                                              0, std::vector<int64_t>{ 1 }),
                            std::invalid_argument);
            CHECK_THROWS_AS(Array::fromBuffer(data, 4 * sizeof(float), Shape({ 2, 2 }), Dtype::Float32(), Device::Cpu(),
                                              0, std::vector<int64_t>{ 2, -1 }),
                            std::invalid_argument);
        }

        SUBCASE("Rejects a strided span that overflows size_t")
        {
            auto data = std::shared_ptr<std::byte[]>(new std::byte[4 * sizeof(float)]());
            const int64_t hugeStride = int64_t{ 1 } << 62;
            CHECK_THROWS_AS(Array::fromBuffer(data, 4 * sizeof(float), Shape({ 2, 2 }), Dtype::Float32(), Device::Cpu(),
                                              0, std::vector<int64_t>{ hugeStride, 1 }),
                            std::overflow_error);
        }

        SUBCASE("Rejects views outside the declared allocation")
        {
            auto data = std::shared_ptr<std::byte[]>(new std::byte[4 * sizeof(float)]());
            CHECK_THROWS_AS(
                Array::fromBuffer(data, 3 * sizeof(float), Shape({ 4 }), Dtype::Float32()), std::invalid_argument);
            CHECK_THROWS_AS(Array::fromBuffer(data, 4 * sizeof(float), Shape({ 2, 2 }), Dtype::Float32(), Device::Cpu(),
                                              0, std::vector<int64_t>{ 3, 1 }),
                            std::invalid_argument);
            CHECK_THROWS_AS(Array::fromBuffer(data, 4 * sizeof(float), Shape({ 1 }), Dtype::Float32(), Device::Cpu(),
                                              4 * sizeof(float)),
                            std::invalid_argument);
        }

        SUBCASE("Rejects a null buffer for non-empty views")
        {
            std::shared_ptr<std::byte[]> data;
            CHECK_THROWS_AS(
                Array::fromBuffer(data, sizeof(float), Shape({ 1 }), Dtype::Float32()), std::invalid_argument);
            CHECK_NOTHROW(Array::fromBuffer(data, 0, Shape({ 0 }), Dtype::Float32()));
        }

        SUBCASE("Rejects a host pointer declared as CUDA storage")
        {
            const Device device = Device::Cuda();
            SKIP_IF_CUDA_UNAVAILABLE(device);
            auto data = std::shared_ptr<std::byte[]>(new std::byte[sizeof(float)]);
            CHECK_THROWS_AS(Array::fromBuffer(std::move(data), sizeof(float), Shape({ 1 }), Dtype::Float32(), device),
                            std::invalid_argument);
        }

        SUBCASE("A buffer round-trip preserves storage access serialization")
        {
            using namespace std::chrono_literals;

            Array source(std::vector<int32_t>{ 1, 2, 3 });
            Array alias =
                Array::fromBuffer(source.buffer(), source.capacity(), source.shape(), source.dtype(), source.device());
            ArrayReadAccess reader = acquireRead(source);
            auto writer = std::async(std::launch::async, [&] { return acquireWrite(alias); });
            CHECK_EQ(writer.wait_for(20ms), std::future_status::timeout);
            reader.release();
            CHECK_EQ(writer.wait_for(1s), std::future_status::ready);
            ArrayWriteAccess writeAccess = writer.get();
            CHECK_EQ(writeAccess.data(), alias.data());
            CHECK_THROWS_AS(Array::fromBuffer(source.buffer(), sizeof(int32_t), Shape({ 2 }), Dtype::Int32()),
                            std::invalid_argument);
        }

        SUBCASE("An empty array with unbounded strides rejects an overflowing at()")
        {
            // An empty array reaches no element, so its span is 0 and the constructor's
            // offset-plus-span check cannot bound its strides. Indexing axis 0 still scales by
            // that stride, so at() has to reject the overflow itself rather than wrapping to a
            // pointer outside the allocation.
            auto data = std::shared_ptr<std::byte[]>(new std::byte[4 * sizeof(float)]());
            const int64_t hugeStride = static_cast<int64_t>(std::numeric_limits<size_t>::max() / sizeof(float));
            Array empty = Array::fromBuffer(data, 4 * sizeof(float), Shape({ 3, 0 }), Dtype::Float32(), Device::Cpu(),
                                            0, std::vector<int64_t>{ hugeStride, 1 });
            REQUIRE_EQ(empty.size(), 0);
            CHECK_THROWS_AS(empty.at(2), std::overflow_error);
            // Index 0 scales the stride to nothing, so it stays a valid (empty) view.
            CHECK_NOTHROW(empty.at(0));
        }

        SUBCASE("Preserves explicit element strides on a device buffer")
        {
            SKIP_IF_CUDA_UNAVAILABLE(Device::Cuda());

            Array source(std::vector<std::vector<int32_t>>{ { 1, 2, 3 }, { 4, 5, 6 } }, Dtype::Int32(), Device::Cuda());
            Array array = Array::fromBuffer(source.buffer(), source.capacity(), Shape({ 3, 2 }), Dtype::Int32(),
                                            Device::Cuda(), 0, std::vector<int64_t>{ 1, 3 });
            CHECK_EQ(array.strides(), std::vector<int64_t>{ 1, 3 });
            CHECK_FALSE(array.isContiguous());
            CHECK_EQ(array.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 1, 4 }, { 2, 5 }, { 3, 6 } });

            array.set(std::vector<std::vector<int32_t>>{ { 10, 40 }, { 20, 50 }, { 30, 60 } });
            CHECK_EQ(source.get<std::vector<std::vector<int32_t>>>(),
                     std::vector<std::vector<int32_t>>{ { 10, 20, 30 }, { 40, 50, 60 } });
        }

        SUBCASE("Aliases a device buffer without copying")
        {
            SKIP_IF_CUDA_UNAVAILABLE(Device::Cuda());

            Array source(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), Device::Cuda());
            std::shared_ptr<std::byte[]> data = source.buffer();

            Array array = Array::fromBuffer(data, source.capacity(), Shape({ 2, 2 }), Dtype::Float32(), Device::Cuda());
            CHECK_EQ(array.device(), Device::Cuda());
            CHECK_EQ(static_cast<const void*>(data.get()), array.data());
            CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } });

            // Mutations through the alias are visible in the source (no copy was made).
            array.at(0).set(std::vector<float>{ 10.f, 20.f });
            CHECK_EQ(source.get<std::vector<std::vector<float>>>(),
                     std::vector<std::vector<float>>{ { 10.f, 20.f }, { 3.f, 4.f } });

            // The buffer's own deleter is preserved: the device memory outlives the source Array.
            Array kept = Array::fromBuffer(data, source.capacity(), Shape({ 4 }), Dtype::Float32(), Device::Cuda());
            source = Array(0);
            CHECK_EQ(kept.get<std::vector<float>>(), std::vector<float>{ 10.f, 20.f, 3.f, 4.f });

            // Honors a non-zero offset into the shared device buffer.
            Array offsetArray =
                Array::fromBuffer(data, kept.capacity(), Shape({ 3 }), Dtype::Float32(), Device::Cuda(), sizeof(float));
            CHECK_EQ(offsetArray.get<std::vector<float>>(), std::vector<float>{ 20.f, 3.f, 4.f });
        }
    }

    TEST_CASE("Array::data()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
        CHECK_UNARY(array.data() != nullptr);

        // A view's pointer is the base pointer advanced by the slice's byte offset, on both devices.
        Array subarray = array.at(1);
        CHECK_EQ(static_cast<const std::byte*>(subarray.data()),
                 static_cast<const std::byte*>(array.data()) + 2 * sizeof(float));

        // The pointer addresses the array's own device and holds the element data.
        if (device.isCuda())
        {
            float values[2] = {};
            CudaRuntime::getInstance().cudaMemcpy(values, subarray.data(), sizeof(values));
            CHECK_EQ(values[0], doctest::Approx(3.f));
            CHECK_EQ(values[1], doctest::Approx(4.f));
        }
        else
        {
            const float* values = static_cast<const float*>(subarray.data());
            CHECK_EQ(values[0], doctest::Approx(3.f));
            CHECK_EQ(values[1], doctest::Approx(4.f));
        }
    }

    TEST_CASE("Array::toString()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Reports the data and the array's own metadata")
        {
            // Scalar (0-dim)
            Array array(7, Dtype::Int32(), device);
            CHECK_EQ(array.toString(), "Array(7, shape=(), dtype='int32', device='" + device.toString() + "')");

            // 1-D
            array = Array(std::vector<int32_t>{ 1, 2, 3 }, Dtype::Int32(), device);
            CHECK_EQ(array.toString(), "Array([1, 2, 3], shape=(3), dtype='int32', device='" + device.toString() + "')");

            // 2-D
            array = Array(std::vector<std::vector<int32_t>>{ { 1, 2 }, { 3, 4 } }, Dtype::Int32(), device);
            CHECK_EQ(array.toString(),
                     "Array([[1, 2], [3, 4]], shape=(2, 2), dtype='int32', device='" + device.toString() + "')");

            // More than 2 dimensions: the data is elided
            array = Array(std::vector<int32_t>{ 1, 2, 3, 4 }, Dtype::Int32(), device).reshape(Shape({ 1, 2, 2 }));
            CHECK_EQ(array.toString(), "Array(..., shape=(1, 2, 2), dtype='int32', device='" + device.toString() + "')");
        }

        SUBCASE("A view reports only its own elements")
        {
            Array array(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 3.f, 4.f } }, Dtype::Float32(), device);
            CHECK_EQ(array.at(1).toString(), "Array([" + std::to_string(3.f) + ", " + std::to_string(4.f) +
                                                 "], shape=(2), dtype='float32', device='" + device.toString() + "')");
        }
    }

    TEST_CASE("Array::buffer()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array array(std::vector<float>{ 1.f, 2.f, 3.f }, Dtype::Float32(), device);
        std::shared_ptr<std::byte[]> buffer = array.buffer();
        CHECK_UNARY(buffer != nullptr);
        // buffer() shares ownership with the Array's storage (offset 0 for a freshly built array).
        CHECK_EQ(static_cast<const void*>(buffer.get()), array.data());

        // The buffer refers to the start of the allocation, ahead of a slice's element 0.
        Array subarray = array.at(1);
        CHECK_EQ(static_cast<const void*>(subarray.buffer().get()), static_cast<const void*>(buffer.get()));
        CHECK_EQ(static_cast<const std::byte*>(subarray.data()), buffer.get() + sizeof(float));

        // The buffer keeps the allocation alive on its own, after the last Array is destroyed.
        std::shared_ptr<std::byte[]> keepalive;
        {
            Array temporary(std::vector<float>{ 4.f, 5.f }, Dtype::Float32(), device);
            keepalive = temporary.buffer();
        }
        CHECK_UNARY(keepalive != nullptr);
        // CUDA retirement may temporarily hold another owner until the constructor's
        // asynchronous copy completes. CPU storage has no deferred retirement.
        if (device.isCpu())
        {
            CHECK_EQ(keepalive.use_count(), 1);
        }
        else
        {
            CHECK_GE(keepalive.use_count(), 1);
        }
    }
}
