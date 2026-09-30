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

#include <array>
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

namespace
{

struct DtypeMapping
{
    Dtype dtype;
    DLDataTypeCode code;
    uint8_t bits;
};

} // namespace

TEST_SUITE("DLPack")
{
    TEST_CASE("Dtype conversion round-trips every Array dtype")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        const std::array<DtypeMapping, 11> mappings{
            DtypeMapping{ Dtype::Bool(), kDLBool, 8 },      DtypeMapping{ Dtype::Int8(), kDLInt, 8 },
            DtypeMapping{ Dtype::Int16(), kDLInt, 16 },     DtypeMapping{ Dtype::Int32(), kDLInt, 32 },
            DtypeMapping{ Dtype::Int64(), kDLInt, 64 },     DtypeMapping{ Dtype::UInt8(), kDLUInt, 8 },
            DtypeMapping{ Dtype::UInt16(), kDLUInt, 16 },   DtypeMapping{ Dtype::UInt32(), kDLUInt, 32 },
            DtypeMapping{ Dtype::UInt64(), kDLUInt, 64 },   DtypeMapping{ Dtype::Float32(), kDLFloat, 32 },
            DtypeMapping{ Dtype::Float64(), kDLFloat, 64 },
        };

        for (const DtypeMapping& mapping : mappings)
        {
            Array array(std::vector<int32_t>{ 0, 1 }, mapping.dtype, device);
            const DLTensor tensor = toDLPack(array);

            CHECK_EQ(tensor.dtype.code, static_cast<uint8_t>(mapping.code));
            CHECK_EQ(tensor.dtype.bits, mapping.bits);
            CHECK_EQ(tensor.dtype.lanes, 1);

            Array result = fromDLPack(tensor);
            CHECK_EQ(result.dtype(), mapping.dtype);
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.get<std::vector<int32_t>>(), std::vector<int32_t>{ 0, 1 });
        }
    }

    TEST_CASE("toDLPack() exports borrowed metadata and data")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array array = Array(std::vector<float>{ 1.f, 2.f, 3.f, 4.f }, Dtype::Float32(), device).reshape(Shape({ 2, 2 }));
        DLTensor tensor = toDLPack(array);

        CHECK_EQ(tensor.data, array.data());
        CHECK_EQ(tensor.device.device_type, device.isCpu() ? kDLCPU : kDLCUDA);
        CHECK_EQ(tensor.device.device_id, device.isCpu() ? 0 : device.ordinal());
        CHECK_EQ(tensor.ndim, 2);
        REQUIRE_NE(tensor.shape, nullptr);
        CHECK_EQ(tensor.shape[0], 2);
        CHECK_EQ(tensor.shape[1], 2);
        CHECK_EQ(tensor.strides, nullptr);
        CHECK_EQ(tensor.byte_offset, 0);

        Array view = fromDLPack(tensor);
        view.set(std::vector<std::vector<float>>{ { 1.f, 2.f }, { 30.f, 4.f } });
        CHECK_EQ(array.get<std::vector<std::vector<float>>>(),
                 (std::vector<std::vector<float>>{ { 1.f, 2.f }, { 30.f, 4.f } }));
    }

    TEST_CASE("fromDLPack() imports a borrowed view with a byte offset")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array source(std::vector<int32_t>{ 10, 20, 30, 40 }, Dtype::Int32(), device);
        std::vector<int64_t> shape{ 3 };
        DLTensor tensor = toDLPack(source);
        tensor.shape = shape.data();
        tensor.byte_offset = sizeof(int32_t);

        Array array = fromDLPack(tensor);
        CHECK_EQ(array.shape(), Shape({ 3 }));
        CHECK_EQ(array.dtype(), Dtype::Int32());
        CHECK_EQ(array.device(), device);
        CHECK_EQ(array.get<std::vector<int32_t>>(), std::vector<int32_t>{ 20, 30, 40 });

        array.set(std::vector<int32_t>{ 2, 3, 4 });
        CHECK_EQ(source.get<std::vector<int32_t>>(), std::vector<int32_t>{ 10, 2, 3, 4 });
    }

    TEST_CASE("Scalar and empty tensors preserve their shapes")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Scalar")
        {
            Array source(1.5f, Dtype::Float32(), device);
            DLTensor tensor = toDLPack(source);

            Array array = fromDLPack(tensor);
            CHECK_EQ(array.shape(), Shape());
            CHECK_EQ(array.device(), device);
            CHECK_EQ(array.item<float>(), doctest::Approx(1.5f));

            tensor = toDLPack(array);
            CHECK_EQ(tensor.ndim, 0);
            CHECK_EQ(tensor.shape, nullptr);
            CHECK_EQ(tensor.strides, nullptr);
        }

        SUBCASE("Empty")
        {
            Array source(std::vector<float>{}, Dtype::Float32(), device);
            source = source.reshape(Shape({ 0, 3 }));
            DLTensor tensor = toDLPack(source);

            Array array = fromDLPack(tensor);
            CHECK_EQ(array.shape(), Shape({ 0, 3 }));
            CHECK_EQ(array.device(), device);
            CHECK_EQ(array.size(), 0);
            CHECK_EQ(array.data(), nullptr);

            tensor = toDLPack(array);
            CHECK_EQ(tensor.data, nullptr);
            CHECK_EQ(tensor.ndim, 2);
        }
    }

    TEST_CASE("fromDLPack() validates layout and metadata")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array source(std::vector<float>(6), Dtype::Float32(), device);
        std::vector<int64_t> shape{ 2, 3 };

        SUBCASE("Explicit C-contiguous strides")
        {
            std::vector<int64_t> strides{ 3, 1 };
            DLTensor tensor = toDLPack(source);
            tensor.ndim = 2;
            tensor.shape = shape.data();
            tensor.strides = strides.data();
            CHECK_NOTHROW(fromDLPack(tensor));
        }

        SUBCASE("Non-contiguous strides")
        {
            std::vector<int64_t> strides{ 1, 2 };
            DLTensor tensor = toDLPack(source);
            tensor.ndim = 2;
            tensor.shape = shape.data();
            tensor.strides = strides.data();
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
        }

        SUBCASE("Unsupported dtype")
        {
            DLTensor tensor = toDLPack(source);
            tensor.dtype = { static_cast<uint8_t>(kDLFloat), 16, 1 };
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
            tensor.dtype = { static_cast<uint8_t>(kDLFloat), 32, 2 };
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
        }

        SUBCASE("Unsupported device")
        {
            DLTensor tensor = toDLPack(source);
            tensor.device = { kDLROCM, 0 };
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
            tensor.device = { kDLCUDA, -1 };
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
        }

        SUBCASE("Invalid pointers")
        {
            DLTensor tensor = toDLPack(source);
            int64_t* shapeData = tensor.shape;
            tensor.shape = nullptr;
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);

            tensor.shape = shapeData;
            tensor.data = nullptr;
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
        }

        SUBCASE("Invalid dimensions")
        {
            DLTensor tensor = toDLPack(source);
            shape[0] = -1;
            tensor.shape = shape.data();
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
        }
    }
}
