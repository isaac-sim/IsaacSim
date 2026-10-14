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
#include <chrono>
#include <cstdint>
#include <future>
#include <stdexcept>
#include <utility>
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
            const DLTensor tensor = toDLPackBorrowed(array);

            CHECK_EQ(tensor.dtype.code, static_cast<uint8_t>(mapping.code));
            CHECK_EQ(tensor.dtype.bits, mapping.bits);
            CHECK_EQ(tensor.dtype.lanes, 1);

            Array result = fromDLPackBorrowed(tensor);
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

    TEST_CASE("fromDLPackBorrowed() imports a view with a byte offset")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array source(std::vector<int32_t>{ 10, 20, 30, 40 }, Dtype::Int32(), device);
        std::vector<int64_t> shape{ 3 };
        DLTensor tensor = toDLPackBorrowed(source);
        tensor.shape = shape.data();
        tensor.byte_offset = sizeof(int32_t);

        Array array = fromDLPackBorrowed(tensor);
        CHECK_EQ(array.shape(), Shape({ 3 }));
        CHECK_EQ(array.dtype(), Dtype::Int32());
        CHECK_EQ(array.device(), device);
        CHECK_EQ(array.get<std::vector<int32_t>>(), std::vector<int32_t>{ 20, 30, 40 });

        array.set(std::vector<int32_t>{ 2, 3, 4 });
        CHECK_EQ(source.get<std::vector<int32_t>>(), std::vector<int32_t>{ 10, 2, 3, 4 });
    }

    TEST_CASE("Managed exports round-trip an Array view offset")
    {
        Array source = Array(std::vector<std::vector<int32_t>>{ { 1, 2 }, { 3, 4 } });
        Array row = source.at(1);
        DLManagedTensorVersioned* managed = toDLPackVersioned(row);

        // The export resolves the view offset into `data`, so the re-imported Array addresses the
        // same elements from its own offset 0 rather than inheriting the exporter's offset.
        CHECK_EQ(managed->dl_tensor.data, row.data());
        // A versioned export is read-only, so the import is too: read it through the const overload.
        Array imported = fromDLPackVersioned(managed);
        CHECK_EQ(std::as_const(imported).data(), std::as_const(row).data());
        CHECK_EQ(imported.get<std::vector<int32_t>>(), std::vector<int32_t>{ 3, 4 });
    }

    // DLPack permits a non-zero byte_offset, but most consumers (warp, NumPy, PyTorch) ignore the
    // field and read straight from `data`. Exporting an offset view as base pointer + byte_offset
    // therefore hands those consumers the wrong element, so every export resolves the offset itself.
    TEST_CASE("Exports address element 0 directly rather than through byte_offset")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array source = Array(std::vector<int32_t>{ 1, 2, 3, 4, 5, 6 }, Dtype::Int32(), device).reshape(Shape({ 3, 2 }));
        Array row = source.at(2);
        REQUIRE_NE(row.offset(), 0);

        SUBCASE("toDLPackBorrowed()")
        {
            const DLTensor tensor = toDLPackBorrowed(row);
            CHECK_EQ(tensor.data, row.data());
            CHECK_EQ(tensor.byte_offset, 0);
        }

        SUBCASE("toDLPackManaged()")
        {
            DLManagedTensor* managed = toDLPackManaged(row);
            CHECK_EQ(managed->dl_tensor.data, row.data());
            CHECK_EQ(managed->dl_tensor.byte_offset, 0);
            managed->deleter(managed);
        }

        SUBCASE("toDLPackVersioned()")
        {
            DLManagedTensorVersioned* managed = toDLPackVersioned(row);
            CHECK_EQ(managed->dl_tensor.data, row.data());
            CHECK_EQ(managed->dl_tensor.byte_offset, 0);
            managed->deleter(managed);
        }
    }

    TEST_CASE("Strided tensors round-trip without copying")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array source = Array(std::vector<int32_t>{ 1, 2, 3, 4, 5, 6 }, Dtype::Int32(), device).reshape(Shape({ 2, 3 }));
        Array transposed = transposeView(source);
        DLTensor tensor = toDLPackBorrowed(transposed);

        REQUIRE_NE(tensor.strides, nullptr);
        CHECK_EQ(tensor.strides[0], 1);
        CHECK_EQ(tensor.strides[1], 3);

        Array result = fromDLPackBorrowed(tensor);
        CHECK_EQ(result.data(), transposed.data());
        CHECK_EQ(result.strides(), std::vector<int64_t>{ 1, 3 });
        CHECK_EQ(result.get<std::vector<std::vector<int32_t>>>(),
                 std::vector<std::vector<int32_t>>{ { 1, 4 }, { 2, 5 }, { 3, 6 } });
    }

    TEST_CASE("Managed tensors retain data and metadata")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        DLManagedTensor* managed = nullptr;
        {
            Array source =
                Array(std::vector<int32_t>{ 1, 2, 3, 4, 5, 6 }, Dtype::Int32(), device).reshape(Shape({ 2, 3 }));
            managed = toDLPackManaged(transposeView(source));
        }

        REQUIRE_NE(managed, nullptr);
        REQUIRE_NE(managed->deleter, nullptr);
        REQUIRE_NE(managed->dl_tensor.shape, nullptr);
        REQUIRE_NE(managed->dl_tensor.strides, nullptr);
        CHECK_EQ(managed->dl_tensor.shape[0], 3);
        CHECK_EQ(managed->dl_tensor.shape[1], 2);
        CHECK_EQ(managed->dl_tensor.strides[0], 1);
        CHECK_EQ(managed->dl_tensor.strides[1], 3);

        Array imported = fromDLPackManaged(managed);
        CHECK_EQ(imported.get<std::vector<std::vector<int32_t>>>(),
                 std::vector<std::vector<int32_t>>{ { 1, 4 }, { 2, 5 }, { 3, 6 } });
    }

    TEST_CASE("Versioned managed tensors propagate read-only state")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        DLManagedTensorVersioned* managed = nullptr;
        {
            Array source =
                Array(std::vector<int32_t>{ 1, 2, 3, 4, 5, 6 }, Dtype::Int32(), device).reshape(Shape({ 2, 3 }));
            managed = toDLPackVersioned(transposeView(source));
        }

        REQUIRE_NE(managed, nullptr);
        CHECK_EQ(managed->version.major, DLPACK_MAJOR_VERSION);
        CHECK_EQ(managed->version.minor, DLPACK_MINOR_VERSION);
        CHECK_NE(managed->flags & DLPACK_FLAG_BITMASK_READ_ONLY, 0);
        REQUIRE_NE(managed->dl_tensor.shape, nullptr);
        REQUIRE_NE(managed->dl_tensor.strides, nullptr);
        CHECK_EQ(managed->dl_tensor.shape[0], 3);
        CHECK_EQ(managed->dl_tensor.strides[1], 3);

        Array imported = fromDLPackVersioned(managed);
        CHECK_FALSE(imported.isWriteable());
        CHECK_THROWS_AS(imported.data(), std::runtime_error);
        CHECK_EQ(imported.get<std::vector<std::vector<int32_t>>>(),
                 std::vector<std::vector<int32_t>>{ { 1, 4 }, { 2, 5 }, { 3, 6 } });
    }

    TEST_CASE("Managed tensor validation")
    {
        // Distinct names, so a bare nullptr needs no cast to pick an overload.
        CHECK_THROWS_AS(fromDLPackManaged(nullptr), std::invalid_argument);
        CHECK_THROWS_AS(fromDLPackVersioned(nullptr), std::invalid_argument);

        Array source(std::vector<float>{ 1.f });
        DLManagedTensor* managed = toDLPackManaged(source);
        CHECK_THROWS_AS(fromDLPackManaged(managed, 1), std::invalid_argument);
        // The failed import consumed the managed tensor and called its deleter.
    }

    // The scenario behind a reusable staging buffer: a provider publishes its buffer, the consumer
    // keeps the tensor, and the provider comes back on the same thread to refill it. Blocking there
    // could never succeed, because only this thread can drop the consumer's reference.
    TEST_CASE("A retained export does not let the publishing thread block on its own buffer")
    {
        Array source(std::vector<int32_t>{ 1, 2, 3 });
        DLManagedTensorVersioned* managed = toDLPackVersioned(source);

        CHECK_FALSE(tryAcquireWrite(source).has_value());
        CHECK_THROWS_AS(acquireWrite(source), std::runtime_error);

        managed->deleter(managed);
        CHECK_UNARY(tryAcquireWrite(source).has_value());
    }

    TEST_CASE("Managed export retains read admission until the consumer releases it")
    {
        using namespace std::chrono_literals;

        Array source(std::vector<int32_t>{ 1, 2, 3 });
        DLManagedTensor* managed = toDLPackManaged(source);
        std::promise<void> writerStarted;
        auto writerStartedFuture = writerStarted.get_future();
        auto writer = std::async(std::launch::async,
                                 [&]
                                 {
                                     writerStarted.set_value();
                                     ArrayWriteAccess access = acquireWrite(source);
                                     static_cast<int32_t*>(access.data())[0] = 7;
                                 });
        writerStartedFuture.wait();
        CHECK_EQ(writer.wait_for(20ms), std::future_status::timeout);
        managed->deleter(managed);
        CHECK_EQ(writer.wait_for(1s), std::future_status::ready);
        writer.get();
        CHECK_EQ(source.get<std::vector<int32_t>>(), std::vector<int32_t>{ 7, 2, 3 });
    }

    TEST_CASE("Scalar and empty tensors preserve their shapes")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Scalar")
        {
            Array source(1.5f, Dtype::Float32(), device);
            DLTensor tensor = toDLPackBorrowed(source);

            Array array = fromDLPackBorrowed(tensor);
            CHECK_EQ(array.shape(), Shape());
            CHECK_EQ(array.device(), device);
            CHECK_EQ(array.item<float>(), doctest::Approx(1.5f));

            tensor = toDLPackBorrowed(array);
            CHECK_EQ(tensor.ndim, 0);
            CHECK_EQ(tensor.shape, nullptr);
            CHECK_EQ(tensor.strides, nullptr);
        }

        SUBCASE("Empty")
        {
            Array source(std::vector<float>{}, Dtype::Float32(), device);
            source = source.reshape(Shape({ 0, 3 }));
            DLTensor tensor = toDLPackBorrowed(source);

            Array array = fromDLPackBorrowed(tensor);
            CHECK_EQ(array.shape(), Shape({ 0, 3 }));
            CHECK_EQ(array.device(), device);
            CHECK_EQ(array.size(), 0);
            CHECK_EQ(array.data(), nullptr);

            tensor = toDLPackBorrowed(array);
            CHECK_EQ(tensor.data, nullptr);
            CHECK_EQ(tensor.ndim, 2);
        }
    }

    TEST_CASE("fromDLPackBorrowed() validates layout and metadata")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        Array source(std::vector<float>(6), Dtype::Float32(), device);
        std::vector<int64_t> shape{ 2, 3 };

        SUBCASE("Explicit C-contiguous strides")
        {
            std::vector<int64_t> strides{ 3, 1 };
            DLTensor tensor = toDLPackBorrowed(source);
            tensor.ndim = 2;
            tensor.shape = shape.data();
            tensor.strides = strides.data();
            CHECK_NOTHROW(fromDLPackBorrowed(tensor));
            CHECK_NOTHROW(fromDLPack(tensor));
        }

        SUBCASE("Non-contiguous strides")
        {
            std::vector<int64_t> strides{ 1, 2 };
            DLTensor tensor = toDLPackBorrowed(source);
            tensor.ndim = 2;
            tensor.shape = shape.data();
            tensor.strides = strides.data();
            Array result = fromDLPackBorrowed(tensor);
            CHECK_EQ(result.strides(), strides);
            CHECK_FALSE(result.isContiguous());
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
        }

        SUBCASE("Irrelevant negative strides")
        {
            std::vector<int64_t> strides{ -99, 1 };
            shape = { 1, 3 };
            DLTensor tensor = toDLPack(source);
            tensor.ndim = 2;
            tensor.shape = shape.data();
            tensor.strides = strides.data();
            CHECK_EQ(fromDLPack(tensor).strides(), std::vector<int64_t>{ 3, 1 });
            shape[0] = 0;
            strides[1] = -1;
            CHECK_EQ(fromDLPack(tensor).size(), 0);
        }

        SUBCASE("Extreme empty shape")
        {
            shape = { 0, std::numeric_limits<int64_t>::max(), 3 };
            Array empty = Array::fromBuffer({}, Shape(shape), Dtype::Float32());
            CHECK_EQ(empty.size(), 0);
            CHECK(empty.isContiguous());
            CHECK_EQ(fromDLPack(toDLPack(empty)).shape(), Shape(shape));
            CHECK_EQ(fromDLPackBorrowed(toDLPack(empty)).size(), 0);
        }

        SUBCASE("Negative strides")
        {
            std::vector<int64_t> strides{ 3, -1 };
            DLTensor tensor = toDLPackBorrowed(source);
            tensor.ndim = 2;
            tensor.shape = shape.data();
            tensor.strides = strides.data();
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);
            CHECK_THROWS_AS(fromDLPack(tensor), std::invalid_argument);
        }

        SUBCASE("Unsupported dtype")
        {
            DLTensor tensor = toDLPackBorrowed(source);
            tensor.dtype = { static_cast<uint8_t>(kDLFloat), 16, 1 };
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);
            tensor.dtype = { static_cast<uint8_t>(kDLFloat), 32, 2 };
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);
        }

        SUBCASE("Unsupported device")
        {
            DLTensor tensor = toDLPackBorrowed(source);
            tensor.device = { kDLROCM, 0 };
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);
            tensor.device = { kDLCUDA, -1 };
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);
        }

        SUBCASE("Invalid pointers")
        {
            DLTensor tensor = toDLPackBorrowed(source);
            int64_t* shapeData = tensor.shape;
            tensor.shape = nullptr;
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);

            tensor.shape = shapeData;
            tensor.data = nullptr;
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);
        }

        SUBCASE("Invalid dimensions")
        {
            DLTensor tensor = toDLPackBorrowed(source);
            shape[0] = -1;
            tensor.shape = shape.data();
            CHECK_THROWS_AS(fromDLPackBorrowed(tensor), std::invalid_argument);
        }
    }
}
