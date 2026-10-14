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

#include "OvPhysxTensorHelpers.hpp"

#include <doctest/doctest.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace isaacsim::physics_engines::ovphysx;
namespace array = isaacsim::common::array;

namespace
{

// Run `callback` and return the message of the exception it throws, or "" if it does not throw.
// Lets a test pin the *specific* rejected branch by matching a substring of the guard's message.
std::string getThrownMessage(const std::function<void()>& callback)
{
    try
    {
        callback();
    }
    catch (const std::exception& exception)
    {
        return exception.what();
    }
    return {};
}

bool contains(const std::string& text, const char* substring)
{
    return text.find(substring) != std::string::npos;
}

} // namespace

//=============================================================================
// Index validation -- the engine reads a selection as int32[K] without checking it,
// so a wider dtype or a higher-rank selection must be rejected before it reaches ovphysx.
//=============================================================================
TEST_CASE("requireInt32Indices")
{
    SUBCASE("1-D int32 indices are accepted")
    {
        const array::Array indices(std::vector<int32_t>{ 0, 2, 4 });
        CHECK(getThrownMessage([&] { requireInt32Indices(indices, "read"); }).empty());
    }

    SUBCASE("empty int32 indices are accepted")
    {
        const array::Array indices = array::zeros(array::Shape(std::vector<int64_t>{ 0 }), array::Dtype::Int32());
        CHECK(getThrownMessage([&] { requireInt32Indices(indices, "read"); }).empty());
    }

    SUBCASE("a wider integer dtype is rejected")
    {
        const array::Array indices(std::vector<int64_t>{ 0, 1 });
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "read"); }), "int32 convention"));
    }

    SUBCASE("a higher-rank selection is rejected")
    {
        const array::Array indices(std::vector<std::vector<int32_t>>{ { 0, 1 }, { 2, 3 } });
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "read"); }), "must be 1-D"));
    }

    SUBCASE("a strided selection is rejected")
    {
        // A broadcast view replicates one element across the axis with a zero stride, so reading it as
        // a packed int32[K] would hand ovphysx three indices that do not exist in the buffer.
        const array::Array indices = array::zeros(array::Shape(std::vector<int64_t>{ 1 }), array::Dtype::Int32())
                                         .broadcastView(array::Shape(std::vector<int64_t>{ 3 }));
        REQUIRE_FALSE(indices.isContiguous());
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "read"); }), "C-contiguous"));
    }
}

//=============================================================================
// Output validation -- the engine writes through a caller-supplied `out` without
// checking it, so a wrong dtype or capacity must be rejected before the write.
//=============================================================================
TEST_CASE("requireMatchingOutput")
{
    const array::Shape shape(std::vector<int64_t>{ 4, 3 });

    SUBCASE("a matching buffer is accepted")
    {
        const array::Array output = array::zeros(shape, array::Dtype::Float32());
        CHECK(getThrownMessage([&] { requireMatchingOutput(output, shape, array::Dtype::Float32(), "read"); }).empty());
    }

    SUBCASE("a buffer of matching element count but different rank is accepted")
    {
        // The binding reports some data 1-D (masses [N]) while callers legitimately
        // allocate the matching 2-D buffer ([N, 1]), so the check is on element count.
        const array::Array output = array::zeros(array::Shape(std::vector<int64_t>{ 12, 1 }), array::Dtype::Float32());
        CHECK(getThrownMessage([&] { requireMatchingOutput(output, shape, array::Dtype::Float32(), "read"); }).empty());
    }

    SUBCASE("a dtype mismatch is rejected")
    {
        const array::Array output = array::zeros(shape, array::Dtype::Int32());
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, shape, array::Dtype::Float32(), "read"); }),
                       "dtype mismatch"));
    }

    SUBCASE("an undersized buffer is rejected")
    {
        const array::Array output = array::zeros(array::Shape(std::vector<int64_t>{ 4 }), array::Dtype::Float32());
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, shape, array::Dtype::Float32(), "read"); }),
                       "element count mismatch"));
    }

    SUBCASE("an element count that overflows size_t is rejected")
    {
        const array::Array output = array::zeros(shape, array::Dtype::Float32());
        const array::Shape overflowing(std::vector<int64_t>{ std::numeric_limits<int64_t>::max(), 3 });
        CHECK_FALSE(
            getThrownMessage([&] { requireMatchingOutput(output, overflowing, array::Dtype::Float32(), "read"); }).empty());
    }

    SUBCASE("a strided buffer is rejected")
    {
        // Right dtype and right element count, but transposing permutes the strides: the caller is
        // handed back a buffer the engine would have filled in the wrong order.
        const array::Array output =
            array::transposeView(array::zeros(array::Shape(std::vector<int64_t>{ 3, 4 }), array::Dtype::Float32()));
        REQUIRE_FALSE(output.isContiguous());
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, shape, array::Dtype::Float32(), "read"); }),
                       "C-contiguous"));
    }

    SUBCASE("a read-only buffer is rejected")
    {
        // Slicing a broadcast view yields contiguous strides but keeps the read-only flag, so this
        // passes the layout check and still cannot be written through.
        array::Array broadcast =
            array::zeros(shape, array::Dtype::Float32()).broadcastView(array::Shape(std::vector<int64_t>{ 2, 4, 3 }));
        const array::Array output = broadcast.at(0);
        REQUIRE(output.isContiguous());
        REQUIRE_FALSE(output.isWriteable());
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, shape, array::Dtype::Float32(), "read"); }),
                       "writeable"));
    }
}

//=============================================================================
// DLPack conversion -- ovphysx reads a boolean tensor as uint8, so the boolean
// dtype must not reach it under DLPack's own kDLBool spelling.
//=============================================================================
TEST_CASE("convertToDLTensor")
{
    SUBCASE("shape, dtype and device come from the array")
    {
        const array::Array source = array::zeros(array::Shape(std::vector<int64_t>{ 2, 3 }), array::Dtype::Float32());
        const DLTensor tensor = convertToDLTensor(source);
        CHECK(tensor.ndim == 2);
        CHECK(tensor.shape[0] == 2);
        CHECK(tensor.shape[1] == 3);
        CHECK(tensor.dtype.code == static_cast<uint8_t>(kDLFloat));
        CHECK(tensor.dtype.bits == 32);
        CHECK(tensor.device.device_type == kDLCPU);
        CHECK(tensor.strides == nullptr);
    }

    SUBCASE("a boolean array is handed over as uint8")
    {
        const array::Array source = array::zeros(array::Shape(std::vector<int64_t>{ 4 }), array::Dtype::Bool());
        const DLTensor tensor = convertToDLTensor(source);
        CHECK(tensor.dtype.code == static_cast<uint8_t>(kDLUInt));
        CHECK(tensor.dtype.bits == 8);
    }

    SUBCASE("scoped access exposes an offset view exactly once")
    {
        const array::Array source = array::Array(std::vector<std::vector<int32_t>>{ { 1, 2 }, { 3, 4 } }).at(1);
        const array::ArrayReadAccess access = array::acquireRead(source);
        const DLTensor tensor = convertToDLTensor(source, access);
        CHECK(tensor.data == source.data());
        CHECK(tensor.byte_offset == 0);
    }
}

//=============================================================================
// Ownership propagation -- a result handed to a caller must keep the allocation
// behind it alive even after the impl drops its own reference.
//=============================================================================
TEST_CASE("shareBytes")
{
    SUBCASE("the returned buffer keeps its owner alive and points inside it")
    {
        auto owner = std::make_shared<std::vector<float>>(8, 1.0f);
        const std::weak_ptr<std::vector<float>> observer = owner;
        std::shared_ptr<std::byte[]> bytes = shareBytes(owner, owner->data() + 2);

        owner.reset();
        REQUIRE_FALSE(observer.expired());
        CHECK(*reinterpret_cast<const float*>(bytes.get()) == 1.0f);

        bytes.reset();
        CHECK(observer.expired());
    }
}
