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
#include <string>

using namespace isaacsim::physics_engines::ovphysx;
using isaacsim::physics::tensors::DeviceKind;
using isaacsim::physics::tensors::DType;
using isaacsim::physics::tensors::TensorDescription;

namespace
{

// Run `callback` and return the message of the std::runtime_error it throws, or ""
// if it does not throw. Lets a test pin the *specific* rejected branch by
// matching a substring of the guard's message.
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

// Contiguous CPU float32 out buffer of the given shape (empty strides == C-contiguous).
TensorDescription makeCpuOutput(std::vector<int64_t> shape, DType dtype = DType::eFloat32)
{
    TensorDescription descriptor;
    descriptor.dtype = dtype;
    descriptor.shape = std::move(shape);
    descriptor.device = DeviceKind::eCpu;
    descriptor.deviceOrdinal = -1;
    return descriptor;
}

} // namespace

//=============================================================================
// Checked sizing -- all host allocations derived from tensor shapes use these
// helpers, so malformed dimensions must fail before a signed wrap or undersized
// allocation can occur.
//=============================================================================
TEST_CASE("checked tensor sizing")
{
    SUBCASE("normal dimensions produce their full element count")
    {
        CHECK(checkedElementCount({ 2, 3, 4 }, "shape") == 24);
    }

    SUBCASE("an empty dimension list represents a scalar")
    {
        CHECK(checkedElementCount({}, "shape") == 1);
    }

    SUBCASE("a zero dimension produces an empty tensor")
    {
        CHECK(checkedElementCount({ 8, 0, 4 }, "shape") == 0);
    }

    SUBCASE("a negative dimension is rejected")
    {
        CHECK(contains(getThrownMessage(
                           [] {
                               checkedElementCount({ 2, -1, 4 }, "shape");
                           }),
                       "negative tensor dimension"));
    }

    SUBCASE("dimension products that exceed size_t are rejected")
    {
        const std::string message = getThrownMessage(
            [] {
                checkedElementCount({ std::numeric_limits<int64_t>::max(), 3 }, "shape");
            });
        CHECK_FALSE(message.empty());
    }

    SUBCASE("an exact size_t product is accepted")
    {
        const size_t left = std::numeric_limits<size_t>::max() / 2;
        CHECK(checkedSizeProduct(left, 2, "bytes") == left * 2);
    }

    SUBCASE("a byte-size product that exceeds size_t is rejected")
    {
        CHECK(contains(getThrownMessage([] { checkedSizeProduct(std::numeric_limits<size_t>::max(), 2, "bytes"); }),
                       "element count overflow"));
    }
}

//=============================================================================
// requireMatchingOutput -- the core memory-safety guard for caller-supplied GET
// `out` buffers. Branch matrix: dtype, element-count (incl. 1-D vs 2-D same
// capacity), contiguity, GPU-ordinal, and the all-pass case.
//=============================================================================
TEST_CASE("requireMatchingOutput branch matrix")
{
    // The SDF distances-and-gradients contract shape: [N, maxQ, 4].
    const std::vector<int64_t> sdfShape = { 2, 5, 4 };

    SUBCASE("matching CPU out passes")
    {
        TensorDescription output = makeCpuOutput({ 2, 5, 4 });
        REQUIRE_NOTHROW(requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"));
    }

    SUBCASE("matching int32 out passes")
    {
        TensorDescription output = makeCpuOutput({ 4 }, DType::eInt32);
        REQUIRE_NOTHROW(requireMatchingOutput(output, { 4 }, DType::eInt32, "idx"));
    }

    SUBCASE("dtype mismatch throws")
    {
        TensorDescription output = makeCpuOutput({ 2, 5, 4 }, DType::eFloat64);
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }),
                       "dtype mismatch"));
    }

    SUBCASE("element-count mismatch throws")
    {
        TensorDescription output = makeCpuOutput({ 2, 5, 3 }); // 30 vs binding 40
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }),
                       "element count mismatch"));
    }

    SUBCASE("1-D binding vs 2-D out with same capacity is allowed")
    {
        // masses reported [N] by the binding; caller legitimately supplies [N, 1].
        TensorDescription output = makeCpuOutput({ 5, 1 });
        REQUIRE_NOTHROW(requireMatchingOutput(output, { 5 }, DType::eFloat32, "masses"));
    }

    SUBCASE("non-contiguous out throws")
    {
        TensorDescription output = makeCpuOutput({ 2, 5, 4 });
        // Row-major strides would be {20, 4, 1}; a wrong leading stride makes it strided.
        output.strides = { 21, 4, 1 };
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }),
                       "C-contiguous"));
    }

    SUBCASE("GPU out with negative ordinal throws")
    {
        TensorDescription output = makeCpuOutput({ 2, 5, 4 });
        output.device = DeviceKind::eGpu;
        output.deviceOrdinal = -1;
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }),
                       "device ordinal"));
    }

    SUBCASE("GPU out with valid ordinal passes")
    {
        TensorDescription output = makeCpuOutput({ 2, 5, 4 });
        output.device = DeviceKind::eGpu;
        output.deviceOrdinal = 0;
        REQUIRE_NOTHROW(requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"));
    }

    SUBCASE("zero-element out matches a zero-element binding")
    {
        // A 0-row selection is a legitimate no-op read, not a mismatch.
        TensorDescription output = makeCpuOutput({ 0 });
        REQUIRE_NOTHROW(requireMatchingOutput(output, { 0 }, DType::eFloat32, "empty"));
    }

    SUBCASE("overcount out throws (too large, not just too small)")
    {
        TensorDescription output = makeCpuOutput({ 2, 5, 5 }); // 50 vs binding 40
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }),
                       "element count mismatch"));
    }

    SUBCASE("CPU out with a stray non-negative ordinal still passes")
    {
        // The ordinal check is GPU-only; a CPU buffer's device_id is ignored, so a
        // leftover ordinal must not be treated as an error.
        TensorDescription output = makeCpuOutput({ 2, 5, 4 });
        output.deviceOrdinal = 3; // CPU device, stray ordinal
        REQUIRE_NOTHROW(requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"));
    }

    SUBCASE("dtype is checked before element count")
    {
        // Both wrong: dtype AND count. dtype is validated first, so its message wins --
        // pins the guard order so a reorder that reports the wrong cause is caught.
        TensorDescription output = makeCpuOutput({ 2, 5, 3 }, DType::eFloat64);
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }),
                       "dtype mismatch"));
    }

    SUBCASE("negative output dimensions are rejected before comparing capacity")
    {
        TensorDescription output = makeCpuOutput({ 2, -5, 4 });
        CHECK(contains(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }),
                       "negative tensor dimension"));
    }

    SUBCASE("overflowing output dimensions are rejected before comparing capacity")
    {
        TensorDescription output = makeCpuOutput({ std::numeric_limits<int64_t>::max(), 3 });
        CHECK_FALSE(getThrownMessage([&] { requireMatchingOutput(output, sdfShape, DType::eFloat32, "sdf"); }).empty());
    }
}

//=============================================================================
// isContiguousRowMajor -- underpins the contiguity checks above and in
// requireInt32Indices.
//=============================================================================
TEST_CASE("isContiguousRowMajor")
{
    SUBCASE("empty strides count as contiguous")
    {
        TensorDescription descriptor = makeCpuOutput({ 3, 4 });
        CHECK(isContiguousRowMajor(descriptor));
    }

    SUBCASE("correct row-major strides are contiguous")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 3, 4 });
        descriptor.strides = { 12, 4, 1 };
        CHECK(isContiguousRowMajor(descriptor));
    }

    SUBCASE("wrong strides are not contiguous")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 3 });
        descriptor.strides = { 1, 2 }; // column-major-ish
        CHECK_FALSE(isContiguousRowMajor(descriptor));
    }

    SUBCASE("strides size mismatch is not contiguous")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 3 });
        descriptor.strides = { 3 }; // rank 2 shape, rank 1 strides
        CHECK_FALSE(isContiguousRowMajor(descriptor));
    }

    SUBCASE("unit-dim stride is irrelevant to contiguity")
    {
        TensorDescription descriptor = makeCpuOutput({ 4, 1 });
        descriptor.strides = { 1, 999 }; // the size-1 dim's stride never matters
        CHECK(isContiguousRowMajor(descriptor));
    }

    SUBCASE("negative dimensions are not contiguous")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, -3 });
        descriptor.strides = { 3, 1 };
        CHECK_FALSE(isContiguousRowMajor(descriptor));
    }

    SUBCASE("overflowing row-major strides are not contiguous")
    {
        TensorDescription descriptor = makeCpuOutput({ 3, std::numeric_limits<int64_t>::max() });
        descriptor.strides = { std::numeric_limits<int64_t>::max(), 1 };
        CHECK_FALSE(isContiguousRowMajor(descriptor));
    }
}

//=============================================================================
// requireInt32Indices -- guards the flat int32[K] index buffer of indexed
// get/set (and the SDF query slot's sibling paths).
//=============================================================================
TEST_CASE("requireInt32Indices")
{
    int32_t indexData[3] = { 0, 1, 2 };

    SUBCASE("omitted index (empty shape) is skipped")
    {
        TensorDescription indices; // default: no shape
        REQUIRE_NOTHROW(requireInt32Indices(indices, "idx"));
    }

    SUBCASE("1-D contiguous int32 passes")
    {
        TensorDescription indices = makeCpuOutput({ 3 }, DType::eInt32);
        indices.data = indexData;
        REQUIRE_NOTHROW(requireInt32Indices(indices, "idx"));
    }

    SUBCASE("well-formed zero-size int32 passes but is still validated")
    {
        TensorDescription indices = makeCpuOutput({ 0 }, DType::eInt32);
        REQUIRE_NOTHROW(requireInt32Indices(indices, "idx"));
    }

    SUBCASE("non-int32 dtype throws")
    {
        TensorDescription indices = makeCpuOutput({ 3 }, DType::eInt64);
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "idx"); }), "int32"));
    }

    SUBCASE("zero-size non-int32 is still rejected")
    {
        // Confirms the supplied-but-empty tensor's dtype cannot slip past the guard.
        TensorDescription indices = makeCpuOutput({ 0 }, DType::eInt64);
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "idx"); }), "int32"));
    }

    SUBCASE("multi-dim index throws")
    {
        TensorDescription indices = makeCpuOutput({ 2, 3 }, DType::eInt32);
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "idx"); }), "1-D"));
    }

    SUBCASE("strided index throws")
    {
        TensorDescription indices = makeCpuOutput({ 3 }, DType::eInt32);
        indices.data = indexData;
        indices.strides = { 2 };
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "idx"); }), "C-contiguous"));
    }

    SUBCASE("non-empty index without storage throws")
    {
        TensorDescription indices = makeCpuOutput({ 3 }, DType::eInt32);
        CHECK(contains(getThrownMessage([&] { requireInt32Indices(indices, "idx"); }), "no data"));
    }
}

//=============================================================================
// convertToDLDataType -- umbrella DType -> DLPack dtype mapping.
//=============================================================================
TEST_CASE("convertToDLDataType mapping")
{
    auto check = [](DType dataType, uint8_t code, uint8_t bits)
    {
        DLDataType actualDataType = convertToDLDataType(dataType);
        CHECK(actualDataType.code == code);
        CHECK(actualDataType.bits == bits);
        CHECK(actualDataType.lanes == 1);
    };

    check(DType::eFloat32, static_cast<uint8_t>(kDLFloat), 32);
    check(DType::eFloat64, static_cast<uint8_t>(kDLFloat), 64);
    check(DType::eInt8, static_cast<uint8_t>(kDLInt), 8);
    check(DType::eInt16, static_cast<uint8_t>(kDLInt), 16);
    check(DType::eInt32, static_cast<uint8_t>(kDLInt), 32);
    check(DType::eInt64, static_cast<uint8_t>(kDLInt), 64);
    check(DType::eUInt8, static_cast<uint8_t>(kDLUInt), 8);
    check(DType::eUInt16, static_cast<uint8_t>(kDLUInt), 16);
    check(DType::eUInt32, static_cast<uint8_t>(kDLUInt), 32);
    check(DType::eUInt64, static_cast<uint8_t>(kDLUInt), 64);

    SUBCASE("bool maps to uint8 (warp-compatible round-trip)")
    {
        check(DType::eBool, static_cast<uint8_t>(kDLUInt), 8);
    }

    SUBCASE("unknown falls back to float32")
    {
        check(DType::eUnknown, static_cast<uint8_t>(kDLFloat), 32);
    }
}

//=============================================================================
// convertToDLTensor -- umbrella TensorDescription -> borrowed DLTensor. Covers the device,
// dtype, and stride-normalization plumbing the SDF (and every read/write) path
// depends on: qDL = convertToDLTensor(queryPoints).
//=============================================================================
TEST_CASE("convertToDLTensor device / dtype / strides")
{
    int dummy = 0;

    SUBCASE("CPU contiguous float32 -> {kDLCPU, 0}, null strides, data preserved")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 3 });
        descriptor.data = &dummy;
        DLTensor tensor = convertToDLTensor(descriptor);
        CHECK(tensor.data == &dummy);
        CHECK(tensor.device.device_type == kDLCPU);
        CHECK(tensor.device.device_id == 0);
        CHECK(tensor.dtype.code == static_cast<uint8_t>(kDLFloat));
        CHECK(tensor.dtype.bits == 32);
        CHECK(tensor.ndim == 2);
        CHECK(tensor.shape == descriptor.shape.data());
        CHECK(tensor.strides == nullptr);
        CHECK(tensor.byte_offset == 0);
    }

    SUBCASE("GPU out carries its device ordinal")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 5, 4 });
        descriptor.device = DeviceKind::eGpu;
        descriptor.deviceOrdinal = 3;
        DLTensor tensor = convertToDLTensor(descriptor);
        CHECK(tensor.device.device_type == kDLCUDA);
        CHECK(tensor.device.device_id == 3);
    }

    SUBCASE("GPU tensor with no ordinal throws")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 5, 4 });
        descriptor.device = DeviceKind::eGpu;
        descriptor.deviceOrdinal = -1;
        CHECK(contains(getThrownMessage([&] { convertToDLTensor(descriptor); }), "device ordinal"));
    }

    SUBCASE("CPU tensor with a negative ordinal fills device_id 0")
    {
        TensorDescription descriptor = makeCpuOutput({ 4 }); // CPU, deviceOrdinal -1 by default
        DLTensor tensor = convertToDLTensor(descriptor);
        CHECK(tensor.device.device_type == kDLCPU);
        CHECK(tensor.device.device_id == 0);
    }

    SUBCASE("explicit C-contiguous strides normalize to null")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 3, 4 });
        descriptor.strides = { 12, 4, 1 };
        DLTensor tensor = convertToDLTensor(descriptor);
        CHECK(tensor.strides == nullptr);
    }

    SUBCASE("genuine strides pass through so ovphysx can reject them")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 3 });
        descriptor.strides = { 1, 2 }; // column-major-ish, non-contiguous
        DLTensor tensor = convertToDLTensor(descriptor);
        REQUIRE(tensor.strides != nullptr);
        CHECK(tensor.strides == descriptor.strides.data());
    }

    SUBCASE("negative dimensions are rejected")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, -3 });
        CHECK(contains(getThrownMessage([&] { convertToDLTensor(descriptor); }), "negative tensor dimension"));
    }

    SUBCASE("stride rank mismatch is rejected")
    {
        TensorDescription descriptor = makeCpuOutput({ 2, 3 });
        descriptor.strides = { 3 };
        CHECK(contains(getThrownMessage([&] { convertToDLTensor(descriptor); }), "strides must match"));
    }
}
