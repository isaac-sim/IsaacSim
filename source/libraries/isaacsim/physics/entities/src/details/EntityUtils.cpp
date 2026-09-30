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

#include "EntityUtils.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace isaacsim
{
namespace physics
{
namespace entities
{
namespace details
{

static array::Dtype convertDtype(tensors::DType dtype)
{
    switch (dtype)
    {
    case tensors::DType::eBool:
        return array::Dtype::Bool();
    case tensors::DType::eInt8:
        return array::Dtype::Int8();
    case tensors::DType::eInt16:
        return array::Dtype::Int16();
    case tensors::DType::eInt32:
        return array::Dtype::Int32();
    case tensors::DType::eInt64:
        return array::Dtype::Int64();
    case tensors::DType::eUInt8:
        return array::Dtype::UInt8();
    case tensors::DType::eUInt16:
        return array::Dtype::UInt16();
    case tensors::DType::eUInt32:
        return array::Dtype::UInt32();
    case tensors::DType::eUInt64:
        return array::Dtype::UInt64();
    case tensors::DType::eFloat32:
        return array::Dtype::Float32();
    case tensors::DType::eFloat64:
        return array::Dtype::Float64();
    default:
        throw std::invalid_argument("convertDtype: unsupported tensors::DType");
    }
}

static tensors::DType convertDtype(array::Dtype dtype)
{
    switch (dtype.kind())
    {
    case array::Dtype::Kind::eBool:
        return tensors::DType::eBool;
    case array::Dtype::Kind::eInt8:
        return tensors::DType::eInt8;
    case array::Dtype::Kind::eInt16:
        return tensors::DType::eInt16;
    case array::Dtype::Kind::eInt32:
        return tensors::DType::eInt32;
    case array::Dtype::Kind::eInt64:
        return tensors::DType::eInt64;
    case array::Dtype::Kind::eUInt8:
        return tensors::DType::eUInt8;
    case array::Dtype::Kind::eUInt16:
        return tensors::DType::eUInt16;
    case array::Dtype::Kind::eUInt32:
        return tensors::DType::eUInt32;
    case array::Dtype::Kind::eUInt64:
        return tensors::DType::eUInt64;
    case array::Dtype::Kind::eFloat32:
        return tensors::DType::eFloat32;
    case array::Dtype::Kind::eFloat64:
        return tensors::DType::eFloat64;
    default:
        throw std::invalid_argument("convertDtype: unsupported array::Dtype");
    }
}

static array::Device convertDevice(tensors::DeviceKind kind, int32_t ordinal)
{
    switch (kind)
    {
    case tensors::DeviceKind::eCpu:
        return array::Device::Cpu();
    case tensors::DeviceKind::eGpu:
        return array::Device::Cuda(ordinal >= 0 ? ordinal : 0);
    case tensors::DeviceKind::eEngineDefault:
        return array::Device::Cpu();
    default:
        throw std::invalid_argument("convertDevice: unsupported tensors::DeviceKind");
    }
}

array::Array convertTensorDescriptionToArray(const tensors::TensorDescription& description)
{
    array::Dtype dtype = convertDtype(description.dtype);
    array::Device device = convertDevice(description.device, description.deviceOrdinal);
    array::Shape shape(description.shape);

    std::shared_ptr<std::byte[]> buffer;
    if (description.keepAlive)
    {
        buffer = std::shared_ptr<std::byte[]>(description.keepAlive, static_cast<std::byte*>(description.data));
    }
    else
    {
        buffer = std::shared_ptr<std::byte[]>(std::shared_ptr<void>{}, static_cast<std::byte*>(description.data));
    }

    return array::Array::fromBuffer(buffer, shape, dtype, device);
}

tensors::TensorDescription convertArrayToTensorDescription(const array::Array& array)
{
    tensors::TensorDescription tensorDescription;
    tensorDescription.dtype = convertDtype(array.dtype());
    tensorDescription.device = array.device().isCpu() ? tensors::DeviceKind::eCpu : tensors::DeviceKind::eGpu;
    tensorDescription.deviceOrdinal = array.device().ordinal();
    tensorDescription.shape = array.shape().shape();
    tensorDescription.data = const_cast<void*>(array.data());
    tensorDescription.keepAlive = std::shared_ptr<void>(array.buffer(), array.buffer().get());
    return tensorDescription;
}

// Returns the number of rows, that is, the product of every dimension but the last.
static size_t computeRowCount(const std::string& context, const std::vector<int64_t>& dimensions)
{
    size_t rowCount = 1;
    for (size_t i = 0; i + 1 < dimensions.size(); ++i)
    {
        if (dimensions[i] < 0)
        {
            throw std::invalid_argument(context + ": array has a negative dimension");
        }
        rowCount *= static_cast<size_t>(dimensions[i]);
    }
    return rowCount;
}

// Returns the size of the last axis, which is the axis along which arrays are split and joined.
static size_t computeColumnCount(const std::string& context, const std::vector<int64_t>& dimensions)
{
    if (dimensions.back() < 0)
    {
        throw std::invalid_argument(context + ": array has a negative last dimension");
    }
    return static_cast<size_t>(dimensions.back());
}

// Validates that an array can take part in a split or join along its last axis.
static void checkLastAxisOperand(const std::string& context, const array::Array& array)
{
    if (array.ndim() == 0)
    {
        throw std::invalid_argument(context + ": array must have at least one dimension");
    }
    if (!array.device().isCpu())
    {
        throw std::logic_error(context + ": not implemented for non-CPU devices");
    }
}

std::vector<array::Array> splitArray(const array::Array& array, size_t index)
{
    const std::string context = "splitArray()";
    checkLastAxisOperand(context, array);

    const std::vector<int64_t> dimensions = array.shape().shape();
    const size_t columnCount = computeColumnCount(context, dimensions);
    if (index > columnCount)
    {
        throw std::out_of_range(context + ": index " + std::to_string(index) +
                                " out of range for last axis with size " + std::to_string(columnCount));
    }

    const size_t rowCount = computeRowCount(context, dimensions);
    const size_t elementSize = array.dtype().size();
    const size_t leftColumnCount = index;
    const size_t rightColumnCount = columnCount - index;

    auto leftBuffer = std::shared_ptr<std::byte[]>(new std::byte[rowCount * leftColumnCount * elementSize]());
    auto rightBuffer = std::shared_ptr<std::byte[]>(new std::byte[rowCount * rightColumnCount * elementSize]());

    const std::byte* source = static_cast<const std::byte*>(array.data());
    for (size_t row = 0; row < rowCount; ++row)
    {
        const std::byte* rowSource = source + row * columnCount * elementSize;
        std::memcpy(leftBuffer.get() + row * leftColumnCount * elementSize, rowSource, leftColumnCount * elementSize);
        std::memcpy(rightBuffer.get() + row * rightColumnCount * elementSize, rowSource + leftColumnCount * elementSize,
                    rightColumnCount * elementSize);
    }

    std::vector<int64_t> leftDimensions = dimensions;
    leftDimensions.back() = static_cast<int64_t>(leftColumnCount);
    std::vector<int64_t> rightDimensions = dimensions;
    rightDimensions.back() = static_cast<int64_t>(rightColumnCount);

    return { array::Array::fromBuffer(std::move(leftBuffer), array::Shape(leftDimensions), array.dtype(), array.device()),
             array::Array::fromBuffer(
                 std::move(rightBuffer), array::Shape(rightDimensions), array.dtype(), array.device()) };
}

array::Array joinArrays(const std::vector<array::Array>& arrays)
{
    const std::string context = "joinArrays()";
    if (arrays.empty())
    {
        throw std::invalid_argument(context + ": at least one array is required");
    }

    const array::Array& reference = arrays.front();
    checkLastAxisOperand(context, reference);

    const std::vector<int64_t> referenceDimensions = reference.shape().shape();
    const size_t rowCount = computeRowCount(context, referenceDimensions);
    const size_t elementSize = reference.dtype().size();

    // Collect the width of each array and validate that they agree on everything but the last axis
    std::vector<size_t> columnCounts;
    columnCounts.reserve(arrays.size());
    size_t joinedColumnCount = 0;
    for (const array::Array& current : arrays)
    {
        checkLastAxisOperand(context, current);
        if (current.dtype() != reference.dtype())
        {
            throw std::invalid_argument(context + ": dtype '" + current.dtype().toString() +
                                        "' does not match dtype '" + reference.dtype().toString() + "'");
        }
        if (current.device() != reference.device())
        {
            throw std::invalid_argument(context + ": device '" + current.device().toString() +
                                        "' does not match device '" + reference.device().toString() + "'");
        }
        const std::vector<int64_t> currentDimensions = current.shape().shape();
        if (currentDimensions.size() != referenceDimensions.size() ||
            !std::equal(referenceDimensions.begin(), referenceDimensions.end() - 1, currentDimensions.begin()))
        {
            throw std::invalid_argument(context + ": shape " + current.shape().toString() +
                                        " is not compatible with shape " + reference.shape().toString() +
                                        "; all dimensions but the last must match");
        }
        const size_t columnCount = computeColumnCount(context, currentDimensions);
        columnCounts.push_back(columnCount);
        joinedColumnCount += columnCount;
    }

    auto buffer = std::shared_ptr<std::byte[]>(new std::byte[rowCount * joinedColumnCount * elementSize]());
    size_t columnOffset = 0;
    for (size_t i = 0; i < arrays.size(); ++i)
    {
        const std::byte* source = static_cast<const std::byte*>(arrays[i].data());
        const size_t columnCount = columnCounts[i];
        for (size_t row = 0; row < rowCount; ++row)
        {
            std::memcpy(buffer.get() + (row * joinedColumnCount + columnOffset) * elementSize,
                        source + row * columnCount * elementSize, columnCount * elementSize);
        }
        columnOffset += columnCount;
    }

    std::vector<int64_t> joinedDimensions = referenceDimensions;
    joinedDimensions.back() = static_cast<int64_t>(joinedColumnCount);

    return array::Array::fromBuffer(
        std::move(buffer), array::Shape(joinedDimensions), reference.dtype(), reference.device());
}

} // namespace details
} // namespace entities
} // namespace physics
} // namespace isaacsim
