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

    // Rows are copied out as dense blocks, so compact a strided or broadcast operand first; this is a
    // view when it is already contiguous.
    const array::Array contiguousArray = array.contiguous();
    array::ArrayReadAccess sourceAccess = array::acquireRead(contiguousArray);
    const std::byte* source = static_cast<const std::byte*>(sourceAccess.data());
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

    return { array::Array::fromBuffer(std::move(leftBuffer), rowCount * leftColumnCount * elementSize,
                                      array::Shape(leftDimensions), array.dtype(), array.device()),
             array::Array::fromBuffer(std::move(rightBuffer), rowCount * rightColumnCount * elementSize,
                                      array::Shape(rightDimensions), array.dtype(), array.device()) };
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
        // Same dense-block read as splitArray(): compact a strided or broadcast operand first.
        const array::Array contiguousArray = arrays[i].contiguous();
        array::ArrayReadAccess sourceAccess = array::acquireRead(contiguousArray);
        const std::byte* source = static_cast<const std::byte*>(sourceAccess.data());
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

    return array::Array::fromBuffer(std::move(buffer), rowCount * joinedColumnCount * elementSize,
                                    array::Shape(joinedDimensions), reference.dtype(), reference.device());
}

} // namespace details
} // namespace entities
} // namespace physics
} // namespace isaacsim
