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

#include "isaacsim/common/array/Shape.hpp"

#include <algorithm>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace isaacsim
{
namespace common
{
namespace array
{

Shape::Shape(const details::SupportedShapeSpecification& shape)
    : m_shape(std::visit(
          [](const auto& value) -> std::vector<int64_t>
          {
              using T = std::decay_t<decltype(value)>;
              if constexpr (std::is_arithmetic_v<T>)
              {
                  return { static_cast<int64_t>(value) };
              }
              else
              {
                  std::vector<int64_t> result;
                  result.reserve(value.size());
                  for (const auto& dimension : value)
                  {
                      result.push_back(static_cast<int64_t>(dimension));
                  }
                  return result;
              }
          },
          shape))
{
}

int64_t Shape::operator[](int64_t index) const
{
    if ((index < 0 && index < -static_cast<int64_t>(m_shape.size())) || index >= static_cast<int64_t>(m_shape.size()))
    {
        throw std::out_of_range("Shape::operator[]: index " + std::to_string(index) + " out of range for shape " +
                                toString());
    }
    if (index < 0)
    {
        index += static_cast<int64_t>(m_shape.size());
    }
    return m_shape[index];
}

bool Shape::operator==(const Shape& other) const
{
    return m_shape == other.m_shape;
}

bool Shape::operator!=(const Shape& other) const
{
    return m_shape != other.m_shape;
}

std::vector<int64_t> Shape::shape() const
{
    return m_shape;
}

const int64_t* Shape::data() const
{
    return m_shape.data();
}

size_t Shape::ndim() const
{
    return m_shape.size();
}

size_t Shape::size() const
{
    if (std::any_of(m_shape.begin(), m_shape.end(), [](int64_t dim) { return dim < 0; }))
    {
        throw std::invalid_argument("Shape::size(): negative dimensions are not allowed");
    }
    return std::accumulate(m_shape.begin(), m_shape.end(), size_t{ 1 }, std::multiplies<size_t>());
}

bool Shape::canBroadcastTo(const Shape& other) const
{
    if (m_shape.size() > other.m_shape.size())
    {
        return false;
    }
    auto iterator = m_shape.rbegin();
    auto otherIterator = other.m_shape.rbegin();
    for (; iterator != m_shape.rend(); ++iterator, ++otherIterator)
    {
        if (*iterator != *otherIterator && *iterator != 1)
        {
            return false;
        }
    }
    return true;
}

Shape Shape::resolve(const Shape& other) const
{
    if (std::any_of(m_shape.begin(), m_shape.end(), [](int64_t dim) { return dim < 0; }))
    {
        throw std::invalid_argument("Shape::resolve(): current shape contains undefined (negative) dimension");
    }

    size_t knownProduct = 1;
    const size_t totalElements = size();
    std::optional<size_t> inferredIndex;
    std::vector<int64_t> dimensions = other.m_shape;

    for (size_t i = 0; i < dimensions.size(); ++i)
    {
        if (dimensions[i] == -1)
        {
            if (inferredIndex.has_value())
            {
                throw std::invalid_argument("Shape::resolve(): only one dimension can be -1");
            }
            inferredIndex = i;
        }
        else if (dimensions[i] < 0)
        {
            throw std::invalid_argument("Shape::resolve(): negative dimensions other than -1 are not allowed");
        }
        else
        {
            knownProduct *= static_cast<size_t>(dimensions[i]);
        }
    }

    if (inferredIndex.has_value())
    {
        if (!knownProduct)
        {
            if (totalElements)
            {
                throw std::invalid_argument(
                    "Shape::resolve(): cannot infer dimension with a zero-size known dimension and non-zero total "
                    "size");
            }
            dimensions[*inferredIndex] = 0;
        }
        else
        {
            if (totalElements % knownProduct)
            {
                throw std::invalid_argument("Shape::resolve(): cannot infer dimension: total size " +
                                            std::to_string(totalElements) + " is not divisible by " +
                                            std::to_string(knownProduct));
            }
            dimensions[*inferredIndex] = static_cast<int64_t>(totalElements / knownProduct);
        }
    }
    else
    {
        if (knownProduct != totalElements)
        {
            throw std::invalid_argument("Shape::resolve(): cannot reshape shape " + toString() + " into " +
                                        other.toString());
        }
    }

    return Shape(dimensions);
}

std::string Shape::toString() const
{
    std::stringstream ss;
    ss << "(";
    for (size_t i = 0; i < m_shape.size(); ++i)
    {
        if (i > 0)
        {
            ss << ", ";
        }
        ss << std::to_string(m_shape[i]);
    }
    ss << ")";
    return ss.str();
}

} // namespace array
} // namespace common
} // namespace isaacsim
