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

#include "isaacsim/common/array/Dtype.hpp"

#include <algorithm>
#include <stdexcept>

namespace isaacsim
{
namespace common
{
namespace array
{

namespace
{

// Signed integer dtype of the given width, in bytes. Widths that are not exact type sizes, and
// widths above 8, are rounded up to the next signed type (int64 at most).
Dtype signedDtypeOfSize(size_t size)
{
    switch (size)
    {
    case 1:
        return Dtype::Int8();
    case 2:
        return Dtype::Int16();
    case 4:
        return Dtype::Int32();
    default:
        return Dtype::Int64();
    }
}

} // namespace

Dtype::Dtype(const std::string& name) : m_kind(fromString(name).kind())
{
}

bool Dtype::operator==(const Dtype& other) const
{
    return m_kind == other.m_kind;
}

bool Dtype::operator!=(const Dtype& other) const
{
    return m_kind != other.m_kind;
}

Dtype::Kind Dtype::kind() const
{
    return m_kind;
}

size_t Dtype::size() const
{
    switch (m_kind)
    {
    case Kind::eBool:
        return sizeof(bool);
    case Kind::eInt8:
        return sizeof(int8_t);
    case Kind::eInt16:
        return sizeof(int16_t);
    case Kind::eInt32:
        return sizeof(int32_t);
    case Kind::eInt64:
        return sizeof(int64_t);
    case Kind::eUInt8:
        return sizeof(uint8_t);
    case Kind::eUInt16:
        return sizeof(uint16_t);
    case Kind::eUInt32:
        return sizeof(uint32_t);
    case Kind::eUInt64:
        return sizeof(uint64_t);
    case Kind::eFloat32:
        return sizeof(float);
    case Kind::eFloat64:
        return sizeof(double);
    }
    return 0;
}

bool Dtype::isFloating() const
{
    return m_kind == Kind::eFloat32 || m_kind == Kind::eFloat64;
}

bool Dtype::isIntegral() const
{
    return !isFloating() && m_kind != Kind::eBool;
}

bool Dtype::isSigned() const
{
    switch (m_kind)
    {
    case Kind::eInt8:
    case Kind::eInt16:
    case Kind::eInt32:
    case Kind::eInt64:
    case Kind::eFloat32:
    case Kind::eFloat64:
        return true;
    default:
        return false;
    }
}

bool Dtype::isUnsigned() const
{
    switch (m_kind)
    {
    case Kind::eUInt8:
    case Kind::eUInt16:
    case Kind::eUInt32:
    case Kind::eUInt64:
        return true;
    default:
        return false;
    }
}

std::string Dtype::toString() const
{
    switch (m_kind)
    {
    case Kind::eBool:
        return "bool";
    case Kind::eInt8:
        return "int8";
    case Kind::eInt16:
        return "int16";
    case Kind::eInt32:
        return "int32";
    case Kind::eInt64:
        return "int64";
    case Kind::eUInt8:
        return "uint8";
    case Kind::eUInt16:
        return "uint16";
    case Kind::eUInt32:
        return "uint32";
    case Kind::eUInt64:
        return "uint64";
    case Kind::eFloat32:
        return "float32";
    case Kind::eFloat64:
        return "float64";
    }
    return "";
}

Dtype Dtype::fromString(const std::string& name)
{
    if (name == "bool")
        return Dtype(Kind::eBool);
    else if (name == "int8")
        return Dtype(Kind::eInt8);
    else if (name == "int16")
        return Dtype(Kind::eInt16);
    else if (name == "int32")
        return Dtype(Kind::eInt32);
    else if (name == "int64")
        return Dtype(Kind::eInt64);
    else if (name == "uint8")
        return Dtype(Kind::eUInt8);
    else if (name == "uint16")
        return Dtype(Kind::eUInt16);
    else if (name == "uint32")
        return Dtype(Kind::eUInt32);
    else if (name == "uint64")
        return Dtype(Kind::eUInt64);
    else if (name == "float32")
        return Dtype(Kind::eFloat32);
    else if (name == "float64")
        return Dtype(Kind::eFloat64);
    throw std::invalid_argument("Unknown dtype: '" + std::string(name) + "'");
}

Dtype Dtype::promoteDtypes(Dtype a, Dtype b)
{
    if (a == b)
    {
        return a;
    }
    // Boolean is the least-ranked kind: it always yields to the other operand.
    else if (a == Bool())
    {
        return b;
    }
    else if (b == Bool())
    {
        return a;
    }
    // Two floating-point types: the wider one.
    else if (a.isFloating() && b.isFloating())
    {
        return a.size() >= b.size() ? a : b;
    }
    // Floating-point and integer: the smallest float that holds every integer value exactly,
    // which rules out float32 for integers wider than its 24-bit mantissa.
    else if (a.isFloating() || b.isFloating())
    {
        const Dtype floating = a.isFloating() ? a : b;
        const Dtype integral = a.isFloating() ? b : a;
        return (floating.size() >= 8 || integral.size() >= 4) ? Float64() : floating;
    }
    // Two integers of the same signedness: the wider one.
    else if (a.isSigned() == b.isSigned())
    {
        return a.size() >= b.size() ? a : b;
    }
    // Signed and unsigned integer: the smallest signed type holding both ranges, or float64 when
    // no signed type is wide enough (as with int64 and uint64).
    const Dtype signedType = a.isSigned() ? a : b;
    const Dtype unsignedType = a.isSigned() ? b : a;
    const size_t requiredSize = std::max(signedType.size(), 2 * unsignedType.size());
    return requiredSize > 8 ? Float64() : signedDtypeOfSize(requiredSize);
}

Dtype Dtype::Bool()
{
    return Dtype(Kind::eBool);
}

Dtype Dtype::Int8()
{
    return Dtype(Kind::eInt8);
}

Dtype Dtype::Int16()
{
    return Dtype(Kind::eInt16);
}

Dtype Dtype::Int32()
{
    return Dtype(Kind::eInt32);
}

Dtype Dtype::Int64()
{
    return Dtype(Kind::eInt64);
}

Dtype Dtype::UInt8()
{
    return Dtype(Kind::eUInt8);
}

Dtype Dtype::UInt16()
{
    return Dtype(Kind::eUInt16);
}

Dtype Dtype::UInt32()
{
    return Dtype(Kind::eUInt32);
}

Dtype Dtype::UInt64()
{
    return Dtype(Kind::eUInt64);
}

Dtype Dtype::Float32()
{
    return Dtype(Kind::eFloat32);
}

Dtype Dtype::Float64()
{
    return Dtype(Kind::eFloat64);
}

} // namespace array
} // namespace common
} // namespace isaacsim
