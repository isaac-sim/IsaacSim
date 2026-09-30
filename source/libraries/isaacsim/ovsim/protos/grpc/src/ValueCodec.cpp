// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ValueCodec.hpp"

#include <isaacsim/common/array/Array.hpp>

#include <limits>
#include <string>
#include <type_traits>
#include <vector>

namespace isaacsim
{
namespace ovsim
{
namespace protos
{
namespace grpc
{

namespace array = isaacsim::common::array;
namespace worldstate = nvidia::omniverse::simulation::worldstate::v1;

namespace
{

::grpc::Status invalid(const std::string& message)
{
    return { ::grpc::StatusCode::INVALID_ARGUMENT, message };
}

::grpc::Status unsupported(const std::string& message)
{
    return { ::grpc::StatusCode::UNIMPLEMENTED, message };
}

template <typename T, typename Packed>
void encodeScalars(const array::Array& source, Packed* packed)
{
    const auto values = source.reshape(array::Shape{ -1 }).get<std::vector<T>>();
    for (const T value : values)
    {
        packed->add_values(value);
    }
}

template <typename T, typename Packed, typename AddTuple>
void encodeTuples(const array::Array& source, Packed* packed, AddTuple addTuple)
{
    const auto rows = source.get<std::vector<std::vector<T>>>();
    for (const auto& row : rows)
    {
        addTuple(packed->add_values(), row);
    }
}

::grpc::Status encodeArray(const array::Array& source, ValueColumn* column)
{
    if (source.device() != array::Device::Cpu())
    {
        return unsupported("Only CPU arrays are supported by the initial gRPC value codec.");
    }
    const array::Array& staged = source;
    if (staged.ndim() != 2 || staged.shape()[0] < 0 || staged.shape()[1] < 1 || staged.shape()[1] > 4)
    {
        return unsupported("Supported arrays must have shape (rows, lanes) with one to four lanes.");
    }

    const int64_t lanes = staged.shape()[1];
    switch (staged.dtype().kind())
    {
    case array::Dtype::Kind::eBool:
        if (lanes != 1)
        {
            return unsupported("Boolean vector values are not supported.");
        }
        encodeScalars<bool>(staged, column->mutable_packed_bool());
        break;
    case array::Dtype::Kind::eUInt8:
        if (lanes != 1)
        {
            return unsupported("Unsigned byte vector values are not supported.");
        }
        encodeScalars<uint8_t>(staged, column->mutable_packed_uchar());
        break;
    case array::Dtype::Kind::eInt32:
        if (lanes == 1)
        {
            encodeScalars<int32_t>(staged, column->mutable_packed_int());
        }
        else if (lanes == 2)
        {
            encodeTuples<int32_t>(staged, column->mutable_packed_int2(),
                                  [](auto* value, const auto& row)
                                  {
                                      value->set_x(row[0]);
                                      value->set_y(row[1]);
                                  });
        }
        else if (lanes == 3)
        {
            encodeTuples<int32_t>(staged, column->mutable_packed_int3(),
                                  [](auto* value, const auto& row)
                                  {
                                      value->set_x(row[0]);
                                      value->set_y(row[1]);
                                      value->set_z(row[2]);
                                  });
        }
        else
        {
            encodeTuples<int32_t>(staged, column->mutable_packed_int4(),
                                  [](auto* value, const auto& row)
                                  {
                                      value->set_x(row[0]);
                                      value->set_y(row[1]);
                                      value->set_z(row[2]);
                                      value->set_w(row[3]);
                                  });
        }
        break;
    case array::Dtype::Kind::eUInt32:
        if (lanes != 1)
        {
            return unsupported("Unsigned 32-bit vector values are not supported.");
        }
        encodeScalars<uint32_t>(staged, column->mutable_packed_uint());
        break;
    case array::Dtype::Kind::eInt64:
        if (lanes != 1)
        {
            return unsupported("Signed 64-bit vector values are not supported.");
        }
        encodeScalars<int64_t>(staged, column->mutable_packed_int64());
        break;
    case array::Dtype::Kind::eUInt64:
        if (lanes != 1)
        {
            return unsupported("Unsigned 64-bit vector values are not supported.");
        }
        encodeScalars<uint64_t>(staged, column->mutable_packed_uint64());
        break;
    case array::Dtype::Kind::eFloat32:
        if (lanes == 1)
        {
            encodeScalars<float>(staged, column->mutable_packed_float());
        }
        else if (lanes == 2)
        {
            encodeTuples<float>(staged, column->mutable_packed_float2(),
                                [](auto* value, const auto& row)
                                {
                                    value->set_x(row[0]);
                                    value->set_y(row[1]);
                                });
        }
        else if (lanes == 3)
        {
            encodeTuples<float>(staged, column->mutable_packed_float3(),
                                [](auto* value, const auto& row)
                                {
                                    value->set_x(row[0]);
                                    value->set_y(row[1]);
                                    value->set_z(row[2]);
                                });
        }
        else
        {
            encodeTuples<float>(staged, column->mutable_packed_float4(),
                                [](auto* value, const auto& row)
                                {
                                    value->set_x(row[0]);
                                    value->set_y(row[1]);
                                    value->set_z(row[2]);
                                    value->set_w(row[3]);
                                });
        }
        break;
    case array::Dtype::Kind::eFloat64:
        if (lanes == 1)
        {
            encodeScalars<double>(staged, column->mutable_packed_double());
        }
        else if (lanes == 2)
        {
            encodeTuples<double>(staged, column->mutable_packed_double2(),
                                 [](auto* value, const auto& row)
                                 {
                                     value->set_x(row[0]);
                                     value->set_y(row[1]);
                                 });
        }
        else if (lanes == 3)
        {
            encodeTuples<double>(staged, column->mutable_packed_double3(),
                                 [](auto* value, const auto& row)
                                 {
                                     value->set_x(row[0]);
                                     value->set_y(row[1]);
                                     value->set_z(row[2]);
                                 });
        }
        else
        {
            encodeTuples<double>(staged, column->mutable_packed_double4(),
                                 [](auto* value, const auto& row)
                                 {
                                     value->set_x(row[0]);
                                     value->set_y(row[1]);
                                     value->set_z(row[2]);
                                     value->set_w(row[3]);
                                 });
        }
        break;
    default:
        return unsupported("The array data type is not supported by the minimal gRPC value codec.");
    }
    return ::grpc::Status::OK;
}

template <typename T, typename Packed>
array::Array decodeScalars(const Packed& packed)
{
    std::vector<T> values;
    values.reserve(static_cast<size_t>(packed.values_size()));
    for (const auto value : packed.values())
    {
        values.push_back(static_cast<T>(value));
    }
    return array::Array(values).reshape(array::Shape{ static_cast<int64_t>(values.size()), static_cast<int64_t>(1) });
}

template <typename T, size_t Lanes, typename Packed, typename GetTuple>
array::Array decodeTuples(const Packed& packed, GetTuple getTuple)
{
    if (packed.values().empty())
    {
        return array::Array(std::vector<T>{}).reshape(array::Shape{ static_cast<int64_t>(0), static_cast<int64_t>(Lanes) });
    }
    std::vector<std::vector<T>> rows;
    rows.reserve(static_cast<size_t>(packed.values_size()));
    for (const auto& value : packed.values())
    {
        rows.push_back(getTuple(value));
    }
    return array::Array(rows).reshape(array::Shape{ static_cast<int64_t>(rows.size()), static_cast<int64_t>(Lanes) });
}

template <typename ValueType>
::grpc::Status decodeValueImplementation(const ValueColumn& column, ValueType* value)
{
    if (!value)
    {
        return invalid("The destination OV SIM value is null.");
    }
    switch (column.packed_case())
    {
    case ValueColumn::kPackedBool:
        *value = decodeScalars<bool>(column.packed_bool());
        break;
    case ValueColumn::kPackedUchar:
        for (const uint32_t item : column.packed_uchar().values())
        {
            if (item > std::numeric_limits<uint8_t>::max())
            {
                return invalid("An unsigned byte value is outside the range 0..255.");
            }
        }
        *value = decodeScalars<uint8_t>(column.packed_uchar());
        break;
    case ValueColumn::kPackedInt:
        *value = decodeScalars<int32_t>(column.packed_int());
        break;
    case ValueColumn::kPackedUint:
        *value = decodeScalars<uint32_t>(column.packed_uint());
        break;
    case ValueColumn::kPackedInt64:
        *value = decodeScalars<int64_t>(column.packed_int64());
        break;
    case ValueColumn::kPackedUint64:
        *value = decodeScalars<uint64_t>(column.packed_uint64());
        break;
    case ValueColumn::kPackedFloat:
        *value = decodeScalars<float>(column.packed_float());
        break;
    case ValueColumn::kPackedDouble:
        *value = decodeScalars<double>(column.packed_double());
        break;
    case ValueColumn::kPackedString:
        *value = std::vector<std::string>(column.packed_string().values().begin(), column.packed_string().values().end());
        break;
    case ValueColumn::kPackedInt2:
        *value = decodeTuples<int32_t, 2>(column.packed_int2(),
                                          [](const auto& item) {
                                              return std::vector<int32_t>{ item.x(), item.y() };
                                          });
        break;
    case ValueColumn::kPackedInt3:
        *value = decodeTuples<int32_t, 3>(column.packed_int3(),
                                          [](const auto& item) {
                                              return std::vector<int32_t>{ item.x(), item.y(), item.z() };
                                          });
        break;
    case ValueColumn::kPackedInt4:
        *value = decodeTuples<int32_t, 4>(column.packed_int4(),
                                          [](const auto& item) {
                                              return std::vector<int32_t>{ item.x(), item.y(), item.z(), item.w() };
                                          });
        break;
    case ValueColumn::kPackedFloat2:
        *value = decodeTuples<float, 2>(column.packed_float2(),
                                        [](const auto& item) {
                                            return std::vector<float>{ item.x(), item.y() };
                                        });
        break;
    case ValueColumn::kPackedFloat3:
        *value = decodeTuples<float, 3>(column.packed_float3(),
                                        [](const auto& item) {
                                            return std::vector<float>{ item.x(), item.y(), item.z() };
                                        });
        break;
    case ValueColumn::kPackedFloat4:
        *value = decodeTuples<float, 4>(column.packed_float4(),
                                        [](const auto& item) {
                                            return std::vector<float>{ item.x(), item.y(), item.z(), item.w() };
                                        });
        break;
    case ValueColumn::kPackedDouble2:
        *value = decodeTuples<double, 2>(column.packed_double2(),
                                         [](const auto& item) {
                                             return std::vector<double>{ item.x(), item.y() };
                                         });
        break;
    case ValueColumn::kPackedDouble3:
        *value = decodeTuples<double, 3>(column.packed_double3(),
                                         [](const auto& item) {
                                             return std::vector<double>{ item.x(), item.y(), item.z() };
                                         });
        break;
    case ValueColumn::kPackedDouble4:
        *value = decodeTuples<double, 4>(column.packed_double4(),
                                         [](const auto& item) {
                                             return std::vector<double>{ item.x(), item.y(), item.z(), item.w() };
                                         });
        break;
    default:
        return unsupported("The protocol value type is not supported by the minimal gRPC value codec.");
    }
    return ::grpc::Status::OK;
}

} // namespace

::grpc::Status encodeValue(const InputValueType& value, ValueColumn* column)
{
    if (!column)
    {
        return invalid("The destination value column is null.");
    }
    return std::visit(
        [column](const auto& source) -> ::grpc::Status
        {
            using T = std::decay_t<decltype(source)>;
            if constexpr (std::is_same_v<T, std::string>)
            {
                column->mutable_packed_string()->add_values(source);
                return ::grpc::Status::OK;
            }
            else if constexpr (std::is_same_v<T, std::vector<std::string>>)
            {
                for (const auto& item : source)
                {
                    column->mutable_packed_string()->add_values(item);
                }
                return ::grpc::Status::OK;
            }
            else if constexpr (std::is_same_v<T, array::Array>)
            {
                return encodeArray(source, column);
            }
            else
            {
                return unsupported("Nested string arrays are not supported by the minimal gRPC value codec.");
            }
        },
        value);
}

::grpc::Status encodeValue(const OutputValueType& value, ValueColumn* column)
{
    if (!column)
    {
        return invalid("The destination value column is null.");
    }
    return std::visit(
        [column](const auto& source) -> ::grpc::Status
        {
            using T = std::decay_t<decltype(source)>;
            if constexpr (std::is_same_v<T, std::vector<std::string>>)
            {
                for (const auto& item : source)
                {
                    column->mutable_packed_string()->add_values(item);
                }
                return ::grpc::Status::OK;
            }
            else if constexpr (std::is_same_v<T, array::Array>)
            {
                return encodeArray(source, column);
            }
            else
            {
                return unsupported("Nested string arrays are not supported by the minimal gRPC value codec.");
            }
        },
        value);
}

::grpc::Status decodeValue(const ValueColumn& column, OutputValueType* value)
{
    return decodeValueImplementation(column, value);
}

::grpc::Status decodeValue(const ValueColumn& column, InputValueType* value)
{
    return decodeValueImplementation(column, value);
}

::grpc::Status getValueCount(const ValueColumn& column, size_t* count)
{
    if (!count)
    {
        return invalid("The destination row count is null.");
    }
    const auto setCount = [count](const auto& packed) { *count = static_cast<size_t>(packed.values_size()); };
    switch (column.packed_case())
    {
    case ValueColumn::kPackedBool:
        setCount(column.packed_bool());
        break;
    case ValueColumn::kPackedUchar:
        setCount(column.packed_uchar());
        break;
    case ValueColumn::kPackedInt:
        setCount(column.packed_int());
        break;
    case ValueColumn::kPackedUint:
        setCount(column.packed_uint());
        break;
    case ValueColumn::kPackedInt64:
        setCount(column.packed_int64());
        break;
    case ValueColumn::kPackedUint64:
        setCount(column.packed_uint64());
        break;
    case ValueColumn::kPackedFloat:
        setCount(column.packed_float());
        break;
    case ValueColumn::kPackedDouble:
        setCount(column.packed_double());
        break;
    case ValueColumn::kPackedString:
        setCount(column.packed_string());
        break;
    case ValueColumn::kPackedInt2:
        setCount(column.packed_int2());
        break;
    case ValueColumn::kPackedInt3:
        setCount(column.packed_int3());
        break;
    case ValueColumn::kPackedInt4:
        setCount(column.packed_int4());
        break;
    case ValueColumn::kPackedFloat2:
        setCount(column.packed_float2());
        break;
    case ValueColumn::kPackedFloat3:
        setCount(column.packed_float3());
        break;
    case ValueColumn::kPackedFloat4:
        setCount(column.packed_float4());
        break;
    case ValueColumn::kPackedDouble2:
        setCount(column.packed_double2());
        break;
    case ValueColumn::kPackedDouble3:
        setCount(column.packed_double3());
        break;
    case ValueColumn::kPackedDouble4:
        setCount(column.packed_double4());
        break;
    default:
        return unsupported("The protocol value type is not supported by the minimal gRPC value codec.");
    }
    return ::grpc::Status::OK;
}

} // namespace grpc
} // namespace protos
} // namespace ovsim
} // namespace isaacsim
