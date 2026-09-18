// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <grpcpp/support/status.h>
#include <isaacsim/ovsim/protos/grpc/Export.h>
#include <ovsim/interfaces/data/Data.hpp>

#include <worldstate.pb.h>

namespace isaacsim
{
namespace ovsim
{
namespace protos
{
namespace grpc
{

using InputValueType = ::ovsim::interfaces::data::InputValueType;
using OutputValueType = ::ovsim::interfaces::data::OutputValueType;
using ValueColumn = nvidia::omniverse::simulation::worldstate::v1::ValueColumn;

// The wire domain is symmetric. The current API aliases differ only because input accepts a scalar string convenience
// while vectorized output represents strings as std::vector<std::string>; unifying those aliases is deferred.

/**
 * @brief Encode an OV SIM API input as a protocol value column.
 *
 * @param[in] value Value to encode.
 * @param[out] column Destination protocol column.
 * @return An OK status on success, or an error status when the destination is null or the value is unsupported.
 */
ISAACSIM_OVSIM_PROTOS_GRPC_API ::grpc::Status encodeValue(const InputValueType& value, ValueColumn* column);

/**
 * @brief Encode an OV SIM API output as a protocol value column.
 *
 * @param[in] value Value to encode.
 * @param[out] column Destination protocol column.
 * @return An OK status on success, or an error status when the destination is null or the value is unsupported.
 */
ISAACSIM_OVSIM_PROTOS_GRPC_API ::grpc::Status encodeValue(const OutputValueType& value, ValueColumn* column);

/**
 * @brief Decode a protocol value column to the OV SIM API output representation.
 *
 * @param[in] column Protocol column to decode.
 * @param[out] value Destination API value.
 * @return An OK status on success, or an error status when the destination is null or the column is invalid.
 */
ISAACSIM_OVSIM_PROTOS_GRPC_API ::grpc::Status decodeValue(const ValueColumn& column, OutputValueType* value);

/**
 * @brief Decode a protocol value column to the OV SIM API input representation.
 *
 * @param[in] column Protocol column to decode.
 * @param[out] value Destination API value.
 * @return An OK status on success, or an error status when the destination is null or the column is invalid.
 */
ISAACSIM_OVSIM_PROTOS_GRPC_API ::grpc::Status decodeValue(const ValueColumn& column, InputValueType* value);

/**
 * @brief Get the number of logical rows stored in a protocol value column.
 *
 * @param[in] column Protocol column to inspect.
 * @param[out] count Destination row count.
 * @return An OK status on success, or an error status when the destination is null or the column is unsupported.
 */
ISAACSIM_OVSIM_PROTOS_GRPC_API ::grpc::Status getValueCount(const ValueColumn& column, size_t* count);

} // namespace grpc
} // namespace protos
} // namespace ovsim
} // namespace isaacsim
