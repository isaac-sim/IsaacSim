// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <google/rpc/status.pb.h>
#include <grpcpp/support/status.h>
#include <isaacsim/ovsim/protos/grpc/Export.h>

namespace isaacsim
{
namespace ovsim
{
namespace protos
{
namespace grpc
{

/**
 * @brief Encode a non-OK gRPC status as the standard asynchronous status payload.
 *
 * @param[in] source gRPC status to encode.
 * @param[out] destination Destination protocol payload.
 * @return An OK status on success, or an error status when either the source or destination is invalid.
 */
ISAACSIM_OVSIM_PROTOS_GRPC_API ::grpc::Status encodeStatus(const ::grpc::Status& source,
                                                           ::google::rpc::Status* destination);

/**
 * @brief Decode and validate a standard asynchronous status payload.
 *
 * @param[in] source Protocol payload to decode.
 * @param[out] destination Destination gRPC status.
 * @return An OK status on success, or an error status when either the source or destination is invalid.
 */
ISAACSIM_OVSIM_PROTOS_GRPC_API ::grpc::Status decodeStatus(const ::google::rpc::Status& source,
                                                           ::grpc::Status* destination);

} // namespace grpc
} // namespace protos
} // namespace ovsim
} // namespace isaacsim
