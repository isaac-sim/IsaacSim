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
