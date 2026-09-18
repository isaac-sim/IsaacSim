// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "StatusCodec.hpp"

#include <google/protobuf/wrappers.pb.h>

namespace isaacsim
{
namespace ovsim
{
namespace protos
{
namespace grpc
{

namespace
{

bool isKnownStatusCode(int code)
{
    switch (static_cast<::grpc::StatusCode>(code))
    {
    case ::grpc::StatusCode::OK:
    case ::grpc::StatusCode::CANCELLED:
    case ::grpc::StatusCode::UNKNOWN:
    case ::grpc::StatusCode::INVALID_ARGUMENT:
    case ::grpc::StatusCode::DEADLINE_EXCEEDED:
    case ::grpc::StatusCode::NOT_FOUND:
    case ::grpc::StatusCode::ALREADY_EXISTS:
    case ::grpc::StatusCode::PERMISSION_DENIED:
    case ::grpc::StatusCode::RESOURCE_EXHAUSTED:
    case ::grpc::StatusCode::FAILED_PRECONDITION:
    case ::grpc::StatusCode::ABORTED:
    case ::grpc::StatusCode::OUT_OF_RANGE:
    case ::grpc::StatusCode::UNIMPLEMENTED:
    case ::grpc::StatusCode::INTERNAL:
    case ::grpc::StatusCode::UNAVAILABLE:
    case ::grpc::StatusCode::DATA_LOSS:
    case ::grpc::StatusCode::UNAUTHENTICATED:
        return true;
    case ::grpc::StatusCode::DO_NOT_USE:
        return false;
    }
    return false;
}

} // namespace

::grpc::Status encodeStatus(const ::grpc::Status& source, ::google::rpc::Status* destination)
{
    if (!destination)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The destination status payload is null." };
    }
    if (source.ok())
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "An asynchronous failure payload cannot contain OK." };
    }

    destination->Clear();
    destination->set_code(static_cast<int>(source.error_code()));
    destination->set_message(source.error_message());
    if (!source.error_details().empty())
    {
        ::google::protobuf::BytesValue details;
        details.set_value(source.error_details());
        if (!destination->add_details()->PackFrom(details))
        {
            destination->Clear();
            return { ::grpc::StatusCode::INTERNAL, "The gRPC status details could not be encoded." };
        }
    }
    return ::grpc::Status::OK;
}

::grpc::Status decodeStatus(const ::google::rpc::Status& source, ::grpc::Status* destination)
{
    if (!destination)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The destination gRPC status is null." };
    }
    if (!isKnownStatusCode(source.code()) || source.code() == static_cast<int>(::grpc::StatusCode::OK))
    {
        return { ::grpc::StatusCode::INTERNAL, "The server returned an invalid asynchronous failure status code." };
    }

    std::string details;
    if (!source.details().empty())
    {
        if (source.details_size() != 1)
        {
            return { ::grpc::StatusCode::INTERNAL,
                     "The server returned unsupported asynchronous failure status details." };
        }
        ::google::protobuf::BytesValue packedDetails;
        if (!source.details(0).UnpackTo(&packedDetails))
        {
            return { ::grpc::StatusCode::INTERNAL, "The server returned malformed asynchronous failure status details." };
        }
        details = packedDetails.value();
    }
    *destination = ::grpc::Status(static_cast<::grpc::StatusCode>(source.code()), source.message(), details);
    return ::grpc::Status::OK;
}

} // namespace grpc
} // namespace protos
} // namespace ovsim
} // namespace isaacsim
