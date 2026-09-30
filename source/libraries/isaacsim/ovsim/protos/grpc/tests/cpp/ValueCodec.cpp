// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>
#include <isaacsim/common/array/Array.hpp>

#include <StatusCodec.hpp>
#include <ValueCodec.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace array = isaacsim::common::array;
namespace codec = isaacsim::ovsim::protos::grpc;

namespace
{

template <typename T>
void checkScalarRoundTrip(const std::vector<T>& values)
{
    const array::Array source(values);
    codec::ValueColumn column;
    REQUIRE(codec::encodeValue(codec::OutputValueType(source.reshape(array::Shape{ -1, 1 })), &column).ok());
    size_t count = 0;
    REQUIRE(codec::getValueCount(column, &count).ok());
    CHECK_EQ(count, values.size());

    codec::OutputValueType decoded;
    REQUIRE(codec::decodeValue(column, &decoded).ok());
    const auto& result = std::get<array::Array>(decoded);
    CHECK_EQ(result.dtype(), source.dtype());
    CHECK_EQ(result.shape(), array::Shape({ static_cast<int64_t>(values.size()), static_cast<int64_t>(1) }));
    CHECK_EQ(result.get<std::vector<std::vector<T>>>(),
             array::Array(values).reshape(array::Shape{ -1, 1 }).get<std::vector<std::vector<T>>>());
}

template <typename T>
void checkVectorRoundTrip(const std::vector<std::vector<T>>& values)
{
    const array::Array source(values);
    codec::ValueColumn column;
    REQUIRE(codec::encodeValue(codec::OutputValueType(source), &column).ok());
    size_t count = 0;
    REQUIRE(codec::getValueCount(column, &count).ok());
    CHECK_EQ(count, values.size());

    codec::OutputValueType decoded;
    REQUIRE(codec::decodeValue(column, &decoded).ok());
    const auto& result = std::get<array::Array>(decoded);
    CHECK_EQ(result.dtype(), source.dtype());
    CHECK_EQ(result.shape(), source.shape());
    CHECK_EQ(result.get<std::vector<std::vector<T>>>(), values);
}

} // namespace

TEST_SUITE("OV SIM gRPC value codec")
{
    TEST_CASE("round-trips the approved scalar types")
    {
        checkScalarRoundTrip<bool>({ false, true });
        checkScalarRoundTrip<uint8_t>({ 0, 255 });
        checkScalarRoundTrip<int32_t>({ -2, 3 });
        checkScalarRoundTrip<uint32_t>({ 2, 3 });
        checkScalarRoundTrip<int64_t>({ -4, 5 });
        checkScalarRoundTrip<uint64_t>({ 4, 5 });
        checkScalarRoundTrip<float>({ 1.25F, -2.5F });
        checkScalarRoundTrip<double>({ 1.25, -2.5 });
    }

    TEST_CASE("round-trips the approved fixed-vector types")
    {
        checkVectorRoundTrip<int32_t>({ { 1, 2 }, { 3, 4 } });
        checkVectorRoundTrip<int32_t>({ { 1, 2, 3 }, { 4, 5, 6 } });
        checkVectorRoundTrip<int32_t>({ { 1, 2, 3, 4 }, { 5, 6, 7, 8 } });
        checkVectorRoundTrip<float>({ { 1.0F, 2.0F }, { 3.0F, 4.0F } });
        checkVectorRoundTrip<float>({ { 1.0F, 2.0F, 3.0F }, { 4.0F, 5.0F, 6.0F } });
        checkVectorRoundTrip<float>({ { 1.0F, 2.0F, 3.0F, 4.0F }, { 5.0F, 6.0F, 7.0F, 8.0F } });
        checkVectorRoundTrip<double>({ { 1.0, 2.0 }, { 3.0, 4.0 } });
        checkVectorRoundTrip<double>({ { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 } });
        checkVectorRoundTrip<double>({ { 1.0, 2.0, 3.0, 4.0 }, { 5.0, 6.0, 7.0, 8.0 } });
    }

    TEST_CASE("round-trips strings and empty fixed vectors")
    {
        codec::ValueColumn scalarString;
        REQUIRE(codec::encodeValue(codec::InputValueType(std::string("scalar input")), &scalarString).ok());
        codec::OutputValueType decodedScalarString;
        REQUIRE(codec::decodeValue(scalarString, &decodedScalarString).ok());
        CHECK_EQ(std::get<std::vector<std::string>>(decodedScalarString), std::vector<std::string>({ "scalar input" }));

        codec::ValueColumn strings;
        REQUIRE(codec::encodeValue(codec::OutputValueType(std::vector<std::string>{ "a", "b" }), &strings).ok());
        size_t stringCount = 0;
        REQUIRE(codec::getValueCount(strings, &stringCount).ok());
        CHECK_EQ(stringCount, 2);
        codec::OutputValueType decodedStrings;
        REQUIRE(codec::decodeValue(strings, &decodedStrings).ok());
        CHECK_EQ(std::get<std::vector<std::string>>(decodedStrings), std::vector<std::string>({ "a", "b" }));
        codec::InputValueType decodedInputStrings;
        REQUIRE(codec::decodeValue(strings, &decodedInputStrings).ok());
        CHECK_EQ(std::get<std::vector<std::string>>(decodedInputStrings), std::vector<std::string>({ "a", "b" }));

        codec::ValueColumn emptyVectors;
        emptyVectors.mutable_packed_float3();
        size_t emptyCount = 1;
        REQUIRE(codec::getValueCount(emptyVectors, &emptyCount).ok());
        CHECK_EQ(emptyCount, 0);
        codec::OutputValueType decodedVectors;
        REQUIRE(codec::decodeValue(emptyVectors, &decodedVectors).ok());
        CHECK_EQ(std::get<array::Array>(decodedVectors).shape(), array::Shape({ 0, 3 }));
    }

    TEST_CASE("reports unsupported protocol and array shapes")
    {
        codec::ValueColumn unsupportedProtocol;
        unsupportedProtocol.mutable_packed_quatf();
        codec::OutputValueType decoded;
        CHECK_EQ(codec::decodeValue(unsupportedProtocol, &decoded).error_code(), ::grpc::StatusCode::UNIMPLEMENTED);
        size_t count = 0;
        CHECK_EQ(codec::getValueCount(unsupportedProtocol, &count).error_code(), ::grpc::StatusCode::UNIMPLEMENTED);
        CHECK_EQ(codec::getValueCount(unsupportedProtocol, nullptr).error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);

        codec::ValueColumn unsupportedArray;
        const array::Array byteVectors(std::vector<std::vector<uint8_t>>{ { 1, 2 } });
        CHECK_EQ(codec::encodeValue(codec::OutputValueType(byteVectors), &unsupportedArray).error_code(),
                 ::grpc::StatusCode::UNIMPLEMENTED);
    }
}

TEST_SUITE("OV SIM gRPC status codec")
{
    TEST_CASE("round-trips a non-OK status including transport details")
    {
        const ::grpc::Status source(::grpc::StatusCode::INVALID_ARGUMENT, "bad authored scene", "binary details");
        ::google::rpc::Status payload;
        REQUIRE(codec::encodeStatus(source, &payload).ok());
        CHECK_EQ(payload.code(), static_cast<int>(::grpc::StatusCode::INVALID_ARGUMENT));
        CHECK_EQ(payload.message(), "bad authored scene");
        CHECK_EQ(payload.details_size(), 1);

        ::grpc::Status decoded;
        REQUIRE(codec::decodeStatus(payload, &decoded).ok());
        CHECK_EQ(decoded.error_code(), source.error_code());
        CHECK_EQ(decoded.error_message(), source.error_message());
        CHECK_EQ(decoded.error_details(), source.error_details());
    }

    TEST_CASE("rejects statuses that cannot represent asynchronous failures")
    {
        ::google::rpc::Status payload;
        CHECK_EQ(codec::encodeStatus(::grpc::Status::OK, &payload).error_code(), ::grpc::StatusCode::INVALID_ARGUMENT);
        CHECK_EQ(codec::encodeStatus(::grpc::Status(::grpc::StatusCode::INTERNAL, "failure"), nullptr).error_code(),
                 ::grpc::StatusCode::INVALID_ARGUMENT);

        ::grpc::Status decoded;
        CHECK_EQ(codec::decodeStatus(payload, &decoded).error_code(), ::grpc::StatusCode::INTERNAL);
        payload.set_code(1000);
        CHECK_EQ(codec::decodeStatus(payload, &decoded).error_code(), ::grpc::StatusCode::INTERNAL);
        payload.set_code(static_cast<int>(::grpc::StatusCode::INTERNAL));
        payload.set_message("failure");
        payload.add_details()->set_type_url("type.googleapis.com/example.Unsupported");
        CHECK_EQ(codec::decodeStatus(payload, &decoded).error_code(), ::grpc::StatusCode::INTERNAL);
    }
}
