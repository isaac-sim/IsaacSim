// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>
#include <grpcpp/grpcpp.h>
#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/ovsim/clients/grpc/GrpcSession.hpp>
#include <isaacsim/ovsim/clients/local/control/authoring/Authoring.hpp>
#include <isaacsim/ovsim/clients/local/data/Data.hpp>
#include <isaacsim/ovsim/servers/grpc/Server.hpp>

#include <chrono>
#include <simulation.grpc.pb.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>
#include <worldstate.grpc.pb.h>

// The Windows SDK defines IN as an annotation macro, which collides with the generated FilterOp enumerator.
#if defined(IN)
#    undef IN
#endif

namespace array = isaacsim::common::array;
namespace grpc_client = isaacsim::ovsim::clients::grpc;
namespace local = isaacsim::ovsim::clients::local;
namespace server_api = isaacsim::ovsim::servers::grpc;
namespace controlplane = nvidia::omniverse::simulation::v1;
namespace worldstate = nvidia::omniverse::simulation::worldstate::v1;

namespace
{

constexpr auto g_kSimulationInitializationTimeout = std::chrono::seconds(30);
constexpr auto g_kSimulationStatusPollInterval = std::chrono::milliseconds(100);

std::string authorFallingCube()
{
    if (!local::control::authoring::createStage())
    {
        throw std::runtime_error("Could not create the integration-test stage.");
    }
    const std::string cube = "/World/Cube";
    local::control::authoring::definePrim(cube, "Cube");
    local::data::write(cube, "size", array::Array(1.0), std::nullopt);
    local::data::write(
        cube, "position", array::Array(std::vector<std::vector<double>>{ { 0.0, 0.0, 2.0 } }), std::nullopt);
    local::control::authoring::definePrim(cube, "ColliderBody");
    local::control::authoring::definePrim(cube, "RigidBody");

    const std::string physicsScene = "/World/PhysicsScene";
    local::control::authoring::definePrim(physicsScene, "PhysicsScene");
    local::data::write(physicsScene, "physics:gravityDirection",
                       array::Array(std::vector<std::vector<double>>{ { 0.0, 0.0, -1.0 } }), std::nullopt);
    local::data::write(physicsScene, "physics:gravityMagnitude", array::Array(9.81), std::nullopt);

    const std::string usd = local::control::authoring::exportStageToString();
    if (!local::control::authoring::closeStage())
    {
        throw std::runtime_error("Could not close the integration-test authoring stage.");
    }
    return usd;
}

double readHeight(grpc_client::GrpcSession& client)
{
    const auto result = client.read("/World/Cube", "position", std::nullopt);
    return std::get<array::Array>(result).get<std::vector<std::vector<double>>>()[0][2];
}

worldstate::ReadWorldStateRequest createLatestReadRequest(const std::string& simulationId,
                                                          const std::vector<std::string>& paths,
                                                          const std::string& attribute = {})
{
    worldstate::ReadWorldStateRequest request;
    request.set_simulation_id(simulationId);
    auto* read = request.mutable_read();
    read->add_select("usd-path");
    if (!attribute.empty())
    {
        read->add_select(attribute);
    }
    (void)read->mutable_simulation_time_range();
    read->set_change_history(false);
    read->set_timestamped(false);
    auto* predicate = read->mutable_filter()->add_predicates();
    predicate->set_attribute("usd-path");
    predicate->set_op(worldstate::IN);
    auto* packedPaths = predicate->mutable_values()->mutable_packed_string();
    for (const auto& path : paths)
    {
        packedPaths->add_values(path);
    }
    return request;
}

worldstate::WriteWorldStateRequest createScalarAttributeWriteRequest(const std::string& simulationId,
                                                                     int64_t simulationTimeNs,
                                                                     const std::vector<std::string>& paths,
                                                                     const std::string& attribute,
                                                                     double value)
{
    worldstate::WriteWorldStateRequest request;
    request.set_simulation_id(simulationId);
    request.set_simulation_time_ns(simulationTimeNs);
    auto* update = request.add_write()->mutable_update();
    update->set_write_mode(worldstate::UPSERT);
    update->mutable_keys()->mutable_attribute()->set_attribute_name("usd-path");
    auto* packedPaths = update->mutable_keys()->mutable_values()->mutable_packed_string();
    for (const auto& path : paths)
    {
        packedPaths->add_values(path);
    }
    auto* values = update->add_values();
    values->mutable_attribute()->set_attribute_name(attribute);
    for (size_t index = 0; index < paths.size(); ++index)
    {
        values->mutable_values()->mutable_packed_double()->add_values(value);
    }
    return request;
}

worldstate::WriteWorldStateRequest createPositionWriteRequest(const std::string& simulationId,
                                                              int64_t simulationTimeNs,
                                                              const std::vector<std::string>& paths)
{
    worldstate::WriteWorldStateRequest request;
    request.set_simulation_id(simulationId);
    request.set_simulation_time_ns(simulationTimeNs);
    auto* update = request.add_write()->mutable_update();
    update->set_write_mode(worldstate::UPSERT);
    update->mutable_keys()->mutable_attribute()->set_attribute_name("usd-path");
    auto* packedPaths = update->mutable_keys()->mutable_values()->mutable_packed_string();
    for (const auto& path : paths)
    {
        packedPaths->add_values(path);
    }
    auto* position = update->add_values();
    position->mutable_attribute()->set_attribute_name("position");
    auto* packedPositions = position->mutable_values()->mutable_packed_double3();
    for (size_t index = 0; index < paths.size(); ++index)
    {
        auto* value = packedPositions->add_values();
        value->set_x(0.0);
        value->set_y(0.0);
        value->set_z(2.0);
    }
    return request;
}

worldstate::WriteWorldStateRequest createStepRequest(const std::string& simulationId, int64_t simulationTimeNs)
{
    worldstate::WriteWorldStateRequest request;
    request.set_simulation_id(simulationId);
    request.set_simulation_time_ns(simulationTimeNs);
    auto* watermark = request.add_write()->mutable_update_simulation_time_ns();
    watermark->set_simulation_time_ns(simulationTimeNs);
    watermark->set_scope(worldstate::ALL);
    return request;
}

} // namespace

TEST_SUITE("OV SIM gRPC server")
{
    TEST_CASE("requires an explicit listen address")
    {
        CHECK_THROWS_AS(server_api::Server(""), std::invalid_argument);
    }

    TEST_CASE("returns canonical statuses to direct gRPC callers")
    {
        const std::string usd = authorFallingCube();
        server_api::Server server("127.0.0.1:0");
        const auto channel = ::grpc::CreateChannel(server.getEndpoint(), ::grpc::InsecureChannelCredentials());
        auto control = controlplane::ControlPlaneService::NewStub(channel);
        auto world = worldstate::WorldStateService::NewStub(channel);
        const std::string simulationId = "direct-status-test";

        controlplane::CreateSimulationRequest create;
        create.set_simulation_id(simulationId);
        create.set_usd_content(usd);
        controlplane::CreateSimulationResponse createResponse;
        ::grpc::ClientContext createContext;
        REQUIRE(control->CreateSimulation(&createContext, create, &createResponse).ok());

        ::grpc::ClientContext duplicateContext;
        CHECK_EQ(control->CreateSimulation(&duplicateContext, create, &createResponse).error_code(),
                 ::grpc::StatusCode::ALREADY_EXISTS);
        create.set_simulation_id("competing-id");
        ::grpc::ClientContext competingContext;
        CHECK_EQ(control->CreateSimulation(&competingContext, create, &createResponse).error_code(),
                 ::grpc::StatusCode::RESOURCE_EXHAUSTED);

        controlplane::Simulation simulation;
        const auto initializationDeadline = std::chrono::steady_clock::now() + g_kSimulationInitializationTimeout;
        while (std::chrono::steady_clock::now() < initializationDeadline)
        {
            controlplane::GetSimulationRequest get;
            get.set_simulation_id(simulationId);
            ::grpc::ClientContext getContext;
            REQUIRE(control->GetSimulation(&getContext, get, &simulation).ok());
            if (simulation.status() == controlplane::READY)
            {
                break;
            }
            REQUIRE_NE(simulation.status(), controlplane::FAILED);
            std::this_thread::sleep_for(g_kSimulationStatusPollInterval);
        }
        INFO("Last simulation status: " << simulation.status());
        REQUIRE_EQ(simulation.status(), controlplane::READY);

        auto emptyRead = createLatestReadRequest(simulationId, {});
        worldstate::ReadWorldStateResponse readResponse;
        ::grpc::ClientContext emptyReadContext;
        const auto emptyReadStatus = world->ReadWorldState(&emptyReadContext, emptyRead, &readResponse);
        INFO(emptyReadStatus.error_message());
        CHECK(emptyReadStatus.ok());
        CHECK(readResponse.result_sets().empty());

        auto automaticLatestRead = createLatestReadRequest(simulationId, { "/World/Cube" }, "size");
        automaticLatestRead.mutable_read()->clear_change_history();
        automaticLatestRead.mutable_read()->clear_timestamped();
        readResponse.Clear();
        ::grpc::ClientContext automaticLatestReadContext;
        const auto automaticLatestReadStatus =
            world->ReadWorldState(&automaticLatestReadContext, automaticLatestRead, &readResponse);
        INFO(automaticLatestReadStatus.error_message());
        CHECK(automaticLatestReadStatus.ok());
        CHECK_EQ(readResponse.result_sets_size(), 1);

        auto unfilteredRead = createLatestReadRequest(simulationId, { "/World/Cube" }, "size");
        unfilteredRead.mutable_read()->clear_filter();
        ::grpc::ClientContext unfilteredReadContext;
        CHECK_EQ(world->ReadWorldState(&unfilteredReadContext, unfilteredRead, &readResponse).error_code(),
                 ::grpc::StatusCode::UNIMPLEMENTED);

        auto duplicateRead = createLatestReadRequest(simulationId, { "/World/Cube", "/World/Cube" });
        ::grpc::ClientContext duplicateReadContext;
        CHECK_EQ(world->ReadWorldState(&duplicateReadContext, duplicateRead, &readResponse).error_code(),
                 ::grpc::StatusCode::INVALID_ARGUMENT);

        worldstate::WriteWorldStateResponse writeResponse;
        auto emptyWrite = createPositionWriteRequest(simulationId, 123, {});
        ::grpc::ClientContext emptyWriteContext;
        const auto emptyWriteStatus = world->WriteWorldState(&emptyWriteContext, emptyWrite, &writeResponse);
        INFO(emptyWriteStatus.error_message());
        CHECK(emptyWriteStatus.ok());

        auto sizeWrite = createScalarAttributeWriteRequest(simulationId, 16'666'667, { "/World/Cube" }, "size", 2.0);
        ::grpc::ClientContext sizeWriteContext;
        const auto sizeWriteStatus = world->WriteWorldState(&sizeWriteContext, sizeWrite, &writeResponse);
        INFO(sizeWriteStatus.error_message());
        REQUIRE(sizeWriteStatus.ok());

        auto sizeRead = createLatestReadRequest(simulationId, { "/World/Cube" }, "size");
        ::grpc::ClientContext sizeReadContext;
        const auto sizeReadStatus = world->ReadWorldState(&sizeReadContext, sizeRead, &readResponse);
        INFO(sizeReadStatus.error_message());
        REQUIRE(sizeReadStatus.ok());
        REQUIRE_EQ(readResponse.result_sets_size(), 1);
        REQUIRE_EQ(readResponse.result_sets(0).columns_size(), 2);
        REQUIRE_EQ(readResponse.result_sets(0).columns(1).packed_double().values_size(), 1);
        CHECK_EQ(readResponse.result_sets(0).columns(1).packed_double().values(0), doctest::Approx(2.0));

        auto missingWrite =
            createScalarAttributeWriteRequest(simulationId, 16'666'667, { "/World/Cube" }, "missing-attribute", 1.0);
        ::grpc::ClientContext missingWriteContext;
        CHECK_EQ(world->WriteWorldState(&missingWriteContext, missingWrite, &writeResponse).error_code(),
                 ::grpc::StatusCode::INVALID_ARGUMENT);

        auto write = createPositionWriteRequest(simulationId, 16'666'667, { "/World/Cube" });
        ::grpc::ClientContext writeContext;
        REQUIRE(world->WriteWorldState(&writeContext, write, &writeResponse).ok());
        ::grpc::ClientContext replayWriteContext;
        CHECK_EQ(world->WriteWorldState(&replayWriteContext, write, &writeResponse).error_code(),
                 ::grpc::StatusCode::ABORTED);

        auto stepRequest = createStepRequest(simulationId, 16'666'667);
        ::grpc::ClientContext stepContext;
        REQUIRE(world->WriteWorldState(&stepContext, stepRequest, &writeResponse).ok());
        ::grpc::ClientContext replayStepContext;
        CHECK(world->WriteWorldState(&replayStepContext, stepRequest, &writeResponse).ok());

        readResponse.Clear();
        ::grpc::ClientContext advancedEmptyReadContext;
        const auto advancedEmptyReadStatus = world->ReadWorldState(&advancedEmptyReadContext, emptyRead, &readResponse);
        INFO(advancedEmptyReadStatus.error_message());
        CHECK(advancedEmptyReadStatus.ok());
        CHECK(readResponse.result_sets().empty());
        CHECK_EQ(readResponse.read_watermark_ns(), 16'666'667);
        CHECK_EQ(readResponse.available_watermark_ns(), 16'666'667);

        auto emptyProjectionRead = createLatestReadRequest(simulationId, { "/World/Cube" });
        emptyProjectionRead.mutable_read()->clear_select();
        readResponse.Clear();
        ::grpc::ClientContext emptyProjectionReadContext;
        const auto emptyProjectionReadStatus =
            world->ReadWorldState(&emptyProjectionReadContext, emptyProjectionRead, &readResponse);
        INFO(emptyProjectionReadStatus.error_message());
        CHECK(emptyProjectionReadStatus.ok());
        CHECK(readResponse.result_sets().empty());
        CHECK_EQ(readResponse.read_watermark_ns(), 16'666'667);
        CHECK_EQ(readResponse.available_watermark_ns(), 16'666'667);

        auto skippedStepRequest = createStepRequest(simulationId, 50'000'000);
        ::grpc::ClientContext skippedStepContext;
        CHECK_EQ(world->WriteWorldState(&skippedStepContext, skippedStepRequest, &writeResponse).error_code(),
                 ::grpc::StatusCode::INVALID_ARGUMENT);

        controlplane::DeleteSimulationRequest deleteRequest;
        deleteRequest.set_simulation_id(simulationId);
        controlplane::DeleteSimulationResponse deleteResponse;
        ::grpc::ClientContext deleteContext;
        REQUIRE(control->DeleteSimulation(&deleteContext, deleteRequest, &deleteResponse).ok());
        server.shutdown();
    }

    TEST_CASE("runs lifecycle, vector data, and drift-free steps in process")
    {
        const std::string usd = authorFallingCube();
        server_api::Server server("127.0.0.1:0");
        grpc_client::GrpcSession client(
            std::unordered_map<std::string, std::string>{ { "endpoint", server.getEndpoint() } });

        REQUIRE(client.importStageFromString(usd));
        client.initialize();
        const auto pathResult = client.read("/World/Cube", "usd-path", std::nullopt);
        REQUIRE(std::holds_alternative<std::vector<std::string>>(pathResult));
        const std::vector<std::string> expectedPaths{ "/World/Cube" };
        CHECK_EQ(std::get<std::vector<std::string>>(pathResult), expectedPaths);

        const double initialHeight = readHeight(client);
        CHECK(initialHeight == doctest::Approx(2.0));
        const auto sizeResult = client.read("/World/Cube", "size", std::nullopt);
        REQUIRE(std::holds_alternative<array::Array>(sizeResult));
        CHECK(std::get<array::Array>(sizeResult).get<std::vector<std::vector<double>>>()[0][0] == doctest::Approx(1.0));

        const array::Array floatPosition(std::vector<std::vector<float>>{ { 0.0F, 0.0F, 3.0F } });
        client.write("/World/Cube", "position", floatPosition, std::nullopt);
        CHECK(readHeight(client) == doctest::Approx(3.0));
        CHECK_THROWS_AS(client.write("/World/Cube", "position", floatPosition, std::nullopt), std::runtime_error);
        client.step();

        const array::Array doublePosition(std::vector<std::vector<double>>{ { 0.0, 0.0, 4.0 } });
        client.write("/World/Cube", "position", doublePosition, std::nullopt);
        CHECK(readHeight(client) == doctest::Approx(4.0));
        client.step();

        const double beforeFall = readHeight(client);
        for (size_t index = 0; index < 10; ++index)
        {
            client.step();
        }
        CHECK(readHeight(client) < beforeFall);

        grpc_client::GrpcSession competing(
            std::unordered_map<std::string, std::string>{ { "endpoint", server.getEndpoint() } });
        CHECK_FALSE(competing.importStageFromString(usd));

        CHECK(client.closeStage());
        CHECK(client.closeStage());
        server.shutdown();
    }
}
