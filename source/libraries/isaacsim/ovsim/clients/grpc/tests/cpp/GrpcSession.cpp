// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>
#include <grpcpp/grpcpp.h>
#include <isaacsim/ovsim/clients/grpc/GrpcSession.hpp>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <regex>
#include <simulation.grpc.pb.h>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

using isaacsim::ovsim::clients::grpc::GrpcSession;
namespace controlplane = nvidia::omniverse::simulation::v1;

namespace
{
using Configuration = std::unordered_map<std::string, std::string>;

class BlockingControlPlaneService final : public controlplane::ControlPlaneService::Service
{
public:
    ::grpc::Status CreateSimulation(::grpc::ServerContext*,
                                    const controlplane::CreateSimulationRequest*,
                                    controlplane::CreateSimulationResponse*) override
    {
        return ::grpc::Status::OK;
    }

    ::grpc::Status GetSimulation(::grpc::ServerContext*,
                                 const controlplane::GetSimulationRequest* request,
                                 controlplane::Simulation* response) override
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_getStarted = true;
        m_condition.notify_all();
        m_condition.wait(lock, [this]() { return m_deleted; });
        response->set_simulation_id(request->simulation_id());
        response->set_status(controlplane::DELETING);
        return ::grpc::Status::OK;
    }

    ::grpc::Status DeleteSimulation(::grpc::ServerContext*,
                                    const controlplane::DeleteSimulationRequest*,
                                    controlplane::DeleteSimulationResponse*) override
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_deleted = true;
        }
        m_condition.notify_all();
        return ::grpc::Status::OK;
    }

    bool waitForGet(std::chrono::seconds timeout)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_condition.wait_for(lock, timeout, [this]() { return m_getStarted; });
    }

private:
    std::mutex m_mutex;
    std::condition_variable m_condition;
    bool m_getStarted{ false };
    bool m_deleted{ false };
};

class LifecycleControlPlaneService final : public controlplane::ControlPlaneService::Service
{
public:
    explicit LifecycleControlPlaneService(bool failInitialization = false) : m_failInitialization(failInitialization)
    {
    }

    ::grpc::Status CreateSimulation(::grpc::ServerContext*,
                                    const controlplane::CreateSimulationRequest* request,
                                    controlplane::CreateSimulationResponse*) override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_simulationIds.push_back(request->simulation_id());
        m_activeSimulationId = request->simulation_id();
        return ::grpc::Status::OK;
    }

    ::grpc::Status GetSimulation(::grpc::ServerContext*,
                                 const controlplane::GetSimulationRequest* request,
                                 controlplane::Simulation* response) override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (request->simulation_id() != m_activeSimulationId)
        {
            return { ::grpc::StatusCode::NOT_FOUND, "The simulation has been deleted." };
        }
        response->set_simulation_id(request->simulation_id());
        if (m_failInitialization)
        {
            response->set_status(controlplane::FAILED);
            response->mutable_error()->set_code(static_cast<int>(::grpc::StatusCode::INVALID_ARGUMENT));
            response->mutable_error()->set_message("authored stage rejected");
        }
        else
        {
            response->set_status(controlplane::READY);
        }
        return ::grpc::Status::OK;
    }

    ::grpc::Status DeleteSimulation(::grpc::ServerContext*,
                                    const controlplane::DeleteSimulationRequest* request,
                                    controlplane::DeleteSimulationResponse*) override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (request->simulation_id() != m_activeSimulationId)
        {
            return { ::grpc::StatusCode::NOT_FOUND, "The simulation has been deleted." };
        }
        m_activeSimulationId.clear();
        return ::grpc::Status::OK;
    }

    std::vector<std::string> getSimulationIds() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_simulationIds;
    }

private:
    bool m_failInitialization;
    mutable std::mutex m_mutex;
    std::vector<std::string> m_simulationIds;
    std::string m_activeSimulationId;
};

bool isUuidV4(const std::string& value)
{
    static const std::regex s_kPattern("[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}");
    return std::regex_match(value, s_kPattern);
}

std::unique_ptr<::grpc::Server> startServer(controlplane::ControlPlaneService::Service& service, int* port)
{
    ::grpc::ServerBuilder builder;
    builder.AddListeningPort("127.0.0.1:0", ::grpc::InsecureServerCredentials(), port);
    builder.RegisterService(&service);
    return builder.BuildAndStart();
}
} // namespace

static_assert(!std::is_copy_constructible_v<GrpcSession>);
static_assert(!std::is_copy_assignable_v<GrpcSession>);
static_assert(std::is_nothrow_move_constructible_v<GrpcSession>);
static_assert(std::is_nothrow_move_assignable_v<GrpcSession>);

TEST_SUITE("GrpcSession::construction")
{
    TEST_CASE("requires an explicit endpoint")
    {
        CHECK_THROWS_AS(GrpcSession{ std::nullopt }, std::invalid_argument);
        CHECK_THROWS_AS(GrpcSession(Configuration{}), std::invalid_argument);
        CHECK_THROWS_AS(GrpcSession(Configuration{ { "other", "value" } }), std::invalid_argument);
        CHECK_THROWS_AS(GrpcSession(Configuration{ { "endpoint", "" } }), std::invalid_argument);
    }

    TEST_CASE("preserves standard gRPC channel targets")
    {
        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:50051" } });
        CHECK_EQ(session.getConfiguration().at("endpoint"), "127.0.0.1:50051");
    }

    TEST_CASE("sessions hold independent configurations")
    {
        GrpcSession first(Configuration{ { "endpoint", "host-a:50051" } });
        GrpcSession second(Configuration{ { "endpoint", "host-b:50052" } });
        CHECK_EQ(first.getConfiguration().at("endpoint"), "host-a:50051");
        CHECK_EQ(second.getConfiguration().at("endpoint"), "host-b:50052");
    }
}

TEST_SUITE("GrpcSession::local validation")
{
    TEST_CASE("empty data batches are transport-free no-ops")
    {
        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:1" } });
        const auto numeric = session.read(std::vector<std::string>{}, "position", std::nullopt);
        REQUIRE(std::holds_alternative<isaacsim::common::array::Array>(numeric));
        CHECK_EQ(std::get<isaacsim::common::array::Array>(numeric).shape(), isaacsim::common::array::Shape({ 0, 1 }));

        const auto paths = session.read(std::vector<std::string>{}, "usd-path", std::nullopt);
        REQUIRE(std::holds_alternative<std::vector<std::string>>(paths));
        CHECK(std::get<std::vector<std::string>>(paths).empty());
        CHECK_NOTHROW(session.write(std::vector<std::string>{}, "position", std::string("unused"), std::nullopt));
    }

    TEST_CASE("rejects timestamps and duplicate paths before transport")
    {
        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:1" } });
        const std::vector<std::string> duplicates{ "/World/Cube", "/World/Cube" };
        CHECK_THROWS_AS(session.read(duplicates, "position", std::nullopt), std::invalid_argument);
        CHECK_THROWS_AS(session.read("/World/Cube", "position", 1.0), std::invalid_argument);
        CHECK_THROWS_AS(session.write("/World/Cube", "position", std::string("bad"), 1.0), std::invalid_argument);
        CHECK_THROWS_AS(session.write("/World/Cube", "size", std::string("bad"), std::nullopt), std::invalid_argument);
    }

    TEST_CASE("lifecycle calls require a created simulation")
    {
        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:1" } });
        CHECK_FALSE(session.closeStage());
        CHECK_FALSE(session.importStageFromString(""));
        CHECK_THROWS_AS(session.initialize(), std::runtime_error);
        CHECK_THROWS_AS(session.step(), std::runtime_error);
    }

    TEST_CASE("initialization polling does not block deletion")
    {
        BlockingControlPlaneService service;
        ::grpc::ServerBuilder builder;
        int port = 0;
        builder.AddListeningPort("127.0.0.1:0", ::grpc::InsecureServerCredentials(), &port);
        builder.RegisterService(&service);
        auto server = builder.BuildAndStart();
        REQUIRE(server);

        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:" + std::to_string(port) } });
        REQUIRE(session.importStageFromString("#usda 1.0"));
        auto initialization = std::async(std::launch::async, [&session]() { session.initialize(); });
        REQUIRE(service.waitForGet(std::chrono::seconds(5)));

        CHECK(session.closeStage());
        REQUIRE_EQ(initialization.wait_for(std::chrono::seconds(5)), std::future_status::ready);
        CHECK_THROWS_AS(initialization.get(), std::runtime_error);
        server->Shutdown();
    }

    TEST_CASE("propagates an asynchronous initialization root cause")
    {
        LifecycleControlPlaneService service(true);
        int port = 0;
        auto server = startServer(service, &port);
        REQUIRE(server);

        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:" + std::to_string(port) } });
        REQUIRE(session.importStageFromString("#usda 1.0"));
        CHECK_THROWS_WITH_AS(session.initialize(),
                             "Remote simulation initialization failed with gRPC status 3: authored stage rejected",
                             std::runtime_error);
        CHECK(session.closeStage());
        server->Shutdown();
    }

    TEST_CASE("generates a fresh UUID for every accepted lifecycle")
    {
        LifecycleControlPlaneService service;
        int port = 0;
        auto server = startServer(service, &port);
        REQUIRE(server);

        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:" + std::to_string(port) } });
        REQUIRE(session.importStageFromString("#usda 1.0"));
        REQUIRE(session.closeStage());
        REQUIRE(session.importStageFromString("#usda 1.0"));

        const auto simulationIds = service.getSimulationIds();
        REQUIRE_EQ(simulationIds.size(), 2);
        CHECK(isUuidV4(simulationIds[0]));
        CHECK(isUuidV4(simulationIds[1]));
        CHECK_NE(simulationIds[0], simulationIds[1]);
        CHECK(session.closeStage());
        server->Shutdown();
    }
}

TEST_SUITE("GrpcSession::unsupported surface")
{
    TEST_CASE("unsupported methods remain explicit")
    {
        GrpcSession session(Configuration{ { "endpoint", "127.0.0.1:1" } });
        CHECK_THROWS_AS(session.createStage(), std::logic_error);
        CHECK_THROWS_AS(session.openStage("scene.usd"), std::logic_error);
        CHECK_THROWS_AS(session.saveStage("scene.usd"), std::logic_error);
        CHECK_THROWS_AS(session.exportStageToString(), std::logic_error);
        CHECK_THROWS_AS(session.invalidate(), std::logic_error);
        CHECK_THROWS_AS(session.play(), std::logic_error);
        CHECK_THROWS_AS(session.pause(), std::logic_error);
        CHECK_THROWS_AS(session.stop(), std::logic_error);
    }
}
