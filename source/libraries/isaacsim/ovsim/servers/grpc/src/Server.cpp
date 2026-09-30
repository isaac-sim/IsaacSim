// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "isaacsim/ovsim/servers/grpc/Server.hpp"

#include "HostedSimulation.hpp"
#include "StatusCodec.hpp"
#include "ValueCodec.hpp"

#include <grpcpp/grpcpp.h>
#include <isaacsim/physics_engines/ovphysx/Backend.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <simulation.grpc.pb.h>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include <worldstate.grpc.pb.h>

// The Windows SDK defines IN as an annotation macro, which collides with the generated FilterOp enumerator.
#if defined(IN)
#    undef IN
#endif

namespace isaacsim
{
namespace ovsim
{
namespace servers
{
namespace grpc
{

namespace controlplane = nvidia::omniverse::simulation::v1;
namespace protocol = isaacsim::ovsim::protos::grpc;
namespace worldstate = nvidia::omniverse::simulation::worldstate::v1;

namespace
{

std::vector<std::string> packedStrings(const worldstate::ValueColumn& column)
{
    if (column.packed_case() != worldstate::ValueColumn::kPackedString)
    {
        return {};
    }
    return { column.packed_string().values().begin(), column.packed_string().values().end() };
}

bool hasDuplicates(const std::vector<std::string>& values)
{
    std::unordered_set<std::string> unique;
    for (const auto& value : values)
    {
        if (!unique.emplace(value).second)
        {
            return true;
        }
    }
    return false;
}

::grpc::Status validateWatermark(const worldstate::WriteWorldStateRequest& request,
                                 const worldstate::UpdateSimulationTimeNs& watermark)
{
    if (watermark.simulation_time_ns() != request.simulation_time_ns())
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The request and UpdateSimulationTimeNs timestamps must match." };
    }
    if ((watermark.scope() != worldstate::WATERMARK_SCOPE_UNSPECIFIED && watermark.scope() != worldstate::ALL) ||
        !watermark.attributes().empty())
    {
        return { ::grpc::StatusCode::UNIMPLEMENTED, "Only the global ALL watermark scope is supported." };
    }
    return ::grpc::Status::OK;
}

class ControlPlaneService final : public controlplane::ControlPlaneService::Service
{
public:
    explicit ControlPlaneService(HostedSimulation& simulation) : m_simulation(simulation)
    {
    }

    ::grpc::Status CreateSimulation(::grpc::ServerContext*,
                                    const controlplane::CreateSimulationRequest* request,
                                    controlplane::CreateSimulationResponse*) override
    {
        return m_simulation.createSimulation(request->simulation_id(), request->usd_content());
    }

    ::grpc::Status GetSimulation(::grpc::ServerContext*,
                                 const controlplane::GetSimulationRequest* request,
                                 controlplane::Simulation* response) override
    {
        HostedSimulation::Snapshot snapshot;
        const auto status = m_simulation.getSnapshot(request->simulation_id(), &snapshot);
        if (!status.ok())
        {
            return status;
        }
        response->set_simulation_id(snapshot.simulationId);
        switch (snapshot.state)
        {
        case HostedSimulation::State::eInitializing:
            response->set_status(controlplane::INITIALIZING);
            break;
        case HostedSimulation::State::eReady:
            response->set_status(controlplane::READY);
            break;
        case HostedSimulation::State::eFailed:
            response->set_status(controlplane::FAILED);
            {
                const auto errorStatus = protocol::encodeStatus(snapshot.error, response->mutable_error());
                if (!errorStatus.ok())
                {
                    return errorStatus;
                }
            }
            break;
        case HostedSimulation::State::eDeleting:
            response->set_status(controlplane::DELETING);
            break;
        }
        return ::grpc::Status::OK;
    }

    ::grpc::Status DeleteSimulation(::grpc::ServerContext*,
                                    const controlplane::DeleteSimulationRequest* request,
                                    controlplane::DeleteSimulationResponse*) override
    {
        return m_simulation.deleteSimulation(request->simulation_id());
    }

private:
    HostedSimulation& m_simulation;
};

class WorldStateService final : public worldstate::WorldStateService::Service
{
public:
    explicit WorldStateService(HostedSimulation& simulation) : m_simulation(simulation)
    {
    }

    ::grpc::Status ReadWorldState(::grpc::ServerContext*,
                                  const worldstate::ReadWorldStateRequest* request,
                                  worldstate::ReadWorldStateResponse* response) override
    {
        if (!request->has_read())
        {
            return { ::grpc::StatusCode::UNIMPLEMENTED, "Read continuations are not supported." };
        }
        const auto& read = request->read();

        const std::vector<std::string> selected(read.select().begin(), read.select().end());
        if (selected.size() > 2 || hasDuplicates(selected) ||
            (!selected.empty() && std::count(selected.begin(), selected.end(), "usd-path") != 1))
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT,
                     "The minimal server accepts a duplicate-free projection of usd-path and one attribute." };
        }
        if (read.simulation_time_range().has_simulation_time_ns() ||
            read.simulation_time_range().has_start_simulation_time_ns() || read.change_history() || read.timestamped())
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT,
                     "Only the latest-value preset (no bounds, change_history/timestamped unset or false) is "
                     "supported." };
        }
        if (read.filter().predicates_size() != 1)
        {
            return { ::grpc::StatusCode::UNIMPLEMENTED, "Only a single usd-path IN filter is supported." };
        }
        const auto& predicate = read.filter().predicates(0);
        if (predicate.attribute() != "usd-path" || predicate.op() != worldstate::IN || predicate.negate())
        {
            return { ::grpc::StatusCode::UNIMPLEMENTED, "Only a non-negated usd-path IN filter is supported." };
        }
        if (predicate.values().packed_case() != worldstate::ValueColumn::kPackedString)
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "The usd-path IN filter values must be strings." };
        }
        std::vector<std::string> paths = packedStrings(predicate.values());
        if (hasDuplicates(paths))
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "The usd-path IN filter must contain unique string paths." };
        }
        if (paths.empty() || selected.empty())
        {
            HostedSimulation::Snapshot snapshot;
            const auto status = m_simulation.getSnapshot(request->simulation_id(), &snapshot);
            if (!status.ok())
            {
                return status;
            }
            if (snapshot.state != HostedSimulation::State::eReady)
            {
                return m_simulation.getReadyStatus(request->simulation_id());
            }
            response->set_read_watermark_ns(snapshot.simulationTimeNs);
            response->set_available_watermark_ns(snapshot.simulationTimeNs);
            return ::grpc::Status::OK;
        }

        const auto attributeIterator =
            std::find_if(selected.begin(), selected.end(), [](const std::string& item) { return item != "usd-path"; });
        int64_t simulationTimeNs = 0;
        ::ovsim::interfaces::data::OutputValueType value = std::vector<std::string>{};
        if (attributeIterator != selected.end())
        {
            const auto status =
                m_simulation.read(request->simulation_id(), paths, *attributeIterator, &value, &simulationTimeNs);
            if (!status.ok())
            {
                return status;
            }
        }
        else
        {
            HostedSimulation::Snapshot snapshot;
            const auto status = m_simulation.getSnapshot(request->simulation_id(), &snapshot);
            if (!status.ok())
            {
                return status;
            }
            if (snapshot.state != HostedSimulation::State::eReady)
            {
                return m_simulation.getReadyStatus(request->simulation_id());
            }
            simulationTimeNs = snapshot.simulationTimeNs;
        }

        auto* result = response->add_result_sets();
        result->set_simulation_time_ns(simulationTimeNs);
        for (const auto& attribute : selected)
        {
            result->add_attributes(attribute);
            auto* column = result->add_columns();
            if (attribute == "usd-path")
            {
                for (const auto& path : paths)
                {
                    column->mutable_packed_string()->add_values(path);
                }
            }
            else
            {
                const auto status = protocol::encodeValue(value, column);
                if (!status.ok())
                {
                    return status;
                }
                size_t count = 0;
                const auto countStatus = protocol::getValueCount(*column, &count);
                if (!countStatus.ok())
                {
                    return countStatus;
                }
                if (count != paths.size())
                {
                    return { ::grpc::StatusCode::INTERNAL,
                             "The local implementation returned a row count that does not match the requested paths." };
                }
            }
        }
        response->set_read_watermark_ns(simulationTimeNs);
        response->set_available_watermark_ns(simulationTimeNs);
        return ::grpc::Status::OK;
    }

    ::grpc::Status WriteWorldState(::grpc::ServerContext*,
                                   const worldstate::WriteWorldStateRequest* request,
                                   worldstate::WriteWorldStateResponse*) override
    {
        if (request->write().empty())
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "A direct world-state request must contain one operation." };
        }
        if (request->write_size() == 1 && request->write(0).has_update_simulation_time_ns())
        {
            const auto status = validateWatermark(*request, request->write(0).update_simulation_time_ns());
            return status.ok() ? m_simulation.step(request->simulation_id(), request->simulation_time_ns()) : status;
        }
        if (request->write_size() != 1 || !request->write(0).has_update())
        {
            return { ::grpc::StatusCode::UNIMPLEMENTED,
                     "Only one attribute update or one step watermark operation is supported." };
        }
        const auto& update = request->write(0).update();
        if (update.write_mode() != worldstate::WRITE_MODE_UNSPECIFIED && update.write_mode() != worldstate::UPSERT)
        {
            return { ::grpc::StatusCode::UNIMPLEMENTED, "Only UPSERT attribute writes are supported." };
        }
        if (update.keys().attribute().attribute_name() != "usd-path" || update.values_size() != 1 ||
            update.values(0).attribute().attribute_name().empty())
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT,
                     "The update must contain usd-path keys and exactly one named attribute value column." };
        }
        if (update.keys().values().packed_case() != worldstate::ValueColumn::kPackedString)
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "Attribute update keys must be strings." };
        }
        std::vector<std::string> paths = packedStrings(update.keys().values());
        if (hasDuplicates(paths))
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "Attribute updates require unique string paths." };
        }

        ::ovsim::interfaces::data::InputValueType decoded = std::vector<std::string>{};
        const auto& valueColumn = update.values(0).values();
        const auto decodeStatus = protocol::decodeValue(valueColumn, &decoded);
        if (!decodeStatus.ok())
        {
            return decodeStatus;
        }
        size_t valueCount = 0;
        const auto countStatus = protocol::getValueCount(valueColumn, &valueCount);
        if (!countStatus.ok())
        {
            return countStatus;
        }
        if (valueCount != paths.size())
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "Attribute updates require exactly one value row per path." };
        }
        if (paths.empty())
        {
            return m_simulation.getReadyStatus(request->simulation_id());
        }
        return m_simulation.write(request->simulation_id(), std::move(paths),
                                  update.values(0).attribute().attribute_name(), std::move(decoded),
                                  request->simulation_time_ns());
    }

    ::grpc::Status ReadSimulationTime(::grpc::ServerContext*,
                                      const worldstate::ReadSimulationTimeRequest*,
                                      worldstate::ReadSimulationTimeResponse*) override
    {
        return { ::grpc::StatusCode::UNIMPLEMENTED, "ReadSimulationTime is not implemented by the minimal server." };
    }

private:
    HostedSimulation& m_simulation;
};

} // namespace

class Server::Implementation
{
public:
    explicit Implementation(std::string listenAddress)
        : m_controlService(m_simulation), m_worldStateService(m_simulation), m_requestedAddress(std::move(listenAddress))
    {
        if (m_requestedAddress.empty())
        {
            throw std::invalid_argument("The gRPC listen address must be non-empty.");
        }
        if (!isaacsim::physics_engines::ovphysx::activate())
        {
            throw std::runtime_error("The OvPhysX backend could not be activated.");
        }
        m_backendActive = true;

        ::grpc::ServerBuilder builder;
        int selectedPort = 0;
        builder.AddListeningPort(m_requestedAddress, ::grpc::InsecureServerCredentials(), &selectedPort);
        builder.RegisterService(&m_controlService);
        builder.RegisterService(&m_worldStateService);
        m_server = builder.BuildAndStart();
        if (!m_server || selectedPort == 0)
        {
            isaacsim::physics_engines::ovphysx::shutdown();
            m_backendActive = false;
            throw std::runtime_error("The OV SIM gRPC server could not bind to '" + m_requestedAddress + "'.");
        }

        m_endpoint = m_requestedAddress;
        if (m_endpoint.size() >= 2 && m_endpoint.compare(m_endpoint.size() - 2, 2, ":0") == 0)
        {
            m_endpoint.replace(m_endpoint.size() - 1, 1, std::to_string(selectedPort));
        }
    }

    ~Implementation()
    {
        try
        {
            shutdown();
        }
        catch (...)
        {
        }
    }

    const std::string& getEndpoint() const
    {
        return m_endpoint;
    }

    void wait()
    {
        m_server->Wait();
    }

    void shutdown()
    {
        std::lock_guard<std::mutex> lock(m_shutdownMutex);
        if (m_shutdown)
        {
            if (!m_shutdownError.empty())
            {
                throw std::runtime_error(m_shutdownError);
            }
            return;
        }
        m_shutdown = true;
        if (m_server)
        {
            m_server->Shutdown();
        }
        const auto cleanupStatus = m_simulation.shutdown();
        if (!cleanupStatus.ok())
        {
            m_shutdownError = "Hosted simulation cleanup failed: " + cleanupStatus.error_message();
        }
        if (m_backendActive)
        {
            try
            {
                isaacsim::physics_engines::ovphysx::shutdown();
                m_backendActive = false;
            }
            catch (const std::exception& error)
            {
                if (!m_shutdownError.empty())
                {
                    m_shutdownError += "; ";
                }
                m_shutdownError += std::string("OvPhysX backend shutdown failed: ") + error.what();
            }
        }
        if (!m_shutdownError.empty())
        {
            throw std::runtime_error(m_shutdownError);
        }
    }

private:
    HostedSimulation m_simulation;
    ControlPlaneService m_controlService;
    WorldStateService m_worldStateService;
    std::string m_requestedAddress;
    std::string m_endpoint;
    std::unique_ptr<::grpc::Server> m_server;
    std::mutex m_shutdownMutex;
    bool m_shutdown{ false };
    bool m_backendActive{ false };
    std::string m_shutdownError;
};

Server::Server(std::string listenAddress) : m_implementation(std::make_unique<Implementation>(std::move(listenAddress)))
{
}

Server::~Server() = default;

const std::string& Server::getEndpoint() const
{
    return m_implementation->getEndpoint();
}

void Server::wait()
{
    m_implementation->wait();
}

void Server::shutdown()
{
    m_implementation->shutdown();
}

} // namespace grpc
} // namespace servers
} // namespace ovsim
} // namespace isaacsim
