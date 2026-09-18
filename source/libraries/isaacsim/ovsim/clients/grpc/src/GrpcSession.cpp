// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "isaacsim/ovsim/clients/grpc/GrpcSession.hpp"

#include "StatusCodec.hpp"
#include "ValueCodec.hpp"

#include <grpcpp/grpcpp.h>
#include <isaacsim/common/array/Array.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <random>
#include <simulation.grpc.pb.h>
#include <sstream>
#include <stdexcept>
#include <thread>
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
namespace clients
{
namespace grpc
{

namespace controlplane = nvidia::omniverse::simulation::v1;
namespace protocol = isaacsim::ovsim::protos::grpc;
namespace worldstate = nvidia::omniverse::simulation::worldstate::v1;
namespace array = isaacsim::common::array;

namespace
{

constexpr auto g_kRpcTimeout = std::chrono::seconds(5);
constexpr auto g_kInitializeTimeout = std::chrono::minutes(5);
constexpr auto g_kPollInterval = std::chrono::milliseconds(100);
constexpr std::array<int64_t, 3> g_kStepNanoseconds{ 16'666'667, 16'666'667, 16'666'666 };

void setDeadline(::grpc::ClientContext& context)
{
    context.set_deadline(std::chrono::system_clock::now() + g_kRpcTimeout);
}

std::string generateUuidV4()
{
    std::array<uint8_t, 16> bytes{};
    std::random_device random;
    for (auto& byte : bytes)
    {
        byte = static_cast<uint8_t>(random());
    }
    bytes[6] = static_cast<uint8_t>((bytes[6] & 0x0fU) | 0x40U);
    bytes[8] = static_cast<uint8_t>((bytes[8] & 0x3fU) | 0x80U);

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (size_t index = 0; index < bytes.size(); ++index)
    {
        if (index == 4 || index == 6 || index == 8 || index == 10)
        {
            output << '-';
        }
        output << std::setw(2) << static_cast<unsigned int>(bytes[index]);
    }
    return output.str();
}

// These transport failures do not reveal whether a mutating RPC reached the server. Preserve operation state so a
// caller can retry safely; automatic retry and reconciliation are deferred.
bool isUnknownOutcome(const ::grpc::Status& status)
{
    return status.error_code() == ::grpc::StatusCode::DEADLINE_EXCEEDED ||
           status.error_code() == ::grpc::StatusCode::UNAVAILABLE ||
           status.error_code() == ::grpc::StatusCode::UNKNOWN || status.error_code() == ::grpc::StatusCode::CANCELLED;
}

void throwStatus(const std::string& operation, const ::grpc::Status& status)
{
    throw std::runtime_error(operation + " failed with gRPC status " + std::to_string(status.error_code()) + ": " +
                             status.error_message());
}

std::vector<std::string> normalizePaths(const PathType& paths)
{
    std::vector<std::string> normalized = std::holds_alternative<std::string>(paths) ?
                                              std::vector<std::string>{ std::get<std::string>(paths) } :
                                              std::get<std::vector<std::string>>(paths);
    std::unordered_set<std::string> unique;
    for (const auto& path : normalized)
    {
        if (path.empty())
        {
            throw std::invalid_argument("OV SIM paths must be non-empty.");
        }
        if (!unique.emplace(path).second)
        {
            throw std::invalid_argument("OV SIM path batches must not contain duplicates.");
        }
    }
    return normalized;
}

OutputValueType createEmptyReadResult(const std::string& attributeName)
{
    if (attributeName == "usd-path")
    {
        return std::vector<std::string>{};
    }
    return array::Array(std::vector<int32_t>{}).reshape(array::Shape{ 0, 1 });
}

controlplane::CreateSimulationRequest makeCreateSimulationRequest(const std::string& simulationId,
                                                                  const std::string& usdContent)
{
    controlplane::CreateSimulationRequest request;
    request.set_simulation_id(simulationId);
    request.set_usd_content(usdContent);
    return request;
}

controlplane::GetSimulationRequest makeGetSimulationRequest(const std::string& simulationId)
{
    controlplane::GetSimulationRequest request;
    request.set_simulation_id(simulationId);
    return request;
}

controlplane::DeleteSimulationRequest makeDeleteSimulationRequest(const std::string& simulationId)
{
    controlplane::DeleteSimulationRequest request;
    request.set_simulation_id(simulationId);
    return request;
}

::grpc::Status validateSimulationResponse(const std::string& simulationId,
                                          const controlplane::Simulation& response,
                                          ::grpc::Status* asynchronousFailure)
{
    if (!asynchronousFailure)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The asynchronous failure destination is null." };
    }
    *asynchronousFailure = ::grpc::Status::OK;
    if (response.simulation_id() != simulationId)
    {
        return { ::grpc::StatusCode::INTERNAL, "The server returned an inconsistent simulation identifier." };
    }
    switch (response.status())
    {
    case controlplane::FAILED:
        if (!response.has_error())
        {
            return { ::grpc::StatusCode::INTERNAL, "The failed simulation response omitted its root-cause status." };
        }
        return protocol::decodeStatus(response.error(), asynchronousFailure);
    case controlplane::INITIALIZING:
    case controlplane::READY:
    case controlplane::DELETING:
        break;
    case controlplane::SIMULATION_STATUS_UNSPECIFIED:
    default:
        return { ::grpc::StatusCode::INTERNAL, "The server returned an invalid simulation status." };
    }
    if (response.has_error())
    {
        return { ::grpc::StatusCode::INTERNAL, "A non-failed simulation response contained an error status." };
    }
    return ::grpc::Status::OK;
}

::grpc::Status getSimulation(controlplane::ControlPlaneService::StubInterface& control,
                             const std::string& simulationId,
                             controlplane::Simulation* response,
                             ::grpc::Status* asynchronousFailure)
{
    if (!response)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The simulation response destination is null." };
    }
    const auto request = makeGetSimulationRequest(simulationId);
    ::grpc::ClientContext context;
    setDeadline(context);
    const auto status = control.GetSimulation(&context, request, response);
    if (!status.ok())
    {
        return status;
    }
    return validateSimulationResponse(simulationId, *response, asynchronousFailure);
}

worldstate::WriteWorldStateRequest makeStepRequest(const std::string& simulationId, int64_t simulationTimeNs)
{
    worldstate::WriteWorldStateRequest request;
    request.set_simulation_id(simulationId);
    request.set_simulation_time_ns(simulationTimeNs);
    auto* watermark = request.add_write()->mutable_update_simulation_time_ns();
    watermark->set_simulation_time_ns(simulationTimeNs);
    watermark->set_scope(worldstate::ALL);
    return request;
}

worldstate::ReadWorldStateRequest makeReadRequest(const std::string& simulationId,
                                                  const std::vector<std::string>& paths,
                                                  const std::string& attributeName)
{
    worldstate::ReadWorldStateRequest request;
    request.set_simulation_id(simulationId);
    request.set_max_wait_ms(0);
    auto* read = request.mutable_read();
    read->add_select("usd-path");
    if (attributeName != "usd-path")
    {
        read->add_select(attributeName);
    }
    (void)read->mutable_simulation_time_range();
    read->set_change_history(false);
    read->set_timestamped(false);
    auto* predicate = read->mutable_filter()->add_predicates();
    predicate->set_attribute("usd-path");
    predicate->set_op(worldstate::IN);
    predicate->set_negate(false);
    for (const auto& path : paths)
    {
        predicate->mutable_values()->mutable_packed_string()->add_values(path);
    }
    return request;
}

::grpc::Status parseReadResponse(const std::vector<std::string>& paths,
                                 const std::string& attributeName,
                                 const worldstate::ReadWorldStateResponse& response,
                                 OutputValueType* value)
{
    if (!value)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The destination read value is null." };
    }
    if (response.result_sets_size() != 1)
    {
        return { ::grpc::StatusCode::INTERNAL, "The server returned an unexpected number of result sets." };
    }
    const auto& result = response.result_sets(0);
    if (result.attributes_size() != result.columns_size())
    {
        return { ::grpc::StatusCode::INTERNAL, "The server returned mismatched attributes and columns." };
    }

    const worldstate::ValueColumn* pathColumn = nullptr;
    const worldstate::ValueColumn* valueColumn = nullptr;
    for (int index = 0; index < result.attributes_size(); ++index)
    {
        if (result.attributes(index) == "usd-path")
        {
            pathColumn = &result.columns(index);
        }
        if (result.attributes(index) == attributeName)
        {
            valueColumn = &result.columns(index);
        }
    }
    if (!pathColumn || !valueColumn || pathColumn->packed_case() != worldstate::ValueColumn::kPackedString)
    {
        return { ::grpc::StatusCode::INTERNAL, "The server omitted a requested result column." };
    }
    const std::vector<std::string> returnedPaths(
        pathColumn->packed_string().values().begin(), pathColumn->packed_string().values().end());
    if (returnedPaths != paths)
    {
        return { ::grpc::StatusCode::INTERNAL, "The server returned paths in an inconsistent row order." };
    }
    const auto decodeStatus = protocol::decodeValue(*valueColumn, value);
    if (!decodeStatus.ok())
    {
        return decodeStatus;
    }
    size_t count = 0;
    const auto countStatus = protocol::getValueCount(*valueColumn, &count);
    if (!countStatus.ok())
    {
        return countStatus;
    }
    if (count != paths.size())
    {
        return { ::grpc::StatusCode::INTERNAL, "The server returned an inconsistent result row count." };
    }
    return ::grpc::Status::OK;
}

::grpc::Status makeWriteRequest(const std::string& simulationId,
                                int64_t simulationTimeNs,
                                const std::vector<std::string>& paths,
                                const array::Array& positions,
                                worldstate::WriteWorldStateRequest* request)
{
    if (!request)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The destination write request is null." };
    }
    request->Clear();
    request->set_simulation_id(simulationId);
    request->set_simulation_time_ns(simulationTimeNs);
    auto* update = request->add_write()->mutable_update();
    update->set_write_mode(worldstate::UPSERT);
    update->mutable_keys()->mutable_attribute()->set_attribute_name("usd-path");
    for (const auto& path : paths)
    {
        update->mutable_keys()->mutable_values()->mutable_packed_string()->add_values(path);
    }
    auto* values = update->add_values();
    values->mutable_attribute()->set_attribute_name("position");
    return protocol::encodeValue(InputValueType(positions), values->mutable_values());
}

} // namespace

class GrpcSession::Implementation
{
public:
    explicit Implementation(const std::optional<std::unordered_map<std::string, std::string>>& configuration)
    {
        if (!configuration || configuration->empty())
        {
            throw std::invalid_argument("GrpcSession requires a non-empty configuration.");
        }
        m_configuration = *configuration;
        const auto endpoint = m_configuration.find("endpoint");
        if (endpoint == m_configuration.end() || endpoint->second.empty())
        {
            throw std::invalid_argument("GrpcSession requires a non-empty 'endpoint' configuration value.");
        }
        m_channel = ::grpc::CreateChannel(endpoint->second, ::grpc::InsecureChannelCredentials());
        m_control = controlplane::ControlPlaneService::NewStub(m_channel);
        m_worldState = worldstate::WorldStateService::NewStub(m_channel);
    }

    const std::unordered_map<std::string, std::string>& getConfiguration() const
    {
        return m_configuration;
    }

    ::grpc::Status createSimulation(const std::string& usdContent)
    {
        if (usdContent.empty())
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "USD content must be non-empty." };
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_simulationId.empty() && m_deleteAccepted)
        {
            controlplane::Simulation response;
            ::grpc::Status asynchronousFailure;
            const auto getStatus = getSimulation(*m_control, m_simulationId, &response, &asynchronousFailure);
            if (getStatus.error_code() == ::grpc::StatusCode::NOT_FOUND)
            {
                _clearLifecycle();
            }
            else if (!getStatus.ok())
            {
                return getStatus;
            }
            else if (!asynchronousFailure.ok())
            {
                return asynchronousFailure;
            }
            else
            {
                return { ::grpc::StatusCode::FAILED_PRECONDITION,
                         "The previous remote simulation has not finished deletion." };
            }
        }
        if (!m_simulationId.empty())
        {
            return { ::grpc::StatusCode::ALREADY_EXISTS, "This session already owns a remote simulation." };
        }

        m_simulationId = generateUuidV4();
        const auto request = makeCreateSimulationRequest(m_simulationId, usdContent);
        controlplane::CreateSimulationResponse response;
        ::grpc::ClientContext context;
        setDeadline(context);
        const auto status = m_control->CreateSimulation(&context, request, &response);
        if (!status.ok() && !isUnknownOutcome(status))
        {
            _clearLifecycle();
        }
        return status;
    }

    ::grpc::Status deleteSimulation()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_simulationId.empty())
        {
            return { ::grpc::StatusCode::FAILED_PRECONDITION, "This session does not own a remote simulation." };
        }
        const auto request = makeDeleteSimulationRequest(m_simulationId);
        controlplane::DeleteSimulationResponse response;
        ::grpc::ClientContext context;
        setDeadline(context);
        const auto status = m_control->DeleteSimulation(&context, request, &response);
        if (status.ok())
        {
            m_deleteAccepted = true;
            return status;
        }
        if (status.error_code() == ::grpc::StatusCode::NOT_FOUND)
        {
            _clearLifecycle();
            return ::grpc::Status::OK;
        }
        return status;
    }

    ::grpc::Status initialize()
    {
        const auto deadline = std::chrono::steady_clock::now() + g_kInitializeTimeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            std::string simulationId;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (m_simulationId.empty())
                {
                    return { ::grpc::StatusCode::FAILED_PRECONDITION, "No remote simulation has been created." };
                }
                if (m_deleteAccepted)
                {
                    return { ::grpc::StatusCode::FAILED_PRECONDITION, "The remote simulation is being deleted." };
                }
                simulationId = m_simulationId;
            }

            controlplane::Simulation response;
            ::grpc::Status asynchronousFailure;
            const auto status = getSimulation(*m_control, simulationId, &response, &asynchronousFailure);
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (m_simulationId != simulationId || m_deleteAccepted)
                {
                    return { ::grpc::StatusCode::FAILED_PRECONDITION,
                             "The remote simulation changed while initialization was pending." };
                }
            }
            if (!status.ok())
            {
                if (isUnknownOutcome(status))
                {
                    std::this_thread::sleep_for(g_kPollInterval);
                    continue;
                }
                return status;
            }
            if (response.status() == controlplane::READY)
            {
                return ::grpc::Status::OK;
            }
            if (response.status() == controlplane::FAILED)
            {
                return asynchronousFailure;
            }
            if (response.status() == controlplane::DELETING)
            {
                return { ::grpc::StatusCode::FAILED_PRECONDITION, "The remote simulation is being deleted." };
            }
            std::this_thread::sleep_for(g_kPollInterval);
        }
        return { ::grpc::StatusCode::DEADLINE_EXCEEDED,
                 "Remote simulation initialization exceeded the five-minute timeout." };
    }

    ::grpc::Status step()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_simulationId.empty() || m_deleteAccepted)
        {
            return { ::grpc::StatusCode::FAILED_PRECONDITION, "No active remote simulation is available." };
        }
        if (!m_pendingStepTimeNs.has_value())
        {
            m_pendingStepTimeNs = _getCurrentTime();
        }

        const auto request = makeStepRequest(m_simulationId, *m_pendingStepTimeNs);
        worldstate::WriteWorldStateResponse response;
        ::grpc::ClientContext context;
        setDeadline(context);
        const auto status = m_worldState->WriteWorldState(&context, request, &response);
        if (status.ok())
        {
            _incrementTime();
            m_pendingStepTimeNs.reset();
        }
        else if (!isUnknownOutcome(status))
        {
            m_pendingStepTimeNs.reset();
        }
        return status;
    }

    ::grpc::Status read(const std::vector<std::string>& paths, const std::string& attributeName, OutputValueType* value)
    {
        if (!value)
        {
            return { ::grpc::StatusCode::INVALID_ARGUMENT, "The destination read value is null." };
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_simulationId.empty() || m_deleteAccepted)
        {
            return { ::grpc::StatusCode::FAILED_PRECONDITION, "No active remote simulation is available." };
        }

        const auto request = makeReadRequest(m_simulationId, paths, attributeName);

        worldstate::ReadWorldStateResponse response;
        ::grpc::ClientContext context;
        setDeadline(context);
        const auto status = m_worldState->ReadWorldState(&context, request, &response);
        if (!status.ok())
        {
            return status;
        }
        return parseReadResponse(paths, attributeName, response, value);
    }

    ::grpc::Status write(const std::vector<std::string>& paths, const array::Array& positions)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_simulationId.empty() || m_deleteAccepted)
        {
            return { ::grpc::StatusCode::FAILED_PRECONDITION, "No active remote simulation is available." };
        }
        if (m_pendingStepTimeNs.has_value())
        {
            return { ::grpc::StatusCode::FAILED_PRECONDITION,
                     "A previous step has an unknown outcome; resolve it before writing." };
        }

        worldstate::WriteWorldStateRequest request;
        const auto encodeStatus = makeWriteRequest(m_simulationId, _getCurrentTime(), paths, positions, &request);
        if (!encodeStatus.ok())
        {
            return encodeStatus;
        }

        worldstate::WriteWorldStateResponse response;
        ::grpc::ClientContext context;
        setDeadline(context);
        return m_worldState->WriteWorldState(&context, request, &response);
    }

private:
    int64_t _getCurrentTime() const
    {
        return m_simulationTimeNs + g_kStepNanoseconds.at(m_timePhase % g_kStepNanoseconds.size());
    }

    void _incrementTime()
    {
        m_simulationTimeNs = _getCurrentTime();
        m_timePhase = (m_timePhase + 1) % g_kStepNanoseconds.size();
    }

    void _clearLifecycle()
    {
        m_simulationId.clear();
        m_deleteAccepted = false;
        m_simulationTimeNs = 0;
        m_timePhase = 0;
        m_pendingStepTimeNs.reset();
    }

    std::unordered_map<std::string, std::string> m_configuration;
    std::shared_ptr<::grpc::Channel> m_channel;
    std::unique_ptr<controlplane::ControlPlaneService::Stub> m_control;
    std::unique_ptr<worldstate::WorldStateService::Stub> m_worldState;
    std::mutex m_mutex;
    std::string m_simulationId;
    bool m_deleteAccepted{ false };
    int64_t m_simulationTimeNs{ 0 };
    size_t m_timePhase{ 0 };
    std::optional<int64_t> m_pendingStepTimeNs;
};

GrpcSession::GrpcSession(const std::optional<std::unordered_map<std::string, std::string>>& configuration)
    : m_implementation(std::make_unique<Implementation>(configuration))
{
}

GrpcSession::~GrpcSession() = default;
GrpcSession::GrpcSession(GrpcSession&&) noexcept = default;
GrpcSession& GrpcSession::operator=(GrpcSession&&) noexcept = default;

const std::unordered_map<std::string, std::string>& GrpcSession::getConfiguration() const
{
    return m_implementation->getConfiguration();
}

bool GrpcSession::createStage()
{
    throw std::logic_error("Remote empty-stage creation is not supported; import authored USD content instead.");
}

bool GrpcSession::openStage(const std::string&)
{
    throw std::logic_error("Remote stage file loading is not supported.");
}

bool GrpcSession::saveStage(const std::string&)
{
    throw std::logic_error("Remote stage saving is not supported.");
}

bool GrpcSession::importStageFromString(const std::string& usdString)
{
    return m_implementation->createSimulation(usdString).ok();
}

std::string GrpcSession::exportStageToString()
{
    throw std::logic_error("Remote stage export is not supported.");
}

bool GrpcSession::closeStage()
{
    return m_implementation->deleteSimulation().ok();
}

bool GrpcSession::addReferenceToStage(const std::string&, const std::string&, const std::string&)
{
    throw std::logic_error("Remote stage authoring is not supported.");
}

bool GrpcSession::definePrim(const std::string&, const std::string&)
{
    throw std::logic_error("Remote stage authoring is not supported.");
}

bool GrpcSession::movePrim(const std::string&, const std::string&)
{
    throw std::logic_error("Remote stage authoring is not supported.");
}

bool GrpcSession::removePrim(const std::string&)
{
    throw std::logic_error("Remote stage authoring is not supported.");
}

bool GrpcSession::createPrimAttribute(const std::string&, const std::string&, const std::string&)
{
    throw std::logic_error("Remote stage authoring is not supported.");
}

bool GrpcSession::removePrimAttribute(const std::string&, const std::string&)
{
    throw std::logic_error("Remote stage authoring is not supported.");
}

void GrpcSession::setAuthoringParameter(const std::string&, const std::string&, const InputParameterType&)
{
    throw std::logic_error("Remote authoring parameters are not supported.");
}

OutputParameterType GrpcSession::getAuthoringParameter(const std::string&, const std::string&)
{
    throw std::logic_error("Remote authoring parameters are not supported.");
}

void GrpcSession::play()
{
    throw std::logic_error("Remote play is not supported.");
}

void GrpcSession::pause()
{
    throw std::logic_error("Remote pause is not supported.");
}

void GrpcSession::stop()
{
    throw std::logic_error("Remote stop is not supported.");
}

void GrpcSession::initialize()
{
    const auto status = m_implementation->initialize();
    if (!status.ok())
    {
        throwStatus("Remote simulation initialization", status);
    }
}

void GrpcSession::invalidate()
{
    throw std::logic_error("Remote invalidation is not supported; call closeStage to delete the simulation.");
}

void GrpcSession::step()
{
    const auto status = m_implementation->step();
    if (!status.ok())
    {
        throwStatus("Remote simulation step", status);
    }
}

void GrpcSession::setSimulationParameter(const std::string&, const std::string&, const InputParameterType&)
{
    throw std::logic_error("Remote simulation parameters are not supported.");
}

OutputParameterType GrpcSession::getSimulationParameter(const std::string&, const std::string&)
{
    throw std::logic_error("Remote simulation parameters are not supported.");
}

OutputValueType GrpcSession::read(const PathType& paths, const std::string& attributeName, std::optional<double> timestamp)
{
    if (timestamp.has_value())
    {
        throw std::invalid_argument("Explicit and historical remote read timestamps are not supported.");
    }
    if (attributeName.empty())
    {
        throw std::invalid_argument("The read attribute name must be non-empty.");
    }
    const auto normalized = normalizePaths(paths);
    if (normalized.empty())
    {
        return createEmptyReadResult(attributeName);
    }
    OutputValueType value = std::vector<std::string>{};
    const auto status = m_implementation->read(normalized, attributeName, &value);
    if (!status.ok())
    {
        throwStatus("Remote world-state read", status);
    }
    return value;
}

void GrpcSession::write(const PathType& paths,
                        const std::string& attributeName,
                        const InputValueType& values,
                        std::optional<double> timestamp)
{
    if (timestamp.has_value())
    {
        throw std::invalid_argument("Explicit and historical remote write timestamps are not supported.");
    }
    if (attributeName != "position")
    {
        throw std::invalid_argument("The minimal remote client only supports position writes.");
    }
    const auto normalized = normalizePaths(paths);
    if (normalized.empty())
    {
        return;
    }
    if (!std::holds_alternative<array::Array>(values))
    {
        throw std::invalid_argument("Remote position values must be an Array.");
    }
    const auto& source = std::get<array::Array>(values);
    array::Array positions = source.ndim() == 1 && normalized.size() == 1 ? source.reshape(array::Shape{ 1, 3 }) : source;
    if (positions.ndim() != 2 || positions.shape()[0] != static_cast<int64_t>(normalized.size()) ||
        positions.shape()[1] != 3 ||
        (positions.dtype() != array::Dtype::Float32() && positions.dtype() != array::Dtype::Float64()))
    {
        throw std::invalid_argument("Remote position values must have float32 or float64 shape (number-of-paths, 3).");
    }
    const auto status = m_implementation->write(normalized, positions);
    if (!status.ok())
    {
        throwStatus("Remote world-state write", status);
    }
}

} // namespace grpc
} // namespace clients
} // namespace ovsim
} // namespace isaacsim
