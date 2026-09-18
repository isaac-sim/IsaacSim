// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "HostedSimulation.hpp"

#include <isaacsim/ovsim/clients/local/control/authoring/Authoring.hpp>
#include <isaacsim/ovsim/clients/local/control/simulation/Simulation.hpp>
#include <isaacsim/ovsim/clients/local/data/Data.hpp>

#include <algorithm>
#include <array>
#include <exception>
#include <utility>

namespace isaacsim
{
namespace ovsim
{
namespace servers
{
namespace grpc
{

namespace local = isaacsim::ovsim::clients::local;

namespace
{

::grpc::Status notFound()
{
    return { ::grpc::StatusCode::NOT_FOUND, "The requested simulation does not exist." };
}

::grpc::Status internalError(const std::string& operation, const std::exception& error)
{
    return { ::grpc::StatusCode::INTERNAL, operation + " failed: " + error.what() };
}

::grpc::Status invalidArgument(const std::string& operation, const std::invalid_argument& error)
{
    return { ::grpc::StatusCode::INVALID_ARGUMENT, operation + " failed: " + error.what() };
}

::grpc::Status appendSecondaryFailure(const ::grpc::Status& primary,
                                      const std::string& operation,
                                      const ::grpc::Status& secondary)
{
    if (secondary.ok())
    {
        return primary;
    }
    return { primary.error_code(),
             primary.error_message() + "; " + operation + " failed with gRPC status " +
                 std::to_string(secondary.error_code()) + ": " + secondary.error_message(),
             primary.error_details() };
}

class LocalNativeSimulation final : public NativeSimulation
{
public:
    bool importStageFromString(const std::string& usdContent) override
    {
        return local::control::authoring::importStageFromString(usdContent);
    }

    void initialize() override
    {
        local::control::simulation::setParameter("physics", "physics-engine", std::string("ovphysx"));
        local::control::simulation::initialize();
    }

    void invalidate() override
    {
        local::control::simulation::invalidate();
    }

    bool closeStage() override
    {
        return local::control::authoring::closeStage();
    }

    ::ovsim::interfaces::data::OutputValueType read(const std::vector<std::string>& paths,
                                                    const std::string& attribute) override
    {
        return local::data::read(paths, attribute, std::nullopt);
    }

    void write(const std::vector<std::string>& paths,
               const std::string& attribute,
               const ::ovsim::interfaces::data::InputValueType& value) override
    {
        local::data::write(paths, attribute, value, std::nullopt);
    }

    void step() override
    {
        local::control::simulation::step();
    }
};

std::unique_ptr<NativeSimulation> requireNativeSimulation(std::unique_ptr<NativeSimulation> nativeSimulation)
{
    if (!nativeSimulation)
    {
        throw std::invalid_argument("HostedSimulation requires a native simulation adapter.");
    }
    return nativeSimulation;
}

} // namespace

HostedSimulation::HostedSimulation() : HostedSimulation(std::make_unique<LocalNativeSimulation>())
{
}

HostedSimulation::HostedSimulation(std::unique_ptr<NativeSimulation> nativeSimulation)
    : m_nativeSimulation(requireNativeSimulation(std::move(nativeSimulation))),
      m_worker(&HostedSimulation::_runWorker, this)
{
}

HostedSimulation::~HostedSimulation()
{
    (void)shutdown();
}

::grpc::Status HostedSimulation::createSimulation(std::string simulationId, std::string usdContent)
{
    if (simulationId.empty() || usdContent.empty())
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "simulation_id and usd_content must be non-empty." };
    }
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        if (m_hasSimulation)
        {
            return m_simulationId == simulationId ?
                       ::grpc::Status(::grpc::StatusCode::ALREADY_EXISTS, "The simulation identifier already exists.") :
                       ::grpc::Status(
                           ::grpc::StatusCode::RESOURCE_EXHAUSTED, "This server already hosts a different simulation.");
        }
        m_hasSimulation = true;
        m_simulationId = simulationId;
        m_state = State::eInitializing;
        m_error = ::grpc::Status::OK;
        m_deleteRequested = false;
        m_stageImported = false;
        m_simulationInitialized = false;
        m_simulationTimeNs = 0;
        m_latestVisibleTimeNs = 0;
        m_timePhase = 0;
        m_latestWriteTimes.clear();
        _enqueue([this, simulationId = std::move(simulationId), usdContent = std::move(usdContent)]() mutable
                 { _initializeOnWorker(std::move(simulationId), std::move(usdContent)); });
    }
    return ::grpc::Status::OK;
}

::grpc::Status HostedSimulation::getSnapshot(const std::string& simulationId, Snapshot* snapshot) const
{
    if (!snapshot)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "The destination simulation snapshot is null." };
    }
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (!m_hasSimulation || simulationId != m_simulationId)
    {
        return notFound();
    }
    snapshot->simulationId = m_simulationId;
    snapshot->state = m_state;
    snapshot->error = m_error;
    snapshot->simulationTimeNs = m_latestVisibleTimeNs;
    return ::grpc::Status::OK;
}

::grpc::Status HostedSimulation::getReadyStatus(const std::string& simulationId) const
{
    return _getReadyStatus(simulationId);
}

::grpc::Status HostedSimulation::deleteSimulation(std::string simulationId)
{
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        if (!m_hasSimulation || simulationId != m_simulationId)
        {
            return notFound();
        }
        if (m_deleteRequested)
        {
            return ::grpc::Status::OK;
        }
        m_deleteRequested = true;
        m_state = State::eDeleting;
    }
    _enqueue(
        [this, simulationId = std::move(simulationId)]()
        {
            const auto status = _cleanupOnWorker(simulationId, true);
            if (!status.ok())
            {
                std::lock_guard<std::mutex> lock(m_stateMutex);
                if (m_hasSimulation && m_simulationId == simulationId)
                {
                    m_state = State::eFailed;
                    m_error = status;
                    m_deleteRequested = false;
                }
            }
        });
    return ::grpc::Status::OK;
}

::grpc::Status HostedSimulation::read(std::string simulationId,
                                      std::vector<std::string> paths,
                                      std::string attribute,
                                      ::ovsim::interfaces::data::OutputValueType* value,
                                      int64_t* simulationTimeNs)
{
    if (!value || !simulationTimeNs)
    {
        return { ::grpc::StatusCode::INVALID_ARGUMENT, "A destination for the read value or timestamp is null." };
    }
    return _invoke(
        [this, simulationId = std::move(simulationId), paths = std::move(paths), attribute = std::move(attribute),
         value, simulationTimeNs]()
        {
            const auto status = _getReadyStatus(simulationId);
            if (!status.ok())
            {
                return status;
            }
            try
            {
                *value = m_nativeSimulation->read(paths, attribute);
                *simulationTimeNs = m_latestVisibleTimeNs;
                return ::grpc::Status::OK;
            }
            catch (const std::invalid_argument& error)
            {
                return ::grpc::Status(::grpc::StatusCode::INVALID_ARGUMENT, error.what());
            }
            catch (const std::exception& error)
            {
                return internalError("World-state read", error);
            }
        });
}

::grpc::Status HostedSimulation::write(std::string simulationId,
                                       std::vector<std::string> paths,
                                       std::string attribute,
                                       ::ovsim::interfaces::data::InputValueType value,
                                       int64_t simulationTimeNs)
{
    return _invoke(
        [this, simulationId = std::move(simulationId), paths = std::move(paths), attribute = std::move(attribute),
         value = std::move(value), simulationTimeNs]()
        {
            auto status = _getReadyStatus(simulationId);
            if (!status.ok())
            {
                return status;
            }

            const int64_t expected = _getCurrentTime();
            if (simulationTimeNs <= m_simulationTimeNs)
            {
                return ::grpc::Status(
                    ::grpc::StatusCode::ABORTED, "World-state writes must use a timestamp newer than the watermark.");
            }
            if (simulationTimeNs != expected)
            {
                return ::grpc::Status(::grpc::StatusCode::INVALID_ARGUMENT,
                                      "The minimal server only supports the next drift-free 1/60-second timestamp.");
            }
            for (const auto& path : paths)
            {
                const auto found = m_latestWriteTimes.find(path + "\n" + attribute);
                if (found != m_latestWriteTimes.end() && found->second >= simulationTimeNs)
                {
                    return ::grpc::Status(
                        ::grpc::StatusCode::ABORTED, "The attribute was already written at this timestamp.");
                }
            }
            try
            {
                m_nativeSimulation->write(paths, attribute, value);
            }
            catch (const std::invalid_argument& error)
            {
                return ::grpc::Status(::grpc::StatusCode::INVALID_ARGUMENT, error.what());
            }
            catch (const std::exception& error)
            {
                return internalError("World-state write", error);
            }
            for (const auto& path : paths)
            {
                m_latestWriteTimes[path + "\n" + attribute] = simulationTimeNs;
            }
            {
                std::lock_guard<std::mutex> lock(m_stateMutex);
                m_latestVisibleTimeNs = std::max(m_latestVisibleTimeNs, simulationTimeNs);
            }
            return ::grpc::Status::OK;
        });
}

::grpc::Status HostedSimulation::step(std::string simulationId, int64_t simulationTimeNs)
{
    return _invoke(
        [this, simulationId = std::move(simulationId), simulationTimeNs]()
        {
            auto status = _getReadyStatus(simulationId);
            if (!status.ok())
            {
                return status;
            }
            if (simulationTimeNs == m_simulationTimeNs)
            {
                return ::grpc::Status::OK;
            }
            const int64_t expected = _getCurrentTime();
            if (simulationTimeNs != expected)
            {
                return ::grpc::Status(::grpc::StatusCode::INVALID_ARGUMENT,
                                      "The minimal server only supports the next drift-free 1/60-second timestamp.");
            }
            try
            {
                m_nativeSimulation->step();
            }
            catch (const std::exception& error)
            {
                return internalError("Simulation step", error);
            }
            _incrementTime();
            {
                std::lock_guard<std::mutex> lock(m_stateMutex);
                m_latestVisibleTimeNs = std::max(m_latestVisibleTimeNs, simulationTimeNs);
            }
            return ::grpc::Status::OK;
        });
}

::grpc::Status HostedSimulation::shutdown()
{
    std::string simulationId;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        if (m_hasSimulation)
        {
            simulationId = m_simulationId;
            m_deleteRequested = true;
            m_state = State::eDeleting;
        }
    }

    std::future<::grpc::Status> cleanupFuture;
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        if (m_stopping)
        {
            m_queueCondition.notify_all();
        }
        else
        {
            if (!simulationId.empty())
            {
                auto cleanup = std::make_shared<std::packaged_task<::grpc::Status()>>(
                    [this, simulationId]()
                    {
                        const auto status = _cleanupOnWorker(simulationId, true);
                        if (!status.ok())
                        {
                            std::lock_guard<std::mutex> stateLock(m_stateMutex);
                            if (m_hasSimulation && m_simulationId == simulationId)
                            {
                                m_state = State::eFailed;
                                m_error = status;
                                m_deleteRequested = false;
                            }
                        }
                        return status;
                    });
                cleanupFuture = cleanup->get_future();
                m_tasks.emplace([cleanup]() { (*cleanup)(); });
            }
            m_stopping = true;
        }
    }
    m_queueCondition.notify_all();
    if (m_worker.joinable())
    {
        m_worker.join();
    }
    if (cleanupFuture.valid())
    {
        m_shutdownStatus = cleanupFuture.get();
    }
    return m_shutdownStatus;
}

void HostedSimulation::_enqueue(std::function<void()> task)
{
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        if (m_stopping)
        {
            return;
        }
        m_tasks.emplace(std::move(task));
    }
    m_queueCondition.notify_one();
}

void HostedSimulation::_runWorker()
{
    while (true)
    {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCondition.wait(lock, [this]() { return m_stopping || !m_tasks.empty(); });
            if (m_tasks.empty())
            {
                if (m_stopping)
                {
                    break;
                }
                continue;
            }
            task = std::move(m_tasks.front());
            m_tasks.pop();
        }
        task();
    }
}

void HostedSimulation::_initializeOnWorker(std::string simulationId, std::string usdContent)
{
    ::grpc::Status initializationStatus;
    try
    {
        if (!m_nativeSimulation->importStageFromString(usdContent))
        {
            initializationStatus = { ::grpc::StatusCode::INTERNAL, "USD import failed: the stage could not be imported." };
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_stateMutex);
            if (!m_hasSimulation || m_simulationId != simulationId)
            {
                return;
            }
            m_stageImported = true;
            if (m_deleteRequested)
            {
                return;
            }
        }

        if (initializationStatus.ok())
        {
            m_nativeSimulation->initialize();
            {
                std::lock_guard<std::mutex> lock(m_stateMutex);
                if (!m_hasSimulation || m_simulationId != simulationId)
                {
                    return;
                }
                m_simulationInitialized = true;
                if (!m_deleteRequested)
                {
                    m_state = State::eReady;
                    m_error = ::grpc::Status::OK;
                }
            }
        }
    }
    catch (const std::invalid_argument& error)
    {
        initializationStatus = invalidArgument("Simulation initialization", error);
    }
    catch (const std::exception& error)
    {
        initializationStatus = internalError("Simulation initialization", error);
    }

    if (!initializationStatus.ok())
    {
        initializationStatus = appendSecondaryFailure(
            initializationStatus, "Initialization rollback", _cleanupOnWorker(simulationId, false));
        std::lock_guard<std::mutex> lock(m_stateMutex);
        if (m_hasSimulation && m_simulationId == simulationId)
        {
            if (!m_deleteRequested)
            {
                m_state = State::eFailed;
            }
            m_error = initializationStatus;
        }
    }
}

::grpc::Status HostedSimulation::_cleanupOnWorker(const std::string& simulationId, bool clearOnSuccess)
{
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        if (!m_hasSimulation || m_simulationId != simulationId)
        {
            return ::grpc::Status::OK;
        }
    }
    if (m_simulationInitialized)
    {
        try
        {
            m_nativeSimulation->invalidate();
            m_simulationInitialized = false;
        }
        catch (const std::exception& error)
        {
            return internalError("Simulation invalidation", error);
        }
    }
    if (m_stageImported)
    {
        try
        {
            if (!m_nativeSimulation->closeStage())
            {
                return { ::grpc::StatusCode::INTERNAL, "Stage close reported failure." };
            }
            m_stageImported = false;
        }
        catch (const std::exception& error)
        {
            return internalError("Stage close", error);
        }
    }
    if (clearOnSuccess)
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        if (m_hasSimulation && m_simulationId == simulationId)
        {
            m_hasSimulation = false;
            m_simulationId.clear();
            m_error = ::grpc::Status::OK;
            m_deleteRequested = false;
            m_latestWriteTimes.clear();
        }
    }
    return ::grpc::Status::OK;
}

::grpc::Status HostedSimulation::_getReadyStatus(const std::string& simulationId) const
{
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (!m_hasSimulation || m_simulationId != simulationId)
    {
        return notFound();
    }
    if (m_state != State::eReady)
    {
        switch (m_state)
        {
        case State::eInitializing:
            return { ::grpc::StatusCode::FAILED_PRECONDITION, "Simulation '" + simulationId + "' is still initializing." };
        case State::eFailed:
            return m_error.ok() ? ::grpc::Status(::grpc::StatusCode::INTERNAL,
                                                 "Simulation '" + simulationId + "' has no recorded failure status.") :
                                  m_error;
        case State::eDeleting:
            return { ::grpc::StatusCode::FAILED_PRECONDITION,
                     "Simulation '" + simulationId + "' cleanup is in progress." };
        case State::eReady:
            break;
        }
    }
    return ::grpc::Status::OK;
}

int64_t HostedSimulation::_getCurrentTime() const
{
    static constexpr std::array<int64_t, 3> s_kStepNanoseconds{ 16'666'667, 16'666'667, 16'666'666 };
    return m_simulationTimeNs + s_kStepNanoseconds.at(m_timePhase % s_kStepNanoseconds.size());
}

void HostedSimulation::_incrementTime()
{
    m_simulationTimeNs = _getCurrentTime();
    m_timePhase = (m_timePhase + 1) % 3;
    for (auto iterator = m_latestWriteTimes.begin(); iterator != m_latestWriteTimes.end();)
    {
        if (iterator->second <= m_simulationTimeNs)
        {
            iterator = m_latestWriteTimes.erase(iterator);
        }
        else
        {
            ++iterator;
        }
    }
}

} // namespace grpc
} // namespace servers
} // namespace ovsim
} // namespace isaacsim
