// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <grpcpp/support/status.h>
#include <isaacsim/ovsim/servers/grpc/Export.h>
#include <ovsim/interfaces/data/Data.hpp>

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace ovsim
{
namespace servers
{
namespace grpc
{

/**
 * @class NativeSimulation
 * @brief Abstracts the native simulation operations used by the gRPC host.
 */
class NativeSimulation
{
public:
    /** @brief Destroy the native simulation adapter. */
    virtual ~NativeSimulation() = default;

    /**
     * @brief Import a USD stage from an in-memory string.
     *
     * @param[in] usdContent USD layer content to import.
     * @return True if the stage was imported successfully; otherwise, false.
     */
    virtual bool importStageFromString(const std::string& usdContent) = 0;

    /** @brief Initialize the simulation. */
    virtual void initialize() = 0;

    /** @brief Invalidate the simulation. */
    virtual void invalidate() = 0;

    /**
     * @brief Close the imported stage.
     *
     * @return True if the stage was closed successfully; otherwise, false.
     */
    virtual bool closeStage() = 0;

    /**
     * @brief Read an attribute from one or more prim paths.
     *
     * @param[in] paths Prim paths to read.
     * @param[in] attribute Attribute name to read.
     * @return The attribute values in path order.
     */
    virtual ::ovsim::interfaces::data::OutputValueType read(const std::vector<std::string>& paths,
                                                            const std::string& attribute) = 0;

    /**
     * @brief Write an attribute on one or more prim paths.
     *
     * @param[in] paths Prim paths to update.
     * @param[in] attribute Attribute name to write.
     * @param[in] value Value to write.
     */
    virtual void write(const std::vector<std::string>& paths,
                       const std::string& attribute,
                       const ::ovsim::interfaces::data::InputValueType& value) = 0;

    /** @brief Advance the simulation by one step. */
    virtual void step() = 0;
};

/**
 * @class HostedSimulation
 * @brief Owns the single native simulation exposed by the gRPC server.
 *
 * @details Lifecycle requests are asynchronous. Data and step requests block until their work completes on the
 * simulation worker thread.
 */
class ISAACSIM_OVSIM_SERVERS_GRPC_API HostedSimulation
{
public:
    /** @brief Lifecycle state of the hosted simulation. */
    enum class State
    {
        eInitializing, ///< The stage and simulation are being initialized.
        eReady, ///< The simulation accepts data and step requests.
        eFailed, ///< Initialization or cleanup failed.
        eDeleting, ///< Simulation cleanup is in progress.
    };

    /** @brief Snapshot of the hosted simulation's externally visible state. */
    struct Snapshot
    {
        std::string simulationId; ///< Simulation identifier.
        State state{ State::eInitializing }; ///< Current lifecycle state.
        ::grpc::Status error; ///< Failure status when @ref state is @ref State::eFailed.
        int64_t simulationTimeNs{ 0 }; ///< Latest visible simulation timestamp, in nanoseconds.
    };

    /** @brief Construct a host backed by the local OV SIM implementation. */
    HostedSimulation();

    /**
     * @brief Construct a host backed by a supplied native simulation adapter.
     *
     * @param[in] nativeSimulation Native adapter owned by the host.
     * @throws std::invalid_argument If @p nativeSimulation is null.
     */
    explicit HostedSimulation(std::unique_ptr<NativeSimulation> nativeSimulation);

    /** @brief Shut down the worker and destroy the host. */
    ~HostedSimulation();

    HostedSimulation(const HostedSimulation&) = delete;
    HostedSimulation& operator=(const HostedSimulation&) = delete;

    /**
     * @brief Begin creating a hosted simulation.
     *
     * @param[in] simulationId Non-empty identifier for the simulation.
     * @param[in] usdContent Non-empty USD layer content to import.
     * @return An OK status when initialization was scheduled, or an error status when the request is invalid.
     */
    ::grpc::Status createSimulation(std::string simulationId, std::string usdContent);

    /**
     * @brief Get the current state of a hosted simulation.
     *
     * @param[in] simulationId Identifier of the simulation to inspect.
     * @param[out] snapshot Destination state snapshot.
     * @return An OK status on success, or an error status when the simulation or destination does not exist.
     */
    ::grpc::Status getSnapshot(const std::string& simulationId, Snapshot* snapshot) const;

    /**
     * @brief Check whether a hosted simulation is ready for synchronous operations.
     *
     * @param[in] simulationId Identifier of the simulation to inspect.
     * @return An OK status when the simulation is ready; otherwise, a status describing its current state.
     */
    ::grpc::Status getReadyStatus(const std::string& simulationId) const;

    /**
     * @brief Begin deleting a hosted simulation.
     *
     * @param[in] simulationId Identifier of the simulation to delete.
     * @return An OK status when cleanup was scheduled or already requested; otherwise, a not-found status.
     */
    ::grpc::Status deleteSimulation(std::string simulationId);

    /**
     * @brief Read an attribute from the hosted simulation.
     *
     * @param[in] simulationId Identifier of the simulation to read.
     * @param[in] paths Prim paths to read.
     * @param[in] attribute Attribute name to read.
     * @param[out] value Destination attribute value.
     * @param[out] simulationTimeNs Destination visible simulation timestamp, in nanoseconds.
     * @return An OK status on success, or a status describing validation or simulation failure.
     */
    ::grpc::Status read(std::string simulationId,
                        std::vector<std::string> paths,
                        std::string attribute,
                        ::ovsim::interfaces::data::OutputValueType* value,
                        int64_t* simulationTimeNs);

    /**
     * @brief Write an attribute to the hosted simulation at the next timestamp.
     *
     * @param[in] simulationId Identifier of the simulation to update.
     * @param[in] paths Prim paths to update.
     * @param[in] attribute Attribute name to write.
     * @param[in] value Value to write.
     * @param[in] simulationTimeNs Requested simulation timestamp, in nanoseconds.
     * @return An OK status on success, or a status describing validation or simulation failure.
     */
    ::grpc::Status write(std::string simulationId,
                         std::vector<std::string> paths,
                         std::string attribute,
                         ::ovsim::interfaces::data::InputValueType value,
                         int64_t simulationTimeNs);

    /**
     * @brief Advance the hosted simulation to the requested timestamp.
     *
     * @param[in] simulationId Identifier of the simulation to advance.
     * @param[in] simulationTimeNs Requested simulation timestamp, in nanoseconds.
     * @return An OK status on success, or a status describing validation or simulation failure.
     */
    ::grpc::Status step(std::string simulationId, int64_t simulationTimeNs);

    /**
     * @brief Clean up the hosted simulation and stop the worker thread.
     *
     * @return An OK status on success, or the cleanup failure status.
     */
    ::grpc::Status shutdown();

private:
    template <typename Function>
    ::grpc::Status _invoke(Function&& function)
    {
        auto task = std::make_shared<std::packaged_task<::grpc::Status()>>(std::forward<Function>(function));
        auto future = task->get_future();
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);
            if (m_stopping)
            {
                return { ::grpc::StatusCode::UNAVAILABLE, "The simulation worker is shutting down." };
            }
            m_tasks.emplace([task]() { (*task)(); });
        }
        m_queueCondition.notify_one();
        return future.get();
    }

    void _enqueue(std::function<void()> task);
    // Synchronous gRPC handlers may execute concurrently on different threads. Keep all process-global native stage
    // and physics calls serialized on this stable worker thread so they never depend on handler thread affinity.
    void _runWorker();
    void _initializeOnWorker(std::string simulationId, std::string usdContent);
    ::grpc::Status _cleanupOnWorker(const std::string& simulationId, bool clearOnSuccess);
    ::grpc::Status _getReadyStatus(const std::string& simulationId) const;
    int64_t _getCurrentTime() const;
    void _incrementTime();

    mutable std::mutex m_stateMutex;
    bool m_hasSimulation{ false };
    std::string m_simulationId;
    State m_state{ State::eInitializing };
    ::grpc::Status m_error;
    bool m_deleteRequested{ false };
    bool m_stageImported{ false };
    bool m_simulationInitialized{ false };
    int64_t m_simulationTimeNs{ 0 };
    int64_t m_latestVisibleTimeNs{ 0 };
    size_t m_timePhase{ 0 };
    std::unordered_map<std::string, int64_t> m_latestWriteTimes;

    std::unique_ptr<NativeSimulation> m_nativeSimulation;

    std::mutex m_queueMutex;
    std::condition_variable m_queueCondition;
    std::queue<std::function<void()>> m_tasks;
    bool m_stopping{ false };
    ::grpc::Status m_shutdownStatus;
    std::thread m_worker;
};

} // namespace grpc
} // namespace servers
} // namespace ovsim
} // namespace isaacsim
