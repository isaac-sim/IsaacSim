// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ovsim/clients/grpc/Export.h>
#include <ovsim/interfaces/control/authoring/Authoring.hpp>
#include <ovsim/interfaces/control/simulation/Simulation.hpp>
#include <ovsim/interfaces/data/Data.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace isaacsim
{
namespace ovsim
{
namespace clients
{
namespace grpc
{

using PathType = ::ovsim::interfaces::data::PathType;
using InputValueType = ::ovsim::interfaces::data::InputValueType;
using OutputValueType = ::ovsim::interfaces::data::OutputValueType;
using InputParameterType = ::ovsim::interfaces::control::authoring::InputParameterType;
using OutputParameterType = ::ovsim::interfaces::control::authoring::OutputParameterType;

/** @brief Minimal remote OV SIM client session using insecure gRPC. */
class ISAACSIM_OVSIM_CLIENTS_GRPC_API GrpcSession
{
public:
    /** @brief Create a remote OV SIM session using insecure gRPC.
     * @param[in] configuration Configuration containing a non-empty `endpoint` entry. The endpoint uses the standard
     * gRPC channel-target syntax, such as `127.0.0.1:50051`.
     * @throws std::invalid_argument If `configuration` or its `endpoint` entry is absent or empty.
     */
    explicit GrpcSession(const std::optional<std::unordered_map<std::string, std::string>>& configuration = std::nullopt);

    /** @brief Destroy the session and release its gRPC resources. */
    ~GrpcSession();

    GrpcSession(const GrpcSession&) = delete;
    GrpcSession& operator=(const GrpcSession&) = delete;

    /** @brief Move-construct a session by transferring its remote connection and lifecycle state. */
    GrpcSession(GrpcSession&&) noexcept;

    /**
     * @brief Move-assign a session by transferring its remote connection and lifecycle state.
     * @return Reference to this session.
     */
    GrpcSession& operator=(GrpcSession&&) noexcept;

    /** @brief Get the connection configuration supplied at construction.
     * @return Session connection configuration.
     * @note The returned reference remains valid until this session is destroyed or moved from.
     */
    const std::unordered_map<std::string, std::string>& getConfiguration() const;

    // authoring
    /** @copydoc isaacsim::physics::ovsim::control::authoring::createStage
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool createStage();

    /** @copydoc isaacsim::physics::ovsim::control::authoring::openStage
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool openStage(const std::string& usdPath);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::saveStage
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool saveStage(const std::string& usdPath);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::importStageFromString
     * @return `true` if asynchronous remote simulation creation is accepted; otherwise `false`.
     * @note A new internal simulation UUID is generated for each lifecycle. A session cannot create another
     * simulation until the current lifecycle has been deleted.
     */
    bool importStageFromString(const std::string& usdString);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::exportStageToString
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    std::string exportStageToString();

    /** @copydoc isaacsim::physics::ovsim::control::authoring::closeStage
     * @return `true` if asynchronous remote deletion is accepted or the simulation is already absent; otherwise
     * `false`.
     */
    bool closeStage();

    /** @copydoc isaacsim::physics::ovsim::control::authoring::addReferenceToStage
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool addReferenceToStage(const std::string& usdPath, const std::string& path, const std::string& typeName);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::definePrim
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool definePrim(const std::string& path, const std::string& typeName);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::movePrim
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool movePrim(const std::string& targetPath, const std::string& destinationPath);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::removePrim
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool removePrim(const std::string& path);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::createPrimAttribute
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool createPrimAttribute(const std::string& path, const std::string& attributeName, const std::string& typeName);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::removePrimAttribute
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    bool removePrimAttribute(const std::string& path, const std::string& attributeName);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::setParameter
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    void setAuthoringParameter(const std::string& provider,
                               const std::string& parameterName,
                               const InputParameterType& value);

    /** @copydoc isaacsim::physics::ovsim::control::authoring::getParameter
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    OutputParameterType getAuthoringParameter(const std::string& provider, const std::string& parameterName);

    // simulation
    /** @copydoc isaacsim::physics::ovsim::control::simulation::play
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    void play();

    /** @copydoc isaacsim::physics::ovsim::control::simulation::pause
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    void pause();

    /** @copydoc isaacsim::physics::ovsim::control::simulation::stop
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    void stop();

    /** @copydoc isaacsim::physics::ovsim::control::simulation::initialize
     * @note Polls the remote lifecycle until it is ready, with a five-minute overall timeout.
     * @throws std::runtime_error If the remote lifecycle fails, a transport error occurs, or the timeout expires.
     */
    void initialize();

    /** @copydoc isaacsim::physics::ovsim::control::simulation::invalidate
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    void invalidate();

    /** @copydoc isaacsim::physics::ovsim::control::simulation::step
     * @note Advances the protocol timestamp by one drift-free 60 Hz interval. The first implementation keeps the
     * server's existing fixed physics time step.
     * @throws std::runtime_error If the remote operation fails.
     */
    void step();

    /** @copydoc isaacsim::physics::ovsim::control::simulation::setParameter
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    void setSimulationParameter(const std::string& provider,
                                const std::string& parameterName,
                                const InputParameterType& value);

    /** @copydoc isaacsim::physics::ovsim::control::simulation::getParameter
     * @throws std::logic_error Always, because the remote operation is not implemented.
     */
    OutputParameterType getSimulationParameter(const std::string& provider, const std::string& parameterName);

    // data
    /** @copydoc isaacsim::physics::ovsim::data::read
     * @note Supports latest-value reads for the minimal protocol codec set. Returned numeric arrays are CPU arrays.
     * @throws std::invalid_argument If paths, the attribute name, or the timestamp is unsupported or malformed.
     * @throws std::runtime_error If the remote operation or value decoding fails.
     */
    OutputValueType read(const PathType& paths, const std::string& attributeName, std::optional<double> timestamp);

    /** @copydoc isaacsim::physics::ovsim::data::write
     * @note Supports immediate CPU float32/float64 position arrays with shape `(path_count, 3)`.
     * @throws std::invalid_argument If paths, values, the attribute name, or the timestamp is unsupported or malformed.
     * @throws std::runtime_error If the remote operation or value encoding fails.
     */
    void write(const PathType& paths,
               const std::string& attributeName,
               const InputValueType& values,
               std::optional<double> timestamp);

private:
    class Implementation;
    std::unique_ptr<Implementation> m_implementation;
};

} // namespace grpc
} // namespace clients
} // namespace ovsim
} // namespace isaacsim
