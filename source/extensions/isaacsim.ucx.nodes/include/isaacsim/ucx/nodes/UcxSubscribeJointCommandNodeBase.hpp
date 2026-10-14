// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ucx/core/UcxListenerRegistry.hpp>
#include <isaacsim/ucx/core/UcxUtils.hpp>
#include <isaacsim/ucx/nodes/UcxNode.hpp>
#include <isaacsim/ucx/nodes/UcxPendingRequest.hpp>
#include <omni/graph/core/CppWrappers.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <vector>

namespace isaacsim::ucx::nodes
{

/**
 * @struct JointCommandData
 * @brief Data structure for joint command message payload.
 * @details
 * Contains robot joint command data including positions, velocities, and efforts
 * for all joints in the articulation.
 */
struct JointCommandData
{
    double timestamp; //!< Timestamp value in seconds
    int64_t numJoints; //!< Number of joints
    std::vector<double> positionCommand; //!< Joint position commands
    std::vector<double> velocityCommand; //!< Joint velocity commands
    std::vector<double> effortCommand; //!< Joint effort commands
    std::vector<double> stiffness; //!< Optional joint proportional gains.
    std::vector<double> damping; //!< Optional joint derivative gains.
    bool valid; //!< Flag indicating if data was successfully parsed
};

/**
 * @brief Base class template for UCX joint state subscriber nodes.
 * @details
 * Provides common functionality for receiving joint command data via UCX.
 * Derived classes implement message parsing via parseMessage().
 * The base class handles setting outputs via setOutputs().
 *
 * @tparam DatabaseT The OmniGraph database type for the derived node
 */
template <typename DatabaseT>
class UCXSubscribeJointCommandNodeBase : public UcxNode
{
public:
    using NodeObj = omni::graph::core::NodeObj; ///< Alias for the OmniGraph node object type.
    using GraphInstanceID = omni::graph::core::GraphInstanceID; ///< Alias for the OmniGraph instance ID type.

    /**
     * @brief Initialize the node instance.
     * @details
     * Default implementation - derived classes should override to initialize state.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        // Default: no initialization needed
        // Derived classes should override and reserve buffer if needed
    }

    /** @brief Destroys the node, retiring every outstanding receive before its buffer is freed. */
    virtual ~UCXSubscribeJointCommandNodeBase()
    {
        // Not left to reset(): `~UcxNode()` dispatches to `UcxNode::reset()`, not this override,
        // so a node destroyed without a prior releaseInstance() would free a posted buffer.
        m_receive.cancelAndDrain();
    }

    /**
     * @brief Reset the node state.
     * @details
     * Cancels any in-flight receive request and resets the connection.
     */
    virtual void reset() override
    {
        // Release the in-flight receive before the listener: it may hold the last reference to
        // the ucxx::Endpoint, whose close callback would otherwise fire on a dangling pointer.
        m_receive.reset();
        UcxNode::reset();
    }

protected:
    /** @brief Size of the receive buffer, in bytes. */
    static constexpr size_t s_kReceiveBufferSize = 65536;

    /**
     * @brief Compute implementation for joint state subscription.
     * @details
     * Advances the deferred receive by one step and outputs any message that has arrived.
     * Never blocks.
     *
     * @param[in,out] db Database accessor for node inputs/outputs
     * @param[in] port Port number for UCX communication
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Time a posted receive may stay outstanding before it is
     *                      cancelled and re-posted, in milliseconds (0 = no deadline)
     * @return bool True if execution succeeded, false otherwise
     */
    bool computeImpl(DatabaseT& db, uint16_t port, uint64_t tag, uint32_t timeoutMs)
    {
        // Drop any receive posted against a different port or tag before the listener can be
        // swapped out from under it: its buffer may hold a message from the old configuration.
        discardReceiveOnConfigChange(port, tag);

        if (!this->ensureListenerReady(db, port))
        {
            return false;
        }

        if (!this->waitForConnection())
        {
            return true;
        }

        return pollReceive(db, port, tag, timeoutMs);
    }

    /**
     * @brief Discard an outstanding receive when the node's port or tag has changed.
     * @details
     * Runs on the update thread, so the request is abandoned rather than drained. Abandoning retires
     * the buffer too, so a message received under the old configuration is never published.
     *
     * @param[in] port Port currently configured on the node
     * @param[in] tag UCX tag currently configured on the node
     */
    void discardReceiveOnConfigChange(uint16_t port, uint64_t tag)
    {
        if (!m_receive.isOutstanding() || (port == m_postedPort && tag == m_postedTag))
        {
            return;
        }

        m_receive.abandon();
        // A latched warning describes the configuration that has just been abandoned, so it must
        // not swallow the first report of the same problem under the new port or tag.
        _clearReceiveWarningLatches();
    }

    /** @brief Re-arms the warning categories the receive path reports. */
    void _clearReceiveWarningLatches()
    {
        _clearWarningLatch(UcxWarningCategory::ePostFailed);
        _clearWarningLatch(UcxWarningCategory::eRequestFailed);
        _clearWarningLatch(UcxWarningCategory::eDeadlineExpired);
    }

    /**
     * @brief Advance the deferred receive by one step, without blocking.
     * @details
     * Posts a receive request if none is outstanding, then polls it. On completion the message
     * is parsed with parseMessage() and published with setOutputs().
     *
     * @param[in,out] db Database accessor for node inputs/outputs
     * @param[in] port Port the request is posted against
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Deadline for the posted request, in milliseconds (0 = none)
     * @return bool True if the node executed successfully, false on an unexpected error
     */
    bool pollReceive(DatabaseT& db, uint16_t port, uint64_t tag, uint32_t timeoutMs)
    {
        try
        {
            m_receive.pruneCompleted();

            if (!m_receive.isOutstanding() && !postReceive(db, port, tag, timeoutMs))
            {
                return true;
            }

            if (!m_receive.isCompleted())
            {
                enforceReceiveDeadline(db, port, tag, timeoutMs);
                return true;
            }

            std::shared_ptr<ucxx::Request> request = m_receive.releaseRequest();

            try
            {
                request->checkError();
            }
            catch (const std::exception& e)
            {
                _logWarningOnce(db, UcxWarningCategory::eRequestFailed, "Receive request failed: %s", e.what());
                return true;
            }

            // A message arrived, so any previously reported receive problem is over.
            _clearReceiveWarningLatches();

            JointCommandData data = parseMessage(m_receive.buffer());

            if (data.valid)
            {
                setOutputs(db, data);
            }

            return true;
        }
        catch (const std::exception& e)
        {
            db.logError("Exception during message receive: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Post a receive request into the receive buffer.
     *
     * @param[in,out] db Database accessor for logging
     * @param[in] port Port the request is posted against
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Deadline for the posted request, in milliseconds (0 = none)
     * @return bool True if a request is now outstanding, false otherwise
     * @pre No request is outstanding, so sizing the buffer cannot touch a live request.
     */
    bool postReceive(DatabaseT& db, uint16_t port, uint64_t tag, uint32_t timeoutMs)
    {
        std::string errorMessage;
        const bool posted = m_receive.post(s_kReceiveBufferSize,
                                           [&](void* buffer, size_t size, std::shared_ptr<ucxx::Request>& request)
                                           {
                                               return this->m_listener->tagReceiveWithRequest(
                                                          buffer, size, tag, 0xFFFFFFFFFFFFFFFF, errorMessage,
                                                          request) == isaacsim::ucx::core::UcxReceiveResult::eSuccess;
                                           });

        if (!posted)
        {
            _logWarningOnce(
                db, UcxWarningCategory::ePostFailed, "Failed to post UCX receive request: %s", errorMessage.c_str());
            return false;
        }

        // Recorded so a later compute() can tell that the configuration has moved on.
        m_postedPort = port;
        m_postedTag = tag;

        // The post time is recorded rather than a deadline, so a change to inputs:timeoutMs while
        // the request is outstanding still takes effect.
        m_receivePostedAt = std::chrono::steady_clock::now();

        return true;
    }

    /**
     * @brief Cancel and re-post the outstanding receive once its deadline has passed.
     *
     * @param[in,out] db Database accessor for logging
     * @param[in] port Port the replacement request is posted against
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Deadline for the replacement request, in milliseconds
     */
    void enforceReceiveDeadline(DatabaseT& db, uint16_t port, uint64_t tag, uint32_t timeoutMs)
    {
        if (timeoutMs == 0 || std::chrono::steady_clock::now() < m_receivePostedAt + std::chrono::milliseconds(timeoutMs))
        {
            return;
        }

        // The message can land between the caller's isCompleted() check and this one; cancelling
        // here would discard it, so leave the request for the next poll to publish.
        if (m_receive.isCompleted())
        {
            return;
        }

        _logWarningOnce(db, UcxWarningCategory::eDeadlineExpired,
                        "No UCX message received on tag %llu within %u ms. Cancelling the pending receive and "
                        "posting a new one; set inputs:timeoutMs to 0 to keep a single receive outstanding until "
                        "a message arrives.",
                        static_cast<unsigned long long>(tag), timeoutMs);

        // Abandon rather than drain: the update thread must not sleep on the cancellation.
        m_receive.abandon();
        postReceive(db, port, tag, timeoutMs);
    }

    /**
     * @brief Parse joint command message from raw bytes.
     * @details
     * Pure virtual function that derived classes must implement to parse
     * the raw message buffer into a JointCommandData structure.
     *
     * @param[in] buffer Raw message buffer
     * @return JointCommandData Parsed joint command data with valid flag set
     */
    virtual JointCommandData parseMessage(const std::vector<uint8_t>& buffer) = 0;

    /**
     * @brief Set node outputs from joint command data.
     * @details
     * Sets the database outputs from the parsed joint command data structure.
     *
     * @param[in,out] db Database accessor for node outputs
     * @param[in] data Joint command data to output
     */
    void setOutputs(DatabaseT& db, const JointCommandData& data)
    {
        db.outputs.positionCommand().resize(data.numJoints);
        db.outputs.velocityCommand().resize(data.numJoints);
        db.outputs.effortCommand().resize(data.numJoints);

        std::memcpy(db.outputs.positionCommand().data(), data.positionCommand.data(), sizeof(double) * data.numJoints);
        std::memcpy(db.outputs.velocityCommand().data(), data.velocityCommand.data(), sizeof(double) * data.numJoints);
        std::memcpy(db.outputs.effortCommand().data(), data.effortCommand.data(), sizeof(double) * data.numJoints);

        db.outputs.stiffness().resize(data.stiffness.size());
        std::copy(data.stiffness.begin(), data.stiffness.end(), db.outputs.stiffness().begin());
        db.outputs.damping().resize(data.damping.size());
        std::copy(data.damping.begin(), data.damping.end(), db.outputs.damping().begin());

        db.outputs.timeStamp() = data.timestamp;
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
    }

    UcxPendingRequest m_receive; //!< Deferred receive request and the buffer it writes into
    std::chrono::steady_clock::time_point m_receivePostedAt{}; //!< When the outstanding request was posted
    uint64_t m_postedTag = 0; //!< Tag the outstanding request was posted against
    uint16_t m_postedPort = 0; //!< Port the outstanding request was posted against
};

// NOTE: To use this base class:
// 1. Derive your OGN node class from UCXSubscribeJointCommandNodeBase<YourDatabase>
// 2. Implement static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Reserve receive buffer: state.m_receive.reserveBuffer(s_kReceiveBufferSize)
// 3. Implement static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Call state.reset()
// 4. Implement static bool compute(YourDatabase& db) that:
//    - Extracts inputs from db (port, tag, timeoutMs)
//    - Gets the per-instance state: auto& state = db.template perInstanceState<YourClass>()
//    - Calls state.computeImpl(db, port, tag, timeoutMs)
// 5. Implement virtual JointCommandData parseMessage(const std::vector<uint8_t>& buffer) override
//    - Parse raw bytes into JointCommandData structure
// 6. See OgnUCXSubscribeJointCommand.cpp for examples

} // namespace isaacsim::ucx::nodes
